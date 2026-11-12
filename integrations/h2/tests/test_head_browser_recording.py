"""All-mode head integration and playable video, without robot commands."""
import asyncio
import csv
import io
import json
import math
from multiprocessing import Array, get_context
from pathlib import Path
import shutil
import subprocess
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from head_camera.browser_head import BrowserHeadBridge, install_hooks, matrix_pose
from head_camera.recording import HeadVideo
from pico_tracker.geometry import rotation
from quest_h2_teleop import command
from test_hybrid_hands import controller, web


def matrix(angle=0.):
    r = rotation([0., 0., 0., 0., math.sin(angle/2), 0., math.cos(angle/2)])
    return [r[i]+[1.6 if i == 1 else 0.] for i in range(3)]+[[0., 0., 0., 1.]]


def flat(value):
    return [value[r][c] for c in range(4) for r in range(4)]


class BrowserCoordinates(unittest.TestCase):
    def test_column_major_head_pose_matches_all_rotation_branches(self):
        for angle in (0., .3, 2.8, -2.8, math.pi):
            value = matrix_pose(flat(matrix(angle)))
            expected = rotation([0., 0., 0., 0., math.sin(angle/2), 0., math.cos(angle/2)])
            actual = rotation(value)
            for i in range(3):
                for j in range(3):self.assertAlmostEqual(actual[i][j], expected[i][j])
            self.assertEqual(value[:3], [0., 1.6, 0.])
        for axis in (0, 2):
            q = [0., 0., 0., 0.]
            q[axis] = 1.
            r = rotation([0., 0., 0.]+q)
            self.assertAlmostEqual(abs(matrix_pose([r[i]+[0.] for i in range(3)]+[[0.,0.,0.,1.]])[3+axis]),1.)

    def test_invalid_rotation_reflection_and_nonfinite_camera_events_are_rejected(self):
        for value in ([0.]*16, [[1.,0.,0.,0.]]*4, flat(matrix())):
            value = list(value)
            if value == flat(matrix()): value[0] = float('nan')
            with self.assertRaises(ValueError):matrix_pose(value)
        value = matrix();value[0][0] = -1.
        with self.assertRaises(ValueError):matrix_pose(value)


class FakeVuer:
    def __init__(self, *_, **__):
        self.values = Array('d', flat(matrix()), lock=True)

    @property
    def head_pose(self):
        return [[self.values[4*c+r] for c in range(4)] for r in range(4)]

    async def on_cam_move(self, event, session, fps=60):
        self.values[:] = event.value['camera']['matrix']


class BrowserIntegration(unittest.TestCase):
    def setUp(self):
        self.events = []
        self.bridge = BrowserHeadBridge()
        self.tv = SimpleNamespace(TeleVuer=FakeVuer)
        tv = self.tv
        sentinel = self.observation = object()

        class FakeWrapper:
            def __init__(self):self.tvuer = tv.TeleVuer()
            def get_tele_data(self):return sentinel

        class FakeArm:
            def __init__(self):
                self.lowcmd_publisher = Mock()
                self.crc = Mock()
            def get_current_motor_q(self):return [0.]*29+[.12,-.2]+[0.]*4

        class FakeIPC:
            cmd_map = {'CMD_START':'r','CMD_STOP':'q'}
            def __init__(self, on_press, get_state):self.press, self.state = on_press, get_state

        self.arm = SimpleNamespace(H2_ArmController=FakeArm)
        self.ipc = SimpleNamespace(IPC_Server=FakeIPC)
        self.package = SimpleNamespace(TeleVuerWrapper=FakeWrapper)
        self.restore = install_hooks(self.bridge,self.arm,self.ipc,self.tv,self.package)
        self.addCleanup(self.restore)
        self.wrapper = self.package.TeleVuerWrapper()
        self.controller = self.arm.H2_ArmController()
        self.server = self.ipc.IPC_Server(on_press=self.events.append,get_state=lambda:{'READY':True})

    def event(self, angle=0.):
        value = SimpleNamespace(value={'camera':{'matrix':flat(matrix(angle))}})
        asyncio.run(self.wrapper.tvuer.on_cam_move(value,None))

    def test_start_requires_live_browser_head_and_preserves_arm_hand_data(self):
        with self.assertRaises(ValueError):self.server.press('r')
        self.assertEqual(self.events,[])
        self.event()
        self.server.press('r')
        self.assertEqual(self.events,['r'])
        self.assertEqual(self.bridge.publisher.target,(.12,-.2))
        self.event(.3)
        self.assertIs(self.wrapper.get_tele_data(),self.observation)
        self.assertNotEqual(self.bridge.publisher.target,(.12,-.2))
        self.assertTrue(self.server.state()['HEAD_LIVE'])

    def test_stale_browser_data_holds_last_head_target_and_stop_ends_following(self):
        self.event();self.server.press('r');self.event(.3);self.wrapper.get_tele_data()
        target = self.bridge.publisher.target
        self.wrapper.tvuer.head_received_mono.value = time.monotonic()-1
        self.wrapper.get_tele_data()
        self.assertEqual(self.bridge.publisher.target,target)
        self.assertFalse(self.server.state()['HEAD_LIVE'])
        self.server.press('q')
        self.event(.6);self.wrapper.get_tele_data()
        self.assertEqual(self.bridge.publisher.target,target)
        self.assertFalse(self.bridge.active)

    def test_recenter_uses_current_robot_neck_and_source_is_shared_across_vuer_process(self):
        def camera_child():self.event(.4)
        child = get_context('fork').Process(target=camera_child)
        child.start();child.join(3)
        if child.is_alive():child.kill();child.join();self.fail('Camera callback did not exit')
        self.assertEqual(child.exitcode,0)
        self.assertTrue(self.server.state()['HEAD_READY'])
        self.server.press('r');self.event(.6);self.wrapper.get_tele_data()
        self.server.press('h')
        self.assertEqual(self.bridge.publisher.target,(.12,-.2))
        self.assertIn('CMD_HEAD_RECENTER',self.server.cmd_map)


class DashboardHeadModes(unittest.TestCase):
    def test_follow_headset_can_be_saved_in_every_dashboard_mode(self):
        with tempfile.TemporaryDirectory() as folder, patch.object(web,'ROOT',Path(folder)):
            for source in ('vr','wuji','pico_head','pico_wuji','pico_dual_wuji','pico_tracker_hands'):
                up,_ = controller(source)
                self.assertTrue(up.set_head_follow(True)[0])
                self.assertEqual(json.loads((Path(folder)/'config/head_follow.json').read_text()),{'enabled':True})

    def test_browser_head_readiness_is_checked_before_starting_wuji_publishers(self):
        up,events = controller('wuji')
        up.hb.pop('HEAD_READY')
        with patch.object(web,'head_settings',return_value={'enabled':True}):
            self.assertFalse(up.start()[0])
            self.assertEqual(events,[])
            up.hb['HEAD_READY'] = True
            self.assertTrue(up.start()[0])
            self.assertEqual(events,['glove_start','CMD_START'])

    def test_browser_launcher_retains_official_h2_arguments(self):
        for mode in ('vr','wuji'):
            args=command(mode)
            self.assertTrue(args[1].endswith('/head_camera/browser_teleop.py'))
            self.assertEqual(args[args.index('--arm')+1],'H2')
            self.assertIn('--ipc',args)

    def test_head_only_sessions_are_eligible_for_automatic_recording(self):
        up,_=controller('pico_head')
        up.running=lambda:True
        up.external_pids=lambda:[123]
        up.hb['START']=True
        self.assertTrue(up.recording_snapshot()['started'])


class HeadRecording(unittest.TestCase):
    def test_missing_camera_config_has_a_visible_error_and_starts_no_encoder(self):
        with tempfile.TemporaryDirectory() as folder:
            video=HeadVideo(folder,30,threading.Event(),Path(folder)/'missing.json')
            video.start();video.join(1)
            self.assertFalse(video.is_alive())
            self.assertIsNone(video.encoder)
            self.assertTrue(video.error)

    @unittest.skipUnless(shutil.which('ffmpeg') and shutil.which('ffprobe'),'FFmpeg video tools unavailable')
    def test_real_encoder_creates_a_playable_head_video_with_frame_index(self):
        from PIL import Image
        with tempfile.TemporaryDirectory() as folder:
            out=Path(folder);config=out/'receiver.json'
            config.write_text(json.dumps({'enabled':True,'host':'127.0.0.1','port':55558}))
            frames=[]
            for i in range(12):
                data=io.BytesIO();Image.new('RGB',(64,48),(i*15,70,130)).save(data,format='JPEG');frames.append(data.getvalue())
            values=iter(frames);last=[None]
            def subscribe():
                last[0]=next(values,last[0]);return SimpleNamespace(jpg=last[0])
            stop=threading.Event();video=HeadVideo(out,15,stop,config,subscribe)
            video.start()
            deadline=time.monotonic()+5
            while video.frames<12 and video.is_alive() and time.monotonic()<deadline:time.sleep(.02)
            stop.set();video.join(5)
            if video.is_alive():video.finish_encoder(force=True);video.join(1)
            self.assertEqual(video.error,'')
            self.assertEqual(video.frames,12)
            result=subprocess.run(['ffprobe','-v','error','-select_streams','v:0','-count_frames',
                                   '-show_entries','stream=codec_name,width,height,nb_read_frames',
                                   '-of','json',str(out/'head.mp4')],capture_output=True,text=True,check=True)
            stream=json.loads(result.stdout)['streams'][0]
            self.assertEqual(stream['codec_name'],'h264')
            self.assertEqual((stream['width'],stream['height']),(64,48))
            self.assertGreaterEqual(int(stream['nb_read_frames']),8)
            with (out/'head_frames.csv').open() as file:self.assertEqual(len(list(csv.reader(file))),13)


if __name__=='__main__':unittest.main()
