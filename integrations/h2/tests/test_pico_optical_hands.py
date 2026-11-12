"""Optical fingers, tracker ownership and STOP without robot commands."""
import copy
import json
import math
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import Mock, patch
from http.server import ThreadingHTTPServer
import threading
from urllib.request import Request, urlopen

from pico_tracker.hands import hand_landmarks, read_hands, hands_status, OpticalFingers
from pico_tracker.input import TrackerInput
from pico_tracker.geometry import rotation, matvec
from quest_h2_teleop import command, PICO_HAND_SOURCE
from test_hybrid_hands import controller, web
from wuji_glove import dashboard_gateway as gateway


def joints():
    wrist = [1., 2., 3.]
    data = [wrist+[0., 0., 0., 1.] for _ in range(26)]
    for i in range(2, 26):
        data[i] = [wrist[0]+.001*i, wrist[1]+.002*i, wrist[2]-.003*i, 0., 0., 0., 1.]
    return data


def frame(session='input-session'):
    points = hand_landmarks(joints())
    return {'session': session, 'hands': {side: {'active': True, 'landmarks': copy.deepcopy(points)}
                                        for side in ('left', 'right')}}


class OpticalCoordinates(unittest.TestCase):
    def test_openxr_palm_is_removed_and_all_fingertips_keep_webxr_indices(self):
        data = joints()
        data[0][:3] = [900., 800., 700.]
        output = hand_landmarks(data)
        self.assertEqual(len(output), 25)
        self.assertEqual(output[0], [0., 0., 0.])
        for index in (4, 9, 14, 19, 24):
            i = index+1
            self.assertEqual(output[index], [round(.002*i, 6), round(-.003*i, 6), round(.001*i, 6)])

    def test_world_rotation_and_translation_do_not_change_finger_targets(self):
        data = joints()
        expected = hand_landmarks(data)
        q = [0., 0., math.sqrt(.5), math.sqrt(.5)]
        r = rotation([0., 0., 0.]+q)
        moved = []
        for joint in data:
            p = matvec(r, joint[:3])
            moved.append([p[0]+4., p[1]-2., p[2]+.7]+q)
        self.assertEqual(hand_landmarks(moved), expected)

    def test_invalid_or_missing_wrist_and_nonfinite_joints_are_rejected(self):
        for change in ('missing', 'quaternion', 'nan'):
            data = joints()
            if change == 'missing':data.pop()
            elif change == 'quaternion':data[1][3:] = [0.]*4
            else:data[5][0] = float('nan')
            with self.assertRaises(ValueError):hand_landmarks(data)

    def test_sdk_hand_loss_does_not_remove_other_hand(self):
        sdk = Mock()
        sdk.get_left_hand_is_active.return_value = 1
        sdk.get_left_hand_tracking_state.return_value = joints()
        sdk.get_right_hand_is_active.return_value = 0
        hands = read_hands(sdk)
        self.assertTrue(hands['left']['active'])
        self.assertFalse(hands['right']['active'])
        sdk.get_right_hand_tracking_state.assert_not_called()
        self.assertFalse(hands_status({'hands': hands})['ready'])
        self.assertFalse(hands_status({'hands': None})['ready'])

    def test_full_optical_payload_stays_within_atomic_pipe_budget(self):
        data = frame()
        data.update(version=1, source='xrobotoolkit_sdk', seq=1234567,
                    trackers={f'PICO_TEST_{i}': [.123456789]*3+[.5]*4 for i in range(3)},
                    head_pose=[.123456789]*3+[.5]*4,
                    sample_at_unix_ns=1234567890123456789, sample_mono=123456.123456789,
                    packet_timestamp_ns=1234567890123456789, motion_timestamp_ns=0,
                    pose_age_ms={f'PICO_TEST_{i}':1234.56 for i in range(3)})
        self.assertLessEqual(len((json.dumps(data,separators=(',', ':'))+'\n').encode()),4096)


class OpticalOwnership(unittest.TestCase):
    def test_official_finger_controller_receives_no_input_before_start(self):
        factory = Mock()
        fingers = OpticalFingers(factory)
        try:
            self.assertFalse(fingers.enabled.value)
            self.assertEqual(list(fingers.arrays['left']),[0.]*75)
            factory.assert_called_once_with(fingers.arrays['left'],fingers.arrays['right'],
                                            xr_motion_data_ready_in=fingers.enabled)
            fingers.start(frame())
            self.assertTrue(fingers.enabled.value)
            fingers.stop()
            self.assertFalse(fingers.enabled.value)
        finally:fingers.close()

    def test_lost_left_holds_its_own_buffer_and_right_keeps_updating(self):
        fingers = OpticalFingers(Mock())
        try:
            value = frame()
            fingers.start(value)
            left = list(fingers.arrays['left'])
            before = list(fingers.arrays['right'])
            value['hands']['left'] = {'active': False, 'landmarks': None}
            value['hands']['right']['landmarks'][4][0] += .02
            status = fingers.update(value)
            self.assertFalse(status['sides']['left'])
            self.assertTrue(status['sides']['right'])
            self.assertEqual(list(fingers.arrays['left']), left)
            self.assertNotEqual(list(fingers.arrays['right']),before)
            self.assertTrue(fingers.enabled.value)
            fingers.update(None)
            self.assertFalse(fingers.enabled.value)
        finally:fingers.close()

    def test_new_input_session_cannot_resume_existing_fingers(self):
        fingers = OpticalFingers(Mock())
        try:
            fingers.start(frame())
            fingers.update(frame('new-session'))
            self.assertFalse(fingers.enabled.value)
        finally:fingers.close()

    def test_stop_reaps_the_official_child_if_its_signal_handler_does_not_exit(self):
        fingers = OpticalFingers(Mock())
        child = Mock()
        child.is_alive.side_effect = [True,True]
        fingers.children = [child]
        fingers.close()
        child.terminate.assert_called_once()
        child.kill.assert_called_once()
        self.assertEqual(child.join.call_count,2)
        self.assertFalse(fingers.enabled.value)

    def test_native_mode_owns_trackers_and_optical_fingers_without_wuji(self):
        argv = command(PICO_HAND_SOURCE)
        self.assertTrue(argv[1].endswith('/pico_tracker/arm_teleop.py'))
        self.assertIn('--both-arms',argv)
        self.assertIn('--pico-hands',argv)
        up,events = controller(PICO_HAND_SOURCE)
        up.tracker = Mock()
        up.tracker.status.return_value = {'ready': True}
        self.assertTrue(up.start()[0])
        self.assertEqual(events,['CMD_START'])
        up.tracker.status.assert_called_with(both=True,optical_hands=True)

    def test_missing_optical_hand_blocks_start_with_no_glove_activation(self):
        up,events = controller(PICO_HAND_SOURCE)
        up.tracker = Mock()
        up.tracker.status.return_value = {'ready': False,'message':'Show left hand'}
        self.assertFalse(up.start()[0])
        self.assertEqual(events,[])

    def test_optical_readiness_is_independent_of_tracker_only_readiness(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root/'config').mkdir()
            (root/'config/pico_tracker.json').write_text(json.dumps({'left_sn':'left','right_sn':'right'}))
            path = root/'latest.json'
            data = frame()
            data.update(received_mono=time.monotonic(),trackers={s:[0.,1.,0.,0.,0.,0.,1.] for s in ('left','right')},
                        head_pose=[0.,1.,0.,0.,0.,0.,1.],pose_age_ms={'left':0.,'right':0.})
            path.write_text(json.dumps(data))
            inputs = TrackerInput(root,path)
            self.assertTrue(inputs.status(both=True,optical_hands=True)['ready'])
            data['hands']['left']['active'] = False
            path.write_text(json.dumps(data))
            self.assertTrue(inputs.status(both=True)['ready'])
            self.assertFalse(inputs.status(both=True,optical_hands=True)['ready'])

    def test_dashboard_can_save_optical_mode_with_no_gloves_connected(self):
        up,_ = controller()
        up.wuji.status = lambda: {'active':False,'connected':False}
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'hand_source.json'
            with patch.object(web,'HAND_SOURCE_PATH',path):
                self.assertTrue(up.set_hand_source(PICO_HAND_SOURCE)[0])
            self.assertEqual(json.loads(path.read_text()),{'source':PICO_HAND_SOURCE})

    def test_laptop_mode_switch_closes_gloves_and_keeps_tracker_input(self):
        bridge,tracker = Mock(),Mock()
        response = Mock(status=200)
        response.read.return_value = b'{"message":"selected"}'
        response.getheader.return_value = 'application/json'
        backend = Mock()
        backend.getresponse.return_value = response
        server = ThreadingHTTPServer(('127.0.0.1',0),gateway.handler(bridge,tracker))
        thread = threading.Thread(target=server.serve_forever,daemon=True)
        thread.start()
        try:
            with patch.object(gateway,'HTTPConnection',return_value=backend):
                req = Request(f'http://127.0.0.1:{server.server_port}/api/hand-source',
                              data=json.dumps({'source':PICO_HAND_SOURCE}).encode(),
                              headers={'Content-Type':'application/json'},method='POST')
                with urlopen(req,timeout=3) as result:self.assertEqual(result.status,200)
            bridge.close.assert_called_once()
            tracker.close.assert_not_called()
        finally:
            server.shutdown();server.server_close();thread.join(timeout=2)
