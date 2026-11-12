"""Source ownership and combined START/STOP, without contacting a robot."""
import importlib.util
import json
from pathlib import Path
import socket
import tempfile
import sys
import threading
import time
import unittest
from unittest.mock import Mock, patch

from test_dashboard_modes import web
from quest_h2_teleop import command

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('wuji_input_under_test', ROOT / 'deployment/robot/quest_wuji.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class Glove:
    def __init__(self, events, active=False, start_ok=True):
        self.events, self.active, self.start_ok = events, active, start_ok

    def status(self): return {'active': self.active, 'connected': True}

    def start(self):
        self.events.append('glove_start')
        self.active = self.start_ok
        return self.start_ok, 'glove'

    def stop(self):
        self.events.append('glove_stop')
        self.active = False
        return True, 'stopped'


def controller(source='wuji', active=False, start_ok=True, ipc_ok=True):
    up = web.Upstream.__new__(web.Upstream)
    events = []
    up.lock = threading.Lock()
    up.hand_source = source
    up.hb = {'READY': True, 'START': False, 'HEAD_READY': True}
    up.online = lambda: True
    up.wuji = Glove(events, active, start_ok)
    up.auto_recording = None
    up.process = None
    up.external_pids = lambda: []
    up._send = lambda cmd: (events.append(cmd) or ipc_ok, 'ipc')
    up.running = lambda: False
    return up, events


class HybridTests(unittest.TestCase):
    def test_hand_feedback_requires_enabled_motor_entries_and_valid_angles(self):
        healthy = {'q': [0.] * 20, 'modes': [1] * 16 + [0] * 4, 'errors': [0, 0]}
        limits = [(-1., 1.)] * 20
        self.assertTrue(module.Dex5Feedback.assess(healthy, .01, limits)['ready'])
        disabled = dict(healthy, modes=[0] * 20)
        self.assertFalse(module.Dex5Feedback.assess(disabled, .01, limits)['ready'])
        bad_q = [0.] * 20
        bad_q[17] = -4.84
        result = module.Dex5Feedback.assess(dict(healthy, q=bad_q), .01, limits)
        self.assertFalse(result['ready'])
        self.assertEqual(result['invalid_joint_indices'], [17])
        self.assertFalse(module.Dex5Feedback.assess(healthy, 2., limits)['ready'])

    def test_bad_hand_feedback_never_spawns_an_adapter(self):
        feedback = Mock()
        feedback.status.return_value = {'ready': False, 'message': 'Left hand inactive'}
        with tempfile.TemporaryDirectory() as temporary:
            glove = module.WujiHand(Path(temporary), sys.executable, feedback=feedback)
            try:
                with patch.object(module.subprocess, 'Popen') as spawn:
                    ok, message = glove.start()
                    self.assertFalse(ok)
                    self.assertEqual(message, 'Left hand inactive')
                    spawn.assert_not_called()
            finally:
                glove.close()

    def test_adapter_start_waits_for_ready_signal_and_stop_exits_child(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            directory = root / 'wuji_glove'
            directory.mkdir()
            (directory / 'left_dex5_adapter.py').write_text(
                'import argparse,sys\nfrom pathlib import Path\n'
                'p=argparse.ArgumentParser();p.add_argument("--ready-file");a,_=p.parse_known_args()\n'
                'sys.stdin.readline();Path(a.ready_file).write_text("ready")\n'
                'for line in sys.stdin: pass\n')
            glove = module.WujiHand(root, sys.executable)
            try:
                glove.frame = {'header': {'seq': 1, 'frame_id': 'l_wrist'},
                               'poses': [{'pose': {'position': [0, 0, 0]}} for _ in range(5)]}
                glove.received = time.monotonic()
                with patch.object(module.Path, 'home', return_value=root):
                    self.assertTrue(glove.start()[0])
                self.assertTrue(glove.status()['active'])
                self.assertTrue(glove.stop()[0])
                self.assertFalse(glove.status()['active'])
            finally:
                glove.close()

    def test_official_arms_only_has_no_finger_controller(self):
        self.assertNotIn('--ee', command('wuji'))
        vr = command('vr')
        self.assertEqual(vr[vr.index('--ee') + 1], 'dex5')
        for mode in ('vr', 'wuji'):
            argv = command(mode)
            self.assertEqual(argv[argv.index('--arm') + 1], 'H2')
            self.assertEqual(argv[argv.index('--input-mode') + 1], 'hand')

    def test_hybrid_start_activates_glove_then_sends_arm_start(self):
        up, events = controller()
        self.assertTrue(up.start()[0])
        self.assertEqual(events, ['glove_start', 'CMD_START'])

    def test_vr_start_never_activates_glove(self):
        up, events = controller('vr')
        self.assertTrue(up.start()[0])
        self.assertEqual(events, ['CMD_START'])

    def test_glove_start_failure_does_not_start_arm_following(self):
        up, events = controller(start_ok=False)
        self.assertFalse(up.start()[0])
        self.assertEqual(events, ['glove_start'])

    def test_ipc_start_failure_stops_new_glove_publisher(self):
        up, events = controller(ipc_ok=False)
        self.assertFalse(up.start()[0])
        self.assertEqual(events, ['glove_start', 'CMD_START', 'glove_stop'])

    def test_stop_ends_both_controllers(self):
        up, events = controller(active=True)
        self.assertTrue(up.stop()[0])
        self.assertEqual(events, ['glove_stop', 'CMD_STOP'])

    def test_stop_still_ends_glove_after_arm_process_is_lost(self):
        up, events = controller(active=True)
        up.online = lambda: False
        self.assertTrue(up.stop()[0])
        self.assertEqual(events, ['glove_stop'])

    def test_switching_source_requires_both_controllers_stopped(self):
        up, _ = controller()
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'source.json'
            with patch.object(web, 'HAND_SOURCE_PATH', path):
                up.running = lambda: True
                self.assertFalse(up.set_hand_source('vr')[0])
                self.assertFalse(path.exists())
                up.running = lambda: False
                up.wuji.active = True
                self.assertFalse(up.set_hand_source('vr')[0])
                up.wuji.active = False
                self.assertTrue(up.set_hand_source('vr')[0])
                self.assertEqual(json.loads(path.read_text()), {'source': 'vr'})

    def test_receiving_input_does_not_create_any_motor_process(self):
        with tempfile.TemporaryDirectory() as temporary:
            glove = module.WujiHand(Path(temporary), 'python3')
            try:
                with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as connection:
                    frame = {'header': {'seq': 1, 'frame_id': 'l_wrist'},
                             'poses': [{'pose': {'position': [0, 0, 0]}} for _ in range(5)]}
                    connection.sendto(json.dumps(frame).encode(), str(glove.path))
                deadline = time.monotonic() + 1
                while not glove.status()['connected'] and time.monotonic() < deadline:
                    time.sleep(.01)
                self.assertTrue(glove.status()['connected'])
                self.assertFalse(glove.status()['active'])
                self.assertIsNone(glove.process)
            finally:
                glove.close()


if __name__ == '__main__': unittest.main()
