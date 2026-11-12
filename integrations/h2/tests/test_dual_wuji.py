"""Dual-device routing and atomic START/STOP without hardware or motor channels."""
import json
from pathlib import Path
import socket
import tempfile
import time
import unittest
from unittest.mock import Mock, patch

from test_hybrid_hands import module, controller
from pico_tracker.input import TrackerInput
from quest_h2_teleop import command
from wuji_glove import dashboard_gateway as gateway


class Hand:
    def __init__(self, side, events, connected=True, ready=True, start_ok=True):
        self.side, self.events = side, events
        self.connected, self.ready, self.start_ok = connected, ready, start_ok
        self.active = False

    def status(self):
        return {'connected': self.connected, 'active': self.active, 'error': '',
                'hand_feedback': {'ready': self.ready, 'message': self.side+' feedback'}}

    def start(self):
        self.events.append(self.side+'_start')
        self.active = self.start_ok
        return self.start_ok, self.side+' start'

    def stop(self):
        self.events.append(self.side+'_stop')
        self.active = False
        return True, self.side+' stopped'

    def conflict(self): return ''


def group(**right_options):
    events = []
    left, right = Hand('left', events), Hand('right', events, **right_options)
    return module.WujiHands(left, right, dual=True), events


class DualOwnership(unittest.TestCase):
    def test_small_encoder_zero_offset_is_accepted_but_large_bad_feedback_is_rejected(self):
        data = {'q': [-.027]+[0.]*19, 'modes': [1]*16+[0]*4, 'errors': [0, 0]}
        limits = [(-.001, 1.)]*20
        self.assertTrue(module.Dex5Feedback.assess(data, .01, limits, 'right')['ready'])
        data['q'][0] = -4.8
        self.assertFalse(module.Dex5Feedback.assess(data, .01, limits, 'right')['ready'])

    def test_both_gloves_start_before_arm_start(self):
        hands, events = group()
        up, _ = controller('pico_dual_wuji')
        up.wuji = hands
        up.tracker = Mock()
        up.tracker.status.return_value = {'ready': True}
        up._send = lambda cmd: (events.append(cmd) or True, 'ok')
        self.assertTrue(up.tracker_status()['ready'])
        with up.lock:
            self.assertTrue(up._begin_following()[0])
        self.assertEqual(events, ['left_start', 'right_start', 'CMD_START'])
        up.tracker.status.assert_called_with(both=True)

    def test_right_only_test_starts_no_left_or_arm_controller(self):
        hands, events = group()
        hands.hands['left'].connected = False
        up, _ = controller('pico_dual_wuji')
        up.wuji, up.grasp = hands, None
        up._send = Mock()
        self.assertTrue(up.start_wuji_test('right')[0])
        self.assertEqual(events, ['right_start'])
        self.assertEqual(hands.status()['test_side'], 'right')
        up._send.assert_not_called()
        up.online = lambda: False
        self.assertTrue(up.stop()[0])
        self.assertFalse(hands.status()['active'])

    def test_right_only_test_rejects_existing_arm_or_grasp_ownership(self):
        hands, events = group()
        up, _ = controller('pico_dual_wuji')
        up.wuji, up.grasp = hands, None
        up.running = lambda: True
        self.assertFalse(up.start_wuji_test('right')[0])
        self.assertEqual(events, [])

    def test_standalone_grasp_cannot_overlap_active_wuji_fingers(self):
        hands, events = group()
        hands.hands['right'].active = True
        up, _ = controller('pico_dual_wuji')
        up.wuji, up.grasp = hands, Mock()
        self.assertFalse(up.grasp_start()[0])
        up.grasp.start.assert_not_called()

    def test_missing_right_input_or_feedback_never_starts_left_publisher(self):
        for options in ({'connected': False}, {'ready': False}):
            hands, events = group(**options)
            self.assertFalse(hands.start()[0])
            self.assertEqual(events, [])
            self.assertFalse(hands.status()['active'])

    def test_right_activation_failure_rolls_back_both_publishers(self):
        hands, events = group(start_ok=False)
        self.assertFalse(hands.start()[0])
        self.assertEqual(events, ['left_start', 'right_start', 'left_stop', 'right_stop'])
        self.assertFalse(hands.status()['active'])

    def test_failed_arm_start_stops_both_finger_publishers(self):
        hands, events = group()
        up, _ = controller('pico_dual_wuji')
        up.wuji = hands
        up.tracker = Mock()
        up.tracker.status.return_value = {'ready': True}
        up._send = lambda cmd: (events.append(cmd) or False, 'failed')
        with up.lock:
            self.assertFalse(up._begin_following()[0])
        self.assertEqual(events, ['left_start', 'right_start', 'CMD_START', 'left_stop', 'right_stop'])

    def test_stop_ends_both_hands_after_arm_heartbeat_loss(self):
        hands, events = group()
        up, _ = controller('pico_dual_wuji')
        up.wuji = hands
        up.online = lambda: False
        hands.hands['left'].active = hands.hands['right'].active = True
        self.assertTrue(up.stop()[0])
        self.assertEqual(events, ['left_stop', 'right_stop'])

    def test_legacy_left_mode_does_not_require_or_start_right_glove(self):
        hands, events = group(connected=False, ready=False)
        hands.set_dual(False)
        self.assertTrue(hands.status()['connected'])
        self.assertTrue(hands.start()[0])
        self.assertEqual(events, ['left_start'])

    def test_dual_launch_explicitly_enables_both_arm_targets(self):
        self.assertIn('--both-arms', command('pico_dual_wuji'))
        self.assertNotIn('--both-arms', command('pico_wuji'))
        self.assertNotIn('--ee', command('pico_dual_wuji'))

    def test_left_and_right_sockets_reject_opposite_hand_frames(self):
        with tempfile.TemporaryDirectory() as temporary:
            left = module.WujiHand(Path(temporary), 'python3')
            right = module.WujiHand(Path(temporary), 'python3', side='right')
            try:
                frame = {'header': {'seq': 1, 'frame_id': 'r_wrist'},
                         'poses': [{'pose': {'position': [0, 0, 0]}} for _ in range(5)]}
                with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as sender:
                    for hand in (left, right): sender.sendto(json.dumps(frame).encode(), str(hand.path))
                deadline = time.monotonic()+1
                while not right.status()['connected'] and time.monotonic() < deadline: time.sleep(.01)
                self.assertTrue(right.status()['connected'])
                self.assertFalse(left.status()['connected'])
                self.assertFalse(right.status()['active'])
                self.assertNotEqual(left.path, right.path)
            finally:
                left.close(); right.close()


class DualTrackers(unittest.TestCase):
    def test_both_trackers_must_be_distinct_and_live(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            path = root/'input.json'
            pose = [0., 1., 0., 0., 0., 0., 1.]
            frame = {'received_mono': time.monotonic(), 'trackers': {'a': pose, 'b': pose},
                     'pose_age_ms': {'a': 0, 'b': 0}, 'head_pose': pose}
            path.write_text(json.dumps(frame))
            trackers = TrackerInput(root, path)
            self.assertTrue(trackers.select('a', 'left')[0])
            self.assertFalse(trackers.select('a', 'right')[0])
            self.assertFalse(trackers.status(both=True)['ready'])
            self.assertTrue(trackers.select('b', 'right')[0])
            self.assertTrue(trackers.status(both=True)['ready'])
            self.assertTrue(trackers.select('', 'right')[0])
            self.assertFalse(trackers.status(both=True)['ready'])
            self.assertTrue(trackers.select('b', 'right')[0])
            frame['pose_age_ms']['b'] = 2000
            path.write_text(json.dumps(frame))
            self.assertFalse(trackers.status(both=True)['ready'])
            self.assertTrue(trackers.status()['ready'])


class LaptopRouting(unittest.TestCase):
    def test_uncalibrated_right_glove_never_starts_a_source_or_relay(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root/'config').mkdir()
            (root/'config/wuji_gloves.json').write_text(json.dumps({'right': 'RIGHT-SN'}))
            bridge = gateway.GloveBridge('right')
            response = Mock(stdout=json.dumps({'right_hand': {'calibrated': False}}))
            with patch.object(gateway, 'ROOT', root), patch.object(gateway.subprocess, 'run', return_value=response), patch.object(gateway.subprocess, 'Popen') as spawn:
                ok, message = bridge.connect()
                self.assertFalse(ok)
                self.assertIn('Calibrate the right', message)
                spawn.assert_not_called()

    def test_undefined_right_serial_never_selects_a_device_automatically(self):
        with tempfile.TemporaryDirectory() as temporary:
            bridge = gateway.GloveBridge('right')
            with patch.object(gateway, 'ROOT', Path(temporary)), patch.object(gateway.subprocess, 'Popen') as spawn:
                self.assertFalse(bridge.connect()[0])
                spawn.assert_not_called()
