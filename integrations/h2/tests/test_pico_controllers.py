"""Controller wrist sources and Wuji ownership without hardware or motor writes."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import Mock, patch

from pico_tracker.controllers import (CONTROLLER_SOURCE, CONTROLLER_IDS,
                                      controller_frame, read_controllers)
from pico_tracker.input import TrackerInput
from pico_tracker.pose_match import PoseReference
from pico_tracker import stream_trackers
from quest_h2_teleop import command
from wuji_glove import dashboard_gateway as gateway
from test_hybrid_hands import controller, web
from test_pico_optical_hands import frame as optical_frame

POSE = [.2, 1.1, -.4, 0., 0., 0., 1.]


def frame():
    return {'received_mono': time.monotonic(), 'head_pose': POSE,
            'trackers': {'physical-wrist': POSE}, 'pose_age_ms': {'physical-wrist': 0},
            'controllers': {side: {'pose': list(POSE), 'pose_age_ms': 0.}
                            for side in ('left', 'right')}}


class ControllerInput(unittest.TestCase):
    def test_reads_official_controller_getters_and_keeps_each_pose_age(self):
        sdk = Mock()
        sdk.get_left_controller_pose.return_value = POSE
        sdk.get_right_controller_pose.return_value = POSE
        previous, changed = {}, {}
        self.assertEqual(read_controllers(sdk, previous, changed, 10.)['left']['pose_age_ms'], 0)
        sdk.get_right_controller_pose.return_value = [0.]*7
        data = read_controllers(sdk, previous, changed, 11.5)
        self.assertEqual(data['left']['pose_age_ms'], 1500.)
        self.assertIsNone(data['right']['pose'])
        sdk.get_left_hand_tracking_state.assert_not_called()
        sdk.get_motion_tracker_pose.assert_not_called()

    def test_no_fallback_to_optical_hands_or_physical_wrist_trackers(self):
        data = frame()
        del data['controllers']['right']
        output = controller_frame(data)
        self.assertEqual(set(output['trackers']), {CONTROLLER_IDS['left']})
        self.assertIn('physical-wrist', data['trackers'])
        self.assertEqual(controller_frame(dict(data, controllers=None))['trackers'], {})

    def test_both_controller_poses_required_without_glove_or_optical_pose_substitution(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            path = root/'latest.json'
            data = frame()
            path.write_text(json.dumps(data))
            inputs = TrackerInput(root, path)
            self.assertTrue(inputs.status(both=True, controllers=True)['ready'])
            data['controllers']['right']['pose_age_ms'] = 1500
            path.write_text(json.dumps(data))
            self.assertFalse(inputs.status(both=True, controllers=True)['ready'])
            data['controllers']['right']['pose_age_ms'] = float('nan')
            path.write_text(json.dumps(data))
            self.assertFalse(inputs.status(both=True, controllers=True)['ready'])
            data['received_mono'] -= 2
            path.write_text(json.dumps(data))
            self.assertFalse(inputs.status(both=True, controllers=True)['connected'])

    def test_switching_to_controller_roles_preserves_physical_tracker_assignments(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root/'config').mkdir()
            path = root/'config/pico_tracker.json'
            original = {'left_sn': 'physical-left', 'right_sn': 'physical-right',
                        'offset_tracker_m': [.1, .2, .3]}
            path.write_text(json.dumps(original))
            inputs = TrackerInput(root)
            self.assertEqual(inputs.config(controllers=True)['left_sn'], CONTROLLER_IDS['left'])
            self.assertNotIn('offset_tracker_m', inputs.config(controllers=True))
            self.assertEqual(inputs.config(), original)
            self.assertEqual(json.loads(path.read_text()), original)
            self.assertNotEqual(PoseReference(root).path, PoseReference(root, controllers=True).path)

    def test_native_controller_launch_uses_both_arms_and_dashboard_owned_fingers(self):
        args = command(CONTROLLER_SOURCE)
        self.assertIn('--both-arms', args)
        self.assertIn('--controllers', args)
        self.assertNotIn('--pico-hands', args)
        self.assertNotIn('--ee', args)
        self.assertNotIn('--controllers', command('pico_dual_wuji'))


class ControllerOwnership(unittest.TestCase):
    def test_controller_start_and_stop_keep_gloves_under_existing_dashboard_owner(self):
        up, events = controller(CONTROLLER_SOURCE)
        up.tracker = Mock()
        up.tracker.status.return_value = {'ready': True}
        self.assertTrue(up.tracker_status()['ready'])
        up.tracker.status.assert_called_with(both=True, controllers=True)
        with up.lock:
            self.assertTrue(up._begin_following()[0])
        self.assertEqual(events, ['glove_start', 'CMD_START'])
        self.assertTrue(up.stop()[0])
        self.assertEqual(events[-2:], ['glove_stop', 'CMD_STOP'])

    def test_missing_controller_blocks_start_before_glove_activation(self):
        up, events = controller(CONTROLLER_SOURCE)
        up.tracker = Mock()
        up.tracker.status.return_value = {'ready': False, 'message': 'Turn on right controller'}
        self.assertFalse(up.start()[0])
        self.assertEqual(events, [])

    def test_mode_selection_requires_both_gloves_and_persists_new_option(self):
        up, _ = controller()
        up.wuji.set_dual = Mock()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'source.json'
            with patch.object(web, 'HAND_SOURCE_PATH', path):
                self.assertTrue(up.set_hand_source(CONTROLLER_SOURCE)[0])
            up.wuji.set_dual.assert_called_once_with(True)
            self.assertEqual(json.loads(path.read_text())['source'], CONTROLLER_SOURCE)

    def test_laptop_connects_both_glove_streams_in_controller_mode(self):
        bridges = gateway.GloveBridges()
        bridges.hands = {side: Mock() for side in ('left', 'right')}
        for hand in bridges.hands.values(): hand.connect.return_value = (True, 'input')
        connection = Mock()
        connection.getresponse.return_value.status = 200
        connection.getresponse.return_value.read.return_value = json.dumps({'hand_source': CONTROLLER_SOURCE}).encode()
        with patch.object(gateway, 'HTTPConnection', return_value=connection), patch.object(gateway.h2_link, 'selected_host', return_value='robot'):
            self.assertTrue(bridges.connect()[0])
        self.assertTrue(bridges.dual)
        for hand in bridges.hands.values(): hand.connect.assert_called_once_with()


class ControllerTransport(unittest.TestCase):
    def test_controller_packets_continue_with_frozen_motion_tracker_timestamp(self):
        sdk = Mock()
        sdk.get_time_stamp_ns.side_effect = [10, 10, 11, 11]
        sdk.get_motion_timestamp_ns.return_value = 5
        sdk.get_motion_tracker_pose.return_value = []
        sdk.get_motion_tracker_serial_numbers.return_value = []
        sdk.get_headset_pose.return_value = POSE
        sdk.get_left_controller_pose.return_value = POSE
        sdk.get_right_controller_pose.return_value = POSE
        sdk.get_left_hand_is_active.return_value = 0
        sdk.get_right_hand_is_active.return_value = 0
        with patch.dict(sys.modules, {'xrobotoolkit_sdk': sdk}), \
             patch.object(stream_trackers.os, 'dup', return_value=99), \
             patch.object(stream_trackers.os, 'dup2'), \
             patch.object(stream_trackers.os, 'set_blocking'), \
             patch.object(stream_trackers.os, 'close'), \
             patch.object(stream_trackers.os, 'write') as write, \
             patch.object(stream_trackers.time, 'sleep', side_effect=[None, KeyboardInterrupt]):
            with self.assertRaises(KeyboardInterrupt): stream_trackers.main()
        packets = [json.loads(call.args[1]) for call in write.call_args_list]
        self.assertEqual([p['packet_timestamp_ns'] for p in packets], [10, 11])
        self.assertTrue(all(p['timestamp_source'] == 'packet' for p in packets))
        self.assertEqual(set(packets[-1]['controllers']), {'left', 'right'})
        sdk.init.assert_called_once_with()
        sdk.close.assert_called_once_with()

    def test_complete_optical_tracker_and_controller_frame_fits_atomic_pipe(self):
        data = optical_frame()
        pose = [.123456789]*3+[.5]*4
        data.update(version=1, source='xrobotoolkit_sdk', seq=1234567,
                    trackers={f'PICO_TEST_{i}': pose for i in range(3)}, head_pose=pose,
                    sample_at_unix_ns=1234567890123456789, sample_mono=123456.123456789,
                    packet_timestamp_ns=1234567890123456789, motion_timestamp_ns=0,
                    pose_age_ms={f'PICO_TEST_{i}': 1234.56 for i in range(3)},
                    controllers={side: {'pose': pose, 'pose_age_ms': 1234.56} for side in ('left', 'right')})
        self.assertLessEqual(len((json.dumps(data, separators=(',', ':'))+'\n').encode()), 4096)
