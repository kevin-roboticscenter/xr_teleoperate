import json
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest

from head_camera.head_only import hold_upper_body
from pico_tracker.input import TrackerInput
from quest_h2_teleop import command


class HeadOnly(unittest.TestCase):
    def test_hold_uses_measured_upper_body_without_leg_or_zero_targets(self):
        calls = []
        helper = SimpleNamespace(upper_body_joints=list(range(15, 31)),
                                 SetJointCommand=lambda j, q: calls.append((j, q)),
                                 low_cmd=SimpleNamespace(motor_cmd=[SimpleNamespace(q=0.) for _ in range(35)]))
        lowstate = SimpleNamespace(motor_state=[SimpleNamespace(q=.01*j+.1) for j in range(35)],
                                   mode_machine=3)
        neck = hold_upper_body(helper, lowstate)
        self.assertEqual(calls, [(j, lowstate.motor_state[j].q) for j in range(15, 31)])
        self.assertEqual(neck, [lowstate.motor_state[j].q for j in (29, 30)])
        self.assertEqual(helper.low_cmd.mode_machine, 3)
        self.assertEqual(helper.low_cmd.motor_cmd[31].q, 1.)

    def test_head_ready_without_gloves_or_wrist_tracker_assignments(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            frame = root / 'latest.json'
            frame.write_text(json.dumps({'received_mono': time.monotonic(), 'head_pose': [0.,0.,0.,0.,0.,0.,1.],
                                         'trackers': {}, 'pose_age_ms': {}}))
            input = TrackerInput(root, frame_path=frame)
            status = input.status(head_only=True)
            self.assertTrue(status['ready'])
            self.assertEqual(status['required_sides'], [])
            self.assertFalse(input.status()['ready'])

    def test_launch_command_cannot_enter_arm_or_hand_teleop(self):
        args = command('pico_head')
        self.assertTrue(args[1].endswith('/head_camera/head_only.py'))
        self.assertNotIn('--both-arms', args)


if __name__ == '__main__':
    unittest.main()
