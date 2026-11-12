import math
from types import SimpleNamespace
import unittest

from head_camera.head_follow import HeadPoseMapper, HeadCommandPublisher, LIMITS


def pose(axis=None, angle=0):
    xyz = [0., 0., 0.]
    if axis is not None:
        xyz[axis] = math.sin(angle/2)
    return [0., 0., 0., *xyz, math.cos(angle/2)]


class HeadCoordinates(unittest.TestCase):
    def test_neutral_preserves_measured_neck_without_jump(self):
        for p, y in [(0., 0.), (.3, -.8), (-.2, 1.6)]:
            mapper = HeadPoseMapper(pose(1, .9), (p, y))
            result = mapper.target(pose(1, .9))
            self.assertAlmostEqual(result[0], p)
            self.assertAlmostEqual(result[1], y)

    def test_pico_left_turn_is_robot_positive_yaw(self):
        p, y = HeadPoseMapper(pose(), (0., 0.)).target(pose(1, .4))
        self.assertAlmostEqual(y, .4)
        self.assertAlmostEqual(p, 0.)

    def test_pico_look_up_is_robot_negative_pitch(self):
        p, y = HeadPoseMapper(pose(), (0., 0.)).target(pose(0, .3))
        self.assertAlmostEqual(p, -.3)
        self.assertAlmostEqual(y, 0.)

    def test_roll_does_not_command_neck_roll(self):
        self.assertEqual(HeadPoseMapper(pose(), (0., 0.)).target(pose(2, .6)), (0., 0.))

    def test_robot_mechanical_limits(self):
        mapper = HeadPoseMapper(pose(), (0., 0.))
        self.assertEqual(mapper.target(pose(1, 2.)), (0., LIMITS[1][1]))
        self.assertEqual(mapper.target(pose(0, 1.)), (LIMITS[0][0], 0.))


class SamePublisher(unittest.TestCase):
    def test_neck_is_inserted_before_crc_and_arms_are_preserved(self):
        msg = SimpleNamespace(motor_cmd=[SimpleNamespace(q=.12, mode=1, kp=4., kd=2., dq=0., tau=.7)
                                        for _ in range(35)], crc=0)
        records = []

        class CRC:
            def Crc(self, m):
                records.append(('crc', m.motor_cmd[29].q, m.motor_cmd[30].q))
                return 1234

        class Publisher:
            def Write(self, m):
                records.append(('write', m.motor_cmd[29].q, m.motor_cmd[30].q))
                self_assert.assertEqual(m.crc, 1234)
                self_assert.assertEqual(m.motor_cmd[15].q, .12)
                self_assert.assertEqual(m.motor_cmd[15].tau, .7)
                self_assert.assertEqual(m.motor_cmd[29].kp, 30.)
                self_assert.assertEqual(m.motor_cmd[30].kd, 1.)

        self_assert = self
        ticks = iter([1., 1.004])
        adapter = HeadCommandPublisher(Publisher(), CRC(), (0., 0.), clock=lambda: next(ticks))
        adapter.set_target((-.2, .3))
        adapter.Write(msg)
        self.assertEqual([r[0] for r in records], ['crc', 'write'])
        self.assertEqual(records[0][1:], records[1][1:])
        self.assertAlmostEqual(msg.motor_cmd[29].q, -.04)
        self.assertAlmostEqual(msg.motor_cmd[30].q, .04)

    def test_nonfinite_target_rejected(self):
        adapter = HeadCommandPublisher(None, None, (0., 0.))
        with self.assertRaises(ValueError):
            adapter.set_target((float('nan'), 0.))


if __name__ == '__main__':
    unittest.main()
