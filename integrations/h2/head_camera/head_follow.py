"""PICO pose adapter for Unitree's existing H2 arm SDK publisher.

Joint IDs and gains follow Unitree's h2_arm_sdk_dds_example.py. Limits and
velocity come from the pinned official H2.urdf. No DDS writer is created here.
"""
import json
import math
from pathlib import Path
import threading
import time
from pico_tracker.geometry import heading_basis, matmul, rotation, transpose

PITCH, YAW = 29, 30
LIMITS = ((-.5236, .83775), (-1.7453, 1.7453))


def settings(root):
    try:
        return json.loads((Path(root) / 'config/head_follow.json').read_text())
    except FileNotFoundError:
        return {'enabled': False}


class HeadPoseMapper:
    def __init__(self, head_pose, neck):
        self.initial = rotation(head_pose)
        self.basis = heading_basis(head_pose)
        p, y = neck
        self.neutral = [[math.cos(p)*math.cos(y), -math.cos(p)*math.sin(y), math.sin(p)],
                        [math.sin(y), math.cos(y), 0.],
                        [-math.sin(p)*math.cos(y), math.sin(p)*math.sin(y), math.cos(p)]]

    def target(self, head_pose):
        delta = matmul(rotation(head_pose), transpose(self.initial))
        mapped = matmul(matmul(self.basis, delta), transpose(self.basis))
        desired = matmul(mapped, self.neutral)
        angles = (math.atan2(desired[0][2], desired[2][2]),
                  math.atan2(desired[1][0], desired[1][1]))
        return tuple(max(lo, min(hi, q)) for q, (lo, hi) in zip(angles, LIMITS))


class HeadCommandPublisher:
    """Insert neck targets in the arm writer thread, then calculate the CRC.

    Updating the controller's shared message from a second thread can invalidate
    its CRC. This adapter performs both operations inside the existing Write call.
    """
    def __init__(self, publisher, crc, initial, clock=time.monotonic):
        self.publisher, self.crc, self.clock = publisher, crc, clock
        self.lock = threading.Lock()
        self.target = tuple(initial)
        self.command = tuple(initial)
        self.previous = clock()

    def set_target(self, target):
        if len(target) != 2 or not all(math.isfinite(q) for q in target):
            raise ValueError('Provide finite head pitch and yaw')
        with self.lock:
            self.target = tuple(max(lo, min(hi, q)) for q, (lo, hi) in zip(target, LIMITS))

    def Write(self, msg):
        now = self.clock()
        # Apply the official URDF's 10 rad/s velocity to the command trajectory.
        step = 10. * max(0., min(now-self.previous, .004))
        self.previous = now
        with self.lock:
            self.command = tuple(q + max(-step, min(step, t-q))
                                 for q, t in zip(self.command, self.target))
            command = self.command
        for joint, q in zip((PITCH, YAW), command):
            motor = msg.motor_cmd[joint]
            motor.mode, motor.q, motor.dq, motor.tau = 1, q, 0., 0.
            motor.kp, motor.kd = 30., 1.
        msg.crc = self.crc.Crc(msg)
        return self.publisher.Write(msg)
