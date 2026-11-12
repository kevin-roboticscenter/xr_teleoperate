#!/usr/bin/env python3
"""Headset-to-H2 neck test using Unitree's unchanged SDK example helpers.

Prepare subscribes only. START snapshots and holds the existing arm posture;
only the neck follows PICO. No hand publisher or arm-zero/home sequence runs.
"""
import importlib.util
import json
import os
from pathlib import Path
import signal
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from head_camera.head_follow import HeadPoseMapper, HeadCommandPublisher
from pico_tracker.geometry import pose_valid
from pico_tracker.input import read_frame


def hold_upper_body(helper, lowstate):
    """Use official SetJointCommand and gains, retaining measured joint angles."""
    for joint in helper.upper_body_joints:
        helper.SetJointCommand(joint, lowstate.motor_state[joint].q)
    helper.low_cmd.mode_pr = 0
    helper.low_cmd.mode_machine = lowstate.mode_machine
    helper.low_cmd.motor_cmd[31].q = 1.
    return [lowstate.motor_state[j].q for j in (29, 30)]


def main():
    from quest_h2_teleop import environment
    sys.path[:0] = environment()['PYTHONPATH'].split(os.pathsep)
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from teleop.utils.ipc import IPC_Server
    source = ROOT / 'upstream/unitree-h2-head-sdk/h2_arm_sdk_dds_example.py'
    spec = importlib.util.spec_from_file_location('official_h2_upper_body_sdk', source)
    official = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(official)
    lock = threading.RLock()
    state = {'START': False, 'STOP': False, 'READY': False, 'RECORD_RUNNING': False,
             'INPUT': 'pico_head', 'HEAD_FOLLOW': True, 'HEAD_LIVE': False}
    helper = ipc = publisher = mapper = None
    session = None
    writes = 0

    def capture(frame):
        nonlocal mapper
        if not frame or not pose_valid(frame.get('head_pose')):
            raise ValueError('Enable Head + Send in XRoboToolkit')
        if helper.low_state is None:
            raise ValueError('Waiting for H2 joint feedback')
        neck = [helper.low_state.motor_state[j].q for j in (29, 30)]
        mapper = HeadPoseMapper(frame['head_pose'], neck)
        return neck

    def on_press(key):
        nonlocal publisher, session
        with lock:
            if key == 'q':
                state.update(START=False, STOP=True, READY=False)
            elif key == 'r':
                if not state['READY'] or state['START'] or state['STOP']:
                    raise ValueError('Head test is not ready to start')
                frame = read_frame()
                capture(frame)
                neck = hold_upper_body(helper, helper.low_state)
                publisher = HeadCommandPublisher(helper.arm_sdk_publisher, helper.crc, neck)
                session = frame['session']
                state.update(START=True, HEAD_LIVE=True, HEAD_TARGET=neck,
                             INPUT_MESSAGE='Head following; arms held at START pose; fingers idle')
                print(json.dumps({'head_only_start': True, 'head_pose': frame['head_pose'],
                                  'neck': neck, 'arm_hold': [helper.low_state.motor_state[j].q
                                                          for j in helper.upper_body_joints if j < 29]}), flush=True)
            elif key == 'h':
                if not state['START']:
                    raise ValueError('Start head following first')
                neck = capture(read_frame())
                publisher.set_target(neck)
                state['HEAD_TARGET'] = neck

    def get_state():
        with lock:
            return dict(state)

    signal.signal(signal.SIGINT, lambda *_: on_press('q'))
    signal.signal(signal.SIGTERM, lambda *_: on_press('q'))
    try:
        ChannelFactoryInitialize(0, os.environ.get('H2_DDS_INTERFACE', 'eth0'))
        helper = official.Custom()
        helper.Init()
        deadline = time.monotonic()+10
        while helper.low_state is None:
            if state['STOP'] or time.monotonic() > deadline:
                raise RuntimeError('H2 joint feedback unavailable')
            time.sleep(.02)
        ipc = IPC_Server(on_press=on_press, get_state=get_state)
        ipc.cmd_map = dict(ipc.cmd_map, CMD_HEAD_RECENTER='h')
        ipc.start()
        state['READY'] = True
        print('READY: head-only test. START captures neutral; no arm zeroing or hand control.', flush=True)
        last_log = 0.
        while not state['STOP']:
            tick = time.monotonic()
            with lock:
                if state['START']:
                    frame = read_frame()
                    live = bool(frame and frame['session'] == session and pose_valid(frame.get('head_pose')))
                    state['HEAD_LIVE'] = live
                    if live:
                        target = mapper.target(frame['head_pose'])
                        publisher.set_target(target)
                        state['HEAD_TARGET'] = list(target)
                    publisher.Write(helper.low_cmd)
                    writes += 1
                    if tick-last_log > 1:
                        measured = [helper.low_state.motor_state[j].q for j in (29, 30)]
                        print(json.dumps({'head_live': live, 'target': state.get('HEAD_TARGET'),
                                          'measured': measured, 'writes': writes}), flush=True)
                        last_log = tick
            time.sleep(max(0., .004-(time.monotonic()-tick)))
    finally:
        state.update(START=False, STOP=True, READY=False, HEAD_LIVE=False)
        # Release the SDK with Unitree's example fade, retaining the arm pose.
        # A test never returns the arms to zero or publishes finger commands.
        if publisher is not None and writes:
            for i in range(101):
                helper.low_cmd.motor_cmd[31].q = 1-i/100
                publisher.Write(helper.low_cmd)
                time.sleep(.02)
        if ipc is not None:
            ipc.stop()


if __name__ == '__main__':
    main()
