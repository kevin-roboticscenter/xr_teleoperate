#!/usr/bin/env python3
"""Native PICO wrist input, unchanged Unitree PR321 H2 IK and arm control.

LAUNCH initializes arms exactly through the existing H2 controller. START
captures a neutral tracker pose and the measured robot wrist transform.
--both-arms enables separate left/right wrist trackers. In left-only mode the
right arm holds its START values. --pico-hands uses optical finger input through
the unchanged Unitree Dex5 controller; other modes use dashboard-owned fingers.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from quest_h2_teleop import UPSTREAM, environment
from pico_tracker.geometry import IncrementalWrist, heading_basis, pose_valid
from pico_tracker.input import read_frame, TrackerInput
from pico_tracker.hands import OpticalFingers, hands_status
from head_camera.head_follow import HeadPoseMapper, HeadCommandPublisher, settings as head_settings


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--both-arms', action='store_true')
    parser.add_argument('--pico-hands', action='store_true')
    args = parser.parse_args()
    # Importing this module for tests never initializes DDS or a motor controller.
    sys.path[:0] = environment()['PYTHONPATH'].split(os.pathsep)
    os.chdir(UPSTREAM / 'teleop')
    import numpy as np
    import pinocchio as pin
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from teleop.robot_control.robot_arm import H2_ArmController
    from teleop.robot_control.robot_arm_ik import H2_ArmIK
    from teleop.utils.ipc import IPC_Server

    inputs = TrackerInput(ROOT)
    config = inputs.config()
    sides = ('left', 'right') if args.both_arms else ('left',)
    serials = {side: config.get(side+'_sn') for side in sides}
    readiness = inputs.status(both=args.both_arms, optical_hands=args.pico_hands)
    if not readiness['ready']:
        raise RuntimeError(readiness['message'])
    state = {'START': False, 'STOP': False, 'READY': False, 'RECORD_RUNNING': False,
             'INPUT': 'pico_tracker', 'LEFT_SN': serials['left'],
             'RIGHT_SN': serials.get('right'), 'TRACKER_LIVE': False}
    lock = threading.RLock()
    controller = ipc = fingers = None
    mappers = {}
    session = None
    right_q = None
    head_enabled = bool(head_settings(ROOT).get('enabled', False))
    head_mapper = head_publisher = None
    state.update(HEAD_FOLLOW=head_enabled, HEAD_LIVE=False)

    def recenter_head(frame):
        nonlocal head_mapper
        if not frame or not pose_valid(frame.get('head_pose')):
            raise ValueError('Enable Head + Send in XRoboToolkit')
        neck = controller.get_current_motor_q()[[29, 30]]
        head_mapper = HeadPoseMapper(frame['head_pose'], neck)
        head_publisher.set_target(neck)
        state.update(HEAD_LIVE=True, HEAD_TARGET=neck.tolist())

    def fk(q):
        pin.forwardKinematics(ik.reduced_robot.model, ik.reduced_robot.data, q)
        pin.updateFramePlacements(ik.reduced_robot.model, ik.reduced_robot.data)
        return [ik.reduced_robot.data.oMf[ik.reduced_robot.model.getFrameId(name)].homogeneous.copy()
                for name in ('L_ee', 'R_ee')]

    def on_press(key):
        nonlocal mappers, session, right_q
        with lock:
            if key == 'q':
                state.update(START=False, STOP=True, READY=False)
                if fingers is not None:
                    fingers.stop()
            elif key == 'h':
                if not state['START'] or not head_enabled:
                    raise ValueError('Start teleop with head following enabled first')
                recenter_head(read_frame())
            elif key == 'r':
                if not state['READY'] or state['STOP'] or state['START']:
                    raise ValueError('Not ready to start')
                frame = read_frame()
                if not frame or any(sn not in frame['trackers'] or
                                    frame.get('pose_age_ms', {}).get(sn, 1e9) >= 1000
                                    for sn in serials.values()):
                    raise ValueError('Move each selected wrist tracker to verify live input')
                basis = heading_basis(frame.get('head_pose'))
                if args.pico_hands and not hands_status(frame)['ready']:
                    raise ValueError(hands_status(frame)['message'])
                q = controller.get_current_dual_arm_q()
                poses = dict(zip(('left', 'right'), fk(q)))
                mappers = {side: IncrementalWrist(frame['trackers'][sn], poses[side].tolist(), basis,
                           config.get('offset_tracker_m' if side == 'left' else 'right_offset_tracker_m', [0., 0., 0.]))
                           for side, sn in serials.items()}
                session, right_q = frame['session'], q[7:].copy()
                if head_enabled:
                    recenter_head(frame)
                if fingers is not None:
                    fingers.start(frame)
                state.update(START=True, TRACKER_LIVE=True, INPUT_MESSAGE='Following wrist trackers')
                controller.speed_gradual_max()
                print(json.dumps({'neutral_captured': True, 'serials': serials,
                                  'trackers': {side: frame['trackers'][sn] for side, sn in serials.items()},
                                  'robot_wrists': {side: poses[side].tolist() for side in sides},
                                  'heading_basis': basis, 'right_q': right_q.tolist()}), flush=True)

    def get_state():
        with lock:
            return dict(state)

    def interrupt(*_):
        on_press('q')

    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    try:
        ik = H2_ArmIK()
        ChannelFactoryInitialize(0, os.environ.get('H2_DDS_INTERFACE', 'eth0'))
        controller = H2_ArmController(motion_mode=True)
        if args.pico_hands:
            fingers = OpticalFingers()
        if head_enabled:
            head_publisher = HeadCommandPublisher(controller.lowcmd_publisher,
                                                  controller.crc,
                                                  controller.get_current_motor_q()[[29, 30]])
            controller.lowcmd_publisher = head_publisher
        ipc = IPC_Server(on_press=on_press, get_state=get_state)
        ipc.cmd_map = dict(ipc.cmd_map, CMD_HEAD_RECENTER='h')
        ipc.start()
        with lock:
            state['READY'] = True
        print(f'READY: native PICO {"both wrists" if args.both_arms else "left wrist"}; press dashboard START in neutral.', flush=True)
        last_seq, last_log = None, 0.
        while not state['STOP']:
            tick = time.monotonic()
            with lock:
                active = state['START']
            if active:
                frame = read_frame()
                if fingers is not None:
                    finger_status = fingers.update(frame)
                    with lock:
                        state['PICO_FINGERS'] = finger_status
                if head_enabled:
                    head_live = bool(frame and frame['session'] == session and
                                     pose_valid(frame.get('head_pose')))
                    with lock:
                        state['HEAD_LIVE'] = head_live
                        if head_live:
                            target = head_mapper.target(frame['head_pose'])
                            head_publisher.set_target(target)
                            state['HEAD_TARGET'] = list(target)
                live = bool(frame and frame['session'] == session and all(
                            sn in frame['trackers'] and pose_valid(frame['trackers'][sn]) and
                            frame.get('pose_age_ms', {}).get(sn, 1e9) < 1000 for sn in serials.values()))
                with lock:
                    state['TRACKER_LIVE'] = live
                    state['INPUT_MESSAGE'] = ('Following wrist trackers' if live else
                                              'Tracker input unavailable; holding last arm target. STOP before reconnecting.')
                if live and frame['seq'] != last_seq:
                    left_target = np.array(mappers['left'].target(frame['trackers'][serials['left']]))
                    q = controller.get_current_dual_arm_q()
                    dq = controller.get_current_dual_arm_dq()
                    _, right_target = fk(q)
                    if args.both_arms:
                        right_target = np.array(mappers['right'].target(frame['trackers'][serials['right']]))
                    solution, _ = ik.solve_ik(left_target, right_target, q, dq)
                    if not args.both_arms:
                        solution[7:] = right_q
                    torque = pin.rnea(ik.reduced_robot.model, ik.reduced_robot.data,
                                      solution, np.zeros(14), np.zeros(14))
                    controller.ctrl_dual_arm(solution, torque)
                    last_seq = frame['seq']
                    if tick-last_log > 1:
                        print(json.dumps({'seq': last_seq, 'left_target': left_target.tolist(), 'right_target': right_target.tolist(),
                                          'measured_q': q.tolist(), 'command_q': solution.tolist(),
                                          'input_age_ms': (tick-frame.get('sample_mono_on_robot', frame['received_mono']))*1000}), flush=True)
                        last_log = tick
            time.sleep(max(0., .01-(time.monotonic()-tick)))
    finally:
        state.update(START=False, STOP=True, READY=False)
        if fingers is not None:
            fingers.close()
        if controller is not None:
            controller.ctrl_dual_arm_go_home()
        if ipc is not None:
            ipc.stop()


if __name__ == '__main__':
    main()
