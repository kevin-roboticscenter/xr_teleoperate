#!/usr/bin/env python3
"""Adapt official Wuji CLI skeleton JSONL to the existing Unitree Dex5 retargeter.

Default: compute targets only. --publish-left or --publish-right enables only
the selected hand's command topic. No arm or body command channel is created.
"""
import argparse
import json
import math
import os
from pathlib import Path
import sys
import threading
import time

import numpy as np

NAMES = ['wrist', 'thumb_cmc', 'thumb_mcp', 'thumb_ip', 'thumb_tip',
         'index_finger_mcp', 'index_finger_pip', 'index_finger_dip', 'index_finger_tip',
         'middle_finger_mcp', 'middle_finger_pip', 'middle_finger_dip', 'middle_finger_tip',
         'ring_finger_mcp', 'ring_finger_pip', 'ring_finger_dip', 'ring_finger_tip',
         'pinky_mcp', 'pinky_pip', 'pinky_dip', 'pinky_tip']
WEBXR_SLOTS = [0, 1, 2, 3, 4, 6, 7, 8, 9, 11, 12, 13, 14, 16, 17, 18, 19, 21, 22, 23, 24]
# Wuji left wrist: +X thumb, +Y dorsal, -Z fingertips.
# Dex5 left base:  +X thumb, +Y fingertips, +Z dorsal.
WUJI_TO_DEX5_LEFT = np.array([[1., 0., 0.], [0., 0., -1.], [0., 1., 0.]])
# Both native wrist frames have +X toward the thumb and -Z along the fingers.
# Their palm normals mirror, as do the left/right Dex5 base-frame Z axes.
WUJI_TO_DEX5_RIGHT = WUJI_TO_DEX5_LEFT.copy()


def landmarks(frame, side='left'):
    if side not in ('left', 'right') or frame['header']['frame_id'] != side[0]+'_wrist':
        raise ValueError(f'Expected calibrated {side}-wrist skeleton')
    transform = WUJI_TO_DEX5_LEFT if side == 'left' else WUJI_TO_DEX5_RIGHT
    result = np.zeros((25, 3))
    if 'poses' in frame:
        points = np.asarray([p['pose']['position'] for p in frame['poses']], dtype=float)
        if points.shape != (5, 3) or not np.isfinite(points).all():
            raise ValueError('Invalid Wuji fingertip poses')
        # Official tip_poses are already wrist-relative, in thumb-to-pinky order.
        result[[4, 9, 14, 19, 24]] = points @ transform.T
    else:
        joints = {j['name']: j for j in frame['joints']}
        points = np.asarray([joints[n]['pose']['position'] for n in NAMES], dtype=float)
        if points.shape != (21, 3) or not np.isfinite(points).all():
            raise ValueError('Invalid Wuji skeleton positions')
        result[WEBXR_SLOTS] = (points - points[0]) @ transform.T
    return result


def frame_lines():
    for line in sys.stdin:
        try:
            frame = json.loads(line)
        except json.JSONDecodeError:
            continue
        if 'joints' in frame or 'poses' in frame:
            yield frame


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    publishing = parser.add_mutually_exclusive_group()
    publishing.add_argument('--publish-left', action='store_true')
    publishing.add_argument('--publish-right', action='store_true')
    parser.add_argument('--side', choices=('left', 'right'), default=None)
    parser.add_argument('--seconds', type=float, default=0., help='0 runs until input ends or the operator stops it')
    parser.add_argument('--interface', default='eth0')
    parser.add_argument('--smoothing-alpha', type=float, default=None,
                        help='Official retargeter low_pass_alpha override for this process only')
    parser.add_argument('--scaling-factor', type=float, default=None)
    parser.add_argument('--project-dist', type=float, default=None)
    parser.add_argument('--escape-dist', type=float, default=None)
    parser.add_argument('--retarget-profile', type=Path, default=None,
                        help='Glove-specific settings for the existing official retargeter')
    parser.add_argument('--ready-file', type=Path, default=None)
    parser.add_argument('--trace-jsonl', type=Path)
    parser.add_argument('--clock-offset-ms', type=float, default=0.,
                        help='Estimated robot clock minus glove-host clock, for timing reports')
    args = parser.parse_args()
    side = args.side or ('right' if args.publish_right else 'left')
    if (args.publish_left and side != 'left') or (args.publish_right and side != 'right'):
        parser.error('Publish flag must match the selected hand side')
    if args.smoothing_alpha is not None and not 0 < args.smoothing_alpha <= 1:
        parser.error('--smoothing-alpha must be greater than 0 and at most 1')
    for name in ('scaling_factor', 'project_dist', 'escape_dist'):
        value = getattr(args, name)
        if value is not None and (not math.isfinite(value) or value <= 0):
            parser.error(f'--{name.replace("_", "-")} must be finite and positive')
    if args.trace_jsonl is not None:
        args.trace_jsonl = args.trace_jsonl.resolve()
    if args.retarget_profile is not None:
        args.retarget_profile = args.retarget_profile.resolve()
    if args.ready_file is not None:
        args.ready_file = args.ready_file.resolve()
    root = Path(__file__).resolve().parents[1]
    upstream = root / 'upstream/dex5-pr321'
    sys.path[:0] = [str(upstream), str(upstream / 'teleop/robot_control/dex-retargeting/src')]
    os.chdir(upstream / 'teleop')
    from teleop.robot_control.hand_retargeting import HandRetargeting, HandType
    retarget = HandRetargeting(HandType.UNITREE_DEX5)
    overrides = {}
    if args.retarget_profile is not None:
        profile = args.retarget_profile.resolve()
        overrides.update(json.loads(profile.read_text())['retargeting'])
        allowed = {'urdf_path', 'target_joint_names', 'scaling_factor',
                   'low_pass_alpha', 'project_dist', 'escape_dist'}
        if set(overrides) - allowed:
            parser.error('Unknown retarget profile setting')
        if 'urdf_path' in overrides:
            overrides['urdf_path'] = str((profile.parent / overrides['urdf_path']).resolve())
    overrides.update({key: getattr(args, key) for key in
                      ('scaling_factor', 'project_dist', 'escape_dist')
                      if getattr(args, key) is not None})
    if args.smoothing_alpha is not None:
        overrides['low_pass_alpha'] = args.smoothing_alpha
    if overrides:
        from dex_retargeting import RetargetingConfig
        config = dict(retarget.cfg[side], **overrides)
        if config.get('project_dist', .03) >= config.get('escape_dist', .05):
            parser.error('project_dist must be smaller than escape_dist')
        setattr(retarget, side+'_retargeting', RetargetingConfig.from_dict(config).build())
    sequence_retargeting = getattr(retarget, side+'_retargeting')
    hardware_mapping = getattr(retarget, side+'_dex_retargeting_to_hardware')
    indices = sequence_retargeting.optimizer.target_link_human_indices
    active_alpha = sequence_retargeting.filter.alpha
    if not set(indices.flatten()).issubset(WEBXR_SLOTS):
        raise ValueError('Retargeter needs landmarks absent from the Wuji skeleton')

    def target(frame):
        points = landmarks(frame, side)
        vectors = points[indices[1]] - points[indices[0]]
        q = sequence_retargeting.retarget(vectors)[hardware_mapping]
        if q.shape != (20,) or not np.isfinite(q).all():
            raise ValueError('Retargeter produced invalid joint targets')
        return q

    if not (args.publish_left or args.publish_right):
        values = [target(frame) for frame in frame_lines()]
        if not values:
            raise ValueError('No skeleton frames received')
        values = np.asarray(values)
        print(json.dumps({'mode': 'dry_run', 'side': side, 'frames': len(values), 'command_writes': 0,
                          'smoothing_alpha': active_alpha,
                          'retarget_overrides': overrides,
                          'joint_min': values.min(axis=0).tolist(),
                          'joint_max': values.max(axis=0).tolist(),
                          'last_target': values[-1].tolist()}), flush=True)
        return

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_, HandState_
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__HandCmd_, unitree_hg_msg_dds__MotorCmd_
    from teleop.utils.dex5_gains import GainStore
    gains = GainStore(root / 'config/dex5_gains.json').load()
    shared, lock = {}, threading.Lock()

    def receive_state(message):
        with lock:
            shared['state'] = np.asarray([m.q for m in message.motor_state])
            shared['errors'] = list(message.error)
            shared['feedback_at'] = time.monotonic()

    def read_input():
        try:
            for frame in frame_lines():
                with lock:
                    shared['frame'] = frame
                    shared['received_at'] = time.monotonic()
                    shared['received_wall_us'] = time.time_ns() / 1000
        finally:
            with lock:
                shared['eof'] = True

    ChannelFactoryInitialize(0, args.interface)
    subscriber = ChannelSubscriber(f'rt/dex5/{side}/state', HandState_)
    subscriber.Init(receive_state, 10)
    publisher = ChannelPublisher(f'rt/dex5/{side}/cmd', HandCmd_)
    publisher.Init()
    message = unitree_hg_msg_dds__HandCmd_()
    message.motor_cmd = [unitree_hg_msg_dds__MotorCmd_() for _ in range(20)]
    for i, motor in enumerate(message.motor_cmd):
        motor.mode, motor.dq, motor.tau = 1, 0., 0.
        motor.kp = gains.thumb_kp if i >= 16 else gains.finger_kp
        motor.kd = gains.thumb_kd if i >= 16 else gains.finger_kd
    threading.Thread(target=read_input, daemon=True).start()
    started, last_report, writes, first = None, 0., 0, None
    feedback_min = feedback_max = command_min = command_max = None
    last_sequence = None
    q = None
    solve_ms = 0.
    initial_wait = time.monotonic()
    trace = args.trace_jsonl.open('w', buffering=65536) if args.trace_jsonl else None
    try:
        print(json.dumps({'mode': side+'_only', 'command_topic': f'rt/dex5/{side}/cmd',
                          'pid': os.getpid(),
                          'seconds': args.seconds, 'smoothing_alpha': active_alpha,
                          'retarget_overrides': overrides,
                          'gains': gains.describe()}), flush=True)
        while True:
            tick = time.monotonic()
            with lock:
                frame, state, eof = shared.get('frame'), shared.get('state'), shared.get('eof', False)
                errors = shared.get('errors')
                received_at = shared.get('received_at')
                received_wall_us = shared.get('received_wall_us')
                feedback_at = shared.get('feedback_at')
            sampled_at = time.monotonic()
            if eof:
                break
            if frame is None or state is None:
                if tick - initial_wait > 10.:
                    raise RuntimeError(f'Timed out waiting for glove frames and {side}-hand feedback')
                time.sleep(.01)
                continue
            if started is None:
                limits = sequence_retargeting.optimizer.robot.joint_limits[hardware_mapping]
                margin = math.radians(2.)  # measured encoder zero offsets; optimizer limits stay unchanged
                if (state.shape != (20,) or not np.isfinite(state).all() or
                        np.any(state < limits[:, 0] - margin) or np.any(state > limits[:, 1] + margin)):
                    raise RuntimeError(f'{side.title()}-hand feedback is outside the model range; hand initialization is required')
                started, first = tick, state.copy()
            if args.seconds > 0 and tick - started >= args.seconds:
                break
            sequence = frame['header']['seq']
            if sequence != last_sequence:
                solve_started = time.perf_counter()
                q, last_sequence = target(frame), sequence
                solve_ms = (time.perf_counter() - solve_started) * 1000
            # Follow the official Wuji live-example one-second ease from current pose.
            alpha = min((tick - started) / 1., 1.)
            command = first + alpha * (q - first)
            for i, motor in enumerate(message.motor_cmd):
                motor.q = float(command[i])
            if writes == 0 and args.ready_file is not None:
                args.ready_file.write_text('ready\n')
            publisher.Write(message)
            writes += 1
            timings = {'solve_ms': round(solve_ms, 3),
                       'input_queue_ms': round((sampled_at-received_at)*1000, 3),
                       'feedback_age_ms': round((sampled_at-feedback_at)*1000, 3),
                       'source_to_robot_receive_ms_estimate': round(
                           (received_wall_us-frame['header']['timestamp_us'])/1000-args.clock_offset_ms, 3)}
            if 'forwarded_at_us' in frame:
                timings['sdk_to_forward_ms'] = round(
                    (frame['forwarded_at_us']-frame['header']['timestamp_us'])/1000, 3)
                timings['transport_ms_estimate'] = round(
                    (received_wall_us-frame['forwarded_at_us'])/1000-args.clock_offset_ms, 3)
            if 'callback_at_us' in frame:
                timings['callback_to_forward_ms'] = round(
                    (frame['forwarded_at_us'] - frame['callback_at_us']) / 1000, 3)
                timings['callback_gap_ms'] = frame['callback_gap_ms']
            if trace is not None:
                trace.write(json.dumps({'elapsed': tick-started, 'seq': sequence,
                                        'target': command.tolist(), 'feedback': state.tolist(),
                                        'timings': timings}) + '\n')
            feedback_min = state.copy() if feedback_min is None else np.minimum(feedback_min, state)
            feedback_max = state.copy() if feedback_max is None else np.maximum(feedback_max, state)
            command_min = command.copy() if command_min is None else np.minimum(command_min, command)
            command_max = command.copy() if command_max is None else np.maximum(command_max, command)
            if tick - last_report >= 1.:
                print(json.dumps({'elapsed': round(tick-started, 2), 'writes': writes,
                                  'hand_errors': errors, 'target': command.round(4).tolist(),
                                  'feedback': state.round(4).tolist(), 'timings': timings}), flush=True)
                if trace is not None:
                    trace.flush()
                last_report = tick
            time.sleep(max(0., .01 - (time.monotonic() - tick)))
    finally:
        subscriber.Close()
        publisher.Close()
        if trace is not None:
            trace.close()
        print(json.dumps({'finished': True, 'command_writes': writes,
                          'feedback_range': None if feedback_min is None else (feedback_max-feedback_min).tolist(),
                          'command_range': None if command_min is None else (command_max-command_min).tolist()}), flush=True)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
