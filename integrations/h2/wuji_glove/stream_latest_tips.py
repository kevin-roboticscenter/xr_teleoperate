#!/usr/bin/env python3
"""Forward current fingertip positions using the official Wuji SDK callback API."""
import argparse
import json
import sys
import threading
import time

from wuji_sdk import SdkManager


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sn', required=True)
    parser.add_argument('--side', choices=('left', 'right'), default='left')
    args = parser.parse_args()
    manager = SdkManager.instance()
    condition = threading.Condition()
    shared = {}
    subscription = None

    def receive(frame):
        if frame.header.frame_id != args.side[0]+'_wrist':
            failed(f'Expected {args.side} glove frames; received {frame.header.frame_id}')
            return
        callback_at_us = time.time_ns() // 1000
        value = {'header': {'frame_id': frame.header.frame_id,
                            'seq': frame.header.seq,
                            'timestamp_us': frame.header.timestamp_us},
                 'poses': [{'pose': {'position': list(p.pose.position)},
                            'confidence': p.confidence} for p in frame.poses],
                 'callback_at_us': callback_at_us}
        with condition:
            previous = shared.get('callback_at_us')
            value['callback_gap_ms'] = (callback_at_us - previous) / 1000 if previous else 0.
            shared['callback_at_us'] = callback_at_us
            shared['frame'] = value
            condition.notify()

    def failed(message):
        with condition:
            shared['error'] = message
            condition.notify()

    try:
        glove = manager.connect(sn=args.sn, device_name=args.side+'_glove')
        subscription = glove.tip_poses().subscribe_with_callback(receive, on_error=failed)
        last_sequence = None
        while True:
            with condition:
                condition.wait_for(lambda: 'error' in shared or (
                    'frame' in shared and shared['frame']['header']['seq'] != last_sequence))
                if 'error' in shared:
                    raise RuntimeError(shared['error'])
                value = dict(shared['frame'])
            last_sequence = value['header']['seq']
            value['forwarded_at_us'] = time.time_ns() // 1000
            print(json.dumps(value, separators=(',', ':')), flush=True)
    finally:
        if subscription is not None:
            subscription.close()
        manager.disconnect_all()


if __name__ == '__main__':
    try:
        main()
    except (KeyboardInterrupt, BrokenPipeError):
        pass
