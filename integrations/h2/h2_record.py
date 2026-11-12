#!/usr/bin/env python3
"""Record one take on PC2: head/wrist video + existing joint data, read-only.

Sends nothing to the robot. Started/stopped (SIGINT) by quest_h2_web.py's REC button,
or by hand:  ~/anaconda3/envs/tv/bin/python h2_record.py --out ~/h2-recordings/test

Output directory:
  cam_a.mjpeg / cam_b.mjpeg   camera or TeleImager JPEG bytes, one JPEG per frame
  head.mp4                    head-camera H.264 video when --head-camera is selected
  cam_a_frames.csv / ...      frame index, wall-clock time (PC2 time.time()), byte size
  joints.csv                  50 Hz snapshot: arm state/cmd (14 + 14), Dex5 state/cmd (40 + 40); empty = no fresh data
  meta.json                   settings, column groups, start/end, frame/row counts, topic rates
  status.json                 live status for the control page (rewritten every 0.5 s)
All timestamps share PC2's clock, so tools/render_take.py can line the charts up with the video.
"""
import argparse
import csv
import json
import os
from pathlib import Path
import signal
import socket
import threading
import time

import cv2

# The two UVC hand-camera feeds (A/B; physical left/right not yet confirmed). By USB port, since
# /dev/videoN numbers can shift and both report the same name and serial.
CAMS = {
    'cam_a': '/dev/v4l/by-path/pci-0000:00:14.0-usb-0:1:1.0-video-index0',
    'cam_b': '/dev/v4l/by-path/pci-0000:00:14.0-usb-0:6:1.0-video-index0',
}
ARM_JOINTS = [  # H2_JointArmIndex 15..28 in robot_arm.py
    'l_shoulder_pitch', 'l_shoulder_roll', 'l_shoulder_yaw', 'l_elbow', 'l_wrist_roll', 'l_wrist_pitch', 'l_wrist_yaw',
    'r_shoulder_pitch', 'r_shoulder_roll', 'r_shoulder_yaw', 'r_elbow', 'r_wrist_roll', 'r_wrist_pitch', 'r_wrist_yaw']
ARM_INDEX = list(range(15, 29))
HAND_JOINTS = [  # Dex5 DDS index order (robot_hand_unitree.py / tools/dex5_monitor.py)
    'Roll_21', 'Pitch_22', 'Pitch_23', 'Pitch_24', 'Roll_31', 'Pitch_32', 'Pitch_33', 'Pitch_34',
    'Roll_41', 'Pitch_42', 'Pitch_43', 'Pitch_44', 'Roll_51', 'Pitch_52', 'Pitch_53', 'Pitch_54',
    'Yaw_11', 'Roll_12', 'Pitch_13', 'Pitch_14']
FRESH = 0.25   # seconds; older DDS samples are written as empty cells


class Camera(threading.Thread):
    def __init__(self, name, path, out, size, fps, stop, stream_port=None):
        super().__init__(daemon=True)
        self.name, self.path, self.out, self.size, self.fps, self.stop = name, path, out, size, fps, stop
        self.frames, self.bytes, self.rate, self.error = 0, 0, 0.0, ''
        self.stream_port = stream_port

    def _open(self):
        cap = cv2.VideoCapture(self.path, cv2.CAP_V4L2)
        if not cap.isOpened():
            raise RuntimeError(f'cannot open {self.path}')
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.size[0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.size[1])
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)   # hand back the camera's own JPEG bytes
        return cap

    def run(self):
        if self.stream_port is not None:
            return self._run_stream()
        cap = None
        with open(self.out / f'{self.name}.mjpeg', 'wb') as video, \
                open(self.out / f'{self.name}_frames.csv', 'w', newline='') as index:
            log = csv.writer(index)
            log.writerow(['frame', 't', 'bytes'])
            window, failures = [], 0
            while not self.stop.is_set():
                try:
                    if cap is None:
                        cap = self._open()
                    ok, frame = cap.read()
                    t = time.time()
                    if not ok:
                        failures += 1
                        if failures >= 30:
                            raise RuntimeError('no frames for 30 reads')
                        continue
                    failures = 0
                    if frame.ndim == 2 and frame.shape[0] == 1:
                        data = frame.tobytes()
                    else:   # backend decoded anyway: re-encode
                        data = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 92])[1].tobytes()
                    video.write(data)
                    log.writerow([self.frames, f'{t:.6f}', len(data)])
                    self.frames += 1
                    self.bytes += len(data)
                    window = [x for x in window if t - x < 1.0] + [t]
                    self.rate, self.error = len(window), ''
                    if self.frames % 30 == 0:
                        index.flush()
                except Exception as exc:
                    self.error, self.rate = str(exc), 0.0
                    if cap is not None:
                        cap.release()
                        cap = None
                    self.stop.wait(1.0)
            if cap is not None:
                cap.release()

    def _run_stream(self):
        # Share TeleImager's existing capture instead of competing for the USB camera.
        from teleimager.image_client import ZMQ_SubscriberManager
        subscriber = ZMQ_SubscriberManager.get_instance()
        previous, window = None, []
        with open(self.out / f'{self.name}.mjpeg', 'wb') as video, \
                open(self.out / f'{self.name}_frames.csv', 'w', newline='') as index:
            log = csv.writer(index)
            log.writerow(['frame', 't', 'bytes'])
            while not self.stop.wait(1 / self.fps):
                try:
                    frame = subscriber.subscribe('127.0.0.1', self.stream_port, request_bgr=False)
                    if frame.jpg is None or frame.jpg is previous:
                        continue
                    previous, t = frame.jpg, time.time()
                    video.write(frame.jpg)
                    log.writerow([self.frames, f'{t:.6f}', len(frame.jpg)])
                    self.frames += 1
                    self.bytes += len(frame.jpg)
                    window = [x for x in window if t - x < 1] + [t]
                    self.rate, self.error = len(window), ''
                except Exception as exc:
                    self.error = str(exc)
                    self.stop.wait(.25)


class Joints(threading.Thread):
    """Latest-sample snapshots of the robot's DDS topics at a fixed rate."""

    def __init__(self, out, hz, interface, stop):
        super().__init__(daemon=True)
        self.out, self.hz, self.stop = out, hz, stop
        self.rows, self.error = 0, ''
        self.latest, self.counts, self.rates = {}, {}, {}
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_, HandState_, LowCmd_, LowState_
        ChannelFactoryInitialize(0, interface)
        topics = {'arm_state': ('rt/lowstate', LowState_), 'arm_cmd': ('rt/arm_sdk', LowCmd_)}
        for side in ('left', 'right'):
            topics[f'{side}_hand_state'] = (f'rt/dex5/{side}/state', HandState_)
            topics[f'{side}_hand_cmd'] = (f'rt/dex5/{side}/cmd', HandCmd_)
        self.subs = []
        for key, (topic, kind) in topics.items():
            sub = ChannelSubscriber(topic, kind)
            sub.Init(self._callback(key), 10)
            self.subs.append(sub)
            self.counts[key] = 0

    def _callback(self, key):
        def store(msg):
            self.latest[key] = (time.time(), msg)
            self.counts[key] += 1
        return store

    def _values(self, key, now, field, indices):
        sample = self.latest.get(key)
        if sample is None or now - sample[0] > FRESH:
            return [''] * len(indices)
        items = getattr(sample[1], field)
        return [f'{items[i].q:.5f}' if i < len(items) else '' for i in indices]

    @staticmethod
    def header():
        cols = ['t']
        cols += [f'arm_state.{j}' for j in ARM_JOINTS] + [f'arm_cmd.{j}' for j in ARM_JOINTS]
        for kind in ('state', 'cmd'):
            for side, s in (('left', 'L'), ('right', 'R')):
                cols += [f'{side}_hand_{kind}.{j}{s}' for j in HAND_JOINTS]
        return cols

    def run(self):
        hand = list(range(len(HAND_JOINTS)))
        with open(self.out / 'joints.csv', 'w', newline='') as f:
            log = csv.writer(f)
            log.writerow(self.header())
            last_counts, last_rate_t = dict(self.counts), time.time()
            next_t = time.time()
            while not self.stop.is_set():
                now = time.time()
                row = [f'{now:.6f}']
                row += self._values('arm_state', now, 'motor_state', ARM_INDEX)
                row += self._values('arm_cmd', now, 'motor_cmd', ARM_INDEX)
                for kind, field in (('state', 'motor_state'), ('cmd', 'motor_cmd')):
                    for side in ('left', 'right'):
                        row += self._values(f'{side}_hand_{kind}', now, field, hand)
                log.writerow(row)
                self.rows += 1
                if now - last_rate_t >= 1.0:
                    self.rates = {k: round((self.counts[k] - last_counts[k]) / (now - last_rate_t))
                                  for k in self.counts}
                    last_counts, last_rate_t = dict(self.counts), now
                    f.flush()
                next_t += 1.0 / self.hz
                self.stop.wait(max(0.0, next_t - time.time()))
        for sub in self.subs:
            try:
                sub.Close()
            except Exception:
                pass


def write_json(path, data):
    tmp = path.with_suffix('.tmp')
    tmp.write_text(json.dumps(data, indent=1))
    os.replace(tmp, path)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--out', required=True)
    parser.add_argument('--size', default='1920x1080', help='camera capture size, MJPEG (1920x1080 or 1280x720)')
    parser.add_argument('--fps', type=int, default=30)
    parser.add_argument('--camera-streams', action='store_true', help='Record the existing TeleImager wrist streams')
    parser.add_argument('--head-camera', action='store_true', help='Include the existing PC3 head-camera stream as head.mp4')
    parser.add_argument('--joint-hz', type=float, default=50.0)
    parser.add_argument('--max-minutes', type=float, default=20.0, help='stop by itself after this long')
    parser.add_argument('--network-interface', default='eth0')
    args = parser.parse_args()
    out = Path(args.out).expanduser()
    out.mkdir(parents=True, exist_ok=False)
    size = tuple(int(v) for v in args.size.lower().split('x'))

    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())

    started = time.time()
    meta = {'host': socket.gethostname(), 'started': started,
            'started_local': time.strftime('%Y-%m-%d %H:%M:%S %Z', time.localtime(started)),
            'cameras': {n: {'path': p, 'size': (480, 640) if args.camera_streams else size, 'fps': args.fps,
                            'source': 'teleimager' if args.camera_streams else 'uvc',
                            'rotation': 'already upright' if args.camera_streams else 'cw90 to stand upright'}
                        for n, p in CAMS.items()},
            'joint_hz': args.joint_hz, 'clock': 'PC2 time.time() for frames and joint rows',
            'joints': {'arm': ARM_JOINTS, 'hand': HAND_JOINTS, 'units': 'rad',
                       'state': 'measured (rt/lowstate, rt/dex5/*/state)',
                       'cmd': 'commanded = the action (rt/arm_sdk, rt/dex5/*/cmd)'}}
    write_json(out / 'meta.json', meta)

    cams = [Camera(n, p, out, size, args.fps, stop,
                   stream_port=(55556 if n == 'cam_a' else 55557) if args.camera_streams else None)
            for n, p in CAMS.items()]
    if args.head_camera:
        from head_camera.recording import HeadVideo
        head = HeadVideo(out, args.fps, stop, Path(__file__).parent/'head_camera/receiver.json')
        cams.append(head)
        meta['cameras']['head'] = dict(head.config, source='teleimager', file='head.mp4', codec='H.264',
                                       rotation='already upright', fps=args.fps)
        write_json(out/'meta.json', meta)
    for cam in cams:
        cam.start()
    joints = None
    try:
        joints = Joints(out, args.joint_hz, args.network_interface, stop)
        joints.start()
    except Exception as exc:
        meta['joints_error'] = str(exc)
    print(f'recording to {out}', flush=True)

    def status(final=False):
        elapsed = time.time() - started
        return {'dir': str(out), 'elapsed': round(elapsed, 1), 'final': final,
                'cams': {c.name: {'frames': c.frames, 'fps': c.rate, 'mb': round(c.bytes / 1e6, 1), 'error': c.error}
                         for c in cams},
                'joints': {'rows': joints.rows if joints else 0, 'rates': joints.rates if joints else {},
                           'error': meta.get('joints_error', '')}}

    while not stop.wait(0.5):
        write_json(out / 'status.json', status())
        if time.time() - started > args.max_minutes * 60:
            print('max duration reached', flush=True)
            stop.set()
    for thread in cams + ([joints] if joints else []):
        thread.join(timeout=5)
        if thread.is_alive() and getattr(thread, 'name', '') == 'head':
            thread.finish_encoder(force=True)
            thread.join(timeout=1)
    meta.update(ended=time.time(), duration=round(time.time() - started, 2), final_status=status(True))
    write_json(out / 'meta.json', meta)
    write_json(out / 'status.json', status(True))
    print(f'saved {out}', flush=True)


if __name__ == '__main__':
    main()
