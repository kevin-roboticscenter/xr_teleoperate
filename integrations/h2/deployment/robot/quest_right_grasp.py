#!/usr/bin/env python3
"""Explicit right-only Dex5 grasp. Runs on PC2 independently of HTTP/Quest/Wi-Fi.

The preset is a model-bounded joint-position target, not a measured-force grip or
an official Unitree grasp preset. No commands are sent at import/dashboard startup.
"""
import argparse
from contextlib import contextmanager
import fcntl
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading
import time
import uuid

RUNTIME = Path.home() / '.cache/h2-right-grasp'
RIGHT_CMD = 'rt/dex5/right/cmd'
RIGHT_STATE = 'rt/dex5/right/state'
# PR #321 DDS order: four fingers (spread, MCP, PIP, DIP), then thumb.
JOINT_NAMES = tuple(f'{kind}_{finger}{joint}R' for finger in (2, 3, 4, 5)
                    for kind, joint in (('Roll', 1), ('Pitch', 2), ('Pitch', 3), ('Pitch', 4))) + (
                        'Yaw_11R', 'Roll_12R', 'Pitch_13R', 'Pitch_14R')
FULL_GRASP = (0.0, 1.35, 1.55, 1.25) * 4 + (-0.35, 1.55, 1.25, 1.15)
OPEN_HAND = (0.0,) * 20


def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    name = None
    try:
        with tempfile.NamedTemporaryFile('w', dir=path.parent, delete=False) as handle:
            name = handle.name
            json.dump(value, handle, allow_nan=False)
        os.replace(name, path)
    finally:
        if name and os.path.exists(name):
            os.unlink(name)


def read_json(path):
    try:
        return json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return {}


def quest_pids():
    result = subprocess.run(['pgrep', '-f', r'^[^ ]*python[0-9.]* [^ ]*teleop_hand_and_arm\.py'],
                            capture_output=True, text=True)
    return [int(pid) for pid in result.stdout.split()]


def worker_busy(directory):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    with (directory / 'worker.lock').open('a') as handle:
        try:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            return True
    return False


class GraspManager:
    """File IPC is local to PC2; losing the remote browser has no control effect."""
    def __init__(self, gain_store, interface='eth0', directory=RUNTIME):
        self.gain_store, self.interface, self.directory = gain_store, interface, Path(directory)

    @contextmanager
    def teleop_slot(self):
        self.directory.mkdir(parents=True, exist_ok=True)
        with (self.directory / 'action.lock').open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            yield

    def status(self):
        busy = worker_busy(self.directory)
        state = read_json(self.directory / 'status.json')
        age = max(0, time.time() - state.get('updated', 0))
        if busy and (not state or age > 2):
            state = dict(state, phase='unresponsive', message='Grasp process is active; status unavailable. Hand position is unconfirmed.')
        if not busy and state.get('phase') not in ('released', 'error', 'interrupted'):
            state = dict(state, phase='idle', message='Right-hand hold is inactive.')
        return dict(state, active=busy, status_age=age if state else None)

    def start(self, teleop_running):
        with self.teleop_slot():
            if teleop_running() or quest_pids():
                return False, 'Stop Quest teleop before using standalone right-hand grasp.'
            if self.status()['active']:
                return False, 'Right-hand grasp is already active.'
            profile = self.gain_store.load()
            session = str(uuid.uuid4())
            args = [sys.executable, str(Path(__file__).resolve()), '--worker',
                    '--directory', str(self.directory), '--interface', self.interface,
                    '--session', session, '--finger', str(profile.finger_multiplier),
                    '--thumb', str(profile.thumb_multiplier)]
            with (self.directory / 'worker.log').open('a') as log:
                process = subprocess.Popen(args, stdin=subprocess.DEVNULL, stdout=log,
                                           stderr=subprocess.STDOUT, start_new_session=True,
                                           close_fds=True)
            deadline = time.monotonic() + 6
            while time.monotonic() < deadline:
                state = self.status()
                if state.get('session') == session:
                    if state.get('phase') == 'error':
                        return False, state.get('message', 'Grasp failed')
                    if state.get('phase') in ('closing', 'holding'):
                        return True, 'Right hand closing. H2 will keep holding through Wi-Fi loss.'
                if process.poll() is not None:
                    return False, state.get('message', 'Grasp process exited; check worker.log')
                time.sleep(.05)
            # Never kill/restart a possibly commanding process on an HTTP timeout.
            return False, 'Grasp startup is unconfirmed. Check right-hand hold status before retrying.'

    def release(self):
        with self.teleop_slot():
            state = self.status()
            if not state['active'] or not state.get('session'):
                return False, 'No active right-hand grasp process; no hand command sent.'
            atomic_json(self.directory / 'request.json', {'session': state['session'], 'action': 'release'})
            return True, 'Release requested. Wait for “Released” feedback before starting Quest teleop.'


class GraspMotion:
    """Timed trajectory retaining the closed target after contact with an object."""
    def __init__(self, initial, now):
        if len(initial) != 20 or not all(math.isfinite(q) for q in initial):
            raise ValueError('Expected 20 finite right-hand joint positions')
        self.position = tuple(initial)
        self.begin(FULL_GRASP, now, 'closing')

    def begin(self, target, now, phase):
        self.origin, self.target, self.started, self.phase = self.position, target, now, phase
        self.duration = max(3.0, max(abs(a-b) for a, b in zip(self.origin, target)) / .6)

    def step(self, now, release=False):
        if release and self.phase not in ('releasing', 'released'):
            self.begin(OPEN_HAND, now, 'releasing')
        ratio = min(1.0, max(0.0, (now - self.started) / self.duration))
        self.position = tuple(a + (b-a)*ratio for a, b in zip(self.origin, self.target))
        if ratio == 1:
            self.phase = 'released' if self.phase in ('releasing', 'released') else 'holding'
        return self.position


class RightDds:
    def __init__(self, interface, profile):
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_, HandState_
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__HandCmd_, unitree_hg_msg_dds__MotorCmd_
        self.lock, self.q, self.last, self.error = threading.Lock(), None, 0., None
        ChannelFactoryInitialize(0, interface)
        self.subscriber = ChannelSubscriber(RIGHT_STATE, HandState_)
        self.subscriber.Init(self.receive, 10)
        self.publisher = ChannelPublisher(RIGHT_CMD, HandCmd_)
        self.publisher.Init()
        self.message = unitree_hg_msg_dds__HandCmd_()
        self.message.motor_cmd = [unitree_hg_msg_dds__MotorCmd_() for _ in range(20)]
        for index, cmd in enumerate(self.message.motor_cmd):
            cmd.mode, cmd.dq, cmd.tau = 1, 0., 0.
            cmd.kp = profile.thumb_kp if index >= 16 else profile.finger_kp
            cmd.kd = profile.thumb_kd if index >= 16 else profile.finger_kd

    def receive(self, msg):
        q = tuple(float(m.q) for m in msg.motor_state)
        if len(q) == 20 and all(math.isfinite(value) for value in q):
            with self.lock:
                self.q, self.last = q, time.monotonic()
                self.error = list(msg.error)

    def feedback(self):
        with self.lock:
            return self.q, time.monotonic() - self.last, self.error

    def write(self, positions):
        for cmd, q in zip(self.message.motor_cmd, positions):
            cmd.q = q
        if self.publisher.Write(self.message) is False:
            raise RuntimeError('Right-hand DDS write failed; physical hand position is unconfirmed.')


def run_worker(args, profile, dds_factory=RightDds):
    directory = Path(args.directory)
    directory.mkdir(parents=True, exist_ok=True)
    with (directory / 'worker.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            return 1
        state = {'session': args.session, 'pid': os.getpid(), 'gains': profile.describe(), 'writes': 0}
        def report(phase, message, **extra):
            state.update(phase=phase, message=message, updated=time.time(), **extra)
            atomic_json(directory / 'status.json', state)
        interrupted = threading.Event()
        def interrupt(*_):
            interrupted.set()
        signal.signal(signal.SIGTERM, interrupt)
        signal.signal(signal.SIGINT, interrupt)
        try:
            report('starting', 'Waiting for right-hand joint feedback; no commands sent yet.')
            if quest_pids():
                raise RuntimeError('Quest teleop is running; no standalone grasp commands sent.')
            dds = dds_factory(args.interface, profile)
            deadline = time.monotonic() + 5
            while True:
                q, age, error = dds.feedback()
                if interrupted.is_set():
                    raise RuntimeError('Startup interrupted; no hand commands sent.')
                if q is not None and age < .5:
                    if error and any(error):
                        raise RuntimeError(f'Right hand reports errors {error}; no commands sent.')
                    break
                if time.monotonic() >= deadline:
                    raise RuntimeError('No fresh right-hand feedback; no grasp commands sent.')
                time.sleep(.01)
            motion, last_report, released_at = GraspMotion(q, time.monotonic()), 0., None
            while not interrupted.is_set():
                tick = time.monotonic()
                request = read_json(directory / 'request.json')
                release = request.get('session') == args.session and request.get('action') == 'release'
                dds.write(motion.step(tick, release))
                state['writes'] += 1
                if tick - last_report >= .25:
                    _, age, error = dds.feedback()
                    report(motion.phase, {'closing': 'Closing full right hand…',
                           'holding': 'Holding right hand on H2. Wi-Fi and Quest are not required.',
                           'releasing': 'Opening right hand…',
                           'released': 'Open target sent; finishing release.'}[motion.phase],
                           feedback_age=round(age, 3), hand_error=error)
                    last_report = tick
                if motion.phase == 'released':
                    released_at = tick if released_at is None else released_at
                    if tick - released_at >= 1:
                        q, age, _ = dds.feedback()
                        confirmed = q is not None and age < .5 and max(abs(a-b) for a,b in zip(q, OPEN_HAND)) < .2
                        report('released', 'Released; open feedback confirmed.' if confirmed else
                               'Open target sent; physical opening is unconfirmed.', open_confirmed=confirmed)
                        return 0
                time.sleep(max(0., .01 - (time.monotonic() - tick)))
            report('interrupted', 'Grasp process stopped publishing. Hand position is unconfirmed; it was not commanded open.')
            return 1
        except Exception as exc:
            report('error', str(exc))
            return 1


def main(argv=None, dds_factory=RightDds):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--worker', action='store_true', required=True)
    parser.add_argument('--directory', type=Path, default=RUNTIME)
    parser.add_argument('--interface', default='eth0')
    parser.add_argument('--session', required=True)
    parser.add_argument('--finger', type=float, required=True)
    parser.add_argument('--thumb', type=float, required=True)
    args = parser.parse_args(argv)
    from quest_h2_teleop import UPSTREAM  # sets upstream SDK utilities import path
    from teleop.utils.dex5_gains import GainProfile
    return run_worker(args, GainProfile(args.finger, args.thumb), dds_factory)


if __name__ == '__main__':
    sys.exit(main())
