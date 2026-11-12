"""Receive calibrated glove input; START/STOP own the existing left-hand adapter."""
import fcntl
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import threading
import time
import xml.etree.ElementTree as ET


class Dex5Feedback:
    """Read-only hand availability and raw driver feedback."""
    NAMES = [name for finger in (2, 3, 4, 5) for name in
             (f'Roll_{finger}1L', f'Pitch_{finger}2L', f'Pitch_{finger}3L', f'Pitch_{finger}4L')]
    NAMES += ['Yaw_11L', 'Roll_12L', 'Pitch_13L', 'Pitch_14L']

    def __init__(self, root, side='left'):
        if side not in ('left', 'right'): raise ValueError('Invalid hand side')
        self.side = side
        from unitree_sdk2py.core.channel import ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandState_
        suffix = 'L' if side == 'left' else 'R'
        tree = ET.parse(Path(root) / f'upstream/dex5-pr321/assets/unitree_hand_Dex5/Dex5-URDF-{suffix}.urdf')
        self.limits = []
        for name in [n[:-1]+suffix for n in self.NAMES]:
            limit = tree.find(f"joint[@name='{name}']/limit")
            self.limits.append((float(limit.get('lower')), float(limit.get('upper'))))
        self.lock, self.data, self.received = threading.Lock(), None, 0.
        self.subscriber = ChannelSubscriber(f'rt/dex5/{side}/state', HandState_)
        self.subscriber.Init(self._receive, 10)

    def _receive(self, message):
        data = {'q': [float(x.q) for x in message.motor_state],
                'modes': [int(x.mode) for x in message.motor_state],
                'temperatures': [list(x.temperature) for x in message.motor_state[:16]],
                'motor_flags': [list(x.reserve) for x in message.motor_state[:16]],
                'errors': list(message.error)}
        with self.lock: self.data, self.received = data, time.monotonic()

    @staticmethod
    def assess(data, age, limits, side='left'):
        result = dict(data or {})
        reason = ''
        if data is None or age is None or age > 1:
            reason = 'Waiting for fresh left-hand feedback'
        else:
            q = data['q']
            enabled = sum(x != 0 for x in data['modes'][:16])
            result['enabled_entries'] = enabled
            # Healthy open-hand feedback includes small encoder zero offsets
            # (observed right Pitch_42R=-0.027 rad with all motors enabled).
            margin = math.radians(2.)
            invalid = [i for i, (value, (lo, hi)) in enumerate(zip(q, limits))
                       if not math.isfinite(value) or not lo - margin <= value <= hi + margin]
            result['invalid_joint_indices'] = invalid
            if len(q) != 20: reason = 'Left-hand joint feedback is incomplete'
            elif enabled != 16: reason = f'Left hand: {enabled}/16 motor entries enabled'
            elif any(data['errors']): reason = 'Left hand reports a driver error'
            elif invalid: reason = 'Left-hand joint feedback is outside the model range'
        result.update(ready=not reason, message=reason or 'Left-hand feedback ready',
                      age_ms=round(age * 1000, 1) if age is not None else None)
        result['message'] = result['message'].replace('Left', side.title()).replace('left', side)
        return result

    def status(self):
        with self.lock:
            return self.assess(self.data, time.monotonic() - self.received if self.received else None, self.limits, self.side)

    def close(self): self.subscriber.Close()


class WujiHand:
    def __init__(self, root, python, interface='eth0', feedback=None, side='left'):
        if side not in ('left', 'right'): raise ValueError('Invalid hand side')
        self.side = side
        self.root, self.python, self.interface = Path(root), python, interface
        self.feedback = feedback
        self.path = self.root / 'wuji_glove' / ('input.sock' if side == 'left' else 'input_right.sock')
        self.path.parent.mkdir(exist_ok=True)
        self.lease = self.path.with_suffix('.lock').open('a')
        fcntl.flock(self.lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
        self.path.unlink(missing_ok=True)
        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.socket.bind(str(self.path))
        self.path.chmod(0o600)
        self.socket.settimeout(.2)
        self.lock = threading.RLock()
        self.closed = threading.Event()
        self.writer_stop = threading.Event()
        self.frame, self.received, self.process = None, 0., None
        self.error, self.log_path = '', None
        self.reader = threading.Thread(target=self._receive, daemon=True)
        self.reader.start()

    def _receive(self):
        while not self.closed.is_set():
            try:
                frame = json.loads(self.socket.recv(65536))
                assert frame['header']['frame_id'] == self.side[0]+'_wrist'
                assert len(frame['poses']) == 5
                assert all(len(p['pose']['position']) == 3 and all(
                    math.isfinite(float(x)) for x in p['pose']['position']) for p in frame['poses'])
                with self.lock:
                    self.frame, self.received = frame, time.monotonic()
            except socket.timeout:
                pass
            except (ValueError, KeyError, TypeError, AssertionError):
                continue
            except OSError:
                if not self.closed.is_set(): self.error = 'Glove input socket closed'
                break

    def status(self):
        with self.lock:
            age = time.monotonic() - self.received if self.received else None
            active = self.process is not None and self.process.poll() is None
            return {'connected': age is not None and age < 2, 'active': active,
                    'hand_feedback': self.feedback.status() if self.feedback is not None else None,
                    'input_age_ms': round(age * 1000, 1) if age is not None else None,
                    'side': self.side, 'error': self.error,
                    'log_file': str(self.log_path or '')}

    def conflict(self):
        """Identify another writer before creating the left command publisher."""
        for entry in Path('/proc').iterdir():
            if not entry.name.isdigit(): continue
            try:
                argv = [x.decode() for x in (entry / 'cmdline').read_bytes().split(b'\0') if x]
                if not argv or not Path(argv[0]).name.startswith('python'): continue
                if any(x.endswith('/left_dex5_adapter.py') for x in argv) and '--publish-'+self.side in argv:
                    if self.process is None or int(entry.name) != self.process.pid:
                        return f'Stop the existing {self.side}-hand test first'
                if any(x.endswith(('/teleop_hand_and_arm.py', '/head_camera/browser_teleop.py')) for x in argv) and '--ee' in argv:
                    if argv[argv.index('--ee') + 1] == 'dex5':
                        return 'VR finger control is running; STOP before switching to Wuji'
            except (OSError, UnicodeError, IndexError): pass
        return ''

    def start(self):
        with self.lock:
            if self.status()['active']: return True, 'Wuji already active'
            if self.feedback is not None:
                feedback = self.feedback.status()
                if not feedback['ready']: return False, feedback['message']
            conflict = self.conflict()
            if conflict: return False, conflict
            if not self.status()['connected']: return False, f'Connect the calibrated {self.side} Wuji glove first'
            logs = Path.home() / 'h2-logs'
            logs.mkdir(exist_ok=True)
            self.log_path = logs / f'wuji_{self.side}_dashboard_{time.strftime("%Y%m%dT%H%M%S")}.log'
            ready_path = logs / f'wuji_ready_{time.time_ns()}.flag'
            args = [self.python, '-u', str(self.root / 'wuji_glove/left_dex5_adapter.py'),
                    '--publish-'+self.side, '--side', self.side, '--seconds', '0', '--interface', self.interface,
                    '--ready-file', str(ready_path)]
            profile = self.root / f'wuji_glove/config/{self.side}_dex5_retarget.json'
            if profile.exists(): args += ['--retarget-profile', str(profile)]
            else: args += ['--smoothing-alpha', '0.5']
            with self.log_path.open('a') as log:
                self.process = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=log,
                                                stderr=subprocess.STDOUT, text=True,
                                                bufsize=1, start_new_session=True)
            self.writer_stop = threading.Event()
            self.error = ''
            threading.Thread(target=self._write, args=(self.process, self.writer_stop), daemon=True).start()
            process = self.process
        deadline = time.monotonic() + 12
        while time.monotonic() < deadline:
            if process.poll() is not None:
                lines = self.log_path.read_text(errors='replace').splitlines()
                self.error = 'Wuji activation failed: ' + (lines[-1] if lines else 'adapter exited')
                return False, self.error
            if ready_path.exists(): return True, f'Wuji {self.side} fingers activated'
            time.sleep(.02)
        self.stop()
        self.error = 'Wuji activation timed out before valid hand feedback and input arrived'
        return False, self.error

    def _write(self, process, stop):
        sequence = None
        try:
            while not stop.wait(.005) and process.poll() is None:
                with self.lock: frame = self.frame
                if frame is not None and frame['header']['seq'] != sequence:
                    process.stdin.write(json.dumps(frame, separators=(',', ':')) + '\n')
                    process.stdin.flush()
                    sequence = frame['header']['seq']
        except (OSError, ValueError) as exc:
            if not stop.is_set(): self.error = f'Glove adapter input ended: {exc}'
        finally:
            try: process.stdin.close()
            except OSError: pass

    def stop(self):
        with self.lock:
            process = self.process
            self.writer_stop.set()
            if process is None or process.poll() is not None: return True, 'Wuji is stopped'
            process.send_signal(signal.SIGINT)
            try: process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.error = 'Wuji adapter has not exited; STOP again'
                return False, self.error
            self.process = None
            return True, 'Wuji hand publisher stopped; hand keeps its last pose'

    def close(self):
        self.stop()
        self.closed.set()
        self.socket.close()
        self.reader.join(timeout=1)
        self.path.unlink(missing_ok=True)
        self.lease.close()
        if self.feedback is not None: self.feedback.close()


class WujiHands:
    """Own each finger publisher; combined START either activates both or rolls back."""
    def __init__(self, left, right, dual=False):
        self.hands = {'left': left, 'right': right}
        self.dual = dual
        self.test_side = None

    @property
    def required_sides(self):
        return ('left', 'right') if self.dual else ('left',)

    def set_dual(self, dual):
        if self.status()['active']: raise ValueError('STOP before changing glove mode')
        self.dual = bool(dual)

    def status(self):
        hands = {side: hand.status() for side, hand in self.hands.items()}
        result = dict(hands['left'])
        result['hands'] = hands
        result['required_sides'] = list(self.required_sides)
        result['test_side'] = self.test_side
        result['connected'] = all(hands[side]['connected'] for side in self.required_sides)
        result['active'] = any(value['active'] for value in hands.values())
        result['error'] = next((hands[side]['error'] for side in self.required_sides if hands[side]['error']), '')
        feedback = [hands[side].get('hand_feedback') for side in self.required_sides]
        if all(value is not None for value in feedback):
            result['hand_feedback'] = next((value for value in feedback if not value['ready']), feedback[0])
        return result

    def conflict(self):
        return next((reason for side in self.required_sides if (reason := self.hands[side].conflict())), '')

    def start(self):
        self.test_side = None
        for side in self.required_sides:
            status = self.hands[side].status()
            if not status['connected']: return False, f'Connect the calibrated {side} Wuji glove first'
            feedback = status.get('hand_feedback')
            if feedback is not None and not feedback['ready']: return False, feedback['message']
            reason = self.hands[side].conflict()
            if reason: return False, reason
        for side in self.required_sides:
            ok, message = self.hands[side].start()
            if not ok:
                self.stop()
                return False, message
        return True, 'Both Wuji hands activated' if self.dual else 'Wuji left fingers activated'

    def start_test(self, side):
        if side not in self.hands: raise ValueError('Choose left or right hand')
        if self.status()['active']: return False, 'STOP the existing hand test first'
        ok, message = self.hands[side].start()
        if ok: self.test_side = side
        return ok, message

    def stop(self):
        results = [hand.stop() for hand in self.hands.values()]
        self.test_side = None
        return all(ok for ok, _ in results), '; '.join(message for _, message in results)

    def close(self):
        for hand in self.hands.values(): hand.close()
