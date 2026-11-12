"""Dashboard-owned official Wuji hand-model calibration; no robot commands."""
from contextlib import contextmanager
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import threading
import uuid


class CalibrationError(ValueError):
    pass


class CalibrationCancelled(Exception):
    pass


def user_name(value):
    if isinstance(value, str):
        return value
    if isinstance(value, dict):
        return value.get('display_name') or value.get('name')
    return None


class CalibrationManager:
    def __init__(self, root, disconnect_inputs, check_robot_idle,
                 run=subprocess.run, popen=subprocess.Popen, state_root=None):
        self.root = Path(root)
        self.disconnect_inputs, self.check_robot_idle = disconnect_inputs, check_robot_idle
        self.run, self.popen = run, popen
        self.state_root = Path(state_root or Path.home()/'.local/state/h2-teleop/wuji-calibration')
        self.lock = threading.RLock()
        self.process = self.worker = None
        self.busy = False
        self.cancel_requested = self.cancel_sent = False
        self.state = {'phase': 'idle', 'message': 'Choose a profile, glove and hand to calibrate.'}
        self.setup_cache = {}

    def active(self):
        return self.busy

    @contextmanager
    def idle_operation(self):
        with self.lock:
            if self.active():
                raise CalibrationError('Finish or cancel the active calibration first.')
            yield

    def status(self):
        with self.lock:
            return dict(self.state, active=self.active())

    def cli(self, *args, timeout=15):
        result = self.run(['wuji', *args, '--json'], stdin=subprocess.DEVNULL,
                          capture_output=True, text=True, timeout=timeout)
        try:
            data = json.loads(result.stdout)
        except (ValueError, TypeError):
            raise CalibrationError((result.stderr or result.stdout or 'Wuji CLI returned no JSON.').strip()[-1000:])
        if not isinstance(data, dict):
            raise CalibrationError('Wuji CLI returned an unexpected JSON document.')
        if result.returncode or data.get('error'):
            error = data.get('error', {})
            raise CalibrationError(str(error.get('message') or result.stderr or 'Wuji CLI command failed.'))
        return data

    def setup(self, scan=False):
        with self.lock:
            if self.active():
                return dict(self.setup_cache, calibration=self.status())
            profiles = self.cli('user', 'list')
            current = self.cli('user', 'show')
            if scan or 'devices' not in self.setup_cache:
                devices = self.cli('devices').get('devices', [])
                self.setup_cache['devices'] = [d for d in devices if d.get('device_type') in (None, 'wuji_glove')]
            config = self.root/'config/wuji_gloves.json'
            self.setup_cache.update(profiles=profiles.get('users', []), current=current,
                                    assignments=json.loads(config.read_text()) if config.exists() else {})
            return dict(self.setup_cache, calibration=self.status())

    @staticmethod
    def validate_selection(payload):
        profile, side, sn = payload.get('profile'), payload.get('side'), payload.get('sn')
        if (not isinstance(profile, str) or not profile.strip() or len(profile) > 128 or
                profile.startswith('-') or any(c in profile for c in ('/', '\\', '\x00'))):
            raise CalibrationError('Choose a valid named SDK profile.')
        if side not in ('left', 'right'):
            raise CalibrationError('Choose left or right hand.')
        if not isinstance(sn, str) or not re.fullmatch(r'[A-Za-z0-9._-]{1,80}', sn):
            raise CalibrationError('Choose the glove serial number.')
        return profile, side, sn

    def profile(self, action, name):
        if (not isinstance(name, str) or not name.strip() or len(name) > 128 or
                name.startswith('-') or any(c in name for c in ('/', '\\', '\x00'))):
            raise CalibrationError('Enter a valid profile name.')
        if action not in ('create', 'switch'):
            raise CalibrationError('Choose create or switch profile.')
        with self.idle_operation():
            self.check_robot_idle()
            self.disconnect_inputs()
            args = ['user', action, name]
            if action == 'create':
                args += ['--switch']
            result = self.cli(*args)
            if user_name(result.get('current_user')) != name:
                raise CalibrationError('The CLI did not confirm the selected profile.')
            self.setup_cache = {}
            return result

    def _current(self, expected):
        current = self.cli('user', 'show')
        if user_name(current) != expected:
            raise CalibrationError('The active SDK profile changed. Refresh and select the intended profile.')
        if current.get('is_default') or expected.casefold() == 'default':
            raise CalibrationError('Create or select a named profile; Default cannot store a hand model.')
        return current

    def _hand_side(self, sn, expected):
        result = self.cli('get', 'hand_side', '--sn', sn)
        side = str(result.get('value', '')).lower()
        if side not in ('left', 'right'):
            raise CalibrationError('The glove did not report a valid hand side.')
        if side != expected:
            raise CalibrationError(f'This glove reports {side}. Select the {side} hand or a different glove.')

    def _assign(self, side, sn):
        path = self.root/'config/wuji_gloves.json'
        values = json.loads(path.read_text()) if path.exists() else {}
        other = 'right' if side == 'left' else 'left'
        if values.get(other) == sn:
            raise CalibrationError('This glove is assigned to the other hand. Select the matching device.')
        values[side] = sn
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix('.tmp')
        fd = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
        with os.fdopen(fd, 'w') as output:
            json.dump(values, output)
            output.write('\n')
        temporary.replace(path)

    def assign(self, payload):
        profile, side, sn = self.validate_selection(payload)
        with self.idle_operation():
            self.check_robot_idle()
            self.disconnect_inputs()
            current = self._current(profile)
            if not current.get(side+'_hand', {}).get('calibrated'):
                raise CalibrationError(f'Calibrate this profile’s {side} hand first.')
            self._hand_side(sn, side)
            self._assign(side, sn)
            self.setup_cache = {}
            return {'message': f'{side.title()} glove assigned. Reconnect Wuji input on the teleop dashboard.'}

    def start(self, payload):
        profile, side, sn = self.validate_selection(payload)
        if payload.get('ready') is not True:
            raise CalibrationError('Confirm you are wearing the selected glove and ready to perform the poses.')
        with self.idle_operation():
            self.cancel_requested = self.cancel_sent = False
            self.busy = True
            self.state = {'phase': 'preparing', 'message': 'Checking profile and glove…',
                          'profile': profile, 'side': side, 'sn': sn, 'feedback': None,
                          'result': None, 'backup': None, 'exit_code': None}
            self.worker = threading.Thread(target=self._calibrate, args=(dict(payload),), daemon=False)
            self.worker.start()
        return self.status()

    def _cancel_checkpoint(self):
        with self.lock:
            if self.cancel_requested:
                raise CalibrationCancelled()

    def _set(self, **values):
        with self.lock:
            self.state.update(values)

    def _calibrate(self, payload):
        profile, side, sn = self.validate_selection(payload)
        terminal = None
        log = None
        identity_mismatch = False
        try:
            current = self._current(profile)
            calibrated = bool(current.get(side+'_hand', {}).get('calibrated'))
            if calibrated and payload.get('overwrite') is not True:
                raise CalibrationError('Replacing this profile’s saved hand model needs confirmation.')
            self._cancel_checkpoint()
            self.check_robot_idle()
            self.disconnect_inputs()
            self._cancel_checkpoint()
            self._hand_side(sn, side)
            if payload.get('assign') is True:
                config = self.root/'config/wuji_gloves.json'
                assignments = json.loads(config.read_text()) if config.exists() else {}
                if assignments.get('right' if side == 'left' else 'left') == sn:
                    raise CalibrationError('This glove is assigned to the other hand. Resolve the assignment first.')
            self._cancel_checkpoint()
            self.state_root.mkdir(parents=True, exist_ok=True, mode=0o700)
            job = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'-'+uuid.uuid4().hex[:8]
            if calibrated:
                backup = self.state_root/f'before-{side}-{job}.zip'
                self.cli('user', 'export', str(backup))
                self._set(backup=str(backup))
            self._current(profile)
            self._cancel_checkpoint()
            log_path = self.state_root/f'calibration-{job}.jsonl'
            log = log_path.open('x', encoding='utf-8')
            stderr_path = self.state_root/f'calibration-{job}.stderr.log'
            with stderr_path.open('x', encoding='utf-8') as stderr:
                with self.lock:
                    self._cancel_checkpoint()
                    self.process = self.popen(['wuji', 'calib', 'hand-model', '--sn', sn, '--jsonl'],
                                              stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                              stderr=stderr, text=True, bufsize=1, start_new_session=True)
                    process = self.process
                    self.state.update(phase='running', message='Follow the pose instructions.', log_file=str(log_path))
                for line in process.stdout:
                    log.write(line)
                    log.flush()
                    try:
                        event = json.loads(line)
                    except ValueError:
                        continue
                    if (not isinstance(event, dict) or event.get('schema_version') != 2 or
                            event.get('operation') != 'calibrate' or event.get('calibration') != 'hand_model'):
                        continue
                    bound = user_name(event.get('user'))
                    if bound and bound != profile and event.get('event') not in ('result', 'error', 'cancelled'):
                        identity_mismatch = True
                        self.cancel()
                        self._set(message='SDK profile changed during connection; cancelling calibration.')
                    if bound:
                        self._set(bound_profile=bound)
                    if isinstance(event.get('feedback'), dict):
                        self._set(feedback=event['feedback'])
                    if event.get('event') in ('result', 'error', 'cancelled'):
                        terminal = event
                        self._set(phase='finishing', message='Finishing calibration and releasing the glove…')
                code = process.wait()
                process.stdout.close()
            self._set(exit_code=code)
            if identity_mismatch:
                raise CalibrationError('The SDK bound calibration to another profile; the operation was cancelled. Refresh the profile selection.')
            if terminal is None:
                raise CalibrationError('Calibration exited without a terminal result. Check its log before retrying.')
            if terminal.get('event') == 'cancelled' and code == 9:
                self._set(phase='cancelled', message='Calibration cancelled; glove released.')
                return
            if terminal.get('event') != 'result' or terminal.get('status') != 'ok' or code != 0:
                error = terminal.get('error', {})
                self._set(error_kind=error.get('kind'), error_code=error.get('code'))
                raise CalibrationError(str(error.get('message') or f'Calibration failed (exit {code}).'))
            result = terminal.get('result', {})
            actual_user = user_name(result.get('sdk_user')) or user_name(terminal.get('user'))
            actual_sn = terminal.get('device', {}).get('sn')
            if actual_user != profile or result.get('handedness') != side or actual_sn != sn:
                raise CalibrationError('The CLI result belongs to a different profile, hand or glove. Inspect the saved model before reconnecting.')
            self._set(result=result)
            warning = None
            if payload.get('assign') is True:
                try:
                    self._assign(side, sn)
                except (CalibrationError, OSError, ValueError) as error:
                    warning = 'Model saved, but glove assignment needs attention: '+str(error)
            self._set(phase='succeeded', warning=warning,
                      message=f'{side.title()} hand calibrated for {profile}. Reconnect Wuji input to use it.')
        except CalibrationCancelled:
            self._set(phase='cancelled', message='Calibration cancelled before pose collection.', exit_code=9)
        except Exception as error:
            self._set(phase='failed', message=str(error))
        finally:
            process = self.process
            if process is not None and process.poll() is None:
                # Keep ownership and drain the stream until the CLI has released
                # the glove. A failed log write must not orphan calibration.
                with self.lock:
                    if not self.cancel_sent:
                        process.send_signal(signal.SIGINT)
                        self.cancel_sent = True
                process.communicate()
            if process is not None and process.stdout is not None:
                process.stdout.close()
            if log is not None:
                try:
                    log.close()
                except OSError:
                    pass
            with self.lock:
                self.process = None
                self.setup_cache = {}
                self.busy = False

    def cancel(self):
        with self.lock:
            if not self.active() or self.state['phase'] in ('succeeded', 'failed', 'cancelled'):
                return self.status()
            self.cancel_requested = True
            self.state.update(phase='cancelling', message='Cancelling; waiting for the CLI to release the glove…')
            if self.process is not None and not self.cancel_sent and self.process.poll() is None:
                self.process.send_signal(signal.SIGINT)
                self.cancel_sent = True
            return self.status()

    def close(self):
        self.cancel()
        if self.worker is not None:
            self.worker.join(timeout=10)
