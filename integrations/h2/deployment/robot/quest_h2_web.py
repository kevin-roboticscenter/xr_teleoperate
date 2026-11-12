#!/usr/bin/env python3
"""Phone control page for the PR #321 Quest -> H2 + Dex5 teleop (runs on PC2).

Three explicit steps, so a second person (the safety operator) drives the robot state
while the Quest wearer keeps both hands busy:
  Switch to FSM 4 : optional, the validated whole-body path 0 -> 1 -> 4 of prepare_arm_mode.py
                 (only while teleop is not running; confirm tap; no retries).
  Walking mode : explicit SDK request 4/601 -> 703, then enable/confirm Arm SDK control.
  LAUNCH       : spawn upstream (arms snap to q=0, Dex5 hands open). Needs a confirm tap and
                 a fresh FSM 4 or 703 reading and enabled Arm SDK control.
                 FSM 601 does not accept rt/arm_sdk commands on H2.
  START        : CMD_START, only enabled once upstream reports READY.
  STOP         : CMD_STOP (arms go home, process exits); SIGINT if IPC is unreachable.
  REC          : h2_record.py saves both wrist cameras
                 (raw MJPEG) + arm/hand state and cmd at 50 Hz to ~/h2-recordings/<time>/.
  Hand stiffness: saves finger/thumb multipliers for the next LAUNCH, including across restarts.
Standalone right grasp runs in an independent PC2 process and persists across HTTP/Wi-Fi loss.
Nothing moves when this server starts. Logs go to ~/h2-logs (PC2 /tmp is wiped on reboot).
The IPC protocol of upstream teleop/utils/ipc.py is spoken directly with our own zmq
context, because IPC_Client.stop() terminates the shared zmq context.

  ~/anaconda3/envs/tv/bin/python quest_h2_web.py [--host 0.0.0.0] [--port 8787]
"""
import argparse
from contextlib import nullcontext
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import uuid

import zmq

from quest_h2_teleop import (GAIN_SETTINGS_PATH, ROOT, UPSTREAM, command, environment,
                            PICO_HAND_SOURCE, NATIVE_SOURCES, CONTROLLER_SOURCE,
                            DUAL_WUJI_SOURCES, WUJI_SOURCES)
from pico_tracker.input import TrackerInput
from pico_tracker.pose_match import MatchDwell, matching_enabled
from head_camera.head_follow import settings as head_settings
from teleop.utils.dex5_gains import GainProfile, GainStore, MIN_MULTIPLIER, MAX_MULTIPLIER

LOGDIR = Path.home() / 'h2-logs'
RECDIR = Path.home() / 'h2-recordings'          # one folder per REC take (cameras + joints)
RECORDER = Path(__file__).resolve().parent / 'h2_record.py'
HB_ADDR = 'ipc://@xr_teleoperate_hb.ipc'
REQ_ADDR = 'ipc://@xr_teleoperate_data.ipc'
PAGE_PATH = '/h2-teleop'   # '/' redirects here; the browser supplies the Wi-Fi hostname.
# Matches Unitree's example/h2/high_level/h2_arm_sdk_dds_example.py.
TELEOP_FSM_IDS = (4, 703)
HAND_SOURCE_PATH = GAIN_SETTINGS_PATH.parent / 'hand_source.json'
POSE_MATCH_SOURCES = ('pico_wuji', *DUAL_WUJI_SOURCES, PICO_HAND_SOURCE)


class Fsm:
    """Polls H2 state and offers explicit preparation and normal-motion requests."""

    def __init__(self, interface):
        self.lock = threading.Lock()
        self.fsm, self.fsm_time, self.error = None, 0.0, ''
        self.arm_sdk, self.arm_sdk_time, self.arm_sdk_error = None, 0.0, ''
        self.busy, self.msg = False, ''
        try:
            from unitree_sdk2py.core.channel import ChannelFactoryInitialize
            from unitree_sdk2py.h2.loco.h2_loco_client import LocoClient
            ChannelFactoryInitialize(0, interface)
            self.client = LocoClient()
            self.client.SetTimeout(2.0)
            self.client.Init()
        except Exception as exc:
            self.client, self.error = None, f'DDS init failed: {exc}'
        threading.Thread(target=self._poll, daemon=True).start()

    def _read(self):
        with self.lock:
            code, state = self.client.GetFsmId()
            try:
                sdk_code, sdk_enabled = self.client.GetArmSdkStatus()
                if sdk_code == 0 and isinstance(sdk_enabled, bool):
                    self.arm_sdk, self.arm_sdk_time, self.arm_sdk_error = sdk_enabled, time.monotonic(), ''
                else:
                    self.arm_sdk, self.arm_sdk_time = None, 0.0
                    self.arm_sdk_error = f'Arm SDK status unavailable (code {sdk_code})'
            except Exception as exc:
                self.arm_sdk, self.arm_sdk_time = None, 0.0
                self.arm_sdk_error = f'Arm SDK status unavailable: {exc}'
        if code == 0:
            self.fsm, self.fsm_time, self.error = int(state), time.monotonic(), ''
        else:
            self.fsm_time = 0.0
            self.error = f'FSM read failed (code {code})'
        return code == 0

    def _poll(self):
        while self.client is not None:
            if not self.busy:
                try:
                    self._read()
                except Exception as exc:
                    self.fsm_time, self.arm_sdk_time = 0.0, 0.0
                    self.error = f'FSM read error: {exc}'
            time.sleep(1.0)

    def current(self):
        return self.fsm if time.monotonic() - self.fsm_time < 3.0 else None

    def walking_status(self):
        mode = self.current()
        sdk = self.arm_sdk if time.monotonic() - self.arm_sdk_time < 3.0 else None
        ready = mode == 703 and sdk is True and not self.busy
        if self.busy:
            message = self.msg or 'Changing body mode…'
        elif ready:
            message = 'Ready for walking · Unitree remote walks; Quest controls arms and fingers'
        elif mode is None:
            message = 'Walking readiness unavailable · waiting for H2 status'
        elif mode in (0, 1):
            message = 'Not ready for walking · prepare standing first'
        elif mode == 4:
            message = 'Standing mode · press Enable walking with arm teleop'
        elif mode == 703:
            message = ('Walking mode active · enable Arm SDK control' if sdk is False else
                       'Walking mode active · waiting for Arm SDK confirmation')
        else:
            message = f'Walking with Quest arms unavailable in mode {mode}'
        return {'ready': ready, 'message': message, 'arm_sdk_enabled': sdk,
                'arm_sdk_error': self.arm_sdk_error, 'mode_confirmed': mode == 703}

    def to_fixstand(self, teleop_running):
        if self.client is None:
            return False, self.error
        if teleop_running:
            return False, 'Teleop is running; press STOP before changing mode'
        if self.busy:
            return False, 'Mode switch already in progress'
        if not self._read():
            return False, self.error
        state = self.fsm
        if state == 4:
            return False, 'Already in FSM 4'
        if state not in (0, 1):
            return False, f'FSM {state} is not on the validated path (0→1→4); handle it manually'
        path = [1, 4] if state == 0 else [4]
        self.busy, self.msg = True, f'Switching {state}→' + '→'.join(map(str, path))
        threading.Thread(target=self._transition, args=(path,), daemon=True).start()
        return True, self.msg

    def to_motion(self, teleop_running):
        if self.client is None:
            return False, self.error
        if teleop_running:
            return False, 'Press STOP to end arm teleop before changing body mode'
        if self.busy:
            return False, 'Mode switch already in progress'
        if not self._read():
            return False, self.error
        if self.walking_status()['ready']:
            return False, 'Ready for walking already; use the Unitree remote'
        if self.fsm not in (4, 601, 703):
            return False, 'Prepare FSM 4 with support before enabling arm-compatible motion mode'
        self.busy, self.msg = True, f'Enabling walking + Quest arms (mode {self.fsm}→703)'
        threading.Thread(target=self._transition, args=([703],), daemon=True).start()
        return True, self.msg

    def enable_arm_sdk(self):
        with self.lock:
            result = self.client.EnableArmSDK()
        return result == 0, f'EnableArmSDK returned {result}'

    def _transition(self, path):
        try:
            for target in path:
                if self.current() != target:
                    with self.lock:
                        result = self.client.SetFsmId(target)
                    if result != 0:
                        self.msg = f'FSM {target} request failed ({result}); not retried'
                        return
                    self.msg = f'FSM {target} requested, waiting for confirmation…'
                deadline = time.monotonic() + 10
                while time.monotonic() < deadline:
                    if self._read() and self.fsm == target:
                        break
                    time.sleep(0.25)
                else:
                    self.msg = f'FSM {target} not confirmed within 10 s; switch aborted'
                    return
            if path[-1] == 703:
                self.msg = 'Walking mode confirmed; enabling Quest arm control…'
                ok, message = self.enable_arm_sdk()
                if not ok:
                    self.msg = f'Walking setup incomplete: {message}'
                    return
                deadline = time.monotonic() + 5
                while time.monotonic() < deadline:
                    if self._read() and self.fsm == 703 and self.arm_sdk is True:
                        self.msg = 'Ready for walking · use Unitree remote for legs and Quest for arms/fingers'
                        break
                    time.sleep(0.25)
                else:
                    self.msg = 'Walking setup incomplete: mode 703 and Arm SDK were not both confirmed'
            else:
                self.msg = f'FSM {path[-1]} confirmed'
        except Exception as exc:
            self.msg = f'Mode switch error: {exc}'
        finally:
            self.busy = False


class Upstream:
    """Owns at most one upstream process and tracks its IPC heartbeat."""

    def __init__(self, fsm, gain_store=None):
        self.fsm = fsm
        self.ctx = zmq.Context()
        self.lock = threading.Lock()
        self.process = None
        self.log_path = None
        self.hb = {}
        self.hb_time = 0.0
        self.started_at = None      # time CMD_START was accepted
        self.pose_waiting, self.pose_wait_generation = False, 0
        self.pose_wait_progress, self.pose_wait_message = 0., ''
        self.last_exit = None       # return code of the last process we spawned
        self.gain_store = gain_store if gain_store is not None else GainStore(GAIN_SETTINGS_PATH)
        self.active_gains = None
        self.camera = None
        self.grasp = None
        self.auto_recording = None
        self.wuji = None
        self.tracker = TrackerInput(ROOT)
        self.hand_source = 'vr'
        if HAND_SOURCE_PATH.exists():
            value = json.loads(HAND_SOURCE_PATH.read_text())['source']
            if value not in ('vr', 'wuji', *NATIVE_SOURCES): raise ValueError('Invalid saved hand source')
            self.hand_source = value
        threading.Thread(target=self._hb_loop, daemon=True).start()

    def _hb_loop(self):
        sub = self.ctx.socket(zmq.SUB)
        sub.setsockopt(zmq.LINGER, 0)
        sub.setsockopt_string(zmq.SUBSCRIBE, '')
        sub.connect(HB_ADDR)
        while True:
            if sub.poll(200):
                try:
                    self.hb, self.hb_time = sub.recv_json(), time.monotonic()
                except Exception:
                    pass

    def online(self):
        return time.monotonic() - self.hb_time < 0.6

    def _send(self, cmd):
        req = self.ctx.socket(zmq.REQ)
        req.setsockopt(zmq.LINGER, 0)
        req.connect(REQ_ADDR)
        try:
            reqid = str(uuid.uuid4())
            req.send_json({'reqid': reqid, 'cmd': cmd})
            if not req.poll(1500):
                return False, 'No reply from teleop (timeout)'
            reply = req.recv_json()
            return reply.get('status') == 'ok' and reply.get('repid') == reqid, reply.get('msg', str(reply))
        finally:
            req.close()

    @staticmethod
    def external_pids():
        # argv0 must be a python interpreter, so shells/ssh whose command line mentions the file never match
        out = subprocess.run(['pgrep', '-f', r'^[^ ]*python[0-9.]* [^ ]*(teleop_hand_and_arm|browser_teleop|arm_teleop|head_only)\.py'],
                             capture_output=True, text=True).stdout
        return [int(p) for p in out.split()]

    def running(self):
        if self.process is not None and self.process.poll() is not None:
            self.last_exit, self.process = self.process.returncode, None
        return self.process is not None or bool(self.external_pids())

    def body_mode(self, target):
        # Serialize body-mode requests with LAUNCH so initialization cannot race them.
        with self.lock:
            if target == 4:
                return self.fsm.to_fixstand(self.running())
            if target == 703:
                return self.fsm.to_motion(self.running())
            return False, 'Unsupported body-mode request'

    def grasp_start(self):
        with self.lock:
            if getattr(self, 'wuji', None) is not None and self.wuji.status()['active']:
                return False, 'STOP Wuji finger following before starting the standalone grasp'
            grasp = getattr(self, 'grasp', None)
            if grasp is None:
                return False, 'Standalone grasp is unavailable.'
            return grasp.start(self.running)

    def start_wuji_test(self, side):
        grasp = getattr(self, 'grasp', None)
        with self.lock, (grasp.teleop_slot() if grasp else nullcontext()):
            if self.running(): return False, 'STOP arm teleop before the finger-only test'
            if grasp is not None and grasp.status()['active']:
                return False, 'Release the standalone grasp before the finger-only test'
            if getattr(self, 'wuji', None) is None: return False, 'Wuji input unavailable'
            return self.wuji.start_test(side)

    def grasp_release(self):
        with self.lock:
            grasp = getattr(self, 'grasp', None)
            if grasp is None:
                return False, 'Standalone grasp is unavailable.'
            return grasp.release()

    def set_hand_source(self, value):
        if value not in ('vr', 'wuji', *NATIVE_SOURCES): raise ValueError('Unknown arms/hands mode')
        with self.lock:
            glove = getattr(self, 'wuji', None)
            if self.running() or (glove is not None and glove.status()['active']):
                return False, 'STOP before changing the hand source'
            if glove is not None and callable(getattr(glove, 'set_dual', None)):
                glove.set_dual(value in DUAL_WUJI_SOURCES)
            HAND_SOURCE_PATH.parent.mkdir(parents=True, exist_ok=True)
            temporary = HAND_SOURCE_PATH.with_suffix('.tmp')
            temporary.write_text(json.dumps({'source': value}) + '\n')
            temporary.replace(HAND_SOURCE_PATH)
            self.hand_source = value
            if value == PICO_HAND_SOURCE:
                return True, 'Both PICO wrist trackers + PICO optical fingers selected; Wuji gloves are not required'
            if value == 'pico_head':
                return True, 'PICO camera + head-only mode selected; gloves and wrist trackers are not required'
            if value == CONTROLLER_SOURCE:
                return True, 'Both PICO controllers drive arms/wrists; both Wuji gloves drive fingers'
            if value == 'pico_dual_wuji':
                return True, 'Both PICO wrist trackers + both Wuji gloves selected'
            if value == 'pico_wuji':
                return True, 'Native PICO left wrist tracker + Wuji left fingers selected; right arm and fingers hold their starting pose'
            return True, ('PICO arms + VR fingers selected' if value == 'vr' else
                          'PICO arms + Wuji left fingers selected; right fingers keep their last pose')

    def select_tracker(self, serial, side='left'):
        with self.lock:
            if self.running():
                return False, 'STOP before changing the wrist tracker'
            return self.tracker.select(serial, side)

    def set_head_follow(self, enabled):
        if type(enabled) is not bool:
            raise ValueError('Provide enabled: true or false')
        with self.lock:
            if self.running():
                return False, 'STOP before changing head following'
            path = ROOT / 'config/head_follow.json'
            path.parent.mkdir(parents=True, exist_ok=True)
            temporary = path.with_suffix('.tmp')
            temporary.write_text(json.dumps({'enabled': enabled})+'\n')
            temporary.replace(path)
            return True, 'Head following selected for the next START' if enabled else 'Head following off'

    def tracker_status(self):
        tracker = getattr(self, 'tracker', None)
        if tracker is None: return None
        if getattr(self, 'hand_source', 'vr') == CONTROLLER_SOURCE:
            return tracker.status(both=True, controllers=True)
        if getattr(self, 'hand_source', 'vr') == PICO_HAND_SOURCE:
            return tracker.status(both=True, optical_hands=True)
        return tracker.status(head_only=True) if getattr(self, 'hand_source', 'vr') == 'pico_head' else tracker.status(both=True) if self.hand_source == 'pico_dual_wuji' else tracker.status()

    def launch(self):
        grasp = getattr(self, 'grasp', None)
        with self.lock, (grasp.teleop_slot() if grasp else nullcontext()):
            if grasp is not None and grasp.status()['active']:
                return False, 'Release the standalone right-hand grasp before launching Quest teleop (LAUNCH opens both hands).'
            if self.running():
                return False, 'Teleop is already running'
            glove = getattr(self, 'wuji', None)
            if glove is not None and glove.status()['active']:
                return False, 'STOP the existing Wuji test before LAUNCH'
            if glove is not None and callable(getattr(glove, 'conflict', None)):
                conflict = glove.conflict()
                if conflict: return False, conflict
            if getattr(self, 'hand_source', 'vr') in WUJI_SOURCES and (
                    glove is None or not glove.status()['connected']):
                return False, 'Connect both Wuji gloves before LAUNCH' if self.hand_source in DUAL_WUJI_SOURCES else 'Connect the left Wuji glove before LAUNCH'
            if getattr(self, 'hand_source', 'vr') in NATIVE_SOURCES and not self.tracker_status()['ready']:
                return False, self.tracker_status()['message']
            if self.fsm.busy or self.fsm.current() not in TELEOP_FSM_IDS:
                return False, f'FSM {self.fsm.current()} does not accept arm teleop: use supported FSM 4 or arm-compatible motion mode 703'
            try:
                profile = self.gain_store.load()
            except (ValueError, OSError) as exc:
                return False, f'Hand stiffness settings could not be loaded: {exc}'
            ok, message = self.fsm.enable_arm_sdk()
            if not ok:
                return False, message
            LOGDIR.mkdir(exist_ok=True)
            self.log_path = LOGDIR / f'web_dex5_{time.strftime("%Y%m%dT%H%M%S")}.log'
            latest = LOGDIR / 'latest.log'
            if latest.is_symlink() or latest.exists():
                latest.unlink()
            latest.symlink_to(self.log_path)
            camera = getattr(self, 'camera', None)
            with (camera.launch_slot() if camera is not None else nullcontext()), self.log_path.open('a') as log:
                self.process = subprocess.Popen(command(getattr(self, 'hand_source', 'vr')), cwd=UPSTREAM / 'teleop', env=environment(profile),
                                                stdout=log, stderr=subprocess.STDOUT,
                                                stdin=subprocess.DEVNULL, start_new_session=True)
            self.active_gains = profile
            self.pose_waiting = False
            self.pose_wait_generation = getattr(self, 'pose_wait_generation', 0)+1
            self.pose_wait_progress, self.pose_wait_message = 0., ''
            self.started_at, self.last_exit, self.hb, self.hb_time = None, None, {}, 0.0
            hands = 'hands opening' if getattr(self, 'hand_source', 'vr') in ('vr', PICO_HAND_SOURCE) else 'Wuji fingers wait for START'
            if getattr(self, 'hand_source', 'vr') == 'pico_head':
                return True, 'Head-only test prepared; START holds the current arm pose and follows the headset. Fingers stay idle.'
            return True, f'Launched (pid {self.process.pid}); arms moving to zero, {hands}'

    def save_hand_gains(self, values):
        profile = GainProfile.from_dict(values)
        with self.lock:
            self.gain_store.save(profile)
        return True, 'Hand stiffness saved for the next LAUNCH or Grasp & hold. Active settings stay unchanged.'

    def hand_gain_status(self, running):
        active = self.active_gains if running else None
        try:
            saved = self.gain_store.load()
            return {'saved': saved.describe(), 'active': active.describe() if active else None,
                    'pending': running and active is not None and saved != active,
                    'external': running and active is None,
                    'min': MIN_MULTIPLIER, 'max': MAX_MULTIPLIER, 'error': ''}
        except (ValueError, OSError) as exc:
            return {'saved': None, 'active': active.describe() if active else None,
                    'pending': False, 'external': running and active is None,
                    'min': MIN_MULTIPLIER, 'max': MAX_MULTIPLIER, 'error': str(exc)}

    def recording_snapshot(self):
        with self.lock:
            running = self.running()
            pids = ([self.process.pid] if self.process is not None else self.external_pids()) if running else []
            return {'running': bool(pids), 'session': tuple(sorted(pids)) if pids else None,
                    'started': bool(self.online() and self.hb.get('START'))}

    def start(self):
        with self.lock:
            hb = self.hb if self.online() else {}
            if not hb.get('READY'):
                return False, 'Not READY yet'
            if hb.get('START'):
                return False, 'Already following'
            if getattr(self, 'pose_waiting', False):
                return False, 'Already waiting for the matching pose; STOP cancels'
            if (getattr(self, 'hand_source', 'vr') in ('vr', 'wuji') and head_settings(ROOT).get('enabled') and
                    not hb.get('HEAD_READY')):
                return False, 'Enter the headset browser XR view and enable live head tracking before START'
            if getattr(self, 'hand_source', 'vr') in NATIVE_SOURCES and not self.tracker_status()['ready']:
                return False, self.tracker_status()['message']
            if getattr(self, 'hand_source', 'vr') in POSE_MATCH_SOURCES and matching_enabled(ROOT):
                assessment = hb.get('POSE_MATCH', {})
                if not assessment.get('calibrated'):
                    return False, assessment.get('message', 'Match the prepared robot and Save matching pose first')
                if not assessment.get('robot_matches'):
                    return False, assessment.get('message', 'Robot ready pose does not match the saved reference')
                self.pose_wait_generation = getattr(self, 'pose_wait_generation', 0)+1
                self.pose_waiting, self.pose_wait_progress = True, 0.
                self.pose_wait_message = 'Waiting for pose match; match both wrists and look forward'
                threading.Thread(target=self._wait_for_pose, args=(self.pose_wait_generation,), daemon=True).start()
                return True, self.pose_wait_message
            return self._begin_following()

    def _begin_following(self):
        """Called with the controller lock held, after the match when required."""
        glove = getattr(self, 'wuji', None)
        uses_glove = getattr(self, 'hand_source', 'vr') in WUJI_SOURCES
        if uses_glove:
            if glove is None: return False, 'Wuji input unavailable'
            ok, msg = glove.start()
            if not ok: return False, msg
        ok, msg = self._send('CMD_START')
        if not ok and uses_glove and glove is not None:
            glove.stop()
        if ok:
            self.started_at = time.monotonic()
            if getattr(self, 'auto_recording', None):
                self.auto_recording.notify()
        return ok, 'Following started' if ok else msg

    def save_matching_pose(self):
        with self.lock:
            hb = self.hb if self.online() else {}
            if self.hand_source not in POSE_MATCH_SOURCES:
                return False, 'Select a native PICO wrist tracker mode'
            if (not hb.get('READY') or hb.get('START') or hb.get('STOP') or
                    getattr(self, 'pose_waiting', False)):
                return False, 'Prepare arms, then save the matching pose before pressing Start'
            tracker = self.tracker_status()
            if not tracker['ready']:
                return False, tracker['message']
            ok, message = self._send('CMD_POSE_CALIBRATE')
            if ok:
                self.pose_wait_message = ''
            return ok, 'Matching pose saved for this operator and tracker placement' if ok else message

    def _match_tick(self, dwell, now):
        """One match observation, under lock; kept separate for offline checks."""
        hb = self.hb if self.online() else {}
        assessment = hb.get('POSE_MATCH', {})
        tracker = self.tracker_status()
        mode_ready = not self.fsm.busy and self.fsm.current() in TELEOP_FSM_IDS
        ready = bool(hb.get('READY') and not hb.get('START') and not hb.get('STOP') and
                     tracker['ready'] and mode_ready)
        progress = dwell.update(assessment, ready, now)
        self.pose_wait_progress = progress['progress']
        self.pose_wait_message = (tracker['message'] if not tracker['ready'] else
                                  'Waiting for a supported robot mode' if not mode_ready else
                                  'Waiting for fresh teleop state' if not hb else
                                  assessment.get('message', 'Waiting for matching pose feedback'))
        if not progress['complete']:
            return None
        # The native arm process rechecks the current measured wrists and input
        # before accepting CMD_START. Fingers are activated only after this wait.
        ok, message = self._begin_following()
        self.pose_waiting = False
        self.pose_wait_message = message if ok else 'Could not start: '+message
        return ok, message

    def _wait_for_pose(self, generation):
        dwell = MatchDwell()
        try:
            while True:
                with self.lock:
                    if not self.pose_waiting or generation != self.pose_wait_generation:
                        return
                    if not matching_enabled(ROOT):
                        self.pose_waiting = False
                        self.pose_wait_message = 'Pose matching off; press Start following'
                        return
                    if not self.running() or self.hb.get('STOP'):
                        self.pose_waiting = False
                        self.pose_wait_message = 'Pose check cancelled; prepare arms again'
                        return
                    result = self._match_tick(dwell, time.monotonic())
                    if result is not None:
                        print(time.strftime('%H:%M:%S'), 'pose-match/start', *result, flush=True)
                        return
                time.sleep(.05)
        except Exception as exc:
            with self.lock:
                if generation == self.pose_wait_generation:
                    self.pose_waiting = False
                    self.pose_wait_message = f'Pose check stopped: {exc}'
            print(time.strftime('%H:%M:%S'), 'pose-match/error', str(exc), flush=True)

    def pose_match_status(self, hb):
        enabled = getattr(self, 'hand_source', 'vr') in POSE_MATCH_SOURCES and matching_enabled(ROOT)
        value = dict(hb.get('POSE_MATCH', {})) if enabled else {}
        waiting = bool(getattr(self, 'pose_waiting', False))
        value.update(enabled=enabled, waiting=waiting,
                     progress=getattr(self, 'pose_wait_progress', 0.) if waiting else 0.,
                     can_calibrate=bool(enabled and hb.get('READY') and not hb.get('START') and
                                        not hb.get('STOP') and not waiting))
        if waiting or not value.get('message') or (not hb.get('START') and getattr(self, 'pose_wait_message', '')):
            value['message'] = (getattr(self, 'pose_wait_message', '') or
                                'Prepare arms, match the robot, then Save matching pose')
        return value

    def stop(self):
        with self.lock:
            self.pose_waiting = False
            self.pose_wait_generation = getattr(self, 'pose_wait_generation', 0)+1
            self.pose_wait_progress, self.pose_wait_message = 0., ''
            glove = getattr(self, 'wuji', None)
            was_active = glove is not None and glove.status()['active']
            glove_ok, glove_message = glove.stop() if glove is not None else (True, '')
            if self.online():
                ok, msg = self._send('CMD_STOP')
                if ok:
                    return glove_ok, ('STOP sent; head tracking is ending. ' if getattr(self, 'hand_source', 'vr') == 'pico_head' else
                                      'STOP sent; arms returning to zero. ') + glove_message
            pids = ([self.process.pid] if self.process is not None and self.process.poll() is None
                    else self.external_pids())
            if not pids:
                if was_active: return glove_ok, glove_message
                return False, 'Teleop is not running'
            for pid in pids:
                os.kill(pid, signal.SIGINT)
            return glove_ok, ('No heartbeat; sent SIGINT to end head tracking. ' if getattr(self, 'hand_source', 'vr') == 'pico_head' else
                              'No heartbeat; sent SIGINT (teleop returns arms to zero, then exits). ') + glove_message

    def status(self):
        running = self.running()
        hb = self.hb if self.online() else {}
        if not running:
            state, phase = ('Stopped' if self.last_exit in (None, 0) else f'Teleop exited with code {self.last_exit}; check the log'), 'idle'
        elif not hb and self.hb_time > 0:
            state, phase = '⚠ Heartbeat lost (may still be following): press STOP', 'lost'
        elif hb.get('STOP'):
            state, phase = 'Stopping, arms returning to zero…', 'stopping'
        elif hb.get('START'):
            secs = int(time.monotonic() - self.started_at) if self.started_at else 0
            state, phase = f'Following {secs // 60:02d}:{secs % 60:02d}', 'following'
        elif hb.get('READY'):
            state, phase = 'READY: press START when the operator is in position', 'ready'
        else:
            state, phase = ('Initializing… (arms to zero; Wuji waits for START)' if
                            getattr(self, 'hand_source', 'vr') in WUJI_SOURCES else
                            'Initializing… (arms to zero, hands opening)'), 'init'
        log = ''
        if self.log_path and self.log_path.exists():
            log = '\n'.join(self.log_path.read_text(errors='replace').splitlines()[-40:])
        fsm = self.fsm.current()
        glove = getattr(self, 'wuji', None)
        glove_status = glove.status() if glove is not None else None
        if not running and glove_status and glove_status.get('active'):
            state, phase = f'Wuji {glove_status.get("test_side") or "hand"} finger test following · STOP ends the test', 'hand_test'
        feedback = glove_status.get('hand_feedback') if glove_status is not None else None
        tracker = self.tracker_status()
        pose_match = self.pose_match_status(hb)
        if phase == 'ready' and getattr(self, 'hand_source', 'vr') in NATIVE_SOURCES and tracker and not tracker['ready']:
            state, phase = tracker['message'], 'tracker_wait'
        if phase == 'following' and hb.get('INPUT') in ('pico_tracker', 'pico_controllers') and not hb.get('TRACKER_LIVE'):
            state = hb.get('INPUT_MESSAGE', 'Tracker input unavailable; arms holding last target')
        if phase in ('ready', 'tracker_wait') and getattr(self, 'hand_source', 'vr') in WUJI_SOURCES and feedback and not feedback['ready']:
            state, phase = feedback['message'] + ' · arms initialized; resolve hand feedback before START', 'hand_fault'
        if phase in ('ready', 'tracker_wait', 'hand_fault') and pose_match['waiting']:
            state, phase = 'Waiting for pose match · '+pose_match['message'], 'matching'
        elif phase == 'ready' and pose_match['enabled']:
            state = ('Arms prepared · save matching pose' if not pose_match.get('calibrated') else
                     'Arms prepared · '+pose_match['message'] if not pose_match.get('robot_matches') else
                     'Arms prepared · press Start pose check')
        if (phase == 'ready' and getattr(self, 'hand_source', 'vr') in ('vr', 'wuji') and
                head_settings(ROOT).get('enabled') and not hb.get('HEAD_READY')):
            state, phase = 'Enter the headset browser XR view for live head tracking', 'head_wait'
        if getattr(self, 'hand_source', 'vr') == 'pico_head' and phase == 'init':
            state = 'Preparing head tracking; arms and fingers are not following'
        if getattr(self, 'hand_source', 'vr') == 'pico_head' and phase == 'stopping':
            state = 'Stopping head tracking…'
        if getattr(self, 'hand_source', 'vr') == 'pico_head' and phase == 'following' and not hb.get('HEAD_LIVE'):
            state = 'Head input unavailable; holding neck target'
        return {'state': state, 'phase': phase, 'running': running, 'heartbeat': hb, 'pose_match': pose_match,
                'hand_source': getattr(self, 'hand_source', 'vr'),
                'wuji': glove_status,
                'pico_tracker': tracker,
                'head_follow': dict(head_settings(ROOT), enabled=getattr(self, 'hand_source', 'vr') == 'pico_head' or bool(head_settings(ROOT).get('enabled')),
                                    active=bool(hb.get('HEAD_FOLLOW') and hb.get('START')),
                                    supported=True, source='browser' if getattr(self, 'hand_source', 'vr') in ('vr', 'wuji') else 'native_pico',
                                    live=bool(hb.get('HEAD_LIVE')), target=hb.get('HEAD_TARGET')),
                'fsm': fsm, 'fsm_busy': self.fsm.busy, 'fsm_msg': self.fsm.msg, 'fsm_error': self.fsm.error,
                'walking': self.fsm.walking_status(),
                'log_file': str(self.log_path or ''), 'log': log,
                'hand_gains': self.hand_gain_status(running),
                'right_grasp': self.grasp.status() if getattr(self, 'grasp', None) else None,
                'camera': self.camera.status() if getattr(self, 'camera', None) else None}

    def close(self):
        if getattr(self, 'wuji', None): self.wuji.stop()
        if self.running():
            self.stop()


class Recorder:
    """REC button: runs h2_record.py (read-only: wrist cameras + joint data) as its own process,
    so a stuck camera can never block the robot controls. Supports automatic session recording."""

    def __init__(self, interface):
        self.interface = interface
        self.lock = threading.Lock()
        self.process = None
        self.dir = None
        self.takes = 0

    def running(self):
        return self.process is not None and self.process.poll() is None

    def start(self):
        with self.lock:
            if self.running():
                return False, 'Already recording'
            RECDIR.mkdir(exist_ok=True)
            LOGDIR.mkdir(exist_ok=True)
            self.takes += 1
            self.dir = RECDIR / (time.strftime('%Y%m%dT%H%M%S') + '_' + uuid.uuid4().hex[:8])
            with (LOGDIR / 'recorder.log').open('a') as log:
                self.process = subprocess.Popen(
                    [sys.executable, str(RECORDER), '--out', str(self.dir), '--network-interface', self.interface,
                     '--camera-streams', '--head-camera'], env=environment(),
                    stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, start_new_session=True)
            return True, f'Recording take {self.takes} → {self.dir.name}'

    def stop(self):
        with self.lock:
            if not self.running():
                return False, 'Not recording'
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                self.process.kill()
            return True, f'Saved {self.dir.name}'

    def status(self):
        info = {'recording': self.running(), 'dir': self.dir.name if self.dir else '', 'takes': self.takes,
                'head_video_supported': True, 'head_video_file': 'head.mp4'}
        try:
            info.update(json.loads((self.dir / 'status.json').read_text()))
            info['dir'] = self.dir.name
        except Exception:
            pass
        return info

    def close(self):
        if self.running():
            self.stop()


PAGE ='''<!doctype html><html lang="en"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>H2 Teleop Control</title><style>
:root{--bg:#f1f4f6;--fg:#182b34;--muted:#667782;--card:#fff;--line:#dce4e8;--accent:#117661}
*{box-sizing:border-box}[hidden]{display:none!important}body{margin:0;background:var(--bg);color:var(--fg);font:14px/1.5 system-ui,-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif}button,input,select{font:inherit}button{cursor:pointer;touch-action:manipulation}button:disabled{cursor:default;opacity:.4}a{color:#126caa}small{display:block;color:var(--muted);font-size:12px;line-height:1.55}h1,h2{margin:0}h1{font-size:27px;font-weight:650;letter-spacing:-.8px}h2{font-size:17px;font-weight:650}label{font-weight:550}.app-shell{max-width:1380px;margin:0 auto;padding:28px 32px}.topbar{display:flex;justify-content:space-between;align-items:center;margin-bottom:24px}.eyebrow{font-size:10px;letter-spacing:1.6px;font-weight:700;color:var(--muted)}.top-meta{color:var(--muted);font-size:12px;background:#e6ecef;border:1px solid var(--line);padding:6px 12px;border-radius:20px}.control-bar{position:sticky;top:12px;z-index:10;background:#fff;border:1px solid var(--line);border-radius:16px;padding:18px 22px;margin-bottom:22px;box-shadow:0 5px 18px #20354209}.status-row{display:flex;align-items:baseline;gap:20px;justify-content:space-between;margin-bottom:14px}#state{font-size:20px;font-weight:650;letter-spacing:-.3px}#state.following{color:#087c50}#state.ready{color:#176fac}#state.init,#state.stopping,#state.matching{color:#9e620b}#state.lost{color:#c73030}#fsm{font-size:11px;max-width:60%;color:var(--muted);text-align:right}#fsm.bad{color:#b14e38}#fsm.ok{color:#087c50}.primary-actions{display:grid;grid-template-columns:1fr 1fr 1fr;gap:10px}button{border:1px solid var(--line);border-radius:9px;padding:10px 14px;background:#edf2f5;color:var(--fg);font-weight:600;transition:filter .15s}button:hover:not(:disabled){filter:brightness(.96)}.primary-actions button{font-size:15px;min-height:49px}.step{font-size:10px;display:inline-flex;justify-content:center;align-items:center;width:19px;height:19px;border:1px solid currentColor;border-radius:50%;margin-right:7px;opacity:.7}#launch{background:#edf2f5}#start{background:#117661;border-color:#117661;color:white}#stop{background:#c7433f;border-color:#c7433f;color:white}.control-note{font-size:11px;margin-top:9px}#msg:empty{display:none}#msg{font-size:12px;color:#906015;margin-top:10px}.workspace{display:grid;grid-template-columns:minmax(0,1.8fr) minmax(300px,1fr);gap:20px;align-items:start}.panel{padding:21px;background:var(--card);border:1px solid var(--line);border-radius:14px;margin-bottom:18px;min-width:0}.panel-heading{display:flex;gap:12px;align-items:center;justify-content:space-between;margin-bottom:12px}#head-camera-state,#camera-state,.detail-status,#head-video-state{font-size:12px;color:var(--muted);margin-bottom:10px}#head-camera-image{width:100%;height:auto;aspect-ratio:16/9;object-fit:contain;border-radius:9px;background:#e5ebee;margin:0 0 15px}.head-toolbar{display:flex;align-items:center;justify-content:space-between;gap:12px;margin-bottom:10px}.switch-label{display:flex;align-items:center;gap:8px;font-size:13px}input[type=checkbox]{accent-color:var(--accent);width:16px;height:16px}#head-recenter{font-size:12px;padding:7px 11px}details>summary{cursor:pointer;font-size:14px;font-weight:600;list-style-position:inside}details>summary span{float:right;font-size:11px;font-weight:400;color:var(--muted)}details[open]>summary{margin-bottom:17px}.help{margin-top:10px}.help>summary{font-size:12px;color:var(--muted);font-weight:500}.input-setup{font-size:12px}.input-setup select{display:block;width:100%;border:1px solid var(--line);border-radius:8px;background:#f8fafb;color:var(--fg);font-size:12px!important;padding:10px!important;margin:8px 0 14px!important}.input-setup button{width:100%;font-size:12px!important;padding:10px!important;margin:10px 0!important;background:#edf4f7!important;color:#244958!important;border:1px solid #d8e5eb}#wuji-state,#right-glove-state,#tracker-state{font-size:12px;margin:8px 0 12px;overflow-wrap:anywhere}.input-setup small{font-size:11px}.settings-grid{display:grid;grid-template-columns:1fr 1fr;gap:0 18px;align-items:start}.secondary-actions{display:flex;gap:10px;flex-wrap:wrap}.secondary-actions button{font-size:12px;margin:12px 0}.waiting{color:#956016}.ready{color:#087c50}#hand-gains h2,#right-grasp h2{display:none}#hand-gains label{display:block;font-size:13px;margin-top:15px}#hand-gains input[type=range]{width:100%;height:26px;accent-color:var(--accent);margin:8px 0}#hand-gains output{float:right;font-variant-numeric:tabular-nums}#hand-gains button,#right-grasp button{font-size:12px;margin:10px 8px 0 0}#gain-state,#gain-message,#grasp-state,#grasp-settings,#recinfo,#auto-rec-state{font-size:12px;margin-top:10px;color:var(--muted)}#grasp-hold{background:#117661;color:white}#grasp-release{background:#a85b26;color:white}#rec{margin:10px 0;font-size:13px}#rec.on{background:#fff0ef;border-color:#d24640;color:#b6312b}#camera-image{width:100%;max-width:840px;display:block;margin:10px auto;border-radius:8px}pre{font:11px/1.6 ui-monospace,monospace;white-space:pre-wrap;overflow-wrap:anywhere;background:#f4f7f9;border-radius:8px;padding:14px;max-height:300px;overflow:auto}footer{font-size:11px;color:var(--muted);padding:4px 0 12px}#source-help{margin-top:14px}
@media(max-width:900px){.app-shell{padding:20px}.workspace{grid-template-columns:1fr}.input-setup{margin-top:0}.control-bar{top:8px}.status-row{display:block}#fsm{max-width:100%;text-align:left;margin-top:4px}.settings-grid{gap:0 14px}.pov-panel{margin-bottom:0}}
@media(max-width:560px){.app-shell{padding:14px 10px}.topbar{margin:0 5px 18px}h1{font-size:24px}.top-meta{display:none}.control-bar{padding:14px;border-radius:12px}.primary-actions{gap:6px}.primary-actions button{font-size:12px;padding:10px 6px;min-height:47px}.step{display:none}#state{font-size:18px}.panel{padding:16px}.settings-grid{grid-template-columns:1fr}.panel-heading{display:block}.panel-heading .eyebrow{display:block;margin-top:4px}.head-toolbar{align-items:center}.control-note{font-size:10px}}
</style>
<div class="app-shell">
<header class="topbar"><div><div class="eyebrow">UNITREE H2</div><h1>Teleoperation</h1></div><span class="top-meta">PICO · Quest · Wuji</span></header>
<section class="control-bar" aria-label="Teleoperation controls"><div class="status-row"><div id="state" role="status" aria-live="polite">Connecting…</div><div id="fsm">Reading robot mode…</div></div><div class="primary-actions"><button id="launch" disabled onclick="launchConfirm()"><span class="step">1</span> Prepare arms</button><button id="start" disabled onclick="send('start')"><span class="step">2</span> Start following</button><button id="stop" disabled onclick="send('stop')">Stop following</button></div><div id="msg" role="status" aria-live="polite"></div><small class="control-note">Start captures your neutral pose. Stop ends arm, finger and head following; use the remote for walking.</small></section>
<section class="panel" id="pose-match" hidden aria-label="Start pose matching"><div class="panel-heading"><h2>Start pose</h2><button id="pose-save" onclick="saveMatchingPose()" disabled>Save matching pose</button></div><div id="pose-state" role="status" aria-live="polite"></div><div id="pose-left"></div><div id="pose-right"></div><progress id="pose-progress" max="1" value="0" hidden style="width:100%;margin-top:10px" aria-label="Continuous pose match"></progress><small>First use: Prepare arms, physically match H2’s wrist positions and angles, look forward, then Save matching pose. Start pose check waits for a steady match before following. Save again for a different wearer or tracker mounting. This checks wrists; elbow placement is inferred.</small></section>
<main><div class="workspace"><section class="panel pov-panel"><div class="panel-heading"><h2>Robot POV</h2><span class="eyebrow">HEAD CAMERA · LEFT EYE</span></div><div id="head-camera-state">Waiting for head camera…</div><img id="head-camera-image" alt="H2 head camera left eye"  hidden><div class="head-toolbar"><label class="switch-label"><input id="head-follow" type="checkbox" onchange="setHeadFollow()"> Follow headset</label><button id="head-recenter" onclick="send('head-recenter')" disabled>Recenter</button></div><div id="head-follow-state" class="detail-status"></div><details class="help"><summary>Set up the headset view</summary><small>Select Follow headset before preparing. Browser modes use the headset browser XR view; native PICO modes use Head + Send in XRoboToolkit. Look forward at START; yaw and pitch follow your headset. Recenter keeps the current robot neck pose. STOP ends following. In PICO XRoboToolkit: Remote Vision → ZEDMINI → Listen → this laptop's IP. This uses the H2 left-eye camera for both eyes. Recording includes the head camera as head.mp4.</small></details><div id="head-video-state"></div></section><section class="panel input-setup"><div class="panel-heading"><h2>Inputs</h2><span class="eyebrow">CONNECT & SELECT</span></div>
<label for="hand-source"><strong>Control mode</strong></label>
<select id="hand-source" onchange="setHandSource()" style="display:block;width:100%;font-size:20px;padding:12px;margin:12px 0" disabled>
<option value="pico_head">PICO head only · camera + neck</option>
<option value="vr">PICO / Quest browser · arms + hands</option>
<option value="wuji">Browser arms + Wuji left hand</option>
<option value="pico_wuji">PICO left wrist tracker + Wuji left hand</option>
<option value="pico_dual_wuji">PICO wrist trackers + both Wuji gloves</option>
<option value="pico_controller_wuji">PICO controllers + both Wuji gloves</option>
<option value="pico_tracker_hands">PICO wrist trackers + PICO fingers</option>
</select>
<button id="wuji-connect" onclick="send('wuji/connect')" style="font-size:20px;padding:14px;background:#1e40af" disabled>Connect Wuji glove</button>
<div id="wuji-state" role="status">Reading glove status…</div>
<p><a href="/wuji-calibration" target="_blank" rel="noopener">Calibrate Wuji gloves / select operator profile</a></p>
<details class="help" id="glove-tests"><summary>Individual hand test</summary><div id="right-glove-test" hidden>
<button id="right-glove-connect" onclick="send('wuji/right/connect')" style="font-size:18px;padding:12px;background:#1e40af">Connect right glove</button>
<button id="right-glove-start" onclick="send('wuji/test-right')" style="font-size:18px;padding:12px;background:#15803d">START right-hand test</button>
<div id="right-glove-state" role="status"></div>
<small>Right fingers only. The left glove may stay unplugged for this test. STOP ends following; the fingers keep their last pose.</small>
</div></details>
<div id="tracker-controls" hidden>
<button id="tracker-connect" onclick="send('pico/connect')" style="font-size:20px;padding:14px;background:#1e40af">Connect PICO tracker input</button>
<details class="help" id="tracker-assignments"><summary>Wrist tracker assignments</summary><label for="tracker-sn">Left wrist tracker serial</label>
<select id="tracker-sn" onchange="selectTracker()" style="width:100%;font-size:18px;padding:10px"></select>
<div id="right-tracker-select" hidden>
<label for="right-tracker-sn">Right wrist tracker serial</label>
<select id="right-tracker-sn" onchange="selectTracker('right')" style="width:100%;font-size:18px;padding:10px"></select>
</div>
</details><div id="tracker-state" role="status"></div>
<details class="help" id="tracker-setup-help"><summary>Headset & input setup</summary><small>XRoboToolkit PC Service IP: <span id="native-service-ip"></span>. <span id="tracker-mode-instructions">Enable Head, Object Tracking, and Send. Mount each selected tracker rigidly on the back of its hand. At START, look forward and match your arm pose to the robot. That pose becomes the reference. Two-glove mode follows both arms and wrists. Left-only mode holds the right arm.</span></small></details>
</div>
<small id="source-help">Wuji currently uses the calibrated left glove; right fingers keep their last pose. STOP ends arm and Wuji following.</small>
</section></div>
<details class="panel wrist-panel"><summary>Wrist cameras<span>Live preview</span></summary><strong>H2 hand cameras</strong><div id="camera-state">Starting cameras…</div><img id="camera-image" alt="Both H2 hand-camera views" hidden><small>A/B identify the two USB feeds; left/right labels await physical confirmation. The current headset video setting appears above; desktop preview and recording remain available. Reload the headset page after LAUNCH or STOP if it disconnects.</small></details>
<div class="settings-grid"><details class="panel"><summary>Robot mode<span>Stand / walk</span></summary><div id="walking" class="waiting" role="status">Checking walking readiness…</div><div class="secondary-actions"><button id="fsm4" disabled onclick="fsmConfirm()">Prepare standing</button><button id="motion" disabled onclick="motionConfirm()">Enable walking with arm teleop</button></div><small>Walking mode and Arm SDK status do not verify physical balance or the remote link. Use the Unitree remote for body control.</small></details>
<details class="panel"><summary>Hand stiffness<span>Fingers & thumb</span></summary><section id="hand-gains">
<h2 id="gain-heading">Hand stiffness</h2>
<small>Both hands · 1.00× = original settings · Range 0.50×–2.50× in 0.05× steps. Higher gains are unvalidated tuning settings. Saved settings apply at the next LAUNCH or Grasp &amp; hold. Grip force is not measured.</small>
<label for="finger-gain">Four-finger stiffness <output id="finger-value" for="finger-gain">1.20×</output></label>
<input id="finger-gain" type="range" min="0.5" max="2.5" step="0.05" value="1.2" oninput="editHandGains()" disabled>
<label for="thumb-gain">Thumb stiffness <output id="thumb-value" for="thumb-gain">1.20×</output></label>
<input id="thumb-gain" type="range" min="0.5" max="2.5" step="0.05" value="1.2" oninput="editHandGains()" disabled>
<button id="gain-save" onclick="saveHandGains()" disabled>Save for next launch</button>
<button id="gain-reset" onclick="saveHandGains(true)" disabled>Reset to original settings (1.00×)</button>
<div id="gain-state" aria-live="polite">Loading hand settings…</div>
<div id="gain-message" aria-live="polite"></div>
</section></details>
<details class="panel"><summary>Standalone grasp<span>Right hand</span></summary><section id="right-grasp">
<h2 id="grasp-heading">Right hand · standalone bag hold</h2>
<small>Full-hand position grasp, including thumb. Quest is not needed. The hold runs on H2 and continues when Wi-Fi or this page disconnects, while H2 remains powered. Reconnect to release. The arms and body remain under their existing control.</small>
<div id="grasp-state" role="status" aria-live="polite">Reading hold status…</div>
<div id="grasp-settings"></div>
<button id="grasp-hold" disabled onclick="send('grasp-hold')">Grasp &amp; hold right hand</button>
<button id="grasp-release" disabled onclick="send('grasp-release')">Release right hand</button>
<small>First use: test the grasp with the bag supported before carrying it. This preset has not been physically validated; holding force and bag retention are not measured. Dashboard STOP ends Quest arm teleop; use Release above to open a standalone grasp.</small>
</section></details>
<details class="panel"><summary>Recording<span>Head & wrist video</span></summary><label style="display:block;margin:18px 0 8px"><input id="auto-rec" type="checkbox" disabled onchange="setAutoRecording()"> Automatically record head + wrist video when following starts</label>
<div id="auto-rec-state" style="font-size:14px;margin-bottom:12px" aria-live="polite">Reading automatic recording settings…</div>
<button id="rec" disabled onclick="toggleRec()">● REC head + wrist video</button>
<div id="recinfo"></div>
</details></div>
<details class="panel help"><summary>Operating notes & browser link</summary><small>For standing with arm teleop: prepare FSM 4 with support, plant both feet, then enable arm-compatible motion mode (703). FSM 601 does not accept arm teleop. Keep the gantry attached until standing is stable. The displayed mode does not verify physical balance.<br>
Flow: LAUNCH → operator puts on the Quest, hands down → START → after the take, operator holds hands down → STOP → then take off the headset.<br>
STOP ends arm teleop and leaves the body mode selected; the hands keep their last pose. Use the Unitree remote for body control.<br>Quest URL: <a id="quest-link" target="_blank" rel="noopener">Open Vuer</a></small>
</details>
<details class="panel"><summary>Session log</summary><small id="logfile"></small><pre id="log"></pre></details>
</main><footer>H2 control · Robot POV and wrist feeds are live views. Recording includes the head camera as head.mp4.</footer></div>
<script>
const questHost=window.location.hostname;
document.querySelector('#native-service-ip').textContent=questHost;
const questLink=document.querySelector('#quest-link');
questLink.href=`https://${questHost}:8012/?ws=wss://${questHost}:8012`;
questLink.textContent=questLink.href;
let busy=false,phase='',fails=0,graspActive=false;
const $=s=>document.querySelector(s);
async function refresh(){
  try{const r=await fetch('/api/status',{cache:'no-store'});if(!r.ok)throw Error('Status unavailable');const s=await r.json();fails=0;phase=s.phase;
    $('#state').textContent=s.state;$('#state').className=s.phase;$('#log').textContent=s.log;$('#logfile').textContent=s.log_file;
    const f=s.fsm,modeReady=f===4||f===703;$('#fsm').className=modeReady?'ok':'bad';
    $('#fsm').textContent=s.fsm_busy?('FSM '+f+' · '+s.fsm_msg):f===4?'FSM 4 · fixed stand (use support)':f===703?'FSM 703 · arm-compatible motion mode; confirm stable standing':f===601?'FSM 601 · arms unavailable; select arm-compatible mode 703':f===null?('FSM unavailable ⚠ '+s.fsm_error):('FSM '+f+' · select supported stand or arm-compatible motion mode'+(s.running?' (teleop running! press STOP)':s.fsm_msg?' (last: '+s.fsm_msg+')':''));
    $('#fsm4').disabled=busy||s.running||s.fsm_busy||!(f===0||f===1);
    const walking=s.walking||{ready:false,message:'Walking status unavailable'};
    $('#walking').textContent=walking.message;$('#walking').className=walking.ready?'ready':'waiting';
    $('#motion').disabled=busy||s.running||s.fsm_busy||walking.ready||!(f===4||f===601||f===703);
    $('#motion').textContent=walking.ready?'Walking with arm teleop enabled':'Enable walking with arm teleop';
    renderRec(s.rec||{});
    renderHandGains(s.hand_gains);
    renderGrasp(s.right_grasp,s.running||!!(s.wuji&&s.wuji.active),s.hand_gains);
    renderCamera(s.camera);
    const ho=s.hand_source==='pico_head',hf=s.head_follow||{},hn=true;
    $('#head-follow').checked=ho||!!hf.enabled&&hn;$('#head-follow').disabled=s.running||!hn||ho;
    $('#launch').textContent=ho?'Prepare head tracking':'Prepare arms';
    $('#start').textContent=ho?'Start head following':s.pose_match&&s.pose_match.enabled?'Start pose check':'Start following';
    $('#head-recenter').disabled=!(hf.active&&phase==='following');
    $('#head-follow-state').textContent=hf.active?(hf.live?'Head following active':'Head input unavailable; holding neck target'):hf.enabled?'Head following selected for the next START':'Head following off';
    const hv=s.head_video;
    $('#head-video-state').textContent=hv?(hv.streaming?'PICO POV streaming · '+hv.fps+' fps':hv.error?'PICO POV: '+hv.error:'PICO POV ready — select Listen in XRoboToolkit'):'PICO POV bridge unavailable';
    renderWuji(s);
    renderPoseMatch(s);
    $('#launch').disabled=busy||s.running||!!(s.wuji&&s.wuji.active)||graspActive||!modeReady||s.fsm_busy;$('#start').disabled=busy||s.phase!=='ready'||!!(s.pose_match&&s.pose_match.enabled&&(!s.pose_match.calibrated||!s.pose_match.robot_matches));$('#stop').disabled=busy||(!s.running&&!(s.wuji&&s.wuji.active));
  }catch(e){if(++fails>2){$('#auto-rec').disabled=true;$('#grasp-hold').disabled=$('#grasp-release').disabled=true;$('#grasp-state').textContent=graspActive?'Connection lost — H2 was holding and is configured to continue. Live hand status unavailable. Reconnect to release.':'Hold status unavailable — reconnect to H2.';$('#state').textContent='Control page disconnected — check the robot and use its remote if needed';$('#state').className='lost';$('#walking').textContent='Walking readiness unavailable · H2 disconnected';$('#walking').className='waiting';$('#motion').disabled=$('#fsm4').disabled=$('#launch').disabled=$('#start').disabled=true}}
}
function renderPoseMatch(s){
  const p=s.pose_match||{};$('#pose-match').hidden=!p.enabled;
  if(!p.enabled)return;
  const following=s.phase==='following',live=!!(s.pico_tracker&&s.pico_tracker.ready);
  $('#pose-save').disabled=busy||!p.can_calibrate||!live;
  $('#pose-state').textContent=following?'Following · start pose matched':p.waiting?('Waiting for pose match · '+p.message+' · '+Math.round(100*(p.progress||0))+'%'):p.message;
  $('#pose-progress').hidden=!p.waiting;$('#pose-progress').value=p.progress||0;
  for(const side of ['left','right']){
    const v=(p.sides||{})[side],e=$('#pose-'+side);e.hidden=!v||following;
    if(!v)continue;
    const directions=[['forward','back'],['left','right'],['up','down']];
    const hints=(v.move_m||[]).map((x,i)=>Math.abs(x)<.01?'':directions[i][x>=0?0:1]+' '+Math.round(Math.abs(x)*100)+' cm').filter(Boolean);
    e.className=v.matched?'ready':'waiting';
    e.textContent=(side==='left'?'Left':'Right')+': '+(v.matched?'matched':hints.join(', ')||'adjust wrist angle')+' ('+(v.position_error_m*100).toFixed(1)+' cm, '+Number(v.orientation_error_deg).toFixed(1)+'°)';
  }
}
function saveMatchingPose(){
  if(confirm('Match the prepared robot’s wrist positions and angles, and look straight forward. Save this pose for the current wearer and wrist tracker mounts?'))send('pico/pose-calibrate');
}
function renderWuji(s){
  const w=s.wuji||{},b=s.glove_bridge||{};
  const controllers=s.hand_source==='pico_controller_wuji';
  const dual=s.hand_source==='pico_dual_wuji'||controllers;
  const optical=s.hand_source==='pico_tracker_hands';
  const usesGlove=s.hand_source==='wuji'||s.hand_source==='pico_wuji'||dual;
  $('#hand-source').value=s.hand_source||'vr';
  $('#hand-source').disabled=busy||s.running||w.active;
  $('#wuji-connect').hidden=!usesGlove;
  $('#right-glove-test').hidden=!usesGlove;$('#glove-tests').hidden=!usesGlove;
  $('#wuji-connect').textContent=dual?'Connect both Wuji gloves':'Connect Wuji glove';
  $('#wuji-connect').disabled=busy||s.running||w.active||w.connected;
  const feedback=w.hand_feedback;
  const right=(w.hands||{}).right||{},rf=right.hand_feedback||{};
  $('#right-glove-connect').disabled=busy||s.running||w.active||right.connected;
  $('#right-glove-start').disabled=busy||s.running||w.active||graspActive||!right.connected||!rf.ready;
  $('#right-glove-state').textContent=right.active?'Right fingers following · STOP ends the test':!rf.ready?(rf.message||'Waiting for right-hand motor feedback'):right.connected?'Right-hand test READY · press START right-hand test':((b.hands||{}).right||{}).error||'Connect the calibrated right glove';
  $('#wuji-state').textContent=feedback&&!feedback.ready?feedback.message+(w.connected?' · glove input connected':''):w.error?w.error:w.active?'Wuji left fingers following':w.connected?'Wuji left glove connected · START activates fingers':b.error?b.error:usesGlove?'Connect the calibrated left glove':'VR controls both hands';
  if(dual){const hands=w.hands||{};$('#wuji-state').textContent=(b.error||w.error||'')+' '+['left','right'].map(side=>{const h=hands[side]||{},f=h.hand_feedback||{};return side+': '+(!f.ready?(f.message||'waiting for motor feedback'):!h.connected?'connect calibrated glove':h.active?'following':'ready')}).join(' · ')}
  const headOnly=s.hand_source==='pico_head',native=headOnly||s.hand_source==='pico_wuji'||dual||optical,t=s.pico_tracker||{},p=s.tracker_bridge||{};
  if(optical){const hands=(t.hands||{}).sides||{};$('#wuji-state').textContent='PICO fingers · '+['left','right'].map(side=>side+': '+(hands[side]?'tracked':'show hand to headset')).join(' · ')}
  $('#tracker-connect').textContent=headOnly?'Connect PICO headset':controllers?'Connect PICO controller input':'Connect PICO tracker input';
  $('#wuji-state').hidden=headOnly;$('#tracker-assignments').hidden=headOnly||controllers;$('#tracker-setup-help').hidden=headOnly;
  $('#tracker-mode-instructions').textContent=controllers?'Turn on both PICO controllers. Enable Head, Controller, and Send in XRoboToolkit. The left/right controller roles are automatic; wrist trackers and optical hand tracking are not required. Look forward and hold the controllers in your neutral arm pose at START. Wuji gloves control finger positions.':'Enable Head, Object Tracking, and Send. Mount each selected tracker rigidly on the back of its hand. At START, look forward and match your arm pose to the robot. That pose becomes the reference. Two-glove mode follows both arms and wrists. Left-only mode holds the right arm.';
  $('#tracker-controls').hidden=!native;
  $('#tracker-connect').disabled=busy||s.running||p.running;
  $('#tracker-state').textContent=(p.error||t.message||'Tracker status unavailable')+(t.connected?' · '+t.sample_hz+' Hz':'');
  const select=$('#tracker-sn'),serials=t.serial_numbers||[];
  if(JSON.stringify(serials)!==select.dataset.serials){select.replaceChildren(new Option('Choose left tracker',''),...serials.map(sn=>new Option(sn,sn)));select.dataset.serials=JSON.stringify(serials)}
  select.value=t.left_sn||'';select.disabled=busy||s.running;
  $('#right-tracker-select').hidden=!(dual||optical);
  const rightSelect=$('#right-tracker-sn');
  if(JSON.stringify(serials)!==rightSelect.dataset.serials){rightSelect.replaceChildren(new Option('Choose right tracker',''),...serials.map(sn=>new Option(sn,sn)));rightSelect.dataset.serials=JSON.stringify(serials)}
  rightSelect.value=t.right_sn||'';rightSelect.disabled=busy||s.running;
  const startSteps=s.pose_match&&s.pose_match.enabled?'Prepare arms → Save matching pose on first use → Start pose check → hold the matching wrist pose.':'Prepare arms → look forward in your neutral arm pose → Start following.';
  $('#source-help').textContent=headOnly?'Head + Send in XRoboToolkit. Prepare head tracking → look forward → Start head following. Arms hold their current pose; finger following is off. Stop ends the test.':controllers?'XRoboToolkit: Head + Controller + Send. Connect PICO controller input + both calibrated Wuji gloves. Controllers drive arms/wrists; gloves drive fingers. '+startSteps+' Stop ends arms, fingers and head following.':optical?'XRoboToolkit: Head + Hand + Object Tracking + Send. Connect PICO tracker input and show both hands. Trackers drive arms/wrists; optical hand tracking drives fingers. '+startSteps+' Wuji gloves are not required.':native?'Connect tracker(s) + calibrated glove(s). '+startSteps+' Stop ends arms, fingers and head following.':'Browser modes: Connect glove if selected → LAUNCH → enter headset Pass-through → START. Wuji controls left fingers; right fingers keep their last pose.';
}
async function selectTracker(side='left'){
  busy=true;
  try{const r=await fetch('/api/pico/tracker',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({serial:$(side==='right'?'#right-tracker-sn':'#tracker-sn').value,side})});const s=await r.json();$('#msg').textContent=s.message}
  catch(e){$('#msg').textContent='Tracker selection failed: '+e.message}
  finally{busy=false;refresh()}
}
async function setHandSource(){
  busy=true;
  try{const r=await fetch('/api/hand-source',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({source:$('#hand-source').value})});const s=await r.json();$('#msg').textContent=s.message}
  catch(e){$('#msg').textContent='Source change failed: '+e.message}
  finally{busy=false;refresh()}
}
function renderGrasp(g,teleop,gains){
  graspActive=!!(g&&g.active);
  $('#grasp-hold').disabled=busy||teleop||graspActive||!g;
  $('#grasp-release').disabled=busy||!graspActive;
  $('#grasp-state').textContent=g?g.message:'Standalone grasp unavailable';
  const profile=graspActive?g.gains:(gains&&gains.saved);
  $('#grasp-settings').textContent=profile?(graspActive?'Active hold: ':'Next hold: ')+describeHandGains(profile)+(graspActive&&g.feedback_age>0.5?' · Hand feedback is stale; physical grip unconfirmed.':''):'';
  if(teleop&&!graspActive)$('#grasp-state').textContent='Stop Quest teleop to use standalone right-hand grasp.';
}
let cameraRefresh=0,headCameraRefresh=0;
async function setHeadFollow(){try{const r=await fetch('/api/head-follow',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({enabled:$('#head-follow').checked})});const s=await r.json();$('#msg').textContent=s.message}catch(e){$('#msg').textContent='Head setting failed: '+e.message}finally{refresh()}}
function renderCamera(c){
  if(!c){$('#camera-state').textContent='Camera service unavailable';return}
  $('#camera-state').textContent=c.live?'Live · '+c.fps+' fps · '+c.display:'Waiting for camera frames'+(c.error?': '+c.error:'');
  $('#camera-image').hidden=!c.live;
  if(c.live&&Date.now()-cameraRefresh>1000){cameraRefresh=Date.now();$('#camera-image').src='/api/camera.jpg?t='+cameraRefresh}
  const h=c.head;
  $('#head-camera-state').textContent=h&&h.live?'Live · '+h.fps+' fps · '+h.view:'Waiting for head camera'+(h&&h.error?': '+h.error:'');
  $('#head-camera-image').hidden=!(h&&h.live);
  if(h&&h.live&&Date.now()-headCameraRefresh>250){headCameraRefresh=Date.now();$('#head-camera-image').src='/api/head-camera.jpg?t='+headCameraRefresh}
}
let gainsDirty=false,gainsSaving=false;
function showHandGainValues(){
  $('#finger-value').textContent=Number($('#finger-gain').value).toFixed(2)+'×';
  $('#thumb-value').textContent=Number($('#thumb-gain').value).toFixed(2)+'×';
}
function editHandGains(){gainsDirty=true;showHandGainValues();$('#gain-message').textContent='Unsaved change — press Save for next launch.'}
function describeHandGains(g){return 'fingers '+Number(g.finger_multiplier).toFixed(2)+'×, thumb '+Number(g.thumb_multiplier).toFixed(2)+'×'}
function renderHandGains(g){
  if(!g)return;
  for(const id of ['finger-gain','thumb-gain','gain-save','gain-reset'])$('#'+id).disabled=gainsSaving;
  if(g.saved&&!gainsDirty&&!gainsSaving){$('#finger-gain').value=g.saved.finger_multiplier;$('#thumb-gain').value=g.saved.thumb_multiplier;showHandGainValues()}
  $('#finger-gain').min=$('#thumb-gain').min=g.min;$('#finger-gain').max=$('#thumb-gain').max=g.max;
  if(g.error){$('#gain-state').textContent='Settings error: '+g.error;return}
  let message='Saved for next launch or grasp: '+describeHandGains(g.saved)+'.';
  if(g.active)message+=' Running: '+describeHandGains(g.active)+'.'+(g.pending?' Saved changes are pending the next LAUNCH.':'');
  else if(g.external)message+=' Running settings unavailable for a process launched outside this page.';
  else message+=' Teleop is stopped.';
  $('#gain-state').textContent=message;
}
async function saveHandGains(reset=false){
  if(gainsSaving)return;
  if(reset){$('#finger-gain').value=1;$('#thumb-gain').value=1;showHandGainValues()}
  gainsDirty=true;gainsSaving=true;
  for(const id of ['finger-gain','thumb-gain','gain-save','gain-reset'])$('#'+id).disabled=true;
  try{const r=await fetch('/api/hand-gains',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({finger_multiplier:Number($('#finger-gain').value),thumb_multiplier:Number($('#thumb-gain').value)})});
    const s=await r.json();if(!r.ok)throw Error(s.message);gainsDirty=false;$('#gain-message').textContent=s.message;
  }catch(e){$('#gain-message').textContent='Could not save: '+e.message}finally{gainsSaving=false;await refresh()}
}
let autoRecSaving=false;
async function setAutoRecording(){
  autoRecSaving=true;$('#auto-rec').disabled=true;
  try{const r=await fetch('/api/auto-record',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({enabled:$('#auto-rec').checked})});const s=await r.json();if(!r.ok)throw Error(s.message);$('#msg').textContent=s.message}
  catch(e){$('#msg').textContent='Could not save automatic recording setting: '+e.message}
  finally{autoRecSaving=false;refresh()}
}
let recOn=false;
function renderRec(r){
  const a=r.auto;
  $('#auto-rec').disabled=busy||autoRecSaving||!a;
  if(a&&!autoRecSaving)$('#auto-rec').checked=a.enabled;
  $('#auto-rec-state').textContent=a?a.message+' Long sessions use consecutive 20-minute takes. STOP REC pauses automatic recording for the current session.':'Automatic recording unavailable';
  recOn=!!r.recording;$('#rec').disabled=busy;$('#rec').className=recOn?'on':'';
  $('#rec').textContent=recOn?'■ STOP REC':'● REC head + wrist video';
  const c=r.cams||{},j=r.joints||{},rt=j.rates||{},t=Math.floor(r.elapsed||0);
  const cams=Object.keys(c).map(k=>k+' '+(c[k].error?'⚠ '+c[k].error:c[k].fps+' fps')).join(' · ');
  const mb=Object.values(c).reduce((a,x)=>a+(x.mb||0),0);
  const joints=j.error?'joints ⚠ '+j.error:'arm '+(rt.arm_state||0)+'/'+(rt.arm_cmd||0)+' Hz · hands '+(rt.left_hand_state||0)+'/'+(rt.left_hand_cmd||0)+' Hz (state/cmd)';
  $('#recinfo').className=recOn?'on':'';
  $('#recinfo').textContent=r.dir?((recOn?'● REC '+String(Math.floor(t/60)).padStart(2,'0')+':'+String(t%60).padStart(2,'0'):'Saved')+' · '+r.dir+' · '+cams+' · '+joints+' · '+(mb>=1000?(mb/1000).toFixed(1)+' GB':Math.round(mb)+' MB')):'Not recording (takes saved to ~/h2-recordings on PC2)';
}
function toggleRec(){send(recOn?'rec-stop':'rec-start')}
function fsmConfirm(){
  if(confirm('The whole body will power up (0→1→4); legs and torso will move.\\nIs the robot on the gantry or standing stably, with nobody nearby?'))send('fsm4');
}
function motionConfirm(){
  if(confirm('Enable walking with arm teleop using Unitree mode 703. The legs may move or step.\\nBoth feet must be on the ground; keep support until stable. Use the Unitree remote for walking. Dashboard STOP ends arm teleop only.'))send('motion');
}
function launchConfirm(){
  if($('#hand-source').value==='pico_head'){send('launch');return}
  const hands=['vr','pico_tracker_hands'].includes($('#hand-source').value)?'Both hands will open right away.':'Wuji fingers start when you press START.';
  if(confirm('The arms will move to zero. '+hands+'\\nReady to launch in the displayed body mode?'))send('launch');
}
async function send(a){busy=true;refresh();
  try{const r=await fetch('/api/'+a,{method:'POST'});const s=await r.json();$('#msg').textContent=s.message}
  catch(e){$('#msg').textContent='Request failed: '+e.message}finally{busy=false;refresh()}}
refresh();setInterval(refresh,500);
</script></html>'''


def handler(up, rec, auto=None):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def respond(self, code, payload, ctype='application/json'):
            data = payload if isinstance(payload, bytes) else payload.encode() if isinstance(payload, str) else json.dumps(payload, ensure_ascii=False).encode()
            self.send_response(code)
            self.send_header('Content-Type', ctype + '; charset=utf-8')
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path.split('?', 1)[0] == '/api/head-camera.jpg':
                camera = getattr(up, 'camera', None)
                head = getattr(camera, 'head', None)
                if head is not None and head.status()['live'] and head.jpeg is not None:
                    self.respond(200, head.jpeg, 'image/jpeg')
                else:
                    self.respond(503, {'message': 'Head camera frames are not available yet'})
            elif self.path.split('?', 1)[0] == '/api/camera.jpg':
                camera = getattr(up, 'camera', None)
                if camera is not None and camera.jpeg is not None:
                    self.respond(200, camera.jpeg, 'image/jpeg')
                else:
                    self.respond(503, {'message': 'Camera frames are not available yet'})
            elif self.path == PAGE_PATH:
                self.respond(200, PAGE, 'text/html')
            elif self.path == '/':
                self.send_response(302)
                self.send_header('Location', PAGE_PATH)
                self.send_header('Content-Length', '0')
                self.end_headers()
            elif self.path == '/api/status':
                recording = rec.status()
                if auto is not None:
                    recording['auto'] = auto.status()
                self.respond(200, dict(up.status(), rec=recording))
            else:
                self.respond(404, {'message': 'not found'})

        def do_POST(self):
            if self.path == '/api/head-follow':
                try:
                    size = int(self.headers.get('Content-Length', '0'))
                    if not 0 < size <= 1024: raise ValueError('Provide enabled: true or false')
                    value = json.loads(self.rfile.read(size))
                    if not isinstance(value, dict) or set(value) != {'enabled'}:
                        raise ValueError('Provide only enabled')
                    ok, message = up.set_head_follow(value['enabled'])
                    return self.respond(200 if ok else 409, {'message': message})
                except (ValueError, TypeError) as exc:
                    return self.respond(400, {'message': str(exc)})
            if self.path == '/api/pico/tracker':
                try:
                    size = int(self.headers.get('Content-Length', '0'))
                    if not 0 < size <= 1024: raise ValueError('Provide a tracker serial')
                    value = json.loads(self.rfile.read(size))
                    if not isinstance(value, dict) or not {'serial'} <= set(value) <= {'serial', 'side'} or not isinstance(value['serial'], str):
                        raise ValueError('Provide serial and optional side')
                    side = value.get('side', 'left')
                    if side not in ('left', 'right'): raise ValueError('Choose left or right tracker')
                    ok, message = up.select_tracker(value['serial'], side)
                    return self.respond(200 if ok else 409, {'message': message})
                except (ValueError, TypeError) as exc:
                    return self.respond(400, {'message': str(exc)})
            if self.path == '/api/hand-source':
                try:
                    size = int(self.headers.get('Content-Length', '0'))
                    if not 0 < size <= 1024: raise ValueError('Provide a source')
                    value = json.loads(self.rfile.read(size))
                    if not isinstance(value, dict) or set(value) != {'source'}:
                        raise ValueError('Provide only source')
                    ok, message = up.set_hand_source(value['source'])
                    return self.respond(200 if ok else 409, {'message': message})
                except (ValueError, TypeError) as exc:
                    return self.respond(400, {'message': str(exc)})
            if self.path == '/api/auto-record':
                if auto is None:
                    return self.respond(409, {'message': 'Automatic recording unavailable'})
                try:
                    size = int(self.headers.get('Content-Length', '0'))
                    if not 0 < size <= 1024:
                        raise ValueError('Provide an enabled boolean (maximum 1024 bytes)')
                    value = json.loads(self.rfile.read(size).decode('utf-8'))
                    if not isinstance(value, dict) or set(value) != {'enabled'}:
                        raise ValueError('Provide only enabled: true or false')
                    ok, message = auto.set_enabled(value['enabled'])
                    return self.respond(200 if ok else 409, {'message': message})
                except (ValueError, UnicodeError) as exc:
                    return self.respond(400, {'message': str(exc)})
                except OSError as exc:
                    return self.respond(500, {'message': str(exc)})
            if self.path == '/api/hand-gains':
                try:
                    size = int(self.headers.get('Content-Length', '0'))
                    if not 0 < size <= 1024:
                        raise ValueError('Provide a JSON hand settings object (maximum 1024 bytes)')
                    values = json.loads(self.rfile.read(size).decode('utf-8'))
                    ok, message = up.save_hand_gains(values)
                    return self.respond(200 if ok else 409, {'message': message})
                except (ValueError, UnicodeError) as exc:
                    return self.respond(400, {'message': str(exc)})
                except OSError as exc:
                    return self.respond(500, {'message': f'Could not save settings: {exc}'})
            actions = {'/api/launch': up.launch, '/api/start': up.start, '/api/stop': up.stop,
                       '/api/pico/pose-calibrate': up.save_matching_pose,
                       '/api/head-recenter': lambda: up._send('CMD_HEAD_RECENTER'),
                       '/api/wuji/test-right': lambda: up.start_wuji_test('right'),
                       '/api/fsm4': lambda: up.body_mode(4),
                       '/api/motion': lambda: up.body_mode(703),
                       '/api/rec-start': auto.manual_start if auto else rec.start,
                       '/api/rec-stop': auto.manual_stop if auto else rec.stop,
                       '/api/grasp-hold': up.grasp_start, '/api/grasp-release': up.grasp_release}
            if self.path not in actions:
                return self.respond(404, {'message': 'not found'})
            try:
                ok, message = actions[self.path]()
                print(time.strftime('%H:%M:%S'), self.client_address[0], self.path, ok, message, flush=True)
                self.respond(200 if ok else 409, {'message': message})
            except Exception as exc:
                self.respond(500, {'message': str(exc)})
    return Handler


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', default=8787, type=int)
    parser.add_argument('--network-interface', default='eth0')
    args = parser.parse_args()
    up = Upstream(Fsm(args.network_interface))
    from quest_wuji import Dex5Feedback, WujiHand, WujiHands
    up.wuji = WujiHands(*[WujiHand(ROOT, sys.executable, args.network_interface,
                                 feedback=Dex5Feedback(ROOT, side), side=side)
                         for side in ('left', 'right')], dual=up.hand_source in DUAL_WUJI_SOURCES)
    from quest_right_grasp import GraspManager
    up.grasp = GraspManager(up.gain_store, args.network_interface)
    from quest_camera_service import CameraService
    up.camera = CameraService(up.running)
    rec = Recorder(args.network_interface)
    from quest_auto_record import AutoRecording
    auto = AutoRecording(rec, up.recording_snapshot, Path(__file__).resolve().parent / 'config/auto_recording.json')
    up.auto_recording = auto

    def shutdown(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, shutdown)
    server = ThreadingHTTPServer((args.host, args.port), handler(up, rec, auto))
    print(f'H2 control page on http://{args.host}:{args.port}{PAGE_PATH} (nothing launched)', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        up.close()   # Quest exits; standalone right-hand hold deliberately survives HTTP shutdown.
        auto.close()
        rec.close()
        up.camera.close()
        up.wuji.close()
