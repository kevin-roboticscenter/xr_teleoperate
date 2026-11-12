#!/usr/bin/env python3
"""Laptop dashboard proxy and official Wuji SDK input connection.

The robot dashboard owns motor START/STOP. Connecting here sends input only.
"""
from http.client import HTTPConnection
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import argparse
import json
import os
from pathlib import Path
import shlex
import signal
import socket
import subprocess
import sys
import threading
from urllib.parse import urlsplit

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import h2_link


class GloveBridge:
    def __init__(self, side='left'):
        self.side = side
        self.lock = threading.RLock()
        self.source = self.relay = None
        self.error = ''

    def status(self):
        with self.lock:
            alive = (self.source is not None and self.source.poll() is None and
                     self.relay is not None and self.relay.poll() is None)
            if self.source is not None and not alive and not self.error:
                self.error = 'Glove connection ended; check the glove and H2 SSH connection'
            return {'running': alive, 'error': self.error, 'side': self.side}

    def connect(self):
        with self.lock:
            if self.status()['running']: return True, 'Wuji input already connected'
            self.close()
            gloves = ROOT / 'config/wuji_gloves.json'
            devices = json.loads(gloves.read_text()) if gloves.exists() else {}
            serial = devices.get(self.side)
            if not serial:
                self.error = f'Identify and configure the {self.side} Wuji glove first'
                return False, self.error
            profile = subprocess.run(['wuji', 'user', 'show', '--json'], capture_output=True, text=True, timeout=8)
            profile.check_returncode()
            if not json.loads(profile.stdout).get(self.side+'_hand', {}).get('calibrated'):
                self.error = f'Calibrate the {self.side} glove for the current operator profile first'
                return False, self.error
            config = h2_link.load_config()
            host = h2_link.selected_host(config)
            control = Path.home() / 'h2-network-switch' / ('.ssh-wifi' if config['mode'] == 'wifi' else '.ssh-ethernet')
            ssh = ['ssh', '-S', str(control), '-o', 'HostKeyAlias=192.168.124.162',
                   '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5', f'unitree@{host}']
            probe = subprocess.run(ssh + ['true'], capture_output=True, timeout=7)
            if probe.returncode:
                self.error = 'H2 SSH login needed before connecting the glove'
                return False, self.error
            logs = ROOT / 'wuji_glove/evidence'
            logs.mkdir(exist_ok=True)
            with (logs / f'dashboard-{self.side}-glove-connection.log').open('a') as log:
                self.source = subprocess.Popen(
                    [str(ROOT / '.runtime-glove/bin/python'), '-u',
                     str(ROOT / 'wuji_glove/stream_latest_tips.py'), '--sn', serial, '--side', self.side],
                    stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=log,
                    start_new_session=True)
                command = [os.environ.get('H2_REMOTE_PYTHON', '/home/unitree/anaconda3/envs/tv/bin/python'), '-u',
                           str(Path(os.environ.get('H2_REMOTE_ROOT', '/home/unitree/h2-upstream-pr321')) / 'wuji_glove/send_input.py'), '--side', self.side]
                self.relay = subprocess.Popen(ssh + [shlex.join(command)],
                                              stdin=self.source.stdout, stdout=log, stderr=log,
                                              start_new_session=True)
                self.source.stdout.close()
            self.error = ''
            return True, f'Connecting calibrated {self.side} glove; START activates robot fingers'

    def close(self):
        with self.lock:
            for process in (self.source, self.relay):
                if process is not None and process.poll() is None:
                    process.send_signal(signal.SIGINT)
                    try: process.wait(timeout=4)
                    except subprocess.TimeoutExpired:
                        process.terminate()
                        process.wait(timeout=3)
            self.source = self.relay = None


class GloveBridges:
    def __init__(self):
        self.hands = {side: GloveBridge(side) for side in ('left', 'right')}
        self.dual, self.error = False, ''

    def set_dual(self, dual):
        self.dual = bool(dual)
        if not dual: self.hands['right'].close()

    def status(self):
        hands = {side: bridge.status() for side, bridge in self.hands.items()}
        required = ('left', 'right') if self.dual else ('left',)
        return {'running': all(hands[side]['running'] for side in required), 'hands': hands,
                'error': self.error or next((hands[side]['error'] for side in required if hands[side]['error']), ''),
                'side': 'both' if self.dual else 'left'}

    def connect(self):
        connection = HTTPConnection(h2_link.selected_host(), 8787, timeout=5)
        try:
            connection.request('GET', '/api/status')
            response = connection.getresponse()
            if response.status != 200: raise OSError('H2 status unavailable')
            self.set_dual(json.loads(response.read())['hand_source'] == 'pico_dual_wuji')
        finally:
            connection.close()
        for side in (('left', 'right') if self.dual else ('left',)):
            ok, message = self.hands[side].connect()
            if not ok:
                self.error = message
                return False, message
        self.error = ''
        return True, 'Glove input(s) connected; START activates robot fingers'

    def close(self):
        for bridge in self.hands.values(): bridge.close()


class TrackerBridge(GloveBridge):
    """Official SDK reader relayed over SSH; connecting never starts motors."""
    def status(self):
        value = super().status()
        value['error'] = value['error'].replace('Glove', 'Tracker').replace('glove', 'tracker')
        return value

    def connect(self):
        with self.lock:
            if self.status()['running']: return True, 'PICO input already connected'
            self.close()
            config = h2_link.load_config()
            host = h2_link.selected_host(config)
            control = Path.home() / 'h2-network-switch' / ('.ssh-wifi' if config['mode'] == 'wifi' else '.ssh-ethernet')
            ssh = ['ssh', '-S', str(control), '-o', 'HostKeyAlias=192.168.124.162',
                   '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5', f'unitree@{host}']
            probe = subprocess.run(ssh + ['true'], capture_output=True, timeout=7)
            if probe.returncode:
                self.error = 'H2 SSH login needed before connecting PICO input'
                return False, self.error
            pico = Path.home() / 'h2-pico-official'
            try:
                with socket.create_connection(('127.0.0.1', 63901), timeout=.2): pass
            except OSError:
                with (pico / 'logs/pico-pc-service.log').open('a') as service_log:
                    subprocess.Popen(['bash', str(pico / 'runtime/opt/apps/roboticsservice/runService.sh')],
                                     stdin=subprocess.DEVNULL, stdout=service_log, stderr=service_log,
                                     start_new_session=True)
            with (ROOT / 'wuji_glove/evidence/dashboard-tracker-connection.log').open('a') as log:
                self.source = subprocess.Popen(
                    [str(pico / '.venv/bin/python'), '-u', str(ROOT / 'pico_tracker/stream_trackers.py')],
                    stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=log, start_new_session=True)
                command = [os.environ.get('H2_REMOTE_PYTHON', '/home/unitree/anaconda3/envs/tv/bin/python'), '-u',
                           str(Path(os.environ.get('H2_REMOTE_ROOT', '/home/unitree/h2-upstream-pr321')) / 'pico_tracker/receive.py')]
                self.relay = subprocess.Popen(ssh + [shlex.join(command)], stdin=self.source.stdout,
                                              stdout=log, stderr=log, start_new_session=True)
                self.source.stdout.close()
            self.error = ''
            return True, 'PICO input connected; native app needs Head + Object Tracking + Send'


def handler(bridge, tracker=None, video=None):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_): pass

        def reply(self, code, data, content_type='application/json'):
            if not isinstance(data, bytes): data = json.dumps(data).encode()
            self.send_response(code)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(data)

        def proxy(self):
            connection = None
            try:
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 <= length <= 65536:
                    return self.reply(413, {'message': 'Request is too large'})
                body = self.rfile.read(length) if length else None
                connection = HTTPConnection(h2_link.selected_host(), 8787, timeout=35 if self.path == '/api/start' else 20)
                connection.request(self.command, self.path, body,
                                   {'Content-Type': self.headers.get('Content-Type', 'application/json')})
                response = connection.getresponse()
                data = response.read()
                if self.path == '/api/status' and response.status == 200:
                    status = json.loads(data)
                    status['glove_bridge'] = bridge.status()
                    status['tracker_bridge'] = tracker.status() if tracker else None
                    status['head_video'] = video.status() if video else None
                    data = json.dumps(status).encode()
                if self.command == 'POST' and self.path == '/api/hand-source' and response.status == 200:
                    if json.loads(body)['source'] in ('vr', 'pico_head'): bridge.close()
                    if callable(getattr(bridge, 'set_dual', None)):
                        bridge.set_dual(json.loads(body)['source'] == 'pico_dual_wuji')
                    if tracker and json.loads(body)['source'] not in ('pico_wuji', 'pico_dual_wuji', 'pico_head'): tracker.close()
                if 300 <= response.status < 400:
                    self.send_response(response.status)
                    self.send_header('Location', response.getheader('Location', '/h2-teleop'))
                    self.send_header('Content-Length', '0')
                    self.end_headers()
                else:
                    self.reply(response.status, data, response.getheader('Content-Type', 'application/json'))
            except (OSError, ValueError) as exc:
                self.reply(503, {'message': f'H2 connection unavailable: {exc}'})
            finally:
                if connection is not None: connection.close()

        def do_GET(self): self.proxy()

        def do_POST(self):
            origin = self.headers.get('Origin')
            if origin and urlsplit(origin).netloc != self.headers.get('Host'):
                return self.reply(403, {'message': 'Use this dashboard to send commands'})
            if self.path == '/api/wuji/right/connect' and hasattr(bridge, 'hands'):
                try:
                    ok, message = bridge.hands['right'].connect()
                    return self.reply(200 if ok else 409, {'message': message})
                except (OSError, subprocess.SubprocessError) as exc:
                    bridge.hands['right'].error = str(exc)
                    return self.reply(503, {'message': 'Right glove input connection failed; check the glove and H2 SSH'})
            if self.path == '/api/wuji/connect':
                try:
                    ok, message = bridge.connect()
                    return self.reply(200 if ok else 409, {'message': message})
                except (OSError, subprocess.SubprocessError) as exc:
                    bridge.error = str(exc)
                    return self.reply(503, {'message': 'Glove connection failed; check H2 SSH and the glove'})
            if self.path == '/api/wuji/disconnect':
                bridge.close()
                return self.reply(200, {'message': 'Glove input disconnected'})
            if self.path == '/api/pico/connect' and tracker:
                try:
                    ok, message = tracker.connect()
                    return self.reply(200 if ok else 409, {'message': message})
                except (OSError, subprocess.SubprocessError) as exc:
                    tracker.error = str(exc)
                    return self.reply(503, {'message': 'PICO input connection failed; check H2 SSH and XRoboToolkit'})
            if self.path == '/api/pico/disconnect' and tracker:
                tracker.close()
                return self.reply(200, {'message': 'PICO tracker input disconnected'})
            self.proxy()
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8787)
    args = parser.parse_args()
    bridge = GloveBridges()
    tracker = TrackerBridge()
    from head_camera.video_service import PicoVideoService
    video = PicoVideoService(ROOT, h2_link.selected_host())
    server = ThreadingHTTPServer((args.host, args.port), handler(bridge, tracker, video))
    def stop(*_): raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop)
    try: server.serve_forever()
    except KeyboardInterrupt: pass
    finally:
        server.server_close()
        bridge.close()
        tracker.close()
        video.close()


if __name__ == '__main__': main()
