"""Own the laptop's XRoboToolkit-compatible video adapter process."""
from http.client import HTTPConnection
import json
from pathlib import Path
import socket
import subprocess


class PicoVideoService:
    def __init__(self, root, robot_host):
        self.process = None
        self.error = ''
        root = Path(root)
        try:
            with socket.create_connection(('127.0.0.1', 13579), timeout=.2):
                return
        except OSError:
            pass
        python = root / '.runtime-video/bin/python'
        if not python.exists():
            self.error = 'PICO video environment is not installed'
            return
        (root / 'head_camera/evidence').mkdir(parents=True, exist_ok=True)
        with (root / 'head_camera/evidence/pico-video.log').open('a') as log:
            self.process = subprocess.Popen(
                [str(python), '-u', str(root / 'head_camera/pico_video.py'),
                 '--source', f'http://{robot_host}:8787/api/head-camera.jpg'],
                stdin=subprocess.DEVNULL, stdout=log, stderr=log, start_new_session=True)

    def status(self):
        connection = HTTPConnection('127.0.0.1', 13581, timeout=.2)
        try:
            connection.request('GET', '/')
            response = connection.getresponse()
            return json.loads(response.read())
        except (OSError, ValueError):
            return {'streaming': False, 'fps': 0., 'error': self.error or 'PICO video adapter is unavailable'}
        finally:
            connection.close()

    def close(self):
        if self.process is not None and self.process.poll() is None:
            self.process.terminate()
            try: self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=2)
