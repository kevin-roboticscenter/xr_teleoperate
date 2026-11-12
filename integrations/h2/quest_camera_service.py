"""Manage the official camera stream and a Vuer preview before/after teleop."""
from contextlib import contextmanager
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
from quest_h2_teleop import ROOT, UPSTREAM, display_mode, environment


class CameraService:
    def __init__(self, teleop_running):
        self.teleop_running = teleop_running
        self.lock = threading.RLock()
        self.stop = threading.Event()
        self.stream = self.preview = None
        self.jpeg = None
        self.last_frame = 0.0
        self.fps = 0.0
        self.error = ''
        self.logs = Path.home() / 'h2-logs'
        self.logs.mkdir(exist_ok=True)
        sys.path[:0] = environment()['PYTHONPATH'].split(os.pathsep)
        self.worker = threading.Thread(target=self._run, daemon=True)
        self.worker.start()
        from head_camera.receiver import HeadCameraReceiver
        self.head = HeadCameraReceiver()

    def _spawn(self, args, log_name):
        with (self.logs / log_name).open('a') as log:
            return subprocess.Popen(args, cwd=ROOT, env=environment(),
                                    stdin=subprocess.DEVNULL, stdout=log,
                                    stderr=subprocess.STDOUT, start_new_session=True)

    @staticmethod
    def _terminate(process):
        if process is None or process.poll() is not None:
            return
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=2)

    @contextmanager
    def launch_slot(self):
        # Keep the preview supervisor from reclaiming 8012 during teleop startup.
        with self.lock:
            self._terminate(self.preview)
            self.preview = None
            yield

    def _run(self):
        client = None
        sample = None
        try:
            from teleimager.image_client import HeadImageClient
            while not self.stop.is_set():
                try:
                    with self.lock:
                        if self.stream is None or self.stream.poll() is not None:
                            self.stream = self._spawn([sys.executable, '-m', 'teleimager.image_server',
                                                       '--no-affinity'], 'camera_server.log')
                            if self.stop.wait(2):
                                break
                        if client is None:
                            client = HeadImageClient(host='127.0.0.1', request_bgr=False)
                        if not self.teleop_running() and (self.preview is None or self.preview.poll() is not None):
                            self.preview = self._spawn([sys.executable, str(ROOT / 'quest_camera_preview.py')],
                                                       'camera_preview.log')
                    frame = client.get_frame()
                    if frame.jpg is not None and frame.jpg is not sample:
                        self.jpeg, self.last_frame, self.fps = frame.jpg, time.monotonic(), frame.fps
                        sample = frame.jpg
                    self.error = ''
                except Exception as exc:
                    self.error = str(exc)
                    self.stop.wait(2)
                self.stop.wait(.1)
        finally:
            if client is not None:
                client.close()

    def status(self):
        age = time.monotonic() - self.last_frame if self.last_frame else None
        return {'live': age is not None and age < 2, 'fps': round(self.fps, 1),
                'age': round(age, 2) if age is not None else None, 'error': self.error,
                'preview': self.preview is not None and self.preview.poll() is None,
                'head': self.head.status(),
                'display': ('Desktop camera preview; headset video off for tracking latency test'
                            if display_mode() == 'pass-through' else
                            'Both hand cameras in the headset')}

    def close(self):
        self.stop.set()
        self.worker.join(timeout=4)
        self.head.close()
        with self.lock:
            self._terminate(self.preview)
            self._terminate(self.stream)
