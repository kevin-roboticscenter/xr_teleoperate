"""Dashboard adapter for the official TeleImager stream on H2 PC3."""
import json
from pathlib import Path
import threading
import time


class HeadCameraReceiver:
    def __init__(self):
        self.jpeg = None
        self.last_frame = 0.0
        self.fps = 0.0
        self.error = ''
        self.config = {}
        self.stop = threading.Event()
        try:
            self.config = json.loads(Path(__file__).with_name('receiver.json').read_text())
        except (OSError, ValueError) as exc:
            self.error = str(exc)
        self.worker = threading.Thread(target=self._run, daemon=True)
        self.worker.start()

    def _run(self):
        if not self.config.get('enabled', False):
            return
        manager = None
        sample = None
        try:
            # The deployment already adds the pinned Unitree client to PYTHONPATH.
            from teleimager.image_client import ZMQ_SubscriberManager
            manager = ZMQ_SubscriberManager.get_instance()
            while not self.stop.is_set():
                try:
                    frame = manager.subscribe(self.config['host'], self.config['port'],
                                              request_bgr=False)
                    if frame.jpg is not None and frame.jpg is not sample:
                        self.jpeg = sample = frame.jpg
                        self.last_frame = time.monotonic()
                        self.fps = frame.fps
                    self.error = ''
                except Exception as exc:
                    self.error = str(exc)
                    self.stop.wait(2)
                self.stop.wait(.02)
        except Exception as exc:
            self.error = str(exc)
        finally:
            if manager is not None:
                manager.close()

    def status(self):
        age = time.monotonic() - self.last_frame if self.last_frame else None
        return {'enabled': self.config.get('enabled', False),
                'live': age is not None and age < 2,
                'fps': round(self.fps, 1),
                'age': round(age, 2) if age is not None else None,
                'view': self.config.get('view', 'Head camera'), 'error': self.error}

    def close(self):
        self.stop.set()
        self.worker.join(timeout=3)
