"""Record the existing PC3 TeleImager JPEG stream as playable head.mp4."""
import csv
import json
from pathlib import Path
import shutil
import subprocess
import threading
import time


def load_config(path):
    try:
        value = json.loads(Path(path).read_text())
        if not value.get('enabled') or not isinstance(value.get('host'), str) or not isinstance(value.get('port'), int):
            raise ValueError('Head camera recording source is not configured')
        return value, ''
    except (OSError, ValueError, TypeError) as exc:
        return {}, str(exc)


class HeadVideo(threading.Thread):
    def __init__(self, out, fps, stop, config_path, subscribe=None):
        super().__init__(daemon=True)
        if not isinstance(fps, int) or not 0 < fps <= 120:
            raise ValueError('Head recording FPS must be an integer from 1 to 120')
        self.name = 'head'
        self.out, self.fps, self.stop = Path(out), fps, stop
        self.config, self.error = load_config(config_path)
        self.subscribe = subscribe
        self.frames, self.bytes, self.rate = 0, 0, 0.
        self.encoder = None
        self.encoder_log = None

    def _start_encoder(self):
        ffmpeg = shutil.which('ffmpeg')
        if ffmpeg is None:
            raise RuntimeError('ffmpeg is required to record head.mp4')
        self.encoder_log = (self.out/'head_encoder.log').open('ab')
        try:
            self.encoder = subprocess.Popen(
                [ffmpeg, '-hide_banner', '-loglevel', 'error', '-nostdin', '-n',
                 '-use_wallclock_as_timestamps', '1', '-f', 'image2pipe', '-vcodec', 'mjpeg',
                 '-framerate', str(self.fps), '-i', 'pipe:0', '-an', '-c:v', 'libx264',
                 '-threads', '2', '-preset', 'ultrafast', '-crf', '23', '-pix_fmt', 'yuv420p',
                 '-vsync', 'vfr', '-g', str(self.fps), '-movflags',
                 '+frag_keyframe+empty_moov+default_base_moof', str(self.out/'head.mp4')],
                stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=self.encoder_log,
                bufsize=0, start_new_session=True)
        except Exception:
            self.encoder_log.close()
            self.encoder_log = None
            raise

    def finish_encoder(self, force=False):
        encoder = self.encoder
        if encoder is not None:
            if force and encoder.poll() is None:
                encoder.kill()
            try:
                if encoder.stdin is not None and not encoder.stdin.closed:
                    encoder.stdin.close()
                code = encoder.wait(timeout=3)
                if code:
                    self.error = f'Head video encoder exited with code {code}; see head_encoder.log'
            except (OSError, subprocess.TimeoutExpired):
                encoder.kill()
                encoder.wait(timeout=1)
                self.error = 'Head video encoder timed out during save'
        if self.encoder_log is not None:
            self.encoder_log.close()
            self.encoder_log = None
        path = self.out/'head.mp4'
        if path.exists(): self.bytes = path.stat().st_size

    def run(self):
        if not self.config:
            return
        try:
            if self.subscribe is None:
                from teleimager.image_client import ZMQ_SubscriberManager
                manager = ZMQ_SubscriberManager.get_instance()
                self.subscribe = lambda: manager.subscribe(self.config['host'], self.config['port'], request_bgr=False)
            previous, window, last = None, [], time.monotonic()
            self._start_encoder()
            with (self.out/'head_frames.csv').open('w', newline='') as index:
                writer = csv.writer(index)
                writer.writerow(['frame', 't', 'bytes'])
                while not self.stop.wait(1/self.fps):
                    try:
                        frame = self.subscribe()
                        if frame.jpg is None or frame.jpg is previous:
                            if time.monotonic()-last > 2:
                                self.error, self.rate = 'Head camera stream is not delivering new frames', 0.
                            continue
                        data = frame.jpg
                        if not data.startswith(b'\xff\xd8') or not data.endswith(b'\xff\xd9'):
                            raise ValueError('Invalid head camera JPEG')
                        # The recorder is a separate process. Encoder backpressure
                        # never blocks the teleop loop or its camera preview.
                        view = memoryview(data)
                        while view:
                            written = self.encoder.stdin.write(view)
                            if not written: raise BrokenPipeError('Head encoder input closed')
                            view = view[written:]
                        previous, t, last = data, time.time(), time.monotonic()
                        writer.writerow([self.frames, f'{t:.6f}', len(data)])
                        self.frames += 1
                        window = [v for v in window if t-v < 1.]+[t]
                        self.rate, self.error = len(window), ''
                        path = self.out/'head.mp4'
                        if path.exists(): self.bytes = path.stat().st_size
                        if self.frames % self.fps == 0: index.flush()
                    except (BrokenPipeError, OSError):
                        raise
                    except Exception as exc:
                        self.error, self.rate = str(exc), 0.
                        self.stop.wait(.25)
        except Exception as exc:
            self.error, self.rate = str(exc), 0.
        finally:
            self.finish_encoder()
