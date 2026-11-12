#!/usr/bin/env python3
"""Show the robot camera in the official TeleVuer, without starting motor control."""
import os
import signal
import sys
import threading
from quest_h2_teleop import display_mode, environment

env = environment()
os.environ.update(env)
sys.path[:0] = env['PYTHONPATH'].split(os.pathsep)
from teleimager.image_client import ImageClient
from televuer.televuer import TeleVuer


def main():
    stop = threading.Event()
    client = ImageClient(host='127.0.0.1', request_bgr=True)
    camera = client.get_cam_config()['head_camera']
    viewer = TeleVuer(use_hand_tracking=True, binocular=camera['binocular'],
                      img_shape=camera['image_shape'], display_mode=display_mode(), zmq=True)
    # Install handlers after the Vuer child forks, so it retains normal SIGTERM behavior.
    for sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, lambda *_: stop.set())
    try:
        while not stop.wait(1 / 30):
            frame = client.get_head_frame()
            if frame.bgr is not None:
                viewer.render_to_xr(frame.bgr)
    finally:
        viewer.close()
        client.close()


if __name__ == '__main__':
    main()
