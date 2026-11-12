#!/usr/bin/env python3
"""Run the official browser teleop with optional H2 head following hooks."""
from pathlib import Path
import runpy
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def main():
    from quest_h2_teleop import UPSTREAM, environment
    from head_camera.head_follow import settings
    sys.path[:0] = environment()['PYTHONPATH'].split(':')
    restore = None
    if settings(ROOT).get('enabled', False):
        import televuer
        import televuer.tv_wrapper as tv_module
        import teleop.robot_control.robot_arm as arm_module
        import teleop.utils.ipc as ipc_module
        from head_camera.browser_head import BrowserHeadBridge, install_hooks
        restore = install_hooks(BrowserHeadBridge(), arm_module, ipc_module, tv_module, televuer)
    try:
        runpy.run_path(str(UPSTREAM/'teleop/teleop_hand_and_arm.py'), run_name='__main__')
    finally:
        if restore is not None:
            restore()


if __name__ == '__main__':
    main()
