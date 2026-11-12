#!/usr/bin/env python3
"""Run the official browser teleop with optional H2 head following hooks."""
from pathlib import Path
import runpy
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def install_browser_hooks():
    import televuer
    import televuer.tv_wrapper as tv_module
    import teleop.robot_control.robot_arm as arm_module
    import teleop.utils.ipc as ipc_module
    from head_camera.browser_head import BrowserHeadBridge, install_hooks
    return install_hooks(BrowserHeadBridge(), arm_module, ipc_module, tv_module, televuer)


def run_official(entrypoint, follow_head):
    if not follow_head:
        return runpy.run_path(str(entrypoint), run_name='__main__')
    import logging_mp
    configure = logging_mp.basicConfig
    restore = None

    def configure_then_install(*args, **kwargs):
        nonlocal restore
        # The official entry point owns logging setup. Hook imports create
        # loggers, so install them only after that first configuration call.
        logging_mp.basicConfig = configure
        result = configure(*args, **kwargs)
        restore = install_browser_hooks()
        return result

    logging_mp.basicConfig = configure_then_install
    try:
        return runpy.run_path(str(entrypoint), run_name='__main__')
    finally:
        logging_mp.basicConfig = configure
        if restore is not None:
            restore()


def main():
    from quest_h2_teleop import UPSTREAM, environment
    from head_camera.head_follow import settings
    sys.path[:0] = environment()['PYTHONPATH'].split(':')
    run_official(UPSTREAM/'teleop/teleop_hand_and_arm.py', settings(ROOT).get('enabled', False))


if __name__ == '__main__':
    main()
