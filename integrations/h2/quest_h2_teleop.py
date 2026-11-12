#!/usr/bin/env python3
"""Launch the PR #321 H2/Dex5 controller with local launch-time hand settings."""
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent
UPSTREAM = ROOT / 'upstream/dex5-pr321'
GAIN_SETTINGS_PATH = ROOT / 'config/dex5_gains.json'
PICO_HAND_SOURCE = 'pico_tracker_hands'
NATIVE_SOURCES = ('pico_wuji', 'pico_dual_wuji', 'pico_head', PICO_HAND_SOURCE)
DUAL_TRACKER_SOURCES = ('pico_dual_wuji', PICO_HAND_SOURCE)
sys.path.insert(0, str(UPSTREAM))
from teleop.utils.dex5_gains import GainStore

def display_mode():
    """Select an upstream display option without altering tracking or control."""
    mode = os.environ.get('H2_XR_DISPLAY_MODE', 'pass-through')
    if mode not in ('immersive', 'ego', 'pass-through'):
        raise ValueError('H2_XR_DISPLAY_MODE must be immersive, ego, or pass-through')
    return mode

def environment(gain_profile=None):
    env = os.environ.copy()
    env['PYTHONPATH'] = os.pathsep.join(str(p) for p in (
        UPSTREAM, UPSTREAM / 'teleop/televuer/src',
        UPSTREAM / 'teleop/teleimager/src',
        UPSTREAM / 'teleop/robot_control/dex-retargeting/src'))
    env.setdefault('XR_TELEOP_CERT', str(ROOT / 'certs/cert.pem'))
    env.setdefault('XR_TELEOP_KEY', str(ROOT / 'certs/key.pem'))
    env['PYTHONUNBUFFERED'] = '1'
    profile = gain_profile if gain_profile is not None else GainStore(GAIN_SETTINGS_PATH).load()
    env.update(profile.environment())
    return env

def command(hand_source='vr'):
    if hand_source not in ('vr', 'wuji', *NATIVE_SOURCES):
        raise ValueError('Unknown hand source')
    if hand_source == 'pico_head':
        return [sys.executable, str(ROOT / 'head_camera/head_only.py')]
    if hand_source in ('pico_wuji', 'pico_dual_wuji', PICO_HAND_SOURCE):
        return [sys.executable, str(ROOT / 'pico_tracker/arm_teleop.py'),
                *(['--both-arms'] if hand_source in DUAL_TRACKER_SOURCES else []),
                *(['--pico-hands'] if hand_source == PICO_HAND_SOURCE else [])]
    return [sys.executable, str(ROOT / 'head_camera/browser_teleop.py'),
            '--arm', 'H2', *(['--ee', 'dex5'] if hand_source == 'vr' else []), '--input-mode', 'hand',
            '--display-mode', display_mode(), '--motion', '--headless', '--ipc',
            '--img-server-ip', '127.0.0.1', '--network-interface',
            os.environ.get('H2_DDS_INTERFACE', 'eth0')]

if __name__ == '__main__':
    os.chdir(UPSTREAM / 'teleop')
    os.execve(sys.executable, command() + sys.argv[1:], environment())
