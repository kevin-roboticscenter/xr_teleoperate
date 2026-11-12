#!/usr/bin/env python3
"""Prepare this isolated H2 integration; never connects to or commands a robot."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parent


def git(repo, *args, check=True):
    return subprocess.run(['git', '-C', str(repo), *args], check=check,
                          capture_output=True, text=True)


def prepare(check_only=False):
    manifest = json.loads((ROOT / 'sources.json').read_text())
    runtime = ROOT / 'upstream/dex5-pr321'
    if not check_only:
        git(ROOT.parents[1], 'submodule', 'update', '--init', '--recursive', '--',
            'integrations/h2/upstream/dex5-pr321',
            'integrations/h2/upstream/teleimager-head-camera')
    if git(runtime, 'rev-parse', 'HEAD').stdout.strip() != manifest['runtime_commit']:
        raise RuntimeError('H2 runtime commit differs from the tested PR321 pin')
    head = ROOT / 'upstream/teleimager-head-camera'
    if git(head, 'rev-parse', 'HEAD').stdout.strip() != manifest['head_camera_commit']:
        raise RuntimeError('Head-camera TeleImager commit differs from the pinned version')
    for entry in manifest['patches']:
        repo, patch = ROOT / entry['repository'], ROOT / entry['patch']
        if hashlib.sha256(patch.read_bytes()).hexdigest() != entry['sha256']:
            raise RuntimeError(f'Patch checksum mismatch: {patch.name}')
        if git(repo, 'rev-parse', 'HEAD').stdout.strip() != entry['commit']:
            raise RuntimeError(f'Unexpected dependency commit: {entry["repository"]}')
        applied = git(repo, 'apply', '--reverse', '--check', str(patch), check=False)
        if applied.returncode == 0:
            print(f'{patch.name}: already applied')
            continue
        if check_only:
            raise RuntimeError(f'{patch.name}: not applied; run bootstrap.py first')
        applicable = git(repo, 'apply', '--check', str(patch), check=False)
        if applicable.returncode:
            raise RuntimeError(f'{patch.name}: overlapping local changes; nothing overwritten\n{applicable.stderr}')
        git(repo, 'apply', str(patch))
        print(f'{patch.name}: applied')
    for name, expected in manifest['protected'].items():
        if hashlib.sha256((runtime / name).read_bytes()).hexdigest() != expected:
            raise RuntimeError(f'Protected Unitree arm/IK source changed: {name}')
    sdk = ROOT / 'upstream/unitree-h2-head-sdk'
    sdk_source = json.loads((sdk / 'SOURCE.json').read_text())
    if hashlib.sha256((sdk / 'h2_arm_sdk_dds_example.py').read_bytes()).hexdigest() != sdk_source['sha256']:
        raise RuntimeError('Official head SDK helper checksum mismatch')
    if not check_only:
        for example in (ROOT / 'config').glob('*.example.json'):
            target = example.with_name(example.name.replace('.example.json', '.json'))
            if not target.exists():
                shutil.copy2(example, target)
        model = ROOT / 'wuji_glove/models/Dex5-left-linked.urdf'
        if not model.exists():
            subprocess.run([sys.executable, str(ROOT / 'wuji_glove/build_left_profile.py')], check=True)
    print('H2 sources verified. No robot connection or motor command was made.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='Verify without changing files')
    options = parser.parse_args()
    try:
        prepare(options.check)
    except (OSError, ValueError, RuntimeError, subprocess.CalledProcessError) as error:
        raise SystemExit(str(error))
