#!/usr/bin/env python3
"""Update Wi-Fi networking for the deployed H2 page without starting teleop."""
import argparse
import ipaddress
import json
from pathlib import Path
import shlex
import subprocess
import sys
import time
from h2_link import selected_host


# Executed on H2 using its existing private key; no controller commands are sent.
REMOTE = r'''
import ipaddress, json, os, re, shutil, socket, subprocess, sys, tempfile
from pathlib import Path

address = str(ipaddress.IPv4Address(sys.argv[1]))
check_only = sys.argv[2] == 'check'
certdir = Path.home() / 'h2-upstream-pr321/certs'
cert, key = certdir / 'cert.pem', certdir / 'key.pem'
if not cert.is_file() or not key.is_file():
    raise SystemExit('The deployed H2 certificate or private key is missing')

def openssl(*args):
    return subprocess.run(['openssl', *map(str, args)], capture_output=True, text=True)

previous = openssl('x509', '-in', cert, '-noout', '-ext', 'subjectAltName')
if previous.returncode:
    raise RuntimeError(previous.stderr)
addresses = {str(ipaddress.ip_address(v))
             for v in re.findall(r'IP Address:([^,\s]+)', previous.stdout)}
valid = openssl('x509', '-in', cert, '-noout', '-checkend', '86400')
needed = address not in addresses or valid.returncode != 0
if needed and not check_only:
    addresses.add(address)
    san = 'subjectAltName=DNS:localhost,' + ','.join('IP:' + v for v in sorted(addresses))
    fd, candidate = tempfile.mkstemp(prefix='.wifi-cert-', suffix='.pem', dir=certdir)
    os.close(fd)
    try:
        result = openssl('req', '-new', '-x509', '-key', key, '-out', candidate,
                         '-days', '365', '-sha256', '-subj', '/CN=Quest H2 teleop',
                         '-addext', san)
        if result.returncode:
            raise RuntimeError(result.stderr)
        shutil.copy2(cert, certdir / 'cert.pem.before-wifi')
        os.chmod(candidate, 0o644)
        os.replace(candidate, cert)
    finally:
        if os.path.exists(candidate):
            os.unlink(candidate)
try:
    with socket.create_connection(('127.0.0.1', 8012), timeout=1):
        vuer_running = True
except OSError:
    vuer_running = False
print(json.dumps({'certificate': ('needs update' if check_only else 'updated') if needed else 'current',
                  'vuer_running': vuer_running}))
'''


def wifi_address():
    devices = json.loads(subprocess.check_output(['ip', '-j', '-4', 'addr', 'show', 'up'], text=True))
    addresses = [a['local'] for d in devices
                 if (Path('/sys/class/net') / d['ifname'] / 'wireless').exists()
                 for a in d['addr_info'] if a.get('scope') == 'global']
    if len(addresses) != 1:
        raise RuntimeError('Connect to Wi-Fi, or select the laptop address with --ip ADDRESS')
    return str(ipaddress.IPv4Address(addresses[0]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ip', type=ipaddress.IPv4Address, help='Override automatic Wi-Fi address detection')
    parser.add_argument('--check', action='store_true', help='Show status without changing anything')
    parser.add_argument('--ssh-control-path', help='Reuse an existing SSH connection')
    parser.add_argument('--robot-host', type=ipaddress.IPv4Address,
                        help='Override H2 SSH address; defaults to the selected h2-link profile')
    args = parser.parse_args()
    address = str(args.ip) if args.ip else wifi_address()
    robot_host = str(args.robot_host) if args.robot_host else selected_host()
    print(f'Wi-Fi address: {address}', flush=True)
    print(f'H2 address: {robot_host}', flush=True)
    ssh = ['ssh', '-o', 'ConnectTimeout=5']
    if args.ssh_control_path:
        ssh += ['-o', 'ControlPath=' + args.ssh_control_path]
    remote_command = shlex.join(['python3', '-', address, 'check' if args.check else 'configure'])
    result = subprocess.run(ssh + [f'unitree@{robot_host}', remote_command],
                            input=REMOTE, text=True, stdout=subprocess.PIPE, check=True)
    status = json.loads(result.stdout)
    forwarder = Path.home() / 'h2-socat/laptop_h2_socat.sh'
    if not args.check:
        if not forwarder.is_file():
            raise RuntimeError(f'Forwarding launcher is missing: {forwarder}')
        subprocess.Popen(['/bin/bash', str(forwarder)], stdin=subprocess.DEVNULL,
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                         start_new_session=True)
    deadline = time.monotonic() + (0 if args.check else 3)
    while True:
        listeners = subprocess.check_output(['ss', '-H', '-ltn'], text=True)
        bound = {line.split()[3] for line in listeners.splitlines()}
        forwarding = {'0.0.0.0:8012', '0.0.0.0:8787'} <= bound
        if forwarding or time.monotonic() >= deadline:
            break
        time.sleep(.1)
    print('Forwarding: ' + ('active on all laptop IPv4 addresses' if forwarding else 'not fully active'))
    if not forwarding and not args.check:
        raise RuntimeError('Forwarding failed; check ~/h2-socat/socat.log')
    print('H2 certificate: ' + status['certificate'])
    if status['certificate'] == 'updated':
        if status['vuer_running']:
            print('Vuer is already running: end teleop and relaunch from the control page to load the new certificate.')
        else:
            print('The updated certificate will load at the next teleop launch.')
        print('Quest may ask you to accept the updated local certificate when opening Vuer.')
    print(f'Control page: http://{address}:8787/h2-teleop')
    print(f'Quest: https://{address}:8012/?ws=wss://{address}:8012')


if __name__ == '__main__':
    try:
        main()
    except (OSError, RuntimeError, ValueError, subprocess.CalledProcessError) as exc:
        sys.exit(f'Wi-Fi configuration failed: {exc}')
