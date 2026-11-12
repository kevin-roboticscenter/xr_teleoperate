#!/usr/bin/env python3
"""Select the laptop-to-H2 transport. Does not configure robot Wi-Fi or control motors."""
import argparse
import ipaddress
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

HOME_DIR = Path.home()
CONFIG = HOME_DIR / '.config/h2-teleop/link.json'
FORWARDER = HOME_DIR / 'h2-socat/laptop_h2_socat.sh'
PORTS = tuple(int(p) for p in os.environ.get('H2_FORWARD_PORTS', '8012 8787').split())
BIND = os.environ.get('H2_FORWARD_BIND', '0.0.0.0')


def load_config():
    config = {'mode': 'ethernet', 'ethernet': '192.168.124.162', 'wifi': None}
    if CONFIG.exists():
        config.update(json.loads(CONFIG.read_text()))
    if config['mode'] not in ('ethernet', 'wifi'):
        raise ValueError('Invalid H2 connection mode')
    for key in ('ethernet', 'wifi'):
        if config[key] is not None:
            config[key] = str(ipaddress.IPv4Address(config[key]))
    return config


def selected_host(config=None):
    config = config or load_config()
    host = config[config['mode']]
    if not host:
        raise ValueError('Set the H2 Wi-Fi address first: h2-link wifi --host H2_WIFI_IP')
    return host


def save_config(config):
    CONFIG.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    temporary = CONFIG.with_suffix('.tmp')
    temporary.write_text(json.dumps(config, indent=2) + '\n')
    temporary.chmod(0o600)
    temporary.replace(CONFIG)


def managed_processes():
    """Match only this user's forwarding script and socat listeners on our ports."""
    supervisors, relays = [], []
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            if entry.stat().st_uid != os.getuid():
                continue
            argv = [v.decode(errors='replace') for v in (entry / 'cmdline').read_bytes().split(b'\0') if v]
            if not argv:
                continue
            pid = int(entry.name)
            if Path(argv[0]).name in ('bash', 'sh') and len(argv) > 1 and Path(argv[1]) == FORWARDER:
                supervisors.append(pid)
            if Path(argv[0]).name.startswith('python') and len(argv) > 1 and Path(argv[1]) == HOME_DIR / 'h2-wuji-teleop/wuji_glove/dashboard_gateway.py':
                relays.append(pid)
            if Path(argv[0]).name == 'socat' and len(argv) > 1 and any(
                    argv[1].startswith(f'TCP-LISTEN:{port},bind={BIND},') for port in PORTS):
                relays.append(pid)
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
    return supervisors, relays


def stop_forwarders():
    for phase in (0, 1):
        pids = managed_processes()[phase]
        for pid in pids:
            try:
                os.kill(pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            alive = []
            for pid in pids:
                try:
                    # A zombie no longer owns any sockets or supervises children.
                    state = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()[0]
                    if state != 'Z': alive.append(pid)
                except FileNotFoundError:
                    pass
            if not alive:
                break
            time.sleep(0.05)
        else:
            raise RuntimeError(f'Forwarding processes did not stop: {alive}')


def listeners_ready():
    result = subprocess.run(['ss', '-H', '-ltn'], capture_output=True, text=True, check=True)
    addresses = {line.split()[3] for line in result.stdout.splitlines()}
    return all(f'{BIND}:{port}' in addresses for port in PORTS)


def restart_forwarders():
    if not FORWARDER.is_file():
        raise RuntimeError(f'Missing {FORWARDER}')
    stop_forwarders()
    subprocess.Popen(['/bin/bash', str(FORWARDER)], stdin=subprocess.DEVNULL,
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        if listeners_ready():
            return
        time.sleep(0.1)
    raise RuntimeError('Forwarders did not start; inspect ~/h2-socat/socat.log')


def run_network(args):
    result = subprocess.run(args, capture_output=True, text=True, timeout=8)
    if result.returncode:
        raise RuntimeError(result.stderr.strip() or 'Network configuration command failed')
    return result.stdout.strip()


def wifi_attachment(config):
    devices = json.loads(run_network(['ip', '-j', '-4', 'addr', 'show', 'up']))
    for device in devices:
        interface = device['ifname']
        if not (Path('/sys/class/net') / interface / 'wireless').exists():
            continue
        profile = run_network(['nmcli', '-g', 'GENERAL.CONNECTION', 'device', 'show', interface])
        if profile != config['wifi_connection']:
            continue
        for address in device['addr_info']:
            if address.get('scope') == 'global':
                subnet = ipaddress.IPv4Interface(f'{address["local"]}/{address["prefixlen"]}').network
                return interface, profile, subnet
    raise RuntimeError(f'Connect this laptop to {config["wifi_connection"]} before selecting H2 Wi-Fi')


def prepare_wifi_route(config):
    """Restore the verified PC1-to-PC2 route, resolving H2's current DHCP address."""
    if not config.get('wifi_gateway_hostname'):
        return False  # Direct Wi-Fi addresses and external bridges use normal LAN routing.
    interface, profile, subnet = wifi_attachment(config)
    if str(subnet) != config['wifi_network']:
        raise RuntimeError('The Wi-Fi subnet changed; H2 needs a matching return route before reconnecting')
    hostname = config['wifi_gateway_hostname']
    result = run_network(['getent', 'ahostsv4', hostname])
    candidates = [ipaddress.IPv4Address(line.split()[0]) for line in result.splitlines() if line.strip()]
    gateway = next((str(address) for address in candidates if address in subnet), None)
    if gateway is None:
        raise RuntimeError(f'{hostname} has no address on {subnet}; check that H2 has joined the Wi-Fi')
    # Confirm the same robot radio before replacing its saved DHCP address.
    subprocess.run(['ping', '-c', '1', '-W', '1', '-I', interface, gateway],
                   capture_output=True, timeout=3)
    neighbors = json.loads(run_network(['ip', '-j', 'neigh', 'show', 'to', gateway, 'dev', interface]))
    expected_mac = config.get('wifi_gateway_mac', '').lower()
    observed_macs = {item['lladdr'].lower() for item in neighbors if item.get('lladdr')}
    if expected_mac and not observed_macs:
        raise RuntimeError(f'H2 is not responding on {profile} yet; wait for startup or reconnect it to Wi-Fi in Unitree Explore')
    if expected_mac and expected_mac not in observed_macs:
        raise RuntimeError('The discovered Wi-Fi device does not match this H2; the route was not changed')
    destination = selected_host(config)
    desired = f'{destination}/32 {gateway}'
    original = run_network(['nmcli', '-g', 'ipv4.routes', 'connection', 'show', profile])
    routes = json.loads(run_network(['ip', '-j', 'route', 'get', destination]))
    if desired in original and any(r.get('gateway') == gateway and r.get('dev') == interface for r in routes):
        config['wifi_gateway'] = gateway
        return False
    previous_gateway = config.get('wifi_gateway')
    previous_route = f'{destination}/32 {previous_gateway}' if previous_gateway else None
    try:
        if previous_route and previous_route in original and previous_route != desired:
            run_network(['nmcli', 'connection', 'modify', profile, '-ipv4.routes', previous_route])
        if desired not in original:
            run_network(['nmcli', 'connection', 'modify', profile, '+ipv4.routes', desired])
        run_network(['nmcli', 'device', 'reapply', interface])
        routes = json.loads(run_network(['ip', '-j', 'route', 'get', destination]))
        if not any(r.get('gateway') == gateway and r.get('dev') == interface for r in routes):
            raise RuntimeError('The H2 route did not select the Wi-Fi interface')
    except Exception:
        run_network(['nmcli', 'connection', 'modify', profile, 'ipv4.routes', original])
        run_network(['nmcli', 'device', 'reapply', interface])
        raise
    config['wifi_gateway'] = gateway
    print(f'H2 Wi-Fi route restored through {gateway} on {profile}')
    return True


def switch(mode, host=None):
    previous = load_config()
    config = dict(previous, mode=mode)
    if host:
        config[mode] = str(ipaddress.IPv4Address(host))
    selected_host(config)
    route_changed = prepare_wifi_route(config) if mode == 'wifi' else False
    save_config(config)
    supervisors, relays = managed_processes()
    if previous['mode'] == mode and selected_host(previous) == selected_host(config) and not route_changed and supervisors and relays and listeners_ready():
        print(f'Already using {mode}: unitree@{selected_host(config)}. Existing connections kept open.')
        return
    try:
        restart_forwarders()
    except Exception:
        save_config(previous)
        try:
            restart_forwarders()
        except Exception:
            pass
        raise
    print(f'Selected {mode}: unitree@{selected_host(config)}')
    print('Forwarding restarted. Reload Quest after switching; run h2-wifi to print/update page links.')
    print('This selects a route; it does not join H2 to Wi-Fi or test robot reachability.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='action')
    for mode in ('ethernet', 'wifi'):
        command = sub.add_parser(mode, help=f'select {mode}; restarts page/WebSocket connections')
        command.add_argument('--host', type=ipaddress.IPv4Address, help='H2 address for this connection; saved for later')
    sub.add_parser('status', help='show selected route and local forwarding processes')
    sub.add_parser('host', help='print selected H2 address for scripts/SSH/SCP')
    args = parser.parse_args()
    try:
        if args.action in ('ethernet', 'wifi'):
            switch(args.action, args.host)
        elif args.action == 'host':
            print(selected_host())
        else:
            config = load_config()
            supervisors, relays = managed_processes()
            print(f'Connection: {config["mode"]}\nH2 address: {selected_host(config)}')
            print(f'Ethernet profile: {config["ethernet"]}\nWi-Fi profile: {config["wifi"] or "not configured"}')
            if config.get('wifi_gateway'):
                print(f'Wi-Fi network: {config["wifi_connection"]}\nH2 Wi-Fi gateway: {config["wifi_gateway"]} ({config["wifi_gateway_hostname"]})')
            print(f'Local forwarding: {len(supervisors)} supervisor(s), {len(relays)} relay process(es)')
            print('Status describes local configuration, not a live robot connection test.')
    except (OSError, ValueError, RuntimeError, subprocess.CalledProcessError) as exc:
        print(f'H2 connection: {exc}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
