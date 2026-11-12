#!/usr/bin/env python3
"""Relay SDK JSONL over an authenticated SSH session to the dashboard's local socket.

This relay creates no DDS channels and sends no motor commands.
"""
from pathlib import Path
import argparse
import socket
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--side', choices=('left', 'right'), default='left')
    args = parser.parse_args()
    path = Path(__file__).resolve().parent / ('input.sock' if args.side == 'left' else 'input_right.sock')
    with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as connection:
        connection.connect(str(path))
        for line in sys.stdin:
            if len(line) <= 65536:
                connection.send(line.encode())


if __name__ == '__main__':
    try:
        main()
    except (KeyboardInterrupt, BrokenPipeError):
        pass
