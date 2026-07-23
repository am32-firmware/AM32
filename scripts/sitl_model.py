#!/usr/bin/env python3
'''
live-reload the motor/battery/esc model json of a running AM32 SITL

sends LOAD_MODEL on the SITL state port (default 57734) and prints the
reply. Only motor/battery/esc sections apply at runtime; sim.* keys need
a restart. Reload while the motor is stopped: a mid-spin model swap
behaves like swapping the motor while it spins and can desync
'''

import argparse
import os
import socket
import struct

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('json_path')
parser.add_argument('--port', type=int, default=57734)
parser.add_argument('--host', default='127.0.0.1')
args = parser.parse_args()

path = os.path.abspath(args.json_path)
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.settimeout(2.0)
s.sendto(struct.pack('<HBB', 0x5353, 1, 0) + path.encode() + b'\0',
         (args.host, args.port))
try:
    reply = s.recv(300)
    magic, ok = struct.unpack('<HB', reply[:3])
    print('%s: %s' % ('ok' if ok else 'FAILED', reply[4:].split(b'\0')[0].decode()))
    raise SystemExit(0 if ok else 1)
except socket.timeout:
    raise SystemExit('no reply from SITL state port %s:%u' % (args.host, args.port))
