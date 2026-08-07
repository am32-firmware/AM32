#!/usr/bin/env python3
'''
passive DroneCAN bus monitor for AM32 ESC work

anonymous node (sends nothing by default): shows per-node NodeStatus
(uptime/mode, reboot and bootloader detection), esc.Status telemetry,
decoded AM32 FlexDebug (debug1) and LogMessage broadcasts, plus message
rate counts on exit

an AM32 CAN bootloader stuck at "no signal" shows as mode=MAINTENANCE
with LogMessage "no signal"; stream RawCommand (e.g. esc_measure.py) to
boot it into the app
'''

import argparse
import collections
import struct
import time

import dronecan

from am32_debug import FLEXDEBUG_AM32_DEBUG1, decode_debug1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--uri', default='mcast:0:lo')
    parser.add_argument('--node-id', type=int, default=None, help='only show this node')
    parser.add_argument('--duration', type=float, default=0, help='exit after N seconds (0=forever)')
    parser.add_argument('--quiet', action='store_true', help='only rates and events, no telemetry rows')
    args = parser.parse_args()

    node = dronecan.make_node(args.uri)  # anonymous
    counts = collections.Counter()
    last_uptime = {}

    def want(e):
        return args.node_id is None or e.transfer.source_node_id == args.node_id

    def on_node_status(e):
        if not want(e):
            return
        nid = e.transfer.source_node_id
        m = e.message
        counts['NodeStatus:%u' % nid] += 1
        prev = last_uptime.get(nid)
        if prev is not None and m.uptime_sec < prev:
            print('node %u REBOOTED (uptime %u -> %u)' % (nid, prev, m.uptime_sec))
        if prev is None or m.mode != last_uptime.get('mode%u' % nid):
            mode = {0: 'OPERATIONAL', 1: 'INITIALIZATION', 2: 'MAINTENANCE',
                    3: 'SOFTWARE_UPDATE', 7: 'OFFLINE'}.get(m.mode, str(m.mode))
            print('node %u mode=%s vssc=%u uptime=%u' % (nid, mode, m.vendor_specific_status_code, m.uptime_sec))
        last_uptime[nid] = m.uptime_sec
        last_uptime['mode%u' % nid] = m.mode

    def on_esc_status(e):
        if not want(e):
            return
        nid = e.transfer.source_node_id
        counts['esc.Status:%u' % nid] += 1
        if not args.quiet:
            m = e.message
            print('esc %u: rpm=%u %.2fV %.2fA %.0fC err=%u' %
                  (nid, m.rpm, m.voltage, m.current, m.temperature - 273.15, m.error_count))

    def on_flexdebug(e):
        if not want(e):
            return
        nid = e.transfer.source_node_id
        m = e.message
        data = bytes(m.u8)
        counts['FlexDebug%u:%u' % (m.id, nid)] += 1
        if m.id == FLEXDEBUG_AM32_DEBUG1 and not args.quiet:
            row = decode_debug1(data)
            if row:
                print('debug1 %u: %s' % (nid, ' '.join('%s=%s' % kv for kv in row.items())))

    def on_log(e):
        if not want(e):
            return
        m = e.message
        print('log %u: [%s] %s' % (e.transfer.source_node_id,
                                   bytes(m.source).decode(errors='replace'),
                                   bytes(m.text).decode(errors='replace')))

    node.add_handler(dronecan.uavcan.protocol.NodeStatus, on_node_status)
    node.add_handler(dronecan.uavcan.equipment.esc.Status, on_esc_status)
    node.add_handler(dronecan.dronecan.protocol.FlexDebug, on_flexdebug)
    node.add_handler(dronecan.uavcan.protocol.debug.LogMessage, on_log)

    t0 = time.monotonic()
    try:
        while args.duration <= 0 or time.monotonic() - t0 < args.duration:
            try:
                node.spin(0.1)
            except Exception as ex:
                print('spin err:', ex)
    except KeyboardInterrupt:
        pass
    dur = time.monotonic() - t0
    print('\nmessage rates over %.1fs:' % dur)
    for k, v in sorted(counts.items()):
        print('  %-24s %6u (%.1f/s)' % (k, v, v / dur))


if __name__ == '__main__':
    main()
