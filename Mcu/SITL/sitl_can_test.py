#!/usr/bin/env python3
'''
test AM32 SITL over DroneCAN multicast UDP

runs a throttle ramp via esc.RawCommand and reports esc.Status
telemetry. Needs the SITL binary already running on the same mcast bus,
eg:
  obj/AM32_AM32_SITL_CAN_*.elf --node-id 10 --verbose
  python3 Mcu/SITL/sitl_can_test.py --throttle 0.5 --duration 20
'''

import argparse
import time

import dronecan

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--uri', default='mcast:0')
parser.add_argument('--node-id', type=int, default=100, help='our node ID')
parser.add_argument('--esc-index', type=int, default=0)
parser.add_argument('--throttle', type=float, default=0.5, help='peak throttle 0..1')
parser.add_argument('--ramp-time', type=float, default=3.0, help='seconds to reach peak')
parser.add_argument('--duration', type=float, default=15.0)
parser.add_argument('--rate', type=float, default=50.0, help='RawCommand rate Hz')
args = parser.parse_args()


def main():
    node = dronecan.make_node(args.uri, node_id=args.node_id, bitrate=1000000)

    state = {'nodes': set(), 'status': None, 'nstatus': 0}

    def on_node_status(e):
        nid = e.transfer.source_node_id
        if nid not in state['nodes']:
            state['nodes'].add(nid)
            print('found node %u (mode %u health %u)' % (
                nid, e.message.mode, e.message.health))

    def on_esc_status(e):
        state['status'] = e.message
        state['nstatus'] += 1

    node.add_handler(dronecan.uavcan.protocol.NodeStatus, on_node_status)
    node.add_handler(dronecan.uavcan.equipment.esc.Status, on_esc_status)

    # wait for the ESC to appear
    deadline = time.time() + 10
    while not state['nodes'] and time.time() < deadline:
        node.spin(0.1)
    if not state['nodes']:
        print('FAIL: no DroneCAN node seen')
        return 1

    print('arming and ramping to %.0f%% throttle' % (args.throttle * 100))
    t0 = time.time()
    next_cmd = t0
    last_print = t0
    while time.time() - t0 < args.duration:
        node.spin(0)
        now = time.time()
        if now >= next_cmd:
            next_cmd += 1.0 / args.rate
            # arming plus raw command
            arming = dronecan.uavcan.equipment.safety.ArmingStatus(
                status=dronecan.uavcan.equipment.safety.ArmingStatus().STATUS_FULLY_ARMED)
            node.broadcast(arming)
            ramp = min(1.0, (now - t0) / args.ramp_time)
            value = int(8191 * args.throttle * ramp)
            commands = [0] * (args.esc_index + 1)
            commands[args.esc_index] = value
            node.broadcast(dronecan.uavcan.equipment.esc.RawCommand(cmd=commands))
        if now - last_print >= 1.0 and state['status'] is not None:
            last_print = now
            s = state['status']
            print('rpm=%6d volts=%.2f amps=%.2f temp=%.0fC err=%u (%u pkts)' % (
                s.rpm, s.voltage, s.current,
                s.temperature - 273.15, s.error_count, state['nstatus']))
        time.sleep(0.001)

    # stop the motor
    for _ in range(10):
        node.broadcast(dronecan.uavcan.equipment.esc.RawCommand(cmd=[0]))
        node.spin(0.02)

    if state['nstatus'] == 0:
        print('FAIL: no esc.Status telemetry received')
        return 1
    s = state['status']
    print('final: rpm=%d volts=%.2f amps=%.2f' % (s.rpm, s.voltage, s.current))
    return 0


if __name__ == '__main__':
    exit(main())
