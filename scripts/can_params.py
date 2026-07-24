#!/usr/bin/env python3
'''
list, get, set and save DroneCAN parameters on an AM32 ESC

examples:
  can_params.py --node-id 124 list
  can_params.py --node-id 124 get MOTOR_KV
  can_params.py --node-id 124 set MOTOR_KV 1100
  can_params.py --node-id 124 set DEBUG_RATE 50 --save

streams zero-throttle RawCommand while working so an ESC held in the
bootloader by the no-signal gate boots into the app first
'''

import argparse
import time

import dronecan


class ParamClient(object):
    def __init__(self, args):
        self.args = args
        self.node = dronecan.make_node(args.uri, node_id=args.client_node_id)
        self.target = args.node_id
        self.next_tx = 0.0
        self.app_seen = False
        self.node.add_handler(dronecan.uavcan.equipment.esc.Status, self.on_esc_status)

    def on_esc_status(self, e):
        if e.transfer.source_node_id == self.target:
            self.app_seen = True

    def spin_for(self, duration):
        tend = time.monotonic() + duration
        while time.monotonic() < tend:
            now = time.monotonic()
            if self.args.keepalive and now >= self.next_tx:
                self.node.broadcast(dronecan.uavcan.equipment.esc.RawCommand(cmd=[0]))
                self.next_tx = now + 0.02
            try:
                self.node.spin(0.005)
            except Exception as ex:
                print('spin err:', ex)

    def wait_app(self, timeout=15):
        tstart = time.monotonic()
        while not self.app_seen:
            self.spin_for(0.2)
            if time.monotonic() - tstart > timeout:
                raise SystemExit('no esc.Status from node %u' % self.target)

    def get_set(self, name=None, index=None, value=None, retries=2):
        '''single GetSet transaction, returns (name, value) or None

        retries matter: a fresh client process reuses transfer id 0, which the
        target's canard can drop as a duplicate of the previous session'''
        for _ in range(retries + 1):
            r = self.get_set_once(name, index, value)
            if r is not None:
                return r
        return None

    def get_set_once(self, name=None, index=None, value=None):
        result = [None]
        def cb(e):
            if e is None:
                return
            rname = e.response.name.decode() if e.response.name else ''
            if not rname:
                result[0] = ('', None)
                return
            v = e.response.value
            kind = dronecan.get_active_union_field(v)
            val = getattr(v, kind) if kind != 'empty' else None
            if kind == 'string_value':
                val = bytes(val)
            result[0] = (rname, val)
        req = dronecan.uavcan.protocol.param.GetSet.Request()
        if name is not None:
            req.name = name
        if index is not None:
            req.index = index
        if value is not None:
            req.value = dronecan.uavcan.protocol.param.Value(integer_value=int(value))
        self.node.request(req, self.target, cb)
        tstart = time.monotonic()
        while result[0] is None and time.monotonic() - tstart < 2:
            self.spin_for(0.05)
        return result[0]

    def save(self):
        result = [None]
        def cb(e):
            result[0] = False if e is None else bool(e.response.ok)
        req = dronecan.uavcan.protocol.param.ExecuteOpcode.Request()
        req.opcode = req.OPCODE_SAVE
        self.node.request(req, self.target, cb)
        tstart = time.monotonic()
        while result[0] is None and time.monotonic() - tstart < 2:
            self.spin_for(0.05)
        return result[0]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--uri', default='mcast:0:lo')
    parser.add_argument('--node-id', type=int, required=True)
    parser.add_argument('--client-node-id', type=int, default=100)
    parser.add_argument('--no-keepalive', dest='keepalive', action='store_false',
                        help='do not stream zero-throttle RawCommand')
    sub = parser.add_subparsers(dest='cmd', required=True)
    sub.add_parser('list')
    p = sub.add_parser('get')
    p.add_argument('name')
    p = sub.add_parser('set')
    p.add_argument('name')
    p.add_argument('value', type=int)
    p.add_argument('--save', action='store_true', help='save to eeprom after set')
    args = parser.parse_args()

    c = ParamClient(args)
    c.wait_app()
    if args.cmd == 'list':
        idx = 0
        while True:
            r = c.get_set(index=idx)
            if r is None:
                raise SystemExit('timeout at index %u' % idx)
            if not r[0]:
                break
            print('%3u %-22s %s' % (idx, r[0], r[1]))
            idx += 1
    elif args.cmd == 'get':
        print(c.get_set(name=args.name))
    elif args.cmd == 'set':
        r = c.get_set(name=args.name, value=args.value)
        print('set:', r)
        if r and r[1] == args.value and args.save:
            print('save:', c.save())
        elif args.save:
            raise SystemExit('set failed, not saving')


if __name__ == '__main__':
    main()
