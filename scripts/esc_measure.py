#!/usr/bin/env python3
'''
measure ESC/motor behaviour over DroneCAN for SITL model calibration

drives an AM32 ESC (real or SITL) with esc.RawCommand profiles while
logging esc.Status, FlexDebug (debug1) and NodeStatus to a JSONL file,
and prints steady-state summaries per throttle level

works against real hardware (e.g. mcast:0:lo via a CAN bridge) and the
SITL binary identically, so the same profiles can be compared
'''

import argparse
import json
import struct
import sys
import time

import dronecan

import sim_clock
from am32_debug import FLEXDEBUG_AM32_DEBUG1, decode_debug1


class Recorder(object):
    '''log timestamped rows to JSONL and keep them for summaries.
    Serialisation and file writes happen on a separate thread so
    filesystem latency never delays the receive/command loop'''
    def __init__(self, path, clock=None):
        # timestamps come from the measurement clock: wall time for real
        # hardware, simulation time when driving SITL, so logged
        # durations are physical seconds in both cases
        self.clock = clock or sim_clock.WallClock()
        self.t0 = self.clock.now()
        self.rows = []
        self.q = None
        self.write_error = None
        if path:
            import queue
            import threading
            # open synchronously so a bad path fails the run immediately
            self.fh = open(path, 'w')
            self.q = queue.Queue()
            self.thread = threading.Thread(target=self._writer, daemon=True)
            self.thread.start()

    def _writer(self):
        while True:
            row = self.q.get()
            if row is None:
                self.fh.close()
                return
            try:
                self.fh.write(json.dumps(row) + '\n')
            except Exception as ex:
                if self.write_error is None:
                    self.write_error = ex
                    print('log write failed: %s' % ex)

    def add(self, rtype, **fields):
        row = {'t': round(self.clock.now() - self.t0, 4), 'type': rtype}
        row.update(fields)
        self.rows.append(row)
        if self.q:
            self.q.put(row)

    def close(self):
        if self.q:
            self.q.put(None)
            self.thread.join(timeout=10)
            if self.thread.is_alive():
                print('warning: log writer did not drain in time')
            if self.write_error:
                raise SystemExit('log writing failed: %s' % self.write_error)


class EscMeasure(object):
    def __init__(self, args):
        self.args = args
        self.node = dronecan.make_node(args.uri, node_id=args.client_node_id)
        self.target = args.node_id
        self.clock = sim_clock.make_clock(getattr(args, 'sim_state', None))
        self.rec = Recorder(args.log, clock=self.clock)
        self.throttle = 0.0
        self.next_tx = 0.0
        self.aborted = None
        self.last_esc_status = None
        self.node.add_handler(dronecan.uavcan.equipment.esc.Status, self.on_esc_status)
        self.node.add_handler(dronecan.uavcan.protocol.NodeStatus, self.on_node_status)
        self.node.add_handler(dronecan.dronecan.protocol.FlexDebug, self.on_flexdebug)

    def on_esc_status(self, e):
        if e.transfer.source_node_id != self.target:
            return
        m = e.message
        self.last_esc_status = m
        self.rec.add('status', rpm=m.rpm, volt=round(m.voltage, 3),
                     curr=round(m.current, 3), temp=round(m.temperature - 273.15, 1),
                     err=m.error_count)
        if m.current > self.args.max_current:
            self.abort('current %.1fA > %.1fA limit' % (m.current, self.args.max_current))
        if m.temperature - 273.15 > self.args.max_temp:
            self.abort('temperature %.0fC > %.0fC limit' % (m.temperature - 273.15, self.args.max_temp))

    def on_node_status(self, e):
        if e.transfer.source_node_id != self.target:
            return
        m = e.message
        self.rec.add('node', uptime=m.uptime_sec, mode=m.mode, vssc=m.vendor_specific_status_code)

    def on_flexdebug(self, e):
        if e.transfer.source_node_id != self.target:
            return
        m = e.message
        data = bytes(m.u8)
        if m.id != FLEXDEBUG_AM32_DEBUG1:
            return
        row = decode_debug1(data)
        if row:
            self.rec.add('debug1', **row)

    def abort(self, reason):
        if self.aborted is None:
            self.aborted = reason
            print('ABORT: %s' % reason)

    def set_throttle(self, throttle):
        throttle = max(0.0, min(self.args.max_throttle, throttle))
        if throttle != self.throttle:
            self.rec.add('cmd', throttle=round(throttle, 4))
        self.throttle = throttle

    def spin_for(self, duration):
        '''run the RawCommand stream and message pump for duration
        seconds of measurement time (simulated seconds under
        --sim-state, wall seconds otherwise). Absolute deadlines keep
        the command schedule from drifting with clock granularity'''
        tend = self.clock.now() + duration
        period = 1.0 / self.args.rate
        while self.clock.now() < tend:
            if self.aborted:
                self.throttle = 0.0
            now = self.clock.now()
            if now >= self.next_tx:
                cmd = [0] * self.args.esc_index + [int(self.throttle * 8191)]
                self.node.broadcast(dronecan.uavcan.equipment.esc.RawCommand(cmd=cmd))
                # absolute schedule, but never let a stalled clock build
                # a burst backlog
                self.next_tx = max(now, self.next_tx + period)
            try:
                # the message pump always waits in wall time so sockets
                # keep being serviced whatever the sim ratio
                self.node.spin(min(0.005, period / 2))
            except Exception as ex:
                self.rec.add('error', msg=str(ex))
            if self.aborted:
                raise SystemExit('aborted: %s' % self.aborted)

    def set_param(self, name, value):
        # retried: a fresh client process reuses transfer id 0, which the
        # target's canard can drop as a duplicate of the previous session
        for _ in range(3):
            result = [None]
            def cb(e):
                if e is not None:
                    v = e.response.value
                    kind = dronecan.get_active_union_field(v)
                    result[0] = getattr(v, kind)
            self.node.request(dronecan.uavcan.protocol.param.GetSet.Request(
                name=name, value=dronecan.uavcan.protocol.param.Value(integer_value=value)),
                self.target, cb)
            self.spin_for(0.5)
            if result[0] is not None:
                return result[0]
        return None

    def wait_ready(self):
        '''stream zero throttle until esc.Status seen (boots through bootloader gate)'''
        print('waiting for ESC app (node %u)' % self.target)
        # clock-paced: the ESC boot takes the same simulated time however
        # starved the host is, so a wall timeout here breaks on slow CI
        # runners. A dead sim cannot hang this wait - the sim clock
        # aborts on its own when the state stream stalls
        tstart = self.clock.now()
        while self.last_esc_status is None:
            self.spin_for(0.2)
            if self.clock.now() - tstart > self.args.ready_timeout:
                raise SystemExit('no esc.Status from node %u' % self.target)
        print('ESC ready: %.2fV %.0fC' % (self.last_esc_status.voltage,
                                          self.last_esc_status.temperature - 273.15))
        # keep streaming zero throttle so the ESC's zero-throttle arming
        # latches before the profile applies throttle
        self.spin_for(self.args.arm_time)
        # profile starts: from here an ESC reboot invalidates the run
        # (earlier no-input reboots are absorbed by the sim clock)
        self.clock.arm()

    def mark(self, label):
        self.rec.add('mark', label=label)
        print('== %s' % label)

    def summary(self):
        '''per steady segment: stats over the last half of each cmd level hold'''
        segs = []
        cur = None
        for row in self.rec.rows:
            if row['type'] == 'cmd':
                if cur:
                    segs.append(cur)
                cur = {'throttle': row['throttle'], 't0': row['t'], 't1': row['t'], 'rows': []}
            elif row['type'] == 'status' and cur:
                cur['rows'].append(row)
                cur['t1'] = row['t']
        if cur:
            segs.append(cur)
        print('\n%8s %9s %9s %8s %8s %7s' % ('throttle', 'rpm', 'rpm_sd', 'volt', 'curr', 'n'))
        out = []
        for s in segs:
            tmid = (s['t0'] + s['t1']) / 2
            rows = [r for r in s['rows'] if r['t'] >= tmid]
            if len(rows) < 2:
                continue
            n = len(rows)
            mean = lambda k: sum(r[k] for r in rows) / n
            rpm_m = mean('rpm')
            rpm_sd = (sum((r['rpm'] - rpm_m) ** 2 for r in rows) / n) ** 0.5
            print('%8.3f %9.1f %9.1f %8.3f %8.3f %7u' %
                  (s['throttle'], rpm_m, rpm_sd, mean('volt'), mean('curr'), n))
            out.append({'throttle': s['throttle'], 'rpm': rpm_m, 'rpm_sd': rpm_sd,
                        'volt': mean('volt'), 'curr': mean('curr'), 'n': n})
        return out


def profile_sweep(m, args):
    '''staircase up then down, hold each level'''
    levels = [float(x) for x in args.levels.split(',')]
    m.wait_ready()
    m.spin_for(1.0)
    for lv in levels + list(reversed(levels[:-1])):
        m.mark('sweep %.3f' % lv)
        m.set_throttle(lv)
        m.spin_for(args.hold)
    m.set_throttle(0)
    m.spin_for(2.0)


def profile_step(m, args):
    '''step from zero to each level (spin-up transient) then cut to zero (spin-down)'''
    levels = [float(x) for x in args.levels.split(',')]
    m.wait_ready()
    for lv in levels:
        m.mark('step to %.3f' % lv)
        m.set_throttle(0)
        m.spin_for(2.0)
        m.set_throttle(lv)
        m.spin_for(args.hold)
        m.mark('cut from %.3f' % lv)
        m.set_throttle(0)
        m.spin_for(args.coast)


def profile_cycles(m, args):
    '''repeated steps between two throttle levels for ensemble-averaged transients'''
    levels = [float(x) for x in args.levels.split(',')]
    if len(levels) != 2:
        raise SystemExit('cycles profile needs exactly 2 levels')
    lo, hi = levels
    m.wait_ready()
    # gentle start before jumping to lo: a cold start straight at higher
    # throttle can desync-thrash (seen in SITL with light rotors)
    m.set_throttle(0.1)
    m.spin_for(1.5)
    m.set_throttle(lo)
    m.spin_for(2.0)
    for i in range(args.cycles):
        m.mark('cycle %u up' % i)
        m.set_throttle(hi)
        m.spin_for(args.hold)
        m.mark('cycle %u down' % i)
        m.set_throttle(lo)
        m.spin_for(args.hold)
    m.set_throttle(0)
    m.spin_for(2.0)


def profile_hold(m, args):
    '''constant throttle hold'''
    m.wait_ready()
    m.mark('hold %.3f' % args.throttle)
    m.set_throttle(args.throttle)
    m.spin_for(args.hold)
    m.set_throttle(0)
    m.spin_for(2.0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('profile', choices=['sweep', 'step', 'hold', 'cycles'])
    parser.add_argument('--uri', default='mcast:0:lo')
    parser.add_argument('--node-id', type=int, required=True, help='target ESC node id')
    parser.add_argument('--client-node-id', type=int, default=100)
    parser.add_argument('--esc-index', type=int, default=0)
    parser.add_argument('--rate', type=float, default=100, help='RawCommand rate Hz')
    parser.add_argument('--log', default=None, help='JSONL output file')
    parser.add_argument('--levels', default='0.1,0.15,0.2,0.25,0.3,0.35,0.4,0.45,0.5')
    parser.add_argument('--throttle', type=float, default=0.2, help='hold profile throttle')
    parser.add_argument('--hold', type=float, default=4.0, help='seconds per level')
    parser.add_argument('--coast', type=float, default=5.0, help='step profile coast time')
    parser.add_argument('--cycles', type=int, default=10, help='cycles profile repeat count')
    parser.add_argument('--max-throttle', type=float, default=0.6)
    parser.add_argument('--max-current', type=float, default=4.0, help='abort above this current')
    parser.add_argument('--max-temp', type=float, default=80.0, help='abort above this temperature C')
    parser.add_argument('--ready-timeout', type=float, default=15.0)
    parser.add_argument('--arm-time', type=float, default=1.5, help='zero-throttle stream time after ESC ready')
    parser.add_argument('--telem-rate', type=int, default=None, help='set TELEM_RATE before run')
    parser.add_argument('--debug-rate', type=int, default=None, help='set DEBUG_RATE before run')
    sim_clock.add_argument(parser)
    args = parser.parse_args()

    m = EscMeasure(args)
    try:
        if args.telem_rate is not None or args.debug_rate is not None:
            m.wait_ready()
            if args.telem_rate is not None:
                print('TELEM_RATE ->', m.set_param('TELEM_RATE', args.telem_rate))
            if args.debug_rate is not None:
                print('DEBUG_RATE ->', m.set_param('DEBUG_RATE', args.debug_rate))
        {'sweep': profile_sweep, 'step': profile_step, 'hold': profile_hold,
         'cycles': profile_cycles}[args.profile](m, args)
    finally:
        # always leave the ESC at zero throttle
        m.throttle = 0.0
        m.aborted = None
        try:
            m.spin_for(0.5)
        except Exception:
            pass
        m.summary()
        m.rec.close()


if __name__ == '__main__':
    main()
