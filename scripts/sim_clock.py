#!/usr/bin/env python3
'''
simulation-time clock for the measurement tools

The SITL state port streams physics samples carrying the simulation
timestamp. Pacing the measurement profiles against that clock instead
of the wall clock makes every test correct at any sim/wall ratio: a
slow machine simply takes longer in wall time while the simulated
experiment - hold durations, chirp frequencies, settle windows - stays
exactly as specified. Real hardware keeps the wall clock, where wall
seconds are physical seconds.

Use make_clock(): it returns a WallClock when no state endpoint is
given, so the tools behave identically against real ESCs.
'''

import socket
import struct
import threading
import time

MAGIC_CMD = 0x5353
MAGIC_DATA = 0x5354
SAMPLE = struct.Struct('<Qfffffffffff3sBB3x')


class WallClock(object):
    '''the host monotonic clock (real hardware, and SITL at speed 1)'''

    sim = False

    def now(self):
        return time.monotonic()

    def arm(self):
        pass

    def sleep_hint(self, seconds):
        return seconds

    def close(self):
        pass


class SimClock(object):
    '''simulation time from the SITL state stream

    A reader thread keeps the newest simulation timestamp. now() is the
    elapsed simulated seconds since the clock was created. Sampling is
    coarse on purpose (default 1ms of simulated time, point samples, no
    averaging) - this is a clock, not a measurement channel, and a fine
    period would burden the sim thread.

    The firmware reboots after 2s without input, so SITL restarts are
    expected while a tool is still connecting: until arm() is called the
    clock rebases across the new epoch and now() stays continuous. After
    arm() (the measured profile has started) a restart invalidates the
    run and now() raises.
    '''

    sim = True

    def __init__(self, host='127.0.0.1', port=57734, period_us=1000,
                 stall_timeout=60.0, ready_timeout=20.0):
        self.addr = (host, int(port))
        self.period_us = period_us
        self.stall_timeout = stall_timeout
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1', 0))
        self.sock.settimeout(0.2)
        self.lock = threading.Lock()
        self.t_ns = None
        self.t0_ns = None
        self.last_rx = time.monotonic()
        self.reset_seen = False
        self.armed = False
        self.running = True
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()
        self.sub_thread = threading.Thread(target=self._subscriber, daemon=True)
        self.sub_thread.start()
        # wait for the first sample so callers start on a valid clock
        deadline = time.monotonic() + ready_timeout
        while time.monotonic() < deadline:
            with self.lock:
                if self.t_ns is not None:
                    self.t0_ns = self.t_ns
                    return
            time.sleep(0.02)
        raise SystemExit('sim clock: no state samples from %s:%d - is the '
                         'SITL --state-port correct?' % self.addr)

    def _subscribe_once(self):
        # cmd 0 = subscribe, flags 0 = point samples (not averaged)
        pkt = struct.pack('<HBBI', MAGIC_CMD, 0, 0,
                          int(self.period_us * 1000))
        try:
            self.sock.sendto(pkt, self.addr)
        except OSError:
            pass

    def _subscriber(self):
        # the subscription expires after 2 wall seconds in the sim
        while self.running:
            self._subscribe_once()
            time.sleep(0.5)

    def _reader(self):
        while self.running:
            try:
                d = self.sock.recv(4096)
            except socket.timeout:
                continue
            except OSError:
                return
            if len(d) < 4:
                continue
            magic, ver, count = struct.unpack('<HBB', d[:4])
            if magic != MAGIC_DATA or ver != 2 or count == 0:
                continue
            off = 4 + (count - 1) * SAMPLE.size
            if off + SAMPLE.size > len(d):
                continue
            t_ns = SAMPLE.unpack_from(d, off)[0]
            with self.lock:
                if self.t_ns is not None and t_ns < self.t_ns:
                    if self.armed:
                        # the epoch changed mid-profile: joining the two
                        # would corrupt every duration measured across
                        # the boundary
                        self.reset_seen = True
                    elif self.t0_ns is not None:
                        # expected no-input reboot while connecting:
                        # rebase so now() continues smoothly
                        self.t0_ns = t_ns - (self.t_ns - self.t0_ns)
                self.t_ns = t_ns
                self.last_rx = time.monotonic()

    def arm(self):
        '''the measured profile starts here: restarts stop being
        absorbed and now() raises on the next epoch change'''
        with self.lock:
            self.armed = True

    def now(self):
        with self.lock:
            t_ns, last_rx, reset = self.t_ns, self.last_rx, self.reset_seen
        if reset:
            raise SystemExit('sim clock: simulation time went backwards '
                             '(SITL restarted mid-run)')
        if time.monotonic() - last_rx > self.stall_timeout:
            raise SystemExit('sim clock: no state samples for %.0fs of wall '
                             'time - SITL stalled or exited'
                             % self.stall_timeout)
        return (t_ns - self.t0_ns) * 1e-9

    def sleep_hint(self, seconds):
        '''wall-time sleep for a simulated duration: never longer than
        the requested simulated time, since the sim may run at any
        ratio, and short enough to keep sockets serviced'''
        return min(seconds, 0.005)

    def close(self):
        self.running = False
        try:
            self.sock.close()
        except OSError:
            pass


def make_clock(sim_state=None, **kwargs):
    '''sim_state is "host:port" (or ":port"/"port") for SITL, None for
    wall time'''
    if not sim_state:
        return WallClock()
    host = '127.0.0.1'
    port = sim_state
    if ':' in str(sim_state):
        host, port = str(sim_state).rsplit(':', 1)
        host = host or '127.0.0.1'
    return SimClock(host=host, port=int(port), **kwargs)


def add_argument(parser):
    parser.add_argument('--sim-state', default=None, metavar='[HOST:]PORT',
                        help='pace the run against SITL simulation time from '
                             'this state port instead of the wall clock '
                             '(makes results independent of sim speed)')
