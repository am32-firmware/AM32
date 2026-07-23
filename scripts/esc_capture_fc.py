#!/usr/bin/env python3
'''
capture SITL calibration data from an AM32 ESC attached to a
Betaflight flight controller

Drives the ESC through the FC's motor-test path (MSP_SET_MOTOR while
disarmed) and logs the FC's bidirectional-DShot telemetry
(MSP_MOTOR_TELEMETRY: eRPM-derived mechanical rpm, plus EDT
voltage/current/temperature when the ESC has extended telemetry
enabled) to the same JSONL schema as the DroneCAN tools, so
esc_analyze/esc_square/esc_chirp fit work unchanged.

Betaflight prerequisites (CLI):
  set motor_pwm_protocol = DSHOT300   # or DSHOT600
  set dshot_bidir = ON
  set motor_poles = <your motor's pole count>
  set dshot_edt = ON                  # BF 4.4+, for volt/curr/temp
  save

SAFETY: remove all propellers, and power the ESC from a bench supply
you can switch off. Betaflight holds the last motor-test value it was
given and applies no failsafe to it. This tool aborts on
over-current/over-temperature/telemetry loss/arming and commands stop
when it exits, but it cannot do so if the USB link dies or it is
killed outright. Betaflight also serves its last decoded telemetry
indefinitely once the ESC's replies stop, which is only detectable
here when the throttle is changing.

usage examples:
  esc_capture_fc.py sweep --port /dev/ttyACM0 --poles 14 --log sweep1.jsonl
  esc_capture_fc.py chirp --port /dev/ttyACM0 --poles 14 --duration 120 --log chirp.jsonl
'''

import argparse
import queue
import threading
import signal
import time

import serial

import esc_chirp
import esc_measure
import esc_square
import msp
import sim_clock

# Betaflight reports 100.00% invalid when bidirectional DShot telemetry
# is not arriving at all; anything above this is treated as no signal
INVALID_PCT_LIMIT = 99.0


class BetaflightBackend(object):
    '''EscMeasure-compatible measurement backend talking MSP to a
    Betaflight FC (see esc_measure.EscMeasure for the interface the
    profiles use)'''

    def __init__(self, args):
        self.args = args
        self.motor_index = args.motor - 1
        # telemetry is stamped and queued on the reader thread so no
        # sample is lost or mis-timed when this process is descheduled
        self.telem_q = queue.Queue()
        # held while a command row is written, so telemetry cannot be
        # stamped into the gap between draining and logging it
        self.order_lock = threading.Lock()
        self.port = msp.MspPort(
            args.port, callbacks={msp.MSP_MOTOR_TELEMETRY: self._on_telemetry})
        self.clock = sim_clock.WallClock()
        self.rec = esc_measure.Recorder(args.log, clock=self.clock)
        self.throttle = 0.0
        self.next_tx = 0.0
        self.aborted = None
        self.boxids = b''
        self.motor_count = 8
        self.last_telem = None       # latest valid per-motor telemetry
        self.last_telem_wall = time.monotonic()
        self.telem_invalid = False
        self.edt_seen = False
        self.next_status_poll = 0.0
        self.last_status_wall = time.monotonic()
        self.status_seq = 0
        # Betaflight caches the last decoded telemetry and never marks
        # it stale, so a dead reply path looks like frozen-but-valid
        # data: watch for values that stop changing while spinning
        self.last_fingerprint = None
        self.responded = True
        self.throttle_change_wall = time.monotonic()
        # no MSP_SET_MOTOR until preflight proves 1000 means stop (in
        # 3D mode it would be full reverse)
        self.motor_safe = False

    # -- duck interface used by the profiles --------------------------

    def set_throttle(self, throttle):
        throttle = max(0.0, min(self.args.max_throttle, throttle))
        if throttle != self.throttle:
            # log telemetry that arrived before this command first, so
            # the file stays in time order for the segmenting analyses.
            # The lock keeps the reader from stamping a sample into the
            # gap between the drain and the command row
            with self.order_lock:
                self._poll_telemetry()
                if self.aborted:
                    # that drain tripped an abort: never log a command
                    # the run will not send; spin_for raises next pass
                    self.throttle = 0.0
                    return
                self.rec.add('cmd', throttle=round(throttle, 4))
            if self.responded:
                # start the response clock at the first command the
                # telemetry has not answered; later ones must not keep
                # pushing the deadline back
                self.throttle_change_wall = time.monotonic()
                self.responded = False
        self.throttle = throttle

    def spin_for(self, duration):
        '''stream SET_MOTOR + telemetry polls for duration seconds.
        Absolute deadlines keep the command schedule from drifting'''
        tend = self.clock.now() + duration
        period = 1.0 / self.args.rate
        while self.clock.now() < tend:
            if self.aborted:
                self.throttle = 0.0
            now = self.clock.now()
            if now >= self.next_tx:
                self._send_motor(self.throttle)
                # pipelined: the reply is consumed whenever it arrives
                self.port.send(msp.MSP_MOTOR_TELEMETRY)
                self.next_tx = max(now, self.next_tx + period)
            self._poll_telemetry()
            self._poll_armed()
            time.sleep(0.001)
            if self.aborted:
                raise SystemExit('aborted: %s' % self.aborted)

    def wait_ready(self):
        '''preflight the FC config, then stream zero throttle so the
        ESC's zero-throttle arming latches before any profile runs'''
        self._preflight()
        print('PROPS OFF? starting in %.0fs (Ctrl-C to abort)' %
              self.args.countdown)
        time.sleep(self.args.countdown)
        self.last_telem_wall = time.monotonic()
        self.last_status_wall = time.monotonic()
        tstart = self.clock.now()
        while self.last_telem is None:
            self.spin_for(0.2)
            if self.clock.now() - tstart > self.args.ready_timeout:
                raise SystemExit(
                    'no valid bidirectional DShot telemetry for motor %u - '
                    'check dshot_bidir is ON, the ESC is powered and the '
                    'signal wire is on that motor output'% self.args.motor)
        t = self.last_telem
        print('ESC ready: rpm=%u volt=%.0f temp=%.0fC' %
              (t['rpm'], t['volt'], t['temp']))
        self.spin_for(self.args.arm_time)
        if not self.edt_seen:
            # without EDT frames there is no current or temperature to
            # check, so those aborts can never fire
            print('WARNING: no EDT telemetry (voltage/current/temperature) - '
                  '--max-current and --max-temp protection are INACTIVE.\n'
                  '         enable it with: set dshot_edt = ON (BF 4.4+)')

    def set_param(self, name, value):
        # ESC params are DroneCAN-only; FC-attached ESCs keep their
        # eeprom settings
        print('note: param %s not settable over the FC link' % name)
        return None

    def mark(self, label):
        self.rec.add('mark', label=label)
        print('== %s' % label)

    def abort(self, reason):
        if self.aborted is None:
            self.aborted = reason
            print('ABORT: %s' % reason)

    # summary() has no transport dependency: reuse the DroneCAN one
    summary = esc_measure.EscMeasure.summary

    # -- MSP mechanics -------------------------------------------------

    def _send_motor(self, throttle):
        if not self.motor_safe:
            return
        vals = [1000] * 8
        vals[self.motor_index] = 1000 + int(round(throttle * 1000))
        self.port.send(msp.MSP_SET_MOTOR, b''.join(
            v.to_bytes(2, 'little') for v in vals))

    def _on_telemetry(self, payload):
        '''reader thread: stamp the arrival and queue it'''
        with self.order_lock:
            self.telem_q.put((self.clock.now(), payload))

    def _poll_telemetry(self):
        while True:
            try:
                at, data = self.telem_q.get_nowait()
            except queue.Empty:
                break
            self._handle_telemetry(at, data)
        # stall detection in wall time: a wedged FC or a dead telemetry
        # wire must not leave the motor running open loop. Before the
        # first valid sample the ready_timeout covers it
        if (self.last_telem is not None and
                time.monotonic() - self.last_telem_wall > self.args.stall_timeout):
            self.abort('no valid motor telemetry for %.1fs' %
                       self.args.stall_timeout)

    def _handle_telemetry(self, at, data):
        try:
            motors = msp.parse_motor_telemetry(data)
        except Exception as ex:
            self.rec.add('error', _at=at, msg='telemetry parse: %s' % ex)
            return
        if self.motor_index >= len(motors):
            self.abort('FC reports %u motors, --motor %u out of range' %
                       (len(motors), self.args.motor))
            return
        t = motors[self.motor_index]
        # Betaflight answers this command even with no BDShot signal at
        # all, reporting 100% invalid and zeroed values: that must not
        # pass for telemetry or the safety aborts have nothing to act on
        if t['invalid_pct'] > INVALID_PCT_LIMIT:
            if not self.telem_invalid:
                self.telem_invalid = True
                # explains the gap that follows in the log
                self.rec.add('error', _at=at, msg='no valid BDShot telemetry')
            return
        self.telem_invalid = False
        self.last_telem = t
        self.last_telem_wall = time.monotonic()
        if t['volt'] > 0 or t['temp'] > 0 or t['curr'] > 0:
            self.edt_seen = True
        # Betaflight keeps serving its last decoded values when the
        # BDShot replies stop arriving entirely (no edges leave its
        # quality stats untouched), so identical readings while the
        # motor is being driven mean the reply path is dead
        now = time.monotonic()
        fp = (t['rpm'], t['volt'], t['curr'], t['temp'], t['invalid_pct'])
        if fp != self.last_fingerprint:
            self.last_fingerprint = fp
            self.responded = True
        elif (self.throttle > 0 and not self.responded and
                now - self.throttle_change_wall > self.args.freeze_timeout):
            # telemetry that does not move after the throttle does means
            # Betaflight is serving its cache. Duration alone proves
            # nothing: every field is coarsely quantised, so a steady
            # hold legitimately repeats the same values indefinitely -
            # that case is undetectable here and is documented instead
            self.abort('telemetry has not moved in the %.1fs since the '
                       'throttle changed - the BDShot reply path has '
                       'probably stopped' % self.args.freeze_timeout)
        # a commanded motor that never turns is a stall: separate check,
        # because temperature or current drift keeps the fingerprint
        # changing while the rpm sits at zero
        # a motor that does not start, or desyncs and stops, is not an
        # error here: low throttle levels are marginal by nature and
        # capturing that behaviour is the point (the calibration data
        # sets carry desync budgets). Over-current is what guards a
        # stalled motor, same as the DroneCAN path
        # err carries the BDShot invalid-frame percentage (the DroneCAN
        # tools log the desync counter here; nothing in the fit stack
        # reads err from real captures)
        self.rec.add('status', _at=at, rpm=t['rpm'], volt=round(t['volt'], 3),
                     curr=round(t['curr'], 3), temp=round(float(t['temp']), 1),
                     err=round(t['invalid_pct'], 2))
        if t['curr'] > self.args.max_current:
            self.abort('current %.1fA > %.1fA limit' %
                       (t['curr'], self.args.max_current))
        if t['temp'] > self.args.max_temp:
            self.abort('temperature %.0fC > %.0fC limit' %
                       (t['temp'], self.args.max_temp))

    def _poll_armed(self):
        seq, data = self.port.get_reply(msp.MSP_STATUS)
        if seq != self.status_seq:
            self.status_seq = seq
            self.last_status_wall = time.monotonic()
            if data is not None and len(data) >= 10 and self.boxids:
                if msp.parse_status_armed(data, self.boxids):
                    self.abort('FC is armed - motor test needs a disarmed FC')
        elif time.monotonic() - self.last_status_wall > self.args.status_timeout:
            # a stale disarmed reply must not stand in for a live one
            self.abort('no MSP status reply for %.1fs' %
                       self.args.status_timeout)
        now = self.clock.now()
        if now >= self.next_status_poll:
            self.next_status_poll = now + 0.5
            self.port.send(msp.MSP_STATUS)

    def _preflight(self):
        variant = self.port.request(msp.MSP_FC_VARIANT).decode(
            'ascii', 'replace')
        api = msp.parse_api_version(self.port.request(msp.MSP_API_VERSION))
        print('FC: %s API %u.%u' % (variant, api[1], api[2]))
        if variant != 'BTFL':
            raise SystemExit('unsupported FC variant %r (need Betaflight)'
                             % variant)
        if (api[1], api[2]) < (1, 42):
            raise SystemExit('MSP API %u.%u too old (need 1.42+, '
                             'Betaflight 4.2+)' % (api[1], api[2]))
        features = msp.parse_feature_config(
            self.port.request(msp.MSP_FEATURE_CONFIG))
        if features & msp.FEATURE_3D:
            # in 3D mode the neutral point is 1500 and this tool's stop
            # value would command full reverse
            raise SystemExit('Betaflight 3D mode is enabled - motor stop '
                             'would be full reverse. Disable it:\n'
                             '  feature -3D\n  save')
        cfg = msp.parse_motor_config(self.port.request(msp.MSP_MOTOR_CONFIG))
        self.motor_count = cfg.get('motor_count', 8)
        if not cfg.get('use_dshot_telemetry'):
            raise SystemExit('bidirectional DShot is off - run:\n'
                             '  set dshot_bidir = ON\n  save')
        if cfg.get('use_esc_sensor'):
            # with the ESC sensor feature on, Betaflight overwrites the
            # DShot voltage/current with the sensor's, in different units
            raise SystemExit('the ESC_SENSOR feature is enabled and would '
                             'replace the DShot telemetry values - run:\n'
                             '  feature -ESC_SENSOR\n  save')
        poles = cfg.get('motor_poles')
        print('motors: %u, poles: %s' % (self.motor_count, poles))
        # the FC converts eRPM to rpm with its own pole count, so a
        # stale setting silently scales every logged rpm
        if poles != self.args.poles:
            raise SystemExit('FC motor_poles=%s but the motor has %u poles - '
                             'every logged rpm would be wrong; fix with:\n'
                             '  set motor_poles = %u\n  save'
                             % (poles, self.args.poles, self.args.poles))
        if not 1 <= self.args.motor <= self.motor_count:
            raise SystemExit('--motor %u out of range, FC has %u motors'
                             % (self.args.motor, self.motor_count))
        self.boxids = self.port.request(msp.MSP_BOXIDS)
        status = self.port.request(msp.MSP_STATUS)
        if msp.parse_status_armed(status, self.boxids):
            raise SystemExit('FC is armed - disarm before motor testing')
        # every check that makes 1000 mean stop has now passed
        self.motor_safe = True

    def shutdown(self):
        '''dead-man: stream zeros on the way out. Cannot help if the
        link itself has failed or the process is killed outright - the
        FC keeps the last motor-test value it was given'''
        if not self.motor_safe:
            # preflight never established that 1000 means stop (3D mode
            # or a failed query), and nothing was ever commanded
            self.port.close()
            return
        failed = None
        try:
            end = time.monotonic() + 0.5
            while time.monotonic() < end:
                self._send_motor(0)
                time.sleep(0.02)
        except Exception as ex:
            failed = ex
        if failed is not None:
            print('WARNING: could not command motor stop (%s) - '
                  'CUT POWER TO THE ESC' % failed)
        self.port.close()


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='cmd', required=True)

    def common(p, rate, max_current):
        p.add_argument('--port', required=True, help='FC serial port, e.g. /dev/ttyACM0')
        p.add_argument('--motor', type=int, default=1, help='motor number (1-based, as in the BF configurator)')
        p.add_argument('--log', required=True, help='JSONL output file')
        p.add_argument('--rate', type=float, default=rate, help='command rate Hz')
        p.add_argument('--poles', type=int, required=True, help="the motor's pole count (must match the FC's motor_poles: it scales every logged rpm)")
        p.add_argument('--max-throttle', type=float, default=0.6)
        p.add_argument('--max-current', type=float, default=max_current, help='abort above this current')
        p.add_argument('--max-temp', type=float, default=80.0, help='abort above this temperature C')
        p.add_argument('--stall-timeout', type=float, default=0.5, help='abort after this long without valid telemetry')
        p.add_argument('--status-timeout', type=float, default=3.0, help='abort after this long without an MSP status reply')
        p.add_argument('--freeze-timeout', type=float, default=5.0, help='abort after this long of telemetry that does not respond to a throttle change')
        p.add_argument('--ready-timeout', type=float, default=15.0)
        p.add_argument('--arm-time', type=float, default=3.0, help='zero-throttle stream time before the profile')
        p.add_argument('--countdown', type=float, default=3.0, help='props-off warning delay')

    p = sub.add_parser('hold', help='constant throttle hold')
    common(p, rate=100, max_current=6.0)
    p.add_argument('--throttle', type=float, default=0.2)
    p.add_argument('--hold', type=float, default=4.0)

    p = sub.add_parser('sweep', help='steady staircase up and down')
    common(p, rate=100, max_current=6.0)
    p.add_argument('--levels', default='0.1,0.15,0.2,0.25,0.3,0.35,0.4,0.45,0.5')
    p.add_argument('--hold', type=float, default=4.0, help='seconds per level')

    p = sub.add_parser('step', help='zero-to-level steps with coast down')
    common(p, rate=100, max_current=6.0)
    p.add_argument('--levels', default='0.1,0.2,0.3,0.4')
    p.add_argument('--hold', type=float, default=4.0)
    p.add_argument('--coast', type=float, default=5.0)

    p = sub.add_parser('cycles', help='repeated two-level steps for ensemble transients')
    common(p, rate=100, max_current=6.0)
    p.add_argument('--levels', default='0.2,0.4')
    p.add_argument('--hold', type=float, default=4.0)
    p.add_argument('--cycles', type=int, default=10)

    p = sub.add_parser('square', help='square wave amplitude sweep (esc_square profile)')
    common(p, rate=200, max_current=30.0)
    p.add_argument('--throttle-mid', type=float, default=0.35)
    p.add_argument('--deltas', default='0.03,0.06,0.09,0.12,0.15,0.18,0.21,0.24')
    p.add_argument('--cycles', type=int, default=3)
    p.add_argument('--settle-tol', type=float, default=0.01)
    p.add_argument('--settle-window', type=float, default=0.4)
    p.add_argument('--settle-min', type=float, default=0.6)
    p.add_argument('--settle-max', type=float, default=5.0)

    p = sub.add_parser('chirp', help='exponential frequency chirp (esc_chirp profile)')
    common(p, rate=200, max_current=6.0)
    p.add_argument('--throttle-mid', type=float, default=0.35)
    p.add_argument('--throttle-amp', type=float, default=0.15)
    p.add_argument('--f-start', type=float, default=0.5)
    p.add_argument('--f-stop', type=float, default=40.0)
    p.add_argument('--duration', type=float, default=45.0)
    p.add_argument('--fade-in', type=float, default=3.0)
    p.add_argument('--fade-out', type=float, default=1.0)

    args = parser.parse_args()
    # the shared profiles read these; DroneCAN-only features stay off
    args.telem_rate = None
    args.debug_rate = None

    # a plain terminate must still unwind to the motor-stop path
    def on_signal(signum, _frame):
        raise SystemExit('signal %u' % signum)
    for sig in (signal.SIGTERM, getattr(signal, 'SIGHUP', None)):
        if sig is not None:
            signal.signal(sig, on_signal)

    try:
        m = BetaflightBackend(args)
    except serial.SerialException as ex:
        raise SystemExit('cannot open %s: %s' % (args.port, ex))
    try:
        if args.cmd == 'hold':
            esc_measure.profile_hold(m, args)
            m.summary()
        elif args.cmd == 'sweep':
            esc_measure.profile_sweep(m, args)
            m.summary()
        elif args.cmd == 'step':
            esc_measure.profile_step(m, args)
        elif args.cmd == 'cycles':
            esc_measure.profile_cycles(m, args)
        elif args.cmd == 'square':
            deltas = [float(x) for x in args.deltas.split(',')]
            esc_square.run_profile(m, args, deltas)   # prints saved
        elif args.cmd == 'chirp':
            m.wait_ready()
            esc_chirp.run_chirp(m, args)              # closes + prints saved
        if args.cmd not in ('chirp', 'square'):
            print('saved %s' % args.log)
    except BaseException as ex:
        # keep whatever was captured, and record why it ended
        m.rec.add('error', msg=str(ex) or type(ex).__name__)
        raise
    finally:
        # stop the motor first, then flush the log
        m.shutdown()
        m.rec.close()


if __name__ == '__main__':
    main()
