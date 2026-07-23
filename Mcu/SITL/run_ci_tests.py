#!/usr/bin/env python3
'''
CI test runner for the AM32 SITL: starts the simulator, drives it
through the PWM/DShot and DroneCAN input paths and asserts on the
results. Only needs the python standard library; the DroneCAN test runs
when the dronecan package is importable and is skipped otherwise.

usage: run_ci_tests.py [--sitl path/to/elf]
exits non-zero if any test fails.
'''

import argparse
import glob
import os
import signal
import struct
import subprocess
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sitl_dshot as sd
import sitl_params
import sitl_tones
from sitl_gui_backend import EepromClient, SimStream, ToneStream, AudioStream

HERE = os.path.dirname(os.path.abspath(__file__))
INPUT_PORT = 57833
STATE_PORT = 57834
CAN_URI = 'mcast:7'

failures = []


def check(name, cond, detail):
    status = 'PASS' if cond else 'FAIL'
    print('%s: %s (%s)' % (status, name, detail))
    sys.stdout.flush()
    if not cond:
        failures.append(name)


class Sitl(object):
    def __init__(self, sitl_path, extra_args=()):
        args = [sitl_path, '--input-port', str(INPUT_PORT),
                '--state-port', str(STATE_PORT), '--nosleep'] + list(extra_args)
        self.log = open('sitl_ci.log', 'ab')
        self.proc = subprocess.Popen(args, stdout=self.log, stderr=self.log)
        time.sleep(0.5)

    def __enter__(self):
        return self

    def __exit__(self, *a):
        # SIGTERM (not kill) so a coverage build can flush its .gcda via
        # the SITL's signal handler; fall back to kill if it lingers
        self.proc.terminate()
        try:
            self.proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.proc.kill()
            self.proc.wait()
        self.log.close()
        for f in os.listdir('.'):
            if f.startswith('am32_eeprom.bin'):
                os.unlink(f)


class Sender(object):
    '''background frame sender with rate catch-up, like the GUI'''

    def __init__(self, ptype, bidir=False, rate=500.0):
        self.port = sd.InputPort('127.0.0.1', INPUT_PORT)
        self.ptype = ptype
        self.bidir = bidir
        self.rate = rate
        self.value = 1000 if ptype == sd.TYPE_PWM else 0
        self.cmds = []
        self.running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        nxt = time.time()
        try:
            self._run()
        except OSError:
            pass  # socket closed by stop()

    def _run(self):
        nxt = time.time()
        while self.running:
            now = time.time()
            burst = 0
            while now >= nxt and burst < 10:
                nxt += 1.0 / self.rate
                if self.cmds:
                    self.port.send_dshot(self.cmds.pop(0), ptype=self.ptype,
                                         telem=True, bidir=self.bidir)
                elif self.ptype == sd.TYPE_PWM:
                    self.port.send_pwm(int(self.value))
                else:
                    self.port.send_dshot(int(self.value), ptype=self.ptype,
                                         bidir=self.bidir)
                burst += 1
            if now - nxt > 0.25:
                nxt = now
            time.sleep(0.0005)

    def stop(self):
        self.running = False
        self.port.close()


def sim_sleep(sim, secs, wall_cap=90.0):
    """wait for `secs` of SIMULATION time, polling the state stream. The
    tests drive the firmware over wall time but the firmware lives in
    sim time, so a coverage or sanitizer build (slower than real time)
    would otherwise arm/spin-up too little. Falls back to wall time if
    no samples arrive. SimStream timestamps (smp[0]) are already seconds"""
    t0 = None
    deadline = time.monotonic() + wall_cap
    while time.monotonic() < deadline:
        smp = sim.latest()
        if smp is not None:
            if t0 is None or smp[0] < t0:
                t0 = smp[0]              # (re-)anchor; a reboot resets sim time
            elif smp[0] - t0 >= secs:
                return
        time.sleep(0.01)


def rpm_from_state(sim, window=1.0):
    w = sim.window(window)
    if not w:
        return -1
    return sum(s[1] for s in w) / len(w) * 60.0 / 6.28318


def test_dshot(sitl_path, name, ptype, bidir, edt, value, rpm_lo, rpm_hi, input_type=1):
    with Sitl(sitl_path, ['--can-uri', 'none', '--input-type', str(input_type)]):
        sim = SimStream('127.0.0.1', STATE_PORT, period_us=200)
        sim.enabled = True
        tx = Sender(ptype, bidir=bidir)
        try:
            sim_sleep(sim, 2.2)                # arm at zero throttle
            if edt:
                # the firmware wants 6 repeats while armed and stopped, and
                # ignores the command once spinning. Frames can be lost on
                # overloaded runners, so confirm the latch from the EDT
                # frames in the replies and retry before throttling up
                for _ in range(6):
                    tx.port.get_replies()
                    tx.cmds = [sd.DSHOT_CMD_EDT_ENABLE] * 20
                    sim_sleep(sim, 0.5)
                    got = [sd.decode_reply(r[3], edt_expected=True)
                           for r in tx.port.get_replies()]
                    if any(k not in ('erpm', 'badcrc') for k, v in got):
                        break
            tx.value = value
            sim_sleep(sim, 4.0)
            rpm = rpm_from_state(sim)
            check(name + ' rpm', rpm_lo <= rpm <= rpm_hi,
                  'rpm=%.0f expected %d..%d' % (rpm, rpm_lo, rpm_hi))
            if bidir:
                replies = tx.port.reply_count
                check(name + ' bdshot replies', replies > 500,
                      'replies=%d' % replies)
                erpm = [sd.decode_reply(r[3], edt_expected=edt)
                        for r in tx.port.get_replies()]
                rpms = [sd.erpm_period_to_rpm(v) for k, v in erpm if k == 'erpm']
                if rpms:
                    check(name + ' bdshot rpm agrees',
                          abs(rpms[-1] - rpm) < max(200, rpm * 0.05),
                          'bdshot=%.0f state=%.0f' % (rpms[-1], rpm))
                if edt:
                    edt_vals = dict((k, v) for k, v in erpm
                                    if k in ('temp', 'volt', 'current'))
                    check(name + ' edt values',
                          edt_vals.get('temp') == 38 and 11 < edt_vals.get('volt', 0) < 13,
                          'edt=%s' % edt_vals)
            # motor must stop again at zero throttle
            tx.value = 1000 if ptype == sd.TYPE_PWM else 0
            sim_sleep(sim, 3.0)
            rpm = rpm_from_state(sim, 0.3)
            check(name + ' stops', rpm < 500, 'rpm=%.0f' % rpm)
        finally:
            tx.stop()
            sim.close()


def wait_for_notes(stream, expected, timeout=10.0):
    '''wait until the expected note sequence appears in the tone stream.
    Durations come from the simulated timestamps, so this is exact under
    --nosleep too'''
    deadline = time.time() + timeout
    notes = []
    while time.time() < deadline:
        with stream.lock:
            events = list(stream.events)
        notes = sitl_tones.events_to_notes(events)
        found = sitl_tones.match_notes(notes, expected,
                                       freq_tol=0.02, dur_tol=0.05)
        if found:
            return found, notes
        time.sleep(0.05)
    return None, notes


def test_startup_tune(sitl_path):
    '''the boot tune must arrive on the tone event stream: 3 rising
    notes from TIM1 PSC 55/40/25 at ARR 6665, 200ms each'''
    expected = [(428.6, 0.2), (585.4, 0.2), (923.2, 0.2)]
    tones = ToneStream('127.0.0.1', STATE_PORT)
    try:
        with Sitl(sitl_path, ['--can-uri', 'none']):
            found, notes = wait_for_notes(tones, expected)
            check('startup tune', found is not None,
                  'notes=%s' % ['%.1fHz %.3fs' % n[:2] for n in notes])
    finally:
        tones.close()


def test_beacon_tone(sitl_path):
    '''DShot beacon command 1 plays playDefaultTone: PSC 50 then 30,
    150ms each'''
    expected = [(470.6, 0.15), (774.3, 0.15)]
    tones = ToneStream('127.0.0.1', STATE_PORT)
    try:
        with Sitl(sitl_path, ['--can-uri', 'none', '--input-type', '1']):
            sim = SimStream('127.0.0.1', STATE_PORT, period_us=1000)
            sim.enabled = True
            tx = Sender(sd.TYPE_DSHOT600)
            try:
                sim_sleep(sim, 2.2)            # arm at zero throttle
                tx.cmds = [1] * 8              # DSHOT_CMD_BEACON1
                found, notes = wait_for_notes(tones, expected, timeout=25.0)
                check('beacon tone', found is not None,
                      'notes=%s' % ['%.1fHz %.3fs' % n[:2] for n in notes])
            finally:
                tx.stop()
                sim.close()
    finally:
        tones.close()


def test_physics_audio(sitl_path):
    '''the physics audio stream must carry the boot tune: some 100ms
    sim-time window has the first note frequency (428.6Hz) dominant'''
    audio = AudioStream('127.0.0.1', STATE_PORT)
    try:
        with Sitl(sitl_path, ['--can-uri', 'none']):
            deadline = time.time() + 12
            batches = []
            ok = False
            while time.time() < deadline and not ok:
                time.sleep(0.5)
                batches += audio.take_batches()
                buckets = {}
                for t0, vals in batches:
                    for i, v in enumerate(vals):
                        t = t0 + i * 20833
                        buckets.setdefault(t // 100000000, []).append(v)
                for vals in buckets.values():
                    if len(vals) < 2000:
                        continue
                    g1 = sitl_tones.goertzel(vals, 428.6)
                    g2 = sitl_tones.goertzel(vals, 700.0)
                    if g1 > 1e-3 and g1 > 5 * g2:
                        ok = True
                        break
            check('physics audio boot tune', ok,
                  '%d batches captured' % len(batches))
    finally:
        audio.close()


def can_state_stream(name):
    '''probe for CAN-enabled tests: some CI runners (github macos) have
    no multicast capable route and the SITL cannot bring CAN up. Returns
    a live SimStream, or None after printing a SKIP with the SITL log
    for diagnosis'''
    sim = SimStream('127.0.0.1', STATE_PORT, period_us=200)
    sim.enabled = True
    deadline = time.time() + 5
    while time.time() < deadline and not sim.samples:
        time.sleep(0.2)
    if not sim.samples:
        print('SKIP: %s, SITL state stream never started with CAN enabled, '
              'multicast is probably unavailable on this host. SITL log tail:'
              % name)
        sys.stdout.flush()
        os.system('tail -5 sitl_ci.log')
        sim.close()
        return None
    return sim


def test_dronecan(sitl_path):
    try:
        import dronecan
    except ImportError:
        print('SKIP: dronecan not installed, DroneCAN test skipped')
        return
    with Sitl(sitl_path, ['--can-uri', CAN_URI, '--node-id', '10']):
        sim = can_state_stream('dronecan')
        if sim is None:
            return
        node = dronecan.make_node(CAN_URI, node_id=100, bitrate=1000000)
        status = {}

        def on_esc(e):
            status['rpm'] = e.message.rpm
            status['voltage'] = e.message.voltage

        node.add_handler(dronecan.uavcan.equipment.esc.Status, on_esc)
        t0 = time.time()
        nxt = t0
        while time.time() - t0 < 10:
            node.spin(0)
            now = time.time()
            if now >= nxt:
                nxt += 0.02
                thr = 0.35 if now - t0 > 2.5 else 0.0
                node.broadcast(dronecan.uavcan.equipment.safety.ArmingStatus(status=255))
                node.broadcast(dronecan.uavcan.equipment.esc.RawCommand(cmd=[int(8191 * thr)]))
            time.sleep(0.001)
        # read rpm from the state stream while the motor is still driven;
        # commands have stopped, so a wait here would let it coast down
        rpm = rpm_from_state(sim)
        check('dronecan rpm', 2500 <= rpm <= 5000, 'rpm=%.0f' % rpm)
        check('dronecan telemetry', 2500 <= status.get('rpm', -1) <= 5000
              and 11 < status.get('voltage', 0) < 13,
              'esc.Status=%s' % status)
        node.close()
        sim.close()


def test_dshot_direction(sitl_path):
    """DShot command 8 reverses direction; the rotor omega changes sign"""
    with Sitl(sitl_path, ['--can-uri', 'none', '--input-type', '1']):
        sim = SimStream('127.0.0.1', STATE_PORT, period_us=200)
        sim.enabled = True
        tx = Sender(sd.TYPE_DSHOT600)
        try:
            sim_sleep(sim, 2.2)                   # arm
            tx.value = 400
            sim_sleep(sim, 3.0)
            fwd = sim.latest()[1] if sim.latest() else 0
            # a loaded runner can drop enough input frames that the
            # repeated command never registers; command 8 is absolute
            # (not a toggle) so re-sending it is safe
            for attempt in range(3):
                tx.value = 0
                sim_sleep(sim, 1.5)
                tx.cmds = [8] * 20                # reverse direction
                sim_sleep(sim, 1.5)
                tx.cmds = []
                tx.value = 400
                sim_sleep(sim, 3.0)
                rev = sim.latest()[1] if sim.latest() else 0
                if fwd * rev < 0 and abs(fwd) > 50 and abs(rev) > 50:
                    break
                if attempt < 2:
                    print('  reverse attempt %d failed, retrying'
                          % (attempt + 1), flush=True)
            check('dshot reverse direction',
                  fwd * rev < 0 and abs(fwd) > 50 and abs(rev) > 50,
                  'omega fwd=%.0f rev=%.0f' % (fwd, rev))
        finally:
            tx.stop()
            sim.close()


def test_bidirectional(sitl_path):
    """3D/bidirectional mode: command 10 enables it, then a reverse-half
    throttle spins the motor backwards"""
    with Sitl(sitl_path, ['--can-uri', 'none', '--input-type', '1']):
        sim = SimStream('127.0.0.1', STATE_PORT, period_us=200)
        sim.enabled = True
        tx = Sender(sd.TYPE_DSHOT600, bidir=True)
        try:
            sim_sleep(sim, 2.2)
            # command 10 is absolute (3D on, not a toggle) so a retry
            # can safely re-send it if a loaded runner dropped the burst
            for attempt in range(3):
                tx.cmds = [10] * 20               # enable 3D
                sim_sleep(sim, 1.5)
                tx.cmds = []
                tx.value = 1400                   # forward half
                sim_sleep(sim, 3.0)
                fwd = sim.latest()[1] if sim.latest() else 0
                tx.value = 0
                sim_sleep(sim, 1.5)
                tx.value = 600                    # reverse half
                sim_sleep(sim, 3.0)
                rev = sim.latest()[1] if sim.latest() else 0
                if fwd * rev < 0 and abs(fwd) > 50 and abs(rev) > 50:
                    break
                if attempt < 2:
                    print('  bidirectional attempt %d failed, retrying'
                          % (attempt + 1), flush=True)
                    tx.value = 0
                    sim_sleep(sim, 1.5)
            check('bidirectional reverse',
                  fwd * rev < 0 and abs(fwd) > 50 and abs(rev) > 50,
                  'omega fwd=%.0f rev=%.0f' % (fwd, rev))
        finally:
            tx.stop()
            sim.close()


def test_dshot_edt_toggle(sitl_path):
    """EDT enable (13) then disable (14) command path"""
    with Sitl(sitl_path, ['--can-uri', 'none', '--input-type', '1']):
        sim = SimStream('127.0.0.1', STATE_PORT, period_us=1000)
        sim.enabled = True
        tx = Sender(sd.TYPE_DSHOT600, bidir=True)
        try:
            sim_sleep(sim, 2.2)
            tx.cmds = [sd.DSHOT_CMD_EDT_ENABLE] * 10
            sim_sleep(sim, 1.0)
            tx.cmds = [12] * 10                   # save settings to eeprom
            sim_sleep(sim, 1.0)
            tx.cmds = [sd.DSHOT_CMD_EDT_DISABLE] * 10
            sim_sleep(sim, 1.0)
            tx.cmds = []
            tx.value = 300
            sim_sleep(sim, 2.0)
            # the point is exercising the paths without a desync/crash;
            # a sanitizer/coverage build is what gains from this
            check('dshot edt toggle survives', tx.port.reply_count > 100,
                  'replies=%d' % tx.port.reply_count)
        finally:
            tx.stop()
            sim.close()


def test_debugger_pause(sitl_path):
    """a stopped process (debugger breakpoint, SIGSTOP, laptop suspend)
    must freeze the simulation and resume cleanly: no watchdog reset,
    no catch-up sprint compressing the backlog, motor still spinning.
    SIGSTOP/SIGCONT exercises exactly the wall-clock jump a debugger
    causes, without needing gdb on the CI runner"""
    if not hasattr(signal, 'SIGSTOP'):
        print('SKIP: no SIGSTOP on this platform, debugger pause test skipped')
        return
    with Sitl(sitl_path, ['--can-uri', 'none', '--input-type', '1']) as s:
        sim = SimStream('127.0.0.1', STATE_PORT, period_us=200)
        sim.enabled = True
        tx = Sender(sd.TYPE_DSHOT600)
        try:
            sim_sleep(sim, 2.2)                # arm at zero throttle
            tx.value = 800
            sim_sleep(sim, 3.0)
            rpm = rpm_from_state(sim)
            check('debugger pause spins', 2000 <= rpm <= 4000, 'rpm=%.0f' % rpm)
            t_stop = sim.latest()[0]
            s.proc.send_signal(signal.SIGSTOP)
            time.sleep(3.0)                    # wall time; sim is frozen
            frozen = sim.latest()[0]
            s.proc.send_signal(signal.SIGCONT)
            wall0 = time.monotonic()
            sim_sleep(sim, 2.0)
            wall_elapsed = time.monotonic() - wall0
            rpm = rpm_from_state(sim)
            after = sim.latest()[0]
            check('debugger pause freezes sim time',
                  0 <= frozen - t_stop < 0.1,
                  'sim advanced %.2fs during a 3s stop' % (frozen - t_stop))
            # at speedup 1 simulated time may never advance faster than
            # the wall clock: a catch-up sprint through the 3s backlog
            # would, so paced resumption is provable from the wall time
            # the post-resume simulated seconds took
            check('debugger pause resumes paced',
                  wall_elapsed > (after - frozen) * 0.9,
                  'sim advanced %.2fs in %.2fs of wall' % (after - frozen,
                                                           wall_elapsed))
            check('debugger pause still spinning', 2000 <= rpm <= 4000,
                  'rpm=%.0f' % rpm)
        finally:
            tx.stop()
            sim.close()


def test_stuck_rotor(sitl_path, protection):
    """stuck rotor (prop blocked mid-flight, e.g. hitting a tree
    branch; state port cmd 7): with STUCK_ROTOR_PROTECTION on the
    firmware must cut the output after the commutation timeouts and
    stay off until the throttle is cycled through zero; with it off
    the motor must restart by itself once the obstruction clears"""
    name = 'stuck rotor protection %s' % ('on' if protection else 'off')
    offset = dict((p[1], p[0])
                  for p in sitl_params.PARAMS)['STUCK_ROTOR_PROTECTION']
    with Sitl(sitl_path, ['--can-uri', 'none', '--input-type', '1']):
        ok, msg = EepromClient('127.0.0.1', STATE_PORT).set(
            offset, bytes([1 if protection else 0]))
        check(name + ' eeprom set', ok, msg)
        sim = SimStream('127.0.0.1', STATE_PORT, period_us=200)
        sim.enabled = True
        tx = Sender(sd.TYPE_DSHOT600)
        try:
            sim_sleep(sim, 2.2)                # arm at zero throttle
            tx.value = 800
            sim_sleep(sim, 3.0)
            rpm = rpm_from_state(sim)
            check(name + ' spins', 2000 <= rpm <= 4000, 'rpm=%.0f' % rpm)
            sim.set_stuck(1.0)                 # the prop hits the branch
            sim_sleep(sim, 3.0)
            rpm = rpm_from_state(sim, 0.5)
            check(name + ' stalls', abs(rpm) < 100, 'rpm=%.0f' % rpm)
            sim.set_stuck(0.0)                 # the obstruction clears
            sim_sleep(sim, 4.0)
            rpm = rpm_from_state(sim, 0.5)
            if protection:
                # latched off although the throttle is still raised
                check(name + ' stays off', abs(rpm) < 100, 'rpm=%.0f' % rpm)
                tx.value = 0                   # throttle cycle resets it
                sim_sleep(sim, 1.0)
                tx.value = 800
                sim_sleep(sim, 3.0)
                rpm = rpm_from_state(sim)
                check(name + ' recovers after throttle cycle',
                      2000 <= rpm <= 4000, 'rpm=%.0f' % rpm)
            else:
                check(name + ' restarts by itself', 2000 <= rpm <= 4000,
                      'rpm=%.0f' % rpm)
        finally:
            tx.stop()
            sim.close()


def test_dronecan_params(sitl_path):
    """DroneCAN parameter get/set round-trip through the firmware"""
    try:
        import dronecan
    except ImportError:
        print('SKIP: dronecan not installed, DroneCAN param test skipped')
        return
    with Sitl(sitl_path, ['--can-uri', 'mcast:4', '--node-id', '40']):
        sim = can_state_stream('dronecan param set')
        if sim is None:
            return
        sim.close()
        node = dronecan.make_node('mcast:4', node_id=101)
        try:
            time.sleep(2.0)
            got = {}

            def getset(req, key):
                done = {}
                def cb(e):
                    done['e'] = e
                node.request(req, 40, cb)
                t0 = time.time()
                while 'e' not in done and time.time() - t0 < 3:
                    try:
                        node.spin(0.05)
                    except Exception:
                        pass
                got[key] = done.get('e')

            # read TELEM_RATE, set it, read it back
            gp = dronecan.uavcan.protocol.param.GetSet.Request
            getset(gp(name=b'TELEM_RATE'), 'read')
            newval = dronecan.uavcan.protocol.param.Value(integer_value=77)
            getset(gp(name=b'TELEM_RATE', value=newval), 'set')
            getset(gp(name=b'TELEM_RATE'), 'reread')
            ok = (got['reread'] is not None and
                  got['reread'].response.value.integer_value == 77)
            check('dronecan param set', ok,
                  'reread=%s' % (got['reread'].response.value.integer_value
                                 if got['reread'] else None))
        finally:
            node.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    # don't hardcode the firmware version in the default binary path
    pat = os.path.join(HERE, '..', '..', 'obj', 'AM32_AM32_SITL_CAN_*.elf')
    hits = sorted(glob.glob(pat))
    ap.add_argument('--sitl', default=os.path.normpath(hits[0]) if hits else None)
    args = ap.parse_args()

    if not os.path.exists(args.sitl):
        print('SITL binary not found: %s' % args.sitl)
        sys.exit(2)

    test_dshot(args.sitl, 'dshot600 bidir edt', sd.TYPE_DSHOT600,
               bidir=True, edt=True, value=800, rpm_lo=2000, rpm_hi=4000)
    test_dshot(args.sitl, 'dshot300', sd.TYPE_DSHOT300,
               bidir=False, edt=False, value=600, rpm_lo=2000, rpm_hi=4500)
    test_dshot(args.sitl, 'pwm', sd.TYPE_PWM,
               bidir=False, edt=False, value=1500, rpm_lo=3000, rpm_hi=7000,
               input_type=2)
    test_startup_tune(args.sitl)
    test_beacon_tone(args.sitl)
    test_physics_audio(args.sitl)
    test_dronecan(args.sitl)
    test_dshot_direction(args.sitl)
    test_bidirectional(args.sitl)
    test_dshot_edt_toggle(args.sitl)
    test_stuck_rotor(args.sitl, protection=True)
    test_stuck_rotor(args.sitl, protection=False)
    test_debugger_pause(args.sitl)
    test_dronecan_params(args.sitl)

    if failures:
        print('\n%d FAILED: %s' % (len(failures), ', '.join(failures)))
        sys.exit(1)
    print('\nall tests passed')


if __name__ == '__main__':
    main()
