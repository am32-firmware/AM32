#!/usr/bin/env python3
'''
headless test of the calibration capture GUI

Starts the UI (from source, or a packaged binary with --binary) and
drives it over its control socket exactly as a user would, then checks
the data set it writes. No ESC or flight controller needed: the parts
that need hardware are skipped, everything else is real.

Given --sitl, it also drives a real capture against the SITL ESC over
DroneCAN, which is the only way to cover the parts that need a live
motor - notably that Abort stops a run in flight.

  capture_gui_test.py
  capture_gui_test.py --binary dist/am32-capture
  capture_gui_test.py --sitl obj/AM32_AM32_SITL_CAN_2.20.elf
'''

import argparse
import json
import os
import threading
import socket
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

sys.path.insert(0, os.path.join(ROOT, 'Mcu', 'SITL'))
import sitl_params

# below the macOS ephemeral range (32768+): an ephemeral control port
# gets handed to one of the GUI's own bind(0) sockets before its control
# server binds it, so the bind then fails there
PORT = 28123
SITL_URI = 'mcast:9'
SITL_NODE = 14

failures = []


def check(name, ok, detail=''):
    print('%s: %s%s' % ('PASS' if ok else 'FAIL', name,
                        ' (%s)' % detail if detail else ''))
    sys.stdout.flush()
    if not ok:
        failures.append(name)


class Control(object):
    def __init__(self, port, timeout=30):
        deadline = time.time() + timeout
        last = None
        while time.time() < deadline:
            try:
                self.sock = socket.create_connection(('127.0.0.1', port), timeout=5)
                self.f = self.sock.makefile('r')
                return
            except OSError as ex:
                last = ex
                time.sleep(0.5)
        raise SystemExit('control port never opened: %s' % last)

    def cmd(self, line):
        self.sock.sendall((line + '\n').encode())
        return self.f.readline().strip()


def plot_checks():
    """the built-in plot must work where pyqtgraph cannot: it needs
    PySide6.QtOpenGL, which several distributions package separately"""
    os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
    try:
        from PySide6.QtWidgets import QApplication
        from PySide6.QtGui import QPixmap
    except ImportError as ex:
        check('PySide6 available for the plot check', False, str(ex))
        return
    app = QApplication.instance() or QApplication([])
    sys.path.insert(0, HERE)
    import esc_capture_gui as gui

    xs = [i * 0.01 for i in range(1000)]
    ys = [abs(500 * (i % 200) - 5000) for i in range(1000)]
    w = gui.SimplePlot('sweep', xs, ys)
    w.resize(600, 220)
    pm = QPixmap(600, 220)
    w.render(pm)
    img = pm.toImage()
    seen = set()
    for x in range(0, 600, 3):
        for y in range(0, 220, 3):
            seen.add(img.pixel(x, y))
    check('the built-in plot draws the data', len(seen) > 3,
          '%u distinct colours' % len(seen))
    # a fast oscillation must be drawn as its envelope. Plain
    # decimation aliases it into a moire pattern, which is what a
    # 120s chirp at 200 Hz into ~900 pixels looks like when each
    # column shows one sample instead of that column's range
    import math
    n = 40000
    fast_x = [i * 0.005 for i in range(n)]
    fast_y = [10000 + 9000 * math.sin(i * 0.7) for i in range(n)]
    w = gui.SimplePlot('chirp', fast_x, fast_y)
    w.resize(600, 200)
    pm = QPixmap(600, 200)
    w.render(pm)
    img = pm.toImage()
    bg = img.pixel(3, 190)
    # the amplitude is constant, so the top of the trace must be flat.
    # Aliasing shows up precisely as that edge wandering
    tops = []
    for x in range(150, 450):
        rows = [y for y in range(25, 185) if img.pixel(x, y) != bg]
        if rows:
            tops.append(min(rows))
    wander = (max(tops) - min(tops)) if tops else 999
    check('a fast oscillation is drawn as its envelope, not aliased',
          len(tops) > 250 and wander <= 8,
          '%u columns drawn, envelope wanders %u px' % (len(tops), wander))

    # and it must not divide by zero on a flat or single-point capture
    for name, (px, py) in (('flat', ([0.0, 1.0], [5.0, 5.0])),
                           ('one point', ([0.0], [0.0])),
                           ('empty', ([], []))):
        try:
            p = gui.SimplePlot(name, px, py)
            p.resize(200, 80)
            p.render(QPixmap(200, 80))
            ok, detail = True, ''
        except BaseException as ex:
            ok, detail = False, '%s: %s' % (type(ex).__name__, ex)
        check('the built-in plot survives a %s capture' % name, ok, detail)
    app.processEvents()


def session_checks():
    """things the GUI cannot be driven into reliably: the abort races,
    which wall-clock timing does not hit, and the setup mistakes that
    have to fail with a message rather than an errno.

    Driven directly against Session, so the interleavings are chosen
    rather than hoped for."""
    sys.path.insert(0, HERE)
    import capture_session as cs

    # every one of these used to surface as a bare OSError from deep
    # inside the connect, naming nothing the user could act on
    for field, value, want in (
            ('directory', '', 'output directory'),
            ('uri', '', 'CAN interface'),
            ('uri', 'mavcan:', 'CAN interface')):
        cfg = cs.Config('dronecan', None, 'mcast:0', 1, 123, 14,
                        tempfile.mkdtemp(prefix='am32-cfg-'),
                        5.0, 0.6, 120.0, {})
        setattr(cfg, field, value)
        try:
            cs.Session(cfg, lambda m: None).connect()
            ok, detail = False, 'no error raised'
        except RuntimeError as ex:
            ok, detail = want in str(ex), str(ex)[:70]
        except BaseException as ex:
            ok, detail = False, '%s: %s' % (type(ex).__name__, ex)
        check('empty %s is reported clearly' % field, ok, detail)

    class FakeBackend(object):
        """the slice of the backend duck interface profile_hold uses"""
        def __init__(self):
            self.aborted = None
            self.throttle = 0.0
            self.max_throttle = 0.0
            self.rec = self.node = None

        def _check(self):
            if self.aborted:
                raise SystemExit('aborted: %s' % self.aborted)

        def wait_ready(self):
            self._check()

        def mark(self, _m):
            self._check()

        def set_throttle(self, t):
            self._check()
            self.throttle = t
            self.max_throttle = max(self.max_throttle, t)

        def spin_for(self, _d):
            self._check()

    outdir = tempfile.mkdtemp(prefix='am32-capture-race-')
    cfg = cs.Config('dronecan', None, 'mcast:0', 1, 123, 14, outdir,
                    5.0, 0.6, 120.0, {})
    session = cs.Session(cfg, lambda m: None)

    building = threading.Event()
    release = threading.Event()
    made = []

    def slow_backend(_args):
        building.set()
        release.wait(10)
        b = FakeBackend()
        made.append(b)
        return b

    session._backend = slow_backend
    err = []

    def work():
        try:
            session.run_step('first_spin')
        except BaseException as ex:
            err.append(ex)

    t = threading.Thread(target=work, daemon=True)
    t.start()
    # run_step redirects the process's stdout for the duration of a
    # step, so nothing printed here would be seen; check after the join
    built = building.wait(10)
    # abort while there is no backend to set the flag on yet
    session.abort()
    release.set()
    t.join(20)
    check('backend construction reached', built)
    check('run_step returned', not t.is_alive())
    check('abort during construction stops the step',
          bool(err) and isinstance(err[0], SystemExit), repr(err[:1]))
    check('abort during construction never spun the motor',
          bool(made) and made[0].max_throttle == 0.0,
          'max throttle %r' % (made[0].max_throttle if made else 'n/a'))

    # and the shutdown must detach the backend, so an abort arriving
    # afterwards cannot re-arm it behind the wind-down
    check('backend detached after the step', session.backend is None)
    check('aborted cleared on the way out',
          bool(made) and made[0].aborted is None)
    session.abort()
    check('a late abort cannot re-arm the finished backend',
          bool(made) and made[0].aborted is None)


def wait_idle(c, timeout):
    end = time.time() + timeout
    while time.time() < end:
        if c.cmd('busy') == 'busy 0':
            return True
        time.sleep(0.3)
    return False


def sitl_checks(c, sitl_bin):
    '''the parts that need a live ESC, against the simulated one'''
    outdir = tempfile.mkdtemp(prefix='am32-capture-sitl-')
    eeprom = sitl_params.write_eeprom(
        os.path.join(ROOT, 'Mcu/SITL/data/SEQURE_G431/sitl.param'),
        os.path.join(outdir, 'sitl_eeprom.bin'))
    proc = subprocess.Popen(
        [os.path.abspath(sitl_bin), '--node-id', str(SITL_NODE),
         '--can-uri', SITL_URI, '--eeprom', eeprom,
         '--config', os.path.join(ROOT, 'Mcu/SITL/models/sequre_gt2215.json'),
         '--input-port', '0', '--state-port', '0', '--nosleep'],
        stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT,
        start_new_session=True)
    time.sleep(2)
    try:
        for line in ('uri ' + SITL_URI, 'node %u' % SITL_NODE,
                     'dir ' + outdir, 'max_current 60'):
            c.cmd(line)
        check('connect to SITL', c.cmd('connect').startswith('OK'))
        check('connect finishes', wait_idle(c, 30))
        check('SITL node seen', 'nodes seen: %u' % SITL_NODE in c.cmd('log'))

        check('capture starts', c.cmd('run sweep').startswith('OK'))
        time.sleep(6.0)
        check('capture is running', c.cmd('busy') == 'busy 1')

        # the point of this section: Abort must stop a run in flight,
        # not merely at the next step boundary
        t0 = time.time()
        c.cmd('abort')
        stopped = wait_idle(c, 25)
        took = time.time() - t0
        check('abort stops the capture', stopped, '%.1fs' % took)
        check('abort is prompt', stopped and took < 15, '%.1fs' % took)
        log = c.cmd('log')
        check('abort is reported as an abort, not a failure',
              'aborted' in log.lower() and 'ABORTED' in log, log[-120:])

        # a second capture must not start behind a running one: two
        # workers would command the motor through the same backend
        check('capture starts', c.cmd('run sweep').startswith('OK'))
        time.sleep(3.0)
        c.cmd('run first_spin')
        time.sleep(1.0)
        log = c.cmd('log')
        check('a second capture is refused while one is running',
              'busy' in log.lower() and 'first_spin ->' not in log[-400:],
              log[-120:])
        c.cmd('abort')
        check('abort after the refusal', wait_idle(c, 25))

        # an abort issued straight after the start; the construction
        # window itself is covered deterministically in race_checks(),
        # which timing alone cannot hit reliably
        c.cmd('run sweep')
        time.sleep(0.15)
        c.cmd('abort')
        stopped = wait_idle(c, 30)
        check('an early abort leaves the session usable', stopped)
        check('an early abort runs no capture',
              '== sweep ok ==' not in c.cmd('log'), c.cmd('log')[-120:])

        # and the session must survive it
        check('capture runs again after an abort',
              c.cmd('run first_spin').startswith('OK'))
        check('rerun finishes', wait_idle(c, 90))
        check('rerun captured data', 'first_spin complete' in c.cmd('log'))
        check('rerun wrote a log',
              os.path.getsize(os.path.join(outdir, 'first_spin_010.jsonl')) > 0)
    finally:
        proc.terminate()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--binary', default=None,
                    help='packaged executable to test instead of the source')
    ap.add_argument('--sitl', default=None,
                    help='SITL binary, to also test capture and abort '
                         'against a live simulated ESC')
    args = ap.parse_args()

    env = dict(os.environ)
    env['QT_QPA_PLATFORM'] = 'offscreen'
    if args.binary:
        cmd = [os.path.abspath(args.binary), '--control-port', str(PORT)]
    else:
        cmd = [sys.executable, os.path.join(HERE, 'esc_capture_gui.py'),
               '--control-port', str(PORT)]
    if not args.binary:
        session_checks()
        plot_checks()

    proc = subprocess.Popen(cmd, env=env, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True)
    outdir = tempfile.mkdtemp(prefix='am32-capture-test-')
    try:
        c = Control(PORT)
        check('control port answers', c.cmd('status').startswith('status'))

        # the setup a user fills in
        for line in ('transport dronecan', 'poles 14', 'node 123',
                     'uri mcast:0', 'dir ' + outdir,
                     'meta esc TestESC G431', 'meta motor Test 1404 4300kv',
                     'meta prop none', 'meta power bench 12V',
                     'meta fc none', 'max_current 5', 'chirp_secs 30'):
            r = c.cmd(line)
            if not r.startswith('OK'):
                check('command %r' % line, False, r)
        check('setup accepted', True)

        check('idle after setup', c.cmd('busy') == 'busy 0')

        # a prompt must stop the worker until it is answered: the
        # capture used to carry on while the user was still being told
        # to power cycle the ESC
        c.cmd('testprompt Power cycle|do it now')
        time.sleep(1.0)
        check('a prompt is raised', c.cmd('prompt').startswith('prompt Power cycle'),
              c.cmd('prompt'))
        check('the worker waits for the prompt', c.cmd('busy') == 'busy 1')
        c.cmd('ack')
        end = time.time() + 10
        while time.time() < end and c.cmd('busy') != 'busy 0':
            time.sleep(0.2)
        check('answering the prompt releases the worker',
              c.cmd('busy') == 'busy 0')
        check('the prompt is gone', c.cmd('prompt') == 'prompt')


        # a capture cannot run without hardware, but the tool must say
        # so rather than hang or crash
        c.cmd('run first_spin')
        time.sleep(1.0)
        log = c.cmd('log')
        check('capture without hardware is reported',
              'connect' in log.lower() or 'fail' in log.lower(),
              log[:80])

        # export works with no captures and must still produce the
        # descriptive files
        check('export accepted', c.cmd('export').startswith('OK'))
        time.sleep(1.0)
        for name in ('README.md', 'expected.json', 'model.json'):
            check('wrote %s' % name, os.path.exists(os.path.join(outdir, name)))

        readme = open(os.path.join(outdir, 'README.md')).read()
        check('README describes the rig', 'TestESC G431' in readme
              and 'Test 1404 4300kv' in readme)
        model = json.load(open(os.path.join(outdir, 'model.json')))
        check('model has the motor section',
              model.get('motor', {}).get('poles') == 14,
              json.dumps(model.get('motor', {}))[:80])
        exp = json.load(open(os.path.join(outdir, 'expected.json')))
        check('expected.json is valid json', isinstance(exp, dict))

        if args.sitl:
            sitl_checks(c, args.sitl)

        check('still responsive at the end', c.cmd('status').startswith('status'))
        c.cmd('quit')
    finally:
        try:
            proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            proc.kill()
        out = proc.stdout.read() if proc.stdout else ''
        if 'Traceback' in out:
            check('no tracebacks', False, out[-300:])
        else:
            check('no tracebacks', True)

    if failures:
        print('\n%u FAILED: %s' % (len(failures), ', '.join(failures)))
        sys.exit(1)
    print('\nall capture GUI tests passed')


if __name__ == '__main__':
    main()
