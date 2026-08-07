#!/usr/bin/env python3
'''
CI test for the SITL GUI: runs the real GUI under Qt's offscreen
platform, drives it through the control port (arming, EDT, throttle,
scopes, motor view) and asserts on the telemetry it reports.

usage: gui_ci_test.py --gui-python Mcu/SITL/venv/bin/python3
'''

import argparse
import glob
import json
import os
import random
import re
import socket
import tempfile
import subprocess
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sitl_params
from run_ci_tests import Sitl, check, failures, INPUT_PORT, STATE_PORT

HERE = os.path.dirname(os.path.abspath(__file__))


def free_control_port():
    '''pick an unused localhost port for the GUI control socket.

    It has to vary between runs - a fixed port fails on macOS while the
    previous control connection is still in TIME_WAIT - but must stay
    below the ephemeral range (macOS: 32768+). An ephemeral port would be
    handed straight back to one of the GUI's own bind(0) sockets, which
    open before its control server, so the control bind then fails with
    "address already in use" (seen consistently on macOS).
    '''
    for _ in range(50):
        port = random.randint(20000, 32000)
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                sock.bind(('127.0.0.1', port))
            except OSError:
                continue
        return port
    return random.randint(20000, 32000)


def test_launcher(args, env):
    '''start a second GUI with free simulator ports and launch SITL from
    its process panel. The main test attaches to an external SITL, so it
    cannot exercise this subprocess path.'''
    control_port = free_control_port()
    input_port = 29833
    state_port = 29834
    params = os.path.join(HERE, 'data', 'VIMDRONES_NANO_2216', 'sitl.param')

    with tempfile.TemporaryDirectory(prefix='am32_gui_launcher_') as tmp:
        eeprom = sitl_params.write_eeprom(params,
                                          os.path.join(tmp, 'eeprom.bin'))
        gui = subprocess.Popen(
            [args.gui_python, os.path.join(HERE, 'sitl_gui.py'),
             '--control-port', str(control_port),
             '--port', str(input_port), '--state-port', str(state_port),
             '--can-uri', 'mcast:8'],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, env=env)
        responses = []
        sock = None
        try:
            deadline = time.time() + 45
            while time.time() < deadline:
                try:
                    sock = socket.create_connection(
                        ('127.0.0.1', control_port), timeout=5)
                    break
                except OSError:
                    if gui.poll() is not None:
                        break
                    time.sleep(1.0)
            if sock is None:
                check('GUI launcher control port', False,
                      'GUI exit=%s' % gui.poll())
                return
            sock.settimeout(None)
            f = sock.makefile('r')

            def reader():
                try:
                    for line in f:
                        responses.append(line.rstrip())
                except OSError:
                    pass

            threading.Thread(target=reader, daemon=True).start()

            def send(command, delay=0.2):
                sock.sendall((command + '\n').encode())
                time.sleep(delay)

            send('sim_set binary %s' % args.sitl)
            send('sim_set eeprom %s' % eeprom)
            send('sim_input dshot')
            send('sim_start')
            # Feed zero throttle before the firmware's no-signal reset,
            # then leave it up long enough to cross an emulated reboot.
            send('ds_type dshot600', 0.1)
            send('ds_bidir 1', 0.1)
            send('ds_enable 1', 4.0)
            send('sim_status')
            send('sim_log')
            send('sim_stop')
            send('quit')
            gui.wait(timeout=20)
        except (OSError, subprocess.TimeoutExpired) as ex:
            check('GUI launcher exits cleanly', False, str(ex))
        finally:
            if sock is not None:
                sock.close()
            if gui.poll() is None:
                gui.kill()
                gui.wait()

        running = any('sim_process: running' in r for r in responses)
        log = ' '.join(r for r in responses if r.startswith('STATUS sim_log:'))
        check('launcher GUI exits cleanly', gui.returncode == 0,
              'exit=%s' % gui.returncode)
        check('GUI launches SITL', running,
              '; '.join(r for r in responses if 'sim_process:' in r)
              or 'no status')
        check('GUI-launched SITL initialises CAN',
              'SITL: CAN on mcast:8' in log, log[-300:] or 'no log')
        check('GUI-launched SITL stays alive',
              'simulator exited with status' not in log,
              log[-300:] or 'clean')
        out = gui.stdout.read() if gui.stdout else ''
        check('launcher GUI no tracebacks', 'Traceback' not in out,
              out[-300:] if 'Traceback' in out else 'clean')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--gui-python', required=True)
    # don't hardcode the firmware version in the default binary path
    pat = os.path.join(HERE, '..', '..', 'obj', 'AM32_AM32_SITL_CAN_*.elf')
    hits = sorted(glob.glob(pat))
    ap.add_argument('--sitl', default=os.path.normpath(hits[0]) if hits else None)
    args = ap.parse_args()

    env = dict(os.environ)
    env['QT_QPA_PLATFORM'] = 'offscreen'
    control_port = free_control_port()

    with Sitl(args.sitl, ['--can-uri', 'none', '--input-type', '1']):
        gui = subprocess.Popen(
            [args.gui_python, os.path.join(HERE, 'sitl_gui.py'),
             '--control-port', str(control_port),
             '--port', str(INPUT_PORT), '--state-port', str(STATE_PORT),
             '--can-uri', 'mcast:7'],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env)

        # first launch can be slow (Qt font cache); retry the connection
        responses = []
        s = None
        deadline = time.time() + 45
        while time.time() < deadline:
            try:
                s = socket.create_connection(('127.0.0.1', control_port),
                                             timeout=5)
                break
            except OSError:
                if gui.poll() is not None:
                    print(gui.stdout.read() if gui.stdout else '')
                    print('gui exited before control port came up')
                    sys.exit(1)
                time.sleep(1.0)
        if s is None:
            gui.kill()
            print('control port never came up')
            sys.exit(1)
        # create_connection applies its startup timeout to later reads too;
        # the command channel should remain blocking once it is connected
        s.settimeout(None)
        f = s.makefile('r')

        def reader():
            for line in f:
                responses.append(line.rstrip())

        threading.Thread(target=reader, daemon=True).start()

        def send(cmd):
            s.sendall((cmd + '\n').encode())

        for delay, cmd in [
                (0.1, 'ds_type dshot600'), (0.1, 'ds_bidir 1'),
                (0.1, 'ds_enable 1'), (0.3, 'ds_edt 1')]:
            time.sleep(delay)
            send(cmd)

        # the backend re-sends the EDT enable while the motor is stopped,
        # but the firmware ignores it once spinning: wait for the latch
        # before throttling up so a slow runner can't lose the race
        deadline = time.time() + 20
        while time.time() < deadline:
            time.sleep(1.0)
            send('status')
            if any('EDT:on' in r for r in responses):
                break

        model_path = os.path.join(tempfile.gettempdir(), 'gui_ci_model.json')
        if os.path.exists(model_path):
            os.remove(model_path)
        for delay, cmd in [
                (0.5, 'ds_value 900'),
                (1.0, 'graph_i 1'), (0.2, 'graph_v 1'), (0.2, 'motorview 1'),
                (0.2, 'rpm_graph 1'),
                (0.2, 'audio 1'), (0.2, 'motor_audio 1'),
                (4.0, 'status'),
                (0.3, 'wave sine 1.0 0.2 0.4'), (1.5, 'wave off'),
                # the same waveform generator targeting the CAN throttle
                (0.2, 'can_enable 1'),
                (0.3, 'wave square 1.0 0.2 0.4 can'), (1.0, 'wave off'),
                (0.3, 'stuck 1.0'), (0.5, 'stuck 0'),
                # model builder: create a model from a spec sheet
                (0.2, 'model_new %s size=2216 kv=920 poles=14 idle=0.8@10 '
                      'weight=64 prop=9x4.5 cells=4 voltage=12.4' % model_path),
                # the SITL launcher's control commands (the external SITL
                # already owns the ports, so check they are wired, not a
                # second launch)
                (0.2, 'sim_set binary %s' % (args.sitl or '')),
                (0.2, 'sim_verbose 1'), (0.3, 'sim_status'),
                (1.0, 'quit')]:
            time.sleep(delay)
            send(cmd)

        try:
            gui.wait(timeout=20)
            check('gui exits cleanly', gui.returncode == 0,
                  'exit=%s' % gui.returncode)
        except subprocess.TimeoutExpired:
            gui.kill()
            check('gui exits cleanly', False, 'hung')

        for c in ('stuck 1.0', 'stuck 0'):
            check('gui %s accepted' % c, ('OK %s' % c) in responses,
                  '; '.join(r for r in responses if 'stuck' in r) or 'no reply')
        bds = [r for r in responses if r.startswith('STATUS BDShot')]
        check('gui got BDShot status', len(bds) > 0, '%d lines' % len(bds))
        if bds:
            m = re.search(r'rpm=(\d+)\s+(\w+)\s+EDT:(\w+)', bds[-1])
            check('gui status parses', m is not None, bds[-1])
            if m:
                rpm, spin, edt = int(m.group(1)), m.group(2), m.group(3)
                check('gui rpm', 4000 <= rpm <= 7000, 'rpm=%d' % rpm)
                check('gui spinning', spin == 'spinning', spin)
                check('gui edt on', edt == 'on', 'EDT:%s' % edt)
        # model builder: it should have written a valid model with the
        # spec's Kv, and the SITL launcher commands should be answered
        model_ok = os.path.exists(model_path)
        check('model builder wrote a model', model_ok, model_path)
        if model_ok:
            m = json.load(open(model_path))
            check('built model took Kv from the spec',
                  m.get('motor', {}).get('kv') == 920,
                  str(m.get('motor', {}).get('kv')))
            check('built model has the full schema',
                  set(m) == {'motor', 'battery', 'esc', 'sim'}, str(set(m)))
        check('launcher command accepted', 'OK sim_verbose 1' in responses,
              '; '.join(r for r in responses if 'sim_verbose' in r) or 'no reply')
        check('CAN throttle waveform accepted',
              'OK wave square 1.0 0.2 0.4 can' in responses,
              '; '.join(r for r in responses if 'wave' in r and 'can' in r)
              or 'no reply')
        check('launcher reports stopped',
              any('sim_process: stopped' in r for r in responses),
              '; '.join(r for r in responses if 'sim_process' in r) or 'none')

        out = gui.stdout.read() if gui.stdout else ''
        check('gui no tracebacks', 'Traceback' not in out,
              (out[-300:] if 'Traceback' in out else 'clean'))

    test_launcher(args, env)

    if failures:
        print('\n%d FAILED' % len(failures))
        sys.exit(1)
    print('\ngui test passed')


if __name__ == '__main__':
    main()
