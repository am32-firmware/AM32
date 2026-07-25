#!/usr/bin/env python3
'''
SITL calibration regression tests: run the bench measurement battery
(steady sweep, square amplitude sweep, 120s chirp) against each
calibrated model and compare the results to the REAL hardware
reference values in Mcu/SITL/data/<TARGET>/expected.json.

This regression-proofs both the physics code and the models: a change
that moves SITL away from the bench data fails here.

Designed for CI:
- tolerances in expected.json cover the documented model residuals
  plus run-to-run scatter (measured on an idle host)
- every measurement is paced against SITL SIMULATION time (the tools'
  --sim-state clock), so results are independent of how fast the
  machine runs the simulation: a slow runner takes longer in wall
  time but measures the same experiment. The sim/wall ratio is
  reported as a performance diagnostic only
- desyncs are compared against a per-model budget, never exact
  counts: the 1404 model reproduces the real hardware's marginality
  by design, and chirp cold starts retry
- comparison graphs (real vs this run) are written to --artifacts
  for human review

usage:
  run_calibration_tests.py --sitl ../obj/AM32_AM32_SITL_CAN_*.elf
  run_calibration_tests.py --dataset vim1404 --skip-chirp
'''

import argparse
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
SCRIPTS = os.path.join(REPO, 'scripts')

DATASETS = [
    {
        'name': 'gt2215',
        'data': os.path.join(HERE, 'data/SEQURE_G431'),
        'model': os.path.join(HERE, 'models/sequre_gt2215.json'),
        'real_sweep': 'sweep1.jsonl',
        'real_square': 'square1.jsonl',
        'real_chirp': 'sequre_chirp_120s.jsonl',
        'max_current': 8.0,
        'chirp_max_freq': 15,
    },
    {
        'name': 'nano2216',
        'data': os.path.join(HERE, 'data/VIMDRONES_NANO_2216'),
        'model': os.path.join(HERE, 'models/vimdrones_nano_2216.json'),
        'real_sweep': 'sweep2.jsonl',
        'real_square': 'square1.jsonl',
        'real_chirp': 'nano_chirp_120s.jsonl',
        # the model's limiter-engagement transient peaks ~11A at the
        # 0.8->0.9 step (real bench peaked 9.4A); 12A still trips on a
        # runaway while clearing the documented residual
        'max_current': 12.0,
        'chirp_max_freq': 20,
        # prop-inertia rotor: SITL cold starts at mid throttle are
        # marginal, so sweeps lead with 0.3 and dynamic tests pre-spin
        'start_level': '0.3',
        'prespin': True,
    },
    {
        'name': 'vim1404',
        'data': os.path.join(HERE, 'data/VIMDRONES_L431'),
        'model': os.path.join(HERE, 'models/vimdrones_1404.json'),
        'real_sweep': 'sweep1.jsonl',
        'real_square': 'square2.jsonl',
        'real_chirp': 'vim_chirp_120s_v2.jsonl',
        'max_current': 4.0,
        'chirp_max_freq': 30,
    },
]

# no interface suffix here: stock PyPI pydronecan (what CI installs)
# splits the URI on ':' and silently falls back to bus 0 when an
# interface is present, leaving the tools and the SITL on different
# multicast groups
URI = 'mcast:7'
NODE_ID = 30
INPUT_PORT = 57943
STATE_PORT = 57944
# every measurement is paced against SITL simulation time, so the
# results are identical whatever sim/wall ratio the machine sustains
SIM_STATE = ['--sim-state', '127.0.0.1:%d' % STATE_PORT]

# outer failsafe only: simulated durations take 1/ratio wall seconds on
# a slow machine, and the sim clock aborts on its own if the simulation
# stops advancing
WALL_TIMEOUT = 3600

results = []


def report(dataset, name, ok, detail, skipped=False):
    tag = 'SKIP' if skipped else ('PASS' if ok else 'FAIL')
    print('%s: %s %s (%s)' % (tag, dataset, name, detail), flush=True)
    results.append((dataset, name, tag))


def load_status(path):
    rows = []
    for line in open(path):
        try:
            r = json.loads(line)
        except ValueError:
            continue
        rows.append(r)
    return rows


class Sitl:
    def __init__(self, sitl, model, eeprom, log, physics_log=None):
        self.log = log
        self.logf = open(log, 'w')
        # --nosleep: the per-loop pacing nanosleep collapses the sim
        # ratio to ~1% under CI-runner core contention (wakeup latency
        # dwarfs the 4us loop time); the busy-wait keeps it at 1.0
        cmd = [sitl, '--node-id', str(NODE_ID), '--can-uri', URI,
               '--eeprom', eeprom, '--config', model,
               '--input-port', str(INPUT_PORT),
               '--state-port', str(STATE_PORT), '--verbose', '--nosleep']
        if physics_log:
            cmd += ['--physics-log', physics_log]
        self.proc = subprocess.Popen(
            cmd, stdout=self.logf, stderr=subprocess.STDOUT,
            start_new_session=True)
        time.sleep(2)

    def stop(self):
        try:
            os.killpg(self.proc.pid, signal.SIGTERM)
        except OSError:
            pass
        self.proc.wait(timeout=10)
        self.logf.close()

    def min_ratio(self):
        '''minimum sim/wall ratio seen in the verbose output'''
        ratios = []
        for line in open(self.log):
            m = re.match(r'SITL t=\S+ x([0-9.]+) ', line)
            if m:
                ratios.append(float(m.group(1)))
        return min(ratios) if ratios else 0.0


def run_tool(args, timeout):
    return subprocess.run(args, cwd=REPO, timeout=timeout,
                          capture_output=True, text=True)


def steady_test(ds, exp, out):
    levels = sorted(exp['steady']['levels'])
    # a coasting-prop catch at the lowest level is marginal by design
    # (real cold-start marginality), so retry like the square test
    for attempt in range(exp.get('sweep_start_retries', 2) + 1):
        # prespin separately so the sweep's level sequence matches the
        # real capture exactly (comparison plots align step for step)
        prespin(ds)
        run_tool([sys.executable, os.path.join(SCRIPTS, 'esc_measure.py'),
                  '--uri', URI, '--node-id', str(NODE_ID), 'sweep',
                  '--arm-time', '0.5',
                  '--levels', (','.join(levels) if ds.get('prespin')
                               else ds.get('start_level', '0.1') + ',' + ','.join(levels)),
                  '--hold', '4', '--max-current', str(ds['max_current']),
                  '--max-throttle', '1.0',
                  '--log', out] + SIM_STATE, timeout=WALL_TIMEOUT)
        if started(out) and sweep_complete(out):
            break
        print('  sweep attempt %d incomplete, retrying' % (attempt + 1),
              flush=True)
    rows = load_status(out)
    cmds = [(x['t'], x['throttle']) for x in rows if x['type'] == 'cmd']
    st = [(x['t'], x['rpm']) for x in rows if x['type'] == 'status']
    ok_all = True
    details = []
    for lvl, want in exp['steady']['levels'].items():
        thr = float(lvl)
        vals = []
        for i, (t0, cthr) in enumerate(cmds):
            if abs(cthr - thr) > 1e-6:
                continue
            t1 = cmds[i + 1][0] if i + 1 < len(cmds) else t0 + 4
            vals += [v for t, v in st if t0 + (t1 - t0) * 0.4 < t < t1 and v > 0]
        if not vals:
            ok_all = False
            details.append('%s:no-data' % lvl)
            continue
        got = sum(vals) / len(vals)
        dev = 100.0 * (got - want) / want
        if abs(dev) > exp['steady']['tolerance_pct']:
            ok_all = False
        details.append('%s:%+.1f%%' % (lvl, dev))
    report(ds['name'], 'steady sweep', ok_all, ' '.join(details))
    return rows


def prespin(ds):
    '''spin the motor up briefly so the following tool catches a
    still-rotating prop instead of a marginal cold start'''
    if not ds.get('prespin'):
        return
    run_tool([sys.executable, os.path.join(SCRIPTS, 'esc_measure.py'),
              '--uri', URI, '--node-id', str(NODE_ID), 'hold',
              '--throttle', '0.3', '--hold', '5',
              '--max-current', str(ds['max_current']),
              '--log', os.devnull] + SIM_STATE, timeout=WALL_TIMEOUT)


def started(out):
    st = [x for x in load_status(out) if x['type'] == 'status']
    return max((x['rpm'] for x in st), default=0) > 2000


def sweep_complete(out):
    '''the sweep profile ends by commanding zero throttle; a run the
    sim clock aborted mid-profile never gets there'''
    return any(x['type'] == 'cmd' and x['throttle'] == 0
               for x in load_status(out))


def square_test(ds, exp, out):
    for attempt in range(exp.get('square_start_retries', 0) + 1):
        prespin(ds)
        run_tool([sys.executable, os.path.join(SCRIPTS, 'esc_square.py'), 'run',
                  '--uri', URI, '--node-id', str(NODE_ID), '--cycles', '2',
                  '--arm-time', '0.5',
                  '--max-current', str(ds['max_current']), '--log', out]
                 + SIM_STATE, timeout=WALL_TIMEOUT)
        if started(out):
            break
        print('  square start attempt %d failed, retrying' % (attempt + 1),
              flush=True)
    r = run_tool([sys.executable, os.path.join(SCRIPTS, 'esc_square.py'),
                  'analyze', out], timeout=120)
    got = {}
    up = []
    for line in r.stdout.splitlines():
        m = re.match(r'\s+([0-9.]+)\s+(\d+)\s+(\d+)', line)
        if m:
            got['%.3f' % float(m.group(1))] = int(m.group(3))
            up.append(int(m.group(2)))
    ok_all = True
    details = []
    sq = exp['square']
    for delta, want in sq['down_ms'].items():
        if delta not in got:
            ok_all = False
            details.append('%s:missing' % delta)
            continue
        err = got[delta] - want
        tol = max(sq['down_tolerance_ms'], want * sq['down_tolerance_pct'] / 100.0)
        if abs(err) > tol:
            ok_all = False
        details.append('%s:%+dms' % (delta, err))
    lo, hi = sq['up_flat_ms']
    if up and not all(lo <= u <= hi for u in up):
        ok_all = False
        details.append('up-range:%d..%d' % (min(up), max(up)))
    report(ds['name'], 'square down-curve', ok_all, ' '.join(details))


def chirp_test(ds, exp, sitl_args, out, raw):
    '''sitl_args = (binary, model, eeprom, log): the instance restarts
    per retry so the physics log gets a fresh monotonic segment'''
    retries = exp.get('chirp_start_retries', 2)
    err = 10 ** 9
    sitl = None
    for attempt in range(retries + 1):
        if sitl:
            sitl.stop()
        sitl = Sitl(*sitl_args, physics_log=raw)
        prespin(ds)
        run_tool([sys.executable, os.path.join(SCRIPTS, 'esc_chirp.py'), 'run',
                  '--uri', URI, '--node-id', str(NODE_ID),
                  '--duration', '120', '--f-start', '0.5', '--f-stop', '40',
                  '--arm-time', '0.5',
                  '--max-current', str(ds['max_current']), '--log', out]
                 + SIM_STATE, timeout=WALL_TIMEOUT)
        st = [x for x in load_status(out) if x['type'] == 'status']
        err = max((x['err'] for x in st), default=10 ** 9)
        if err <= exp['desync_budget'] and started(out):
            break
        print('  chirp attempt %d: err=%d, retrying' % (attempt + 1, err),
              flush=True)
    sitl.stop()
    ok_err = err <= exp['desync_budget']
    report(ds['name'], 'chirp desync budget', ok_err,
           'err=%d budget=%d' % (err, exp['desync_budget']))

    env = dict(os.environ, MPLBACKEND='Agg')
    r = subprocess.run([sys.executable, os.path.join(SCRIPTS, 'esc_chirp.py'),
                        'fit', out, '--raw', raw,
                        '--max-freq', str(ds['chirp_max_freq'])],
                       cwd=REPO, timeout=400, capture_output=True,
                       text=True, env=env)
    if r.returncode != 0:
        print('  fit exited %d:\n%s' % (r.returncode, r.stderr.strip()),
              flush=True)
    accel = brake = None
    for line in r.stdout.splitlines():
        m = re.search(r'accelerating.*?([0-9.]+) Hz', line)
        if m:
            accel = float(m.group(1))
        m = re.search(r'braking.*?([0-9.]+) Hz', line)
        if m:
            brake = float(m.group(1))
    ch = exp['chirp']
    ok = r.returncode == 0 and accel is not None and brake is not None
    detail = 'accel=%s (real %.1f) brake=%s (real %.2f)' % (
        accel, ch['accel_3db_hz'], brake, ch['brake_3db_hz'])
    if ok:
        tol = ch['tolerance_pct'] / 100.0
        ok = (abs(accel - ch['accel_3db_hz']) <= ch['accel_3db_hz'] * tol and
              abs(brake - ch['brake_3db_hz']) <= ch['brake_3db_hz'] * tol)
    report(ds['name'], 'chirp 3dB', ok, detail)


def artifacts(ds, run_dir, art_dir):
    '''real-vs-this-run comparison pages for human review'''
    try:
        import plotly  # noqa: F401
    except ImportError:
        print('plotly not installed, skipping artifact graphs', flush=True)
        return
    os.makedirs(art_dir, exist_ok=True)
    pairs = [
        ('sweep', os.path.join(ds['data'], ds['real_sweep']), 'sweep.jsonl', []),
        ('square', os.path.join(ds['data'], ds['real_square']), 'square.jsonl', []),
        ('square_steps', os.path.join(ds['data'], ds['real_square']), 'square.jsonl', ['--steps']),
        ('chirp', os.path.join(ds['data'], ds['real_chirp']), 'chirp.jsonl', []),
    ]
    for tag, real, sim, extra in pairs:
        simpath = os.path.join(run_dir, sim)
        if not (os.path.exists(real) and os.path.exists(simpath)):
            continue
        subprocess.run([sys.executable, os.path.join(SCRIPTS, 'esc_plot.py'),
                        real, simpath, '--html',
                        os.path.join(art_dir, '%s_%s.html' % (ds['name'], tag))]
                       + extra,
                       cwd=REPO, timeout=300, capture_output=True)


def main():
    global ARGS
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sitl', default=None, help='SITL binary')
    parser.add_argument('--dataset', default=None, help='run only this dataset')
    parser.add_argument('--artifacts', default='calibration_artifacts')
    parser.add_argument('--skip-chirp', action='store_true')
    parser.add_argument('--min-ratio', type=float, default=0.0,
                        help='fail if the sim/wall ratio falls below this '
                             '(0 = report only; measurements are sim-paced '
                             'so a slow machine is not an error)')
    ARGS = parser.parse_args()
    sitl_bin = os.path.abspath(ARGS.sitl) if ARGS.sitl else None
    if not sitl_bin:
        import glob
        cands = glob.glob(os.path.join(REPO, 'obj/AM32_AM32_SITL_CAN_*.elf'))
        if not cands:
            raise SystemExit('no SITL binary, pass --sitl')
        sitl_bin = cands[0]

    last_ratio = [0.0]
    for ds in DATASETS:
        if ARGS.dataset and ds['name'] != ARGS.dataset:
            continue
        exp = json.load(open(os.path.join(ds['data'], 'expected.json')))
        run_dir = os.path.abspath('calrun_%s' % ds['name'])
        shutil.rmtree(run_dir, ignore_errors=True)
        os.makedirs(run_dir)
        eeprom = os.path.join(run_dir, 'eeprom.bin')
        shutil.copy(os.path.join(ds['data'], 'sitl_eeprom.bin'), eeprom)
        raw = os.path.join(run_dir, 'chirp_raw.jsonl')
        sitl = Sitl(sitl_bin, ds['model'], eeprom,
                    os.path.join(run_dir, 'sitl.log'))
        try:
            steady_test(ds, exp, os.path.join(run_dir, 'sweep.jsonl'))
            square_test(ds, exp, os.path.join(run_dir, 'square.jsonl'))
            if not ARGS.skip_chirp:
                # fresh instances with a physics log for the raw fit,
                # and their own verbose log for the ratio guard
                sitl.stop()
                sitl = None
                chirp_test(ds, exp,
                           (sitl_bin, ds['model'], eeprom,
                            os.path.join(run_dir, 'sitl_chirp.log')),
                           os.path.join(run_dir, 'chirp.jsonl'), raw)
        finally:
            if sitl:
                last_ratio[0] = sitl.min_ratio()
                sitl.stop()
        ratio = last_ratio[0]
        print('  %s: sim/wall ratio %.2f (diagnostic; measurements are '
              'sim-time paced)' % (ds['name'], ratio), flush=True)
        if ARGS.min_ratio > 0 and ratio < ARGS.min_ratio:
            report(ds['name'], 'sim speed', False,
                   'ratio %.2f < %.2f' % (ratio, ARGS.min_ratio))
        artifacts(ds, run_dir, os.path.abspath(ARGS.artifacts))

    fails = [r for r in results if r[2] == 'FAIL']
    print('\n%d checks, %d failed, %d skipped' % (
        len(results), len(fails),
        len([r for r in results if r[2] == 'SKIP'])))
    if fails:
        sys.exit(1)
    print('all calibration tests passed')



if __name__ == '__main__':
    main()
