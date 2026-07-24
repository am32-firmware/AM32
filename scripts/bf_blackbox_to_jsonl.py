#!/usr/bin/env python3
'''
convert a Betaflight blackbox log into the calibration JSONL schema

The FC logs eRPM and the motor output it applied, timestamped against
its own loop clock, at the blackbox rate (typically 1-2kHz) rather than
the ~200Hz an MSP poll can manage. That makes it the better source for
anything timing sensitive - the chirp's braking tail in particular.

Everything is reconstructed from the log alone, so no clock alignment
with the capturing host is needed: throttle comes from the logged motor
output and rpm from the logged eRPM.

needs the orangebox package (pip install orangebox)

usage:
  bf_blackbox_to_jsonl.py LOG00001.BFL --out chirp_bb.jsonl --chirp
  bf_blackbox_to_jsonl.py LOG00001.BFL --out sweep_bb.jsonl
'''

import argparse
import json
import sys

import esc_chirp


def load_log(path, motor):
    from orangebox import Parser
    p = Parser.load(path)
    names = p.field_names
    idx = {n: i for i, n in enumerate(names)}
    for key in ('time', 'motor[%u]' % motor, 'eRPM[%u]' % motor):
        if key not in idx:
            raise SystemExit('log has no %s field - was dshot_bidir on, and '
                             'is that motor wired?' % key)
    poles = int(p.headers.get('motor_poles', 14))
    lo, hi = 158, 2047
    mo = p.headers.get('motorOutput')
    if mo:
        try:
            lo, hi = [int(x) for x in str(mo).strip('[]').split(',')]
        except Exception:
            pass
    out = []
    ti, mi, ei = idx['time'], idx['motor[%u]' % motor], idx['eRPM[%u]' % motor]
    for f in p.frames():
        d = f.data
        t = d[ti]
        m = d[mi]
        e = d[ei]
        if t is None or m is None or e is None:
            continue
        # blackbox logs eRPM/100; mechanical rpm = erpm*100 / pole pairs
        rpm = (e * 100.0) / (poles / 2.0)
        throttle = (m - lo) / float(hi - lo)
        out.append((t * 1e-6, throttle, rpm))
    return out, poles, (lo, hi)


def find_chirp_start(samples, mid, hold=2.0):
    '''the capture spins up to 0.1, steps to the chirp mid throttle and
    holds for `hold` seconds before the chirp begins, so the chirp
    starts a known interval after that step'''
    prev = None
    for t, thr, _rpm in samples:
        if prev is not None and thr - prev > 0.5 * (mid - 0.1) and thr > mid * 0.7:
            return t + hold
        prev = thr
    return None


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('log')
    ap.add_argument('--out', required=True)
    ap.add_argument('--motor', type=int, default=1, help='motor number (1-based)')
    ap.add_argument('--chirp', action='store_true',
                    help='reconstruct chirp_config/chirp rows so esc_chirp.py fit works')
    ap.add_argument('--duration', type=float, default=120.0)
    ap.add_argument('--f-start', type=float, default=0.5)
    ap.add_argument('--f-stop', type=float, default=40.0)
    ap.add_argument('--fade-in', type=float, default=3.0)
    ap.add_argument('--fade-out', type=float, default=1.0)
    ap.add_argument('--throttle-mid', type=float, default=0.35)
    ap.add_argument('--throttle-amp', type=float, default=0.15)
    ap.add_argument('--chirp-start', type=float, default=None,
                    help='log time the chirp began (default: auto-detect)')
    args = ap.parse_args()

    samples, poles, rng = load_log(args.log, args.motor - 1)
    if not samples:
        raise SystemExit('no usable frames in the log')
    t0 = samples[0][0]
    print('%u frames, %.1fs, %u poles, motor output range %s'
          % (len(samples), samples[-1][0] - t0, poles, list(rng)))
    print('logged rate: %.0f Hz'
          % (len(samples) / max(samples[-1][0] - t0, 1e-6)))

    rows = []
    if args.chirp:
        start = args.chirp_start
        if start is None:
            start = find_chirp_start(samples, args.throttle_mid)
            if start is None:
                raise SystemExit('could not find the chirp start; pass '
                                 '--chirp-start (log seconds)')
        print('chirp starts at log t=%.3fs' % (start - t0))
        chirp = esc_chirp.Chirp(args.duration, args.f_start, args.f_stop,
                                args.fade_in, args.fade_out)
        rows.append({'t': round(start - t0, 4), 'type': 'chirp_config',
                     'duration': args.duration, 'f_start': args.f_start,
                     'f_stop': args.f_stop, 'fade_in': args.fade_in,
                     'fade_out': args.fade_out, 'const_freq': chirp.const_freq,
                     'mid': args.throttle_mid, 'amp': args.throttle_amp})
        for t, thr, rpm in samples:
            ct = t - start
            if -0.5 <= ct <= chirp.record + 0.5:
                _out, freq = chirp.update(max(ct, 0.0))
                # the drive actually applied, from the logged output
                applied = (thr - args.throttle_mid) / args.throttle_amp
                rows.append({'t': round(t - t0, 4), 'type': 'chirp',
                             'ct': round(ct, 4), 'out': round(applied, 5),
                             'freq': round(freq, 3)})
            rows.append({'t': round(t - t0, 4), 'type': 'status',
                         'rpm': int(round(rpm)), 'volt': 0.0, 'curr': 0.0,
                         'temp': 0.0, 'err': 0})
    else:
        last = None
        for t, thr, rpm in samples:
            q = round(thr, 3)
            if last is None or abs(q - last) > 0.004:
                rows.append({'t': round(t - t0, 4), 'type': 'cmd',
                             'throttle': q})
                last = q
            rows.append({'t': round(t - t0, 4), 'type': 'status',
                         'rpm': int(round(rpm)), 'volt': 0.0, 'curr': 0.0,
                         'temp': 0.0, 'err': 0})

    rows.sort(key=lambda r: r['t'])
    with open(args.out, 'w') as f:
        for r in rows:
            f.write(json.dumps(r) + '\n')
    n_status = sum(1 for r in rows if r['type'] == 'status')
    print('wrote %s (%u rows, %u status)' % (args.out, len(rows), n_status))


if __name__ == '__main__':
    main()
