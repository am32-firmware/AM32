#!/usr/bin/env python3
'''
square-wave amplitude sweep of an ESC/motor over DroneCAN

from a mid throttle, step to mid+delta, wait for rpm to settle, step to
mid-delta, settle, repeat for a few cycles, then grow delta and do it
again — up to large throttle jumps. Response time as a function of step
amplitude separates the linear small-signal regime from the
amplitude-dependent effects (regen-limited braking, duty ramp, torque
saturation) and is a torture test of ESC/motor behaviour to compare
between SITL and real hardware.

run:     esc_square.py run --node-id 124 --log square.jsonl
analyze: esc_square.py analyze square.jsonl [--compare ref.jsonl]
                       [--plot out.png]

timing is measured between rpm crossings (t10-90), so transport latency
does not bias the comparison.
'''

import argparse
import json
import math
import time

import esc_measure
import sim_clock


def wait_settle(m, args):
    '''stream until the rpm mean is stable between two adjacent windows
    (or timeout). Returns the settle time. Timing is on the measurement
    clock (simulated seconds under --sim-state)'''
    t0 = m.clock.now()
    m.spin_for(args.settle_min)
    while m.clock.now() - t0 < args.settle_max:
        m.spin_for(0.1)
        now = m.rec.rows[-1]['t'] if m.rec.rows else 0
        w = args.settle_window
        cur = [r['rpm'] for r in m.rec.rows
               if r['type'] == 'status' and r['t'] > now - w]
        prev = [r['rpm'] for r in m.rec.rows
                if r['type'] == 'status' and now - 2 * w < r['t'] <= now - w]
        if len(cur) > 5 and len(prev) > 5:
            a = sum(cur) / len(cur)
            b = sum(prev) / len(prev)
            if abs(a - b) < args.settle_tol * max(abs(a), 1000.0):
                return m.clock.now() - t0
    m.rec.add('mark', label='settle timeout')
    return m.clock.now() - t0


def cmd_run(args):
    deltas = [float(x) for x in args.deltas.split(',')]
    m = esc_measure.EscMeasure(args)
    try:
        run_profile(m, args, deltas)
    finally:
        # always leave the ESC at zero throttle and flush the log,
        # including after a current/temperature abort
        m.throttle = 0.0
        m.aborted = None
        try:
            m.spin_for(0.5)
        except Exception:
            pass
        m.rec.close()


def run_profile(m, args, deltas):
    m.wait_ready()
    if args.telem_rate is not None:
        print('TELEM_RATE ->', m.set_param('TELEM_RATE', args.telem_rate))
    if args.debug_rate is not None:
        print('DEBUG_RATE ->', m.set_param('DEBUG_RATE', args.debug_rate))
    m.rec.add('square_config', mid=args.throttle_mid, deltas=deltas,
              cycles=args.cycles)
    # gentle start, then centre
    m.set_throttle(0.1)
    m.spin_for(1.5)
    m.set_throttle(args.throttle_mid)
    wait_settle(m, args)
    for delta in deltas:
        m.mark('delta %.3f' % delta)
        print('== delta %.3f (%.3f .. %.3f)' %
              (delta, args.throttle_mid - delta, args.throttle_mid + delta))
        for _ in range(args.cycles):
            m.set_throttle(args.throttle_mid + delta)
            wait_settle(m, args)
            m.set_throttle(args.throttle_mid - delta)
            wait_settle(m, args)
        m.set_throttle(args.throttle_mid)
        wait_settle(m, args)
    m.set_throttle(0)
    m.spin_for(1.0)
    print('saved %s' % args.log)


def analyze_log(path):
    '''per-transition timing: list of dicts with delta, direction,
    t10-90, overshoot fraction and a desync flag. Only full swings
    symmetric about the configured mid are counted; the mid->edge
    entry and edge->mid recentre half-steps are skipped'''
    rows = [json.loads(l) for l in open(path)]
    cfg = next((r for r in rows if r['type'] == 'square_config'), None)
    mid = cfg['mid'] if cfg else 0.35
    cmds = [(r['t'], r['throttle']) for r in rows if r['type'] == 'cmd']
    status = [(r['t'], r['rpm']) for r in rows if r['type'] == 'status']
    out = []
    for i in range(1, len(cmds)):
        t0, thr = cmds[i]
        pthr = cmds[i - 1][1]
        if thr <= 0 or pthr <= 0 or thr == pthr:
            continue
        if abs((thr + pthr) / 2 - mid) > 0.005:
            continue
        t1 = cmds[i + 1][0] if i + 1 < len(cmds) else t0 + 5.0
        base = [v for t, v in status if t0 - 0.3 <= t < t0]
        final = [v for t, v in status if t1 - 0.3 <= t < t1]
        seg = [(t, v) for t, v in status if t0 <= t < t1]
        if len(base) < 3 or len(final) < 3 or len(seg) < 5:
            continue
        b = sum(base) / len(base)
        f = sum(final) / len(final)
        if abs(f - b) < 100:
            continue
        up = f > b
        # desync flag: rpm collapsing far below both endpoints
        floor = min(b, f) - 0.8 * abs(f - b) - 500
        desync = any(v < floor for _, v in seg)
        def cross(frac):
            tgt = b + (f - b) * frac
            for t, v in seg:
                if (up and v >= tgt) or (not up and v <= tgt):
                    return t
            return None
        c10, c90 = cross(0.1), cross(0.9)
        if c10 is None or c90 is None:
            continue
        peak = max(v for _, v in seg) if up else min(v for _, v in seg)
        overshoot = (peak - f) / (f - b) if up else (f - peak) / (b - f)
        out.append({'delta': round(abs(thr - pthr) / 2, 4), 'up': up,
                    't1090': c90 - c10, 'overshoot': overshoot,
                    'desync': desync, 'base': b, 'final': f})
    return out


def aggregate(trans):
    '''per (delta, direction): median t10-90 and stats'''
    import collections
    groups = collections.defaultdict(list)
    for tr in trans:
        groups[(tr['delta'], tr['up'])].append(tr)
    out = {}
    for key, g in sorted(groups.items()):
        clean = [t for t in g if not t['desync']]
        use = clean if clean else g
        ts = sorted(t['t1090'] for t in use)
        out[key] = {'t1090': ts[len(ts) // 2],
                    'overshoot': sorted(t['overshoot'] for t in use)[len(use) // 2],
                    'n': len(g), 'desyncs': len(g) - len(clean)}
    return out


def cmd_analyze(args):
    agg = aggregate(analyze_log(args.log))
    ref = aggregate(analyze_log(args.compare)) if args.compare else None
    deltas = sorted(set(d for d, _ in agg))
    hdr = '%7s %9s %9s %6s %6s' % ('delta', 'up_ms', 'down_ms', 'ovs_up', 'ovs_dn')
    if ref:
        hdr += ' | %9s %9s' % ('ref_up', 'ref_dn')
    print(hdr)
    for d in deltas:
        up = agg.get((d, True))
        dn = agg.get((d, False))
        def ms(e):
            return '%9.0f' % (e['t1090'] * 1000) if e else '        -'
        def ov(e):
            return '%6.2f' % e['overshoot'] if e else '     -'
        line = '%7.3f %s %s %s %s' % (d, ms(up), ms(dn), ov(up), ov(dn))
        flags = ''
        for e in (up, dn):
            if e and e['desyncs']:
                flags = '  DESYNCS(%u)' % sum(x['desyncs'] for x in (up, dn) if x)
                break
        if ref:
            line += ' | %s %s' % (ms(ref.get((d, True))), ms(ref.get((d, False))))
        print(line + flags)

    if args.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(10, 6))
        for aggr, name, ls in ((agg, args.log, '-'),) + \
                (((ref, args.compare, '--'),) if ref else ()):
            for up, colour, lbl in ((True, 'tab:green', 'up'), (False, 'tab:red', 'down')):
                pts = [(d, aggr[(d, up)]['t1090'] * 1000) for d in deltas
                       if (d, up) in aggr]
                ax.plot([p[0] for p in pts], [p[1] for p in pts], ls, marker='o',
                        ms=4, color=colour, label='%s %s' % (name, lbl))
        ax.set_xlabel('throttle step delta')
        ax.set_ylabel('t10-90 (ms)')
        ax.set_yscale('log')
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
        fig.tight_layout()
        fig.savefig(args.plot, dpi=120)
        print('saved %s' % args.plot)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='cmd', required=True)

    p = sub.add_parser('run')
    sim_clock.add_argument(p)
    p.add_argument('--uri', default='mcast:0:lo')
    p.add_argument('--node-id', type=int, required=True)
    p.add_argument('--client-node-id', type=int, default=100)
    p.add_argument('--esc-index', type=int, default=0)
    p.add_argument('--rate', type=float, default=200)
    p.add_argument('--log', required=True)
    p.add_argument('--throttle-mid', type=float, default=0.35)
    p.add_argument('--deltas', default='0.03,0.06,0.09,0.12,0.15,0.18,0.21,0.24')
    p.add_argument('--cycles', type=int, default=3, help='square cycles per delta')
    p.add_argument('--settle-tol', type=float, default=0.01)
    p.add_argument('--settle-window', type=float, default=0.4)
    p.add_argument('--settle-min', type=float, default=0.6)
    p.add_argument('--settle-max', type=float, default=5.0)
    p.add_argument('--max-throttle', type=float, default=0.62)
    p.add_argument('--max-current', type=float, default=30.0)
    p.add_argument('--max-temp', type=float, default=80.0)
    p.add_argument('--ready-timeout', type=float, default=15.0)
    p.add_argument('--arm-time', type=float, default=1.5)
    p.add_argument('--telem-rate', type=int, default=200)
    p.add_argument('--debug-rate', type=int, default=50)

    p = sub.add_parser('analyze')
    p.add_argument('log')
    p.add_argument('--compare', default=None)
    p.add_argument('--plot', default=None)

    args = parser.parse_args()
    if args.cmd == 'run':
        cmd_run(args)
    else:
        cmd_analyze(args)


if __name__ == '__main__':
    main()
