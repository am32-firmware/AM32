#!/usr/bin/env python3
'''
analyse esc_measure.py JSONL logs and compare real hardware vs SITL

single log: steady-state table per throttle level plus ensemble-averaged
step transients (aligned on cmd rows, pooled across repeats)

two logs (--compare REF): both printed side by side with deltas —
used to iterate a SITL model json until it matches a hardware log
'''

import argparse
import collections
import json


def load(path):
    return [json.loads(l) for l in open(path)]


def steady(rows):
    '''per cmd-level segment: mean over last half, keyed by throttle (last wins per level)'''
    segs = []
    cur = None
    for r in rows:
        if r['type'] == 'cmd':
            if cur:
                segs.append(cur)
            cur = {'throttle': r['throttle'], 't0': r['t'], 'rows': []}
        elif r['type'] == 'status' and cur:
            cur['rows'].append(r)
    if cur:
        segs.append(cur)
    out = collections.OrderedDict()
    for s in segs:
        if not s['rows']:
            continue
        t1 = s['rows'][-1]['t']
        tmid = (s['t0'] + t1) / 2
        rr = [r for r in s['rows'] if r['t'] >= tmid]
        if len(rr) < 5:
            continue
        n = len(rr)
        mean = lambda k: sum(r[k] for r in rr) / n
        rpm = mean('rpm')
        key = s['throttle']
        ent = out.setdefault(key, {'rpm': [], 'volt': [], 'curr': [], 'sd': []})
        ent['rpm'].append(rpm)
        ent['volt'].append(mean('volt'))
        ent['curr'].append(mean('curr'))
        ent['sd'].append((sum((r['rpm'] - rpm) ** 2 for r in rr) / n) ** 0.5)
    avg = lambda l: sum(l) / len(l)
    return collections.OrderedDict(
        (k, {f: avg(v[f]) for f in v}) for k, v in sorted(out.items()))


def transitions(rows, direction):
    '''list of (t_cmd, from, to) for up or down steps between nonzero levels'''
    cmds = [(r['t'], r['throttle']) for r in rows if r['type'] == 'cmd']
    out = []
    for i in range(1, len(cmds)):
        t, thr = cmds[i]
        pthr = cmds[i - 1][1]
        if pthr <= 0 or thr <= 0:
            continue
        if direction == 'up' and thr > pthr:
            out.append((t, pthr, thr))
        if direction == 'down' and thr < pthr:
            out.append((t, pthr, thr))
    return out


def ensemble(rows, direction, window=0.6, binw=0.01):
    '''pooled rpm-vs-time bins across all transitions in one direction

    only the most common (from, to) level pair is pooled, so one-off
    transitions (e.g. a gentle-start precondition step) do not pollute
    the ensemble'''
    trans = transitions(rows, direction)
    if trans:
        pairs = collections.Counter((f, t) for _, f, t in trans)
        want = pairs.most_common(1)[0][0]
        trans = [t for t in trans if (t[1], t[2]) == want]
    status = [(r['t'], r['rpm']) for r in rows if r['type'] == 'status']
    pool = []
    for t0, _, _ in trans:
        pool += [(t - t0, rpm) for t, rpm in status if -0.05 <= t - t0 <= window]
    bins = collections.OrderedDict()
    for t, v in sorted(pool):
        b = round(t / binw) * binw
        bins.setdefault(b, []).append(v)
    return collections.OrderedDict((b, sum(v) / len(v)) for b, v in bins.items()), len(trans)


def print_steady(st, ref=None):
    hdr = '%8s %9s %8s %8s %8s' % ('throttle', 'rpm', 'rpm_sd', 'volt', 'curr')
    if ref:
        hdr += ' %10s %8s' % ('ref_rpm', 'drpm%')
    print(hdr)
    for thr, v in st.items():
        line = '%8.3f %9.1f %8.1f %8.3f %8.3f' % (thr, v['rpm'], v['sd'], v['volt'], v['curr'])
        if ref and thr in ref:
            rrpm = ref[thr]['rpm']
            dpc = 100.0 * (v['rpm'] - rrpm) / rrpm if rrpm else 0.0
            line += ' %10.1f %+8.2f' % (rrpm, dpc)
        print(line)


def print_ensemble(ens, n, ref_ens=None):
    hdr = '%7s %9s' % ('t_ms', 'rpm')
    if ref_ens:
        hdr += ' %9s %8s' % ('ref_rpm', 'drpm')
    print(hdr + '   (%u transitions)' % n)
    for b, v in ens.items():
        line = '%7.0f %9.0f' % (b * 1000, v)
        if ref_ens is not None and b in ref_ens:
            line += ' %9.0f %8.0f' % (ref_ens[b], v - ref_ens[b])
        print(line)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log', help='JSONL log (e.g. from SITL)')
    parser.add_argument('--compare', default=None, help='reference JSONL log (e.g. real hardware)')
    parser.add_argument('--transients', action='store_true', help='show ensemble step transients')
    parser.add_argument('--binw', type=float, default=0.01, help='transient bin width seconds')
    args = parser.parse_args()

    rows = load(args.log)
    ref_rows = load(args.compare) if args.compare else None
    st = steady(rows)
    ref = steady(ref_rows) if ref_rows else None
    print('steady state (%s%s):' % (args.log, ' vs ' + args.compare if args.compare else ''))
    print_steady(st, ref)
    if args.transients:
        for direction in ('up', 'down'):
            ens, n = ensemble(rows, direction, binw=args.binw)
            if not n:
                continue
            rens = ensemble(ref_rows, direction, binw=args.binw)[0] if ref_rows else None
            print('\n%s transitions:' % direction)
            print_ensemble(ens, n, rens)


if __name__ == '__main__':
    main()
