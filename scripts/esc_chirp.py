#!/usr/bin/env python3
'''
frequency-response (chirp) test of an ESC/motor over DroneCAN

drives esc.RawCommand with a sinusoidal throttle demand whose frequency
sweeps exponentially from f_start to f_stop (same waveform as the
ArduPilot actuator_chirp.lua applet: constant-frequency dwell, Hann
fade in/out), while logging rpm telemetry. The rpm amplitude rolloff
with frequency is a key characteristic of a powertrain; braking
behaviour shows as asymmetry (mean rpm shift) at higher frequencies.

run:     esc_chirp.py run --node-id 124 --log chirp.jsonl
analyze: esc_chirp.py analyze chirp.jsonl [--compare ref.jsonl]
plot:    esc_chirp.py plot chirp.jsonl [--compare ref.jsonl] [--save f.png]
fit:     esc_chirp.py fit chirp.jsonl [--plot f.png]

fit measures the per-cycle up-swing (accelerating) and down-swing
(braking) rpm envelopes against the low-frequency baseline, reports the
-3dB (70.7% amplitude) frequency of each, and fits an asymmetric LPF
(separate rising/falling time constants, 1-pole and 2-pole cascades
plus a transport delay) to the full time series. The (accel, brake)
3dB pair is the characteristic number of a powertrain at a given
throttle range; braking is torque-bounded so its knee is sharper than
1-pole and its cutoff depends on the chirp amplitude

works identically against real hardware and the SITL build. Note: over
a CAN bridge the transport delay dominates the measured phase at high
frequency; amplitude is the robust comparison.
'''

import argparse
import json
import math
import time

import esc_measure
import sim_clock


class Chirp(object):
    '''exponential chirp waveform, matching ArduPilot actuator_chirp.lua'''
    def __init__(self, duration, f_start, f_stop, fade_in, fade_out):
        self.record = duration
        self.wMin = 2 * math.pi * f_start
        self.wMax = 2 * math.pi * f_stop
        self.fade_in = fade_in
        self.fade_out = fade_out
        self.const_freq = 2.0 / f_start
        self.B = math.log(self.wMax / self.wMin)
        self.freq_rads = self.wMin

    def phase(self, t):
        '''accumulated waveform phase at time t (rad)'''
        if t <= self.const_freq:
            self.freq_rads = self.wMin
            return self.wMin * t - self.wMin * self.const_freq
        expFactor = (t - self.const_freq) / (self.record - self.const_freq)
        self.freq_rads = self.wMin * math.exp(self.B * expFactor)
        return (self.wMin * (self.record - self.const_freq) / self.B) * \
            (math.exp(self.B * expFactor) - 1)

    def update(self, t):
        '''(output in -1..1, frequency Hz); output 0 after self.record'''
        if t <= 0:
            return 0.0, self.wMin / (2 * math.pi)
        if t <= self.fade_in:
            window = 0.5 - 0.5 * math.cos(math.pi * t / self.fade_in)
        elif t <= self.record - self.fade_out:
            window = 1.0
        elif t <= self.record:
            tn = (t - (self.record - self.fade_out)) / self.fade_out
            window = 0.5 - 0.5 * math.cos(math.pi * tn + math.pi)
        else:
            return 0.0, self.wMax / (2 * math.pi)
        out = window * math.sin(self.phase(t))
        return out, self.freq_rads / (2 * math.pi)


def cmd_run(args):
    m = esc_measure.EscMeasure(args)
    m.wait_ready()
    for kv in args.param or []:
        name, _, val = kv.partition('=')
        print('%s -> %s' % (name, m.set_param(name, int(val))))
    if args.telem_rate is not None:
        print('TELEM_RATE ->', m.set_param('TELEM_RATE', args.telem_rate))
    if args.debug_rate is not None:
        print('DEBUG_RATE ->', m.set_param('DEBUG_RATE', args.debug_rate))
    run_chirp(m, args)


def run_chirp(m, args):
    '''drive the chirp profile on any measurement backend implementing
    the EscMeasure interface (set_throttle/spin_for/rec/clock)'''
    # spin up gently to the mid throttle
    m.set_throttle(0.1)
    m.spin_for(1.5)
    m.set_throttle(args.throttle_mid)
    m.spin_for(2.0)

    chirp = Chirp(args.duration, args.f_start, args.f_stop,
                  args.fade_in, args.fade_out)
    m.rec.add('chirp_config', duration=args.duration, f_start=args.f_start,
              f_stop=args.f_stop, fade_in=args.fade_in, fade_out=args.fade_out,
              const_freq=chirp.const_freq, mid=args.throttle_mid,
              amp=args.throttle_amp)
    print('chirp: %.2f..%.1f Hz over %.0fs, throttle %.2f +/- %.2f' %
          (args.f_start, args.f_stop, args.duration,
           args.throttle_mid, args.throttle_amp))
    t0 = m.clock.now()
    period = 1.0 / args.rate
    next_status = 0
    while True:
        t = m.clock.now() - t0
        if t > chirp.record + 0.5:
            break
        out, freq = chirp.update(t)
        m.rec.add('chirp', ct=round(t, 4), out=round(out, 5), freq=round(freq, 3))
        m.set_throttle(args.throttle_mid + args.throttle_amp * out)
        if t > next_status:
            print('  t=%.0fs f=%.2fHz' % (t, freq))
            next_status = t + 5
        m.spin_for(period)
    m.set_throttle(0)
    m.spin_for(1.0)
    m.rec.close()
    print('saved %s' % args.log)


def demod(rows, cfg, tref, nbins=24):
    '''amplitude/phase of rpm and drive vs frequency by quadrature
    demodulation against the reconstructed chirp phase'''
    chirp = Chirp(cfg['duration'], cfg['f_start'], cfg['f_stop'],
                  cfg['fade_in'], cfg['fade_out'])
    # chirp rows map log time to chirp time: ct = t - toff
    toff = tref
    status = [(r['t'] - toff, r['rpm']) for r in rows if r['type'] == 'status']
    drive = [(r['ct'], r['out']) for r in rows if r['type'] == 'chirp']
    # log-spaced analysis frequencies
    fbins = [cfg['f_start'] * (cfg['f_stop'] / cfg['f_start']) ** (i / (nbins - 1))
             for i in range(nbins)]

    def t_of_freq(f):
        w = 2 * math.pi * f
        if w <= chirp.wMin:
            return chirp.const_freq
        return chirp.const_freq + math.log(w / chirp.wMin) / chirp.B * \
            (cfg['duration'] - chirp.const_freq)

    out = []
    for f in fbins:
        tc = t_of_freq(f)
        cycles = max(3.0, f * 0.4)  # >= 3 cycles or 0.4s of data
        half = 0.5 * cycles / f
        if tc - half < chirp.const_freq * 0.2 or tc + half > cfg['duration'] - cfg['fade_out']:
            continue
        def dem(series):
            si = sc = n = 0.0
            mean = 0.0
            vals = []
            for t, v in series:
                if tc - half <= t <= tc + half:
                    ph = chirp.phase(t)
                    si += v * math.sin(ph)
                    sc += v * math.cos(ph)
                    mean += v
                    vals.append((t, v))
                    n += 1
            if n < 8:
                return None
            mean /= n
            # incoherent amplitude: sqrt(2) * detrended rms. Immune to
            # transport latency jitter, which scrambles the coherent
            # phase at high frequency over a CAN bridge
            tm = sum(t for t, _ in vals) / n
            num = sum((t - tm) * (v - mean) for t, v in vals)
            den = sum((t - tm) ** 2 for t, v in vals) or 1.0
            slope = num / den
            var = sum((v - mean - slope * (t - tm)) ** 2 for t, v in vals) / n
            return (2 * si / n, 2 * sc / n, mean, math.sqrt(2 * var))
        dr = dem(drive)
        st = dem(status)
        if not dr or not st:
            continue
        amp_d = math.hypot(dr[0], dr[1])
        amp_r = math.hypot(st[0], st[1])
        if amp_d < 1e-6:
            continue
        phase = math.degrees(math.atan2(st[1], st[0]) - math.atan2(dr[1], dr[0]))
        while phase > 180:
            phase -= 360
        while phase < -180:
            phase += 360
        amp_d_inc = dr[3] if dr[3] > 1e-6 else amp_d
        out.append({'freq': f, 'gain': amp_r / amp_d, 'phase': phase,
                    'rpm_mean': st[2], 'rpm_amp': amp_r,
                    'gain_inc': st[3] / amp_d_inc})
    return out


def analyze_log(path):
    rows = [json.loads(l) for l in open(path)]
    cfg = next(r for r in rows if r['type'] == 'chirp_config')
    # reference: log t of the first chirp row minus its ct
    first = next(r for r in rows if r['type'] == 'chirp')
    return demod(rows, cfg, first['t'] - first['ct'])


def chirp_t_of_freq(cfg, f):
    '''chirp time at which the sweep reaches frequency f'''
    const_freq = 2.0 / cfg['f_start']
    if f <= cfg['f_start']:
        return const_freq
    B = math.log(cfg['f_stop'] / cfg['f_start'])
    return const_freq + math.log(f / cfg['f_start']) / B * \
        (cfg['duration'] - const_freq)


def load_raw(path):
    '''load a SITL --physics-log: skip partial lines, split into boot
    segments where sim time restarts, return the longest segment'''
    segs = [[]]
    for line in open(path):
        try:
            r = json.loads(line)
        except ValueError:
            continue
        if r.get('type') != 'status':
            continue
        if segs[-1] and r['t'] < segs[-1][-1][0]:
            segs.append([])
        segs[-1].append((r['t'], r['rpm']))
    return max(segs, key=len)


def align_raw(np, status, raw_status):
    '''align a raw physics log (sim time) to the client log's chirp
    time base by cross-correlating the rpm series; returns the raw
    series shifted into chirp time'''
    dt = 0.005
    def binned(series):
        t = np.array([p[0] for p in series])
        v = np.array([p[1] for p in series])
        n = int(math.ceil((t.max() - t.min()) / dt)) + 1
        idx = ((t - t.min()) / dt).astype(int)
        acc = np.zeros(n)
        cnt = np.zeros(n)
        np.add.at(acc, idx, v)
        np.add.at(cnt, idx, 1)
        filled = acc / np.maximum(cnt, 1)
        # forward fill empty bins from the previous filled bin
        good = np.flatnonzero(cnt > 0)
        filled = filled[good][np.clip(np.searchsorted(good, np.arange(n), 'right') - 1, 0, None)]
        return t.min(), filled
    ct0, cb = binned(status)
    rt0, rb = binned(raw_status)
    a = cb - cb.mean()
    b = rb - rb.mean()
    corr = np.correlate(b, a, mode='valid')
    off = int(corr.argmax()) * dt + rt0 - ct0
    return [(t - off, v) for t, v in raw_status], off


def load_rows(path):
    rows = [json.loads(l) for l in open(path)]
    cfg = next(r for r in rows if r['type'] == 'chirp_config')
    first = next(r for r in rows if r['type'] == 'chirp')
    toff = first['t'] - first['ct']
    return rows, cfg, toff


def cycle_envelope(status, cfg):
    '''per-chirp-cycle envelope: (freq, up_amp, dn_amp, mid) with amps
    normalised by that cycle's drive window (fade in/out).
    status is a list of (chirp_time, rpm)'''
    chirp = Chirp(cfg['duration'], cfg['f_start'], cfg['f_stop'],
                  cfg['fade_in'], cfg['fade_out'])
    percyc = {}
    for t, rpm in status:
        if t < 0 or t > cfg['duration'] - cfg['fade_out']:
            continue
        ph = chirp.phase(t)
        k = math.floor(ph / (2 * math.pi))
        e = percyc.setdefault(k, [t, t, rpm, rpm, 0])
        e[0] = min(e[0], t)
        e[1] = max(e[1], t)
        e[2] = max(e[2], rpm)
        e[3] = min(e[3], rpm)
        e[4] += 1
    out = []
    for k in sorted(percyc):
        t0, t1, hi, lo, n = percyc[k]
        if n < 4:
            continue
        tm = (t0 + t1) / 2
        # exact instantaneous chirp frequency at the cycle centre, so
        # the envelope is monotonic in frequency (the cycle-duration
        # estimate is too noisy at high frequency)
        if tm <= chirp.const_freq:
            freq = cfg['f_start']
        else:
            expFactor = (tm - chirp.const_freq) / (cfg['duration'] - chirp.const_freq)
            freq = cfg['f_start'] * math.exp(chirp.B * expFactor)
        # drive amplitude window (fade in/out) at the cycle centre
        if tm <= cfg['fade_in']:
            window = 0.5 - 0.5 * math.cos(math.pi * tm / cfg['fade_in'])
        elif tm <= cfg['duration'] - cfg['fade_out']:
            window = 1.0
        else:
            window = 0.0
        if window < 0.3:
            continue
        out.append({'freq': freq, 'hi': hi, 'lo': lo, 'window': window})
    return out


def envelope_numbers(env):
    '''baseline centre/amplitude and normalised up/dn amplitude per
    cycle; 3dB crossing frequencies by interpolation. Robust to desync
    collapses: median baseline, and cycles whose trough falls below any
    legitimate swing are dropped'''
    def median(vals):
        v = sorted(vals)
        return v[len(v) // 2]
    base = [e for e in env if e['freq'] < 1.5 * env[0]['freq'] and e['window'] > 0.9]
    if len(base) < 2:
        base = env[:4]
    centre = median([(e['hi'] + e['lo']) / 2 for e in base])
    amp0 = median([(e['hi'] - e['lo']) / 2 / e['window'] for e in base])
    nall = len(env)
    env = [e for e in env if e['lo'] > centre - 1.35 * amp0 and
           e['hi'] < centre + 1.35 * amp0]
    if nall - len(env) > 0:
        print('  (%u collapsed cycles dropped as desync artefacts)' % (nall - len(env)))
    pts = []
    for e in env:
        up = (e['hi'] - centre) / e['window'] / amp0
        dn = (centre - e['lo']) / e['window'] / amp0
        pts.append({'freq': e['freq'], 'up': up, 'dn': dn})
    # median smoothing over 5 cycles
    def med5(key, i):
        w = [pts[j][key] for j in range(max(0, i - 2), min(len(pts), i + 3))]
        return sorted(w)[len(w) // 2]
    for i, p in enumerate(pts):
        p['up_s'] = med5('up', i)
        p['dn_s'] = med5('dn', i)

    def f3db(key):
        thresh = 1 / math.sqrt(2)
        for i in range(1, len(pts)):
            a, b = pts[i - 1], pts[i]
            if a[key] >= thresh and b[key] < thresh and b['freq'] > a['freq']:
                fr = (a[key] - thresh) / max(1e-9, a[key] - b[key])
                return a['freq'] * (b['freq'] / a['freq']) ** fr
        return None
    return centre, amp0, pts, f3db('up_s'), f3db('dn_s')


def sim_asym_stages(np, u, dt, stages, y0, rate_dn=None):
    '''simulate cascaded asymmetric one-pole stages over input array u.
    stages is a list of (tau_up, tau_dn) arrays of candidate parameter
    sets; rate_dn (if given) clamps the downward rate of the final
    stage, modelling torque-bounded braking. Returns y of shape
    (len(u), nparams)'''
    alphas = [(np.minimum(dt / tu, 1.0), np.minimum(dt / td, 1.0))
              for tu, td in stages]
    dmax = rate_dn * dt if rate_dn is not None else None
    nparam = len(stages[0][0])
    y = [np.full(nparam, float(y0)) for _ in range(len(stages))]
    out = np.empty((len(u), nparam))
    last = len(stages) - 1
    for i, ui in enumerate(u):
        x = ui
        for s_i, (au, ad) in enumerate(alphas):
            ys = y[s_i]
            alpha = np.where(x > ys, au, ad)
            dy = alpha * (x - ys)
            if dmax is not None and s_i == last:
                dy = np.maximum(dy, -dmax)
            ys += dy
            x = ys
        out[i] = y[-1]
    return out


def compute_fit(np, status, cfg, centre, amp0, progress, max_freq=None):
    '''fit asymmetric LPF models to the time series (the slow part).
    progress is called with status strings; returns the best model and
    its envelope computed the same way as the measured one'''
    chirp = Chirp(cfg['duration'], cfg['f_start'], cfg['f_stop'],
                  cfg['fade_in'], cfg['fade_out'])
    dt = 0.002
    tmax = cfg['duration'] - cfg['fade_out']
    if max_freq:
        tmax = min(tmax, chirp_t_of_freq(cfg, max_freq))
    tgrid = np.arange(0, tmax, dt)
    u = np.array([centre + amp0 * chirp.update(t)[0] for t in tgrid])
    st = np.array([(t, rpm) for t, rpm in status if t < tmax])
    st_idx = np.clip((st[:, 0] / dt).astype(int), 0, len(tgrid) - 1)

    # cycle segmentation of the measured samples, for the
    # envelope-space cost: one hi/lo pair per chirp cycle, weighting
    # every frequency equally and making the fit latency-immune
    cyc = np.array([math.floor(chirp.phase(t) / (2 * math.pi)) for t in st[:, 0]])
    starts = np.flatnonzero(np.r_[True, np.diff(cyc) != 0])
    counts = np.diff(np.r_[starts, len(cyc)])
    cyc_hi = np.maximum.reduceat(st[:, 1], starts)
    cyc_lo = np.minimum.reduceat(st[:, 1], starts)
    # reject desync-collapsed cycles from the fit reference too, with
    # the same bounds the empirical envelope uses; without this a
    # desync burst biases the fitted tau_dn and the model selection
    keep = (counts >= 4) & (cyc_lo > centre - 1.35 * amp0) & \
        (cyc_hi < centre + 1.35 * amp0)
    ndrop = int((counts >= 4).sum() - keep.sum())
    if ndrop:
        print('  (fit reference: %u collapsed cycles dropped)' % ndrop)
    meas_hi = cyc_hi[keep]
    meas_lo = cyc_lo[keep]
    # equal weight per octave, not per cycle: an exponential sweep has
    # most of its cycles at high frequency, which would otherwise
    # outvote the low-frequency knee region (and its noise floor)
    cyc_t = st[:, 0][starts][keep]
    cyc_w = np.array([1.0 / max(cfg['f_start'],
                                (lambda tm: cfg['f_start'] * math.exp(
                                    chirp.B * (tm - chirp.const_freq) /
                                    (cfg['duration'] - chirp.const_freq))
                                 if tm > chirp.const_freq else cfg['f_start'])(t))
                      for t in cyc_t])
    cyc_w /= cyc_w.sum()

    def env_rms(y_st):
        '''octave-weighted per-cycle envelope rms against the measured
        envelope, for each parameter column of y_st'''
        his = np.maximum.reduceat(y_st, starts, axis=0)[keep]
        los = np.minimum.reduceat(y_st, starts, axis=0)[keep]
        err = (his - meas_hi[:, None]) ** 2 + (los - meas_lo[:, None]) ** 2
        return np.sqrt((err * cyc_w[:, None]).sum(axis=0))

    FAST = 1e-4  # effectively pass-through stage direction

    def stage_builder(name, np_, p):
        '''per-candidate (stages, rate_dn) from flattened parameter
        columns'''
        if name == '1-pole':
            return [(p[0], p[1])], None
        if name == '2-pole':
            return [(p[0], p[1]), (p[0], p[1])], None
        if name == 'mixed':
            # first-order rising, second-order falling
            fast = np_.full_like(p[0], FAST)
            return [(p[0], p[1]), (fast, p[2])], None
        # slew: braking rate limit on a 1-pole (p[2] in rpm/s)
        return [(p[0], p[1])], p[2]

    def fit_candidate(name, nparams):
        rngs = [(1e-3, 0.5)] * nparams
        if name == 'slew':
            rngs[2] = (3e3, 3e5)
        best = None
        for _ in range(4):
            axes = [np.geomspace(lo, hi, 10 if nparams == 2 else 8)
                    for lo, hi in rngs]
            grids = np.meshgrid(*axes)
            p = [g.ravel() for g in grids]
            stages, rd = stage_builder(name, np, p)
            y = sim_asym_stages(np, u, dt, stages, centre, rate_dn=rd)
            rms = env_rms(y[st_idx])
            i = int(rms.argmin())
            if best is None or rms[i] < best[0]:
                best = (float(rms[i]),) + tuple(float(pp[i]) for pp in p)
            rngs = [(b / 2.5, b * 2.5) for b in best[1:]]
        return best

    candidates = [
        ('1-pole', 2),   # tau_up, tau_dn
        ('2-pole', 2),   # tau_up, tau_dn per stage
        ('mixed', 3),    # 1st order rising; 2nd order falling (tau_dn2)
        ('slew', 3),     # 1-pole with braking rate limit (rpm/s)
    ]
    models = {}
    for name, nparams in candidates:
        progress('fitting %s model...' % name)
        best = fit_candidate(name, nparams)
        rms, tu, td = best[0], best[1], best[2]
        td2 = best[3] if nparams > 2 else None
        desc = 'tau_up=%.1fms tau_dn=%.1fms' % (tu * 1000, td * 1000)
        if name == 'mixed' and td2 is not None:
            desc += ' tau_dn2=%.1fms' % (td2 * 1000)
        if name == 'slew' and td2 is not None:
            desc += ' brake_slew=%.0frpm/s' % td2
        print('%s fit: %s envelope-rms=%.0frpm' % (name, desc, rms))
        p = [np.array([tu]), np.array([td])]
        if td2 is not None:
            p.append(np.array([td2]))
        stages, rd = stage_builder(name, np, p)
        y = sim_asym_stages(np, u, dt, stages, centre, rate_dn=rd)[:, 0]
        model_status = [(float(st[i, 0]), float(y[st_idx[i]])) for i in range(len(st))]
        menv = cycle_envelope(model_status, cfg)
        _, _, mpts, mf3_up, mf3_dn = envelope_numbers(menv)
        print('  %s model 3dB: accel %s, brake %s' %
              (name, '%.2f Hz' % mf3_up if mf3_up else 'not reached',
               '%.2f Hz' % mf3_dn if mf3_dn else 'not reached'))
        models[name] = {'rms': rms, 'tau_up': tu, 'tau_dn': td,
                        'tau_dn2': td2 if name == 'mixed' else None,
                        'slew': td2 if name == 'slew' else None,
                        'mpts': mpts, 'menv': menv,
                        'mf3_up': mf3_up, 'mf3_dn': mf3_dn}
    best = min(models, key=lambda k: models[k]['rms'])
    print('best: %s (rms %.0f rpm)' % (best, models[best]['rms']))
    return {'best': best, 'models': models, 'centre': centre, 'amp0': amp0}


def mark_3db(ax, f, colour, yoff=8):
    '''highlight a -3dB crossing point'''
    if not f:
        return
    thresh = 1 / math.sqrt(2)
    ax.plot([f], [thresh], 'o', ms=9, mfc='none', mec=colour, mew=2, zorder=5)
    ax.annotate('%.1f Hz' % f, (f, thresh), textcoords='offset points',
                xytext=(6, yoff), color=colour, fontsize=9, fontweight='bold')
    ax.axvline(f, color=colour, ls=':', lw=1, alpha=0.6)


def draw_model(ax, res, log):
    styles = {'1-pole': '--', '2-pole': ':', 'mixed': '-', 'slew': '-.'}
    for name, m in res['models'].items():
        style = styles.get(name, '--')
        lw = 2.0 if name == res['best'] else 1.2
        ax.plot([p['freq'] for p in m['mpts']], [p['up_s'] for p in m['mpts']],
                style, lw=lw, color='darkgreen', alpha=0.9,
                label='%s up (rms %.0f)' % (name, m['rms']))
        ax.plot([p['freq'] for p in m['mpts']], [p['dn_s'] for p in m['mpts']],
                style, lw=lw, color='darkred', alpha=0.9,
                label='%s down' % name)
    b = res['models'][res['best']]
    mark_3db(ax, b['mf3_up'], 'darkgreen', yoff=-16)
    mark_3db(ax, b['mf3_dn'], 'darkred', yoff=-16)
    desc = 'tau_up %.0fms tau_dn %.0fms' % (b['tau_up'] * 1000, b['tau_dn'] * 1000)
    if b['tau_dn2']:
        desc += ' tau_dn2 %.0fms' % (b['tau_dn2'] * 1000)
    if b['slew']:
        desc += ' brake_slew %.0frpm/s' % b['slew']
    ax.set_title('%s: best %s, %s, envelope-rms %.0frpm' %
                 (log, res['best'], desc, b['rms']), fontsize=10)
    ax.legend(fontsize=8)


def cmd_fit(args):
    try:
        import numpy as np
    except ImportError:
        raise SystemExit('fit needs numpy')
    import matplotlib
    if args.plot:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    rows, cfg, toff = load_rows(args.log)
    status = [(r['t'] - toff, r['rpm']) for r in rows if r['type'] == 'status'
              if 0 <= r['t'] - toff <= cfg['duration']]
    if args.raw:
        raw = load_raw(args.raw)
        status, off = align_raw(np, status, raw)
        status = [(t, v) for t, v in status if 0 <= t <= cfg['duration']]
        print('raw log aligned: offset %.3fs, %u samples' % (off, len(status)))
    env = cycle_envelope(status, cfg)
    if args.max_freq:
        env = [e for e in env if e['freq'] <= args.max_freq]
    centre, amp0, pts, f3_up, f3_dn = envelope_numbers(env)
    print('baseline: centre=%.0f rpm, amplitude=%.0f rpm' % (centre, amp0))
    print('empirical 3dB (70.7% of baseline amplitude):')
    print('  accelerating (up-swing): %s' % ('%.2f Hz' % f3_up if f3_up else 'not reached'))
    print('  braking (down-swing):    %s' % ('%.2f Hz' % f3_dn if f3_dn else 'not reached'))

    # plot the measured envelope immediately
    fig, ax = plt.subplots(figsize=(11, 6))
    ax.plot([p['freq'] for p in pts], [p['up_s'] for p in pts],
            'o-', ms=2.5, lw=0.8, color='tab:green', label='measured up-swing (accel)')
    ax.plot([p['freq'] for p in pts], [p['dn_s'] for p in pts],
            'o-', ms=2.5, lw=0.8, color='tab:red', label='measured down-swing (brake)')
    ax.axhline(1 / math.sqrt(2), color='gray', ls=':', lw=1, label='-3dB (70.7%)')
    mark_3db(ax, f3_up, 'tab:green')
    mark_3db(ax, f3_dn, 'tab:red')
    ax.set_xscale('log')
    ax.set_xlabel('frequency (Hz)')
    ax.set_ylabel('rpm swing relative to baseline amplitude')
    ax.set_title('%s: accel 3dB %s, brake 3dB %s' %
                 (args.log, '%.2f Hz' % f3_up if f3_up else '-',
                  '%.2f Hz' % f3_dn if f3_dn else '-'), fontsize=10)
    ax.legend(fontsize=9)
    ax.grid(alpha=0.3)
    fig.tight_layout()

    if args.plot:
        res = compute_fit(np, status, cfg, centre, amp0, progress=print,
                          max_freq=args.max_freq)
        draw_model(ax, res, args.log)
        fig.savefig(args.plot, dpi=120)
        print('saved %s' % args.plot)
        return

    # interactive: show the data now, fit in a background thread and
    # draw the model when ready
    import threading
    holder = {}
    fit_note = ax.text(0.02, 0.02, 'fitting...', transform=ax.transAxes,
                       color='tab:blue', fontsize=10)

    def worker():
        try:
            holder['res'] = compute_fit(np, status, cfg, centre, amp0,
                                        progress=lambda s: holder.__setitem__('msg', s),
                                        max_freq=args.max_freq)
        except Exception as ex:
            holder['err'] = str(ex)

    threading.Thread(target=worker, daemon=True).start()
    timer = fig.canvas.new_timer(interval=200)

    def on_timer():
        if 'msg' in holder:
            fit_note.set_text(holder.pop('msg'))
            fig.canvas.draw_idle()
        if 'res' in holder:
            draw_model(ax, holder.pop('res'), args.log)
            fit_note.set_text('')
            fig.canvas.draw_idle()
            timer.stop()
        if 'err' in holder:
            fit_note.set_text('fit failed: %s' % holder.pop('err'))
            fig.canvas.draw_idle()
            timer.stop()

    timer.add_callback(on_timer)
    timer.start()
    plt.show()


def cmd_plot(args):
    import matplotlib
    if args.save:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    def load_series(path):
        rows, cfg, toff = load_rows(path)
        chirp = Chirp(cfg['duration'], cfg['f_start'], cfg['f_stop'],
                      cfg['fade_in'], cfg['fade_out'])

        def f_of_t(t):
            if t <= chirp.const_freq:
                return cfg['f_start']
            expFactor = (t - chirp.const_freq) / (cfg['duration'] - chirp.const_freq)
            return cfg['f_start'] * math.exp(chirp.B * expFactor)

        thr = [(f_of_t(r['ct']), cfg['mid'] + cfg['amp'] * r['out'])
               for r in rows if r['type'] == 'chirp'
               if 0 <= r['ct'] <= cfg['duration']]
        status = [(r['t'] - toff, r['rpm']) for r in rows if r['type'] == 'status'
                  if 0 <= r['t'] - toff <= cfg['duration']]
        rpm = [(f_of_t(t), v) for t, v in status]
        if args.max_freq:
            thr = [p for p in thr if p[0] <= args.max_freq]
            rpm = [p for p in rpm if p[0] <= args.max_freq]
        return cfg, thr, rpm, status

    cfg, thr, rpm, status = load_series(args.log)
    if args.raw:
        import numpy as np_align
        raw = load_raw(args.raw)
        status, off = align_raw(np_align, status, raw)
        status = [(t, v) for t, v in status if 0 <= t <= cfg['duration']]
        print('raw log aligned: offset %.3fs, %u samples' % (off, len(status)))
        chirp_f = Chirp(cfg['duration'], cfg['f_start'], cfg['f_stop'],
                        cfg['fade_in'], cfg['fade_out'])
        def f_of_t2(t):
            if t <= chirp_f.const_freq:
                return cfg['f_start']
            e = (t - chirp_f.const_freq) / (cfg['duration'] - chirp_f.const_freq)
            return cfg['f_start'] * math.exp(chirp_f.B * e)
        rpm = [(f_of_t2(t), v) for t, v in status]
        if args.max_freq:
            rpm = [p for p in rpm if p[0] <= args.max_freq]
    fig, (ax_t, ax_r) = plt.subplots(2, 1, sharex=True, figsize=(12, 7))

    ax_t.plot([f for f, _ in thr], [v for _, v in thr], lw=0.6, color='tab:blue')
    ax_t.set_ylabel('throttle demand')
    ax_t.set_xscale('log')

    ax_r.plot([f for f, _ in rpm], [v for _, v in rpm],
              lw=0.6, color='tab:blue', label=args.log)
    if args.compare:
        _, _, rpm2, _ = load_series(args.compare)
        ax_r.plot([f for f, _ in rpm2], [v for _, v in rpm2],
                  lw=0.6, color='tab:orange', alpha=0.8, label=args.compare)
    ax_r.set_ylabel('rpm')
    ax_r.set_xlabel('chirp frequency (Hz)')
    fig.suptitle('chirp %.2f..%.1f Hz, throttle %.2f +/- %.2f' %
                 (cfg['f_start'], cfg['f_stop'], cfg['mid'], cfg['amp']))
    ax_r.legend(fontsize=8)
    fig.tight_layout()

    def draw_fit(res):
        '''overlay the fitted model envelopes on the rpm panel'''
        styles = {'1-pole': ('--', 'black'), '2-pole': (':', 'tab:purple'),
                  'mixed': ('-', 'tab:orange'), 'slew': ('-.', 'tab:cyan')}
        for name, m in res['models'].items():
            style, colour = styles.get(name, ('--', 'gray'))
            lw = 2.0 if name == res['best'] else 1.3
            fr = [e['freq'] for e in m['menv']]
            desc = 'tau up %.0fms dn %.0fms' % (m['tau_up'] * 1000, m['tau_dn'] * 1000)
            if m['tau_dn2']:
                desc += '+%.0fms' % (m['tau_dn2'] * 1000)
            if m['slew']:
                desc += ' slew %.0fk rpm/s' % (m['slew'] / 1000)
            ax_r.plot(fr, [e['hi'] for e in m['menv']], style, lw=lw, color=colour,
                      alpha=0.9, label='%s (%s, rms %.0f)' % (name, desc, m['rms']))
            ax_r.plot(fr, [e['lo'] for e in m['menv']], style, lw=lw, color=colour,
                      alpha=0.9)
        b = res['models'][res['best']]
        thresh = 1 / math.sqrt(2)
        for f, y, colour in ((b['mf3_up'], res['centre'] + thresh * res['amp0'], 'tab:green'),
                             (b['mf3_dn'], res['centre'] - thresh * res['amp0'], 'tab:red')):
            if f:
                ax_r.plot([f], [y], 'o', ms=10, mfc='none', mec=colour, mew=2, zorder=5)
                ax_r.axvline(f, color=colour, ls=':', lw=1, alpha=0.7)
                ax_r.annotate('%.1f Hz' % f, (f, y), textcoords='offset points',
                              xytext=(6, 8), color=colour, fontsize=9, fontweight='bold')
        ax_r.legend(fontsize=8)

    have_numpy = True
    try:
        import numpy as np
    except ImportError:
        have_numpy = False
        print('numpy not available, skipping the model fit overlay')

    if not have_numpy:
        pass
    elif args.save:
        env = cycle_envelope(status, cfg)
        if args.max_freq:
            env = [e for e in env if e['freq'] <= args.max_freq]
        centre, amp0, _, _, _ = envelope_numbers(env)
        draw_fit(compute_fit(np, status, cfg, centre, amp0, progress=print,
                             max_freq=args.max_freq))
    else:
        # show the data now, fit in the background
        import threading
        env = cycle_envelope(status, cfg)
        if args.max_freq:
            env = [e for e in env if e['freq'] <= args.max_freq]
        centre, amp0, _, _, _ = envelope_numbers(env)
        holder = {}
        fit_note = ax_r.text(0.02, 0.04, 'fitting...', transform=ax_r.transAxes,
                             color='tab:blue', fontsize=10)

        def worker():
            try:
                holder['res'] = compute_fit(np, status, cfg, centre, amp0,
                                            progress=lambda m: holder.__setitem__('msg', m),
                                            max_freq=args.max_freq)
            except Exception as ex:
                holder['err'] = str(ex)

        threading.Thread(target=worker, daemon=True).start()
        timer = fig.canvas.new_timer(interval=200)

        def on_timer():
            if 'msg' in holder:
                fit_note.set_text(holder.pop('msg'))
                fig.canvas.draw_idle()
            if 'res' in holder:
                draw_fit(holder.pop('res'))
                fit_note.set_text('')
                fig.canvas.draw_idle()
                timer.stop()
            if 'err' in holder:
                fit_note.set_text('fit failed: %s' % holder.pop('err'))
                fig.canvas.draw_idle()
                timer.stop()

        timer.add_callback(on_timer)
        timer.start()

    if args.save:
        fig.savefig(args.save, dpi=120)
        print('saved %s' % args.save)
    else:
        plt.show()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='cmd', required=True)

    p = sub.add_parser('run')
    sim_clock.add_argument(p)
    p.add_argument('--uri', default='mcast:0:lo')
    p.add_argument('--node-id', type=int, required=True)
    p.add_argument('--client-node-id', type=int, default=100)
    p.add_argument('--esc-index', type=int, default=0)
    p.add_argument('--rate', type=float, default=200, help='RawCommand rate Hz')
    p.add_argument('--log', required=True)
    p.add_argument('--throttle-mid', type=float, default=0.35)
    p.add_argument('--throttle-amp', type=float, default=0.15)
    p.add_argument('--f-start', type=float, default=0.5)
    p.add_argument('--f-stop', type=float, default=40.0)
    p.add_argument('--duration', type=float, default=45.0)
    p.add_argument('--fade-in', type=float, default=3.0)
    p.add_argument('--fade-out', type=float, default=1.0)
    p.add_argument('--max-throttle', type=float, default=0.6)
    p.add_argument('--max-current', type=float, default=6.0)
    p.add_argument('--max-temp', type=float, default=80.0)
    p.add_argument('--ready-timeout', type=float, default=15.0)
    p.add_argument('--arm-time', type=float, default=1.5)
    p.add_argument('--telem-rate', type=int, default=200)
    p.add_argument('--debug-rate', type=int, default=100)
    p.add_argument('--param', action='append',
                   help='NAME=VALUE param to set before the run (repeatable)')

    p = sub.add_parser('analyze')
    p.add_argument('log')
    p.add_argument('--compare', default=None)

    p = sub.add_parser('plot')
    p.add_argument('log')
    p.add_argument('--compare', default=None)
    p.add_argument('--save', default=None, help='write a png instead of showing a window')
    p.add_argument('--max-freq', type=float, default=None,
                   help='ignore data above this frequency (cuts high-frequency noise)')
    p.add_argument('--raw', default=None,
                   help='SITL --physics-log file: use its raw rpm instead of telemetry')

    p = sub.add_parser('fit')
    p.add_argument('log')
    p.add_argument('--plot', default=None, help='write an envelope/fit png')
    p.add_argument('--max-freq', type=float, default=None,
                   help='ignore data above this frequency (cuts high-frequency noise)')
    p.add_argument('--raw', default=None,
                   help='SITL --physics-log file: use its raw rpm instead of telemetry')

    args = parser.parse_args()
    if args.cmd == 'run':
        cmd_run(args)
    elif args.cmd == 'plot':
        cmd_plot(args)
    elif args.cmd == 'fit':
        cmd_fit(args)
    else:
        cmd_analyze(args)


if __name__ == '__main__':
    main()
