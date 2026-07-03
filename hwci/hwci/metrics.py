"""Compute performance metrics from a run.

Produces three things the baseline/report care about:

* steady-state operating points (thrust, power, efficiency g/W, plus loop time
  and CPU load at each throttle),
* worst-case loop timing and CPU load over the whole run,
* host-side demag / desync detection (firmware bemf-timeout flag, commutation-
  interval spikes, RPM collapse, and ESC-eRPM vs stand-RPM divergence).

CPU load uses the idle-residual method: the firmware exposes a free-running
``loop_iters`` counter; its rate (iters/s) is highest when the core is least
loaded, so ``cpu_load = 1 - rate/idle_rate`` where ``idle_rate`` is the rate
measured at zero throttle. This is how CPU load is recovered on a Cortex-M0,
which has no cycle counter.
"""
from __future__ import annotations

import numpy as np

from .config import Profile
from .model import RunResult


def _col(rows: list[dict], name: str) -> np.ndarray:
    out = np.full(len(rows), np.nan)
    for i, r in enumerate(rows):
        v = r.get(name)
        if v is not None and v != "":
            try:
                out[i] = float(v)
            except (TypeError, ValueError):
                pass
    return out


def _loop_iter_rate(t: np.ndarray, iters: np.ndarray) -> np.ndarray:
    """Per-sample loop-iteration rate (Hz), NaN where undefined."""
    rate = np.full(len(t), np.nan)
    dt = np.diff(t)
    di = np.diff(iters)
    with np.errstate(invalid="ignore", divide="ignore"):
        r = np.where((dt > 0) & (di >= 0), di / dt, np.nan)
    rate[1:] = r
    return rate


def _cpu_load(rows: list[dict]) -> tuple[np.ndarray, float]:
    # Prefer the timestamp taken at the actual SWD read (perf_host_t) over the
    # sample-loop schedule time: a host stall between ticks would otherwise
    # attribute too many loop iterations to too little time and corrupt the
    # rate for that sample.
    t_host = _col(rows, "perf_host_t")
    t = t_host if not np.all(np.isnan(t_host)) else _col(rows, "t")
    iters = _col(rows, "perf_loop_iters")
    throttle = _col(rows, "throttle_cmd")
    running = _col(rows, "perf_running")
    rate = _loop_iter_rate(t, iters)
    # Median, not max: one glitched sample must not become the reference the
    # whole run's load is scaled by.
    #
    # The reference must be "motor genuinely stopped" (firmware-reported
    # perf_running == 0), NOT "commanded throttle below a small cutoff": a
    # segment that RAMPS UP FROM ZERO (e.g. efficiency_sweep's t10, 0%->10%)
    # commands throttle under any such cutoff for real, non-idle time while
    # the motor is actively spinning up and commutating - a genuine load, not
    # idle. Observed on the bench: lengthening t10's hold pushed these
    # ramp-up samples from ~37% to ~50% of the throttle-based reference pool,
    # collapsing the reported idle rate from ~60k to ~46k iters/s and
    # understating max_cpu_load_pct by 10+ points. perf_running falls back to
    # the throttle heuristic only for run data recorded before that column
    # existed.
    if not np.all(np.isnan(running)):
        idle_mask = (running == 0) & ~np.isnan(rate)
    else:
        idle_mask = (throttle < 0.02) & ~np.isnan(rate)
    if idle_mask.any():
        idle_rate = float(np.nanmedian(rate[idle_mask]))
    elif not np.all(np.isnan(rate)):
        idle_rate = float(np.nanmedian(rate))
    else:
        return np.full(len(rows), np.nan), float("nan")
    if idle_rate <= 0:
        return np.full(len(rows), np.nan), idle_rate
    load = 100.0 * (1.0 - rate / idle_rate)
    return np.clip(load, 0.0, 100.0), idle_rate


def tail_start_index(n: int, fraction: float) -> int:
    """First index of the trailing ``fraction`` of ``n`` samples.

    The single source of truth for "where does the steady tail begin" -
    hwci.runner's mid-segment perf-stats reset computes its own tick relative
    to THIS function (with a safety margin) specifically so the two can never
    drift apart. They used to be two independently-rounded formulas that
    disagreed by a tick in some segment lengths, which is exactly the kind of
    gap a reset-propagation race can hide in.
    """
    if n <= 0:
        return 0
    return int(n * (1.0 - fraction))


def _tail(idx: np.ndarray, fraction: float) -> np.ndarray:
    if len(idx) == 0:
        return idx
    return idx[tail_start_index(len(idx), fraction):]


def _nanmean(a: np.ndarray) -> float:
    return float(np.nanmean(a)) if a.size and not np.all(np.isnan(a)) else float("nan")


_U32 = 1 << 32


def _wrap32(a: float, b: float) -> int:
    """b - a for firmware u32 counters, correct across a single wraparound."""
    return (int(b) - int(a)) % _U32


def zc_jitter_window(count: np.ndarray, jsum: np.ndarray, isum: np.ndarray,
                     jmax: np.ndarray, idx: np.ndarray) -> dict:
    """Zero-cross jitter over the sample window ``idx``.

    The firmware accumulates |interval deviation| and raw interval per
    commutation into monotonic u32 sums (struct v2, see HWCI_PERF_ZC in
    Inc/hwci_perf.h), so differencing the first and last valid snapshot in the
    window yields per-commutation means with EVERY commutation counted -
    immune to the ~200 Hz host sampling rate, which is far below the multi-kHz
    commutation rate. Returns:

    * ``mean_pct``  - mean |deviation| as % of mean interval (the headline
      zero-cross detection noise figure),
    * ``max_pct``   - worst single deviation (sticky firmware max, reset by
      the runner at each steady tail) as % of mean interval,
    * both ``None`` when the firmware predates v2 or the window saw no
      accumulated commutations (motor stopped / startup-gated).
    """
    none = {"mean_pct": None, "max_pct": None}
    if idx.size == 0:
        return none
    valid = idx[~np.isnan(count[idx]) & ~np.isnan(jsum[idx]) & ~np.isnan(isum[idx])]
    if valid.size < 2:
        return none
    a, b = valid[0], valid[-1]
    n = _wrap32(count[a], count[b])
    d_int = _wrap32(isum[a], isum[b])
    if n <= 0 or d_int <= 0:
        return none
    d_jit = _wrap32(jsum[a], jsum[b])
    mean_interval = d_int / n
    out = {"mean_pct": round(100.0 * d_jit / d_int, 3), "max_pct": None}
    m = jmax[valid]
    if not np.all(np.isnan(m)):
        out["max_pct"] = round(100.0 * float(np.nanmax(m)) / mean_interval, 2)
    return out


def compute(run: RunResult, profile: Profile) -> dict:
    rows = run.rows
    seg = np.array([r.get("segment") for r in rows], dtype=object)
    load, idle_rate = _cpu_load(rows)

    thrust_gf = _col(rows, "stand_thrust_gf")
    current = _col(rows, "stand_current_a")
    eff = _col(rows, "stand_eff_gf_per_w")
    stand_rpm = _col(rows, "stand_rpm")
    stand_v = _col(rows, "stand_voltage_v")
    stand_pw = _col(rows, "stand_elec_power_w")
    motor_temp = _col(rows, "stand_motor_temp_c")
    fet_temp = _col(rows, "stand_fet_temp_c")
    ctrl_exec = _col(rows, "perf_ctrl_exec_us_max")
    ctrl_pmax = _col(rows, "perf_ctrl_period_us_max")
    ctrl_pmin = _col(rows, "perf_ctrl_period_us_min")
    main_max = _col(rows, "perf_main_loop_us_max")
    perf_iters = _col(rows, "perf_loop_iters")
    esc_erpm = _col(rows, "esc_erpm")
    zc_count = _col(rows, "perf_zc_count")
    zc_jsum = _col(rows, "perf_zc_jitter_sum")
    zc_isum = _col(rows, "perf_zc_interval_sum")
    zc_jmax = _col(rows, "perf_zc_jitter_max")

    steady_points = []
    for s in profile.segments:
        if not s.steady:
            continue
        idx = np.where(seg == s.label)[0]
        tail = _tail(idx, profile.steady_tail_fraction)
        if tail.size == 0:
            continue
        jitter = zc_jitter_window(zc_count, zc_jsum, zc_isum, zc_jmax, tail)
        steady_points.append({
            "segment": s.label,
            "throttle": s.throttle,
            "rpm": round(_nanmean(stand_rpm[tail]), 1),
            "thrust_gf": round(_nanmean(thrust_gf[tail]), 2),
            "current_a": round(_nanmean(current[tail]), 3),
            "voltage_v": round(_nanmean(stand_v[tail]), 3),
            "elec_power_w": round(_nanmean(stand_pw[tail]), 2),
            "eff_gf_per_w": round(_nanmean(eff[tail]), 3),
            "ctrl_exec_us_max": _safe_max(ctrl_exec[tail]),
            "main_loop_us_max": _safe_max(main_max[tail]),
            "cpu_load_pct": round(_nanmean(load[tail]), 1),
            "motor_temp_c": round(_nanmean(motor_temp[tail]), 2),
            "fet_temp_c": round(_nanmean(fet_temp[tail]), 2),
            # zero-cross detection noise (report-only for now: gate once the
            # bench has repeat captures establishing its run-to-run spread)
            "zc_jitter_pct": jitter["mean_pct"],
            "zc_jitter_max_pct": jitter["max_pct"],
        })

    demag = detect_demag(run, profile)

    summary = {
        "max_thrust_gf": _safe_max(thrust_gf),
        "max_current_a": round(_safe_maxf(current), 2),
        "peak_efficiency_gf_per_w": round(
            max((p["eff_gf_per_w"] for p in steady_points), default=float("nan")), 3),
        "worst_ctrl_exec_us": _safe_max(ctrl_exec),
        # Steady-window worst case: the runner resets the firmware's sticky
        # accumulators at each steady tail, so these exclude motor start/stop
        # transients (which vary 30%+ run-to-run) and are the values the
        # baseline gate compares.
        "worst_ctrl_exec_us_steady": max(
            (p["ctrl_exec_us_max"] for p in steady_points
             if p.get("ctrl_exec_us_max") is not None), default=None),
        "worst_ctrl_period_us": _safe_max(ctrl_pmax),
        "best_ctrl_period_us": _safe_minf(ctrl_pmin),
        "worst_main_loop_us": _safe_max(main_max),
        "worst_main_loop_us_steady": max(
            (p["main_loop_us_max"] for p in steady_points
             if p.get("main_loop_us_max") is not None), default=None),
        "max_cpu_load_pct": round(_safe_maxf(load), 1),
        "idle_loop_rate_hz": round(idle_rate, 1) if idle_rate == idle_rate else None,
        # Worst steady-point zero-cross jitter (mean-of-window %, and worst
        # single deviation %). Report-only: not compared by baseline.compare.
        "worst_zc_jitter_pct": max(
            (p["zc_jitter_pct"] for p in steady_points
             if p.get("zc_jitter_pct") is not None), default=None),
        "worst_zc_jitter_max_pct": max(
            (p["zc_jitter_max_pct"] for p in steady_points
             if p.get("zc_jitter_max_pct") is not None), default=None),
        "demag_events": demag["event_count"],
        "bemf_timeout_samples": demag["bemf_timeout_samples"],
        "max_motor_temp_c": _safe_maxf(motor_temp),
        "max_fet_temp_c": _safe_maxf(fet_temp),
        # Channel liveness: how many samples each instrumentation channel
        # actually delivered. The baseline gate fails a run whose coverage
        # collapsed vs the baseline (dead SWD/telemetry/stand channel).
        "n_samples": len(rows),
        "perf_sample_count": int(np.count_nonzero(~np.isnan(perf_iters))),
        "stand_sample_count": int(np.count_nonzero(~np.isnan(thrust_gf))),
        "telem_sample_count": int(np.count_nonzero(~np.isnan(esc_erpm))),
    }
    return {"summary": summary, "steady_points": steady_points, "demag": demag}


def detect_demag(run: RunResult, profile: Profile) -> dict:
    rows = run.rows
    throttle = _col(rows, "throttle_cmd")
    comm = _col(rows, "perf_commutation_interval")
    bemf = _col(rows, "perf_bemf_timeout")
    stand_rpm = _col(rows, "stand_rpm")
    esc_erpm = _col(rows, "esc_erpm")
    # Pole pairs are a property of the rig's motor; the run meta records the
    # rig value at run time. profile.pole_pairs is only the offline fallback.
    pp = int(run.meta.get("pole_pairs") or profile.pole_pairs)

    running = throttle > 0.2
    comm_running = comm[running & ~np.isnan(comm) & (comm > 0)]
    median_comm = float(np.median(comm_running)) if comm_running.size else float("nan")
    spike_thr = median_comm * profile.demag_commutation_spike if median_comm == median_comm else np.inf

    # per-sample anomaly flag
    flag = np.zeros(len(rows), dtype=bool)
    bemf_samples = 0
    spike_samples = 0
    for i in range(len(rows)):
        if not running[i]:
            continue
        anom = False
        # bemf_timeout_happened is an incrementing counter in firmware (1, 2,
        # ... latched at 102 under stuck-rotor protection), not a boolean.
        if bemf[i] >= 1:
            bemf_samples += 1
            anom = True
        if comm[i] == comm[i] and comm[i] > spike_thr:
            spike_samples += 1
            anom = True
        flag[i] = anom

    # Stand-RPM collapse while commanded throttle is high and NOT being
    # intentionally reduced: catches desyncs whose transient firmware flags
    # fall between SWD samples. The reference is the running max RPM seen
    # since the throttle last decreased, so spool-up after a step never
    # reads as a collapse.
    #
    # The gate must check the THROTTLE TREND (falling vs. not), not just the
    # size of one tick's change: a smooth multi-second ramp-down moves by a
    # tiny amount each 10ms tick, so a small-delta check like "< 0.02" never
    # fires and the reference RPM is never invalidated - actual RPM then
    # falls (correctly, because the ramp commanded it to) while the stale
    # peak reference stays pinned at the ramp's starting RPM, eventually
    # tripping the drop threshold. Observed on the bench: efficiency_sweep's
    # 4s 100%->0% rampdn flagged a false demag event with zero bemf timeouts,
    # commutation spikes, or eRPM/stand-RPM mismatch - RPM was tracking
    # throttle exactly as commanded.
    rpm_drop_samples = 0
    frac = profile.demag_rpm_drop_fraction
    ref_rpm = float("nan")
    for i in range(len(rows)):
        high = throttle[i] == throttle[i] and throttle[i] > 0.5
        not_decreasing = (i > 0 and throttle[i - 1] == throttle[i - 1]
                          and throttle[i] >= throttle[i - 1] - 1e-6)
        if not (high and not_decreasing):
            ref_rpm = float("nan")
            continue
        r = stand_rpm[i]
        if r != r:
            continue
        if ref_rpm != ref_rpm:
            ref_rpm = r
            continue
        ref_rpm = max(ref_rpm, r)
        if ref_rpm > 1000.0 and r < ref_rpm * (1.0 - frac):
            rpm_drop_samples += 1
            flag[i] = True

    # debounce contiguous flagged samples into events (close gaps <= 3)
    events = _events_from_flags(flag, max_gap=3)

    # ESC eRPM vs stand RPM divergence (telemetry desync indicator)
    mismatch = 0
    for i in range(len(rows)):
        if running[i] and esc_erpm[i] == esc_erpm[i] and stand_rpm[i] == stand_rpm[i] \
                and stand_rpm[i] > 500:
            esc_mech = esc_erpm[i] / pp
            if abs(esc_mech - stand_rpm[i]) / stand_rpm[i] > 0.2:
                mismatch += 1

    return {
        "event_count": len(events),
        "events": [{"start_idx": int(a), "end_idx": int(b)} for a, b in events],
        "bemf_timeout_samples": bemf_samples,
        "comm_spike_samples": spike_samples,
        "rpm_drop_samples": rpm_drop_samples,
        "median_commutation_interval": round(median_comm, 1) if median_comm == median_comm else None,
        "esc_rpm_mismatch_samples": mismatch,
    }


def _events_from_flags(flag: np.ndarray, max_gap: int = 3) -> list[tuple[int, int]]:
    events: list[tuple[int, int]] = []
    start = None
    gap = 0
    for i, f in enumerate(flag):
        if f:
            if start is None:
                start = i
            last = i
            gap = 0
        elif start is not None:
            gap += 1
            if gap > max_gap:
                events.append((start, last))
                start = None
    if start is not None:
        events.append((start, last))
    return events


def _safe_max(a: np.ndarray):
    if a.size and not np.all(np.isnan(a)):
        return int(np.nanmax(a))
    return None


def _safe_maxf(a: np.ndarray) -> float:
    return float(np.nanmax(a)) if a.size and not np.all(np.isnan(a)) else float("nan")


def _safe_minf(a: np.ndarray):
    if a.size and not np.all(np.isnan(a)):
        return int(np.nanmin(a))
    return None
