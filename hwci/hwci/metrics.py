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
    rate = _loop_iter_rate(t, iters)
    # Median, not max: one glitched sample must not become the reference the
    # whole run's load is scaled by.
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


def _tail(idx: np.ndarray, fraction: float) -> np.ndarray:
    if len(idx) == 0:
        return idx
    start = int(len(idx) * (1.0 - fraction))
    return idx[start:]


def _nanmean(a: np.ndarray) -> float:
    return float(np.nanmean(a)) if a.size and not np.all(np.isnan(a)) else float("nan")


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

    steady_points = []
    for s in profile.segments:
        if not s.steady:
            continue
        idx = np.where(seg == s.label)[0]
        tail = _tail(idx, profile.steady_tail_fraction)
        if tail.size == 0:
            continue
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

    # Stand-RPM collapse while commanded throttle is high and steady: catches
    # desyncs whose transient firmware flags fall between SWD samples. The
    # reference is the running max RPM seen since the throttle last moved, so
    # spool-up after a step never reads as a collapse.
    rpm_drop_samples = 0
    frac = profile.demag_rpm_drop_fraction
    ref_rpm = float("nan")
    for i in range(len(rows)):
        high = throttle[i] == throttle[i] and throttle[i] > 0.5
        steady_cmd = (i > 0 and throttle[i - 1] == throttle[i - 1]
                      and abs(throttle[i] - throttle[i - 1]) < 0.02)
        if not (high and steady_cmd):
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
