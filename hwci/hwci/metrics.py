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
    t = _col(rows, "t")
    iters = _col(rows, "perf_loop_iters")
    throttle = _col(rows, "throttle_cmd")
    rate = _loop_iter_rate(t, iters)
    idle_mask = (throttle < 0.02) & ~np.isnan(rate)
    if idle_mask.any():
        idle_rate = float(np.nanmax(rate[idle_mask]))
    elif not np.all(np.isnan(rate)):
        idle_rate = float(np.nanmax(rate))
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
    ctrl_exec = _col(rows, "perf_ctrl_exec_us_max")
    ctrl_pmax = _col(rows, "perf_ctrl_period_us_max")
    ctrl_pmin = _col(rows, "perf_ctrl_period_us_min")
    main_max = _col(rows, "perf_main_loop_us_max")

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
            "rpm": round(_nanmean(_col(rows, "stand_rpm")[tail]), 1),
            "thrust_gf": round(_nanmean(thrust_gf[tail]), 2),
            "current_a": round(_nanmean(current[tail]), 3),
            "voltage_v": round(_nanmean(_col(rows, "stand_voltage_v")[tail]), 3),
            "elec_power_w": round(_nanmean(_col(rows, "stand_elec_power_w")[tail]), 2),
            "eff_gf_per_w": round(_nanmean(eff[tail]), 3),
            "ctrl_exec_us_max": _safe_max(ctrl_exec[tail]),
            "cpu_load_pct": round(_nanmean(load[tail]), 1),
        })

    demag = detect_demag(run, profile)

    summary = {
        "max_thrust_gf": _safe_max(thrust_gf),
        "max_current_a": round(_safe_maxf(current), 2),
        "peak_efficiency_gf_per_w": round(
            max((p["eff_gf_per_w"] for p in steady_points), default=float("nan")), 3),
        "worst_ctrl_exec_us": _safe_max(ctrl_exec),
        "worst_ctrl_period_us": _safe_max(ctrl_pmax),
        "best_ctrl_period_us": _safe_minf(ctrl_pmin),
        "worst_main_loop_us": _safe_max(main_max),
        "max_cpu_load_pct": round(_safe_maxf(load), 1),
        "idle_loop_rate_hz": round(idle_rate, 1) if idle_rate == idle_rate else None,
        "demag_events": demag["event_count"],
        "bemf_timeout_samples": demag["bemf_timeout_samples"],
    }
    return {"summary": summary, "steady_points": steady_points, "demag": demag}


def detect_demag(run: RunResult, profile: Profile) -> dict:
    rows = run.rows
    throttle = _col(rows, "throttle_cmd")
    comm = _col(rows, "perf_commutation_interval")
    bemf = _col(rows, "perf_bemf_timeout")
    stand_rpm = _col(rows, "stand_rpm")
    esc_erpm = _col(rows, "esc_erpm")
    pp = profile.pole_pairs

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
        if bemf[i] == 1:
            bemf_samples += 1
            anom = True
        if comm[i] == comm[i] and comm[i] > spike_thr:
            spike_samples += 1
            anom = True
        flag[i] = anom

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
