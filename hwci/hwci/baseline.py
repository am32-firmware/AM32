"""Baseline storage and regression comparison.

Comparison FAILS CLOSED: a gated metric that is missing, ``None``, or ``NaN``
on either side fails its check. A dead instrumentation channel (loose SWD
cable, unplugged telemetry wire, misconfigured backend) produces empty metrics,
and an empty metric must read as "cannot prove no regression", never as PASS.
Channel-coverage checks additionally verify that every channel that was alive
when the baseline was captured is still alive in the current run.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, asdict
from pathlib import Path

FORMAT_VERSION = 1

# Instrumentation channels whose per-run sample coverage is gated.
_CHANNELS = ("perf", "stand", "telem")


@dataclass
class Thresholds:
    """Pass/fail gates relative to the baseline."""
    # peak & per-point g/W may drop this much. History, all measured on the
    # ARK 4IN1 + JS Technology 2306 1800KV + HQProp 5136 bench:
    #   - 3% initial guess, then 15%, then 25%: successive same-firmware
    #     repeatability checks kept swinging 8-20% per point. Root cause
    #     turned out to be prop-wake impingement on the stand's mounting
    #     plate (5" disc entirely inside the plate footprint, wake blowing
    #     into it) - cancelling ~75% of true thrust and buffeting the load
    #     cell (per-sample CV 55-66%).
    #   - back to 15% after the prop was reversed to exhaust into free air:
    #     four interleaved captures (2 firmwares x 2 runs) show worst
    #     gated-point (>= 20W) run-to-run spread of 9.8%, so 15% is ~1.5x
    #     the observed worst case. Tighten further only with more repeat
    #     captures demonstrating headroom.
    efficiency_drop_pct: float = 15.0
    # g/W below this magnitude is load-cell noise (no-prop rig): efficiency is
    # not a meaningful signal there and is not gated (the check reports "not
    # gated" instead of flapping on noise around zero).
    efficiency_floor_gf_per_w: float = 0.5
    # Below this baseline electrical power, thrust/power is dominated by
    # measurement noise even WITH a prop (observed: 10% throttle at 2.3W
    # swung -73% run-to-run; 20% throttle at 19W swung -20%) - the ratio of
    # two small noisy numbers, not a meaningful efficiency figure. Applies to
    # per-point checks AND to which points count toward "peak efficiency"
    # (excluding them keeps peak from being hijacked by the noisiest point).
    efficiency_min_power_w: float = 20.0
    ctrl_exec_increase_pct: float = 15.0   # worst control-loop exec time
    # Absolute slack for the loop-time gates: allowed even when the relative
    # gate is tighter (a 20us baseline must not fail on +4us of jitter).
    ctrl_exec_abs_us_slack: float = 45.0
    cpu_load_increase_pts: float = 10.0    # percentage-POINT increase allowed
    main_loop_increase_pct: float = 25.0
    allow_new_demag: bool = False          # demag_events must not exceed baseline
    min_coverage_fraction: float = 0.5     # channel coverage vs baseline coverage


def save_baseline(metrics: dict, path: str | Path, meta: dict | None = None) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(
        {"format_version": FORMAT_VERSION, "meta": meta or {},
         "metrics": metrics}, indent=2, sort_keys=True))
    return path


def load_baseline(path: str | Path) -> dict:
    return json.loads(Path(path).read_text())


def _check(name, baseline, current, ok, note=""):
    return {"name": name, "baseline": baseline, "current": current,
            "pass": bool(ok), "note": note}


def _nan(x) -> bool:
    return isinstance(x, float) and math.isnan(x)


def _missing(x) -> bool:
    return x is None or _nan(x)


def _worse_is_lower(baseline, current, drop_pct):
    """current must not fall below baseline by more than drop_pct.

    The allowance is ``abs(baseline) * pct`` (not ``baseline * pct``) so a
    negative baseline compares sanely - with the naive form an identical
    negative value fails its own baseline (found by self-comparing the first
    hardware baseline).
    """
    if _missing(baseline) or _missing(current):
        return False  # fail closed: cannot prove no regression
    return current >= baseline - abs(baseline) * drop_pct / 100.0


def _worse_is_higher(baseline, current, inc_pct, abs_slack=None):
    """current must not exceed baseline by more than inc_pct (relative) or
    abs_slack (absolute), whichever allows MORE - the absolute term is a
    noise floor for small baselines, not a cap on the value itself."""
    if _missing(baseline) or _missing(current):
        return False  # fail closed
    limit = baseline * (1.0 + inc_pct / 100.0)
    if abs_slack is not None:
        limit = max(limit, baseline + abs_slack)
    return current <= limit


def compare(current: dict, baseline: dict, thr: Thresholds | None = None,
            current_meta: dict | None = None) -> dict:
    thr = thr or Thresholds()
    base = baseline["metrics"] if "metrics" in baseline else baseline
    cs, bs = current["summary"], base["summary"]
    checks = []

    # Identity: a baseline captured for another target/profile must not gate
    # this run (renamed profiles / new board revs otherwise mis-compare).
    bmeta = baseline.get("meta", {}) if isinstance(baseline.get("meta"), dict) else {}
    if current_meta:
        for key in ("target", "profile"):
            b, c = bmeta.get(key), current_meta.get(key)
            if b and c:
                checks.append(_check(
                    f"baseline_{key}", b, c, b == c, "identities must match"))

    def _eff_check(name, b, c, power_w=None):
        """g/W gate with two noise floors, both learned from the bench:
        a baseline captured without a prop has efficiency values that are
        load-cell noise around zero (magnitude floor), and even WITH a prop
        low-power points are a ratio of two small noisy numbers (power
        floor: 10% throttle at 2.3W swung -73% run-to-run on an unchanged
        firmware/hardware repeat capture)."""
        if not _missing(b) and abs(b) < thr.efficiency_floor_gf_per_w:
            return _check(name, b, c, True,
                          f"baseline |g/W| < {thr.efficiency_floor_gf_per_w} "
                          "(no-prop noise): not gated")
        if power_w is not None and not _missing(power_w) and power_w < thr.efficiency_min_power_w:
            return _check(name, b, c, True,
                          f"baseline power {power_w:.1f}W < "
                          f"{thr.efficiency_min_power_w}W: not gated")
        return _check(name, b, c, _worse_is_lower(b, c, thr.efficiency_drop_pct),
                      f"<= {thr.efficiency_drop_pct}% drop; missing fails")

    # per-point efficiency: gate every segment present in either side; a
    # steady segment that vanished from the current run fails closed. Needed
    # here (ahead of the peak check below) because "peak efficiency" for the
    # gate is recomputed from these points, not trusted from the summary
    # scalar - see that check for why.
    base_pts = {p["segment"]: p for p in base.get("steady_points", [])}
    cur_pts = {p["segment"]: p for p in current.get("steady_points", [])}

    # peak_efficiency_gf_per_w: recomputed from steady_points restricted to
    # baseline power >= efficiency_min_power_w, NOT taken from the summary
    # scalar. "Peak" is a max() over all throttle points, and a max amplifies
    # whichever point is noisiest - on the bench the literal peak was always
    # the lowest-power point (10% throttle, 2.3W), so gating the raw scalar
    # gated pure noise every time. Falls back to the scalar for baselines
    # captured before steady_points existed.
    if base_pts and cur_pts:
        base_gate_pts = [p for p in base_pts.values()
                         if not _missing(p.get("elec_power_w"))
                         and p["elec_power_w"] >= thr.efficiency_min_power_w]
        cur_gate_pts = [p for p in cur_pts.values()
                        if not _missing(p.get("elec_power_w"))
                        and p["elec_power_w"] >= thr.efficiency_min_power_w]
        b_peak = max((p["eff_gf_per_w"] for p in base_gate_pts), default=None)
        c_peak = max((p["eff_gf_per_w"] for p in cur_gate_pts), default=None)
    else:
        b_peak = bs.get("peak_efficiency_gf_per_w")
        c_peak = cs.get("peak_efficiency_gf_per_w")
    checks.append(_eff_check("peak_efficiency_gf_per_w", b_peak, c_peak))

    # Loop-time gates use the STEADY-window worst case when the baseline has
    # it: the raw run-max is dominated by motor start/stop transients that
    # vary 30%+ run-to-run (705 vs 950 us observed back-to-back on the bench)
    # and would make the gate flap. Old baselines without the steady keys
    # fall back to the run-max.
    for base_key, pct, slack in (
            ("worst_ctrl_exec_us", thr.ctrl_exec_increase_pct,
             thr.ctrl_exec_abs_us_slack),
            ("worst_main_loop_us", thr.main_loop_increase_pct, None)):
        key = (f"{base_key}_steady"
               if not _missing(bs.get(f"{base_key}_steady")) else base_key)
        slack_note = f" or +{slack}us" if slack is not None else ""
        checks.append(_check(
            key, bs.get(key), cs.get(key),
            _worse_is_higher(bs.get(key), cs.get(key), pct, slack),
            f"<= +{pct}%{slack_note}; missing fails"))

    # CPU load: percentage-point increase
    b_cpu, c_cpu = bs.get("max_cpu_load_pct"), cs.get("max_cpu_load_pct")
    cpu_ok = (not _missing(b_cpu) and not _missing(c_cpu)
              and c_cpu <= b_cpu + thr.cpu_load_increase_pts)
    checks.append(_check("max_cpu_load_pct", b_cpu, c_cpu, cpu_ok,
                         f"<= +{thr.cpu_load_increase_pts} points; missing fails"))

    # demag events
    b_dem, c_dem = bs.get("demag_events"), cs.get("demag_events")
    dem_ok = (thr.allow_new_demag
              or (not _missing(b_dem) and not _missing(c_dem) and c_dem <= b_dem))
    checks.append(_check("demag_events", b_dem, c_dem, dem_ok,
                         "must not exceed baseline; missing fails"))

    # Instrumentation coverage: every channel alive at baseline capture must
    # still deliver samples now, else its gates above passed vacuously... which
    # they no longer do, but this check names the DEAD CHANNEL explicitly.
    b_tot, c_tot = bs.get("n_samples"), cs.get("n_samples")
    for chan in _CHANNELS:
        b_n = bs.get(f"{chan}_sample_count")
        if _missing(b_n) or not b_n or _missing(b_tot) or not b_tot:
            continue  # baseline (older format) has no coverage info
        c_n = cs.get(f"{chan}_sample_count")
        b_ratio = b_n / b_tot
        c_ratio = (c_n / c_tot) if not _missing(c_n) and not _missing(c_tot) and c_tot else None
        ok = c_ratio is not None and c_ratio >= thr.min_coverage_fraction * b_ratio
        checks.append(_check(
            f"{chan}_coverage", round(b_ratio, 3),
            round(c_ratio, 3) if c_ratio is not None else None, ok,
            f">= {thr.min_coverage_fraction}x baseline coverage "
            f"(dead {chan} channel?)"))

    # per-point efficiency (base_pts/cur_pts computed above, ahead of the
    # peak check): gate every segment present in either side; a steady
    # segment that vanished from the current run fails closed.
    for label, bp in base_pts.items():
        p = cur_pts.get(label)
        if p is None:
            checks.append(_check(f"eff@{label}", bp.get("eff_gf_per_w"), None,
                                 False, "segment missing from current run"))
            continue
        checks.append(_eff_check(f"eff@{label}", bp.get("eff_gf_per_w"),
                                 p.get("eff_gf_per_w"), bp.get("elec_power_w")))

    passed = all(c["pass"] for c in checks)
    return {"passed": passed, "checks": checks, "thresholds": asdict(thr)}
