"""Baseline storage and regression comparison."""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, asdict
from pathlib import Path


@dataclass
class Thresholds:
    """Pass/fail gates relative to the baseline."""
    efficiency_drop_pct: float = 3.0       # peak & per-point g/W may drop <= 3%
    ctrl_exec_increase_pct: float = 15.0   # worst control-loop exec time
    ctrl_exec_abs_us_max: float = 45.0     # absolute cap (50 us loop budget)
    cpu_load_increase_pts: float = 10.0    # percentage-POINT increase allowed
    main_loop_increase_pct: float = 25.0
    allow_new_demag: bool = False          # demag_events must not exceed baseline


def save_baseline(metrics: dict, path: str | Path, meta: dict | None = None) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(
        {"meta": meta or {}, "metrics": metrics}, indent=2, sort_keys=True))
    return path


def load_baseline(path: str | Path) -> dict:
    return json.loads(Path(path).read_text())


def _check(name, baseline, current, ok, note=""):
    return {"name": name, "baseline": baseline, "current": current,
            "pass": bool(ok), "note": note}


def _worse_is_lower(baseline, current, drop_pct):
    """current must not fall below baseline by more than drop_pct."""
    if baseline is None or current is None or _nan(baseline) or _nan(current):
        return True  # nothing to compare
    return current >= baseline * (1.0 - drop_pct / 100.0)


def _worse_is_higher(baseline, current, inc_pct, abs_cap=None):
    if baseline is None or current is None or _nan(baseline) or _nan(current):
        return True
    ok = current <= baseline * (1.0 + inc_pct / 100.0)
    if abs_cap is not None:
        ok = ok and current <= abs_cap
    return ok


def _nan(x) -> bool:
    return isinstance(x, float) and math.isnan(x)


def compare(current: dict, baseline: dict, thr: Thresholds | None = None) -> dict:
    thr = thr or Thresholds()
    base = baseline["metrics"] if "metrics" in baseline else baseline
    cs, bs = current["summary"], base["summary"]
    checks = []

    checks.append(_check(
        "peak_efficiency_gf_per_w", bs.get("peak_efficiency_gf_per_w"),
        cs.get("peak_efficiency_gf_per_w"),
        _worse_is_lower(bs.get("peak_efficiency_gf_per_w"),
                        cs.get("peak_efficiency_gf_per_w"), thr.efficiency_drop_pct),
        f"<= {thr.efficiency_drop_pct}% drop"))

    checks.append(_check(
        "worst_ctrl_exec_us", bs.get("worst_ctrl_exec_us"),
        cs.get("worst_ctrl_exec_us"),
        _worse_is_higher(bs.get("worst_ctrl_exec_us"), cs.get("worst_ctrl_exec_us"),
                         thr.ctrl_exec_increase_pct, thr.ctrl_exec_abs_us_max),
        f"<= +{thr.ctrl_exec_increase_pct}% and <= {thr.ctrl_exec_abs_us_max}us"))

    checks.append(_check(
        "worst_main_loop_us", bs.get("worst_main_loop_us"),
        cs.get("worst_main_loop_us"),
        _worse_is_higher(bs.get("worst_main_loop_us"), cs.get("worst_main_loop_us"),
                         thr.main_loop_increase_pct),
        f"<= +{thr.main_loop_increase_pct}%"))

    # CPU load: percentage-point increase
    b_cpu, c_cpu = bs.get("max_cpu_load_pct"), cs.get("max_cpu_load_pct")
    cpu_ok = (b_cpu is None or c_cpu is None or _nan(b_cpu) or _nan(c_cpu)
              or c_cpu <= b_cpu + thr.cpu_load_increase_pts)
    checks.append(_check("max_cpu_load_pct", b_cpu, c_cpu, cpu_ok,
                         f"<= +{thr.cpu_load_increase_pts} points"))

    # demag events
    b_dem, c_dem = bs.get("demag_events", 0), cs.get("demag_events", 0)
    dem_ok = c_dem <= b_dem if not thr.allow_new_demag else True
    checks.append(_check("demag_events", b_dem, c_dem, dem_ok,
                         "must not exceed baseline"))

    # per-point efficiency
    base_pts = {p["segment"]: p for p in base.get("steady_points", [])}
    for p in current.get("steady_points", []):
        bp = base_pts.get(p["segment"])
        if bp is None:
            continue
        ok = _worse_is_lower(bp.get("eff_gf_per_w"), p.get("eff_gf_per_w"),
                             thr.efficiency_drop_pct)
        checks.append(_check(f"eff@{p['segment']}", bp.get("eff_gf_per_w"),
                             p.get("eff_gf_per_w"), ok,
                             f"<= {thr.efficiency_drop_pct}% drop"))

    passed = all(c["pass"] for c in checks)
    return {"passed": passed, "checks": checks, "thresholds": asdict(thr)}
