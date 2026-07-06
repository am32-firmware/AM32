"""Auto settings tuner: find the best AM32 EEPROM settings for a motor/prop.

Given a motor/prop on the rig, ``hwci tune`` searches the AM32 settings space
(advance_level, pwm_frequency, variable_pwm, auto_advance, max_ramp, ...) for
the combination that maximizes efficiency (g/W) subject to hard constraints:
no demag/desync/bemf timeouts, zero-cross jitter not regressed vs the default
settings, temperatures bounded, and reliable startup.

Settings trials need NO rebuild: AM32 reads its 192-byte EEprom page once at
boot, so each trial writes the page over SWD (one-shot flash + reset, see
:mod:`hwci.settings`) and runs a short probe profile.

Search shape (dictated by the bench's noise reality - same-firmware efficiency
spread is up to 9.8 % at >= 20 W, and the pack drifts within a session, see
hwci/baseline.py):

* **Coordinate sweeps**, one parameter at a time with the others held at the
  incumbent: advance_level (coarse grid, then refine +/- one step around the
  argmax), then pwm_frequency (variable_pwm forced 0), then a **mode A/B
  stage** on top of the winners ({} incumbent, variable_pwm 1, variable_pwm 2,
  auto_advance 1) with interleaved repeats, then max_ramp as a
  **constraint-only** stage on a step-stress profile.
* **Anchor runs**: the incumbent settings are re-run every ``anchors_every``
  trials; every candidate score is reported raw AND normalized to the linear
  interpolation between surrounding anchors, which cancels pack drift.
* **Finals**: winner vs default, interleaved ABBA blocks on the full
  efficiency sweep plus a startup-reliability check. The winner is confirmed
  only with a positive median paired delta and zero constraint failures.

Every trial is checkpointed to ``manifest.json`` (atomic rewrite), so a
session interrupted by a pack swap, crash, or Ctrl-C resumes exactly where it
left off with ``hwci tune --resume <dir>``.
"""
from __future__ import annotations

import abc
import dataclasses
import hashlib
import json
import math
import os
import statistics
import subprocess
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Optional

import numpy as np
import yaml

from . import metrics as metricsmod
from . import report as reportmod
from .config import (PROFILES_DIR, Profile, RigConfig, load_profile,
                     profile_from_dict, profile_to_dict)
from .debugger.base import MockDebugger
from .model import RunResult
from .runner import (DEFAULT_MIN_CELL_VOLTAGE, BatteryTooLowError,
                     build_live_sources, build_sim_sources, check_battery,
                     run_profile)
from .settings import (DEFAULT_EEPROM_ADDRESS, EEPROM_SIZE, Settings,
                       check_eeprom_layout, default_blob, resolve_eeprom_address,
                       resolve_field)

MANIFEST_VERSION = 1


class TuneSpecError(ValueError):
    """A tune spec failed strict validation (unknown key, bad value)."""


class TunePaused(RuntimeError):
    """The session checkpointed and stopped cleanly (e.g. pack too low with
    --no-prompt); resume later with ``hwci tune --resume <dir>``."""


# ==========================================================================
# Tune spec (strict YAML schema)
# ==========================================================================
def _strict(d: dict | None, ctx: str, known: dict) -> dict:
    """Return d after refusing unknown keys - a typo'd key must never
    silently become 'use the default' (mirrors load_rig strictness)."""
    d = d or {}
    if not isinstance(d, dict):
        raise TuneSpecError(f"tune spec: {ctx} must be a mapping, got {d!r}")
    unknown = sorted(set(d) - set(known))
    if unknown:
        raise TuneSpecError(
            f"tune spec: unknown key(s) {unknown} in {ctx}; "
            f"valid keys: {sorted(known)}")
    return d


@dataclass
class PackSpec:
    min_resting_cell_v: float = 3.5
    prompt_on_swap: bool = True


@dataclass
class ThermalSpec:
    max_start_fet_temp_c: float | None = None
    cool_timeout_s: float = 600.0


@dataclass
class ProbeSpec:
    points: dict[str, float] | None = None   # label -> throttle, ordered
    dwell_s: float = 6.0
    safety: dict = field(default_factory=dict)


@dataclass
class ObjectiveSpec:
    weights: dict[str, float] = field(default_factory=lambda: {
        "t30": 1.0, "t50": 2.0, "t70": 1.0, "t90": 0.5})
    min_power_w: float = 20.0
    noise_floor_pct: float = 3.0


@dataclass
class StartupSpec:
    cycles: int = 5
    max_failed: int = 0
    spin_throttle: float = 0.15
    min_rpm: float = 1000.0


@dataclass
class ConstraintsSpec:
    max_demag_events: int = 0
    max_bemf_timeout_samples: int = 0
    jitter_max_regression_pct: float = 25.0
    max_fet_temp_c: float | None = None
    max_motor_temp_c: float | None = None
    startup: StartupSpec = field(default_factory=StartupSpec)


@dataclass
class ParamSpec:
    name: str
    values: list[int]
    refine_step: int | None = None
    offset: int | None = None      # explicit byte offset (forward-compat)


@dataclass
class StageSpec:
    name: str
    sweep: str | None = None                    # parameter name to grid-sweep
    ab_candidates: list[dict] | None = None     # override dicts to A/B
    fixed: dict = field(default_factory=dict)   # forced during this stage only
    repeats: int = 1
    profile: str | None = None                  # None -> tune probe
    constraint_only: bool = False
    # "grid" tests every listed value; "climb" hill-climbs the sorted value
    # list from the incumbent (valid for unimodal responses like advance or
    # pwm frequency - typically halves the trial count of a wide grid).
    search: str = "grid"
    # "ramp_rate": measure the powertrain (mech time constant + transient
    # current per % of duty lead) with one instrumented step run, COMPUTE
    # max_ramp from the physics, then verify it on the step-stress profile.
    measure: str | None = None
    margin: float = 0.8       # fraction of the physics-derived rate to keep


@dataclass
class FinalsSpec:
    profile: str = "efficiency_sweep"
    pattern: str = "ABBA"
    repeats: int = 2
    startup_check: bool = True


@dataclass
class TuneSpec:
    name: str
    description: str = ""
    battery_cells: int | None = None
    pack: PackSpec = field(default_factory=PackSpec)
    thermal: ThermalSpec = field(default_factory=ThermalSpec)
    probe: ProbeSpec = field(default_factory=ProbeSpec)
    objective: ObjectiveSpec = field(default_factory=ObjectiveSpec)
    constraints: ConstraintsSpec = field(default_factory=ConstraintsSpec)
    anchors_every: int = 5
    parameters: dict[str, ParamSpec] = field(default_factory=dict)
    stages: list[StageSpec] = field(default_factory=list)
    finals: FinalsSpec = field(default_factory=FinalsSpec)

    def param_offsets(self) -> dict[str, int]:
        return {p.name: p.offset for p in self.parameters.values()
                if p.offset is not None}


def _build(cls, d: dict | None, ctx: str, nested: dict | None = None):
    known = {f.name: f for f in dataclasses.fields(cls)}
    d = dict(_strict(d, ctx, known))
    for key, sub in (nested or {}).items():
        if key in d:
            d[key] = sub(d[key])
    return cls(**d)


def tune_spec_from_dict(data: dict) -> TuneSpec:
    known = {f.name: f for f in dataclasses.fields(TuneSpec)}
    data = dict(_strict(data, "top level", known))
    if "name" not in data:
        raise TuneSpecError("tune spec: 'name' is required")

    data["pack"] = _build(PackSpec, data.get("pack"), "pack")
    data["thermal"] = _build(ThermalSpec, data.get("thermal"), "thermal")
    data["probe"] = _build(ProbeSpec, data.get("probe"), "probe")
    data["objective"] = _build(ObjectiveSpec, data.get("objective"), "objective")
    data["constraints"] = _build(
        ConstraintsSpec, data.get("constraints"), "constraints",
        nested={"startup": lambda d: _build(StartupSpec, d,
                                            "constraints.startup")})
    data["finals"] = _build(FinalsSpec, data.get("finals"), "finals")

    params: dict[str, ParamSpec] = {}
    for name, pd in (data.get("parameters") or {}).items():
        pd = _strict(pd, f"parameters.{name}",
                     {"values": None, "refine_step": None, "offset": None})
        if not pd.get("values"):
            raise TuneSpecError(f"tune spec: parameters.{name} needs 'values'")
        p = ParamSpec(name=name, values=[int(v) for v in pd["values"]],
                      refine_step=pd.get("refine_step"),
                      offset=pd.get("offset"))
        f = resolve_field(name, p.offset)   # raises on unknown/read-only
        for v in p.values:
            if not f.lo <= v <= f.hi:
                raise TuneSpecError(
                    f"tune spec: parameters.{name} value {v} outside "
                    f"firmware-valid range [{f.lo}, {f.hi}]")
        params[name] = p
    data["parameters"] = params

    stages: list[StageSpec] = []
    seen = set()
    for sd in data.get("stages") or []:
        s = _build(StageSpec, sd, "stages[]")
        if s.name in seen:
            raise TuneSpecError(f"tune spec: duplicate stage name {s.name!r}")
        seen.add(s.name)
        if sum(map(bool, (s.sweep, s.ab_candidates, s.measure))) != 1:
            raise TuneSpecError(
                f"tune spec: stage {s.name!r} needs exactly one of "
                "'sweep', 'ab_candidates' or 'measure'")
        if s.measure is not None and s.measure != "ramp_rate":
            raise TuneSpecError(
                f"tune spec: stage {s.name!r} measure {s.measure!r} is not "
                "'ramp_rate'")
        if s.measure and s.constraint_only:
            raise TuneSpecError(
                f"tune spec: stage {s.name!r}: 'measure' and "
                "'constraint_only' are mutually exclusive")
        if not 0.1 <= s.margin <= 2.0:
            raise TuneSpecError(
                f"tune spec: stage {s.name!r} margin {s.margin} outside "
                "[0.1, 2.0]")
        if s.search not in ("grid", "climb"):
            raise TuneSpecError(
                f"tune spec: stage {s.name!r} search {s.search!r} is not "
                "'grid' or 'climb'")
        if s.search == "climb" and (not s.sweep or s.constraint_only):
            raise TuneSpecError(
                f"tune spec: stage {s.name!r}: 'climb' only applies to "
                "scored sweep stages")
        if s.sweep and s.sweep not in params:
            raise TuneSpecError(
                f"tune spec: stage {s.name!r} sweeps unknown parameter "
                f"{s.sweep!r}; declare it under parameters:")
        for ov in (s.ab_candidates or []) + [s.fixed]:
            _validate_overrides(ov, params, f"stage {s.name!r}")
        stages.append(s)
    data["stages"] = stages

    if data["finals"].pattern != "ABBA":
        raise TuneSpecError(
            f"tune spec: finals.pattern {data['finals'].pattern!r} is not "
            "supported (only 'ABBA')")
    return TuneSpec(**data)


def _validate_overrides(ov: dict, params: dict[str, ParamSpec], ctx: str):
    if not isinstance(ov, dict):
        raise TuneSpecError(f"tune spec: {ctx}: overrides must be a mapping")
    for name, value in ov.items():
        offset = params[name].offset if name in params else None
        f = resolve_field(name, offset)
        if not f.lo <= int(value) <= f.hi:
            raise TuneSpecError(
                f"tune spec: {ctx}: {name}={value} outside firmware-valid "
                f"range [{f.lo}, {f.hi}]")


def load_tune_spec(path: str | Path) -> TuneSpec:
    data = yaml.safe_load(Path(path).read_text())
    if not isinstance(data, dict):
        raise TuneSpecError(f"tune spec {path}: not a YAML mapping")
    try:
        return tune_spec_from_dict(data)
    except TuneSpecError as e:
        raise TuneSpecError(f"{path}: {e}") from e


# ==========================================================================
# Profiles: probe + inline startup-reliability check
# ==========================================================================
def probe_profile(spec: TuneSpec) -> Profile:
    """The tuner's inner probe (~28 s): tune_probe.yaml with the spec's
    points/dwell/safety overrides applied."""
    d = yaml.safe_load((PROFILES_DIR / "tune_probe.yaml").read_text())
    if spec.probe.points:
        # Keep the low-throttle warmup staircase (see the profile YAML for
        # why it must exist) and rebuild only the measured points.
        segs = [
            {"label": "idle", "throttle": 0.0, "duration_s": 2.0},
            {"label": "w10", "throttle": 0.10, "duration_s": 1.5, "ramp": True},
            {"label": "w20", "throttle": 0.20, "duration_s": 1.5},
        ]
        prev = 0.20
        for label, thr in spec.probe.points.items():
            thr = float(thr)
            # Steps > 10% throttle get a ramp segment: a 0.5->0.7 snap drew a
            # 43-75 A one-sample transient on the 6S bench (harness abort at
            # 40 A), while efficiency_sweep's unramped 10% staircase is
            # bench-proven safe. The dwell stays constant-throttle so the
            # steady tail is unaffected.
            if abs(thr - prev) > 0.10 + 1e-9:
                segs.append({"label": f"r_{label}", "throttle": thr,
                             "duration_s": 1.0, "ramp": True})
            segs.append({"label": label, "throttle": thr,
                         "duration_s": spec.probe.dwell_s, "steady": True})
            prev = thr
        segs.append({"label": "rampdn", "throttle": 0.0, "duration_s": 2.0,
                     "ramp": True})
        d["segments"] = segs
    else:
        for seg in d["segments"]:
            if seg.get("steady"):
                seg["duration_s"] = spec.probe.dwell_s
    if spec.probe.safety:
        d["safety"] = {**(d.get("safety") or {}), **spec.probe.safety}
    return profile_from_dict(d)


def startup_profile(spec: TuneSpec) -> Profile:
    """Inline minimal startup-reliability profile: N cycles of
    {spin low for 1.5 s -> stop 2.5 s}.

    TODO: PR #22's startup_reliability profile/metric is not on this branch;
    delegate to it (profile + its richer per-cycle metric) once merged.
    """
    st = spec.constraints.startup
    segs = [{"label": "idle", "throttle": 0.0, "duration_s": 1.0}]
    for i in range(st.cycles):
        segs.append({"label": f"spin{i}", "throttle": st.spin_throttle,
                     "duration_s": 1.5})
        segs.append({"label": f"stop{i}", "throttle": 0.0, "duration_s": 2.5})
    return profile_from_dict({
        "name": "tune_startup",
        "description": "inline startup-reliability check (auto-tuner)",
        "sample_rate_hz": 100.0,
        "segments": segs,
        "safety": spec.probe.safety or None,
    })


def step_profile(spec: TuneSpec) -> Profile:
    """Step-stress profile for the max_ramp constraint stage: aggressive
    snaps into a high-current regime FROM A SPINNING STATE (0.30 hold).

    Snapping from a stop (as demag_step_stress does) makes the host demag
    detector read the spool-up's long commutation intervals as spike events
    even when nothing desynced; stepping from 0.30 keeps the detector's
    median-interval reference honest, so only a real loss of sync (bemf
    timeouts, interval blow-up, RPM collapse) disqualifies a max_ramp value.
    """
    return profile_from_dict({
        "name": "tune_step",
        "description": "inline step-stress for the max_ramp stage (auto-tuner)",
        "sample_rate_hz": 100.0,
        "segments": [
            {"label": "idle", "throttle": 0.0, "duration_s": 2.0},
            {"label": "spool", "throttle": 0.30, "duration_s": 2.0, "ramp": True},
            {"label": "hold30a", "throttle": 0.30, "duration_s": 2.0},
            {"label": "snap95a", "throttle": 0.95, "duration_s": 1.5},
            {"label": "hold30b", "throttle": 0.30, "duration_s": 2.0},
            {"label": "snap95b", "throttle": 0.95, "duration_s": 1.5},
            {"label": "hold30c", "throttle": 0.30, "duration_s": 2.0},
            {"label": "rampdn", "throttle": 0.0, "duration_s": 1.5, "ramp": True},
        ],
        # The snaps are the point of this profile, and a legitimate
        # 0.3->0.95 snap transient peaks near 50 A on the 6S bench (see
        # demag_step_stress.yaml's 55 A precedent). Probe-level current
        # limits would abort every candidate on that transient before demag
        # is even assessed, so give the current limit snap headroom; all
        # other probe safety limits apply unchanged.
        "safety": {**(spec.probe.safety or {}),
                   "max_current_a": max(
                       (spec.probe.safety or {}).get("max_current_a") or 0.0,
                       55.0)},
    })


def ramp_measure_profile(spec: TuneSpec) -> Profile:
    """Measurement profile for the mech-ramp stage: two moderate snap steps
    from a spinning state (0.20 -> 0.55), sampled at 200 Hz.

    The trial runs with max_ramp at the field maximum so firmware duty slew
    is not the limiter: the eRPM rise time is then the POWERTRAIN's (rotor
    inertia + prop aero load), and the current overshoot measures how much
    a leading duty costs. Two reps -> median estimates. The step tops out
    at 0.55 to stay clear of high-RPM sync margins (this bench desyncs
    arriving at fresh-pack t70)."""
    safety = dict(spec.probe.safety or {})
    # deliberate snaps: same transient headroom rationale as step_profile
    safety["max_current_a"] = max(safety.get("max_current_a") or 0.0, 55.0)
    return profile_from_dict({
        "name": "tune_ramp_measure",
        "description": "inline powertrain step-response measurement "
                       "(auto-tuner mech-ramp stage)",
        "sample_rate_hz": 200.0,
        "segments": [
            {"label": "idle",     "throttle": 0.00, "duration_s": 2.0},
            {"label": "spool",    "throttle": 0.20, "duration_s": 2.0,
             "ramp": True},
            {"label": "hold_lo",  "throttle": 0.20, "duration_s": 1.5},
            {"label": "step_up",  "throttle": 0.55, "duration_s": 2.5},
            {"label": "drop",     "throttle": 0.20, "duration_s": 2.0},
            {"label": "step_up2", "throttle": 0.55, "duration_s": 2.5},
            {"label": "rampdn",   "throttle": 0.00, "duration_s": 1.5,
             "ramp": True},
        ],
        "safety": safety,
    })


def mech_ramp_stats(rows: list[dict]) -> Optional[dict]:
    """Powertrain step-response estimates from a ``tune_ramp_measure`` run.

    Per step: first-order time constant ``tau_ms`` (63.2% eRPM crossing),
    mechanical slew (step height / tau) and the transient-current slope
    ``k_a_per_pct`` (peak current overshoot per % of commanded duty step -
    an upper bound on A per % of duty-lead, since the lead at the current
    peak is at most the full step). Median across steps; None if no usable
    step was found.
    """
    def fget(r, key):
        v = r.get(key)
        try:
            return float(v) if v not in (None, "") else None
        except (TypeError, ValueError):
            return None

    segs: dict[str, list[dict]] = {}
    for r in rows:
        segs.setdefault(str(r.get("segment")), []).append(r)

    ests = []
    for lbl, prev_lbl in (("step_up", "hold_lo"), ("step_up2", "drop")):
        step, prev = segs.get(lbl), segs.get(prev_lbl)
        if not step or not prev:
            continue
        tail = prev[-max(1, len(prev) // 5):]
        base_rpm = statistics.median(
            [v for r in tail if (v := fget(r, "perf_e_rpm")) is not None]
            or [float("nan")])
        base_i = statistics.median(
            [v for r in tail if (v := fget(r, "stand_current_a")) is not None]
            or [0.0])
        plat = step[-max(1, len(step) // 3):]
        hi_rpm = statistics.median(
            [v for r in plat if (v := fget(r, "perf_e_rpm")) is not None]
            or [float("nan")])
        hi_i = statistics.median(
            [v for r in plat if (v := fget(r, "stand_current_a")) is not None]
            or [0.0])
        if not (base_rpm == base_rpm and hi_rpm == hi_rpm):   # NaN guard
            continue
        rise = hi_rpm - base_rpm
        if rise < 1000.0:            # no real step -> nothing to measure
            continue
        t0 = fget(step[0], "t")
        thr0 = fget(prev[-1], "throttle_cmd") or 0.0
        thr1 = fget(step[-1], "throttle_cmd") or 0.0
        step_pct = abs(thr1 - thr0) * 100.0
        if t0 is None or step_pct < 5.0:
            continue
        target = base_rpm + 0.632 * rise
        t63 = next((t for r in step
                    if (v := fget(r, "perf_e_rpm")) is not None
                    and v >= target and (t := fget(r, "t")) is not None), None)
        if t63 is None:
            continue
        tau_ms = max((t63 - t0) * 1000.0, 5.0)   # floor at sampling grain
        i_pk = max([v for r in step
                    if (t := fget(r, "t")) is not None and t <= t0 + 0.5
                    and (v := fget(r, "stand_current_a")) is not None]
                   or [base_i])
        ests.append({
            "tau_ms": tau_ms,
            "slew_erpm_per_s": rise / (tau_ms / 1000.0),
            "k_a_per_pct": max(i_pk - base_i, 0.1) / step_pct,
            "i_peak_a": i_pk,
            "i_hi_a": hi_i,
            "rpm_lo": base_rpm,
            "rpm_hi": hi_rpm,
        })
    if not ests:
        return None
    return {k: statistics.median([e[k] for e in ests]) for k in ests[0]}


def compute_max_ramp(stats: dict, *, current_budget_a: float,
                     lo: int, hi: int, margin: float = 0.8) -> int:
    """max_ramp (0.1 %/ms units) from step-response physics.

    During a duty ramp at rate r the duty leads the mechanical state by
    ~r*tau once quasi-steady, and the transient current is ~k * lead. The
    fastest ramp whose worst-case transient stays inside the budget is
    r = budget / (k * tau); margin keeps a fraction of it.
    """
    lead_pct = current_budget_a / max(stats["k_a_per_pct"], 1e-3)
    rate_pct_per_ms = lead_pct / max(stats["tau_ms"], 1.0)
    setting = int(round(rate_pct_per_ms * 10.0 * margin))
    return max(lo, min(hi, setting))


def startup_stats(result: RunResult, profile: Profile,
                  min_rpm: float = 1000.0) -> dict:
    """Count failed starts in a :func:`startup_profile` run.

    A cycle failed if by the END of its spin segment neither the stand RPM
    nor the perf eRPM/pole_pairs exceeded ``min_rpm``.

    TODO: replace with PR #22's startup_reliability metric when merged.
    """
    rows = result.rows
    seg = np.array([r.get("segment") for r in rows], dtype=object)
    stand_rpm = metricsmod._col(rows, "stand_rpm")
    e_rpm = metricsmod._col(rows, "perf_e_rpm")
    pp = int(result.meta.get("pole_pairs") or profile.pole_pairs)
    cycles = 0
    failed: list[str] = []
    for s in profile.segments:
        if not s.label.startswith("spin"):
            continue
        cycles += 1
        idx = np.where(seg == s.label)[0]
        if idx.size == 0:
            failed.append(s.label)
            continue
        tail = idx[-max(1, idx.size // 4):]     # end of the spin segment
        vals = np.concatenate([stand_rpm[tail],
                               e_rpm[tail] / pp])   # column is true eRPM
        vals = vals[~np.isnan(vals)]
        if not (vals.size and float(vals.max()) >= min_rpm):
            failed.append(s.label)
    return {"cycles": cycles, "failed": len(failed),
            "failed_segments": failed, "min_rpm": min_rpm}


# ==========================================================================
# Objective + constraints
# ==========================================================================
def objective_score(m: dict, objective: ObjectiveSpec) -> float | None:
    """Weighted mean of eff_gf_per_w over steady points at meaningful power.

    Points below ``min_power_w`` are excluded entirely - on the bench they
    are the ratio of two small noisy numbers (a 2.3 W point swung -73 %
    run-to-run on unchanged firmware, see hwci/baseline.py). Labels missing
    from ``weights`` get weight 1.0. None if no point qualifies.
    """
    num = den = 0.0
    for p in m.get("steady_points", []):
        eff, pw = p.get("eff_gf_per_w"), p.get("elec_power_w")
        if eff is None or pw is None or eff != eff or pw != pw:
            continue
        if pw < objective.min_power_w:
            continue
        w = float(objective.weights.get(p["segment"], 1.0))
        num += w * eff
        den += w
    return num / den if den > 0 else None


def check_constraints(m: dict, meta: dict, cons: ConstraintsSpec, *,
                      jitter_reference: float | None,
                      settings_verified: bool,
                      startup: dict | None = None,
                      min_start_rpm: float | None = None) -> list[str]:
    """Hard-constraint check -> list of failure strings (empty = pass).

    A failing trial is DISQUALIFIED, never scored: a desyncing run can post a
    great g/W (undriven windmilling draws no power), so scoring it at all
    would reward exactly the behaviour the tuner must avoid.
    """
    fails: list[str] = []
    if meta.get("aborted"):
        fails.append(f"run aborted: {meta['aborted']}")
    if not settings_verified:
        fails.append("settings readback mismatch (page on device != intended)")
    d = m.get("demag", {})
    if d.get("event_count", 0) > cons.max_demag_events:
        fails.append(f"demag events {d.get('event_count')} > "
                     f"{cons.max_demag_events}")
    if d.get("bemf_timeout_samples", 0) > cons.max_bemf_timeout_samples:
        fails.append(f"bemf timeout samples {d.get('bemf_timeout_samples')} > "
                     f"{cons.max_bemf_timeout_samples}")
    s = m.get("summary", {})
    for key, bound in (("max_fet_temp_c", cons.max_fet_temp_c),
                       ("max_motor_temp_c", cons.max_motor_temp_c)):
        v = s.get(key)
        if bound is not None and v is not None and v == v and v > bound:
            fails.append(f"{key} {v:.1f} > {bound:.1f}")
    j = s.get("worst_zc_jitter_pct")
    if (jitter_reference is not None and j is not None
            and j > jitter_reference
            * (1.0 + cons.jitter_max_regression_pct / 100.0)):
        fails.append(
            f"zc jitter {j:.2f}% regressed > {cons.jitter_max_regression_pct}% "
            f"vs default-settings anchor ({jitter_reference:.2f}%)")
    if startup is not None:
        if startup["failed"] > cons.startup.max_failed:
            fails.append(f"failed starts {startup['failed']}/"
                         f"{startup['cycles']} > {cons.startup.max_failed}")
    elif min_start_rpm is not None:
        pts = m.get("steady_points", [])
        rpms = [p.get("rpm") for p in pts
                if p.get("rpm") is not None and p.get("rpm") == p.get("rpm")]
        if pts and (not rpms or max(rpms) < min_start_rpm):
            fails.append("failed start: no steady point reached "
                         f"{min_start_rpm:.0f} rpm")
    return fails


# ==========================================================================
# Backends: how a trial reaches an ESC (simulated or real)
# ==========================================================================
class TuneBackend(abc.ABC):
    """Writes settings pages and runs profiles against one ESC + rig."""

    eeprom_address: int
    mode: str                    # "sim" | "hw"

    @abc.abstractmethod
    def read_page(self) -> bytes:
        """Current 192-byte settings page on the device."""

    @abc.abstractmethod
    def program(self, blob: bytes, bin_path: Path) -> None:
        """Write ``blob`` as the settings page and reset into it."""

    @abc.abstractmethod
    def run_trial(self, blob: bytes, profile: Profile, bin_path: Path,
                  meta: dict, *, battery_cells: int | None,
                  min_cell_voltage: float) -> tuple[RunResult, dict]:
        """Program ``blob``, run ``profile``, return (result, extra) where
        extra carries ``settings_verified`` and ``resting_v``. Raises
        :class:`BatteryTooLowError` BEFORE arming if the pack is too low."""

    def wait_for_cool(self, max_temp_c: float, timeout_s: float) -> None:
        """Optionally block until the FET temp is below ``max_temp_c``."""

    def close(self) -> None:
        pass


class _SimEepromDevice(MockDebugger):
    """MockDebugger whose flash() actually lands: the blob becomes the
    readable page AND is decoded into the shared RigSimulator - the sim's
    equivalent of 'program page, reset, firmware loads settings at boot'."""

    def __init__(self, sim, eeprom_address: int):
        super().__init__(base=eeprom_address, size=EEPROM_SIZE)
        self._sim = sim

    def flash(self, bin_path: str, load_addr: int) -> None:
        from .sim import SimSettings
        super().flash(bin_path, load_addr)
        blob = Path(bin_path).read_bytes()
        self.poke(load_addr, blob)
        self._sim.set_settings(SimSettings.from_blob(blob))


class SimTuneBackend(TuneBackend):
    """One persistent RigSimulator across all trials (pack state, temperature
    and RNG carry over, exactly like a physical rig would)."""

    mode = "sim"

    def __init__(self, rig: RigConfig | None = None, *,
                 motor_params=None, seed: int = 1234, noise: float = 0.01,
                 base_blob: bytes | None = None):
        from .sim import MotorParams, RigSimulator
        self.rig = rig or RigConfig()
        params = motor_params or MotorParams(
            pole_pairs=self.rig.pole_pairs, demag_prone=True,
            # default sim tunes shouldn't randomly fail startup at the stock
            # startup_power of 100; tests inject a harsher fail_ref
            startup_fail_ref=100.0)
        self.sim = RigSimulator(params=params, seed=seed, noise=noise)
        self.eeprom_address = DEFAULT_EEPROM_ADDRESS
        self.dbg = _SimEepromDevice(self.sim, self.eeprom_address)
        self.dbg.poke(self.eeprom_address, base_blob or default_blob())
        # test hook: report a different resting pack voltage (e.g. fake a
        # drained pack to exercise the swap path)
        self.voltage_fn: Callable[[], float] | None = None

    def read_page(self) -> bytes:
        return self.dbg.read_memory(self.eeprom_address, EEPROM_SIZE)

    def program(self, blob: bytes, bin_path: Path) -> None:
        bin_path.parent.mkdir(parents=True, exist_ok=True)
        bin_path.write_bytes(blob)
        self.dbg.flash(str(bin_path), self.eeprom_address)

    def _resting_voltage(self) -> float:
        return self.voltage_fn() if self.voltage_fn else self.sim.voltage

    def run_trial(self, blob: bytes, profile: Profile, bin_path: Path,
                  meta: dict, *, battery_cells: int | None,
                  min_cell_voltage: float) -> tuple[RunResult, dict]:
        resting_v = self._resting_voltage()
        # Gate only on an injected voltage_fn (pack-drain tests): the sim's
        # nominal battery doesn't represent a real cell count, same reason
        # `hwci run`/`ci` ignore --battery-cells under --sim. A hardware
        # spec's battery_cells must not stop a sim dry-run of the same spec.
        if battery_cells and self.voltage_fn is not None:
            check_battery(resting_v, battery_cells, min_cell_voltage)
        self.program(blob, bin_path)
        verified = self.read_page() == blob
        sources = build_sim_sources(self.rig, profile, sim=self.sim)
        try:
            result = run_profile(profile, sources, realtime=False, meta=meta)
        finally:
            sources.close()
        return result, {"settings_verified": verified,
                        "resting_v": round(resting_v, 3)}

    def wait_for_cool(self, max_temp_c: float, timeout_s: float) -> None:
        waited = 0.0
        while self.sim.temp_c > max_temp_c and waited < timeout_s:
            self.sim.step(1.0, 0.0)   # cool at zero throttle, sim time
            waited += 1.0


class HwTuneBackend(TuneBackend):
    """Real rig: one-shot OpenOCD flash of the settings page, then the
    standard live sources (mirrors cmd_ci's flash-then-open flow)."""

    mode = "hw"

    def __init__(self, rig: RigConfig, *, tare: bool = True):
        from .debugger.openocd import OpenOcdDebugger
        self.rig = rig
        self.tare = tare
        elf = rig.resolved_elf()
        if elf is None:
            raise FileNotFoundError(
                f"no ELF for target {rig.target} in {rig.resolved_obj_dir()}; "
                "build + flash firmware (HWCI_PERF=1) before tuning")
        self.elf_path = str(elf)
        # Layout drift between this module's EEPROM_FIELDS and the flashed
        # firmware is caught HERE, before any page is written.
        check_eeprom_layout(self.elf_path)
        self._make_dbg = lambda: OpenOcdDebugger(
            rig.openocd_configs, openocd_bin=rig.openocd_bin,
            search_dirs=rig.openocd_search_dirs)
        dbg = self._make_dbg().open()
        try:
            self.eeprom_address = resolve_eeprom_address(dbg, self.elf_path)
        finally:
            dbg.close()

    def read_page(self) -> bytes:
        dbg = self._make_dbg().open()
        try:
            return dbg.read_memory(self.eeprom_address, EEPROM_SIZE)
        finally:
            dbg.close()

    def program(self, blob: bytes, bin_path: Path) -> None:
        bin_path.parent.mkdir(parents=True, exist_ok=True)
        bin_path.write_bytes(blob)
        # One-shot program of the 1KB settings sector + verify + reset; the
        # firmware reloads settings on the way back up.
        self._make_dbg().flash(str(bin_path), self.eeprom_address)

    def run_trial(self, blob: bytes, profile: Profile, bin_path: Path,
                  meta: dict, *, battery_cells: int | None,
                  min_cell_voltage: float) -> tuple[RunResult, dict]:
        from .runner import _live_voltage
        self.program(blob, bin_path)
        sources = build_live_sources(
            self.rig, profile, battery_cells=battery_cells,
            min_cell_voltage=min_cell_voltage, tare=self.tare)
        try:
            verified = False
            if sources.perf_reader is not None:
                readback = sources.perf_reader.dbg.read_memory(
                    self.eeprom_address, EEPROM_SIZE)
                verified = readback == blob
            resting_v = _live_voltage(sources.stand, sources.perf_source)
            result = run_profile(profile, sources, realtime=True, meta=meta)
        finally:
            sources.close()
        return result, {
            "settings_verified": verified,
            "resting_v": round(resting_v, 3) if resting_v is not None else None}

    def wait_for_cool(self, max_temp_c: float, timeout_s: float) -> None:
        # Best-effort: poll the ESC's own temperature over a short live
        # session until it cools (the stand's FET probe needs open sources,
        # which would hold the throttle line - keep it simple).
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            page_dbg = self._make_dbg().open()
            try:
                from .perf_reader import PerfReader
                reader = PerfReader(page_dbg, self.elf_path,
                                    check_layout=False)
                temp = reader.read().raw.get("temperature_c")
            except Exception:
                temp = None
            finally:
                page_dbg.close()
            if temp is None or temp <= max_temp_c:
                return
            time.sleep(10.0)


# ==========================================================================
# Session state (manifest + trial ledger, atomic checkpointing, resume)
# ==========================================================================
def _atomic_write_json(path: Path, data: dict) -> None:
    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps(data, indent=2, sort_keys=True))
    os.replace(tmp, path)


def _git_sha(repo_root: str) -> str | None:
    try:
        out = subprocess.run(["git", "rev-parse", "--short", "HEAD"],
                             cwd=repo_root, capture_output=True, text=True)
        return out.stdout.strip() or None
    except Exception:
        return None


def _slug(overrides: dict, kind: str) -> str:
    if kind != "trial":
        base = kind.replace("_", "-")
    elif overrides:
        base = "-".join(f"{k}_{v}" for k, v in sorted(overrides.items()))
    else:
        base = "incumbent"
    return base[:60]


@dataclass
class TrialPlan:
    stage: str
    kind: str                    # trial|anchor|baseline|final_winner|...
    overrides: dict[str, int]    # relative to the BASE page
    profile: Profile
    repeat: int = 0

    def signature(self) -> dict:
        return {"stage": self.stage, "kind": self.kind,
                "overrides": self.overrides, "repeat": self.repeat,
                "profile": self.profile.name}


class Tuner:
    """Runs (or resumes) one tune session against a backend."""

    def __init__(self, spec: TuneSpec, backend: TuneBackend,
                 out_dir: str | Path, *,
                 spec_text: str | None = None,
                 battery_cells: int | None = None,
                 no_prompt: bool = False,
                 resume: bool = False,
                 config_path: str | None = None,
                 prompt_fn: Callable[[str], str] = input,
                 before_trial: Callable[[int, TrialPlan], None] | None = None,
                 log: Callable[[str], None] = print):
        self.spec = spec
        self.backend = backend
        self.out = Path(out_dir)
        self.no_prompt = no_prompt
        self.prompt_fn = prompt_fn
        self.before_trial = before_trial
        self.log = log
        self.battery_cells = (battery_cells if battery_cells is not None
                              else spec.battery_cells)
        self._offsets = spec.param_offsets()
        self._cursor = 0            # next ledger entry to try to reuse
        self.executed = 0           # trials actually run this session

        if resume:
            self.manifest = json.loads((self.out / "manifest.json").read_text())
            if self.manifest.get("format_version") != MANIFEST_VERSION:
                raise TunePaused(
                    f"manifest format {self.manifest.get('format_version')} "
                    f"!= {MANIFEST_VERSION}; cannot resume")
            self.base = Settings(bytes.fromhex(self.manifest["base_blob_hex"]))
            # A crash may have left ANY trial's settings flashed; put the
            # device back on the current incumbent before continuing.
            blob = self.base.apply(self.manifest.get("incumbent") or {},
                                   self._offsets)
            self.backend.program(blob.to_bytes(),
                                 self.out / "resume_settings.bin")
        else:
            self.out.mkdir(parents=True, exist_ok=True)
            if spec_text is not None:
                (self.out / "spec.yaml").write_text(spec_text)
            self.base = Settings(self.backend.read_page())
            self.base.to_bin(self.out / "base_settings.bin")
            self.manifest = {
                "format_version": MANIFEST_VERSION,
                "spec_name": spec.name,
                "spec_sha256": hashlib.sha256(
                    (spec_text or "").encode()).hexdigest(),
                "git_sha": _git_sha(getattr(backend, "rig", RigConfig()).repo_root),
                "mode": backend.mode,
                "config_path": config_path,
                "battery_cells": self.battery_cells,
                "eeprom_address": backend.eeprom_address,
                "base_blob_hex": self.base.hex(),
                "jitter_reference": None,
                "incumbent": {},
                "stages": {},
                "trials": [],
                "pack_events": [],
                "result": None,
            }
            self._save()

    # -- persistence ----------------------------------------------------
    def _save(self) -> None:
        _atomic_write_json(self.out / "manifest.json", self.manifest)

    @property
    def ledger(self) -> list[dict]:
        return self.manifest["trials"]

    @property
    def incumbent(self) -> dict[str, int]:
        return dict(self.manifest["incumbent"])

    # -- battery / thermal gates -----------------------------------------
    def _min_cell_voltage(self) -> float:
        return max(DEFAULT_MIN_CELL_VOLTAGE, self.spec.pack.min_resting_cell_v)

    def _handle_low_battery(self, err: BatteryTooLowError, index: int) -> None:
        self._save()   # checkpoint before anything else
        if self.no_prompt or not self.spec.pack.prompt_on_swap:
            raise TunePaused(
                f"pack too low before trial T{index:03d} ({err}); manifest "
                f"checkpointed - swap the pack and resume with "
                f"'hwci tune --resume {self.out}'")
        self.prompt_fn(
            f"\nPack too low before trial T{index:03d}: {err}\n"
            "Swap/charge the pack, then press Enter to continue... ")
        self.manifest["pack_events"].append({
            "trial_index": index, "event": "pack_swap",
            "reason": str(err), "time": time.time()})
        self._save()

    def _swap_count(self) -> int:
        return len(self.manifest["pack_events"])

    # -- one trial (with replay/reuse) -----------------------------------
    def _trial(self, plan: TrialPlan) -> dict:
        # Replay: skip discarded entries, reuse a completed matching entry.
        while (self._cursor < len(self.ledger)
                and self.ledger[self._cursor].get("discarded")):
            self._cursor += 1
        if self._cursor < len(self.ledger):
            entry = self.ledger[self._cursor]
            sig_match = all(entry.get(k) == v
                            for k, v in plan.signature().items())
            trial_dir = self.out / entry.get("dir", "")
            complete = ((trial_dir / "trial.json").exists()
                        and (trial_dir / "metrics.json").exists())
            if sig_match and complete:
                self._cursor += 1
                return entry
            # Diverged or incomplete: quarantine this and everything after.
            self._quarantine_from(self._cursor)
        return self._execute(plan)

    def _quarantine_from(self, idx: int) -> None:
        for entry in self.ledger[idx:]:
            d = self.out / entry.get("dir", "")
            if d.exists():
                self._quarantine_dir(d)
        del self.ledger[idx:]
        self._save()

    @staticmethod
    def _quarantine_dir(d: Path) -> None:
        target = d.with_name(d.name + ".incomplete")
        n = 1
        while target.exists():
            target = d.with_name(f"{d.name}.incomplete{n}")
            n += 1
        d.rename(target)

    def _execute(self, plan: TrialPlan) -> dict:
        index = len(self.ledger)
        if self.before_trial is not None:
            self.before_trial(index, plan)
        thermal = self.spec.thermal
        if thermal.max_start_fet_temp_c is not None:
            self.backend.wait_for_cool(thermal.max_start_fet_temp_c,
                                       thermal.cool_timeout_s)
        blob = self.base.apply(plan.overrides, self._offsets)
        rel_dir = f"trials/T{index:03d}-{_slug(plan.overrides, plan.kind)}"
        trial_dir = self.out / rel_dir
        # Quarantine leftovers from a crash mid-trial: any dir claiming this
        # trial index (whatever its slug) that never completed.
        for stale in sorted(self.out.glob(f"trials/T{index:03d}-*")):
            if stale.is_dir() and ".incomplete" not in stale.name:
                self._quarantine_dir(stale)

        rig = getattr(self.backend, "rig", RigConfig())
        meta = {
            "target": rig.target, "mode": self.backend.mode,
            "profile": plan.profile.name,
            "profile_def": profile_to_dict(plan.profile),
            "pole_pairs": rig.pole_pairs,
            "motor": rig.motor_name, "prop": rig.prop,
            "tune_stage": plan.stage, "tune_kind": plan.kind,
            "tune_overrides": plan.overrides,
            "settings_blob_sha256": blob.sha256(),
        }
        while True:
            try:
                result, extra = self.backend.run_trial(
                    blob.to_bytes(), plan.profile,
                    trial_dir / "settings.bin", meta,
                    battery_cells=self.battery_cells,
                    min_cell_voltage=self._min_cell_voltage())
                break
            except BatteryTooLowError as e:
                self._handle_low_battery(e, index)

        result.meta.update(extra)
        result.save(trial_dir)
        m = metricsmod.compute(result, plan.profile)
        (trial_dir / "metrics.json").write_text(json.dumps(m, indent=2))

        is_startup = plan.profile.name == "tune_startup"
        st = (startup_stats(result, plan.profile,
                            self.spec.constraints.startup.min_rpm)
              if is_startup else None)
        fails = check_constraints(
            m, result.meta, self.spec.constraints,
            jitter_reference=self.manifest["jitter_reference"],
            settings_verified=bool(extra.get("settings_verified")),
            startup=st,
            min_start_rpm=(None if is_startup
                           else self.spec.constraints.startup.min_rpm))
        score = objective_score(m, self.spec.objective) if not fails else None
        summary = m.get("summary", {})

        entry = {
            **plan.signature(),
            "index": index,
            "dir": rel_dir,
            "blob_sha256": blob.sha256(),
            "settings_verified": bool(extra.get("settings_verified")),
            "resting_v": extra.get("resting_v"),
            "score_raw": None if score is None else round(score, 5),
            "score_norm": None,
            "disqualified": fails or None,
            "startup": st,
            "jitter_pct": summary.get("worst_zc_jitter_pct"),
            "fet_temp_c": _none_if_nan(summary.get("max_fet_temp_c")),
            "discarded": False,
        }
        (trial_dir / "trial.json").write_text(
            json.dumps(entry, indent=2, sort_keys=True))
        self.ledger.append(entry)
        self._cursor = len(self.ledger)
        self.executed += 1
        self._save()
        if fails:
            verdict = f"DISQUALIFIED ({'; '.join(fails)})"
        elif score is not None:
            verdict = f"score {score:.3f} g/W"
        else:
            verdict = "ok (no scored points)"
        self.log(f"  T{index:03d} {plan.stage}/{plan.kind} "
                 f"{plan.overrides or '{}'} -> {verdict}")
        return entry

    # -- anchor-normalized scoring ---------------------------------------
    @staticmethod
    def _normalize(entries: list[dict], anchors: list[tuple[int, float]],
                   positions: dict[int, int]) -> None:
        """Set score_norm on each entry: raw scaled by the first anchor over
        the linear interpolation between surrounding anchors (cancels pack
        drift). No/one usable anchor -> raw is kept as-is."""
        usable = [(p, s) for p, s in anchors if s is not None and s > 0]
        for e in entries:
            raw = e.get("score_raw")
            if raw is None:
                continue
            pos = positions[e["index"]]
            e["score_norm"] = round(raw * Tuner._drift_factor(usable, pos), 5)

    @staticmethod
    def _drift_factor(anchors: list[tuple[int, float]], pos: int) -> float:
        if not anchors:
            return 1.0
        ref = anchors[0][1]
        if pos <= anchors[0][0]:
            interp = anchors[0][1]
        elif pos >= anchors[-1][0]:
            interp = anchors[-1][1]
        else:
            interp = anchors[0][1]
            for (p0, s0), (p1, s1) in zip(anchors, anchors[1:]):
                if p0 <= pos <= p1:
                    frac = (pos - p0) / max(1, (p1 - p0))
                    interp = s0 + (s1 - s0) * frac
                    break
        return ref / interp if interp > 0 else 1.0

    # -- candidate scoring / winner picking --------------------------------
    def _distance_to_default(self, overrides: dict[str, int]) -> float:
        dist = 0.0
        for name, v in overrides.items():
            f = resolve_field(name, self._offsets.get(name))
            span = max(1, f.hi - f.lo)
            dist += abs(int(v) - self.base.get(name, self._offsets.get(name))) \
                / span
        return dist

    def _pick_winner(self, cands: list[dict]) -> dict | None:
        """cands: [{overrides, entries, order}] -> winner cand or None.

        Disqualified candidates never win. Within ``noise_floor_pct`` of the
        best score the tie breaks toward lower jitter, then lower FET temp,
        then the settings closest to default.
        """
        scored = []
        for c in cands:
            if any(e.get("disqualified") for e in c["entries"]):
                c["score"] = None
                continue
            vals = [e.get("score_norm") if e.get("score_norm") is not None
                    else e.get("score_raw") for e in c["entries"]]
            vals = [v for v in vals if v is not None]
            if not vals:
                c["score"] = None
                continue
            c["score"] = statistics.median(vals)
            c["jitter"] = _median_of(c["entries"], "jitter_pct")
            c["fet"] = _median_of(c["entries"], "fet_temp_c")
            scored.append(c)
        if not scored:
            return None
        best = max(c["score"] for c in scored)
        floor = best * (1.0 - self.spec.objective.noise_floor_pct / 100.0)
        tied = [c for c in scored if c["score"] >= floor]
        tied.sort(key=lambda c: (
            c["jitter"] if c.get("jitter") is not None else math.inf,
            c["fet"] if c.get("fet") is not None else math.inf,
            self._distance_to_default(c["overrides"]),
            c["order"]))
        return tied[0]

    # -- stages ------------------------------------------------------------
    def _merged(self, stage: StageSpec, overrides: dict) -> dict:
        return {**self.incumbent, **stage.fixed, **overrides}

    def _stage_profile(self, stage: StageSpec) -> Profile:
        return self._profile_by_name(stage.profile)

    def _profile_by_name(self, name: str | None) -> Profile:
        if name in (None, "tune_probe", "probe"):
            return probe_profile(self.spec)
        if name == "tune_startup":
            return startup_profile(self.spec)
        if name == "tune_step":
            return step_profile(self.spec)
        return load_profile(name)

    def _run_scored_stage(self, stage: StageSpec) -> None:
        profile = self._stage_profile(stage)
        anchors: list[tuple[int, float]] = []
        positions: dict[int, int] = {}
        entries_all: list[dict] = []
        pos = 0
        since_anchor = [0]

        def run_one(kind: str, overrides: dict, repeat: int = 0) -> dict:
            nonlocal pos
            e = self._trial(TrialPlan(stage=stage.name, kind=kind,
                                      overrides=overrides, profile=profile,
                                      repeat=repeat))
            positions[e["index"]] = pos
            entries_all.append(e)
            pos += 1
            return e

        def anchor() -> None:
            e = run_one("anchor", self.incumbent)
            anchors.append((positions[e["index"]], e.get("score_raw")))
            since_anchor[0] = 0

        def candidate(overrides: dict, repeat: int = 0) -> dict:
            if since_anchor[0] >= self.spec.anchors_every:
                anchor()
            e = run_one("trial", overrides, repeat)
            since_anchor[0] += 1
            return e

        anchor()
        cands: list[dict] = []
        if stage.sweep:
            param = self.spec.parameters[stage.sweep]
            f = resolve_field(param.name, param.offset)
            values = list(dict.fromkeys(param.values))
            inc_val = self.base.apply(self.incumbent, self._offsets).get(
                param.name, param.offset)
            if inc_val not in values:
                values.append(inc_val)

            def test_value(v: int) -> dict:
                ent = [candidate(self._merged(stage, {param.name: v}), r)
                       for r in range(stage.repeats)]
                c = {"overrides": self._merged(stage, {param.name: v}),
                     "value": v, "entries": ent, "order": len(cands)}
                cands.append(c)
                return c

            if stage.search == "climb":
                self._climb(sorted(values), inc_val, test_value)
            else:
                for v in values:
                    test_value(v)
            if param.refine_step:
                # Refine around the raw ARGMAX (not the tie-break winner: the
                # tie set spans the whole noise floor and would wander).
                # Raw scores: anchors are not complete yet, but refinement
                # only needs a neighborhood, not a drift-corrected ranking.
                center = _argmax_value(cands)
                if center is not None:
                    for dv in (-param.refine_step, param.refine_step):
                        v = center + dv
                        if v in values or not f.lo <= v <= f.hi:
                            continue
                        values.append(v)
                        ent = [candidate(
                                   self._merged(stage, {param.name: v}), r)
                               for r in range(stage.repeats)]
                        cands.append({
                            "overrides": self._merged(stage, {param.name: v}),
                            "value": v, "entries": ent,
                            "order": len(cands)})
        else:
            candidates = stage.ab_candidates or []
            ents: list[list[dict]] = [[] for _ in candidates]
            for r in range(stage.repeats):      # interleaved: c0 c1 c2 | c0..
                for i, ov in enumerate(candidates):
                    ents[i].append(candidate(self._merged(stage, ov), r))
            cands = [{"overrides": self._merged(stage, ov), "raw_ov": ov,
                      "entries": ents[i], "order": i}
                     for i, ov in enumerate(candidates)]
        anchor()

        self._normalize(entries_all, anchors, positions)
        self._save()
        winner = self._pick_winner(cands)
        if winner is not None:
            self.manifest["incumbent"] = dict(winner["overrides"])
        self.manifest["stages"][stage.name] = {
            "winner": None if winner is None else winner["overrides"],
            "winner_score": None if winner is None else round(winner["score"], 5),
            "trials": [e["index"] for e in entries_all],
        }
        self._save()
        self.log(f"stage {stage.name}: winner "
                 f"{None if winner is None else winner['overrides']}")

    @staticmethod
    def _climb(ordered: list[int], start_val: int,
               test_value: Callable[[int], dict]) -> None:
        """Hill-climb a sorted value list from the value nearest ``start_val``.

        Valid for unimodal responses (advance, pwm frequency): walk in the
        first improving direction and stop at the first non-improvement -
        for a wide grid this typically tests 3-4 values instead of all of
        them. Raw single-value scores are compared (same basis the grid
        sweep's refine step uses); a disqualified value reads as worse than
        any qualified one, so the climb walks away from constraint
        violations instead of stopping on them.
        """
        def raw(c: dict) -> Optional[float]:
            if any(e.get("disqualified") for e in c["entries"]):
                return None
            vals = [e["score_raw"] for e in c["entries"]
                    if e.get("score_raw") is not None]
            return statistics.median(vals) if vals else None

        def better(a: dict, b: dict) -> bool:
            ra, rb = raw(a), raw(b)
            return rb is not None and (ra is None or rb > ra)

        i = min(range(len(ordered)), key=lambda k: abs(ordered[k] - start_val))
        cur = test_value(ordered[i])
        moved = False
        for direction in (1, -1):
            j = i + direction
            while 0 <= j < len(ordered):
                nxt = test_value(ordered[j])
                if not better(cur, nxt):
                    break
                cur, moved = nxt, True
                j += direction
            if moved:
                break   # went uphill one way; the other side is downhill

    def _trial_rows(self, entry: dict) -> list[dict]:
        import csv
        path = self.out / entry.get("dir", "") / "samples.csv"
        with open(path, newline="") as f:
            return list(csv.DictReader(f))

    def _run_measure_stage(self, stage: StageSpec) -> None:
        """Physics-based max_ramp: measure the powertrain's step response
        once (firmware slew unrestricted), compute the fastest ramp whose
        worst-case transient stays inside the current budget, then VERIFY
        it on the step-stress profile (backing off 0.7x on failure)."""
        f = resolve_field("max_ramp", None)
        e = self._trial(TrialPlan(
            stage=stage.name, kind="measure",
            overrides=self._merged(stage, {"max_ramp": f.hi}),
            profile=ramp_measure_profile(self.spec)))
        indices = [e["index"]]
        stats = None
        if e.get("disqualified"):
            self.log(f"stage {stage.name}: measurement run disqualified "
                     f"({e['disqualified']}); keeping incumbent max_ramp")
        else:
            stats = mech_ramp_stats(self._trial_rows(e))
            if stats is None:
                self.log(f"stage {stage.name}: no usable step response in "
                         "the measurement run; keeping incumbent max_ramp")

        winner_val = None
        computed = None
        if stats is not None:
            limit = ramp_measure_profile(self.spec).safety.max_current_a
            budget = max(0.75 * (limit or 55.0) - stats["i_hi_a"], 5.0)
            computed = compute_max_ramp(stats, current_budget_a=budget,
                                        lo=f.lo, hi=f.hi,
                                        margin=stage.margin)
            self.log(
                f"stage {stage.name}: tau {stats['tau_ms']:.0f} ms, slew "
                f"{stats['slew_erpm_per_s']:.0f} eRPM/s, "
                f"k {stats['k_a_per_pct']:.2f} A/%, budget {budget:.1f} A "
                f"-> max_ramp {computed}")
            v = computed
            while True:
                ev = self._trial(TrialPlan(
                    stage=stage.name, kind="verify",
                    overrides=self._merged(stage, {"max_ramp": v}),
                    profile=step_profile(self.spec)))
                indices.append(ev["index"])
                if not ev.get("disqualified"):
                    winner_val = v
                    break
                nxt = max(f.lo, int(v * 0.7))
                if nxt == v:
                    break               # already at the floor: no winner
                self.log(f"stage {stage.name}: verify failed at max_ramp "
                         f"{v}; backing off to {nxt}")
                v = nxt

        if winner_val is not None:
            self.manifest["incumbent"] = self._merged(
                stage, {"max_ramp": winner_val})
        self.manifest["stages"][stage.name] = {
            "winner": (None if winner_val is None
                       else {"max_ramp": winner_val}),
            "winner_score": None,
            "measured": (None if stats is None
                         else {k: round(v, 3) for k, v in stats.items()}),
            "computed_max_ramp": computed,
            "trials": indices,
        }
        self._save()
        self.log(f"stage {stage.name}: winner "
                 f"{None if winner_val is None else {'max_ramp': winner_val}}")

    def _run_constraint_stage(self, stage: StageSpec) -> None:
        """Constraint-only sweep (e.g. max_ramp on a step profile): values are
        tried in listed order (list them best-first); the first value with
        ZERO constraint failures wins and the stage stops there."""
        profile = self._stage_profile(stage)
        param = self.spec.parameters[stage.sweep]
        indices, winner_val = [], None
        for v in param.values:
            e = self._trial(TrialPlan(
                stage=stage.name, kind="trial",
                overrides=self._merged(stage, {param.name: v}),
                profile=profile))
            indices.append(e["index"])
            if not e.get("disqualified"):
                winner_val = v
                break
        if winner_val is not None:
            self.manifest["incumbent"] = self._merged(
                stage, {param.name: winner_val})
        self.manifest["stages"][stage.name] = {
            "winner": (None if winner_val is None
                       else {param.name: winner_val}),
            "winner_score": None,
            "trials": indices,
        }
        self._save()
        self.log(f"stage {stage.name}: winner "
                 f"{None if winner_val is None else {param.name: winner_val}}")

    # -- finals -------------------------------------------------------------
    def _run_finals(self) -> dict:
        finals = self.spec.finals
        profile = self._profile_by_name(finals.profile)
        winner_ov = self.incumbent
        deltas: list[float] = []
        winner_fails = 0
        indices: list[int] = []
        pattern = [("final_winner", winner_ov), ("final_default", {}),
                   ("final_default", {}), ("final_winner", winner_ov)]
        for rep in range(finals.repeats):
            while True:     # a pack swap mid-block restarts the whole block
                swaps_before = self._swap_count()
                block: list[dict] = []
                for kind, ov in pattern:
                    e = self._trial(TrialPlan(stage="finals", kind=kind,
                                              overrides=ov, profile=profile,
                                              repeat=rep))
                    block.append(e)
                    if self._swap_count() != swaps_before and len(block) > 1:
                        break   # block straddled a swap - restart it
                if self._swap_count() == swaps_before or len(block) == 1:
                    break
                for e in block:     # discard the straddling partial block
                    e["discarded"] = True
                self._save()
                self.log(f"finals block {rep}: pack swap mid-block - "
                         "restarting the block")
            indices += [e["index"] for e in block]
            w = [e for e in block if e["kind"] == "final_winner"]
            d = [e for e in block if e["kind"] == "final_default"]
            winner_fails += sum(1 for e in w if e.get("disqualified"))
            for we, de in zip(w, d):
                if (we.get("score_raw") is not None
                        and de.get("score_raw") is not None):
                    deltas.append(we["score_raw"] - de["score_raw"])

        startup_ok = True
        st = None
        if finals.startup_check:
            sp = startup_profile(self.spec)
            e = self._trial(TrialPlan(stage="finals", kind="final_startup",
                                      overrides=winner_ov, profile=sp))
            indices.append(e["index"])
            st = e.get("startup")
            startup_ok = not e.get("disqualified")

        median_delta = statistics.median(deltas) if deltas else None
        confirmed = (bool(winner_ov) and winner_fails == 0 and startup_ok
                     and median_delta is not None and median_delta > 0)
        result = {
            "winner_overrides": winner_ov,
            "median_paired_delta": (None if median_delta is None
                                    else round(median_delta, 5)),
            "paired_deltas": [round(x, 5) for x in deltas],
            "winner_constraint_failures": winner_fails,
            "startup": st,
            "confirmed": confirmed,
            "trials": indices,
        }
        self.manifest["result"] = result
        self._save()
        return result

    # -- top level -----------------------------------------------------------
    def run(self) -> dict:
        spec = self.spec
        self.log(f"tune session {spec.name!r} -> {self.out} "
                 f"({self.backend.mode}, eeprom @ "
                 f"0x{self.backend.eeprom_address:08x})")
        # The incumbent is REPLAYED, not loaded: the manifest holds its
        # latest value, but deterministic planning must see it evolve stage
        # by stage exactly as the original session did (anchors run "the
        # incumbent as of that trial", and their signatures must match).
        self.manifest["incumbent"] = {}
        # Session baseline: the DEFAULT (base-page) settings. Its jitter is
        # the reference every trial's jitter-regression constraint compares
        # against, and its score anchors the whole session.
        e = self._trial(TrialPlan(stage="baseline", kind="baseline",
                                  overrides={}, profile=probe_profile(spec)))
        if e.get("disqualified"):
            self.log(f"WARNING: baseline run disqualified: "
                     f"{e['disqualified']}; retrying once")
            e = self._trial(TrialPlan(stage="baseline", kind="baseline",
                                      overrides={},
                                      profile=probe_profile(spec), repeat=1))
        if e.get("disqualified"):
            # Everything downstream leans on this run: the jitter gate's
            # reference, drift normalization, and the finals' default legs.
            # Bench experience: a session run past a broken baseline burns
            # every trial and ends unconfirmable. Quarantine the baseline
            # entries so a resume re-runs them fresh after the fix.
            self._quarantine_from(0)
            raise TunePaused(
                "baseline (default settings) disqualified twice: "
                f"{e['disqualified']} - fix the rig or spec limits, then "
                f"resume with 'hwci tune --resume {self.out}'")
        if self.manifest["jitter_reference"] is None:
            self.manifest["jitter_reference"] = e.get("jitter_pct")
            if e.get("jitter_pct") is None:
                self.log("WARNING: baseline has no zc-jitter data; the "
                         "jitter regression gate is OFF for this session")
            self._save()

        for stage in spec.stages:
            if stage.measure:
                self._run_measure_stage(stage)
            elif stage.constraint_only:
                self._run_constraint_stage(stage)
            else:
                self._run_scored_stage(stage)

        result = self._run_finals()
        best = (self.base.apply(result["winner_overrides"], self._offsets)
                if result["confirmed"] else self.base)
        # Leave the DEVICE on the session's verdict too: the last finals
        # trial flashed the unconfirmed winner, which would silently stay
        # active while the report says "default kept".
        self.backend.program(best.to_bytes(), self.out / "best_settings.bin")
        (self.out / "settings_diff.md").write_text(self._diff_md(best))
        (self.out / "report.md").write_text(self._report_md(result))
        pdf = reportmod.write_tune_pdf(
            self.out, self.manifest, result, self.base.diff(best),
            log=self.log)
        if pdf is not None:
            self.log(f"PDF report: {pdf}")
        self.log(f"winner {result['winner_overrides'] or '{}'} "
                 f"{'CONFIRMED' if result['confirmed'] else 'NOT confirmed'} "
                 f"(median paired delta: {result['median_paired_delta']})")
        return result

    # -- reporting -----------------------------------------------------------
    def _diff_md(self, best: Settings) -> str:
        out = ["# Settings diff: default -> best\n",
               "| setting | default | best |", "|---|---|---|"]
        diff = self.base.diff(best)
        if not diff:
            out.append("| (no change - default settings won) | | |")
        for name, a, b in diff:
            out.append(f"| {name} | {a} | {b} |")
        out.append("")
        return "\n".join(out)

    def _report_md(self, result: dict) -> str:
        m = self.manifest
        out = [f"# AM32 auto-tune report: {m['spec_name']}\n"]
        out.append(f"- **mode**: {m['mode']}")
        out.append(f"- **git_sha**: {m['git_sha']}")
        out.append(f"- **eeprom_address**: 0x{m['eeprom_address']:08x}")
        out.append(f"- **battery_cells**: {m['battery_cells']}")
        out.append(f"- **jitter reference (default)**: "
                   f"{m['jitter_reference']} %")
        out.append(f"- **pack swaps**: {len(m['pack_events'])}")
        out.append("")
        out.append("## Verdict\n")
        out.append(f"- winner: `{result['winner_overrides'] or '{} (default)'}`")
        out.append(f"- confirmed: **{result['confirmed']}** "
                   f"(median paired delta {result['median_paired_delta']} g/W "
                   f"over {len(result['paired_deltas'])} ABBA pairs, "
                   f"{result['winner_constraint_failures']} winner "
                   "constraint failures)")
        if result.get("startup") is not None:
            st = result["startup"]
            out.append(f"- startup: {st['failed']}/{st['cycles']} failed")
        out.append("\n## Stages\n")
        out.append("| stage | winner | score (g/W, normalized) |")
        out.append("|---|---|---|")
        for name, s in m["stages"].items():
            out.append(f"| {name} | `{s['winner']}` | {s['winner_score']} |")
        out.append("\n## Trials\n")
        out.append("| # | stage | kind | overrides | raw g/W | norm g/W "
                   "| disqualified |")
        out.append("|---|---|---|---|---|---|---|")
        for e in m["trials"]:
            dq = "; ".join(e["disqualified"]) if e.get("disqualified") else ""
            if e.get("discarded"):
                dq = (dq + " " if dq else "") + "(discarded: pack swap)"
            out.append(f"| {e['index']} | {e['stage']} | {e['kind']} | "
                       f"`{e['overrides']}` | {e['score_raw']} | "
                       f"{e['score_norm']} | {dq} |")
        out.append("")
        return "\n".join(out)


def _argmax_value(cands: list[dict]) -> Optional[int]:
    """Value of the best-scoring qualified sweep candidate (median raw)."""
    best_v, best_s = None, None
    for c in cands:
        if any(e.get("disqualified") for e in c["entries"]):
            continue
        vals = [e["score_raw"] for e in c["entries"]
                if e.get("score_raw") is not None]
        if not vals:
            continue
        s = statistics.median(vals)
        if best_s is None or s > best_s:
            best_v, best_s = c["value"], s
    return best_v


def _none_if_nan(v):
    return None if (isinstance(v, float) and math.isnan(v)) else v


def _median_of(entries: list[dict], key: str) -> Optional[float]:
    vals = [e[key] for e in entries if e.get(key) is not None]
    return statistics.median(vals) if vals else None
