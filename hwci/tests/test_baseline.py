"""Tests for baseline save/compare regression gating."""
import copy

from hwci import baseline as bl
from hwci import metrics as metricsmod
from hwci.config import RigConfig, load_profile
from hwci.runner import build_sim_sources, run_profile


def _metrics(profile_name="ci_smoke"):
    rig = RigConfig()
    profile = load_profile(profile_name)
    sources = build_sim_sources(rig, profile)
    try:
        result = run_profile(profile, sources, realtime=False, meta={})
    finally:
        sources.close()
    return metricsmod.compute(result, profile)


def test_identical_metrics_pass():
    m = _metrics()
    base = {"metrics": m}
    result = bl.compare(m, base)
    assert result["passed"]


def test_efficiency_regression_fails():
    m = _metrics()
    base = {"metrics": copy.deepcopy(m)}
    # Current efficiency drops 10% -> should fail the 3% gate.
    m["summary"]["peak_efficiency_gf_per_w"] *= 0.90
    for p in m["steady_points"]:
        p["eff_gf_per_w"] *= 0.90
    result = bl.compare(m, base)
    assert not result["passed"]
    assert any(c["name"] == "peak_efficiency_gf_per_w" and not c["pass"]
               for c in result["checks"])


def test_loop_time_regression_fails():
    m = _metrics()
    base = {"metrics": copy.deepcopy(m)}
    # Well past both the +15% and the +45us absolute slack.
    m["summary"]["worst_ctrl_exec_us_steady"] = (
        base["metrics"]["summary"]["worst_ctrl_exec_us_steady"] + 100)
    result = bl.compare(m, base)
    assert not result["passed"]


def test_loop_time_gate_prefers_steady_key():
    m = _metrics()
    base = {"metrics": copy.deepcopy(m)}
    # A start-transient spike in the raw run-max must NOT fail the gate when
    # the steady-window value is unchanged (observed 705 vs 950us run-to-run).
    m["summary"]["worst_ctrl_exec_us"] = 950
    result = bl.compare(m, base)
    assert result["passed"]


def test_loop_time_gate_falls_back_without_steady_key():
    m = _metrics()
    base = {"metrics": copy.deepcopy(m)}
    del base["metrics"]["summary"]["worst_ctrl_exec_us_steady"]
    m["summary"]["worst_ctrl_exec_us"] = (
        base["metrics"]["summary"]["worst_ctrl_exec_us"] + 100)
    result = bl.compare(m, base)
    assert not result["passed"]


def test_loop_time_equality_passes():
    # A baseline must pass against itself even with large absolute values
    # (950us start transients failed the old <=45us absolute-cap semantics).
    m = _metrics()
    base = {"metrics": copy.deepcopy(m)}
    for k in ("worst_ctrl_exec_us", "worst_ctrl_exec_us_steady"):
        base["metrics"]["summary"][k] = 950
        m["summary"][k] = 950
    assert bl.compare(m, base)["passed"]


def test_negative_efficiency_equality_passes():
    # Naive percent math fails an identical negative value against itself.
    assert bl._worse_is_lower(-1.0, -1.0, 3.0)
    assert not bl._worse_is_lower(-1.0, -1.06, 3.0)  # real 6% worsening fails
    assert bl._worse_is_lower(9.0, 8.8, 3.0)         # positive within 3%


def test_noprop_efficiency_noise_not_gated():
    m = _metrics()
    base = {"metrics": copy.deepcopy(m)}
    # No-prop rig: baseline g/W is noise around zero; must not gate at all.
    base["metrics"]["summary"]["peak_efficiency_gf_per_w"] = -0.218
    m["summary"]["peak_efficiency_gf_per_w"] = 0.3  # different noise, still ok
    for p in base["metrics"]["steady_points"]:
        p["eff_gf_per_w"] = -0.1
    for p in m["steady_points"]:
        p["eff_gf_per_w"] = 0.2
    result = bl.compare(m, base)
    assert result["passed"]
    assert any("not gated" in c["note"] for c in result["checks"])


def test_new_demag_fails():
    m = _metrics()
    base = {"metrics": copy.deepcopy(m)}
    base["metrics"]["summary"]["demag_events"] = 0
    m["summary"]["demag_events"] = 2
    result = bl.compare(m, base)
    assert not result["passed"]


def test_save_and_load(tmp_path):
    m = _metrics()
    path = bl.save_baseline(m, tmp_path / "base.json", meta={"target": "X"})
    loaded = bl.load_baseline(path)
    assert loaded["format_version"] == bl.FORMAT_VERSION
    assert loaded["metrics"]["summary"]["max_thrust_gf"] == m["summary"]["max_thrust_gf"]
    assert loaded["meta"]["target"] == "X"


# --- fail-closed behaviour --------------------------------------------------

def test_missing_current_metric_fails():
    m = _metrics()
    base = {"metrics": copy.deepcopy(m)}
    m["summary"]["worst_ctrl_exec_us_steady"] = None  # dead perf channel
    result = bl.compare(m, base)
    assert not result["passed"]
    assert any(c["name"] == "worst_ctrl_exec_us_steady" and not c["pass"]
               for c in result["checks"])


def test_nan_current_metric_fails():
    m = _metrics()
    base = {"metrics": copy.deepcopy(m)}
    m["summary"]["max_cpu_load_pct"] = float("nan")
    result = bl.compare(m, base)
    assert not result["passed"]


def test_dead_channel_coverage_fails():
    m = _metrics()
    base = {"metrics": copy.deepcopy(m)}
    # Perf channel died mid-run: only a handful of samples made it.
    m["summary"]["perf_sample_count"] = 3
    result = bl.compare(m, base)
    assert any(c["name"] == "perf_coverage" and not c["pass"]
               for c in result["checks"])
    assert not result["passed"]


def test_identity_mismatch_fails():
    m = _metrics()
    base = {"meta": {"target": "OTHER_TARGET", "profile": "ci_smoke"},
            "metrics": copy.deepcopy(m)}
    result = bl.compare(m, base,
                        current_meta={"target": "ARK_4IN1_F051",
                                      "profile": "ci_smoke"})
    assert not result["passed"]
    assert any(c["name"] == "baseline_target" and not c["pass"]
               for c in result["checks"])


def test_missing_steady_segment_fails():
    m = _metrics()
    base = {"metrics": copy.deepcopy(m)}
    m["steady_points"] = m["steady_points"][:-1]  # a segment produced no data
    result = bl.compare(m, base)
    assert not result["passed"]
