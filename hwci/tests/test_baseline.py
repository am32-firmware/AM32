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
    m["summary"]["worst_ctrl_exec_us"] = 48  # over the 45us absolute cap
    result = bl.compare(m, base)
    assert not result["passed"]


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
    m["summary"]["worst_ctrl_exec_us"] = None  # dead perf channel
    result = bl.compare(m, base)
    assert not result["passed"]
    assert any(c["name"] == "worst_ctrl_exec_us" and not c["pass"]
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
