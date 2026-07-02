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
    # 30% drop is well past the 15% gate (widened from an initial 3% guess
    # after a same-firmware repeatability run showed up to 8.7% swing at
    # high power on the physical bench - see Thresholds.efficiency_drop_pct).
    m["summary"]["peak_efficiency_gf_per_w"] *= 0.70
    for p in m["steady_points"]:
        p["eff_gf_per_w"] *= 0.70
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


def test_low_power_point_not_gated_even_with_prop():
    # A real prop can still produce a low-power point (e.g. 10% throttle)
    # where thrust/power is a ratio of two small noisy numbers - observed on
    # the bench: 2.3W swung -73% efficiency run-to-run on unchanged hardware.
    m = _metrics()
    base = {"metrics": copy.deepcopy(m)}
    base["metrics"]["steady_points"][0]["elec_power_w"] = 2.3
    base["metrics"]["steady_points"][0]["eff_gf_per_w"] = 7.72
    m["steady_points"][0]["elec_power_w"] = 2.3
    m["steady_points"][0]["eff_gf_per_w"] = 2.05  # -73%, would fail the % gate
    result = bl.compare(m, base)
    label = base["metrics"]["steady_points"][0]["segment"]
    eff_check = next(c for c in result["checks"] if c["name"] == f"eff@{label}")
    assert eff_check["pass"]
    assert "not gated" in eff_check["note"]


def test_peak_efficiency_excludes_low_power_points():
    # "Peak" is a max() over throttle points, which amplifies whichever point
    # is noisiest. If the literal peak sits at a low-power point, the GATE
    # must recompute peak from points above the power floor instead of
    # trusting the summary scalar - else it gates pure noise every run.
    m = _metrics()
    base = {"metrics": copy.deepcopy(m)}
    pts_b = base["metrics"]["steady_points"]
    pts_c = m["steady_points"]
    assert len(pts_b) >= 2
    # Low-power point has the highest g/W in both runs (as observed on the
    # bench) but swings wildly; a real, well-powered point stays stable.
    pts_b[0]["elec_power_w"], pts_b[0]["eff_gf_per_w"] = 2.3, 50.0
    pts_c[0]["elec_power_w"], pts_c[0]["eff_gf_per_w"] = 2.3, 5.0
    pts_b[1]["elec_power_w"], pts_b[1]["eff_gf_per_w"] = 100.0, 1.0
    pts_c[1]["elec_power_w"], pts_c[1]["eff_gf_per_w"] = 100.0, 1.0
    result = bl.compare(m, base)
    peak_check = next(c for c in result["checks"]
                      if c["name"] == "peak_efficiency_gf_per_w")
    assert peak_check["baseline"] == 1.0   # not 50.0 - the noisy point excluded
    assert peak_check["current"] == 1.0
    assert peak_check["pass"]


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
