"""End-to-end simulated runs through the runner + metrics + persistence."""
from hwci import metrics as metricsmod
from hwci.config import RigConfig, load_profile
from hwci.model import RunResult
from hwci.runner import build_sim_sources, run_profile


def _run(profile_name, demag_prone=True):
    rig = RigConfig()
    profile = load_profile(profile_name)
    sources = build_sim_sources(rig, profile, demag_prone=demag_prone)
    try:
        return run_profile(profile, sources, realtime=False,
                           meta={"target": "ARK_4IN1_F051"}), profile
    finally:
        sources.close()


def test_ci_smoke_runs_and_has_steady_points():
    result, profile = _run("ci_smoke")
    assert result.meta["aborted"] is None
    assert len(result.rows) > 100
    m = metricsmod.compute(result, profile)
    assert len(m["steady_points"]) == 2          # hold25, hold45
    assert m["summary"]["max_thrust_gf"] > 0
    assert m["summary"]["peak_efficiency_gf_per_w"] > 0
    assert 0 <= m["summary"]["max_cpu_load_pct"] <= 100
    # Smooth ramps must not be flagged as demag.
    assert m["summary"]["demag_events"] == 0


def test_efficiency_sweep_curve_is_monotonic_thrust():
    result, profile = _run("efficiency_sweep")
    m = metricsmod.compute(result, profile)
    thrusts = [p["thrust_gf"] for p in m["steady_points"]]
    assert thrusts == sorted(thrusts)            # thrust rises with throttle
    assert len(m["steady_points"]) == 10


def test_demag_profile_detects_events():
    result, profile = _run("demag_step_stress", demag_prone=True)
    m = metricsmod.compute(result, profile)
    assert m["summary"]["demag_events"] >= 1
    assert m["demag"]["bemf_timeout_samples"] >= 1


def test_save_load_roundtrip(tmp_path):
    result, profile = _run("ci_smoke")
    result.save(tmp_path / "run")
    loaded = RunResult.load(tmp_path / "run")
    assert len(loaded.rows) == len(result.rows)
    assert loaded.meta["profile"] == "ci_smoke"
    # metrics identical after roundtrip
    m1 = metricsmod.compute(result, profile)
    m2 = metricsmod.compute(loaded, profile)
    assert m1["summary"]["max_thrust_gf"] == m2["summary"]["max_thrust_gf"]
