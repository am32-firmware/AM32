"""Auto-tuner core: end-to-end sim tunes (injected optimum), objective
weighting, constraint disqualification, anchor normalization, tie-breaks."""
import pytest
import yaml

from hwci.model import RunResult
from hwci.sim import MotorParams
from hwci.tuner import (SimTuneBackend, Tuner, objective_score, startup_stats,
                        startup_profile, tune_spec_from_dict)


def small_spec(**extra) -> dict:
    d = {
        "name": "test",
        "probe": {"dwell_s": 1.0},
        "objective": {"min_power_w": 5.0, "noise_floor_pct": 0.5},
        "anchors_every": 4,
        "parameters": {
            "advance_level": {"values": [14, 18, 22, 26, 30, 34, 38],
                              "refine_step": 2},
            "pwm_frequency": {"values": [8, 16, 24, 48]},
            "variable_pwm": {"values": [0, 1, 2]},
            "auto_advance": {"values": [0, 1]},
            "max_ramp": {"values": [160, 80, 40]},
        },
        "stages": [
            {"name": "advance", "sweep": "advance_level"},
            {"name": "pwm", "sweep": "pwm_frequency",
             "fixed": {"variable_pwm": 0}},
        ],
        "finals": {"profile": "tune_probe", "repeats": 1,
                   "startup_check": False},
    }
    d.update(extra)
    return d


def make_backend(**params) -> SimTuneBackend:
    defaults = dict(pole_pairs=7, demag_prone=True, startup_fail_ref=100.0)
    defaults.update(params)
    return SimTuneBackend(motor_params=MotorParams(**defaults), noise=0.0)


def run_tune(tmp_path, spec_d, backend, **kw):
    spec = tune_spec_from_dict(spec_d)
    t = Tuner(spec, backend, tmp_path / "tune",
              spec_text=yaml.safe_dump(spec_d), no_prompt=True,
              log=lambda s: None, **kw)
    return t, t.run()


# --------------------------------------------------------------------------
# end-to-end: injected optimum is found
# --------------------------------------------------------------------------
def test_e2e_finds_injected_optimum(tmp_path):
    backend = make_backend(advance_optimum=27.0, pwm_optimum_khz=24.0)
    _, result = run_tune(tmp_path, small_spec(), backend)
    winner = result["winner_overrides"]
    # advance grid is every 4, refine step 2: winner within one refine step
    assert abs(winner["advance_level"] - 27) <= 2
    assert winner["pwm_frequency"] == 24


def test_e2e_confirms_a_real_improvement_and_writes_outputs(tmp_path):
    # optimum far from the default (26) so finals see a real paired delta
    backend = make_backend(advance_optimum=33.0)
    tuner, result = run_tune(tmp_path, small_spec(), backend)
    assert result["confirmed"]
    assert result["median_paired_delta"] > 0
    out = tmp_path / "tune"
    assert (out / "report.md").exists()
    assert (out / "settings_diff.md").exists()
    assert (out / "base_settings.bin").exists()
    best = (out / "best_settings.bin").read_bytes()
    assert best[23] == result["winner_overrides"]["advance_level"]
    # identity bytes preserved from the base page (never mutated)
    base = (out / "base_settings.bin").read_bytes()
    assert best[1] == base[1] and best[3:5] == base[3:5]


def test_e2e_default_optimum_keeps_default_settings(tmp_path):
    # optimum AT the default: winner is within noise of default; finals must
    # not confirm an improvement, and best_settings.bin stays the base page
    backend = make_backend(advance_optimum=26.0)
    _, result = run_tune(tmp_path, small_spec(), backend)
    out = tmp_path / "tune"
    if not result["confirmed"]:
        assert (out / "best_settings.bin").read_bytes() == \
            (out / "base_settings.bin").read_bytes()


# --------------------------------------------------------------------------
# constraint disqualification
# --------------------------------------------------------------------------
def test_constraint_only_stage_disqualifies_desyncing_ramp(tmp_path):
    spec_d = small_spec(stages=[
        {"name": "ramp", "sweep": "max_ramp", "constraint_only": True,
         "profile": "tune_step"},
    ])
    _, result = run_tune(tmp_path, spec_d, make_backend())
    # 160 desyncs on the step profile (disqualified: demag + bemf timeouts);
    # 80 is the first listed value that passes
    ledger = (tmp_path / "tune" / "manifest.json").read_text()
    assert "demag events" in ledger
    assert result["winner_overrides"]["max_ramp"] == 80


def test_disqualified_candidate_never_wins(tmp_path):
    tuner, _ = run_tune(tmp_path, small_spec(stages=[]), make_backend())
    cands = [
        {"overrides": {"advance_level": 30}, "order": 0, "value": 30,
         "entries": [{"score_raw": 99.0, "score_norm": 99.0,
                      "disqualified": ["demag events 1 > 0"],
                      "jitter_pct": 0.1, "fet_temp_c": None}]},
        {"overrides": {"advance_level": 22}, "order": 1, "value": 22,
         "entries": [{"score_raw": 5.0, "score_norm": 5.0,
                      "disqualified": None,
                      "jitter_pct": 0.5, "fet_temp_c": None}]},
    ]
    assert tuner._pick_winner(cands)["overrides"] == {"advance_level": 22}


def test_all_disqualified_yields_no_winner(tmp_path):
    tuner, _ = run_tune(tmp_path, small_spec(stages=[]), make_backend())
    cands = [{"overrides": {}, "order": 0, "value": 0,
              "entries": [{"score_raw": None, "score_norm": None,
                           "disqualified": ["run aborted: x"],
                           "jitter_pct": None, "fet_temp_c": None}]}]
    assert tuner._pick_winner(cands) is None


# --------------------------------------------------------------------------
# objective weighting
# --------------------------------------------------------------------------
def _metrics(points):
    return {"steady_points": [
        {"segment": lbl, "eff_gf_per_w": eff, "elec_power_w": pw}
        for lbl, eff, pw in points]}


def test_objective_weighted_mean():
    spec = tune_spec_from_dict({
        "name": "t",
        "objective": {"weights": {"a": 1.0, "b": 3.0}, "min_power_w": 10.0}})
    m = _metrics([("a", 4.0, 50.0), ("b", 8.0, 50.0)])
    assert objective_score(m, spec.objective) == pytest.approx(
        (1 * 4.0 + 3 * 8.0) / 4)


def test_objective_excludes_low_power_points():
    spec = tune_spec_from_dict({"name": "t"})
    m = _metrics([("t30", 40.0, 2.3),      # noise point, must not score
                  ("t50", 6.0, 50.0), ("t70", 5.0, 100.0)])
    assert objective_score(m, spec.objective) == pytest.approx(
        (2 * 6.0 + 1 * 5.0) / 3)


def test_objective_unlisted_label_gets_weight_one():
    spec = tune_spec_from_dict(
        {"name": "t", "objective": {"weights": {}, "min_power_w": 1.0}})
    m = _metrics([("x", 4.0, 50.0), ("y", 8.0, 50.0)])
    assert objective_score(m, spec.objective) == pytest.approx(6.0)


def test_objective_none_when_no_point_qualifies():
    spec = tune_spec_from_dict({"name": "t"})
    assert objective_score(_metrics([("t30", 40.0, 2.0)]),
                           spec.objective) is None


# --------------------------------------------------------------------------
# anchor normalization
# --------------------------------------------------------------------------
def test_drift_factor_interpolates_between_anchors():
    anchors = [(0, 10.0), (5, 8.0)]
    assert Tuner._drift_factor(anchors, 0) == pytest.approx(1.0)
    assert Tuner._drift_factor(anchors, 5) == pytest.approx(10.0 / 8.0)
    assert Tuner._drift_factor(anchors, 2) == pytest.approx(10.0 / 9.2)
    assert Tuner._drift_factor(anchors, 9) == pytest.approx(10.0 / 8.0)
    assert Tuner._drift_factor([], 3) == 1.0


def test_e2e_finds_optimum_under_injected_linear_drift(tmp_path):
    # The rig degrades linearly (sagging pack modeled as falling motor
    # efficiency): raw scores of later trials read lower. Anchor
    # normalization must cancel it so the argmax stays at the optimum.
    backend = make_backend(advance_optimum=27.0)

    def drift(index, plan):
        backend.sim.params.motor_efficiency = 0.82 * (1.0 - 0.004 * index)

    spec_d = small_spec(stages=[{"name": "advance",
                                 "sweep": "advance_level"}])
    _, result = run_tune(tmp_path, spec_d, backend, before_trial=drift)
    assert abs(result["winner_overrides"]["advance_level"] - 27) <= 2
    # anchors themselves normalize flat (same settings, drift cancelled)
    import json
    m = json.loads((tmp_path / "tune" / "manifest.json").read_text())
    anchors = [e for e in m["trials"]
               if e["stage"] == "advance" and e["kind"] == "anchor"]
    norms = [e["score_norm"] for e in anchors if e["score_norm"]]
    assert max(norms) - min(norms) <= 0.02 * norms[0]
    # ...while their raw scores visibly drifted
    raws = [e["score_raw"] for e in anchors]
    assert max(raws) - min(raws) > 0.02 * raws[0]


# --------------------------------------------------------------------------
# tie-break order: jitter, then FET temp, then closest-to-default
# --------------------------------------------------------------------------
def _cand(order, ov, score, jitter, fet):
    return {"overrides": ov, "order": order, "value": order,
            "entries": [{"score_raw": score, "score_norm": score,
                         "disqualified": None, "jitter_pct": jitter,
                         "fet_temp_c": fet}]}


def test_tie_breaks_on_jitter_first(tmp_path):
    tuner, _ = run_tune(tmp_path, small_spec(stages=[]), make_backend())
    tuner.spec.objective.noise_floor_pct = 3.0
    cands = [_cand(0, {"advance_level": 30}, 6.00, jitter=0.9, fet=40.0),
             _cand(1, {"advance_level": 34}, 6.05, jitter=0.5, fet=60.0)]
    assert tuner._pick_winner(cands)["overrides"] == {"advance_level": 34}


def test_tie_breaks_on_fet_temp_second(tmp_path):
    tuner, _ = run_tune(tmp_path, small_spec(stages=[]), make_backend())
    tuner.spec.objective.noise_floor_pct = 3.0
    cands = [_cand(0, {"advance_level": 30}, 6.00, jitter=0.5, fet=40.0),
             _cand(1, {"advance_level": 34}, 6.05, jitter=0.5, fet=60.0)]
    assert tuner._pick_winner(cands)["overrides"] == {"advance_level": 30}


def test_tie_breaks_on_distance_to_default_third(tmp_path):
    tuner, _ = run_tune(tmp_path, small_spec(stages=[]), make_backend())
    tuner.spec.objective.noise_floor_pct = 3.0
    # default advance_level is 26 (base page): 28 is closer than 34
    cands = [_cand(0, {"advance_level": 34}, 6.05, jitter=0.5, fet=40.0),
             _cand(1, {"advance_level": 28}, 6.00, jitter=0.5, fet=40.0)]
    assert tuner._pick_winner(cands)["overrides"] == {"advance_level": 28}


def test_outside_noise_floor_score_wins_regardless(tmp_path):
    tuner, _ = run_tune(tmp_path, small_spec(stages=[]), make_backend())
    tuner.spec.objective.noise_floor_pct = 3.0
    cands = [_cand(0, {"advance_level": 30}, 6.50, jitter=9.9, fet=99.0),
             _cand(1, {"advance_level": 34}, 6.00, jitter=0.1, fet=20.0)]
    assert tuner._pick_winner(cands)["overrides"] == {"advance_level": 30}


# --------------------------------------------------------------------------
# startup stats (inline PR #22 stand-in)
# --------------------------------------------------------------------------
def test_startup_stats_counts_failed_cycles():
    spec = tune_spec_from_dict({"name": "t", "constraints": {
        "startup": {"cycles": 3, "spin_throttle": 0.15, "min_rpm": 1000}}})
    profile = startup_profile(spec)
    rows = []
    t = 0.0
    for seg in profile.segments:
        n = int(seg.duration_s * 100)
        for _ in range(n):
            rpm = 0.0
            if seg.label.startswith("spin"):
                # spin0 healthy, spin1 fails (never spins), spin2 healthy
                rpm = 0.0 if seg.label == "spin1" else 4000.0
            rows.append({"t": t, "segment": seg.label,
                         "stand_rpm": rpm, "perf_e_rpm": rpm * 7})
            t += 0.01
    result = RunResult(meta={"pole_pairs": 7}, rows=rows)
    st = startup_stats(result, profile, min_rpm=1000.0)
    assert st == {"cycles": 3, "failed": 1, "failed_segments": ["spin1"],
                  "min_rpm": 1000.0}


def test_startup_stats_uses_perf_erpm_when_stand_is_dead():
    spec = tune_spec_from_dict({"name": "t", "constraints": {
        "startup": {"cycles": 1}}})
    profile = startup_profile(spec)
    rows = [{"t": i * 0.01, "segment": "spin0", "stand_rpm": None,
             "perf_e_rpm": 4000 * 7} for i in range(150)]
    result = RunResult(meta={"pole_pairs": 7}, rows=rows)
    assert startup_stats(result, profile, min_rpm=1000.0)["failed"] == 0
