"""Tune spec loading is STRICT: unknown keys/params fail loudly (a typo must
never silently become 'use the default'), values are validated against the
firmware-accepted ranges before anything reaches the bench."""
from pathlib import Path

import pytest

from hwci.settings import SettingsError
from hwci.tuner import TuneSpecError, load_tune_spec, tune_spec_from_dict

REPO_ROOT = Path(__file__).resolve().parents[2]


def _minimal(**extra) -> dict:
    return {"name": "t", **extra}


def test_minimal_spec_gets_documented_defaults():
    spec = tune_spec_from_dict(_minimal())
    assert spec.objective.weights == {"t30": 1.0, "t50": 2.0,
                                      "t70": 1.0, "t90": 0.5}
    assert spec.objective.min_power_w == 20.0
    assert spec.objective.noise_floor_pct == 3.0
    assert spec.anchors_every == 5
    assert spec.constraints.max_demag_events == 0
    assert spec.constraints.jitter_max_regression_pct == 25.0
    assert spec.constraints.startup.cycles == 5
    assert spec.finals.pattern == "ABBA"


def test_example_spec_loads():
    spec = load_tune_spec(REPO_ROOT / "hwci" / "tunes" / "example.yaml")
    assert {s.name for s in spec.stages} == {"advance", "pwm", "modes", "ramp"}
    assert spec.parameters["advance_level"].refine_step == 2
    assert spec.stages[3].constraint_only


def test_name_is_required():
    with pytest.raises(TuneSpecError, match="'name' is required"):
        tune_spec_from_dict({})


def test_unknown_top_level_key_fails():
    with pytest.raises(TuneSpecError, match="advnce_level"):
        tune_spec_from_dict(_minimal(advnce_level=1))


@pytest.mark.parametrize("block,bad", [
    ("objective", {"weights": {}, "min_powr_w": 5}),
    ("constraints", {"max_demag_evnts": 1}),
    ("pack", {"min_resting_cel_v": 3.4}),
    ("probe", {"dwel_s": 3}),
    ("finals", {"repeat": 3}),
])
def test_unknown_nested_key_fails(block, bad):
    with pytest.raises(TuneSpecError, match="unknown key"):
        tune_spec_from_dict(_minimal(**{block: bad}))


def test_unknown_startup_key_fails():
    with pytest.raises(TuneSpecError, match="constraints.startup"):
        tune_spec_from_dict(_minimal(
            constraints={"startup": {"cycels": 3}}))


def test_unknown_parameter_without_offset_fails():
    with pytest.raises(SettingsError, match="unknown setting"):
        tune_spec_from_dict(_minimal(
            parameters={"warp_drive": {"values": [1, 2]}}))


def test_unknown_parameter_with_offset_is_forward_compat():
    spec = tune_spec_from_dict(_minimal(
        parameters={"future_field": {"values": [1, 2], "offset": 60}}))
    assert spec.parameters["future_field"].offset == 60


def test_parameter_value_outside_firmware_range_fails():
    with pytest.raises(TuneSpecError, match="firmware-valid range"):
        tune_spec_from_dict(_minimal(
            parameters={"advance_level": {"values": [10, 43]}}))


def test_stage_sweeping_undeclared_parameter_fails():
    with pytest.raises(TuneSpecError, match="unknown parameter"):
        tune_spec_from_dict(_minimal(
            stages=[{"name": "s", "sweep": "advance_level"}]))


def test_stage_needs_exactly_one_of_sweep_or_ab():
    params = {"advance_level": {"values": [20]}}
    with pytest.raises(TuneSpecError, match="exactly one"):
        tune_spec_from_dict(_minimal(
            parameters=params, stages=[{"name": "s"}]))
    with pytest.raises(TuneSpecError, match="exactly one"):
        tune_spec_from_dict(_minimal(
            parameters=params,
            stages=[{"name": "s", "sweep": "advance_level",
                     "ab_candidates": [{}]}]))


def test_duplicate_stage_name_fails():
    params = {"advance_level": {"values": [20]}}
    with pytest.raises(TuneSpecError, match="duplicate stage"):
        tune_spec_from_dict(_minimal(
            parameters=params,
            stages=[{"name": "s", "sweep": "advance_level"},
                    {"name": "s", "sweep": "advance_level"}]))


def test_ab_candidate_out_of_range_fails():
    with pytest.raises(TuneSpecError, match="firmware-valid range"):
        tune_spec_from_dict(_minimal(
            stages=[{"name": "s", "ab_candidates": [{"variable_pwm": 5}]}]))


def test_unsupported_finals_pattern_fails():
    with pytest.raises(TuneSpecError, match="ABBA"):
        tune_spec_from_dict(_minimal(finals={"pattern": "AABB"}))


def test_non_mapping_spec_fails(tmp_path):
    p = tmp_path / "spec.yaml"
    p.write_text("- just\n- a list\n")
    with pytest.raises(TuneSpecError, match="not a YAML mapping"):
        load_tune_spec(p)
