"""Tests for strict rig-config validation and profile (de)serialization."""
import pytest

from hwci.config import (RigConfig, load_profile, load_rig,
                         profile_from_dict, profile_to_dict)

VALID_RIG = """\
target: ARK_4IN1_F051
debugger_backend: openocd
telem_backend: serial
throttle_backend: flightstand
stand_backend: grpc
pole_pairs: 11
"""


def _write(tmp_path, text):
    p = tmp_path / "rig.yaml"
    p.write_text(text)
    return str(p)


def test_valid_rig_loads(tmp_path):
    rig = load_rig(_write(tmp_path, VALID_RIG))
    assert rig.debugger_backend == "openocd"
    assert rig.pole_pairs == 11


def test_unknown_key_rejected(tmp_path):
    with pytest.raises(ValueError, match="unknown key"):
        load_rig(_write(tmp_path, VALID_RIG + "debugger_bakend: openocd\n"))


def test_unknown_backend_value_rejected(tmp_path):
    bad = VALID_RIG.replace("stand_backend: grpc", "stand_backend: gprc")
    with pytest.raises(ValueError, match="stand_backend"):
        load_rig(_write(tmp_path, bad))


def test_sim_backend_rejected_in_rig_file(tmp_path):
    bad = VALID_RIG.replace("stand_backend: grpc", "stand_backend: sim")
    with pytest.raises(ValueError, match="not allowed in a rig file"):
        load_rig(_write(tmp_path, bad))


def test_omitted_backend_rejected_in_rig_file(tmp_path):
    # An omitted backend would default to "sim" - a rig file must be explicit.
    partial = VALID_RIG.replace("telem_backend: serial\n", "")
    with pytest.raises(ValueError, match="telem_backend"):
        load_rig(_write(tmp_path, partial))


def test_flightstand_throttle_needs_a_stand(tmp_path):
    bad = VALID_RIG.replace("stand_backend: grpc", "stand_backend: none")
    with pytest.raises(ValueError, match="flightstand"):
        load_rig(_write(tmp_path, bad))


def test_none_backends_allowed(tmp_path):
    text = VALID_RIG.replace("stand_backend: grpc", "stand_backend: none")
    text = text.replace("throttle_backend: flightstand",
                        "throttle_backend: external")
    rig = load_rig(_write(tmp_path, text))
    assert rig.stand_backend == "none"


def test_no_config_gives_sim_defaults():
    rig = load_rig(None)
    assert rig.stand_backend == "sim"
    rig.validate()  # sim allowed for the built-in default


def test_profile_roundtrips_through_dict():
    p = load_profile("demag_step_stress")
    q = profile_from_dict(profile_to_dict(p))
    assert q == p
