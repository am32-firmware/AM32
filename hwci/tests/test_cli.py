"""CLI-layer tests: sim/hw mode selection, baseline bootstrap, self-describing runs."""
import argparse

from hwci import cli
from hwci.config import load_profile
from hwci.model import RunResult


def _ns(**kw):
    return argparse.Namespace(**kw)


def test_sim_requires_explicit_flag_or_no_config():
    assert cli._use_sim(_ns(sim=True, config="rig.yaml"))
    assert cli._use_sim(_ns(sim=False, config=None))
    # A rig config without --sim is ALWAYS a hardware run - a sim backend
    # value can no longer flip it (load_rig rejects sim backends anyway).
    assert not cli._use_sim(_ns(sim=False, config="rig.yaml"))


def test_missing_baseline_warns_and_skips(tmp_path, capsys):
    assert cli._load_baseline(str(tmp_path / "nope.json")) is None
    assert "not found" in capsys.readouterr().err
    assert cli._load_baseline(None) is None


def test_profile_for_prefers_embedded_definition():
    profile = load_profile("ci_smoke")
    edited = cli.profile_to_dict(profile)
    edited["demag_rpm_drop_fraction"] = 0.11  # differs from today's YAML
    result = RunResult(meta={"profile": "ci_smoke", "profile_def": edited})
    assert cli._profile_for(result).demag_rpm_drop_fraction == 0.11
    # No embedded definition -> falls back to loading by name.
    legacy = RunResult(meta={"profile": "ci_smoke"})
    assert cli._profile_for(legacy).name == "ci_smoke"


def test_selftest_runs_clean(capsys):
    rc = cli.cmd_selftest(_ns(profile="ci_smoke"))
    assert rc == 0
    out = capsys.readouterr().out
    assert "steady points" in out
