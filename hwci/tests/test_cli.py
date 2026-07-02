"""CLI-layer tests: sim/hw mode selection, baseline bootstrap, self-describing runs."""
import argparse

from hwci import cli
from hwci.config import RigConfig, load_profile
from hwci.model import RunResult
from hwci.runner import DEFAULT_MIN_CELL_VOLTAGE


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


def test_run_and_ci_expose_battery_flags_with_defaults():
    parser = cli.build_parser()
    run_ns = parser.parse_args(["run", "--profile", "ci_smoke"])
    assert run_ns.battery_cells is None
    assert run_ns.min_cell_voltage == DEFAULT_MIN_CELL_VOLTAGE

    ci_ns = parser.parse_args(["ci", "--battery-cells", "6",
                               "--min-cell-voltage", "3.5"])
    assert ci_ns.battery_cells == 6
    assert ci_ns.min_cell_voltage == 3.5


def test_run_and_ci_expose_no_tare_flag():
    parser = cli.build_parser()
    assert parser.parse_args(["run", "--profile", "ci_smoke"]).no_tare is False
    assert parser.parse_args(["ci", "--no-tare"]).no_tare is True


def test_sim_runs_are_never_marked_tared():
    # The simulator has no load cells to zero; meta must say so even when
    # taring wasn't explicitly disabled, or a sim run dir would claim a
    # pre-flight step that never happened.
    rig = RigConfig()
    profile = load_profile("ci_smoke")
    result = cli._execute(rig, profile, sim=True)
    assert result.meta["tared"] is False


def test_battery_check_does_not_apply_in_sim_mode():
    # The built-in simulator's nominal pack voltage doesn't represent any
    # particular real cell count, so --battery-cells must be a no-op under
    # --sim rather than spuriously aborting every simulated/offline run.
    rig = RigConfig()
    profile = load_profile("ci_smoke")
    result = cli._execute(rig, profile, sim=True, battery_cells=6)
    assert result.meta["aborted"] is None
    assert result.meta["battery_cells"] == 6  # still recorded for the record
