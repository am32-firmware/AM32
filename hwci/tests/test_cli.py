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


# --------------------------------------------------------------------------
# hwci tune / hwci settings
# --------------------------------------------------------------------------
def test_tune_sim_end_to_end(tmp_path, capsys):
    import yaml
    spec = tmp_path / "spec.yaml"
    spec.write_text(yaml.safe_dump({
        "name": "cli-e2e",
        "probe": {"dwell_s": 1.0},
        "objective": {"min_power_w": 5.0},
        "parameters": {"advance_level": {"values": [22, 26, 30]}},
        "stages": [{"name": "advance", "sweep": "advance_level"}],
        "finals": {"profile": "tune_probe", "repeats": 1,
                   "startup_check": False},
    }))
    out = tmp_path / "tune"
    rc = cli.main(["tune", "--sim", "--spec", str(spec), "--out", str(out),
                   "--no-prompt"])
    assert rc == 0
    assert (out / "report.md").exists()
    assert (out / "best_settings.bin").exists()
    assert len((out / "best_settings.bin").read_bytes()) == 192
    assert (out / "manifest.json").exists()
    assert (out / "spec.yaml").exists()


def test_tune_requires_spec_or_resume(capsys):
    rc = cli.main(["tune", "--sim", "--no-prompt"])
    assert rc == 2
    assert "--spec" in capsys.readouterr().err


def test_settings_round_trip_against_mock_device(tmp_path, capsys, monkeypatch):
    from hwci.settings import Settings
    # fresh in-process simulated page for this test
    monkeypatch.setattr(cli, "_SIM_SETTINGS_DEVICE", None)

    # read: prints fields and saves the page
    page = tmp_path / "page.bin"
    assert cli.main(["settings", "read", "--sim", "--bin", str(page)]) == 0
    assert "advance_level" in capsys.readouterr().out
    assert len(page.read_bytes()) == 192

    # diff against a modified blob reports the difference (exit 1)
    modified = tmp_path / "mod.bin"
    Settings.from_bin(page).apply({"advance_level": 30}).to_bin(modified)
    assert cli.main(["settings", "diff", "--sim", "--bin", str(modified)]) == 1
    assert "advance_level" in capsys.readouterr().out

    # write flashes the blob (MockDebugger records the flash), then verify
    assert cli.main(["settings", "write", "--sim", "--bin", str(modified)]) == 0
    assert cli._SIM_SETTINGS_DEVICE.flashed  # flash path actually used
    assert "verified" in capsys.readouterr().out

    # device now matches: diff is clean and read shows the new value
    assert cli.main(["settings", "diff", "--sim", "--bin", str(modified)]) == 0
    cli.main(["settings", "read", "--sim"])
    assert "advance_level    =  30" in capsys.readouterr().out


def test_settings_write_requires_bin(capsys):
    rc = cli.main(["settings", "write", "--sim"])
    assert rc == 2
    assert "--bin" in capsys.readouterr().err
