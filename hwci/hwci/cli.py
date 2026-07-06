"""Command-line interface for the AM32 hardware-CI harness.

Examples
--------
  hwci profiles                         # list built-in test profiles
  hwci selftest                         # run ci_smoke in the simulator
  hwci run --profile efficiency_sweep --sim --out runs/sweep
  hwci analyze runs/sweep
  hwci baseline-save runs/sweep --out baselines/ARK_4IN1_F051.json
  hwci ci --profile ci_smoke --config rig.yaml \
          --baseline baselines/ARK_4IN1_F051.json --out runs/ci

Mode selection: a run is SIMULATED only when ``--sim`` is passed or no
``--config`` is given; a rig config always means hardware (and refuses
simulator backends), so a typo can never silently gate simulated data.
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from datetime import datetime
from pathlib import Path

import yaml

from . import baseline as bl
from . import metrics as metricsmod
from . import report as reportmod
from .config import (Profile, RigConfig, list_profiles, load_profile,
                     load_rig, profile_from_dict, profile_to_dict)
from .model import RunResult
from .runner import (DEFAULT_MIN_CELL_VOLTAGE, build_live_sources,
                     build_sim_sources, run_profile)


def _git_sha(repo_root: str) -> str | None:
    try:
        out = subprocess.run(["git", "rev-parse", "--short", "HEAD"],
                             cwd=repo_root, capture_output=True, text=True)
        return out.stdout.strip() or None
    except Exception:
        return None


def _make_meta(rig: RigConfig, profile: Profile, mode: str,
               extra: dict | None = None) -> dict:
    meta = {
        "target": rig.target,
        "profile": profile.name,
        # Full profile definition: the run dir stays analyzable even if the
        # profile YAML changes later (or was a custom file path).
        "profile_def": profile_to_dict(profile),
        "mode": mode,  # "sim" | "hw" - shown in the report, never inferred later
        "pole_pairs": rig.pole_pairs,
        "git_sha": _git_sha(rig.repo_root),
        "motor": rig.motor_name,
        "prop": rig.prop,
        "timestamp": datetime.now().isoformat(timespec="seconds"),
    }
    if extra:
        meta.update(extra)
    return meta


def _use_sim(args) -> bool:
    """Simulation is explicit: --sim, or no rig config at all."""
    return bool(getattr(args, "sim", False)) or not getattr(args, "config", None)


def _default_out(profile_name: str, sim: bool) -> Path:
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    tag = "sim" if sim else "hw"
    return Path("runs") / f"{profile_name}-{tag}-{stamp}"


def _profile_for(result: RunResult) -> Profile:
    """The exact profile a run was made with (from meta), else by name."""
    pd = result.meta.get("profile_def")
    if pd:
        return profile_from_dict(pd)
    return load_profile(result.meta.get("profile", "ci_smoke"))


def _load_baseline(path: str | None) -> dict | None:
    if not path:
        return None
    p = Path(path)
    if not p.exists():
        print(f"WARNING: baseline {p} not found - skipping the regression "
              "gate (bootstrap run? capture one with 'hwci baseline-save')",
              file=sys.stderr)
        return None
    return bl.load_baseline(p)


def _execute(rig: RigConfig, profile: Profile, sim: bool, *,
             realtime: bool | None = None,
             battery_cells: int | None = None,
             min_cell_voltage: float = DEFAULT_MIN_CELL_VOLTAGE,
             tare: bool = True) -> RunResult:
    """Build sources, run the profile, always close sources.

    The battery and tare pre-flight steps only apply to hardware: a simulated
    pack doesn't represent any real cell count and simulated load cells have
    no zero drift, so neither is ever passed to the simulator sources.
    """
    sources = (build_sim_sources(rig, profile) if sim
               else build_live_sources(rig, profile, battery_cells=battery_cells,
                                       min_cell_voltage=min_cell_voltage,
                                       tare=tare))
    # Recorded so a drifting thrust offset in saved data can be told apart
    # from "this run simply wasn't tared".
    tared = bool(tare) and not sim and rig.stand_backend == "grpc"
    try:
        return run_profile(
            profile, sources,
            realtime=(not sim) if realtime is None else realtime,
            meta=_make_meta(rig, profile, "sim" if sim else "hw",
                           extra={"battery_cells": battery_cells,
                                  "tared": tared}))
    finally:
        sources.close()


def _analyze_and_report(run_dir: Path, result: RunResult, profile: Profile,
                        baseline_path: str | None, no_plots: bool):
    """Compute metrics, write metrics.json + report.md, gate vs baseline."""
    m = metricsmod.compute(result, profile)
    (run_dir / "metrics.json").write_text(json.dumps(m, indent=2))
    baseline = _load_baseline(baseline_path)
    comparison = (bl.compare(m, baseline, current_meta=result.meta)
                  if baseline is not None else None)
    reportmod.write_report(run_dir, m, comparison, result.meta,
                           plots=not no_plots)
    return m, comparison


def _verdict_rc(result: RunResult, comparison: dict | None) -> int:
    """Exit code: 2 aborted (never trust the data), 1 gate FAIL, 0 PASS."""
    if result.meta.get("aborted"):
        print(f"ABORTED: {result.meta['aborted']}", file=sys.stderr)
        return 2
    if comparison is not None:
        print("VERDICT:", "PASS" if comparison["passed"] else "FAIL")
        return 0 if comparison["passed"] else 1
    return 0


# --------------------------------------------------------------------------
def cmd_profiles(args) -> int:
    for name in list_profiles():
        p = load_profile(name)
        print(f"{name:22s} {p.duration_s:5.0f}s  {p.description.strip().splitlines()[0]}")
    return 0


def cmd_selftest(args) -> int:
    rig = RigConfig()
    profile = load_profile(args.profile)
    sources = build_sim_sources(rig, profile, demag_prone=True)
    try:
        result = run_profile(profile, sources, realtime=False,
                             meta=_make_meta(rig, profile, "sim"))
    finally:
        sources.close()
    m = metricsmod.compute(result, profile)
    print(json.dumps(m["summary"], indent=2))
    print(f"\n{len(result.rows)} samples, "
          f"{len(m['steady_points'])} steady points, "
          f"{m['demag']['event_count']} demag events")
    return 0


def cmd_build(args) -> int:
    from .build import build_firmware
    rig = load_rig(args.config)
    target = args.target or rig.target
    arts = build_firmware(rig.repo_root, target, hwci_perf=not args.no_perf,
                          arm_sdk_prefix=args.arm_sdk_prefix)
    print(f"built {arts.elf}\n      {arts.bin}")
    return 0


def cmd_flash(args) -> int:
    from .build import build_firmware
    from .debugger.openocd import OpenOcdDebugger
    rig = load_rig(args.config)
    if args.bin:
        binf = Path(args.bin)
    else:
        arts = build_firmware(rig.repo_root, rig.target,
                              hwci_perf=not args.no_perf,
                              arm_sdk_prefix=args.arm_sdk_prefix)
        binf = arts.bin
    dbg = OpenOcdDebugger(rig.openocd_configs, openocd_bin=rig.openocd_bin,
                          search_dirs=rig.openocd_search_dirs)
    dbg.flash(str(binf), rig.app_load_addr)
    print(f"flashed {binf} @ 0x{rig.app_load_addr:08x}")
    return 0


def cmd_run(args) -> int:
    sim = _use_sim(args)
    rig = load_rig(args.config) if args.config else RigConfig()
    profile = load_profile(args.profile)
    result = _execute(rig, profile, sim,
                      realtime=True if args.realtime else None,
                      battery_cells=args.battery_cells,
                      min_cell_voltage=args.min_cell_voltage,
                      tare=not args.no_tare)
    out = Path(args.out) if args.out else _default_out(profile.name, sim)
    result.save(out)
    print(f"saved {len(result.rows)} samples to {out}")
    return _verdict_rc(result, None)


def cmd_analyze(args) -> int:
    result = RunResult.load(args.run_dir)
    m = metricsmod.compute(result, _profile_for(result))
    (Path(args.run_dir) / "metrics.json").write_text(json.dumps(m, indent=2))
    print(json.dumps(m["summary"], indent=2))
    return 0


def cmd_baseline_save(args) -> int:
    result = RunResult.load(args.run_dir)
    m = metricsmod.compute(result, _profile_for(result))
    out = args.out or f"baselines/{result.meta.get('target', 'unknown')}.json"
    bl.save_baseline(m, out, meta=result.meta)
    print(f"baseline saved to {out}")
    return 0


def cmd_report(args) -> int:
    result = RunResult.load(args.run_dir)
    _, comparison = _analyze_and_report(
        Path(args.run_dir), result, _profile_for(result),
        args.baseline, args.no_plots)
    print(f"report written to {Path(args.run_dir) / 'report.md'}")
    return _verdict_rc(result, comparison)


def cmd_ci(args) -> int:
    sim = _use_sim(args)
    rig = load_rig(args.config) if args.config else RigConfig()
    profile = load_profile(args.profile)
    out = Path(args.out) if args.out else _default_out(profile.name, sim)

    if not sim:
        from .build import build_firmware
        arts = build_firmware(rig.repo_root, rig.target, hwci_perf=True,
                              arm_sdk_prefix=args.arm_sdk_prefix)
        if rig.debugger_backend == "openocd":
            from .debugger.openocd import OpenOcdDebugger
            dbg = OpenOcdDebugger(rig.openocd_configs, openocd_bin=rig.openocd_bin,
                                  search_dirs=rig.openocd_search_dirs)
            dbg.flash(str(arts.bin), rig.app_load_addr)
            dbg.close()
        else:
            print("WARNING: debugger_backend is not 'openocd' - firmware was "
                  "built but NOT flashed; testing whatever is on the target",
                  file=sys.stderr)
        rig.elf_path = str(arts.elf)

    result = _execute(rig, profile, sim,
                      battery_cells=args.battery_cells,
                      min_cell_voltage=args.min_cell_voltage,
                      tare=not args.no_tare)
    result.save(out)

    m, comparison = _analyze_and_report(out, result, profile,
                                        args.baseline, args.no_plots)
    print(json.dumps(m["summary"], indent=2))
    return _verdict_rc(result, comparison)


def cmd_tune(args) -> int:
    from . import tuner as tunermod

    if args.resume:
        out = Path(args.resume)
        manifest = json.loads((out / "manifest.json").read_text())
        spec_text = (out / "spec.yaml").read_text()
        spec = tunermod.tune_spec_from_dict(yaml.safe_load(spec_text))
        sim = manifest.get("mode") == "sim"
        config_path = manifest.get("config_path")
        rig = (load_rig(config_path) if (config_path and not sim)
               else RigConfig())
        battery_cells = manifest.get("battery_cells")
        resume = True
    else:
        if not args.spec:
            print("error: hwci tune needs --spec (or --resume)", file=sys.stderr)
            return 2
        spec_text = Path(args.spec).read_text()
        spec = tunermod.load_tune_spec(args.spec)
        sim = _use_sim(args)
        rig = load_rig(args.config) if args.config else RigConfig()
        out = Path(args.out) if args.out else _default_out(
            f"tune-{spec.name}", sim)
        config_path = args.config
        battery_cells = args.battery_cells
        resume = False

    backend = (tunermod.SimTuneBackend(rig) if sim
               else tunermod.HwTuneBackend(rig))
    try:
        tuner = tunermod.Tuner(
            spec, backend, out, spec_text=spec_text,
            battery_cells=battery_cells, no_prompt=args.no_prompt,
            resume=resume, config_path=config_path)
        result = tuner.run()
    except tunermod.TunePaused as e:
        print(f"PAUSED: {e}", file=sys.stderr)
        return 3
    finally:
        backend.close()
    pdf = out / "tune_report.pdf"
    print(f"report: {out / 'report.md'}"
          + (f"\nPDF report: {pdf}" if pdf.exists() else "")
          + f"\nbest settings: {out / 'best_settings.bin'} "
          f"({'winner' if result['confirmed'] else 'default kept'})")
    return 0


# Persistent in-process simulated settings page for `hwci settings --sim`,
# so read/write/diff round-trip within one process (tests, demos).
_SIM_SETTINGS_DEVICE = None


def _sim_settings_device():
    from .debugger.base import MockDebugger
    from .settings import DEFAULT_EEPROM_ADDRESS, default_blob

    class _Device(MockDebugger):
        def flash(self, bin_path: str, load_addr: int) -> None:
            super().flash(bin_path, load_addr)
            self.poke(load_addr, Path(bin_path).read_bytes())

    global _SIM_SETTINGS_DEVICE
    if _SIM_SETTINGS_DEVICE is None:
        _SIM_SETTINGS_DEVICE = _Device(base=DEFAULT_EEPROM_ADDRESS, size=192)
        _SIM_SETTINGS_DEVICE.poke(DEFAULT_EEPROM_ADDRESS, default_blob())
    return _SIM_SETTINGS_DEVICE, DEFAULT_EEPROM_ADDRESS


def cmd_settings(args) -> int:
    from . import settings as st

    sim = _use_sim(args)
    if sim:
        dbg, addr = _sim_settings_device()
        closer = lambda: None  # noqa: E731 - kept alive across invocations
        flash = dbg.flash
    else:
        from .debugger.openocd import OpenOcdDebugger
        rig = load_rig(args.config)
        elf = rig.resolved_elf()
        if elf is None:
            print(f"error: no ELF for target {rig.target}; the live "
                  "eeprom_address is resolved from the flashed ELF",
                  file=sys.stderr)
            return 2
        make = lambda: OpenOcdDebugger(  # noqa: E731
            rig.openocd_configs, openocd_bin=rig.openocd_bin,
            search_dirs=rig.openocd_search_dirs)
        dbg = make().open()
        closer = dbg.close
        addr = st.resolve_eeprom_address(dbg, str(elf))

        def flash(bin_path: str, load_addr: int) -> None:
            # One-shot flash must not race the open Tcl-RPC session on the
            # same ST-Link: close, program+reset, reopen to verify.
            nonlocal dbg, closer
            dbg.close()
            make().flash(bin_path, load_addr)
            dbg = make().open()
            closer = dbg.close

    try:
        current = st.Settings.from_device(dbg, addr)
        if args.action == "read":
            print(f"eeprom @ 0x{addr:08x}  sha256 {current.sha256()[:16]}")
            for name, value in current.describe().items():
                print(f"  {name:16s} = {value:3d}   "
                      f"({st.EEPROM_FIELDS[name].description})")
            if args.bin:
                current.to_bin(args.bin)
                print(f"saved page to {args.bin}")
            return 0
        if not args.bin:
            print(f"error: hwci settings {args.action} needs --bin",
                  file=sys.stderr)
            return 2
        other = st.Settings.from_bin(args.bin)
        if args.action == "diff":
            diff = current.diff(other)
            for name, a, b in diff:
                print(f"  {name:16s} device={a:3d}  {args.bin}={b:3d}")
            print(f"{len(diff)} byte(s) differ" if diff
                  else "no differences")
            return 1 if diff else 0
        # write
        flash(str(Path(args.bin)), addr)
        readback = st.Settings.from_device(dbg, addr)
        if readback != other:
            print("ERROR: readback after write does not match the file",
                  file=sys.stderr)
            return 1
        print(f"wrote {args.bin} to eeprom @ 0x{addr:08x} (verified)")
        return 0
    finally:
        closer()


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="hwci", description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)

    def add_common(sp):
        sp.add_argument("--config", help="rig config YAML (hardware run); "
                                         "omit for the built-in simulator")

    def add_preflight(sp):
        sp.add_argument("--battery-cells", type=int, metavar="N",
                        help="LiPo cell count under test (e.g. 6 for a 6S "
                             "pack); if given, refuses to start when pack "
                             "voltage is below N * --min-cell-voltage "
                             "(hardware only - ignored under --sim)")
        sp.add_argument("--min-cell-voltage", type=float, metavar="V",
                        default=DEFAULT_MIN_CELL_VOLTAGE,
                        help="per-cell cutoff volts for --battery-cells "
                             f"(default: {DEFAULT_MIN_CELL_VOLTAGE})")
        sp.add_argument("--no-tare", action="store_true",
                        help="skip the automatic pre-run load-cell tare "
                             "(hardware runs tare by default, with the ESC "
                             "signal held at zero throttle so AM32's "
                             "no-signal beacon beeps can't shake the cells "
                             "mid-tare)")

    sp = sub.add_parser("profiles", help="list test profiles")
    sp.set_defaults(func=cmd_profiles)

    sp = sub.add_parser("selftest", help="run a profile in the simulator")
    sp.add_argument("--profile", default="ci_smoke")
    sp.set_defaults(func=cmd_selftest)

    sp = sub.add_parser("build", help="build firmware (HWCI_PERF=1)")
    add_common(sp)
    sp.add_argument("--target")
    sp.add_argument("--no-perf", action="store_true", help="build without instrumentation")
    sp.add_argument("--arm-sdk-prefix")
    sp.set_defaults(func=cmd_build)

    sp = sub.add_parser("flash", help="flash firmware via OpenOCD")
    add_common(sp)
    sp.add_argument("--bin", help="bin to flash (default: build it)")
    sp.add_argument("--no-perf", action="store_true")
    sp.add_argument("--arm-sdk-prefix")
    sp.set_defaults(func=cmd_flash)

    sp = sub.add_parser("run", help="run a profile and save the data")
    add_common(sp)
    add_preflight(sp)
    sp.add_argument("--profile", required=True)
    sp.add_argument("--sim", action="store_true", help="force the simulator")
    sp.add_argument("--realtime", action="store_true", help="pace sim in real time")
    sp.add_argument("--out")
    sp.set_defaults(func=cmd_run)

    sp = sub.add_parser("analyze", help="compute metrics for a run dir")
    sp.add_argument("run_dir")
    sp.set_defaults(func=cmd_analyze)

    sp = sub.add_parser("baseline-save", help="save a run's metrics as the baseline")
    sp.add_argument("run_dir")
    sp.add_argument("--out")
    sp.set_defaults(func=cmd_baseline_save)

    sp = sub.add_parser("report", help="write a Markdown report for a run dir")
    sp.add_argument("run_dir")
    sp.add_argument("--baseline")
    sp.add_argument("--no-plots", action="store_true")
    sp.set_defaults(func=cmd_report)

    sp = sub.add_parser(
        "tune", help="auto-tune AM32 EEPROM settings for the motor/prop")
    add_common(sp)
    sp.add_argument("--spec", help="tune spec YAML (see hwci/tunes/example.yaml)")
    sp.add_argument("--out", help="session directory (default: runs/tune-...)")
    sp.add_argument("--sim", action="store_true", help="force the simulator")
    sp.add_argument("--battery-cells", type=int, metavar="N",
                    help="LiPo cell count; gates every trial on resting pack "
                         "voltage (overrides the spec's battery_cells)")
    sp.add_argument("--no-prompt", action="store_true",
                    help="never wait for input: checkpoint and exit cleanly "
                         "(exit code 3) instead of prompting for a pack swap")
    sp.add_argument("--resume", metavar="DIR",
                    help="resume a checkpointed session from its directory "
                         "(spec/config/mode are reloaded from the session)")
    sp.set_defaults(func=cmd_tune)

    sp = sub.add_parser(
        "settings", help="read/write/diff the AM32 EEPROM settings page")
    sp.add_argument("action", choices=["read", "write", "diff"])
    add_common(sp)
    sp.add_argument("--bin", help="192-byte settings blob (read: save to; "
                                  "write/diff: source)")
    sp.add_argument("--sim", action="store_true",
                    help="in-process simulated page (offline)")
    sp.set_defaults(func=cmd_settings)

    sp = sub.add_parser("ci", help="build+flash+run+analyze+gate (full pipeline)")
    add_common(sp)
    add_preflight(sp)
    sp.add_argument("--profile", default="ci_smoke")
    sp.add_argument("--baseline")
    sp.add_argument("--sim", action="store_true")
    sp.add_argument("--arm-sdk-prefix")
    sp.add_argument("--out")
    sp.add_argument("--no-plots", action="store_true")
    sp.set_defaults(func=cmd_ci)

    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
