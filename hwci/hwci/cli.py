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
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from datetime import datetime
from pathlib import Path

from . import baseline as bl
from . import metrics as metricsmod
from . import report as reportmod
from .config import RigConfig, list_profiles, load_profile, load_rig
from .model import RunResult
from .runner import build_live_sources, build_sim_sources, run_profile


def _git_sha(repo_root: str) -> str | None:
    try:
        out = subprocess.run(["git", "rev-parse", "--short", "HEAD"],
                             cwd=repo_root, capture_output=True, text=True)
        return out.stdout.strip() or None
    except Exception:
        return None


def _make_meta(rig: RigConfig, profile_name: str, extra: dict | None = None) -> dict:
    meta = {
        "target": rig.target,
        "profile": profile_name,
        "git_sha": _git_sha(rig.repo_root),
        "motor": rig.motor_name,
        "prop": rig.prop,
        "timestamp": datetime.now().isoformat(timespec="seconds"),
    }
    if extra:
        meta.update(extra)
    return meta


def _use_sim(args, rig: RigConfig) -> bool:
    return bool(getattr(args, "sim", False)) or rig.stand_backend == "sim"


def _default_out(profile_name: str, sim: bool) -> Path:
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    tag = "sim" if sim else "hw"
    return Path("runs") / f"{profile_name}-{tag}-{stamp}"


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
                             meta=_make_meta(rig, profile.name))
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
    rig = load_rig(args.config)
    profile = load_profile(args.profile)
    sim = _use_sim(args, rig)
    sources = (build_sim_sources(rig, profile) if sim
               else build_live_sources(rig, profile))
    try:
        result = run_profile(profile, sources, realtime=args.realtime or not sim,
                             meta=_make_meta(rig, profile.name))
    finally:
        sources.close()
    out = Path(args.out) if args.out else _default_out(profile.name, sim)
    result.save(out)
    print(f"saved {len(result.rows)} samples to {out}")
    if result.meta.get("aborted"):
        print(f"ABORTED: {result.meta['aborted']}", file=sys.stderr)
        return 2
    return 0


def cmd_analyze(args) -> int:
    result = RunResult.load(args.run_dir)
    profile = load_profile(result.meta.get("profile", "ci_smoke"))
    m = metricsmod.compute(result, profile)
    (Path(args.run_dir) / "metrics.json").write_text(json.dumps(m, indent=2))
    print(json.dumps(m["summary"], indent=2))
    return 0


def cmd_baseline_save(args) -> int:
    result = RunResult.load(args.run_dir)
    profile = load_profile(result.meta.get("profile", "ci_smoke"))
    m = metricsmod.compute(result, profile)
    out = args.out or f"baselines/{result.meta.get('target', 'unknown')}.json"
    bl.save_baseline(m, out, meta=result.meta)
    print(f"baseline saved to {out}")
    return 0


def cmd_report(args) -> int:
    result = RunResult.load(args.run_dir)
    profile = load_profile(result.meta.get("profile", "ci_smoke"))
    m = metricsmod.compute(result, profile)
    comparison = None
    if args.baseline:
        comparison = bl.compare(m, bl.load_baseline(args.baseline))
    path = reportmod.write_report(args.run_dir, m, comparison, result.meta,
                                  plots=not args.no_plots)
    print(f"report written to {path}")
    if comparison is not None:
        print("PASS" if comparison["passed"] else "FAIL")
        return 0 if comparison["passed"] else 1
    return 0


def cmd_ci(args) -> int:
    rig = load_rig(args.config)
    profile = load_profile(args.profile)
    sim = _use_sim(args, rig)
    out = Path(args.out) if args.out else _default_out(profile.name, sim)

    if not sim:
        from .build import build_firmware
        from .debugger.openocd import OpenOcdDebugger
        arts = build_firmware(rig.repo_root, rig.target, hwci_perf=True,
                              arm_sdk_prefix=args.arm_sdk_prefix)
        if rig.debugger_backend == "openocd":
            dbg = OpenOcdDebugger(rig.openocd_configs, openocd_bin=rig.openocd_bin,
                                  search_dirs=rig.openocd_search_dirs)
            dbg.flash(str(arts.bin), rig.app_load_addr)
            dbg.close()
        rig.elf_path = str(arts.elf)

    sources = (build_sim_sources(rig, profile) if sim
               else build_live_sources(rig, profile))
    try:
        result = run_profile(profile, sources, realtime=not sim,
                             meta=_make_meta(rig, profile.name))
    finally:
        sources.close()
    result.save(out)

    m = metricsmod.compute(result, profile)
    (out / "metrics.json").write_text(json.dumps(m, indent=2))
    comparison = bl.compare(m, bl.load_baseline(args.baseline)) if args.baseline else None
    reportmod.write_report(out, m, comparison, result.meta, plots=not args.no_plots)

    print(json.dumps(m["summary"], indent=2))
    if result.meta.get("aborted"):
        print(f"ABORTED: {result.meta['aborted']}", file=sys.stderr)
        return 2
    if comparison is not None:
        print("VERDICT:", "PASS" if comparison["passed"] else "FAIL")
        return 0 if comparison["passed"] else 1
    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="hwci", description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)

    def add_common(sp):
        sp.add_argument("--config", help="rig config YAML (default: built-in sim)")

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

    sp = sub.add_parser("ci", help="build+flash+run+analyze+gate (full pipeline)")
    add_common(sp)
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
