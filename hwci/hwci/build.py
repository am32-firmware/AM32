"""Firmware build helpers (wrap the AM32 Makefile)."""
from __future__ import annotations

import subprocess
from dataclasses import dataclass
from pathlib import Path


@dataclass
class BuildArtifacts:
    elf: Path
    bin: Path
    hex: Path


def find_artifact(obj_dir: Path, target: str, ext: str) -> Path | None:
    """Newest ``obj/AM32_<target>_*.<ext>`` build artifact, or None.

    The single place that knows the Makefile's artifact naming; RigConfig's
    ELF resolution reuses it so the flashed binary and the parsed ELF can
    never be picked by two different rules.
    """
    hits = sorted(obj_dir.glob(f"AM32_{target}_*.{ext}"))
    return hits[-1] if hits else None


def build_firmware(repo_root: str | Path, target: str, *,
                   hwci_perf: bool = True, jobs: int = 4,
                   arm_sdk_prefix: str | None = None,
                   extra_make_args: list[str] | None = None) -> BuildArtifacts:
    """Run ``make <target>`` (with HWCI_PERF=1 by default) and return artifacts."""
    repo_root = Path(repo_root)
    cmd = ["make", target, f"-j{jobs}"]
    if hwci_perf:
        cmd.append("HWCI_PERF=1")
    if arm_sdk_prefix:
        cmd.append(f"ARM_SDK_PREFIX={arm_sdk_prefix}")
    if extra_make_args:
        cmd += extra_make_args
    proc = subprocess.run(cmd, cwd=repo_root, capture_output=True, text=True)
    if proc.returncode != 0:
        raise RuntimeError(
            f"firmware build failed (rc={proc.returncode}):\n"
            f"{proc.stdout[-2000:]}\n{proc.stderr[-2000:]}")
    obj = repo_root / "obj"
    elf, binf, hexf = (find_artifact(obj, target, e) for e in ("elf", "bin", "hex"))
    if elf is None or binf is None:
        raise RuntimeError(f"build produced no artifacts for {target} in {obj}")
    if hwci_perf:
        # The Makefile does not encode HWCI_PERF in object paths, so a prior
        # non-instrumented build leaves up-to-date objects and this "build"
        # silently packages firmware WITHOUT the perf struct (caught on the
        # bench: flashed an ELF with no hwci_perf symbol). Verify, don't hope.
        try:
            from . import elf as elfmod
            elfmod.find_symbol(str(elf), "hwci_perf")
        except ImportError:
            pass  # no pyelftools: PerfReader will catch it at run time
        except Exception as e:
            raise RuntimeError(
                f"{elf} lacks the hwci_perf symbol - stale non-instrumented "
                f"objects in obj/ (run 'make clean' and rebuild): {e}") from e
    return BuildArtifacts(elf=elf, bin=binf, hex=hexf)
