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


def _find(obj_dir: Path, target: str, ext: str) -> Path | None:
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
    elf, binf, hexf = (_find(obj, target, e) for e in ("elf", "bin", "hex"))
    if elf is None or binf is None:
        raise RuntimeError(f"build produced no artifacts for {target} in {obj}")
    return BuildArtifacts(elf=elf, bin=binf, hex=hexf)
