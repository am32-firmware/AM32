"""Rig and test-profile configuration (YAML-backed)."""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import yaml

from .flightstand.base import SafetyLimits

PROFILES_DIR = Path(__file__).parent / "profiles"
DEFAULT_TARGET = "ARK_4IN1_F051"


@dataclass
class Segment:
    """One phase of a test profile."""
    label: str
    throttle: float           # target throttle, 0..1
    duration_s: float
    ramp: bool = False        # ramp linearly from the previous throttle
    steady: bool = False      # use this segment for steady-state metrics


@dataclass
class Profile:
    name: str
    description: str = ""
    sample_rate_hz: float = 100.0
    arm_settle_s: float = 2.0
    segments: list[Segment] = field(default_factory=list)
    safety: SafetyLimits = field(default_factory=SafetyLimits)
    # analysis knobs
    steady_tail_fraction: float = 0.5   # use last half of a steady segment
    demag_commutation_spike: float = 3.0  # x median commutation interval
    demag_rpm_drop_fraction: float = 0.25  # rpm fell >25% while throttle high
    pole_pairs: int = 7

    @property
    def duration_s(self) -> float:
        return sum(s.duration_s for s in self.segments)


def _safety_from(d: dict) -> SafetyLimits:
    d = d or {}
    return SafetyLimits(
        max_thrust_n=d.get("max_thrust_n"),
        max_current_a=d.get("max_current_a"),
        max_rpm=d.get("max_rpm"),
        max_voltage_v=d.get("max_voltage_v"),
        max_motor_temp_c=d.get("max_motor_temp_c"),
    )


def profile_from_dict(d: dict) -> Profile:
    segs = [Segment(label=s.get("label", f"seg{i}"),
                    throttle=float(s["throttle"]),
                    duration_s=float(s["duration_s"]),
                    ramp=bool(s.get("ramp", False)),
                    steady=bool(s.get("steady", False)))
            for i, s in enumerate(d.get("segments", []))]
    return Profile(
        name=d["name"],
        description=d.get("description", ""),
        sample_rate_hz=float(d.get("sample_rate_hz", 100.0)),
        arm_settle_s=float(d.get("arm_settle_s", 2.0)),
        segments=segs,
        safety=_safety_from(d.get("safety")),
        steady_tail_fraction=float(d.get("steady_tail_fraction", 0.5)),
        demag_commutation_spike=float(d.get("demag_commutation_spike", 3.0)),
        demag_rpm_drop_fraction=float(d.get("demag_rpm_drop_fraction", 0.25)),
        pole_pairs=int(d.get("pole_pairs", 7)),
    )


def load_profile(name_or_path: str) -> Profile:
    """Load a profile by built-in name (in profiles/) or by file path."""
    p = Path(name_or_path)
    if not p.exists():
        cand = PROFILES_DIR / f"{name_or_path}.yaml"
        if cand.exists():
            p = cand
        else:
            raise FileNotFoundError(
                f"profile {name_or_path!r} not found "
                f"(looked in {PROFILES_DIR} and as a path)")
    return profile_from_dict(yaml.safe_load(p.read_text()))


def list_profiles() -> list[str]:
    return sorted(p.stem for p in PROFILES_DIR.glob("*.yaml"))


@dataclass
class RigConfig:
    """How the host reaches the hardware. Defaults are the offline simulator."""
    target: str = DEFAULT_TARGET
    repo_root: str = str(Path(__file__).resolve().parents[2])
    obj_dir: str | None = None              # default <repo_root>/obj
    elf_path: str | None = None             # default: glob obj for the target

    debugger_backend: str = "sim"           # "openocd" | "sim"
    openocd_configs: list[str] = field(
        default_factory=lambda: ["interface/stlink.cfg", "target/stm32f0x.cfg"])
    openocd_search_dirs: list[str] = field(default_factory=list)
    openocd_bin: str = "openocd"
    app_load_addr: int = 0x08001000

    telem_backend: str = "sim"              # "serial" | "sim" | "none"
    telem_port: str = "/dev/ttyUSB0"
    telem_baud: int = 115200

    throttle_backend: str = "sim"           # "flightstand" | "external" | "sim"
    throttle_port: str = "/dev/ttyACM0"
    throttle_baud: int = 115200

    stand_backend: str = "sim"              # "grpc" | "sim"
    stand_host: str = "127.0.0.1"
    stand_port: int = 50051
    stand_signals: dict = field(default_factory=dict)

    motor_name: str = "sim-motor"
    pole_pairs: int = 7
    prop: str = "sim-prop"

    def resolved_obj_dir(self) -> Path:
        return Path(self.obj_dir) if self.obj_dir else Path(self.repo_root) / "obj"

    def resolved_elf(self) -> Path | None:
        if self.elf_path:
            return Path(self.elf_path)
        hits = sorted(self.resolved_obj_dir().glob(f"AM32_{self.target}_*.elf"))
        return hits[-1] if hits else None


def load_rig(path: str | None) -> RigConfig:
    if not path:
        return RigConfig()
    data = yaml.safe_load(Path(path).read_text()) or {}
    cfg = RigConfig()
    for key, value in data.items():
        if hasattr(cfg, key):
            setattr(cfg, key, value)
    return cfg
