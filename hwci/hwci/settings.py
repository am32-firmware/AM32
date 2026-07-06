"""AM32 EEPROM settings: read, mutate and write the 192-byte settings page.

AM32 keeps its settings in a 192-byte ``EEprom_t`` union (Inc/eeprom.h) stored
in a dedicated 1 KB flash page and read ONCE at boot (``loadEEpromSettings`` in
Src/main.c). Changing a setting therefore never needs a rebuild: write the page
over SWD with the debugger's one-shot ``program ... verify reset exit`` flow
(:meth:`hwci.debugger.openocd.OpenOcdDebugger.flash`) and the reset boots the
firmware into the new settings.

Two safety properties are load-bearing here:

* **Validation refuses out-of-range writes.** The firmware silently falls back
  to defaults on bad values (e.g. ``pwm_frequency`` outside 8..144 keeps the
  compile-time ARR, main.c ``loadEEpromSettings``), so an out-of-range "trial"
  would run the DEFAULT setting while the tuner records it as the trial value -
  a lie in the data. Rejecting the write is the only honest behaviour.
* **Trial blobs are seeded from the device's current page**, mutating only the
  tuned bytes. That preserves the version/identity bytes (offsets 1, 3, 4),
  motor_kv, input_type and servo calibration - so the boot-time
  version-mismatch rewrite in main.c (which saves the page back when
  VERSION_MAJOR/MINOR or EEPROM_VERSION differ) never fires mid-tune, and rig
  calibration is never clobbered.

The live settings address is the firmware global ``eeprom_address``
(Src/main.c): it starts at the target's ``EEPROM_START_ADD`` but the bootloader
devinfo check may rewrite it at boot (0x1f -> 0x08007c00, 0x35 -> 0x0800f800,
0x2b -> 0x0801f800). :func:`resolve_eeprom_address` reads the global off the
running target (flash is memory-mapped, same read path as the perf struct) and
sanity-checks it against the known values.
"""
from __future__ import annotations

import hashlib
import warnings
from dataclasses import dataclass
from pathlib import Path

from . import elf
from .debugger.base import Debugger

EEPROM_SIZE = 192
UNION_TAG = "EEprom_u"
EEPROM_ADDRESS_SYMBOL = "eeprom_address"

# F051 EEPROM_START_ADD (Inc/targets.h) - the ARK 4IN1 value.
DEFAULT_EEPROM_ADDRESS = 0x08007C00
# Every value the firmware's eeprom_address global can legally hold: the
# per-target EEPROM_START_ADD values, which are also exactly the values the
# bootloader-devinfo rewrite in Src/main.c can install.
KNOWN_EEPROM_ADDRESSES = frozenset({0x08007C00, 0x0800F800, 0x0801F800})


class SettingsError(ValueError):
    pass


@dataclass(frozen=True)
class Field:
    """One tunable byte of the EEprom_t settings struct."""
    name: str
    offset: int
    lo: int          # inclusive firmware-valid range
    hi: int
    description: str
    size: int = 1


# Tunable settings, offsets/ranges from Inc/eeprom.h comments and the
# validation code in Src/main.c loadEEpromSettings(). ``lo``/``hi`` are the
# FIRMWARE-accepted ranges; a tune spec may (and should) sweep a narrower
# window - e.g. pwm_frequency is valid 8..144 but sensibly tuned 8..48.
EEPROM_FIELDS: dict[str, Field] = {f.name: f for f in [
    Field("max_ramp", 5, 1, 255,
          "throttle ramp limit, 0.1%/ms steps up to 25%/ms (default 160)"),
    Field("variable_pwm", 21, 0, 2,
          "0=fixed PWM, 1=RPM-scaled within pwm_frequency range, 2=auto range"),
    Field("advance_level", 23, 10, 42,
          "timing advance, value-10 degrees*(48/32) (16 = firmware fallback)"),
    Field("pwm_frequency", 24, 8, 144,
          "PWM frequency in kHz; firmware accepts 8..144, tune 8..48"),
    Field("startup_power", 25, 50, 150,
          "startup duty boost; firmware accepts 50..150"),
    Field("auto_advance", 47, 0, 1,
          "1 = firmware maps advance from duty cycle, ignoring advance_level"),
]}

# Identity/version bytes that a settings write must NEVER change: a mismatch
# against the running firmware's VERSION_MAJOR/MINOR or EEPROM_VERSION makes
# boot rewrite the whole page (main.c), invalidating the trial blob.
READ_ONLY_OFFSETS: dict[int, str] = {
    1: "eeprom_version",
    3: "version.major",
    4: "version.minor",
}

_OFFSET_TO_NAME: dict[int, str] = {
    **{f.offset: f.name for f in EEPROM_FIELDS.values()},
    **READ_ONLY_OFFSETS,
}


def resolve_field(name: str, offset: int | None = None) -> Field:
    """Resolve a tune parameter to a :class:`Field`.

    Known names come from :data:`EEPROM_FIELDS`. An explicit ``offset`` (for
    settings this module doesn't know yet - forward-compat with newer
    firmware) yields an ad-hoc full-range byte field, but never one of the
    read-only identity offsets.
    """
    if name in EEPROM_FIELDS:
        f = EEPROM_FIELDS[name]
        if offset is not None and offset != f.offset:
            raise SettingsError(
                f"parameter {name!r}: explicit offset {offset} contradicts "
                f"the known offset {f.offset}")
        return f
    if offset is None:
        raise SettingsError(
            f"unknown setting {name!r} and no explicit offset given; known: "
            f"{sorted(EEPROM_FIELDS)}")
    if not 0 <= offset < EEPROM_SIZE:
        raise SettingsError(f"parameter {name!r}: offset {offset} outside "
                            f"the {EEPROM_SIZE}-byte settings page")
    if offset in READ_ONLY_OFFSETS:
        raise SettingsError(
            f"parameter {name!r}: offset {offset} is the read-only identity "
            f"byte {READ_ONLY_OFFSETS[offset]!r}")
    return Field(name, offset, 0, 255, "explicit-offset parameter")


class Settings:
    """A 192-byte EEprom page image. Immutable-style: ``apply`` returns a copy."""

    def __init__(self, data: bytes | bytearray):
        if len(data) != EEPROM_SIZE:
            raise SettingsError(
                f"settings blob must be {EEPROM_SIZE} bytes, got {len(data)}")
        self._data = bytearray(data)

    @classmethod
    def from_device(cls, dbg: Debugger, addr: int) -> "Settings":
        """Read the current page off the target (flash is memory-mapped)."""
        return cls(dbg.read_memory(addr, EEPROM_SIZE))

    @classmethod
    def from_bin(cls, path: str | Path) -> "Settings":
        return cls(Path(path).read_bytes())

    def get(self, name: str, offset: int | None = None) -> int:
        return self._data[resolve_field(name, offset).offset]

    def apply(self, overrides: dict[str, int],
              offsets: dict[str, int] | None = None) -> "Settings":
        """Return a copy with ``overrides`` applied, validating every value.

        ``offsets`` optionally maps names to explicit byte offsets (see
        :func:`resolve_field`). Out-of-range values are refused, never
        clamped: the firmware would silently fall back to its default and
        the "trial" would not test what it claims to.
        """
        out = Settings(self._data)
        for name, value in overrides.items():
            f = resolve_field(name, (offsets or {}).get(name))
            if not isinstance(value, int) or isinstance(value, bool):
                raise SettingsError(f"{name}: value {value!r} is not an int")
            if not f.lo <= value <= f.hi:
                raise SettingsError(
                    f"{name}: {value} outside firmware-valid range "
                    f"[{f.lo}, {f.hi}] ({f.description}); the firmware would "
                    "silently ignore it, so the write is refused")
            out._data[f.offset] = value
        return out

    def to_bytes(self) -> bytes:
        return bytes(self._data)

    def to_bin(self, path: str | Path) -> Path:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(self.to_bytes())
        return path

    def sha256(self) -> str:
        return hashlib.sha256(self._data).hexdigest()

    def hex(self) -> str:
        return self._data.hex()

    def diff(self, other: "Settings") -> list[tuple[str, int, int]]:
        """Byte-level diff -> [(name, self_value, other_value)]; bytes with no
        known field name are reported as ``offset:N``."""
        out = []
        for i, (a, b) in enumerate(zip(self._data, other._data)):
            if a != b:
                out.append((_OFFSET_TO_NAME.get(i, f"offset:{i}"), a, b))
        return out

    def describe(self) -> dict[str, int]:
        """Values of every known tunable field (for reports/CLI)."""
        return {name: self._data[f.offset]
                for name, f in sorted(EEPROM_FIELDS.items(),
                                      key=lambda kv: kv[1].offset)}

    def __eq__(self, other) -> bool:
        return isinstance(other, Settings) and self._data == other._data


def default_blob() -> bytes:
    """A plausible AM32 default settings page for the OFFLINE simulator.

    Field values follow the firmware defaults in ``loadEEpromSettings`` /
    the configurator defaults; identity bytes are non-zero so the tuner's
    "seed from device, mutate only tuned bytes" flow is exercised for real.
    Never used on hardware - there the page is always read off the device.
    """
    d = bytearray(EEPROM_SIZE)
    d[0] = 0x01    # reserved/boot byte
    d[1] = 2       # eeprom_version
    d[3] = 2       # version.major
    d[4] = 18      # version.minor
    d[5] = 160     # max_ramp (25%/ms)
    d[6] = 1       # minimum_duty_cycle
    d[8] = 10      # absolute_voltage_cutoff
    d[9] = 100     # current_P
    d[11] = 100    # current_D
    d[20] = 1      # comp_pwm
    d[21] = 1      # variable_pwm
    d[22] = 1      # stuck_rotor_protection
    d[23] = 26     # advance_level (16 degrees)
    d[24] = 24     # pwm_frequency kHz
    d[25] = 100    # startup_power
    d[26] = 55     # motor_kv -> (55*40)+20 = 2220
    d[27] = 14     # motor_poles
    d[30] = 5      # beep_volume
    d[32], d[33], d[34], d[35] = 128, 128, 128, 50   # servo cal
    d[37] = 30     # low_cell_volt_cutoff (3.0v)
    d[43], d[44] = 141, 102   # limits (off)
    d[45] = 5      # sine_mode_power
    d[46] = 1      # input_type
    return bytes(d)


def resolve_eeprom_address(dbg: Debugger, elf_path: str) -> int:
    """Read the live ``eeprom_address`` global off the running target.

    The global's RAM address comes from the ELF symbol table; its VALUE is
    what boot-time devinfo handling left there. Sanity-checked against the
    known per-target page addresses - a garbage value here would make the
    tuner "program" an arbitrary flash sector.
    """
    sym = elf.find_symbol(elf_path, EEPROM_ADDRESS_SYMBOL)
    addr = int.from_bytes(dbg.read_memory(sym.address, 4), "little")
    if addr not in KNOWN_EEPROM_ADDRESSES:
        raise SettingsError(
            f"live eeprom_address reads 0x{addr:08x}, not one of the known "
            f"settings pages {sorted(f'0x{a:08x}' for a in KNOWN_EEPROM_ADDRESSES)}; "
            "refusing to write settings anywhere else")
    return addr


def check_eeprom_layout(elf_path: str) -> None:
    """Cross-check :data:`EEPROM_FIELDS` offsets against the ELF's DWARF.

    Mirrors ``PerfReader._check_layout``: missing DWARF only warns (the
    canonical offsets in this module still apply), but DWARF that lacks the
    ``EEprom_u`` union entirely, or disagrees on an offset, fails hard -
    that is exactly the firmware/host drift this check exists to catch.
    """
    try:
        members = {m.name: m for m in elf.union_layout(elf_path, UNION_TAG)}
    except elf.StructNotFoundError as e:
        raise SettingsError(f"cannot cross-check EEprom layout: {e}") from e
    except elf.ElfError as e:
        warnings.warn(f"EEprom layout cross-check skipped ({e}); relying on "
                      "the canonical offsets in hwci/hwci/settings.py")
        return
    for f in EEPROM_FIELDS.values():
        m = members.get(f.name)
        if m is None or m.offset != f.offset or m.size != f.size:
            raise SettingsError(
                f"firmware/host EEprom layout mismatch at {f.name!r}: "
                f"ELF={m} canonical(off={f.offset},size={f.size}); "
                "update EEPROM_FIELDS in hwci/hwci/settings.py")
