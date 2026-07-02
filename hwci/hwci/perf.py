"""Decoder for the firmware ``hwci_perf`` instrumentation struct.

This mirrors, byte-for-byte, the C struct defined in ``Inc/hwci_perf.h``. The
canonical layout below is the source of truth for offline use and tests; on the
real rig :mod:`hwci.elf` can additionally derive the layout from the ELF's DWARF
debug info and cross-check it against this table, so a firmware layout change is
caught instead of silently misread.

All fields are little-endian and naturally aligned (the struct is deliberately
not packed because Cortex-M0 cannot do unaligned word access).
"""
from __future__ import annotations

import struct
from dataclasses import dataclass

# ASCII "HWC1" little-endian == 0x31435748
MAGIC = 0x31435748
VERSION = 1

# (name, struct_code).  Order and codes must match Inc/hwci_perf.h exactly.
# Pad fields are decoded then dropped from the public dict.
FIELDS: list[tuple[str, str]] = [
    ("magic", "I"),
    ("version", "H"),
    ("size", "H"),
    ("ctrl_exec_us_last", "H"),
    ("ctrl_exec_us_max", "H"),
    ("ctrl_period_us_last", "H"),
    ("ctrl_period_us_max", "H"),
    ("ctrl_period_us_min", "H"),
    ("main_loop_us_last", "H"),
    ("main_loop_us_max", "H"),
    ("input", "H"),
    ("duty_cycle", "H"),
    ("e_rpm", "H"),
    ("voltage_cv", "H"),
    ("current_ca", "h"),
    ("temperature_c", "h"),
    ("bemf_timeout_state", "B"),
    ("armed", "B"),
    ("running", "B"),
    ("_pad0", "B"),
    ("_pad1", "H"),
    ("loop_iters", "I"),
    ("zero_cross_count", "I"),
    ("commutation_interval", "I"),
    ("commutation_interval_max", "I"),
    ("update_count", "I"),
    ("host_cmd", "I"),
]

_FORMAT = "<" + "".join(code for _, code in FIELDS)
SIZE = struct.calcsize(_FORMAT)  # 64 bytes
_NAMES = [name for name, _ in FIELDS]
_PUBLIC = [name for name in _NAMES if not name.startswith("_pad")]

# Host command values understood by the firmware (see Inc/hwci_perf.h).
CMD_NONE = 0
CMD_RESET_STATS = 0xA5

# Byte offset of host_cmd within the struct, for the debugger write-back.
HOST_CMD_OFFSET = sum(struct.calcsize(code) for name, code in FIELDS
                      if _NAMES.index(name) < _NAMES.index("host_cmd"))


class PerfDecodeError(ValueError):
    """Raised when a buffer cannot be decoded as a valid hwci_perf struct."""


@dataclass
class PerfSample:
    """One decoded snapshot of the firmware instrumentation struct."""

    raw: dict
    host_monotonic: float | None = None  # host time.monotonic() at read, if set

    # --- convenience accessors with engineering units -------------------
    @property
    def ctrl_exec_us_max(self) -> int:
        return self.raw["ctrl_exec_us_max"]

    @property
    def ctrl_period_us_max(self) -> int:
        return self.raw["ctrl_period_us_max"]

    @property
    def ctrl_period_us_min(self) -> int:
        return self.raw["ctrl_period_us_min"]

    @property
    def main_loop_us_max(self) -> int:
        return self.raw["main_loop_us_max"]

    @property
    def loop_iters(self) -> int:
        return self.raw["loop_iters"]

    @property
    def zero_cross_count(self) -> int:
        return self.raw["zero_cross_count"]

    @property
    def voltage(self) -> float:
        """Battery voltage in volts (firmware reports centivolts)."""
        return self.raw["voltage_cv"] / 100.0

    @property
    def current(self) -> float:
        """Phase current in amps (firmware reports centiamps)."""
        return self.raw["current_ca"] / 100.0

    @property
    def e_rpm(self) -> int:
        """Electrical RPM (firmware reports eRPM/100)."""
        return self.raw["e_rpm"] * 100

    def mech_rpm(self, pole_pairs: int) -> float:
        """Mechanical RPM derived from eRPM and motor pole pairs."""
        return self.e_rpm / pole_pairs if pole_pairs else 0.0


def decode(data: bytes, *, host_monotonic: float | None = None,
           validate: bool = True) -> PerfSample:
    """Decode ``data`` (>= :data:`SIZE` bytes) into a :class:`PerfSample`."""
    if len(data) < SIZE:
        raise PerfDecodeError(f"need {SIZE} bytes, got {len(data)}")
    values = struct.unpack(_FORMAT, data[:SIZE])
    raw = dict(zip(_NAMES, values))
    if validate:
        if raw["magic"] != MAGIC:
            raise PerfDecodeError(
                f"bad magic 0x{raw['magic']:08x} (expected 0x{MAGIC:08x}); "
                "is the firmware built with HWCI_PERF=1?")
        if raw["version"] != VERSION:
            raise PerfDecodeError(
                f"struct version {raw['version']} != host-expected {VERSION}; "
                "rebuild or update hwci/hwci/perf.py")
        if raw["size"] != SIZE:
            raise PerfDecodeError(
                f"struct size {raw['size']} != host-expected {SIZE}; "
                "firmware/host layout drift - rebuild or update hwci/hwci/perf.py")
    public = {k: raw[k] for k in _PUBLIC}
    return PerfSample(raw=public, host_monotonic=host_monotonic)


def encode(raw: dict) -> bytes:
    """Inverse of :func:`decode` (used by the simulator and tests)."""
    full = {name: 0 for name in _NAMES}
    full.update({"magic": MAGIC, "version": VERSION, "size": SIZE})
    full.update(raw)
    return struct.pack(_FORMAT, *(full[name] for name in _NAMES))
