"""Decoder for the firmware ``hwci_perf`` instrumentation struct.

This mirrors, byte-for-byte, the C struct defined in ``Inc/hwci_perf.h``. The
canonical layouts below are the source of truth for offline use and tests; on
the real rig :mod:`hwci.elf` can additionally derive the layout from the ELF's
DWARF debug info and cross-check it against this table, so a firmware layout
change is caught instead of silently misread.

Struct versions: the host must keep decoding EVERY version it has ever known,
not just the newest - an A/B bench session flashes old firmware whose struct
predates newer fields (e.g. v1 lacks the zero-cross jitter block), and the
harness must still read it. New fields are appended after ``host_cmd`` so its
offset never moves between versions and RESET_STATS works on any vintage.

All fields are little-endian and naturally aligned (the struct is deliberately
not packed because Cortex-M0 cannot do unaligned word access).
"""
from __future__ import annotations

import struct
from dataclasses import dataclass

# ASCII "HWC1" little-endian == 0x31435748
MAGIC = 0x31435748
VERSION = 2

# (name, struct_code).  Order and codes must match Inc/hwci_perf.h exactly.
# Pad fields are decoded then dropped from the public dict.
FIELDS_V1: list[tuple[str, str]] = [
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

# v2 appends the zero-cross jitter block (see HWCI_PERF_ZC in Inc/hwci_perf.h).
FIELDS: list[tuple[str, str]] = FIELDS_V1 + [
    ("zc_count", "I"),
    ("zc_jitter_sum", "I"),
    ("zc_interval_sum", "I"),
    ("zc_jitter_max", "H"),
    ("_pad2", "H"),
]

FIELDS_BY_VERSION: dict[int, list[tuple[str, str]]] = {1: FIELDS_V1, 2: FIELDS}


def _format(fields: list[tuple[str, str]]) -> str:
    return "<" + "".join(code for _, code in fields)


_FORMAT_BY_VERSION = {v: _format(f) for v, f in FIELDS_BY_VERSION.items()}
SIZE_BY_VERSION = {v: struct.calcsize(fmt) for v, fmt in _FORMAT_BY_VERSION.items()}

_FORMAT = _FORMAT_BY_VERSION[VERSION]
SIZE = SIZE_BY_VERSION[VERSION]  # 80 bytes (v1: 64)
_NAMES = [name for name, _ in FIELDS]

# magic + version + size header, enough to pick the right layout for the rest.
_HEADER = struct.Struct("<IHH")

# Host command values understood by the firmware (see Inc/hwci_perf.h).
CMD_NONE = 0
CMD_RESET_STATS = 0xA5

# Byte offset of host_cmd within the struct, for the debugger write-back.
# Identical in every version by construction (new fields append after it);
# the assert keeps that invariant honest if a future layout forgets.
HOST_CMD_OFFSET = sum(struct.calcsize(code) for name, code in FIELDS
                      if _NAMES.index(name) < _NAMES.index("host_cmd"))
assert all(
    sum(struct.calcsize(c) for n, c in flds[:next(
        i for i, (n, _) in enumerate(flds) if n == "host_cmd")]) == HOST_CMD_OFFSET
    for flds in FIELDS_BY_VERSION.values()), "host_cmd offset drifted between versions"


class PerfDecodeError(ValueError):
    """Raised when a buffer cannot be decoded as a valid hwci_perf struct."""


@dataclass
class PerfSample:
    """One decoded snapshot of the firmware instrumentation struct.

    ``raw`` holds only the fields the sampled firmware actually has: a v1
    sample has no ``zc_*`` keys. Downstream consumers use ``raw.get()``.
    """

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
    """Decode ``data`` into a :class:`PerfSample`.

    The struct version in the header selects the layout, so buffers from any
    known firmware vintage decode correctly. ``data`` may be longer than the
    selected layout (a v2-sized read of a v1 target); the excess is ignored.
    """
    if len(data) < _HEADER.size:
        raise PerfDecodeError(f"need at least {_HEADER.size} bytes, got {len(data)}")
    magic, version, size = _HEADER.unpack_from(data)
    if validate:
        if magic != MAGIC:
            raise PerfDecodeError(
                f"bad magic 0x{magic:08x} (expected 0x{MAGIC:08x}); "
                "is the firmware built with HWCI_PERF=1?")
        if version not in FIELDS_BY_VERSION:
            raise PerfDecodeError(
                f"struct version {version} unknown to host "
                f"(knows {sorted(FIELDS_BY_VERSION)}); update hwci/hwci/perf.py")
    fields = FIELDS_BY_VERSION.get(version) or FIELDS
    ver = version if version in FIELDS_BY_VERSION else VERSION
    fmt, expected = _FORMAT_BY_VERSION[ver], SIZE_BY_VERSION[ver]
    if len(data) < expected:
        raise PerfDecodeError(
            f"v{ver} struct needs {expected} bytes, got {len(data)}")
    names = [name for name, _ in fields]
    raw = dict(zip(names, struct.unpack(fmt, data[:expected])))
    if validate and raw["size"] != expected:
        raise PerfDecodeError(
            f"struct size {raw['size']} != host-expected {expected} for "
            f"v{version}; firmware/host layout drift - rebuild or update "
            "hwci/hwci/perf.py")
    public = {k: v for k, v in raw.items() if not k.startswith("_pad")}
    return PerfSample(raw=public, host_monotonic=host_monotonic)


def encode(raw: dict, version: int = VERSION) -> bytes:
    """Inverse of :func:`decode` (used by the simulator and tests)."""
    fields = FIELDS_BY_VERSION[version]
    full = {name: 0 for name, _ in fields}
    full.update({"magic": MAGIC, "version": version,
                 "size": SIZE_BY_VERSION[version]})
    full.update({k: v for k, v in raw.items() if k in full})
    return struct.pack(_FORMAT_BY_VERSION[version],
                       *(full[name] for name, _ in fields))
