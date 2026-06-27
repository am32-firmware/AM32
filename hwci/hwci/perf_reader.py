"""Read and decode the firmware ``hwci_perf`` struct via a Debugger backend."""
from __future__ import annotations

import time
import warnings

from . import elf, perf
from .debugger.base import Debugger

SYMBOL = "hwci_perf"
STRUCT_TAG = "hwci_perf_s"


class PerfReader:
    """Locate ``hwci_perf`` once from the ELF, then sample it on demand.

    On construction the symbol address is read from ``elf_path``. If the ELF
    carries DWARF (AM32 builds with ``-g``), the on-target layout is cross-
    checked against :data:`hwci.perf.FIELDS` so a firmware/host layout drift is
    caught loudly instead of silently misread.
    """

    def __init__(self, dbg: Debugger, elf_path: str, *, check_layout: bool = True):
        self.dbg = dbg
        self.elf_path = elf_path
        sym = elf.find_symbol(elf_path, SYMBOL)
        self.address = sym.address
        if sym.size and sym.size != perf.SIZE:
            warnings.warn(
                f"hwci_perf ELF size {sym.size} != host {perf.SIZE}; "
                "rebuild firmware or update hwci/hwci/perf.py")
        if check_layout:
            self._check_layout()

    def _check_layout(self) -> None:
        try:
            members = {m.name: m for m in elf.struct_layout(self.elf_path, STRUCT_TAG)}
        except elf.ElfError:
            return  # no DWARF -> rely on the canonical layout
        off = 0
        import struct as _struct
        for name, code in perf.FIELDS:
            size = _struct.calcsize(code)
            if not name.startswith("_pad"):
                m = members.get(name)
                if m is None or m.offset != off or m.size != size:
                    raise perf.PerfDecodeError(
                        f"firmware/host layout mismatch at {name!r}: "
                        f"ELF={m} canonical(off={off},size={size})")
            off += size

    def read(self) -> perf.PerfSample:
        data = self.dbg.read_memory(self.address, perf.SIZE)
        return perf.decode(data, host_monotonic=time.monotonic())

    def reset_stats(self) -> None:
        """Ask the firmware to clear its sticky min/max accumulators."""
        self.dbg.write_u32(self.address + perf.HOST_CMD_OFFSET, perf.CMD_RESET_STATS)
