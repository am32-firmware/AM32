"""Read and decode the firmware ``hwci_perf`` struct via a Debugger backend."""
from __future__ import annotations

import time
import warnings

from . import elf, perf
from .debugger.base import Debugger, DebuggerError

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
        # The ELF's struct size identifies the firmware's layout version: an
        # A/B session flashes old firmware whose struct predates newer fields,
        # and the harness must keep reading it (perf.decode is version-aware).
        known = set(perf.SIZE_BY_VERSION.values())
        if sym.size and sym.size not in known:
            raise perf.PerfDecodeError(
                f"hwci_perf ELF size {sym.size} matches no known version "
                f"({sorted(known)}); rebuild firmware or update hwci/hwci/perf.py")
        self._read_size = sym.size or perf.SIZE
        if check_layout:
            self._check_layout()

    def _check_layout(self) -> None:
        try:
            members = {m.name: m for m in elf.struct_layout(self.elf_path, STRUCT_TAG)}
        except elf.StructNotFoundError as e:
            # DWARF exists but the struct DIE is gone (renamed tag, LTO/-g1
            # stripping types): decoding would proceed on faith exactly when
            # the cross-check matters most. Fail hard.
            raise perf.PerfDecodeError(
                f"cannot cross-check hwci_perf layout: {e}") from e
        except elf.ElfError as e:
            warnings.warn(f"hwci_perf layout cross-check skipped ({e}); "
                          "relying on the canonical layout in hwci/hwci/perf.py")
            return
        # Pick the canonical layout matching the firmware's vintage by probing
        # for a v2-only member, then verify every field of THAT layout.
        fields = perf.FIELDS if "zc_count" in members else perf.FIELDS_V1
        off = 0
        import struct as _struct
        for name, code in fields:
            size = _struct.calcsize(code)
            if not name.startswith("_pad"):
                m = members.get(name)
                if m is None or m.offset != off or m.size != size:
                    raise perf.PerfDecodeError(
                        f"firmware/host layout mismatch at {name!r}: "
                        f"ELF={m} canonical(off={off},size={size})")
            off += size

    def read(self) -> perf.PerfSample:
        data = self.dbg.read_memory(self.address, self._read_size)
        return perf.decode(data, host_monotonic=time.monotonic())

    def reset_stats(self, *, verify: bool = True, timeout_s: float = 1.0) -> None:
        """Ask the firmware to clear its sticky min/max accumulators.

        With ``verify`` (the default) this polls until the firmware consumes
        the command (it clears ``host_cmd`` within ~64 main-loop iterations).
        A reset that silently never lands would let the previous run's maxima
        pollute this run's gated metrics.
        """
        cmd_addr = self.address + perf.HOST_CMD_OFFSET
        self.dbg.write_u32(cmd_addr, perf.CMD_RESET_STATS)
        if not verify:
            return
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            val = int.from_bytes(self.dbg.read_memory(cmd_addr, 4), "little")
            if val == perf.CMD_NONE:
                return
            time.sleep(0.01)
        raise DebuggerError(
            "firmware did not acknowledge RESET_STATS within "
            f"{timeout_s}s (is the target running HWCI_PERF firmware?)")
