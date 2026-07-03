"""Tests for ELF symbol + DWARF struct-layout extraction."""
import struct

import pytest

pytest.importorskip("elftools")

from hwci import elf, perf  # noqa: E402


def test_find_symbol(host_perf_elf):
    sym = elf.find_symbol(host_perf_elf, "hwci_perf")
    assert sym.size == perf.SIZE == 80


def test_dwarf_layout_matches_canonical(host_perf_elf):
    members = {m.name: m for m in elf.struct_layout(host_perf_elf, "hwci_perf_s")}
    offset = 0
    for name, code in perf.FIELDS:
        size = struct.calcsize(code)
        if not name.startswith("_pad"):
            assert name in members, f"{name} missing from DWARF"
            assert members[name].offset == offset, (
                f"{name}: DWARF off {members[name].offset} != canonical {offset}")
            assert members[name].size == size, (
                f"{name}: DWARF size {members[name].size} != canonical {size}")
            assert members[name].signed == (code in ("b", "h", "i")), (
                f"{name}: signedness mismatch")
        offset += size
    assert offset == perf.SIZE


def test_missing_symbol_raises(host_perf_elf):
    with pytest.raises(elf.ElfError):
        elf.find_symbol(host_perf_elf, "definitely_not_a_symbol")
