"""Tests for PerfReader against the in-memory MockDebugger."""
import pytest

pytest.importorskip("elftools")

from hwci import perf  # noqa: E402
from hwci.debugger.base import MockDebugger  # noqa: E402
from hwci.perf_reader import PerfReader  # noqa: E402


def _reader_with_mock(elf_path):
    # Find the symbol address from the ELF, then map the mock memory there.
    from hwci import elf as elfmod
    addr = elfmod.find_symbol(elf_path, "hwci_perf").address
    dbg = MockDebugger(base=addr, size=perf.SIZE + 256)
    reader = PerfReader(dbg, elf_path)
    return reader, dbg


def test_read_decodes_seeded_struct(host_perf_elf):
    reader, dbg = _reader_with_mock(host_perf_elf)
    dbg.poke(reader.address, perf.encode({
        "ctrl_exec_us_max": 21,
        "loop_iters": 5000,
        "voltage_cv": 1580,
    }))
    sample = reader.read()
    assert sample.ctrl_exec_us_max == 21
    assert sample.loop_iters == 5000
    assert sample.voltage == pytest.approx(15.80)
    assert sample.host_monotonic is not None


def test_reset_stats_writes_command(host_perf_elf):
    reader, dbg = _reader_with_mock(host_perf_elf)
    dbg.poke(reader.address, perf.encode({}))
    reader.reset_stats(verify=False)
    word = dbg.read_memory(reader.address + perf.HOST_CMD_OFFSET, 4)
    assert int.from_bytes(word, "little") == perf.CMD_RESET_STATS


class _FirmwareLikeMock(MockDebugger):
    """Consumes host_cmd immediately, like the firmware's apply_cmd."""

    def write_u32(self, addr: int, value: int) -> None:
        super().write_u32(addr, 0)  # command applied and cleared


def test_reset_stats_verify_passes_when_consumed(host_perf_elf):
    from hwci import elf as elfmod
    addr = elfmod.find_symbol(host_perf_elf, "hwci_perf").address
    dbg = _FirmwareLikeMock(base=addr, size=perf.SIZE + 256)
    reader = PerfReader(dbg, host_perf_elf)
    dbg.poke(reader.address, perf.encode({}))
    reader.reset_stats()  # must not raise


def test_reset_stats_verify_times_out_when_not_consumed(host_perf_elf):
    from hwci.debugger.base import DebuggerError
    reader, dbg = _reader_with_mock(host_perf_elf)
    dbg.poke(reader.address, perf.encode({}))
    with pytest.raises(DebuggerError):
        reader.reset_stats(timeout_s=0.05)


def test_layout_check_passes_for_real_header(host_perf_elf):
    # Should not raise: DWARF layout matches canonical perf.FIELDS.
    reader, _ = _reader_with_mock(host_perf_elf)
    assert reader.address > 0


def test_v1_firmware_reads_and_resets(host_perf_elf_v1):
    # The A side of an A/B session runs old firmware with the 64-byte v1
    # struct: the reader must size its SWD read from the ELF, decode without
    # the zc_* fields, pass the DWARF layout cross-check against the v1
    # canonical table, and land RESET_STATS at the version-stable offset 60.
    reader, dbg = _reader_with_mock(host_perf_elf_v1)
    assert reader._read_size == perf.SIZE_BY_VERSION[1]
    dbg.poke(reader.address, perf.encode(
        {"ctrl_exec_us_max": 33, "loop_iters": 999}, version=1))
    sample = reader.read()
    assert sample.ctrl_exec_us_max == 33
    assert sample.loop_iters == 999
    assert "zc_count" not in sample.raw
    reader.reset_stats(verify=False)
    word = dbg.read_memory(reader.address + perf.HOST_CMD_OFFSET, 4)
    assert int.from_bytes(word, "little") == perf.CMD_RESET_STATS


def test_missing_struct_die_fails_hard(host_perf_elf, monkeypatch):
    # DWARF present but the struct tag gone (rename/LTO): decoding on faith is
    # exactly the drift the cross-check exists to catch -> must raise.
    import hwci.perf_reader as pr
    monkeypatch.setattr(pr, "STRUCT_TAG", "definitely_not_a_struct")
    from hwci import elf as elfmod
    addr = elfmod.find_symbol(host_perf_elf, "hwci_perf").address
    dbg = MockDebugger(base=addr, size=perf.SIZE + 256)
    with pytest.raises(perf.PerfDecodeError):
        PerfReader(dbg, host_perf_elf)
