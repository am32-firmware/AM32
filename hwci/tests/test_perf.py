"""Tests for the hwci_perf struct decoder."""
import struct

import pytest

from hwci import perf


def test_layout_sizes():
    assert perf.SIZE_BY_VERSION[1] == 64
    assert perf.SIZE == perf.SIZE_BY_VERSION[2] == 80


def test_host_cmd_offset_matches_layout():
    # host_cmd sits at 60 in EVERY version (v2+ fields append after it), so
    # the reset-command write lands regardless of the flashed vintage.
    assert perf.HOST_CMD_OFFSET == 60


def test_roundtrip_encode_decode():
    sample = {
        "ctrl_exec_us_max": 18,
        "ctrl_period_us_min": 49,
        "ctrl_period_us_max": 53,
        "main_loop_us_max": 12,
        "loop_iters": 123456,
        "zero_cross_count": 9001,
        "voltage_cv": 1612,     # 16.12 V
        "current_ca": 2550,     # 25.50 A
        "e_rpm": 321,           # 32100 eRPM
        "temperature_c": 47,
        "armed": 1,
        "running": 1,
        "commutation_interval": 110,
    }
    blob = perf.encode(sample)
    assert len(blob) == perf.SIZE
    decoded = perf.decode(blob)
    assert decoded.ctrl_exec_us_max == 18
    assert decoded.ctrl_period_us_min == 49
    assert decoded.loop_iters == 123456
    assert decoded.voltage == pytest.approx(16.12)
    assert decoded.current == pytest.approx(25.50)
    assert decoded.e_rpm == 32100


def test_roundtrip_zc_jitter_fields():
    blob = perf.encode({
        "zc_count": 700000,
        "zc_jitter_sum": 123456789,
        "zc_interval_sum": 4000000000,   # near the u32 wrap
        "zc_jitter_max": 41,
    })
    r = perf.decode(blob).raw
    assert r["zc_count"] == 700000
    assert r["zc_jitter_sum"] == 123456789
    assert r["zc_interval_sum"] == 4000000000
    assert r["zc_jitter_max"] == 41


def test_v1_roundtrip_has_no_zc_keys():
    # Old firmware (A side of an A/B session) reports a 64-byte v1 struct;
    # it must decode cleanly and simply not carry the jitter fields.
    blob = perf.encode({"ctrl_exec_us_max": 18, "loop_iters": 42}, version=1)
    assert len(blob) == perf.SIZE_BY_VERSION[1]
    s = perf.decode(blob)
    assert s.ctrl_exec_us_max == 18
    assert s.loop_iters == 42
    assert "zc_count" not in s.raw


def test_v1_decodes_from_oversized_read():
    # PerfReader sizes its read from the ELF symbol, but a v2-sized buffer
    # holding a v1 struct (plus trailing junk) must still decode as v1.
    blob = perf.encode({"loop_iters": 7}, version=1) + b"\xa5" * 16
    s = perf.decode(blob)
    assert s.loop_iters == 7
    assert "zc_count" not in s.raw


def test_mech_rpm():
    blob = perf.encode({"e_rpm": 140})  # 14000 eRPM
    s = perf.decode(blob)
    assert s.mech_rpm(7) == pytest.approx(2000.0)


def test_bad_magic_raises():
    blob = bytearray(perf.encode({}))
    struct.pack_into("<I", blob, 0, 0xDEADBEEF)
    with pytest.raises(perf.PerfDecodeError):
        perf.decode(bytes(blob))


def test_version_mismatch_raises():
    blob = bytearray(perf.encode({}))
    struct.pack_into("<H", blob, 4, 99)
    with pytest.raises(perf.PerfDecodeError):
        perf.decode(bytes(blob))


def test_short_buffer_raises():
    with pytest.raises(perf.PerfDecodeError):
        perf.decode(b"\x00" * 10)


def test_negative_current_is_signed():
    blob = perf.encode({"current_ca": -150})
    s = perf.decode(blob)
    assert s.current == pytest.approx(-1.5)
