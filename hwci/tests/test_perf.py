"""Tests for the hwci_perf struct decoder."""
import struct

import pytest

from hwci import perf


def test_layout_is_64_bytes():
    assert perf.SIZE == 64


def test_host_cmd_offset_matches_layout():
    # host_cmd is the final u32 in the 64-byte struct.
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
