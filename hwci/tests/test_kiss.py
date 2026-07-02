"""Tests for the KISS ESC telemetry decoder."""
import struct

from hwci.esc_telem import crc8, encode_frame, parse_frame
from hwci.esc_telem.kiss import KissStream


def _make_frame(temp, volt_cv, cur_ca, cons, erpm100):
    body = struct.pack(">bHHHH", temp, volt_cv, cur_ca, cons, erpm100)
    return body + bytes([crc8(body)])


def test_crc8_known_vector():
    # CRC of an all-zero 9-byte body is 0 for this polynomial/init.
    assert crc8(b"\x00" * 9) == 0


def test_crc8_table_matches_bitwise_reference():
    def crc8_bitwise(data):
        crc = 0
        for byte in data:
            crc ^= byte
            for _ in range(8):
                crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
        return crc
    for vec in (b"\x01", b"\xff" * 9, bytes(range(9)), b"\xa5\x5a\x00\x42"):
        assert crc8(vec) == crc8_bitwise(vec)


def test_encode_parse_roundtrip():
    blob = encode_frame(temperature_c=42, voltage_cv=1650, current_ca=1234,
                        consumption_mah=56, erpm100=200)
    f = parse_frame(blob)
    assert f.crc_ok
    assert (f.temperature_c, f.voltage_v, f.current_a,
            f.consumption_mah, f.e_rpm) == (42, 16.5, 12.34, 56, 20000)


def test_parse_valid_frame():
    frame = _make_frame(temp=42, volt_cv=1650, cur_ca=1234, cons=56, erpm100=200)
    f = parse_frame(frame)
    assert f.crc_ok
    assert f.temperature_c == 42
    assert f.voltage_v == 1650 / 100.0
    assert f.current_a == 1234 / 100.0
    assert f.consumption_mah == 56
    assert f.e_rpm == 20000


def test_bad_crc_flagged():
    frame = bytearray(_make_frame(20, 1600, 100, 1, 100))
    frame[-1] ^= 0xFF
    f = parse_frame(bytes(frame))
    assert not f.crc_ok


def test_stream_resync_after_garbage():
    good = _make_frame(25, 1600, 500, 10, 150)
    stream = KissStream()
    # Prepend 3 junk bytes; the framer must slide past them and recover.
    frames = list(stream.feed(b"\x11\x22\x33" + good))
    assert len(frames) == 1
    assert frames[0].temperature_c == 25
    assert frames[0].e_rpm == 15000


def test_stream_multiple_frames():
    f1 = _make_frame(10, 1500, 100, 1, 50)
    f2 = _make_frame(11, 1510, 110, 2, 60)
    stream = KissStream()
    out = list(stream.feed(f1 + f2))
    assert [f.temperature_c for f in out] == [10, 11]


def test_stream_partial_then_complete():
    f = _make_frame(30, 1700, 800, 20, 250)
    stream = KissStream()
    assert list(stream.feed(f[:6])) == []   # nothing yet
    out = list(stream.feed(f[6:]))
    assert len(out) == 1
    assert out[0].e_rpm == 25000
