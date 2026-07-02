"""KISS / BLHeli ESC serial telemetry decoding.

AM32 emits a 10-byte KISS telemetry frame on the dedicated telemetry wire
(``USE_SERIAL_TELEMETRY`` is enabled on the ARK 4IN1 target). The frame is the
struct ``kiss_telem_pkt_t`` in ``Inc/kiss_telemetry.h``:

    byte 0      int8   temperature, degrees C
    byte 1..2   u16 BE voltage, centivolts
    byte 3..4   u16 BE current, centiamps
    byte 5..6   u16 BE consumption, mAh
    byte 7..8   u16 BE eRPM / 100
    byte 9      u8     CRC-8 over bytes 0..8

The CRC-8 matches ``get_crc8``/``update_crc8`` in ``Src/functions.c``
(polynomial 0x07, init 0x00). There is no start delimiter, so the streaming
parser resyncs by sliding a 10-byte window until the CRC validates.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Iterator

FRAME_LEN = 10


def _crc8_table() -> bytes:
    table = bytearray(256)
    for byte in range(256):
        crc = byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
        table[byte] = crc
    return bytes(table)


_CRC8_TABLE = _crc8_table()


def crc8(data: bytes) -> int:
    """BLHeli/KISS CRC-8 (poly 0x07, init 0x00), matching Src/functions.c.

    Table-driven: the streaming framer runs this once per byte-slide when
    resynchronising on a noisy line, so the bitwise loop would burn a core.
    """
    crc = 0
    for byte in data:
        crc = _CRC8_TABLE[crc ^ byte]
    return crc


@dataclass
class KissFrame:
    temperature_c: int
    voltage_v: float
    current_a: float
    consumption_mah: int
    e_rpm: int
    crc_ok: bool

    def mech_rpm(self, pole_pairs: int) -> float:
        return self.e_rpm / pole_pairs if pole_pairs else 0.0


def parse_frame(data: bytes) -> KissFrame:
    """Parse exactly one 10-byte frame. ``crc_ok`` flags CRC validity."""
    if len(data) < FRAME_LEN:
        raise ValueError(f"need {FRAME_LEN} bytes, got {len(data)}")
    temp, volt_cv, cur_ca, cons, erpm100, crc = struct.unpack(
        ">bHHHHB", data[:FRAME_LEN])
    return KissFrame(
        temperature_c=temp,
        voltage_v=volt_cv / 100.0,
        current_a=cur_ca / 100.0,
        consumption_mah=cons,
        e_rpm=erpm100 * 100,
        crc_ok=(crc == crc8(data[:FRAME_LEN - 1])),
    )


def encode_frame(*, temperature_c: int, voltage_cv: int, current_ca: int,
                 consumption_mah: int, erpm100: int) -> bytes:
    """Build one CRC'd 10-byte frame (inverse of :func:`parse_frame`).

    The single owner of the wire layout for producers (simulator, tests) so
    an encoder can never drift from the parser."""
    body = struct.pack(">bHHHH",
                       max(-128, min(127, temperature_c)),
                       voltage_cv & 0xFFFF, current_ca & 0xFFFF,
                       consumption_mah & 0xFFFF, erpm100 & 0xFFFF)
    return body + bytes([crc8(body)])


class KissStream:
    """Incremental framer for the KISS byte stream.

    Feed raw bytes from the serial port with :meth:`feed`; it yields every
    CRC-valid frame found. Because the protocol is delimiter-less, a window that
    fails CRC is advanced by a single byte to resynchronise.
    """

    def __init__(self) -> None:
        self._buf = bytearray()

    def feed(self, data: bytes) -> Iterator[KissFrame]:
        self._buf.extend(data)
        while len(self._buf) >= FRAME_LEN:
            window = bytes(self._buf[:FRAME_LEN])
            # CRC first: on a misaligned/noisy line the framer slides one byte
            # at a time, and unpacking + constructing a KissFrame per slide
            # would dominate the telemetry thread.
            if crc8(window[:FRAME_LEN - 1]) == window[FRAME_LEN - 1]:
                del self._buf[:FRAME_LEN]
                yield parse_frame(window)
            else:
                del self._buf[:1]  # resync

    def reset(self) -> None:
        self._buf.clear()
