"""External serial throttle source (DShot/PWM signal generator bridge).

For runs where the ESC signal is produced by a dedicated generator rather than
the Flight Stand - e.g. a small MCU (Arduino/Teensy/STM32) running a DShot
generator, exposing a trivial serial protocol:

    ARM\\n            -> arm / idle
    T <0..2047>\\n    -> set DShot throttle value
    DISARM\\n

This keeps scripted DShot300/600 demag-stress sequences possible even if the
stand only emits PWM. Swap in any generator by matching this line protocol, or
subclass and override :meth:`_write`.
"""
from __future__ import annotations

from .base import ThrottleSource

DSHOT_MIN = 48
DSHOT_MAX = 2047


class ExternalSerialThrottle(ThrottleSource):
    def __init__(self, port: str, baud: int = 115200):
        self.port = port
        self.baud = baud
        self._ser = None

    def _open(self):
        import serial  # only needed on the rig
        if self._ser is None:
            self._ser = serial.Serial(self.port, self.baud, timeout=0.5)
        return self._ser

    def _write(self, line: str) -> None:
        ser = self._open()
        ser.write((line + "\n").encode())
        ser.flush()

    def arm(self) -> None:
        self._write("ARM")

    def set(self, throttle: float) -> None:
        throttle = max(0.0, min(1.0, throttle))
        value = int(DSHOT_MIN + throttle * (DSHOT_MAX - DSHOT_MIN))
        self._write(f"T {value}")

    def disarm(self) -> None:
        self._write("DISARM")

    def close(self) -> None:
        if self._ser is not None:
            self._ser.close()
            self._ser = None
