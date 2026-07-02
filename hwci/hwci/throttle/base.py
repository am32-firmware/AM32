"""Throttle-source abstraction.

The throttle source is whatever generates the ESC signal during a test. Two
backends are provided:

* :class:`~hwci.throttle.flightstand_src.FlightStandThrottle` - the Flight Stand
  drives its own ESC output; simplest and perfectly synchronized with logging.
* :class:`~hwci.throttle.external.ExternalSerialThrottle` - a separate DShot/PWM
  signal generator (e.g. an MCU bridge), for scripted DShot sequences or when
  the stand can't emit the protocol you need.

Throttle is always a normalized float in [0, 1]; each backend maps it to its
native units (PWM microseconds, DShot 48..2047, stand output units).
"""
from __future__ import annotations

import abc


class ThrottleSource(abc.ABC):
    @abc.abstractmethod
    def arm(self) -> None:
        """Bring the ESC to the armed/zero-throttle state."""

    @abc.abstractmethod
    def set(self, throttle: float) -> None:
        """Command throttle in [0, 1]."""

    def quiesce(self) -> None:
        """Make the signal line idle LOW so the AM32 bootloader will jump to
        the app on the next ESC reset. Zero throttle is not enough for DShot
        (frames keep the line high 40-75% of the time); backends that can
        drop the signal entirely should override this."""
        self.set(0.0)

    def disarm(self) -> None:
        self.set(0.0)

    def close(self) -> None:
        pass

    def __enter__(self) -> "ThrottleSource":
        self.arm()
        return self

    def __exit__(self, *exc) -> None:
        self.disarm()
        self.close()
