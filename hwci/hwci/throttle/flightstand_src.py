"""Throttle source that uses the Flight Stand's own ESC output."""
from __future__ import annotations

import time

from ..flightstand.base import ThrustStand
from .base import ThrottleSource


class FlightStandThrottle(ThrottleSource):
    def __init__(self, stand: ThrustStand, arm_settle_s: float = 1.0):
        self.stand = stand
        self.arm_settle_s = arm_settle_s

    def arm(self) -> None:
        self.stand.set_throttle(0.0)
        # Hold zero throttle long enough for the ESC to arm.
        time.sleep(self.arm_settle_s)

    def set(self, throttle: float) -> None:
        self.stand.set_throttle(throttle)

    def quiesce(self) -> None:
        # Deactivate the stand's ESC output entirely: the line is driven to
        # logic 0, which is what lets the AM32 bootloader exit to the app
        # (verified on the ARK 4IN1 bench; DShot-at-zero is NOT enough).
        self.stand.deactivate()
