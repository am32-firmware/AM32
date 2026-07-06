"""Simulated Flight Stand backed by :class:`hwci.sim.RigSimulator`.

Lets the entire harness run end-to-end with no hardware. The same
:class:`~hwci.sim.RigSimulator` instance also feeds the simulated KISS telemetry
and perf-struct sources so all three channels stay consistent (see
:func:`hwci.runner.build_sim_sources`).
"""
from __future__ import annotations

import time
from typing import TYPE_CHECKING, Callable

from .base import SafetyLimits, StandSafetyTripped, StandSample, ThrustStand

if TYPE_CHECKING:  # runtime import is lazy: hwci.sim itself imports
    from ..sim import MotorParams, RigSimulator   # this package (StandSample)

__all__ = ["SimulatedStand", "StandSafetyTripped"]


class SimulatedStand(ThrustStand):
    def __init__(
        self,
        rig: "RigSimulator | None" = None,
        *,
        clock: Callable[[], float] = time.monotonic,
        fixed_dt: float | None = None,
        params: "MotorParams | None" = None,
    ):
        # Imported here, not at module top: hwci.sim imports this package for
        # StandSample, so a top-level import is a cycle that makes
        # `import hwci.sim` fail whenever it happens to run first.
        from ..sim import MotorParams, RigSimulator
        self.rig = rig or RigSimulator(params=params or MotorParams())
        self._clock = clock
        self._fixed_dt = fixed_dt
        self._throttle = 0.0
        self._last_t: float | None = None
        self._limits = SafetyLimits()

    def open(self) -> "SimulatedStand":
        self._last_t = self._clock()
        return self

    def set_throttle(self, throttle: float) -> None:
        self._throttle = max(0.0, min(1.0, throttle))

    def set_safety_limits(self, limits: SafetyLimits) -> None:
        self._limits = limits

    def read_sample(self) -> StandSample:
        now = self._clock()
        if self._fixed_dt is not None:
            dt = self._fixed_dt
        else:
            dt = 0.0 if self._last_t is None else max(0.0, now - self._last_t)
        self._last_t = now
        self.rig.step(dt, self._throttle)
        sample = self.rig.stand_sample(now)
        self._check_limits(sample)
        return sample

    def _check_limits(self, s: StandSample) -> None:
        self._limits.check(thrust_n=s.thrust_n, current_a=s.current_a,
                           rpm=s.rpm, voltage_v=s.voltage_v)

    def close(self) -> None:
        self._throttle = 0.0
