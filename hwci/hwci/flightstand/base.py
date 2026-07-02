"""Thrust-stand backend abstraction.

A :class:`ThrustStand` represents a Tyto Robotics Flight Stand (50, or any of
the family) and provides throttle control + synchronized force/electrical
measurement. The real backend talks gRPC to the Flight Stand Software; the
simulator implements the same interface for offline development and tests.

Units are SI internally (Newtons, N*m, RPM, V, A). ``grams-force`` and
``g/W`` (the usual drone efficiency figure) are derived properties.
"""
from __future__ import annotations

import abc
from dataclasses import dataclass

G0 = 9.80665  # standard gravity, m/s^2


class StandSafetyTripped(RuntimeError):
    """A profile safety limit was breached; the run must abort immediately."""


@dataclass
class StandSample:
    t: float            # host monotonic timestamp, seconds
    throttle: float     # last commanded throttle, 0..1 (echoed)
    thrust_n: float
    torque_nm: float
    rpm: float          # mechanical RPM
    voltage_v: float
    current_a: float
    motor_temp_c: float | None = None  # optional probe (None = not fitted)
    fet_temp_c: float | None = None

    @property
    def elec_power_w(self) -> float:
        return self.voltage_v * self.current_a

    @property
    def mech_power_w(self) -> float:
        omega = self.rpm * 2.0 * 3.141592653589793 / 60.0
        return self.torque_nm * omega

    @property
    def thrust_gf(self) -> float:
        return self.thrust_n / G0 * 1000.0

    @property
    def efficiency_gf_per_w(self) -> float:
        p = self.elec_power_w
        return self.thrust_gf / p if p > 1e-6 else 0.0

    @property
    def motor_efficiency(self) -> float:
        p = self.elec_power_w
        return self.mech_power_w / p if p > 1e-6 else 0.0


@dataclass
class SafetyLimits:
    """Cutoffs enforced host-side by the runner on every sample (see
    :func:`hwci.runner.enforce_safety`); a breach aborts the test immediately.
    Backends may ALSO enforce them (the simulator does; the vendor Flight
    Stand Software has its own UI cutoffs), but the runner check is the one
    that is guaranteed to exist on every rig."""
    max_thrust_n: float | None = None
    max_current_a: float | None = None
    max_rpm: float | None = None
    max_voltage_v: float | None = None
    max_motor_temp_c: float | None = None

    def check(self, *, thrust_n: float | None = None,
              current_a: float | None = None, rpm: float | None = None,
              voltage_v: float | None = None,
              temp_c: float | None = None) -> None:
        """Raise :class:`StandSafetyTripped` if any provided value exceeds
        its limit. ``None`` values (channel not available) are skipped."""
        def _over(value, limit):
            return value is not None and limit is not None and value > limit
        if _over(thrust_n, self.max_thrust_n):
            raise StandSafetyTripped(
                f"thrust {thrust_n:.2f} N > limit {self.max_thrust_n:.2f} N")
        if _over(current_a, self.max_current_a):
            raise StandSafetyTripped(
                f"current {current_a:.1f} A > limit {self.max_current_a:.1f} A")
        if _over(rpm, self.max_rpm):
            raise StandSafetyTripped(
                f"rpm {rpm:.0f} > limit {self.max_rpm:.0f}")
        if _over(voltage_v, self.max_voltage_v):
            raise StandSafetyTripped(
                f"voltage {voltage_v:.2f} V > limit {self.max_voltage_v:.2f} V")
        if _over(temp_c, self.max_motor_temp_c):
            raise StandSafetyTripped(
                f"temp {temp_c:.0f} C > limit {self.max_motor_temp_c:.0f} C")


class ThrustStand(abc.ABC):
    @abc.abstractmethod
    def open(self) -> "ThrustStand":
        ...

    @abc.abstractmethod
    def set_throttle(self, throttle: float) -> None:
        """Command throttle in [0, 1]."""

    @abc.abstractmethod
    def read_sample(self) -> StandSample:
        ...

    def set_safety_limits(self, limits: SafetyLimits) -> None:  # optional
        pass

    def deactivate(self) -> None:  # optional: drop the ESC signal (line low)
        pass

    def tare(self) -> None:  # optional: zero the load cells
        pass

    def close(self) -> None:  # optional
        pass

    def __enter__(self) -> "ThrustStand":
        return self.open()

    def __exit__(self, *exc) -> None:
        self.set_throttle(0.0)
        self.close()
