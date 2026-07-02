"""Tests for the simulated Flight Stand backend."""
import pytest

from hwci.flightstand.base import SafetyLimits
from hwci.flightstand.simulator import SimulatedStand, StandSafetyTripped
from hwci.sim import MotorParams, RigSimulator


def _stand(**kw):
    # Deterministic: fixed timestep, no noise.
    rig = RigSimulator(params=kw.pop("params", MotorParams()), noise=0.0)
    return SimulatedStand(rig, fixed_dt=0.01).open()


def test_throttle_produces_thrust():
    stand = _stand()
    stand.set_throttle(0.7)
    last = None
    for _ in range(200):
        last = stand.read_sample()
    assert last.thrust_n > 0
    assert last.rpm > 1000
    assert last.efficiency_gf_per_w > 0


def test_zero_throttle_zero_thrust():
    stand = _stand()
    stand.set_throttle(0.0)
    for _ in range(50):
        s = stand.read_sample()
    assert s.thrust_n == pytest.approx(0.0, abs=1e-6)


def test_safety_limit_trips():
    stand = _stand()
    stand.set_safety_limits(SafetyLimits(max_current_a=5.0))
    stand.set_throttle(1.0)
    with pytest.raises(StandSafetyTripped):
        for _ in range(500):
            stand.read_sample()


def test_context_manager_zeroes_on_exit():
    rig = RigSimulator(noise=0.0)
    with SimulatedStand(rig, fixed_dt=0.01) as stand:
        stand.set_throttle(0.5)
        stand.read_sample()
    assert stand._throttle == 0.0
