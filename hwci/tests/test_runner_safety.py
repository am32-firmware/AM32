"""Runner-level (host-side) safety enforcement - must trip on ANY rig, even
when the stand backend does no checking of its own."""
from hwci.config import Profile, Segment
from hwci.esc_telem.kiss import KissFrame
from hwci.flightstand.base import SafetyLimits
from hwci.flightstand.simulator import SimulatedStand
from hwci.runner import Sources, run_profile
from hwci.sim import MotorParams, RigSimulator
from hwci.throttle.flightstand_src import FlightStandThrottle


def _profile(**safety):
    return Profile(
        name="safety-test",
        sample_rate_hz=100.0,
        segments=[Segment(label="wot", throttle=1.0, duration_s=2.0)],
        safety=SafetyLimits(**safety),
    )


def _frame(current_a: float, temp_c: int = 30) -> KissFrame:
    return KissFrame(temperature_c=temp_c, voltage_v=16.0, current_a=current_a,
                     consumption_mah=0, e_rpm=10000, crc_ok=True)


def test_runner_trips_on_stand_current_without_backend_limits():
    # The stand itself gets NO limits (mirrors the gRPC backend, which cannot
    # enforce them until the vendor RPC is mapped) - the runner must trip.
    sim = RigSimulator(params=MotorParams(), noise=0.0)
    stand = SimulatedStand(sim, fixed_dt=0.01).open()
    sources = Sources(
        throttle=FlightStandThrottle(stand, arm_settle_s=0.0),
        stand=stand,
        perf_source=lambda: None,
        telem_source=lambda: None,
    )
    result = run_profile(_profile(max_current_a=5.0), sources, realtime=False)
    assert result.meta["aborted"] is not None
    assert "safety" in result.meta["aborted"]
    assert "current" in result.meta["aborted"]


def test_runner_trips_on_telemetry_only_rig():
    # No stand at all: the ESC telemetry current must still enforce the limit.
    class _DummyThrottle:
        def arm(self): pass
        def set(self, throttle): pass
        def disarm(self): pass
        def close(self): pass

    sources = Sources(
        throttle=_DummyThrottle(),
        stand=None,
        perf_source=lambda: None,
        telem_source=lambda: _frame(current_a=60.0),
    )
    result = run_profile(_profile(max_current_a=45.0), sources, realtime=False)
    assert result.meta["aborted"] is not None
    assert "current" in result.meta["aborted"]


def test_standless_run_completes_within_limits():
    class _DummyThrottle:
        def arm(self): pass
        def set(self, throttle): pass
        def disarm(self): pass
        def close(self): pass

    sources = Sources(
        throttle=_DummyThrottle(),
        stand=None,
        perf_source=lambda: None,
        telem_source=lambda: _frame(current_a=10.0),
    )
    result = run_profile(_profile(max_current_a=45.0), sources, realtime=False)
    assert result.meta["aborted"] is None
    assert len(result.rows) == 200
    assert result.rows[0]["stand_thrust_gf"] == ""  # stand columns empty
