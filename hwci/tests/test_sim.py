"""Tests for the rig simulator: cross-channel consistency + demag injection."""
import pytest

from hwci import perf
from hwci.esc_telem import parse_frame
from hwci.sim import MotorParams, RigSimulator


def _settle(rig, throttle, n=200, dt=0.005):
    for _ in range(n):
        rig.step(dt, throttle)


def test_thrust_increases_with_throttle():
    lo = RigSimulator(noise=0.0)
    hi = RigSimulator(noise=0.0)
    _settle(lo, 0.3)
    _settle(hi, 0.8)
    assert hi.thrust_n > lo.thrust_n > 0


def test_efficiency_is_plausible():
    rig = RigSimulator(noise=0.0)
    _settle(rig, 0.5)
    s = rig.stand_sample(1.0)
    # A few g/W is realistic for a loaded 5" setup.
    assert 1.0 < s.efficiency_gf_per_w < 15.0


def test_kiss_channel_matches_state():
    rig = RigSimulator(noise=0.0)
    _settle(rig, 0.6)
    frame = parse_frame(rig.kiss_bytes())
    assert frame.crc_ok
    assert frame.voltage_v == pytest.approx(rig.voltage, abs=0.05)
    assert frame.e_rpm == pytest.approx(rig.e_rpm, rel=0.02, abs=200)


def test_perf_channel_decodes_and_is_consistent():
    rig = RigSimulator(noise=0.0)
    _settle(rig, 0.7)
    sample = perf.decode(rig.perf_bytes())
    assert sample.raw["running"] == 1
    assert 0 < sample.raw["commutation_interval_max"]
    assert sample.loop_iters > 0
    assert sample.e_rpm == pytest.approx(rig.e_rpm, rel=0.02, abs=200)


def test_demag_injection_triggers_desync():
    rig = RigSimulator(params=MotorParams(demag_prone=True), noise=0.0)
    _settle(rig, 0.1, n=50)
    rpm_before = rig.rpm
    rig.step(0.005, 1.0)   # violent step into a high-current regime
    assert rig.desync_count >= 1
    # Sync loss collapses RPM rather than spooling up.
    assert rig.rpm < rpm_before * 1.1
    sample = perf.decode(rig.perf_bytes())
    assert sample.raw["bemf_timeout_state"] == 1


def test_no_demag_when_not_prone():
    rig = RigSimulator(params=MotorParams(demag_prone=False), noise=0.0)
    _settle(rig, 0.1, n=50)
    rig.step(0.005, 1.0)
    assert rig.desync_count == 0


def test_perf_channel_carries_zc_jitter_accumulators():
    rig = RigSimulator(noise=0.0)
    _settle(rig, 0.7)
    a = perf.decode(rig.perf_bytes()).raw
    _settle(rig, 0.7)
    b = perf.decode(rig.perf_bytes()).raw
    # monotonic sums, growing while running
    assert b["zc_count"] > a["zc_count"] > 0
    assert b["zc_jitter_sum"] > a["zc_jitter_sum"]
    assert b["zc_interval_sum"] > a["zc_interval_sum"]
    # mean fractional jitter is the sim's modeled ~0.5% of the interval
    mean_pct = 100.0 * (b["zc_jitter_sum"] - a["zc_jitter_sum"]) / (
        b["zc_interval_sum"] - a["zc_interval_sum"])
    assert 0.1 < mean_pct < 2.0
    assert b["zc_jitter_max"] >= 1
