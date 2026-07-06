"""Tests for the rig simulator: cross-channel consistency + demag injection."""
import pytest

from hwci import perf
from hwci.esc_telem import parse_frame
from hwci.settings import Settings, default_blob
from hwci.sim import MotorParams, RigSimulator, SimSettings


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


def test_perf_channel_carries_confirm_reject_counter():
    rig = RigSimulator(noise=0.0)
    _settle(rig, 0.7)
    a = perf.decode(rig.perf_bytes()).raw
    _settle(rig, 0.7)
    b = perf.decode(rig.perf_bytes()).raw
    assert b["zc_confirm_reject"] > a["zc_confirm_reject"] > 0
    # clean running: rejects stay a small fraction of accepted ZCs
    ratio = (b["zc_confirm_reject"] - a["zc_confirm_reject"]) / (
        b["zc_count"] - a["zc_count"])
    assert 0.001 < ratio < 0.05


def test_perf_channel_carries_phase_histogram():
    rig = RigSimulator(noise=0.0)
    _settle(rig, 0.7)
    a = perf.decode(rig.perf_bytes()).raw["zc_phase_hist"]
    _settle(rig, 0.7)
    b = perf.decode(rig.perf_bytes()).raw["zc_phase_hist"]
    delta = [(y - x) % 65536 for x, y in zip(a, b)]
    assert sum(delta) > 0
    # the sim locks 20% of edges onto the throttle-derived bin
    peak = int(0.7 * 32) & 31
    assert max(range(32), key=delta.__getitem__) == peak


# --------------------------------------------------------------------------
# AM32 settings model (SimSettings)
# --------------------------------------------------------------------------
def _sim_with(overrides, **params):
    blob = Settings(default_blob()).apply(overrides).to_bytes()
    rig = RigSimulator(params=MotorParams(**params), noise=0.0)
    rig.set_settings(SimSettings.from_blob(blob))
    return rig


def _steady_efficiency(rig, throttle=0.6, n=400, dt=0.005):
    _settle(rig, throttle, n=n, dt=dt)
    s = rig.stand_sample(1.0)
    return (s.thrust_n * 1000.0 / 9.80665) / (s.voltage_v * s.current_a)


def test_simsettings_decodes_the_same_blob_settings_writes():
    blob = Settings(default_blob()).apply(
        {"advance_level": 30, "pwm_frequency": 16, "auto_advance": 1}).to_bytes()
    s = SimSettings.from_blob(blob)
    assert (s.advance_level, s.pwm_frequency, s.auto_advance) == (30, 16, 1)
    assert s.max_ramp == 160 and s.startup_power == 100


def test_efficiency_has_interior_maximum_in_advance():
    # optimum injectable: put it at 27 and check 27 beats both grid edges
    effs = {adv: _steady_efficiency(
                _sim_with({"advance_level": adv}, advance_optimum=27.0))
            for adv in (13, 27, 41)}
    assert effs[27] > effs[13]
    assert effs[27] > effs[41]


def test_efficiency_has_interior_maximum_in_pwm_frequency():
    effs = {khz: _steady_efficiency(_sim_with({"pwm_frequency": khz}))
            for khz in (8, 24, 96)}
    assert effs[24] > effs[8]
    assert effs[24] > effs[96]


def test_auto_advance_scores_as_equivalent_fixed_advance():
    auto = _steady_efficiency(_sim_with({"auto_advance": 1,
                                         "advance_level": 40}))
    fixed18 = _steady_efficiency(_sim_with({"auto_advance": 0,
                                            "advance_level": 18}))
    assert auto == pytest.approx(fixed18, rel=1e-6)


def test_variable_pwm_adds_jitter_and_low_throttle_bonus():
    lo_var = _steady_efficiency(_sim_with({"variable_pwm": 1}), throttle=0.3)
    lo_fix = _steady_efficiency(_sim_with({"variable_pwm": 0}), throttle=0.3)
    assert lo_var > lo_fix  # small low-throttle efficiency bonus
    # jitter penalty: compare accumulated mean jitter fraction
    var, fix = _sim_with({"variable_pwm": 1}), _sim_with({"variable_pwm": 0})
    _settle(var, 0.6, n=400)
    _settle(fix, 0.6, n=400)
    var_pct = var.zc_jitter_sum / var.zc_interval_sum
    fix_pct = fix.zc_jitter_sum / fix.zc_interval_sum
    assert var_pct > fix_pct


def test_startup_reliability_responds_to_startup_power():
    def failed_starts(power, attempts=40):
        rig = _sim_with({"startup_power": power})
        fails = 0
        for _ in range(attempts):
            _settle(rig, 0.0, n=100)    # stop and coast down between attempts
            _settle(rig, 0.15, n=100)   # spin attempt (0.5 s)
            if rig.rpm < 500.0:
                fails += 1
        return fails

    assert failed_starts(50) > failed_starts(150)
    assert failed_starts(150) == 0


def test_low_max_ramp_prevents_step_desync():
    fast = _sim_with({"max_ramp": 160}, demag_prone=True)
    slow = _sim_with({"max_ramp": 40}, demag_prone=True)
    for rig in (fast, slow):
        _settle(rig, 0.1, n=50)
        rig.step(0.005, 1.0)  # violent step into a high-current regime
    assert fast.desync_count >= 1
    assert slow.desync_count == 0


def test_settings_none_keeps_legacy_behaviour():
    a = RigSimulator(noise=0.0)
    b = RigSimulator(noise=0.0)
    b.set_settings(None)
    _settle(a, 0.6)
    _settle(b, 0.6)
    assert a.rpm == b.rpm
    assert a.current == b.current
