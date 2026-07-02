"""Metric-math tests on synthetic rows: demag channels and CPU-load robustness."""
from hwci import metrics as metricsmod
from hwci.config import Profile, Segment
from hwci.model import RunResult


def _profile():
    return Profile(name="synthetic", sample_rate_hz=100.0,
                   segments=[Segment(label="hold", throttle=0.9,
                                     duration_s=1.0, steady=True)])


def _rows(n, **cols):
    """n rows at 100 Hz; cols maps column -> list or callable(i)."""
    rows = []
    for i in range(n):
        row = {"t": i * 0.01, "segment": "hold", "throttle_cmd": 0.9}
        for k, v in cols.items():
            row[k] = v(i) if callable(v) else v[i]
        rows.append(row)
    return rows


def test_bemf_counter_values_all_count():
    # Firmware bemf_timeout_happened is a counter (2, 3, ... latched at 102),
    # not a boolean - every non-zero sample is a timeout state.
    vals = [0, 0, 1, 2, 50, 102, 0, 0, 0, 0]
    rows = _rows(len(vals), perf_bemf_timeout=vals)
    d = metricsmod.detect_demag(RunResult(rows=rows), _profile())
    assert d["bemf_timeout_samples"] == 4
    assert d["event_count"] >= 1


def test_rpm_collapse_detected_at_high_throttle():
    # Steady 90% throttle; RPM collapses 40% mid-segment (desync whose
    # transient firmware flags fell between SWD samples).
    rpm = [20000.0] * 30 + [11000.0] * 10 + [20000.0] * 60
    rows = _rows(len(rpm), stand_rpm=rpm)
    d = metricsmod.detect_demag(RunResult(rows=rows), _profile())
    assert d["rpm_drop_samples"] >= 5
    assert d["event_count"] >= 1


def test_spoolup_is_not_a_collapse():
    # RPM rising monotonically after a step must not flag (the reference is
    # the running max, which trails a rising signal).
    rpm = [5000.0 + 150.0 * i for i in range(100)]
    rows = _rows(len(rpm), stand_rpm=rpm)
    d = metricsmod.detect_demag(RunResult(rows=rows), _profile())
    assert d["rpm_drop_samples"] == 0


def test_smooth_rampdown_is_not_a_collapse():
    # A multi-second ramp-down moves throttle by a tiny amount each 10ms
    # tick; RPM falling in lockstep with a commanded ramp-down is expected
    # deceleration, not desync (observed false positive on the bench: a
    # efficiency_sweep 100%->0% rampdn flagged with zero bemf timeouts,
    # commutation spikes, or eRPM mismatch).
    n = 400  # 4s at 100Hz
    throttle = [1.0 - 1.0 * i / (n - 1) for i in range(n)]  # smooth 1.0 -> 0.0
    rpm = [28000.0 * t for t in throttle]  # RPM tracks throttle exactly
    rows = []
    for i in range(n):
        rows.append({"t": i * 0.01, "segment": "rampdn",
                     "throttle_cmd": throttle[i], "stand_rpm": rpm[i]})
    d = metricsmod.detect_demag(
        RunResult(rows=rows),
        Profile(name="synthetic", sample_rate_hz=100.0,
               segments=[Segment(label="rampdn", throttle=0.0,
                                 duration_s=4.0, ramp=True)]))
    assert d["rpm_drop_samples"] == 0
    assert d["event_count"] == 0


def test_collapse_during_ramp_up_still_detected():
    # The trend-based gate must not blind the detector to a real collapse
    # that happens to occur while throttle is rising.
    n = 60
    throttle = [0.5 + 0.4 * i / (n - 1) for i in range(n)]  # 0.5 -> 0.9 rising
    rpm = [20000.0 + 100.0 * i for i in range(n)]
    for i in range(20, 30):
        rpm[i] = 11000.0  # desync collapse mid-ramp, throttle still rising
    rows = []
    for i in range(n):
        rows.append({"t": i * 0.01, "segment": "rampup",
                     "throttle_cmd": throttle[i], "stand_rpm": rpm[i]})
    d = metricsmod.detect_demag(
        RunResult(rows=rows),
        Profile(name="synthetic", sample_rate_hz=100.0,
               segments=[Segment(label="rampup", throttle=0.9,
                                 duration_s=0.6, ramp=True)]))
    assert d["rpm_drop_samples"] >= 5
    assert d["event_count"] >= 1


def test_rig_pole_pairs_from_meta_overrides_profile():
    # rig meta says 11 pole pairs; profile default is 7. eRPM 154000 at
    # 14000 stand RPM matches 11pp exactly -> no mismatch when meta is used.
    n = 50
    rows = _rows(n, stand_rpm=lambda i: 14000.0, esc_erpm=lambda i: 154000)
    d = metricsmod.detect_demag(
        RunResult(meta={"pole_pairs": 11}, rows=rows), _profile())
    assert d["esc_rpm_mismatch_samples"] == 0
    d7 = metricsmod.detect_demag(RunResult(meta={}, rows=rows), _profile())
    assert d7["esc_rpm_mismatch_samples"] == n  # wrong pp -> pervasive mismatch


def test_cpu_load_idle_rate_ignores_stall_glitch():
    # 120k iters/s idle; one sample pair spans a 100 ms host stall (11x the
    # iteration delta) - a max-based idle rate would scale every other
    # sample to ~91% load; the median must shrug it off.
    n = 100
    rows = []
    iters = 0
    for i in range(n):
        di = 1200 if i != 50 else 1200 * 11  # the stall sample
        iters += di
        rows.append({"t": i * 0.01, "segment": "idle", "throttle_cmd": 0.0,
                     "perf_loop_iters": iters})
    m = metricsmod._cpu_load(rows)
    load, idle_rate = m
    assert abs(idle_rate - 120000.0) < 1000.0


def test_cpu_load_prefers_perf_host_t():
    # Same counter stream, but perf_host_t records the TRUE read times
    # including the stall - the rate stays flat and load stays ~0 everywhere.
    n = 100
    rows = []
    iters = 0
    t_true = 0.0
    for i in range(n):
        dt_true = 0.01 if i != 50 else 0.11  # actual wall clock incl. stall
        t_true += dt_true
        iters += int(120000 * dt_true)
        rows.append({"t": i * 0.01, "segment": "idle", "throttle_cmd": 0.0,
                     "perf_loop_iters": iters, "perf_host_t": t_true})
    load, idle_rate = metricsmod._cpu_load(rows)
    assert abs(idle_rate - 120000.0) < 1500.0
    import numpy as np
    assert np.nanmax(load) < 5.0
