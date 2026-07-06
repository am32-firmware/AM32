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


def test_zc_jitter_from_accumulator_deltas():
    # 100 commutations per tick at a 200-tick mean interval with 2 ticks of
    # mean deviation -> 1.0% jitter, computed from first/last snapshot deltas.
    n = 100
    rows = _rows(
        n,
        perf_zc_count=lambda i: 100 * i,
        perf_zc_jitter_sum=lambda i: 2 * 100 * i,
        perf_zc_interval_sum=lambda i: 200 * 100 * i,
        perf_zc_jitter_max=lambda i: 9,
    )
    m = metricsmod.compute(RunResult(rows=rows), _profile())
    pt = m["steady_points"][0]
    assert pt["zc_jitter_pct"] == 1.0
    assert pt["zc_jitter_max_pct"] == 4.5  # 9 / 200
    assert m["summary"]["worst_zc_jitter_pct"] == 1.0
    assert m["summary"]["worst_zc_jitter_max_pct"] == 4.5


def test_zc_jitter_survives_u32_wrap():
    # interval_sum wraps u32 mid-window (it grows ~2M ticks/s on hardware);
    # modular differencing must keep the ratio exact.
    n = 100
    start = (1 << 32) - 200 * 100 * 50  # wraps halfway through the run
    rows = _rows(
        n,
        perf_zc_count=lambda i: 100 * i,
        perf_zc_jitter_sum=lambda i: 2 * 100 * i,
        perf_zc_interval_sum=lambda i: (start + 200 * 100 * i) % (1 << 32),
        perf_zc_jitter_max=lambda i: 9,
    )
    m = metricsmod.compute(RunResult(rows=rows), _profile())
    assert m["steady_points"][0]["zc_jitter_pct"] == 1.0


def test_zc_jitter_none_for_v1_runs():
    # Runs captured on pre-v2 firmware have no zc columns: the metric must
    # report None (unavailable), never 0 (which would read as "perfect").
    rows = _rows(50, perf_loop_iters=lambda i: 1200 * i)
    m = metricsmod.compute(RunResult(rows=rows), _profile())
    pt = m["steady_points"][0]
    assert pt["zc_jitter_pct"] is None
    assert pt["zc_jitter_max_pct"] is None
    assert m["summary"]["worst_zc_jitter_pct"] is None


def test_zc_jitter_none_when_no_commutations_accumulate():
    # Counters present but frozen (motor stopped / startup-gated): no window
    # delta, so the metric is unavailable rather than 0/0 noise.
    rows = _rows(
        50,
        perf_zc_count=lambda i: 5000,
        perf_zc_jitter_sum=lambda i: 777,
        perf_zc_interval_sum=lambda i: 999999,
        perf_zc_jitter_max=lambda i: 3,
    )
    m = metricsmod.compute(RunResult(rows=rows), _profile())
    assert m["steady_points"][0]["zc_jitter_pct"] is None


def _startup_profile(n=3):
    segs = [Segment(label="idle", throttle=0.0, duration_s=0.5)]
    for i in range(1, n + 1):
        segs.append(Segment(label=f"start{i}", throttle=0.08, duration_s=1.0))
        segs.append(Segment(label=f"off{i}", throttle=0.0, duration_s=0.5))
    return Profile(name="synthetic-start", sample_rate_hz=100.0, segments=segs)


def _startup_rows(outcomes, ticks_per_start=100, ticks_per_off=50, run_delay=20):
    """Rows for idle + N start/off cycles; outcomes[i] says attempt i starts."""
    rows, t = [], 0.0
    def add(label, n, running, erpm):
        nonlocal t
        for j in range(n):
            rows.append({"t": t, "segment": label, "throttle_cmd": 0.08,
                         "perf_running": running(j), "perf_e_rpm": erpm(j)})
            t += 0.01
    add("idle", 50, lambda j: 0, lambda j: 0)
    for i, ok in enumerate(outcomes, start=1):
        if ok:
            add(f"start{i}", ticks_per_start,
                lambda j: 1 if j >= run_delay else 0,
                lambda j: 21000 if j >= run_delay else 0)
        else:
            add(f"start{i}", ticks_per_start, lambda j: 0, lambda j: 0)
        add(f"off{i}", ticks_per_off, lambda j: 0, lambda j: 0)
    return rows


def test_startup_stats_counts_failures_and_times():
    prof = _startup_profile(3)
    rows = _startup_rows([True, False, True], run_delay=20)
    st = metricsmod.startup_stats(RunResult(rows=rows), prof)
    assert st["attempts"] == 3
    assert st["failures"] == 1
    assert st["time_to_run_ms_max"] == 200.0  # 20 ticks at 100 Hz
    fails = [a for a in st["per_attempt"] if not a["success"]]
    assert [a["segment"] for a in fails] == ["start2"]


def test_startup_requires_running_and_erpm_together():
    # A stale running flag with no eRPM (or vice versa) is not a start.
    prof = _startup_profile(1)
    rows = _startup_rows([True])
    for r in rows:
        if r["segment"] == "start1":
            r["perf_e_rpm"] = 0  # running asserted but motor not turning
    st = metricsmod.startup_stats(RunResult(rows=rows), prof)
    assert st["failures"] == 1


def test_startup_stats_zero_for_profiles_without_start_segments():
    st = metricsmod.startup_stats(
        RunResult(rows=_rows(20, perf_running=lambda i: 1)), _profile())
    assert st["attempts"] == 0 and st["failures"] == 0


def test_compute_summary_carries_start_outcomes():
    prof = _startup_profile(2)
    rows = _startup_rows([True, False])
    m = metricsmod.compute(RunResult(rows=rows), prof)
    assert m["summary"]["start_attempts"] == 2
    assert m["summary"]["start_failures"] == 1
    # profiles without start segments report None, not 0-failures-implied-pass
    m2 = metricsmod.compute(RunResult(rows=_rows(20)), _profile())
    assert m2["summary"]["start_attempts"] is None
    assert m2["summary"]["start_failures"] is None


def test_spoolup_intervals_after_step_are_not_spikes():
    # First hardware run of demag_step_stress: a 10->90 snap spools from low
    # RPM where commutation intervals are honestly several times the
    # high-RPM-dominated median - flagged 2 false demag events with zero
    # corroborating signals. Long intervals within the settle window after a
    # commanded step-up must not count as spikes.
    n = 200  # 2 s at 100 Hz
    throttle = [0.1] * 50 + [0.9] * 150          # snap at i=50
    comm = [500.0] * 50 + [400.0] * 30 + [100.0] * 120  # long during spool
    rows = []
    for i in range(n):
        rows.append({"t": i * 0.01, "segment": "step", "throttle_cmd": throttle[i],
                     "perf_commutation_interval": comm[i]})
    d = metricsmod.detect_demag(
        RunResult(rows=rows),
        Profile(name="synthetic", sample_rate_hz=100.0,
                segments=[Segment(label="step", throttle=0.9, duration_s=2.0)]))
    assert d["comm_spike_samples"] == 0
    assert d["event_count"] == 0


def test_steady_state_comm_spike_still_detected():
    # The step-up suppression must not blind the detector to a real spike
    # long after the last throttle increase.
    n = 400
    comm = [100.0] * n
    for i in range(300, 310):
        comm[i] = 900.0  # 9x median, 2.5 s after the only step-up
    rows = []
    for i in range(n):
        rows.append({"t": i * 0.01, "segment": "hold", "throttle_cmd": 0.9,
                     "perf_commutation_interval": comm[i]})
    d = metricsmod.detect_demag(RunResult(rows=rows), _profile())
    assert d["comm_spike_samples"] >= 8
    assert d["event_count"] >= 1
