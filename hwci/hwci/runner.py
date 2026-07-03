"""Test runner: execute a profile, sampling stand + ESC telemetry + perf struct.

The runner is source-agnostic: it drives a :class:`ThrottleSource`, reads a
:class:`ThrustStand`, and calls two callables for the perf struct and ESC
telemetry. :func:`build_sim_sources` wires all of these to one shared
:class:`~hwci.sim.RigSimulator` for offline runs; :func:`build_live_sources`
wires them to OpenOCD + serial + the gRPC stand on the rig.

Safety: the profile's :class:`~hwci.flightstand.base.SafetyLimits` are enforced
HERE, on every sample, against both the stand reading and the ESC telemetry.
Backends may additionally enforce them, but the runner check is the one that
exists on every rig (the vendor gRPC API has no mapped set-limit RPC yet).

Pre-flight: :func:`check_battery` gates a hardware run on pack voltage BEFORE
the throttle is ever armed (see :func:`build_live_sources`). It is opt-in
(``--battery-cells``) and separate from the per-sample ``SafetyLimits`` above -
a low-but-not-crashing pack should never even start a test, both to protect
the pack and because a sagging supply quietly corrupts efficiency data.
:func:`tare_for_run` then zeroes the load cells on every hardware run that has
a stand (default-on, ``--no-tare`` to skip) - with the ESC signal already up
at zero throttle, because AM32 beeps the motor whenever it has no input.
"""
from __future__ import annotations

import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Optional

from . import perf
from .config import Profile, RigConfig
from .esc_telem.kiss import KissFrame, KissStream, parse_frame
from .flightstand.base import SafetyLimits, StandSafetyTripped, StandSample, ThrustStand
from .metrics import tail_start_index
from .model import RunResult, make_row
from .perf import PerfSample
from .perf_reader import PerfReader
from .throttle.base import ThrottleSource

PerfSourceFn = Callable[[], Optional[PerfSample]]
TelemSourceFn = Callable[[], Optional[KissFrame]]


@dataclass
class Sources:
    throttle: ThrottleSource
    stand: Optional[ThrustStand]
    perf_source: PerfSourceFn
    telem_source: TelemSourceFn
    perf_reader: Optional[PerfReader] = None
    closers: list = field(default_factory=list)

    def close(self) -> None:
        for c in reversed(self.closers):
            try:
                c()
            except Exception:
                pass


# Ticks of margin between issuing a mid-segment perf-stats reset and the
# metrics tail window's first sample - see the call site in run_profile for
# why this must be a margin, not an exact boundary match.
RESET_MARGIN_TICKS = 20


def _tail_reset_tick(n: int, steady_tail_fraction: float) -> int:
    """Tick within a steady segment to reset perf stats at, strictly before
    the metrics tail window - computed from the SAME tail_start_index() the
    metrics module uses, so the two can never disagree by a rounding tick."""
    return max(0, tail_start_index(n, steady_tail_fraction) - RESET_MARGIN_TICKS)


def _segment_throttle(seg, tick_in_seg: int, n_ticks: int, prev: float) -> float:
    if not seg.ramp or n_ticks <= 1:
        return seg.throttle
    frac = tick_in_seg / (n_ticks - 1)
    return prev + (seg.throttle - prev) * min(1.0, frac)


def enforce_safety(limits: SafetyLimits, stand: StandSample | None,
                   telem: KissFrame | None) -> None:
    """Host-side safety check against every channel that produced data."""
    if stand is not None:
        limits.check(thrust_n=stand.thrust_n, current_a=stand.current_a,
                     rpm=stand.rpm, voltage_v=stand.voltage_v,
                     temp_c=stand.motor_temp_c)
    if telem is not None:
        limits.check(current_a=telem.current_a, temp_c=telem.temperature_c)


# Default per-cell low-voltage threshold for check_battery(). Matches AM32
# firmware's own default (Src/main.c: `low_cell_volt_cutoff = 330` -> 3.30
# V/cell), so the harness refuses to start a test at roughly the same pack
# voltage the ESC would eventually cut power at mid-run anyway.
DEFAULT_MIN_CELL_VOLTAGE = 3.3


class BatteryTooLowError(RuntimeError):
    """Raised before a run is armed: the pack is already at/below the
    minimum for its declared cell count, or its voltage can't be verified at
    all. Fails closed - a test must not start on a battery this low, both to
    avoid over-discharging it and because a sagging supply quietly corrupts
    efficiency data."""


def _live_voltage(stand: ThrustStand | None,
                  perf_source: PerfSourceFn) -> float | None:
    """Best pack-voltage reading available before the throttle is armed: the
    stand's HV bus channel if one is wired, else the ESC's own ADC via the
    perf struct (already alive by the time build_live_sources calls this -
    it runs after _ensure_app_alive). None if neither is available/readable."""
    if stand is not None:
        return stand.read_sample().voltage_v
    pf = _safe(perf_source)
    return pf.voltage if pf is not None else None


def check_battery(voltage_v: float | None, battery_cells: int,
                  min_cell_voltage: float = DEFAULT_MIN_CELL_VOLTAGE) -> None:
    """Refuse to proceed if the pack is at/below a safe minimum for its
    declared cell count. Only called when the caller passed
    ``--battery-cells`` - this check is opt-in, not a default gate."""
    minimum = battery_cells * min_cell_voltage
    if voltage_v is None:
        raise BatteryTooLowError(
            "cannot verify battery voltage before starting (no stand or "
            "perf voltage channel available on this rig) - wire a voltage "
            "channel or drop --battery-cells to skip this check")
    if voltage_v < minimum:
        raise BatteryTooLowError(
            f"battery {voltage_v:.2f} V < minimum {minimum:.2f} V for a "
            f"{battery_cells}S pack at {min_cell_voltage:.2f} V/cell - "
            "charge or swap the pack before running a test")


# Pre-run tare choreography. AM32 beeps the motor whenever it sees NO input
# signal (the disconnect beacon) and again as it arms - each beep is a real
# torque pulse through the mount, so a tare taken on a signal-less ESC bakes
# that twitching into the load-cell zero. Bring the signal up at zero
# throttle FIRST (the beacon stops), wait out the arm tune, and only then
# zero the load cells on a mechanically quiet rig.
ARM_TUNE_SETTLE_S = 2.0  # arm tune keeps shaking the motor after arm() returns
TARE_SETTLE_S = 1.5      # post-tare readings stay noisy for ~1-2 s (bench)


def tare_for_run(stand: ThrustStand, throttle: ThrottleSource, *,
                 settle: Callable[[float], None] = time.sleep) -> float | None:
    """Zero the load cells with the ESC held quiet at zero throttle (see the
    choreography note above). Returns the post-tare thrust residual in gf so
    the caller can surface it, or None if it can't be read - the residual
    read is best-effort, the tare itself is not."""
    throttle.arm()             # signal present at zero: beacon stops, ESC arms
    settle(ARM_TUNE_SETTLE_S)
    stand.tare()
    settle(TARE_SETTLE_S)
    try:
        return stand.read_sample().thrust_n * 1000.0 / 9.80665
    except Exception:
        return None


class _CachedPoller:
    """Background thread that keeps the most recent value of a slow source.

    An SWD struct read through ST-Link costs milliseconds; done inline it
    would consume the whole tick budget at 100-200 Hz and add jitter to every
    sample. Read errors are counted, never swallowed silently.
    """

    def __init__(self, fn, *, interval_s: float = 0.002,
                 max_age_s: float = 1.0, name: str = "poller"):
        self._fn = fn
        self._interval = interval_s
        self._max_age = max_age_s
        self._latest = None
        self._latest_at = float("-inf")
        self._errors = 0
        self._last_error: Exception | None = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True, name=name)
        self._thread.start()

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                value = self._fn()
            except Exception as e:
                self._errors += 1
                self._last_error = e
            else:
                self._latest = value
                self._latest_at = time.monotonic()
            self._stop.wait(self._interval)

    def latest(self):
        """Most recent value, or None once the source has been dead for
        max_age_s (a stale sample repeated forever would defeat the
        channel-coverage gate downstream)."""
        if time.monotonic() - self._latest_at > self._max_age:
            return None
        return self._latest

    @property
    def errors(self) -> int:
        return self._errors

    @property
    def last_error(self) -> Exception | None:
        return self._last_error

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)


def run_profile(profile: Profile, sources: Sources, *,
                realtime: bool = True, meta: dict | None = None) -> RunResult:
    period = 1.0 / profile.sample_rate_hz
    rows: list[dict] = []
    aborted: str | None = None

    sources.throttle.arm()
    if sources.perf_reader is not None:
        sources.perf_reader.reset_stats()  # measure worst-case for THIS run

    # Slow live sources move to a caching poller thread so the tick loop stays
    # deterministic. The sim path stays inline (cheap and reproducible).
    perf_poller: _CachedPoller | None = None
    perf_get: Callable[[], PerfSample | None]
    telem_errors = 0
    if realtime and sources.perf_reader is not None:
        perf_poller = _CachedPoller(sources.perf_source, name="perf-poller")
        perf_get = perf_poller.latest
    else:
        def perf_get():
            return _safe(sources.perf_source)

    start = time.monotonic()
    tick = 0
    prev_throttle = 0.0
    try:
        for seg in profile.segments:
            n = max(1, round(seg.duration_s * profile.sample_rate_hz))
            # Reset the firmware's sticky min/max accumulators BEFORE a
            # steady segment's measurement tail begins (with a margin - see
            # _tail_reset_tick), so the tail's worst case reflects THIS
            # operating point - not the spin-up transient of the run so far
            # (~700-950 us on the bench, varying 30%+ run-to-run, which would
            # drown steady loop-time regressions).
            #
            # The reset must land strictly BEFORE the tail's first sample,
            # not AT it: reset_stats() clears the firmware register over an
            # SWD round trip, and in realtime mode perf_get() reads an
            # independent background poller's CACHED value (_CachedPoller,
            # ~2ms interval) - so the tick that ISSUES the reset can still
            # observe the pre-reset cached sample. Observed on the bench: an
            # 877us arming-tune-transient value latched at t10's first tick
            # persisted for the segment's first 5s and was still visible in
            # the very sample the reset was issued on, making the "steady"
            # max equal the raw run max. A margin many times the poller
            # interval (and typical SWD round trip) makes this exclusion
            # instead of a race.
            tail_reset_tick = (_tail_reset_tick(n, profile.steady_tail_fraction)
                               if seg.steady and sources.perf_reader is not None
                               else None)
            for i in range(n):
                t_sched = tick * period
                if realtime:
                    target = start + t_sched
                    delay = target - time.monotonic()
                    if delay > 0:
                        time.sleep(delay)
                if tail_reset_tick is not None and i == tail_reset_tick:
                    try:
                        sources.perf_reader.reset_stats(verify=False)
                    except Exception:
                        pass  # a missed reset degrades one segment, not the run
                throttle = _segment_throttle(seg, i, n, prev_throttle)
                sources.throttle.set(throttle)
                stand = sources.stand.read_sample() if sources.stand is not None else None
                pf = perf_get()
                tm = _safe(sources.telem_source)
                enforce_safety(profile.safety, stand, tm)
                # Record the ACTUAL sample time: after a host stall the
                # schedule time would lie about how much wall clock the
                # sample covers (and corrupt counter-rate math downstream).
                t = (time.monotonic() - start) if realtime else t_sched
                rows.append(make_row(t, seg.label, throttle, stand, tm, pf))
                tick += 1
            prev_throttle = seg.throttle
    except StandSafetyTripped as e:
        aborted = f"safety: {e}"
    finally:
        try:
            sources.throttle.disarm()
        finally:
            if perf_poller is not None:
                perf_poller.close()

    full_meta = {
        "profile": profile.name,
        "sample_rate_hz": profile.sample_rate_hz,
        "n_samples": len(rows),
        "aborted": aborted,
        "wall_time_s": round(time.monotonic() - start, 3),
        "perf_read_errors": perf_poller.errors if perf_poller is not None else 0,
    }
    if perf_poller is not None and perf_poller.last_error is not None:
        full_meta["perf_last_error"] = repr(perf_poller.last_error)
    if meta:
        full_meta.update(meta)
    return RunResult(meta=full_meta, rows=rows)


def _safe(fn):
    try:
        return fn()
    except Exception:
        return None


# --------------------------------------------------------------------------
# Source builders
# --------------------------------------------------------------------------
def build_sim_sources(rig: RigConfig, profile: Profile, *,
                      demag_prone: bool = True,
                      sim=None, settings_blob: bytes | None = None) -> Sources:
    """All three channels fed by one RigSimulator (deterministic, no hardware).

    ``sim`` optionally supplies an existing :class:`~hwci.sim.RigSimulator` so
    one simulator (pack state, temperature, RNG) persists across multiple
    runs - the auto-tuner's trials share a rig exactly like hardware does.
    ``settings_blob`` installs a 192-byte AM32 settings page on it (decoded by
    :class:`~hwci.sim.SimSettings`, the sim's analogue of flash+reset).
    """
    from .flightstand.simulator import SimulatedStand
    from .sim import MotorParams, RigSimulator, SimSettings
    from .throttle.flightstand_src import FlightStandThrottle

    period = 1.0 / profile.sample_rate_hz
    if sim is None:
        sim = RigSimulator(params=MotorParams(pole_pairs=rig.pole_pairs,
                                              demag_prone=demag_prone))
    if settings_blob is not None:
        sim.set_settings(SimSettings.from_blob(settings_blob))
    stand = SimulatedStand(sim, fixed_dt=period).open()
    stand.set_safety_limits(profile.safety)
    throttle = FlightStandThrottle(stand, arm_settle_s=0.0)
    return Sources(
        throttle=throttle,
        stand=stand,
        perf_source=lambda: perf.decode(sim.perf_bytes()),
        telem_source=lambda: parse_frame(sim.kiss_bytes()),
        closers=[stand.close],
    )


def _ensure_app_alive(dbg, perf_reader: PerfReader,
                      throttle: ThrottleSource) -> None:
    """Get the ESC out of the AM32 bootloader and into the app.

    The bootloader only jumps to the app when the throttle signal line idles
    LOW at boot. A floating line reads high, and an ACTIVE DShot output keeps
    the line high 40-75% of each frame - either way the ESC parks in the
    bootloader after a flash/power-cycle and the run would produce no perf
    data and never arm. Detected via the hwci_perf magic: quiesce the
    throttle source (signal dropped, line driven low), reset, and wait for
    the app to publish the magic. The throttle is re-activated later by
    ``arm()``.
    """
    from .perf import PerfDecodeError

    def app_alive() -> bool:
        # Only a decode failure means "bootloader/no instrumentation";
        # a debugger error is a different fault and must propagate.
        try:
            perf_reader.read()
            return True
        except PerfDecodeError:
            return False

    if app_alive():
        return
    for _attempt in range(2):
        throttle.quiesce()  # drop the signal so the line is driven low
        time.sleep(0.2)     # let the output state settle
        dbg.reset_run()
        deadline = time.monotonic() + 8.0  # boot + arming tune
        while time.monotonic() < deadline:
            time.sleep(0.25)
            if app_alive():
                return
    raise RuntimeError(
        "ESC app never came up (hwci_perf magic invalid after reset). "
        "Either the flashed firmware was built without HWCI_PERF=1, or the "
        "ESC is stuck in the AM32 bootloader because the throttle signal "
        "line idles high at reset (check the stand's ESC output wiring and "
        "that the throttle backend can drive it).")


class _SerialTelemetry:
    """Background thread holding the most recent KISS frame from a serial port."""

    def __init__(self, port: str, baud: int):
        import serial
        self._ser = serial.Serial(port, baud, timeout=0.05)
        self._stream = KissStream()
        self._latest: KissFrame | None = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self) -> None:
        while not self._stop.is_set():
            chunk = self._ser.read(64)
            if chunk:
                for frame in self._stream.feed(chunk):
                    self._latest = frame

    def latest(self) -> KissFrame | None:
        return self._latest

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)
        self._ser.close()


def build_live_sources(rig: RigConfig, profile: Profile, *,
                       battery_cells: int | None = None,
                       min_cell_voltage: float = DEFAULT_MIN_CELL_VOLTAGE,
                       tare: bool = True) -> Sources:
    """Wire OpenOCD + serial telemetry + gRPC/external throttle on the rig.

    Backend values dispatch STRICTLY: an unknown value raises instead of
    falling back to a simulator (fabricated data gating a hardware run is the
    one unrecoverable failure mode). ``none`` disables a channel explicitly.

    If ``battery_cells`` is given, pack voltage is checked against
    ``battery_cells * min_cell_voltage`` before the throttle is armed (see
    :func:`check_battery`) - the run refuses to start on a pack that's
    already too low.

    Unless ``tare`` is False, a stand's load cells are zeroed as the last
    pre-flight step via :func:`tare_for_run` (ESC signal up at zero throttle
    first, so AM32's beacon/arm beeps don't shake the cells mid-tare). A tare
    that fails aborts the run: after any mechanical change an untared cell
    reads a bogus thrust offset, which is worse than no run.
    """
    closers: list = []

    # --- thrust stand ---
    stand: ThrustStand | None
    if rig.stand_backend == "grpc":
        from .flightstand.grpc_client import FlightStandGrpc, SignalMap
        sig = SignalMap(**rig.stand_signals) if rig.stand_signals else SignalMap()
        stand = FlightStandGrpc(rig.stand_host, rig.stand_port, signals=sig).open()
        stand.set_safety_limits(profile.safety)
        closers.append(stand.close)
    elif rig.stand_backend == "none":
        stand = None
    else:
        raise ValueError(
            f"stand_backend {rig.stand_backend!r} is not a live backend "
            "(expected 'grpc' or 'none')")

    # --- throttle source ---
    if rig.throttle_backend == "external":
        from .throttle.external import ExternalSerialThrottle
        throttle = ExternalSerialThrottle(rig.throttle_port, rig.throttle_baud)
    elif rig.throttle_backend == "flightstand":
        if stand is None:
            raise ValueError(
                "throttle_backend 'flightstand' needs stand_backend 'grpc'")
        from .throttle.flightstand_src import FlightStandThrottle
        throttle = FlightStandThrottle(stand, arm_settle_s=profile.arm_settle_s)
    else:
        raise ValueError(
            f"throttle_backend {rig.throttle_backend!r} is not a live backend "
            "(expected 'flightstand' or 'external')")
    closers.append(throttle.close)

    # --- perf struct via debugger ---
    perf_reader = None
    perf_source: PerfSourceFn = lambda: None
    if rig.debugger_backend == "openocd":
        from .debugger.openocd import OpenOcdDebugger
        elf = rig.resolved_elf()
        if elf is None:
            raise FileNotFoundError(
                f"no ELF for target {rig.target} in {rig.resolved_obj_dir()}; "
                "build with HWCI_PERF=1 first")
        dbg = OpenOcdDebugger(rig.openocd_configs, openocd_bin=rig.openocd_bin,
                              search_dirs=rig.openocd_search_dirs).open()
        perf_reader = PerfReader(dbg, str(elf))
        perf_source = perf_reader.read
        closers.append(dbg.close)
    elif rig.debugger_backend != "none":
        raise ValueError(
            f"debugger_backend {rig.debugger_backend!r} is not a live backend "
            "(expected 'openocd' or 'none')")

    # --- ESC telemetry ---
    telem_source: TelemSourceFn = lambda: None
    if rig.telem_backend == "serial":
        telem = _SerialTelemetry(rig.telem_port, rig.telem_baud)
        telem_source = telem.latest
        closers.append(telem.close)
    elif rig.telem_backend != "none":
        raise ValueError(
            f"telem_backend {rig.telem_backend!r} is not a live backend "
            "(expected 'serial' or 'none')")

    sources = Sources(throttle=throttle, stand=stand, perf_source=perf_source,
                      telem_source=telem_source, perf_reader=perf_reader,
                      closers=closers)
    try:
        if perf_reader is not None:
            _ensure_app_alive(dbg, perf_reader, throttle)
        if battery_cells is not None:
            check_battery(_live_voltage(stand, perf_source), battery_cells,
                         min_cell_voltage)
        if tare and stand is not None:
            residual_gf = tare_for_run(stand, throttle)
            if residual_gf is not None:
                print(f"tared load cells (ESC armed quiet at zero): "
                      f"residual {residual_gf:+.1f} gf", file=sys.stderr)
    except Exception:
        sources.close()
        raise
    return sources
