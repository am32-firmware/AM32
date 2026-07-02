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
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Optional

from . import perf
from .config import Profile, RigConfig
from .esc_telem.kiss import KissFrame, KissStream, parse_frame
from .flightstand.base import SafetyLimits, StandSafetyTripped, StandSample, ThrustStand
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
                     rpm=stand.rpm, voltage_v=stand.voltage_v)
    if telem is not None:
        limits.check(current_a=telem.current_a, temp_c=telem.temperature_c)


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
            for i in range(n):
                t_sched = tick * period
                if realtime:
                    target = start + t_sched
                    delay = target - time.monotonic()
                    if delay > 0:
                        time.sleep(delay)
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
                      demag_prone: bool = True) -> Sources:
    """All three channels fed by one RigSimulator (deterministic, no hardware)."""
    from .flightstand.simulator import SimulatedStand
    from .sim import MotorParams, RigSimulator
    from .throttle.flightstand_src import FlightStandThrottle

    period = 1.0 / profile.sample_rate_hz
    sim = RigSimulator(params=MotorParams(pole_pairs=rig.pole_pairs,
                                          demag_prone=demag_prone))
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


def build_live_sources(rig: RigConfig, profile: Profile) -> Sources:
    """Wire OpenOCD + serial telemetry + gRPC/external throttle on the rig.

    Backend values dispatch STRICTLY: an unknown value raises instead of
    falling back to a simulator (fabricated data gating a hardware run is the
    one unrecoverable failure mode). ``none`` disables a channel explicitly.
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

    return Sources(throttle=throttle, stand=stand, perf_source=perf_source,
                   telem_source=telem_source, perf_reader=perf_reader,
                   closers=closers)
