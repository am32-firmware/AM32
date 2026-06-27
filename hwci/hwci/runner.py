"""Test runner: execute a profile, sampling stand + ESC telemetry + perf struct.

The runner is source-agnostic: it drives a :class:`ThrottleSource`, reads a
:class:`ThrustStand`, and calls two callables for the perf struct and ESC
telemetry. :func:`build_sim_sources` wires all of these to one shared
:class:`~hwci.sim.RigSimulator` for offline runs; :func:`build_live_sources`
wires them to OpenOCD + serial + the gRPC stand on the rig.
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Optional

from . import perf
from .config import Profile, RigConfig
from .esc_telem.kiss import KissFrame, KissStream, parse_frame
from .flightstand.base import StandSample, ThrustStand
from .flightstand.simulator import StandSafetyTripped
from .model import RunResult, make_row
from .perf import PerfSample
from .perf_reader import PerfReader
from .throttle.base import ThrottleSource

PerfSourceFn = Callable[[], Optional[PerfSample]]
TelemSourceFn = Callable[[], Optional[KissFrame]]


@dataclass
class Sources:
    throttle: ThrottleSource
    stand: ThrustStand
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


def run_profile(profile: Profile, sources: Sources, *,
                realtime: bool = True, meta: dict | None = None) -> RunResult:
    period = 1.0 / profile.sample_rate_hz
    rows: list[dict] = []
    aborted: str | None = None

    sources.throttle.arm()
    if sources.perf_reader is not None:
        sources.perf_reader.reset_stats()  # measure worst-case for THIS run

    start = time.monotonic()
    tick = 0
    prev_throttle = 0.0
    try:
        for seg in profile.segments:
            n = max(1, round(seg.duration_s * profile.sample_rate_hz))
            for i in range(n):
                t = tick * period
                if realtime:
                    target = start + t
                    delay = target - time.monotonic()
                    if delay > 0:
                        time.sleep(delay)
                throttle = _segment_throttle(seg, i, n, prev_throttle)
                sources.throttle.set(throttle)
                stand = sources.stand.read_sample()
                pf = _safe(sources.perf_source)
                tm = _safe(sources.telem_source)
                rows.append(make_row(t, seg.label, throttle, stand, tm, pf))
                tick += 1
            prev_throttle = seg.throttle
    except StandSafetyTripped as e:
        aborted = f"safety: {e}"
    finally:
        sources.throttle.disarm()

    full_meta = {
        "profile": profile.name,
        "sample_rate_hz": profile.sample_rate_hz,
        "pole_pairs": profile.pole_pairs,
        "n_samples": len(rows),
        "aborted": aborted,
        "wall_time_s": round(time.monotonic() - start, 3),
    }
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
    sim = RigSimulator(params=MotorParams(pole_pairs=profile.pole_pairs,
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
    """Wire OpenOCD + serial telemetry + gRPC/external throttle on the rig."""
    closers: list = []

    # --- thrust stand ---
    if rig.stand_backend == "grpc":
        from .flightstand.grpc_client import FlightStandGrpc, SignalMap
        sig = SignalMap(**rig.stand_signals) if rig.stand_signals else SignalMap()
        stand = FlightStandGrpc(rig.stand_host, rig.stand_port, signals=sig).open()
    else:
        from .flightstand.simulator import SimulatedStand
        stand = SimulatedStand().open()
    stand.set_safety_limits(profile.safety)
    closers.append(stand.close)

    # --- throttle source ---
    if rig.throttle_backend == "external":
        from .throttle.external import ExternalSerialThrottle
        throttle = ExternalSerialThrottle(rig.throttle_port, rig.throttle_baud)
    else:
        from .throttle.flightstand_src import FlightStandThrottle
        throttle = FlightStandThrottle(stand, arm_settle_s=profile.arm_settle_s)
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

    # --- ESC telemetry ---
    telem_source: TelemSourceFn = lambda: None
    if rig.telem_backend == "serial":
        telem = _SerialTelemetry(rig.telem_port, rig.telem_baud)
        telem_source = telem.latest
        closers.append(telem.close)

    return Sources(throttle=throttle, stand=stand, perf_source=perf_source,
                   telem_source=telem_source, perf_reader=perf_reader,
                   closers=closers)
