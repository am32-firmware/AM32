"""_ensure_app_alive: bootloader-stuck ESC recovery on live-source bring-up.

On this rig the stand's inactive ESC output leaves the signal line high, and
the AM32 bootloader then never jumps to the app after a flash/power-cycle
(observed on the ARK 4IN1 bench: perf magic reads 0x00000000, PC loops in the
bootloader). The runner must drive zero throttle, reset, and wait for the
app's magic instead of handing a dead perf channel to the run.
"""
from __future__ import annotations

import pytest

from hwci.perf import PerfDecodeError
from hwci.runner import _ensure_app_alive


class FakeReader:
    """perf_reader.read() fails until the fake target has been reset."""

    def __init__(self, alive_after_resets: int):
        self.alive_after_resets = alive_after_resets
        self.resets = 0

    def read(self):
        if self.resets >= self.alive_after_resets:
            return object()
        raise PerfDecodeError("bad magic 0x00000000")


class FakeDbg:
    def __init__(self, reader: FakeReader):
        self._reader = reader

    def reset_run(self):
        self._reader.resets += 1


class FakeThrottle:
    def __init__(self):
        self.commands = []

    def set(self, throttle):
        self.commands.append(throttle)

    def quiesce(self):
        self.commands.append("quiesce")


def test_already_alive_touches_nothing():
    reader = FakeReader(alive_after_resets=0)
    dbg, throttle = FakeDbg(reader), FakeThrottle()
    _ensure_app_alive(dbg, reader, throttle)
    assert reader.resets == 0
    assert throttle.commands == []


def test_stuck_in_bootloader_recovers_via_reset():
    reader = FakeReader(alive_after_resets=1)
    dbg, throttle = FakeDbg(reader), FakeThrottle()
    _ensure_app_alive(dbg, reader, throttle)
    assert reader.resets == 1
    # the signal must be DROPPED (not DShot-at-zero) before the reset so the
    # line is driven low and the bootloader jumps to the app
    assert throttle.commands == ["quiesce"]


def test_never_alive_raises_actionable_error(monkeypatch):
    # collapse the wait loops so the failure path is fast
    import hwci.runner as runner
    monkeypatch.setattr(runner.time, "sleep", lambda s: None)
    clock = iter(range(0, 10_000))
    monkeypatch.setattr(runner.time, "monotonic", lambda: float(next(clock)))

    reader = FakeReader(alive_after_resets=99)
    dbg, throttle = FakeDbg(reader), FakeThrottle()
    with pytest.raises(RuntimeError, match="bootloader|HWCI_PERF"):
        _ensure_app_alive(dbg, reader, throttle)
    assert reader.resets == 2  # both attempts exhausted


def test_debugger_error_propagates():
    class DeadProbeReader:
        def read(self):
            raise ConnectionError("SWD gone")

    reader = DeadProbeReader()
    with pytest.raises(ConnectionError):
        _ensure_app_alive(FakeDbg(FakeReader(0)), reader, FakeThrottle())
