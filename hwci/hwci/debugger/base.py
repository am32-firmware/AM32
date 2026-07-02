"""Debugger backend abstraction.

A :class:`Debugger` flashes firmware and performs *background* (non-halting)
memory reads/writes over SWD while the MCU runs. That background access is the
only way to read CPU-load / loop-time data off the STM32F051, whose Cortex-M0
core has no SWO/ITM/DWT trace hardware.

Both the OpenOCD (ST-Link) backend and a J-Link backend would implement this
interface; :class:`MockDebugger` implements it in-process for offline tests and
simulator runs.
"""
from __future__ import annotations

import abc


class DebuggerError(RuntimeError):
    pass


class Debugger(abc.ABC):
    @abc.abstractmethod
    def flash(self, bin_path: str, load_addr: int) -> None:
        """Program ``bin_path`` at ``load_addr`` and reset into it."""

    @abc.abstractmethod
    def read_memory(self, addr: int, length: int) -> bytes:
        """Read ``length`` bytes from target RAM without halting the core."""

    @abc.abstractmethod
    def write_u32(self, addr: int, value: int) -> None:
        """Write a 32-bit word to target RAM without halting the core."""

    def reset_run(self) -> None:  # optional
        pass

    def close(self) -> None:  # optional
        pass

    def __enter__(self) -> "Debugger":
        return self

    def __exit__(self, *exc) -> None:
        self.close()


class MockDebugger(Debugger):
    """In-memory debugger backed by a flat byte buffer.

    Used by the simulator and unit tests. ``base`` is the address that maps to
    the start of the buffer; reads/writes outside the buffer raise.
    """

    def __init__(self, base: int = 0x20000000, size: int = 0x2000):
        self.base = base
        self.mem = bytearray(size)
        self.flashed: list[tuple[str, int]] = []

    # --- helpers for tests/simulator ---------------------------------
    def poke(self, addr: int, data: bytes) -> None:
        off = addr - self.base
        if off < 0 or off + len(data) > len(self.mem):
            raise DebuggerError(f"address 0x{addr:08x} out of mock range")
        self.mem[off:off + len(data)] = data

    # --- Debugger interface ------------------------------------------
    def flash(self, bin_path: str, load_addr: int) -> None:
        self.flashed.append((bin_path, load_addr))

    def read_memory(self, addr: int, length: int) -> bytes:
        off = addr - self.base
        if off < 0 or off + length > len(self.mem):
            raise DebuggerError(f"read 0x{addr:08x}+{length} out of mock range")
        return bytes(self.mem[off:off + length])

    def write_u32(self, addr: int, value: int) -> None:
        self.poke(addr, (value & 0xFFFFFFFF).to_bytes(4, "little"))
