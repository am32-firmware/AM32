"""OpenOCD (ST-Link) debugger backend.

Drives a persistent ``openocd`` process and talks to its Tcl-RPC port (6666) to
read/write target memory while the firmware runs. Cortex-M memory access goes
through the AHB-AP and does not require halting the core, which is what makes
non-intrusive CPU-load/loop-time sampling possible (the same mechanism VS Code
"live watch" uses; note ``-gdb-max-connections`` in Mcu/f051/openocd.cfg).

Flashing is done with a separate one-shot ``openocd`` invocation so the
persistent read session is never left in a halted state.

This backend cannot be unit-tested without hardware; the offline test path uses
:class:`hwci.debugger.base.MockDebugger`. The Tcl-RPC framing and command set
here follow the documented OpenOCD interface.
"""
from __future__ import annotations

import collections
import shutil
import socket
import struct
import subprocess
import threading
import time

from .base import Debugger, DebuggerError

# OpenOCD Tcl-RPC terminates every command and reply with this byte.
_RPC_SEP = b"\x1a"

# Default config matching Mcu/f051/openocd.cfg (ST-Link + STM32F0 target).
DEFAULT_CONFIGS = ["interface/stlink.cfg", "target/stm32f0x.cfg"]
APP_LOAD_ADDR = 0x08001000  # AM32 app sits above the bootloader


class OpenOcdDebugger(Debugger):
    def __init__(
        self,
        configs: list[str] | None = None,
        *,
        openocd_bin: str = "openocd",
        tcl_port: int = 6666,
        search_dirs: list[str] | None = None,
        connect_timeout: float = 10.0,
    ):
        self.configs = configs or list(DEFAULT_CONFIGS)
        self.openocd_bin = openocd_bin
        self.tcl_port = tcl_port
        self.search_dirs = search_dirs or []
        self.connect_timeout = connect_timeout
        self._proc: subprocess.Popen | None = None
        self._sock: socket.socket | None = None
        self._log_tail: collections.deque[str] = collections.deque(maxlen=200)
        self._drain_thread: threading.Thread | None = None
        # The Tcl-RPC socket carries one command/reply at a time; the perf
        # poller thread and the runner (stat resets at steady tails) both use
        # it, so serialize access or the reply framing interleaves.
        self._rpc_lock = threading.Lock()
        if shutil.which(openocd_bin) is None:
            raise DebuggerError(f"{openocd_bin!r} not found on PATH")

    # --- config helpers ----------------------------------------------
    def _base_args(self) -> list[str]:
        args = [self.openocd_bin]
        for d in self.search_dirs:
            args += ["-s", d]
        for c in self.configs:
            args += ["-f", c]
        return args

    # --- flashing (one-shot) -----------------------------------------
    def flash(self, bin_path: str, load_addr: int = APP_LOAD_ADDR) -> None:
        cmd = self._base_args() + [
            "-c", f"program {{{bin_path}}} 0x{load_addr:08x} verify reset exit",
        ]
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
        if proc.returncode != 0:
            raise DebuggerError(
                f"flash failed (rc={proc.returncode}):\n{proc.stderr}\n{proc.stdout}")

    # --- persistent read session -------------------------------------
    def open(self) -> "OpenOcdDebugger":
        """Start openocd (init, no halt) and connect to the Tcl-RPC port."""
        cmd = self._base_args() + [
            "-c", f"tcl_port {self.tcl_port}",
            "-c", "gdb_port disabled",
            "-c", "telnet_port disabled",
            "-c", "init",
            # leave the core running; only attach for background memory access
        ]
        self._proc = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        # openocd logs continuously to stdout. NOBODY reading the pipe kills
        # the session: once the 64 KiB pipe buffer fills, openocd blocks on
        # write and every Tcl-RPC call times out from then on (observed on
        # the bench as the perf channel dying at the same sample every run).
        # Drain it forever; keep a tail for error diagnostics.
        self._drain_thread = threading.Thread(
            target=self._drain_stdout, daemon=True, name="openocd-drain")
        self._drain_thread.start()
        self._connect_rpc()
        # Ensure the core is running (a fresh attach can leave it halted).
        try:
            self._rpc("resume")
        except DebuggerError:
            pass
        return self

    def _drain_stdout(self) -> None:
        proc = self._proc
        if proc is None or proc.stdout is None:
            return
        try:
            for line in proc.stdout:
                self._log_tail.append(line.rstrip("\n"))
        except (OSError, ValueError):
            pass  # pipe closed on shutdown

    def _connect_rpc(self) -> None:
        deadline = time.monotonic() + self.connect_timeout
        last_err: Exception | None = None
        while time.monotonic() < deadline:
            try:
                self._sock = socket.create_connection(
                    ("127.0.0.1", self.tcl_port), timeout=2.0)
                return
            except OSError as e:  # openocd not listening yet
                last_err = e
                if self._proc and self._proc.poll() is not None:
                    time.sleep(0.1)  # let the drain thread catch the tail
                    out = "\n".join(self._log_tail)
                    raise DebuggerError(f"openocd exited early:\n{out}")
                time.sleep(0.2)
        raise DebuggerError(f"could not connect to openocd Tcl-RPC: {last_err}")

    def _rpc(self, command: str) -> str:
        with self._rpc_lock:
            return self._rpc_locked(command)

    def _rpc_locked(self, command: str) -> str:
        if self._sock is None:
            raise DebuggerError("RPC session not open; call open() first")
        try:
            self._sock.sendall(command.encode() + _RPC_SEP)
            chunks = bytearray()
            while _RPC_SEP not in chunks:
                data = self._sock.recv(4096)
                if not data:
                    raise DebuggerError("openocd RPC closed unexpectedly")
                chunks += data
        except (socket.timeout, OSError) as e:
            # One glitched command desyncs the reply framing (the late reply
            # would be read as the answer to the NEXT command), so a single
            # hiccup would poison every subsequent read for the rest of the
            # run. Rebuild the socket - openocd itself is still fine - and
            # surface the error for this call only (observed on the bench:
            # one SWD read timeout at motor spin-up killed the whole perf
            # channel).
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None
            self._connect_rpc()
            raise DebuggerError(f"openocd RPC error for {command!r}: {e}") from e
        return chunks.split(_RPC_SEP, 1)[0].decode(errors="replace")

    # --- Debugger interface ------------------------------------------
    def read_memory(self, addr: int, length: int) -> bytes:
        nwords = (length + 3) // 4
        # read_memory returns space-separated decimal values (one per word).
        out = self._rpc(f"read_memory 0x{addr:08x} 32 {nwords}")
        tokens = out.replace("{", " ").replace("}", " ").split()
        try:
            words = [int(t, 0) for t in tokens]
        except ValueError as e:
            raise DebuggerError(f"unparseable read_memory reply {out!r}: {e}")
        if len(words) < nwords:
            raise DebuggerError(
                f"short read: wanted {nwords} words, got {len(words)} ({out!r})")
        return struct.pack(f"<{nwords}I", *words[:nwords])[:length]

    def write_u32(self, addr: int, value: int) -> None:
        # OpenOCD reports failure IN-BAND: a successful mww replies with an
        # empty string, a failed one with error text (no exception). Swallowing
        # it would let e.g. a stats-reset silently no-op.
        out = self._rpc(f"mww 0x{addr:08x} 0x{value & 0xFFFFFFFF:08x}")
        if out.strip():
            raise DebuggerError(f"mww 0x{addr:08x} failed: {out.strip()!r}")

    def reset_run(self) -> None:
        self._rpc("reset run")

    def close(self) -> None:
        try:
            if self._sock is not None:
                try:
                    self._rpc("exit")
                except DebuggerError:
                    pass
                self._sock.close()
        finally:
            self._sock = None
            if self._proc is not None:
                self._proc.terminate()
                try:
                    self._proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self._proc.kill()
                self._proc = None
