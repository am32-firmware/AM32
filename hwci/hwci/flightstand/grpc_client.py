"""Real Flight Stand gRPC backend.

The Tyto Robotics Flight Stand Software exposes a gRPC API (you install
``grpcio``/``grpcio-tools`` and generate Python stubs from their ``.proto``).
The model is: a *board* exposes numbered *input* signals (e.g. thrust FZ = 11)
and *output* actuators (the ESC/servo throttle).

Because the upstream ``.proto`` ships with the Flight Stand Software (and is not
redistributed here), the few proto-specific calls are isolated in
:class:`_StubAdapter`. Generate the stubs during rig setup (see hwci/README.md)
into a package and pass its module names; everything above the adapter -- the
:class:`~hwci.flightstand.base.ThrustStand` interface the runner uses -- is
backend-agnostic.

Until the stubs are generated this backend raises a clear, actionable error;
the simulator backend is the default so development is never blocked.
"""
from __future__ import annotations

import importlib
import time
from dataclasses import dataclass

from .base import SafetyLimits, StandSample, ThrustStand


@dataclass
class SignalMap:
    """Maps StandSample fields to the board's numbered input signals.

    Confirm these against your stand: open the ``.proto`` (or print the board's
    inputs via the upstream ``example.py``) and read off the signal numbers.
    Thrust (FZ) = 11 is documented by Tyto; the rest are placeholders to verify.
    """
    thrust: int = 11
    torque: int | None = None
    rpm: int | None = None
    voltage: int | None = None
    current: int | None = None
    # Output (actuator) used to command throttle, and its raw range.
    esc_output: int = 0
    esc_min: float = 1000.0   # e.g. PWM microseconds; or DShot/normalized units
    esc_max: float = 2000.0
    thrust_is_grams: bool = True   # many stands report gram-force, not Newtons


G0 = 9.80665


class _StubAdapter:
    """The only proto-aware surface. Reconcile names with the actual stubs."""

    def __init__(self, host: str, port: int, pb2: str, pb2_grpc: str):
        try:
            self._pb2 = importlib.import_module(pb2)
            self._pb2_grpc = importlib.import_module(pb2_grpc)
            import grpc
        except ImportError as e:
            raise RuntimeError(
                "Flight Stand gRPC stubs / grpcio not importable "
                f"({e}). Generate them during setup:\n"
                "  pip install grpcio grpcio-tools\n"
                "  python -m grpc_tools.protoc -I<proto_dir> "
                "--python_out=<pkg> --grpc_out=<pkg> <proto_dir>/*.proto\n"
                "then pass pb2/pb2_grpc module names to FlightStandGrpc.") from e
        self._grpc = grpc
        self._channel = grpc.insecure_channel(f"{host}:{port}")
        # NOTE: stub class name comes from the service name in the .proto.
        self._stub = self._make_stub()

    # --- proto-specific operations (verify against the shipped .proto) ----
    def _make_stub(self):
        # The generated grpc module exposes one <ServiceName>Stub class.
        stub_classes = [v for k, v in vars(self._pb2_grpc).items()
                        if k.endswith("Stub")]
        if not stub_classes:
            raise RuntimeError("no *Stub class found in grpc stubs module")
        return stub_classes[0](self._channel)

    def list_boards(self):
        """Return available boards. Map to the proto's list/discover RPC."""
        raise NotImplementedError(
            "map _StubAdapter.list_boards() to your proto's board-list RPC")

    def connect_board(self, board) -> None:
        raise NotImplementedError(
            "map _StubAdapter.connect_board() to your proto's connect RPC")

    def read_signal(self, signal_id: int) -> float:
        raise NotImplementedError(
            "map _StubAdapter.read_signal() to your proto's sample RPC")

    def set_output(self, output_id: int, value: float) -> None:
        raise NotImplementedError(
            "map _StubAdapter.set_output() to your proto's set-output RPC")


class FlightStandGrpc(ThrustStand):
    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 50051,
        *,
        signals: SignalMap | None = None,
        pb2_module: str = "flightstand_pb2",
        pb2_grpc_module: str = "flightstand_pb2_grpc",
        board_index: int = 0,
    ):
        self.host = host
        self.port = port
        self.signals = signals or SignalMap()
        self._pb2_module = pb2_module
        self._pb2_grpc_module = pb2_grpc_module
        self._board_index = board_index
        self._api: _StubAdapter | None = None
        self._throttle = 0.0

    def open(self) -> "FlightStandGrpc":
        self._api = _StubAdapter(self.host, self.port,
                                 self._pb2_module, self._pb2_grpc_module)
        boards = self._api.list_boards()
        self._api.connect_board(boards[self._board_index])
        return self

    def set_throttle(self, throttle: float) -> None:
        throttle = max(0.0, min(1.0, throttle))
        self._throttle = throttle
        s = self.signals
        raw = s.esc_min + throttle * (s.esc_max - s.esc_min)
        self._api.set_output(s.esc_output, raw)

    def _read(self, signal_id: int | None) -> float:
        if signal_id is None:
            return 0.0
        return self._api.read_signal(signal_id)

    def read_sample(self) -> StandSample:
        s = self.signals
        thrust = self._read(s.thrust)
        thrust_n = thrust * G0 / 1000.0 if s.thrust_is_grams else thrust
        return StandSample(
            t=time.monotonic(),
            throttle=self._throttle,
            thrust_n=thrust_n,
            torque_nm=self._read(s.torque),
            rpm=self._read(s.rpm),
            voltage_v=self._read(s.voltage),
            current_a=self._read(s.current),
        )

    def set_safety_limits(self, limits: SafetyLimits) -> None:
        # The Flight Stand Software also enforces cutoffs configured in its UI;
        # mirror critical ones here once the set-limit RPC is mapped.
        self._limits = limits

    def close(self) -> None:
        if self._api is not None:
            try:
                self._api.set_output(self.signals.esc_output, self.signals.esc_min)
            except Exception:
                pass
