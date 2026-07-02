"""Real Flight Stand gRPC backend (Tyto Robotics Flight Stand Software API v1).

Mapped against ``flight_stand_api_v1.proto`` from
https://gitlab.com/TytoRobotics/flightstand-api (the pre-compiled Python stubs
in ``languages/python`` of that repo must be importable, e.g. via a ``.pth``
file in the venv).

The Tyto model: *boards* (USB measurement units) expose *inputs* (sensors,
identified by ``InputType``: FORCE_FZ=11 is thrust) and *outputs* (the ESC
signal, ``OutputType.ESC``). The latest value of every signal comes back in
one ``ListSamples`` round trip; throttle is commanded with ``UpdateOutput``
on the output's ``output_target`` field (a µs-style value, 1000-2000 for
standard PWM).

The Flight Stand Software runs on Windows only. For a Linux bench, run it on
a Windows PC on the same network, launched with ``--remote``, and point
``stand_host`` at that PC. Units over the API are SI: thrust in Newtons,
rotation speed in rad/s (converted to RPM here).

The numeric ids in :class:`SignalMap` ARE Tyto ``InputType`` enum values, so
a rig file maps channels without touching code; ``esc_output`` indexes the
ESC-type outputs sorted by resource name.
"""
from __future__ import annotations

import importlib
import math
import time

from dataclasses import dataclass

from .base import SafetyLimits, StandSample, ThrustStand


@dataclass
class SignalMap:
    """Maps StandSample fields to Tyto ``InputType`` enum values.

    Defaults follow the proto: FORCE_FZ=11 thrust, TORQUE_MZ=14, ROTATION_
    SPEED_FREQUENCY=15 (rad/s), VOLTAGE_HV_INPUT=6, CURRENT_HALL_CURRENT=8.
    Set a channel to ``null``/None in the rig file if the bench lacks it;
    reads then report 0.0 and the metrics coverage gate flags it.
    """
    thrust: int = 11
    torque: int | None = 14
    rpm: int | None = 15
    voltage: int | None = 6
    current: int | None = 8
    # Optional temperature channels. Values may be an InputType enum int, or
    # a string: an input resource name ("/boards/COM3/inputs/29", stable per
    # board) or a raw signal name ("/signals/37", renumbers on reconnect -
    # prefer the input name).
    motor_temp: int | str | None = None
    fet_temp: int | str | None = None
    # Output (actuator) used to command throttle, and its raw range.
    esc_output: int = 0
    esc_min: float = 1000.0   # raw value of the LOWEST real throttle step
    esc_max: float = 2000.0   # raw value of full throttle
    # Raw value for throttle == 0. Leave None where zero throttle IS esc_min
    # (standard PWM: 1000 us). For DShot it must be 0: AM32 only arms on
    # sustained DShot 0 (what an FC sends while disarmed), values 1-47 are
    # DShot COMMANDS that must never be emitted (a ramp sweeping them could
    # trigger beacons or even settings changes), and real throttle starts at
    # 48. So DShot: esc_zero=0, esc_min=48, esc_max=2047.
    esc_zero: float | None = None
    thrust_is_grams: bool = False  # gRPC API is SI: Newtons
    rpm_is_rad_per_s: bool = True  # gRPC API reports rotation in rad/s


G0 = 9.80665
RADS_TO_RPM = 60.0 / (2.0 * math.pi)


def _k_to_c(kelvin: float | None) -> float | None:
    return kelvin - 273.15 if kelvin is not None else None


class _StubAdapter:
    """The only proto-aware surface, mapped to Tyto's FlightStand service."""

    RPC_TIMEOUT_S = 2.0

    def __init__(self, host: str, port: int, pb2: str, pb2_grpc: str):
        try:
            self._pb2 = importlib.import_module(pb2)
            self._pb2_grpc = importlib.import_module(pb2_grpc)
            from google.protobuf import field_mask_pb2
            import grpc
        except ImportError as e:
            raise RuntimeError(
                "Flight Stand gRPC stubs / grpcio not importable "
                f"({e}). Clone https://gitlab.com/TytoRobotics/flightstand-api "
                "and make languages/python importable (pip install grpcio "
                "protobuf, then add a .pth file pointing at it).") from e
        self._grpc = grpc
        self._field_mask_pb2 = field_mask_pb2
        self._channel = grpc.insecure_channel(f"{host}:{port}")
        self._stub = self._make_stub()
        self._sig_by_type: dict[int, str] = {}
        self._esc_outputs: list = []
        # Fail fast with an actionable message if the core is not running.
        try:
            self._stub.GetServerStatus(self._pb2.GetServerStatusRequest(),
                                       timeout=self.RPC_TIMEOUT_S)
        except grpc.RpcError as e:
            raise RuntimeError(
                f"Flight Stand core not reachable at {host}:{port} "
                f"({e.code().name}). Start the Flight Stand Software "
                "(with --remote if it runs on another machine).") from e
        # A cutoff latched from a previous session blocks output commands.
        try:
            self._stub.ClearCutoff(self._pb2.ClearCutoffRequest(),
                                   timeout=self.RPC_TIMEOUT_S)
        except grpc.RpcError:
            pass  # no cutoff to clear

    # --- proto-specific operations ----------------------------------------
    def _make_stub(self):
        # The generated grpc module exposes one <ServiceName>Stub class.
        stub_classes = [v for k, v in vars(self._pb2_grpc).items()
                        if k.endswith("Stub")]
        if not stub_classes:
            raise RuntimeError("no *Stub class found in grpc stubs module")
        return stub_classes[0](self._channel)

    def list_boards(self):
        """Boards that are connected and ready."""
        resp = self._stub.ListBoards(self._pb2.ListBoardsRequest(),
                                     timeout=self.RPC_TIMEOUT_S)
        boards = [b for b in resp.boards if b.ready]
        if not boards:
            raise RuntimeError(
                "Flight Stand core is running but reports no ready boards - "
                "check the stand's USB connections on the machine running "
                "the Flight Stand Software.")
        return boards

    def connect_board(self, board) -> None:
        """USB boards auto-connect; discover the signal/output mapping.

        Inputs and outputs are discovered across ALL ready boards (the
        Flight Stand 50 enumerates as separate force and power units), so
        ``board`` only anchors the error message.
        """
        ins = self._stub.ListInputs(self._pb2.ListInputsRequest(),
                                    timeout=self.RPC_TIMEOUT_S).inputs
        self._sig_by_type = {}
        self._sig_by_input_name = {}
        for i in ins:
            # First input of each type wins, matching the vendor helper's
            # find_input_by_type().
            self._sig_by_type.setdefault(int(i.input_type), i.signal_name)
            self._sig_by_input_name[i.name] = i.signal_name

        outs = self._stub.ListOutputs(self._pb2.ListOutputsRequest(),
                                      timeout=self.RPC_TIMEOUT_S).outputs
        self._esc_outputs = sorted(
            (o for o in outs if o.output_type == self._pb2.ESC and not o.closed),
            key=lambda o: o.name)
        if not self._esc_outputs:
            raise RuntimeError(
                f"no ESC output found on {board.name} (or any board) - "
                "cannot command throttle")

    def input_types_available(self) -> dict[int, str]:
        """InputType -> signal_name map discovered at connect (diagnostics)."""
        return dict(self._sig_by_type)

    def _resolve(self, sid) -> str | None:
        """Channel id -> signal name. int = InputType enum; str = input
        resource name (preferred, stable) or raw /signals/N name."""
        if isinstance(sid, str):
            if sid.startswith("/signals/"):
                return sid
            return self._sig_by_input_name.get(sid)
        return self._sig_by_type.get(sid)

    def read_signal(self, signal_id) -> float:
        return self.read_signals([signal_id]).get(signal_id, 0.0)

    def read_signals(self, signal_ids: list) -> dict:
        """Latest value for each wanted channel in ONE round trip.

        ``ListSamples`` returns the most recent sample of every signal;
        inactive samples (sensor off/disconnected) are omitted so the
        caller's 0.0 default and the coverage gate see a dead channel.
        """
        wanted = {}
        for sid in signal_ids:
            name = self._resolve(sid)
            if name is not None:
                wanted[name] = sid
        if not wanted:
            return {}
        resp = self._stub.ListSamples(self._pb2.ListSamplesRequest(),
                                      timeout=self.RPC_TIMEOUT_S)
        out: dict[int, float] = {}
        for sample in resp.sample_group.samples:
            t = wanted.get(sample.signal_name)
            if t is not None and sample.active:
                out[t] = sample.value
        return out

    def set_output(self, output_id: int, value: float, *,
                   active: bool = True) -> None:
        try:
            output = self._esc_outputs[output_id]
        except IndexError:
            raise RuntimeError(
                f"esc_output index {output_id} out of range - only "
                f"{len(self._esc_outputs)} ESC output(s) discovered")
        target = self._pb2.OutputTarget(active=active, target_value=value,
                                        rate_limit_per_second=0.0)
        mask = self._field_mask_pb2.FieldMask(paths=["output_target"])
        req = self._pb2.UpdateOutputRequest(
            output=self._pb2.Output(name=output.name, output_target=target),
            mask=mask)
        self._stub.UpdateOutput(req, timeout=self.RPC_TIMEOUT_S)

    def tare(self) -> None:
        self._stub.TareInputs(self._pb2.TareInputsRequest(),
                              timeout=30.0)  # taring takes a few seconds

    def close_channel(self) -> None:
        self._channel.close()


class FlightStandGrpc(ThrustStand):
    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 50051,
        *,
        signals: SignalMap | None = None,
        pb2_module: str = "flight_stand_api_v1_pb2",
        pb2_grpc_module: str = "flight_stand_api_v1_pb2_grpc",
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
        if throttle <= 0.0 and s.esc_zero is not None:
            raw = s.esc_zero            # e.g. DShot 0: disarm-idle/motor stop
        else:
            raw = s.esc_min + throttle * (s.esc_max - s.esc_min)
        self._api.set_output(s.esc_output, raw)

    def read_sample(self) -> StandSample:
        s = self.signals
        wanted = [sid for sid in (s.thrust, s.torque, s.rpm, s.voltage,
                                  s.current, s.motor_temp, s.fet_temp)
                  if sid is not None]
        values = self._api.read_signals(wanted) if wanted else {}

        def _get(signal_id) -> float:
            return values.get(signal_id, 0.0) if signal_id is not None else 0.0

        thrust = _get(s.thrust)
        thrust_n = thrust * G0 / 1000.0 if s.thrust_is_grams else thrust
        rpm = _get(s.rpm)
        if s.rpm_is_rad_per_s:
            rpm *= RADS_TO_RPM
        return StandSample(
            t=time.monotonic(),
            throttle=self._throttle,
            thrust_n=thrust_n,
            torque_nm=_get(s.torque),
            rpm=rpm,
            voltage_v=_get(s.voltage),
            current_a=_get(s.current),
            # None (not 0.0) when unmapped/inactive: a dead temp probe must
            # read as missing, not as a freezing motor. The API is strict SI,
            # so temperatures arrive in Kelvin.
            motor_temp_c=_k_to_c(values.get(s.motor_temp)) if s.motor_temp is not None else None,
            fet_temp_c=_k_to_c(values.get(s.fet_temp)) if s.fet_temp is not None else None,
        )

    def set_safety_limits(self, limits: SafetyLimits) -> None:
        # NOTE: this backend cannot enforce limits itself until the vendor
        # cutoff-condition RPC is mapped in _StubAdapter. Enforcement happens
        # in the runner (hwci.runner.enforce_safety) against every sample;
        # configure the Flight Stand Software's own UI cutoffs as a second
        # layer.
        self._limits = limits

    def deactivate(self) -> None:
        """Drop the ESC signal entirely (output inactive drives logic 0)."""
        self._api.set_output(self.signals.esc_output, self.signals.esc_min,
                             active=False)

    def tare(self) -> None:
        if self._api is not None:
            self._api.tare()

    def close(self) -> None:
        if self._api is not None:
            park = (self.signals.esc_zero if self.signals.esc_zero is not None
                    else self.signals.esc_min)
            try:
                # Park at zero throttle, then drop the signal entirely.
                self._api.set_output(self.signals.esc_output, park)
                self._api.set_output(self.signals.esc_output, park,
                                     active=False)
            except Exception:
                pass
            try:
                self._api.close_channel()
            except Exception:
                pass
