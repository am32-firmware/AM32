"""DShot throttle mapping: zero throttle must be DShot 0, never 1-47.

AM32 only arms on sustained DShot 0 (verified on the ARK 4IN1 bench: at a
constant DShot 48 the ESC decodes input=48 but never arms, Src/main.c requires
adjusted_input == 0 for one second). Values 1-47 are DShot COMMANDS - a ramp
must never sweep through them.
"""
from __future__ import annotations

import pytest

from hwci.flightstand.grpc_client import FlightStandGrpc, SignalMap


class RecordingApi:
    def __init__(self):
        self.calls = []

    def set_output(self, output_id, value, *, active=True):
        self.calls.append((output_id, value, active))


def make_stand(**signal_overrides) -> tuple[FlightStandGrpc, RecordingApi]:
    stand = FlightStandGrpc(signals=SignalMap(**signal_overrides))
    stand._api = RecordingApi()
    return stand, stand._api


def test_dshot_zero_throttle_is_dshot_zero():
    stand, api = make_stand(esc_zero=0.0, esc_min=48.0, esc_max=2047.0)
    stand.set_throttle(0.0)
    assert api.calls[-1] == (0, 0.0, True)


def test_dshot_positive_throttle_starts_at_48_never_commands():
    stand, api = make_stand(esc_zero=0.0, esc_min=48.0, esc_max=2047.0)
    for t in (1e-6, 0.001, 0.01, 0.10, 1.0):
        stand.set_throttle(t)
        raw = api.calls[-1][1]
        assert raw >= 48.0, f"throttle {t} emitted DShot command value {raw}"
    stand.set_throttle(1.0)
    assert api.calls[-1][1] == pytest.approx(2047.0)


def test_pwm_default_zero_is_esc_min():
    # esc_zero=None: standard PWM where 1000 us is both zero and idle
    stand, api = make_stand(esc_min=1000.0, esc_max=2000.0)
    stand.set_throttle(0.0)
    assert api.calls[-1][1] == pytest.approx(1000.0)
    stand.set_throttle(0.5)
    assert api.calls[-1][1] == pytest.approx(1500.0)


def test_close_parks_at_esc_zero_then_deactivates():
    stand, api = make_stand(esc_zero=0.0, esc_min=48.0, esc_max=2047.0)
    stand.close()
    assert api.calls[-2] == (0, 0.0, True)
    assert api.calls[-1] == (0, 0.0, False)
