"""Pre-flight battery check: refuse to arm a test on a pack that's already
too low for its declared cell count (see check_battery/_live_voltage in
hwci/runner.py, wired into build_live_sources)."""
import pytest

from hwci.perf import PerfSample
from hwci.runner import (DEFAULT_MIN_CELL_VOLTAGE, BatteryTooLowError,
                         _live_voltage, check_battery)


class _FakeStand:
    def __init__(self, voltage_v):
        self._voltage_v = voltage_v

    def read_sample(self):
        return type("Sample", (), {"voltage_v": self._voltage_v})()


def _boom():
    raise RuntimeError("SWD gone")


# --------------------------------------------------------------------------
# check_battery: pure threshold logic
# --------------------------------------------------------------------------
def test_healthy_6s_pack_passes():
    check_battery(24.9, battery_cells=6)  # 4.15 V/cell - matches bench baseline


def test_6s_pack_below_default_cutoff_raises():
    with pytest.raises(BatteryTooLowError, match="6S"):
        check_battery(18.5, battery_cells=6)  # 3.08 V/cell < 3.3 V/cell default


def test_error_message_names_actual_and_minimum_voltage():
    with pytest.raises(BatteryTooLowError) as exc_info:
        check_battery(18.5, battery_cells=6)
    msg = str(exc_info.value)
    assert "18.50" in msg
    assert f"{6 * DEFAULT_MIN_CELL_VOLTAGE:.2f}" in msg


def test_unverifiable_voltage_fails_closed():
    with pytest.raises(BatteryTooLowError, match="cannot verify"):
        check_battery(None, battery_cells=6)


def test_custom_cutoff_overrides_default():
    check_battery(19.0, battery_cells=6, min_cell_voltage=3.0)  # 3.17 V/cell, above 3.0 floor
    with pytest.raises(BatteryTooLowError):
        check_battery(19.0, battery_cells=6, min_cell_voltage=3.3)  # same pack, stricter floor


def test_boundary_voltage_is_not_too_low():
    # exactly at the minimum should pass (strict "<" in the implementation)
    check_battery(6 * DEFAULT_MIN_CELL_VOLTAGE, battery_cells=6)


# --------------------------------------------------------------------------
# _live_voltage: stand preferred, perf struct as fallback
# --------------------------------------------------------------------------
def test_live_voltage_reads_stand_when_present():
    stand = _FakeStand(voltage_v=22.2)
    assert _live_voltage(stand, perf_source=_boom) == 22.2  # perf never consulted


def test_live_voltage_falls_back_to_perf_without_a_stand():
    pf = PerfSample(raw={"voltage_cv": 2210})
    assert _live_voltage(None, perf_source=lambda: pf) == pytest.approx(22.10)


def test_live_voltage_none_when_no_stand_and_perf_unreadable():
    assert _live_voltage(None, perf_source=_boom) is None
