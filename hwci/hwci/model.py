"""Run data model: flat, columnar samples plus run metadata.

A run is a list of flat row dicts (one per sample tick) over a fixed column set,
serialized as ``samples.csv`` next to a ``meta.json``. Keeping rows flat avoids
a pandas dependency and makes the metric math trivial with numpy.
"""
from __future__ import annotations

import csv
import json
from dataclasses import dataclass, field
from pathlib import Path

from .esc_telem.kiss import KissFrame
from .flightstand.base import StandSample
from .perf import PerfSample

COLUMNS = [
    "t", "segment", "throttle_cmd",
    # thrust stand
    "stand_thrust_n", "stand_thrust_gf", "stand_torque_nm", "stand_rpm",
    "stand_voltage_v", "stand_current_a", "stand_elec_power_w",
    "stand_eff_gf_per_w", "stand_motor_temp_c", "stand_fet_temp_c",
    # ESC KISS telemetry
    "esc_erpm", "esc_voltage_v", "esc_current_a", "esc_temp_c",
    # firmware perf struct (perf_host_t = host monotonic clock at the actual
    # SWD read, so counter-rate math is immune to sample-loop scheduling jitter)
    "perf_host_t",
    "perf_ctrl_exec_us_last", "perf_ctrl_exec_us_max",
    "perf_ctrl_period_us_max", "perf_ctrl_period_us_min",
    "perf_main_loop_us_max", "perf_loop_iters", "perf_zero_cross_count",
    "perf_commutation_interval", "perf_commutation_interval_max",
    # zero-cross jitter accumulators (struct v2+; blank when the flashed
    # firmware predates them - metrics treat blank as "metric unavailable")
    "perf_zc_count", "perf_zc_jitter_sum", "perf_zc_interval_sum",
    "perf_zc_jitter_max",
    "perf_bemf_timeout", "perf_e_rpm",
    # ESC input/arming state (proves the ESC decoded the throttle protocol)
    "perf_input", "perf_armed", "perf_running",
]


def make_row(t: float, segment: str, throttle_cmd: float,
             stand: StandSample | None,
             telem: KissFrame | None,
             perf: PerfSample | None) -> dict:
    row = {c: "" for c in COLUMNS}
    row["t"] = round(t, 6)
    row["segment"] = segment
    row["throttle_cmd"] = round(throttle_cmd, 4)
    if stand is not None:
        row.update(
            stand_thrust_n=round(stand.thrust_n, 5),
            stand_thrust_gf=round(stand.thrust_gf, 3),
            stand_torque_nm=round(stand.torque_nm, 6),
            stand_rpm=round(stand.rpm, 1),
            stand_voltage_v=round(stand.voltage_v, 3),
            stand_current_a=round(stand.current_a, 3),
            stand_elec_power_w=round(stand.elec_power_w, 3),
            stand_eff_gf_per_w=round(stand.efficiency_gf_per_w, 4),
        )
        if stand.motor_temp_c is not None:
            row["stand_motor_temp_c"] = round(stand.motor_temp_c, 2)
        if stand.fet_temp_c is not None:
            row["stand_fet_temp_c"] = round(stand.fet_temp_c, 2)
    if telem is not None:
        row.update(
            esc_erpm=telem.e_rpm,
            esc_voltage_v=round(telem.voltage_v, 3),
            esc_current_a=round(telem.current_a, 3),
            esc_temp_c=telem.temperature_c,
        )
    if perf is not None:
        r = perf.raw
        if perf.host_monotonic is not None:
            row["perf_host_t"] = round(perf.host_monotonic, 6)
        row.update(
            perf_ctrl_exec_us_last=r["ctrl_exec_us_last"],
            perf_ctrl_exec_us_max=r["ctrl_exec_us_max"],
            perf_ctrl_period_us_max=r["ctrl_period_us_max"],
            perf_ctrl_period_us_min=r["ctrl_period_us_min"],
            perf_main_loop_us_max=r["main_loop_us_max"],
            perf_loop_iters=r["loop_iters"],
            perf_zero_cross_count=r["zero_cross_count"],
            perf_commutation_interval=r["commutation_interval"],
            perf_commutation_interval_max=r["commutation_interval_max"],
            perf_bemf_timeout=r["bemf_timeout_state"],
            perf_e_rpm=perf.e_rpm,
            perf_input=r["input"],
            perf_armed=r["armed"],
            perf_running=r["running"],
        )
        if "zc_count" in r:  # struct v2+
            row.update(
                perf_zc_count=r["zc_count"],
                perf_zc_jitter_sum=r["zc_jitter_sum"],
                perf_zc_interval_sum=r["zc_interval_sum"],
                perf_zc_jitter_max=r["zc_jitter_max"],
            )
    return row


@dataclass
class RunResult:
    meta: dict = field(default_factory=dict)
    rows: list[dict] = field(default_factory=list)

    def save(self, run_dir: str | Path) -> Path:
        run_dir = Path(run_dir)
        run_dir.mkdir(parents=True, exist_ok=True)
        with open(run_dir / "samples.csv", "w", newline="") as fh:
            writer = csv.DictWriter(fh, fieldnames=COLUMNS)
            writer.writeheader()
            writer.writerows(self.rows)
        with open(run_dir / "meta.json", "w") as fh:
            json.dump(self.meta, fh, indent=2, sort_keys=True)
        return run_dir

    @classmethod
    def load(cls, run_dir: str | Path) -> "RunResult":
        run_dir = Path(run_dir)
        meta = json.loads((run_dir / "meta.json").read_text())
        rows: list[dict] = []
        with open(run_dir / "samples.csv", newline="") as fh:
            for raw in csv.DictReader(fh):
                rows.append({k: _coerce(v) for k, v in raw.items()})
        return cls(meta=meta, rows=rows)


def _coerce(value: str):
    if value == "" or value is None:
        return None
    try:
        f = float(value)
        return int(f) if f.is_integer() else f
    except ValueError:
        return value
