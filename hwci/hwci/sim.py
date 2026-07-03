"""Offline rig simulator.

Produces self-consistent data across all three measurement channels from a
single throttle command:

  * thrust-stand sample (thrust, torque, RPM, V, A)
  * ESC KISS telemetry frame (temp, V, A, consumption, eRPM)
  * firmware ``hwci_perf`` struct bytes (loop times, counters, snapshot)

It is a phenomenological model, not a high-fidelity one - enough to exercise the
whole harness (metrics, baseline, report, runner) without hardware and to make
demag-detection logic testable by injecting desync events on aggressive steps.
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass, field

from . import perf
from .esc_telem.kiss import encode_frame
from .flightstand.base import StandSample


@dataclass
class MotorParams:
    pole_pairs: int = 7
    kv: float = 1900.0                 # rpm / V
    batt_voltage: float = 16.8         # 4S nominal
    internal_resistance: float = 0.02  # ohm (battery + wiring)
    throttle_rpm_fraction: float = 0.85
    ct: float = 1.6e-8                 # thrust [N] = ct * rpm^2
    cq: float = 1.43e-10               # torque [Nm] = cq * rpm^2
    motor_efficiency: float = 0.82
    idle_power_w: float = 3.0
    tau_s: float = 0.08               # spool-up time constant
    ambient_temp_c: float = 25.0
    # demag behaviour
    demag_prone: bool = False
    demag_step_threshold: float = 0.35   # throttle jump that can desync
    demag_current_a: float = 18.0        # current above which a jump desyncs
    desync_ticks: int = 8


@dataclass
class RigSimulator:
    params: MotorParams = field(default_factory=MotorParams)
    seed: int = 1234
    noise: float = 0.01  # fractional measurement noise

    def __post_init__(self):
        self._rng = random.Random(self.seed)
        self.rpm = 0.0
        self.throttle = 0.0
        self._prev_cmd = 0.0
        self.consumption_mah = 0.0
        self.temp_c = self.params.ambient_temp_c
        self.loop_iters = 0
        self.zero_cross_count = 0
        self.update_count = 0
        self.desync_remaining = 0
        self.desync_count = 0
        self._ctrl_exec_max = 0
        self._commutation_max = 0
        # zero-cross jitter accumulators (perf struct v2)
        self.zc_count = 0
        self.zc_jitter_sum = 0
        self.zc_interval_sum = 0
        self.zc_jitter_max = 0

    # --- model -------------------------------------------------------
    def _rpm_max(self) -> float:
        p = self.params
        return p.kv * p.batt_voltage * p.throttle_rpm_fraction

    def step(self, dt: float, throttle: float) -> None:
        p = self.params
        throttle = max(0.0, min(1.0, throttle))
        cmd_jump = throttle - self._prev_cmd

        # Detect a demag/desync trigger: a big, fast throttle increase into a
        # high-current regime on a demag-prone motor.
        target_rpm = throttle * self._rpm_max()
        provisional_current = self._current_for_rpm(target_rpm, p.batt_voltage)
        if (p.demag_prone and self.desync_remaining == 0
                and cmd_jump > p.demag_step_threshold
                and provisional_current > p.demag_current_a):
            self.desync_remaining = p.desync_ticks
            self.desync_count += 1

        if self.desync_remaining > 0:
            # Loss of sync: rotor falls back, electrical drive flails.
            self.rpm *= 0.7
            self.desync_remaining -= 1
        else:
            alpha = 1.0 - math.exp(-dt / max(p.tau_s, 1e-3))
            self.rpm += (target_rpm - self.rpm) * alpha

        self.rpm = max(self.rpm, 0.0)
        self.throttle = throttle
        self._prev_cmd = throttle

        # bookkeeping that the perf struct exposes
        erev_per_s = self.rpm * p.pole_pairs / 60.0
        n_comm = int(erev_per_s * 6.0 * dt)
        self.zero_cross_count += n_comm
        # zero-cross jitter accumulators: firmware gates on zero_crosses >=
        # 100 (startup/desync-recovery excluded), deviation ~0.5% of the
        # commutation interval in clean running, ballooning during a desync
        if n_comm > 0 and self.zero_cross_count >= 100:
            interval = int((1.0 / (erev_per_s * 6.0)) / 0.5e-6)
            frac = 0.05 if self.desync_remaining > 0 else 0.005
            dev = max(1, int(interval * frac * self._rng.uniform(0.3, 1.7)))
            self.zc_count = (self.zc_count + n_comm) & 0xFFFFFFFF
            self.zc_jitter_sum = (self.zc_jitter_sum + dev * n_comm) & 0xFFFFFFFF
            self.zc_interval_sum = (self.zc_interval_sum + interval * n_comm) & 0xFFFFFFFF
            self.zc_jitter_max = min(max(self.zc_jitter_max, dev), 0xFFFF)
        # idle loop iterations: ~120 kHz when idle, dropping with motor load
        idle_hz = 120000.0 * (1.0 - 0.25 * throttle)
        self.loop_iters += int(idle_hz * dt)
        self.update_count += int(idle_hz * dt)
        # current draw heats the ESC slowly
        cur = self.current
        self.temp_c += (p.ambient_temp_c + 0.9 * cur - self.temp_c) * min(1.0, dt / 5.0)
        self.consumption_mah += cur * dt / 3.6  # A*s -> mAh

    def _current_for_rpm(self, rpm: float, v: float) -> float:
        p = self.params
        torque = p.cq * rpm * rpm
        omega = rpm * 2.0 * math.pi / 60.0
        mech = torque * omega
        elec = mech / p.motor_efficiency + p.idle_power_w
        return elec / max(v, 1.0)

    # --- derived measurements ---------------------------------------
    @property
    def thrust_n(self) -> float:
        return self.params.ct * self.rpm * self.rpm

    @property
    def torque_nm(self) -> float:
        return self.params.cq * self.rpm * self.rpm

    @property
    def current(self) -> float:
        p = self.params
        cur = self._current_for_rpm(self.rpm, p.batt_voltage)
        if self.desync_remaining > 0:
            cur *= 1.6  # current spike during desync
        return cur

    @property
    def voltage(self) -> float:
        p = self.params
        return p.batt_voltage - self.current * p.internal_resistance

    @property
    def e_rpm(self) -> float:
        return self.rpm * self.params.pole_pairs

    def _n(self, value: float) -> float:
        """Apply multiplicative measurement noise."""
        if self.noise <= 0:
            return value
        return value * (1.0 + self._rng.uniform(-self.noise, self.noise))

    # --- channel outputs --------------------------------------------
    def stand_sample(self, t: float) -> StandSample:
        return StandSample(
            t=t,
            throttle=self.throttle,
            thrust_n=self._n(self.thrust_n),
            torque_nm=self._n(self.torque_nm),
            rpm=self._n(self.rpm),
            voltage_v=self._n(self.voltage),
            current_a=self._n(self.current),
        )

    def kiss_bytes(self) -> bytes:
        return encode_frame(
            temperature_c=int(self.temp_c),
            voltage_cv=int(self.voltage * 100),
            current_ca=int(self.current * 100),
            consumption_mah=int(self.consumption_mah),
            erpm100=int(self.e_rpm / 100),
        )

    def perf_bytes(self) -> bytes:
        p = self.params
        erev_per_s = self.e_rpm / 60.0
        commutations_per_s = erev_per_s * 6.0
        if commutations_per_s > 1.0:
            comm_interval = int((1.0 / commutations_per_s) / 0.5e-6)
        else:
            comm_interval = 0xFFFF
        if self.desync_remaining > 0:
            comm_interval = min(0xFFFFFF, comm_interval * 8)
        self._commutation_max = max(self._commutation_max, comm_interval)
        ctrl_exec = 16 + int(self._rng.uniform(0, 6))
        self._ctrl_exec_max = max(self._ctrl_exec_max, ctrl_exec)
        return perf.encode({
            "ctrl_exec_us_last": ctrl_exec,
            "ctrl_exec_us_max": self._ctrl_exec_max,
            "ctrl_period_us_last": 50,
            "ctrl_period_us_max": 52,
            "ctrl_period_us_min": 48,
            "main_loop_us_last": 5,
            "main_loop_us_max": 9,
            "input": int(self.throttle * 2000),
            "duty_cycle": int(self.throttle * 2000),
            "e_rpm": int(self.e_rpm / 100) & 0xFFFF,
            "voltage_cv": int(self.voltage * 100) & 0xFFFF,
            "current_ca": int(self.current * 100) & 0x7FFF,
            "temperature_c": int(self.temp_c),
            "bemf_timeout_state": 1 if self.desync_remaining > 0 else 0,
            "armed": 1,
            "running": 1 if self.rpm > 100 else 0,
            "loop_iters": self.loop_iters & 0xFFFFFFFF,
            # firmware clamps zero_crosses at 10000 (and resets it on
            # desync/stop) - mirror the saturation so host logic tested
            # against the sim cannot assume a monotonic counter
            "zero_cross_count": min(self.zero_cross_count, 10000),
            "commutation_interval": comm_interval,
            "commutation_interval_max": self._commutation_max,
            "update_count": self.update_count & 0xFFFFFFFF,
            "host_cmd": 0,
            "zc_count": self.zc_count,
            "zc_jitter_sum": self.zc_jitter_sum,
            "zc_interval_sum": self.zc_interval_sum,
            "zc_jitter_max": self.zc_jitter_max,
        })
