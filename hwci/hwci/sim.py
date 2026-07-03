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
from .settings import Settings


@dataclass(frozen=True)
class SimSettings:
    """The AM32 settings the simulator responds to, decoded from the SAME
    192-byte blob :mod:`hwci.settings` writes to hardware - one decoder for
    both paths, so a tuner bug in blob encoding shows up offline too."""
    advance_level: int = 26
    pwm_frequency: int = 24
    variable_pwm: int = 1
    auto_advance: int = 0
    startup_power: int = 100
    max_ramp: int = 160

    @classmethod
    def from_blob(cls, blob: bytes) -> "SimSettings":
        s = Settings(blob)
        return cls(advance_level=s.get("advance_level"),
                   pwm_frequency=s.get("pwm_frequency"),
                   variable_pwm=s.get("variable_pwm"),
                   auto_advance=s.get("auto_advance"),
                   startup_power=s.get("startup_power"),
                   max_ramp=s.get("max_ramp"))


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
    # --- settings response (active only when RigSimulator.settings is set) --
    # Phenomenological, tuner-testable shapes: efficiency has an interior
    # maximum in timing advance (gaussian) and in log2 PWM frequency; the
    # optima are injectable so tests can move the "truth" and check the tuner
    # finds it.
    advance_optimum: float = 26.0        # eeprom advance_level units (10..42)
    advance_sigma: float = 8.0
    advance_eff_depth: float = 0.10      # efficiency lost far from optimum
    auto_advance_equiv: float = 18.0     # auto_advance scores as this advance
    pwm_optimum_khz: float = 24.0
    pwm_sigma_log2: float = 1.0
    pwm_eff_depth: float = 0.05
    variable_pwm_low_bonus: float = 0.015   # small efficiency bonus...
    variable_pwm_low_throttle: float = 0.45  # ...below this throttle
    variable_pwm_jitter_mult: float = 1.4    # ...at a small ZC-jitter cost
    # start attempts fail with probability (fail_ref - startup_power)/scale
    # (clipped to [0, 0.9]): 50 -> 0.5, 100 -> ~0.12, 150 -> 0.
    startup_fail_ref: float = 115.0
    startup_fail_scale: float = 130.0


@dataclass
class RigSimulator:
    params: MotorParams = field(default_factory=MotorParams)
    seed: int = 1234
    noise: float = 0.01  # fractional measurement noise
    # AM32 settings under test; None = legacy behaviour (no settings model),
    # which stays bit-identical so settings-less runs are unaffected.
    settings: SimSettings | None = None

    def __post_init__(self):
        self._rng = random.Random(self.seed)
        self._start_blocked = False
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
        # confirm-loop rejection counter (perf struct v3)
        self.zc_confirm_reject = 0
        # PWM-phase histogram of accepted ZCs (perf struct v4): the sim
        # models mild phase locking - 20% of edges pile onto one bin
        self.zc_phase_hist = [0] * 32
        self._phase_rr = 0

    # --- model -------------------------------------------------------
    def set_settings(self, settings: SimSettings | None) -> None:
        """Install the AM32 settings under test (the sim's "reboot into new
        settings"); also clears any blocked-start latch from the previous
        trial, like the real MCU reset would."""
        self.settings = settings
        self._start_blocked = False

    def _rpm_max(self) -> float:
        p = self.params
        return p.kv * p.batt_voltage * p.throttle_rpm_fraction

    def _settings_eff_factor(self) -> float:
        """Efficiency multiplier (<= 1 + small bonus) from the settings model."""
        s = self.settings
        if s is None:
            return 1.0
        p = self.params
        adv = p.auto_advance_equiv if s.auto_advance else float(s.advance_level)
        g_adv = math.exp(-((adv - p.advance_optimum) ** 2)
                         / (2.0 * p.advance_sigma ** 2))
        g_pwm = math.exp(-(math.log2(max(s.pwm_frequency, 1) / p.pwm_optimum_khz) ** 2)
                         / (2.0 * p.pwm_sigma_log2 ** 2))
        factor = ((1.0 - p.advance_eff_depth * (1.0 - g_adv))
                  * (1.0 - p.pwm_eff_depth * (1.0 - g_pwm)))
        if s.variable_pwm and self.throttle < p.variable_pwm_low_throttle:
            factor *= 1.0 + p.variable_pwm_low_bonus
        return factor

    def _demag_step_threshold(self) -> float:
        """Effective throttle-jump threshold for a desync: a low max_ramp
        slew-limits the duty step the firmware actually applies, so the jump
        needed to shed sync scales up as max_ramp comes down (deterministic
        realization of "low max_ramp reduces desync probability")."""
        p = self.params
        if self.settings is None or self.settings.max_ramp <= 0:
            return p.demag_step_threshold
        return p.demag_step_threshold * (160.0 / self.settings.max_ramp)

    def step(self, dt: float, throttle: float) -> None:
        p = self.params
        throttle = max(0.0, min(1.0, throttle))
        cmd_jump = throttle - self._prev_cmd

        # Detect a demag/desync trigger: a big, fast throttle increase into a
        # high-current regime on a demag-prone motor.
        target_rpm = throttle * self._rpm_max()

        # Startup reliability model (settings runs only): each stopped->spin
        # transition is a start attempt that fails with a probability set by
        # startup_power; a failed start leaves the rotor twitching at ~zero
        # RPM until the throttle is dropped and a new attempt begins.
        if self.settings is not None:
            if self._start_blocked:
                if throttle <= 0.02:
                    self._start_blocked = False
                target_rpm = 0.0
            elif (throttle > 0.02 and self._prev_cmd <= 0.02
                    and self.rpm < 200.0):
                p_fail = max(0.0, min(0.9, (p.startup_fail_ref
                                            - self.settings.startup_power)
                                     / p.startup_fail_scale))
                if self._rng.random() < p_fail:
                    self._start_blocked = True
                    target_rpm = 0.0

        provisional_current = self._current_for_rpm(target_rpm, p.batt_voltage)
        if (p.demag_prone and self.desync_remaining == 0
                and cmd_jump > self._demag_step_threshold()
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
            if self.settings is not None and self.settings.variable_pwm:
                frac *= p.variable_pwm_jitter_mult
            dev = max(1, int(interval * frac * self._rng.uniform(0.3, 1.7)))
            self.zc_count = (self.zc_count + n_comm) & 0xFFFFFFFF
            self.zc_jitter_sum = (self.zc_jitter_sum + dev * n_comm) & 0xFFFFFFFF
            self.zc_interval_sum = (self.zc_interval_sum + interval * n_comm) & 0xFFFFFFFF
            self.zc_jitter_max = min(max(self.zc_jitter_max, dev), 0xFFFF)
            # confirm rejects: rare in clean running, balloon during a desync.
            # Stochastic rounding: the per-tick expectation (~0.8 rejects) is
            # below 1, so a plain int() would truncate every tick to zero.
            reject_frac = 0.2 if self.desync_remaining > 0 else 0.01
            n_rej = int(n_comm * reject_frac + self._rng.random())
            self.zc_confirm_reject = (self.zc_confirm_reject + n_rej) & 0xFFFFFFFF
            # v4 phase histogram: 80% uniform across bins (round-robin the
            # integer remainder), 20% locked onto the throttle-derived bin
            peak_bin = int(self.throttle * 32) & 31
            per_bin, rem = divmod(int(n_comm * 0.8), 32)
            for i in range(32):
                self.zc_phase_hist[i] = (self.zc_phase_hist[i] + per_bin) & 0xFFFF
            self.zc_phase_hist[self._phase_rr] = (
                self.zc_phase_hist[self._phase_rr] + rem) & 0xFFFF
            self._phase_rr = (self._phase_rr + 1) & 31
            n_peak = int(n_comm * 0.2 + self._rng.random())
            self.zc_phase_hist[peak_bin] = (
                self.zc_phase_hist[peak_bin] + n_peak) & 0xFFFF
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
        elec = mech / (p.motor_efficiency * self._settings_eff_factor()) \
            + p.idle_power_w
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
            "zc_confirm_reject": self.zc_confirm_reject,
            "zc_phase_hist": tuple(self.zc_phase_hist),
        })
