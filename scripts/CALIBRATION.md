# Calibrating a SITL model against a real ESC/motor

The procedure used for the SEQURE G431 + GT2215 bench calibration,
written down so further ESC/motor combinations follow the same steps.
Data lives in `Mcu/SITL/data/<TARGET>/` with a README describing each
capture; the model json goes in `Mcu/SITL/models/`.

## 0. recon

- Bridge the ESC onto a multicast bus (dronecan_bridge.py, or direct
  USB for low latency — preferred for the chirp runs).
- `can_monitor.py` — confirm the node, note NodeStatus behaviour. An
  AM32 CAN ESC parks in its bootloader ("no signal") until a
  RawCommand stream arrives; all tools here stream zero throttle.
- `can_params.py list` — record the full parameter set. Set and SAVE:
  MOTOR_KV and MOTOR_POLES to match the motor (KV affects low-rpm
  power protect), TELEM_RATE 50, DEBUG_RATE 50 (FlexDebug on).
- For debug1 v2 fields (duty, clamp, raw ADC) the ESC needs this
  tree's firmware: back up stock flash over stlink first
  (`openocd ... dump_image`), flash the app at its base address only,
  eeprom page untouched.

## 1. baseline measurements (esc_measure.py)

- first spin: `hold --throttle 0.1 --hold 3 --max-current 3`
- static curve: `sweep --levels 0.05..0.5 --hold 4`
- transients: `cycles --levels 0.2,0.5 --cycles 12 --hold 1.2
  --telem-rate 200 --debug-rate 100`
- watch the braking bus-voltage transient in the down-steps (status
  volt + debug1 adc_volts): a supply that cannot absorb regen pumps
  the bus up and dominates spin-down behaviour.
- current calibration if the sense scale is unknown: steady
  finger-load at ~0.25 throttle while reading the bench supply
  display; solve mV/A from the debug1 adc_current delta. Check
  MILLIVOLT_PER_AMP and the auto-offset behaviour (CURRENT_AUTO_OFFSET).

## 2. chirp (esc_chirp.py) — the primary dynamics metric

- `run --duration 120 --f-start 0.5 --f-stop 40 --throttle-mid 0.35
  --throttle-amp 0.15` on the real ESC (direct USB if possible).
- `fit LOG --max-freq 15` → the characteristic pair: empirical -3dB
  accelerating / braking, and the asymmetric 1-pole tau_up/tau_dn.
- short sweeps (45s) are NOT equivalent for SITL iteration health:
  desync-marginal configs fail there chaotically. Iterate with the
  full 120s profile.

## 3. SITL model fit (order matters)

Run the same profiles against the SITL build (`--can-uri mcast:N:lo`,
mirror the ESC's eeprom params, `--physics-log` for raw truth,
dedicated raw file per run):

1. `battery.voltage` — match the reported idle voltage.
2. `motor.kv` — scale until the steady rpm curve matches (±2%);
   this is an effective value, typically above nameplate.
3. `battery.resistance` — from the loaded voltage sag vs measured
   current.
4. `battery.capacitance` + `battery.sink_resistance` — fit the
   braking bus-voltage transient (peak + decay) and the down-step
   time. capacitance 0 = legacy stiff source.
5. `esc.commutation_transfer` — fraction of the conduction current
   handed to the incoming phase instantly at commutation. Real motors
   measure as if this is ~1.0; leaving it 0 loses small-signal torque
   gain (low chirp bandwidth) and destabilises zero-cross detection at
   light inertia.
6. `motor.inductance` — sets the per-commutation current build and
   with it the accelerating torque at high slip. Fit to the up-step /
   accel bandwidth. CAUTION: too low → current ripple corrupts the
   sim's zero-cross detection → desyncs. This is an effective
   commutation-transfer value, not the datasheet phase inductance.
7. `motor.inertia` — accel bandwidth scales with 1/J; braking energy
   scales with J (re-touch sink_resistance after changing J).
8. drag terms (damping/static_friction/load_k_omega2) — need a
   current reference (validated no-load current, or loaded points).

Validation set for a model version: steady sweep ±2-3%, step cycles
t10-90 up/down vs the real ensembles, 120s chirp accel/brake 3dB vs
real, zero desyncs through the sweep (err field / rpm collapses),
clean start at low throttle.

## known model limits (as of the GT2215 calibration)

- braking is ~15% stronger than real in the fitted model (chirp brake
  3dB 3.5 vs 3.2 Hz, down-step 120 vs 150 ms) and sink_resistance has
  saturated as a lever — something beyond the supply now sets the
  braking floor (motor dissipation path).
- SITL desyncs are host-scheduler starvation, proven by A/B: full-core
  CPU load induces them reliably (the firmware thread can be starved
  1.5-2.5ms while sim time advances — impossible on silicon), and
  running the SITL with --realtime (SCHED_FIFO, needs rtprio rlimit or
  sudo prlimit) under the same load produces zero. RULE: run
  measurement SITL sessions on an idle host or with --realtime. The
  durable fix (scheduler progress gating) is future work.
- the braking-curve divergence at large step amplitude traces to the
  LINEAR sink model: the real supply bus pumps to ~24V where the
  linear sink caps the sim rail at ~21V, giving the sim extra
  absorption exactly where real braking saturates (k=0 vs k=1 square
  sweeps showed the commutation-transfer projection contributes
  almost nothing to this curve). Next model change: nonlinear sink
  knee fitted to the measured bus excursions.
- cold start straight to 0.2+ throttle can still thrash at light
  inertia; precondition via a lower throttle first.
- no thermal model; temperature is a constant.
- bench-supply current limit (CC foldback) is not modelled (the
  regen-side sink_resistance is, which is what dominates braking).
