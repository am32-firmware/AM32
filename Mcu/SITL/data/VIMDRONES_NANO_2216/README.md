# VimDrones Nano ESC + T-Motor AIR 2216 II 920KV + 10x4.5 prop

Third calibration target, and the first with a propeller: the
motor/prop from an Edu450 frame, powered at 3S (12.6 V bench supply).
Captured 2026-07-20.

## setup

- VimDrones Nano ESC `com.vimdrones.esc_nano#M1`, bus node 30 on
  `mcast:0:lo` (three more nano ESCs M2-M4 on nodes 31-33 and a
  MatekL431 are on the same bus, unused)
- motor: T-Motor AIR 2216 II 920KV, 14 poles (eeprom MOTOR_POLES=14,
  MOTOR_KV=900), 10x4.5 inch propeller fitted - Edu450 frame
  drivetrain on 3S
- this tree's firmware (current sense reads 0.00-0.04 A at zero
  throttle, debug1 v2 active), TELEM_RATE=200, DEBUG_RATE=100
- eeprom CURRENT_LIMIT=8 (protects the bench supply): SITL model
  eeproms must set the same 8A limit to match
- full throttle bench-verified by the user before these captures

## files

- `first_spin_010.jsonl` — 0.1 throttle hold: 1079 rpm, 0.11 A
- `sweep1.jsonl` — staircase 0.05..1.00: the 1.00 step transiently
  spiked to 9.9 A before the firmware limiter caught it and tripped
  the tool's 9.5 A abort (kept for the record of that transient)
- `sweep2.jsonl` — the clean steady reference, 0.05..0.95 up and
  down. rpm rises to 7000 at 0.80 then the 8 A current limiter
  governs: 0.85-0.95 all sit at ~7100 rpm / ~7.9 A. Supply sags
  12.40 -> 11.70 V at the limit. Currents follow the expected
  near-cubic prop law below the limit (0.5: 2.5 A, 0.8: 7.6 A)
- `nano_chirp_120s.jsonl` — 120 s chirp 0.5->40 Hz at 0.35±0.15
  (same profile as the other combos; stays in the linear band,
  current peaks 5.75 A). Empirical 3dB: accelerating 3.41 Hz,
  braking 2.77 Hz — far slower than the unloaded combos (prop
  inertia) and nearly SYMMETRIC: the prop's aerodynamic drag does
  most of the braking, so the supply-absorption limit no longer
  dominates down-transients
- `square1.jsonl` — square amplitude sweep (mid 0.35, deltas
  0.03..0.24): up 122..174 ms and down 118..189 ms, both growing
  with amplitude and roughly symmetric, unlike both unloaded combos.
  Braking still pumps the bus to 20.7 V on the larger down-steps
  (supply cannot absorb regen), current peaks 7.1 A

## observations for the model fit

- zero desyncs anywhere (err=0 in every capture): the prop-loaded
  motor is far more stable than the unloaded combos
- the fit needs: prop load dominating load_k_omega2 (near-cubic
  power), prop-dominated inertia J, the 8 A firmware current limit
  mirrored in the SITL eeprom, and the supply sag (~0.09 ohm from
  the sweep) plus the regen pump to 20.7 V for braking
- the current-limited plateau (0.85-0.95) is a new regime to
  reproduce: firmware duty-folding against the limit

## SITL model v1 (Mcu/SITL/models/vimdrones_nano_2216.json)

kv 944 effective, R 0.09, L 15u, J 3.35e-5 (prop dominated),
load_k_omega2 1.9e-7 (the prop constant: the naive fit from input
power over-estimates it ~20% by absorbing motor losses), battery
12.40V/0.09 ohm, sink Rsink 4 / imax 0.8 (fit to the braking bus
pump), CURRENT_LIMIT=8 mirrored in `sitl_eeprom.bin`.

SITL files: `sitl_sweep_v1.jsonl`, `sitl_chirp_120s_v1.jsonl`,
`sitl_square_v1.jsonl`.

v1 vs real:

- steady: within 2% over 0.4..0.8 and at 0.2, -3.7% dip at 0.3
  (this combo's curve-shape residual); THE 8A CURRENT-LIMIT PLATEAU
  REPRODUCES (sim 0.85-0.95 pinned ~7000-7150 rpm vs real ~7100)
- low-throttle current reads low (0.04 vs 0.11 A at 0.1): the ESC's
  constant switching overhead is not modelled
- chirp 3dB: accel 3.31 vs real 3.41 Hz; brake 2.18 vs 2.77 (~20%
  slow - the braking residual inverts sign with a prop: the
  unloaded combos braked too fast, this one too slow)
- square: up 114..127 ms vs real 122..174 (matches small amplitude;
  real grows at large amplitude where up-transients hit the 8A
  limiter, sim stays flat - the limiter-transient regime is a new
  residual); down 168..206 vs real 118..189 (~15-40% slow, same
  braking residual); braking bus pump 23.2 V vs real 20.7
- zero desyncs in all sim runs, matching the real hardware
- SITL COLD-START with prop inertia is marginal at mid throttle
  (the real motor starts cleanly from 0.05): sweeps lead with a 0.3
  level and dynamic tests pre-spin the motor first, as encoded in
  run_calibration_tests.py

## SITL model v2 (refit on the current physics)

The v1 fit predates the persistent-diode / inertial-comparator /
floating-terminal physics changes, which slowed the simulated
down-curve to 60%+ over real at small amplitudes. v2 refits on the
current physics: R 0.0625 with battery 0.1175 ohm (same total sag,
faster small-signal settling), damping 1.5e-5 (approximates the
speed-proportional ESC/iron losses; decouples down-speed from
up-speed where pure R could not), sink Rsink 0.35 / imax 0.9
(absorption engages near the set voltage instead of only at deep
pump).

v2 vs real:

- steady: all levels within 3.7% including the 8A limiter plateau
- square down-curve: within +21/-27 ms across 0.06..0.24 (v1:
  +50..+80 ms); up-steps 90..102 ms, faster than the real 122..174 -
  the bench PSU's CC foldback that stretches real up-surges is not
  modelled, and the up_flat window floor allows for this
- chirp 3dB: accel 4.12 vs real 3.41 (fast, the price of the
  R/damping rebalance), brake 3.03 vs 2.77 (v1 was 20% slow)
- zero desyncs, cold-start marginality unchanged
