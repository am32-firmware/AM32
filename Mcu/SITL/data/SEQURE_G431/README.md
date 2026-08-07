# SEQURE G431 CAN ESC calibration data

Measurements of a real SEQURE G431 CAN ESC (`com.sequre.esc`, stock AM32
2.20, hw 2.3, DroneCAN node 124 via a serial CAN bridge on `mcast:0:lo`)
driving an EMAX GT2215/10 1100 kv motor (12N14P, 14 poles), no prop,
16 V bench supply with a 5 A current limit. Captured 2026-07-19 with
`scripts/esc_measure.py`; compare with `scripts/esc_analyze.py`.

The matching SITL model is `Mcu/SITL/models/sequre_gt2215.json` (v9).
SITL logs here were produced by the identical profiles against the SITL
build (same eeprom settings as the real ESC: MOTOR_KV=1100 POLES=14
TELEM_RATE=50 DEBUG_RATE=50 VARIABLE_PWM=1 PWM 24k COMP_PWM=1
ADVANCE_LEVEL=16, INPUT_SIGNAL_TYPE=0, REQUIRE_ARMING=0).

Only the captures for the current model and firmware are kept in the
tree; superseded intermediate runs (models v3-v7, early exploratory
spins) were pruned. The fit history below summarises what they showed.

## files

real hardware:

- `sweep1.jsonl` — staircase 0.10..0.50 up and down, 4 s per level
  (the steady-state reference)
- `sequre_chirp_120s.jsonl` — 120 s exponential chirp 0.5..40 Hz at
  throttle 0.35±0.15, ESC direct on USB (low lag, ~196 Hz telemetry).
  esc_chirp.py fit: empirical 3dB accel 11.6 Hz / brake 3.2 Hz
- `square1.jsonl` — square-wave amplitude sweep (esc_square.py, mid
  0.35, delta 0.03..0.24). KEY RESULT: real up-time is flat ~30 ms at
  every amplitude; down-time grows monotonically with amplitude
  (40 -> ~180 ms — braking is absorption-limited slew)
- `cycles_v2.jsonl` — cycles run with debug1 v2 fields logged
  (this tree's firmware on the real ESC)
- `finger_load1.jsonl` / `finger_load2.jsonl` — current-sense
  calibration by finger load (see below)
- `desync_ringdump_hostload.txt` — SITL commutation ring dumps from
  the host-load desync investigation (see below)
- `stock_flash_backup_2.20.bin` (not committed) — full 128 KB flash
  dump of the stock SEQURE firmware taken over stlink BEFORE flashing
  this tree's SEQURE_G431_CAN build at 0x08004000 (bootloader +
  eeprom untouched). Restore stock app:
  `openocd -c "adapter serial <stlink>" -f Mcu/g431/openocd.cfg \
     -c "reset halt" -c "flash write_image erase stock_flash_backup_2.20.bin 0x08000000" \
     -c "reset run" -c shutdown`

SITL, model v9:

- `sitl_sweep_v9.jsonl`, `sitl_chirp_120s_v9.jsonl`,
  `sitl_square_v9.jsonl` — the same three profiles against SITL.
  The --physics-log raw internal logs are NOT committed (pure SITL
  output, ~60 MB each - regenerate by rerunning the chirp against
  the model with --physics-log). Compare with:
  `python3 scripts/esc_plot.py Mcu/SITL/data/SEQURE_G431/square1.jsonl Mcu/SITL/data/SEQURE_G431/sitl_square_v9.jsonl`

## fit history (v3-v7, captures pruned)

- v3: after the supply-model fix and L/J/C fit, steps matched
  50/150 ms vs real 40/150 but the chirp exposed a small-signal accel
  deficit (raw physics 6.8 Hz vs real 11.6) — the chirp is a much
  sharper discriminator than steps.
- v4 (J=4.75e-6): accel 8.2-8.4 Hz. The residual was a ~0.65
  small-signal torque-gain factor from per-commutation current build.
- v5 (commutation_transfer k=1, J=3.9e-6): accel exact (11.66 Hz) but
  the light J was an identifiability artefact that overstated braking.
- v6 (back to physical J=4.75e-6): brake exact (3.28 vs 3.2 Hz),
  accel honest residual 9.5 Hz. The square sweep then showed sim
  down-time getting FASTER at large amplitude where real grows
  monotonically — the k=1 projection overstates large-signal regen.
- v7 (saturating supply sink: C=2e-3, Rsink=14, imax=0.35): square
  down-curve monotonic 59..131 ms (real 40..183), accel 10.1 Hz. The
  energy ledger showed the remaining braking excess is
  electromagnetic regen current ~2x real with copper losses matching
  — pointing at the trapezoid BEMF waveform.
- v8 (below): refit after the comparator response-time latch removed
  a chatter artifact the earlier fits had absorbed.

## desync investigation

`desync_ringdump_hostload.txt` preserves the commutation ring dumps
that cracked the load-induced desyncs: bursts of comparator EDGE
events right after commutation (discrete-step diode chatter) plus
MAINLOOP starvation gaps of 1.5-2.5 ms under host CPU load. The
initial verdict (host-scheduler starvation, SCHED_FIFO as the
workaround) was superseded by the durable fixes in Mcu/SITL:
the firmware progress lease (sim.fw_lag_max_ns) and the comparator
response-time latch (sim.comparator_min_toggle_ns) — with both, the
full load reproduction runs desync-free without privileges. See
Mcu/SITL/TIMING-DESIGN.md for the complete story.

## model v9 (current)

Refit after the solver rework that removed the discrete-step diode
chatter at the source (persistent diode states with sub-step
zero-crossing events) and replaced the comparator refractory latch
with inertial propagation. The comparator front end is now modelled
as the real board has it: ZERO hysteresis (the firmware configures
none), 3 mV band-limited input noise through an 800 ns divider RC.
Parameters: kv 1165 (the honest solver restores the old "effective
kv" - v8's kv=nameplate was an artifact interaction), J 4.22e-6,
Rsink 8, sink_current_max 0.26 (recalibrated with repeat-averaged
chirps; single runs scatter ~1 Hz on the 3dB metric).

- `sitl_sweep_v9.jsonl` — steady within ±2% (top-end +2.1%
  concavity remains)
- `sitl_chirp_120s_v9.jsonl` — raw-physics accel 3dB 11.0-12.0 Hz
  across runs (real 11.6), brake 3.07-3.18 (real 3.2)
- `sitl_square_v9.jsonl` — down-curve 65..159 ms vs real 40..183:
  0.06-0.15 amplitudes near exact (81/80, 105/111, 126/136,
  141/140), endpoint 159 vs 166. Up flat ~30-36 ms (real ~30).
  Chirp brake and square endpoint trade against each other inside
  the regen-excess residual; this point balances both within ~5%
- reported rpm noise is now at or below the real ESC's at every
  operating point (the chatter was the noise source)
- RESIDUALS: bus overshoot ~32 V vs real 24.2 during heavy braking
  (regen power still ~2x - the trapezoid BEMF suspect, awaiting the
  bench BEMF capture); sim desyncs a handful of times per long run
  (err 2-7) where the real GT2215 never does

## model v8 (superseded by v9, captures pruned)

Refit after the comparator response-time latch
(sim.comparator_min_toggle_ns, default 2 us): the latch removed a
chatter artifact the earlier fits had absorbed (steady state moved
~+7%), so every parameter was re-fit against the same bench data:
kv 1093, resistance 0.1, inertia 4.0e-6, load_k_omega2 2e-9,
sink_current_max 0.12. The desync fragility that previously blocked
parameter scans is gone — all v8 runs are err=0 including L=5uH
probes that used to thrash.

- `sitl_sweep_v8.jsonl` — steady state within ±1.5% of the real
  sweep except +2.0/+2.5% at 0.45/0.50 (the known real-curve
  concavity; awaits the BEMF waveform capture). kv is now 1093,
  essentially the 1100 nameplate — the old 1165-1173 "effective kv"
  was the chatter artifact.
- `sitl_chirp_120s_v8.jsonl` — raw-physics 3dB accel
  11.4-12.8 Hz across runs (real 11.6 — the accel deficit is CLOSED;
  metric scatter is ~1 Hz), brake 3.4-3.9 Hz (real 3.2, sim still
  ~15% fast at small signal).
- `sitl_square_v8.jsonl` — down-time curve 45..175 ms vs real
  40..183 monotonic, largest-amplitude endpoint matches (175 vs
  166-183); mid-amplitude points ~10-15% fast. Up flat ~30 ms both.
- REMAINING (all converge on the trapezoid BEMF suspect, resolved by
  the bench open-circuit BEMF capture): small-signal brake ~15%
  fast, top-end steady +2.5% concavity, bus overshoot ~32V vs real
  24.2V during heavy braking (regen power still ~2x).

## calibration results

- steady-state rpm matches within about ±1.5 % over 0.05..0.50
  throttle with motor.kv=1093 (essentially the 1100 nameplate; the
  ESC's eeprom MOTOR_KV stays 1100 as on the real ESC)
- reported voltage matches within ~10 mV (battery.voltage tuned to
  15.75 so the reported value matches; the ESC reads its 16 V supply as
  ~15.68 V)

## resolved (2026-07-19): the transient asymmetry

Root causes found via debug1 v2 (ensembles aligned on the ESC-side
adj_input change to exclude the 40-70 ms variable CAN-bridge latency)
and state-port instrumentation of the sim internals:

- **up-step too slow / creep** was the model's `motor.inductance`
  guess (25 uH): the per-commutation current build time constant
  2L/R_line matched the 60-degree commutation period, so phase current
  (and torque) never reached its DC value. Fitting L to the transient
  (9 uH) fixed it — no code change; commutation alignment was
  verified clean (torque efficiency 0.95-0.99).
- **down-step braking too strong** was missing supply physics: the
  real bench PSU cannot absorb regen. Measured on hardware: braking
  pumps the bus from 15.64 V to a 20.4 V plateau, which self-limits
  brake torque. motor.c models this: `battery.capacitance` (bus
  capacitance) + `battery.sink_resistance` + `battery.sink_current_max`
  (weak, saturating absorption above the set voltage, e.g. a PSU
  downprogrammer); capacitance 0 keeps the legacy stiff bidirectional
  source.
- **cold-start**: the real ESC starts cleanly straight to 0.2 throttle;
  SITL desync-thrashes on that jump with light inertia (esc_measure's
  cycles profile preconditions via 0.1 for this reason).
- **current sense**: the amp is bidirectional, biased near mid-rail
  (~2044 counts = 1.65 V). Stock firmware read 0.000 A permanently
  because with CURRENT_OFFSET=0 the int16 `actual_current` arithmetic
  wrapped negative and clamped to 0. Fixed in this tree: int32 math
  with clamping plus CURRENT_AUTO_OFFSET (boot-time zero of the amp
  bias while the motor has never run, 100-cycle warmup for the 50-deep
  smoothing buffer), enabled for SEQURE_G431/SEQURE_G431_CAN/SITL.
  **Calibrated 2026-07-19 by finger-load test** (`finger_load1.jsonl`,
  `finger_load2.jsonl`: steady press at 0.25 throttle, supply display
  read 2.37-2.49 A while the sense saw +5.8 counts): true sensitivity
  ~2.0 mV/A, not the configured 5 — MILLIVOLT_PER_AMP is now 2 for
  the SEQURE_G431 targets. Resolution ~0.4 A/count (large ESC).
  Cross-checks: no-load bus current at 0.25 throttle reads 0.40 A vs
  0.364 A predicted by the SITL model (first drag-parameter
  validation); the loaded voltage sag gives a real source resistance
  of ~0.085 ohm (battery.resistance updated from 0.03).
- real rpm noise grows with throttle (sd ~6 rpm at 0.1, ~70 at 0.5);
  SITL is much cleaner (sd < 20).
