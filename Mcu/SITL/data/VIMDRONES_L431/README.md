# VimDrones L431 + 1404 4300KV bench data

Second calibration target (after SEQURE_G431). Captured 2026-07-19.

## setup

- VimDrones L431 DroneCAN ESC, `com.vimdrones.esc_dev#M1`, stock
  AM32 2.20, bus node 123, reached via dronecan_bridge.py on
  `mcast:0:lo` as for the SEQURE
  (https://shop.vimdrones.com/products/vimdrones-esc-development-board)
- motor: 1404 size, 4300 KV, 14 poles (eeprom MOTOR_POLES=14, so
  mech rpm = 2.857e6/ci with ci in 0.5us ticks, same scaling as the
  GT2215)
- bench supply 12.0 V (reads 11.85-11.90 under load)
- no prop
- eeprom as found: TELEM_RATE=200 (kept - 4x the SEQURE's rate,
  helpful for the lighter rotor), ADVANCE_LEVEL=16, PWM 24kHz
  variable, CURRENT_LIMIT off. Changed and saved: DEBUG_RATE 0->100
  (FlexDebug debug1 v1 at 100Hz)

## current sense

On the STOCK firmware the ESC reported ~+3.27 A at zero throttle
(sense amp bias ~98 mV, same error family as the SEQURE's): sweep1
carries that offset and the stock gain (MILLIVOLT_PER_AMP 30) —
treat rpm/volt as the real signals in it.

FIXED 2026-07-19: the ESC now runs this tree's VIMDRONES_L431_CAN
build (flashed over stlink at 0x08004000, bootloader AM32_BOOTLOADER
_L431 v18 and eeprom untouched) with CURRENT_AUTO_OFFSET enabled and
debug1 v2. Idle now reads 0.00-0.02 A; 0.24 A at 0.5 throttle
no-load. Gain is still the stock 30 mV/A, not yet checked against
the bench supply. rpm behaviour is unchanged by the reflash (4372 /
10556 / 20980 rpm at 0.1/0.25/0.5 vs stock 4372/10557/21004).
Stock flash is backed up in `stock_flash_backup_2.20.bin` (128 KB
from 0x08000000, not committed); restore with
`st-flash write stock_flash_backup_2.20.bin 0x08000000`.

BOOT0 GOTCHA: with the SWD harness attached, BOOT0 is pulled high,
so ANY software reset (st-flash --reset, AM32's own system resets)
lands in the ST ROM bootloader and the ESC goes silent on CAN. A
power-cycle with BOOT0 low is the recovery. Consider setting option
bits nSWBOOT0=0/nBOOT0=1 to boot main flash regardless of the pin.

Only the captures for the current model and firmware are kept in the
tree; the stock-firmware chirp/square runs and first-spin checks were
pruned after the recaptures below reproduced their signatures
(stock chirp: 8.0/3.4 Hz; stock square up 39->70 ms growing, down
41->158 with the last-point drop).

## files

- `sweep1.jsonl` — steady sweep 0.05..0.50 x 4s (stock firmware,
  rpm/volt valid): 2128 -> 21004 rpm, monotonic, up/down legs match
  within noise, supply sag only ~50 mV. The steady-state reference.

### captures on this tree's firmware (valid current)

- `vim_chirp_120s_v2.jsonl` — same chirp profile, this tree's
  firmware: accel 3dB 8.8 Hz / brake 3.5 Hz (stock run: 8.0/3.4 —
  the ~1 Hz spread is the metric's run-to-run scatter). Current
  peaks 1.09 A. err reached 7 (real desyncs in the troughs again).
- `square2.jsonl` — same square profile: up 35->75 ms growing with
  amplitude, down 45->160 ms with the same last-point drop (133 ms
  at delta 0.24) as the stock run - both signatures reproduce.
  Current peaks 1.83 A on transients. err 8 cumulative.

The v2/2 files are the model-fit references (current channel valid,
magnitude confirmed against the bench supply display).

## SITL model v2 (current)

Refit after the diode-chatter solver rework and the physical
comparator front end (zero hysteresis as on the real board, 5 mV
band-limited noise, 800 ns divider RC, inertial propagation).
J 1.75e-7, Rsink 15, sink_current_max 0.045, kv 3660 with the drag
reshaped toward linear (damping 9e-8, k_omega2 1.2e-10) to centre
the steady error on the upper half where the dynamic tests operate.

- `sitl_sweep_v2.jsonl` — steady within 1.6% over 0.30..0.50 and
  2.2% over 0.10..0.25 (bottom step +6.8%); the residual -1.6% dip
  at 0.35 is the waveform-family curve-shape residual (the GT2215
  shows the mirror image at its top end). Current 0.23-0.24 A at
  0.5 vs real 0.24
- `sitl_chirp_120s_v2.jsonl` — accel 3dB 8.1 Hz (real
  8.0-8.8), brake 3.34-3.45 (real 3.4-3.5)
- `sitl_square_v2.jsonl` — down-curve 56..155 ms vs real 45..160:
  0.09-0.18 amplitudes essentially exact (100/100, 127/120,
  141/142, 155/155), endpoint region ~9% fast with the last-point
  drop reproduced (115 vs 133). Braking bus pump 17.5 V vs real
  17.97
- reported rpm noise now at or below the real ESC's everywhere
  (was up to 5x noisier before the solver rework); desync
  marginality matches real (err 0-7 across runs vs real 4-8)

## SITL model v1 (superseded by v2, captures pruned)

Fitted against the captures above (procedure per scripts/CALIBRATION.md,
using the v2 valid-current files). kv 3620 effective (nameplate 4300 -
this motor runs ~16%% below duty*kv*V, unlike the GT2215 which sat on
its nameplate), L=20uH (5uH burned 5W in PWM ripple copper - the
energy ledger localised it), J=1.5e-7, supply 12.0V/0.15ohm with a
weak regen sink (25 ohm, 30mA knee).

SITL files: `sitl_sweep_v1.jsonl`, `sitl_chirp_120s_v1.jsonl`,
`sitl_square_v1.jsonl`. Compare:
`python3 scripts/esc_plot.py Mcu/SITL/data/VIMDRONES_L431/square2.jsonl Mcu/SITL/data/VIMDRONES_L431/sitl_square_v2.jsonl`

v1 vs real:

- steady sweep: within +-4%% (top half exact to ~2%%, +4%% hump at
  0.20-0.25, +7%% at the 0.05 bottom step)
- steady current: 0.19-0.27A at 0.5 across runs vs real 0.24
- chirp raw 3dB: accel 9.2Hz vs real 8.0-8.8 (~8%% fast); brake
  4.3Hz vs real 3.4-3.5 (~25%% fast - same small-signal braking
  residual family as the SEQURE model v8)
- square: up-time GROWS with amplitude like the real motor (the AM32
  ramp interaction reproduces) but runs fast at large deltas (50 vs
  75ms); down-curve monotonic with the same last-point drop, ~20%%
  fast at mid/large amplitude (125 vs 160ms); braking bus pump 16.8-
  18.0V vs real 17.97
- desync marginality: the sim desyncs at the same operating points
  (chirp troughs, low-rpm square steps) with err 0-28 run-to-run
  (real: 4-8 consistently). SITL cold starts into mid throttle are
  noticeably more fragile than the real ESC - batteries precondition
  the sweep via 0.1 and chirp runs may need a retry.

Open residuals mirror the GT2215's and point at the same suspect
(trapezoid BEMF + regen conduction alignment): small-signal brake
too fast, large-amplitude transients too fast at matched J. The J
that fits the chirp (1.5e-7) under-fits the square (wants 2.0e-7) -
the same small-vs-large-signal tension seen on the GT2215 before its
refit. Bench J + BEMF waveform measurements would close both.

## real desyncs

This ESC/motor genuinely desyncs on the bench: esc.Status
error_count reached 4 during the chirp (deep troughs, momentary
collapse to ~170 rpm with recovery) and 5 cumulative after the
square. The SEQURE/GT2215 never desynced in any bench run. The SITL
model for this combo should reproduce that marginality, not hide it.
