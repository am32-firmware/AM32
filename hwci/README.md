# AM32 Hardware-CI Harness (ARK 4IN1 ESC)

A hardware-in-the-loop test harness for improving AM32 on the **ARK 4IN1 ESC**.
It builds and flashes firmware, drives a motor through a **Tyto Robotics Flight
Stand 50**, and at the same time reads firmware **CPU-load / loop-time**
instrumentation off the MCU over **SWD**, then turns it all into metrics, a
baseline, and a pass/fail report you can gate pull requests on.

```
                 ┌───────────────────────── Ubuntu 24.04 host ─────────────────────────┐
                 │                                                                       │
  ST-Link  ◄─SWD─┤  OpenOCD ──background RAM reads──►  hwci_perf struct (loop µs, iters) │
                 │                                                                       │
  USB-serial ◄───┤  KISS telemetry reader  ◄────────  ESC telem wire (eRPM, V, A, °C)    │
                 │                                                                       │
  Flight Stand◄──┤  gRPC client  ──throttle──►  ESC signal   ──measures──► thrust/torque/│
   (gRPC)        │                                              RPM/V/A → efficiency g/W │
                 │                                                                       │
                 │  runner → metrics → baseline compare → report.md + plots + exit code  │
                 └───────────────────────────────────────────────────────────────────────┘
            ARK 4IN1 ESC (4× STM32F051 / Cortex-M0) ── motor ── prop ── Flight Stand 50
```

## Why it's built this way (read this first)

The ARK 4IN1 runs **four independent STM32F051 MCUs** (one per channel), each a
**Cortex-M0 (ARMv6-M)**. The M0 has **no DWT cycle counter, no ITM, and no SWO**
— so the usual "profile over SWO/ITM trace" approach is **impossible on any
debug probe**, ST-Link or J-Link alike.

So CPU load and loop times are recovered a different way:

1. The firmware keeps a tiny instrumentation struct (`hwci_perf`) in RAM,
   updated from the 20 kHz control loop and the main loop using the existing
   1 µs free-running timer (`UTILITY_TIMER`/TIM17).
2. The debugger reads that struct **without halting the core** via SWD
   background memory access (works on the M0; it's the same mechanism as IDE
   "live watch"). Both ST-Link (OpenOCD) and J-Link support it; **ST-Link +
   OpenOCD is the default** because you already use it to flash the bootloader
   and the OpenOCD config already ships in the repo.
3. **CPU load** uses the idle-residual method: the firmware exposes a
   free-running `loop_iters` counter whose rate (iters/s) is highest when the
   core is least loaded, so `cpu_load = 1 − rate/idle_rate`.

Efficiency and demag come from the thrust stand + ESC telemetry, correlated on
the host. Demag/desync is detected from RPM/thrust collapse at high throttle,
ESC-eRPM vs stand-RPM divergence, commutation-interval spikes, and the firmware
bemf-timeout flag.

## What it measures

| Channel | Source | Metrics |
|---|---|---|
| CPU load | `hwci_perf.loop_iters` via SWD | % load vs idle baseline, per operating point |
| Loop times | `hwci_perf` ctrl/main timers via SWD | worst-case 20 kHz exec µs, period jitter, main-loop µs |
| Efficiency | Flight Stand thrust + V·A | thrust (gf), electrical power (W), **g/W** per throttle |
| Demag | stand RPM + KISS eRPM + perf | desync events, eRPM/RPM mismatch, commutation spikes |
| Health | KISS telemetry | voltage, current, temperature, consumption |

## Firmware instrumentation (`HWCI_PERF`)

The instrumentation is **opt-in and zero-cost when off**. Build it with:

```
make ARK_4IN1_F051 HWCI_PERF=1
```

* When `HWCI_PERF` is **unset** (all production/release builds), every hook
  expands to nothing — the binary is byte-for-byte identical.
* When **set**, it adds ~**530 B flash** and **72 B RAM** on the F051 and emits
  the `hwci_perf` symbol the host locates via the ELF (address + DWARF layout,
  so host and firmware can never silently disagree — there's a test for it).

Files: `Inc/hwci_perf.h`, `Src/hwci_perf.c`, three hooks in `Src/main.c`
(`tenKhzRoutine` enter/exit, `while(1)` top), one `Makefile` flag.

## Hardware setup

### Debug (one channel at a time)

The ARK 4IN1 exposes all four SWD pairs on a single 10-pin debug header:

| pin | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 |
|----|---|---|---|---|---|---|---|---|---|----|
| sig | 3V3 | SWDIO1 | SWCLK1 | SWDIO2 | SWCLK2 | SWDIO3 | SWCLK3 | SWDIO4 | SWCLK4 | GND |

Wire the ST-Link to the SWDIO/SWCLK pair of the channel whose motor is on the
stand (channel 1 by convention). **Do not** connect the ST-Link 3V3 when the ESC
is powered from a battery/supply — share GND only. All four dies run identical
firmware, so one channel's loop-time/CPU data is representative; efficiency is
per-motor.

### Signal, telemetry, power

* **Throttle**: Flight Stand ESC output → channel-1 signal pin (PWM or DShot).
  If your stand can't emit the protocol you want, use an external DShot
  generator (`throttle_backend: external`).
* **ESC telemetry**: channel-1 telemetry pad → USB-serial adapter (115200 8N1)
  → host. The ARK target already has `USE_SERIAL_TELEMETRY`.
* **Power**: bench supply or battery within the ARK 4IN1's 3–8S range; common
  ground between supply, ESC, stand, and host.
* **Safety**: set conservative cutoffs in the profile `safety:` block (current,
  thrust, RPM, voltage, temperature) — the **runner enforces them host-side on
  every sample** (stand reading and ESC telemetry alike) and aborts the run on
  breach. Also configure the Flight Stand Software's own UI cutoffs as an
  independent second layer (the vendor set-limit RPC is not mapped yet). Secure
  the motor/prop; use a prop guard; keep clear during demag-stress runs.
* **Battery**: pass `--battery-cells N` to `hwci run`/`hwci ci` (e.g. `6` for a
  6S pack) to refuse to *start* a test when the pack is already at or below
  `N * --min-cell-voltage` (default 3.3 V/cell, matching AM32 firmware's own
  low-voltage-cutoff default). This is a pre-flight gate checked once before
  the throttle is armed — separate from, and in addition to, the per-sample
  `safety:` limits above. It reads the stand's voltage channel, or the perf
  struct on a stand-less bench. Opt-in and hardware-only (has no effect under
  `--sim`, since the simulator's pack has no real cell count).

## Software setup (Ubuntu 24.04)

```bash
cd hwci
./scripts/setup_ubuntu.sh        # apt deps, ARM toolchain, OpenOCD, udev, venv
```

Then, manually (vendor bits the script can't automate):

1. The **Tyto Flight Stand Software runs on Windows only** (no Linux build as
   of v2.4.x). Install it on a Windows PC on the bench network, plug the
   stand's USB into that PC, and launch the software with the `--remote`
   flag so the gRPC API accepts non-local connections. Point `stand_host`
   in `rig.yaml` at that PC. (The GUI is also where you configure the
   vendor-side safety cutoffs. Load cells don't need a manual tare before
   each run: `hwci run`/`hwci ci` tare automatically as the last pre-flight
   step — with the ESC signal already up at zero throttle, because AM32
   beeps the motor whenever it has no input signal and those beeps shake
   the cells mid-tare. `--no-tare` skips it.)
2. Clone Tyto's API repo — it ships **pre-compiled Python stubs**, no protoc
   run needed — and make it importable in the harness venv:
   ```bash
   git clone https://gitlab.com/TytoRobotics/flightstand-api ~/flightstand-api
   .venv/bin/pip install grpcio protobuf
   echo ~/flightstand-api/languages/python > \
       .venv/lib/python3*/site-packages/flightstand_api.pth
   ```
   `hwci/hwci/flightstand/grpc_client.py` is mapped against
   `flight_stand_api_v1.proto` (Flight Stand Software 2.4.x); the
   proto-aware calls stay isolated in `_StubAdapter` if Tyto's API moves.
3. `cp config/rig.example.yaml rig.yaml` and edit ports/host/signal map/motor.
4. Edit `/etc/udev/rules.d/99-hwci.rules` with your USB-serial serial numbers so
   `/dev/esc-telem` (and `/dev/esc-throttle`) appear.
5. ESC telemetry only streams if the AM32 EEPROM setting
   `telemetry_on_interval` is enabled (AM32 configurator: "30ms telemetry").
   With it off, the KISS channel is silent even when wired correctly. On a
   bench without the telemetry wire, set `telem_backend: none` — the perf
   struct still reports eRPM/voltage/current/temperature over SWD.

Verify the harness with **no hardware** at any time:

```bash
python -m hwci selftest          # runs ci_smoke in the built-in simulator
python -m pytest                 # 75 offline tests
```

## Usage

```bash
hwci profiles                                   # list test profiles
hwci build --config rig.yaml                    # make ARK_4IN1_F051 HWCI_PERF=1
hwci flash --config rig.yaml                    # OpenOCD program @ 0x08001000
hwci run  --profile efficiency_sweep --config rig.yaml --battery-cells 6 --out runs/r1
hwci analyze runs/r1                            # -> metrics.json
hwci report runs/r1 --baseline baselines/ARK_4IN1_F051.json
hwci ci   --profile ci_smoke --config rig.yaml --battery-cells 6 \
          --baseline baselines/ARK_4IN1_F051.json --out runs/ci   # full gate
```

`--battery-cells` is optional but recommended on hardware runs: it refuses to
start the test at all if the pack is already too low (see Safety, below),
instead of arming and collecting a run's worth of data from a sagging supply.

`hwci ci` builds (HWCI_PERF=1) → flashes → runs the profile → computes metrics →
compares to the baseline → writes `report.md` + `summary.png`, and **exits
non-zero on regression** so CI fails the build.

Built-in profiles: `ci_smoke` (fast gate), `efficiency_sweep` (10–100 %
staircase, the primary baseline), `demag_step_stress` (aggressive steps).

## Auto-tuning AM32 settings (`hwci tune`)

Given a motor/prop on the rig, `hwci tune` automatically searches the AM32
EEPROM settings (`advance_level`, `pwm_frequency`, `variable_pwm`,
`auto_advance`, `max_ramp`, …) for the combination that maximizes efficiency
(g/W), subject to hard constraints: no demag/desync/bemf timeouts, zero-cross
jitter not regressed vs the default settings, temperatures bounded, and
reliable startup. **No rebuild per trial**: AM32 reads its 192-byte EEprom
page once at boot, so each trial one-shot-flashes the page over SWD (+ reset)
and runs a ~28 s probe. The live page address is read from the firmware's
`eeprom_address` global (the bootloader can relocate it) and the field
offsets are cross-checked against the flashed ELF's DWARF before anything is
written. Trial blobs are seeded from the device's current page and mutate
only the tuned bytes, so version/identity bytes and rig calibration survive.

```bash
hwci tune --spec tunes/example.yaml --config rig.yaml --battery-cells 4 \
          --out runs/tune-1            # hardware
hwci tune --spec tunes/example.yaml --sim --out runs/tune-sim   # offline dry run
hwci tune --resume runs/tune-1        # continue an interrupted session
```

**Spec format** (`tunes/example.yaml`, strict validation — unknown keys or
parameters fail loudly): `parameters:` declares the tunable fields and their
grids (unknown-to-hwci fields can be addressed with an explicit `offset:`);
`stages:` runs coordinate sweeps (`sweep:` one parameter, others at the
incumbent, optional `refine_step` around the argmax) and A/B mode stages
(`ab_candidates:` with interleaved `repeats`); `objective:` weights the
steady probe points (points under `min_power_w` are bench noise and never
score); `constraints:` are hard disqualifiers, never traded against score. A
`constraint_only: true` sweep (e.g. `max_ramp` on the step-stress profile)
tries values in listed order and picks the first with zero failures — list
them best-first.

**Noise handling**: same-firmware g/W spread reaches ~10 % on the bench and
the pack sags within a session, so the incumbent is re-run every
`anchors_every` trials and every score is reported raw *and* normalized to
the interpolation between surrounding anchors (cancels drift). Candidates
within `noise_floor_pct` of the best tie-break toward lower jitter, then
lower FET temperature, then closest-to-default. The finals run winner vs
default in interleaved ABBA blocks on the full efficiency sweep (plus a
startup-reliability check); the winner is confirmed only with a positive
median paired delta and zero constraint failures — otherwise
`best_settings.bin` keeps the defaults.

**Session dir / resume**: `runs/tune-1/` holds `manifest.json` (atomically
rewritten after every trial), `spec.yaml`, `base_settings.bin`, one standard
run dir per trial under `trials/T007-advance_level_22/` (plus its
`settings.bin` + `trial.json`), and at the end `report.md`,
`best_settings.bin`, `settings_diff.md`, and — when the `plot` extra
(matplotlib) is installed — a `tune_report.pdf` (verdict, settings diff,
full default/best tunable settings, per-trial objective + ABBA-delta plots,
full stage/trial tables, and high-level raw run data from every trial).
`--resume` replays the
deterministic plan: completed trials are reused from disk, partial trial
dirs are quarantined as `*.incomplete` and redone, and the incumbent page is
re-programmed first (a crash may have left arbitrary trial settings
flashed).

**Pack swaps**: when the resting voltage drops below
`battery_cells * pack.min_resting_cell_v`, the session checkpoints and
prompts you to swap the pack (recorded as a `pack_event`; ABBA blocks that
straddle a swap are discarded and restarted). With `--no-prompt` it exits
cleanly (code 3) instead — swap, then `--resume`.

The settings page is also scriptable directly:

```bash
hwci settings read  --config rig.yaml --bin page.bin   # dump current page
hwci settings diff  --config rig.yaml --bin other.bin  # exit 1 if different
hwci settings write --config rig.yaml --bin page.bin   # flash + verify
```

## Capturing the first baseline

Once wired and configured:

```bash
cd hwci
# 1. Sanity-check the full path on hardware with the short profile:
hwci ci --profile ci_smoke --config rig.yaml --battery-cells 6 --out runs/smoke
# 2. Capture the performance baseline:
hwci ci --profile efficiency_sweep --config rig.yaml --battery-cells 6 --out runs/baseline
hwci baseline-save runs/baseline --out baselines/ARK_4IN1_F051.json
git add baselines/ARK_4IN1_F051.json && git commit -m "hwci: ARK 4IN1 baseline"
```

A `--baseline` pointing at a file that doesn't exist yet prints a warning and
skips the gate (instead of failing), so the very first CI run — including a
`save_baseline` dispatch — can bootstrap the baseline itself.

`runs/baseline/report.md` is your benchmark: thrust & efficiency per throttle,
worst-case 20 kHz loop time, CPU load, and zero demag events on the swept ramp.
Every later change is graded against it.

The regression gate **fails closed**: a metric that is missing/NaN (dead SWD or
telemetry channel, misconfigured backend) fails its check, and per-channel
coverage checks (`perf_coverage`, `stand_coverage`, `telem_coverage`) name the
dead channel explicitly. A green run therefore proves the instrumentation was
alive, not just that nothing compared worse.

## CI integration

`.github/workflows/hwci.yml` runs on a self-hosted runner labelled
`[self-hosted, hwci]` wired to the rig. It is **manual-dispatch only**: the rig
needs hands-on preparation (battery connected/charged, prop and torque arm
checked, Flight Stand Software running), so a human always starts the run —
nothing triggers from pushes, PRs, or labels, and fork code never reaches the
bench unless a maintainer explicitly enters a PR number in the dispatch form.
Pick a profile (and optionally a PR number and `save_baseline`) in the form;
the workflow serializes hardware access with a concurrency group, posts
`report.md` to the job summary, uploads the run data, and fails the job on
regression (exit 1) or abort (exit 2). Set the repo/runner variable
`HWCI_RIG_CONFIG` to the path of `rig.yaml` on the bench.

## Repo layout

```
Inc/hwci_perf.h, Src/hwci_perf.c   firmware instrumentation (HWCI_PERF)
hwci/hwci/perf.py, elf.py          perf struct decode + ELF/DWARF symbol lookup
hwci/hwci/debugger/                OpenOCD (ST-Link) + Mock backends
hwci/hwci/flightstand/             gRPC client + simulator + base interface
hwci/hwci/esc_telem/kiss.py        KISS telemetry parser
hwci/hwci/throttle/                flight-stand / external / base throttle sources
hwci/hwci/sim.py                   offline rig simulator (all 3 channels)
hwci/hwci/runner.py metrics.py baseline.py report.py config.py cli.py
hwci/hwci/profiles/*.yaml          test profiles
hwci/tests/                        75 offline tests (sim, DWARF layout, fail-closed gating)
```

## Honest limitations

* The **gRPC client** is mapped against Tyto's published
  `flight_stand_api_v1.proto` (Flight Stand Software 2.4.x): inputs are
  discovered by `InputType` (FORCE_FZ=11 etc.), all signals are read in one
  `ListSamples` round trip, and throttle goes out via `UpdateOutput` on the
  ESC output's `output_target`. Tyto guarantees no cross-version API
  compatibility, so re-check `_StubAdapter` after a Flight Stand Software
  update. The API reports SI units (Newtons, rad/s) — `thrust_is_grams:
  false`, `rpm_is_rad_per_s: true` in the signal map.
* The **AM32 bootloader only jumps to the app when the throttle signal line
  idles low at boot**. An inactive stand output can leave the line high and
  park the ESC in the bootloader after every flash or power-cycle (observed
  on the ARK 4IN1 bench). Live-source bring-up detects this via the perf
  magic, commands zero throttle, resets the MCU, and waits for the app —
  see `_ensure_app_alive` in `hwci/runner.py`.
* `HWCI_PERF` is validated for the STM32F0 (ARK 4IN1). The timestamp macro uses
  the shared `get_timer_us16()` helper, so STM32/GigaDevice/Artery targets
  compile with `HWCI_PERF=1`; NXP and WCH are `#error`-gated (no usable
  free-running 1 µs timer). With the flag unset, every target is untouched.
* CPU load is a statistical idle-residual figure, not a per-function profile
  (the M0 can't do PC sampling); it's stable and comparable run-to-run, which is
  what regression gating needs.
