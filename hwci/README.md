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
  thrust, RPM) — the stand enforces them and the run aborts on breach. Secure
  the motor/prop; use a prop guard; keep clear during demag-stress runs.

## Software setup (Ubuntu 24.04)

```bash
cd hwci
./scripts/setup_ubuntu.sh        # apt deps, ARM toolchain, OpenOCD, udev, venv
```

Then, manually (vendor bits the script can't automate):

1. Install the **Tyto Flight Stand Software** and enable its Python/gRPC API.
2. Generate the gRPC stubs from Tyto's `.proto` and make them importable, then
   reconcile method/signal names in `hwci/hwci/flightstand/grpc_client.py`
   (the proto-specific calls are isolated in one `_StubAdapter` class):
   ```bash
   pip install grpcio grpcio-tools
   python -m grpc_tools.protoc -I<proto_dir> \
       --python_out=hwci/flightstand/_generated \
       --grpc_out=hwci/flightstand/_generated <proto_dir>/*.proto
   ```
3. `cp config/rig.example.yaml rig.yaml` and edit ports/host/signal map/motor.
4. Edit `/etc/udev/rules.d/99-hwci.rules` with your USB-serial serial numbers so
   `/dev/esc-telem` (and `/dev/esc-throttle`) appear.

Verify the harness with **no hardware** at any time:

```bash
python -m hwci selftest          # runs ci_smoke in the built-in simulator
python -m pytest                 # 39 offline tests
```

## Usage

```bash
hwci profiles                                   # list test profiles
hwci build --config rig.yaml                    # make ARK_4IN1_F051 HWCI_PERF=1
hwci flash --config rig.yaml                    # OpenOCD program @ 0x08001000
hwci run  --profile efficiency_sweep --config rig.yaml --out runs/r1
hwci analyze runs/r1                            # -> metrics.json
hwci report runs/r1 --baseline baselines/ARK_4IN1_F051.json
hwci ci   --profile ci_smoke --config rig.yaml \
          --baseline baselines/ARK_4IN1_F051.json --out runs/ci   # full gate
```

`hwci ci` builds (HWCI_PERF=1) → flashes → runs the profile → computes metrics →
compares to the baseline → writes `report.md` + `summary.png`, and **exits
non-zero on regression** so CI fails the build.

Built-in profiles: `ci_smoke` (fast gate), `efficiency_sweep` (10–100 %
staircase, the primary baseline), `demag_step_stress` (aggressive steps).

## Capturing the first baseline

Once wired and configured:

```bash
cd hwci
# 1. Sanity-check the full path on hardware with the short profile:
hwci ci --profile ci_smoke --config rig.yaml --out runs/smoke
# 2. Capture the performance baseline:
hwci ci --profile efficiency_sweep --config rig.yaml --out runs/baseline
hwci baseline-save runs/baseline --out baselines/ARK_4IN1_F051.json
git add baselines/ARK_4IN1_F051.json && git commit -m "hwci: ARK 4IN1 baseline"
```

`runs/baseline/report.md` is your benchmark: thrust & efficiency per throttle,
worst-case 20 kHz loop time, CPU load, and zero demag events on the swept ramp.
Every later change is graded against it.

## CI integration

`.github/workflows/hwci.yml` runs on a self-hosted runner labelled
`[self-hosted, hwci]` wired to the rig. It triggers on manual dispatch (pick a
profile) or when a maintainer adds the **`hw-test`** label to a PR, serializes
hardware access with a concurrency group, posts `report.md` to the job summary,
uploads the run data, and fails the job on regression. Set the repo/runner
variable `HWCI_RIG_CONFIG` to the path of `rig.yaml` on the bench.

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
hwci/tests/                        39 offline tests (sim + DWARF layout check)
```

## Honest limitations

* The **gRPC client** is structured against Tyto's documented model (boards →
  numbered input signals, thrust FZ=11 → outputs) but the exact rpc/field names
  ship with the Flight Stand Software; confirm them in `_StubAdapter` on the
  bench. Everything else is backend-agnostic and the simulator exercises the
  whole pipeline today.
* `HWCI_PERF` is validated for the STM32F0 (ARK 4IN1). The hooks are no-ops on
  other targets unless ported (the timestamp source differs per MCU).
* CPU load is a statistical idle-residual figure, not a per-function profile
  (the M0 can't do PC sampling); it's stable and comparable run-to-run, which is
  what regression gating needs.
