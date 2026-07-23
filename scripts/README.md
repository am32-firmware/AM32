# AM32 DroneCAN scripts

pydronecan tools for driving and measuring AM32 ESCs — real hardware or
the SITL build — over DroneCAN. Developed for calibrating the SITL
motor/ESC model against real hardware.

All tools take `--uri` (default `mcast:0:lo`). A real ESC can be bridged
onto the multicast bus with `dronecan_bridge.py` from pydronecan; the
SITL binary joins the bus natively with `--can-uri`.

## capturing calibration data from a real ESC/motor setup

To contribute a calibration data set for a new ESC/motor combination,
capture the standard battery below and send the JSONL files along with
a description of the setup. The fitting procedure that turns the data
into a SITL model is in `CALIBRATION.md`; the existing data sets under
`Mcu/SITL/data/` show the result.

Record with the data: ESC model and AM32 build target, motor
(size/model, nameplate KV, pole count), propeller if fitted, supply
voltage and any current limit (bench supply rating and the eeprom
CURRENT_LIMIT if you set one), and anything unusual about the rig.

Setup, once the ESC is reachable over DroneCAN (`dronecan_bridge.py`
for a serial CAN adapter):

- set the eeprom for the capture (values are what our fits assume):
  `MOTOR_POLES` correct for the motor (rpm scaling depends on it),
  **`TELEM_RATE 200`** (the chirp analysis needs the high-rate
  telemetry), `DEBUG_RATE 100` (FlexDebug commutation-interval data):
  `can_params.py --node-id N set TELEM_RATE 200` etc., then
  `set <last one> --save`
- **disable or silence any other ESCs on the same CAN bus** —
  disconnect them, or set their `TELEM_RATE 0` and `DEBUG_RATE 0` —
  so the bus has bandwidth headroom at 200 Hz and the logs only
  contain the ESC under test
- sanity-check the current sense with a zero-throttle hold
  (`esc_measure.py --node-id N hold --throttle 0 --hold 5`): a large
  non-zero reading at rest means the sense offset/gain is off (a
  known stock-firmware issue on some targets); note it — rpm and
  voltage stay valid regardless
- pick the abort limit for your rig and pass it to every command
  below as `--max-current` (the tools cut throttle if it is
  exceeded); with a propeller start gently and check current at each
  level before going higher

The capture battery (matching the existing data sets; motor spins in
all of them):

```
# first spin, sanity
esc_measure.py --node-id N hold --throttle 0.1 --hold 6 --max-current A --log first_spin_010.jsonl

# steady staircase, 4s per level, up then down. No prop: 0.05..0.50.
# With a prop go as high as current allows, e.g. 0.05..0.95:
esc_measure.py --node-id N sweep --levels 0.05,0.1,...,0.5 --hold 4 \
    --max-current A --max-throttle 1.0 --log sweep1.jsonl

# 120s exponential frequency chirp at 0.35 throttle +/-0.15
esc_chirp.py --node-id N run --duration 120 --f-start 0.5 --f-stop 40 \
    --max-current A --log chirp_120s.jsonl

# square-wave amplitude sweep (mid 0.35, growing deltas, 3 cycles)
esc_square.py --node-id N run --max-current A --log square1.jsonl
```

The chirp and square operate between roughly 0.11 and 0.59 throttle:
check the sweep's current at 0.6 fits your supply before running them.
A quick look at the results: `esc_analyze.py sweep1.jsonl`,
`esc_chirp.py fit chirp_120s.jsonl --plot`, `esc_square.py analyze
square1.jsonl`, and `esc_plot.py <log>` for time series.

## capturing via a Betaflight flight controller (no CAN needed)

`esc_capture_fc.py` runs the same capture battery with the ESC wired
to a Betaflight FC instead of a CAN adapter: it drives the motor
through the FC's motor-test path over MSP (USB) and logs the FC's
bidirectional-DShot telemetry — eRPM plus the EDT
voltage/current/temperature frames — to the same JSONL files. Needs
only `pip install pyserial`.

**REMOVE ALL PROPELLERS before using it.** Betaflight keeps the last
motor-test value it was given, and applies no failsafe of its own to
it. The tool aborts on over-current, over-temperature, telemetry loss
or arming, and commands motor stop when it exits — including on
Ctrl-C, an abort, or SIGTERM — but it cannot do so if the USB link
dies or the process is killed outright. **Power the ESC from a
current-limited bench supply you can switch off**, and do not rely on
software as the only way to stop the motor.

Betaflight setup (CLI, then `save`):

```
set motor_pwm_protocol = DSHOT300   # or DSHOT600
set dshot_bidir = ON
set motor_poles = 14                # your motor's pole count
set dshot_edt = ON                  # BF 4.4+: EDT volt/current/temp
feature -3D                         # 3D mode makes 1000 full reverse
```

The capture battery, ESC on motor output 1 (`--poles` must match the
motor and the FC setting: it scales every logged rpm):

```
esc_capture_fc.py hold  --port /dev/ttyACM0 --poles 14 --throttle 0.1 --hold 6 --max-current A --log first_spin_010.jsonl
esc_capture_fc.py sweep --port /dev/ttyACM0 --poles 14 --levels 0.05,0.1,...,0.5 --hold 4 \
    --max-current A --max-throttle 1.0 --log sweep1.jsonl
esc_capture_fc.py chirp --port /dev/ttyACM0 --poles 14 --duration 120 --f-start 0.5 --f-stop 40 \
    --max-current A --log chirp_120s.jsonl
esc_capture_fc.py square --port /dev/ttyACM0 --poles 14 --max-current A --log square1.jsonl
```

Differences from the DroneCAN path:

- telemetry arrives at the MSP poll rate (~100 Hz, enough for the
  chirp fit) instead of `TELEM_RATE`
- EDT values are coarse: Betaflight passes the DShot telemetry through
  nearly raw, so voltage lands in whole volts and current in 0.5 A
  steps. Note your bench supply readings alongside the capture
- without `dshot_edt` there is no voltage/current/temperature at all,
  and `--max-current`/`--max-temp` cannot protect anything; the tool
  warns when it sees no EDT frames
- Betaflight never marks its DShot telemetry stale: if the ESC's
  replies stop arriving it keeps serving the last values it decoded.
  The tool notices when telemetry fails to respond to a throttle
  change, but during a constant-throttle hold cached values are
  indistinguishable from live ones, so the current and temperature
  limits can be acting on stale readings — one more reason for the
  switchable supply
- `err` is the BDShot invalid-frame percentage, not the desync counter
- ESC eeprom settings cannot be changed over the FC link: set
  `motor_poles` on the FC and everything else with the AM32
  configurator first, and include the ESC's settings dump with a
  submission so the SITL model can mirror them

## simulation-time pacing (SITL only)

All three measurement tools accept `--sim-state [HOST:]PORT` pointing
at the SITL `--state-port`. The run is then paced against the
simulation clock instead of the wall clock: hold durations, settle
windows and chirp frequencies are in *simulated* seconds, and logged
timestamps are simulated seconds too. A machine that runs the
simulation at half speed simply takes twice as long in wall time and
measures exactly the same experiment, which is what makes the
calibration suite usable on slow CI runners. Real hardware keeps the
wall clock (there, wall seconds are physical seconds).

Verified equal at 1.0x, 0.5x and 0.2x simulation speed (identical rpm
per level to within 1 rpm). Implementation: `scripts/sim_clock.py`.

## esc_measure.py

Drives an ESC with `esc.RawCommand` test profiles while logging
`esc.Status`, AM32 `FlexDebug` (debug1) and `NodeStatus` to a JSONL
file, then prints per-throttle-level steady-state summaries
(rpm/rpm-stddev/voltage/current).

Profiles:

- `sweep` — throttle staircase up then back down (`--levels`, `--hold`):
  steady-state rpm/current/voltage curves
- `step` — zero→level step then cut to zero (`--hold`, `--coast`):
  spin-up transients (inertia) and spin-down coast (friction/drag)
- `hold` — single constant throttle (`--throttle`, `--hold`)

Safety: aborts to zero throttle if reported current exceeds
`--max-current` (default 4 A) or temperature exceeds `--max-temp`
(default 80 C); commanded throttle is clamped to `--max-throttle`
(default 0.6). Always leaves the ESC at zero throttle on exit.

```
esc_measure.py sweep --node-id 124 --log sweep.jsonl
esc_measure.py step --node-id 124 --levels 0.2,0.4 --log step.jsonl
esc_measure.py hold --node-id 124 --throttle 0.1 --hold 3
```

`--telem-rate`/`--debug-rate` set the ESC's TELEM_RATE/DEBUG_RATE
parameters (RAM only, not saved) before the run; FlexDebug is only sent
when DEBUG_RATE > 0.

The JSONL rows are `{"t": seconds, "type": ...}` with types `status`
(esc.Status), `debug1` (FlexDebug decode), `node` (NodeStatus), `cmd`
(commanded throttle changes), `mark` (profile phase labels).

debug1 decoding lives in `am32_debug.py` and handles both layouts:
v1 (18 bytes) and v2 (29 bytes, adds `duty`, `duty_max` low-rpm/temp
duty clamp, `adj_input` post-mapping throttle, `adc_current`/`adc_volts`
raw ADC counts, and `armed`/`running`/`sine` flags). debug1 `ci` is the
commutation interval in 0.5 us ticks: mech rpm = 2.857e6/ci for a
14-pole motor.

## can_params.py

DroneCAN parameter client for AM32 ESCs:

```
can_params.py --node-id 124 list
can_params.py --node-id 124 get MOTOR_KV
can_params.py --node-id 124 set MOTOR_KV 1100 --save
```

`set` is RAM-only unless `--save` is given (eeprom save opcode); without
a save the ESC reverts on reboot.

## esc_chirp.py

Frequency-response test: sinusoidal throttle demand with an
exponential frequency sweep (same waveform as ArduPilot's
actuator_chirp.lua — dwell at f_start, Hann fade in/out), logging rpm
telemetry to the same JSONL format. The rpm amplitude rolloff with
frequency characterises the powertrain bandwidth; braking limitations
show as a rise of mean rpm with frequency.

```
esc_chirp.py run --node-id 124 --log chirp.jsonl \
    --throttle-mid 0.35 --throttle-amp 0.15 --f-start 0.5 --f-stop 40
esc_chirp.py analyze chirp.jsonl --compare sitl_chirp.jsonl
```

`--param NAME=VALUE` (repeatable) sets ESC parameters before the run,
e.g. AM32 braking settings. The analyzer prints coherent gain/phase
(quadrature demodulation against the reconstructed chirp phase) and an
incoherent gain (detrended RMS) per log-spaced frequency bin. Over a
CAN bridge with variable latency the coherent columns are unreliable
at high frequency — compare the `inc_db` column; phase is only
meaningful for SITL or a low-jitter transport.

`fit` extracts the characteristic numbers: per-chirp-cycle up-swing
(accelerating) and down-swing (braking) rpm envelopes against the
low-frequency baseline, the empirical -3dB (70.7%) frequency of each,
and an octave-weighted envelope fit of four asymmetric LPF candidates
(1-pole, 2-pole, mixed order, 1-pole with braking slew limit). `plot`
overlays the fitted model envelopes on the rpm response with the best
model's 3dB points highlighted; both commands show the data first and
fit in the background. `--max-freq` cuts noisy high-frequency data
from display and fit. `--raw FILE` substitutes a SITL `--physics-log`
raw log (aligned by rpm cross-correlation) for the telemetry rpm, to
separate physics behaviour from the telemetry path.

## esc_plot.py

Interactive time-series viewer for any of the JSONL logs: throttle
demand on a top graph, rpm on a bottom graph, with a linked time axis
so zooming either keeps both in sync (phase lag stays visible without
the traces sharing one busy plot). Give two logs to overlay them for
comparison, aligned at each log's first throttle command; `--save
out.png` renders headless; `--html out.html` writes a single
self-contained interactive page (plotly.js and all data embedded,
~5MB) that can be put on a web server and viewed with nothing
installed.

```
esc_plot.py square1.jsonl sitl_square.jsonl
esc_plot.py square1.jsonl sitl_square.jsonl --html cmp.html
esc_plot.py square1.jsonl sitl_square.jsonl --steps --html steps.html
```

`--steps` (html only) renders a per-step overlay grid instead of the
timeline: every throttle step grouped by amplitude and direction,
each occurrence aligned at its command edge in ms. Use it for
square-test comparisons - two runs settle at different rates, so
their timelines diverge and overlaying them directly is misleading.

Requires the `plotly` python package for `--html` only
(`pip install --user plotly`).

## esc_report.py

Generates a static html report of real-vs-SITL calibration results
into a directory ready to rsync to a web server: an index page
describing each data set, linking to per-dataset pages with the
interactive chirp, square and steady-sweep comparison plots.
plotly.js is written once into the directory and shared by all
pages, so everything works offline with nothing installed. Edit the
DATASETS table at the top of the script to add a data set or reword
a description.

```
esc_report.py --out report
rsync -av report/ myserver:public_html/am32-sitl/
```

## Mcu/SITL/run_calibration_tests.py

Calibration regression tests (used by the SITL-calibration CI
workflow): runs the bench measurement battery (steady sweep, square
amplitude sweep, 120s chirp) against each calibrated model and
compares to the REAL hardware reference values in
`Mcu/SITL/data/<TARGET>/expected.json`, using each dataset's
committed `sitl_eeprom.bin` so the SITL runs the same ESC settings
as the bench. Tolerances cover the documented model residuals plus
run-to-run scatter; the whole suite self-skips (exit 0) on runners
that cannot hold the sim/wall ratio near 1, since every test is
wall-clock paced; desyncs check against a per-model budget (the 1404
reproduces the real hardware's marginality by design). Real-vs-run
comparison graphs land in --artifacts.

## esc_analyze.py

Analyses esc_measure.py JSONL logs: steady-state table per throttle
level, and with `--transients` ensemble-averaged step responses pooled
across repeated cycles (only the most common from→to level pair is
pooled). `--compare REF.jsonl` prints a second log side by side with
deltas — the core loop for calibrating a SITL model json against a
hardware log:

```
esc_analyze.py sitl_cycles.jsonl --compare real_cycles.jsonl --transients
```

## sitl_model.py

Live-reloads a motor/battery/esc model json into a running SITL via the
state port (LOAD_MODEL): `sitl_model.py model.json --port 57934`. The
port must match the SITL's `--state-port`. sim.* keys need a SITL
restart; reload with the motor stopped.

## can_monitor.py

Passive (anonymous, transmit-free) bus monitor: NodeStatus per node with
reboot/bootloader detection, esc.Status telemetry, decoded AM32 debug1
FlexDebug, LogMessage broadcasts, and message-rate totals on exit.

```
can_monitor.py --node-id 124 --duration 10
```

## AM32 CAN bootloader "no signal" gate

On a bus with any CAN traffic, the AM32 CAN bootloader will not take the
no-CAN fallback boot path; after reset it waits in the bootloader
(NodeStatus mode=MAINTENANCE, LogMessage `no signal` every 5 s) until it
sees a RawCommand. esc_measure.py and can_params.py stream zero-throttle
RawCommand while waiting so the app boots automatically. The app also
reboots when idle with no RawCommand stream, so an untouched ESC on a
quiet-ish bus parks in the bootloader — this is normal.
