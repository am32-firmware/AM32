# AM32 SITL (software in the loop)

Runs the AM32 firmware as a native Linux executable against a simulation
of the motor, bridge and battery, with DroneCAN input/output over
multicast UDP. This allows testing of the firmware logic (startup,
commutation, DroneCAN protocol, parameters) without ESC hardware.

## Building

```
make AM32_SITL_CAN
```

produces `obj/AM32_AM32_SITL_CAN_<version>.elf`, a normal Linux
executable.

### Sanitizers and valgrind

`SITL_SANITIZE=address` builds with AddressSanitizer (heap/stack
overflow and use-after-free), `SITL_SANITIZE=undefined` with UBSan, and
both can be combined (`address,undefined`):

```
make AM32_SITL_CAN SITL_SANITIZE=address
```

Both run at close to full speed, so it is practical to leave a
sanitizer on while driving the sim or running the whole test suite.
`./asan.sh` and `./ubsan.sh` build the respective sanitizer if stale
and run it, writing any reports to `asan.<pid>` / `ubsan.<pid>` files
so they are not buried in the SITL's verbose output. ASan's leak
detector only reports on a clean exit, so under the (kill-terminated)
test harness ASan catches overflows and use-after-free during the run
rather than leaks at exit. The whole test suite passes under each, and
both run as their own CI jobs.

`./valgrind.sh` runs the normal build under valgrind memcheck instead
(slower - about a tenth of real time - but needs no special build).
`--fair-sched=yes` is essential there or the firmware busy-wait starves
the sim thread; the wrapper sets it.

## Running

```
obj/AM32_AM32_SITL_CAN_*.elf --node-id 10 --verbose
```

Options:

- `--config FILE` JSON file with motor/battery/esc/sim properties (see
  `example.json`, all keys optional)
- `--eeprom FILE` eeprom backing file (default `am32_eeprom.bin`). A
  missing file is seeded with the AM32 configurator default settings
- `--can-uri URI` CAN interface, default `mcast:0` (group 239.65.82.N
  port 57732, wire compatible with libcanard and ArduPilot SITL). An
  optional interface may be given (`mcast:0:lo`). `none` disables CAN,
  for pure PWM/DShot testing
- `--node-id N` force the DroneCAN node ID, otherwise DNA is used
- `--input-port N` UDP port for PWM/DShot input (default 57733, 0
  disables)
- `--state-port N` UDP port for high rate simulation state streaming
  and runtime motor model loading (default 57734, 0 disables)
- `--bind-any` bind the input and state ports on all interfaces instead
  of loopback only, needed when the GUI runs on a different host. The
  ports accept unauthenticated control, so only use on trusted networks
- `--input-type N` force the eeprom INPUT_SIGNAL_TYPE setting (0=auto
  1=dshot 2=servo 5=dronecan)
- `--speedup X` simulation speed relative to wall clock, 0 = free running
- `--uid STR` string used to derive the 16 byte unique ID
- `--verbose` 1Hz state line on stderr
- `--nosleep` busy wait instead of sleeping. Uses two full CPU cores but
  avoids OS sleep/wakeup latency for the most accurate wall clock pacing
- `--bootloader ELF` chain with the bootloader SITL from the
  am32-bootloader repo: the process boots into the bootloader first
  (as hardware does) and every reset lands back in it with the right
  reset cause; the bootloader execs this firmware on jump. The two
  share the eeprom file, the input/CAN ports, and `<eeprom>.bkup` (the
  RTC backup registers carrying the DroneCAN firmware-update handoff).
  The bootloader keeps its flash in `<eeprom>.blflash`. Default off

The virtual ESC can then be controlled with the DroneCAN GUI tool or
pydronecan on `mcast:0`. A test script is included:

```
python3 Mcu/SITL/sitl_can_test.py --throttle 0.5 --duration 20
```

which arms, ramps the throttle via `esc.RawCommand` and reports the
`esc.Status` telemetry (RPM, voltage, current, temperature).

Note that with no DroneCAN traffic directed at the node it will reboot
every 2 seconds from the firmware signal-timeout logic, exactly as real
hardware does. Reboots (including `RestartNode` and watchdog resets)
re-exec the process; the eeprom file persists.

Each instance holds a lock on its eeprom file: two instances sharing an
eeprom (and therefore a node ID) would interleave their DroneCAN
transfers on the bus, which shows up as erratic telemetry. To run
multiple ESCs give each its own `--eeprom` and `--node-id`.

## PWM/DShot input over UDP

The SITL listens on a UDP port (default 57733) for PWM or DShot input
frames. Each packet is one frame on the virtual signal wire, synthesized
into input-capture edge timestamps and decoded by the firmware's
unmodified `Src/signal.c`/`Src/dshot.c` logic, including input type
auto-detection, CRC checking, zero-throttle arming, DShot commands and
bidirectional DShot auto-detect (idle high line).

packet format (little endian):

| field | size | meaning |
|-------|------|---------|
| magic | u16  | 0x4453 |
| type  | u8   | 0=PWM 1=DSHOT150 2=DSHOT300 3=DSHOT600 4=SERIAL19200 5=LINE_LEVEL |
| len   | u8   | payload bytes after the header (4; for type 4 the number of serial bytes, 1..200) |
| flags | u16  | bit0: line idle level (1 = idle high, bidir DShot; for type 5 the constant level), bit1: line floating (types 4/5) |
| data  | u16  | PWM pulse width in us, or the full 16 bit DShot frame; for type 4 replaced by the raw serial bytes |

Bidirectional DShot replies (eRPM plus extended telemetry frames) are
sent back to the most recent sender in the same format, with `data`
carrying the 16 bit GCR-decoded reply frame.

Type 4 carries raw bytes framed as 19200 baud 8N1 on the simulated
signal wire and type 5 sets a constant line state (driven high, driven
low, or floating). Both exist for the bootloader SITL (see the
am32-bootloader repo), which bit-bangs the 4-way configuration protocol
on the signal pin and detects the input type from the line state at
boot; the main firmware ignores type 4 and uses type 5 only as the idle
line level. The bootloader replies with type 4 packets carrying its
serial output. `sitl_fourway.py` implements the 4-way client side.

Tools in `Mcu/SITL/`:

- `sitl_gui.py` — Qt (PySide6) GUI driving both the PWM/DShot input and
  DroneCAN input with per-input enable switches (for failover testing),
  BDShot/EDT and esc.Status telemetry with rates, and an
  `INPUT_SIGNAL_TYPE` parameter panel. The simulation panel selects the
  motor model (the JSON files in `Mcu/SITL/models/`, applied to the
  running simulation over the state port; switch at zero throttle for
  clean results) and has optional high rate views, both default off:
  pyqtgraph scopes of the phase currents and the phase terminal
  voltages, each in its own window (sample period down to the 500ns
  physics step and adjustable window; the sample rate is automatically
  limited to about 200k samples/s of wall clock, so fine periods take
  effect as the speedup is lowered — the PWM dead time diode conduction
  is visible on the voltages at fine sample periods), and a motor/bridge
  animation showing rotor angle, per phase bridge modes and the
  comparator. A speedup slider (0.01x to 2x)
  changes the simulation pace at runtime, for watching the animation in
  slow motion; input frames arriving faster than the slowed simulation
  consumes them are dropped, as on a real wire. A stuck rotor slider
  blocks the prop with a virtual obstruction (think of a branch caught
  in the prop), from free through partial drag to completely stuck, for
  exercising the firmware's `STUCK_ROTOR_PROTECTION`; the Release
  button clears it. `--control-port N` accepts UI
  commands over a localhost TCP connection for scripted tests (default
  off); `--log FILE` records every UI action with timestamps and
  `--replay FILE` plays a recording back, so a failing interactive
  session can be reproduced exactly. Install the
  dependencies (PySide6, pyqtgraph, dronecan; Linux/Windows/macOS) into
  a self-contained environment with

```
python3 Mcu/SITL/make_gui_env.py
```

  which creates `Mcu/SITL/venv` and prints the interpreter to run the
  GUI with. A system python with the packages from
  `Mcu/SITL/requirements-gui.txt` installed works too. The UI backends
  live in `sitl_gui_backend.py`, UI-independent for headless tests
- `dshot_test.py` — headless scripted test (arming, throttle, EDT,
  bad-CRC injection), e.g.:

```
obj/AM32_AM32_SITL_CAN_*.elf --can-uri none --input-type 1
python3 Mcu/SITL/dshot_test.py --type dshot600 --bidir --edt --throttle 800
```

Note that the eeprom default `INPUT_SIGNAL_TYPE` is DRONECAN_IN, which
disables the PWM/DShot input interrupts at startup — set it to 0/1/2
first (via `--input-type`, the GUI parameter panel, or
`dshot_test.py --input-type`). Also be aware of the current firmware
input arbitration: once any `esc.RawCommand` has been received, the 1kHz
DroneCAN input keep-alive overrides the `dshot`/`inputSet` flags and
PWM/DShot input is dead until a reboot (signal timeout after the CAN
stream stops) followed by zero-throttle re-arming. Running both inputs
at once exercises exactly this behaviour, which is what the input
priority/failover parameter work is developing against.

## macOS

Builds and runs natively (Apple Silicon or Intel) with the stock Xcode
command line tools: `make AM32_SITL_CAN`. The GUI bootstrap is the same
`python3 Mcu/SITL/make_gui_env.py`. Multicast CAN over loopback works
without configuration.

## Windows

The SITL builds under Cygwin (packages: gcc-core, make) with the same
`make AM32_SITL_CAN`, producing a native console executable, and the
POSIX signal based scheduler runs correctly under the Cygwin runtime.
The GUI uses a normal Windows python: `py Mcu/SITL/make_gui_env.py`
creates the environment and prints the interpreter to use; after that
`sitl_gui.bat` in the repository root launches the GUI (double click or
from cmd, extra arguments are passed through). Notes:

- to run the binary from outside a Cygwin shell (cmd, double click),
  copy `C:\cygwin64\bin\cygwin1.dll` next to it - its only Cygwin
  dependency (the CI artifact ships it bundled).
- Windows Firewall must allow inbound UDP for the SITL binary (or ports
  57732-57734) for CAN and the input/state ports to receive.
- a socket never receives its own multicast on Windows, so the CAN TX
  self test is skipped there; on a machine with several interfaces pass
  an explicit one as `--can-uri mcast:0:<ip>`.

## Headless / CI use

Everything runs without a display: the SITL is a plain console binary
and the GUI works under Qt's offscreen platform
(`QT_QPA_PLATFORM=offscreen`) driven through `--control-port`, so full
interactive scenarios can run in CI. On a minimal Debian/Ubuntu the
requirements are:

```
apt install gcc make python3 python3-venv \
    libgl1 libegl1 libfontconfig1 libxkbcommon0
pip install dronecan        # for the DroneCAN tests
python3 Mcu/SITL/make_gui_env.py   # for GUI-driven tests
```

Multicast CAN over loopback works on a stock VM with no route
configuration (the SITL self-tests its TX at startup). Timing notes for
slow or virtualised runners: the simulation paces itself and reports
the achieved ratio in `--verbose` (x1.00 = real time); the python test
senders keep their average frame rate under coarse sleep granularity by
sending catch-up bursts, which matters because the firmware's
bidirectional DShot auto-detect needs more than 100 frames before
zero-throttle arming completes, putting a floor of roughly 100Hz on the
usable frame rate.

## Debugging with gdb

The SITL debugs like any host program, with one wrinkle: emulated
interrupts are delivered by parking the firmware thread with SIGUSR1,
which gdb must pass through silently. `Mcu/SITL/gdbinit` sets that up:

```
gdb -x Mcu/SITL/gdbinit --args obj/AM32_AM32_SITL_CAN_*.elf --can-uri none --input-type 1
(gdb) break tenKhzRoutine
(gdb) run
```

Attach to a running simulator with `gdb -x Mcu/SITL/gdbinit -p <pid>`.
Stopping (a breakpoint, single stepping, ^C) freezes the firmware and
the simulation together: simulated time, the emulated watchdog and the
ESC's own timeouts all stop with the process, and on resume the pacer
rebases to the wall clock instead of sprinting through the backlog, so
pausing under the debugger never breaks the physics. For deterministic
stepping of firmware code with simulated time frozen between steps use
`set scheduler-locking step`.

## Architecture

The firmware runs unmodified (built as `am32_main()`) in one thread. A
simulation thread owns simulated time, advancing it in fixed physics
steps (500ns default) and delivering emulated interrupts (BEMF
comparator, commutation timer, 20kHz loop timer, CAN RX) by suspending
the firmware thread with a signal and running the handler, reproducing
the run-to-completion interrupt semantics of the real MCU. Simulated
time is decoupled from wall time and paced to `--speedup`.

The emulated MCU follows the G431 target: TIM1 PWM generation with
preloaded ARR/CCR, a 2MHz interval timer, one-shot commutation timer,
20kHz loop timer and 1MHz utility timer, plus comparator/EXTI blanking
behaviour matching `Mcu/g431`. Timer reads from interrupt context step
the physics, so `delayMicros()` inside handlers and the comparator
filter loop behave as on hardware.

The motor model (`sim/motor.c`) is a trapezoidal back-EMF BLDC model
based on open-bldc-csim: per-phase currents, solved star point voltage
(so the floating phase terminal voltage and its zero crossings are
physical), fet Rds_on, body diode clamping of a floating phase carrying
current, and battery voltage sag from internal resistance. The
comparator compares the floating phase against the virtual neutral with
configurable noise and hysteresis, so the firmware's blanking and
filtering logic is genuinely exercised at PWM switching level.
