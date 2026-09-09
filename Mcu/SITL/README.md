# AM32 SITL (software in the loop)

Runs the AM32 firmware as a native executable against a simulation of
the motor, bridge and battery, with DroneCAN input/output over
multicast UDP. This allows testing of the firmware logic (startup,
commutation, DroneCAN protocol, parameters) without ESC hardware.

This directory is the simulator itself: the emulated MCU peripherals
(`Inc/`, `Src/`) and the motor/battery physics (`sim/`). It builds as a
normal target of this repository:

```
make AM32_SITL_CAN
```

producing `obj/AM32_AM32_SITL_CAN_<version>.elf`. `SITL_SANITIZE=address`
or `=undefined` builds it under a sanitizer, `SITL_COVERAGE=1` under
gcov, and `SITL_CROSS=win` cross-builds a native Windows exe with
MinGW-w64.

Everything around the simulator - the motor models, the calibration
datasets captured from real hardware, the Qt control GUI, the DroneCAN
measurement tools and the test suites - lives in the
[ESCSim](https://github.com/am32-firmware/ESCSim) repository under
`SITL/`, which pins this repository as the `modules/am32-firmware`
submodule. Keeping them there means one shared copy is used to test
every AM32 branch, rather than each branch carrying its own.

To run the tests against a firmware checkout:

```
git clone https://github.com/am32-firmware/ESCSim
make AM32_SITL_CAN
AM32_ROOT=$PWD python3 ESCSim/SITL/run_ci_tests.py
```

which is what the SITL CI jobs do. See `ESCSim/SITL/README.md` for the
runtime options, the UDP PWM/DShot input protocol, the GUI, gdb use and
the simulator architecture, and `ESCSim/SITL/TIMING-DESIGN.md` for the
design of the host-timing-immune scheduler in `Src/sitl_sched.c`.
