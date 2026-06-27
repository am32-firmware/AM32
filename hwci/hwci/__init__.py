"""AM32 hardware-in-the-loop CI harness for the ARK 4IN1 ESC.

This package builds and flashes AM32 firmware, drives a motor on a Tyto Robotics
Flight Stand, and simultaneously reads firmware timing/CPU-load instrumentation
off the STM32F051 (Cortex-M0) over SWD, then turns the captured data into
metrics, baselines and pass/fail reports.

See hwci/README.md for the architecture and the rationale behind reading an
instrumented RAM struct (the M0 has no SWO/ITM/DWT trace hardware).
"""

__version__ = "0.1.0"
