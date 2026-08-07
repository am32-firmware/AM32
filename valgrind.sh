#!/bin/bash
# run the SITL under valgrind memcheck
#
# --fair-sched=yes is essential: valgrind serialises threads behind one
# lock and the default scheduler lets the firmware thread hold it
# through its startup-tune busy-wait, starving the sim thread so sim
# time never advances. stdbuf line-buffers the output (piped stdout
# would otherwise buffer and show nothing). --trace-children keeps
# valgrind attached across the SITL's execv on reset/reboot.
#
# expect ~1/10 real time; the measurement tools should be run with
# --sim-state 127.0.0.1:57734 so they pace against simulation time.
exec stdbuf -oL -eL valgrind -q --fair-sched=yes --trace-children=yes \
    --error-exitcode=0 \
    ./obj/AM32_AM32_SITL_CAN_2.20.elf --verbose \
    --bootloader ../AM32-bootloader.2/obj/AM32_SITL_BOOTLOADER_PB4_CAN_V18.elf \
    "$@"
