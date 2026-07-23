#!/bin/bash
# build (if needed) and run the SITL under AddressSanitizer
#
# ASan instruments memory accesses at compile time, so the SITL must be
# built with SITL_SANITIZE=address. Unlike valgrind it runs at close to
# full speed, so it is practical to leave on while driving the sim or
# running the test suite.
#
#   ./asan.sh                 # build if stale, then run with --verbose
#   ./asan.sh --config m.json # pass-through args
#
# leak reports only print on a clean exit; the SITL runs until killed,
# so ASan here mainly catches overflows and use-after-free during the
# run rather than leaks at exit.
set -e
cd "$(dirname "$0")"

ELF=obj/AM32_AM32_SITL_CAN_2.20.elf
if ! nm "$ELF" 2>/dev/null | grep -q __asan_init; then
    echo "building the SITL with AddressSanitizer..."
    rm -f "$ELF" "${ELF%.elf}.d"
    make AM32_SITL_CAN SITL_SANITIZE=address
fi

# reports go to asan.<pid> files (log_path) rather than being buried in
# the SITL's verbose stderr; cat asan.* after a run, or watch the dir.
# verify_asan_link_order=0: the SITL execs a non-instrumented bootloader
# at startup (and back) and ASan's execve interceptor propagates libasan
# through the chain, which makes some ASan versions wrongly complain the
# runtime is not first. The app is correctly linked (libasan first in
# its DT_NEEDED, runs clean directly), so the check is spurious here.
ASAN_LOG="${ASAN_LOG:-asan}"
rm -f "$ASAN_LOG".* 2>/dev/null || true
export ASAN_OPTIONS="${ASAN_OPTIONS:-detect_leaks=1:halt_on_error=0:abort_on_error=0:verify_asan_link_order=0:log_path=$ASAN_LOG}"
echo "ASan reports (if any) will be written to $ASAN_LOG.<pid>" >&2

# a bootloader chain is optional; pass --no-bootloader to run the fully
# instrumented app on its own (no exec into the non-instrumented
# bootloader)
BOOT=(--bootloader ../AM32-bootloader.2/obj/AM32_SITL_BOOTLOADER_PB4_CAN_V18.elf)
if [ "$1" = "--no-bootloader" ]; then
    BOOT=()
    shift
fi
exec stdbuf -oL -eL "$ELF" --verbose "${BOOT[@]}" "$@"
