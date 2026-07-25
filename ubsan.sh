#!/bin/bash
# build (if needed) and run the SITL under UndefinedBehaviorSanitizer
#
# UBSan instruments operations at compile time (signed overflow, bad
# shifts, misaligned or null pointers, out-of-range enum/bool, etc), so
# the SITL must be built with SITL_SANITIZE=undefined. It runs at close
# to full speed, so it is practical to leave on while driving the sim or
# the test suite.
#
#   ./ubsan.sh                 # build if stale, then run with --verbose
#   ./ubsan.sh --config m.json # pass-through args
#
# each undefined operation is reported (with a stack) as it happens and
# the SITL keeps running (halt_on_error=0).
set -e
cd "$(dirname "$0")"

ELF=obj/AM32_AM32_SITL_CAN_2.20.elf
if ! nm "$ELF" 2>/dev/null | grep -q __ubsan_handle_add_overflow; then
    echo "building the SITL with UndefinedBehaviorSanitizer..."
    rm -f "$ELF" "${ELF%.elf}.d"
    make AM32_SITL_CAN SITL_SANITIZE=undefined
fi

# reports go to ubsan.<pid> files (log_path) rather than being buried in
# the SITL's verbose stderr; cat ubsan.* after a run, or watch the dir.
UBSAN_LOG="${UBSAN_LOG:-ubsan}"
rm -f "$UBSAN_LOG".* 2>/dev/null || true
export UBSAN_OPTIONS="${UBSAN_OPTIONS:-print_stacktrace=1:halt_on_error=0:log_path=$UBSAN_LOG}"
echo "UBSan reports (if any) will be written to $UBSAN_LOG.<pid>" >&2

# a bootloader chain is optional; pass --no-bootloader to run the fully
# instrumented app on its own (no exec into the non-instrumented
# bootloader)
BOOT=(--bootloader ../AM32-bootloader.2/obj/AM32_SITL_BOOTLOADER_PB4_CAN_V18.elf)
if [ "$1" = "--no-bootloader" ]; then
    BOOT=()
    shift
fi
exec stdbuf -oL -eL "$ELF" --verbose "${BOOT[@]}" "$@"
