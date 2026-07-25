#!/bin/bash
# build the SITL with coverage, run the test suite, and produce a gcov
# report over the firmware sources.
#
#   Mcu/SITL/run_coverage.sh                # text summary + html in coverage/
#   Mcu/SITL/run_coverage.sh --no-build     # reuse the current build/gcda
#
# coverage measures the real ESC firmware (Src/) plus the SITL harness
# (Mcu/SITL); the generated DSDL and libcanard vendor code are excluded.
set -e
cd "$(dirname "$0")/../.."   # repo root

BUILD=1
[ "$1" = "--no-build" ] && BUILD=0

if [ "$BUILD" = 1 ]; then
    rm -f obj/AM32_AM32_SITL_CAN_2.20.elf obj/AM32_AM32_SITL_CAN_2.20.d \
          obj/*.gcda obj/*.gcno
    make AM32_SITL_CAN SITL_COVERAGE=1
fi
rm -f obj/*.gcda

mkdir -p coverage
python3 Mcu/SITL/run_ci_tests.py --sitl obj/AM32_AM32_SITL_CAN_*.elf || true

FILTERS=(--filter 'Src/' --filter 'Mcu/SITL/sim/' --filter 'Mcu/SITL/Src/'
         --exclude 'Src/DroneCAN/dsdl_generated/'
         --exclude 'Src/DroneCAN/libcanard/')

echo "=== coverage summary ==="
gcovr --gcov-ignore-parse-errors --root . --object-directory obj \
    "${FILTERS[@]}" --print-summary --txt coverage/coverage.txt

gcovr --gcov-ignore-parse-errors --root . --object-directory obj \
    "${FILTERS[@]}" --html-details coverage/index.html
echo "html report: coverage/index.html"
