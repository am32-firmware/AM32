#!/bin/sh
set -eu
out=${1:?usage: run_demag_guard.sh OUTPUT_DIRECTORY}
mkdir -p "$out"
for mcu in G431 G071; do
    for sense in sense proxy fallback midrail; do
        if [ "$mcu" = G071 ] && [ "$sense" = midrail ]; then
            continue
        fi
        defs="-DMCU_$mcu"
        case "$sense" in
            proxy) defs="$defs -DDEMAG_GUARD_CURRENT=0" ;;
            fallback) defs="$defs -DNO_CURRENT_SENSE" ;;
            midrail) defs="$defs -DTEST_MIDRAIL" ;;
        esac
        ${CC:-cc} -std=c11 -O2 -Wall -Wextra -Werror \
            -fsanitize=address,undefined $defs \
            -Itests/demag_guard_stubs -IInc tests/demag_guard_test.c \
            -o "$out/test-$mcu-$sense"
        "$out/test-$mcu-$sense"
    done
done
