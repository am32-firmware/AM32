#!/usr/bin/env bash
# Shared by the Linux command line and the native Windows VS Code task.
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."

action=${1:---build}
if [[ $# -gt 1 || ($action != --build && $action != --check) ]]; then
    echo "Usage: bash env_setup_scripts/sitl_build.sh [--build|--check]" >&2
    exit 2
fi
for tool in gcc make awk cp; do
    if ! command -v "$tool" >/dev/null; then
        echo "Missing $tool. Run the SITL setup script for your operating system." >&2
        exit 1
    fi
done

suffix=
case $(uname -s) in
    CYGWIN*)
        suffix=.exe
        if [[ $(gcc -dumpmachine) != x86_64-*-cygwin ]]; then
            echo 'SITL requires the 64-bit Cygwin GCC, not MinGW or ARM GCC.' >&2
            exit 1
        fi
        ;;
    Linux) ;;
    *) echo 'This workflow supports Linux and 64-bit Cygwin on Windows.' >&2; exit 1 ;;
esac
echo "Compiler: $(command -v gcc) ($(gcc -dumpmachine))"
gcc --version | sed -n '1p'
make --version | sed -n '1p'
[[ $action == --check ]] && exit 0

# The firmware makefile compiles all translation units in one invocation.
# Rebuild explicitly so header, flag and toolchain changes are always included.
# Use a separate directory from the hardware firmware's obj/debug.elf.
make --no-print-directory -B AM32_SITL_CAN OBJ=build/sitl SHELL=/bin/bash \
    SITL_CC=gcc SITL_CROSS= SITL_SANITIZE= SITL_COVERAGE=
version=$(awk '$2 == "VERSION_MAJOR" {major=$3} $2 == "VERSION_MINOR" {minor=$3}
               END {print major "." minor}' Inc/version.h)
binary="build/sitl/AM32_SITL_CAN$suffix"
cp -f "build/sitl/AM32_AM32_SITL_CAN_$version.elf" "$binary"
if [[ -n $suffix ]]; then
    cp -f /bin/cygwin1.dll build/sitl/cygwin1.dll
    binary=$(cygpath -aw "$binary")
else
    binary="$PWD/$binary"
fi
echo
echo "Built: $binary"
echo 'Stop the simulation, select this file with SITL binary Browse, then start it again.'
