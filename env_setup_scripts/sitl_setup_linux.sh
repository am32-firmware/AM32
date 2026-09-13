#!/usr/bin/env bash
# Debian/Ubuntu host compiler; the ARM SDK is not used by SITL.
set -euo pipefail
if ! command -v apt-get >/dev/null; then
    echo 'Install GCC, GNU Make and Git with your package manager, then run:' >&2
    echo '  bash env_setup_scripts/sitl_build.sh --check' >&2
    exit 1
fi
sudo apt-get update
sudo apt-get install -y build-essential git
bash "$(dirname -- "${BASH_SOURCE[0]}")/sitl_build.sh" --check
echo 'Open AM32-SITL.code-workspace in VS Code and press Ctrl+Shift+B.'
