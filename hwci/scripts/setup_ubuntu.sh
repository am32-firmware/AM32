#!/usr/bin/env bash
# Provision an Ubuntu 24.04 box as an AM32 hardware-CI bench / self-hosted runner.
# Idempotent: safe to re-run. Does NOT install the Flight Stand Software (vendor
# binary) or generate its gRPC stubs - see hwci/README.md for those steps.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HWCI_DIR="$(dirname "$HERE")"
REPO_ROOT="$(dirname "$HWCI_DIR")"

echo "==> apt packages"
sudo apt-get update
sudo apt-get install -y build-essential git python3-venv python3-pip openocd usbutils

echo "==> ARM toolchain"
if ! ls "$REPO_ROOT"/tools/linux/*/bin/arm-none-eabi-gcc >/dev/null 2>&1 \
   && ! command -v arm-none-eabi-gcc >/dev/null 2>&1; then
  ( cd "$REPO_ROOT" && make arm_sdk_install ) || sudo apt-get install -y gcc-arm-none-eabi
fi

echo "==> udev rules (ST-Link + serial symlinks)"
sudo cp "$HERE/99-hwci.rules" /etc/udev/rules.d/99-hwci.rules
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "==> serial/usb group membership"
sudo usermod -aG dialout,plugdev "$USER" || true

echo "==> python harness venv"
cd "$HWCI_DIR"
python3 -m venv .venv
# shellcheck disable=SC1091
. .venv/bin/activate
pip install --upgrade pip
pip install -e '.[plot,flightstand,dev]'

echo "==> self-test (simulator, no hardware needed)"
python -m hwci selftest

cat <<'EOF'

Done. Remaining manual steps (see hwci/README.md):
  1. Log out/in so dialout/plugdev group membership takes effect.
  2. Install the Tyto Flight Stand Software and generate its gRPC Python stubs.
  3. cp config/rig.example.yaml rig.yaml  and edit for your wiring.
  4. Edit /etc/udev/rules.d/99-hwci.rules with your USB-serial serial numbers
     so /dev/esc-telem and /dev/esc-throttle appear, then re-trigger udev.
  5. Capture a baseline:  hwci ci --profile efficiency_sweep --config rig.yaml \
        --out runs/baseline && hwci baseline-save runs/baseline \
        --out baselines/ARK_4IN1_F051.json
EOF
