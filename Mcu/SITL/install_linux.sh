#!/bin/bash
# install the Debian/Ubuntu packages needed to build and run the AM32 SITL
# and its Python tools / control GUI.
#
#   ./Mcu/SITL/install_linux.sh
#
# The distro PySide6 is split into per-module packages; the GUI's live
# scopes need QtOpenGLWidgets, which is easy to miss. This installs the
# full set. For a self-contained setup instead, use the bundled venv:
#   python3 Mcu/SITL/make_gui_env.py

set -e

sudo apt-get update
sudo apt-get install -y \
    build-essential gcc-mingw-w64-x86-64 \
    python3 python3-pip python3-venv \
    python3-numpy python3-serial python3-pyqtgraph \
    python3-pyside6.qtcore python3-pyside6.qtgui python3-pyside6.qtwidgets \
    python3-pyside6.qtopengl python3-pyside6.qtopenglwidgets \
    python3-pyside6.qtsvg python3-pyside6.qtmultimedia \
    libgl1 libegl1 libxkbcommon0 libfontconfig1 libdbus-1-3 libxcb-cursor0 \
    libvdpau-va-gl1

# dronecan and pymavlink are not packaged for apt
echo
echo "== installing dronecan and pymavlink with pip (not in apt) =="
python3 -m pip install --user dronecan pymavlink || {
    echo
    echo "pip refused (externally-managed python). Either:"
    echo "  python3 -m pip install --user --break-system-packages dronecan pymavlink"
    echo "  or use an isolated venv:  python3 Mcu/SITL/make_gui_env.py"
    exit 1
}

echo
echo "done. run the GUI with:  python3 Mcu/SITL/sitl_gui.py"
