#!/usr/bin/env python3
'''
build the calibration capture GUI into a single executable

Produces dist/am32-capture (or am32-capture.exe on Windows) with
PyInstaller. Run it on the platform you want a binary for - PyInstaller
does not cross compile - which is why CI builds it on both.

  python3 scripts/build_capture_gui.py [--onedir]

matplotlib is deliberately excluded: the GUI plots with pyqtgraph and
the chirp fit uses esc_chirp.fit_numbers(), which is numpy only. That
keeps the bundle to a sane size.
'''

import argparse
import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

# pulled in only by code paths the GUI does not use
EXCLUDES = [
    # PyInstaller refuses to bundle two Qt bindings, and a system
    # pyqtgraph install often drags PyQt in alongside our PySide6
    'PyQt5', 'PyQt6', 'PySide2',
    'matplotlib', 'tkinter', 'PySide6.QtWebEngineCore', 'PySide6.QtQuick',
    'PySide6.Qt3DCore', 'PySide6.QtMultimedia', 'PySide6.QtCharts',
    'PySide6.QtDataVisualization', 'PySide6.QtBluetooth', 'PySide6.QtSql',
    'PySide6.QtTest', 'PySide6.QtDesigner', 'cv2', 'MAVProxy',
    'scipy', 'pandas', 'PIL',
]

# imported dynamically, so PyInstaller cannot see them
HIDDEN = [
    'dronecan', 'serial', 'serial.tools.list_ports', 'pyqtgraph',
    # mavcan: URIs (an ArduPilot board used as the CAN adapter)
    'dronecan.driver.mavcan', 'pymavlink', 'pymavlink.mavutil',
    'pymavlink.dialects.v20.ardupilotmega',
    'esc_measure', 'esc_chirp', 'esc_square', 'esc_analyze', 'esc_settings',
    'esc_capture_fc', 'esc_eeprom_fc', 'msp', 'sim_clock', 'am32_debug',
    'capture_session',
]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--onedir', action='store_true',
                    help='a directory instead of a single file (starts faster)')
    ap.add_argument('--name', default='am32-capture')
    args = ap.parse_args()

    try:
        import PyInstaller  # noqa: F401
    except ImportError:
        raise SystemExit('PyInstaller is not installed: pip install pyinstaller')

    cmd = [sys.executable, '-m', 'PyInstaller', '--noconfirm', '--clean',
           '--name', args.name, '--windowed',
           '--paths', HERE,
           '--distpath', os.path.join(ROOT, 'dist'),
           '--workpath', os.path.join(ROOT, 'build', 'pyinstaller'),
           '--specpath', os.path.join(ROOT, 'build')]
    cmd.append('--onedir' if args.onedir else '--onefile')
    for h in HIDDEN:
        cmd += ['--hidden-import', h]
    for e in EXCLUDES:
        cmd += ['--exclude-module', e]
    # dronecan ships its DSDL definitions as data files
    try:
        import dronecan
        dsdl = os.path.join(os.path.dirname(dronecan.__file__), 'dsdl_specs')
        if os.path.isdir(dsdl):
            sep = ';' if sys.platform.startswith('win') else ':'
            cmd += ['--add-data', '%s%s%s' % (dsdl, sep, 'dronecan/dsdl_specs')]
    except ImportError:
        pass
    cmd.append(os.path.join(HERE, 'esc_capture_gui.py'))

    print(' '.join(cmd))
    subprocess.run(cmd, check=True, cwd=ROOT)

    out = os.path.join(ROOT, 'dist', args.name)
    if sys.platform.startswith('win'):
        out += '.exe'
    if os.path.exists(out):
        print('built %s (%.1f MB)' % (out, os.path.getsize(out) / 1e6))
    else:
        print('build finished but %s is missing' % out)


if __name__ == '__main__':
    main()
