#!/usr/bin/env python3
'''
build the AM32 SITL control GUI into a single executable

Produces dist/am32-sitl-gui (or .exe on Windows) with PyInstaller, and
bundles the SITL firmware binary so the GUI can run the simulator out
of the box. Run it on the platform you want a binary for - PyInstaller
does not cross compile.

  python3 Mcu/SITL/build_sitl_gui.py [--onedir] [--sitl PATH]
'''

import argparse
import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))       # Mcu/SITL
ROOT = os.path.dirname(os.path.dirname(HERE))

sys.path.insert(0, HERE)
import sitl_params

# pulled in dynamically or lazily, so PyInstaller cannot see them
HIDDEN = [
    'dronecan', 'dronecan.app.node_monitor', 'dronecan.app.dynamic_node_id',
    'dronecan.driver.mavcan', 'pymavlink', 'pymavlink.mavutil',
    'pymavlink.dialects.v20.ardupilotmega',
    'pyqtgraph', 'numpy', 'serial', 'serial.tools.list_ports',
    'sitl_dshot', 'sitl_tones', 'sitl_params', 'sitl_gui_backend',
    'sitl_param_dialog', 'sitl_wave_dialog', 'sim_runner', 'model_builder',
]

EXCLUDES = [
    'PyQt5', 'PyQt6', 'PySide2', 'matplotlib', 'tkinter',
    'PySide6.QtWebEngineCore', 'PySide6.QtQuick', 'PySide6.Qt3DCore',
    'PySide6.QtCharts', 'PySide6.QtDataVisualization', 'PySide6.QtBluetooth',
    'PySide6.QtSql', 'PySide6.QtTest', 'PySide6.QtDesigner', 'cv2', 'MAVProxy',
    'scipy', 'pandas', 'PIL',
]


def _find_sitl(given):
    import glob
    if given:
        return given if os.path.isfile(given) else None
    win = os.path.join(ROOT, 'sitl-windows', 'AM32_SITL_CAN.exe')
    if sys.platform.startswith('win') and os.path.isfile(win):
        return win
    hits = sorted(glob.glob(os.path.join(ROOT, 'obj',
                                         'AM32_AM32_SITL_CAN_*.elf')))
    return hits[0] if hits else None


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--onedir', action='store_true')
    ap.add_argument('--name', default='am32-sitl-gui')
    ap.add_argument('--sitl', help='SITL binary to bundle (else auto-detect)')
    args = ap.parse_args()

    try:
        import PyInstaller  # noqa: F401
    except ImportError:
        raise SystemExit('PyInstaller is not installed: pip install pyinstaller')

    sep = ';' if sys.platform.startswith('win') else ':'
    cmd = [sys.executable, '-m', 'PyInstaller', '--noconfirm', '--clean',
           '--name', args.name, '--windowed', '--paths', HERE,
           '--distpath', os.path.join(ROOT, 'dist'),
           '--workpath', os.path.join(ROOT, 'build', 'pyinstaller-sitl'),
           '--specpath', os.path.join(ROOT, 'build')]
    cmd.append('--onedir' if args.onedir else '--onefile')
    for h in HIDDEN:
        cmd += ['--hidden-import', h]
    for e in EXCLUDES:
        cmd += ['--exclude-module', e]

    # the models directory, so the model list is populated
    models = os.path.join(HERE, 'models')
    if os.path.isdir(models):
        cmd += ['--add-data', '%s%s%s' % (models, sep, 'models')]

    # the SITL binary and a default eeprom, under sitl/, where
    # sim_runner.bundled_* find them. The Windows SITL is a native
    # MinGW-w64 exe, so no runtime DLL needs bundling
    sitl_bin = _find_sitl(args.sitl)
    if sitl_bin:
        stage = os.path.join(ROOT, 'build', 'sitl_gui_stage')
        shutil.rmtree(stage, ignore_errors=True)
        os.makedirs(stage)
        name = 'AM32_SITL_CAN.exe' if sys.platform.startswith('win') \
            else 'AM32_SITL_CAN'
        staged = os.path.join(stage, name)
        shutil.copy(sitl_bin, staged)
        cmd += ['--add-binary', '%s%s%s' % (staged, sep, 'sitl')]
        # the default eeprom is generated from the parameter file, not
        # copied: no eeprom binary is committed
        params = os.path.join(HERE, 'data/VIMDRONES_NANO_2216/sitl.param')
        if os.path.isfile(params):
            ee = sitl_params.write_eeprom(
                params, os.path.join(stage, 'default_eeprom.bin'))
            cmd += ['--add-data', '%s%s%s' % (ee, sep, 'sitl')]
        print('bundling simulator: %s' % sitl_bin)
    else:
        print('no SITL binary found - the SITL panel will need one chosen '
              'with Browse (pass --sitl to bundle it)')

    try:
        import dronecan
        dsdl = os.path.join(os.path.dirname(dronecan.__file__), 'dsdl_specs')
        if os.path.isdir(dsdl):
            cmd += ['--add-data', '%s%s%s' % (dsdl, sep, 'dronecan/dsdl_specs')]
    except ImportError:
        pass

    cmd.append(os.path.join(HERE, 'sitl_gui.py'))
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
