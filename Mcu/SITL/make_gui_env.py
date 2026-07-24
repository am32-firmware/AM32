#!/usr/bin/env python3
'''
create a self-contained python environment for the SITL GUI in
Mcu/SITL/venv and install the GUI dependencies (PySide6, pyqtgraph,
dronecan) into it. Works the same on Linux, Windows and macOS.

usage: python3 make_gui_env.py
then run the GUI with the interpreter this prints.
'''

import os
import subprocess
import sys
import venv

here = os.path.dirname(os.path.abspath(__file__))
env_dir = os.path.join(here, 'venv')
requirements = os.path.join(here, 'requirements-gui.txt')

if sys.platform == 'win32':
    python = os.path.join(env_dir, 'Scripts', 'python.exe')
else:
    python = os.path.join(env_dir, 'bin', 'python3')

if not os.path.exists(python):
    print('creating %s ...' % env_dir)
    venv.EnvBuilder(with_pip=True).create(env_dir)

print('installing GUI dependencies ...')
subprocess.check_call([python, '-m', 'pip', 'install', '--upgrade',
                       '-r', requirements])

print('\nGUI environment ready. Run the GUI with:')
print('  %s %s' % (python, os.path.join(here, 'sitl_gui.py')))
