@echo off
rem launch the AM32 SITL GUI using the environment created by
rem "py Mcu\SITL\make_gui_env.py". Lives in the repo root so the venv
rem paths resolve relative to it
cd /d "%~dp0"
if not exist "Mcu\SITL\venv\Scripts\pythonw.exe" (
    echo GUI environment not found, run:  py Mcu\SITL\make_gui_env.py
    pause
    exit /b 1
)
start "" "Mcu\SITL\venv\Scripts\pythonw.exe" "Mcu\SITL\sitl_gui.py" %*
