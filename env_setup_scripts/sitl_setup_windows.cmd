@echo off
setlocal
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0sitl_windows.ps1" -Action Setup %*
exit /b %errorlevel%
