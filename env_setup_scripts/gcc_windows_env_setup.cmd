REM Change to the parent directory of the directory where this script is located
cd "%~dp0.."

REM Download location for tools
set WINDOWS_TOOLS=https://firmware.ardupilot.org/Tools/AM32-tools/windows-tools.zip

REM Set the destination directory for the download
set DOWNLOAD_DIR=%cd%\downloads

REM Create the downloads directory if it doesn't exist
if not exist "%DOWNLOAD_DIR%" (
    mkdir "%DOWNLOAD_DIR%"
)

echo Downloading Windows tools to %DOWNLOAD_DIR%...
powershell -Command "Invoke-WebRequest -Uri '%WINDOWS_TOOLS%' -OutFile '%DOWNLOAD_DIR%\windows-tools.zip'"
if %errorlevel% neq 0 (
    echo Failed to download windows-tools.zip
    exit /b 1
)

REM Set the destination directory for the unzipped files
set UNZIP_DIR=%cd%

REM Create the windows directory if it doesn't exist
if not exist "%UNZIP_DIR%" (
    mkdir "%UNZIP_DIR%"
)

echo Unpacking windows-tools.zip to %UNZIP_DIR%...
powershell -Command "Expand-Archive -Path '%DOWNLOAD_DIR%\windows-tools.zip' -Force -DestinationPath '%UNZIP_DIR%'"
if %errorlevel% neq 0 (
    echo Failed to unpack windows-tools.zip
    exit /b 1
)

REM Copy .vscode/settings.json.windows to .vscode/settings.json
if exist .vscode\settings.json.windows (
    copy /Y .vscode\settings.json.windows .vscode\settings.json
    if %errorlevel% neq 0 (
        echo Failed to copy .vscode\settings.json.windows to .vscode\settings.json
        exit /b 1
    )
    echo Copied .vscode\settings.json.windows to .vscode\settings.json
) else (
    echo Source file .vscode\settings.json.windows not found.
    exit /b 1
)

echo Script completed successfully.