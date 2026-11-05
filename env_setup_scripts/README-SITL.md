# Building SITL in VS Code

If you already build hardware boards with **Makefile Tools**, use the same
target selector: type **SITL**, select **AM32_SITL_CAN**, and use **Makefile:
Build the current target**. No workspace switch is required. On Windows,
run the Cygwin setup below once; the native Windows makefile delegates the
SITL target to that compiler. Hardware targets still use the ARM SDK.

If the selector only offers `all`, run **Makefile: Clean configure**. In
`.vscode/settings.json`, these settings make Windows target discovery work
and restrict the list to board names and build actions:

```json
"makefile.makePath": "${workspaceFolder}/tools/windows/make/bin/make.exe",
"makefile.configureOnOpen": true,
"makefile.phonyOnlyTargets": true
```

SITL setup updates these settings when the normal Windows board tools are
installed, preserving other settings and saving a `.sitl-backup` copy first.
For settings files containing comments, apply the changes in VS Code's
settings editor. Linux uses `/usr/bin/make`; its board selector also includes
SITL. Direct Linux Makefile Tools builds produce the usual versioned
`obj/AM32_AM32_SITL_CAN_<version>.elf`.

For a checkout without the hardware build tools or Makefile Tools extension,
open **AM32-SITL.code-workspace** with **File > Open Workspace from File**.
After installing the tools below, **Ctrl+Shift+B** builds `AM32_SITL_CAN`.
Compiler errors appear in the Problems panel and link to the source lines.
**Terminal > Run Task > AM32 SITL: Check tools** reports the selected tools.

The dedicated workspace uses [VS Code tasks](https://code.visualstudio.com/docs/debugtest/tasks),
so no Makefile Tools extension or changes to the hardware build settings
are required. Install VS Code separately if needed. The ARM setup scripts
and STLink/JLink launch configurations remain available for hardware work.

## Windows 11 (64 bit)

1. Clone the AM32 branch you want to test, or download and extract its source
   ZIP. Use a local writable directory, for example `C:\Users\Alka\AM32`.
2. In PowerShell or Command Prompt, from the checkout root, run:

   ```powershell
   .\env_setup_scripts\sitl_setup_windows.cmd
   ```

3. In your normal board target selector, select **AM32_SITL_CAN** and build.
   Alternatively, open `AM32-SITL.code-workspace` and press **Ctrl+Shift+B**.

SITL uses **Cygwin x86_64 GCC (`gcc-core`), GNU Make and GDB**, along with
Cygwin's base shell utilities and C library headers. The script reuses an
existing Cygwin installation, or installs one for your user in
`%LOCALAPPDATA%\AM32-Tools\cygwin64`. It downloads the
[official Cygwin installer](https://cygwin.com/install.html), which verifies
the signed package index and package checksums. No global PATH changes or
administrator rights are needed for a new per-user installation. Adding
packages to an administrator-owned installation may require running the
setup command as administrator or choosing a new per-user directory.

For a custom installation location:

```powershell
.\env_setup_scripts\sitl_setup_windows.cmd -CygwinRoot C:\Tools\cygwin64
```

The selected root is remembered in the ignored `build/sitl-cygwin-root.txt`.
`AM32_CYGWIN_ROOT` can override it. A different official package mirror can
be selected with `-Mirror https://mirror.example/cygwin/`.

The ARM GCC in `tools/windows`, MSVC, Git Bash and WSL do not produce the
Cygwin executable used by this workflow. Native Windows Python, the ARM
SDK, USBIP and the ESCSim source checkout are not needed to build firmware.
The GUI ZIP already contains its own Python/Qt runtime. Cygwin provides
the POSIX APIs used by SITL; this is the same compiler family used by the
native Windows CI build.

Git is only needed to clone and manage the source checkout; an existing
Git for Windows installation is suitable. Building an extracted source
ZIP does not need Git.

The resulting files are:

```text
build\sitl\AM32_SITL_CAN.exe
build\sitl\cygwin1.dll
```

Keep the DLL beside the executable when copying it to another directory or
machine. The versioned `.elf` in the same directory is also a Windows host
executable, not hardware firmware. Stop any simulator using this output
before rebuilding, because Windows locks running executables.

The same build can be run without VS Code:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\env_setup_scripts\sitl_windows.ps1
```

## Linux

On Debian/Ubuntu, install the host tools with:

```sh
bash env_setup_scripts/sitl_setup_linux.sh
```

On other Linux distributions, install GCC, GNU Make, Git and the standard
C development headers using the distribution's package manager. Open the
same workspace and press **Ctrl+Shift+B**, or run:

```sh
bash env_setup_scripts/sitl_build.sh
```

The stable output is `build/sitl/AM32_SITL_CAN`. It is a Linux executable;
build on Windows to get the Windows GUI's replacement firmware.

## Run and debug with the ESCSim GUI

Install Microsoft's **C/C++** VS Code extension (`ms-vscode.cpptools`). Run
the SITL setup script again if you installed the build tools before GDB
support was added. Windows setup records the selected Cygwin GDB in
`.vscode/settings.json`, for example:

```json
"am32.sitl.gdbPath": "C:/cygwin64/bin/gdb.exe"
```

On Windows, use a checkout path without spaces for debugging, such as
`C:\Users\Alka\AM32`. The C/C++ debugger's Cygwin path conversion can
truncate paths at spaces; command-line and build-task builds still support
them. Use Cygwin GDB, not the ARM toolchain's GDB.

1. Stop any simulator launched by the GUI so its UDP ports are free. Use
   one ESC and leave the benchmark selection at **None**.
2. In VS Code's **Run and Debug** view, select **AM32 SITL (ESCSim GUI)**.
   Press **Ctrl+F5** to build and run without debugging. For breakpoints,
   use **F5**. Both use the native host executable, not the ARM debugger.
   The launch waits for PWM/DShot input before booting, so opening the GUI
   afterwards does not cause a no-input reset under the debugger.
3. Open the ESCSim SITL GUI on the same machine with its default ports.
   Leave **Start simulator** alone: VS Code owns this process. In the
   PWM/DShot panel, enable DShot with zero throttle, allow the ESC to arm,
   then raise the throttle. The motor views, telemetry and virtual scope
   can use the simulator's UDP state stream.
4. Set breakpoints in firmware source, inspect variables, step, and resume
   in VS Code. Stop the debug session before rebuilding or starting a
   simulator from the GUI. For **Ctrl+F5**, stop the process in its VS Code
   terminal with **Ctrl+C**, or close that terminal with its trash button.

The launch uses UDP input port **57733**, state port **57734**, and CAN
`mcast:0`. It forces DShot input and stores its EEPROM separately in
`build/sitl/vscode_eeprom.bin`. GUI launcher fields (binary, bootloader,
EEPROM and input mode) do not configure this externally started process;
change its arguments in `.vscode/launch.json` instead. No bootloader is
used by this launch configuration.

Diagnostics are appended to `build/sitl/vscode-sitl.log` (`--log-file`),
including startup, periodic motor state and reset reasons. Open this file
in VS Code to inspect the run. Cygwin GDB can leave its child's diagnostic
pipe unread; writing there would eventually fill the pipe and stop physics
and telemetry. Ordinary GUI launches still use stderr as before.

On POSIX hosts the launch passes `SIGUSR1` through GDB for interrupt
delivery. Windows, including Cygwin, uses OS thread suspension to avoid
Cygwin GDB's spurious signal stops. Breakpoints pause the firmware and physics
threads. The GUI can remain responsive but receives no new physics samples
until you resume. Builds
retain `-O2`, so some variables may be optimized out and stepping may skip
or revisit source lines. For scope measurements, let the simulation run
uninterrupted.

The launch enables `--wait-for-input` and `--exit-on-reset`. Firmware resets
(including input loss and watchdog resets) end this run with a diagnostic;
press **F5** to boot again. This preserves the firmware's reset behaviour
without creating a replacement process that Cygwin GDB cannot follow. These
options are off by default for ordinary GUI-launched simulations and benchmarks,
which continue to reboot automatically. The startup wait is for PWM/DShot;
remove it when creating a launch configuration for DroneCAN input.

The current **Start benchmark** action launches its own simulator and
cannot use a VS Code-owned process. Use manual controls for this debugging
workflow. The GUI's process log also only displays output from processes
it launches; use `build/sitl/vscode-sitl.log` for this process.

## Test an edited firmware in the GUI

1. Stop the simulator (all ESCs if several are running).
2. Build the selected SITL target (or use **Ctrl+Shift+B** in the standalone
   workspace) and wait for the build to complete with no errors.
3. In the GUI, use **Browse** next to **SITL binary** and select the output
   above. Selecting it applies to all ESC tabs. Leave **Bootloader** blank
   for the demag benchmarks; select a compatible host bootloader when USB
   configurator access is needed.
4. Choose the benchmark and click **Start benchmark**. To test another edit,
   stop the simulator, rebuild, and start the benchmark again. The output
   path for Windows and standalone workspace builds stays the same across
   firmware version changes.

Windows and standalone workspace builds go into `build/sitl`, separate from
hardware firmware in `obj`. The task performs a full rebuild so edits to
headers and compiler settings are included. It leaves the packaged GUI and your EEPROM files untouched.
See the existing `make AM32_SITL_CAN` workflow for sanitizer,
coverage and cross-compiler options.
