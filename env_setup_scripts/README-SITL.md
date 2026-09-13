# Building SITL in VS Code

Open **AM32-SITL.code-workspace** from the root of the AM32 checkout with
VS Code's **File > Open Workspace from File**. After installing the tools
below, **Ctrl+Shift+B** builds `AM32_SITL_CAN` from that checkout. Compiler
errors appear in VS Code's Problems panel and link to the source lines.
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

3. Open `AM32-SITL.code-workspace` and press **Ctrl+Shift+B**.

SITL uses **Cygwin x86_64 GCC (`gcc-core`) and GNU Make**, along with
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

## Test an edited firmware in the GUI

1. Stop the simulator (all ESCs if several are running).
2. Build with **Ctrl+Shift+B** and wait for `Built:` with no errors.
3. In the GUI, use **Browse** next to **SITL binary** and select the output
   above. Selecting it applies to all ESC tabs. Leave **Bootloader** blank
   for the demag benchmarks; select a compatible host bootloader when USB
   configurator access is needed.
4. Choose the benchmark and click **Start benchmark**. To test another edit,
   stop the simulator, rebuild, and start the benchmark again. The output
   path stays the same across firmware version changes.

Builds go into `build/sitl`, separate from hardware firmware in `obj`.
The task performs a full rebuild so edits to headers and compiler settings
are included. It leaves the packaged GUI and your EEPROM files untouched.
This task is for building the firmware; a VS Code debugger launch is not
configured. See the existing `make AM32_SITL_CAN` workflow for sanitizer,
coverage and cross-compiler options.
