# AM32 Development Guide

This document explains **how the AM32 firmware build system works**, how to use it effectively, and how to extend it safely.

The project supports **ARM (STM32 / GD32 / AT32)** and **WCH RISC-V (CH32)** microcontrollers using a unified, reproducible CMake-based build system.

## TL;DR; Quick Start

- Install Git, Cmake, Ninja, Python3

```bash
git clone https://github.com/am32-firmware/AM32
cd AM32
cmake --preset arm
cmake --build --preset build-arm
```

## 1. How the Build System Works

AM32 uses a **family → board** generation model.

### Core idea

* **MCU Family** (e.g. `F415`, `G431`, `V203`) defines:

  * CPU architecture and compiler flags
  * linker scripts
  * startup code and drivers
  * OpenOCD + SVD configuration

* **Boards** are declared in **`Inc/targets.h`**

  * Each board name becomes **one firmware target**
  * CMake does **not** contain a manual list of boards

* During configuration:

  1. CMake scans `targets.h`
  2. Boards are grouped by MCU suffix (`_F415`, `_G431`, `_V203`, …)
  3. One executable is generated **per board**

There is no duplication of rules and no per-board CMake logic.

## 2. Naming Conventions

The build system relies on strict naming rules.

### Board names

* Must be defined in `Inc/targets.h`
* Must contain an MCU suffix:

  * `_F031`, `_F051`, `_F415`, `_G431`, `_V203`, etc.
* Example:

  ```
  AM32REF_F051
  ```

### MCU family files

| MCU  | File                                        |
| ---- | ------------------------------------------- |
| F415 | `cmake/targets/arm/target_f415.cmake`       |
| G431 | `cmake/targets/arm/target_g431.cmake`       |
| V203 | `cmake/targets/wch_riscv/target_v203.cmake` |

The `FAMILY` argument **must match the suffix** used in `targets.h`.

### CAN targets

* Board name must end with `_CAN`
* Automatically:

  * switches linker script
  * adds CAN sources
* If `_CAN` is used but no CAN linker script exists → **configure error**

## 3. Prerequisites

The build system **automatically downloads**:

* ARM GCC
* WCH RISC-V GCC
* OpenOCD for ARM and WCH version.

You only need base build tools.

---

### Windows

#### Option 1: PowerShell (recommended)

Open **PowerShell as Administrator**:

```powershell
winget install Git.Git
winget install Kitware.CMake
winget install Ninja-build.Ninja
winget install Python.Python.3
```

Restart your terminal or VS Code after installation.

#### Option 2: Manual installation

1. Git: [https://git-scm.com/download/win](https://git-scm.com/download/win)
2. CMake: [https://cmake.org/download/](https://cmake.org/download/) (enable *Add to PATH*)
3. Ninja: [https://github.com/ninja-build/ninja/releases](https://github.com/ninja-build/ninja/releases)
4. Python 3: [https://www.python.org/downloads/](https://www.python.org/downloads/) (enable *Add to PATH*)

---

### macOS

Using Homebrew:

```bash
brew install cmake ninja python3 git
```

---

### Linux (Debian / Ubuntu)

```bash
sudo apt update
sudo apt install cmake ninja-build python3 git build-essential
```

---

## 4. Configure vs Build

### Configure step

For ARM Targets
```bash
cmake --preset arm
```

For WCH RISC-V Targets
```bash
cmake --preset wch-riscv
```

This step:

* downloads toolchains (first run only)
* scans `targets.h`
* generates **all firmware targets**
* generates `.vscode/launch.json`
* creates `compile_commands.json` for IntelliSense

### Build all targets

ARM targets
```bash
cmake --build --preset build-arm
```

WCH RISC-V targets
```bash
cmake --build --preset build-wch
```

This step:

* compiles firmware
* produces `.elf`, `.bin`, `.hex`
* prints memory usage

**Configure is required only once per change in structure.**

---

## 5. VS Code Setup

1. Install VS Code
2. Open the AM32 repository root
3. Install recommended extensions:

   * C/C++
   * CMake Tools
   * Cortex-Debug

---

## 6. Building in VS Code

### 1. Select Configure Preset

Bottom status bar → **CMake: No Kit Selected**

* `ARM` → STM32 / GD32 / AT32
* `WCH RISC-V` → CH32V203

### 2. Configure

VS Code usually auto-configures.
If not:

```
CMake: Configure
```

> First configure may take several minutes due to toolchain downloads.

### 3. Build

* Click **Build** in status bar
* Or press **F7**

This builds **all boards for the selected architecture**.

---

## 7. Debugging in VS Code

1. Open **Run and Debug** (Ctrl+Shift+D)
2. Select a configuration (e.g. *Debug STM32 F4*)
3. Press **F5**

This:

* flashes firmware via OpenOCD
* attaches GDB
* loads SVD for register view

Ensure:

* ST-Link (ARM) or WCH-Link (RISC-V) is connected
* no other OpenOCD instance is running

---

## 8. Command Line Usage

### Configure

```bash
cmake --preset arm
cmake --preset wch-riscv
```

### Build

```bash
cmake --build --preset build-arm
cmake --build --preset build-arm --target AM32REF_F051
cmake --build --preset build-arm --verbose
```

---

## 9. Flashing from CLI

Flashing uses **build directories**, not presets.

```bash
cmake --build --preset build-arm --target flash_AM32REF_F051
```

---

## 10. Firmware Versioning & Reproducibility

* Version comes from `Inc/version.h`
* Git hash is embedded in firmware
* **Clean build**

  * commit timestamp used
  * identical commit → identical binary
* **Dirty build**

  * `-dirty` suffix
  * system time used

This guarantees traceability and prevents confusion with unofficial builds.

---

## 11. Project Structure

```
CMakeLists.txt              Main entry point
CMakePresets.json           ARM / WCH environments
cmake/
 ├─ manage_tools.cmake      Toolchain download logic
 ├─ modules/
 │   ├─ TargetFactory.cmake Core target generation
 │   ├─ TargetUtils.cmake  Helpers (sources, parsing)
 │   ├─ VersionUtils.cmake Version & git handling
 │   └─ VscodeGenerator.cmake Debug config generation
 └─ targets/
     ├─ arm/
     └─ wch_riscv/
Inc/targets.h               Master board list
Mcu/                         MCU-specific code
```

---

## 12. How to Add a New Board

1. Edit `Inc/targets.h`
2. Add a board definition containing the MCU suffix:

```c
#ifdef MY_NEW_BOARD_F415
#define FILE_NAME "MY_NEW_BOARD_F415"
#endif
```

3. Reconfigure:

   * VS Code: *Delete Cache and Reconfigure*
   * CLI: `cmake --preset arm`

CMake will automatically generate the new target.

---

## 13. How to Add a New MCU Family

1. Create `Mcu/f722/`

2. Add:

   * startup code
   * drivers
   * `ldscript.ld`
   * optional `ldscript_CAN.ld`
   * SVD file

3. Create `cmake/targets/arm/target_f722.cmake`

4. Reconfigure — no registration required.

---

## 14. How to Add a New Architecture

1. Create toolchain file:

   ```
   cmake/toolchains/xtensa-esp32.cmake
   ```

2. Extend toolchain selection logic in `CMakeLists.txt`

3. Add preset to `CMakePresets.json`

4. Create `cmake/targets/<arch>/target_*.cmake`
