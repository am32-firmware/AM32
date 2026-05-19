# AM32 Development Guide

## 1. Prerequisites

You need the base build tools installed on your system.

* **ARM Toolchain:** Downloaded automatically by CMake.
* **WCH RISC-V Toolchain:**
  * **Linux/macOS:** Downloaded automatically by CMake.
  * **Windows:** Requires manual installation of **MounRiver Studio**.


### Windows

#### Step 1: Install Base Tools (via Winget)

Open **PowerShell as Administrator** and run:

```powershell
winget install Git.Git
winget install Kitware.CMake
winget install Ninja-build.Ninja
winget install Python.Python.3.12

```

#### Step 2: Install WCH Toolchain (MounRiver Studio)

The build system on Windows relies on the official MounRiver Studio toolchain.

1. Download and install **MounRiver_Studio_Setup_V230.zip** from [http://www.mounriver.com/download](http://www.mounriver.com/download).
2. **Run the IDE once** to ensure it downloads toolchains.
3. **Set the Environment Variable:**
Tell CMake where the toolchain is located. Update the path below to match your installation (point to the `RISC-V Embedded GCC` folder).
**In PowerShell (User Scope):**
```powershell
[System.Environment]::SetEnvironmentVariable("AM32_WCH_TOOLCHAIN_PATH", "C:\MounRiver\MounRiver_Studio2\resources\app\resources\win32\components\WCH", "User")
```
Or set the actual path in case of non standard installation destination.

*Restart your terminal or VS Code after running this command.*

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

## 2. Quick Start

1. **Clone the repository:**
```sh
git clone https://github.com/am32-firmware/AM32
cd AM32

```

2. **Open in VS Code:**
```sh
code .
```

3. **Install Recommended Extensions:**
VS Code will prompt you to install extensions (C/C++, CMake Tools, Cortex-Debug). Click **Install All**.
4. **Wait for Configuration:**
* Select your preset in the status bar (e.g., `ARM` or `WCH RISC-V`).
* CMake will configure the project.

* *Note:* If choosing `ARM`, tools will download automatically. If choosing `WCH`, it will verify your MounRiver path on Windows, but will download for MacOs/Linux.

---

## 3. How the Build System Works

AM32 uses a **family → board** generation model.

### Core idea

* **MCU Family** (e.g. `F415`, `G431`, `V203`) defines:
* CPU architecture and flags
* Linker scripts
* Startup code and drivers
* OpenOCD + SVD configuration

* **Boards** are declared in **`Inc/targets.h`**:
* Each board name becomes **one firmware target**.
* CMake does **not** contain a manual list of boards.

* **Generation Process:**
1. CMake scans `targets.h`.
2. Boards are grouped by MCU suffix (`_F415`, `_G431`, `_V203`, …).
3. One executable is generated **per board**.


### Managing Target Visibility (Skip Lists)

The main `CMakeLists.txt` contains two lists that control which boards are built. This helps keep the release clean while allowing development on experimental hardware.

1. **`TARGET_SKIP_LIST` (Broken / Deprecated)**
* Targets in this list are **never built**, neither locally nor in CI.
* Use this for boards that are known to be broken, deprecated, or require hardware fixes.

2. **`TARGET_RELEASE_SKIP_LIST` (Private / Test)**
* Targets in this list are built during **CI / Local Development** but are **excluded from Official Releases**.
* Use this for:
  * Private hardware prototypes.
  * Test boards not ready for the public.
* *Control Variable:* `AM32_OFFICIAL_RELEASE` (passed by GitHub Actions during release builds).

To add a target to either list, simply edit the `set(...)` block in the root `CMakeLists.txt`.

### Naming Conventions

* **Board Names:** Must be defined in `Inc/targets.h` and contain a valid MCU suffix.
* *Example:* `AM32REF_F051`

* **CAN Targets:** Must end with `_CAN`.
* Automatically switches linker scripts and includes CAN sources.

---

## 4. Building & Flashing

### Command Line Usage

**Configure:**

```bash
cmake --preset arm          # Configure for ARM (STM32/AT32/GD32)
cmake --preset wch-riscv    # Configure for WCH (CH32V203)

```

**Build:**

```bash
cmake --build --preset build-arm   # Build all ARM targets
cmake --build --preset build-wch   # Build all WCH targets

```

**Flash Specific Target:**

```bash
cmake --build --preset build-arm --target flash_AM32REF_F051

```

### VS Code Usage

1. **Select Preset:** Click **CMake: [No Kit Selected]** in the status bar and choose `ARM` or `WCH RISC-V`.
2. **Build:** Press **F7** (Build All).
3. **Debug:** Go to **Run and Debug (Ctrl+Shift+D)**, select a target (e.g., *Debug STM32 F031*), and press **F5**.

---

## 5. Firmware Versioning

* **Version Number:** Defined in `Inc/version.h`.
* **Git Hash:** Embedded automatically.
* **Clean Build:** Uses git commit timestamp. Identical commits produce identical binaries.
* **Dirty Build:** Suffixes `-dirty` and uses system time.

---

## 6. How to Extend the Project

### A. How to Add a New Board

1. Edit `Inc/targets.h`.
2. Add a board definition containing the MCU suffix:
```c
#ifdef MY_NEW_BOARD_F415
#define FILE_NAME "MY_NEW_BOARD_F415"
#endif
```

3. **Reconfigure CMake** (VS Code: *Developer: Reload Window* or *Delete Cache and Reconfigure*).
4. The new target `AM32_MY_NEW_BOARD_F415_...` will appear automatically.

### B. How to Add a New MCU Family

1. Create `Mcu/f722/` folder.
2. Add necessary files:
* Startup code (`.s`)
* Drivers / HAL
* `ldscript.ld` (and optional `ldscript_CAN.ld`)
* SVD file for debugging

3. Create `cmake/targets/arm/target_f722.cmake` (you can use existing targets as an example)
4. Reconfigure. The build system will auto-register the new family logic.

### C. How to Add a New Architecture

1. Create a toolchain file: e.g.: `cmake/toolchains/xtensa-esp32.cmake`.
2. Extend toolchain selection logic in `CMakeLists.txt`.
3. Add a new preset to `CMakePresets.json`.
4. Create target logic in `cmake/targets/<arch>/target_*.cmake`.