# AM32 ESC Firmware Code Analysis & Refactoring Plan

## Repository Structure Overview

The AM32 firmware is a multi-platform ESC (Electronic Speed Controller) firmware for brushless motors, supporting various STM32 microcontrollers.

### Key Directories:
- **Src/**: Main source files
  - `main.c` (2235 lines) - Main application logic
  - `dshot.c` - DSHOT protocol implementation
  - `functions.c` - Utility functions
  - `signal.c` - Signal processing
  - `sounds.c` - Sound/beep generation
  - `kiss_telemetry.c` - KISS telemetry protocol
  - `firmwareversion.c` - Version information
  - `DroneCAN/` - DroneCAN protocol support

- **Inc/**: Header files
  - `common.h` - Common definitions and data structures
  - `targets.h` (4703 lines) - Target-specific configurations
  - Various protocol headers

- **Mcu/**: MCU-specific implementations for different targets
- **Keil_Projects/**: Keil IDE project files
- **make/**: Build system

### Supported Targets:
- STM32F031, STM32F051, STM32F415, STM32F421
- STM32G031, STM32G071, STM32G431, STM32L431
- GD32E230, AT32F415, AT32F421, and others

## Program Flow Analysis

### Main Program Flow

```
START
  ↓
┌─────────────────────────────────────────────────────────────────────────────────┐
│                              INITIALIZATION                                     │
├─────────────────────────────────────────────────────────────────────────────────┤
│ 1. initAfterJump()         - Post-bootloader initialization                    │
│ 2. checkDeviceInfo()       - Validate device information                       │
│ 3. initCorePeripherals()   - Initialize core MCU peripherals                   │
│ 4. enableCorePeripherals() - Enable initialized peripherals                    │
│ 5. loadEEpromSettings()    - Load configuration from EEPROM                    │
│ 6. Configuration Setup    - Apply settings based on EEPROM                     │
└─────────────────────────────────────────────────────────────────────────────────┘
  ↓
┌─────────────────────────────────────────────────────────────────────────────────┐
│                           MODE CONFIGURATION                                   │
├─────────────────────────────────────────────────────────────────────────────────┤
│ • RC Car Mode Setup        - Reverse handling, throttle mapping               │
│ • Sine Start Configuration - Sinusoidal startup parameters                    │
│ • Direction Setup          - Forward/reverse based on EEPROM                  │
│ • PWM Configuration        - Timer and duty cycle setup                       │
│ • Safety Features          - Watchdog, current limiting, voltage cutoff       │
└─────────────────────────────────────────────────────────────────────────────────┘
  ↓
┌─────────────────────────────────────────────────────────────────────────────────┐
│                              MAIN LOOP                                        │
├─────────────────────────────────────────────────────────────────────────────────┤
│ WHILE(1) {                                                                     │
│   ┌─────────────────────────────────────────────────────────────────────────┐  │
│   │                    SIGNAL PROCESSING                                   │  │
│   ├─────────────────────────────────────────────────────────────────────────┤  │
│   │ • setInput()           - Process input signals (PWM/DSHOT/etc)        │  │
│   │ • processDshot()       - Handle DSHOT protocol                        │  │
│   │ • Signal Timeout Check - Safety timeout handling                      │  │
│   └─────────────────────────────────────────────────────────────────────────┘  │
│                                    ↓                                           │
│   ┌─────────────────────────────────────────────────────────────────────────┐  │
│   │                    MOTOR CONTROL                                       │  │
│   ├─────────────────────────────────────────────────────────────────────────┤  │
│   │ IF (stepper_sine == 0) {                                               │  │
│   │   • getBemfState()     - Read back-EMF for position sensing           │  │
│   │   • commutate()        - Switch motor phases                          │  │
│   │   • zcfoundroutine()   - Zero-crossing detection                      │  │
│   │   • PID Controllers    - Speed, current, stall protection            │  │
│   │ } ELSE {                                                               │  │
│   │   • advanceincrement() - Sine wave stepping                          │  │
│   │   • Phase positioning  - Sinusoidal motor control                    │  │
│   │ }                                                                      │  │
│   └─────────────────────────────────────────────────────────────────────────┘  │
│                                    ↓                                           │
│   ┌─────────────────────────────────────────────────────────────────────────┐  │
│   │                    TELEMETRY & MONITORING                              │  │
│   ├─────────────────────────────────────────────────────────────────────────┤  │
│   │ • ADC Processing       - Temperature, voltage, current                │  │
│   │ • Telemetry Output     - KISS/Serial telemetry                        │  │
│   │ • Safety Monitoring    - Over-current, over-temperature, low voltage  │  │
│   │ • Watchdog Refresh     - System reliability                           │  │
│   └─────────────────────────────────────────────────────────────────────────┘  │
│                                    ↓                                           │
│   ┌─────────────────────────────────────────────────────────────────────────┐  │
│   │                    MODE-SPECIFIC PROCESSING                            │  │
│   ├─────────────────────────────────────────────────────────────────────────┤  │
│   │ • BRUSHED_MODE         - Brushed motor control                        │  │
│   │ • GIMBAL_MODE          - Gimbal servo control                         │  │
│   │ • FIXED_DUTY_MODE      - Fixed duty cycle operation                   │  │
│   │ • DRONECAN_SUPPORT     - DroneCAN protocol handling                   │  │
│   └─────────────────────────────────────────────────────────────────────────┘  │
│ }                                                                              │
└─────────────────────────────────────────────────────────────────────────────────┘
```

### Interrupt-Driven Subsystems

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                           INTERRUPT HANDLERS                                   │
├─────────────────────────────────────────────────────────────────────────────────┤
│ 1. tenKhzRoutine()         - 20kHz control loop (renamed from tenKhzRoutine)   │
│    • Duty cycle ramping    - Smooth acceleration/deceleration                  │
│    • PID calculations      - Current, speed, stall protection                  │
│    • PWM updates           - Motor phase control                               │
│    • Arming logic          - Safety state management                           │
│                                                                                 │
│ 2. PeriodElapsedCallback() - Timer-based commutation                           │
│    • Commutation timing    - Phase switching control                           │
│    • Zero-crossing events  - Position feedback                                 │
│                                                                                 │
│ 3. Input Capture ISR       - Signal input processing                           │
│    • PWM signal capture    - Servo/PWM input                                   │
│    • DSHOT decoding        - Digital signal processing                         │
│                                                                                 │
│ 4. ADC Conversion ISR      - Sensor data acquisition                           │
│    • Current measurement   - Motor current monitoring                          │
│    • Temperature reading   - Thermal protection                                │
│    • Voltage monitoring    - Battery voltage sensing                           │
└─────────────────────────────────────────────────────────────────────────────────┘
```

## Code Issues Identified

### 1. **main.c Complexity**
- **Size**: 2,235 lines in a single file
- **Cyclomatic Complexity**: Very high due to nested conditions and multiple modes
- **Readability**: Difficult to navigate and understand
- **Maintainability**: Hard to modify without introducing bugs

### 2. **Duplicate Code Patterns**
- **Input mapping**: Multiple similar `map()` calls throughout
- **GPIO operations**: Repeated patterns for pin manipulation
- **PID calculations**: Similar PID logic in multiple places
- **Safety checks**: Repeated boundary and safety validations
- **EEPROM handling**: Similar read/write patterns

### 3. **Magic Numbers**
- Hardcoded values without clear constants
- Timing values scattered throughout code
- Threshold values without explanation

### 4. **Function Responsibilities**
- Functions doing too many things
- Mixed levels of abstraction
- Tight coupling between unrelated functionality

## Refactoring Plan

### Phase 1: Function Extraction & Modularization
1. **Motor Control Module** (`motor_control.c/h`)
   - `commutate()`, `getBemfState()`, `zcfoundroutine()`
   - `startMotor()`, `advanceincrement()`

2. **Input Processing Module** (`input_processing.c/h`)
   - `setInput()`, `processDshot()`
   - Signal validation and mapping

3. **Control Loops Module** (`control_loops.c/h`)
   - `tenKhzRoutine()`, `PeriodElapsedCallback()`
   - PID controllers and timing

4. **Safety & Monitoring Module** (`safety_monitoring.c/h`)
   - Temperature, voltage, current monitoring
   - Safety cutoffs and protections

5. **Configuration Module** (`configuration.c/h`)
   - EEPROM handling, settings validation
   - Mode-specific configuration

### Phase 2: Duplicate Code Elimination
1. **Common Input Mapping Functions**
2. **Unified GPIO Operations**
3. **Standardized Safety Checks**
4. **Common PID Implementation**

### Phase 3: Constants & Configuration
1. **Constants Header** (`constants.h`)
2. **Default Configuration** (`defaults.h`)
3. **Target-Specific Overrides**

### Benefits of Refactoring
- **Improved Readability**: Smaller, focused functions
- **Better Maintainability**: Easier to modify and extend
- **Reduced Bugs**: Less duplicate code means fewer places to fix
- **Enhanced Testing**: Smaller units can be tested independently
- **Code Reuse**: Common functions can be shared across targets

### Implementation Strategy
- Use `inline` functions for performance-critical code
- Maintain exact same execution paths
- Preserve all existing functionality
- Add comprehensive documentation
- Ensure binary compatibility across targets