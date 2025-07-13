# AM32 ESC Firmware Flow Diagram

## System Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                        AM32 ESC Firmware                        │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌──────────────┐     ┌──────────────┐     ┌──────────────┐   │
│  │ Initialization│     │  Main Loop   │     │  Interrupts  │   │
│  └──────┬───────┘     └──────┬───────┘     └──────┬───────┘   │
│         │                    │                     │            │
│         ▼                    ▼                     ▼            │
│  ┌──────────────┐     ┌──────────────┐     ┌──────────────┐   │
│  │ Core Init    │     │ Input Process│     │ Timer ISRs   │   │
│  │ EEPROM Load  │     │ Motor Control│     │ ADC ISR      │   │
│  │ Mode Setup   │     │ Telemetry    │     │ COMP ISR     │   │
│  └──────────────┘     └──────────────┘     └──────────────┘   │
└─────────────────────────────────────────────────────────────────┘
```

## Detailed Program Flow

### 1. System Initialization

```
main()
  │
  ├─► initAfterJump()
  ├─► checkDeviceInfo() 
  ├─► initCorePeripherals()
  ├─► enableCorePeripherals()
  ├─► loadEEpromSettings()
  │     └─► Validate EEPROM version
  │     └─► Apply settings
  │
  ├─► Mode Configuration
  │     ├─► RC Car Mode
  │     ├─► Brushed Mode  
  │     ├─► Fixed Duty Mode
  │     ├─► Fixed Speed Mode
  │     └─► Normal Brushless Mode
  │
  ├─► Input Configuration
  │     ├─► DSHOT
  │     ├─► Servo PWM
  │     ├─► ADC Input
  │     └─► CRSF
  │
  └─► Start Main Loop
```

### 2. Main Control Loop

```
while(1)
  │
  ├─► Calculate e_com_time (electrical commutation time)
  │
  ├─► Process Input Signal
  │     ├─► Fixed Modes: setInput()
  │     └─► Normal Mode: processDshot() or setInput()
  │
  ├─► Update BEMF Thresholds
  │
  ├─► Watchdog Reload
  │
  ├─► Variable PWM Frequency Adjustment
  │
  ├─► Signal Timeout Handling
  │     ├─► Armed timeout (0.5s)
  │     └─► Not armed timeout (2s)
  │
  ├─► Telemetry Processing (1s interval)
  │     ├─► Current consumption
  │     ├─► Temperature
  │     ├─► Voltage
  │     └─► Extended DSHOT telemetry
  │
  ├─► Motor Control State Machine
  │     │
  │     ├─► Brushless Mode
  │     │     ├─► BEMF timeout handling
  │     │     ├─► Desync detection
  │     │     ├─► Zero crossing detection
  │     │     └─► Commutation control
  │     │
  │     ├─► Stepper/Sine Mode
  │     │     ├─► Sine wave generation
  │     │     ├─► Step advance
  │     │     └─► Transition to running
  │     │
  │     └─► Brushed Mode
  │           └─► runBrushedLoop()
  │
  ├─► ADC Processing (when flag set)
  │     ├─► Temperature conversion
  │     ├─► Battery voltage
  │     ├─► Current sensing
  │     └─► Low voltage cutoff
  │
  └─► DroneCAN Update (if enabled)
```

### 3. Key Interrupt Service Routines

```
┌─────────────────────────────────────┐
│        tenKhzRoutine() - 20kHz      │
├─────────────────────────────────────┤
│ • Duty cycle calculations           │
│ • Current limiting                  │
│ • Speed control PID                 │
│ • Stall protection                  │
│ • Signal timeout counting           │
│ • Telemetry timing                  │
└─────────────────────────────────────┘

┌─────────────────────────────────────┐
│     PeriodElapsedCallback()         │
├─────────────────────────────────────┤
│ • Zero crossing detection           │
│ • Commutation timing                │
│ • BEMF state machine               │
└─────────────────────────────────────┘

┌─────────────────────────────────────┐
│      ADC DMA Callback               │
├─────────────────────────────────────┤
│ • ADC conversion complete           │
│ • Data processing trigger           │
└─────────────────────────────────────┘
```

### 4. Motor Control States

```
                    ┌─────────────┐
                    │   STOPPED   │
                    └──────┬──────┘
                           │ Input > threshold
                    ┌──────▼──────┐
                    │   STARTUP   │
                    │ (Sine Mode) │
                    └──────┬──────┘
                           │ Speed reached
                    ┌──────▼──────┐
                    │   RUNNING   │
                    │ (BEMF Mode) │
                    └──────┬──────┘
                           │ Desync/Stop
                    └──────┴──────┘
```

## Key Data Flows

### Input Signal Processing
```
DSHOT/PWM Input → DMA Buffer → Signal Validation → Input Scaling → Motor Control
```

### Commutation Control
```
BEMF Comparator → Zero Cross Detection → Timing Calculation → Commutation → PWM Update
```

### Telemetry Flow
```
ADC Values → Filtering → Calculation → Packet Formation → UART/DSHOT Telemetry
```

## Module Dependencies After Refactoring

```
┌─────────────────────────────────────────────────────────────┐
│                          main.c                             │
│  ┌───────────────────────────────────────────────────────┐ │
│  │                     Includes:                          │ │
│  │  • motor_control.h   - Motor control functions        │ │
│  │  • pid_control.h     - PID controllers                │ │
│  │  • adc_telemetry.h   - ADC and telemetry             │ │
│  │  • pwm_control.h     - PWM management                 │ │
│  │  • system_config.h   - System configuration           │ │
│  └───────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────┘

Each module contains inline functions to maintain performance:
• No function call overhead
• Compiler optimization preserved
• Clear separation of concerns
```

## Execution Flow Timing

```
┌────────────────────────────────────────────────────────────┐
│                    Time-Critical Path                      │
├────────────────────────────────────────────────────────────┤
│                                                            │
│  BEMF Event ──► Comparator ISR ──► Zero Cross Detection   │
│      │                                      │              │
│      └──────────── < 10μs ─────────────────┘              │
│                                                            │
│  Commutation Timer ──► PeriodElapsedCallback ──► comStep  │
│         │                                           │      │
│         └──────────────── < 5μs ───────────────────┘      │
│                                                            │
└────────────────────────────────────────────────────────────┘
```

## Configuration Modes

```
┌─────────────────┬─────────────────┬─────────────────────┐
│  Input Mode     │  Motor Mode     │  Special Features   │
├─────────────────┼─────────────────┼─────────────────────┤
│ • DSHOT         │ • Brushless     │ • Sine Startup      │
│ • Servo PWM     │ • Brushed       │ • Variable PWM      │
│ • ADC           │ • Gimbal        │ • Current Limiting  │
│ • CRSF          │                 │ • Stall Protection  │
│ • Serial        │                 │ • Temperature Limit │
└─────────────────┴─────────────────┴─────────────────────┘
```