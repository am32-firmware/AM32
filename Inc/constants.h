/*
 * constants.h
 *
 * AM32 ESC Firmware Constants
 * Centralizes all magic numbers and configuration values
 */

#ifndef CONSTANTS_H_
#define CONSTANTS_H_

#include <stdint.h>

// ==================== TIMING CONSTANTS ====================
#define LOOP_FREQUENCY_HZ                   10000U    // Main loop frequency
#define PID_LOOP_DIVIDER                    10U       // PID loop runs at 1kHz
#define BEMF_TIMEOUT_DEFAULT                10U       // Default BEMF timeout
#define BEMF_TIMEOUT_CRAWLER                100U      // BEMF timeout for crawler mode
#define SIGNAL_TIMEOUT_ARMED                5000U     // Half second timeout when armed
#define SIGNAL_TIMEOUT_DISARMED             20000U    // 2 seconds when not armed
#define ARMED_TIMEOUT_THRESHOLD             10000U    // 1 second arming timeout
#define TELEMETRY_INTERVAL_MS               32U       // Telemetry output interval

// ==================== MOTOR CONTROL CONSTANTS ====================
#define MINIMUM_DUTY_CYCLE_DEFAULT          0U        // Default minimum duty cycle
#define STARTUP_MAX_DUTY_CYCLE_DEFAULT      400U      // Maximum startup duty cycle
#define STALL_PROTECT_MINIMUM_DUTY_DEFAULT  50U       // Minimum duty for stall protection
#define THROTTLE_MAX_LOW_RPM                1000U     // Max throttle at low RPM
#define THROTTLE_MAX_HIGH_RPM               2000U     // Max throttle at high RPM
#define LOW_RPM_LEVEL_DEFAULT               20U       // Low RPM threshold
#define HIGH_RPM_LEVEL_DEFAULT              70U       // High RPM threshold

// ==================== COMMUTATION CONSTANTS ====================
#define COMMUTATION_INTERVALS_COUNT         6U        // Number of commutation intervals
#define ZERO_CROSS_THRESHOLD                100U      // Zero crossing threshold
#define AVERAGE_INTERVAL_THRESHOLD          5000U     // Average interval threshold
#define COMMUTATION_INTERVAL_INITIAL        5000U     // Initial commutation interval
#define COMMUTATION_INTERVAL_MAX            45000U    // Maximum commutation interval
#define REVERSE_SPEED_THRESHOLD             1000U     // Reverse speed change threshold

// ==================== INPUT SIGNAL CONSTANTS ====================
#define INPUT_SIGNAL_MIN                    0U        // Minimum input signal
#define INPUT_SIGNAL_MAX                    2047U     // Maximum input signal
#define INPUT_SIGNAL_ARMED_MIN              47U       // Minimum armed signal
#define INPUT_SIGNAL_NEUTRAL                1000U     // Neutral signal position
#define SERVO_DEAD_BAND_DEFAULT             5U        // Default servo dead band
#define REVERSING_DEAD_BAND_DEFAULT         0U        // Default reversing dead band
#define DSHOT_COMMAND_THRESHOLD             48U       // DSHOT command threshold
#define DSHOT_FORWARD_THRESHOLD             1047U     // DSHOT forward threshold

// ==================== SAFETY CONSTANTS ====================
#define TEMPERATURE_OFFSET_DEFAULT          230U      // Default temperature offset
#define LOW_VOLTAGE_COUNT_THRESHOLD         10000U    // Low voltage count threshold (10s)
#define CELL_COUNT_VOLTAGE_THRESHOLD        370U      // Cell count detection voltage
#define LOW_CELL_VOLTAGE_CUTOFF             300U      // Low cell voltage cutoff (3.0V)
#define CURRENT_OFFSET_DEFAULT              0U        // Default current offset
#define MILLIVOLT_PER_AMP_DEFAULT           66U       // Default current sense ratio
#define VOLTAGE_DIVIDER_DEFAULT             110U      // Default voltage divider ratio

// ==================== PWM CONSTANTS ====================
#define PWM_FREQUENCY_MIN                   8U        // Minimum PWM frequency (kHz)
#define PWM_FREQUENCY_MAX                   48U       // Maximum PWM frequency (kHz)
#define PWM_FREQUENCY_DEFAULT               24U       // Default PWM frequency (kHz)
#define TIMER1_RESOLUTION                   2000U     // Timer1 resolution steps
#define DEAD_TIME_DEFAULT                   60U       // Default dead time

// ==================== PID CONSTANTS ====================
#define PID_INTEGRAL_LIMIT_DEFAULT          10000     // Default PID integral limit
#define PID_OUTPUT_LIMIT_DEFAULT            50000     // Default PID output limit
#define CURRENT_PID_KP_DEFAULT              400       // Current PID proportional gain
#define CURRENT_PID_KI_DEFAULT              0         // Current PID integral gain
#define CURRENT_PID_KD_DEFAULT              1000      // Current PID derivative gain
#define SPEED_PID_KP_DEFAULT                10        // Speed PID proportional gain
#define SPEED_PID_KI_DEFAULT                0         // Speed PID integral gain
#define SPEED_PID_KD_DEFAULT                100       // Speed PID derivative gain
#define STALL_PID_KP_DEFAULT                1         // Stall PID proportional gain
#define STALL_PID_KI_DEFAULT                0         // Stall PID integral gain
#define STALL_PID_KD_DEFAULT                50        // Stall PID derivative gain

// ==================== SINE MODE CONSTANTS ====================
#define SINE_MODE_STEPS                     144U      // Number of sine wave steps
#define SINE_MODE_CHANGEOVER_DEFAULT        10U       // Default sine mode changeover level
#define SINE_MODE_POWER_DEFAULT             5U        // Default sine mode power
#define SINE_AMPLITUDE_DEFAULT              500U      // Default sine wave amplitude
#define PHASE_ADVANCE_DEFAULT               16U       // Default phase advance
#define PHASE_ADVANCE_MIN                   0U        // Minimum phase advance
#define PHASE_ADVANCE_MAX                   32U       // Maximum phase advance

// ==================== BRUSHED MODE CONSTANTS ====================
#define BRUSHED_RAMP_RATE                   5U        // Brushed motor ramp rate
#define BRUSHED_MAX_DUTY                    2000U     // Maximum brushed duty cycle
#define BRUSHED_BRAKE_DUTY                  1000U     // Brushed brake duty cycle

// ==================== TELEMETRY CONSTANTS ====================
#define TELEMETRY_PACKET_SIZE               10U       // Standard telemetry packet size
#define TELEMETRY_EXTENDED_SIZE             49U       // Extended telemetry packet size
#define TELEMETRY_TEMPERATURE_SCALE         1U        // Temperature scaling factor
#define TELEMETRY_VOLTAGE_SCALE             25U       // Voltage scaling factor  
#define TELEMETRY_CURRENT_SCALE             50U       // Current scaling factor
#define TELEMETRY_RPM_SCALE                 10U       // RPM scaling factor

// ==================== DSHOT CONSTANTS ====================
#define DSHOT_FRAMETIME_300_HIGH            8U        // DSHOT300 high time
#define DSHOT_FRAMETIME_300_LOW             4U        // DSHOT300 low time
#define DSHOT_FRAMETIME_600_HIGH            4U        // DSHOT600 high time
#define DSHOT_FRAMETIME_600_LOW             2U        // DSHOT600 low time
#define DSHOT_PRIORITY_THRESHOLD            500U      // DSHOT priority threshold
#define DSHOT_EXTENDED_TELEMETRY_TEMP       0x0020    // Extended telemetry temp flag
#define DSHOT_EXTENDED_TELEMETRY_CURRENT    0x0060    // Extended telemetry current flag
#define DSHOT_EXTENDED_TELEMETRY_VOLTAGE    0x0040    // Extended telemetry voltage flag

// ==================== BUFFER SIZES ====================
#define DMA_BUFFER_SIZE                     64U       // DMA buffer size
#define EEPROM_BUFFER_SIZE                  128U      // EEPROM buffer size
#define TELEMETRY_BUFFER_SIZE               16U       // Telemetry buffer size
#define COMMUTATION_BUFFER_SIZE             6U        // Commutation intervals buffer

// ==================== MATHEMATICAL CONSTANTS ====================
#define MICROSECONDS_PER_SECOND             1000000U  // Microseconds per second
#define MILLISECONDS_PER_SECOND             1000U     // Milliseconds per second
#define DEGREES_PER_CIRCLE                  360U      // Degrees in a circle
#define PERCENT_TO_SCALE                    100U      // Percentage scaling
#define ELECTRICAL_DEGREES_PER_STEP         2.5f      // Electrical degrees per step

// ==================== VERSION CONSTANTS ====================
#define VERSION_MAJOR                       2U        // Major version
#define VERSION_MINOR                       16U       // Minor version
#define EEPROM_VERSION                      1U        // EEPROM version

// ==================== FILTER CONSTANTS ====================
#define FILTER_LEVEL_MIN                    2U        // Minimum filter level
#define FILTER_LEVEL_MAX                    12U       // Maximum filter level
#define FILTER_LEVEL_DEFAULT                6U        // Default filter level
#define BEMF_COUNTS_MIN                     3U        // Minimum BEMF counts
#define BEMF_COUNTS_MAX                     15U       // Maximum BEMF counts

// ==================== RAMP CONSTANTS ====================
#define MAX_RAMP_STARTUP                    20U       // Maximum startup ramp
#define MAX_RAMP_LOW_RPM                    5U        // Maximum low RPM ramp
#define MAX_RAMP_HIGH_RPM                   1U        // Maximum high RPM ramp
#define RAMP_DIVIDER_DEFAULT                1U        // Default ramp divider

// ==================== SPECIAL MODE CONSTANTS ====================
#define RC_CAR_BRAKE_POWER                  200U      // RC car brake power
#define GIMBAL_STEP_DELAY                   300U      // Gimbal step delay
#define CRAWLER_THROTTLE_LIMIT              400U      // Crawler throttle limit
#define FIXED_DUTY_POWER_MAX                100U      // Maximum fixed duty power
#define FIXED_SPEED_RPM_MAX                 10000U    // Maximum fixed speed RPM

#endif /* CONSTANTS_H_ */