/* AM32- multi-purpose brushless controller firmware for the stm32f051 */

//===========================================================================
//=============================== Changelog =================================
//===========================================================================
/*
 * 1.54 Changelog;
 * --Added firmware name to targets and firmware version to main
 * --added two more dshot to beacons 1-3 currently working
 * --added KV option to firmware, low rpm power protection is based on KV
 * --start power now controls minimum idle power as well as startup strength.
 * --change default timing to 22.5
 * --Lowered default minimum idle setting to 1.5 percent duty cycle, slider
range from 1-2.
 * --Added dshot commands to save settings and reset ESC.
 *
 *1.56 Changelog.
 * -- added check to stall protection to wait until after 40 zero crosses to fix
high startup throttle hiccup.
 * -- added TIMER 1 update interrupt and PWM changes are done once per pwm
period
 * -- reduce commutation interval averaging length
 * -- reduce false positive filter level to 2 and eliminate threshold where
filter is stopped.
 * -- disable interrupt before sounds
 * -- disable TIM1 interrupt during stepper sinusoidal mode
 * -- add 28us delay for dshot300
 * -- report 0 rpm until the first 10 successful steps.
 * -- move serial ADC telemetry calculations and desync check to 10Khz
interrupt.
 *
 * 1.57
 * -- remove spurious commutations and rpm data at startup by polling for longer
interval on startup
 *
 * 1.58
 * -- move signal timeout to 10khz routine and set armed timeout to one quarter
second 2500 / 10000
 * 1.59
 * -- moved comp order definitions to target.h
 * -- fixed update version number if older than new version
 * -- cleanup, moved all input and output to IO.c
 * -- moved comparator functions to comparator.c
 * -- removed ALOT of useless variables
 * -- added siskin target
 * -- moved pwm changes to 10khz routine
 * -- moved basic functions to functions.c
 * -- moved peripherals setup to periherals.c
 * -- added crawler mode settings
 *
 * 1.60
 * -- added sine mode hysteresis
 * -- increased power in stall protection and lowered start rpm for crawlers
 * -- removed onehot125 from crawler mode
 * -- reduced maximum startup power from 400 to 350
 * -- change minimum duty cycle to DEAD_TIME
 * -- version and name moved to permanent spot in FLASH memory, thanks mikeller
 *
 * 1.61
 * -- moved duty cycle calculation to 10khz and added max change option.
 * -- decreased maximum interval change to 25%
 * -- reduce wait time on fast acceleration (fast_accel)
 * -- added check in interrupt for early zero cross
 *
 * 1.62
 * --moved control to 10khz loop
 * --changed condition for low rpm filter for duty cycle from || to &&
 * --introduced max deceleration and set it to 20ms to go from 100 to 0
 * --added configurable servo throttle ranges
 *
 *
 *1.63
 *-- increase time for zero cross error detection below 250us commutation
interval
 *-- increase max change a low rpm x10
 *-- set low limit of throttle ramp to a lower point and increase upper range
 *-- change desync event from full restart to just lower throttle.

 *1.64
 * --added startup check for continuous high signal, reboot to enter bootloader.
 *-- added brake on stop from eeprom
 *-- added stall protection from eeprom
 *-- added motor pole divider for sinusoidal and low rpm power protection
 *-- fixed dshot commands, added confirmation beeps and removed blocking
behavior
 *--
 *1.65
 *-- Added 32 millisecond telemetry output
 *-- added low voltage cutoff , divider value and cutoff voltage needs to be
added to eeprom
 *-- added beep to indicate cell count if low voltage active
 *-- added current reading on pa3 , conversion factor needs to be added to
eeprom
 *-- fixed servo input capture to only read positive pulse to handle higher
refresh rates.
 *-- disabled oneshot 125.
 *-- extended servo range to match full output range of receivers
 *-- added RC CAR style reverse, proportional brake on first reverse , double
tap to change direction
 *-- added brushed motor control mode
 *-- added settings to EEPROM version 1
 *-- add gimbal control option.
 *--
 *1.66
 *-- move idwg init to after input tune
 *-- remove reset after save command -- dshot
 *-- added wraith32 target
 *-- added average pulse check for signal detection
 *--
 *1.67
 *-- Rework file structure for multiple MCU support
 *-- Add g071 mcu
 *--
 *1.68
 *--increased allowed average pulse length to avoid double startup
 *1.69
 *--removed line re-enabling comparator after disabling.
 *1.70 fix dshot for Kiss FC
 *1.71 fix dshot for Ardupilot / Px4 FC
 *1.72 Fix telemetry output and add 1 second arming.
 *1.73 Fix false arming if no signal. Remove low rpm throttle protection below
300kv *1.74 Add Sine Mode range and drake brake strength adjustment *1.75
Disable brake on stop for PWM_ENABLE_BRIDGE Removed automatic brake on stop on
neutral for RC car proportional brake. Adjust sine speed and stall protection
speed to more closely match makefile fixes from Cruwaller Removed gd32 build,
until firmware is functional *1.76 Adjust g071 PWM frequency, and startup power
to be same frequency as f051. Reduce number of polling back emf checks for g071
 *1.77 increase PWM frequency range to 8-48khz
 *1.78 Fix bluejay tunes frequency and speed.
           Fix g071 Dead time
           Increment eeprom version
 *1.79 Add stick throttle calibration routine
           Add variable for telemetry interval
 *1.80 -Enable Comparator blanking for g071 on timer 1 channel 4
           -add hardware group F for Iflight Blitz
           -adjust parameters for pwm frequency
           -add sine mode power variable and eeprom setting
           -fix telemetry rpm during sine mode
           -fix sounds for extended pwm range
           -Add adjustable braking strength when driving
 *1.81 -Add current limiting PID loop
           -fix current sense scale
           -Increase brake power on maximum reverse ( car mode only)
           -Add HK and Blpwr targets
           -Change low kv motor throttle limit
           -add reverse speed threshold changeover based on motor kv
           -doubled filter length for motors under 900kv
*1.82  -Add speed control pid loop.
*1.83  -Add stall protection pid loop.
           -Improve sine mode transition.
           -decrease speed step re-entering sine mode
           -added fixed duty cycle and speed mode build option
           -added rpm_controlled by input signal ( to be added to config tool )
*1.84  -Change PID value to int for faster calculations
           -Enable two channel brushed motor control for dual motors
           -Add current limit max duty cycle
*1.85  -fix current limit not allowing full rpm on g071 or low pwm frequency
                -remove unused brake on stop conditional
*1.86  - create do-once in sine mode instead of setting pwm mode each time.
*1.87  - fix fixed mode max rpm limits
*1.88  - Fix stutter on sine mode re-entry due to position reset
*1.89  - Fix drive by rpm mode scaling.
           - Fix dshot px4 timings
*1.90  - Disable comp interrupts for brushed mode
           - Re-enter polling mode after prop strike or desync
           - add G071 "N" variant
           - add preliminary Extended Dshot
*1.91  - Reset average interval time on desync only after 100 zero crosses
*1.92  - Move g071 comparator blanking to TIM1 OC5
           - Increase ADC read frequency and current sense filtering
           - Add addressable LED strip for G071 targets
*1.93  - Optimization for build process
       - Add firmware file name to each target hex file
       -fix extended telemetry not activating dshot600
       -fix low voltage cuttoff timeout
*1.94  - Add selectable input types
*1.95  - reduce timeout to 0.5 seconds when armed
*1.96  - Improved erpm accuracy dshot and serial telemetry, thanks Dj-Uran
             - Fix PID loop integral.
                 - add overcurrent low voltage cuttoff to brushed mode.
*1.97    - enable input pullup
*1.98    - Dshot erpm rounding compensation.
*1.99    - Add max duty cycle change to individual targets ( will later become
an settings option)
                 - Fix dshot telemetry delay f4 and e230 mcu
*2.00    - Cleanup of target structure
*2.01    - Increase 10khztimer to 20khz, increase max duty cycle change.
*2.02	 - Increase startup power for inverted output targets.
*2.03    - Move chime from dshot direction change commands to save command.
*2.04    - Fix current protection, max duty cycle not increasing
                 - Fix double startup chime
                 - Change current averaging method for more precision
                 - Fix startup ramp speed adjustment
*2.05		 - Fix ramp tied to input frequency
*2.06    - fix input pullups
         - Remove half xfer insterrupt from servo routine
                                 - update running brake and brake on stop
*2.07    - Dead time change f4a
*2.08		 - Move zero crosss timing
*2.09    - filter out short zero crosses
*2.10    - Polling only below commutation intverval of 1500-2000us
				 - fix tune frequency again
*2.11    - RC-Car mode fix
*2.12    - Reduce Advance on hard braking
*2.13    - Remove Input capture filter for dshot2400
         - Change dshot 300 speed detection threshold 
*2.14    - Reduce G071 zero cross checks
         - Assign all mcu's duty cycle resolution 2000 steps
*2.15    - Enforce 1/2 commutation interval as minimum for g071
         - Revert timing change on braking
				 - Add per target over-ride option to max duty cycle change.
				 - todo fix signal detection
*2.16    - add L431 
				 - add variable auto timing
				 - add droneCAN
*/
#include "main.h"
#include "ADC.h"
#include "IO.h"
#include "common.h"
#include "comparator.h"
#include "dshot.h"
#include "eeprom.h"
#include "functions.h"
#include "peripherals.h"
#include "phaseouts.h"
#include "serial_telemetry.h"
#include "kiss_telemetry.h"
#include "signal.h"
#include "sounds.h"
#include "targets.h"
#include <stdint.h>
#include <string.h>
#include <assert.h>

// Include new modular headers
#include "motor_control.h"
#include "pid_control.h"
#include "adc_telemetry.h"
#include "pwm_control.h"
#include "system_config.h"

#ifdef USE_LED_STRIP
#include "WS2812.h"
#endif

#ifdef USE_CRSF_INPUT
#include "crsf.h"
#endif

#if DRONECAN_SUPPORT
#include "DroneCAN/DroneCAN.h"
#endif

#include <version.h>

void zcfoundroutine(void);

// firmware build options !! fixed speed and duty cycle modes are not to be used
// with sinusoidal startup !!

//#define FIXED_DUTY_MODE  // bypasses signal input and arming, uses a set duty
// cycle. For pumps, slot cars etc 
//#define FIXED_DUTY_MODE_POWER 100     //
// 0-100 percent not used in fixed speed mode

// #define FIXED_SPEED_MODE  // bypasses input signal and runs at a fixed rpm
// using the speed control loop PID 
//#define FIXED_SPEED_MODE_RPM  1000  //
// intended final rpm , ensure pole pair numbers are entered correctly in config
// tool.

// #define BRUSHED_MODE         // overrides all brushless config settings,
// enables two channels for brushed control 
//#define GIMBAL_MODE     // also
// sinusoidal_startup needs to be on, maps input to sinusoidal angle.

//===========================================================================
//=============================  Defaults =============================
//===========================================================================

uint8_t drive_by_rpm = 0;
uint32_t MAXIMUM_RPM_SPEED_CONTROL = 10000;
uint32_t MINIMUM_RPM_SPEED_CONTROL = 1000;

// assign speed control PID values values are x10000
fastPID speedPid = { // commutation speed loop time
    .Kp = 10,
    .Ki = 0,
    .Kd = 100,
    .integral_limit = 10000,
    .output_limit = 50000
};

fastPID currentPid = { // 1khz loop time
    .Kp = 400,
    .Ki = 0,
    .Kd = 1000,
    .integral_limit = 20000,
    .output_limit = 100000
};

fastPID stallPid = { // 1khz loop time
    .Kp = 1,
    .Ki = 0,
    .Kd = 50,
    .integral_limit = 10000,
    .output_limit = 50000
};

EEprom_t eepromBuffer;
volatile uint8_t ramp_divider;
volatile uint8_t max_ramp_startup = RAMP_SPEED_STARTUP;
volatile uint8_t max_ramp_low_rpm = RAMP_SPEED_LOW_RPM;
volatile uint8_t max_ramp_high_rpm = RAMP_SPEED_HIGH_RPM;
char send_esc_info_flag;
uint32_t eeprom_address = EEPROM_START_ADD; 
uint16_t prop_brake_duty_cycle = 0;
uint16_t ledcounter = 0;
uint32_t process_time = 0;
uint32_t start_process = 0;
uint16_t one_khz_loop_counter = 0;
uint16_t target_e_com_time_high;
uint16_t target_e_com_time_low;
uint8_t compute_dshot_flag = 0;
uint8_t crsf_input_channel = 1;
uint8_t crsf_output_PWM_channel = 2;
uint8_t telemetry_interval_ms = 30;
uint8_t temp_advance;
uint16_t motor_kv = 2000;
uint8_t dead_time_override = DEAD_TIME;
uint16_t stall_protect_target_interval = TARGET_STALL_PROTECTION_INTERVAL;
uint16_t enter_sine_angle = 180;
char do_once_sinemode = 0;
uint8_t auto_advance_level;

//============================= Servo Settings ==============================
uint16_t servo_low_threshold = 1100; // anything below this point considered 0
uint16_t servo_high_threshold = 1900; // anything above this point considered 2000 (max)
uint16_t servo_neutral = 1500;
uint8_t servo_dead_band = 100;

//========================= Battery Cuttoff Settings ========================
char LOW_VOLTAGE_CUTOFF = 0; // Turn Low Voltage CUTOFF on or off
uint16_t low_cell_volt_cutoff = 330; // 3.3volts per cell

//=========================== END EEPROM Defaults ===========================

const char filename[30] __attribute__((section(".file_name"))) = FILE_NAME;
_Static_assert(sizeof(FIRMWARE_NAME) <=13,"Firmware name too long");   // max 12 character firmware name plus NULL 

// move these to targets folder or peripherals for each mcu
uint16_t ADC_CCR = 30;
uint16_t current_angle = 90;
uint16_t desired_angle = 90;
char return_to_center = 0;
uint16_t target_e_com_time = 0;
int16_t Speed_pid_output;
char use_speed_control_loop = 0;
int32_t input_override = 0;
int16_t use_current_limit_adjust = 2000;
char use_current_limit = 0;
int32_t stall_protection_adjust = 0;

uint32_t MCU_Id = 0;
uint32_t REV_Id = 0;

uint16_t armed_timeout_count;
uint16_t reverse_speed_threshold = 1500;
uint8_t desync_happened = 0;
char maximum_throttle_change_ramp = 1;

char crawler_mode = 0; // no longer used //
uint16_t velocity_count = 0;
uint16_t velocity_count_threshold = 75;

char low_rpm_throttle_limit = 1;

uint16_t low_voltage_count = 0;
uint16_t telem_ms_count;

uint16_t VOLTAGE_DIVIDER = TARGET_VOLTAGE_DIVIDER; // 100k upper and 10k lower resistor in divider
uint16_t
    battery_voltage; // scale in volts * 10.  1260 is a battery voltage of 12.60
char cell_count = 0;
char brushed_direction_set = 0;

uint16_t tenkhzcounter = 0;
int32_t consumed_current = 0;
int32_t smoothed_raw_current = 0;
int16_t actual_current = 0;

char lowkv = 0;

uint16_t min_startup_duty = 120;
uint16_t sin_mode_min_s_d = 120;
char bemf_timeout = 10;

char startup_boost = 50;
char reversing_dead_band = 1;

uint16_t low_pin_count = 0;

uint8_t max_duty_cycle_change = 2;
char fast_accel = 1;
char fast_deccel = 0;
uint16_t last_duty_cycle = 0;
uint16_t duty_cycle_setpoint = 0;
char play_tone_flag = 0;

typedef enum { GPIO_PIN_RESET = 0U,
    GPIO_PIN_SET } GPIO_PinState;

uint16_t startup_max_duty_cycle = 200;
uint16_t minimum_duty_cycle = DEAD_TIME;
uint16_t stall_protect_minimum_duty = DEAD_TIME;
char desync_check = 0;
char low_kv_filter_level = 20;

uint16_t tim1_arr = TIM1_AUTORELOAD; // current auto reset value
uint16_t TIMER1_MAX_ARR = TIM1_AUTORELOAD; // maximum auto reset register value
uint16_t duty_cycle_maximum = 2000; // restricted by temperature or low rpm throttle protect
uint16_t low_rpm_level = 20; // thousand erpm used to set range for throttle resrictions
uint16_t high_rpm_level = 70; //
uint16_t throttle_max_at_low_rpm = 400;
uint16_t throttle_max_at_high_rpm = 2000;

uint16_t commutation_intervals[6] = { 0 };
uint32_t average_interval = 0;
uint32_t last_average_interval;
int e_com_time;

uint16_t ADC_smoothed_input = 0;
uint8_t degrees_celsius;
int16_t converted_degrees;
uint8_t temperature_offset;
uint16_t ADC_raw_temp;
uint16_t ADC_raw_volts;
uint16_t ADC_raw_current;
uint16_t ADC_raw_input;
uint8_t PROCESS_ADC_FLAG = 0;
char send_telemetry = 0;
char telemetry_done = 0;
char prop_brake_active = 0;

char dshot_telemetry = 0;

uint8_t last_dshot_command = 0;
char old_routine = 1;
uint16_t adjusted_input = 0;

#define TEMP30_CAL_VALUE ((uint16_t*)((uint32_t)0x1FFFF7B8))
#define TEMP110_CAL_VALUE ((uint16_t*)((uint32_t)0x1FFFF7C2))

uint16_t smoothedcurrent = 0;
const uint8_t numReadings = 50; // the readings from the analog input
uint8_t readIndex = 0; // the index of the current reading
uint32_t total = 0;
uint16_t readings[50];

uint8_t bemf_timeout_happened = 0;
uint8_t changeover_step = 5;
uint8_t filter_level = 5;
uint8_t running = 0;
uint16_t advance = 0;
uint8_t advancedivisor = 6;
char rising = 1;

////Sine Wave PWM ///////////////////
int16_t pwmSin[] = {
    180, 183, 186, 189, 193, 196, 199, 202, 205, 208, 211, 214, 217, 220, 224,
    227, 230, 233, 236, 239, 242, 245, 247, 250, 253, 256, 259, 262, 265, 267,
    270, 273, 275, 278, 281, 283, 286, 288, 291, 293, 296, 298, 300, 303, 305,
    307, 309, 312, 314, 316, 318, 320, 322, 324, 326, 327, 329, 331, 333, 334,
    336, 337, 339, 340, 342, 343, 344, 346, 347, 348, 349, 350, 351, 352, 353,
    354, 355, 355, 356, 357, 357, 358, 358, 359, 359, 359, 360, 360, 360, 360,
    360, 360, 360, 360, 360, 359, 359, 359, 358, 358, 357, 357, 356, 355, 355,
    354, 353, 352, 351, 350, 349, 348, 347, 346, 344, 343, 342, 340, 339, 337,
    336, 334, 333, 331, 329, 327, 326, 324, 322, 320, 318, 316, 314, 312, 309,
    307, 305, 303, 300, 298, 296, 293, 291, 288, 286, 283, 281, 278, 275, 273,
    270, 267, 265, 262, 259, 256, 253, 250, 247, 245, 242, 239, 236, 233, 230,
    227, 224, 220, 217, 214, 211, 208, 205, 202, 199, 196, 193, 189, 186, 183,
    180, 177, 174, 171, 167, 164, 161, 158, 155, 152, 149, 146, 143, 140, 136,
    133, 130, 127, 124, 121, 118, 115, 113, 110, 107, 104, 101, 98, 95, 93,
    90, 87, 85, 82, 79, 77, 74, 72, 69, 67, 64, 62, 60, 57, 55,
    53, 51, 48, 46, 44, 42, 40, 38, 36, 34, 33, 31, 29, 27, 26,
    24, 23, 21, 20, 18, 17, 16, 14, 13, 12, 11, 10, 9, 8, 7,
    6, 5, 5, 4, 3, 3, 2, 2, 1, 1, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 3, 3, 4, 5, 5,
    6, 7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 18, 20, 21, 23,
    24, 26, 27, 29, 31, 33, 34, 36, 38, 40, 42, 44, 46, 48, 51,
    53, 55, 57, 60, 62, 64, 67, 69, 72, 74, 77, 79, 82, 85, 87,
    90, 93, 95, 98, 101, 104, 107, 110, 113, 115, 118, 121, 124, 127, 130,
    133, 136, 140, 143, 146, 149, 152, 155, 158, 161, 164, 167, 171, 174, 177
};

// int sin_divider = 2;
int16_t phase_A_position;
int16_t phase_B_position;
int16_t phase_C_position;
uint16_t step_delay = 100;
char stepper_sine = 0;
char forward = 1;
uint16_t gate_drive_offset = DEAD_TIME;

uint8_t stuckcounter = 0;
uint16_t k_erpm;
uint16_t e_rpm; // electrical revolution /100 so,  123 is 12300 erpm

uint16_t adjusted_duty_cycle;

uint8_t bad_count = 0;
uint8_t bad_count_threshold = CPU_FREQUENCY_MHZ / 24;
uint8_t dshotcommand;
uint16_t armed_count_threshold = 1000;

char armed = 0;
uint16_t zero_input_count = 0;

uint16_t input = 0;
uint16_t newinput = 0;
char inputSet = 0;
char dshot = 0;
char servoPwm = 0;
uint32_t zero_crosses;

uint8_t zcfound = 0;

uint8_t bemfcounter;
uint8_t min_bemf_counts_up = TARGET_MIN_BEMF_COUNTS;
uint8_t min_bemf_counts_down = TARGET_MIN_BEMF_COUNTS;

uint16_t lastzctime;
uint16_t thiszctime;

uint16_t duty_cycle = 0;
char step = 1;
uint32_t commutation_interval = 12500;
uint16_t waitTime = 0;
uint16_t signaltimeout = 0;
uint8_t ubAnalogWatchdogStatus = RESET;

#ifdef NEED_INPUT_READY
volatile char input_ready = 0;
#endif

// Function removed - now in pid_control.h as inline function
// int32_t doPidCalculations(struct fastPID* pidnow, int actual, int target) { ... }

void loadEEpromSettings()
{
    read_flash_bin(eepromBuffer.buffer, eeprom_address, sizeof(eepromBuffer.buffer));
    
    if (eepromBuffer.eeprom_version < 1) {
        eepromBuffer.limits.temperature = TEMP_LIMIT;
        eepromBuffer.limits.current_limit = CURRENT_LIMIT;
        eepromBuffer.limits.use_limits = USE_CURRENT_LIMIT;
        eepromBuffer.eeprom_version = 1;
        saveEEpromSettings();
    }
    
    if (eepromBuffer.eeprom_version < 2) {
        eepromBuffer.flags.TLM_FLAGS = 0x09;
        eepromBuffer.eeprom_version = 2;
        saveEEpromSettings();
    }

    if (eepromBuffer.eeprom_version < 3) {
        eepromBuffer.limits.current_limit_max_duty_cycle = CURRENT_LIMIT_MAX_DUTY_CYCLE;
        eepromBuffer.limits.low_rpm_current_limit = LOW_RPM_CURRENT_LIMIT;
        eepromBuffer.eeprom_version = 3;
        saveEEpromSettings();
    }

    if (eepromBuffer.eeprom_version < 4) {
        eepromBuffer.pwm_frequency = 24;
        eepromBuffer.eeprom_version = 4;
        saveEEpromSettings();
    }

    if (eepromBuffer.eeprom_version < 5) {
        eepromBuffer.input_type = AUTO_IN;
        eepromBuffer.eeprom_version = 5;
        saveEEpromSettings();
    }

    if (eepromBuffer.eeprom_version < 6) {
        eepromBuffer.dead_time = DEAD_TIME;
        eepromBuffer.eeprom_version = 6;
        saveEEpromSettings();
    }

    if (eepromBuffer.eeprom_version < 7) {
        eepromBuffer.active_brake_power = 10;
        eepromBuffer.eeprom_version = 7;
        saveEEpromSettings();
    }

    // Apply settings
    current_limit = eepromBuffer.limits.current_limit;
    use_current_limit_adjust = 2000;
    use_current_limit = eepromBuffer.limits.use_limits;
    low_rpm_current_limit = eepromBuffer.limits.low_rpm_current_limit;
    reverse_speed_threshold = map(motor_kv, 200, 1100, 2500, 1500);
    
    motor_kv = eepromBuffer.motor_kv * 100;
    CURRENT_LIMIT_MAX_DUTY_CYCLE = eepromBuffer.limits.current_limit_max_duty_cycle;
    TIMER1_MAX_ARR = map((eepromBuffer.pwm_frequency * 100 + 800), 1200, 4800, TIM1_AUTORELOAD - 2500, TIM1_AUTORELOAD);
    target_e_com_time_high = 60 * (200000 / (eepromBuffer.motor_kv * 100 + 100));
    target_e_com_time_low = 60 * (200000 / (eepromBuffer.motor_kv * 1200 + 1200));
    use_speed_control_loop = 0;
    if (eepromBuffer.brake_on_stop == 0) {
        stall_protect_target_interval = TARGET_STALL_PROTECTION_INTERVAL;
    } else {
        stall_protect_target_interval = 10500;
    }
    if (eepromBuffer.stall_protection == 0) {
        stall_protect_minimum_duty = 0;
    }
    drive_by_rpm = eepromBuffer.flags.DRIVE_BY_RPM;
    EDT_ARM_ENABLE = eepromBuffer.flags.TLM_FLAGS & 0x1;
    telemetry_interval_ms = map(eepromBuffer.flags.TLM_FLAGS >> 4, 0, 15, 0, 30);
    if (eepromBuffer.flags.TLM_FLAGS >> 6 == 0b00) {
        dshot_extended_telemetry = 0;
    } else if (eepromBuffer.flags.TLM_FLAGS >> 6 == 0b01) {
        dshot_extended_telemetry = 1;
    }
    crsf_input_channel = eepromBuffer.flags.CRSF_INPUT_CHANNEL;
    crsf_output_PWM_channel = eepromBuffer.flags.CRSF_OUTPUT_CHANNEL;
    
    if (eepromBuffer.dead_time < DEAD_TIME) {
        eepromBuffer.dead_time = DEAD_TIME;
    }
    dead_time_override = eepromBuffer.dead_time;
    DEAD_TIME = eepromBuffer.dead_time;
    
    detectMotorKV();
    
    switch (eepromBuffer.flags.SERVO_TYPE) {
    case 1:
        servo_low_threshold = map(eepromBuffer.servo_settings.low_threshold, 0, 255, 100, 1500);
        servo_high_threshold = map(eepromBuffer.servo_settings.high_threshold, 0, 255, 1600, 2500);
        servo_neutral = map(eepromBuffer.servo_settings.neutral, 0, 255, 1000, 2000);
        servo_dead_band = map(eepromBuffer.servo_settings.dead_zone, 0, 100, 0, 200);
        break;
    }
}

void saveEEpromSettings()
{
    save_flash_nolib(eepromBuffer.buffer, sizeof(eepromBuffer.buffer), eeprom_address);
}

uint16_t getSmoothedCurrent()
{
    total = total - readings[readIndex];
    readings[readIndex] = ADC_raw_current;
    total = total + readings[readIndex];
    readIndex = readIndex + 1;
    if (readIndex >= numReadings) {
        readIndex = 0;
    }
    smoothedcurrent = total / numReadings;
    return smoothedcurrent;
}

void getBemfState()
{
    uint8_t current_state = 0;
    if (!zcfound) {
        if (step == 1 || step == 4) { // c floating
            current_state = PHASE_C_COMP;
        }
        if (step == 2 || step == 5) { // a floating
            current_state = PHASE_A_COMP;
        }
        if (step == 3 || step == 6) { // b floating
            current_state = PHASE_B_COMP;
        }

        if (rising) {
            if (current_state) {
                bemfcounter++;
            } else {
                bad_count++;
                if (bad_count > bad_count_threshold) {
                    bemfcounter = 0;
                    bad_count = 0;
                }
            }
        } else { // falling bemf
            if (!current_state) {
                bemfcounter++;
            } else {
                bad_count++;
                if (bad_count > bad_count_threshold) {
                    bemfcounter = 0;
                    bad_count = 0;
                }
            }
        }
    }
}

void commutate()
{
    commutation_intervals[step - 1] = commutation_interval;
    if (forward == 1) {
        step++;
        if (step > 6) {
            step = 1;
            desync_check = 1;
        }
        rising = step % 2;
    } else {
        step--;
        if (step < 1) {
            step = 6;
            desync_check = 1;
        }
        rising = !(step % 2);
    }
    if (!prop_brake_active) {
        comStep(step);
    }
    changeCompInput();
    if (eepromBuffer.auto_advance) {
        COMP_DelayMS->COMPX_BLANKW = ((commutation_interval >> 1) * auto_advance_level) >> 10;
        if (commutation_interval < 400) {
            COMP_DelayMS->COMPX_BLANKW = commutation_interval / 2 - 5;
        }
    } else {
        if (commutation_interval > 500) {
            COMP_DelayMS->COMPX_BLANKW = commutation_interval / 5;
        } else {
            COMP_DelayMS->COMPX_BLANKW = 5 + commutation_interval / 5 + (commutation_interval >> 1) * advance / 128;
        }
    }
    bemfcounter = 0;
    zcfound = 0;
    waitTime = 0;
    advance = (9 * advance + eepromBuffer.advance_level) >> 2;
}

void PeriodElapsedCallback()
{
    DISABLE_COM_TIMER_INT(); // disable interrupt
    commutate();
    commutation_interval = (2 * commutation_interval + thiszctime) / 3;
    advance = 3;
    if (average_interval > 125) {
        old_routine = 0;
    }
    if (zero_crosses < 10000) {
        zero_crosses++;
    }
    if (zero_crosses > 30) {
        PLAY_TONE_FLAG = 0;
    }
    if (commutation_interval < 45) {
        commutation_interval = 45;
    }
    SET_INTERVAL_TIMER_COUNT(commutation_interval);
    ENABLE_COM_TIMER_INT();
}

void interruptRoutine()
{
    if (average_interval > 125) {
        if ((INTERVAL_TIMER_COUNT < commutation_interval >> 1)) {
            maskPhaseInterrupts();
            getBemfState();
            if (!zcfound && commutation_interval < 1500) {
                enableCompInterrupts();
            }
        }
    } else {
        if (INTERVAL_TIMER_COUNT < (commutation_interval >> 1) && commutation_interval < 500) {
            enableCompInterrupts();
        }
    }
}

void zcfoundroutine()
{
    SET_INTERVAL_TIMER_COUNT(waitTime);
    bemfcounter = 0;
    bad_count = 0;
    maskPhaseInterrupts();
    zero_crosses++;
    zcfound = 1;
    thiszctime = INTERVAL_TIMER_COUNT;
    if (rising) {
        lastzctime = thiszctime;
    }
    if (!rising && commutation_interval > 2000) {
        SET_PRESCALER_TIMER_COUNT(1);
    }
    if (!rising && commutation_interval <= 2000 && commutation_interval > 1000) {
        SET_PRESCALER_TIMER_COUNT(0);
    }
}

void advanceincrement()
{
    if (forward == 1) {
        phase_A_position++;
        if (phase_A_position > 359) {
            phase_A_position = 0;
        }
        phase_B_position = phase_A_position + 120;
        if (phase_B_position > 359) {
            phase_B_position -= 360;
        }
        phase_C_position = phase_B_position + 120;
        if (phase_C_position > 359) {
            phase_C_position -= 360;
        }
    } else {
        phase_A_position--;
        if (phase_A_position < 0) {
            phase_A_position = 359;
        }
        phase_B_position = phase_A_position + 120;
        if (phase_B_position > 359) {
            phase_B_position -= 360;
        }
        phase_C_position = phase_B_position + 120;
        if (phase_C_position > 359) {
            phase_C_position -= 360;
        }
    }
}

void tenKhzRoutine()
{
    if (maximum_throttle_change_ramp) {
        duty_cycle_setpoint = map(adjusted_input, low_rpm_throttle_limit * min_startup_duty / 100, input_override, minimum_duty_cycle, duty_cycle_maximum);
        duty_cycle_setpoint = input_override - map((input_override - duty_cycle_setpoint), 0, input_override, 0, use_current_limit_adjust);
        if (adjusted_input < 20) {
            duty_cycle_setpoint = 0;
        }
        if (duty_cycle_setpoint > duty_cycle_maximum) {
            duty_cycle_setpoint = duty_cycle_maximum;
        }
        if (commutation_interval < 100) {
            actual_current = (actual_current + (smoothed_raw_current * 3300 / 41 - CURRENT_OFFSET * 100) / MILLIVOLT_PER_AMP) >> 1;
        }
        
        limitDutyCycleChange();
        
        if (degrees_celsius > TEMP_LIMIT && degrees_celsius < 200) {
            uint8_t temperature_offset = (degrees_celsius - TEMP_LIMIT) * (LOOP_FREQUENCY_HZ / 1000);
            if (duty_cycle > temperature_offset) {
                duty_cycle = duty_cycle - temperature_offset;
            } else {
                duty_cycle = 0;
            }
        }
        if (zero_crosses < 100 || commutation_interval > 500) {
            duty_cycle_setpoint = min_startup_duty + stall_protection_adjust;
            if (duty_cycle_setpoint < minimum_duty_cycle) {
                duty_cycle_setpoint = minimum_duty_cycle;
            }
            if (duty_cycle_setpoint > 255) {
                duty_cycle_setpoint = 255;
            }
        }

        if (crawler_mode) {
            if (velocity_count < velocity_count_threshold) {
                velocity_count++;
                duty_cycle_setpoint = min_startup_duty;
            }
        }
        last_duty_cycle = duty_cycle;
    }
    
    // Update control loops
    updateSpeedControl(k_erpm, input);
    updateCurrentLimit(actual_current);
    updateStallProtection(k_erpm, average_interval, stall_protect_target_interval);

    one_khz_loop_counter++;
    if (one_khz_loop_counter > 20) { // 1khz loop
        if (compute_dshot_flag) {
            compute_dshot_flag = 0;
            computeDshotDMA();
            if (dshotcommand == 0) {
                if (dshot_telemetry < 3) {
                    send_telemetry = 1;
                }
            }
        }
        telem_ms_count++;
        if (telem_ms_count > telemetry_interval_ms && dshot_telemetry == 2) {
            send_telemetry = 1;
            telem_ms_count = 0;
        }
        tenkhzcounter++;
        one_khz_loop_counter = 0;
    }
    
    if (!armed && inputSet) {
        if (zero_input_count > 250) {
            switch(dshot) {
            case 0:
                if (adjusted_input == 0) {
                    armed = 1;
#ifdef USE_ADC_INPUT
                    zero_input_count = 0;
#else
                    zero_input_count = 250;
#endif
                }
                break;
            case 1:
                if (newinput == 0 && adjusted_input < 48) {
                    if (EDT_ARMED) {
                        armed = 1;
                        EDT_ARM_ENABLE = 1;
                        zero_input_count = 0;
                    }
                }
                break;
            }
        } else {
            zero_input_count++;
        }
    }

    if (armed) {
        if (zero_input_count > 100 || adjusted_input == 0) {
            if (adjusted_input == 0) {
                armed = 0;
                EDT_ARMED = 0;
            } else {
                zero_input_count++;
            }
        } else {
            zero_input_count = 0;
        }
    }

    if (armed && adjusted_input > 10) {
        if (TIMER1->sr & TIM_IT_UPDATE) {
            TIMER1->sr &= ~TIM_IT_UPDATE;
            adjusted_duty_cycle = (duty_cycle * tim1_arr) / 2000 + dead_time_override;
            tim1_arr = (tim1_arr + TIMER1_MAX_ARR) >> 1;
            SET_AUTO_RELOAD_PWM(tim1_arr);
            SET_DUTY_CYCLE_ALL(adjusted_duty_cycle);
            generatePwmTimerEvent();
        }
    }

    if (signaltimeout > LOOP_FREQUENCY_HZ) {
        input = 0;
        adjusted_input = 0;
        armed = 0;
        inputSet = 0;
        zero_input_count = 0;
        SET_DUTY_CYCLE_ALL(0);
    } else {
        signaltimeout++;
    }
}

void processDshot()
{
    if (dshotcommand > 0 && dshotcommand < 48) {
        commandcount++;
        if (commandcount >= commonCommands[dshotcommand].command_count) {
            commandcount = 0;
            switch (dshotcommand) {
            case CMD_STOP:
                armed = 0;
                newinput = 0;
                break;
            case CMD_DIRECTION1:
                forward = 1 - forward;
                break;
            case CMD_DIRECTION2:
                forward = 1 - forward;
                break;
            case CMD_SAVE_SETTINGS:
                saveEEpromSettings();
                break;
            case CMD_EXTENDED_TELEMETRY:
                dshot_extended_telemetry = 1;
                send_extended_dshot = 0b1111111111111111;
                break;
            }
        }
    } else {
        if (EDT_ARM_ENABLE == 1) {
            if (dshotcommand == 0) {
                EDT_ARMED = 0;
            }
            if ((dshotcommand > 3) && (EDT_ARMED == 0)) {
                EDT_ARMED = 1;
            }
        }
        commandcount = 0;
    }
}

void startMotor()
{
    if (running == 0 && armed) {
        commutation_interval = 10000;
        running = 1;
        SET_INTERVAL_TIMER_COUNT(commutation_interval);
        enableCompInterrupts();
    }
    if (input < 47) {
        running = 0;
        zero_crosses = 0;
        armed = 0;
        SET_DUTY_CYCLE_ALL(0);
    }
}

void setInput()
{
#if defined(FIXED_DUTY_MODE) || defined(FIXED_SPEED_MODE)
#ifdef FIXED_DUTY_MODE
    adjusted_input = ((FIXED_DUTY_MODE_POWER * 20) + 47);
#else
    adjusted_input = map(k_erpm, 0, 300, 48, 488);
    target_e_com_time = 60000000 / FIXED_SPEED_MODE_RPM / (eepromBuffer.motor_poles / 2);
    use_speed_control_loop = 1;
#endif
#else

    if ((input_override > 0) && (input_override < 2047)) {
        adjusted_input = map(input_override, 0, 2047, 47, 2047);
    } else {
        adjusted_input = newinput;
    }

    if (bemf_timeout_happened || !running || stepper_sine) {
        if (newinput > 1500 && eepromBuffer.bi_direction == 0) {
            adjusted_input = 2047;
        }
    }

    if (eepromBuffer.bi_direction == 1 && dshot == 0) {
#ifdef RC_CAR_REVERSE
        // RC car mode implementation
        if (return_to_center) {
            if (forward == 0 && adjusted_input > servo_neutral) {
                prop_brake_active = 0;
                return_to_center = 0;
            }
            if (forward == 1 && adjusted_input < servo_neutral) {
                prop_brake_active = 0;
                return_to_center = 0;
            }
            adjusted_input = 0;
        }
        
        if (prop_brake_active == 0 && eepromBuffer.rc_car_reverse) {
            if (forward == 1 && adjusted_input > (servo_neutral + servo_dead_band)) {
                adjusted_input = map(adjusted_input, servo_neutral + servo_dead_band, 2000, 47, 2047);
            } else if ((forward == 0 && adjusted_input < (servo_neutral - servo_dead_band))) {
                adjusted_input = map(adjusted_input, 1000, servo_neutral - servo_dead_band, 2047, 47);
            } else if (adjusted_input < (servo_neutral + servo_dead_band) && adjusted_input > (servo_neutral - servo_dead_band)) {
                adjusted_input = 0;
                if (running || zero_crosses > 100) {
                    prop_brake_duty_cycle = (eepromBuffer.drag_brake_strength * 40) + 3;
                    prop_brake_active = 1;
                } else {
                    prop_brake_duty_cycle = 0;
                }
            }
        }

        if (prop_brake_active && (forward == 1 && adjusted_input < (servo_neutral - servo_dead_band))) {
            if (zero_crosses > 100 && running) {
                prop_brake_duty_cycle = prop_brake_duty_cycle + map((servo_neutral - adjusted_input), 0, servo_neutral, 1, (eepromBuffer.driving_brake_strength * 10));
            } else {
                if (prop_brake_duty_cycle < (eepromBuffer.drag_brake_strength * 40) + 3) {
                    prop_brake_duty_cycle = (eepromBuffer.drag_brake_strength * 40) + 3;
                } else {
                    prop_brake_duty_cycle = prop_brake_duty_cycle - 5;
                }
            }
            if (prop_brake_duty_cycle > 1980) {
                prop_brake_duty_cycle = 1980;
            }
            adjusted_input = 0;
            proportionalBrake();
            if (prop_brake_duty_cycle < 800 && commutation_interval > 2000) {
                running = 0;
                zero_crosses = 0;
                maskPhaseInterrupts();
                SET_DUTY_CYCLE_ALL(0);
                forward = 0;
                prop_brake_duty_cycle = 0;
                prop_brake_active = 0;
                return_to_center = 1;
            }
        }

        if (prop_brake_active && (forward == 0 && adjusted_input > (servo_neutral + servo_dead_band))) {
            if (zero_crosses > 100 && running) {
                prop_brake_duty_cycle = prop_brake_duty_cycle + map((adjusted_input - servo_neutral), 0, servo_neutral, 1, (eepromBuffer.driving_brake_strength * 10));
            } else {
                if (prop_brake_duty_cycle < (eepromBuffer.drag_brake_strength * 40) + 3) {
                    prop_brake_duty_cycle = (eepromBuffer.drag_brake_strength * 40) + 3;
                } else {
                    prop_brake_duty_cycle = prop_brake_duty_cycle - 5;
                }
            }
            if (prop_brake_duty_cycle > 1980) {
                prop_brake_duty_cycle = 1980;
            }
            adjusted_input = 0;
            proportionalBrake();
            if (prop_brake_duty_cycle < 800 && commutation_interval > 2000) {
                running = 0;
                zero_crosses = 0;
                maskPhaseInterrupts();
                SET_DUTY_CYCLE_ALL(0);
                forward = 1;
                prop_brake_duty_cycle = 0;
                prop_brake_active = 0;
                return_to_center = 1;
            }
        }
        
        if (prop_brake_active == 0) {
            prop_brake_duty_cycle = ((prop_brake_duty_cycle * 15) >> 4) - 5;
        }
        // ... existing code ...
    } else {
        // Regular bi-directional mode
        if (eepromBuffer.bi_direction == 1 && dshot == 0) {
            if (adjusted_input <= servo_neutral) {
                forward = 1;
                adjusted_input = servo_neutral - adjusted_input;
                if (commutation_interval > reverse_speed_threshold && running) {
                    forward = 0;
                    zero_crosses = 0;
                    maskPhaseInterrupts();
                    running = 0;
                }
            } else {
                forward = 0;
                adjusted_input = adjusted_input - servo_neutral;
                if (commutation_interval > reverse_speed_threshold && running) {
                    forward = 1;
                    zero_crosses = 0;
                    maskPhaseInterrupts();
                    running = 0;
                }
            }
            if (adjusted_input > (servo_dead_band << 1)) {
                adjusted_input = map(adjusted_input, servo_dead_band << 1, servo_high_threshold - servo_neutral, 47, 2047);
            } else {
                adjusted_input = 0;
            }
        }
    }
#else // !RC_CAR_REVERSE
        // Regular bi-directional mode
        if (eepromBuffer.bi_direction == 1 && dshot == 0) {
            if (adjusted_input <= servo_neutral) {
                forward = 1;
                adjusted_input = servo_neutral - adjusted_input;
                if (commutation_interval > reverse_speed_threshold && running) {
                    forward = 0;
                    zero_crosses = 0;
                    maskPhaseInterrupts();
                    running = 0;
                }
            } else {
                forward = 0;
                adjusted_input = adjusted_input - servo_neutral;
                if (commutation_interval > reverse_speed_threshold && running) {
                    forward = 1;
                    zero_crosses = 0;
                    maskPhaseInterrupts();
                    running = 0;
                }
            }
            if (adjusted_input > (servo_dead_band << 1)) {
                adjusted_input = map(adjusted_input, servo_dead_band << 1, servo_high_threshold - servo_neutral, 47, 2047);
            } else {
                adjusted_input = 0;
            }
        }
#endif // RC_CAR_REVERSE
    }

    if (!eepromBuffer.bi_direction) {
        if (adjusted_input < 30) {
            bemf_timeout = 100;
        } else {
            bemf_timeout = 10;
        }
    }

    if (!inputSet) {
        if (input_type == DSHOT_IN && dshot == 0) {
            detectInput();
            servoPwm = 0;
        } else if (input_type == SERVO_IN && servoPwm == 0) {
            detectInput();
            dshot = 0;
        } else if (input_type == AUTO_IN) {
            detectInput();
        }
        if (dshot || servoPwm) {
            inputSet = 1;
        }
    }
#endif // FIXED_DUTY_MODE
}

void advanceincrement()
{
    if (forward == 1) {
        phase_A_position++;
        if (phase_A_position > 359) {
            phase_A_position = 0;
        }
        updateSinewavePhases();
    } else {
        phase_A_position--;
        if (phase_A_position < 0) {
            phase_A_position = 359;
        }
        updateSinewavePhases();
    }
}

void runBrushedLoop()
{
    if (eepromBuffer.rc_car_reverse) {
        // RC car brushed mode
        if (adjusted_input < (servo_neutral - servo_dead_band)) {
            adjusted_input = servo_neutral - adjusted_input;
            if (adjusted_input <= servo_dead_band) {
                adjusted_input = 0;
            }
            brushed_direction_set = 0;
        } else if (adjusted_input > (servo_neutral + servo_dead_band)) {
            adjusted_input = adjusted_input - servo_neutral;
            if (adjusted_input <= servo_dead_band) {
                adjusted_input = 0;
            }
            brushed_direction_set = 1;
        } else {
            adjusted_input = 0;
        }
    } else {
        // Regular brushed mode
        if (adjusted_input < 1000) {
            adjusted_input = (1000 - adjusted_input) << 1;
            brushed_direction_set = 0;
        } else {
            if (adjusted_input >= 1000) {
                adjusted_input = (adjusted_input - 1000) << 1;
                brushed_direction_set = 1;
            }
        }
    }

    if (running == 0 && adjusted_input > 48) {
        running = 1;
    }
    if (running && adjusted_input < 48) {
        running = 0;
    }

    if (brushed_direction_set) {
        phaseAPWM();
        phaseBFLOAT();
        phaseCLOW();
    } else {
        phaseAFLOAT();
        phaseBPWM();
        phaseCLOW();
    }
    if (adjusted_input > 2047) {
        adjusted_input = 2047;
    }
    
    duty_cycle_setpoint = map(adjusted_input, 48, 2047, minimum_duty_cycle, duty_cycle_maximum);
    limitDutyCycleChange();
    
    if (degrees_celsius > TEMP_LIMIT && degrees_celsius < 200) {
        duty_cycle = duty_cycle - map(degrees_celsius, TEMP_LIMIT, TEMP_LIMIT + 10, 1, duty_cycle_setpoint / 10);
    }
    updateCurrentLimit(actual_current);
    duty_cycle_setpoint = duty_cycle_setpoint - ((2000 - use_current_limit_adjust) / 2);
    
    if (duty_cycle > duty_cycle_setpoint) {
        duty_cycle = duty_cycle_setpoint;
    }
    
    adjusted_duty_cycle = (duty_cycle * tim1_arr) / 2000 + dead_time_override;
    SET_AUTO_RELOAD_PWM(TIMER1_MAX_ARR);
    SET_DUTY_CYCLE_ALL(adjusted_duty_cycle);
    e_com_time = commutation_interval;
    if (zero_crosses < 10000) {
        zero_crosses++;
    }
}

static void checkDeviceInfo(void)
{
    const struct devinfo {
        uint32_t magic1;
        uint32_t magic2;
        const uint8_t deviceInfo[9];
    } *devinfo = (struct devinfo *)(0x1000 - 32);
    const uint32_t MAGIC1 = 0xDEADBEEF;
    const uint32_t MAGIC2 = 0xCAFEBABE;
    if (devinfo->magic1 == MAGIC1 && devinfo->magic2 == MAGIC2) {
        char deviceInfo[10];
        memcpy(deviceInfo, devinfo->deviceInfo, 9);
        deviceInfo[9] = '\0';
        target_e_com_time = atoi(deviceInfo);
    } else {
        target_e_com_time = 0;
    }
}

int main(void)
{
    initAfterJump();
    checkDeviceInfo();
    initCorePeripherals();
    enableCorePeripherals();
    loadEEpromSettings();

    updateVersionIfNeeded();
    applyMotorModeSettings();
    
    tim1_arr = TIMER1_MAX_ARR;

    initializeHardwarePlatform();

#ifdef USE_LED_STRIP
    send_LED_RGB(125, 0, 0);
#endif

#ifdef USE_CRSF_INPUT
    inputSet = 1;
    playStartupTune();
    MX_IWDG_Init();
    LL_IWDG_ReloadCounter(IWDG);
#else
#if defined(FIXED_DUTY_MODE) || defined(FIXED_SPEED_MODE)
    MX_IWDG_Init();
    RELOAD_WATCHDOG_COUNTER();
    inputSet = 1;
    armed = 1;
    adjusted_input = 48;
    newinput = 48;
    comStep(2);
#ifdef FIXED_SPEED_MODE
    use_speed_control_loop = 1;
    eepromBuffer.use_sine_start = 0;
    target_e_com_time = 60000000 / FIXED_SPEED_MODE_RPM / (eepromBuffer.motor_poles / 2);
    input = 48;
#endif
#else
#ifdef BRUSHED_MODE
    commutation_interval = 5000;
    eepromBuffer.use_sine_start = 0;
    maskPhaseInterrupts();
    playBrushedStartupTune();
#else
#ifdef MCU_AT415
    play_tone_flag = 5;
#else
    playStartupTune();
#endif
#endif
    zero_input_count = 0;
    MX_IWDG_Init();
    RELOAD_WATCHDOG_COUNTER();
#ifdef GIMBAL_MODE
    eepromBuffer.bi_direction = 1;
    eepromBuffer.use_sine_start = 1;
#endif

    configureInputMode();

#endif // end fixed duty mode ifdef
#endif // end crsf input

#ifdef MCU_F051
    MCU_Id = DBGMCU->IDCODE &= 0xFFF;
    REV_Id = DBGMCU->IDCODE >> 16;

    if (REV_Id >= 4096) {
        temperature_offset = 0;
    } else {
        temperature_offset = 230;
    }
#endif

#ifdef NEUTRONRC_G071
    setInputPullDown();
#else
    setInputPullUp();
#endif

    applyStartupBoost();

    while (1) {
        e_com_time = ((commutation_intervals[0] + commutation_intervals[1] + commutation_intervals[2] + commutation_intervals[3] + commutation_intervals[4] + commutation_intervals[5]) + 4) >> 1;
        
#if defined(FIXED_DUTY_MODE) || defined(FIXED_SPEED_MODE)
        setInput();
#endif

#ifdef NEED_INPUT_READY
#ifdef MCU_F031
        if (input_ready) {
            setInput();
            input_ready = 0;
        }
#else
        if (input_ready) {
            processDshot();
            input_ready = 0;
        }
#endif
#endif

        if (zero_crosses < 5) {
            min_bemf_counts_up = TARGET_MIN_BEMF_COUNTS * 2;
            min_bemf_counts_down = TARGET_MIN_BEMF_COUNTS * 2;
        } else {
            min_bemf_counts_up = TARGET_MIN_BEMF_COUNTS;
            min_bemf_counts_down = TARGET_MIN_BEMF_COUNTS;
        }
        
        RELOAD_WATCHDOG_COUNTER();

        updateVariablePWMFrequency();

        if (signaltimeout > (LOOP_FREQUENCY_HZ >> 1)) { // half second timeout when armed
            if (armed) {
                allOff();
                armed = 0;
                input = 0;
                inputSet = 0;
                zero_input_count = 0;
                SET_DUTY_CYCLE_ALL(0);
                resetInputCaptureTimer();
                for (int i = 0; i < 64; i++) {
                    dma_buffer[i] = 0;
                }
                NVIC_SystemReset();
            }
            if (signaltimeout > LOOP_FREQUENCY_HZ << 1) { // 2 second when not armed
                allOff();
                armed = 0;
                input = 0;
                inputSet = 0;
                zero_input_count = 0;
                SET_DUTY_CYCLE_ALL(0);
                resetInputCaptureTimer();
                for (int i = 0; i < 64; i++) {
                    dma_buffer[i] = 0;
                }
                NVIC_SystemReset();
            }
        }

#ifdef USE_CUSTOM_LED
        if ((input >= 47) && (input < 1947)) {
            if (ledcounter > (2000 >> forward)) {
                GPIOB->BSRR = LL_GPIO_PIN_3;
            } else {
                GPIOB->BRR = LL_GPIO_PIN_3;
            }
            if (ledcounter > (4000 >> forward)) {
                ledcounter = 0;
            }
        }
        if (input > 1947) {
            GPIOB->BSRR = LL_GPIO_PIN_3;
        }
        if (input < 47) {
            GPIOB->BRR = LL_GPIO_PIN_3;
        }
#endif

        if (tenkhzcounter > LOOP_FREQUENCY_HZ) { // 1s sample interval
            updateTelemetryData();
            tenkhzcounter = 0;
        }

#ifndef BRUSHED_MODE
        if ((zero_crosses > 1000) || (adjusted_input == 0)) {
            bemf_timeout_happened = 0;
        }
        if (zero_crosses > 100 && adjusted_input < 200) {
            bemf_timeout_happened = 0;
        }
        if (eepromBuffer.use_sine_start && adjusted_input < 160) {
            bemf_timeout_happened = 0;
        }

        if (crawler_mode) {
            if (adjusted_input < 400) {
                bemf_timeout_happened = 0;
            }
        } else {
            if (adjusted_input < 150) {
                bemf_timeout = 100;
            } else {
                bemf_timeout = 10;
            }
        }
#endif
        average_interval = e_com_time / 3;
        
        if (isMotorDesync()) {
            zero_crosses = 0;
            desync_happened++;
            if ((!eepromBuffer.bi_direction && (input > 47)) || commutation_interval > 1000) {
                running = 0;
            }
            old_routine = 1;
            if (zero_crosses > 100) {
                average_interval = 5000;
            }
            last_duty_cycle = min_startup_duty / 2;
        }
        desync_check = 0;
        last_average_interval = average_interval;

#if !defined(MCU_G031) && !defined(NEED_INPUT_READY)
        if (dshot_telemetry && (commutation_interval > DSHOT_PRIORITY_THRESHOLD)) {
            NVIC_SetPriority(IC_DMA_IRQ_NAME, 0);
            NVIC_SetPriority(COM_TIMER_IRQ, 1);
            NVIC_SetPriority(COMPARATOR_IRQ, 1);
        } else {
            NVIC_SetPriority(IC_DMA_IRQ_NAME, 1);
            NVIC_SetPriority(COM_TIMER_IRQ, 0);
            NVIC_SetPriority(COMPARATOR_IRQ, 0);
        }
#endif

        if (send_telemetry) {
#ifdef USE_SERIAL_TELEMETRY
            makeTelemPackage((int8_t)degrees_celsius, battery_voltage, actual_current,
                (uint16_t)(consumed_current >> 16), e_rpm);
            send_telem_DMA(10);
            send_telemetry = 0;
#endif
        } else if (send_esc_info_flag) {
            makeInfoPacket();
            send_telem_DMA(49);
            send_esc_info_flag = 0;
        }
        
        if (PROCESS_ADC_FLAG == 1) {
            processADCReadings();
            checkLowVoltage();
            PROCESS_ADC_FLAG = 0;
#ifdef USE_ADC_INPUT
            if (ADC_raw_input < 10) {
                zero_input_count++;
            } else {
                zero_input_count = 0;
            }
#endif
        }
        
#ifdef USE_ADC_INPUT
        signaltimeout = 0;
        ADC_smoothed_input = (((10 * ADC_smoothed_input) + ADC_raw_input) / 11);
        newinput = ADC_smoothed_input / 2;
        if (newinput > 2000) {
            newinput = 2000;
        }
#endif
        
        stuckcounter = 0;
        if (stepper_sine == 0) {
            e_rpm = running * (600000 / e_com_time);
            k_erpm = e_rpm / 10;

            updateDutyCycleMaximum(k_erpm);

            if (zero_crosses < 100 && commutation_interval > 500) {
                filter_level = 12;
            } else {
                filter_level = map(average_interval, 100, 500, 3, 12);
            }
            if (commutation_interval < 50) {
                filter_level = 2;
            }

            updateAutoAdvance();

#ifdef CUSTOM_RAMP
            if (old_routine && running) {
                maskPhaseInterrupts();
                getBemfState();
                if (!zcfound) {
                    if (rising) {
                        if (bemfcounter > min_bemf_counts_up) {
                            zcfound = 1;
                            zcfoundroutine();
                        }
                    } else {
                        if (bemfcounter > min_bemf_counts_down) {
                            zcfound = 1;
                            zcfoundroutine();
                        }
                    }
                }
            }
#endif
            if (INTERVAL_TIMER_COUNT > 45000 && running == 1) {
                bemf_timeout_happened++;
                maskPhaseInterrupts();
                old_routine = 1;
                if (input < 48) {
                    running = 0;
                    commutation_interval = 5000;
                }
                zero_crosses = 0;
                zcfoundroutine();
            }
        } else { // stepper sine mode
#ifdef GIMBAL_MODE
            step_delay = 300;
            maskPhaseInterrupts();
            allpwm();
            if (newinput > 1000) {
                desired_angle = map(newinput, 1000, 2000, 180, 360);
            } else {
                desired_angle = map(newinput, 0, 1000, 0, 180);
            }
            if (current_angle > desired_angle) {
                forward = 1;
                advanceincrement();
                delayMicros(step_delay);
                current_angle--;
            }
            if (current_angle < desired_angle) {
                forward = 0;
                advanceincrement();
                delayMicros(step_delay);
                current_angle++;
            }
#else
            if (input > 48 && armed) {
                if (input > 48 && input < 137) { // sine wave stepper
                    if (do_once_sinemode) {
                        DISABLE_COM_TIMER_INT();
                        maskPhaseInterrupts();
                        SET_DUTY_CYCLE_ALL(0);
                        allpwm();
                        do_once_sinemode = 0;
                    }
                    advanceincrement();
                    step_delay = map(input, 48, 120, 7000 / eepromBuffer.motor_poles, 810 / eepromBuffer.motor_poles);
                    delayMicros(step_delay);
                    e_rpm = 600 / step_delay;
                } else {
                    do_once_sinemode = 1;
                    advanceincrement();
                    if (input > 200) {
                        phase_A_position = 0;
                        step_delay = 80;
                    }

                    delayMicros(step_delay);
                    if (phase_A_position == 0) {
                        stepper_sine = 0;
                        running = 1;
                        old_routine = 1;
                        commutation_interval = 9000;
                        average_interval = 9000;
                        last_average_interval = average_interval;
                        SET_INTERVAL_TIMER_COUNT(9000);
                        zero_crosses = 20;
                        prop_brake_active = 0;
                        step = changeover_step;
                        if (eepromBuffer.stall_protection) {
                            last_duty_cycle = stall_protect_minimum_duty;
                        }
                        commutate();
                        generatePwmTimerEvent();
                    }
                }
            } else {
                do_once_sinemode = 1;
                applyBrakeOnStop();
                e_rpm = 0;
            }
#endif // gimbal mode
        } // stepper/sine mode end

#ifdef BRUSHED_MODE
        runBrushedLoop();
#endif
#if DRONECAN_SUPPORT
        DroneCAN_update();
#endif
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line
       number, tex: printf("Wrong parameters value: file %s on line %d\r\n", file,
       line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
