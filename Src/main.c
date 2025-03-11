
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
#include "signal.h"
#include "sounds.h"
#include "targets.h"
#include <stdint.h>
#include <string.h>
#include <assert.h>

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
char eeprom_layout_version = EEPROM_VERSION;
uint8_t telemetry_interval_ms = 30;
char temp_advance = 1;
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
uint8_t adc_counter = 0;
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
const uint8_t numReadings = 100; // the readings from the analog input
uint8_t readIndex = 0; // the index of the current reading
uint32_t total = 0;
uint16_t readings[100];

uint8_t bemf_timeout_happened = 0;
uint8_t changeover_step = 5;
uint8_t filter_level = 5;
uint8_t running = 0;
uint16_t advance = 0;
uint8_t advancedivisor = 6;
char rising = 1;

////Space Vector PWM ////////////////
// const int pwmSin[] ={128, 132, 136, 140, 143, 147, 151, 155, 159, 162, 166,
// 170, 174, 178, 181, 185, 189, 192, 196, 200, 203, 207, 211, 214, 218, 221,
// 225, 228, 232, 235, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248,
// 248, 249, 250, 250, 251, 252, 252, 253, 253, 253, 254, 254, 254, 255, 255,
// 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 254, 254, 253,
// 253, 253, 252, 252, 251, 250, 250, 249, 248, 248, 247, 246, 245, 244, 243,
// 242, 241, 240, 239, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248,
// 248, 249, 250, 250, 251, 252, 252, 253, 253, 253, 254, 254, 254, 255, 255,
// 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 254, 254, 253,
// 253, 253, 252, 252, 251, 250, 250, 249, 248, 248, 247, 246, 245, 244, 243,
// 242, 241, 240, 239, 238, 235, 232, 228, 225, 221, 218, 214, 211, 207, 203,
// 200, 196, 192, 189, 185, 181, 178, 174, 170, 166, 162, 159, 155, 151, 147,
// 143, 140, 136, 132, 128, 124, 120, 116, 113, 109, 105, 101, 97, 94, 90, 86,
// 82, 78, 75, 71, 67, 64, 60, 56, 53, 49, 45, 42, 38, 35, 31, 28, 24, 21, 18,
// 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 8, 7, 6, 6, 5, 4, 4, 3, 3, 3, 2, 2, 2,
// 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 6, 6, 7, 8,
// 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9,
// 8, 8, 7, 6, 6, 5, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
// 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 6, 6, 7, 8, 8, 9, 10, 11, 12, 13, 14, 15, 16,
// 17, 18, 21, 24, 28, 31, 35, 38, 42, 45, 49, 53, 56, 60, 64, 67, 71, 75, 78,
// 82, 86, 90, 94, 97, 101, 105, 109, 113, 116, 120, 124};

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


int32_t doPidCalculations(struct fastPID* pidnow, int actual, int target)
{

    pidnow->error = actual - target;
    pidnow->integral = pidnow->integral + pidnow->error * pidnow->Ki;
    if (pidnow->integral > pidnow->integral_limit) {
        pidnow->integral = pidnow->integral_limit;
    }
    if (pidnow->integral < -pidnow->integral_limit) {
        pidnow->integral = -pidnow->integral_limit;
    }

    pidnow->derivative = pidnow->Kd * (pidnow->error - pidnow->last_error);
    pidnow->last_error = pidnow->error;

    pidnow->pid_output = pidnow->error * pidnow->Kp + pidnow->integral + pidnow->derivative;

    if (pidnow->pid_output > pidnow->output_limit) {
        pidnow->pid_output = pidnow->output_limit;
    }
    if (pidnow->pid_output < -pidnow->output_limit) {
        pidnow->pid_output = -pidnow->output_limit;
    }
    return pidnow->pid_output;
}

void loadEEpromSettings()
{
    read_flash_bin(eepromBuffer.buffer, eeprom_address, sizeof(eepromBuffer.buffer));

    if (eepromBuffer.advance_level > 3) {
        eepromBuffer.advance_level = 2;
    }

    if (eepromBuffer.pwm_frequency < 49 && eepromBuffer.pwm_frequency > 7) {
        if (eepromBuffer.pwm_frequency < 49 && eepromBuffer.pwm_frequency > 23) {
            TIMER1_MAX_ARR = map(eepromBuffer.pwm_frequency, 24, 48, TIM1_AUTORELOAD, TIM1_AUTORELOAD / 2);
        }
        if (eepromBuffer.pwm_frequency < 24 && eepromBuffer.pwm_frequency > 11) {
            TIMER1_MAX_ARR = map(eepromBuffer.pwm_frequency, 12, 24, TIM1_AUTORELOAD * 2, TIM1_AUTORELOAD);
        }
        if (eepromBuffer.pwm_frequency < 12 && eepromBuffer.pwm_frequency > 7) {
            TIMER1_MAX_ARR = map(eepromBuffer.pwm_frequency, 7, 16, TIM1_AUTORELOAD * 3,
                TIM1_AUTORELOAD / 2 * 3);
        }
        SET_AUTO_RELOAD_PWM(TIMER1_MAX_ARR);
    } else {
        tim1_arr = TIM1_AUTORELOAD;
        SET_AUTO_RELOAD_PWM(tim1_arr);
    }

    if (eepromBuffer.startup_power < 151 && eepromBuffer.startup_power > 49) {
        if(!eepromBuffer.comp_pwm){        // higher startup power for non-complementary pwm
            min_startup_duty = (eepromBuffer.startup_power) *2 ;
            minimum_duty_cycle = (eepromBuffer.startup_power / 2);
            stall_protect_minimum_duty = minimum_duty_cycle + 10;
			}else{
            min_startup_duty = (eepromBuffer.startup_power);
            minimum_duty_cycle = (eepromBuffer.startup_power / 3);
            stall_protect_minimum_duty = minimum_duty_cycle + 10;
			}
    } else {
        min_startup_duty = 150;
        minimum_duty_cycle = (min_startup_duty / 2) + 10;
    }
    motor_kv = (eepromBuffer.motor_kv * 40) + 20;
#ifdef THREE_CELL_MAX
		motor_kv =  motor_kv / 2;
#endif
    setVolume(2);
    if (eepromBuffer.eeprom_version > 0) { // these commands weren't introduced until eeprom version 1.
#ifdef CUSTOM_RAMP

#else
        if (eepromBuffer.beep_volume > 11) {
            setVolume(5);
        } else {
            setVolume(eepromBuffer.beep_volume);
        }
#endif
        servo_low_threshold = (eepromBuffer.servo.low_threshold * 2) + 750; // anything below this point considered 0
        servo_high_threshold = (eepromBuffer.servo.high_threshold * 2) + 1750;
        ; // anything above this point considered 2000 (max)
        servo_neutral = (eepromBuffer.servo.neutral) + 1374;
        servo_dead_band = eepromBuffer.servo.dead_band;

        if (eepromBuffer.low_voltage_cut_off == 0x01) {
            LOW_VOLTAGE_CUTOFF = 1;
        } else {
            LOW_VOLTAGE_CUTOFF = 0;
        }

        low_cell_volt_cutoff = eepromBuffer.low_cell_volt_cutoff + 250; // 2.5 to 3.5 volts per cell range

#ifndef HAS_HALL_SENSORS
        eepromBuffer.use_hall_sensors = 0;
#endif

        if (eepromBuffer.sine_mode_changeover_thottle_level < 5 || eepromBuffer.sine_mode_changeover_thottle_level > 25) { // sine mode changeover 5-25 percent throttle
            eepromBuffer.sine_mode_changeover_thottle_level = 5;
        }
        if (eepromBuffer.drag_brake_strength == 0 || eepromBuffer.drag_brake_strength > 10) { // drag brake 1-10
            eepromBuffer.drag_brake_strength = 10;
        }

        if (eepromBuffer.driving_brake_strength == 0 || eepromBuffer.driving_brake_strength > 9) { // motor brake 1-9
            eepromBuffer.driving_brake_strength = 10;
        }

        if(eepromBuffer.driving_brake_strength < 10){
            dead_time_override = DEAD_TIME + (150 - (eepromBuffer.driving_brake_strength * 10));
            if (dead_time_override > 200) {
                dead_time_override = 200;
            }
        min_startup_duty = min_startup_duty + dead_time_override;
        minimum_duty_cycle = minimum_duty_cycle + dead_time_override;
        throttle_max_at_low_rpm = throttle_max_at_low_rpm + dead_time_override;
        startup_max_duty_cycle = startup_max_duty_cycle + dead_time_override;
#ifdef STMICRO
        TIM1->BDTR |= dead_time_override;
#endif
#ifdef ARTERY
        TMR1->brk |= dead_time_override;
#endif
#ifdef GIGADEVICES
        TIMER_CCHP(TIMER0) |= dead_time_override;
#endif
        }
        if (eepromBuffer.limits.temperature < 70 || eepromBuffer.limits.temperature > 140) {
            eepromBuffer.limits.temperature = 255;
        }

        if (eepromBuffer.limits.current > 0 && eepromBuffer.limits.current < 100) {
            use_current_limit = 1;
        }
        
        if (eepromBuffer.sine_mode_power == 0 || eepromBuffer.sine_mode_power > 10) {
            eepromBuffer.sine_mode_power = 5;
        }

        // unsinged int cant be less than 0
        if (eepromBuffer.input_type < 10) {
            switch (eepromBuffer.input_type) {
            case AUTO_IN:
                dshot = 0;
                servoPwm = 0;
                EDT_ARMED = 1;
                break;
            case DSHOT_IN:
                dshot = 1;
                EDT_ARMED = 1;
                break;
            case SERVO_IN:
                servoPwm = 1;
                break;
            case SERIAL_IN:
                break;
            case EDTARM_IN:
                EDT_ARM_ENABLE = 1;
                EDT_ARMED = 0;
                dshot = 1;
                break;
            };
        } else {
            dshot = 0;
            servoPwm = 0;
            EDT_ARMED = 1;
        }
        if (motor_kv < 300) {
            low_rpm_throttle_limit = 0;
        }
        low_rpm_level = motor_kv / 100 / (32 / eepromBuffer.motor_poles);

        high_rpm_level = motor_kv / 12 / (32 / eepromBuffer.motor_poles);				
    }
    reverse_speed_threshold = map(motor_kv, 300, 3000, 1000, 500);
}

void saveEEpromSettings()
{

    eepromBuffer.eeprom_version = eeprom_layout_version;

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
#ifdef MCU_F031
    if (step == 1 || step == 4) {
        current_state = PHASE_C_EXTI_PORT->IDR & PHASE_C_EXTI_PIN;
    }
    if (step == 2 || step == 5) { //        in phase two or 5 read from phase A Pf1
        current_state = PHASE_A_EXTI_PORT->IDR & PHASE_A_EXTI_PIN;
    }
    if (step == 3 || step == 6) { // phase B pf0
        current_state = PHASE_B_EXTI_PORT->IDR & PHASE_B_EXTI_PIN;
    }
#else
    current_state = !getCompOutputLevel(); // polarity reversed
#endif
    if (rising) {
        if (current_state) {
            bemfcounter++;
        } else {
            bad_count++;
            if (bad_count > bad_count_threshold) {
                bemfcounter = 0;
            }
        }
    } else {
        if (!current_state) {
            bemfcounter++;
        } else {
            bad_count++;
            if (bad_count > bad_count_threshold) {
                bemfcounter = 0;
            }
        }
    }
}

void commutate()
{
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
    __disable_irq(); // don't let dshot interrupt
    if (!prop_brake_active) {
        comStep(step);
    }
    __enable_irq();
    changeCompInput();
#ifndef NO_POLLING_START
	if (average_interval > 2500) {
      old_routine = 1;
   }
#endif
    bemfcounter = 0;
    zcfound = 0;
    commutation_intervals[step - 1] = commutation_interval; // just used to calulate average
    e_com_time = ((commutation_intervals[0] + commutation_intervals[1] + commutation_intervals[2] + commutation_intervals[3] + commutation_intervals[4] + commutation_intervals[5]) + 4) >> 1; // COMMUTATION INTERVAL IS 0.5US INCREMENTS
#ifdef USE_PULSE_OUT
		if(rising){
			GPIOB->scr = GPIO_PINS_8;
		}else{
			GPIOB->clr = GPIO_PINS_8;
		}
#endif
}

void PeriodElapsedCallback()
{
    DISABLE_COM_TIMER_INT(); // disable interrupt
    commutate();
    commutation_interval = ((commutation_interval)+((lastzctime + thiszctime) >> 1))>>1;
  	if (!eepromBuffer.auto_advance) {
	  advance = (commutation_interval >> 3) * temp_advance; // 60 divde 8 7.5 degree increments
	} else {
	  advance = (commutation_interval * auto_advance_level) >> 6; // 60 divde 64 0.9375 degree increments
    }
    waitTime = (commutation_interval >> 1) - advance;
    if (!old_routine) {
        enableCompInterrupts(); // enable comp interrupt
    }
    if (zero_crosses < 10000) {
        zero_crosses++;
    }
}

void interruptRoutine()
{
   if (average_interval > 125) {
        if ((INTERVAL_TIMER_COUNT < 125) && (duty_cycle < 600) && (zero_crosses < 500)) { // should be impossible, desync?exit anyway
           return;
        }
        stuckcounter++; // stuck at 100 interrupts before the main loop happens
                        // again.
        if (stuckcounter > 100) {
            maskPhaseInterrupts();
            zero_crosses = 0;
            return;
        }
    }
        for (int i = 0; i < filter_level; i++) {
#ifdef MCU_F031
            if (((current_GPIO_PORT->IDR & current_GPIO_PIN) == !(rising))) {
#else
            if (getCompOutputLevel() == rising) {
#endif
                return;
            }
        }
    __disable_irq();
    maskPhaseInterrupts();
    lastzctime = thiszctime;
    thiszctime = INTERVAL_TIMER_COUNT;  
    SET_INTERVAL_TIMER_COUNT(0);
    SET_AND_ENABLE_COM_INT(waitTime+1); // enable COM_TIMER interrupt
    __enable_irq();
}

void startMotor()
{
    if (running == 0) {
        commutate();
        commutation_interval = 10000;
        SET_INTERVAL_TIMER_COUNT(5000);
        running = 1;
    }
    enableCompInterrupts();
}

void setInput()
{

    if (eepromBuffer.bi_direction) {
        if (dshot == 0) {
            if (eepromBuffer.rc_car_reverse) {
                if (newinput > (1000 + (servo_dead_band << 1))) {
                    if (forward == eepromBuffer.dir_reversed) {
                        adjusted_input = 0;
                        //               if (running) {
                        prop_brake_active = 1;
                        if (return_to_center) {
                            forward = 1 - eepromBuffer.dir_reversed;
                            prop_brake_active = 0;
                            return_to_center = 0;
                        }
                    }
                    if (prop_brake_active == 0) {
                        return_to_center = 0;
                        adjusted_input = map(newinput, 1000 + (servo_dead_band << 1), 2000, 47, 2047);
                    }
                }
                if (newinput < (1000 - (servo_dead_band << 1))) {
                    if (forward == (1 - eepromBuffer.dir_reversed)) {
                        adjusted_input = 0;
                        prop_brake_active = 1;
                        if (return_to_center) {
                            forward = eepromBuffer.dir_reversed;
                            prop_brake_active = 0;
                            return_to_center = 0;
                        }
                    }
                    if (prop_brake_active == 0) {
                        return_to_center = 0;
                        adjusted_input = map(newinput, 0, 1000 - (servo_dead_band << 1), 2047, 47);
                    }
                }
                if (newinput >= (1000 - (servo_dead_band << 1)) && newinput <= (1000 + (servo_dead_band << 1))) {
                    adjusted_input = 0;
                    if (prop_brake_active) {
                        prop_brake_active = 0;
                        return_to_center = 1;
                    }
                }
            } else {
                if (newinput > (1000 + (servo_dead_band << 1))) {
                    if (forward == eepromBuffer.dir_reversed) {
                        if (((commutation_interval > reverse_speed_threshold) && (duty_cycle < 200)) || stepper_sine) {
                            forward = 1 - eepromBuffer.dir_reversed;
                            zero_crosses = 0;
                            old_routine = 1;
                            maskPhaseInterrupts();
                            brushed_direction_set = 0;
                        } else {
                            newinput = 1000;
                        }
                    }
                    adjusted_input = map(newinput, 1000 + (servo_dead_band << 1), 2000, 47, 2047);
                }
                if (newinput < (1000 - (servo_dead_band << 1))) {
                    if (forward == (1 - eepromBuffer.dir_reversed)) {
                        if (((commutation_interval > reverse_speed_threshold) && (duty_cycle < 200)) || stepper_sine) {
                            zero_crosses = 0;
                            old_routine = 1;
                            forward = eepromBuffer.dir_reversed;
                            maskPhaseInterrupts();
                            brushed_direction_set = 0;
                        } else {
                            newinput = 1000;
                        }
                    }
                    adjusted_input = map(newinput, 0, 1000 - (servo_dead_band << 1), 2047, 47);
                }

                if (newinput >= (1000 - (servo_dead_band << 1)) && newinput <= (1000 + (servo_dead_band << 1))) {
                    adjusted_input = 0;
                    brushed_direction_set = 0;
                }
            }
        }

        if (dshot) {
            if (newinput > 1047) {

                if (forward == eepromBuffer.dir_reversed) {
                    if (((commutation_interval > reverse_speed_threshold) && (duty_cycle < 200)) || stepper_sine) {
                        forward = 1 - eepromBuffer.dir_reversed;
                        zero_crosses = 0;
                        old_routine = 1;
                        maskPhaseInterrupts();
                        brushed_direction_set = 0;
                    } else {
                        newinput = 0;
                    }
                }
                adjusted_input = ((newinput - 1048) * 2 + 47) - reversing_dead_band;
            }
            if (newinput <= 1047 && newinput > 47) {
                if (forward == (1 - eepromBuffer.dir_reversed)) {
                    if (((commutation_interval > reverse_speed_threshold) && (duty_cycle < 200)) || stepper_sine) {
                        zero_crosses = 0;
                        old_routine = 1;
                        forward = eepromBuffer.dir_reversed;
                        maskPhaseInterrupts();
                        brushed_direction_set = 0;
                    } else {
                        newinput = 0;
                    }
                }
                adjusted_input = ((newinput - 48) * 2 + 47) - reversing_dead_band;
            }
            if (newinput < 48) {
                adjusted_input = 0;
                brushed_direction_set = 0;
            }
        }
    } else {
        adjusted_input = newinput;
    }
#ifndef BRUSHED_MODE
    if ((bemf_timeout_happened > bemf_timeout) && eepromBuffer.stuck_rotor_protection) {
        allOff();
        maskPhaseInterrupts();
        input = 0;
        bemf_timeout_happened = 102;
#ifdef USE_RGB_LED
        GPIOB->BRR = LL_GPIO_PIN_8; // on red
        GPIOB->BSRR = LL_GPIO_PIN_5; //
        GPIOB->BSRR = LL_GPIO_PIN_3;
#endif
    } else {
#ifdef FIXED_DUTY_MODE
        input = FIXED_DUTY_MODE_POWER * 20 + 47;
#else
        if (eepromBuffer.use_sine_start) {
            if (adjusted_input < 30) { // dead band ?
                input = 0;
            }
            if (adjusted_input > 30 && adjusted_input < (eepromBuffer.sine_mode_changeover_thottle_level * 20)) {
                input = map(adjusted_input, 30,
                    (eepromBuffer.sine_mode_changeover_thottle_level * 20), 47, 160);
            }
            if (adjusted_input >= (eepromBuffer.sine_mode_changeover_thottle_level * 20)) {
                input = map(adjusted_input, (eepromBuffer.sine_mode_changeover_thottle_level * 20),
                    2047, 160, 2047);
            }
        } else {
            if (use_speed_control_loop) {
                if (drive_by_rpm) {
                    target_e_com_time = 60000000 / map(adjusted_input, 47, 2047, MINIMUM_RPM_SPEED_CONTROL, MAXIMUM_RPM_SPEED_CONTROL) / (eepromBuffer.motor_poles / 2);
                    if (adjusted_input < 47) { // dead band ?
                        input = 0;
                        speedPid.error = 0;
                        input_override = 0;
                    } else {
                        input = (uint16_t)(input_override / 10000); // speed control pid override
                        if (input > 2047) {
                            input = 2047;
                        }
                        if (input < 48) {
                            input = 48;
                        }
                    }
                } else {

                    input = (uint16_t)(input_override / 10000); // speed control pid override
                    if (input > 2047) {
                        input = 2047;
                    }
                    if (input < 48) {
                        input = 48;
                    }
                }
            } else {

                input = adjusted_input;
            }
        }
#endif
    }
#endif
#ifndef BRUSHED_MODE
if (!stepper_sine && armed) {
        if (input >= 47 + (80 * eepromBuffer.use_sine_start)) {
            if (running == 0) {
                allOff();
                if (!old_routine) {
                    startMotor();
                }
                running = 1;
                last_duty_cycle = min_startup_duty;
            }

            if (eepromBuffer.use_sine_start) {
                duty_cycle_setpoint = map(input, 137, 2047, minimum_duty_cycle+40, 2000);
            } else {
                duty_cycle_setpoint = map(input, 47, 2047, minimum_duty_cycle, 2000);
            }

            if (!eepromBuffer.rc_car_reverse) {
                prop_brake_active = 0;
            }
        }

        if (input < 47 + (80 * eepromBuffer.use_sine_start)) {
            if (play_tone_flag != 0) {
                switch (play_tone_flag) {
									
                case 1:
                    playDefaultTone();
                    break;
                case 2:
                    playChangedTone();
                    break;
                case 3:
                    playBeaconTune3();
                    break;
                case 4:
                    playInputTune2();
                    break;
                case 5:
                    playDefaultTone();
                    break;
                }
                play_tone_flag = 0;
            }

            if (!eepromBuffer.comp_pwm) {
                duty_cycle_setpoint = 0;
                if (!running) {
                    old_routine = 1;
                    zero_crosses = 0;
                    if (eepromBuffer.brake_on_stop) {
                        fullBrake();
                    } else {
                        if (!prop_brake_active) {
                            allOff();
                        }
                    }
                }
                if (eepromBuffer.rc_car_reverse && prop_brake_active) {
#ifndef PWM_ENABLE_BRIDGE
                    prop_brake_duty_cycle = (getAbsDif(1000, newinput) + 1000);
                    if (prop_brake_duty_cycle >= (TIMER1_MAX_ARR - 1)) {
                        fullBrake();
                    } else {
                        proportionalBrake();
                    }
#endif
                }
            } else {
                if (!running) {

                    old_routine = 1;
                    zero_crosses = 0;
                    bad_count = 0;
                    if (eepromBuffer.brake_on_stop) {
                        if (!eepromBuffer.use_sine_start) {
#ifndef PWM_ENABLE_BRIDGE
                            prop_brake_duty_cycle = (1980) + eepromBuffer.drag_brake_strength * 2;
                            proportionalBrake();
                            prop_brake_active = 1;
#else
                            // todo add proportional braking for pwm/enable style bridge.
#endif
                        }
                    } else {
                        allOff();
                    }
                    duty_cycle_setpoint = 0;
                }

                phase_A_position = ((step - 1) * 60) + enter_sine_angle;
                if (phase_A_position > 359) {
                    phase_A_position -= 360;
                }
                phase_B_position = phase_A_position + 119;
                if (phase_B_position > 359) {
                    phase_B_position -= 360;
                }
                phase_C_position = phase_A_position + 239;
                if (phase_C_position > 359) {
                    phase_C_position -= 360;
                }

                if (eepromBuffer.use_sine_start == 1) {
                    stepper_sine = 1;
                }
                duty_cycle_setpoint = 0;
            }
        }
        if (!prop_brake_active) {
            if (input >= 47 && (zero_crosses < (uint32_t)(30 >> eepromBuffer.stall_protection))) {
                if (duty_cycle_setpoint < min_startup_duty) {
                    duty_cycle_setpoint = min_startup_duty;
                }
                if (duty_cycle_setpoint > startup_max_duty_cycle) {
                    duty_cycle_setpoint = startup_max_duty_cycle;
                }
            }

            if (duty_cycle_setpoint > duty_cycle_maximum) {
                duty_cycle_setpoint = duty_cycle_maximum;
            }
            if (use_current_limit) {
                if (duty_cycle_setpoint > use_current_limit_adjust) {
                    duty_cycle_setpoint = use_current_limit_adjust;
                }
            }

            if (stall_protection_adjust > 0 && input > 47) {

                duty_cycle_setpoint = duty_cycle_setpoint + (uint16_t)(stall_protection_adjust/10000);
            }
        }
    }
#endif
}

void tenKhzRoutine()
{ // 20khz as of 2.00 to be renamed
    duty_cycle = duty_cycle_setpoint;
    tenkhzcounter++;
    ledcounter++;
    one_khz_loop_counter++;
    if (!armed) {
        if (cell_count == 0) {
            if (inputSet) {
                if (adjusted_input == 0) {
                    armed_timeout_count++;
                    if (armed_timeout_count > LOOP_FREQUENCY_HZ) { // one second
                        if (zero_input_count > 30) {
                            armed = 1;
#ifdef USE_LED_STRIP
                            //	send_LED_RGB(0,0,0);
                            delayMicros(1000);
                            send_LED_RGB(0, 255, 0);
#endif
#ifdef USE_RGB_LED
                            GPIOB->BRR = LL_GPIO_PIN_3; // turn on green
                            GPIOB->BSRR = LL_GPIO_PIN_8; // turn on green
                            GPIOB->BSRR = LL_GPIO_PIN_5;
#endif
                            if ((cell_count == 0) && LOW_VOLTAGE_CUTOFF) {
                                cell_count = battery_voltage / 370;
                                for (int i = 0; i < cell_count; i++) {
                                    playInputTune();
                                    delayMillis(100);
                                    RELOAD_WATCHDOG_COUNTER();
                                }
                            } else {
#ifdef MCU_AT415
															play_tone_flag = 4;
#else
															playInputTune();
#endif
                            }
                            if (!servoPwm) {
                                eepromBuffer.rc_car_reverse = 0;
                            }
                        } else {
                            inputSet = 0;
                            armed_timeout_count = 0;
                        }
                    }
                } else {
                    armed_timeout_count = 0;
                }
            }
        }
    }

    if (eepromBuffer.telementry_on_interval) {
        telem_ms_count++;
        if (telem_ms_count > telemetry_interval_ms * 20) {
            send_telemetry = 1;
            telem_ms_count = 0;
        }
    }

#ifndef BRUSHED_MODE

    if (!stepper_sine) {
#ifndef CUSTOM_RAMP
        if (old_routine && running) {
	//				send_LED_RGB(255, 0, 0);
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
        if (one_khz_loop_counter > PID_LOOP_DIVIDER) { // 1khz PID loop
            one_khz_loop_counter = 0;
            if (use_current_limit && running) {
                use_current_limit_adjust -= (int16_t)(doPidCalculations(&currentPid, actual_current,
                                                          eepromBuffer.limits.current * 2 * 100)
                    / 10000);
                if (use_current_limit_adjust < minimum_duty_cycle) {
                    use_current_limit_adjust = minimum_duty_cycle;
                }
                if (use_current_limit_adjust > 2000) {
                    use_current_limit_adjust = 2000;
                }
            }
            if (eepromBuffer.stall_protection && running) { // this boosts throttle as the rpm gets lower, for crawlers
                                               // and rc cars only, do not use for multirotors.
                stall_protection_adjust += (doPidCalculations(&stallPid, commutation_interval,
                                               stall_protect_target_interval));
                if (stall_protection_adjust > 150 * 10000) {
                    stall_protection_adjust = 150 * 10000;
                }
                if (stall_protection_adjust <= 0) {
                    stall_protection_adjust = 0;
                }
            }
            if (use_speed_control_loop && running) {
                input_override += doPidCalculations(&speedPid, e_com_time, target_e_com_time);
                if (input_override > 2047 * 10000) {
                    input_override = 2047 * 10000;
                }
                if (input_override < 0) {
                    input_override = 0;
                }
                if (zero_crosses < 100) {
                    speedPid.integral = 0;
                }
            }
        }
        if (maximum_throttle_change_ramp) {
            //	max_duty_cycle_change = map(k_erpm, low_rpm_level,
            // high_rpm_level, 1, 40);
#ifdef VOLTAGE_BASED_RAMP
            uint16_t voltage_based_max_change = map(battery_voltage, 800, 2200, 10, 1);
            if (average_interval > 200) {
                max_duty_cycle_change = voltage_based_max_change;
            } else {
                max_duty_cycle_change = voltage_based_max_change * 3;
            }
#else
            if (zero_crosses < 150 || last_duty_cycle < 150) {   
                max_duty_cycle_change = RAMP_SPEED_STARTUP;
            } else {
                if (average_interval > 500) {
                    max_duty_cycle_change = RAMP_SPEED_LOW_RPM;
                } else {
                    max_duty_cycle_change = RAMP_SPEED_HIGH_RPM;
                }
            }
#endif
#ifdef CUSTOM_RAMP
   //         max_duty_cycle_change = eepromBuffer[30];
#endif
            if ((duty_cycle - last_duty_cycle) > max_duty_cycle_change) {
                duty_cycle = last_duty_cycle + max_duty_cycle_change;
                if (commutation_interval > 500) {
                    fast_accel = 1;
									  temp_advance = eepromBuffer.advance_level;
                } else {
                    fast_accel = 0;
                }

            } else if ((last_duty_cycle - duty_cycle) > max_duty_cycle_change) {
                duty_cycle = last_duty_cycle - max_duty_cycle_change;
                fast_accel = 0;
							  temp_advance = eepromBuffer.advance_level;
            } else {
							if(duty_cycle < 300 && commutation_interval < 300){
								temp_advance = eepromBuffer.advance_level;
							}else{
								temp_advance =  eepromBuffer.advance_level;
							}

                fast_accel = 0;
            }
        }
        if ((armed && running) && input > 47) {
            if (eepromBuffer.variable_pwm) {
            }
            adjusted_duty_cycle = ((duty_cycle * tim1_arr) / 2000) + 1;

        } else {

            if (prop_brake_active) {
                adjusted_duty_cycle = TIMER1_MAX_ARR - ((prop_brake_duty_cycle * tim1_arr) / 2000) + 1;
            } else {
                adjusted_duty_cycle = ((duty_cycle * tim1_arr) / 2000);
            }
        }
        last_duty_cycle = duty_cycle;
        SET_AUTO_RELOAD_PWM(tim1_arr);
        SET_DUTY_CYCLE_ALL(adjusted_duty_cycle);
    }
#endif // ndef brushed_mode
#if defined(FIXED_DUTY_MODE) || defined(FIXED_SPEED_MODE)
    if (getInputPinState()) {
        signaltimeout++;
        if (signaltimeout > LOOP_FREQUENCY_HZ) {
            NVIC_SystemReset();
        }
    } else {
        signaltimeout = 0;
    }
#else
    signaltimeout++;

#endif
}

void processDshot()
{
    if (compute_dshot_flag == 1) {
        computeDshotDMA();
        compute_dshot_flag = 0;
    }
    if (compute_dshot_flag == 2) {
        make_dshot_package(e_com_time);
        compute_dshot_flag = 0;
        return;
    }
    setInput();
}

void advanceincrement()
{
    if (!forward) {
        phase_A_position++;
        if (phase_A_position > 359) {
            phase_A_position = 0;
        }
        phase_B_position++;
        if (phase_B_position > 359) {
            phase_B_position = 0;
        }
        phase_C_position++;
        if (phase_C_position > 359) {
            phase_C_position = 0;
        }
    } else {
        phase_A_position--;
        if (phase_A_position < 0) {
            phase_A_position = 359;
        }
        phase_B_position--;
        if (phase_B_position < 0) {
            phase_B_position = 359;
        }
        phase_C_position--;
        if (phase_C_position < 0) {
            phase_C_position = 359;
        }
    }
#ifdef GIMBAL_MODE
    setPWMCompare1(((2 * pwmSin[phase_A_position]) + gate_drive_offset) * TIMER1_MAX_ARR / 2000);
    setPWMCompare2(((2 * pwmSin[phase_B_position]) + gate_drive_offset) * TIMER1_MAX_ARR / 2000);
    setPWMCompare3(((2 * pwmSin[phase_C_position]) + gate_drive_offset) * TIMER1_MAX_ARR / 2000);
#else
    setPWMCompare1(
        (((2 * pwmSin[phase_A_position] / SINE_DIVIDER) + gate_drive_offset) * TIMER1_MAX_ARR / 2000) * eepromBuffer.sine_mode_power / 10);
    setPWMCompare2(
        (((2 * pwmSin[phase_B_position] / SINE_DIVIDER) + gate_drive_offset) * TIMER1_MAX_ARR / 2000) * eepromBuffer.sine_mode_power / 10);
    setPWMCompare3(
        (((2 * pwmSin[phase_C_position] / SINE_DIVIDER) + gate_drive_offset) * TIMER1_MAX_ARR / 2000) * eepromBuffer.sine_mode_power / 10);
#endif
}

void zcfoundroutine()
{ // only used in polling mode, blocking routine.
    thiszctime = INTERVAL_TIMER_COUNT;
    SET_INTERVAL_TIMER_COUNT(0);
    commutation_interval = (thiszctime + (3 * commutation_interval)) / 4;
    advance = (commutation_interval >> 3) * 2; //   7.5 degree increments
    waitTime = commutation_interval / 2 - advance;
//			if(thiszctime < (commutation_interval - (commutation_interval>>2))){
//					waitTime = waitTime + commutation_interval - thiszctime;
//			    thiszctime = commutation_interval - (commutation_interval>>2);
//		}else if(thiszctime > (commutation_interval + (commutation_interval>>2))){
//			    waitTime = waitTime - thiszctime - commutation_interval; 
//			    thiszctime = commutation_interval - (commutation_interval>>2);
//		}
	
    while ((INTERVAL_TIMER_COUNT) < (waitTime)) {
        if (zero_crosses < 5) {
            break;
        }
    }
#ifdef MCU_GDE23
    TIMER_CAR(COM_TIMER) = waitTime;
#endif
#ifdef STMICRO
    COM_TIMER->ARR = waitTime;
#endif
#ifdef MCU_AT32
		COM_TIMER->pr = waitTime;
#endif
    commutate();
    bemfcounter = 0;
    bad_count = 0;

    zero_crosses++;
#ifdef NO_POLLING_START     // changes to interrupt mode after 30 zero crosses, does not re-enter
       if (zero_crosses > 30) {
            old_routine = 0;
            enableCompInterrupts(); // enable interrupt
        }
#else
    if (eepromBuffer.stall_protection || eepromBuffer.rc_car_reverse) {
        if (zero_crosses >= 20 && commutation_interval <= 2000) {
            old_routine = 0;
            enableCompInterrupts(); // enable interrupt
        }
    } else {
       if (commutation_interval < 2000) {
            old_routine = 0;
            enableCompInterrupts(); // enable interrupt
        }
    }
 #endif
}
#ifdef BRUSHED_MODE
void runBrushedLoop()
{

    uint16_t brushed_duty_cycle = 0;

    if (brushed_direction_set == 0 && adjusted_input > 48) {
        if (forward) {
            allOff();
            delayMicros(10);
            twoChannelForward();
        } else {
            allOff();
            delayMicros(10);
            twoChannelReverse();
        }
        brushed_direction_set = 1;
    }

    brushed_duty_cycle = map(adjusted_input, 48, 2047, 0,
        (TIMER1_MAX_ARR - (TIMER1_MAX_ARR / 20)));

    if (degrees_celsius > eepromBuffer.limits.temperature) {
        duty_cycle_maximum = map(degrees_celsius, eepromBuffer.limits.temperature,
            eepromBuffer.limits.temperature + 20, TIMER1_MAX_ARR / 2, 1);
    } else {
        duty_cycle_maximum = TIMER1_MAX_ARR - 50;
    }
    if (brushed_duty_cycle > duty_cycle_maximum) {
        brushed_duty_cycle = duty_cycle_maximum;
    }

    if (use_current_limit) {
        use_current_limit_adjust -= (int16_t)(doPidCalculations(&currentPid, actual_current,
                                                  CURRENT_LIMIT * 100)
            / 10000);
        if (use_current_limit_adjust < minimum_duty_cycle) {
            use_current_limit_adjust = minimum_duty_cycle;
        }

        if (brushed_duty_cycle > use_current_limit_adjust) {
            brushed_duty_cycle = use_current_limit_adjust;
        }
    }
    if ((brushed_duty_cycle > 0) && armed) {
        SET_DUTY_CYCLE_ALL(brushed_duty_cycle);
        //	  	TIM1->CCR1 = brushed_duty_cycle;
        //		TIM1->CCR2 = brushed_duty_cycle;
        //		TIM1->CCR3 = brushed_duty_cycle;

    } else {
        SET_DUTY_CYCLE_ALL(0);
        //		TIM1->CCR1 = 0;
        //// 		TIM1->CCR2 = 0; 		TIM1->CCR3 = 0;
        brushed_direction_set = 0;
    }
}
#endif


/*
  check device info from the bootloader, confirming pin code and eeprom location
 */
static void checkDeviceInfo(void)
{
#define DEVINFO_MAGIC1 0x5925e3da
#define DEVINFO_MAGIC2 0x4eb863d9

    const struct devinfo {
        uint32_t magic1;
        uint32_t magic2;
        const uint8_t deviceInfo[9];
    } *devinfo = (struct devinfo *)(0x1000 - 32);
    if (devinfo->magic1 != DEVINFO_MAGIC1 ||
        devinfo->magic2 != DEVINFO_MAGIC2) {
        // bootloader does not support this feature, nothing to do
        return;
    }
    // change eeprom_address based on the code in the bootloaders device info
    switch (devinfo->deviceInfo[4]) {
        case 0x1f:
            eeprom_address = 0x08007c00;
            break;
        case 0x35:
            eeprom_address = 0x0800f800;
            break;
        case 0x2b:
            eeprom_address = 0x0801f800;
            break;
    }

    // TODO: check pin code and reboot to bootloader if incorrect

}

int main(void)
{

    initAfterJump();

    checkDeviceInfo();

    initCorePeripherals();

    enableCorePeripherals();

    loadEEpromSettings();

    if (VERSION_MAJOR != eepromBuffer.version.major || VERSION_MINOR != eepromBuffer.version.minor || eeprom_layout_version > eepromBuffer.eeprom_version) {
        eepromBuffer.version.major = VERSION_MAJOR;
        eepromBuffer.version.minor = VERSION_MINOR;
        for (size_t i = 0; i < 12; i++) {
            strlen(FIRMWARE_NAME) > i ? eepromBuffer.firmware_name[i] = (uint8_t)FIRMWARE_NAME[i] : 0;
        }
        saveEEpromSettings();
    }

    // if (eepromBuffer.use_sine_start) {
        //    min_startup_duty = sin_mode_min_s_d;
    // }
    
    if (eepromBuffer.dir_reversed == 1) {
        forward = 0;
    } else {
        forward = 1;
    }
    tim1_arr = TIMER1_MAX_ARR;
 //   startup_max_duty_cycle = startup_max_duty_cycle * TIMER1_MAX_ARR / 2000 + dead_time_override; // adjust for pwm frequency
 //   throttle_max_at_low_rpm = throttle_max_at_low_rpm * TIMER1_MAX_ARR / 2000; // adjust to new pwm frequency
 //   throttle_max_at_high_rpm = TIMER1_MAX_ARR; // adjust to new pwm frequency
    if (!eepromBuffer.comp_pwm) {
        eepromBuffer.use_sine_start = 0; // sine start requires complementary pwm.
    }

    if (eepromBuffer.rc_car_reverse) { // overrides a whole lot of things!
        throttle_max_at_low_rpm = 1000;
        eepromBuffer.bi_direction = 1;
        eepromBuffer.use_sine_start = 0;
        low_rpm_throttle_limit = 1;
        eepromBuffer.variable_pwm = 0;
        // eepromBuffer.stall_protection = 1;
        eepromBuffer.comp_pwm = 0;
        eepromBuffer.stuck_rotor_protection = 0;
        minimum_duty_cycle = minimum_duty_cycle + 50;
        stall_protect_minimum_duty = stall_protect_minimum_duty + 50;
        min_startup_duty = min_startup_duty + 50;
    }

#ifdef MCU_F031
    GPIOF->BSRR = LL_GPIO_PIN_6; // uncomment to take bridge out of standby mode
                                 // and set oc level
    GPIOF->BRR = LL_GPIO_PIN_7; // out of standby mode
    GPIOA->BRR = LL_GPIO_PIN_11;
#endif

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
    // bi_direction = 1;
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

#ifdef USE_ADC_INPUT
    armed_count_threshold = 5000;
    inputSet = 1;

#else
    // checkForHighSignal();     // will reboot if signal line is high for 10ms

    receiveDshotDma();
    if (drive_by_rpm) {
        use_speed_control_loop = 1;
    }
#endif

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

#ifdef USE_INVERTED_HIGH
  min_startup_duty = min_startup_duty + 200;
  minimum_duty_cycle = minimum_duty_cycle + 100;
  startup_max_duty_cycle = startup_max_duty_cycle + 200;
#endif

    while (1) {
#if defined(FIXED_DUTY_MODE) || defined(FIXED_SPEED_MODE)
        setInput();
#endif
#ifdef MCU_F031
        if (input_ready) {
            processDshot();
            input_ready = 0;
        }
#endif
if(zero_crosses < 5){
	  min_bemf_counts_up = TARGET_MIN_BEMF_COUNTS * 2;
		min_bemf_counts_down = TARGET_MIN_BEMF_COUNTS * 2;
}else{
	 min_bemf_counts_up = TARGET_MIN_BEMF_COUNTS;
	min_bemf_counts_down = TARGET_MIN_BEMF_COUNTS;
}
        RELOAD_WATCHDOG_COUNTER();

        if (eepromBuffer.variable_pwm) {
            tim1_arr = map(commutation_interval, 96, 200, TIMER1_MAX_ARR / 2,
                TIMER1_MAX_ARR);
        }
        if (signaltimeout > (LOOP_FREQUENCY_HZ >> 1)) { // half second timeout when armed;
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

        if (tenkhzcounter > LOOP_FREQUENCY_HZ) { // 1s sample interval 10000
            consumed_current += (actual_current << 16) / 360;
            switch (dshot_extended_telemetry) {

            case 1:
                send_extended_dshot = 0b0010 << 8 | degrees_celsius;
                dshot_extended_telemetry = 2;
                break;
            case 2:
                send_extended_dshot = 0b0110 << 8 | (uint8_t)actual_current / 50;
                dshot_extended_telemetry = 3;
                break;
            case 3:
                send_extended_dshot = 0b0100 << 8 | (uint8_t)(battery_voltage / 25);
                dshot_extended_telemetry = 1;
                break;
            }
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
            if (adjusted_input < 150) { // startup duty cycle should be low enough to not burn motor
                bemf_timeout = 100;
            } else {
                bemf_timeout = 10;
            }
        }
#endif
        average_interval = e_com_time / 3;
        if (desync_check && zero_crosses > 10) {
            if ((getAbsDif(last_average_interval, average_interval) > average_interval >> 1) && (average_interval < 2000)) { // throttle resitricted before zc 20.
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
            //	}
            last_average_interval = average_interval;
        }

#ifndef MCU_F031
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
        } else if(send_esc_info_flag ) {
           makeInfoPacket();
           send_telem_DMA(48);
           send_esc_info_flag = 0;
        }
        adc_counter++;
        if (adc_counter > 200) { // for adc and telemetry
#if defined(STMICRO)
            ADC_DMA_Callback();
            LL_ADC_REG_StartConversion(ADC1);
            converted_degrees = __LL_ADC_CALC_TEMPERATURE(3300, ADC_raw_temp, LL_ADC_RESOLUTION_12B);
#endif
#ifdef MCU_GDE23
            ADC_DMA_Callback();
            // converted_degrees = (1.43 - ADC_raw_temp * 3.3 / 4096) * 1000 / 4.3 + 25;
            converted_degrees = ((int32_t)(357.5581395348837f * (1 << 16)) - ADC_raw_temp * (int32_t)(0.18736373546511628f * (1 << 16))) >> 16;
            adc_software_trigger_enable(ADC_REGULAR_CHANNEL);
#endif
#ifdef ARTERY
            ADC_DMA_Callback();
            adc_ordinary_software_trigger_enable(ADC1, TRUE);
            converted_degrees = getConvertedDegrees(ADC_raw_temp);
#endif
            degrees_celsius = converted_degrees;
            battery_voltage = ((7 * battery_voltage) + ((ADC_raw_volts * 3300 / 4095 * VOLTAGE_DIVIDER) / 100)) >> 3;
            smoothed_raw_current = getSmoothedCurrent();
            actual_current = ((smoothed_raw_current * 3300 / 41) - (CURRENT_OFFSET * 100)) / (MILLIVOLT_PER_AMP);
            if (actual_current < 0) {
                actual_current = 0;
            }
            if (LOW_VOLTAGE_CUTOFF) {
                if (battery_voltage < (cell_count * low_cell_volt_cutoff)) {
                    low_voltage_count++;
                    if (low_voltage_count > (20000 - (stepper_sine * 900))) {
                        input = 0;
                        allOff();
                        maskPhaseInterrupts();
                        running = 0;
                        zero_input_count = 0;
                        armed = 0;
                    }
                } else {
                    low_voltage_count = 0;
                }
            }
            adc_counter = 0;
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

            e_rpm = running * (600000 / e_com_time); // in tens of rpm
            k_erpm = e_rpm / 10; // ecom time is time for one electrical revolution in microseconds

            if (low_rpm_throttle_limit) { // some hardware doesn't need this, its on
                                          // by default to keep hardware / motors
                                          // protected but can slow down the response
                                          // in the very low end a little.
                duty_cycle_maximum = map(k_erpm, low_rpm_level, high_rpm_level, throttle_max_at_low_rpm,
                    throttle_max_at_high_rpm); // for more performance lower the
                                               // high_rpm_level, set to a
                                               // consvervative number in source.
            }else{
							duty_cycle_maximum = 2000;
						}

            if (degrees_celsius > eepromBuffer.limits.temperature) {
              duty_cycle_maximum = map(degrees_celsius, eepromBuffer.limits.temperature - 10, eepromBuffer.limits.temperature + 10,
                throttle_max_at_high_rpm / 2, 1);
            }
            if (zero_crosses < 100 && commutation_interval > 500) {
              filter_level = 12;
            } else {
              filter_level = map(average_interval, 100, 500, 3, 12);
            }
            if (commutation_interval < 50) {
              filter_level = 2;
            }

            if (eepromBuffer.auto_advance) {
              auto_advance_level = map(duty_cycle, 100, 2000, 13, 23);
            }

            /**************** old routine*********************/
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
        } else { // stepper sine

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
                        // disable commutation interrupt in case set
                        DISABLE_COM_TIMER_INT();
                        maskPhaseInterrupts();
                        SET_DUTY_CYCLE_ALL(0);
                        allpwm();
                        do_once_sinemode = 0;
                    }
                    advanceincrement();
                    step_delay = map(input, 48, 120, 7000 / eepromBuffer.motor_poles, 810 / eepromBuffer.motor_poles);
                    delayMicros(step_delay);
                    e_rpm = 600 / step_delay; // in hundreds so 33 e_rpm is 3300 actual erpm

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
                        // comStep(step);// rising bemf on a same as position 0.
                        if (eepromBuffer.stall_protection) {
                            last_duty_cycle = stall_protect_minimum_duty;
                        }
                        commutate();
                        generatePwmTimerEvent();
                    }
                }

            } else {
                do_once_sinemode = 1;
                if (eepromBuffer.brake_on_stop) {
#ifndef PWM_ENABLE_BRIDGE
                    duty_cycle = (TIMER1_MAX_ARR - 19) + eepromBuffer.drag_brake_strength * 2;
                    adjusted_duty_cycle = TIMER1_MAX_ARR - ((duty_cycle * tim1_arr) / TIMER1_MAX_ARR) + 1;
                    proportionalBrake();
                    SET_DUTY_CYCLE_ALL(adjusted_duty_cycle);
                    prop_brake_active = 1;
#else
                    // todo add braking for PWM /enable style bridges.
#endif
                } else {
                    SET_DUTY_CYCLE_ALL(0);
                    allOff();
                }
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
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line
       number, tex: printf("Wrong parameters value: file %s on line %d\r\n", file,
       line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
