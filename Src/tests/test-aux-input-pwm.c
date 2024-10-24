#include "functions.h"
// #include "mcu.h"
#include "peripherals.h"
#include "IO.h"
#include "dma.h"
#include "signal.h"
#include "spi.h"
#include "vreg.h"
spi_t spi;

// the pwm read code depends on a lot of variables defined in main
// so I copy and pasted them all for expediency while testing
//===========================================================================
//=============================  Defaults =============================
//===========================================================================

uint8_t drive_by_rpm = 0;
uint32_t MAXIMUM_RPM_SPEED_CONTROL = 10000;
uint32_t MINIMUM_RPM_SPEED_CONTROL = 1000;

enum inputType {
    AUTO_IN,
    DSHOT_IN,
    SERVO_IN,
    SERIAL_IN,
    EDTARM,
};


uint32_t eeprom_address = EEPROM_START_ADD; 
char set_hysteris = 0;
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
char eeprom_layout_version = 2;
char dir_reversed = 0;
char comp_pwm = 1;
char VARIABLE_PWM = 1;
char bi_direction = 0;
char stuck_rotor_protection = 1; // Turn off for Crawlers
char brake_on_stop = 0;
char stall_protection = 0;
char use_sin_start = 0;
char TLM_ON_INTERVAL = 0;
uint8_t telemetry_interval_ms = 30;
uint8_t TEMPERATURE_LIMIT = 255; // degrees 255 to disable
char advance_level = 2; // 7.5 degree increments 0 , 7.5, 15, 22.5)
char temp_advance = 1;
uint16_t motor_kv = 2000;
char motor_poles = 14;
uint16_t CURRENT_LIMIT = 202;
uint8_t sine_mode_power = 5;
char drag_brake_strength = 10; // Drag Brake Power when brake on stop is enabled
uint8_t driving_brake_strength = 10;
uint8_t dead_time_override = DEAD_TIME;
char sine_mode_changeover_thottle_level = 5; // Sine Startup Range
uint16_t stall_protect_target_interval = TARGET_STALL_PROTECTION_INTERVAL;
char USE_HALL_SENSOR = 0;
uint16_t enter_sine_angle = 180;
char do_once_sinemode = 0;
uint8_t auto_advance_level;
char auto_advance = 0;

//============================= Servo Settings ==============================
uint16_t servo_low_threshold = 1100; // anything below this point considered 0
uint16_t servo_high_threshold = 1900; // anything above this point considered 2000 (max)
uint16_t servo_neutral = 1500;
uint8_t servo_dead_band = 100;

//========================= Battery Cuttoff Settings ========================
char LOW_VOLTAGE_CUTOFF = 0; // Turn Low Voltage CUTOFF on or off
uint16_t low_cell_volt_cutoff = 330; // 3.3volts per cell

//=========================== END EEPROM Defaults ===========================

#ifdef USE_MAKE
typedef struct __attribute__((packed)) {
    uint8_t version_major;
    uint8_t version_minor;
    char device_name[12];
} firmware_info_s;

// firmware_info_s __attribute__((section(".firmware_info"))) firmware_info = {
//     version_major : VERSION_MAJOR,
//     version_minor : VERSION_MINOR,
//     device_name : FIRMWARE_NAME
// };
#endif
const char filename[30] __attribute__((section(".file_name"))) = FILE_NAME;

char firmware_name[12] = FIRMWARE_NAME;

uint8_t EEPROM_VERSION;
// move these to targets folder or peripherals for each mcu
char RC_CAR_REVERSE = 0; // have to set bidirectional, comp_pwm off and stall
                         // protection off, no sinusoidal startup
uint16_t ADC_CCR = 30;
uint16_t current_angle = 90;
uint16_t desired_angle = 90;
char return_to_center = 0;
uint16_t target_e_com_time = 0;
int16_t Speed_pid_output;
char use_speed_control_loop = 0;
float input_override = 0;
int16_t use_current_limit_adjust = 2000;
char use_current_limit = 0;
float stall_protection_adjust = 0;

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
float consumed_current = 0;
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

uint8_t eepromBuffer[176] = { 0 };

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

int main()
{
    vreg5V_initialize();
    vreg5V_enable();
    dma_initialize();
    // mcu_setup();
    input_timer_initialize();
    // TIM15->CCMR1 = 1;
    input_timer_enable();
    receiveDshotDma();
    // compute();
    while(1) {
    }
}

void saveEEpromSettings()
{}
