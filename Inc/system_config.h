/*
 * system_config.h
 *
 * System configuration and initialization functions
 */

#ifndef INC_SYSTEM_CONFIG_H_
#define INC_SYSTEM_CONFIG_H_

#include <stdint.h>
#include "main.h"
#include "eeprom.h"
#include "common.h"

// System configuration variables
extern char crawler_mode;
extern char brushed_direction_set;
extern uint16_t motor_kv;
extern uint8_t dead_time_override;
extern uint16_t stall_protect_target_interval;
extern uint8_t drive_by_rpm;
extern uint16_t reverse_speed_threshold;
extern char lowkv;
extern uint16_t armed_timeout_count;
extern uint16_t servo_low_threshold;
extern uint16_t servo_high_threshold;
extern uint16_t servo_neutral;
extern uint8_t servo_dead_band;
extern uint16_t low_cell_volt_cutoff;
extern uint32_t MCU_Id;
extern uint32_t REV_Id;
extern uint16_t sin_mode_min_s_d;
extern char startup_boost;
extern char reversing_dead_band;
extern uint8_t auto_advance_level;
extern uint8_t crsf_input_channel;
extern uint8_t crsf_output_PWM_channel;
extern uint8_t telemetry_interval_ms;

// Function declarations
void initAfterJump(void);
void initCorePeripherals(void);
void enableCorePeripherals(void);
void loadEEpromSettings(void);
void saveEEpromSettings(void);
void setInputPullUp(void);
void setInputPullDown(void);
void MX_IWDG_Init(void);

// Inline configuration functions
static inline void applyMotorModeSettings(void) {
    if (eepromBuffer.dir_reversed == 1) {
        forward = 0;
    } else {
        forward = 1;
    }
    
    if (!eepromBuffer.comp_pwm) {
        eepromBuffer.use_sine_start = 0; // sine start requires complementary pwm
    }
    
    if (eepromBuffer.rc_car_reverse) {
        throttle_max_at_low_rpm = 1000;
        eepromBuffer.bi_direction = 1;
        eepromBuffer.use_sine_start = 0;
        low_rpm_throttle_limit = 1;
        eepromBuffer.variable_pwm = 0;
        eepromBuffer.comp_pwm = 0;
        eepromBuffer.stuck_rotor_protection = 0;
        minimum_duty_cycle = minimum_duty_cycle + 50;
        stall_protect_minimum_duty = stall_protect_minimum_duty + 50;
        min_startup_duty = min_startup_duty + 50;
    }
}

static inline void detectMotorKV(void) {
    motor_kv = eepromBuffer.motor_kv * 100;
    if (motor_kv < 300) {
        motor_kv = 200;
    }
    if (motor_kv >= 300 && motor_kv < 800) {
        dead_time_override = DEAD_TIME + 10;
        if (motor_kv >= 600) {
            dead_time_override = DEAD_TIME + 20;
        }
    }
    if (motor_kv < 1100) {
        lowkv = 1;
        low_rpm_level = 25;
        high_rpm_level = 70;
        reverse_speed_threshold = map(motor_kv, 200, 1100, 2500, 1500);
    } else {
        lowkv = 0;
        low_rpm_level = 35;
        high_rpm_level = 75;
        reverse_speed_threshold = 1500;
    }
}

static inline void updateVersionIfNeeded(void) {
    if (VERSION_MAJOR != eepromBuffer.version.major || 
        VERSION_MINOR != eepromBuffer.version.minor || 
        EEPROM_VERSION > eepromBuffer.eeprom_version) {
        eepromBuffer.version.major = VERSION_MAJOR;
        eepromBuffer.version.minor = VERSION_MINOR;
        eepromBuffer.eeprom_version = EEPROM_VERSION;
        saveEEpromSettings();
    }
}

static inline void configureInputMode(void) {
#ifdef USE_ADC_INPUT
    armed_count_threshold = 5000;
    inputSet = 1;
#else
    #ifdef USE_CRSF_INPUT
        inputSet = 1;
    #else
        receiveDshotDma();
        if (drive_by_rpm) {
            use_speed_control_loop = 1;
        }
    #endif
#endif
}

static inline void applyStartupBoost(void) {
#ifdef USE_STARTUP_BOOST
    min_startup_duty = min_startup_duty + 200 + ((eepromBuffer.pwm_frequency * 100) / 24);
    minimum_duty_cycle = minimum_duty_cycle + 50 + ((eepromBuffer.pwm_frequency * 50) / 24);
    startup_max_duty_cycle = startup_max_duty_cycle + 400;
#endif
}

static inline void initializeHardwarePlatform(void) {
#ifdef MCU_F031
    GPIOF->BSRR = LL_GPIO_PIN_6; // out of standby mode
    GPIOF->BRR = LL_GPIO_PIN_7;
    GPIOA->BRR = LL_GPIO_PIN_11;
#endif
#ifdef MCU_G031
    GPIOA->BRR = LL_GPIO_PIN_11;
    GPIOA->BSRR = LL_GPIO_PIN_12; // Pa12 attached to enable on dev board
#endif
}

#endif /* INC_SYSTEM_CONFIG_H_ */