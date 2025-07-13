/*
 * pwm_control.h
 *
 * PWM control and duty cycle calculation functions
 */

#ifndef INC_PWM_CONTROL_H_
#define INC_PWM_CONTROL_H_

#include <stdint.h>
#include "main.h"
#include "common.h"

// PWM control variables
extern uint16_t tim1_arr;
extern uint16_t TIMER1_MAX_ARR;
extern uint16_t duty_cycle;
extern uint16_t adjusted_duty_cycle;
extern uint16_t last_duty_cycle;
extern uint16_t duty_cycle_setpoint;
extern uint16_t duty_cycle_maximum;
extern uint16_t minimum_duty_cycle;
extern uint16_t stall_protect_minimum_duty;
extern uint16_t startup_max_duty_cycle;
extern uint16_t min_startup_duty;
extern uint8_t max_duty_cycle_change;
extern char fast_accel;
extern char fast_deccel;
extern uint16_t low_rpm_level;
extern uint16_t high_rpm_level;
extern uint16_t throttle_max_at_low_rpm;
extern uint16_t throttle_max_at_high_rpm;
extern char low_rpm_throttle_limit;
extern uint16_t prop_brake_duty_cycle;
extern char prop_brake_active;
extern char maximum_throttle_change_ramp;
extern uint8_t auto_advance_level;

// Function declarations
void SET_DUTY_CYCLE_ALL(uint16_t duty);
void generatePwmTimerEvent(void);
void proportionalBrake(void);
void fullBrake(void);

// Inline PWM control functions
static inline void updateVariablePWMFrequency(void) {
    if (eepromBuffer.variable_pwm == 1) {
        tim1_arr = map(commutation_interval, 96, 200, TIMER1_MAX_ARR / 2, TIMER1_MAX_ARR);
    }
    if (eepromBuffer.variable_pwm == 2) {
        if (average_interval < 250 && average_interval > 100) {
            tim1_arr = average_interval * (CPU_FREQUENCY_MHZ / 9);
        }
        if (average_interval < 100 && average_interval > 0) {
            tim1_arr = 100 * (CPU_FREQUENCY_MHZ / 9);
        }
        if ((average_interval >= 250) || (average_interval == 0)) {
            tim1_arr = 250 * (CPU_FREQUENCY_MHZ / 9);
        }
    }
}

static inline void updateDutyCycleMaximum(uint16_t k_erpm) {
    if (low_rpm_throttle_limit) {
        duty_cycle_maximum = map(k_erpm, low_rpm_level, high_rpm_level, 
                                throttle_max_at_low_rpm, throttle_max_at_high_rpm);
    } else {
        duty_cycle_maximum = 2000;
    }
    
    if (degrees_celsius > eepromBuffer.limits.temperature) {
        duty_cycle_maximum = map(degrees_celsius, 
                                eepromBuffer.limits.temperature - 10, 
                                eepromBuffer.limits.temperature + 10,
                                throttle_max_at_high_rpm / 2, 1);
    }
}

static inline void applyBrakeOnStop(void) {
    if (eepromBuffer.brake_on_stop == 1) {
#ifndef PWM_ENABLE_BRIDGE
        prop_brake_duty_cycle = eepromBuffer.drag_brake_strength * 200;
        adjusted_duty_cycle = tim1_arr - ((prop_brake_duty_cycle * tim1_arr) / 2000);
        if (adjusted_duty_cycle < 100) {
            fullBrake();
        } else {
            proportionalBrake();
            SET_DUTY_CYCLE_ALL(adjusted_duty_cycle);
            prop_brake_active = 1;
        }
#endif
    } else if (eepromBuffer.brake_on_stop == 2) {
        comStep(2);
        SET_DUTY_CYCLE_ALL(DEAD_TIME + ((eepromBuffer.active_brake_power * tim1_arr) / 2000) * 10);
    } else {
        SET_DUTY_CYCLE_ALL(0);
        allOff();
    }
}

static inline void limitDutyCycleChange(void) {
    int16_t change = duty_cycle_setpoint - last_duty_cycle;
    int16_t max_change = max_duty_cycle_change;
    
    if (commutation_interval > 500) {
        max_change = max_duty_cycle_change << 2;
    }
    if (average_interval > 500) {
        max_change = max_duty_cycle_change << 3;
    }
    
    if (change > max_change) {
        duty_cycle = last_duty_cycle + max_change;
        fast_accel = 1;
        fast_deccel = 0;
    } else if (change < -max_change) {
        duty_cycle = last_duty_cycle - max_change;
        fast_deccel = 1;
        fast_accel = 0;
    } else {
        duty_cycle = duty_cycle_setpoint;
        fast_accel = 0;
        fast_deccel = 0;
    }
}

static inline void updateAutoAdvance(void) {
    if (eepromBuffer.auto_advance) {
        auto_advance_level = map(duty_cycle, 100, 2000, 13, 23);
    }
}

#endif /* INC_PWM_CONTROL_H_ */