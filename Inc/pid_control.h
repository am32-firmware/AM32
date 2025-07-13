/*
 * pid_control.h
 *
 * PID control functions for speed, current, and stall protection
 */

#ifndef INC_PID_CONTROL_H_
#define INC_PID_CONTROL_H_

#include <stdint.h>
#include "common.h"

// PID controllers
extern fastPID speedPid;
extern fastPID currentPid; 
extern fastPID stallPid;

// PID control variables
extern char use_speed_control_loop;
extern char use_current_limit;
extern int16_t use_current_limit_adjust;
extern int32_t stall_protection_adjust;
extern int16_t Speed_pid_output;
extern int32_t input_override;
extern uint16_t target_e_com_time;
extern uint16_t MAXIMUM_RPM_SPEED_CONTROL;
extern uint16_t MINIMUM_RPM_SPEED_CONTROL;

// Inline PID calculation function
static inline int32_t doPidCalculations(struct fastPID* pidnow, int32_t actual, int32_t target)
{
    pidnow->error = (target - actual);
    pidnow->integral = pidnow->integral + pidnow->error;
    
    if (pidnow->integral > pidnow->integral_limit) {
        pidnow->integral = pidnow->integral_limit;
    }
    if (pidnow->integral < -pidnow->integral_limit) {
        pidnow->integral = -pidnow->integral_limit;
    }
    
    pidnow->derivative = pidnow->error - pidnow->last_error;
    pidnow->pid_output = (pidnow->Kp * pidnow->error + pidnow->Ki * pidnow->integral + pidnow->Kd * pidnow->derivative) >> 10;
    
    if (pidnow->pid_output > pidnow->output_limit) {
        pidnow->pid_output = pidnow->output_limit;
    }
    if (pidnow->pid_output < -pidnow->output_limit) {
        pidnow->pid_output = -pidnow->output_limit;
    }
    
    pidnow->last_error = pidnow->error;
    
    return pidnow->pid_output;
}

// Inline function for speed control
static inline void updateSpeedControl(uint16_t k_erpm, uint16_t input) {
    if (use_speed_control_loop) {
        if (drive_by_rpm) {
            target_e_com_time = 600000 / map(input, 47, 2047, MINIMUM_RPM_SPEED_CONTROL, MAXIMUM_RPM_SPEED_CONTROL); 
        }
        if ((zero_crosses < 10000) && (commutation_interval > 800)) {
            Speed_pid_output = map(k_erpm, 5, 100, -250, 2000);
            speedPid.integral = Speed_pid_output << 10;
        } else {
            Speed_pid_output = doPidCalculations(&speedPid, e_com_time, target_e_com_time);
        }
        if (Speed_pid_output >= 0 && running) {
            input_override = Speed_pid_output;
        } else {
            input_override = 0;
        }
    } else {
        input_override = 0;
        if (zero_crosses < 100) {
            speedPid.integral = 0;
        }
    }
}

// Inline function for current limiting
static inline void updateCurrentLimit(int16_t actual_current) {
    if (use_current_limit) {
        if (actual_current > eepromBuffer.limits.current_limit * 65536 / 100) {
            use_current_limit_adjust = doPidCalculations(&currentPid, actual_current, eepromBuffer.limits.current_limit * 65536 / 100);
        } else {
            if (commutation_interval < 2000) {
                currentPid.integral = (int32_t)adjusted_input << 10;
            }
            use_current_limit_adjust = 2000;
        }
    } else {
        use_current_limit_adjust = 2000;
    }
}

// Inline function for stall protection
static inline void updateStallProtection(uint16_t k_erpm, uint16_t average_interval, int16_t stall_protect_target_interval) {
    if (eepromBuffer.stall_protection && running) {
        if (duty_cycle > stall_protect_minimum_duty && (zero_crosses < 350 || average_interval > stall_protect_target_interval)) {
            if (commutation_interval > 2000) {
                stall_protection_adjust = doPidCalculations(&stallPid, k_erpm, 20);
            } else {
                if (duty_cycle < 600) {
                    stall_protection_adjust = map(commutation_interval, 100, 2000, -(stall_protect_minimum_duty - duty_cycle), 0);
                    stallPid.integral = (stall_protection_adjust) << 10;
                } else {
                    stall_protection_adjust = doPidCalculations(&stallPid, k_erpm, 20);
                }
            }
        } else {
            stall_protection_adjust = 0;
            stallPid.integral = 0;
        }
    } else {
        stall_protection_adjust = 0;
    }
}

#endif /* INC_PID_CONTROL_H_ */