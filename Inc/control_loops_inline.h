/*
 * control_loops_inline.h
 *
 * AM32 ESC Control Loops Inline Implementations
 * Contains inline function implementations for control loops
 */

#ifndef CONTROL_LOOPS_INLINE_H_
#define CONTROL_LOOPS_INLINE_H_

// External variables that need to be declared in main.c
extern uint16_t duty_cycle;
extern uint16_t duty_cycle_setpoint;
extern uint16_t last_duty_cycle;
extern uint16_t adjusted_duty_cycle;
extern uint16_t max_duty_cycle_change;
extern uint16_t duty_cycle_maximum;
extern uint16_t tim1_arr;
extern uint32_t tenkhzcounter;
extern uint32_t ledcounter;
extern uint32_t one_khz_loop_counter;
extern uint32_t telem_ms_count;
extern uint32_t consumed_current;
extern uint16_t send_extended_dshot;
extern uint8_t dshot_extended_telemetry;
extern uint8_t send_telemetry;
extern uint8_t armed;
extern uint8_t inputSet;
extern uint8_t cell_count;
extern uint16_t adjusted_input;
extern uint16_t zero_input_count;
extern uint16_t armed_timeout_count;
extern uint16_t battery_voltage;
extern uint8_t degrees_celsius;
extern int16_t actual_current;
extern uint8_t stepper_sine;
extern uint8_t old_routine;
extern uint8_t running;
extern uint8_t zcfound;
extern uint8_t rising;
extern uint8_t bemfcounter;
extern uint8_t min_bemf_counts_up;
extern uint8_t min_bemf_counts_down;
extern uint8_t use_current_limit;
extern int16_t use_current_limit_adjust;
extern uint8_t minimum_duty_cycle;
extern uint32_t stall_protection_adjust;
extern uint32_t stall_protect_target_interval;
extern uint32_t input_override;
extern uint32_t target_e_com_time;
extern uint32_t e_com_time;
extern uint32_t zero_crosses;
extern uint32_t average_interval;
extern uint32_t commutation_interval;
extern uint16_t max_ramp_startup;
extern uint16_t max_ramp_low_rpm;
extern uint16_t max_ramp_high_rpm;
extern uint8_t ramp_divider;
extern uint8_t prop_brake_active;
extern uint16_t prop_brake_duty_cycle;
extern uint16_t telemetry_interval_ms;
extern uint8_t PROCESS_ADC_FLAG;
extern fastPID currentPid;
extern fastPID speedPid;
extern fastPID stallPid;

// ==================== INLINE FUNCTION IMPLEMENTATIONS ====================

/**
 * @brief PID calculation function
 */
static inline int32_t doPidCalculations(struct fastPID* pidnow, int actual, int target)
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

/**
 * @brief Update duty cycle with ramping
 */
static inline uint16_t updateDutyCycleRamp(uint16_t target_duty, uint16_t max_change)
{
    uint16_t new_duty = target_duty;
    
    if ((new_duty - last_duty_cycle) > max_change) {
        new_duty = last_duty_cycle + max_change;
    }
    if ((last_duty_cycle - new_duty) > max_change) {
        new_duty = last_duty_cycle - max_change;
    }
    
    return new_duty;
}

/**
 * @brief Process current limiting PID loop
 */
static inline int16_t processCurrentLimitPID(int16_t target_current, int16_t actual_current)
{
    if (use_current_limit && running) {
        int16_t adjustment = -(int16_t)(doPidCalculations(&currentPid, actual_current, target_current) / 10000);
        
        use_current_limit_adjust += adjustment;
        
        if (use_current_limit_adjust < minimum_duty_cycle) {
            use_current_limit_adjust = minimum_duty_cycle;
        }
        if (use_current_limit_adjust > THROTTLE_MAX_HIGH_RPM) {
            use_current_limit_adjust = THROTTLE_MAX_HIGH_RPM;
        }
        
        return use_current_limit_adjust;
    }
    
    return target_current;
}

/**
 * @brief Process speed control PID loop
 */
static inline int32_t processSpeedControlPID(uint32_t target_speed, uint32_t actual_speed)
{
    if (use_speed_control_loop && running) {
        input_override += doPidCalculations(&speedPid, actual_speed, target_speed);
        
        if (input_override > INPUT_SIGNAL_MAX * 10000) {
            input_override = INPUT_SIGNAL_MAX * 10000;
        }
        if (input_override < 0) {
            input_override = 0;
        }
        
        if (zero_crosses < ZERO_CROSS_THRESHOLD) {
            speedPid.integral = 0;
        }
        
        return input_override;
    }
    
    return 0;
}

/**
 * @brief Process stall protection PID loop
 */
static inline int32_t processStallProtectionPID(uint32_t target_interval, uint32_t actual_interval)
{
    if (eepromBuffer.stall_protection && running) {
        stall_protection_adjust += doPidCalculations(&stallPid, actual_interval, target_interval);
        
        if (stall_protection_adjust > 150 * 10000) {
            stall_protection_adjust = 150 * 10000;
        }
        if (stall_protection_adjust <= 0) {
            stall_protection_adjust = 0;
        }
        
        return stall_protection_adjust;
    }
    
    return 0;
}

/**
 * @brief Calculate maximum duty cycle change
 */
static inline uint16_t calculateMaxDutyCycleChange(uint32_t zero_cross_count, 
                                                  uint16_t last_duty, 
                                                  uint32_t avg_interval)
{
    uint16_t max_change;
    
#ifdef VOLTAGE_BASED_RAMP
    uint16_t voltage_based_max_change = map(battery_voltage, 800, 2200, 10, 1);
    if (avg_interval > 200) {
        max_change = voltage_based_max_change;
    } else {
        max_change = voltage_based_max_change * 3;
    }
#else
    if (zero_cross_count < 150 || last_duty < 150) {   
        max_change = max_ramp_startup;
    } else {
        if (avg_interval > 500) {
            max_change = max_ramp_low_rpm;
        } else {
            max_change = max_ramp_high_rpm;
        }
    }
#endif

    return max_change;
}

/**
 * @brief Update PWM timing parameters
 */
static inline void updatePWMTiming(uint32_t commutation_int, uint32_t avg_interval)
{
    if (eepromBuffer.variable_pwm == 1) {
        tim1_arr = map(commutation_int, 96, 200, TIMER1_MAX_ARR / 2, TIMER1_MAX_ARR);
    }
    
    if (eepromBuffer.variable_pwm == 2) {
        if (avg_interval < 250 && avg_interval > 100) {
            tim1_arr = avg_interval * (CPU_FREQUENCY_MHZ / 9);
        } else if (avg_interval < 100 && avg_interval > 0) {
            tim1_arr = 100 * (CPU_FREQUENCY_MHZ / 9);
        } else if ((avg_interval >= 250) || (avg_interval == 0)) {
            tim1_arr = 250 * (CPU_FREQUENCY_MHZ / 9);
        }
    }
}

/**
 * @brief Process arming logic
 */
static inline uint8_t processArmingLogic(uint16_t input_val)
{
    if (!armed) {
        if (cell_count == 0) {
            if (inputSet) {
                if (input_val == 0) {
                    armed_timeout_count++;
                    if (armed_timeout_count > ARMED_TIMEOUT_THRESHOLD) {
                        if (zero_input_count > 30) {
                            armed = 1;
                            
                            // Cell count detection and beeping
                            if ((cell_count == 0) && eepromBuffer.low_voltage_cut_off == 1) {
                                cell_count = battery_voltage / CELL_COUNT_VOLTAGE_THRESHOLD;
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
                            
                            return 1;
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
    
    return armed;
}

/**
 * @brief Update telemetry timing
 */
static inline uint8_t updateTelemetryTiming(uint16_t interval_ms)
{
    if (eepromBuffer.telemetry_on_interval) {
        telem_ms_count++;
        if (telem_ms_count > ((interval_ms - 1 + eepromBuffer.telemetry_on_interval) * 20)) {
            send_telemetry = 1;
            telem_ms_count = 0;
            return 1;
        }
    }
    
    return 0;
}

/**
 * @brief Process extended DSHOT telemetry
 */
static inline uint16_t processExtendedDshotTelemetry(uint8_t temp_celsius, 
                                                    uint16_t current_amps, 
                                                    uint16_t voltage_volts)
{
    uint16_t telemetry_data = 0;
    
    switch (dshot_extended_telemetry) {
        case 1:
            telemetry_data = DSHOT_EXTENDED_TELEMETRY_TEMP | temp_celsius;
            dshot_extended_telemetry = 2;
            break;
        case 2:
            telemetry_data = DSHOT_EXTENDED_TELEMETRY_CURRENT | (uint8_t)(current_amps / TELEMETRY_CURRENT_SCALE);
            dshot_extended_telemetry = 3;
            break;
        case 3:
            telemetry_data = DSHOT_EXTENDED_TELEMETRY_VOLTAGE | (uint8_t)(voltage_volts / TELEMETRY_VOLTAGE_SCALE);
            dshot_extended_telemetry = 1;
            break;
        default:
            dshot_extended_telemetry = 1;
            break;
    }
    
    return telemetry_data;
}

/**
 * @brief Apply duty cycle limits
 */
static inline uint16_t applyDutyCycleLimits(uint16_t requested_duty)
{
    uint16_t limited_duty = requested_duty;
    
    if (limited_duty > duty_cycle_maximum) {
        limited_duty = duty_cycle_maximum;
    }
    
    if (limited_duty < minimum_duty_cycle) {
        limited_duty = minimum_duty_cycle;
    }
    
    return limited_duty;
}

/**
 * @brief Calculate ramp divider
 */
static inline uint8_t calculateRampDivider(uint32_t motor_speed, uint16_t input_level)
{
    uint8_t divider = RAMP_DIVIDER_DEFAULT;
    
    if (motor_speed > 1000) {
        divider = 1;
    } else if (motor_speed > 500) {
        divider = 2;
    } else {
        divider = 4;
    }
    
    if (input_level < 100) {
        divider *= 2;
    }
    
    return divider;
}

/**
 * @brief Main 20kHz control routine
 */
static inline void tenKhzRoutine(void)
{
    duty_cycle = duty_cycle_setpoint;
    tenkhzcounter++;
    ledcounter++;
    one_khz_loop_counter++;
    
    // Process arming logic
    processArmingLogic(adjusted_input);
    
    // Update telemetry timing
    updateTelemetryTiming(telemetry_interval_ms);
    
    // Handle extended DSHOT telemetry
    if (tenkhzcounter > LOOP_FREQUENCY_HZ) {
        consumed_current += (actual_current << 16) / 360;
        send_extended_dshot = processExtendedDshotTelemetry(degrees_celsius, actual_current, battery_voltage);
        tenkhzcounter = 0;
    }

#ifndef BRUSHED_MODE
    if (!stepper_sine) {
#ifndef CUSTOM_RAMP
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
        // 1kHz PID loop
        if (one_khz_loop_counter > PID_LOOP_DIVIDER) {
            PROCESS_ADC_FLAG = 1;
            one_khz_loop_counter = 0;
            
            // Process PID controllers
            processCurrentLimitPID(eepromBuffer.limits.current * 2 * 100, actual_current);
            processStallProtectionPID(stall_protect_target_interval, commutation_interval);
            processSpeedControlPID(target_e_com_time, e_com_time);
        }
        
        // Duty cycle ramping
        if (tenkhzcounter % ramp_divider == 0) {
            max_duty_cycle_change = calculateMaxDutyCycleChange(zero_crosses, last_duty_cycle, average_interval);
            
            if ((duty_cycle - last_duty_cycle) > max_duty_cycle_change) {
                duty_cycle = last_duty_cycle + max_duty_cycle_change;
            }
            if ((last_duty_cycle - duty_cycle) > max_duty_cycle_change) {
                duty_cycle = last_duty_cycle - max_duty_cycle_change;
            }
        } else {
            duty_cycle = last_duty_cycle;
        }

        // Apply duty cycle and update PWM
        if ((armed && running) && input > INPUT_SIGNAL_ARMED_MIN) {
            updatePWMTiming(commutation_interval, average_interval);
            adjusted_duty_cycle = ((duty_cycle * tim1_arr) / TIMER1_RESOLUTION) + 1;
        } else {
            if (prop_brake_active) {
                adjusted_duty_cycle = tim1_arr - ((prop_brake_duty_cycle * tim1_arr) / TIMER1_RESOLUTION);
            } else {
                if ((eepromBuffer.brake_on_stop == 2) && armed) {
                    comStep(2);
                    adjusted_duty_cycle = DEAD_TIME + ((eepromBuffer.active_brake_power * tim1_arr) / TIMER1_RESOLUTION) * 10;
                } else {
                    adjusted_duty_cycle = ((duty_cycle * tim1_arr) / TIMER1_RESOLUTION);
                }
            }
        }
        
        last_duty_cycle = duty_cycle;
        SET_AUTO_RELOAD_PWM(tim1_arr);
        SET_DUTY_CYCLE_ALL(adjusted_duty_cycle);
    }
#endif // ndef BRUSHED_MODE
}

#endif /* CONTROL_LOOPS_INLINE_H_ */