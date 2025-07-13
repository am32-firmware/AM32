/*
 * motor_control_inline.h
 *
 * AM32 ESC Motor Control Inline Implementations
 * Contains inline function implementations for optimal performance
 */

#ifndef MOTOR_CONTROL_INLINE_H_
#define MOTOR_CONTROL_INLINE_H_

// External variables that need to be declared in main.c
extern uint8_t bemfcounter;
extern uint8_t min_bemf_counts_up;
extern uint8_t min_bemf_counts_down;
extern uint8_t zcfound;
extern uint8_t rising;
extern uint8_t running;
extern uint8_t step;
extern uint8_t bad_count;
extern uint8_t bad_count_threshold;
extern uint8_t stepper_sine;
extern uint8_t old_routine;
extern uint16_t duty_cycle;
extern uint16_t last_duty_cycle;
extern uint16_t adjusted_duty_cycle;
extern uint16_t prop_brake_duty_cycle;
extern uint16_t tim1_arr;
extern uint32_t commutation_interval;
extern uint32_t commutation_intervals[COMMUTATION_BUFFER_SIZE];
extern uint32_t e_com_time;
extern uint32_t average_interval;
extern uint32_t zero_crosses;
extern int16_t phase_A_position;
extern int16_t phase_B_position;
extern int16_t phase_C_position;
extern int16_t pwmSin[];
extern int32_t smoothed_raw_current;
extern int16_t actual_current;
extern uint8_t prop_brake_active;
extern uint8_t armed;
extern uint16_t input;
extern uint8_t forward;
extern uint8_t brushed_direction_set;
extern uint8_t changeover_step;
extern uint16_t stall_protect_minimum_duty;
extern uint16_t minimum_duty_cycle;
extern uint16_t startup_max_duty_cycle;
extern uint16_t duty_cycle_maximum;
extern uint16_t k_erpm;
extern uint16_t low_rpm_level;
extern uint16_t high_rpm_level;
extern uint16_t throttle_max_at_low_rpm;
extern uint16_t throttle_max_at_high_rpm;
extern uint8_t degrees_celsius;
extern uint8_t filter_level;
extern uint8_t auto_advance_level;
extern uint8_t temp_advance;
extern uint32_t step_delay;
extern uint16_t current_angle;
extern uint16_t desired_angle;
extern uint8_t do_once_sinemode;

// ==================== INLINE FUNCTION IMPLEMENTATIONS ====================

/**
 * @brief Get smoothed current reading
 */
static inline uint16_t getSmoothedCurrent(void)
{
    static uint16_t total = 0;
    static uint16_t readings[8] = {0};
    static uint8_t readIndex = 0;
    static uint16_t smoothedcurrent = 0;
    static const uint8_t numReadings = 8;
    
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

/**
 * @brief Read back-EMF state for position sensing
 */
static inline void getBemfState(void)
{
    uint8_t current_state = 0;
    
#if defined(MCU_F031) || defined(MCU_G031)
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

/**
 * @brief Switch motor phases for commutation
 */
static inline void commutate(void)
{
    if (!running) {
        return;
    }
    
    // Update commutation intervals
    commutation_intervals[step] = commutation_interval;
    
    // Calculate average interval
    e_com_time = ((commutation_intervals[0] + commutation_intervals[1] + 
                   commutation_intervals[2] + commutation_intervals[3] + 
                   commutation_intervals[4] + commutation_intervals[5]) + 4) >> 1;
    
    // Advance to next step
    if (forward) {
        step++;
        if (step > 5) {
            step = 0;
        }
    } else {
        step--;
        if (step < 0) {
            step = 5;
        }
    }
    
    // Set motor phases
    comStep(step);
    
    // Zero crossing detection setup
    zcfound = 0;
    bemfcounter = 0;
    
    // Set rising/falling detection
    if (step == 0 || step == 3) {
        rising = 0;
    } else {
        rising = 1;
    }
    
    zero_crosses++;
}

/**
 * @brief Zero-crossing detection routine
 */
static inline void zcfoundroutine(void)
{
    if (!old_routine) {
        return;
    }
    
    commutation_interval = INTERVAL_TIMER_COUNT;
    
    if (zero_crosses > 5) {
        if (average_interval > 2000) {
            SET_INTERVAL_TIMER_COUNT(average_interval / 2);
        } else {
            SET_INTERVAL_TIMER_COUNT(average_interval);
        }
    } else {
        SET_INTERVAL_TIMER_COUNT(commutation_interval);
    }
    
    commutate();
    generatePwmTimerEvent();
    
    old_routine = 0;
}

/**
 * @brief Start motor from stopped state
 */
static inline void startMotor(void)
{
    if (running) {
        return;
    }
    
    running = 1;
    stepper_sine = 0;
    zero_crosses = 0;
    old_routine = 1;
    
    commutation_interval = COMMUTATION_INTERVAL_INITIAL;
    average_interval = COMMUTATION_INTERVAL_INITIAL;
    
    // Set initial step based on direction
    if (forward) {
        step = 0;
    } else {
        step = 3;
    }
    
    // Initialize duty cycle
    duty_cycle = minimum_duty_cycle;
    last_duty_cycle = minimum_duty_cycle;
    
    // Start commutation
    commutate();
    generatePwmTimerEvent();
}

/**
 * @brief Advance sine wave increment for stepper mode
 */
static inline void advanceincrement(void)
{
    if (forward) {
        phase_A_position++;
        if (phase_A_position >= SINE_MODE_STEPS) {
            phase_A_position = 0;
        }
    } else {
        phase_A_position--;
        if (phase_A_position < 0) {
            phase_A_position = SINE_MODE_STEPS - 1;
        }
    }
    
    // Calculate phase B and C positions
    phase_B_position = phase_A_position + 48;
    if (phase_B_position >= SINE_MODE_STEPS) {
        phase_B_position -= SINE_MODE_STEPS;
    }
    
    phase_C_position = phase_A_position + 96;
    if (phase_C_position >= SINE_MODE_STEPS) {
        phase_C_position -= SINE_MODE_STEPS;
    }
    
    // Apply sine wave values
    SET_DUTY_CYCLE_A(DEAD_TIME + ((pwmSin[phase_A_position] + SINE_AMPLITUDE_DEFAULT) * tim1_arr) / 2000);
    SET_DUTY_CYCLE_B(DEAD_TIME + ((pwmSin[phase_B_position] + SINE_AMPLITUDE_DEFAULT) * tim1_arr) / 2000);
    SET_DUTY_CYCLE_C(DEAD_TIME + ((pwmSin[phase_C_position] + SINE_AMPLITUDE_DEFAULT) * tim1_arr) / 2000);
}

/**
 * @brief Timer period elapsed callback
 */
static inline void PeriodElapsedCallback(void)
{
    if (!old_routine) {
        return;
    }
    
    commutation_interval = INTERVAL_TIMER_COUNT;
    
    if (commutation_interval > COMMUTATION_INTERVAL_MAX) {
        // Timeout occurred
        running = 0;
        old_routine = 1;
        zero_crosses = 0;
        return;
    }
    
    // Zero crossing detection
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

/**
 * @brief Calculate commutation timing
 */
static inline uint32_t calculateCommutationTiming(uint32_t interval)
{
    uint32_t adjusted_interval = interval;
    
    if (zero_crosses > 5) {
        if (adjusted_interval > 2000) {
            adjusted_interval = interval / 2;
        }
    }
    
    return adjusted_interval;
}

/**
 * @brief Update motor phase positions
 */
static inline void updatePhasePositions(void)
{
    // Update positions based on advance timing
    int16_t advance_offset = temp_advance;
    
    phase_A_position = (phase_A_position + advance_offset) % SINE_MODE_STEPS;
    phase_B_position = (phase_B_position + advance_offset) % SINE_MODE_STEPS;
    phase_C_position = (phase_C_position + advance_offset) % SINE_MODE_STEPS;
}

/**
 * @brief Apply advance timing correction
 */
static inline uint32_t applyAdvanceTiming(uint32_t base_timing)
{
    uint32_t advanced_timing = base_timing;
    
    if (eepromBuffer.auto_advance) {
        auto_advance_level = map(duty_cycle, 100, 2000, 13, 23);
        advanced_timing = (base_timing * auto_advance_level) / 32;
    } else {
        advanced_timing = (base_timing * temp_advance) / 32;
    }
    
    return advanced_timing;
}

/**
 * @brief Check for motor stall condition
 */
static inline uint8_t checkMotorStall(void)
{
    if (commutation_interval > COMMUTATION_INTERVAL_MAX) {
        return 1;
    }
    
    if (zero_crosses < 5 && duty_cycle > startup_max_duty_cycle) {
        return 1;
    }
    
    return 0;
}

/**
 * @brief Apply brake to motor
 */
static inline void applyMotorBrake(uint8_t brake_power)
{
    if (brake_power == 0) {
        allOff();
        return;
    }
    
    if (eepromBuffer.brake_on_stop == 1) {
        // Proportional brake
        prop_brake_duty_cycle = brake_power * 2;
        adjusted_duty_cycle = tim1_arr - ((prop_brake_duty_cycle * tim1_arr) / 2000);
        
        if (adjusted_duty_cycle < 100) {
            fullBrake();
        } else {
            proportionalBrake();
            SET_DUTY_CYCLE_ALL(adjusted_duty_cycle);
            prop_brake_active = 1;
        }
    } else if (eepromBuffer.brake_on_stop == 2) {
        // Active brake
        comStep(2);
        SET_DUTY_CYCLE_ALL(DEAD_TIME + ((brake_power * tim1_arr) / 2000) * 10);
    }
}

/**
 * @brief Calculate duty cycle limits
 */
static inline uint16_t calculateDutyCycleLimits(void)
{
    uint16_t max_duty = THROTTLE_MAX_HIGH_RPM;
    
    if (eepromBuffer.low_rpm_throttle_limit) {
        max_duty = map(k_erpm, low_rpm_level, high_rpm_level, 
                      throttle_max_at_low_rpm, throttle_max_at_high_rpm);
    }
    
    // Temperature limiting
    if (degrees_celsius > eepromBuffer.limits.temperature) {
        max_duty = map(degrees_celsius, 
                      eepromBuffer.limits.temperature - 10, 
                      eepromBuffer.limits.temperature + 10,
                      throttle_max_at_high_rpm / 2, 1);
    }
    
    return max_duty;
}

#endif /* MOTOR_CONTROL_INLINE_H_ */