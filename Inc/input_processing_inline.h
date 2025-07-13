/*
 * input_processing_inline.h
 *
 * AM32 ESC Input Processing Inline Implementations
 * Contains inline function implementations for input processing
 */

#ifndef INPUT_PROCESSING_INLINE_H_
#define INPUT_PROCESSING_INLINE_H_

// External variables that need to be declared in main.c
extern uint16_t newinput;
extern uint16_t adjusted_input;
extern uint16_t input;
extern uint8_t dshot;
extern uint8_t servoPwm;
extern uint8_t forward;
extern uint8_t return_to_center;
extern uint8_t prop_brake_active;
extern uint8_t servo_dead_band;
extern uint8_t reversing_dead_band;
extern uint8_t drive_by_rpm;
extern uint8_t use_speed_control_loop;
extern uint32_t target_e_com_time;
extern uint32_t input_override;
extern uint32_t commutation_interval;
extern uint16_t duty_cycle;
extern uint8_t stepper_sine;
extern uint8_t brushed_direction_set;
extern uint16_t reverse_speed_threshold;
extern uint8_t zero_crosses;
extern uint8_t old_routine;
extern uint8_t masked_phase_interrupts;
extern uint8_t armed;
extern uint8_t inputSet;
extern uint16_t zero_input_count;
extern uint16_t signaltimeout;
extern uint16_t armed_timeout_count;
extern uint16_t armed_count_threshold;
extern uint32_t MAXIMUM_RPM_SPEED_CONTROL;
extern uint32_t MINIMUM_RPM_SPEED_CONTROL;
extern fastPID speedPid;
extern uint8_t bemf_timeout_happened;
extern uint8_t bemf_timeout;
extern uint8_t stuck_rotor_protection;

// ==================== INLINE FUNCTION IMPLEMENTATIONS ====================

/**
 * @brief Validate input signal range
 */
static inline uint16_t validateInputSignal(uint16_t input_val)
{
    if (input_val > INPUT_SIGNAL_MAX) {
        return INPUT_SIGNAL_MAX;
    }
    if (input_val < INPUT_SIGNAL_MIN) {
        return INPUT_SIGNAL_MIN;
    }
    return input_val;
}

/**
 * @brief Apply input deadband
 */
static inline uint16_t applyInputDeadband(uint16_t input_val, uint8_t deadband)
{
    uint16_t neutral = INPUT_SIGNAL_NEUTRAL;
    
    if (input_val >= (neutral - deadband) && input_val <= (neutral + deadband)) {
        return neutral;
    }
    return input_val;
}

/**
 * @brief Map input signal to throttle range
 */
static inline uint16_t mapInputToThrottle(uint16_t input_val, uint16_t min_input, 
                                        uint16_t max_input, uint16_t min_output, 
                                        uint16_t max_output)
{
    return map(input_val, min_input, max_input, min_output, max_output);
}

/**
 * @brief Process bidirectional input
 */
static inline uint16_t processBidirectionalInput(uint16_t input_val)
{
    uint16_t deadband_width = servo_dead_band << 1;
    uint16_t neutral = INPUT_SIGNAL_NEUTRAL;
    
    if (input_val > (neutral + deadband_width)) {
        // Forward direction
        if (forward == eepromBuffer.dir_reversed) {
            if (((commutation_interval > reverse_speed_threshold) && 
                 (duty_cycle < 200)) || stepper_sine) {
                forward = 1 - eepromBuffer.dir_reversed;
                zero_crosses = 0;
                old_routine = 1;
                maskPhaseInterrupts();
                brushed_direction_set = 0;
            } else {
                return neutral;
            }
        }
        return map(input_val, neutral + deadband_width, INPUT_SIGNAL_MAX, 
                  INPUT_SIGNAL_ARMED_MIN, INPUT_SIGNAL_MAX);
    }
    
    if (input_val < (neutral - deadband_width)) {
        // Reverse direction
        if (forward == (1 - eepromBuffer.dir_reversed)) {
            if (((commutation_interval > reverse_speed_threshold) && 
                 (duty_cycle < 200)) || stepper_sine) {
                zero_crosses = 0;
                old_routine = 1;
                forward = eepromBuffer.dir_reversed;
                maskPhaseInterrupts();
                brushed_direction_set = 0;
            } else {
                return neutral;
            }
        }
        return map(input_val, INPUT_SIGNAL_MIN, neutral - deadband_width, 
                  INPUT_SIGNAL_MAX, INPUT_SIGNAL_ARMED_MIN);
    }
    
    // In deadband
    brushed_direction_set = 0;
    return 0;
}

/**
 * @brief Process RC car mode input
 */
static inline uint16_t processRCCarInput(uint16_t input_val)
{
    uint16_t deadband_width = servo_dead_band << 1;
    uint16_t neutral = INPUT_SIGNAL_NEUTRAL;
    
    if (input_val > (neutral + deadband_width)) {
        // Forward throttle
        if (forward == eepromBuffer.dir_reversed) {
            adjusted_input = 0;
            prop_brake_active = 1;
            if (return_to_center) {
                forward = 1 - eepromBuffer.dir_reversed;
                prop_brake_active = 0;
                return_to_center = 0;
            }
        }
        if (prop_brake_active == 0) {
            return_to_center = 0;
            return map(input_val, neutral + deadband_width, INPUT_SIGNAL_MAX, 
                      INPUT_SIGNAL_ARMED_MIN, INPUT_SIGNAL_MAX);
        }
        return 0;
    }
    
    if (input_val < (neutral - deadband_width)) {
        // Reverse throttle
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
            return map(input_val, INPUT_SIGNAL_MIN, neutral - deadband_width, 
                      INPUT_SIGNAL_MAX, INPUT_SIGNAL_ARMED_MIN);
        }
        return 0;
    }
    
    // In deadband (neutral)
    if (prop_brake_active) {
        prop_brake_active = 0;
        return_to_center = 1;
    }
    return 0;
}

/**
 * @brief Process DSHOT input
 */
static inline uint16_t processDshotInput(uint16_t input_val)
{
    if (input_val > DSHOT_FORWARD_THRESHOLD) {
        // Forward direction
        if (forward == eepromBuffer.dir_reversed) {
            if (((commutation_interval > reverse_speed_threshold) && 
                 (duty_cycle < 200)) || stepper_sine) {
                forward = 1 - eepromBuffer.dir_reversed;
                zero_crosses = 0;
                old_routine = 1;
                maskPhaseInterrupts();
                brushed_direction_set = 0;
            } else {
                return 0;
            }
        }
        return ((input_val - 1048) * 2 + INPUT_SIGNAL_ARMED_MIN) - reversing_dead_band;
    }
    
    if (input_val <= DSHOT_FORWARD_THRESHOLD && input_val > INPUT_SIGNAL_ARMED_MIN) {
        // Reverse direction
        if (forward == (1 - eepromBuffer.dir_reversed)) {
            if (((commutation_interval > reverse_speed_threshold) && 
                 (duty_cycle < 200)) || stepper_sine) {
                zero_crosses = 0;
                old_routine = 1;
                forward = eepromBuffer.dir_reversed;
                maskPhaseInterrupts();
                brushed_direction_set = 0;
            } else {
                return 0;
            }
        }
        return ((input_val - 48) * 2 + INPUT_SIGNAL_ARMED_MIN) - reversing_dead_band;
    }
    
    // Below threshold
    brushed_direction_set = 0;
    return 0;
}

/**
 * @brief Process sine mode input
 */
static inline uint16_t processSineModeInput(uint16_t input_val)
{
    if (input_val < 30) {
        return 0;
    }
    
    uint16_t changeover_level = eepromBuffer.sine_mode_changeover_thottle_level * 20;
    
    if (input_val > 30 && input_val < changeover_level) {
        return map(input_val, 30, changeover_level, INPUT_SIGNAL_ARMED_MIN, 160);
    }
    
    if (input_val >= changeover_level) {
        return map(input_val, changeover_level, INPUT_SIGNAL_MAX, 160, INPUT_SIGNAL_MAX);
    }
    
    return 0;
}

/**
 * @brief Process speed control input
 */
static inline uint16_t processSpeedControlInput(uint16_t input_val)
{
    if (drive_by_rpm) {
        target_e_com_time = 60000000 / map(input_val, INPUT_SIGNAL_ARMED_MIN, INPUT_SIGNAL_MAX, 
                                          MINIMUM_RPM_SPEED_CONTROL, MAXIMUM_RPM_SPEED_CONTROL) / 
                           (eepromBuffer.motor_poles / 2);
        
        if (input_val < INPUT_SIGNAL_ARMED_MIN) {
            speedPid.error = 0;
            input_override = 0;
            return 0;
        } else {
            uint16_t speed_output = (uint16_t)(input_override / 10000);
            if (speed_output > INPUT_SIGNAL_MAX) {
                speed_output = INPUT_SIGNAL_MAX;
            }
            if (speed_output < 48) {
                speed_output = 48;
            }
            return speed_output;
        }
    }
    
    return input_val;
}

/**
 * @brief Check for input timeout
 */
static inline uint8_t checkInputTimeout(void)
{
    if (armed) {
        return (signaltimeout > (LOOP_FREQUENCY_HZ >> 1));
    } else {
        return (signaltimeout > (LOOP_FREQUENCY_HZ << 1));
    }
}

/**
 * @brief Filter input signal
 */
static inline uint16_t filterInputSignal(uint16_t input_val)
{
    // Simple moving average filter
    static uint16_t filter_history[4] = {0, 0, 0, 0};
    static uint8_t filter_index = 0;
    
    filter_history[filter_index] = input_val;
    filter_index = (filter_index + 1) % 4;
    
    uint32_t sum = 0;
    for (uint8_t i = 0; i < 4; i++) {
        sum += filter_history[i];
    }
    
    return sum >> 2;
}

/**
 * @brief Process input arming sequence
 */
static inline uint8_t processInputArming(uint16_t input_val)
{
    if (!armed) {
        if (inputSet) {
            if (input_val == 0) {
                armed_timeout_count++;
                if (armed_timeout_count > ARMED_TIMEOUT_THRESHOLD) {
                    if (zero_input_count > 30) {
                        return 1; // Armed
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
    
    return armed;
}

/**
 * @brief Handle direction change request
 */
static inline uint8_t handleDirectionChange(uint8_t requested_direction)
{
    // Only allow direction change if motor is stopped or at low speed
    if (((commutation_interval > reverse_speed_threshold) && 
         (duty_cycle < 200)) || stepper_sine) {
        forward = requested_direction;
        zero_crosses = 0;
        old_routine = 1;
        maskPhaseInterrupts();
        brushed_direction_set = 0;
        return 1;
    }
    
    return 0;
}

/**
 * @brief Main input processing function
 */
static inline void setInput(void)
{
    uint16_t processed_input = newinput;
    
    // Apply input validation
    processed_input = validateInputSignal(processed_input);
    
    if (eepromBuffer.bi_direction) {
        if (dshot == 0) {
            if (eepromBuffer.rc_car_reverse) {
                adjusted_input = processRCCarInput(processed_input);
            } else {
                adjusted_input = processBidirectionalInput(processed_input);
            }
        } else {
            adjusted_input = processDshotInput(processed_input);
        }
    } else {
        adjusted_input = processed_input;
    }
    
#ifndef BRUSHED_MODE
    // Check for stuck rotor protection
    if ((bemf_timeout_happened > bemf_timeout) && eepromBuffer.stuck_rotor_protection) {
        allOff();
        maskPhaseInterrupts();
        input = 0;
        bemf_timeout_happened = 102;
        return;
    }
    
    // Process input based on mode
    if (eepromBuffer.use_sine_start) {
        input = processSineModeInput(adjusted_input);
    } else {
        if (use_speed_control_loop) {
            input = processSpeedControlInput(adjusted_input);
        } else {
            input = adjusted_input;
        }
    }
#endif
}

/**
 * @brief Process DSHOT protocol
 */
static inline void processDshot(void)
{
    // DSHOT processing logic would go here
    // This is a placeholder for the actual DSHOT processing
    // which involves decoding the DSHOT frame and extracting commands
    
    if (dshot) {
        // Process DSHOT frame
        // Extract throttle value and commands
        // Handle telemetry requests
    }
}

/**
 * @brief Detect input signal type
 */
static inline input_type_t detectInputType(void)
{
    // Auto-detection logic based on signal characteristics
    if (dshot) {
        return INPUT_TYPE_DSHOT;
    }
    
    if (servoPwm) {
        return INPUT_TYPE_PWM;
    }
    
    return INPUT_TYPE_PWM; // Default to PWM
}

#endif /* INPUT_PROCESSING_INLINE_H_ */