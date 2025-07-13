/*
 * main_refactored_demo.c
 *
 * AM32 ESC Firmware - Refactored Main File
 * Demonstrates the improved structure after refactoring
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

// New modular headers
#include "constants.h"
#include "motor_control.h"
#include "input_processing.h"
#include "control_loops.h"

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

// ==================== GLOBAL VARIABLES ====================
// (Variables that need to be globally accessible)

// Configuration and state
EEprom_t eepromBuffer;
uint8_t drive_by_rpm = 0;
uint8_t forward = 1;
uint8_t running = 0;
uint8_t armed = 0;
uint8_t stepper_sine = 0;

// Motor control variables
uint16_t duty_cycle = 0;
uint16_t duty_cycle_setpoint = 0;
uint16_t last_duty_cycle = 0;
uint16_t adjusted_duty_cycle = 0;
uint16_t tim1_arr = TIMER1_MAX_ARR;
uint32_t commutation_interval = COMMUTATION_INTERVAL_INITIAL;
uint32_t commutation_intervals[COMMUTATION_INTERVALS_COUNT];
uint32_t e_com_time = 0;
uint32_t average_interval = 0;
uint32_t zero_crosses = 0;
uint8_t step = 0;
uint8_t bemfcounter = 0;
uint8_t zcfound = 0;
uint8_t rising = 0;
uint8_t old_routine = 0;
uint8_t bad_count = 0;
uint8_t bad_count_threshold = 4;

// Input processing variables
uint16_t newinput = 0;
uint16_t adjusted_input = 0;
uint16_t input = 0;
uint16_t signaltimeout = 0;
uint16_t zero_input_count = 0;
uint8_t dshot = 0;
uint8_t servo_dead_band = SERVO_DEAD_BAND_DEFAULT;
uint8_t reversing_dead_band = REVERSING_DEAD_BAND_DEFAULT;

// Control loop variables
uint32_t tenkhzcounter = 0;
uint32_t one_khz_loop_counter = 0;
uint8_t use_current_limit = 0;
int16_t use_current_limit_adjust = THROTTLE_MAX_HIGH_RPM;
uint32_t stall_protection_adjust = 0;
uint32_t input_override = 0;
uint32_t target_e_com_time = 0;

// Safety and monitoring
uint16_t battery_voltage = 0;
int16_t actual_current = 0;
uint8_t degrees_celsius = 0;
uint8_t cell_count = 0;
uint8_t bemf_timeout_happened = 0;
uint8_t bemf_timeout = BEMF_TIMEOUT_DEFAULT;

// PID Controllers
fastPID speedPid = {
    .Kp = SPEED_PID_KP_DEFAULT,
    .Ki = SPEED_PID_KI_DEFAULT,
    .Kd = SPEED_PID_KD_DEFAULT,
    .integral_limit = PID_INTEGRAL_LIMIT_DEFAULT,
    .output_limit = PID_OUTPUT_LIMIT_DEFAULT
};

fastPID currentPid = {
    .Kp = CURRENT_PID_KP_DEFAULT,
    .Ki = CURRENT_PID_KI_DEFAULT,
    .Kd = CURRENT_PID_KD_DEFAULT,
    .integral_limit = 20000,
    .output_limit = 100000
};

fastPID stallPid = {
    .Kp = STALL_PID_KP_DEFAULT,
    .Ki = STALL_PID_KI_DEFAULT,
    .Kd = STALL_PID_KD_DEFAULT,
    .integral_limit = PID_INTEGRAL_LIMIT_DEFAULT,
    .output_limit = PID_OUTPUT_LIMIT_DEFAULT
};

// ==================== FUNCTION DECLARATIONS ====================

void loadEEpromSettings(void);
void saveEEpromSettings(void);
void runBrushedLoop(void);
void processDshot(void);
static void checkDeviceInfo(void);
void interruptRoutine(void);

// ==================== HELPER FUNCTIONS ====================

/**
 * @brief Load EEPROM settings with validation
 */
void loadEEpromSettings(void)
{
    read_flash_bin(eepromBuffer.buffer, eeprom_address, sizeof(eepromBuffer.buffer));
    
    // Validate and update EEPROM version
    if (eepromBuffer.eeprom_version < EEPROM_VERSION) {
        eepromBuffer.max_ramp = 160;
        eepromBuffer.minimum_duty_cycle = 1;
        eepromBuffer.disable_stick_calibration = 0;
        eepromBuffer.absolute_voltage_cutoff = 10;
        eepromBuffer.current_P = 100;
        eepromBuffer.current_I = 0;
        eepromBuffer.current_D = 100;
        eepromBuffer.active_brake_power = 0;
        
        // Clear reserved bytes
        for (int i = 0; i < 4; i++) {
            eepromBuffer.reserved_eeprom_3[i] = 0;
        }
    }
    
    // Validate advance level
    if (eepromBuffer.advance_level > 42 || 
        (eepromBuffer.advance_level < 10 && eepromBuffer.advance_level > 3)) {
        temp_advance = PHASE_ADVANCE_DEFAULT;
    } else if (eepromBuffer.advance_level < 4) {
        temp_advance = (eepromBuffer.advance_level << 3);
    } else if (eepromBuffer.advance_level < 43 && eepromBuffer.advance_level > 9) {
        temp_advance = eepromBuffer.advance_level - 10;
    }
    
    // Configure PWM frequency
    if (eepromBuffer.pwm_frequency >= PWM_FREQUENCY_MIN && 
        eepromBuffer.pwm_frequency <= PWM_FREQUENCY_MAX) {
        if (eepromBuffer.pwm_frequency <= PWM_FREQUENCY_MAX && 
            eepromBuffer.pwm_frequency > PWM_FREQUENCY_DEFAULT) {
            TIMER1_MAX_ARR = map(eepromBuffer.pwm_frequency, PWM_FREQUENCY_DEFAULT, 
                                PWM_FREQUENCY_MAX, TIM1_AUTORELOAD, TIM1_AUTORELOAD / 6);
        }
        SET_AUTO_RELOAD_PWM(TIMER1_MAX_ARR);
    } else {
        tim1_arr = TIM1_AUTORELOAD;
        SET_AUTO_RELOAD_PWM(tim1_arr);
    }
    
    // Configure minimum duty cycle
    if (eepromBuffer.minimum_duty_cycle < 51 && eepromBuffer.minimum_duty_cycle > 0) {
        minimum_duty_cycle = eepromBuffer.minimum_duty_cycle * 10;
    } else {
        minimum_duty_cycle = MINIMUM_DUTY_CYCLE_DEFAULT;
    }
    
    // Configure startup power
    if (eepromBuffer.startup_power < 151 && eepromBuffer.startup_power > 49) {
        min_startup_duty = minimum_duty_cycle + eepromBuffer.startup_power;
    } else {
        min_startup_duty = minimum_duty_cycle;
    }
    
    startup_max_duty_cycle = minimum_duty_cycle + STARTUP_MAX_DUTY_CYCLE_DEFAULT;
    motor_kv = (eepromBuffer.motor_kv * 40) + 20;
    
#ifdef THREE_CELL_MAX
    motor_kv = motor_kv / 2;
#endif
    
    setVolume(2);
    
    // Configure throttle limits
    if (eepromBuffer.motor_kv > 300) {
        low_rpm_throttle_limit = 0;
    }
    
    if (eepromBuffer.motor_kv < 30) {
        throttle_max_at_low_rpm = 1000;
        low_rpm_level = 20;
        high_rpm_level = 70;
    }
    
    reverse_speed_threshold = map(eepromBuffer.motor_kv, 30, 300, 
                                 REVERSE_SPEED_THRESHOLD / 2, REVERSE_SPEED_THRESHOLD);
    
    // Configure PID parameters
    if (eepromBuffer.current_P > 0) {
        currentPid.Kp = eepromBuffer.current_P * 4;
    }
    if (eepromBuffer.current_I > 0) {
        currentPid.Ki = eepromBuffer.current_I;
    }
    if (eepromBuffer.current_D > 0) {
        currentPid.Kd = eepromBuffer.current_D * 4;
    }
}

/**
 * @brief Save EEPROM settings
 */
void saveEEpromSettings(void)
{
    save_flash_nolib(eepromBuffer.buffer, sizeof(eepromBuffer.buffer), eeprom_address);
}

/**
 * @brief Check device information
 */
static void checkDeviceInfo(void)
{
    const struct devinfo {
        uint32_t magic1;
        uint32_t magic2;
        const uint8_t deviceInfo[9];
    } *devinfo = (struct devinfo *)(0x1000 - 32);
    
    if (devinfo->magic1 != 0x464c457f || devinfo->magic2 != 0x464c457f) {
        return;
    }
    
    // Process device information
    firmwareVersion.deviceInfo = devinfo->deviceInfo;
}

/**
 * @brief Interrupt routine for input processing
 */
void interruptRoutine(void)
{
    // Input processing interrupt
    if (input_ready) {
        processDshot();
        input_ready = 0;
    }
}

/**
 * @brief Process DSHOT commands
 */
void processDshot(void)
{
    // DSHOT command processing
    if (dshot_command) {
        switch (dshot_command) {
            case 1:
                // Beep command
                playBeaconTune3();
                break;
            case 2:
                // ESC info
                send_esc_info_flag = 1;
                break;
            case 12:
                // Save settings
                saveEEpromSettings();
                playInputTune();
                break;
            case 20:
                // Spin direction 1
                if (eepromBuffer.dir_reversed == 0) {
                    eepromBuffer.dir_reversed = 1;
                } else {
                    eepromBuffer.dir_reversed = 0;
                }
                break;
            case 21:
                // Spin direction 2
                if (eepromBuffer.dir_reversed == 1) {
                    eepromBuffer.dir_reversed = 0;
                } else {
                    eepromBuffer.dir_reversed = 1;
                }
                break;
            default:
                break;
        }
        dshot_command = 0;
    }
}

/**
 * @brief Brushed motor control loop
 */
void runBrushedLoop(void)
{
#ifdef BRUSHED_MODE
    // Brushed motor control implementation
    if (input > INPUT_SIGNAL_ARMED_MIN && armed) {
        duty_cycle = map(input, INPUT_SIGNAL_ARMED_MIN, INPUT_SIGNAL_MAX, 
                        minimum_duty_cycle, BRUSHED_MAX_DUTY);
        
        SET_DUTY_CYCLE_ALL(duty_cycle);
        
        if (eepromBuffer.bi_direction) {
            if (forward) {
                SET_BRUSHED_DIRECTION_FORWARD();
            } else {
                SET_BRUSHED_DIRECTION_REVERSE();
            }
        }
    } else {
        SET_DUTY_CYCLE_ALL(0);
    }
#endif
}

// ==================== MAIN FUNCTION ====================

int main(void)
{
    // Initialization sequence
    initAfterJump();
    checkDeviceInfo();
    initCorePeripherals();
    enableCorePeripherals();
    loadEEpromSettings();
    
    // Version check and update
    if (VERSION_MAJOR != eepromBuffer.version.major || 
        VERSION_MINOR != eepromBuffer.version.minor || 
        EEPROM_VERSION > eepromBuffer.eeprom_version) {
        eepromBuffer.version.major = VERSION_MAJOR;
        eepromBuffer.version.minor = VERSION_MINOR;
        eepromBuffer.eeprom_version = EEPROM_VERSION;
        saveEEpromSettings();
    }
    
    // Configure direction
    if (eepromBuffer.dir_reversed == 1) {
        forward = 0;
    } else {
        forward = 1;
    }
    
    // Initialize timer
    tim1_arr = TIMER1_MAX_ARR;
    if (!eepromBuffer.comp_pwm) {
        eepromBuffer.use_sine_start = 0;
    }
    
    // RC car mode configuration
    if (eepromBuffer.rc_car_reverse) {
        throttle_max_at_low_rpm = THROTTLE_MAX_LOW_RPM;
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
    
    // Target-specific initialization
#ifdef MCU_F031
    GPIOF->BSRR = LL_GPIO_PIN_6;
    GPIOF->BRR = LL_GPIO_PIN_7;
    GPIOA->BRR = LL_GPIO_PIN_11;
#endif
    
#ifdef MCU_G031
    GPIOA->BRR = LL_GPIO_PIN_11;
    GPIOA->BSRR = LL_GPIO_PIN_12;
#endif
    
    // Initialize watchdog and startup
    MX_IWDG_Init();
    RELOAD_WATCHDOG_COUNTER();
    
#ifdef USE_CRSF_INPUT
    inputSet = 1;
    playStartupTune();
#else
    #ifdef FIXED_DUTY_MODE
        inputSet = 1;
        armed = 1;
        adjusted_input = 48;
        newinput = 48;
        comStep(2);
    #else
        #ifdef BRUSHED_MODE
            commutation_interval = COMMUTATION_INTERVAL_INITIAL;
            eepromBuffer.use_sine_start = 0;
            maskPhaseInterrupts();
            playBrushedStartupTune();
        #else
            playStartupTune();
        #endif
    #endif
#endif
    
    // Set input pull-up/down
#ifdef NEUTRONRC_G071
    setInputPullDown();
#else
    setInputPullUp();
#endif
    
    // Main control loop
    while (1) {
        // Calculate commutation timing
        e_com_time = ((commutation_intervals[0] + commutation_intervals[1] + 
                      commutation_intervals[2] + commutation_intervals[3] + 
                      commutation_intervals[4] + commutation_intervals[5]) + 4) >> 1;
        
        // Process input (now handled by inline function)
        setInput();
        
        // Update watchdog
        RELOAD_WATCHDOG_COUNTER();
        
        // Update PWM timing
        if (eepromBuffer.variable_pwm == 1) {
            tim1_arr = map(commutation_interval, 96, 200, 
                          TIMER1_MAX_ARR / 2, TIMER1_MAX_ARR);
        }
        
        // Signal timeout handling
        if (checkInputTimeout()) {
            if (armed) {
                allOff();
                armed = 0;
                input = 0;
                inputSet = 0;
                zero_input_count = 0;
                SET_DUTY_CYCLE_ALL(0);
                resetInputCaptureTimer();
                NVIC_SystemReset();
            }
            if (signaltimeout > SIGNAL_TIMEOUT_DISARMED) {
                allOff();
                NVIC_SystemReset();
            }
        }
        
        // LED control
#ifdef USE_CUSTOM_LED
        if ((input >= INPUT_SIGNAL_ARMED_MIN) && (input < 1947)) {
            if (ledcounter > (2000 >> forward)) {
                GPIOB->BSRR = LL_GPIO_PIN_3;
            } else {
                GPIOB->BRR = LL_GPIO_PIN_3;
            }
            if (ledcounter > (4000 >> forward)) {
                ledcounter = 0;
            }
        }
#endif
        
        // Telemetry processing
        if (send_telemetry) {
#ifdef USE_SERIAL_TELEMETRY
            makeTelemPackage((int8_t)degrees_celsius, battery_voltage, 
                           actual_current, (uint16_t)(consumed_current >> 16), e_rpm);
            send_telem_DMA(TELEMETRY_PACKET_SIZE);
            send_telemetry = 0;
#endif
        }
        
        // ADC processing
        if (PROCESS_ADC_FLAG == 1) {
            ADC_DMA_Callback();
            LL_ADC_REG_StartConversion(ADC1);
            
            // Temperature calculation
            converted_degrees = __LL_ADC_CALC_TEMPERATURE(3300, ADC_raw_temp, 
                                                         LL_ADC_RESOLUTION_12B);
            degrees_celsius = converted_degrees;
            
            // Voltage measurement
            battery_voltage = ((7 * battery_voltage) + 
                              ((ADC_raw_volts * 3300 / 4095 * VOLTAGE_DIVIDER) / 100)) >> 3;
            
            // Current measurement
            smoothed_raw_current = getSmoothedCurrent();
            actual_current = ((smoothed_raw_current * 3300 / 41) - 
                             (CURRENT_OFFSET * 100)) / (MILLIVOLT_PER_AMP);
            if (actual_current < 0) {
                actual_current = 0;
            }
            
            // Low voltage protection
            if (eepromBuffer.low_voltage_cut_off == 1) {
                if (battery_voltage < (cell_count * LOW_CELL_VOLTAGE_CUTOFF)) {
                    low_voltage_count++;
                } else {
                    if (!LOW_VOLTAGE_CUTOFF) {
                        low_voltage_count = 0;
                    }
                }
            }
            
            if (low_voltage_count > LOW_VOLTAGE_COUNT_THRESHOLD) {
                LOW_VOLTAGE_CUTOFF = 1;
                input = 0;
                allOff();
                maskPhaseInterrupts();
                running = 0;
                zero_input_count = 0;
                armed = 0;
            }
            
            PROCESS_ADC_FLAG = 0;
        }
        
        // Motor control logic
        if (stepper_sine == 0) {
            // Calculate RPM
            e_rpm = running * (600000 / e_com_time);
            k_erpm = e_rpm / 10;
            
            // Calculate duty cycle limits
            duty_cycle_maximum = calculateDutyCycleLimits();
            
            // Filter level calculation
            if (zero_crosses < ZERO_CROSS_THRESHOLD && commutation_interval > 500) {
                filter_level = FILTER_LEVEL_MAX;
            } else {
                filter_level = map(average_interval, 100, 500, 
                                  FILTER_LEVEL_MIN, FILTER_LEVEL_MAX);
            }
            
            // Auto advance calculation
            if (eepromBuffer.auto_advance) {
                auto_advance_level = map(duty_cycle, 100, THROTTLE_MAX_HIGH_RPM, 
                                        13, 23);
            }
            
            // Timeout handling
            if (INTERVAL_TIMER_COUNT > COMMUTATION_INTERVAL_MAX && running == 1) {
                bemf_timeout_happened++;
                maskPhaseInterrupts();
                old_routine = 1;
                if (input < 48) {
                    running = 0;
                    commutation_interval = COMMUTATION_INTERVAL_INITIAL;
                }
                zero_crosses = 0;
                zcfoundroutine();
            }
        } else {
            // Sine wave mode
#ifdef GIMBAL_MODE
            step_delay = GIMBAL_STEP_DELAY;
            maskPhaseInterrupts();
            allpwm();
            
            if (newinput > INPUT_SIGNAL_NEUTRAL) {
                desired_angle = map(newinput, INPUT_SIGNAL_NEUTRAL, INPUT_SIGNAL_MAX, 
                                   180, DEGREES_PER_CIRCLE);
            } else {
                desired_angle = map(newinput, INPUT_SIGNAL_MIN, INPUT_SIGNAL_NEUTRAL, 
                                   0, 180);
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
            // Sine wave stepper mode
            if (input > INPUT_SIGNAL_ARMED_MIN && armed) {
                if (input > INPUT_SIGNAL_ARMED_MIN && input < 137) {
                    if (do_once_sinemode) {
                        DISABLE_COM_TIMER_INT();
                        maskPhaseInterrupts();
                        SET_DUTY_CYCLE_ALL(0);
                        allpwm();
                        do_once_sinemode = 0;
                    }
                    
                    advanceincrement();
                    step_delay = map(input, INPUT_SIGNAL_ARMED_MIN, 120, 
                                    7000 / eepromBuffer.motor_poles, 
                                    810 / eepromBuffer.motor_poles);
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
                applyMotorBrake(eepromBuffer.drag_brake_strength);
                e_rpm = 0;
            }
#endif
        }
        
        // Brushed mode
#ifdef BRUSHED_MODE
        runBrushedLoop();
#endif
        
        // DroneCAN support
#if DRONECAN_SUPPORT
        DroneCAN_update();
#endif
    }
}

// ==================== ASSERT FUNCTION ====================

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{
    /* Assert failed - add debug handling here */
}
#endif /* USE_FULL_ASSERT */