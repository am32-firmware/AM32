/*
 * control_loops.h
 *
 * AM32 ESC Control Loops Module
 * Handles PID controllers, timing loops, and duty cycle control
 */

#ifndef CONTROL_LOOPS_H_
#define CONTROL_LOOPS_H_

#include "main.h"
#include "common.h"
#include "constants.h"
#include "targets.h"

// ==================== CONTROL LOOP FUNCTIONS ====================

/**
 * @brief Main 20kHz control routine
 * @details High-frequency control loop for duty cycle ramping and PID calculations
 * @note Originally named tenKhzRoutine, now runs at 20kHz
 */
static inline void tenKhzRoutine(void);

/**
 * @brief PID calculation function
 * @details Performs PID calculations for various control loops
 * @param pidnow Pointer to PID structure
 * @param actual Current actual value
 * @param target Target setpoint value
 * @return PID output value
 */
static inline int32_t doPidCalculations(struct fastPID* pidnow, int actual, int target);

/**
 * @brief Update duty cycle with ramping
 * @details Applies smooth duty cycle ramping to prevent sudden changes
 * @param target_duty Target duty cycle
 * @param max_change Maximum allowed change per cycle
 * @return Updated duty cycle value
 */
static inline uint16_t updateDutyCycleRamp(uint16_t target_duty, uint16_t max_change);

/**
 * @brief Process current limiting PID loop
 * @details Handles current limiting control to protect hardware
 * @param target_current Target current limit
 * @param actual_current Measured current
 * @return Current limit adjustment
 */
static inline int16_t processCurrentLimitPID(int16_t target_current, int16_t actual_current);

/**
 * @brief Process speed control PID loop
 * @details Handles speed control for constant RPM operation
 * @param target_speed Target speed (e_com_time)
 * @param actual_speed Actual speed (e_com_time)
 * @return Speed control adjustment
 */
static inline int32_t processSpeedControlPID(uint32_t target_speed, uint32_t actual_speed);

/**
 * @brief Process stall protection PID loop
 * @details Handles stall protection for crawler and RC car modes
 * @param target_interval Target commutation interval
 * @param actual_interval Actual commutation interval
 * @return Stall protection adjustment
 */
static inline int32_t processStallProtectionPID(uint32_t target_interval, uint32_t actual_interval);

/**
 * @brief Calculate maximum duty cycle change
 * @details Determines maximum allowed duty cycle change based on conditions
 * @param zero_cross_count Number of zero crossings
 * @param last_duty Previous duty cycle
 * @param avg_interval Average commutation interval
 * @return Maximum allowed duty cycle change
 */
static inline uint16_t calculateMaxDutyCycleChange(uint32_t zero_cross_count, 
                                                  uint16_t last_duty, 
                                                  uint32_t avg_interval);

/**
 * @brief Update PWM timing parameters
 * @details Updates PWM frequency and timing based on motor speed
 * @param commutation_int Current commutation interval
 * @param avg_interval Average commutation interval
 */
static inline void updatePWMTiming(uint32_t commutation_int, uint32_t avg_interval);

/**
 * @brief Process arming logic
 * @details Handles motor arming/disarming state machine
 * @param input_val Current input value
 * @return 1 if armed, 0 if disarmed
 */
static inline uint8_t processArmingLogic(uint16_t input_val);

/**
 * @brief Update telemetry timing
 * @details Manages telemetry output timing
 * @param interval_ms Telemetry interval in milliseconds
 * @return 1 if telemetry should be sent, 0 otherwise
 */
static inline uint8_t updateTelemetryTiming(uint16_t interval_ms);

/**
 * @brief Process extended DSHOT telemetry
 * @details Handles extended DSHOT telemetry data cycling
 * @param temp_celsius Temperature in Celsius
 * @param current_amps Current in amperes
 * @param voltage_volts Voltage in volts
 * @return Extended telemetry data
 */
static inline uint16_t processExtendedDshotTelemetry(uint8_t temp_celsius, 
                                                    uint16_t current_amps, 
                                                    uint16_t voltage_volts);

/**
 * @brief Apply duty cycle limits
 * @details Applies various duty cycle limits based on conditions
 * @param requested_duty Requested duty cycle
 * @return Limited duty cycle value
 */
static inline uint16_t applyDutyCycleLimits(uint16_t requested_duty);

/**
 * @brief Calculate ramp divider
 * @details Calculates appropriate ramp divider based on conditions
 * @param motor_speed Current motor speed
 * @param input_level Input level
 * @return Ramp divider value
 */
static inline uint8_t calculateRampDivider(uint32_t motor_speed, uint16_t input_level);

// ==================== CONTROL LOOPS INLINE IMPLEMENTATIONS ====================

#include "control_loops_inline.h"

#endif /* CONTROL_LOOPS_H_ */