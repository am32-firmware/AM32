/*
 * motor_control.h
 *
 * AM32 ESC Motor Control Module
 * Handles motor commutation, BEMF sensing, and phase control
 */

#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_

#include "main.h"
#include "common.h"
#include "constants.h"
#include "targets.h"

// ==================== MOTOR CONTROL FUNCTIONS ====================

/**
 * @brief Read back-EMF state for position sensing
 * @details Samples the back-EMF signal to determine rotor position
 * @note This function is called from the main loop and interrupt routines
 */
static inline void getBemfState(void);

/**
 * @brief Switch motor phases for commutation
 * @details Advances the motor to the next electrical phase
 * @note Uses inline for optimal performance in interrupt context
 */
static inline void commutate(void);

/**
 * @brief Zero-crossing detection routine
 * @details Detects zero-crossing events for sensorless control
 * @note Only used in polling mode, blocking routine
 */
static inline void zcfoundroutine(void);

/**
 * @brief Start motor from stopped state
 * @details Initiates motor startup sequence
 */
static inline void startMotor(void);

/**
 * @brief Advance sine wave increment for stepper mode
 * @details Advances the sine wave position for smooth motor control
 */
static inline void advanceincrement(void);

/**
 * @brief Timer period elapsed callback
 * @details Handles timer-based commutation events
 */
static inline void PeriodElapsedCallback(void);

/**
 * @brief Get smoothed current reading
 * @details Applies filtering to current measurements
 * @return Smoothed current value
 */
static inline uint16_t getSmoothedCurrent(void);

/**
 * @brief Calculate commutation timing
 * @details Determines optimal commutation timing based on motor speed
 * @param interval Current commutation interval
 * @return Adjusted timing value
 */
static inline uint32_t calculateCommutationTiming(uint32_t interval);

/**
 * @brief Update motor phase positions
 * @details Updates A, B, C phase positions for sine wave control
 */
static inline void updatePhasePositions(void);

/**
 * @brief Apply advance timing correction
 * @details Adjusts commutation timing for motor efficiency
 * @param base_timing Base commutation timing
 * @return Corrected timing value
 */
static inline uint32_t applyAdvanceTiming(uint32_t base_timing);

/**
 * @brief Check for motor stall condition
 * @details Monitors motor for stall conditions
 * @return True if stall detected, false otherwise
 */
static inline uint8_t checkMotorStall(void);

/**
 * @brief Apply brake to motor
 * @details Applies regenerative or active braking
 * @param brake_power Brake power level (0-100%)
 */
static inline void applyMotorBrake(uint8_t brake_power);

/**
 * @brief Calculate duty cycle limits
 * @details Determines safe duty cycle limits based on conditions
 * @return Maximum allowed duty cycle
 */
static inline uint16_t calculateDutyCycleLimits(void);

// ==================== MOTOR CONTROL INLINE IMPLEMENTATIONS ====================

#include "motor_control_inline.h"

#endif /* MOTOR_CONTROL_H_ */