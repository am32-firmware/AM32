/*
 * input_processing.h
 *
 * AM32 ESC Input Processing Module
 * Handles signal processing, input validation, and DSHOT protocol
 */

#ifndef INPUT_PROCESSING_H_
#define INPUT_PROCESSING_H_

#include "main.h"
#include "common.h"
#include "constants.h"
#include "targets.h"

// ==================== INPUT PROCESSING TYPES ====================

/**
 * @brief Input signal type enumeration
 */
typedef enum {
    INPUT_TYPE_PWM = 0,
    INPUT_TYPE_DSHOT = 1,
    INPUT_TYPE_SERIAL = 2,
    INPUT_TYPE_CRSF = 3,
    INPUT_TYPE_DRONECAN = 4
} input_type_t;

/**
 * @brief Input signal processing state
 */
typedef struct {
    uint16_t raw_input;
    uint16_t filtered_input;
    uint16_t adjusted_input;
    uint8_t signal_valid;
    uint8_t armed;
    uint8_t timeout_count;
    input_type_t type;
} input_state_t;

// ==================== INPUT PROCESSING FUNCTIONS ====================

/**
 * @brief Process input signals (PWM/DSHOT/etc)
 * @details Main input processing function that handles all signal types
 */
static inline void setInput(void);

/**
 * @brief Handle DSHOT protocol processing
 * @details Processes DSHOT commands and throttle values
 */
static inline void processDshot(void);

/**
 * @brief Validate input signal range
 * @details Ensures input signals are within valid bounds
 * @param input Raw input value
 * @return Validated input value
 */
static inline uint16_t validateInputSignal(uint16_t input);

/**
 * @brief Apply input deadband
 * @details Applies deadband around neutral position
 * @param input Raw input value
 * @param deadband Deadband width
 * @return Processed input value
 */
static inline uint16_t applyInputDeadband(uint16_t input, uint8_t deadband);

/**
 * @brief Map input signal to throttle range
 * @details Maps input signal to appropriate throttle range
 * @param input Raw input value
 * @param min_input Minimum input value
 * @param max_input Maximum input value
 * @param min_output Minimum output value
 * @param max_output Maximum output value
 * @return Mapped throttle value
 */
static inline uint16_t mapInputToThrottle(uint16_t input, uint16_t min_input, 
                                        uint16_t max_input, uint16_t min_output, 
                                        uint16_t max_output);

/**
 * @brief Process bidirectional input
 * @details Handles forward/reverse input processing
 * @param input Raw input value
 * @return Processed bidirectional input
 */
static inline uint16_t processBidirectionalInput(uint16_t input);

/**
 * @brief Process RC car mode input
 * @details Handles RC car specific input processing with braking
 * @param input Raw input value
 * @return Processed RC car input
 */
static inline uint16_t processRCCarInput(uint16_t input);

/**
 * @brief Check for input timeout
 * @details Monitors input signal for timeout conditions
 * @return 1 if timeout detected, 0 otherwise
 */
static inline uint8_t checkInputTimeout(void);

/**
 * @brief Filter input signal
 * @details Applies digital filtering to input signal
 * @param input Raw input value
 * @return Filtered input value
 */
static inline uint16_t filterInputSignal(uint16_t input);

/**
 * @brief Detect input signal type
 * @details Automatically detects the type of input signal
 * @return Detected input type
 */
static inline input_type_t detectInputType(void);

/**
 * @brief Process input arming sequence
 * @details Handles the arming/disarming logic
 * @param input Current input value
 * @return 1 if armed, 0 if disarmed
 */
static inline uint8_t processInputArming(uint16_t input);

/**
 * @brief Handle direction change request
 * @details Processes direction change requests safely
 * @param requested_direction Requested direction (0=reverse, 1=forward)
 * @return 1 if direction change allowed, 0 otherwise
 */
static inline uint8_t handleDirectionChange(uint8_t requested_direction);

/**
 * @brief Process sine mode input
 * @details Handles input processing for sine wave mode
 * @param input Raw input value
 * @return Processed sine mode input
 */
static inline uint16_t processSineModeInput(uint16_t input);

/**
 * @brief Process speed control input
 * @details Handles input for speed control mode
 * @param input Raw input value
 * @return Processed speed control input
 */
static inline uint16_t processSpeedControlInput(uint16_t input);

// ==================== INPUT PROCESSING INLINE IMPLEMENTATIONS ====================

#include "input_processing_inline.h"

#endif /* INPUT_PROCESSING_H_ */