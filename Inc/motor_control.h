/*
 * motor_control.h
 *
 * Motor control functions including commutation, startup, and BEMF detection
 */

#ifndef INC_MOTOR_CONTROL_H_
#define INC_MOTOR_CONTROL_H_

#include <stdint.h>
#include "main.h"
#include "common.h"

// Motor control state
extern uint8_t stepper_sine;
extern uint8_t running;
extern uint8_t changeover_step;
extern uint8_t step;
extern uint16_t commutation_interval;
extern uint32_t zero_crosses;
extern uint8_t bemfcounter;
extern uint8_t zcfound;
extern uint8_t filter_level;
extern uint8_t advance;
extern uint8_t rising;
extern uint16_t phase_A_position;
extern uint16_t phase_B_position;
extern uint16_t phase_C_position;
extern int16_t pwmSin[];
extern uint16_t gate_drive_offset;
extern uint8_t bemf_timeout_happened;
extern uint8_t old_routine;
extern uint16_t lastzctime;
extern uint16_t thiszctime;
extern uint16_t waitTime;
extern uint16_t enter_sine_angle;
extern char do_once_sinemode;
extern uint16_t adjusted_duty_cycle;
extern uint16_t tim1_arr;
extern uint16_t startup_max_duty_cycle;
extern uint16_t duty_cycle;
extern uint16_t min_bemf_counts_up;
extern uint16_t min_bemf_counts_down;
extern uint8_t desync_check;
extern uint32_t average_interval;
extern uint32_t last_average_interval;

// Function declarations
void commutate(void);
void startMotor(void);
void zcfoundroutine(void);
void getBemfState(void);
void advanceincrement(void);

// Inline motor control functions
static inline void updateSinewavePhases(void) {
    phase_B_position = phase_A_position + 120;
    if (phase_B_position > 359) {
        phase_B_position -= 360;
    }
    phase_C_position = phase_B_position + 120;
    if (phase_C_position > 359) {
        phase_C_position -= 360;
    }
}

static inline void checkMotorStuck(uint16_t current_angle, uint16_t desired_angle) {
    if ((getAbsDif(current_angle, desired_angle) < 10) && (commutation_interval > 12000)) {
        stuckcounter++;
        if (stuckcounter > 100) {
            zero_crosses = 0;
            commutation_interval = 30000;
            if (current_angle > desired_angle) {
                forward = 0;
                phase_A_position += 10;
                if (phase_A_position > 359) {
                    phase_A_position -= 360;
                }
                updateSinewavePhases();
            } else {
                forward = 1;
                phase_A_position -= 10;
                if (phase_A_position < 0) {
                    phase_A_position += 360;
                }
                updateSinewavePhases();
            }
            SET_INTERVAL_TIMER_COUNT(30000);
        }
    } else {
        stuckcounter = 0;
    }
}

static inline void resetMotorState(void) {
    running = 0;
    zero_crosses = 0;
    bemfcounter = 0;
    zcfound = 0;
    commutation_interval = 5000;
    old_routine = 1;
}

static inline uint8_t isMotorDesync(void) {
    return (desync_check && zero_crosses > 10 && 
            getAbsDif(last_average_interval, average_interval) > (average_interval >> 1) && 
            average_interval < 2000);
}

#endif /* INC_MOTOR_CONTROL_H_ */