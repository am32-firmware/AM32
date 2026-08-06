/*
 * IO.h
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 */

#include "main.h"

extern volatile char out_put;
extern volatile char inputSet;
extern char dshot;
extern volatile char servoPwm;
extern volatile char send_telemetry;
extern volatile int16_t degrees_celsius;
extern char crawler_mode;

extern uint16_t ADC_raw_volts;
extern uint16_t servo_low_threshold; // anything below this point considered 0
extern uint16_t
    servo_high_threshold; // anything above this point considered 2000 (max)
extern uint16_t servo_neutral;
extern uint8_t servo_dead_band;
extern volatile char inputSet;
extern char dshot;
extern volatile char servoPwm;

void detectInput();
