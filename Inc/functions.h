/*
 * functions.h
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 */

#ifndef FUNCTIONS_H_
#define FUNCTIONS_H_

#include "main.h"
#include "targets.h"

uint32_t getAbsDif(int number1, int number2);
void delayMicros(uint32_t micros);
void delayMillis(uint32_t millis);
long map(long x, long in_min, long in_max, long out_min, long out_max);
#ifdef ARTERY
void gpio_mode_QUICK(gpio_type* gpio_periph, uint32_t mode,
                     uint32_t pull_up_down, uint32_t pin);
void gpio_mode_set(gpio_type* gpio_periph, uint32_t mode, uint32_t pull_up_down,
                   uint32_t pin);
#endif
#endif /* FUNCTIONS_H_ */
