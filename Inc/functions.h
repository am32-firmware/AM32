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

/*
  get current value of UTILITY_TIMER timer as 16bit microseconds
 */
static inline uint16_t get_timer_us16(void)
{
#if defined(STMICRO)
    return UTILITY_TIMER->CNT;
#elif defined(GIGADEVICES)
    return TIMER_CNT(UTILITY_TIMER);
#elif defined(ARTERY)
    return UTILITY_TIMER->cval;
#elif defined(NXP)
    // Return nothing since NXP micro-tick works differently
    return 0;
#elif defined(WCH)
    return UTILITY_TIMER->CNT >> 1;
#else
#error unsupported MCU
#endif
}

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
