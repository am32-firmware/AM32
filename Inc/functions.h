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

/* Preserve an enclosing critical section (for example a settings write). */
static inline uint32_t irq_save(void)
{
#ifdef WCH
    /* Match WCH's __disable_irq(): CSR 0x800, not the standard mstatus CSR. */
    uint32_t state;
    __asm volatile ("csrr %0, 0x800" : "=r" (state) :: "memory");
    __disable_irq();
    __asm volatile ("" ::: "memory");
#else
    const uint32_t state = __get_PRIMASK();
    __disable_irq();
#endif
    return state;
}

static inline void irq_restore(uint32_t state)
{
#ifdef WCH
    __asm volatile ("csrw 0x800, %0" :: "r" (state) : "memory");
#else
    if (!state) {
        __enable_irq();
    }
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
