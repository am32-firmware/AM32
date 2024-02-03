/*
 * functions.c
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 */

#include "functions.h"
#include "targets.h"

// long map(long x, long in_min, long in_max, long out_min, long out_max)
//{
//     if (x < in_min) {
//         x = in_min;
//     }
//     if (x > in_max) {
//         x = in_max;
//     }
//     return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
// }

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
    if (x >= in_max)
        return out_max;
    if (in_min > in_max)
        return map(x, in_max, in_min, out_max, out_min);
    if (out_min == out_max)
        return out_min;
    const long in_mid = (in_min + in_max) >> 1;
    const long out_mid = (out_min + out_max) >> 1;
    if (in_min == in_mid)
        return out_mid;
    if (x <= in_mid)
        return map(x, in_min, in_mid, out_min, out_mid);
    else
        return map(x, in_mid + 1, in_max, out_mid, out_max);
}

int getAbsDif(int number1, int number2)
{
    int result = number1 - number2;
    if (result < 0) {
        result = -result;
    }
    return result;
}

#ifdef STMICRO
void delayMicros(uint32_t micros)
{
    UTILITY_TIMER->CNT = 0;
    while (UTILITY_TIMER->CNT < micros) {
    }
}

void delayMillis(uint32_t millis)
{
    UTILITY_TIMER->CNT = 0;
    UTILITY_TIMER->PSC = CPU_FREQUENCY_MHZ * 100;
    LL_TIM_GenerateEvent_UPDATE(UTILITY_TIMER);
    while (UTILITY_TIMER->CNT < millis * 10) {
    }
    UTILITY_TIMER->PSC = CPU_FREQUENCY_MHZ; // back to micros
    LL_TIM_GenerateEvent_UPDATE(UTILITY_TIMER);
}

#endif

#ifdef GIGADEVICES
void delayMicros(uint32_t micros)
{
    TIMER_CNT(UTILITY_TIMER) = 0;
    while (TIMER_CNT(UTILITY_TIMER) < micros) {
    }
}

void delayMillis(uint32_t millis)
{
    TIMER_CNT(UTILITY_TIMER) = 0;
    timer_prescaler_config(UTILITY_TIMER, 50000, TIMER_PSC_RELOAD_NOW);
    while (TIMER_CNT(UTILITY_TIMER) < (millis * 2)) {
    }
    TIMER_PSC(UTILITY_TIMER) = CPU_FREQUENCY_MHZ; // back to micros
    timer_prescaler_config(UTILITY_TIMER, CPU_FREQUENCY_MHZ,
        TIMER_PSC_RELOAD_NOW);
}
#endif

#ifdef ARTERY
void delayMicros(uint32_t micros)
{
    UTILITY_TIMER->cval = 0;

    while (UTILITY_TIMER->cval < micros) {
    }
}

void delayMillis(uint32_t millis)
{
    UTILITY_TIMER->cval = 0;
    UTILITY_TIMER->div = (CPU_FREQUENCY_MHZ * 100);
    UTILITY_TIMER->swevt |= TMR_OVERFLOW_SWTRIG;
    while (UTILITY_TIMER->cval < (millis * 10)) {
    }
    UTILITY_TIMER->div = CPU_FREQUENCY_MHZ;
    UTILITY_TIMER->swevt |= TMR_OVERFLOW_SWTRIG;
}

#ifdef MCU_AT421
void gpio_mode_QUICK(gpio_type* gpio_periph, uint32_t mode,
    uint32_t pull_up_down, uint32_t pin)
{
    gpio_periph->cfgr = (((((gpio_periph->cfgr))) & (~(((pin * pin) * (0x3UL << (0U)))))) | (((pin * pin) * mode)));
}
void gpio_mode_set(gpio_type* gpio_periph, uint32_t mode, uint32_t pull_up_down,
    uint32_t pin)
{
    gpio_periph->cfgr = (((((gpio_periph->cfgr))) & (~(((pin * pin) * (0x3UL << (0U)))))) | (((pin * pin) * mode)));
    gpio_periph->pull = ((((((gpio_periph->pull))) & (~(((pin * pin) * (0x3UL << (0U)))))) | (((pin * pin) * pull_up_down))));
}
#endif
#ifdef MCU_AT415
void gpio_mode_QUICK(gpio_type* gpio_periph, uint32_t mode,
    uint32_t pull_up_down, uint32_t pin)
{
    __disable_irq();
    gpio_init_type gpio_init_struct;
    gpio_default_para_init(&gpio_init_struct);

    if (GPIO_MODE_MUX) {
    }

    gpio_init_struct.gpio_mode = mode;
    gpio_init_struct.gpio_pins = pin;
    gpio_init_struct.gpio_pull = pull_up_down;

    gpio_init(gpio_periph, &gpio_init_struct);

    __enable_irq();
}
#endif
#endif
