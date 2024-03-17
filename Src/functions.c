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
		if (x <= in_min)
        return out_min;
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

int findIndex(uint8_t *array, uint8_t size, uint8_t target) 
{
    int i=0;
    while((i<size) && (array[i] != target)) i++;

    return (i<size) ? (i) : (-1);
}

bool searchSequence(uint8_t array[], uint8_t size, uint8_t sequence[], uint8_t sequenceSize) {
    // Loop through the array
    for (int i = 0; i < size; i++) {
        // Check if the current position matches the first element of the sequence
        if (array[i] == sequence[0]) {
            // Check if the sequence matches starting from the current position
            bool found = true;
            for (int j = 0; j < sequenceSize; j++) {
                if (array[(i + j) % size] != sequence[j]) {
                    found = false;
                    break;
                }
            }
            if (found) {
                // Sequence found
                return true;
            }
        }
    }
    // Sequence not found
    return false;
}

uint32_t getAbsDif(int number1, int number2)
{
    int result = number1 - number2;
    if (result < 0) {
        result = -result;
    }
    return (uint32_t)result;
}

/*
  get current value of UTILITY_TIMER timer as 16bit microseconds
 */
static inline uint16_t get_timer_us16(void) {
#if defined(STMICRO)
    return UTILITY_TIMER->CNT;
#elif defined(GIGADEVICES)
    return TIMER_CNT(UTILITY_TIMER);
#elif defined(ARTERY)
    return UTILITY_TIMER->cval;
#else
    #error unsupported MCU
#endif
}

/*
  delay by microseconds, max 65535
 */
void delayMicros(uint32_t micros)
{
    const uint16_t cval_start = get_timer_us16();
    while ((uint16_t)(get_timer_us16() - cval_start) < (uint16_t)micros) {
    }
}

/*
  delay in millis, convenience wrapper around delayMicros
 */
void delayMillis(uint32_t millis)
{
    while (millis-- > 0) {
        delayMicros(1000UL);
    }
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
