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
#include <stdbool.h>

int findIndex(uint8_t *array, uint8_t size, uint8_t target);
bool searchSequence(uint8_t array[], uint8_t size, uint8_t sequence[], uint8_t sequenceSize);
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

#define REVERSE_ARRAY(arr, size) do{\
    int start = 0;\
    int end = size - 1;\
    while (start < end) {\
        int temp = arr[start];\
        arr[start] = arr[end];\
        arr[end] = temp;\
        start++;\
        end--;\
    }\
} while(0)

#define ZERO_ANY(T, a, n) do{\
   T *a_ = (a);\
   uint8_t n_ = (n);\
   for (; n_ > 0; --n_, ++a_)\
     *a_ = (T) { 0 };\
} while (0)

#define ARRAY_SIZE(a) (sizeof (a) / sizeof *(a))
#define ZERO_ANY_A(T, a) ZERO_ANY(T, (a), ARRAY_SIZE(a))

#define ZERO(a, n) do{\
   uint8_t i_ = 0, n_ = (n);\
   for (; i_ < n_; ++i_)\
     (a)[i_] = 0;\
} while (0)

#define ZERO_A(a) ZERO((a), ARRAY_SIZE(a))

#endif /* FUNCTIONS_H_ */
