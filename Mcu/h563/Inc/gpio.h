#pragma once

#include "stm32h563xx.h"
#include <stdbool.h>

typedef struct {
    GPIO_TypeDef* port;
    uint8_t pin;
    uint8_t af;
    uint8_t mode;
} gpio_t;

#define GPIO_INPUT 0b00
#define GPIO_OUTPUT 0b01
#define GPIO_AF 0b10
#define GPIO_ANALOG 0b11

#define DEF_GPIO(i, j, k, l) { \
    .port = i, \
    .pin = j, \
    .af = k, \
    .mode = l, \
}

void gpio_initialize(gpio_t* gpio);
void gpio_set(gpio_t* gpio);
void gpio_reset(gpio_t* gpio);
void gpio_toggle(gpio_t* gpio);
bool gpio_read(gpio_t* gpio);
