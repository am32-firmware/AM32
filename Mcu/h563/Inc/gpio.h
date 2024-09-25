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

#define GPIO_SPEED_LOW 0b00
#define GPIO_SPEED_MEDIUM 0b01
#define GPIO_SPEED_FAST 0b10
#define GPIO_SPEED_VERYFAST 0b11


#define GPIOA_CLOCK_ENABLE() do { \
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN; \
} while(0)

#define GPIOB_CLOCK_ENABLE() do { \
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN; \
} while(0)

#define GPIOC_CLOCK_ENABLE() do { \
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN; \
} while(0)

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
void gpio_set_speed(gpio_t* gpio, uint8_t speed);

