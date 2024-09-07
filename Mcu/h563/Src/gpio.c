#include "gpio.h"
#include "stm32h563xx.h"


void gpio_initialize(gpio_t* gpio)
{
    if (!gpio) {
        return;
    }

    switch ((uint32_t)gpio->port) {
        case GPIOA_BASE:
            RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
            break;
        case GPIOB_BASE:
            RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
            break;
        case GPIOC_BASE:
            RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
            break;
#if defined(GPIOD)
        case GPIOD_BASE:
            RCC->AHB2ENR |= RCC_AHB2ENR_GPIODEN;
#endif
#if defined(GPIOE)
        case GPIOE_BASE:
            RCC->AHB2ENR |= RCC_AHB2ENR_GPIOEEN;
#endif
#if defined(GPIOF)
        case GPIOF_BASE:
            RCC->AHB2ENR |= RCC_AHB2ENR_GPIOFEN;
#endif
#if defined(GPIOG)
        case GPIOG_BASE:
            RCC->AHB2ENR |= RCC_AHB2ENR_GPIOGEN;
#endif
#if defined(GPIOH)
        case GPIOH_BASE:
            RCC->AHB2ENR |= RCC_AHB2ENR_GPIOHEN;
#endif
#if defined(GPIOI)
        case GPIOI_BASE:
            RCC->AHB2ENR |= RCC_AHB2ENR_GPIOIEN;
#endif
            break;
    }

    //
    // configure alternate function
    //
    __IO uint32_t* afr;
    uint8_t shift;
    if (gpio->pin > 7) {
        afr = &gpio->port->AFR[1];
        shift = 4*(gpio->pin - 8);
    } else {
        afr = &gpio->port->AFR[0];
        shift = 4*gpio->pin;
    }
    *afr |= gpio->af << shift;

    //
    // configure mode
    //
    uint32_t moder = gpio->port->MODER;
    moder &= ~(0b11 << gpio->pin*2);
    moder |= gpio->mode << gpio->pin*2;
    gpio->port->MODER = moder;
}

void gpio_set(gpio_t* gpio)
{
    gpio->port->ODR |= 1 << gpio->pin;
}

void gpio_reset(gpio_t* gpio)
{
    gpio->port->ODR &= ~(1 << gpio->pin);
}

void gpio_toggle(gpio_t* gpio)
{
    if (gpio->port->ODR & (1 << gpio->pin)) {
        gpio_reset(gpio);
    } else {
        gpio_set(gpio);
    }
}

bool gpio_read(gpio_t* gpio)
{
     return gpio->port->IDR & 1 << gpio->pin;
}

void gpio_set_speed(gpio_t* gpio, uint8_t speed)
{
    gpio->port->OSPEEDR |= (speed & 0b11) << (2*gpio->pin);
}
