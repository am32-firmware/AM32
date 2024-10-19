#include "clock.h"
#include "stm32h563xx.h"

void clock_hse_enable()
{
    // turn the HSE on
    RCC->CR |= RCC_CR_HSEON;

    // wait for high speed external oscillator (HSE) to be ready
    while (!(RCC->CR & RCC_CR_HSERDY));
}