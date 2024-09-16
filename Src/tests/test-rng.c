#include "functions.h"
#include "targets.h"

uint32_t randNumber;

int main()
{
    RCC->CR |= RCC_CR_HSI48ON;
    while (!(RCC->CR & RCC_CR_HSI48RDY));

    RCC->AHB2ENR |= RCC_AHB2ENR_RNGEN;
    RNG->CR |= RNG_CR_RNGEN;
    // for (int i = 0; i < 16; i++) {
    //     asm("nop");
    // }

    while(1) {
        while (!(RNG->SR & RNG_SR_DRDY));
        randNumber = RNG->DR;
    }
}