#include "clock.h"
#include "stm32h563xx.h"

void clock_hse_enable()
{
    // turn the HSE on
    RCC->CR |= RCC_CR_HSEON;

    // wait for high speed external oscillator (HSE) to be ready
    while (!(RCC->CR & RCC_CR_HSERDY));
}

// hsi divider can only be configured when HSI
// is NOT selected for any pll input
void clock_hsi_config_divider(uint8_t hsidiv)
{
    // at startup, system clock is HSI = 64MHz / 2 = 32MHz
    // the HSI divider at startup is 2
    RCC->CR &= ~(RCC_CR_HSIDIV_Msk);

    // 00: division by 1, hsi_ck, hsi_ker_ck = 64 MHz
    // 01: division by 2, hsi_ck, hsi_ker_ck = 32 MHz (default after reset)
    // 10: division by 4, hsi_ck, hsi_ker_ck = 16 MHz
    // 11: division by 8, hsi_ck, hsi_ker_ck = 8 MHz
    RCC->CR |= hsidiv << RCC_CR_HSIDIV_Pos;

    while (!(RCC->CR & RCC_CR_HSIDIVF))
    {
        // wait for hsi to switch over
    }
}