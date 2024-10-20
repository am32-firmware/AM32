#include "clock.h"
#include "stm32h563xx.h"

uint32_t HCLK_FREQUENCY = 32000000;

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

void clock_system_set_source(uint8_t source)
{
    uint32_t cfgr1 = RCC->CFGR1;
    cfgr1 &= ~(RCC_CFGR1_SW_Msk);
    cfgr1 |= source << RCC_CFGR1_SW_Pos;
    RCC->CFGR1 = cfgr1;

    while (!clock_system_switch_complete())
    {
        // do nothing
    }
    
}

bool clock_system_switch_complete()
{
    uint32_t cfgr1 = RCC->CFGR1;
    uint32_t sw = cfgr1 & RCC_CFGR1_SW_Msk;
    uint32_t sws = (cfgr1 & RCC_CFGR1_SWS_Msk) >> RCC_CFGR1_SWS_Pos;
    return (sw == sws);
}
void clock_pll1_set_source(uint8_t source)
{
    // // set pll clock source to HSE
    RCC->PLL1CFGR |= source << RCC_PLL1CFGR_PLL1SRC_Pos;
}
void clock_pll1_configure_prescaler(uint8_t prescaler)
{
    // The frequency of the reference clock provided to the PLLs (refx_ck) must range from 1 to
    // 16 MHz. The DIVMx dividers of the RCC PLL clock source selection register
    // (RCC_PLL1CFGR) must be properly programmed in order to match this condition.
    // divide by 12, 2MHz for a 24MHz HSE

    // uint8_t prescaler = 25;
    // set the prescaler for pll1 (PLL1M)
    uint32_t pll1cfgr = RCC->PLL1CFGR;
    pll1cfgr &= ~RCC_PLL1CFGR_PLL1M_Msk;
    RCC->PLL1CFGR |= pll1cfgr | (prescaler << RCC_PLL1CFGR_PLL1M_Pos);

}

void clock_update_hclk_frequency()
{

    uint8_t source = (RCC->CFGR1 & RCC_CFGR1_SWS_Msk) >> RCC_CFGR1_SWS_Pos;
    switch (source)
    {
        case (CLOCK_SYS_SRC_HSI):
        {
            uint8_t hsidiv = ((RCC->CR & RCC_CR_HSIDIV_Msk) >> RCC_CR_HSIDIV_Pos);
            uint8_t divider = 0;
            switch (hsidiv) {
                case 0b00:
                {
                    divider = 1;
                    break;
                }
                case 0b01:
                {
                    divider = 2;
                    break;
                }
                case 0b10:
                {
                    divider = 4;
                    break;
                }
                case 0b11:
                {
                    divider = 8;
                    break;
                }
                default:
                {
                    while(1);
                }
            }

            HCLK_FREQUENCY = 64000000 / divider;
            break;
        }
        case (CLOCK_SYS_SRC_HSE):
        {
            HCLK_FREQUENCY = 25000000;
            break;
        }
    }
}
