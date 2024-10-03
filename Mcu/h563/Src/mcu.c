#include "mcu.h"

#include "stm32h563xx.h"

void mcu_setup()
{
    mcu_setup_core_voltage();
    mcu_enable_prefetch();
    mcu_setup_clocks();
    mcu_enable_icache();
    mcu_enable_dcache();
}

void mcu_setup_clocks()
{

    // ~~~~~~~~ USE HSE ~~~~~~~~~~~~~~
    // turn the HSE on
    RCC->CR |= RCC_CR_HSEON;

    // wait for high speed external oscillator (HSE) to be ready
    while (!(RCC->CR & RCC_CR_HSERDY));

    // // set pll clock source to HSI
    // RCC->PLL1CFGR |= 0b01 << RCC_PLL1CFGR_PLL1SRC_Pos;

    // // set pll clock source to HSE
    RCC->PLL1CFGR |= 0b11 << RCC_PLL1CFGR_PLL1SRC_Pos;

    // // // ~~~~~~~~ \USE HSE ~~~~~~~~~~~~~

    // The frequency of the reference clock provided to the PLLs (refx_ck) must range from 1 to
    // 16 MHz. The DIVMx dividers of the RCC PLL clock source selection register
    // (RCC_PLL1CFGR) must be properly programmed in order to match this condition.
    // divide by 12, 2MHz for a 24MHz HSE
    uint32_t pll1cfgr = RCC->PLL1CFGR;
    pll1cfgr &= ~RCC_PLL1CFGR_PLL1M_Msk;
    RCC->PLL1CFGR |= pll1cfgr | (12 << RCC_PLL1CFGR_PLL1M_Pos);

    // enable PLL1 p_clk output for use as SYSCLK
    RCC->PLL1CFGR |= 1 << RCC_PLL1CFGR_PLL1PEN_Pos;

    // set pll multiplier
    uint32_t pll1divr = RCC->PLL1DIVR;
    // pll1divr &= ~(RCC_PLL1DIVR_PLL1N_Msk);
    pll1divr &= ~(RCC_PLL1DIVR_PLL1N_Msk);
    // RCC->PLL1DIVR = pll1divr | ((250-1) << RCC_PLL1DIVR_PLL1N_Pos);
    // pll1divr |= ((SYSCLK_FREQUENCY/1000000 - 1) << RCC_PLL1DIVR_PLL1N_Pos);
    // RCC->PLL1DIVR = pll1divr | (249 << RCC_PLL1DIVR_PLL1N_Pos);
    RCC->PLL1DIVR = pll1divr | (249 << RCC_PLL1DIVR_PLL1N_Pos);

    // turn the pll on
    RCC->CR |= RCC_CR_PLL1ON;

    // wait for pll to be ready
    while (!(RCC->CR & RCC_CR_PLL1RDY));

    // switch system clock to pll1 p_clk
    RCC->CFGR1 |= 0b11 << RCC_CFGR1_SW_Pos;
}

void mcu_setup_core_voltage()
{
    // Set core voltage regulator output scaling for maximum performance
    PWR->VOSCR |= 0b11 << PWR_VOSCR_VOS_Pos;
    while (!(PWR->VOSSR & PWR_VOSSR_VOSRDY));
    while (!(PWR->VOSSR & PWR_VOSSR_ACTVOSRDY));
}

void mcu_enable_prefetch()
{
    // set flash latency to 5 wait states for 250MHz SYSCLK
    FLASH->ACR |= 5 << FLASH_ACR_LATENCY_Pos;

    // enable prefetch buffer
    FLASH->ACR |= FLASH_ACR_PRFTEN;
}

void mcu_enable_icache()
{
    // wait for any ongoing cache invalidation
    while (ICACHE->CR & ICACHE_SR_BUSYF);
    // enable icache miss monitor, hit monitor, and icache itself
    ICACHE->CR |= ICACHE_CR_MISSMEN | ICACHE_CR_HITMEN | ICACHE_CR_EN;
}

void mcu_enable_dcache()
{

}