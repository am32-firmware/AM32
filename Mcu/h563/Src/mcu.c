#include "stm32h563xx.h"

#include "mcu.h"

#include "clock.h"
#include "dma.h"
#include "flash.h"
#include "power.h"

void mcu_setup()
{
    mcu_setup_flash();
    mcu_setup_core_voltage();
    mcu_setup_clocks();
    mcu_enable_icache();
    // comment if you don't need it,
    // most applications use dma
    dma_initialize();
}

void mcu_setup_clocks()
{
    clock_hse_enable();

    clock_pll1_set_source(CLOCK_PLL1_SRC_HSE);
    // HSE frequency is 25MHz
    // set prescaler to 25 for 1MHz input clock
    clock_pll1_set_prescaler(25);
    clock_pll1_enable_pclk();
    clock_pll1_set_multiplier(250);
    clock_pll1_enable();
    clock_system_set_source(CLOCK_SYS_SRC_PLL1);
    clock_update_hclk_frequency();

    clock_per_set_source(CLOCK_PER_SRC_HSE);
}

void mcu_setup_core_voltage()
{
    // Set core voltage regulator output scaling for maximum performance
    power_set_core_voltage(POWER_VOSCR_0);
}


void mcu_setup_flash()
{
    flash_set_latency(5);
    flash_enable_prefetch();
}

void mcu_enable_icache()
{
    // wait for any ongoing cache invalidation
    while (ICACHE->CR & ICACHE_SR_BUSYF);
    // enable icache miss monitor, hit monitor, and icache itself
    ICACHE->CR |= ICACHE_CR_MISSMEN | ICACHE_CR_HITMEN | ICACHE_CR_EN;
}
