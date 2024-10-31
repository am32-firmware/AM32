#include "stm32h563xx.h"

#include "mcu.h"
#include "stm32h5xx_ll_cortex.h"
#include "clock.h"
#include "dma.h"
#include "flash.h"
#include "power.h"

void mcu_setup()
{
    mcu_setup_flash();
    mcu_setup_core_voltage();
    mcu_setup_clocks();
    mcu_setup_mpu();
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

void mcu_setup_mpu()
{
    // see also https://community.st.com/t5/stm32-mcus/how-to-avoid-a-hardfault-when-icache-is-enabled-on-the-stm32h5/ta-p/630085
    LL_MPU_Disable();
    LL_MPU_ConfigRegion(
        LL_MPU_REGION_NUMBER0,
        LL_MPU_REGION_ALL_RW,
        LL_MPU_ATTRIBUTES_NUMBER0,
        // 0x0900a800,
        // 0x0900c000
        0x08fff800,
        0x08ffffff
        // EEPROM_BASE,
        // EEPROM_BASE + EEPROM_PAGE_SIZE
    );
    LL_MPU_Enable(LL_MPU_CTRL_PRIVILEGED_DEFAULT);
}
