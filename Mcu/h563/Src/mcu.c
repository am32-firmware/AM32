#include "stm32h563xx.h"


#include "can.h"
#include "clock.h"
#include "dma.h"
#include "flash.h"
#include "lan8671.h"
#include "mcu.h"
#include "power.h"
#include "rs485.h"
#include "stm32h5xx_ll_cortex.h"
#include "targets.h"
#include "vreg.h"

void mcu_setup(uint16_t coreFrequencyMHz)
{
    mcu_check_reset_flags();
    mcu_setup_flash();
    mcu_setup_core_voltage();
    mcu_setup_clocks(coreFrequencyMHz);
    mcu_setup_mpu();
    mcu_enable_icache();
    // comment if you don't need it,
    // most applications use dma
    dma_initialize();
    // this should go under board setup, not mcu
    // vreg5V_initialize();
    // vreg5V_enable();
    lan8671_shutdown();
    rs485_initialize();
    can_initialize();
}

// check the reset status register (RSR)
void mcu_check_reset_flags()
{
    uint32_t rsr = RCC->RSR;
    // reset the flag values
    RCC->RSR |= RCC_RSR_RMVF;

    if (rsr & RCC_RSR_IWDGRSTF) {
        // spin
        while (1) {
        }
    }

}

void mcu_setup_clocks(uint16_t coreFrequencyMHz)
{
    clock_hse_enable();

    clock_pll1_set_source(CLOCK_PLL1_SRC_HSE);
    // HSE frequency is 25MHz
    // set prescaler to 25 for 1MHz input clock
    clock_pll1_set_prescaler(AM32_HSE_VALUE/1000000);
    clock_pll1_enable_pclk();
    clock_pll1_set_multiplier(coreFrequencyMHz);
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
        // enable the region where temperature sensor calibrations are stored
        0x08fff800,
        0x08ffffff
    );
    LL_MPU_Enable(LL_MPU_CTRL_PRIVILEGED_DEFAULT);
}
