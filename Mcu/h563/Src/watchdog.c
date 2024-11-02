#include "watchdog.h"

#include "stm32h5xx.h"

#include "stm32h5xx_ll_bus.h"
#include "stm32h5xx_ll_iwdg.h"

#include "clock.h"


// Bits 15:0 KEY[15:0]: Key value (write only, read 0x0000)
// These bits can be used for several functions, depending upon the value written by the
// application:
// - 0xAAAA: reloads the RL[11:0] value into the IWDCNT down-counter (watchdog refresh),
// and write-protects registers. This value must be written by software at regular intervals,
// otherwise the watchdog generates a reset when the counter reaches 0.
// - 0x5555: enables write-accesses to the registers.
// - 0xCCCC: enables the watchdog (except if the hardware watchdog option is selected) and
// write-protects registers.
// - values different from 0x5555: write-protects registers.
// Note that only IWDG_PR, IWDG_RLR, IWDG_EWCR and IWDG_WINR registers have a
// write-protection mechanism.
void watchdog_enable()
{
    // - 0xCCCC: enables the watchdog (except if the hardware watchdog
    //   option is selected) and write-protects registers.
    IWDG->KR = 0x0000CCCCU;
}

void watchdog_unlock()
{
    // unlock watchdog via key register
    // - 0x5555: enables write-accesses
    //   to the registers.
    IWDG->KR = 0x00005555U;
}

#define IWDG_RLR_MAX (0xfff - 1)


void watchdog_initialize_period(uint32_t period_us)
{

    clock_update_hclk_frequency();

    uint8_t PR = 0;
    uint32_t prescaler = 1 << (PR + 2);

    uint32_t reload_value = ((period_us * (HCLK_FREQUENCY/1000000)) / prescaler);
    while (reload_value > IWDG_RLR_MAX)
    {
        PR++;
        prescaler = 1 << (PR + 2);
        reload_value = ((period_us * (HCLK_FREQUENCY/1000000)) / prescaler);
    }

    watchdog_initialize(prescaler, reload_value);
    // reload value is 12 bit max
    // prescaler goes to 1024



}

void watchdog_initialize(
    iwdgPrescaler_e prescaler,
    uint16_t reload)
{
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_WWDG);

    // unlock watchdog via key register
    watchdog_unlock();

    // set prescaler
    IWDG->PR = LL_IWDG_PRESCALER_16;
    // set reload register
    IWDG->RLR = reload;
    // while (IWDG->SR); // wait for the registers to be updated
    IWDG->KR = 0x0000AAAA;


    watchdog_unlock();

}


void MX_IWDG_Init(void)
{

}

// RLR is a 12 bit value
// reset value is 0xfff
void watchdog_set_reload(uint32_t reload)
{
    while (IWDG->SR & IWDG_SR_RVU)
    {
        // reload value can only be updated
        // when RVU bit is reset
    }
    IWDG->RLR = reload;
}

void watchdog_set_prescaler(iwdgPrescaler_e prescaler)
{
    while (IWDG->SR & IWDG_SR_PVU)
    {
        // The prescaler value can be updated only when PVU bit is reset.
    }
    IWDG->PR = prescaler;
}
