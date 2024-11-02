#include "watchdog.h"

#include "stm32h5xx.h"

#include "stm32h5xx_ll_bus.h"
#include "stm32h5xx_ll_iwdg.h"



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

void watchdog_initialize(iwdgPrescaler_e prescaler, uint16_t reload)
{

    watchdog_unlock();

}

void reloadWatchDogCounter()
{
    LL_IWDG_ReloadCounter(IWDG);
}


void MX_IWDG_Init(void)
{
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_WWDG);
    // unlock watchdog via key register
    IWDG->KR = 0x0000CCCCU;
    IWDG->KR = 0x00005555U;
    // set prescaler
    IWDG->PR = LL_IWDG_PRESCALER_16;
    // set reload register
    IWDG->RLR = 4000;
    // while (IWDG->SR); // wait for the registers to be updated
    IWDG->KR = 0x0000AAAA;
}

// RLR is a 12 bit value
// reset value is 0xfff
void watchdog_set_rlr(uint32_t rlr)
{
    IWDG->RLR = rlr;
}
