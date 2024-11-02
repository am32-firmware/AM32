#include "watchdog.h"

#include "stm32h5xx.h"

#include "stm32h5xx_ll_bus.h"
#include "stm32h5xx_ll_iwdg.h"

void watchdog_initialize()
{
    
}

void reloadWatchDogCounter()
{
    LL_IWDG_ReloadCounter(IWDG);
}


void MX_IWDG_Init(void)
{
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_WWDG);
    IWDG->KR = 0x0000CCCCU;
    IWDG->KR = 0x00005555U;
    IWDG->PR = LL_IWDG_PRESCALER_16;
    IWDG->RLR = 4000;
    // while (IWDG->SR); // wait for the registers to be updated
    IWDG->KR = 0x0000AAAA;
}

