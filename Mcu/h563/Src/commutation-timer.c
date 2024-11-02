#include "commutation-timer.h"

#include "stm32h5xx_ll_tim.h"

#include "clock.h"

void commutation_timer_initialize(void)
{
    COM_TIMER_ENABLE_CLOCK();

    clock_update_hclk_frequency();
    // 2MHz clock frequency
    COM_TIMER->PSC = ((HCLK_FREQUENCY/1000000)/2) - 1;
    COM_TIMER->ARR = 4000;
    NVIC_SetPriority(COM_TIMER_IRQ, 0);
    NVIC_EnableIRQ(COM_TIMER_IRQ);
    LL_TIM_EnableARRPreload(COM_TIMER);
}
