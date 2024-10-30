#include "utility-timer.h"

#include "clock.h"
#include "stm32h563xx.h"
#include "stm32h5xx_ll_tim.h"

#include "targets.h"

void utility_timer_initialize()
{
    UTILITY_TIMER_ENABLE_CLOCK();
    clock_update_hclk_frequency();
    UTILITY_TIMER->PSC = (HCLK_FREQUENCY / 1000000) - 1;
    UTILITY_TIMER->ARR = 0XFFFF;
    LL_TIM_DisableARRPreload(UTILITY_TIMER);
}

void utility_timer_enable()
{
    LL_TIM_EnableCounter(UTILITY_TIMER);
    LL_TIM_GenerateEvent_UPDATE(UTILITY_TIMER);
}
