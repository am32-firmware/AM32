#include "ten-khz-timer.h"
#include "stm32h563xx.h"
#include "stm32h5xx_ll_bus.h"
#include "stm32h5xx_ll_tim.h"
#include "targets.h"

void ten_khz_timer_initialize(void)
{
    TEN_KHZ_TIMER_ENABLE_CLOCK();
    NVIC_SetPriority(TEN_KHZ_TIMER_IRQn, 3);
    NVIC_EnableIRQ(TEN_KHZ_TIMER_IRQn);
    // TEN_KHZ_TIMER->PSC = 47;
    TEN_KHZ_TIMER->PSC = 470;
    TEN_KHZ_TIMER->ARR = 1000000 / LOOP_FREQUENCY_HZ;
}

void ten_khz_timer_enable(void)
{
    LL_TIM_EnableCounter(TEN_KHZ_TIMER); // 10khz timer
    LL_TIM_GenerateEvent_UPDATE(TEN_KHZ_TIMER);
}

void ten_khz_timer_interrupt_enable()
{
    TEN_KHZ_TIMER->DIER |= (0x1UL << (0U)); // enable interrupt
}
