#include "ten-khz-timer.h"
#include "cmsis_gcc.h"
#include "stm32h563xx.h"
#include "stm32h5xx_ll_bus.h"
#include "stm32h5xx_ll_tim.h"
#include "targets.h"
#include "clock.h"

void ten_khz_timer_initialize(void)
{
    TEN_KHZ_TIMER_ENABLE_CLOCK();
    NVIC_SetPriority(TEN_KHZ_TIMER_IRQn, 3);
    NVIC_EnableIRQ(TEN_KHZ_TIMER_IRQn);
    clock_update_hclk_frequency();

    // 1MHz clock frequency
    TEN_KHZ_TIMER->PSC = (HCLK_FREQUENCY/1000000) - 1;
    // 100us reload period
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

__WEAK void tenKhzRoutine()
{
}