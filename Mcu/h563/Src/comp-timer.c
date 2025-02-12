#include "comp-timer.h"
#include "clock.h"
#include "debug.h"
#include "targets.h"


void comp_timer_disable()
{
    COMP_TIMER->CR1 &= ~TIM_CR1_CEN;
}

void comp_timer_enable()
{
    COMP_TIMER->CNT = 0;
    COMP_TIMER->CR1 |= TIM_CR1_CEN;
}

void comp_timer_initialize()
{
    COMP_TIMER_ENABLE_CLOCK();

    clock_update_hclk_frequency();

    // max reload period
    COMP_TIMER->ARR = 0xffffffff;

    // update preloaded registers
    COMP_TIMER->EGR |= TIM_EGR_UG;
}
