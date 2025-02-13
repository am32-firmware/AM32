#include "commutation-timer.h"

#include "stm32h5xx_ll_tim.h"

#include "clock.h"

void commutation_timer_disable()
{
    COM_TIMER->CR1 &= ~TIM_CR1_CEN;
}

void commutation_timer_enable()
{
    COM_TIMER->CNT = 0;
    COM_TIMER->CR1 |= TIM_CR1_CEN;
}

void commutation_timer_initialize()
{
    COM_TIMER_ENABLE_CLOCK();

    clock_update_hclk_frequency();

    // max reload period
    COM_TIMER->ARR = 0xffffffff;

    // update preloaded registers
    COM_TIMER->EGR |= TIM_EGR_UG;
}
