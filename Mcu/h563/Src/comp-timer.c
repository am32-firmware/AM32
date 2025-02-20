#include "comp-timer.h"
#include "clock.h"
#include "debug.h"
#include "stm32h563xx.h"
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

    // corresponds to ~period of bridge timer
    // this interrupt will trigger if the comparator has not changed state
    // for the period that the bridge pwm takes
    COMP_TIMER->CCR1 = 10000;
    // update preloaded registers
    COMP_TIMER->EGR |= TIM_EGR_UG;

    // comp_timer_interrupt_enable();
    NVIC_SetPriority(COMP_TIMER_IRQn, 1);
    NVIC_EnableIRQ(COMP_TIMER_IRQn);

}

void comp_timer_interrupt_disable()
{
    COMP_TIMER->DIER &= ~TIM_DIER_CC1IE; // disable interrupt
    COMP_TIMER->SR &= ~TIM_SR_CC1IF; // clear flag
}

void comp_timer_interrupt_enable()
{
    COMP_TIMER->DIER |= TIM_DIER_CC1IE; // enable interrupt
}

__WEAK void comp_timer_interrupt_handler()
{
    if (COMP_TIMER->SR & TIM_SR_CC1IF) {
        COMP_TIMER->SR &= ~TIM_SR_CC1IF;
        debug_toggle_3();
    }
}