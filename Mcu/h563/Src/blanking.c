#include "blanking.h"
#include "clock.h"
#include "debug.h"
#include "targets.h"


void blanking_disable()
{
    BLANKING_TIMER->CR1 &= ~TIM_CR1_CEN;
}

void blanking_enable()
{
    BLANKING_TIMER->CNT = 0;
    BLANKING_TIMER->CR1 |= TIM_CR1_CEN;
}

void blanking_initialize()
{
    BLANKING_TIMER_ENABLE_CLOCK();

    clock_update_hclk_frequency();

    // 1MHz clock frequency
    BLANKING_TIMER->PSC = (HCLK_FREQUENCY/1000000) - 1;
    // max reload period
    BLANKING_TIMER->ARR = 0xffff;
    // 25us
    BLANKING_TIMER->CCR1 = 25;

    // update preloaded registers
    BLANKING_TIMER->EGR |= TIM_EGR_UG;


    blanking_interrupt_enable();
    NVIC_SetPriority(BLANKING_TIMER_IRQn, 1);
    NVIC_EnableIRQ(BLANKING_TIMER_IRQn);

}

void blanking_interrupt_enable()
{
    BLANKING_TIMER->DIER |= TIM_DIER_CC1IE; // enable interrupt
}

__WEAK void blanking_interrupt_handler()
{
    if (BLANKING_TIMER->SR & TIM_SR_CC1IF) {
        BLANKING_TIMER->SR &= ~TIM_SR_CC1IF;
        blanking_disable();
    }
}
