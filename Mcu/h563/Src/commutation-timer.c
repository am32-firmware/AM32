#include "commutation-timer.h"
#include "stm32h5xx_ll_tim.h"

#include "clock.h"
// this is wrong

// #define COMM_TIMEN (RCC_APB2ENR_TIM1EN)
// #define COMM_TIM TIM1

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

void disableComTimerInt()
{
    COM_TIMER->DIER &= ~((0x1UL << (0U)));
}

void enableComTimerInt()
{
    COM_TIMER->DIER |= (0x1UL << (0U));
}

void setAndEnableComInt(uint16_t time)
{
    COM_TIMER->CNT = 0;
    COM_TIMER->ARR = time;
    COM_TIMER->SR = 0x00;
    COM_TIMER->DIER |= (0x1UL << (0U));
}
