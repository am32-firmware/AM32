#include "commutation-timer.h"

#define COMM_TIMEN (RCC_APB2ENR_TIM1EN)
#define COMM_TIM TIM1

void comm_timer_initialize(void)
{
    RCC->APB2ENR |= COMM_TIMEN;
    COMM_TIM->CR1 |= TIM_CR1_CEN;
}
void comm_timer_set_duty(uint16_t duty)
{
    COMM_TIM->CCR1 = duty;
}
