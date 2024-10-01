#pragma once

#include "stm32h563xx.h"


    // if (LL_TIM_IsActiveFlag_UPDATE(TIM6) == 1) {
    //     LL_TIM_ClearFlag_UPDATE(TIM6);
    //     tenKhzRoutine();
    // }

// struct timer_s;
// typedef void (*timerCallback_p)(struct timer_s* timer);

// typedef struct timer_s
// {
//     TIM_TypeDef* ref;
//     timerCallback_p callback;
// } timer_t;

void ten_khz_timer_initialize();
void ten_khz_timer_enable();
void ten_khz_timer_interrupt_enable();
void tenKhzRoutine();