#pragma once

#include "stm32h563xx.h"
#include "targets.h"

#define DISABLE_COM_TIMER_INT() (COM_TIMER->DIER &= ~((0x1UL << (0U))))
#define ENABLE_COM_TIMER_INT() (COM_TIMER->DIER |= (0x1UL << (0U)))
#define SET_AND_ENABLE_COM_INT(time)                                  \
    (COM_TIMER->CNT = 0, COM_TIMER->ARR = time, COM_TIMER->SR = 0x00, \
    COM_TIMER->DIER |= (0x1UL << (0U)))

void commutation_timer_initialize(void);
void disableComTimerInt();
