#pragma once

#include "stm32h563xx.h"
#include "targets.h"

void comm_timer_initialize(void);
void comm_timer_set_duty(uint16_t duty);
