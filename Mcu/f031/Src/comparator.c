/*
 * comparator.c
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 */

#include "comparator.h"
#include "common.h"
#include "targets.h"


void maskPhaseInterrupts()
{
    EXTI->IMR &= ~(1 << current_EXTI_LINE);
    EXTI->PR |= 1 << PHASE_A_EXTI_LINE;
    EXTI->PR |= 1 << PHASE_B_EXTI_LINE;
    EXTI->PR |= 1 << PHASE_C_EXTI_LINE;
}

void enableCompInterrupts() { EXTI->IMR |= (1 << current_EXTI_LINE); }

void changeCompInput()
{
  EXTI->RTSR = rising << current_EXTI_LINE;
  EXTI->FTSR = !rising << current_EXTI_LINE;
}
