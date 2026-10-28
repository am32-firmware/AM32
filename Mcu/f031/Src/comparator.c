/*
 * comparator.c
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 */

#include "comparator.h"
#include "targets.h"
#include "common.h"


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
    // A coasting blind step skips comStep(), but must still select its sense phase.
    if (step == 1 || step == 4) {
        current_GPIO_PIN = PHASE_C_EXTI_PIN;
        current_GPIO_PORT = PHASE_C_EXTI_PORT;
        current_EXTI_LINE = PHASE_C_EXTI_LINE;
    }
    if (step == 2 || step == 5) {
        current_GPIO_PIN = PHASE_A_EXTI_PIN;
        current_GPIO_PORT = PHASE_A_EXTI_PORT;
        current_EXTI_LINE = PHASE_A_EXTI_LINE;
    }
    if (step == 3 || step == 6) {
        current_GPIO_PIN = PHASE_B_EXTI_PIN;
        current_GPIO_PORT = PHASE_B_EXTI_PORT;
        current_EXTI_LINE = PHASE_B_EXTI_LINE;
    }
  EXTI->RTSR = rising << current_EXTI_LINE;
  EXTI->FTSR = !rising << current_EXTI_LINE;
}
