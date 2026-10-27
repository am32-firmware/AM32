/*
 * comparator.c
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 */

#include "comparator.h"
#include "common.h"
#include "targets.h"

COMP_TypeDef* active_COMP = COMP1;

uint8_t getCompOutputLevel() { return LL_COMP_ReadOutputLevel(active_COMP); }

void maskPhaseInterrupts()
{
  EXTI->IMR &= ~(1 << 21);
  EXTI->PR = EXTI_LINE;
}

void enableCompInterrupts() { EXTI->IMR |= (1 << 21); }

void changeCompInput()
{
    // A coasting blind step skips comStep(), but must still select its sense phase.
    if (step == 1 || step == 4) {
        COMP->CSR = PHASE_C_COMP;
    }
    if (step == 2 || step == 5) {
        COMP->CSR = PHASE_A_COMP;
    }
    if (step == 3 || step == 6) {
        COMP->CSR = PHASE_B_COMP;
    }

if((average_interval < 400)){
COMP->CSR = COMP->CSR & ~(1<<2);
}else{
COMP->CSR  = COMP->CSR | 1<<2;
}
  EXTI->RTSR = !rising << 21;
  EXTI->FTSR = rising << 21;
}
