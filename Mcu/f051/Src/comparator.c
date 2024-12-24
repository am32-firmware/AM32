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
uint8_t medium_speed_set;

void maskPhaseInterrupts()
{
    EXTI->IMR &= ~(1 << 21);
    LL_EXTI_ClearFlag_0_31(EXTI_LINE);
}

void enableCompInterrupts() { EXTI->IMR |= (1 << 21); }

void changeCompInput()
{

    if (step == 1 || step == 4) { // c floating
        COMP->CSR = PHASE_C_COMP;
    }
    if (step == 2 || step == 5) { // a floating
        COMP->CSR = PHASE_A_COMP;
    }
    if (step == 3 || step == 6) { // b floating
        COMP->CSR = PHASE_B_COMP;
    }
if((average_interval < 400)){
COMP->CSR = COMP->CSR & ~(1<<2);

medium_speed_set = 0;
}
if((average_interval > 600)){
COMP->CSR  = COMP->CSR | 1<<2;
medium_speed_set = 1;
}
    if (rising) {
        EXTI->RTSR = 0x0;
        EXTI->FTSR = 0x200000;
    } else {
        // falling bemf
        EXTI->FTSR = 0x0;
        EXTI->RTSR = 0x200000;
        //	hcomp1.Init.TriggerMode = COMP_TRIGGERMODE_IT_RISING;
    }
}
