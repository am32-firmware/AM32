/*
 * comparator.c
 *
 *  Created on: Aug. 4, 2026
 *      Author: Nong jun
 */

#include "comparator.h"
#include "common.h"
#include "targets.h"

COMP1_TypeDef* currentComp;

void enableCompInterrupts()
{
    if (rising){
        DDL_COMP1_SetInterrupt(currentComp, DDL_COMP1_EDGE_INT_FALLING);
    }else {
        DDL_COMP1_SetInterrupt(currentComp, DDL_COMP1_EDGE_INT_RISING);
    }
}

uint8_t getCompOutputLevel(void)
{
    return ((currentComp->CR & COMP_CR_VAL)!=0) ? SET : RESET;
}

void maskPhaseInterrupts()
{
    COMP1->CR &= ~(0x7<<COMP_CR_REN_Pos);
    COMP2->CR &= ~(0x7<<COMP_CR_REN_Pos);
    COMP3->CR &= ~(0x7<<COMP_CR_REN_Pos);
    COMP1->ISR |= COMP_ISR_IFLG;
    COMP2->ISR |= COMP_ISR_IFLG;
    COMP3->ISR |= COMP_ISR_IFLG;
}

void changeCompInput()
{
    if (step == 1 || step == 4) {
        currentComp = COMP3;
    }
    if (step == 2 || step == 5) {
        currentComp = COMP2;
    }
    if (step == 3 || step == 6) {
        currentComp = COMP1;
    }

}
