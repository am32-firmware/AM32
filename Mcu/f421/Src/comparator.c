/*
 * comparator.c
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 */

#include "comparator.h"
#include "targets.h"
#include "common.h"

uint8_t getCompOutputLevel() { return CMP->ctrlsts_bit.cmpvalue; }

void maskPhaseInterrupts()
{
    EXINT->inten &= ~EXTI_LINE;
    EXINT->intsts = EXTI_LINE;
}

void enableCompInterrupts() { EXINT->inten |= EXTI_LINE; }

void changeCompInput()
{
//    if (step == 1 || step == 4) { // c floating
//        CMP->ctrlsts = PHASE_C_COMP;
//    }
//    if (step == 2 || step == 5) { // a floating
//        CMP->ctrlsts = PHASE_A_COMP;
//    }
//    if (step == 3 || step == 6) { // b floating
//        CMP->ctrlsts = PHASE_B_COMP;
//    }
//    if (rising) {

//        EXINT->polcfg1 = 0;
//        EXINT->polcfg2 |= (uint32_t)EXTI_LINE;
//    } else {
//        // falling bemf
//        EXINT->polcfg1 |= (uint32_t)EXTI_LINE;
//        EXINT->polcfg2 = 0;
//    }
    if((average_interval < 400)){ 
        //set comp to high speed mode
        CMP->ctrlsts = CMP->ctrlsts & ~(1<<2);
    }
    if((average_interval > 600)){
        //set comp to medium speed mode
        CMP->ctrlsts  = CMP->ctrlsts | 1<<2;
    }
	EXINT->polcfg1 = !rising << 21;
    EXINT->polcfg2 = rising << 21;
}
