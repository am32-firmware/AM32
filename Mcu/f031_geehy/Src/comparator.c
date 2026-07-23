/*
 * peripherals.c
 *
 *  Created on: 4. 24, 2026
 *      Author: Nong Jun
 */

#include "comparator.h"
#include "common.h"
#include "targets.h"

COMP1_TypeDef* currentComp;

inline void maskPhaseInterrupts()
{
    DDL_COMP1_SetInterrupt(COMP1, DDL_COMP1_EDGE_INT_DISABLE);
    DDL_COMP1_SetInterrupt(COMP2, DDL_COMP1_EDGE_INT_DISABLE);
    DDL_COMP1_SetInterrupt(COMP3, DDL_COMP1_EDGE_INT_DISABLE);
    DDL_COMP1_ClearFlag_IT(COMP1);
    DDL_COMP1_ClearFlag_IT(COMP2);
    DDL_COMP1_ClearFlag_IT(COMP3);
}

void enableCompInterrupts() {
  if (rising)
  {
     DDL_COMP1_SetInterrupt(currentComp, DDL_COMP1_EDGE_INT_FALLING);
  }
  else 
  {
     DDL_COMP1_SetInterrupt(currentComp, DDL_COMP1_EDGE_INT_RISING);
  }
}

uint8_t getCompOutputLevel(void)
{
    return ((currentComp->CR & COMP_CR_VAL)!=0) ? SET : RESET;
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
