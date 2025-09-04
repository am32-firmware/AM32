/*
 * comparator.c
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 */

#include "comparator.h"

#include "targets.h"

uint8_t getCompOutputLevel()
{
  return CMP->ctrlsts1_bit.cmp1value;
}

void maskPhaseInterrupts()
{
  //	EXTI->IMR &= (0 << 21);
  //	LL_EXTI_ClearFlag_0_31(EXTI_LINE);

  //	EXTI_INTEN &= ~(uint32_t)EXTI_LINE;
  EXINT->inten &= ~(uint32_t)EXTI_LINE;

  //	EXTI_PD = (uint32_t)EXTI_LINE;
  EXINT->intsts = (uint32_t)EXTI_LINE;
}

void enableCompInterrupts()
{
  //  EXTI->IMR |= (1 << 21);
  //	EXTI_INTEN |= (uint32_t)EXTI_LINE;
  EXINT->inten |= (uint32_t)EXTI_LINE;
  // EXTI->PND = EXTI_LINE;
}

void changeCompInput()
{
  //	TIM3->CNT = 0;
  //	HAL_COMP_Stop_IT(&hcomp1);            // done in comparator interrupt
  // routine

  if (step == 1 || step == 4) { // c floating
    //	COMP->CSR = PHASE_C_COMP;
    //	cmp_mode_init(CMP_HIGHSPEED, PHASE_C_COMP, CMP_HYSTERESIS_NO);
    //	COMP->CTRLSTS1 |= PHASE_C_COMP;
    CMP->ctrlsts1 = PHASE_C_COMP;
  }
  if (step == 2 || step == 5) { // a floating
    //	COMP->CSR = PHASE_A_COMP;
    // cmp_mode_init(CMP_HIGHSPEED, PHASE_A_COMP, CMP_HYSTERESIS_NO);
    //	COMP->CTRLSTS1 |= PHASE_A_COMP;
    CMP->ctrlsts1 = PHASE_A_COMP;
  }
  if (step == 3 || step == 6) { // b floating
    //	COMP->CSR = PHASE_B_COMP;
    //	cmp_mode_init(CMP_HIGHSPEED, PHASE_B_COMP, CMP_HYSTERESIS_NO);
    //	COMP->CTRLSTS1 |= PHASE_B_COMP;
    CMP->ctrlsts1 = PHASE_B_COMP;
  }
  if (rising) {
    //	EXTI->RTSR = 0x0;
    //	EXTI->FTSR = 0x200000;
    EXINT->polcfg1 &= ~(uint32_t)EXTI_LINE;
    EXINT->polcfg2 |= (uint32_t)EXTI_LINE;
  } else {
    // falling bemf
    //	EXTI->FTSR = 0x0;
    //	EXTI->RTSR = 0x200000;
    EXINT->polcfg1 |= (uint32_t)EXTI_LINE;
    EXINT->polcfg2 &= ~(uint32_t)EXTI_LINE;
  }
}
