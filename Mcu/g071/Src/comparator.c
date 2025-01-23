/*
 * comparator.c
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 */

#include "comparator.h"

#include "common.h"
#include "targets.h"

COMP_TypeDef* active_COMP = COMP2;
uint32_t current_EXTI_LINE = LL_EXTI_LINE_18;
uint8_t medium_speed_set;

uint8_t getCompOutputLevel() { return (active_COMP->CSR >> 30 & 1); }

void maskPhaseInterrupts()
{
    EXTI->IMR1 &= ~(1 << 18);
    EXTI->RPR1 = EXTI_LINE;
    EXTI->FPR1 = EXTI_LINE;
#ifdef N_VARIANT
    EXTI->IMR1 &= ~(1 << 17);
    LL_EXTI_ClearRisingFlag_0_31(LL_EXTI_LINE_17);
    LL_EXTI_ClearFallingFlag_0_31(LL_EXTI_LINE_17);
#endif
}

void enableCompInterrupts() { EXTI->IMR1 |= current_EXTI_LINE; }

void changeCompInput()
{
if((average_interval < 400) && medium_speed_set){
LL_COMP_SetPowerMode(active_COMP, LL_COMP_POWERMODE_HIGHSPEED);
medium_speed_set = 0;
}
if((average_interval > 600) && !medium_speed_set){
LL_COMP_SetPowerMode(active_COMP, LL_COMP_POWERMODE_MEDIUMSPEED);
medium_speed_set = 1;
}
    if (step == 1 || step == 4) { // c floating
#ifdef N_VARIANT
        current_EXTI_LINE = PHASE_C_EXTI_LINE;
        active_COMP = PHASE_C_COMP_NUMBER;
#endif
        LL_COMP_ConfigInputs(active_COMP, PHASE_C_COMP, LL_COMP_INPUT_PLUS_IO3);
    }

    if (step == 2 || step == 5) { // a floating
#ifdef N_VARIANT
        current_EXTI_LINE = PHASE_A_EXTI_LINE;
        active_COMP = PHASE_A_COMP_NUMBER;
#endif
        LL_COMP_ConfigInputs(active_COMP, PHASE_A_COMP, LL_COMP_INPUT_PLUS_IO3);
    }

    if (step == 3 || step == 6) { // b floating
#ifdef N_VARIANT
        current_EXTI_LINE = PHASE_B_EXTI_LINE;
        active_COMP = PHASE_B_COMP_NUMBER;
#endif
        LL_COMP_ConfigInputs(active_COMP, PHASE_B_COMP, LL_COMP_INPUT_PLUS_IO3);
    }
    if (rising) {
        LL_EXTI_DisableRisingTrig_0_31(LL_EXTI_LINE_18);
        LL_EXTI_DisableRisingTrig_0_31(LL_EXTI_LINE_17);
        LL_EXTI_EnableFallingTrig_0_31(current_EXTI_LINE);
    } else { // falling bemf
        LL_EXTI_EnableRisingTrig_0_31(current_EXTI_LINE);
        LL_EXTI_DisableFallingTrig_0_31(LL_EXTI_LINE_17);
        LL_EXTI_DisableFallingTrig_0_31(LL_EXTI_LINE_18);
    }
}

// void changeCompInput() {
//	if (step == 1 || step == 4) {   // c floating
//		COMP2->CSR = 0x000281;
//	}

//	if (step == 2 || step == 5) {     // a floating
//		COMP2->CSR = 0x000261;
//	}

//	if (step == 3 || step == 6) {      // b floating
//		COMP2->CSR = 0x000271;
//	}
//	if (rising){
//		  EXTI->RTSR1 &= ~(LL_EXTI_LINE_18);
//		  EXTI->FTSR1 |= LL_EXTI_LINE_18;
//	}else{                          // falling bemf
//		  EXTI->RTSR1 |= LL_EXTI_LINE_18;
//		  EXTI->FTSR1 &= ~(LL_EXTI_LINE_18);
//	}
//}
// void changeCompInput() {
//	if (step == 1 || step == 4) {   // c floating
// #ifdef INVERTED_COMP
//		COMP2->CSR = (((((COMP2->CSR))) & (~((0xFUL << (4U)) | (0x3UL
//<< (8U))))) | (COMP_COMMON | PHASE_C_COMP)); #else 		COMP2->CSR =
//(((((COMP2->CSR))) & (~((0xFUL << (4U)) | (0x3UL << (8U))))) | (PHASE_C_COMP
//|
// LL_COMP_INPUT_PLUS_IO3)); #endif
//	}

//	if (step == 2 || step == 5) {     // a floating
// #ifdef INVERTED_COMP
//		COMP2->CSR = (((((COMP2->CSR))) & (~((0xFUL << (4U)) | (0x3UL
//<< (8U))))) | (COMP_COMMON | PHASE_A_COMP)); #else 		COMP2->CSR =
//(((((COMP2->CSR))) & (~((0xFUL << (4U)) | (0x3UL << (8U))))) | (PHASE_A_COMP
//|
// LL_COMP_INPUT_PLUS_IO3)); #endif

//	}
//	if (step == 3 || step == 6) {      // b floating
// #ifdef INVERTED_COMP
//		COMP2->CSR = (((((COMP2->CSR))) & (~((0xFUL << (4U)) | (0x3UL
//<< (8U))))) | (COMP_COMMON | PHASE_B_COMP)); #else 		COMP2->CSR =
//(((((COMP2->CSR))) & (~((0xFUL << (4U)) | (0x3UL << (8U))))) | (PHASE_B_COMP
//|
// LL_COMP_INPUT_PLUS_IO3)); #endif

//	}

//	if (rising){
//		  EXTI->RTSR1 &= ~(LL_EXTI_LINE_18);
//		  EXTI->FTSR1 |= LL_EXTI_LINE_18;
//	}else{                          // falling bemf
//		  EXTI->RTSR1 |= LL_EXTI_LINE_18;
//		  EXTI->FTSR1 &= ~(LL_EXTI_LINE_18);

//	}
//}
