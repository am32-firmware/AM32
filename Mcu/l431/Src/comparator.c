#include "comparator.h"
#include "targets.h"
#include "common.h"


#ifdef USE_COMP_2
COMP_TypeDef* active_COMP = COMP2;
uint32_t current_EXTI_LINE = LL_EXTI_LINE_22;

uint8_t getCompOutputLevel() { return LL_COMP_ReadOutputLevel(active_COMP);}


void maskPhaseInterrupts(){
	EXTI->IMR1 &= ~(1 << 22);
	LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_22);
}

void enableCompInterrupts(){
    EXTI->IMR1 |= current_EXTI_LINE;
}

void changeCompInput() {
	if (step == 1 || step == 4) {   // c floating
		LL_COMP_ConfigInputs(active_COMP, PHASE_C_COMP , LL_COMP_INPUT_PLUS_IO1);
  }
	if (step == 2 || step == 5) {     // a floating
	LL_COMP_ConfigInputs(active_COMP, PHASE_A_COMP, LL_COMP_INPUT_PLUS_IO1);
	}
	if (step == 3 || step == 6) {      // b floating
    LL_COMP_ConfigInputs(active_COMP, PHASE_B_COMP, LL_COMP_INPUT_PLUS_IO1);
	}
	if (rising){
		  LL_EXTI_DisableRisingTrig_0_31(LL_EXTI_LINE_22);
		  LL_EXTI_EnableFallingTrig_0_31(current_EXTI_LINE);
	}else{                          // falling bemf
		  LL_EXTI_EnableRisingTrig_0_31(current_EXTI_LINE);
		  LL_EXTI_DisableFallingTrig_0_31(LL_EXTI_LINE_22);
	}
}
#endif 
#ifdef USE_COMP_1

COMP_TypeDef* active_COMP = COMP1;
uint32_t current_EXTI_LINE = LL_EXTI_LINE_21;

uint8_t getCompOutputLevel() { return LL_COMP_ReadOutputLevel(active_COMP);}


void maskPhaseInterrupts(){
	EXTI->IMR1 &= ~(1 << 21);
	LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_21);
}

void enableCompInterrupts(){
    EXTI->IMR1 |= current_EXTI_LINE;
}

void changeCompInput() {
	if (step == 1 || step == 4) {   // c floating
		LL_COMP_ConfigInputs(active_COMP, PHASE_C_COMP , LL_COMP_INPUT_PLUS_IO3);
  }
	if (step == 2 || step == 5) {     // a floating
	LL_COMP_ConfigInputs(active_COMP, PHASE_A_COMP, LL_COMP_INPUT_PLUS_IO3);
	}
	if (step == 3 || step == 6) {      // b floating
    LL_COMP_ConfigInputs(active_COMP, PHASE_B_COMP, LL_COMP_INPUT_PLUS_IO3);
	}
	if (rising){
		  LL_EXTI_DisableRisingTrig_0_31(LL_EXTI_LINE_21);
		  LL_EXTI_EnableFallingTrig_0_31(current_EXTI_LINE);
	}else{                          // falling bemf
		  LL_EXTI_EnableRisingTrig_0_31(current_EXTI_LINE);
		  LL_EXTI_DisableFallingTrig_0_31(LL_EXTI_LINE_21);
	}
}
#endif 

