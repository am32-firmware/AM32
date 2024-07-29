#include "comparator.h"
#include "targets.h"
#include "common.h"

COMP_TypeDef* active_COMP = COMP2;
uint32_t current_EXTI_LINE = LL_EXTI_LINE_22;

uint8_t getCompOutputLevel()
{
    return (active_COMP->CSR >> 30 & 1);
}

void maskPhaseInterrupts(){
	EXTI->IMR1 &= ~(1 << 22);
	LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_22);
}

void enableCompInterrupts(){
    EXTI->IMR1 |= current_EXTI_LINE;
}

void changeCompInput() {
	if (step == 1 || step == 4) {   // c floating
		LL_COMP_ConfigInputs(active_COMP, PHASE_C_COMP , PHASE_COM_COMP);
	}

	if (step == 2 || step == 5) {     // a floating
		LL_COMP_ConfigInputs(active_COMP, PHASE_A_COMP, PHASE_COM_COMP);
	}

	if (step == 3 || step == 6) {      // b floating
		LL_COMP_ConfigInputs(active_COMP, PHASE_B_COMP, PHASE_COM_COMP);
	}

	if (rising){
		LL_EXTI_DisableRisingTrig_0_31(LL_EXTI_LINE_22);
		LL_EXTI_EnableFallingTrig_0_31(current_EXTI_LINE);
	}else{                          // falling bemf
		LL_EXTI_EnableRisingTrig_0_31(current_EXTI_LINE);
		LL_EXTI_DisableFallingTrig_0_31(LL_EXTI_LINE_22);
	}
}