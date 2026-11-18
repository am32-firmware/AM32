/*
 * flexpwm.c
 *
 *  Created on: 14 Nov 2024
 *      Author: Youri
 */

#include "flexpwm.h"
#include "functions.h"

/*
 * @brief 	Initializes the PWM for three output phases
 * 			Runs on the main_clk which can be configured to be 12MHz, 96MHz, 192MHz or 16kHz
 * 			Is configured to run at main_clk of 192MHz (see SystemClock_Config()) and reloads at 24kHz (defined by TIM1_AUTORELOAD value)
 * 			PWM_A controls the LOW FET. PWM_B control the HIGH FET. Define INVERT_PWM if the hardware is the other way around.
 */
void initFlexPWM(void)
{
	//Unlock clock configuration registers access
	modifyReg32(&SYSCON->CLKUNLOCK, SYSCON_CLKUNLOCK_UNLOCK(1), 0);

	//Enable peripheral clocks
	MRCC0->MRCC_GLB_CC0_SET = MRCC_MRCC_GLB_RST0_FLEXPWM0(1);

	//Release peripherals from reset
	MRCC0->MRCC_GLB_RST0_SET = MRCC_MRCC_GLB_RST0_FLEXPWM0(1);

	//Enable PWM sub-clock of sub-module 0, 1 and 2
	modifyReg32(&SYSCON->PWM0SUBCTL,
			SYSCON_PWM0SUBCTL_CLK0_EN_MASK | SYSCON_PWM0SUBCTL_CLK1_EN_MASK | SYSCON_PWM0SUBCTL_CLK2_EN_MASK,
			SYSCON_PWM0SUBCTL_CLK0_EN(1) | SYSCON_PWM0SUBCTL_CLK1_EN(1) | SYSCON_PWM0SUBCTL_CLK2_EN(1));

	//Freeze clock configuration registers access
	modifyReg32(&SYSCON->CLKUNLOCK, 0, SYSCON_CLKUNLOCK_UNLOCK(1));

	//Initialize submodules 0, 1 and 2 identically
	for (int submodule = 0; submodule <= 2; submodule++) {
		//Set prescaler to 1 and that buffered registers are set at the next PWM reload if MCTRL[LDOK] is set
		modifyReg16(&FLEXPWM0->SM[submodule].CTRL,
				PWM_CTRL_PRSC_MASK | PWM_CTRL_LDMOD_MASK,
				PWM_CTRL_PRSC(0) | PWM_CTRL_LDMOD(0));

		//Enable complementary channel operation
		//FRCEN must be disabled. Otherwise the motor has hiccups during phase switching
		modifyReg16(&FLEXPWM0->SM[submodule].CTRL2,
				PWM_CTRL2_INDEP_MASK | PWM_CTRL2_FRCEN_MASK,
				PWM_CTRL2_INDEP(0) | PWM_CTRL2_FRCEN(0));

		//Set PWM timing. PWM is complementary so PWMB is the reverse of PWMA ignoring the dead-time
		FLEXPWM0->SM[submodule].INIT = 0;					//set initial value
		FLEXPWM0->SM[submodule].VAL1 = TIM1_AUTORELOAD; 	//set reload value
		FLEXPWM0->SM[submodule].VAL2 = 0; 					//set rising flank of PWMA
		FLEXPWM0->SM[submodule].VAL3 = TIM1_AUTORELOAD / 2; //set falling flank of PWMA

		//Set dead time
		FLEXPWM0->SM[submodule].DTCNT0 = PWM_DTCNT0_DTCNT0(DEAD_TIME);	//PWMA deadtime
		FLEXPWM0->SM[submodule].DTCNT1 = PWM_DTCNT1_DTCNT1(DEAD_TIME);	//PWMB deadtime

		//Disable fault protection as it is not necessary because independent mode is never configured
		modifyReg16(&FLEXPWM0->SM[submodule].DISMAP[0],
				PWM_DISMAP_DIS0X_MASK | PWM_DISMAP_DIS0B_MASK | PWM_DISMAP_DIS0A_MASK, 0);

#ifdef USE_INVERTED_LOW
		//Invert output polarity of PWM A (low FET)
		modifyReg16(&FLEXPWM0->SM[submodule], PWM_OCTRL_POLA_MASK, PWM_OCTRL_POLA(1));
#endif

#ifdef USE_INVERTED_HIGH
		//Invert output polarity of PWM B (high FET)
		modifyReg16(&FLEXPWM0->SM[submodule], PWM_OCTRL_POLB_MASK, PWM_OCTRL_POLB(1));
#endif
	}

	//Set that PWM23 (VAL2 and VAL3) is used for complementary PWM generation
	modifyReg16(&FLEXPWM0->MCTRL, PWM_MCTRL_IPOL_MASK, 0);

	//Set that inverted PWM23 is passed to dead-time logic for all submodules
	modifyReg16(&FLEXPWM0->DTSRCSEL, 0xfff,
			PWM_DTSRCSEL_SM0SEL23(1) | PWM_DTSRCSEL_SM1SEL23(1) | PWM_DTSRCSEL_SM2SEL23(1));

	//Set SWCOUT23 to HIGH
	modifyReg16(&FLEXPWM0->SWCOUT, 0,
			PWM_SWCOUT_SM0OUT23_MASK | PWM_SWCOUT_SM1OUT23_MASK | PWM_SWCOUT_SM2OUT23_MASK);

	//Set that the force signal from submodule 0 also forces updates to the other submodules.
	//Note that submodule 0 should have a 0 in FORCE_SEL for this to work
	modifyReg16(&FLEXPWM0->SM[0].CTRL2, PWM_CTRL2_FORCE_SEL_MASK, 0);
	modifyReg16(&FLEXPWM0->SM[1].CTRL2, PWM_CTRL2_FORCE_SEL_MASK, PWM_CTRL2_FORCE_SEL(1));
	modifyReg16(&FLEXPWM0->SM[2].CTRL2, PWM_CTRL2_FORCE_SEL_MASK, PWM_CTRL2_FORCE_SEL(1));

	//Set that master sync from submodule 0 causes timer counter initialization.
	//Note that submodule 0 must use the local sync signal.
	modifyReg16(&FLEXPWM0->SM[0].CTRL2, PWM_CTRL2_INIT_SEL_MASK, 0);
	modifyReg16(&FLEXPWM0->SM[1].CTRL2, PWM_CTRL2_INIT_SEL_MASK, PWM_CTRL2_INIT_SEL(2));
	modifyReg16(&FLEXPWM0->SM[2].CTRL2, PWM_CTRL2_INIT_SEL_MASK, PWM_CTRL2_INIT_SEL(2));

	//MUX low FET pin to PWM
    modifyReg32(&PHASE_A_PORT_LOW->PCR[PHASE_A_PIN_LOW], PORT_PCR_MUX_MASK, PORT_PCR_MUX(5));
	modifyReg32(&PHASE_B_PORT_LOW->PCR[PHASE_B_PIN_LOW], PORT_PCR_MUX_MASK, PORT_PCR_MUX(5));
	modifyReg32(&PHASE_C_PORT_LOW->PCR[PHASE_C_PIN_LOW], PORT_PCR_MUX_MASK, PORT_PCR_MUX(5));

	//MUX high FET pin to PWM
    modifyReg32(&PHASE_A_PORT_HIGH->PCR[PHASE_A_PIN_HIGH], PORT_PCR_MUX_MASK, PORT_PCR_MUX(5));
	modifyReg32(&PHASE_B_PORT_HIGH->PCR[PHASE_B_PIN_HIGH], PORT_PCR_MUX_MASK, PORT_PCR_MUX(5));
    modifyReg32(&PHASE_C_PORT_HIGH->PCR[PHASE_C_PIN_HIGH], PORT_PCR_MUX_MASK, PORT_PCR_MUX(5));

    //Mask all phases by default
    modifyReg16(&FLEXPWM0->MASK, 0x777, 0x770);

    //Force out event to load all buffered registers
    modifyReg16(&FLEXPWM0->SM[0].CTRL2, 0, PWM_CTRL2_FORCE(1));

	//Enable all six PWM outputs
	FLEXPWM0->OUTEN = PWM_OUTEN_PWMA_EN_MASK | PWM_OUTEN_PWMB_EN_MASK;
}

void enableFlexPWM(void)
{
	//Load prescaler, modulus and PWM values of all three submodules
	modifyReg16(&FLEXPWM0->MCTRL, 0, PWM_MCTRL_LDOK_MASK);

	//Turn on clock source of all three submodules
	modifyReg16(&FLEXPWM0->MCTRL, 0, PWM_MCTRL_RUN_MASK);
}

inline void setPWMCompare1(uint16_t compareone)
{
	//Set PWM falling flank of submodule 0
	FLEXPWM0->SM[0].VAL3 = compareone;
}

inline void setPWMCompare2(uint16_t comparetwo)
{
	//Set PWM falling flank of submodule 1
	FLEXPWM0->SM[1].VAL3 = comparetwo;
}

inline void setPWMCompare3(uint16_t comparethree)
{
	//Set PWM falling flank of submodule 2
	FLEXPWM0->SM[2].VAL3 = comparethree;
}

inline void generatePwmTimerEvent()
{
	//TODO fix this
	//Load prescaler, modulus and PWM values of all three submodules
//	modifyReg16(&FLEXPWM0->MCTRL, 0, PWM_MCTRL_LDOK_MASK);
//
//	//Force update the PWM submodules to re-initialize the counter and output pins
//	modifyReg16(&FLEXPWM0->SM[0].CTRL2, 0, PWM_CTRL2_FORCE(1));
}
