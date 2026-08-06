/*
 * flexpwm.h
 *
 *  Created on: 14 Nov 2024
 *      Author: Youri
 */

#ifndef MCU_A153_SRC_FLEXPWM_H_
#define MCU_A153_SRC_FLEXPWM_H_

#include "main.h"

/*
 * @brief 	ternary operator result if prescaler is zero
 */
#define SET_PRESCALER_PWM_ZERO { \
	modifyReg16(&FLEXPWM0->MCTRL, 0, PWM_MCTRL_CLDOK_MASK); /*Clear LDOK to prevent it already being set*/ \
	modifyReg16(&FLEXPWM0->SM[0].CTRL, PWM_CTRL_PRSC_MASK, PWM_CTRL_PRSC(0)); /*Sets the prescaler to 1*/ \
	modifyReg16(&FLEXPWM0->SM[1].CTRL, PWM_CTRL_PRSC_MASK, PWM_CTRL_PRSC(0)); \
	modifyReg16(&FLEXPWM0->SM[2].CTRL, PWM_CTRL_PRSC_MASK, PWM_CTRL_PRSC(0)); \
	FLEXPWM0->SM[0].VAL1 = TIM1_AUTORELOAD; /*Set reload value to default*/ \
	FLEXPWM0->SM[1].VAL1 = TIM1_AUTORELOAD; \
	FLEXPWM0->SM[2].VAL1 = TIM1_AUTORELOAD; \
	modifyReg16(&FLEXPWM0->MCTRL, 0, PWM_MCTRL_LDOK_MASK);	/*Load prescaler, modulus and PWM values of all submodules*/ \
}

/*
 * @brief 	ternary operator result if prescaler is non zero
 */
#define SET_PRESCALER_PWM_NON_ZERO(presc) { \
	modifyReg16(&FLEXPWM0->MCTRL, 0, PWM_MCTRL_CLDOK_MASK); /*Clear LDOK to prevent it already being set*/ \
	modifyReg16(&FLEXPWM0->SM[0].CTRL, PWM_CTRL_PRSC_MASK, PWM_CTRL_PRSC(7)); /*Sets the prescaler to 128*/ \
	modifyReg16(&FLEXPWM0->SM[1].CTRL, PWM_CTRL_PRSC_MASK, PWM_CTRL_PRSC(7)); \
	modifyReg16(&FLEXPWM0->SM[2].CTRL, PWM_CTRL_PRSC_MASK, PWM_CTRL_PRSC(7)); \
	FLEXPWM0->SM[0].VAL1 = (presc * 63); /*Set reload value using main_clk, actual prescaler value, sounds ratio and prescaler argument*/ \
	FLEXPWM0->SM[1].VAL1 = (presc * 63); \
	FLEXPWM0->SM[2].VAL1 = (presc * 63); \
	modifyReg16(&FLEXPWM0->MCTRL, 0, PWM_MCTRL_LDOK_MASK);	/*Load prescaler, modulus and PWM values of all submodules*/ \
}

/*
 * @brief 	Used for making the sounds. Sets the PWM prescaler to 128 and adjusts the reload value VAL1 to generate the correct sound frequency.
 * 			The presc value is scaled to to our frequency using the other targets.
 */
#define SET_PRESCALER_PWM(presc) ((presc == 0) ? (SET_PRESCALER_PWM_ZERO) : (SET_PRESCALER_PWM_NON_ZERO(presc)))

/*
 * @brief 	This actually sets the prescaler of the PWM module
 */
#define SET_ACTUAL_PRESCALER_PWM(presc) { \
	modifyReg16(&FLEXPWM0->MCTRL, 0, PWM_MCTRL_CLDOK_MASK); /*Clear LDOK to prevent it already being set*/ \
	modifyReg16(&FLEXPWM0->SM[0].CTRL, PWM_CTRL_PRSC_MASK, PWM_CTRL_PRSC(presc)); \
	modifyReg16(&FLEXPWM0->SM[1].CTRL, PWM_CTRL_PRSC_MASK, PWM_CTRL_PRSC(presc)); \
	modifyReg16(&FLEXPWM0->SM[2].CTRL, PWM_CTRL_PRSC_MASK, PWM_CTRL_PRSC(presc)); \
	modifyReg16(&FLEXPWM0->MCTRL, 0, PWM_MCTRL_LDOK_MASK); \
}

/*
 * @brief 	Sets the reload value of the PWM submodules
 */
#define SET_AUTO_RELOAD_PWM(relval) { \
	modifyReg16(&FLEXPWM0->MCTRL, 0, PWM_MCTRL_CLDOK_MASK); /*Clear LDOK to prevent it already being set*/ \
	FLEXPWM0->SM[0].VAL1 = relval; \
	FLEXPWM0->SM[1].VAL1 = relval; \
	FLEXPWM0->SM[2].VAL1 = relval; \
	modifyReg16(&FLEXPWM0->MCTRL, 0, PWM_MCTRL_LDOK_MASK); \
}

/*
 * @brief 	Sets the duty cycle of all PWM signals
 */
#define SET_DUTY_CYCLE_ALL(newdc) { \
	modifyReg16(&FLEXPWM0->MCTRL, 0, PWM_MCTRL_CLDOK_MASK); /*Clear LDOK to prevent it already being set*/ \
	FLEXPWM0->SM[0].VAL3 = newdc; \
	FLEXPWM0->SM[1].VAL3 = newdc; \
	FLEXPWM0->SM[2].VAL3 = newdc; \
	modifyReg16(&FLEXPWM0->MCTRL, 0, PWM_MCTRL_LDOK_MASK); \
}

void initFlexPWM(void);
void enableFlexPWM(void);

void setPWMCompare1(uint16_t compareone);
void setPWMCompare2(uint16_t comparetwo);
void setPWMCompare3(uint16_t comparethree);
void generatePwmTimerEvent(void);

#endif /* MCU_A153_SRC_FLEXPWM_H_ */
