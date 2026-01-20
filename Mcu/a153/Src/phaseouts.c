/*
 * phaseouts.c
 *
 *  Created on: 14 Nov 2024
 *      Author: Youri
 */

#include "phaseouts.h"

extern char prop_brake_active;

void comStep(char newStep)
{
    switch (newStep) {
    case 1: // A-B
    	//A PWM, B LOW, C FLOAT

    	//Set source using DTSRCSEL. Invert PWM signal if necessary for hardware
#ifdef INVERT_PWM
    	modifyReg16(&FLEXPWM0->DTSRCSEL, PWM_DTSRCSEL_SM0SEL23_MASK | PWM_DTSRCSEL_SM1SEL23_MASK | PWM_DTSRCSEL_SM2SEL23_MASK,
    	    	PWM_DTSRCSEL_SM0SEL23(0) | PWM_DTSRCSEL_SM1SEL23(2) | PWM_DTSRCSEL_SM2SEL23(0));
#else
    	modifyReg16(&FLEXPWM0->DTSRCSEL, PWM_DTSRCSEL_SM0SEL23_MASK | PWM_DTSRCSEL_SM1SEL23_MASK | PWM_DTSRCSEL_SM2SEL23_MASK,
    			PWM_DTSRCSEL_SM0SEL23(1) | PWM_DTSRCSEL_SM1SEL23(2) | PWM_DTSRCSEL_SM2SEL23(0));
#endif

    	if (eepromBuffer.comp_pwm) {
    		//Mask phase C to float
    		modifyReg16(&FLEXPWM0->MASK, 0x777, 0x440);
    	} else {
    		//Mask phase C to float and phase A low FET off
    		modifyReg16(&FLEXPWM0->MASK, 0x777, 0x540);
    	}
        break;

    case 2: // C-B
    	//A FLOAT, B LOW, C PWM

    	//Set source using DTSRCSEL
#ifdef INVERT_PWM
    	modifyReg16(&FLEXPWM0->DTSRCSEL, PWM_DTSRCSEL_SM0SEL23_MASK | PWM_DTSRCSEL_SM1SEL23_MASK | PWM_DTSRCSEL_SM2SEL23_MASK,
    	    	PWM_DTSRCSEL_SM0SEL23(0) | PWM_DTSRCSEL_SM1SEL23(2) | PWM_DTSRCSEL_SM2SEL23(0));
#else
    	modifyReg16(&FLEXPWM0->DTSRCSEL, PWM_DTSRCSEL_SM0SEL23_MASK | PWM_DTSRCSEL_SM1SEL23_MASK | PWM_DTSRCSEL_SM2SEL23_MASK,
    			PWM_DTSRCSEL_SM0SEL23(0) | PWM_DTSRCSEL_SM1SEL23(2) | PWM_DTSRCSEL_SM2SEL23(1));
#endif

    	if (eepromBuffer.comp_pwm) {
    		//Mask phase A to float
    		modifyReg16(&FLEXPWM0->MASK, 0x777, 0x110);
    	} else {
    		//Mask phase A to float and phase C low FET off
    		modifyReg16(&FLEXPWM0->MASK, 0x777, 0x510);
    	}
        break;

    case 3: // C-A
    	//A LOW, B FLOAT, C PWM

    	//Set source using DTSRCSEL
#ifdef INVERT_PWM
    	modifyReg16(&FLEXPWM0->DTSRCSEL, PWM_DTSRCSEL_SM0SEL23_MASK | PWM_DTSRCSEL_SM1SEL23_MASK | PWM_DTSRCSEL_SM2SEL23_MASK,
    	    	PWM_DTSRCSEL_SM0SEL23(2) | PWM_DTSRCSEL_SM1SEL23(0) | PWM_DTSRCSEL_SM2SEL23(0));
#else
    	modifyReg16(&FLEXPWM0->DTSRCSEL, PWM_DTSRCSEL_SM0SEL23_MASK | PWM_DTSRCSEL_SM1SEL23_MASK | PWM_DTSRCSEL_SM2SEL23_MASK,
    			PWM_DTSRCSEL_SM0SEL23(2) | PWM_DTSRCSEL_SM1SEL23(0) | PWM_DTSRCSEL_SM2SEL23(1));
#endif

    	if (eepromBuffer.comp_pwm) {
    		//Mask phase B to float
    		modifyReg16(&FLEXPWM0->MASK, 0x777, 0x220);
    	} else {
    		//Mask phase B to float and phase C low FET off
    		modifyReg16(&FLEXPWM0->MASK, 0x777, 0x620);
    	}
        break;

    case 4: // B-A
    	//A LOW, B PWM, C FLOAT

    	//Set source using DTSRCSEL
#ifdef INVERT_PWM
    	modifyReg16(&FLEXPWM0->DTSRCSEL, PWM_DTSRCSEL_SM0SEL23_MASK | PWM_DTSRCSEL_SM1SEL23_MASK | PWM_DTSRCSEL_SM2SEL23_MASK,
    	    	PWM_DTSRCSEL_SM0SEL23(2) | PWM_DTSRCSEL_SM1SEL23(0) | PWM_DTSRCSEL_SM2SEL23(0));
#else
    	modifyReg16(&FLEXPWM0->DTSRCSEL, PWM_DTSRCSEL_SM0SEL23_MASK | PWM_DTSRCSEL_SM1SEL23_MASK | PWM_DTSRCSEL_SM2SEL23_MASK,
    			PWM_DTSRCSEL_SM0SEL23(2) | PWM_DTSRCSEL_SM1SEL23(1) | PWM_DTSRCSEL_SM2SEL23(0));
#endif

    	if (eepromBuffer.comp_pwm) {
    		//Mask phase C to float
    		modifyReg16(&FLEXPWM0->MASK, 0x777, 0x440);
    	} else {
    		//Mask phase C to float and phase B low FET off
    		modifyReg16(&FLEXPWM0->MASK, 0x777, 0x640);
    	}
        break;

    case 5: // B-C
    	//A FLOAT, B PWM, C LOW

    	//Set source using DTSRCSEL
#ifdef INVERT_PWM
    	modifyReg16(&FLEXPWM0->DTSRCSEL, PWM_DTSRCSEL_SM0SEL23_MASK | PWM_DTSRCSEL_SM1SEL23_MASK | PWM_DTSRCSEL_SM2SEL23_MASK,
    	    	PWM_DTSRCSEL_SM0SEL23(0) | PWM_DTSRCSEL_SM1SEL23(0) | PWM_DTSRCSEL_SM2SEL23(2));
#else
    	modifyReg16(&FLEXPWM0->DTSRCSEL, PWM_DTSRCSEL_SM0SEL23_MASK | PWM_DTSRCSEL_SM1SEL23_MASK | PWM_DTSRCSEL_SM2SEL23_MASK,
    			PWM_DTSRCSEL_SM0SEL23(0) | PWM_DTSRCSEL_SM1SEL23(1) | PWM_DTSRCSEL_SM2SEL23(2));
#endif

    	if (eepromBuffer.comp_pwm) {
    		//Mask phase A to float
    		modifyReg16(&FLEXPWM0->MASK, 0x777, 0x110);
    	} else {
    		//Mask phase A to float and phase B low FET off
    		modifyReg16(&FLEXPWM0->MASK, 0x777, 0x310);
    	}
        break;

    case 6: // A-C
    	//A PWM, B FLOAT, C LOW

    	//Set source using DTSRCSEL
#ifdef INVERT_PWM
    	modifyReg16(&FLEXPWM0->DTSRCSEL, PWM_DTSRCSEL_SM0SEL23_MASK | PWM_DTSRCSEL_SM1SEL23_MASK | PWM_DTSRCSEL_SM2SEL23_MASK,
    	    	PWM_DTSRCSEL_SM0SEL23(0) | PWM_DTSRCSEL_SM1SEL23(0) | PWM_DTSRCSEL_SM2SEL23(2));
#else
    	modifyReg16(&FLEXPWM0->DTSRCSEL, PWM_DTSRCSEL_SM0SEL23_MASK | PWM_DTSRCSEL_SM1SEL23_MASK | PWM_DTSRCSEL_SM2SEL23_MASK,
    			PWM_DTSRCSEL_SM0SEL23(1) | PWM_DTSRCSEL_SM1SEL23(0) | PWM_DTSRCSEL_SM2SEL23(2));
#endif

    	//Mask phase to float
    	if (eepromBuffer.comp_pwm) {
    		//Mask phase B to float
    		modifyReg16(&FLEXPWM0->MASK, 0x777, 0x220);
    	} else {
    		//Mask phase B to float and phase A low FET off
    		modifyReg16(&FLEXPWM0->MASK, 0x777, 0x320);
    	}
        break;
    }

    //Force out event
    modifyReg16(&FLEXPWM0->SM[0].CTRL2, 0, PWM_CTRL2_FORCE(1));
}

void fullBrake()
{ // full braking shorting all low sides
	//A LOW, B LOW, C LOW

	//Set source using DTSRCSEL
#ifdef INVERT_PWM
	modifyReg16(&FLEXPWM0->DTSRCSEL, PWM_DTSRCSEL_SM0SEL23_MASK | PWM_DTSRCSEL_SM1SEL23_MASK | PWM_DTSRCSEL_SM2SEL23_MASK,
			PWM_DTSRCSEL_SM0SEL23(0) | PWM_DTSRCSEL_SM1SEL23(0) | PWM_DTSRCSEL_SM2SEL23(0));
#else
	modifyReg16(&FLEXPWM0->DTSRCSEL, PWM_DTSRCSEL_SM0SEL23_MASK | PWM_DTSRCSEL_SM1SEL23_MASK | PWM_DTSRCSEL_SM2SEL23_MASK,
			PWM_DTSRCSEL_SM0SEL23(2) | PWM_DTSRCSEL_SM1SEL23(2) | PWM_DTSRCSEL_SM2SEL23(2));
#endif

	//Force out event
	modifyReg16(&FLEXPWM0->SM[0].CTRL2, 0, PWM_CTRL2_FORCE(1));
}

/*
 * @brief 	Sets high FET output pins to LOW and enables PWM on the low FET output pins.
 * 			This causes the motor to brake and the brake strength is then controlled by the duty cycle.
 */
void proportionalBrake()
{
	//Set source using DTSRCSEL
#ifdef INVERT_PWM
	modifyReg16(&FLEXPWM0->DTSRCSEL, 0xfff,
			PWM_DTSRCSEL_SM0SEL23(0) | PWM_DTSRCSEL_SM1SEL23(0) | PWM_DTSRCSEL_SM2SEL23(0));
#else
	modifyReg16(&FLEXPWM0->DTSRCSEL, 0xfff,
			PWM_DTSRCSEL_SM0SEL23(1) | PWM_DTSRCSEL_SM1SEL23(1) | PWM_DTSRCSEL_SM2SEL23(1));
#endif

	//Mask high FETs so its LOW, PWM_B are the high FETS
	modifyReg16(&FLEXPWM0->MASK, 0x777, 0x070);

	//Force out event
	modifyReg16(&FLEXPWM0->SM[0].CTRL2, 0, PWM_CTRL2_FORCE(1));
}

/*
 * @brief 	Turn off all FETs so all are floating
 */
void allOff()
{
	//A FLOAT, B FLOAT, C FLOAT

	//Mask all phases
	modifyReg16(&FLEXPWM0->MASK, 0x777, 0x770);

	//Force out event
	modifyReg16(&FLEXPWM0->SM[0].CTRL2, 0, PWM_CTRL2_FORCE(1));
}

void allpwm()
{ // for stepper_sine
	//A PWM, B PWM, C PWM

	//Set source using DTSRCSEL
#ifdef INVERT_PWM
	modifyReg16(&FLEXPWM0->DTSRCSEL, PWM_DTSRCSEL_SM0SEL23_MASK | PWM_DTSRCSEL_SM1SEL23_MASK | PWM_DTSRCSEL_SM2SEL23_MASK,
	    	PWM_DTSRCSEL_SM0SEL23(0) | PWM_DTSRCSEL_SM1SEL23(0) | PWM_DTSRCSEL_SM2SEL23(0));
#else
	modifyReg16(&FLEXPWM0->DTSRCSEL, PWM_DTSRCSEL_SM0SEL23_MASK | PWM_DTSRCSEL_SM1SEL23_MASK | PWM_DTSRCSEL_SM2SEL23_MASK,
			PWM_DTSRCSEL_SM0SEL23(1) | PWM_DTSRCSEL_SM1SEL23(1) | PWM_DTSRCSEL_SM2SEL23(1));
#endif

	//Force out event
	modifyReg16(&FLEXPWM0->SM[0].CTRL2, 0, PWM_CTRL2_FORCE(1));
}

void twoChannelForward()
{
	//A PWM, B LOW, C PWM

	//Set source using DTSRCSEL
#ifdef INVERT_PWM
	modifyReg16(&FLEXPWM0->DTSRCSEL, PWM_DTSRCSEL_SM0SEL23_MASK | PWM_DTSRCSEL_SM1SEL23_MASK | PWM_DTSRCSEL_SM2SEL23_MASK,
	    	PWM_DTSRCSEL_SM0SEL23(0) | PWM_DTSRCSEL_SM1SEL23(2) | PWM_DTSRCSEL_SM2SEL23(0));
#else
	modifyReg16(&FLEXPWM0->DTSRCSEL, PWM_DTSRCSEL_SM0SEL23_MASK | PWM_DTSRCSEL_SM1SEL23_MASK | PWM_DTSRCSEL_SM2SEL23_MASK,
			PWM_DTSRCSEL_SM0SEL23(1) | PWM_DTSRCSEL_SM1SEL23(2) | PWM_DTSRCSEL_SM2SEL23(1));
#endif

	//Force out event
	modifyReg16(&FLEXPWM0->SM[0].CTRL2, 0, PWM_CTRL2_FORCE(1));
}

void twoChannelReverse()
{
	//A LOW, B PWM, C LOW

	//Set source using DTSRCSEL
#ifdef INVERT_PWM
	modifyReg16(&FLEXPWM0->DTSRCSEL, PWM_DTSRCSEL_SM0SEL23_MASK | PWM_DTSRCSEL_SM1SEL23_MASK | PWM_DTSRCSEL_SM2SEL23_MASK,
	    	PWM_DTSRCSEL_SM0SEL23(2) | PWM_DTSRCSEL_SM1SEL23(0) | PWM_DTSRCSEL_SM2SEL23(2));
#else
	modifyReg16(&FLEXPWM0->DTSRCSEL, PWM_DTSRCSEL_SM0SEL23_MASK | PWM_DTSRCSEL_SM1SEL23_MASK | PWM_DTSRCSEL_SM2SEL23_MASK,
			PWM_DTSRCSEL_SM0SEL23(2) | PWM_DTSRCSEL_SM1SEL23(1) | PWM_DTSRCSEL_SM2SEL23(2));
#endif

	//Force out event
	modifyReg16(&FLEXPWM0->SM[0].CTRL2, 0, PWM_CTRL2_FORCE(1));
}
