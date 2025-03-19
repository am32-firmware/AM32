/*
 * phaseouts.c
 *
 *  Created on: Apr 22, 2020
 *      Author: Alka
 *      Modified by TempersLee June,21 2024
 */
#include "phaseouts.h"

#include "functions.h"
#include "targets.h"
#include "common.h"

extern char prop_brake_active;

#ifndef PWM_ENABLE_BRIDGE

#ifdef USE_INVERTED_LOW
#pragma message("using inverted low side output")
#define LOW_BITREG_ON BCR
#define LOW_BITREG_OFF BSHR
#else
#define LOW_BITREG_ON BSHR
#define LOW_BITREG_OFF BCR
#endif

#ifdef USE_INVERTED_HIGH
#pragma message("using inverted high side output")
#define HIGH_BITREG_OFF BSHR
#else
#define HIGH_BITREG_OFF BCR
#endif

/*
 * T1CH1---PA8       CH
 * T1CH2---PA9       BH
 * T1CH3---PA10      AH
 * T1CH1N--PA7       CL
 * T1CH2N--PB0       BL
 * T1CH3N--PB1       AL
 * */


void proportionalBrake()
{
    // turn all HIGH channels off for ABC
    PHASE_A_GPIO_PORT_HIGH->CFGHR &= ~(0xf<<8); PHASE_A_GPIO_PORT_HIGH->CFGHR |= (0x3<<8);
    PHASE_A_GPIO_PORT_HIGH->HIGH_BITREG_OFF = PHASE_A_GPIO_HIGH;

    PHASE_B_GPIO_PORT_HIGH->CFGHR &= ~(0xf<<4); PHASE_B_GPIO_PORT_HIGH->CFGHR |= (0x3<<4);
    PHASE_B_GPIO_PORT_HIGH->HIGH_BITREG_OFF = PHASE_B_GPIO_HIGH;

    PHASE_C_GPIO_PORT_HIGH->CFGHR &= ~(0xf<<0); PHASE_C_GPIO_PORT_HIGH->CFGHR |= (0x3<<0);
    PHASE_C_GPIO_PORT_HIGH->HIGH_BITREG_OFF = PHASE_C_GPIO_HIGH;


    // set low channel to PWM, duty cycle will now control braking
    PHASE_A_GPIO_PORT_LOW->CFGLR  &= ~(0xf<<4); PHASE_A_GPIO_PORT_LOW->CFGLR|= (0xb<<4);
    PHASE_B_GPIO_PORT_LOW->CFGLR  &= ~(0xf<<0); PHASE_B_GPIO_PORT_LOW->CFGLR|= (0xb<<0);
    PHASE_C_GPIO_PORT_LOW->CFGLR  &= ~(0xf<<28); PHASE_C_GPIO_PORT_LOW->CFGLR|= (0xb<<28);
}



void phaseBPWM()
{
    if(!eepromBuffer.comp_pwm)
    {  // for future
        PHASE_B_GPIO_PORT_LOW->CFGLR  &= ~(0xf<<0); PHASE_B_GPIO_PORT_LOW->CFGLR|= (0x3<<0);
        PHASE_B_GPIO_PORT_LOW->LOW_BITREG_OFF = PHASE_B_GPIO_LOW; //low close
    }
    else
    {
        PHASE_B_GPIO_PORT_LOW->CFGLR  &= ~(0xf<<0); PHASE_B_GPIO_PORT_LOW->CFGLR|= (0xb<<0); //low pwm
    }
    PHASE_B_GPIO_PORT_HIGH->CFGHR &= ~(0xf<<4); PHASE_B_GPIO_PORT_HIGH->CFGHR |= (0xb<<4);   //high pwm

}
void phaseBFLOAT()
{
    PHASE_B_GPIO_PORT_LOW->CFGLR  &= ~(0xf<<0); PHASE_B_GPIO_PORT_LOW->CFGLR|= (0x3<<0);
     PHASE_B_GPIO_PORT_LOW->LOW_BITREG_OFF = PHASE_B_GPIO_LOW;  //low close

     PHASE_B_GPIO_PORT_HIGH->CFGHR &= ~(0xf<<4); PHASE_B_GPIO_PORT_HIGH->CFGHR |= (0x3<<4);
     PHASE_B_GPIO_PORT_HIGH->HIGH_BITREG_OFF = PHASE_B_GPIO_HIGH; //high close

}
void phaseBLOW()
{
    // low mosfet on
    PHASE_B_GPIO_PORT_LOW->CFGLR  &= ~(0xf<<0); PHASE_B_GPIO_PORT_LOW->CFGLR|= (0x3<<0);
    PHASE_B_GPIO_PORT_LOW->LOW_BITREG_ON = PHASE_B_GPIO_LOW;

    // high close
    PHASE_B_GPIO_PORT_HIGH->CFGHR &= ~(0xf<<4); PHASE_B_GPIO_PORT_HIGH->CFGHR |= (0x3<<4);
    PHASE_B_GPIO_PORT_HIGH->HIGH_BITREG_OFF = PHASE_B_GPIO_HIGH;
}

//////////////////////////////PHASE
/// 2//////////////////////////////////////////////////

void phaseCPWM()
{
    if (!eepromBuffer.comp_pwm)
    {
        PHASE_C_GPIO_PORT_LOW->CFGLR  &= ~(0xf<<28); PHASE_C_GPIO_PORT_LOW->CFGLR |= (0x3<<28);
        PHASE_C_GPIO_PORT_LOW->LOW_BITREG_OFF = PHASE_C_GPIO_LOW;
    }
    else
    {
        PHASE_C_GPIO_PORT_LOW->CFGLR  &= ~(0xf<<28); PHASE_C_GPIO_PORT_LOW->CFGLR |= (0xb<<28);
    }
    PHASE_C_GPIO_PORT_HIGH->CFGHR &= ~(0xf<<0); PHASE_C_GPIO_PORT_HIGH->CFGHR |= (0xb<<0);
}

void phaseCFLOAT()
{
    // floating
    PHASE_C_GPIO_PORT_LOW->CFGLR  &= ~(0xf<<28); PHASE_C_GPIO_PORT_LOW->CFGLR |= (0x3<<28);
    PHASE_C_GPIO_PORT_LOW->LOW_BITREG_OFF = PHASE_C_GPIO_LOW;  //low close

    PHASE_C_GPIO_PORT_HIGH->CFGHR &= ~(0xf<<0); PHASE_C_GPIO_PORT_HIGH->CFGHR |= (0x3<<0);
    PHASE_C_GPIO_PORT_HIGH->HIGH_BITREG_OFF = PHASE_C_GPIO_HIGH;//high close
}

void phaseCLOW()
{
    PHASE_C_GPIO_PORT_LOW->CFGLR  &= ~(0xf<<28); PHASE_C_GPIO_PORT_LOW->CFGLR |= (0x3<<28);
    PHASE_C_GPIO_PORT_LOW->LOW_BITREG_ON = PHASE_C_GPIO_LOW; //low on

    PHASE_C_GPIO_PORT_HIGH->CFGHR &= ~(0xf<<0); PHASE_C_GPIO_PORT_HIGH->CFGHR |= (0x3<<0);
    PHASE_C_GPIO_PORT_HIGH->HIGH_BITREG_OFF = PHASE_C_GPIO_HIGH; //high close
}

///////////////////////////////////////////////PHASE 3
////////////////////////////////////////////////////

void phaseAPWM()
{
    if (!eepromBuffer.comp_pwm)
    {
        PHASE_A_GPIO_PORT_LOW->CFGLR  &= ~(0xf<<4); PHASE_A_GPIO_PORT_LOW->CFGLR|= (0x3<<4);
        PHASE_A_GPIO_PORT_LOW->LOW_BITREG_OFF = PHASE_A_GPIO_LOW; //low close
    }
    else
    {
        PHASE_A_GPIO_PORT_LOW->CFGLR  &= ~(0xf<<4); PHASE_A_GPIO_PORT_LOW->CFGLR|= (0xb<<4);//low pwm
    }
    PHASE_A_GPIO_PORT_HIGH->CFGHR &= ~(0xf<<8); PHASE_A_GPIO_PORT_HIGH->CFGHR |= (0xb<<8); //high pwm
}

void phaseAFLOAT()
{
    PHASE_A_GPIO_PORT_LOW->CFGLR  &= ~(0xf<<4); PHASE_A_GPIO_PORT_LOW->CFGLR|= (0x3<<4);
    PHASE_A_GPIO_PORT_LOW->LOW_BITREG_OFF = PHASE_A_GPIO_LOW;  //low close

    PHASE_A_GPIO_PORT_HIGH->CFGHR &= ~(0xf<<8); PHASE_A_GPIO_PORT_HIGH->CFGHR |= (0x3<<8);
    PHASE_A_GPIO_PORT_HIGH->HIGH_BITREG_OFF = PHASE_A_GPIO_HIGH; //high close
}

void phaseALOW()
{
    PHASE_A_GPIO_PORT_LOW->CFGLR  &= ~(0xf<<4); PHASE_A_GPIO_PORT_LOW->CFGLR|= (0x3<<4);
    PHASE_A_GPIO_PORT_LOW->LOW_BITREG_ON = PHASE_A_GPIO_LOW;   // low on

    PHASE_A_GPIO_PORT_HIGH->CFGHR &= ~(0xf<<8); PHASE_A_GPIO_PORT_HIGH->CFGHR |= (0x3<<8);
    PHASE_A_GPIO_PORT_HIGH->HIGH_BITREG_OFF = PHASE_A_GPIO_HIGH; //high close
}

#else

//////////////////////////////////PHASE 1//////////////////////
void phaseBPWM()
{
    if (!eepromBuffer.comp_pwm)
    {
        // for future
        // gpio_mode_QUICK(PHASE_B_GPIO_PORT_LOW, GPIO_MODE_OUTPUT,
        // GPIO_PULL_NONE, PHASE_B_GPIO_LOW);
        // PHASE_B_GPIO_PORT_LOW->LOW_BITREG_OFF = PHASE_B_GPIO_LOW;
    }
    else
    {
        PHASE_B_GPIO_PORT_LOW->CFGLR  &= ~(0xf<<0); PHASE_B_GPIO_PORT_LOW->CFGLR|= (0x3<<0);
        PHASE_B_GPIO_PORT_LOW->BSHR = PHASE_B_GPIO_LOW; //low on
    }
    PHASE_B_GPIO_PORT_HIGH->CFGHR &= ~(0xf<<4); PHASE_B_GPIO_PORT_HIGH->CFGHR |= (0xb<<4);   //high pwm
}

void phaseBFLOAT()
{
    PHASE_B_GPIO_PORT_LOW->CFGLR  &= ~(0xf<<0); PHASE_B_GPIO_PORT_LOW->CFGLR|= (0x3<<0);
     PHASE_B_GPIO_PORT_LOW->LOW_BITREG_OFF = PHASE_B_GPIO_LOW;  //low close

     PHASE_B_GPIO_PORT_HIGH->CFGHR &= ~(0xf<<4); PHASE_B_GPIO_PORT_HIGH->CFGHR |= (0x3<<4);
     PHASE_B_GPIO_PORT_HIGH->HIGH_BITREG_OFF = PHASE_B_GPIO_HIGH; //high close
}

void phaseBLOW()
{
    // low mosfet on
    PHASE_B_GPIO_PORT_LOW->CFGLR  &= ~(0xf<<0); PHASE_B_GPIO_PORT_LOW->CFGLR|= (0x3<<0);
    PHASE_B_GPIO_PORT_LOW->LOW_BITREG_ON = PHASE_B_GPIO_LOW;

    // high close
    PHASE_B_GPIO_PORT_HIGH->CFGHR &= ~(0xf<<4); PHASE_B_GPIO_PORT_HIGH->CFGHR |= (0x3<<4);
    PHASE_B_GPIO_PORT_HIGH->HIGH_BITREG_OFF = PHASE_B_GPIO_HIGH;
}

//////////////////////////////PHASE
/// 2//////////////////////////////////////////////////

void phaseCPWM()
{
    if (!eepromBuffer.comp_pwm)
    {
        //	gpio_mode_QUICK(PHASE_C_GPIO_PORT_LOW, GPIO_MODE_OUTPUT,
        // GPIO_PULL_NONE,
        // PHASE_C_GPIO_LOW); PHASE_C_GPIO_PORT_LOW->LOW_BITREG_OFF =
        // PHASE_C_GPIO_LOW;
    }
    else
    {
        PHASE_C_GPIO_PORT_LOW->CFGLR  &= ~(0xf<<28); PHASE_C_GPIO_PORT_LOW->CFGLR |= (0x3<<28);
        PHASE_C_GPIO_PORT_LOW->BSHR = PHASE_C_GPIO_LOW;
    }
    PHASE_C_GPIO_PORT_HIGH->CFGHR &= ~(0xf<<0); PHASE_C_GPIO_PORT_HIGH->CFGHR |= (0xb<<0);
}

void phaseCFLOAT()
{
    // floating
    PHASE_C_GPIO_PORT_LOW->CFGLR  &= ~(0xf<<28); PHASE_C_GPIO_PORT_LOW->CFGLR |= (0x3<<28);
    PHASE_C_GPIO_PORT_LOW->LOW_BITREG_OFF = PHASE_C_GPIO_LOW;  //low close

    PHASE_C_GPIO_PORT_HIGH->CFGHR &= ~(0xf<<0); PHASE_C_GPIO_PORT_HIGH->CFGHR |= (0x3<<0);
    PHASE_C_GPIO_PORT_HIGH->HIGH_BITREG_OFF = PHASE_C_GPIO_HIGH;//high close
}

void phaseCLOW()
{
    PHASE_C_GPIO_PORT_LOW->CFGLR  &= ~(0xf<<28); PHASE_C_GPIO_PORT_LOW->CFGLR |= (0x3<<28);
    PHASE_C_GPIO_PORT_LOW->LOW_BITREG_ON = PHASE_C_GPIO_LOW; //low on

    PHASE_C_GPIO_PORT_HIGH->CFGHR &= ~(0xf<<0); PHASE_C_GPIO_PORT_HIGH->CFGHR |= (0x3<<0);
    PHASE_C_GPIO_PORT_HIGH->HIGH_BITREG_OFF = PHASE_C_GPIO_HIGH; //high close
}

///////////////////////////////////////////////PHASE 3
////////////////////////////////////////////////////

void phaseAPWM()
{
    if (!eepromBuffer.comp_pwm)
    {
        //	gpio_mode_QUICK(PHASE_A_GPIO_PORT_LOW, GPIO_MODE_OUTPUT,
        // GPIO_PULL_NONE,
        // PHASE_A_GPIO_LOW); PHASE_A_GPIO_PORT_LOW->LOW_BITREG_OFF =
        // PHASE_A_GPIO_LOW;
    }
    else
    {
        PHASE_A_GPIO_PORT_LOW->CFGLR  &= ~(0xf<<4); PHASE_A_GPIO_PORT_LOW->CFGLR|= (0x3<<4);
        PHASE_A_GPIO_PORT_LOW->BSHR = PHASE_A_GPIO_LOW; //low on
    }
    PHASE_A_GPIO_PORT_HIGH->CFGHR &= ~(0xf<<8); PHASE_A_GPIO_PORT_HIGH->CFGHR |= (0xb<<8); //high pwm
}

void phaseAFLOAT()
{
    PHASE_A_GPIO_PORT_LOW->CFGLR  &= ~(0xf<<4); PHASE_A_GPIO_PORT_LOW->CFGLR|= (0x3<<4);
    PHASE_A_GPIO_PORT_LOW->LOW_BITREG_OFF = PHASE_A_GPIO_LOW;  //low close

    PHASE_A_GPIO_PORT_HIGH->CFGHR &= ~(0xf<<8); PHASE_A_GPIO_PORT_HIGH->CFGHR |= (0x3<<8);
    PHASE_A_GPIO_PORT_HIGH->HIGH_BITREG_OFF = PHASE_A_GPIO_HIGH; //high close
}

void phaseALOW()
{
    PHASE_A_GPIO_PORT_LOW->CFGLR  &= ~(0xf<<4); PHASE_A_GPIO_PORT_LOW->CFGLR|= (0x3<<4);
    PHASE_A_GPIO_PORT_LOW->LOW_BITREG_ON = PHASE_A_GPIO_LOW;   // low on

    PHASE_A_GPIO_PORT_HIGH->CFGHR &= ~(0xf<<8); PHASE_A_GPIO_PORT_HIGH->CFGHR |= (0x3<<8);
    PHASE_A_GPIO_PORT_HIGH->HIGH_BITREG_OFF = PHASE_A_GPIO_HIGH; //high close
}

#endif


void allOff()
{
    phaseAFLOAT();
    phaseBFLOAT();
    phaseCFLOAT();
}

void comStep(int newStep)
{
    switch (newStep) {
    case 1: // A-B
        phaseCFLOAT();
        phaseBLOW();
        phaseAPWM();
        break;

    case 2: // C-B
        phaseAFLOAT();
        phaseBLOW();
        phaseCPWM();
        break;

    case 3: // C-A
        phaseBFLOAT();
        phaseALOW();
        phaseCPWM();
        break;

    case 4: // B-A
        phaseCFLOAT();
        phaseALOW();
        phaseBPWM();
        break;

    case 5: // B-C
        phaseAFLOAT();
        phaseCLOW();
        phaseBPWM();
        break;

    case 6: // A-C
        phaseBFLOAT();
        phaseCLOW();
        phaseAPWM();
        break;
    }
}

void fullBrake()
{ // full braking shorting all low sides
    phaseALOW();
    phaseBLOW();
    phaseCLOW();
}

void allpwm()
{ // for stepper_sine
    phaseAPWM();
    phaseBPWM();
    phaseCPWM();
}

void twoChannelForward()
{
    phaseAPWM();
    phaseBLOW();
    phaseCPWM();
}

void twoChannelReverse()
{
    phaseALOW();
    phaseBPWM();
    phaseCLOW();
}
