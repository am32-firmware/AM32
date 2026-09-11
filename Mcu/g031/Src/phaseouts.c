/*
 * phaseouts.c
 *
 *  Created on: Apr 22, 2020
 *      Author: Alka
 */
#include "phaseouts.h"

#include "targets.h"

extern char prop_brake_active;

#ifndef PWM_ENABLE_BRIDGE

#ifdef USE_INVERTED_LOW
#pragma message("using inverted low side output")
#define LOW_BITREG_ON BRR
#define LOW_BITREG_OFF BSRR
#else
#define LOW_BITREG_ON BSRR
#define LOW_BITREG_OFF BRR
#endif

#ifdef USE_INVERTED_HIGH
#pragma message("using inverted high side output")
#define HIGH_BITREG_ON BRR
#define HIGH_BITREG_OFF BSRR
#else
#define HIGH_BITREG_ON BSRR
#define HIGH_BITREG_OFF BRR
#endif

void proportionalBrake()
{ // alternate all channels between braking (ABC LOW)
    // and coasting (ABC float) put lower channel into
    // alternate mode and turn upper OFF for each
    // channel
    // turn all HIGH channels off for ABC

    LL_GPIO_SetPinMode(PHASE_A_GPIO_PORT_HIGH, PHASE_A_GPIO_HIGH,
        LL_GPIO_MODE_OUTPUT);
    PHASE_A_GPIO_PORT_HIGH->HIGH_BITREG_OFF = PHASE_A_GPIO_HIGH;

    LL_GPIO_SetPinMode(PHASE_B_GPIO_PORT_HIGH, PHASE_B_GPIO_HIGH,
        LL_GPIO_MODE_OUTPUT);
    PHASE_B_GPIO_PORT_HIGH->HIGH_BITREG_OFF = PHASE_B_GPIO_HIGH;

    LL_GPIO_SetPinMode(PHASE_C_GPIO_PORT_HIGH, PHASE_C_GPIO_HIGH,
        LL_GPIO_MODE_OUTPUT);
    PHASE_C_GPIO_PORT_HIGH->HIGH_BITREG_OFF = PHASE_C_GPIO_HIGH;

    // set low channel to PWM, duty cycle will now control braking
    LL_GPIO_SetPinMode(PHASE_A_GPIO_PORT_LOW, PHASE_A_GPIO_LOW,
        LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinMode(PHASE_B_GPIO_PORT_LOW, PHASE_B_GPIO_LOW,
        LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinMode(PHASE_C_GPIO_PORT_LOW, PHASE_C_GPIO_LOW,
        LL_GPIO_MODE_ALTERNATE);
}

void phaseBPWM()
{
    if (!eepromBuffer.comp_pwm) { // for future
        LL_GPIO_SetPinMode(PHASE_B_GPIO_PORT_LOW, PHASE_B_GPIO_LOW,
            LL_GPIO_MODE_OUTPUT);
        PHASE_B_GPIO_PORT_LOW->LOW_BITREG_OFF = PHASE_B_GPIO_LOW;
    } else {
        LL_GPIO_SetPinMode(PHASE_B_GPIO_PORT_LOW, PHASE_B_GPIO_LOW,
            LL_GPIO_MODE_ALTERNATE); // low
    }
    LL_GPIO_SetPinMode(PHASE_B_GPIO_PORT_HIGH, PHASE_B_GPIO_HIGH,
        LL_GPIO_MODE_ALTERNATE); // high
}

void phaseBFLOAT()
{
    LL_GPIO_SetPinMode(PHASE_B_GPIO_PORT_LOW, PHASE_B_GPIO_LOW,
        LL_GPIO_MODE_OUTPUT);
    PHASE_B_GPIO_PORT_LOW->LOW_BITREG_OFF = PHASE_B_GPIO_LOW;
    LL_GPIO_SetPinMode(PHASE_B_GPIO_PORT_HIGH, PHASE_B_GPIO_HIGH,
        LL_GPIO_MODE_OUTPUT);
    PHASE_B_GPIO_PORT_HIGH->HIGH_BITREG_OFF = PHASE_B_GPIO_HIGH;
}

void phaseBLOW()
{
    // low mosfet on
    LL_GPIO_SetPinMode(PHASE_B_GPIO_PORT_LOW, PHASE_B_GPIO_LOW,
        LL_GPIO_MODE_OUTPUT);
    PHASE_B_GPIO_PORT_LOW->LOW_BITREG_ON = PHASE_B_GPIO_LOW;
    LL_GPIO_SetPinMode(PHASE_B_GPIO_PORT_HIGH, PHASE_B_GPIO_HIGH,
        LL_GPIO_MODE_OUTPUT);
    PHASE_B_GPIO_PORT_HIGH->HIGH_BITREG_OFF = PHASE_B_GPIO_HIGH;
}

//////////////////////////////PHASE
/// 2//////////////////////////////////////////////////

void phaseCPWM()
{
    if (!eepromBuffer.comp_pwm) {
        LL_GPIO_SetPinMode(PHASE_C_GPIO_PORT_LOW, PHASE_C_GPIO_LOW,
            LL_GPIO_MODE_OUTPUT);
        PHASE_C_GPIO_PORT_LOW->LOW_BITREG_OFF = PHASE_C_GPIO_LOW;
    } else {
        LL_GPIO_SetPinMode(PHASE_C_GPIO_PORT_LOW, PHASE_C_GPIO_LOW,
            LL_GPIO_MODE_ALTERNATE);
    }
    LL_GPIO_SetPinMode(PHASE_C_GPIO_PORT_HIGH, PHASE_C_GPIO_HIGH,
        LL_GPIO_MODE_ALTERNATE);
}

void phaseCFLOAT()
{
    // floating
    LL_GPIO_SetPinMode(PHASE_C_GPIO_PORT_LOW, PHASE_C_GPIO_LOW,
        LL_GPIO_MODE_OUTPUT);
    PHASE_C_GPIO_PORT_LOW->LOW_BITREG_OFF = PHASE_C_GPIO_LOW;
    LL_GPIO_SetPinMode(PHASE_C_GPIO_PORT_HIGH, PHASE_C_GPIO_HIGH,
        LL_GPIO_MODE_OUTPUT);
    PHASE_C_GPIO_PORT_HIGH->HIGH_BITREG_OFF = PHASE_C_GPIO_HIGH;
}

void phaseCLOW()
{
    LL_GPIO_SetPinMode(PHASE_C_GPIO_PORT_LOW, PHASE_C_GPIO_LOW,
        LL_GPIO_MODE_OUTPUT);
    PHASE_C_GPIO_PORT_LOW->LOW_BITREG_ON = PHASE_C_GPIO_LOW;
    LL_GPIO_SetPinMode(PHASE_C_GPIO_PORT_HIGH, PHASE_C_GPIO_HIGH,
        LL_GPIO_MODE_OUTPUT);
    PHASE_C_GPIO_PORT_HIGH->HIGH_BITREG_OFF = PHASE_C_GPIO_HIGH;
}

///////////////////////////////////////////////PHASE 3
////////////////////////////////////////////////////

void phaseAPWM()
{
    if (!eepromBuffer.comp_pwm) {
        LL_GPIO_SetPinMode(PHASE_A_GPIO_PORT_LOW, PHASE_A_GPIO_LOW,
            LL_GPIO_MODE_OUTPUT);
        PHASE_A_GPIO_PORT_LOW->LOW_BITREG_OFF = PHASE_A_GPIO_LOW;
    } else {
        LL_GPIO_SetPinMode(PHASE_A_GPIO_PORT_LOW, PHASE_A_GPIO_LOW,
            LL_GPIO_MODE_ALTERNATE);
    }
    LL_GPIO_SetPinMode(PHASE_A_GPIO_PORT_HIGH, PHASE_A_GPIO_HIGH,
        LL_GPIO_MODE_ALTERNATE);
}

void phaseAFLOAT()
{
    LL_GPIO_SetPinMode(PHASE_A_GPIO_PORT_LOW, PHASE_A_GPIO_LOW,
        LL_GPIO_MODE_OUTPUT);
    PHASE_A_GPIO_PORT_LOW->LOW_BITREG_OFF = PHASE_A_GPIO_LOW;
    LL_GPIO_SetPinMode(PHASE_A_GPIO_PORT_HIGH, PHASE_A_GPIO_HIGH,
        LL_GPIO_MODE_OUTPUT);
    PHASE_A_GPIO_PORT_HIGH->HIGH_BITREG_OFF = PHASE_A_GPIO_HIGH;
}

void phaseALOW()
{
    LL_GPIO_SetPinMode(PHASE_A_GPIO_PORT_LOW, PHASE_A_GPIO_LOW,
        LL_GPIO_MODE_OUTPUT);
    PHASE_A_GPIO_PORT_LOW->LOW_BITREG_ON = PHASE_A_GPIO_LOW;
    LL_GPIO_SetPinMode(PHASE_A_GPIO_PORT_HIGH, PHASE_A_GPIO_HIGH,
        LL_GPIO_MODE_OUTPUT);
    PHASE_A_GPIO_PORT_HIGH->HIGH_BITREG_OFF = PHASE_A_GPIO_HIGH;
}

// Solid-on high side, GPIO driven. Caller must guarantee the low switch
// of the same leg has been off for at least the dead time.
void phaseAHIGH(void)
{
    LL_GPIO_SetPinMode(PHASE_A_GPIO_PORT_LOW, PHASE_A_GPIO_LOW,
        LL_GPIO_MODE_OUTPUT);
    PHASE_A_GPIO_PORT_LOW->LOW_BITREG_OFF = PHASE_A_GPIO_LOW;
    LL_GPIO_SetPinMode(PHASE_A_GPIO_PORT_HIGH, PHASE_A_GPIO_HIGH,
        LL_GPIO_MODE_OUTPUT);
    PHASE_A_GPIO_PORT_HIGH->HIGH_BITREG_ON = PHASE_A_GPIO_HIGH;
}

void phaseBHIGH(void)
{
    LL_GPIO_SetPinMode(PHASE_B_GPIO_PORT_LOW, PHASE_B_GPIO_LOW,
        LL_GPIO_MODE_OUTPUT);
    PHASE_B_GPIO_PORT_LOW->LOW_BITREG_OFF = PHASE_B_GPIO_LOW;
    LL_GPIO_SetPinMode(PHASE_B_GPIO_PORT_HIGH, PHASE_B_GPIO_HIGH,
        LL_GPIO_MODE_OUTPUT);
    PHASE_B_GPIO_PORT_HIGH->HIGH_BITREG_ON = PHASE_B_GPIO_HIGH;
}

void phaseCHIGH(void)
{
    LL_GPIO_SetPinMode(PHASE_C_GPIO_PORT_LOW, PHASE_C_GPIO_LOW,
        LL_GPIO_MODE_OUTPUT);
    PHASE_C_GPIO_PORT_LOW->LOW_BITREG_OFF = PHASE_C_GPIO_LOW;
    LL_GPIO_SetPinMode(PHASE_C_GPIO_PORT_HIGH, PHASE_C_GPIO_HIGH,
        LL_GPIO_MODE_OUTPUT);
    PHASE_C_GPIO_PORT_HIGH->HIGH_BITREG_ON = PHASE_C_GPIO_HIGH;
}

#else

//////////////////////////////////PHASE 1//////////////////////
void phaseBPWM()
{
    if (!eepromBuffer.comp_pwm) { // for future
                     // LL_GPIO_SetPinMode(PHASE_B_GPIO_PORT_LOW,
                     // PHASE_B_GPIO_LOW, LL_GPIO_MODE_OUTPUT);
                     // PHASE_B_GPIO_PORT_LOW->LOW_BITREG_OFF = PHASE_B_GPIO_LOW;
    } else {
        LL_GPIO_SetPinMode(PHASE_B_GPIO_PORT_ENABLE, PHASE_B_GPIO_ENABLE,
            LL_GPIO_MODE_OUTPUT); // enable on
        PHASE_B_GPIO_PORT_ENABLE->BSRR = PHASE_B_GPIO_ENABLE;
    }
    LL_GPIO_SetPinMode(PHASE_B_GPIO_PORT_PWM, PHASE_B_GPIO_PWM,
        LL_GPIO_MODE_ALTERNATE); // high pwm
}

void phaseBFLOAT()
{
    LL_GPIO_SetPinMode(PHASE_B_GPIO_PORT_ENABLE, PHASE_B_GPIO_ENABLE,
        LL_GPIO_MODE_OUTPUT); // enable off
    PHASE_B_GPIO_PORT_ENABLE->BRR = PHASE_B_GPIO_ENABLE;
    LL_GPIO_SetPinMode(PHASE_B_GPIO_PORT_PWM, PHASE_B_GPIO_PWM,
        LL_GPIO_MODE_OUTPUT); // pwm off
    PHASE_B_GPIO_PORT_PWM->BRR = PHASE_B_GPIO_PWM;
}

void phaseBLOW()
{
    // low mosfet on
    LL_GPIO_SetPinMode(PHASE_B_GPIO_PORT_ENABLE, PHASE_B_GPIO_ENABLE,
        LL_GPIO_MODE_OUTPUT); // enable on
    PHASE_B_GPIO_PORT_ENABLE->BSRR = PHASE_B_GPIO_ENABLE;
    LL_GPIO_SetPinMode(PHASE_B_GPIO_PORT_PWM, PHASE_B_GPIO_PWM,
        LL_GPIO_MODE_OUTPUT); // pwm off
    PHASE_B_GPIO_PORT_PWM->BRR = PHASE_B_GPIO_PWM;
}

//////////////////////////////PHASE
/// 2//////////////////////////////////////////////////

void phaseCPWM()
{
    if (!eepromBuffer.comp_pwm) {
        //	LL_GPIO_SetPinMode(PHASE_C_GPIO_PORT_LOW, PHASE_C_GPIO_LOW,
        // LL_GPIO_MODE_OUTPUT); PHASE_C_GPIO_PORT_LOW->LOW_BITREG_OFF =
        // PHASE_C_GPIO_LOW;
    } else {
        LL_GPIO_SetPinMode(PHASE_C_GPIO_PORT_ENABLE, PHASE_C_GPIO_ENABLE,
            LL_GPIO_MODE_OUTPUT); // enable on
        PHASE_C_GPIO_PORT_ENABLE->BSRR = PHASE_C_GPIO_ENABLE;
    }
    LL_GPIO_SetPinMode(PHASE_C_GPIO_PORT_PWM, PHASE_C_GPIO_PWM,
        LL_GPIO_MODE_ALTERNATE);
}

void phaseCFLOAT()
{
    // floating
    LL_GPIO_SetPinMode(PHASE_C_GPIO_PORT_ENABLE, PHASE_C_GPIO_ENABLE,
        LL_GPIO_MODE_OUTPUT); // enable off
    PHASE_C_GPIO_PORT_ENABLE->BRR = PHASE_C_GPIO_ENABLE;
    LL_GPIO_SetPinMode(PHASE_C_GPIO_PORT_PWM, PHASE_C_GPIO_PWM,
        LL_GPIO_MODE_OUTPUT);
    PHASE_C_GPIO_PORT_PWM->BRR = PHASE_C_GPIO_PWM;
}

void phaseCLOW()
{
    LL_GPIO_SetPinMode(PHASE_C_GPIO_PORT_ENABLE, PHASE_C_GPIO_ENABLE,
        LL_GPIO_MODE_OUTPUT); // enable on
    PHASE_C_GPIO_PORT_ENABLE->BSRR = PHASE_C_GPIO_ENABLE;
    LL_GPIO_SetPinMode(PHASE_C_GPIO_PORT_PWM, PHASE_C_GPIO_PWM,
        LL_GPIO_MODE_OUTPUT);
    PHASE_C_GPIO_PORT_PWM->BRR = PHASE_C_GPIO_PWM;
}

///////////////////////////////////////////////PHASE 3
////////////////////////////////////////////////////

void phaseAPWM()
{
    if (!eepromBuffer.comp_pwm) {
        //	LL_GPIO_SetPinMode(PHASE_A_GPIO_PORT_LOW, PHASE_A_GPIO_LOW,
        // LL_GPIO_MODE_OUTPUT); PHASE_A_GPIO_PORT_LOW->LOW_BITREG_OFF =
        // PHASE_A_GPIO_LOW;
    } else {
        LL_GPIO_SetPinMode(PHASE_A_GPIO_PORT_ENABLE, PHASE_A_GPIO_ENABLE,
            LL_GPIO_MODE_OUTPUT); // enable on
        PHASE_A_GPIO_PORT_ENABLE->BSRR = PHASE_A_GPIO_ENABLE;
    }
    LL_GPIO_SetPinMode(PHASE_A_GPIO_PORT_PWM, PHASE_A_GPIO_PWM,
        LL_GPIO_MODE_ALTERNATE);
}

void phaseAFLOAT()
{
    LL_GPIO_SetPinMode(PHASE_A_GPIO_PORT_ENABLE, PHASE_A_GPIO_ENABLE,
        LL_GPIO_MODE_OUTPUT); // enable on
    PHASE_A_GPIO_PORT_ENABLE->BRR = PHASE_A_GPIO_ENABLE;
    LL_GPIO_SetPinMode(PHASE_A_GPIO_PORT_PWM, PHASE_A_GPIO_PWM,
        LL_GPIO_MODE_OUTPUT);
    PHASE_A_GPIO_PORT_PWM->BRR = PHASE_A_GPIO_PWM;
}

void phaseALOW()
{
    LL_GPIO_SetPinMode(PHASE_A_GPIO_PORT_ENABLE, PHASE_A_GPIO_ENABLE,
        LL_GPIO_MODE_OUTPUT); // enable on
    PHASE_A_GPIO_PORT_ENABLE->BSRR = PHASE_A_GPIO_ENABLE;
    LL_GPIO_SetPinMode(PHASE_A_GPIO_PORT_PWM, PHASE_A_GPIO_PWM,
        LL_GPIO_MODE_OUTPUT);
    PHASE_A_GPIO_PORT_PWM->BRR = PHASE_A_GPIO_PWM;
}

// NOTE: gate drivers with dead-time insertion on the pwm input make the
// sequencing below doubly safe; drivers without it rely entirely on the
// explicit float + delay ordering, same as the discrete case.
void phaseAHIGH(void)
{
    LL_GPIO_SetPinMode(PHASE_A_GPIO_PORT_ENABLE, PHASE_A_GPIO_ENABLE,
        LL_GPIO_MODE_OUTPUT);
    PHASE_A_GPIO_PORT_ENABLE->BSRR = PHASE_A_GPIO_ENABLE;
    LL_GPIO_SetPinMode(PHASE_A_GPIO_PORT_PWM, PHASE_A_GPIO_PWM,
        LL_GPIO_MODE_OUTPUT);
    PHASE_A_GPIO_PORT_PWM->BSRR = PHASE_A_GPIO_PWM;
}

void phaseBHIGH(void)
{
    LL_GPIO_SetPinMode(PHASE_B_GPIO_PORT_ENABLE, PHASE_B_GPIO_ENABLE,
        LL_GPIO_MODE_OUTPUT);
    PHASE_B_GPIO_PORT_ENABLE->BSRR = PHASE_B_GPIO_ENABLE;
    LL_GPIO_SetPinMode(PHASE_B_GPIO_PORT_PWM, PHASE_B_GPIO_PWM,
        LL_GPIO_MODE_OUTPUT);
    PHASE_B_GPIO_PORT_PWM->BSRR = PHASE_B_GPIO_PWM;
}

void phaseCHIGH(void)
{
    LL_GPIO_SetPinMode(PHASE_C_GPIO_PORT_ENABLE, PHASE_C_GPIO_ENABLE,
        LL_GPIO_MODE_OUTPUT);
    PHASE_C_GPIO_PORT_ENABLE->BSRR = PHASE_C_GPIO_ENABLE;
    LL_GPIO_SetPinMode(PHASE_C_GPIO_PORT_PWM, PHASE_C_GPIO_PWM,
        LL_GPIO_MODE_OUTPUT);
    PHASE_C_GPIO_PORT_PWM->BSRR = PHASE_C_GPIO_PWM;
}

#endif

void allOff()
{
    phaseAFLOAT();
    phaseBFLOAT();
    phaseCFLOAT();
}

// Turn on the synchronous FET on the floating leg for active demag: the
// FET in parallel with the body diode that would otherwise carry the
// freewheeling current for this step.
void syncFetOn(int newStep, char forward)
{
    if (forward) { // 1->2->3->4->5->6
        switch (newStep) {
        case 1: phaseCHIGH(); break; // C floated, was LOW  in step 6
        case 2: phaseALOW();  break; // A floated, was PWM  in step 1
        case 3: phaseBHIGH(); break; // B floated, was LOW  in step 2
        case 4: phaseCLOW();  break; // C floated, was PWM  in step 3
        case 5: phaseAHIGH(); break; // A floated, was LOW  in step 4
        case 6: phaseBLOW();  break; // B floated, was PWM  in step 5
        }
    } else { // 6->5->4->3->2->1, previous role mirrored
        switch (newStep) {
        case 1: phaseCLOW();  break; // C floated, was PWM  in step 2
        case 2: phaseAHIGH(); break; // A floated, was LOW  in step 3
        case 3: phaseBLOW();  break; // B floated, was PWM  in step 4
        case 4: phaseCHIGH(); break; // C floated, was LOW  in step 5
        case 5: phaseALOW();  break; // A floated, was PWM  in step 6
        case 6: phaseBHIGH(); break; // B floated, was LOW  in step 1
        }
    }
}

void floatLeg(int newStep)
{
    switch (newStep) {
    case 1: case 4: phaseCFLOAT(); break;
    case 2: case 5: phaseAFLOAT(); break;
    case 3: case 6: phaseBFLOAT(); break;
    }
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
