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
// #define HIGH_BITREG_ON  BRR
#define HIGH_BITREG_OFF BSRR
#else
// #define HIGH_BITREG_ON  BSRR
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

#endif


/*
 * phaseouts_fastdemag.c
 *
 * Fixed-duration fast demagnetization for AM32.
 *
 * Instead of modulating the PWM off-time (ST-style fast demag, which
 * loses effect as duty -> 100%), the fast-decay bridge state is applied
 * as a one-shot pulse at commutation, before the normal comStep()
 * configuration. During the pulse the switch that would close the slow
 * freewheel loop is floated, so the residual current in the newly
 * floating winding is forced through a body diode to the opposite rail
 * and sees the full bus voltage. The diode path self-terminates: once
 * the winding current reaches zero the diodes block, so an over-length
 * pulse cannot reverse the current -- it only delays drive.
 *
 * Rule (derived from the comStep() table in phaseouts.c):
 *  - If the newly floating phase was the PWM (high) phase of the
 *    previous step, its demag current is bridge->motor. It freewheels
 *    via its own low-side diode and returns through the new step's LOW
 *    phase. Fast demag: FLOAT the new LOW phase, keep the new PWM
 *    phase driven. Current is then pushed up through the floated leg's
 *    high-side diode into +Vbus.
 *  - If it was the LOW phase of the previous step, its demag current is
 *    motor->bridge, freewheeling via its own high-side diode into
 *    +Vbus, with the loop closing through the new PWM phase. Fast
 *    demag: FLOAT the new PWM phase, keep the new LOW phase on. Return
 *    current is then pulled from ground through the floated leg's
 *    low-side diode.
 *
 * AM32 reverses rotation by decrementing 'step', which flips which role
 * the floating phase previously had -- hence the 'forward' parameter.
 */

#include "phaseouts.h"
#include "targets.h"
#include "functions.h" // delayMicros()

// Bridge state applied for the demag pulse only. Call comStep(newStep)
// afterwards to restore the normal drive configuration.
void comStepDemag(int newStep, char forward)
{
    if (forward) {
        switch (newStep) {
        case 1: // A-B. C floats, was LOW in step 6 -> float PWM phase (A)
            phaseAFLOAT();
            phaseBLOW();
            phaseCFLOAT();
            break;

        case 2: // C-B. A floats, was PWM in step 1 -> float LOW phase (B)
            phaseAFLOAT();
            phaseBFLOAT();
            phaseCPWM();
            break;

        case 3: // C-A. B floats, was LOW in step 2 -> float PWM phase (C)
            phaseALOW();
            phaseBFLOAT();
            phaseCFLOAT();
            break;

        case 4: // B-A. C floats, was PWM in step 3 -> float LOW phase (A)
            phaseAFLOAT();
            phaseBPWM();
            phaseCFLOAT();
            break;

        case 5: // B-C. A floats, was LOW in step 4 -> float PWM phase (B)
            phaseAFLOAT();
            phaseBFLOAT();
            phaseCLOW();
            break;

        case 6: // A-C. B floats, was PWM in step 5 -> float LOW phase (C)
            phaseAPWM();
            phaseBFLOAT();
            phaseCFLOAT();
            break;
        }
    } else { // step order 6 -> 5 -> ... -> 1, previous roles are mirrored
        switch (newStep) {
        case 1: // from step 2. C floats, was PWM -> float LOW phase (B)
            phaseAPWM();
            phaseBFLOAT();
            phaseCFLOAT();
            break;

        case 2: // from step 3. A floats, was LOW -> float PWM phase (C)
            phaseAFLOAT();
            phaseBLOW();
            phaseCFLOAT();
            break;

        case 3: // from step 4. B floats, was PWM -> float LOW phase (A)
            phaseAFLOAT();
            phaseBFLOAT();
            phaseCPWM();
            break;

        case 4: // from step 5. C floats, was LOW -> float PWM phase (B)
            phaseALOW();
            phaseBFLOAT();
            phaseCFLOAT();
            break;

        case 5: // from step 6. A floats, was PWM -> float LOW phase (C)
            phaseAFLOAT();
            phaseBPWM();
            phaseCFLOAT();
            break;

        case 6: // from step 1. B floats, was LOW -> float PWM phase (A)
            phaseAFLOAT();
            phaseBFLOAT();
            phaseCLOW();
            break;
        }
    }
}

// Simple blocking variant: demag pulse, then normal commutation.
// demag_us should stay small (single-digit microseconds at light load,
// scaled up with phase current -- see notes). Must be shorter than the
// zero-cross blanking window so the comparator never sees the pulse.




void allOff()
{
    phaseAFLOAT();
    phaseBFLOAT();
    phaseCFLOAT();
}

void comStep(int newStep)
{
    // TIM14->CNT = 0;
    switch (newStep) {
    case 1: // A-B
        phaseAPWM();
        phaseBLOW();
        phaseCFLOAT();
        break;

    case 2: // C-B
        phaseAFLOAT();
        phaseBLOW();
        phaseCPWM();
        break;

    case 3: // C-A
        phaseALOW();
        phaseBFLOAT();
        phaseCPWM();
        break;

    case 4: // B-A
        phaseALOW();
        phaseBPWM();
        phaseCFLOAT();
        break;

    case 5: // B-C
        phaseAFLOAT();
        phaseBPWM();
        phaseCLOW();
        break;

    case 6: // A-C
        phaseAPWM();
        phaseBFLOAT();
        phaseCLOW();
        break;
    }

    // stop_time = TIM14->CNT;
}

/*
 * phaseouts_activedemag.c
 *
 * Active (synchronous) fast demagnetization for AM32.
 *
 * Extends the fixed-duration fast-demag pulse: instead of letting the
 * demag current run through MOSFET body diodes, the FETs in parallel
 * with those diodes are turned on for the bulk of the pulse.
 *
 * What this buys and what it doesn't:
 *  - Decay rate is essentially unchanged (Vbus + 2*I*Rds(on) vs
 *    Vbus + 2*Vf -- marginally SLOWER without the diode drops).
 *  - Conduction loss moves from the body diodes (~Vf * I each) into
 *    the channels (I^2 * Rds(on)). At high phase current this is a
 *    large thermal win, and diode reverse-recovery is avoided.
 *
 * HAZARD 1 -- current reversal: a FET conducts both ways. If the
 * active configuration is still applied when the winding current
 * reaches zero, current rebuilds in REVERSE (braking torque, corrupted
 * BEMF). Therefore the active interval must be shorter than the true
 * demag time. This implementation always follows the active interval
 * with the PASSIVE diode configuration (comStepDemag) so the diodes
 * carry the tail and self-terminate.
 *
 * HAZARD 2 -- shoot-through: stock AM32 sequencing guarantees a leg
 * passes through FLOAT between high and low conduction. Active demag
 * breaks that (e.g. A LOW is commanded on the leg that was PWM-high
 * one instant earlier), and these GPIO transitions bypass the timer
 * dead-time generator. All entries/exits below therefore go through
 * allOff() / float plus an explicit dead-time delay.
 *
 * Requires comStepDemag()/comStep() from phaseouts_fastdemag.c /
 * phaseouts.c and delayMicros() from functions.c.
 */
 
#include "phaseouts.h"
#include "targets.h"
#include "functions.h"
 
// GPIO-driven dead time between turning one switch of a leg off and
// the opposite switch on. Tune to your gate driver + FET turn-off
// time; 1 us is conservative for typical ESC hardware.
#ifndef DEMAG_DEADTIME_US
#define DEMAG_DEADTIME_US 1
#endif
 
extern void comStepDemag(int newStep, char forward); // passive tail
void comStepDemagActive(int newStep, char forward);
 
#ifndef PWM_ENABLE_BRIDGE
 
// Stock phaseouts.c only ever turns high sides off via GPIO, so
// HIGH_BITREG_ON is not defined there. Restore both polarities here.
#ifdef USE_INVERTED_LOW
#define LOW_BITREG_ON BRR
#define LOW_BITREG_OFF BSRR
#else
#define LOW_BITREG_ON BSRR
#define LOW_BITREG_OFF BRR
#endif
 
#ifdef USE_INVERTED_HIGH
#define HIGH_BITREG_ON BRR
#define HIGH_BITREG_OFF BSRR
#else
#define HIGH_BITREG_ON BSRR
#define HIGH_BITREG_OFF BRR
#endif
 
// Solid-on high side, GPIO driven. CALLER must guarantee the low
// switch of the same leg has been off for at least the dead time.
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
 
#else // PWM_ENABLE_BRIDGE: enable pin + single pwm input per leg
 
void phaseAHIGH(void)
{
    LL_GPIO_SetPinMode(PHASE_A_GPIO_PORT_ENABLE, PHASE_A_GPIO_ENABLE,
        LL_GPIO_MODE_OUTPUT);
    PHASE_A_GPIO_PORT_ENABLE->BSRR = PHASE_A_GPIO_ENABLE; // enable on
    LL_GPIO_SetPinMode(PHASE_A_GPIO_PORT_PWM, PHASE_A_GPIO_PWM,
        LL_GPIO_MODE_OUTPUT);
    PHASE_A_GPIO_PORT_PWM->BSRR = PHASE_A_GPIO_PWM; // pwm pin high
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
 
// NOTE: gate drivers with dead-time insertion on the pwm input make
// the sequencing below doubly safe; drivers without it rely entirely
// on the explicit float + delay ordering, same as the discrete case.
 
#endif
 
// Active demag bridge state: the passive comStepDemag() configuration
// with each conducting body diode replaced by its FET turned on.
// Steps with one diode in the loop get one xHIGH/xLOW substitution;
// steps with two diodes get both.
void comStepDemagActive(int newStep, char forward)
{
    if (forward) { // step order 1 -> 2 -> ... -> 6 -> 1
        switch (newStep) {
        case 1: // demag loop: gnd -> B FET -> B -> N -> C -> C high -> bus
            phaseAFLOAT();
            phaseBLOW();
            phaseCHIGH();
            break;
        case 2: // gnd -> A low -> A -> N -> B -> B high -> bus, C drives
            phaseALOW();
            phaseBHIGH();
            phaseCPWM();
            break;
        case 3: // gnd -> A FET -> A -> N -> B -> B high -> bus
            phaseALOW();
            phaseBHIGH();
            phaseCFLOAT();
            break;
        case 4: // gnd -> C low -> C -> N -> A -> A high -> bus, B drives
            phaseAHIGH();
            phaseBPWM();
            phaseCLOW();
            break;
        case 5: // gnd -> C FET -> C -> N -> A -> A high -> bus
            phaseAHIGH();
            phaseBFLOAT();
            phaseCLOW();
            break;
        case 6: // gnd -> B low -> B -> N -> C -> C high -> bus, A drives
            phaseAPWM();
            phaseBLOW();
            phaseCHIGH();
            break;
        }
    } else { // step order 6 -> 5 -> ... -> 1 -> 6
        switch (newStep) {
        case 1: // gnd -> C low -> C -> N -> B -> B high -> bus, A drives
            phaseAPWM();
            phaseBHIGH();
            phaseCLOW();
            break;
        case 2: // gnd -> B FET -> B -> N -> A -> A high -> bus
            phaseAHIGH();
            phaseBLOW();
            phaseCFLOAT();
            break;
        case 3: // gnd -> B low -> B -> N -> A -> A high -> bus, C drives
            phaseAHIGH();
            phaseBLOW();
            phaseCPWM();
            break;
        case 4: // gnd -> A FET -> A -> N -> C -> C high -> bus
            phaseALOW();
            phaseBFLOAT();
            phaseCHIGH();
            break;
        case 5: // gnd -> A low -> A -> N -> C -> C high -> bus, B drives
            phaseALOW();
            phaseBPWM();
            phaseCHIGH();
            break;
        case 6: // gnd -> C FET -> C -> N -> B -> B high -> bus
            phaseAFLOAT();
            phaseBHIGH();
            phaseCLOW();
            break;
        }
    }
}
 
// Full commutation with active demag pulse and passive diode tail.
//
//   active_us : synchronous interval. MUST be < true demag time
//               (L*I/Vbus). Start at ~60-70% of the demag time you
//               measure with the passive version and creep up.
//   tail_us   : passive diode interval after the FETs release. Also
//               serves as the dead time before the normal step config
//               re-engages, so it is clamped to DEMAG_DEADTIME_US min.
//
// Blocking variant for bring-up; move the delays onto a one-shot timer
// for flight code. Total added dead time per commutation is
// ~ 2*DEMAG_DEADTIME_US + active_us + tail_us -- budget this against
// your commutation period at max eRPM.
void comStepWithActiveDemag(int newStep, char forward, uint16_t active_us,
    uint16_t tail_us)
{
    allOff(); // every changing leg through float first
    delayMicros(DEMAG_DEADTIME_US);
 
    comStepDemagActive(newStep, forward);
    delayMicros(active_us);
 
    // Release the actively driven demag FETs; diodes carry the tail
    // and self-terminate at current zero. The floating phase's clamp
    // release is observable on the comparator during this interval.
    comStepDemag(newStep, forward);
    delayMicros((tail_us > DEMAG_DEADTIME_US) ? tail_us : DEMAG_DEADTIME_US);
 
    comStep(newStep);
}

void comStepWithFastDemag(int newStep, char forward, uint16_t demag_us)
{
    comStepDemag(newStep, forward);
    delayMicros(demag_us);
    comStep(newStep);
}

//#include "phaseouts.h"
//#include "targets.h"
//#include "functions.h" // delayMicros()
// 
//#ifndef DEMAG_DEADTIME_US
//#define DEMAG_DEADTIME_US 1
//#endif
// 
///* ---- solid-on high-side primitives ------------------------------- */
//#ifndef PWM_ENABLE_BRIDGE
// 
//#ifdef USE_INVERTED_HIGH
//#define HIGH_BITREG_ON BRR
//#define HIGH_BITREG_OFF BSRR
//#else
//#define HIGH_BITREG_ON BSRR
//#define HIGH_BITREG_OFF BRR
//#endif
// 
//void phaseAHIGH(void)
//{
//    LL_GPIO_SetPinMode(PHASE_A_GPIO_PORT_LOW, PHASE_A_GPIO_LOW, LL_GPIO_MODE_OUTPUT);
//    PHASE_A_GPIO_PORT_LOW->LOW_BITREG_OFF = PHASE_A_GPIO_LOW; // ensure low off
//    LL_GPIO_SetPinMode(PHASE_A_GPIO_PORT_HIGH, PHASE_A_GPIO_HIGH, LL_GPIO_MODE_OUTPUT);
//    PHASE_A_GPIO_PORT_HIGH->HIGH_BITREG_ON = PHASE_A_GPIO_HIGH;
//}
//void phaseBHIGH(void)
//{
//    LL_GPIO_SetPinMode(PHASE_B_GPIO_PORT_LOW, PHASE_B_GPIO_LOW, LL_GPIO_MODE_OUTPUT);
//    PHASE_B_GPIO_PORT_LOW->LOW_BITREG_OFF = PHASE_B_GPIO_LOW;
//    LL_GPIO_SetPinMode(PHASE_B_GPIO_PORT_HIGH, PHASE_B_GPIO_HIGH, LL_GPIO_MODE_OUTPUT);
//    PHASE_B_GPIO_PORT_HIGH->HIGH_BITREG_ON = PHASE_B_GPIO_HIGH;
//}
//void phaseCHIGH(void)
//{
//    LL_GPIO_SetPinMode(PHASE_C_GPIO_PORT_LOW, PHASE_C_GPIO_LOW, LL_GPIO_MODE_OUTPUT);
//    PHASE_C_GPIO_PORT_LOW->LOW_BITREG_OFF = PHASE_C_GPIO_LOW;
//    LL_GPIO_SetPinMode(PHASE_C_GPIO_PORT_HIGH, PHASE_C_GPIO_HIGH, LL_GPIO_MODE_OUTPUT);
//    PHASE_C_GPIO_PORT_HIGH->HIGH_BITREG_ON = PHASE_C_GPIO_HIGH;
//}
// 
//#else // enable + single pwm input per leg; driver switches complementarily
// 
//void phaseAHIGH(void)
//{
//    LL_GPIO_SetPinMode(PHASE_A_GPIO_PORT_ENABLE, PHASE_A_GPIO_ENABLE, LL_GPIO_MODE_OUTPUT);
//    PHASE_A_GPIO_PORT_ENABLE->BSRR = PHASE_A_GPIO_ENABLE;
//    LL_GPIO_SetPinMode(PHASE_A_GPIO_PORT_PWM, PHASE_A_GPIO_PWM, LL_GPIO_MODE_OUTPUT);
//    PHASE_A_GPIO_PORT_PWM->BSRR = PHASE_A_GPIO_PWM; // solid high
//}
//void phaseBHIGH(void)
//{
//    LL_GPIO_SetPinMode(PHASE_B_GPIO_PORT_ENABLE, PHASE_B_GPIO_ENABLE, LL_GPIO_MODE_OUTPUT);
//    PHASE_B_GPIO_PORT_ENABLE->BSRR = PHASE_B_GPIO_ENABLE;
//    LL_GPIO_SetPinMode(PHASE_B_GPIO_PORT_PWM, PHASE_B_GPIO_PWM, LL_GPIO_MODE_OUTPUT);
//    PHASE_B_GPIO_PORT_PWM->BSRR = PHASE_B_GPIO_PWM;
//}
//void phaseCHIGH(void)
//{
//    LL_GPIO_SetPinMode(PHASE_C_GPIO_PORT_ENABLE, PHASE_C_GPIO_ENABLE, LL_GPIO_MODE_OUTPUT);
//    PHASE_C_GPIO_PORT_ENABLE->BSRR = PHASE_C_GPIO_ENABLE;
//    LL_GPIO_SetPinMode(PHASE_C_GPIO_PORT_PWM, PHASE_C_GPIO_PWM, LL_GPIO_MODE_OUTPUT);
//    PHASE_C_GPIO_PORT_PWM->BSRR = PHASE_C_GPIO_PWM;
//}
// 
//#endif
 
/* ---- turn on the synchronous FET on the floating leg ------------- */
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
 
/*
 * Drop-in replacement for comStep() at the point commutation is
 * applied. active_us: synchronous interval, MUST be < real demag time
 * (start ~60-70% of the demag time you measure passively, scale with
 * load). Blocking (delayMicros) -- fine for bring-up; for flight code
 * put syncFetOn/floatLeg on one-shot timer events instead of blocking
 * in the commutation path.
 */
void comStepSyncDemag(int newStep, char forward, uint16_t active_us)
{
    comStep(newStep);               // normal commutation, third phase floats
    delayMicros(DEMAG_DEADTIME_US); // leg's previous switch fully off
    syncFetOn(newStep, forward);    // synchronous rectification of freewheel
    delayMicros(active_us);
    floatLeg(newStep);              // diode tail, self-terminating
}


void highSidesOff()
{
    // Force all three high-side FETs off; leave the low sides in
    // whatever state comStep() set (comp_pwm complementary PWM on the
    // driven leg, solid-on low side on the LOW leg). Only OFF writes,
    // so no shoot-through and no dead time needed. Holds until the next
    // comStep() re-enables a high side.
    LL_GPIO_SetPinMode(PHASE_A_GPIO_PORT_HIGH, PHASE_A_GPIO_HIGH, LL_GPIO_MODE_OUTPUT);
    PHASE_A_GPIO_PORT_HIGH->HIGH_BITREG_OFF = PHASE_A_GPIO_HIGH;
    LL_GPIO_SetPinMode(PHASE_B_GPIO_PORT_HIGH, PHASE_B_GPIO_HIGH, LL_GPIO_MODE_OUTPUT);
    PHASE_B_GPIO_PORT_HIGH->HIGH_BITREG_OFF = PHASE_B_GPIO_HIGH;
    LL_GPIO_SetPinMode(PHASE_C_GPIO_PORT_HIGH, PHASE_C_GPIO_HIGH, LL_GPIO_MODE_OUTPUT);
    PHASE_C_GPIO_PORT_HIGH->HIGH_BITREG_OFF = PHASE_C_GPIO_HIGH;
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
