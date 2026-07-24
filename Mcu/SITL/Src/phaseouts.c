/*
  phaseouts.c - SITL bridge output control. Instead of GPIO mode changes
  each phase gets a mode that the motor simulation samples together with
  the emulated TIM1 to derive fet gate states
 */

#include "phaseouts.h"

#include "common.h"
#include "sitl.h"
#include "targets.h"

volatile uint8_t sitl_phase_mode[3];

static void phasePWM(int p)
{
    if (!eepromBuffer.comp_pwm) {
        sitl_phase_mode[p] = SITL_PHASE_PWM_NOCOMP;
    } else {
        sitl_phase_mode[p] = SITL_PHASE_PWM;
    }
}

static void phaseFLOAT(int p)
{
    sitl_phase_mode[p] = SITL_PHASE_FLOAT;
}

static void phaseLOW(int p)
{
    sitl_phase_mode[p] = SITL_PHASE_LOW;
}

void proportionalBrake(void)
{
    for (int p = 0; p < 3; p++) {
        sitl_phase_mode[p] = SITL_PHASE_BRAKE_PWM;
    }
}

void allOff(void)
{
    phaseFLOAT(0);
    phaseFLOAT(1);
    phaseFLOAT(2);
}

// commutation debug logging in the simulation
extern void motor_log_commutation(int step);

void comStep(int newStep)
{
    motor_log_commutation(newStep);
    switch (newStep) {
    case 1: // A-B
        phasePWM(0);
        phaseLOW(1);
        phaseFLOAT(2);
        break;
    case 2: // C-B
        phaseFLOAT(0);
        phaseLOW(1);
        phasePWM(2);
        break;
    case 3: // C-A
        phaseLOW(0);
        phaseFLOAT(1);
        phasePWM(2);
        break;
    case 4: // B-A
        phaseLOW(0);
        phasePWM(1);
        phaseFLOAT(2);
        break;
    case 5: // B-C
        phaseFLOAT(0);
        phasePWM(1);
        phaseLOW(2);
        break;
    case 6: // A-C
        phasePWM(0);
        phaseFLOAT(1);
        phaseLOW(2);
        break;
    }
}

void fullBrake(void)
{
    phaseLOW(0);
    phaseLOW(1);
    phaseLOW(2);
}

void allpwm(void)
{
    phasePWM(0);
    phasePWM(1);
    phasePWM(2);
}

void twoChannelForward(void)
{
    phasePWM(0);
    phaseLOW(1);
    phasePWM(2);
}

void twoChannelReverse(void)
{
    phaseLOW(0);
    phasePWM(1);
    phaseLOW(2);
}
