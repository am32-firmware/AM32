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

void gpio_mode_QUICK(uint32_t gpio_periph, uint32_t mode, uint32_t pull_up_down,
                     uint32_t pin)
{
  GPIO_CTL(gpio_periph) = ((((((GPIO_CTL(gpio_periph)))) & (~(((pin * pin) * (0x3UL << (0U)))))) | (((pin * pin) * mode))));
}

void proportionalBrake()
{
  // alternate all channels between braking (ABC LOW)
  // and coasting (ABC float) put lower channel into
  // alternate mode and turn upper OFF for each
  // channel
  // turn all HIGH channels off for ABC

  //	gpio_mode_QUICK(PHASE_A_GPIO_PORT_HIGH, GPIO_MODE_OUTPUT,
  // GPIO_PUPD_NONE, PHASE_A_GPIO_HIGH);
  gpio_mode_QUICK(PHASE_A_GPIO_PORT_HIGH, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
                  PHASE_A_GPIO_HIGH);
  GPIO_BC(PHASE_A_GPIO_PORT_HIGH) = PHASE_A_GPIO_HIGH;

  gpio_mode_QUICK(PHASE_B_GPIO_PORT_HIGH, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
                  PHASE_B_GPIO_HIGH);
  GPIO_BC(PHASE_B_GPIO_PORT_HIGH) = PHASE_B_GPIO_HIGH;

  gpio_mode_QUICK(PHASE_C_GPIO_PORT_HIGH, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
                  PHASE_C_GPIO_HIGH);
  GPIO_BC(PHASE_C_GPIO_PORT_HIGH) = PHASE_C_GPIO_HIGH;

  // set low channel to PWM, duty cycle will now control braking
  gpio_mode_QUICK(PHASE_A_GPIO_PORT_LOW, GPIO_MODE_AF, GPIO_PUPD_NONE,
                  PHASE_A_GPIO_LOW);
  gpio_mode_QUICK(PHASE_A_GPIO_PORT_LOW, GPIO_MODE_AF, GPIO_PUPD_NONE,
                  PHASE_B_GPIO_LOW);
  gpio_mode_QUICK(PHASE_C_GPIO_PORT_LOW, GPIO_MODE_AF, GPIO_PUPD_NONE,
                  PHASE_C_GPIO_LOW);
}

void phaseBPWM()
{
  if (!eepromBuffer.comp_pwm) { // for future
    gpio_mode_QUICK(PHASE_B_GPIO_PORT_LOW, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
                    PHASE_B_GPIO_LOW);
    GPIO_BC(PHASE_B_GPIO_PORT_LOW) = PHASE_B_GPIO_LOW;
  } else {
    gpio_mode_QUICK(PHASE_B_GPIO_PORT_LOW, GPIO_MODE_AF, GPIO_PUPD_NONE,
                    PHASE_B_GPIO_LOW); // low
    //		GPIO_CTL(PHASE_B_GPIO_PORT_LOW) |= (2 <<
    //(PHASE_B_GPIO_LOW
    //<<2)); 		GPIO_CTL(PHASE_B_GPIO_PORT_LOW) =
    //((((((GPIO_CTL(PHASE_B_GPIO_PORT_LOW)))) & (~(((PHASE_B_GPIO_LOW *
    // PHASE_B_GPIO_LOW) * (0x3UL << (0U)))))) | (((PHASE_B_GPIO_LOW *
    // PHASE_B_GPIO_LOW) * 2))));
  }
  gpio_mode_QUICK(
    PHASE_B_GPIO_PORT_HIGH, GPIO_MODE_AF, GPIO_PUPD_NONE,
    PHASE_B_GPIO_HIGH); // high
  //	  GPIO_CTL(PHASE_B_GPIO_PORT_HIGH) =
  //((((((GPIO_CTL(PHASE_B_GPIO_PORT_HIGH)))) & (~(((PHASE_B_GPIO_HIGH *
  // PHASE_B_GPIO_HIGH) * (0x3UL << (0U)))))) | (((PHASE_B_GPIO_HIGH *
  // PHASE_B_GPIO_HIGH) * 2))));
}

void phaseBFLOAT()
{
  gpio_mode_QUICK(PHASE_B_GPIO_PORT_LOW, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
                  PHASE_B_GPIO_LOW);
  GPIO_BC(PHASE_B_GPIO_PORT_LOW) = PHASE_B_GPIO_LOW;
  gpio_mode_QUICK(PHASE_B_GPIO_PORT_HIGH, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
                  PHASE_B_GPIO_HIGH);
  GPIO_BC(PHASE_B_GPIO_PORT_HIGH) = PHASE_B_GPIO_HIGH;
}

void phaseBLOW()
{
  // low mosfet on
  gpio_mode_QUICK(PHASE_B_GPIO_PORT_LOW, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
                  PHASE_B_GPIO_LOW);
  GPIO_BOP(PHASE_B_GPIO_PORT_LOW) = PHASE_B_GPIO_LOW;
  gpio_mode_QUICK(PHASE_B_GPIO_PORT_HIGH, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
                  PHASE_B_GPIO_HIGH);
  GPIO_BC(PHASE_B_GPIO_PORT_HIGH) = PHASE_B_GPIO_HIGH;
}

//////////////////////////////PHASE
/// 2//////////////////////////////////////////////////

void phaseCPWM()
{
  if (!eepromBuffer.comp_pwm) {
    gpio_mode_QUICK(PHASE_C_GPIO_PORT_LOW, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
                    PHASE_C_GPIO_LOW);
    GPIO_BC(PHASE_C_GPIO_PORT_LOW) = PHASE_C_GPIO_LOW;
  } else {
    gpio_mode_QUICK(PHASE_C_GPIO_PORT_LOW, GPIO_MODE_AF, GPIO_PUPD_NONE,
                    PHASE_C_GPIO_LOW);
    ;
  }
  gpio_mode_QUICK(PHASE_C_GPIO_PORT_HIGH, GPIO_MODE_AF, GPIO_PUPD_NONE,
                  PHASE_C_GPIO_HIGH);
}

void phaseCFLOAT()
{
  // floating
  gpio_mode_QUICK(PHASE_C_GPIO_PORT_LOW, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
                  PHASE_C_GPIO_LOW);
  GPIO_BC(PHASE_C_GPIO_PORT_LOW) = PHASE_C_GPIO_LOW;
  gpio_mode_QUICK(PHASE_C_GPIO_PORT_HIGH, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
                  PHASE_C_GPIO_HIGH);
  GPIO_BC(PHASE_C_GPIO_PORT_HIGH) = PHASE_C_GPIO_HIGH;
}

void phaseCLOW()
{
  gpio_mode_QUICK(PHASE_C_GPIO_PORT_LOW, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
                  PHASE_C_GPIO_LOW);
  GPIO_BOP(PHASE_C_GPIO_PORT_LOW) = PHASE_C_GPIO_LOW;
  gpio_mode_QUICK(PHASE_C_GPIO_PORT_HIGH, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
                  PHASE_C_GPIO_HIGH);
  GPIO_BC(PHASE_C_GPIO_PORT_HIGH) = PHASE_C_GPIO_HIGH;
}

///////////////////////////////////////////////PHASE 3
////////////////////////////////////////////////////

void phaseAPWM()
{
  if (!eepromBuffer.comp_pwm) {
    gpio_mode_QUICK(PHASE_A_GPIO_PORT_LOW, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
                    PHASE_A_GPIO_LOW);
    GPIO_BC(PHASE_A_GPIO_PORT_LOW) = PHASE_A_GPIO_LOW;
  } else {
    gpio_mode_QUICK(PHASE_A_GPIO_PORT_LOW, GPIO_MODE_AF, GPIO_PUPD_NONE,
                    PHASE_A_GPIO_LOW);
  }
  gpio_mode_QUICK(PHASE_A_GPIO_PORT_HIGH, GPIO_MODE_AF, GPIO_PUPD_NONE,
                  PHASE_A_GPIO_HIGH);
}

void phaseAFLOAT()
{
  gpio_mode_QUICK(PHASE_A_GPIO_PORT_LOW, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
                  PHASE_A_GPIO_LOW);
  GPIO_BC(PHASE_A_GPIO_PORT_LOW) = PHASE_A_GPIO_LOW;
  gpio_mode_QUICK(PHASE_A_GPIO_PORT_HIGH, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
                  PHASE_A_GPIO_HIGH);
  GPIO_BC(PHASE_A_GPIO_PORT_HIGH) = PHASE_A_GPIO_HIGH;
}

void phaseALOW()
{
  gpio_mode_QUICK(PHASE_A_GPIO_PORT_LOW, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
                  PHASE_A_GPIO_LOW);
  GPIO_BOP(PHASE_A_GPIO_PORT_LOW) = PHASE_A_GPIO_LOW;
  gpio_mode_QUICK(PHASE_A_GPIO_PORT_HIGH, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
                  PHASE_A_GPIO_HIGH);
  GPIO_BC(PHASE_A_GPIO_PORT_HIGH) = PHASE_A_GPIO_HIGH;
}

#else

//////////////////////////////////PHASE 1//////////////////////
void phaseBPWM()
{
  if (!eepromBuffer.comp_pwm) { // for future
    // gpio_mode_QUICK(PHASE_B_GPIO_PORT_LOW, GPIO_MODE_OUTPUT,
    // GPIO_PUPD_NONE, PHASE_B_GPIO_LOW);
    // GPIO_BC(PHASE_B_GPIO_PORT_LOW) = PHASE_B_GPIO_LOW;
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
    //	gpio_mode_QUICK(PHASE_C_GPIO_PORT_LOW, GPIO_MODE_OUTPUT,
    // GPIO_PUPD_NONE,
    // PHASE_C_GPIO_LOW); GPIO_BC(PHASE_C_GPIO_PORT_LOW) = PHASE_C_GPIO_LOW;
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
    //	gpio_mode_QUICK(PHASE_A_GPIO_PORT_LOW, GPIO_MODE_OUTPUT,
    // GPIO_PUPD_NONE,
    // PHASE_A_GPIO_LOW); GPIO_BC(PHASE_A_GPIO_PORT_LOW) = PHASE_A_GPIO_LOW;
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
{
  // full braking shorting all low sides
  phaseALOW();
  phaseBLOW();
  phaseCLOW();
}

void allpwm()
{
  // for stepper_sine
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
