/*
 * comparator.c
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 *      Modified by TempersLee June 21, 2024
 */

#include "comparator.h"

#include "targets.h"

#ifndef USE_PA2_AS_COMP
uint8_t current_exti_line = 3;
uint16_t current_Gpio_Pin = GPIO_Pin_3;
#endif

uint8_t getCompOutputLevel()
{
#ifdef USE_PA2_AS_COMP
    return (GPIOA->INDR & GPIO_Pin_2);
#else
    return (GPIOA->INDR & current_Gpio_Pin);
#endif
}

void maskPhaseInterrupts()
{
#ifdef USE_PA2_AS_COMP
    EXTI->INTENR &= ~(1 << 2);  //mask
    EXTI->INTFR  = (1<<2);      //clear_flag
#else
    EXTI->INTENR &= ~(0x3 << 3); //mask
    EXTI->INTFR  = (0x3<<3);     //clear_flag
#endif
}

void enableCompInterrupts()
{
#ifdef USE_PA2_AS_COMP
    EXTI->INTENR |= (1 << 2);
#else
    EXTI->INTFR  = (3<<3);      //clear_flag
    EXTI->INTENR |= (1 << current_exti_line);
#endif
}

void changeCompInput()
{
    //	TIM3->CNT = 0;
    //	HAL_COMP_Stop_IT(&hcomp1);            // done in comparator interrupt
    // routine
    if(step == 1 || step == 4) // c floating
    {
        OPA->CR = 0x01;
#ifndef USE_PA2_AS_COMP
        current_exti_line = 3;
        current_Gpio_Pin = GPIO_Pin_3;
#endif
    }
    if(step == 2 || step == 5) // a floating
    {
        OPA->CR = 0x70;
#ifndef USE_PA2_AS_COMP
        current_exti_line = 4;
        current_Gpio_Pin = GPIO_Pin_4;
#endif
    }
    if(step == 3 || step == 6) // b floating
    {
        OPA->CR = 0x30;
#ifndef USE_PA2_AS_COMP
        current_exti_line = 4;
        current_Gpio_Pin = GPIO_Pin_4;
#endif
    }
    if (rising)
    {
#ifdef  USE_PA2_AS_COMP
        EXTI->RTENR = 0;
        EXTI->FTENR = (1<<2);  //反向
#else
        EXTI->RTENR = 0;
        EXTI->FTENR = (1<<current_exti_line);  //反向
#endif
    }
    else
    {
#ifdef USE_PA2_AS_COMP
        EXTI->FTENR = 0;
        EXTI->RTENR = (1<<2);  //反向
#else
        EXTI->FTENR = 0;
        EXTI->RTENR = (1<<current_exti_line);  //反向
#endif
    }
}
