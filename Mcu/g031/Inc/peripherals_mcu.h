/*
 * peripherals_mcu.h — STM32G031 register macro sheet + private declarations.
 */

#ifndef PERIPHERALS_MCU_H_
#define PERIPHERALS_MCU_H_

// load-bearing: this family's peripherals.c calls ADC_Init/enableADC_DMA/
// activateADC under #ifdef USE_ADC and includes only peripherals.h itself
#include "ADC.h"

#include "../../shared/peripherals_macros_stm32_ll.h"

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void MX_ADC1_Init(void);
void MX_COMP2_Init(void);
void MX_COMP1_Init(void);
void MX_TIM1_Init(void);
void MX_TIM2_Init(void);
void MX_TIM3_Init(void);
void MX_TIM14_Init(void);
void MX_TIM17_Init(void);
void MX_TIM16_Init(void);
void MX_TIM6_Init(void);
void reloadWatchDogCounter(void);

#endif /* PERIPHERALS_MCU_H_ */
