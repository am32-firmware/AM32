/*
 * peripherals_mcu.h — STM32L431 register macro sheet + private declarations.
 */

#ifndef PERIPHERALS_MCU_H_
#define PERIPHERALS_MCU_H_

#include "../../shared/peripherals_macros_stm32_ll.h"

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void MX_COMP1_Init(void);
void MX_COMP2_Init(void);
void MX_TIM1_Init(void);
void MX_TIM2_Init(void);
void MX_TIM16_Init(void);
void MX_TIM14_Init(void);
void MX_TIM6_Init(void);
void MX_TIM7_Init(void);
void init_OPAMP(void);
void reloadWatchDogCounter(void);
void UN_TIM_Init(void);
void LED_GPIO_init(void);

#endif /* PERIPHERALS_MCU_H_ */
