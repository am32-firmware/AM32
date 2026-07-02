/*
 * peripherals_mcu.h — AT32F415 register macro sheet + private declarations.
 */

#ifndef PERIPHERALS_MCU_H_
#define PERIPHERALS_MCU_H_

#include "../../shared/peripherals_macros_at32.h"

void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void AT_COMP_Init(void);
void TIM1_Init(void);
void TIM4_Init(void);
void system_clock_config(void);
void TIM11_Init(void);
void TIM9_Init(void);
void TIM10_Init(void);
void reloadWatchDogCounter(void);
void UN_TIM_Init(void);
void LED_GPIO_init(void);

#endif /* PERIPHERALS_MCU_H_ */
