/**
  ******************************************************************************
  * File   : Templates/at32f4xx_it.h 
  * Version: V1.3.0
  * Date   : 2021-03-18
  * Brief  : Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AT32F421_IT_H
#define __AT32F421_IT_H

/* Includes ------------------------------------------------------------------*/
#include "at32f421.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
void ADC1_CMP_IRQHandler(void);
void TMR14_GLOBAL_IRQHandler(void);
void TMR15_GLOBAL_IRQHandler(void);
void TMR16_GLOBAL_IRQHandler(void);
void TMR17_GLOBAL_IRQHandler(void);
void TMR3_GLOBAL_IRQHandler(void);
void DMA1_Channel5_4_IRQHandler(void);
void DMA1_Channel1_IRQHandler(void);
void DMA1_Channel3_2_IRQHandler(void);
#endif /* __AT32F4XX_IT_H */
