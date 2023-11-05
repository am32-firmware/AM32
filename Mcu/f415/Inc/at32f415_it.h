/**
 ******************************************************************************
 * File   : Templates/at32f4xx_it.h
 * Version: V1.3.0
 * Date   : 2021-03-18
 * Brief  : Main Interrupt Service Routines.
 *          This file provides template for all exceptions handler and
 *peripherals interrupt service routine.
 ******************************************************************************
 */

/* Define to prevent recursive inclusion
 * -------------------------------------*/
#ifndef __AT32F415_IT_H
#define __AT32F415_IT_H

/* Includes
 * ------------------------------------------------------------------*/
#include "at32f415.h"

/* Exported types
 * ------------------------------------------------------------*/
/* Exported constants
 * --------------------------------------------------------*/
/* Exported macro
 * ------------------------------------------------------------*/
/* Exported functions -------------------------------------------------------
 */

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
void CMP1_IRQHandler(void);
void TMR4_GLOBAL_IRQHandler(void);
void TMR1_TRG_HALL_TMR11_IRQHandler(void);
void TMR1_BRK_TMR9_IRQHandler(void);
void TMR1_OVF_TMR10_IRQHandler(void);
void EXINT15_10_IRQHandler(void);
void TMR3_GLOBAL_IRQHandler(void);
void DMA1_Channel4_IRQHandler(void);
void DMA1_Channel6_IRQHandler(void);
#endif /* __AT32F4XX_IT_H */
