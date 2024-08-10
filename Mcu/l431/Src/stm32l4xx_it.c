/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_it.h"
#include "ADC.h"
#include "targets.h"
#include "IO.h"
//#include "WS2812.h"


extern void transfercomplete();
extern void PeriodElapsedCallback();
extern void interruptRoutine();
extern void tenKhzRoutine();
extern void processDshot();

extern char send_telemetry;
int interrupt_time = 0;
extern char servoPwm;
extern char dshot_telemetry;
extern char armed;
extern char out_put;
extern uint8_t compute_dshot_flag;
extern uint32_t commutation_interval;

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */

  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32L4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel4 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
	  if(LL_DMA_IsActiveFlag_TC1(DMA1) == 1)
	  {
	    LL_DMA_ClearFlag_GI1(DMA1);
	    ADC_DMA_Callback();
	  }

	  if(LL_DMA_IsActiveFlag_TE2(DMA1) == 1)
	  {
	    LL_DMA_ClearFlag_TE2(DMA1);
	  }
}

/**
  * @brief This function handles DMA1 channel5 global interrupt.
  */
void DMA1_Channel4_IRQHandler(void){
#ifdef USE_SERIAL_TELEMETRY
	  if(LL_DMA_IsActiveFlag_TC4(DMA1))
	  {
	    LL_DMA_ClearFlag_GI4(DMA1);
	    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);
	    /* Call function Transmission complete Callback */

	  }
	  else if(LL_DMA_IsActiveFlag_TE4(DMA1))
	  {
		  LL_DMA_ClearFlag_GI4(DMA1);
		  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);
	    /* Call Error function */
	   // USART_TransferError_Callback();
	  }
#endif
}


void DMA1_Channel5_IRQHandler(void)
{
     if (armed && dshot_telemetry) {
        DMA1->IFCR |= DMA_IFCR_CGIF5;
        DMA1_Channel5->CCR = 0x00;
        if (out_put) {
            receiveDshotDma();
            compute_dshot_flag = 2;
        } else {
            sendDshotDma();
            compute_dshot_flag = 1;
        }
        EXTI->SWIER1 |= LL_EXTI_LINE_15;
       return;
    }
    if (LL_DMA_IsActiveFlag_HT5(DMA1)) {
        if (servoPwm) {
            LL_TIM_IC_SetPolarity(IC_TIMER_REGISTER, IC_TIMER_CHANNEL,
                LL_TIM_IC_POLARITY_FALLING);
            LL_DMA_ClearFlag_HT5(DMA1);
        }
    }
    if (LL_DMA_IsActiveFlag_TC5(DMA1) == 1) {
        DMA1->IFCR |= DMA_IFCR_CGIF5;
        DMA1_Channel5->CCR = 0x00;
        transfercomplete();
        EXTI->SWIER1 |= LL_EXTI_LINE_15;
    } else if (LL_DMA_IsActiveFlag_TE5(DMA1) == 1) {
        LL_DMA_ClearFlag_GI5(DMA1);
    }
}
void TIM6_DAC_IRQHandler(void)
{
	if(LL_TIM_IsActiveFlag_UPDATE(TIM6) == 1) {
		LL_TIM_ClearFlag_UPDATE(TIM6);
		tenKhzRoutine();
	}
}


void TIM1_UP_TIM16_IRQHandler(void)
{

	  if(LL_TIM_IsActiveFlag_UPDATE(TIM16) == 1)
	  {

		PeriodElapsedCallback();
	    LL_TIM_ClearFlag_UPDATE(TIM16);

	  }

}

void COMP_IRQHandler(void)
{
	  if(LL_EXTI_IsActiveFlag_0_31(EXTI_LINE) != RESET)
	  {
	    LL_EXTI_ClearFlag_0_31(EXTI_LINE);
	    interruptRoutine();
	  }
}

void EXTI15_10_IRQHandler(void)
{
	  interrupt_time ++;
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_15);
    processDshot();
}

