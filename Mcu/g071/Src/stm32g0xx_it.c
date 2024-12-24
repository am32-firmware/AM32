/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32g0xx_it.c
 * @brief   Interrupt Service Routines.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

/* Includes
 * ------------------------------------------------------------------*/
#include "stm32g0xx_it.h"

#include "main.h"
/* Private includes
 * ----------------------------------------------------------*/
#include "ADC.h"
#include "IO.h"
#include "WS2812.h"
#include "targets.h"
#include "comparator.h"
#include "common.h"

/* USER CODE END Includes */

/* Private typedef
 * -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define
 * ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro
 * -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables
 * ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes
 * -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code
 * ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables
 * --------------------------------------------------------*/

/* USER CODE BEGIN EV */
extern void transfercomplete();
extern void PeriodElapsedCallback();
extern void interruptRoutine();
extern void tenKhzRoutine();
extern void processDshot();

extern char send_telemetry;
uint16_t interrupt_time = 0;
extern char servoPwm;
extern char dshot_telemetry;
extern char armed;
extern char out_put;
extern uint8_t compute_dshot_flag;

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0+ Processor Interruption and Exception Handlers */
/******************************************************************************/
/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void)
{
    /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

    /* USER CODE END NonMaskableInt_IRQn 0 */
    /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

    /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void)
{
    /* USER CODE BEGIN HardFault_IRQn 0 */

    /* USER CODE END HardFault_IRQn 0 */
    while (1) {
        /* USER CODE BEGIN W1_HardFault_IRQn 0 */
        /* USER CODE END W1_HardFault_IRQn 0 */
    }
}

/**
 * @brief This function handles System service call via SWI instruction.
 */
void SVC_Handler(void)
{
    /* USER CODE BEGIN SVC_IRQn 0 */

    /* USER CODE END SVC_IRQn 0 */
    /* USER CODE BEGIN SVC_IRQn 1 */

    /* USER CODE END SVC_IRQn 1 */
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
/* STM32G0xx Peripheral Interrupt Handlers */
/* Add here the Interrupt Handlers for the used peripherals. */
/* For the available peripheral interrupt handler names, */
/* please refer to the startup file (startup_stm32g0xx.s). */
/******************************************************************************/

/**
 * @brief This function handles DMA1 channel 1 interrupt.
 */
void DMA1_Channel1_IRQHandler(void)
{
    if (armed && dshot_telemetry) {
        DMA1->IFCR |= DMA_IFCR_CGIF1;
        DMA1_Channel1->CCR = 0x00;
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
    /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */
    if (LL_DMA_IsActiveFlag_HT1(DMA1)) {
        if (servoPwm) {
            LL_TIM_IC_SetPolarity(IC_TIMER_REGISTER, IC_TIMER_CHANNEL,
                LL_TIM_IC_POLARITY_FALLING);
            LL_DMA_ClearFlag_HT1(DMA1);
        }
    }
    if (LL_DMA_IsActiveFlag_TC1(DMA1) == 1) {
        DMA1->IFCR |= DMA_IFCR_CGIF1;
        DMA1_Channel1->CCR = 0x00;
        transfercomplete();
        EXTI->SWIER1 |= LL_EXTI_LINE_15;
    } else if (LL_DMA_IsActiveFlag_TE1(DMA1) == 1) {
        LL_DMA_ClearFlag_GI1(DMA1);
    }
}

/**
 * @brief This function handles DMA1 channel 2 and channel 3 interrupts.
 */
void DMA1_Channel2_3_IRQHandler(void)
{
    if (LL_DMA_IsActiveFlag_TC2(DMA1) == 1) {
        LL_DMA_ClearFlag_GI2(DMA1);
        ADC_DMA_Callback();
    }
    if (LL_DMA_IsActiveFlag_TE2(DMA1) == 1) {
        LL_DMA_ClearFlag_TE2(DMA1);
    }

    if (LL_DMA_IsActiveFlag_TC3(DMA1)) {
        send_telemetry = 0;
        LL_DMA_ClearFlag_GI3(DMA1);
        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
        /* Call function Transmission complete Callback */
    } else if (LL_DMA_IsActiveFlag_TE3(DMA1)) {
        LL_DMA_ClearFlag_GI3(DMA1);
        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
        /* Call Error function */
        // USART_TransferError_Callback();
    }
}

/**
 * @brief This function handles ADC1, COMP1 and COMP2 interrupts (COMP
 * interrupts through EXTI lines 17 and 18).
 */
void ADC1_COMP_IRQHandler(void)
{
  if (LL_EXTI_IsActiveFallingFlag_0_31(LL_EXTI_LINE_18)) {
    if((INTERVAL_TIMER->CNT) > (average_interval >> 1)){
      LL_EXTI_ClearFallingFlag_0_31(LL_EXTI_LINE_18);
      interruptRoutine();
    }else{
      if(getCompOutputLevel() == rising){
          LL_EXTI_ClearFallingFlag_0_31(LL_EXTI_LINE_18);
          return;
      }
    }
    return;
  }

  if (LL_EXTI_IsActiveRisingFlag_0_31(LL_EXTI_LINE_18)) {
    if((INTERVAL_TIMER->CNT) > (average_interval >> 1)){
      LL_EXTI_ClearRisingFlag_0_31(LL_EXTI_LINE_18);
      interruptRoutine();
    }else{
      if(getCompOutputLevel() == rising){
          LL_EXTI_ClearRisingFlag_0_31(LL_EXTI_LINE_18);
          return;
      }
    }
    return;
  }
  if (LL_EXTI_IsActiveFallingFlag_0_31(LL_EXTI_LINE_17)) {
    if((INTERVAL_TIMER->CNT) > (average_interval >> 1)){
      LL_EXTI_ClearFallingFlag_0_31(LL_EXTI_LINE_17);
      interruptRoutine();
    }else{
      if(getCompOutputLevel() == rising){
          LL_EXTI_ClearFallingFlag_0_31(LL_EXTI_LINE_17);
          return;
      }
    }
    return;
  }

  if (LL_EXTI_IsActiveRisingFlag_0_31(LL_EXTI_LINE_17)) {
    if((INTERVAL_TIMER->CNT) > (average_interval >> 1)){
      LL_EXTI_ClearRisingFlag_0_31(LL_EXTI_LINE_17);
      interruptRoutine();
    }else{
      if(getCompOutputLevel() == rising){
          LL_EXTI_ClearRisingFlag_0_31(LL_EXTI_LINE_17);
          return;
      }
    }
    return;
  }
}

/**
 * @brief This function handles TIM3 global interrupt.
 */
void TIM3_IRQHandler(void)
{
    /* USER CODE BEGIN TIM3_IRQn 0 */
    if (LL_TIM_IsActiveFlag_CC1(TIM3) == 1) {
        LL_TIM_ClearFlag_CC1(TIM3);
    }

    if (LL_TIM_IsActiveFlag_UPDATE(TIM3) == 1) {
        LL_TIM_ClearFlag_UPDATE(TIM3);
        // update_interupt++;
    }
    /* USER CODE END TIM3_IRQn 0 */
    /* USER CODE BEGIN TIM3_IRQn 1 */

    /* USER CODE END TIM3_IRQn 1 */
}

void TIM16_IRQHandler(void)
{
    /* USER CODE BEGIN TIM3_IRQn 0 */
    if (LL_TIM_IsActiveFlag_CC1(TIM16) == 1) {
        LL_TIM_ClearFlag_CC1(TIM16);
    }

    if (LL_TIM_IsActiveFlag_UPDATE(TIM16) == 1) {
        LL_TIM_ClearFlag_UPDATE(TIM16);
        // update_interupt++;
    }
    /* USER CODE END TIM3_IRQn 0 */
    /* USER CODE BEGIN TIM3_IRQn 1 */

    /* USER CODE END TIM3_IRQn 1 */
}

void TIM6_DAC_LPTIM1_IRQHandler(void)
{
    /* USER CODE BEGIN TIM6_DAC_LPTIM1_IRQn 0 */
    if (LL_TIM_IsActiveFlag_UPDATE(TIM6) == 1) {
        LL_TIM_ClearFlag_UPDATE(TIM6);
        tenKhzRoutine();
    }
    /* USER CODE END TIM6_DAC_LPTIM1_IRQn 0 */

    /* USER CODE BEGIN TIM6_DAC_LPTIM1_IRQn 1 */

    /* USER CODE END TIM6_DAC_LPTIM1_IRQn 1 */
}

/**
 * @brief This function handles TIM14 global interrupt.
 */
void TIM14_IRQHandler(void)
{
    interrupt_time = UTILITY_TIMER->CNT;
    PeriodElapsedCallback();
    LL_TIM_ClearFlag_UPDATE(TIM14);
    interrupt_time = ((uint16_t)UTILITY_TIMER->CNT) - interrupt_time;
}

/* USER CODE BEGIN 1 */
void DMA1_Ch4_7_DMAMUX1_OVR_IRQHandler(void)
{
    /* USER CODE BEGIN DMA1_Ch4_7_DMAMUX1_OVR_IRQn 0 */
    if (LL_DMA_IsActiveFlag_HT6(DMA1)) {
    }
    if (LL_DMA_IsActiveFlag_TC6(DMA1) == 1) {
        LL_DMA_ClearFlag_GI6(DMA1);

        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_6);
        LL_TIM_DisableAllOutputs(TIM16);
        LL_TIM_DisableCounter(TIM16);
        dma_busy = 0;
    } else if (LL_DMA_IsActiveFlag_TE6(DMA1) == 1) {
        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_6);
        LL_TIM_DisableAllOutputs(TIM16);
        LL_TIM_DisableCounter(TIM16);
        dma_busy = 0;
        LL_DMA_ClearFlag_GI6(DMA1);
    }
}

void EXTI4_15_IRQHandler(void)
{
    LL_EXTI_ClearRisingFlag_0_31(LL_EXTI_LINE_15);
    processDshot();
}
