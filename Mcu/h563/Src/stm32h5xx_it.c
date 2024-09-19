/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32f0xx_it.c
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
#include "stm32f0xx_it.h"

#include "main.h"
/* Private includes
 * ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ADC.h"
#include "targets.h"

extern void transfercomplete();
extern void PeriodElapsedCallback();
extern void interruptRoutine();
extern void doPWMChanges();
extern void tenKhzRoutine();
extern void sendDshotDma();
extern void receiveDshotDma();
extern void processDshot();
extern char send_telemetry;
extern char telemetry_done;
extern char servoPwm;
extern char dshot_telemetry;
extern char armed;
extern char out_put;
extern char compute_dshot_flag;
/* USER CODE END EV */

int interrupt_time = 0;
/******************************************************************************/
/*           Cortex-M0 Processor Interruption and Exception Handlers */
/******************************************************************************/
/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void) { }

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void)
{

    while(1);
    // if (READ_REG(FLASH->ECCDR) == 0xFFFF)
    // {
    // //   __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ECCD);
    //   return;
    // }
}

/**
 * @brief This function handles System service call via SWI instruction.
 */
void SVC_Handler(void) { }

/**
 * @brief This function handles Pendable request for system service.
 */
void PendSV_Handler(void) { }

/**
 * @brief This function handles System tick timer.
 */
void SysTick_Handler(void) { }

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers */
/* Add here the Interrupt Handlers for the used peripherals. */
/* For the available peripheral interrupt handler names, */
/* please refer to the startup file (startup_stm32f0xx.s). */
/******************************************************************************/

/**
 * @brief This function handles DMA1 channel 2 and 3 interrupts.
 */

// DMA1 Channel 2 is used for DSHOT serial telemetry (USART2)
void GPDMA1_Channel2_IRQHandler(void)
{
    // disable DMA1 Channel 2
    LL_DMA_DisableChannel(GPDMA1, LL_DMA_CHANNEL_2);

    // check DMA1 Channel 2 Transfer Complete flag
    if (LL_DMA_IsActiveFlag_TC(GPDMA1, LL_DMA_CHANNEL_2)) {
        // clear DMA1 Channel 2 Global Interrupt flag
        LL_DMA_ClearFlag_TC(GPDMA1, LL_DMA_CHANNEL_2);
    // check DMA1 Channel 2 Data Transfer Error flag
    } else if (LL_DMA_IsActiveFlag_DTE(GPDMA1, LL_DMA_CHANNEL_2)) {
        // clear DMA1 Channel 2 Global Interrupt flag
        LL_DMA_ClearFlag_DTE(GPDMA1, LL_DMA_CHANNEL_2);
    }
}

/**
 * @brief This function handles DMA1 channel 4 and 5 interrupts.
 */
void DMA1_Channel4_5_IRQHandler(void)
{
    /* USER CODE BEGIN DMA1_Channel4_5_IRQn 0 */
#ifdef USE_TIMER_15_CHANNEL_1
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
        EXTI->SWIER |= LL_EXTI_LINE_15;
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
        LL_DMA_ClearFlag_GI5(DMA1);
        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_5);
        transfercomplete();
        EXTI->SWIER |= LL_EXTI_LINE_15;
        return;
    } else if (LL_DMA_IsActiveFlag_TE5(DMA1) == 1) {
        LL_DMA_ClearFlag_GI5(DMA1);
    }
#ifdef USE_PA14_TELEMETRY
    if (LL_DMA_IsActiveFlag_TC4(DMA1)) {
        LL_DMA_ClearFlag_GI4(DMA1);
        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);
        /* Call function Transmission complete Callback */
    } else if (LL_DMA_IsActiveFlag_TE4(DMA1)) {
        LL_DMA_ClearFlag_GI4(DMA1);
        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);
        /* Call Error function */
        // USART_TransferError_Callback();
    }
#endif

#endif

    /* USER CODE END DMA1_Channel4_5_IRQn 0 */

    /* USER CODE BEGIN DMA1_Channel4_5_IRQn 1 */
#ifdef USE_TIMER_3_CHANNEL_1
    if (armed && dshot_telemetry) {
        DMA1->IFCR |= DMA_IFCR_CGIF4;
        DMA1_Channel4->CCR = 0x00;
        if (out_put) {
            receiveDshotDma();
            compute_dshot_flag = 2;
        } else {
            sendDshotDma();
            compute_dshot_flag = 1;
        }
        EXTI->SWIER |= LL_EXTI_LINE_15;
        return;
    }
    if (LL_DMA_IsActiveFlag_HT4(DMA1)) {
        if (servoPwm) {
            LL_TIM_IC_SetPolarity(IC_TIMER_REGISTER, IC_TIMER_CHANNEL,
                LL_TIM_IC_POLARITY_FALLING);
            LL_DMA_ClearFlag_HT4(DMA1);
        }
    }
    if (LL_DMA_IsActiveFlag_TC4(DMA1) == 1) {
        LL_DMA_ClearFlag_GI4(DMA1);
        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);
        transfercomplete();
        EXTI->SWIER |= LL_EXTI_LINE_15;
    } else if (LL_DMA_IsActiveFlag_TE4(DMA1) == 1) {
        LL_DMA_ClearFlag_GI4(DMA1);
    }
#endif
}

/**
 * @brief This function handles ADC and COMP interrupts (COMP interrupts
 * through EXTI lines 21 and 22).
 */
// The comparator triggers interruptRoutine
void ADC1_COMP_IRQHandler(void)
{
    // TODO, figure out rising/falling
    if (LL_EXTI_IsActiveFallingFlag_0_31(LL_EXTI_LINE_21) != RESET) {
        LL_EXTI_ClearFallingFlag_0_31(LL_EXTI_LINE_21);
        interruptRoutine();
    }
    //
}

/**
 * @brief This function handles TIM6 global and DAC underrun error interrupts.
 */
// TIM6 is used to trigger the tenKhzRoutine
void TIM6_DAC_IRQHandler(void)
{
    if (LL_TIM_IsActiveFlag_UPDATE(TIM6) == 1) {
        LL_TIM_ClearFlag_UPDATE(TIM6);
        tenKhzRoutine();
    }
}

/**
 * @brief This function handles TIM14 global interrupt.
 */
// TIM14 is the COM_TIMER
// void TIM14_IRQHandler(void)
// {
//     LL_TIM_ClearFlag_UPDATE(TIM14);
//     PeriodElapsedCallback();
// }

/**
 * @brief This function handles TIM16 global interrupt.
 */
void TIM16_IRQHandler(void)
{
    /* USER CODE BEGIN TIM16_IRQn 0 */

    /* USER CODE END TIM16_IRQn 0 */
    /* USER CODE BEGIN TIM16_IRQn 1 */

    /* USER CODE END TIM16_IRQn 1 */
}

/**
 * @brief This function handles USART1 global interrupt / USART1 wake-up
 * interrupt through EXTI line 25.
 */
void USART1_IRQHandler(void)
{
    /* USER CODE BEGIN USART1_IRQn 0 */

    /* USER CODE END USART1_IRQn 0 */
    /* USER CODE BEGIN USART1_IRQn 1 */

    /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void TIM15_IRQHandler(void)
{
    if (LL_TIM_IsActiveFlag_CC1(TIM15) == 1) {
        LL_TIM_ClearFlag_CC1(TIM15);
    }

    if (LL_TIM_IsActiveFlag_UPDATE(TIM15) == 1) {
        LL_TIM_ClearFlag_UPDATE(TIM15);
        // update_interupt++;
    }
}

void TIM3_IRQHandler(void)
{
    if (LL_TIM_IsActiveFlag_CC1(TIM3) == 1) {
        LL_TIM_ClearFlag_CC1(TIM3);
    }

    if (LL_TIM_IsActiveFlag_UPDATE(TIM3) == 1) {
        LL_TIM_ClearFlag_UPDATE(TIM3);
        // update_interupt++;
    }
}

// DMA1 Channel 1 is used for triggering ADC_DMA_Callback
// void DMA1_Channel1_IRQHandler(void)
// {
//     if (LL_DMA_IsActiveFlag_TC(GPDMA1, LL_DMA_CHANNEL_1) == 1) {
//         /* Clear flag DMA global interrupt */
//         /* (global interrupt flag: half transfer and transfer complete flags) */
//         LL_DMA_ClearFlag_TC(GPDMA1, LL_DMA_CHANNEL_1);
//         ADC_DMA_Callback();
//         /* Call interruption treatment function */
//         //   AdcDmaTransferComplete_Callback();
//     }

//     /* Check whether DMA transfer error caused the DMA interruption */
//     if (LL_DMA_IsActiveFlag_DTE(GPDMA1, LL_DMA_CHANNEL_1) == 1) {
//         /* Clear flag DMA transfer error */
//         LL_DMA_ClearFlag_DTE(GPDMA1, LL_DMA_CHANNEL_1);

//         /* Call interruption treatment function */
//     }
// }

void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
    if (LL_TIM_IsActiveFlag_CC1(TIM1) == 1) {
        LL_TIM_ClearFlag_CC1(TIM1);
    }

    if (LL_TIM_IsActiveFlag_UPDATE(TIM1) == 1) {
        LL_TIM_ClearFlag_UPDATE(TIM1);
        //  doPWMChanges();
    }
}

void EXTI4_15_IRQHandler(void)
{
    // TODO, figure out rising/falling
    LL_EXTI_ClearFallingFlag_0_31(LL_EXTI_LINE_15);
    processDshot();
}

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF
 * FILE****/
