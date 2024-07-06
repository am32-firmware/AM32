/**
  ******************************************************************************
  * @file    stm32l476g_eval_idd.c
  * @author  MCD Application Team
  * @brief   This file includes the Idd measurement driver for
  *          STM32L476G-Eval board.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32l476g_eval_idd.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L476G_EVAL
  * @{
  */
  
/** @defgroup STM32L476G_EVAL_IDD STM32L476G_EVAL IDD
  * @brief This file includes the Idd driver for STM32L476G-EVAL board.
  *        It allows user to measure MCU Idd current on board, especially in 
  *        different low power modes.
  * @{
  */

/** @defgroup STM32L476G_EVAL_IDD_Private_Defines Private Defines
  * @{
  */

/**
  * @}
  */


/** @defgroup STM32L476G_EVAL_IDD_Private_Variables Private Variables
  * @{
  */
ADC_HandleTypeDef      heval_AdcHandle;
ADC_ChannelConfTypeDef heval_AdcChannelConfig;

/**
  * @}
  */

/** @defgroup STM32L476G_EVAL_IDD_Private_Functions Private Functions
  * @{
  */

/**
  * @}
  */

/** @defgroup STM32L476G_EVAL_IDD_Exported_Functions Exported Functions
  * @{
  */

/**
  * @brief  Configures IDD measurement components.
  * @retval BSP_IDD_OK if no problem during initialization
  */
uint8_t BSP_IDD_Init(void)
{ 
  uint8_t ret;
  GPIO_InitTypeDef GPIO_InitStruct;
  
  /* Configure IDD_CNT_EN PF 10 */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  GPIO_InitStruct.Pin   = GPIO_PIN_10;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /* Set IDD_CNT_EN  PF 10 to high */ 
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);
  
  /* Configure WakeUp External Pin */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  GPIO_InitStruct.Pin   = GPIO_PIN_5;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Mode  = GPIO_MODE_IT_RISING;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* Enable and set WakeUp EXTI Interrupt to the highest priority */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0x0, 0x0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  
  heval_AdcHandle.Instance          = ADC1;
  /* DeInitialized ADC for IDD measurement */ 
  if (HAL_ADC_DeInit(&heval_AdcHandle) != HAL_OK)
  {
    /* ADC de-initialization Error */
    ret = BSP_IDD_ERROR;
  }
  else
  {
    /* ADC initialization for IDD measurement */
    /* Handle Initialization */
    heval_AdcHandle.Init.ClockPrescaler        = ADC_CLOCK_ASYNC_DIV1;          /* Asynchronous clock mode, input ADC clock not divided */
    heval_AdcHandle.Init.Resolution            = ADC_RESOLUTION_12B;            /* 12-bit resolution for converted data */
    heval_AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;           /* Right-alignment for converted data */
    heval_AdcHandle.Init.ScanConvMode          = DISABLE;                       /* Sequencer disabled (ADC conversion on only 1 channel: channel set on rank 1) */
    heval_AdcHandle.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;           /* EOC flag picked-up to indicate conversion end */
    heval_AdcHandle.Init.LowPowerAutoWait      = DISABLE;                       /* Auto-delayed conversion feature disabled */
    heval_AdcHandle.Init.ContinuousConvMode    = DISABLE;                       /* Continuous mode disabled to have only 1 conversion at each conversion trig */
    heval_AdcHandle.Init.NbrOfConversion       = 1;                             /* Parameter discarded because sequencer is disabled */
    heval_AdcHandle.Init.DiscontinuousConvMode = DISABLE;                       /* Parameter discarded because sequencer is disabled */
    heval_AdcHandle.Init.NbrOfDiscConversion   = 1;                             /* Parameter discarded because sequencer is disabled */
    heval_AdcHandle.Init.ExternalTrigConv      = ADC_SOFTWARE_START;            /* Software start to trig the 1st conversion manually, without external event */
    heval_AdcHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE; /* Parameter discarded because software trigger chosen */
    heval_AdcHandle.Init.DMAContinuousRequests = DISABLE;                       /* DMA one-shot mode selected (not applied to this example) */
    heval_AdcHandle.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;      /* DR register is overwritten with the last conversion result in case of overrun */
    heval_AdcHandle.Init.OversamplingMode      = DISABLE;                       /* No oversampling */

    /* Channel Config Initialization */
    heval_AdcChannelConfig.Channel                      = ADC_CHANNEL_10;                /* Sampled channel number */
    heval_AdcChannelConfig.Rank                         = ADC_REGULAR_RANK_1;          /* Rank of sampled channel number ADCx_CHANNEL */
    heval_AdcChannelConfig.SamplingTime                 = ADC_SAMPLETIME_6CYCLES_5;                /* Sampling time (number of clock cycles unit) */
    heval_AdcChannelConfig.SingleDiff                   = ADC_SINGLE_ENDED;            /* Single-ended input channel */
    heval_AdcChannelConfig.OffsetNumber                 = ADC_OFFSET_NONE;             /* No offset subtraction */ 
    heval_AdcChannelConfig.Offset                       = 0;                                 /* Parameter discarded because offset correction is disabled */

    /* Initialization OK */
    ret = BSP_IDD_OK;
  }
  
  return ret;
}

/**
  * @brief  Board Enter Stop Mode 2.
  * @retval None.
  */
void BSP_IDD_EnterStopMode2(void)
{
#if (PREFETCH_ENABLE != 0)
  /* Disable Prefetch Buffer */
  __HAL_FLASH_PREFETCH_BUFFER_DISABLE();
#endif /* PREFETCH_ENABLE */

  /* Set IDD_CNT_EN PF 10 to low */ 
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET);

  /* Enter STOP 2 mode */
  HAL_SuspendTick();

  HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
}

/**
  * @brief  Board Enter Stop Mode 1.
  * @retval None.
  */
void BSP_IDD_EnterStopMode1(void)
{
  /* Enter STOP 1 mode */
  HAL_SuspendTick();

  HAL_PWREx_EnterSTOP1Mode(PWR_STOPENTRY_WFE);
  
  /* Return from STOP 1 mode */
  HAL_PWREx_DisableLowPowerRunMode();
  
}

/**
  * @brief  Board Exit Stop Mode.
  * @retval None.
  */
void BSP_IDD_ExitStopMode(void)
{
  HAL_ResumeTick();

#if (PREFETCH_ENABLE != 0)
  /* Re-enable Prefetch Buffer */
  __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
#endif /* PREFETCH_ENABLE */
}

/**
  * @brief  Board Enter Sleep Mode.
  * @retval None.
  */
void BSP_IDD_EnterSleepMode(void)
{
#if (PREFETCH_ENABLE != 0)
  /* Disable Prefetch Buffer */
  __HAL_FLASH_PREFETCH_BUFFER_DISABLE();
#endif /* PREFETCH_ENABLE */
  
  /* Set IDD_CNT_EN PF 10 to low */ 
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET);

  RCC->AHB1SMENR  = 0x0; 
  RCC->AHB2SMENR  = 0x0;
  RCC->AHB3SMENR  = 0x0;
  
  RCC->APB1SMENR1 = 0x0;
  RCC->APB1SMENR2 = 0x0; 
  RCC->APB2SMENR  = 0x0; 

  /* Enter SLEEP mode */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/10);

  HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
}

/**
  * @brief  Board Exit Sleep Mode.
  * @retval None.
  */
void BSP_IDD_ExitSleepMode(void)
{
  /* Reset tick period to 1 ms */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
  
#if (PREFETCH_ENABLE != 0)
  /* Re-enable Prefetch Buffer */
  __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
#endif /* PREFETCH_ENABLE */
}

/**
  * @brief  Board Enter Run Mode.
  * @retval None.
  */
void BSP_IDD_EnterRunMode(void)
{
  /* Set IDD_CNT_EN  PF 10 to low */ 
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET);

  RCC->AHB1SMENR  = 0x0; 
  RCC->AHB2SMENR  = 0x0;
  RCC->AHB3SMENR  = 0x0;
  
  RCC->APB1SMENR1 = 0x0;
  RCC->APB1SMENR2 = 0x0; 
  RCC->APB2SMENR  = 0x0; 
}

/**
  * @brief  Board Enter Standby Mode.
  * @retval None.
  */
void BSP_IDD_EnterStandbyMode(void)
{
  HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN5);

  /* Clear wake up Flag */
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF5);
  
  /* Enable wakeup pin WKUP5 */
  HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN5_HIGH);
 
  HAL_PWREx_EnableGPIOPullDown(PWR_GPIO_F, PWR_GPIO_BIT_10);
  HAL_PWREx_EnablePullUpPullDownConfig();

  HAL_PWR_EnterSTANDBYMode();
}

/**
  * @brief  Get IDD current consumption value.
  * @param  LowPowerMode: Low Power Mode (STOP, SLEEP, RUN, STANDBY).
  * @param  Calibration: Offset Calibration.
  * @param  IddValue: Pointer on u32 to store Idd. Value unit is nA.
  * @retval BSP_IDD_OK if no problem during initialization
  */
uint32_t BSP_IDD_GetValue(IDD_ModeTypdef LowPowerMode, uint32_t Calibration, uint32_t *IddValue)
{
  uint32_t ADCxConvertedValue = 0;
  GPIO_InitTypeDef GPIO_InitStruct;
  uint32_t index;
  
  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* ADC Periph clock enable */
  __HAL_RCC_ADC_CLK_ENABLE();
  /* ADC Periph interface clock configuration */
  __HAL_RCC_ADC_CONFIG(RCC_ADCCLKSOURCE_SYSCLK);
  /* Enable GPIO clock ****************************************/
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*##-2- Configure peripheral GPIO ##########################################*/
  /* ADC Channel GPIO pin configuration */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  if (HAL_ADC_DeInit(&heval_AdcHandle) != HAL_OK)
  {
    /* ADC de-initialization Error */
    return BSP_IDD_ERROR;
  }

  if (HAL_ADC_Init(&heval_AdcHandle) != HAL_OK)
  {
    /* ADC initialization Error */
    return BSP_IDD_ERROR;
  }
  
  
  if (HAL_ADC_ConfigChannel(&heval_AdcHandle, &heval_AdcChannelConfig) != HAL_OK)
  {
    /* Channel Configuration Error */
    return BSP_IDD_ERROR;
  }

  /*##-3- Calibrate ADC then Start the conversion process ####################*/  
  if (HAL_ADCEx_Calibration_Start(&heval_AdcHandle, ADC_SINGLE_ENDED) !=  HAL_OK)
  {
    /* ADC Calibration Error */
    return BSP_IDD_ERROR;
  }

  ADCxConvertedValue = 0;
  /* Compute an average of 4 measures */
  for (index = 0; index < 4; index++)
  {  
    if (HAL_ADC_Start(&heval_AdcHandle) != HAL_OK)
    {
      /* Start Conversation Error */
      return BSP_IDD_ERROR;
    }
    
    if (HAL_ADC_PollForConversion(&heval_AdcHandle, 100) != HAL_OK)
    {
      /* End Of Conversion flag not set on time */
      return BSP_IDD_ERROR;
    }
    
      /* Check if the continuous conversion of regular channel is finished */
    if ((HAL_ADC_GetState(&heval_AdcHandle) & HAL_ADC_STATE_REG_EOC) == HAL_ADC_STATE_REG_EOC)
    {
      /*##-5- Get the converted value of regular channel  ########################*/
      ADCxConvertedValue += HAL_ADC_GetValue(&heval_AdcHandle);
    }
  }  

  /* Set IDD_CNT_EN  PF 10 to high */ 
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);

  /* Calculate the average of the converted value */
  ADCxConvertedValue >>= 2;

  if(LowPowerMode != BSP_IDD_MODE_CALIBRATION)
  {
    if(ADCxConvertedValue <= Calibration)
    {
      ADCxConvertedValue = 0;
    }
    else
    {
      ADCxConvertedValue -= Calibration;
    }

    /* V_shunt = (Vdd(in mV) / ADC_FULL_SCALE) * ADCxConvertedValue / IDD_AMPLI_GAIN in mV */
    /* V_shunt = 1000 * (Vdd / ADC_FULL_SCALE) * ADCxConvertedValue / IDD_AMPLI_GAIN in uV */
    /* V_shunt = (20 * ADCxConvertedValue * 3300 ) / 4096 is the voltage in uV. */
    /* With a resistor @ 1000 Ohm, Idd = V_shunt / 1000 is the current in nA
      or, seen differently, Idd = V_shunt is the current in nA. */
    IddValue[0] = (20 * ADCxConvertedValue * EVAL_IDD_VDD ) / EVAL_IDD_FULL_SCALE; 
  }
  else
  {
    /* Return Offset Calibration */
    IddValue[0] = ADCxConvertedValue;
  }
  
  return BSP_IDD_OK;
}


/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */



