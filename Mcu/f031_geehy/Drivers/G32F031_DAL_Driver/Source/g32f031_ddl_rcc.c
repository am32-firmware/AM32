/**
  *
  * @file    g32f031_ddl_rcc.c
  * @brief   RCC DDL module driver.
  *
  * @attention
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *
  * 1. Redistributions of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of the copyright holder nor the names of its contributors
  *    may be used to endorse or promote products derived from this software without
  *    specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
  * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
  * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
  * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
  * OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  * The original code has been modified by Geehy Semiconductor.
  *
  * Copyright (c) 2017 STMicroelectronics.
 *  Copyright (C) 2026 Geehy Semiconductor
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file in
  * the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  */
#if defined(USE_FULL_DDL_DRIVER)

/* Includes ------------------------------------------------------------------*/
#include "g32f031_ddl_rcc.h"
#ifdef  USE_FULL_ASSERT
  #include "g32_assert.h"
#else
  #define ASSERT_PARAM(_PARAM_) ((void)0U)
#endif
/** @addtogroup G32F031_DDL_Driver
  * @{
  */

#if defined(RCC)

/** @addtogroup RCC_DDL RCC
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/** @defgroup RCC_DDL_Private_Functions RCC Private Functions
  * @{
  */
static uint32_t RCC_GetSystemClockFreq(void);
static uint32_t RCC_GetHCLKClockFreq(uint32_t SYSCLK_Frequency);
static uint32_t RCC_GetPCLKClockFreq(uint32_t HCLK_Frequency);
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup RCC_DDL_Exported_Functions RCC Exported Functions
  * @{
  */

/** @addtogroup RCC_DDL_EF_Init
  * @{
  */

/**
  * @brief  Reset the RCC clock configuration to the default reset state.
  * @note   The default reset state of the clock configuration is given below:
  *         - HSI clock is ON and used as system clock source
  *         - AHB, APB prescaler are divided by 1.
  *         - MCO OFF
  *         - All interrupts disabled
  *         - Reset all wakeup-configuration
  * @note   This function doesn't modify the configuration of the
  *         - Peripheral clocks
  *         - LSI, RTC clocks
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: RCC registers are de-initialized
  *          - ERROR: not applicable
  */
ErrorStatus DDL_RCC_DeInit(void)
{
  DDL_RCC_Unlock();

  /* If LSI clock is set as system clock source, switch to HSI clock*/
  if(DDL_RCC_GetSysClkSource())
  {
    if(!DDL_RCC_HSI_IsReady())
    {
      /* Enalbe HSICLK */
      DDL_RCC_HSI_Enable();
      /* Wait for HSI_ON READY */
      while(DDL_RCC_HSI_IsReady() != 1U)
      {}
    }
    /* Set HSI clock as system source clock */
    DDL_RCC_SetSysClkSource(DDL_RCC_SYS_CLKSOURCE_HSI);
    while(DDL_RCC_GetSysClkSource() != RCC_CFG_SWSTS_HSI)
    {}
  }
  else
  {
    /* Do nothing */
  }

  /* Disable all interrupts */
  WRITE_REG(RCC->CFG, 0x00000000U);
  /* Disable all interrupts */
  WRITE_REG(RCC->IER, 0x00000000U);
  /* Clear all interrupt flags */
  WRITE_REG(RCC->ISR, 0x00000000U);
  /* Clear all reset flags, disable PVD reset, disable LOCKUP reset */
  WRITE_REG(RCC->RSTCSR, 0x00000000U);

  DDL_RCC_Lock();

  return SUCCESS;
}

/**
  * @}
  */

/** @addtogroup RCC_DDL_EF_Get_Freq
  * @{
  */

/**
  * @brief  Return the frequencies of different on chip clocks;  System, AHB, APB buses clocks
  * @note   Each time SYSCLK, HCLK, PCLK clock changes, this function
  *         must be called to update structure fields. Otherwise, any
  *         configuration based on this function will be incorrect.
  * @param  RCC_Clocks pointer to a @ref DDL_RCC_ClocksTypeDef structure which will hold the clocks frequencies
  * @retval None
  */
void DDL_RCC_GetSysctrlClocksFreq(DDL_RCC_ClocksTypeDef *RCC_Clocks)
{
  DDL_RCC_Unlock();

  /* Get SYSCLK frequency */
  RCC_Clocks->SYSCLK_Frequency = RCC_GetSystemClockFreq();

  /* HCLK clock frequency */
  RCC_Clocks->HCLK_Frequency   = RCC_GetHCLKClockFreq(RCC_Clocks->SYSCLK_Frequency);

  /* PCLK clock frequency */
  RCC_Clocks->PCLK_Frequency  = RCC_GetPCLKClockFreq(RCC_Clocks->HCLK_Frequency);

  DDL_RCC_Lock();
}

/**
  * @}
  */

/** @addtogroup RCC_DDL_Private_Functions RCC Private Functions
  * @{
  */

/**
  * @brief  Return SYSTEM clock frequency
  * @retval SYSTEM clock frequency (in Hz)
  */
static uint32_t RCC_GetSystemClockFreq(void)
{
  uint32_t frequency = 0U;

  /* Get SYSCLK source */
  switch (DDL_RCC_GetSysClkSource())
  {
    case DDL_RCC_SYS_CLKSOURCE_HSI:  /* HSI used as system clock source */
      frequency = __DDL_RCC_CALC_SYSCLK_FREQ(HSI_VALUE, DDL_RCC_GetHSIPrescaler());
      break;

    case DDL_RCC_SYS_CLKSOURCE_LSI:  /* LSI used as system clock source */
      frequency = LSI_VALUE;
      break;

    default:
      frequency = __DDL_RCC_CALC_SYSCLK_FREQ(HSI_VALUE, DDL_RCC_GetHSIPrescaler());
      break;
  }

  return frequency;
}

/**
  * @brief  Return HCLK clock frequency
  * @param  SYSCLK_Frequency SYSCLK clock frequency
  * @retval HCLK clock frequency (in Hz)
  */
static uint32_t RCC_GetHCLKClockFreq(uint32_t SYSCLK_Frequency)
{
  /* HCLK clock frequency */
  return __DDL_RCC_CALC_HCLK_FREQ(SYSCLK_Frequency, DDL_RCC_GetAHBPrescaler());
}

/**
  * @brief  Return PCLK clock frequency
  * @param  HCLK_Frequency HCLK clock frequency
  * @retval PCLK clock frequency (in Hz)
  */
static uint32_t RCC_GetPCLKClockFreq(uint32_t HCLK_Frequency)
{
  /* PCLK clock frequency */
  return __DDL_RCC_CALC_PCLK_FREQ(HCLK_Frequency, DDL_RCC_GetAPBPrescaler());
}

/**
  * @}
  */

/**
  * @}
  */

#endif /* defined(RCC) */

/**
  * @}
  */

#endif /* USE_FULL_DDL_DRIVER */

