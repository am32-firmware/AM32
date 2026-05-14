/**
  *
  * @file    g32f031_ddl_utils.c
  * @brief   UTILS DDL module driver.
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
  * Copyright (C) 2026 Geehy Semiconductor.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  *
  */
/* Includes ------------------------------------------------------------------*/
#include "g32f031_ddl_utils.h"
#include "g32f031_ddl_rcc.h"
#include "g32f031_ddl_flash.h"

#ifdef  USE_FULL_ASSERT
#include "g32_assert.h"
#else
#define ASSERT_PARAM(_PARAM_) ((void)0U)
#endif /* USE_FULL_ASSERT */

/** @addtogroup G32F031_DDL_Driver
  * @{
  */

/** @addtogroup UTILS_DDL UTILS
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/** @addtogroup UTILS_DDL_Private_Constants UTILS Private Constants
  * @{
  */
#if defined(RCC_MAX_FREQUENCY_SCALE1)
#define UTILS_MAX_FREQUENCY_SCALE1  RCC_MAX_FREQUENCY_SCALE1    /*!< Maximum frequency for system clock at power scale1, in Hz */
#endif /*RCC_MAX_FREQUENCY_SCALE1 */

/* Defines used for FLASH latency according to HCLK Frequency */
#if defined(FLASH_SCALE1_LATENCY1_FREQ)
#define UTILS_SCALE1_LATENCY1_FREQ  FLASH_SCALE1_LATENCY1_FREQ /*!< HCLK frequency to set FLASH latency 1 in power scale 1 */
#endif

/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/** @addtogroup UTILS_DDL_Exported_Functions UTILS Exported Functions
  * @{
  */

/** @addtogroup UTILS_DDL_EF_DELAY
  * @{
  */

/**
  * @brief  This function configures the Cortex-M SysTick source to have 1ms time base.
  * @note   When a RTOS is used, it is recommended to avoid changing the Systick
  *         configuration by calling this function, for a delay use rather osDelay RTOS service.
  * @param  HCLKFrequency HCLK frequency in Hz
  * @note   HCLK frequency can be calculated thanks to function @ref DDL_SYSCTRL_GetSysctrlClocksFreq
  * @retval None
  */
void DDL_Init1msTick(uint32_t HCLKFrequency)
{
  /* Use frequency provided in argument */
  DDL_InitTick(HCLKFrequency, 1000U);
}

/**
  * @brief  This function configures the Cortex-M SysTick source to have 1us time base.
  * @note   When a RTOS is used, it is recommended to avoid changing the Systick
  *         configuration by calling this function, for a delay use rather osDelay RTOS service.
  * @param  HCLKFrequency HCLK frequency in Hz
  * @note   HCLK frequency can be calculated thanks to function @ref DDL_SYSCTRL_GetSysctrlClocksFreq
  * @retval None
  */
void DDL_Init1usTick(uint32_t HCLKFrequency)
{
  /* Use frequency provided in argument */
  DDL_InitTick(HCLKFrequency, 1000000U);
}

/**
  * @brief  This function provides accurate delay (in milliseconds) based
  *         on SysTick counter flag
  * @note   When a RTOS is used, it is recommended to avoid using blocking delay
  *         and use rather osDelay service.
  * @note   To respect 1ms timebase, user should call @ref DDL_Init1msTick function which
  *         will configure Systick to 1ms
  * @param  Delay specifies the delay time length, in milliseconds.
  * @retval None
  */
void DDL_mDelay(uint32_t Delay)
{
  __IO uint32_t  tmp = SysTick->CTRL;  /* Clear the COUNTFLAG first */
  /* Add this code to indicate that local variable is not used */
  ((void)tmp);

  /* Add a period to guaranty minimum wait */
  if(Delay < DDL_MAX_DELAY)
  {
    Delay++;
  }

  while (Delay)
  {
    if((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) != 0U)
    {
      Delay--;
    }
  }
}

/**
  * @brief  This function provides accurate delay (in microseconds) based
  *         on SysTick counter flag
  * @note   When a RTOS is used, it is recommended to avoid using blocking delay
  *         and use rather osDelay service.
  * @note   To respect 1ms timebase, user should call @ref DDL_Init1msTick function which
  *         will configure Systick to 1ms
  * @param  Delay specifies the delay time length, in microseconds.
  * @retval None
  */
void DDL_uDelay(uint32_t Delay)
{
  __IO uint32_t  tmp = SysTick->CTRL;  /* Clear the COUNTFLAG first */
  /* Add this code to indicate that local variable is not used */
  ((void)tmp);

  /* Add a period to guaranty minimum wait */
  if(Delay < DDL_MAX_DELAY)
  {
    Delay++;
  }

  while (Delay)
  {
    if((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) != 0U)
    {
      Delay--;
    }
  }
}

/**
  * @}
  */

/** @addtogroup UTILS_EF_SYSTEM
  *  @brief    System Configuration functions
  *
  @verbatim
 ===============================================================================
           ##### System Configuration functions #####
 ===============================================================================
    [..]
         System, AHB and APB buses clocks configuration

         (+) The maximum frequency of the SYSCLK, HCLK, PCLK1 and PCLK2 is 180000000 Hz.
  @endverbatim
  @internal
             Depending on the device voltage range, the maximum frequency should be
             adapted accordingly to the Refenece manual.
  @endinternal
  * @{
  */

/**
  * @brief  This function sets directly SystemCoreClock CMSIS variable.
  * @note   Variable can be calculated also through SystemCoreClockUpdate function.
  * @param  SYSCLKFrequency SYSCLK frequency in Hz (can be calculated thanks to function @ref DDL_SYSCTRL_GetSysctrlClocksFreq)
  * @retval None
  */
void DDL_SetSystemCoreClock(uint32_t SYSCLKFrequency)
{
  /* SYSCLK frequency */
  SystemCoreClock = SYSCLKFrequency;
}

/**
  * @brief  Update number of Flash wait states in line with new frequency and current
            voltage range.
  * @param  HCLK_Frequency  HCLK frequency
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: Latency has been modified
  *          - ERROR: Latency cannot be modified
  */
ErrorStatus DDL_SetFlashLatency(uint32_t HCLK_Frequency)
{
  uint32_t timeout;
  uint32_t getlatency;
  uint32_t latency = DDL_FLASH_LATENCY0;  /* default value 0WS */
  ErrorStatus status = SUCCESS;


  /* Frequency cannot be equal to 0 */
  if(HCLK_Frequency == 0U)
  {
    status = ERROR;
  }
  else
  {
      if((HCLK_Frequency > UTILS_SCALE1_LATENCY1_FREQ)&&(latency == DDL_FLASH_LATENCY0))
      {
        latency = DDL_FLASH_LATENCY3;
      }
      else if((HCLK_Frequency == UTILS_SCALE1_LATENCY1_FREQ)&&(latency == DDL_FLASH_LATENCY0))
      {
        latency = DDL_FLASH_LATENCY1;
      }

    DDL_FLASH_SetLatency(latency);
    /* Check that the new number of wait states is taken into account to access the Flash
       memory by reading the FLASH_CR register */
    timeout = 2;
    do
    {
        /* Wait for Flash latency to be updated */
        getlatency = DDL_FLASH_GetLatency();
        timeout--;
    } while ((getlatency != latency) && (timeout > 0));

    if(getlatency != latency)
    {
      status = ERROR;
    }
    else
    {
      status = SUCCESS;
    }
  }
  return status;
}

/**
  * @brief  This function configures system clock with HSI
  * @param  UTILS_ClkInitStruct pointer to a @ref DDL_UTILS_ClkInitTypeDef structure that contains
  *                             the configuration information for the BUS prescalers.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: Max frequency configuration done
  *          - ERROR: Max frequency configuration not done
  */
ErrorStatus DDL_ConfigSystemClock_HSI(DDL_UTILS_ClkInitTypeDef *UTILS_ClkInitStruct)
{
    if(DDL_RCC_GetSysClkSource() != DDL_RCC_SYS_CLKSOURCE_HSI)
    {
        DDL_RCC_SetSysClkSource(DDL_RCC_SYS_CLKSOURCE_HSI);
        DDL_RCC_HSI_Enable();
        while (DDL_RCC_HSI_IsReady() != 1U)
        {
          /* Wait for HSI ready */
        }
    }
    else
    {
        if(DDL_RCC_HSI_IsReady() != 1U)
        {
            DDL_RCC_HSI_Enable();
            while (DDL_RCC_HSI_IsReady() != 1U)
            {
              /* Wait for HSI ready */
            }
        }
    }
    DDL_RCC_SetHSIPrescaler(UTILS_ClkInitStruct->SYSCLKDivider);
    DDL_RCC_SetAHBPrescaler(UTILS_ClkInitStruct->AHBCLKDivider);
    DDL_RCC_SetAPBPrescaler(UTILS_ClkInitStruct->APBCLKDivider);

    DDL_SetSystemCoreClock(HSI_VALUE / (DDL_RCC_GetHSIPrescaler() >> RCC_CFG_HSIDIV_Pos));

    return SUCCESS;
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
