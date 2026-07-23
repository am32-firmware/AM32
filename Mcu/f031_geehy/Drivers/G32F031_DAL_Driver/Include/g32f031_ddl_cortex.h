/**
  *
  * @file    g32f031_ddl_cortex.h
  * @brief   Header file of CORTEX DDL module.
  @verbatim
  ==============================================================================
                     ##### How to use this driver #####
  ==============================================================================
    [..]
    The LL CORTEX driver contains a set of generic APIs that can be
    used by user:
      (+) SYSTICK configuration used by DDL_mDelay and DDL_Init1msTick
          functions
      (+) Low power mode configuration (SCB register of Cortex-MCU)
      (+) MPU API to configure and enable regions
          (MPU services provided only on some devices)
      (+) API to access to MCU info (CPUID register)
      (+) API to enable fault handler (SHCSR accesses)

  @endverbatim
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef G32F031_DDL_CORTEX_H
#define G32F031_DDL_CORTEX_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "g32f0xx.h"

/** @addtogroup G32F031_DDL_Driver
  * @{
  */

/** @defgroup CORTEX_DDL CORTEX
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/** @defgroup CORTEX_DDL_Exported_Constants CORTEX Exported Constants
  * @{
  */

/** @defgroup CORTEX_DDL_EC_CLKSOURCE_HCLK SYSTICK Clock Source
  * @{
  */
#define DDL_SYSTICK_CLKSOURCE_HCLK_DIV8     0x00000000U                 /*!< AHB clock divided by 8 selected as SysTick clock source.*/
#define DDL_SYSTICK_CLKSOURCE_HCLK          SysTick_CTRL_CLKSOURCE_Msk  /*!< AHB clock selected as SysTick clock source. */
/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/** @defgroup CORTEX_DDL_Exported_Functions CORTEX Exported Functions
  * @{
  */

/** @defgroup CORTEX_DDL_EF_SYSTICK SYSTICK
  * @{
  */

/**
  * @brief  This function checks if the Systick counter flag is active or not.
  * @note   It can be used in timeout function on application side.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_SYSTICK_IsActiveCounterFlag(void)
{
  return ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == (SysTick_CTRL_COUNTFLAG_Msk));
}

/**
  * @brief  Configures the SysTick clock source
  * @param  Source This parameter can be one of the following values:
  *         @arg @ref DDL_SYSTICK_CLKSOURCE_HCLK_DIV8
  *         @arg @ref DDL_SYSTICK_CLKSOURCE_HCLK
  * @retval None
  */
__STATIC_INLINE void DDL_SYSTICK_SetClkSource(uint32_t Source)
{
  if (Source == DDL_SYSTICK_CLKSOURCE_HCLK)
  {
    SET_BIT(SysTick->CTRL, DDL_SYSTICK_CLKSOURCE_HCLK);
  }
  else
  {
    CLEAR_BIT(SysTick->CTRL, DDL_SYSTICK_CLKSOURCE_HCLK);
  }
}

/**
  * @brief  Get the SysTick clock source
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_SYSTICK_CLKSOURCE_HCLK_DIV8
  *         @arg @ref DDL_SYSTICK_CLKSOURCE_HCLK
  */
__STATIC_INLINE uint32_t DDL_SYSTICK_GetClkSource(void)
{
  return READ_BIT(SysTick->CTRL, DDL_SYSTICK_CLKSOURCE_HCLK);
}

/**
  * @brief  Enable SysTick exception request
  * @retval None
  */
__STATIC_INLINE void DDL_SYSTICK_EnableIT(void)
{
  SET_BIT(SysTick->CTRL, SysTick_CTRL_TICKINT_Msk);
}

/**
  * @brief  Disable SysTick exception request
  * @retval None
  */
__STATIC_INLINE void DDL_SYSTICK_DisableIT(void)
{
  CLEAR_BIT(SysTick->CTRL, SysTick_CTRL_TICKINT_Msk);
}

/**
  * @brief  Checks if the SYSTICK interrupt is enabled or disabled.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_SYSTICK_IsEnabledIT(void)
{
  return (READ_BIT(SysTick->CTRL, SysTick_CTRL_TICKINT_Msk) == (SysTick_CTRL_TICKINT_Msk));
}

/**
  * @}
  */

/** @defgroup CORTEX_DDL_EF_LOW_POWER_MODE LOW POWER MODE
  * @{
  */

/**
  * @brief  Processor uses sleep as its low power mode
  * @retval None
  */
__STATIC_INLINE void DDL_LPM_EnableSleep(void)
{
  /* Clear SLEEPDEEP bit of Cortex System Control Register */
  CLEAR_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPDEEP_Msk));
}

/**
  * @brief  Processor uses deep sleep as its low power mode
  * @retval None
  */
__STATIC_INLINE void DDL_LPM_EnableDeepSleep(void)
{
  /* Set SLEEPDEEP bit of Cortex System Control Register */
  SET_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPDEEP_Msk));
}

/**
  * @brief  Configures sleep-on-exit when returning from Handler mode to Thread mode.
  * @note   Setting this bit to 1 enables an interrupt-driven application to avoid returning to an
  *         empty main application.
  * @retval None
  */
__STATIC_INLINE void DDL_LPM_EnableSleepOnExit(void)
{
  /* Set SLEEPONEXIT bit of Cortex System Control Register */
  SET_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPONEXIT_Msk));
}

/**
  * @brief  Do not sleep when returning to Thread mode.
  * @retval None
  */
__STATIC_INLINE void DDL_LPM_DisableSleepOnExit(void)
{
  /* Clear SLEEPONEXIT bit of Cortex System Control Register */
  CLEAR_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPONEXIT_Msk));
}

/**
  * @brief  Enabled events and all interrupts, including disabled interrupts, can wakeup the
  *         processor.
  * @retval None
  */
__STATIC_INLINE void DDL_LPM_EnableEventOnPend(void)
{
  /* Set SEVEONPEND bit of Cortex System Control Register */
  SET_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SEVONPEND_Msk));
}

/**
  * @brief  Only enabled interrupts or events can wakeup the processor, disabled interrupts are
  *         excluded
  * @retval None
  */
__STATIC_INLINE void DDL_LPM_DisableEventOnPend(void)
{
  /* Clear SEVEONPEND bit of Cortex System Control Register */
  CLEAR_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SEVONPEND_Msk));
}

/**
  * @}
  */

/** @defgroup CORTEX_DDL_EF_MCU_INFO MCU INFO
  * @{
  */

/**
  * @brief  Get Implementer code
  * @retval Value should be equal to 0x41 for ARM
  */
__STATIC_INLINE uint32_t DDL_CPUID_GetImplementer(void)
{
  return (uint32_t)(READ_BIT(SCB->CPUID, SCB_CPUID_IMPLEMENTER_Msk) >> SCB_CPUID_IMPLEMENTER_Pos);
}

/**
  * @brief  Get Variant number (The r value in the rnpn product revision identifier)
  * @retval Value between 0 and 255 (0x0: revision 0)
  */
__STATIC_INLINE uint32_t DDL_CPUID_GetVariant(void)
{
  return (uint32_t)(READ_BIT(SCB->CPUID, SCB_CPUID_VARIANT_Msk) >> SCB_CPUID_VARIANT_Pos);
}

/**
  * @brief  Get Constant number
  * @retval Value should be equal to 0xC for Cortex-M0 devices
  */
__STATIC_INLINE uint32_t DDL_CPUID_GetConstant(void)
{
  return (uint32_t)(READ_BIT(SCB->CPUID, SCB_CPUID_ARCHITECTURE_Msk) >> SCB_CPUID_ARCHITECTURE_Pos);
}

/**
  * @brief  Get Part number
  * @retval Value should be equal to 0xC20 for Cortex-M0
  */
__STATIC_INLINE uint32_t DDL_CPUID_GetParNo(void)
{
  return (uint32_t)(READ_BIT(SCB->CPUID, SCB_CPUID_PARTNO_Msk) >> SCB_CPUID_PARTNO_Pos);
}

/**
  * @brief  Get Revision number (The p value in the rnpn product revision identifier, indicates patch release)
  * @retval Value between 0 and 255 (0x1: patch 1)
  */
__STATIC_INLINE uint32_t DDL_CPUID_GetRevision(void)
{
  return (uint32_t)(READ_BIT(SCB->CPUID, SCB_CPUID_REVISION_Msk) >> SCB_CPUID_REVISION_Pos);
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

#ifdef __cplusplus
}
#endif

#endif /* G32F031_DDL_CORTEX_H */

