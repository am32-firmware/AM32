/**
  *
  * @file    g32f031_ddl_lptmr.c
  * @brief   LPTMR DDL module driver.
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
  * Copyright (c) 2016 STMicroelectronics.
 *  Copyright (C) 2026 Geehy Semiconductor
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  *
  */
#if defined(USE_FULL_DDL_DRIVER)

/* Includes ------------------------------------------------------------------*/
#include "g32f031_ddl_lptmr.h"
#include "g32f031_ddl_rcc.h"

#ifdef  USE_FULL_ASSERT
#include "g32_assert.h"
#else
#define ASSERT_PARAM(_PARAM_) ((void)0U)
#endif /* USE_FULL_ASSERT */

/** @addtogroup G32F031_DDL_Driver
  * @{
  */

#if defined (LPTMR)

/** @addtogroup LPTMR_DDL LPTMR
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/** @addtogroup LPTMR_DDL_Private_Macros LPTMR Private Macros
  * @{
  */

 /* Check of parameters for configuration of LPTMR hierarchical scope:          */
    /* LPTMR instance.                                                             */
#define IS_DDL_LPTMR_PRESCALER(__LPTMR_INSTANCE__, __PRESCALER__)     \
    (   ((__PRESCALER__) <= 0xFUL) \
    )

/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/** @addtogroup LPTMR_DDL_Exported_Functions LPTMR Exported Functions
  * @{
  */

/** @addtogroup LPTMR_DDL_EF_Init
  * @{
  */

/**
  * @brief  Set LPTMRx registers to their reset values.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: LPTMRx registers are de-initialized
  *          - ERROR: invalid LPTMRx instance
  */
ErrorStatus DDL_LPTMR_DeInit(LPTMR_TypeDef *LPTMRx)
{
  ErrorStatus status = SUCCESS;

  /* Check the parameters */
  ASSERT_PARAM(IS_LPTMR_ALL_INSTANCE(LPTMRx));

  if (LPTMRx == LPTMR)
  {
    DDL_RCC_Unlock();

    DDL_RCC_ForceResetAONPeripheral(DDL_RCC_AON_RESET_LPTMR);
    DDL_RCC_ReleaseResetAONPeripheral(DDL_RCC_AON_RESET_LPTMR);

    DDL_RCC_Lock();
  }
  else
  {
    status = ERROR;
  }

  return (status);
}

/**
  * @brief  Set the fields of the time base unit configuration data structure
  *         to their default values.
  * @param  LPTMR_InitStruct pointer to a @ref DDL_LPTMR_InitTypeDef structure (time base unit configuration data structure)
  * @retval None
  */
void DDL_LPTMR_StructInit(DDL_LPTMR_InitTypeDef *LPTMR_InitStruct)
{
  /* Set the default configuration */
  LPTMR_InitStruct->Prescaler   = (uint8_t)0x00UL;
  LPTMR_InitStruct->WakeUpValue = (uint16_t)0x0000UL;
}

/**
  * @brief  Configure the LPTMRx time base unit.
  * @param  LPTMRx Timer Instance
  * @param  LPTMR_InitStruct pointer to a @ref DDL_LPTMR_InitTypeDef structure
  *         (LPTMRx time base unit configuration data structure)
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: LPTMRx registers are de-initialized
  *          - ERROR: not applicable
  */
ErrorStatus DDL_LPTMR_Init(LPTMR_TypeDef *LPTMRx, DDL_LPTMR_InitTypeDef *LPTMR_InitStruct)
{
  ASSERT_PARAM(IS_DDL_LPTMR_PRESCALER(LPTMRx,LPTMR_InitStruct->Prescaler));

  /* Set the Autoreload value */
  DDL_LPTMR_SetWakeUpValue(LPTMRx, LPTMR_InitStruct->WakeUpValue);

  /* Set the Prescaler value */
  DDL_LPTMR_SetPrescaler(LPTMRx, LPTMR_InitStruct->Prescaler);

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

#endif /* LPTMR */

/**
  * @}
  */

#endif /* USE_FULL_DDL_DRIVER */

