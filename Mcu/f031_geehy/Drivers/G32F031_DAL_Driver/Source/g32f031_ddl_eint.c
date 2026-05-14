/**
  *
  * @file    g32f031_ddl_eint.c
  * @author  MCD Application Team
  * @brief   EINT DDL module driver.
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
#include "g32f031_ddl_eint.h"
#include "g32f031_ddl_bus.h"
#include "g32f031_ddl_rcc.h"
#ifdef  USE_FULL_ASSERT
#include "g32_assert.h"
#else
#define ASSERT_PARAM(_PARAM_) ((void)0U)
#endif

/** @addtogroup G32F031_DDL_Driver
  * @{
  */

#if defined (EINT)

/** @defgroup EINT_DDL EINT
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/** @addtogroup EINT_DDL_Private_Macros EINT Private Macros
  * @{
  */

#define IS_DDL_EINT_LINE_0_31(__VALUE__)              (((__VALUE__) & ~DDL_EINT_LINE_ALL_0_31) == 0x00000000U)

#define IS_DDL_EINT_MODE(__VALUE__)                   (((__VALUE__) == DDL_EINT_MODE_IT)            \
                                                   || ((__VALUE__) == DDL_EINT_MODE_EVENT)         \
                                                   || ((__VALUE__) == DDL_EINT_MODE_IT_EVENT))


#define IS_DDL_EINT_TRIGGER(__VALUE__)                (((__VALUE__) == DDL_EINT_TRIGGER_NONE)       \
                                                   || ((__VALUE__) == DDL_EINT_TRIGGER_RISING)     \
                                                   || ((__VALUE__) == DDL_EINT_TRIGGER_FALLING)    \
                                                   || ((__VALUE__) == DDL_EINT_TRIGGER_RISING_FALLING))

/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/** @addtogroup EINT_DDL_Exported_Functions EINT Exported Functions
  * @{
  */

/** @addtogroup EINT_DDL_EF_Init
  * @{
  */

/**
  * @brief  De-initialize the EINT registers to their default reset values.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: EINT registers are de-initialized
  *          - ERROR: not applicable
  */
ErrorStatus DDL_EINT_DeInit(void)
{
  DDL_RCC_Unlock();

  DDL_APB_GRP1_ForceReset(DDL_APB_GRP1_PERIPH_EINT);
  DDL_APB_GRP1_ReleaseReset(DDL_APB_GRP1_PERIPH_EINT);

  DDL_RCC_Lock();
  return SUCCESS;
}

/**
  * @brief  Initialize the EINT registers according to the specified parameters in EINT_InitStruct.
  * @param  EINT_InitStruct pointer to a @ref DDL_EINT_InitTypeDef structure.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: EINT registers are initialized
  *          - ERROR: not applicable
  */
ErrorStatus DDL_EINT_Init(DDL_EINT_InitTypeDef *EINT_InitStruct)
{
  ErrorStatus status = SUCCESS;
  /* Check the parameters */
  ASSERT_PARAM(IS_DDL_EINT_LINE_0_31(EINT_InitStruct->Line_0_31));
  ASSERT_PARAM(IS_FUNCTIONAL_STATE(EINT_InitStruct->LineCommand));
  ASSERT_PARAM(IS_DDL_EINT_MODE(EINT_InitStruct->Mode));

  /* ENABLE LineCommand */
  if (EINT_InitStruct->LineCommand != DISABLE)
  {
    ASSERT_PARAM(IS_DDL_EINT_TRIGGER(EINT_InitStruct->Trigger));

    /* Configure EINT Lines in range from 0 to 17 */
    if (EINT_InitStruct->Line_0_31 != DDL_EINT_LINE_NONE)
    {
      switch (EINT_InitStruct->Mode)
      {
        case DDL_EINT_MODE_IT:
          /* First Disable Event on provided Lines */
          DDL_EINT_DisableEvent_0_31(EINT_InitStruct->Line_0_31);
          /* Then Enable IT on provided Lines */
          DDL_EINT_EnableIT_0_31(EINT_InitStruct->Line_0_31);
          break;
        case DDL_EINT_MODE_EVENT:
          /* First Disable IT on provided Lines */
          DDL_EINT_DisableIT_0_31(EINT_InitStruct->Line_0_31);
          /* Then Enable Event on provided Lines */
          DDL_EINT_EnableEvent_0_31(EINT_InitStruct->Line_0_31);
          break;
        case DDL_EINT_MODE_IT_EVENT:
          /* Directly Enable IT & Event on provided Lines */
          DDL_EINT_EnableIT_0_31(EINT_InitStruct->Line_0_31);
          DDL_EINT_EnableEvent_0_31(EINT_InitStruct->Line_0_31);
          break;
        default:
          status = ERROR;
          break;
      }
      if (EINT_InitStruct->Trigger != DDL_EINT_TRIGGER_NONE)
      {
        switch (EINT_InitStruct->Trigger)
        {
          case DDL_EINT_TRIGGER_RISING:
            /* First Disable Falling Trigger on provided Lines */
            DDL_EINT_DisableFallingTrig_0_31(EINT_InitStruct->Line_0_31);
            /* Then Enable Rising Trigger on provided Lines */
            DDL_EINT_EnableRisingTrig_0_31(EINT_InitStruct->Line_0_31);
            break;
          case DDL_EINT_TRIGGER_FALLING:
            /* First Disable Rising Trigger on provided Lines */
            DDL_EINT_DisableRisingTrig_0_31(EINT_InitStruct->Line_0_31);
            /* Then Enable Falling Trigger on provided Lines */
            DDL_EINT_EnableFallingTrig_0_31(EINT_InitStruct->Line_0_31);
            break;
          case DDL_EINT_TRIGGER_RISING_FALLING:
            DDL_EINT_EnableRisingTrig_0_31(EINT_InitStruct->Line_0_31);
            DDL_EINT_EnableFallingTrig_0_31(EINT_InitStruct->Line_0_31);
            break;
          default:
            status = ERROR;
            break;
        }
      }
    }
  }
  /* DISABLE LineCommand */
  else
  {
    /* De-configure EINT Lines in range from 0 to 17 */
    DDL_EINT_DisableIT_0_31(EINT_InitStruct->Line_0_31);
    DDL_EINT_DisableEvent_0_31(EINT_InitStruct->Line_0_31);
  }
  return status;
}

/**
  * @brief  Set each @ref DDL_EINT_InitTypeDef field to default value.
  * @param  EINT_InitStruct Pointer to a @ref DDL_EINT_InitTypeDef structure.
  * @retval None
  */
void DDL_EINT_StructInit(DDL_EINT_InitTypeDef *EINT_InitStruct)
{
  EINT_InitStruct->Line_0_31      = DDL_EINT_LINE_NONE;
  EINT_InitStruct->LineCommand    = DISABLE;
  EINT_InitStruct->Mode           = DDL_EINT_MODE_IT;
  EINT_InitStruct->Trigger        = DDL_EINT_TRIGGER_FALLING;
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

#endif /* defined (EINT) */

/**
  * @}
  */

#endif /* USE_FULL_DDL_DRIVER */
