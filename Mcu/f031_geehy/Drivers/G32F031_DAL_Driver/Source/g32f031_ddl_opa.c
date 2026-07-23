/**
  *
  * @file    g32f031_ddl_opa.c
  * @brief   OPA DDL module driver.
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
  * Copyright (C) 2026 Geehy Semiconductor
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file in
  * the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  */
#if defined(USE_FULL_DDL_DRIVER)

/* Includes ------------------------------------------------------------------*/
#include "g32f031_ddl_opa.h"
#include "g32f031_ddl_rcc.h"
#include "g32f031_ddl_bus.h"

#ifdef  USE_FULL_ASSERT
#include "g32_assert.h"
#else
#define ASSERT_PARAM(_PARAM_) ((void)0U)
#endif /* USE_FULL_ASSERT */

/** @addtogroup G32F031_DDL_Driver
  * @{
  */

#if defined (OPA)

/** @addtogroup OPA_DDL OPA
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/** @addtogroup OPA_DDL_Private_Macros OPA Private Macros
  * @{
  */

#define IS_DDL_OPA_CHANNEL(__VALUE__)        (((__VALUE__) == DDL_OPA_CHANNEL_0)   ||\
                                             ((__VALUE__) == DDL_OPA_CHANNEL_1))

#define IS_DDL_OPA_GAINSELECT(__VALUE__)         (((__VALUE__) == DDL_OPA_INTERNALGAIN_DISABLE)   ||\
                                                  ((__VALUE__) == DDL_OPA_INTERNALGAIN_1)         ||\
                                                  ((__VALUE__) == DDL_OPA_INTERNALGAIN_4)         ||\
                                                  ((__VALUE__) == DDL_OPA_INTERNALGAIN_6)         ||\
                                                  ((__VALUE__) == DDL_OPA_INTERNALGAIN_8)         ||\
                                                  ((__VALUE__) == DDL_OPA_INTERNALGAIN_10)        ||\
                                                  ((__VALUE__) == DDL_OPA_INTERNALGAIN_12)        ||\
                                                  ((__VALUE__) == DDL_OPA_INTERNALGAIN_16))

#define IS_DDL_OPA_INPUTCONTROL(__VALUE__)  (((__VALUE__) == DDL_OPA_INCTRL_DISABLE)  ||\
                                             ((__VALUE__) == DDL_OPA_INCTRL_ENABLE))

#define IS_DDL_OPA_OUTPUTCONTROL(__VALUE__)        (((__VALUE__) == DDL_OPA_OUTCTRL_DISABLE)   ||\
                                                    ((__VALUE__) == DDL_OPA_OUTCTRL_ENABLE))

#define IS_DDL_OPA_VCMSELECT(__VALUE__)         (((__VALUE__) == DDL_OPA_VCMSEL_DISABLE)   ||\
                                                 ((__VALUE__) == DDL_OPA_VCMSEL_AVDD_0_5)  ||\
                                                 ((__VALUE__) == DDL_OPA_VCMSEL_AVDD_0_25) ||\
                                                 ((__VALUE__) == DDL_OPA_VCMSEL_VBG)       ||\
                                                 ((__VALUE__) == DDL_OPA_VCMSEL_VBG_0_25))

 /**
  * @}
  */
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/** @addtogroup OPA_DDL_Exported_Functions OPA Exported Functions
  * @{
  */

/** @addtogroup OPA_DDL_EF_Init
  * @{
  */

/**
  * @brief  De-initialize OPA registers (Registers restored to their default values).
  * @param  OPAx OPA Instance
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: OPA registers are de-initialized
  *          - ERROR: OPA registers are not de-initialized
  */
ErrorStatus DDL_OPA_DeInit(OPA_TypeDef *OPAx)
{
  ErrorStatus status = SUCCESS;

  /* Check the parameters */
  ASSERT_PARAM(IS_OPA_ALL_INSTANCE(OPAx));

  DDL_RCC_Unlock();

  if (OPAx == OPA)
  {
    /* Force OPA reset */
    DDL_APB_GRP1_ForceReset(DDL_APB_GRP1_PERIPH_OPA);
    /* Release OPA reset */
    DDL_APB_GRP1_ReleaseReset(DDL_APB_GRP1_PERIPH_OPA);
  }
  else
  {
    status = ERROR;
  }

  DDL_RCC_Lock();

  return status;
}

/**
  * @brief  Initialize OPA registers according to the specified parameters in OPA_InitStruct.
  * @param  OPAx OPA Port
  * @param  OPA_InitStruct pointer to a @ref DDL_OPA_InitTypeDef structure
  *         that contains the configuration information for the specified OPA peripheral.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: OPA registers are initialized according to OPA_InitStruct content
  *          - ERROR:   Not applicable
  */
ErrorStatus DDL_OPA_Init(OPA_TypeDef *OPAx, DDL_OPA_InitTypeDef *OPA_InitStruct)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_OPA_ALL_INSTANCE(OPAx));
  ASSERT_PARAM(IS_DDL_OPA_CHANNEL(OPA_InitStruct->Channel));
  ASSERT_PARAM(IS_DDL_OPA_GAINSELECT(OPA_InitStruct->GainSelect));
  ASSERT_PARAM(IS_DDL_OPA_INPUTCONTROL(OPA_InitStruct->InputControl));
  ASSERT_PARAM(IS_DDL_OPA_OUTPUTCONTROL(OPA_InitStruct->OutputControl));
  ASSERT_PARAM(IS_DDL_OPA_VCMSELECT(OPA_InitStruct->VCMSelect));

  MODIFY_REG(OPAx->CR,
             ((OPA_CR_OPA0SELGAIN | OPA_CR_OPA0INSEL | OPA_CR_OPA0OUTSEL) << OPA_InitStruct->Channel) | OPA_CR_OPASELVCM,
             (OPA_InitStruct->GainSelect | OPA_InitStruct->InputControl | OPA_InitStruct->OutputControl)<< OPA_InitStruct->Channel |
             OPA_InitStruct->VCMSelect);

  return (SUCCESS);
}

/**
  * @brief  Set each @ref DDL_OPA_InitTypeDef field to default value.
  * @param  OPA_InitStruct pointer to a @ref DDL_OPA_InitTypeDef structure
  * whose fields will be set to default values.
  * @retval None
  */
void DDL_OPA_StructInit(DDL_OPA_InitTypeDef *OPA_InitStruct)
{
  /* Set OPA_InitStruct fields to default values */
  OPA_InitStruct->Channel        = DDL_OPA_CHANNEL_0;
  OPA_InitStruct->GainSelect     = DDL_OPA_INTERNALGAIN_DISABLE;
  OPA_InitStruct->InputControl   = DDL_OPA_INCTRL_DISABLE;
  OPA_InitStruct->OutputControl  = DDL_OPA_OUTCTRL_DISABLE;
  OPA_InitStruct->VCMSelect      = DDL_OPA_VCMSEL_DISABLE;
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

#endif /* defined (OPA) */

/**
  * @}
  */

#endif /* USE_FULL_DDL_DRIVER */
