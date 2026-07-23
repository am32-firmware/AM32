/**
  *
  * @file    g32f031_ddl_comp0.c
  * @brief   COMP0 DDL module driver.
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
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  *
  */
#if defined(USE_FULL_DDL_DRIVER)

/* Includes ------------------------------------------------------------------*/
#include "g32f031_ddl_comp0.h"
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

#if defined (COMP0)

/** @addtogroup COMP0_DDL COMP0
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/

/** @defgroup COMP0_DDL_Private_Macros COMP0 Private Macros
  * @{
  */

/* Check of parameters for configuration of COMP0 hierarchical scope:         */
/* COMP0 instance.                                                            */

#define IS_COMP0_ALL_SUBMODULE_INSTANCE(__COMP0_SUBMODULE__)      \
    (   ((__COMP0_SUBMODULE__) == COMP0) \
    )


#define IS_DDL_COMP0_INPUT_PLUS(__COMP0__, __INPUT_P__)        \
    (   (((__COMP0__) == COMP0))                      &&      \
        (((__INPUT_P__) == DDL_COMP0_INPUT_PLUS_PA7) ||       \
        ((__INPUT_P__) == DDL_COMP0_INPUT_PLUS_PB9)  ||       \
        ((__INPUT_P__) == DDL_COMP0_INPUT_PLUS_BG_2) ||       \
         ((__INPUT_P__) == DDL_COMP0_INPUT_PLUS_BG))          \
    )

#define IS_DDL_COMP0_INPUT_MINUS(__COMP0__, __INPUT_N__)         \
    (   (((__COMP0__) == COMP0))                        &&      \
        (((__INPUT_N__) == DDL_COMP0_INPUT_MINUS_PA8)   ||      \
        ((__INPUT_N__) == DDL_COMP0_INPUT_MINUS_PA9)    ||      \
        ((__INPUT_N__) == DDL_COMP0_INPUT_MINUS_PGA0)   ||      \
         ((__INPUT_N__) == DDL_COMP0_INPUT_MINUS_PGA1))         \
    )

#define IS_DDL_COMP0_OUTPUT_POLARITY(__POLARITY__)               \
    (   ((__POLARITY__) == DDL_COMP0_OUTPUTPOL_NONINVERTED) ||   \
        ((__POLARITY__) == DDL_COMP0_OUTPUTPOL_INVERTED)         \
    )

#define IS_DDL_COMP0_OUTPUT_HYSN(__HSYN__)           \
    (   ((__HSYN__) == DDL_COMP0_HYSN_DISABLE) ||    \
        ((__HSYN__) == DDL_COMP0_HYSN_20MV) ||       \
        ((__HSYN__) == DDL_COMP0_HYSN_40MV) ||       \
        ((__HSYN__) == DDL_COMP0_HYSN_60MV)          \
    )

#define IS_DDL_COMP0_OUTPUT_HYSP(__HSYP__)           \
    (   ((__HSYP__) == DDL_COMP0_HYSP_DISABLE) ||    \
        ((__HSYP__) == DDL_COMP0_HYSP_20MV) ||       \
        ((__HSYP__) == DDL_COMP0_HYSP_40MV) ||       \
        ((__HSYP__) == DDL_COMP0_HYSP_60MV)          \
    )

#define IS_DDL_COMP0_OUTPUT_FILTERCFG(__FILTER__)           \
    (   ((__FILTER__) == DDL_COMP0_FILTERCFG_1) ||    \
        ((__FILTER__) == DDL_COMP0_FILTERCFG_2) ||          \
        ((__FILTER__) == DDL_COMP0_FILTERCFG_4) ||          \
        ((__FILTER__) == DDL_COMP0_FILTERCFG_8) ||          \
        ((__FILTER__) == DDL_COMP0_FILTERCFG_16) ||         \
        ((__FILTER__) == DDL_COMP0_FILTERCFG_32) ||         \
        ((__FILTER__) == DDL_COMP0_FILTERCFG_64) ||         \
        ((__FILTER__) == DDL_COMP0_FILTERCFG_128) ||        \
        ((__FILTER__) == DDL_COMP0_FILTERCFG_256) ||        \
        ((__FILTER__) == DDL_COMP0_FILTERCFG_512)           \
    )

#define IS_DDL_COMP0_OUTPUT_FILTERPSC(__PSC__)                  \
    (   ((__PSC__) == DDL_COMP0_FILTERPSC_1) ||    \
        ((__PSC__) == DDL_COMP0_FILTERPSC_2) ||          \
        ((__PSC__) == DDL_COMP0_FILTERPSC_4) ||          \
        ((__PSC__) == DDL_COMP0_FILTERPSC_8) ||          \
        ((__PSC__) == DDL_COMP0_FILTERPSC_16) ||         \
        ((__PSC__) == DDL_COMP0_FILTERPSC_32) ||         \
        ((__PSC__) == DDL_COMP0_FILTERPSC_64) ||         \
        ((__PSC__) == DDL_COMP0_FILTERPSC_128)           \
    )

/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/** @addtogroup COMP0_DDL_Exported_Functions COMP0 Exported Functions
  * @{
  */

/** @addtogroup COMP0_DDL_EF_Init
  * @{
  */

/**
 * @brief  Initialize COMP0 function.
 * @param  COMP0x COMP0 instance
 * @param  COMP0_InitStruct Pointer to a @ref DDL_COMP0_InitTypeDef structure
 * @retval An ErrorStatus enumeration value:
 *          - SUCCESS: COMP0 registers are initialized
 *          - ERROR: COMP0 registers are not initialized
 */
ErrorStatus DDL_COMP0_Init(COMP0_TypeDef *COMP0x, DDL_COMP0_InitTypeDef *COMP0_InitStruct)
{
    ErrorStatus status = SUCCESS;

    /* Check parameters */
    ASSERT_PARAM(IS_COMP0_ALL_SUBMODULE_INSTANCE(COMP0x));
    ASSERT_PARAM(IS_DDL_COMP0_INPUT_PLUS(COMP0x, COMP0_InitStruct->InputPlus));
    ASSERT_PARAM(IS_DDL_COMP0_INPUT_MINUS(COMP0x, COMP0_InitStruct->InputMinus));
    ASSERT_PARAM(IS_DDL_COMP0_OUTPUT_POLARITY(COMP0_InitStruct->OutputPol));
    ASSERT_PARAM(IS_DDL_COMP0_OUTPUT_HYSN(COMP0_InitStruct->HsyN));
    ASSERT_PARAM(IS_DDL_COMP0_OUTPUT_HYSP(COMP0_InitStruct->HsyP));
    ASSERT_PARAM(IS_DDL_COMP0_OUTPUT_FILTERCFG(COMP0_InitStruct->FilterCFG));
    ASSERT_PARAM(IS_DDL_COMP0_OUTPUT_FILTERPSC(COMP0_InitStruct->FilterPSC));

    /* COMP0 instance must not be locked */
    if (DDL_COMP0_IsLocked(COMP0x) == 0UL)
    {
        /* Configuration of comparator instance */
        MODIFY_REG(COMP0x->CR,
                    COMP_CR_VPSEL |
                    COMP_CR_VNSEL |
                    COMP_CR_POL |
                    COMP_CR_HYSNEN |
                    COMP_CR_HYSPEN |
                    COMP_CR_CFG |
                    COMP_CR_PSC,
                    COMP0_InitStruct->InputPlus |
                    COMP0_InitStruct->InputMinus |
                    COMP0_InitStruct->OutputPol |
                    COMP0_InitStruct->HsyN |
                    COMP0_InitStruct->HsyP |
                    COMP0_InitStruct->FilterCFG |
                    COMP0_InitStruct->FilterPSC
        );
    }
    else
    {
        status = ERROR;
    }

    return status;
}

/**
 * @brief  De-Initialize COMP0 function.
 * @param  COMP0x COMP0 instance
 * @retval An ErrorStatus enumeration value:
 *         - SUCCESS: COMP0 registers are de-initialized
 *         - ERROR: COMP0 registers are not de-initialized
 * @note   If COMP0 instance is locked, de-initialization can't be performed.
 *         The only way to unlock the COMP0 instance is to perform a system reset.
 */
ErrorStatus DDL_COMP0_DeInit(COMP0_TypeDef *COMP0x)
{
    ErrorStatus status = SUCCESS;

    /* Check parameters */
    ASSERT_PARAM(IS_COMP0_ALL_SUBMODULE_INSTANCE(COMP0x));

    DDL_RCC_Unlock();

    /* COMP0 instance must not be locked */
    if (COMP0x == COMP0)
    {
        /* Force COMP00 reset */
        DDL_APB_GRP1_ForceReset(DDL_APB_GRP1_PERIPH_COMP0);

        /* Release COMP00 reset */
        DDL_APB_GRP1_ReleaseReset(DDL_APB_GRP1_PERIPH_COMP0);
    }
    else
    {
        /* The only way to unlock the COMP0 instance is to perform a system reset */
        status = ERROR;
    }

    DDL_RCC_Lock();

    return status;
}

/**
 * @brief  Set the fields of structure COMP0_InitStruct to default values.
 * @param  COMP0_InitStruct Pointer to a @ref DDL_COMP0_InitTypeDef structure
 *                          whose fields will be set to default values.
 * @retval None
 */
void DDL_COMP0_StructInit(DDL_COMP0_InitTypeDef *COMP0_InitStruct)
{
    /* Set COMP0_InitStruct fields to default values */
    COMP0_InitStruct->InputPlus      = DDL_COMP0_INPUT_PLUS_PA7;
    COMP0_InitStruct->InputMinus     = DDL_COMP0_INPUT_MINUS_PA8;
    COMP0_InitStruct->OutputPol      = DDL_COMP0_OUTPUTPOL_NONINVERTED;
    COMP0_InitStruct->FilterPSC      = DDL_COMP0_FILTERPSC_1;
    COMP0_InitStruct->FilterCFG      = DDL_COMP0_FILTERCFG_1;
    COMP0_InitStruct->HsyP           = DDL_COMP0_HYSP_DISABLE;
    COMP0_InitStruct->HsyN           = DDL_COMP0_HYSN_DISABLE;
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

#endif /* COMP0 */

/**
  * @}
  */

#endif /* USE_FULL_DDL_DRIVER */
