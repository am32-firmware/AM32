/**
  *
  * @file    g32f031_ddl_comp1.c
  * @brief   COMP1 DDL module driver.
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
#include "g32f031_ddl_comp1.h"
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

#if defined (COMP1) || defined (COMP2) || defined (COMP3)

/** @addtogroup COMP1_DDL COMP1
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/

/** @defgroup COMP1_DDL_Private_Macros COMP1 Private Macros
  * @{
  */
/* Check of parameters for configuration of COMP1 hierarchical scope:         */
/* COMP1 instance.                                                            */

#define IS_COMP1_ALL_SUBMODULE_INSTANCE(__COMP1_SUBMODULE__)      \
    (   ((__COMP1_SUBMODULE__) == COMP1) || \
        ((__COMP1_SUBMODULE__) == COMP2) || \
        ((__COMP1_SUBMODULE__) == COMP3)    \
    )

#define IS_DDL_COMP1_INPUT_PLUS(__COMP1__, __INPUT_P__)        \
    (   (((__COMP1__) == COMP1) || \
         ((__COMP1__) == COMP2) || \
         ((__COMP1__) == COMP3))                    &&       \
        (((__INPUT_P__) == DDL_COMP1_INPUT_PLUS_PB6) ||       \
         ((__INPUT_P__) == DDL_COMP1_INPUT_PLUS_VIRTUAL))    \
    )

#define IS_DDL_COMP1_INPUT_MINUS(__COMP1__, __INPUT_N__)         \
    (   (((__COMP1__) == COMP1) || \
         ((__COMP1__) == COMP2) || \
         ((__COMP1__) == COMP3))                    &&       \
        (((__INPUT_N__) == DDL_COMP1_INPUT_MINUS_PA1)   ||      \
         ((__INPUT_N__) == DDL_COMP1_INPUT_MINUS_PB1))  \
    )

#define IS_DDL_COMP1_OUTPUT_POLARITY(__POLARITY__)               \
    (   ((__POLARITY__) == DDL_COMP1_OUTPUTPOL_NONINVERTED) ||   \
        ((__POLARITY__) == DDL_COMP1_OUTPUTPOL_INVERTED)         \
    )

#define IS_DDL_COMP1_OUTPUT_HYSN(__HSYN__)           \
    (   ((__HSYN__) == DDL_COMP1_HYSN_DISABLE) ||    \
        ((__HSYN__) == DDL_COMP1_HYSN_20MV) ||       \
        ((__HSYN__) == DDL_COMP1_HYSN_40MV) ||       \
        ((__HSYN__) == DDL_COMP1_HYSN_60MV)          \
    )

#define IS_DDL_COMP1_OUTPUT_HYSP(__HSYP__)           \
    (   ((__HSYP__) == DDL_COMP1_HYSP_DISABLE) ||    \
        ((__HSYP__) == DDL_COMP1_HYSP_20MV) ||       \
        ((__HSYP__) == DDL_COMP1_HYSP_40MV) ||       \
        ((__HSYP__) == DDL_COMP1_HYSP_60MV)          \
    )

#define IS_DDL_COMP1_OUTPUT_FILTERCFG(__FILTER__)           \
    (   ((__FILTER__) == DDL_COMP1_FILTERCFG_1) ||    \
        ((__FILTER__) == DDL_COMP1_FILTERCFG_2) ||          \
        ((__FILTER__) == DDL_COMP1_FILTERCFG_4) ||          \
        ((__FILTER__) == DDL_COMP1_FILTERCFG_8) ||          \
        ((__FILTER__) == DDL_COMP1_FILTERCFG_16) ||         \
        ((__FILTER__) == DDL_COMP1_FILTERCFG_32) ||         \
        ((__FILTER__) == DDL_COMP1_FILTERCFG_64) ||         \
        ((__FILTER__) == DDL_COMP1_FILTERCFG_128) ||        \
        ((__FILTER__) == DDL_COMP1_FILTERCFG_256) ||        \
        ((__FILTER__) == DDL_COMP1_FILTERCFG_512)           \
    )

#define IS_DDL_COMP1_OUTPUT_FILTERPSC(__PSC__)                  \
    (   ((__PSC__) == DDL_COMP1_FILTERPSC_1) ||    \
        ((__PSC__) == DDL_COMP1_FILTERPSC_2) ||          \
        ((__PSC__) == DDL_COMP1_FILTERPSC_4) ||          \
        ((__PSC__) == DDL_COMP1_FILTERPSC_8) ||          \
        ((__PSC__) == DDL_COMP1_FILTERPSC_16) ||         \
        ((__PSC__) == DDL_COMP1_FILTERPSC_32) ||         \
        ((__PSC__) == DDL_COMP1_FILTERPSC_64) ||         \
        ((__PSC__) == DDL_COMP1_FILTERPSC_128)           \
    )

/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/** @addtogroup COMP1_DDL_Exported_Functions COMP1 Exported Functions
  * @{
  */

/** @addtogroup COMP1_DDL_EF_Init
  * @{
  */

/**
 * @brief  Initialize COMP1 function.
 * @param  COMP1x COMP1 instance
 * @param  COMP1_InitStruct Pointer to a @ref DDL_COMP1_InitTypeDef structure
 * @retval An ErrorStatus enumeration value:
 *          - SUCCESS: COMP1 registers are initialized
 *          - ERROR: COMP1 registers are not initialized
 */
ErrorStatus DDL_COMP1_Init(COMP1_TypeDef *COMP1x, DDL_COMP1_InitTypeDef *COMP1_InitStruct)
{
    ErrorStatus status = SUCCESS;

    /* Check parameters */
    ASSERT_PARAM(IS_COMP1_ALL_SUBMODULE_INSTANCE(COMP1x));
    ASSERT_PARAM(IS_DDL_COMP1_INPUT_PLUS(COMP1x, COMP1_InitStruct->InputPlus));
    ASSERT_PARAM(IS_DDL_COMP1_INPUT_MINUS(COMP1x, COMP1_InitStruct->InputMinus));
    ASSERT_PARAM(IS_DDL_COMP1_OUTPUT_POLARITY(COMP1_InitStruct->OutputPol));
    ASSERT_PARAM(IS_DDL_COMP1_OUTPUT_HYSN(COMP1_InitStruct->HsyN));
    ASSERT_PARAM(IS_DDL_COMP1_OUTPUT_HYSP(COMP1_InitStruct->HsyP));
    ASSERT_PARAM(IS_DDL_COMP1_OUTPUT_FILTERCFG(COMP1_InitStruct->FilterCFG));
    ASSERT_PARAM(IS_DDL_COMP1_OUTPUT_FILTERPSC(COMP1_InitStruct->FilterPSC));

    /* COMP1 instance must not be locked */
    if (DDL_COMP1_IsLocked(COMP1x) == 0UL)
    {
        /* Configuration of comparator instance */
        MODIFY_REG(COMP1x->CR,
                    COMP_CR_VPSEL |
                    COMP_CR_VNSEL |
                    COMP_CR_POL |
                    COMP_CR_HYSNEN |
                    COMP_CR_HYSPEN |
                    COMP_CR_CFG |
                    COMP_CR_PSC,
                    COMP1_InitStruct->InputPlus |
                    COMP1_InitStruct->InputMinus |
                    COMP1_InitStruct->OutputPol |
                    COMP1_InitStruct->HsyN |
                    COMP1_InitStruct->HsyP |
                    COMP1_InitStruct->FilterCFG |
                    COMP1_InitStruct->FilterPSC
        );
    }
    else
    {
        status = ERROR;
    }

    return status;
}

/**
 * @brief  De-Initialize COMP1 function.
 * @param  COMP1x COMP1 instance
 * @retval An ErrorStatus enumeration value:
 *         - SUCCESS: COMP1 registers are de-initialized
 *         - ERROR: COMP1 registers are not de-initialized
 * @note   If COMP1 instance is locked, de-initialization can't be performed.
 *         The only way to unlock the COMP1 instance is to perform a system reset.
 */
ErrorStatus DDL_COMP1_DeInit(COMP1_TypeDef *COMP1x)
{
    ErrorStatus status = SUCCESS;

    /* Check parameters */
    ASSERT_PARAM(IS_COMP1_ALL_SUBMODULE_INSTANCE(COMP1x));

    DDL_RCC_Unlock();

    /* COMP1 instance must not be locked */
    if (COMP1x == COMP1 || COMP1x == COMP2 || COMP1x == COMP3)
    {
        /* Force COMP1 reset */
        DDL_APB_GRP1_ForceReset(DDL_APB_GRP1_PERIPH_COMP1);

        /* Release COMP1 reset */
        DDL_APB_GRP1_ReleaseReset(DDL_APB_GRP1_PERIPH_COMP1);
    }
    else
    {
        /* The only way to unlock the COMP1 instance is to perform a system reset */
        status = ERROR;
    }

    DDL_RCC_Lock();

    return status;
}

/**
 * @brief  Set the fields of structure COMP1_InitStruct to default values.
 * @param  COMP1_InitStruct Pointer to a @ref DDL_COMP1_InitTypeDef structure
 *                          whose fields will be set to default values.
 * @retval None
 */
void DDL_COMP1_StructInit(DDL_COMP1_InitTypeDef *COMP1_InitStruct)
{
    /* Set COMP1_InitStruct fields to default values */
    COMP1_InitStruct->InputPlus      = DDL_COMP1_INPUT_PLUS_PB6;
    COMP1_InitStruct->InputMinus     = DDL_COMP1_INPUT_MINUS_PA1;
    COMP1_InitStruct->OutputPol      = DDL_COMP1_OUTPUTPOL_NONINVERTED;
    COMP1_InitStruct->FilterPSC      = DDL_COMP1_FILTERPSC_1;
    COMP1_InitStruct->FilterCFG      = DDL_COMP1_FILTERCFG_1;
    COMP1_InitStruct->HsyP           = DDL_COMP1_HYSP_DISABLE;
    COMP1_InitStruct->HsyN           = DDL_COMP1_HYSN_DISABLE;
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

#endif /* COMP1 || COMP2 || COMP3 */

/**
  * @}
  */

#endif /* USE_FULL_DDL_DRIVER */
