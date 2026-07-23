/**
  *
  * @file    g32f031_ddl_btmr.c
  * @brief   BTMR DDL module driver.
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
  * Copyright (C) 2026 Geehy Semiconductor.
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
#include "g32f031_ddl_btmr.h"
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

#if defined (BTMR0) || defined (BTMR1)

/** @addtogroup BTMR_DDL BTMR
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/** @addtogroup BTMR_DDL_Private_Macros BTMR Private Macros
  * @{
  */
#define IS_DDL_BTMR_COUNTERMODE(__VALUE__) (((__VALUE__) == DDL_BTMR_COUNTERMODE_UP) \
                                          || ((__VALUE__) == DDL_BTMR_COUNTERMODE_DOWN) \
                                          || ((__VALUE__) == DDL_BTMR_COUNTERMODE_CENTER_UP) \
                                          || ((__VALUE__) == DDL_BTMR_COUNTERMODE_CENTER_DOWN) \
                                          || ((__VALUE__) == DDL_BTMR_COUNTERMODE_CENTER_UP_DOWN))

#define IS_DDL_BTMR_OCMODE(__VALUE__) (((__VALUE__) == DDL_BTMR_OCMODE_FROZEN) \
                                     || ((__VALUE__) == DDL_BTMR_OCMODE_ACTIVE) \
                                     || ((__VALUE__) == DDL_BTMR_OCMODE_INACTIVE) \
                                     || ((__VALUE__) == DDL_BTMR_OCMODE_TOGGLE) \
                                     || ((__VALUE__) == DDL_BTMR_OCMODE_FORCED_INACTIVE) \
                                     || ((__VALUE__) == DDL_BTMR_OCMODE_FORCED_ACTIVE) \
                                     || ((__VALUE__) == DDL_BTMR_OCMODE_PWM1) \
                                     || ((__VALUE__) == DDL_BTMR_OCMODE_PWM2))

#define IS_DDL_BTMR_OCSTATE(__VALUE__) (((__VALUE__) == DDL_BTMR_OCSTATE_DISABLE) \
                                      || ((__VALUE__) == DDL_BTMR_OCSTATE_ENABLE))

#define IS_DDL_BTMR_OCPOLARITY(__VALUE__) (((__VALUE__) == DDL_BTMR_OCPOLARITY_HIGH) \
                                         || ((__VALUE__) == DDL_BTMR_OCPOLARITY_LOW))

#define IS_DDL_BTMR_IC_FILTER(__VALUE__) (((__VALUE__) == DDL_BTMR_IC_FILTER_PCLK_0) \
                                        || ((__VALUE__) == DDL_BTMR_IC_FILTER_PCLK_1) \
                                        || ((__VALUE__) == DDL_BTMR_IC_FILTER_PCLK_2) \
                                        || ((__VALUE__) == DDL_BTMR_IC_FILTER_PCLK_3) \
                                        || ((__VALUE__) == DDL_BTMR_IC_FILTER_PCLK_4) \
                                        || ((__VALUE__) == DDL_BTMR_IC_FILTER_PCLK_5) \
                                        || ((__VALUE__) == DDL_BTMR_IC_FILTER_PCLK_6) \
                                        || ((__VALUE__) == DDL_BTMR_IC_FILTER_PCLK_7))

#define IS_DDL_BTMR_IC_POLARITY(__VALUE__) (((__VALUE__) == DDL_BTMR_IC_POLARITY_RISING) \
                                          || ((__VALUE__) == DDL_BTMR_IC_POLARITY_FALLING) \
                                          || ((__VALUE__) == DDL_BTMR_IC_POLARITY_BOTHEDGE))

/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/
/** @defgroup BTMR_DDL_Private_Functions BTMR Private Functions
  * @{
  */
static ErrorStatus OC0Config(BTMR_TypeDef *BTMRx, DDL_BTMR_OC_InitTypeDef *BTMR_OCInitStruct);
static ErrorStatus IC0Config(BTMR_TypeDef *BTMRx, DDL_BTMR_IC_InitTypeDef *BTMR_ICInitStruct);
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup BTMR_DDL_Exported_Functions BTMR Exported Functions
  * @{
  */

/** @addtogroup BTMR_DDL_EF_Init
  * @{
  */

/**
  * @brief  Set BTMRx registers to their reset values.
  * @param  BTMRx Timer instance
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: BTMRx registers are de-initialized
  *          - ERROR: invalid BTMRx instance
  */
ErrorStatus DDL_BTMR_DeInit(BTMR_TypeDef *BTMRx)
{
  ErrorStatus result = SUCCESS;

  /* Check the parameters */
  ASSERT_PARAM(IS_BTMR_ALL_INSTANCE(BTMRx));

  DDL_RCC_Unlock();
  if (BTMRx == BTMR0)
  {
    DDL_APB_GRP1_ForceReset(DDL_APB_GRP1_PERIPH_BTMR0);
    DDL_APB_GRP1_ReleaseReset(DDL_APB_GRP1_PERIPH_BTMR0);
  }
  else if (BTMRx == BTMR1)
  {
    DDL_APB_GRP1_ForceReset(DDL_APB_GRP1_PERIPH_BTMR1);
    DDL_APB_GRP1_ReleaseReset(DDL_APB_GRP1_PERIPH_BTMR1);
  }
  else
  {
    result = ERROR;
  }

  DDL_RCC_Lock();
  return result;
}

/**
  * @brief  Set the fields of the time base unit configuration data structure
  *         to their default values.
  * @param  BTMR_InitStruct pointer to a @ref DDL_BTMR_InitTypeDef structure (time base unit configuration data structure)
  * @retval None
  */
void DDL_BTMR_StructInit(DDL_BTMR_InitTypeDef *BTMR_InitStruct)
{
  /* Set the default configuration */
  BTMR_InitStruct->Prescaler         = (uint16_t)0x0000;
  BTMR_InitStruct->CounterMode       = DDL_BTMR_COUNTERMODE_UP;
  BTMR_InitStruct->Autoreload        = 0xFFFFFFFFU;
}

/**
  * @brief  Configure the BTMRx time base unit.
  * @param  BTMRx Timer Instance
  * @param  BTMR_InitStruct pointer to a @ref DDL_BTMR_InitTypeDef structure
  *         (BTMRx time base unit configuration data structure)
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: BTMRx registers are de-initialized
  *          - ERROR: not applicable
  */
ErrorStatus DDL_BTMR_Init(BTMR_TypeDef *BTMRx, DDL_BTMR_InitTypeDef *BTMR_InitStruct)
{
  uint32_t tmpcr1;

  /* Check the parameters */
  ASSERT_PARAM(IS_BTMR_ALL_INSTANCE(BTMRx));
  ASSERT_PARAM(IS_DDL_BTMR_COUNTERMODE(BTMR_InitStruct->CounterMode));

  tmpcr1 = DDL_BTMR_ReadReg(BTMRx, CR1);

  if (IS_BTMR_COUNTER_MODE_SELECT_INSTANCE(BTMRx))
  {
    /* Select the Counter Mode */
    MODIFY_REG(tmpcr1, (BTMR_CR1_CNTDIR | BTMR_CR1_CAMSEL), BTMR_InitStruct->CounterMode);
  }

  /* Write to BTMRx CTRL1 */
  DDL_BTMR_WriteReg(BTMRx, CR1, tmpcr1);

  /* Set the Autoreload value */
  DDL_BTMR_SetAutoReload(BTMRx, BTMR_InitStruct->Autoreload);

  /* Set the Prescaler value */
  DDL_BTMR_SetPrescaler(BTMRx, BTMR_InitStruct->Prescaler);

  /* Generate an update event to reload the Prescaler
     and the repetition counter value (if applicable) immediately */
  DDL_BTMR_GenerateEvent_UPDATE(BTMRx);

  return SUCCESS;
}

/**
  * @brief  Set the fields of the BTMRx output channel configuration data
  *         structure to their default values.
  * @param  BTMR_OC_InitStruct pointer to a @ref DDL_BTMR_OC_InitTypeDef structure
  *         (the output channel configuration data structure)
  * @retval None
  */
void DDL_BTMR_OC_StructInit(DDL_BTMR_OC_InitTypeDef *BTMR_OC_InitStruct)
{
  /* Set the default configuration */
  BTMR_OC_InitStruct->OCMode       = DDL_BTMR_OCMODE_FROZEN;
  BTMR_OC_InitStruct->OCState      = DDL_BTMR_OCSTATE_DISABLE;
  BTMR_OC_InitStruct->CompareValue = 0x00000000U;
  BTMR_OC_InitStruct->OCPolarity   = DDL_BTMR_OCPOLARITY_HIGH;
}

/**
  * @brief  Configure the BTMRx output channel.
  * @param  BTMRx Timer Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_BTMR_CHANNEL_CH0
  * @param  BTMR_OC_InitStruct pointer to a @ref DDL_BTMR_OC_InitTypeDef structure (BTMRx output channel configuration
  *         data structure)
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: BTMRx output channel is initialized
  *          - ERROR: BTMRx output channel is not initialized
  */
ErrorStatus DDL_BTMR_OC_Init(BTMR_TypeDef *BTMRx, uint32_t Channel, DDL_BTMR_OC_InitTypeDef *BTMR_OC_InitStruct)
{
  ErrorStatus result = ERROR;

  switch (Channel)
  {
    case DDL_BTMR_CHANNEL_CH0:
      result = OC0Config(BTMRx, BTMR_OC_InitStruct);
      break;
    default:
      break;
  }

  return result;
}

/**
  * @brief  Set the fields of the BTMRx input channel configuration data
  *         structure to their default values.
  * @param  BTMR_ICInitStruct pointer to a @ref DDL_BTMR_IC_InitTypeDef structure (the input channel configuration
  *         data structure)
  * @retval None
  */
void DDL_BTMR_IC_StructInit(DDL_BTMR_IC_InitTypeDef *BTMR_ICInitStruct)
{
  /* Set the default configuration */
  BTMR_ICInitStruct->ICPolarity    = DDL_BTMR_IC_POLARITY_RISING;
  BTMR_ICInitStruct->ICFilter      = DDL_BTMR_IC_FILTER_PCLK_0;
}

/**
  * @brief  Configure the BTMRx input channel.
  * @param  BTMRx Timer Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_BTMR_CHANNEL_CH0
  * @param  BTMR_IC_InitStruct pointer to a @ref DDL_BTMR_IC_InitTypeDef structure (BTMRx input channel configuration data
  *         structure)
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: BTMRx output channel is initialized
  *          - ERROR: BTMRx output channel is not initialized
  */
ErrorStatus DDL_BTMR_IC_Init(BTMR_TypeDef *BTMRx, uint32_t Channel, DDL_BTMR_IC_InitTypeDef *BTMR_IC_InitStruct)
{
  ErrorStatus result = ERROR;

  switch (Channel)
  {
    case DDL_BTMR_CHANNEL_CH0:
      result = IC0Config(BTMRx, BTMR_IC_InitStruct);
      break;
    default:
      break;
  }

  return result;
}

/**
  * @}
  */

/**
  * @}
  */

/** @addtogroup BTMR_DDL_Private_Functions BTMR Private Functions
  *  @brief   Private functions
  * @{
  */
/**
  * @brief  Configure the BTMRx output channel 0.
  * @param  BTMRx Timer Instance
  * @param  BTMR_OCInitStruct pointer to the the BTMRx output channel 1 configuration data structure
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: BTMRx registers are de-initialized
  *          - ERROR: not applicable
  */
static ErrorStatus OC0Config(BTMR_TypeDef *BTMRx, DDL_BTMR_OC_InitTypeDef *BTMR_OCInitStruct)
{
  uint32_t tmpccxcr1;
  uint32_t tmpccxcr2;

  /* Check the parameters */
  ASSERT_PARAM(IS_BTMR_CC0_INSTANCE(BTMRx));
  ASSERT_PARAM(IS_DDL_BTMR_OCMODE(BTMR_OCInitStruct->OCMode));
  ASSERT_PARAM(IS_DDL_BTMR_OCSTATE(BTMR_OCInitStruct->OCState));
  ASSERT_PARAM(IS_DDL_BTMR_OCPOLARITY(BTMR_OCInitStruct->OCPolarity));

  /* Disable the Channel 0: Reset the CC0E Bit */
  CLEAR_BIT(BTMRx->CCXCR1, BTMR_CCXCR1_CC0EN);

  /* Get the BTMRx CCXCR1 register value */
  tmpccxcr1 = DDL_BTMR_ReadReg(BTMRx, CCXCR1);

  /* Get the BTMRx CCXCR2 register value */
  tmpccxcr2 = DDL_BTMR_ReadReg(BTMRx, CCXCR2);

  /* SET Capture/Compare selection Bits */
  SET_BIT(tmpccxcr2, BTMR_CCXCR2_CC0SEL);

  /* Set the Output Compare Mode */
  MODIFY_REG(tmpccxcr1, BTMR_CCXCR1_OC0MOD, BTMR_OCInitStruct->OCMode);

  /* Set the Output Compare Polarity */
  MODIFY_REG(tmpccxcr1, BTMR_CCXCR1_CC0POL, BTMR_OCInitStruct->OCPolarity);

  /* Set the Output State */
  MODIFY_REG(tmpccxcr1, BTMR_CCXCR1_CC0EN, BTMR_OCInitStruct->OCState);

  /* Write to BTMRx CCXCR2 */
  DDL_BTMR_WriteReg(BTMRx, CCXCR2, tmpccxcr2);

  /* Set the Capture Compare Register value */
  DDL_BTMR_OC_SetCompareCH0(BTMRx, BTMR_OCInitStruct->CompareValue);

  /* Write to BTMRx CCXCR1 */
  DDL_BTMR_WriteReg(BTMRx, CCXCR1, tmpccxcr1);

  return SUCCESS;
}

/**
  * @brief  Configure the BTMRx input channel 0.
  * @param  BTMRx Timer Instance
  * @param  BTMR_ICInitStruct pointer to the the BTMRx input channel 1 configuration data structure
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: BTMRx registers are de-initialized
  *          - ERROR: not applicable
  */
static ErrorStatus IC0Config(BTMR_TypeDef *BTMRx, DDL_BTMR_IC_InitTypeDef *BTMR_ICInitStruct)
{
  uint8_t iChannel;
  /* Check the parameters */
  ASSERT_PARAM(IS_BTMR_CC0_INSTANCE(BTMRx));
  ASSERT_PARAM(IS_DDL_BTMR_IC_POLARITY(BTMR_ICInitStruct->ICPolarity));
  ASSERT_PARAM(IS_DDL_BTMR_IC_FILTER(BTMR_ICInitStruct->ICFilter));

  iChannel = BTMR_GET_CHANNEL_INDEX(DDL_BTMR_CHANNEL_CH0);
  /* Disable the Channel 1: Reset the CC1E Bit */
  BTMRx->CCXCR1 &= (uint32_t)~BTMR_CCXCR1_CC0EN;

  /* Reset Capture/Compare selection Bits */
  CLEAR_BIT(BTMRx->CCXCR2, BTMR_CCXCR2_CC0SEL);

  /* Select the Input and set the filter and the prescaler value */
  MODIFY_REG(BTMRx->CCXCR1,
             (BTMR_CCXCR1_IC0F | BTMR_CCXCR1_CC0EDGESEL),
             (BTMR_ICInitStruct->ICFilter | BTMR_ICInitStruct->ICPolarity) << SHIFT_TAB_BTMR_OCxx[iChannel]);

  /* Select the Polarity and set the CC1E Bit */
  MODIFY_REG(BTMRx->CCXCR1,
             (BTMR_CCXCR1_CC0POL),
             (BTMR_ICInitStruct->ICPolarity << SHIFT_TAB_BTMR_OCxx[iChannel]) | BTMR_CCXCR1_CC0EN);

  return SUCCESS;
}

/**
  * @}
  */

/**
  * @}
  */

#endif /* BTMR0 || BTMR1 */

/**
  * @}
  */

#endif /* USE_FULL_DDL_DRIVER */

