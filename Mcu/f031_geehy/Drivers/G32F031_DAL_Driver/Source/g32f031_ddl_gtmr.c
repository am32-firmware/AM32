/**
  *
  * @file    g32f031_ddl_gtmr.c
  * @brief   GTMR DDL module driver.
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
#include "g32f031_ddl_gtmr.h"
#include "g32f031_ddl_bus.h"
#include "g32f031_ddl_rcc.h"

#ifdef  USE_FULL_ASSERT
#include "g32_assert.h"
#else
#define ASSERT_PARAM(_PARAM_) ((void)0U)
#endif /* USE_FULL_ASSERT */

/** @addtogroup G32F031_DDL_Driver
  * @{
  */

#if defined (GTMR)

/** @addtogroup GTMR_DDL GTMR
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/** @addtogroup GTMR_DDL_Private_Macros GTMR Private Macros
  * @{
  */
#define IS_GTMR_COUNTER_MODE_SELECT_INSTANCE(__VALUE__) ((__VALUE__) == GTMR)
#define IS_GTMR_CLOCK_DIVISION_INSTANCE(__VALUE__) ((__VALUE__) == GTMR)
#define IS_DDL_GTMR_COUNTERMODE(__VALUE__) (((__VALUE__) == DDL_GTMR_COUNTERMODE_UP) \
                                          || ((__VALUE__) == DDL_GTMR_COUNTERMODE_DOWN) \
                                          || ((__VALUE__) == DDL_GTMR_COUNTERMODE_CENTER_UP) \
                                          || ((__VALUE__) == DDL_GTMR_COUNTERMODE_CENTER_DOWN) \
                                          || ((__VALUE__) == DDL_GTMR_COUNTERMODE_CENTER_UP_DOWN))

#define IS_DDL_GTMR_CLOCKDIVISION(__VALUE__) (((__VALUE__) == DDL_GTMR_CLOCKDIVISION_DIV1) \
                                            || ((__VALUE__) == DDL_GTMR_CLOCKDIVISION_DIV2) \
                                            || ((__VALUE__) == DDL_GTMR_CLOCKDIVISION_DIV4))

#define IS_DDL_GTMR_OCMODE(__VALUE__) (((__VALUE__) == DDL_GTMR_OCMODE_FROZEN) \
                                     || ((__VALUE__) == DDL_GTMR_OCMODE_ACTIVE) \
                                     || ((__VALUE__) == DDL_GTMR_OCMODE_INACTIVE) \
                                     || ((__VALUE__) == DDL_GTMR_OCMODE_TOGGLE) \
                                     || ((__VALUE__) == DDL_GTMR_OCMODE_FORCED_INACTIVE) \
                                     || ((__VALUE__) == DDL_GTMR_OCMODE_FORCED_ACTIVE) \
                                     || ((__VALUE__) == DDL_GTMR_OCMODE_PWM1) \
                                     || ((__VALUE__) == DDL_GTMR_OCMODE_PWM2))

#define IS_DDL_GTMR_OCSTATE(__VALUE__) (((__VALUE__) == DDL_GTMR_OCSTATE_DISABLE) \
                                      || ((__VALUE__) == DDL_GTMR_OCSTATE_ENABLE))

#define IS_DDL_GTMR_OCPOLARITY(__VALUE__) (((__VALUE__) == DDL_GTMR_OCPOLARITY_HIGH) \
                                         || ((__VALUE__) == DDL_GTMR_OCPOLARITY_LOW))

#define IS_DDL_GTMR_OCIDLESTATE(__VALUE__) (((__VALUE__) == DDL_GTMR_OCIDLESTATE_LOW) \
                                          || ((__VALUE__) == DDL_GTMR_OCIDLESTATE_HIGH))

#define IS_DDL_GTMR_ACTIVEINPUT(__VALUE__) (((__VALUE__) == DDL_GTMR_ACTIVEINPUT_DIRECTTI) \
                                          || ((__VALUE__) == DDL_GTMR_ACTIVEINPUT_INDIRECTTI) \
                                          || ((__VALUE__) == DDL_GTMR_ACTIVEINPUT_TRC))

#define IS_DDL_GTMR_ICPSC(__VALUE__) (((__VALUE__) == DDL_GTMR_ICPSC_DIV1) \
                                    || ((__VALUE__) == DDL_GTMR_ICPSC_DIV2) \
                                    || ((__VALUE__) == DDL_GTMR_ICPSC_DIV4) \
                                    || ((__VALUE__) == DDL_GTMR_ICPSC_DIV8))

#define IS_DDL_GTMR_IC_FILTER(__VALUE__) (((__VALUE__) == DDL_GTMR_IC_FILTER_FDIV1) \
                                        || ((__VALUE__) == DDL_GTMR_IC_FILTER_FDIV1_N2) \
                                        || ((__VALUE__) == DDL_GTMR_IC_FILTER_FDIV1_N4) \
                                        || ((__VALUE__) == DDL_GTMR_IC_FILTER_FDIV1_N8) \
                                        || ((__VALUE__) == DDL_GTMR_IC_FILTER_FDIV2_N6) \
                                        || ((__VALUE__) == DDL_GTMR_IC_FILTER_FDIV2_N8) \
                                        || ((__VALUE__) == DDL_GTMR_IC_FILTER_FDIV4_N6) \
                                        || ((__VALUE__) == DDL_GTMR_IC_FILTER_FDIV4_N8) \
                                        || ((__VALUE__) == DDL_GTMR_IC_FILTER_FDIV8_N6) \
                                        || ((__VALUE__) == DDL_GTMR_IC_FILTER_FDIV8_N8) \
                                        || ((__VALUE__) == DDL_GTMR_IC_FILTER_FDIV16_N5) \
                                        || ((__VALUE__) == DDL_GTMR_IC_FILTER_FDIV16_N6) \
                                        || ((__VALUE__) == DDL_GTMR_IC_FILTER_FDIV16_N8) \
                                        || ((__VALUE__) == DDL_GTMR_IC_FILTER_FDIV32_N5) \
                                        || ((__VALUE__) == DDL_GTMR_IC_FILTER_FDIV32_N6) \
                                        || ((__VALUE__) == DDL_GTMR_IC_FILTER_FDIV32_N8))

#define IS_DDL_GTMR_IC_POLARITY(__VALUE__) (((__VALUE__) == DDL_GTMR_IC_POLARITY_RISING) \
                                          || ((__VALUE__) == DDL_GTMR_IC_POLARITY_FALLING))

#define IS_DDL_GTMR_ENCODERMODE(__VALUE__) (((__VALUE__) == DDL_GTMR_ENCODERMODE_X2_TI1) \
                                          || ((__VALUE__) == DDL_GTMR_ENCODERMODE_X2_TI2) \
                                          || ((__VALUE__) == DDL_GTMR_ENCODERMODE_X4_TI12))

#define IS_DDL_GTMR_IC_POLARITY_ENCODER(__VALUE__) (((__VALUE__) == DDL_GTMR_IC_POLARITY_RISING) \
                                                  || ((__VALUE__) == DDL_GTMR_IC_POLARITY_FALLING))

#define IS_DDL_GTMR_OSSR_STATE(__VALUE__) (((__VALUE__) == DDL_GTMR_OSSR_DISABLE) \
                                         || ((__VALUE__) == DDL_GTMR_OSSR_ENABLE))

#define IS_DDL_GTMR_OSSI_STATE(__VALUE__) (((__VALUE__) == DDL_GTMR_OSSI_DISABLE) \
                                         || ((__VALUE__) == DDL_GTMR_OSSI_ENABLE))

#define IS_DDL_GTMR_LOCK_LEVEL(__VALUE__) (((__VALUE__) == DDL_GTMR_LOCKLEVEL_OFF) \
                                         || ((__VALUE__) == DDL_GTMR_LOCKLEVEL_1)   \
                                         || ((__VALUE__) == DDL_GTMR_LOCKLEVEL_2)   \
                                         || ((__VALUE__) == DDL_GTMR_LOCKLEVEL_3))

#define IS_DDL_GTMR_BREAK_STATE(__VALUE__) (((__VALUE__) == DDL_GTMR_BREAK_DISABLE) \
                                          || ((__VALUE__) == DDL_GTMR_BREAK_ENABLE))

#define IS_DDL_GTMR_BREAK_POLARITY(__VALUE__) (((__VALUE__) == DDL_GTMR_BREAK_POLARITY_LOW) \
                                             || ((__VALUE__) == DDL_GTMR_BREAK_POLARITY_HIGH))

#define IS_DDL_GTMR_AUTOMATIC_OUTPUT_STATE(__VALUE__) (((__VALUE__) == DDL_GTMR_AUTOMATICOUTPUT_DISABLE) \
                                                     || ((__VALUE__) == DDL_GTMR_AUTOMATICOUTPUT_ENABLE))
/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/
/** @defgroup GTMR_DDL_Private_Functions GTMR Private Functions
  * @{
  */
static ErrorStatus OC0Config(GTMR_TypeDef *TMRx, DDL_GTMR_OC_InitTypeDef *TMR_OCInitStruct);
static ErrorStatus OC1Config(GTMR_TypeDef *TMRx, DDL_GTMR_OC_InitTypeDef *TMR_OCInitStruct);
static ErrorStatus OC2Config(GTMR_TypeDef *TMRx, DDL_GTMR_OC_InitTypeDef *TMR_OCInitStruct);
static ErrorStatus OC3Config(GTMR_TypeDef *TMRx, DDL_GTMR_OC_InitTypeDef *TMR_OCInitStruct);
static ErrorStatus IC0Config(GTMR_TypeDef *TMRx, DDL_GTMR_IC_InitTypeDef *TMR_ICInitStruct);
static ErrorStatus IC1Config(GTMR_TypeDef *TMRx, DDL_GTMR_IC_InitTypeDef *TMR_ICInitStruct);
static ErrorStatus IC2Config(GTMR_TypeDef *TMRx, DDL_GTMR_IC_InitTypeDef *TMR_ICInitStruct);
static ErrorStatus IC3Config(GTMR_TypeDef *TMRx, DDL_GTMR_IC_InitTypeDef *TMR_ICInitStruct);
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup GTMR_DDL_Exported_Functions GTMR Exported Functions
  * @{
  */

/** @addtogroup GTMR_DDL_EF_Init
  * @{
  */

/**
  * @brief  Set TMRx registers to their reset values.
  * @param  TMRx Timer instance
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: TMRx registers are de-initialized
  *          - ERROR: invalid TMRx instance
  */
ErrorStatus DDL_GTMR_DeInit(GTMR_TypeDef *TMRx)
{
  ErrorStatus result = SUCCESS;

  /* Check the parameters */
  ASSERT_PARAM(IS_GTMR_ALL_INSTANCE(TMRx));

  DDL_RCC_Unlock();
  if (TMRx == GTMR)
  {
    DDL_APB_GRP1_ForceReset(DDL_APB_GRP1_PERIPH_GTMR);
    DDL_APB_GRP1_ReleaseReset(DDL_APB_GRP1_PERIPH_GTMR);
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
  * @param  TMR_InitStruct pointer to a @ref DDL_GTMR_InitTypeDef structure (time base unit configuration data structure)
  * @retval None
  */
void DDL_GTMR_StructInit(DDL_GTMR_InitTypeDef *TMR_InitStruct)
{
  /* Set the default configuration */
  TMR_InitStruct->Prescaler         = (uint16_t)0x0000;
  TMR_InitStruct->CounterMode       = DDL_GTMR_COUNTERMODE_UP;
  TMR_InitStruct->Autoreload        = 0xFFFFFFFFU;
  TMR_InitStruct->ClockDivision     = DDL_GTMR_CLOCKDIVISION_DIV1;
}

/**
  * @brief  Configure the TMRx time base unit.
  * @param  TMRx Timer Instance
  * @param  TMR_InitStruct pointer to a @ref DDL_GTMR_InitTypeDef structure
  *         (TMRx time base unit configuration data structure)
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: TMRx registers are de-initialized
  *          - ERROR: not applicable
  */
ErrorStatus DDL_GTMR_Init(GTMR_TypeDef *TMRx, DDL_GTMR_InitTypeDef *TMR_InitStruct)
{
  uint32_t tmpcr1;

  /* Check the parameters */
  ASSERT_PARAM(IS_GTMR_ALL_INSTANCE(TMRx));
  ASSERT_PARAM(IS_DDL_GTMR_COUNTERMODE(TMR_InitStruct->CounterMode));
  ASSERT_PARAM(IS_DDL_GTMR_CLOCKDIVISION(TMR_InitStruct->ClockDivision));

  tmpcr1 = DDL_GTMR_ReadReg(TMRx, CR1);

  if (IS_GTMR_COUNTER_MODE_SELECT_INSTANCE(TMRx))
  {
    /* Select the Counter Mode */
    MODIFY_REG(tmpcr1, (GTMR_CR1_CNTDIR | GTMR_CR1_CAMSEL), TMR_InitStruct->CounterMode);
  }

  if (IS_GTMR_CLOCK_DIVISION_INSTANCE(TMRx))
  {
    /* Set the clock division */
    MODIFY_REG(tmpcr1, GTMR_CR1_CLKDIV, TMR_InitStruct->ClockDivision);
  }

  /* Write to TMRx CTRL1 */
  DDL_GTMR_WriteReg(TMRx, CR1, tmpcr1);

  /* Set the Autoreload value */
  DDL_GTMR_SetAutoReload(TMRx, TMR_InitStruct->Autoreload);

  /* Set the Prescaler value */
  DDL_GTMR_SetPrescaler(TMRx, TMR_InitStruct->Prescaler);

  /* Generate an update event to reload the Prescaler
     and the repetition counter value (if applicable) immediately */
  DDL_GTMR_GenerateEvent_UPDATE(TMRx);

  return SUCCESS;
}

/**
  * @brief  Set the fields of the TMRx output channel configuration data
  *         structure to their default values.
  * @param  TMR_OC_InitStruct pointer to a @ref DDL_GTMR_OC_InitTypeDef structure
  *         (the output channel configuration data structure)
  * @retval None
  */
void DDL_GTMR_OC_StructInit(DDL_GTMR_OC_InitTypeDef *TMR_OC_InitStruct)
{
  /* Set the default configuration */
  TMR_OC_InitStruct->OCMode       = DDL_GTMR_OCMODE_FROZEN;
  TMR_OC_InitStruct->OCState      = DDL_GTMR_OCSTATE_DISABLE;
  TMR_OC_InitStruct->CompareValue = 0x00000000U;
  TMR_OC_InitStruct->OCPolarity   = DDL_GTMR_OCPOLARITY_HIGH;
}

/**
  * @brief  Configure the TMRx output channel.
  * @param  TMRx Timer Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_GTMR_CHANNEL_CH0
  *         @arg @ref DDL_GTMR_CHANNEL_CH1
  *         @arg @ref DDL_GTMR_CHANNEL_CH2
  *         @arg @ref DDL_GTMR_CHANNEL_CH3
  * @param  TMR_OC_InitStruct pointer to a @ref DDL_GTMR_OC_InitTypeDef structure (TMRx output channel configuration
  *         data structure)
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: TMRx output channel is initialized
  *          - ERROR: TMRx output channel is not initialized
  */
ErrorStatus DDL_GTMR_OC_Init(GTMR_TypeDef *TMRx, uint32_t Channel, DDL_GTMR_OC_InitTypeDef *TMR_OC_InitStruct)
{
  ErrorStatus result = ERROR;

  switch (Channel)
  {
    case DDL_GTMR_CHANNEL_CH0:
      result = OC0Config(TMRx, TMR_OC_InitStruct);
      break;
    case DDL_GTMR_CHANNEL_CH1:
      result = OC1Config(TMRx, TMR_OC_InitStruct);
      break;
    case DDL_GTMR_CHANNEL_CH2:
      result = OC2Config(TMRx, TMR_OC_InitStruct);
      break;
    case DDL_GTMR_CHANNEL_CH3:
      result = OC3Config(TMRx, TMR_OC_InitStruct);
      break;
    default:
      break;
  }

  return result;
}

/**
  * @brief  Set the fields of the TMRx input channel configuration data
  *         structure to their default values.
  * @param  TMR_ICInitStruct pointer to a @ref DDL_GTMR_IC_InitTypeDef structure (the input channel configuration
  *         data structure)
  * @retval None
  */
void DDL_GTMR_IC_StructInit(DDL_GTMR_IC_InitTypeDef *TMR_ICInitStruct)
{
  /* Set the default configuration */
  TMR_ICInitStruct->ICPolarity    = DDL_GTMR_IC_POLARITY_RISING;
  TMR_ICInitStruct->ICActiveInput = DDL_GTMR_ACTIVEINPUT_DIRECTTI;
  TMR_ICInitStruct->ICPrescaler   = DDL_GTMR_ICPSC_DIV1;
  TMR_ICInitStruct->ICFilter      = DDL_GTMR_IC_FILTER_FDIV1;
}

/**
  * @brief  Configure the TMRx input channel.
  * @param  TMRx Timer Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_GTMR_CHANNEL_CH0
  *         @arg @ref DDL_GTMR_CHANNEL_CH1
  *         @arg @ref DDL_GTMR_CHANNEL_CH2
  *         @arg @ref DDL_GTMR_CHANNEL_CH3
  * @param  TMR_IC_InitStruct pointer to a @ref DDL_GTMR_IC_InitTypeDef structure (TMRx input channel configuration data
  *         structure)
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: TMRx output channel is initialized
  *          - ERROR: TMRx output channel is not initialized
  */
ErrorStatus DDL_GTMR_IC_Init(GTMR_TypeDef *TMRx, uint32_t Channel, DDL_GTMR_IC_InitTypeDef *TMR_IC_InitStruct)
{
  ErrorStatus result = ERROR;

  switch (Channel)
  {
    case DDL_GTMR_CHANNEL_CH0:
      result = IC0Config(TMRx, TMR_IC_InitStruct);
      break;
    case DDL_GTMR_CHANNEL_CH1:
      result = IC1Config(TMRx, TMR_IC_InitStruct);
      break;
    case DDL_GTMR_CHANNEL_CH2:
      result = IC2Config(TMRx, TMR_IC_InitStruct);
      break;
    case DDL_GTMR_CHANNEL_CH3:
      result = IC3Config(TMRx, TMR_IC_InitStruct);
      break;
    default:
      break;
  }

  return result;
}

/**
  * @brief  Fills each TMR_EncoderInitStruct field with its default value
  * @param  TMR_EncoderInitStruct pointer to a @ref DDL_GTMR_ENCODER_InitTypeDef structure (encoder interface
  *         configuration data structure)
  * @retval None
  */
void DDL_GTMR_ENCODER_StructInit(DDL_GTMR_ENCODER_InitTypeDef *TMR_EncoderInitStruct)
{
  /* Set the default configuration */
  TMR_EncoderInitStruct->EncoderMode    = DDL_GTMR_ENCODERMODE_X2_TI1;
  TMR_EncoderInitStruct->IC1Polarity    = DDL_GTMR_IC_POLARITY_RISING;
  TMR_EncoderInitStruct->IC1ActiveInput = DDL_GTMR_ACTIVEINPUT_DIRECTTI;
  TMR_EncoderInitStruct->IC1Prescaler   = DDL_GTMR_ICPSC_DIV1;
  TMR_EncoderInitStruct->IC1Filter      = DDL_GTMR_IC_FILTER_FDIV1;
  TMR_EncoderInitStruct->IC2Polarity    = DDL_GTMR_IC_POLARITY_RISING;
  TMR_EncoderInitStruct->IC2ActiveInput = DDL_GTMR_ACTIVEINPUT_DIRECTTI;
  TMR_EncoderInitStruct->IC2Prescaler   = DDL_GTMR_ICPSC_DIV1;
  TMR_EncoderInitStruct->IC2Filter      = DDL_GTMR_IC_FILTER_FDIV1;
}

/**
  * @brief  Configure the encoder interface of the timer instance.
  * @param  TMRx Timer Instance
  * @param  TMR_EncoderInitStruct pointer to a @ref DDL_GTMR_ENCODER_InitTypeDef structure (TMRx encoder interface
  *         configuration data structure)
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: TMRx registers are initialized
  *          - ERROR: not applicable
  */
ErrorStatus DDL_GTMR_ENCODER_Init(GTMR_TypeDef *TMRx, DDL_GTMR_ENCODER_InitTypeDef *TMR_EncoderInitStruct)
{
  uint32_t tmpccmr1;
  uint32_t tmpccer;

  /* Check the parameters */
//   ASSERT_PARAM(IS_GTMR_ENCODER_INTERFACE_INSTANCE(TMRx));
  ASSERT_PARAM(IS_DDL_GTMR_ENCODERMODE(TMR_EncoderInitStruct->EncoderMode));
  ASSERT_PARAM(IS_DDL_GTMR_IC_POLARITY_ENCODER(TMR_EncoderInitStruct->IC1Polarity));
  ASSERT_PARAM(IS_DDL_GTMR_ACTIVEINPUT(TMR_EncoderInitStruct->IC1ActiveInput));
  ASSERT_PARAM(IS_DDL_GTMR_ICPSC(TMR_EncoderInitStruct->IC1Prescaler));
  ASSERT_PARAM(IS_DDL_GTMR_IC_FILTER(TMR_EncoderInitStruct->IC1Filter));
  ASSERT_PARAM(IS_DDL_GTMR_IC_POLARITY_ENCODER(TMR_EncoderInitStruct->IC2Polarity));
  ASSERT_PARAM(IS_DDL_GTMR_ACTIVEINPUT(TMR_EncoderInitStruct->IC2ActiveInput));
  ASSERT_PARAM(IS_DDL_GTMR_ICPSC(TMR_EncoderInitStruct->IC2Prescaler));
  ASSERT_PARAM(IS_DDL_GTMR_IC_FILTER(TMR_EncoderInitStruct->IC2Filter));

  /* Disable the CC0 and CC1: Reset the CC0EN and CC1EN Bits */
  TMRx->CCEN &= (uint32_t)~(GTMR_CCEN_CC0EN | GTMR_CCEN_CC1EN);

  /* Get the TMRx CCM1 register value */
  tmpccmr1 = DDL_GTMR_ReadReg(TMRx, CCM1);

  /* Get the TMRx CCEN register value */
  tmpccer = DDL_GTMR_ReadReg(TMRx, CCEN);

  /* Configure TI1 */
  tmpccmr1 &= (uint32_t)~(GTMR_CCM1_CC0SEL | GTMR_CCM1_IC0F | GTMR_CCM1_IC0PSC);
  tmpccmr1 |= (uint32_t)(TMR_EncoderInitStruct->IC1ActiveInput >> 16U);
  tmpccmr1 |= (uint32_t)(TMR_EncoderInitStruct->IC1Filter >> 16U);
  tmpccmr1 |= (uint32_t)(TMR_EncoderInitStruct->IC1Prescaler >> 16U);

  /* Configure TI2 */
  tmpccmr1 &= (uint32_t)~(GTMR_CCM1_CC1SEL | GTMR_CCM1_IC1F | GTMR_CCM1_IC1PSC);
  tmpccmr1 |= (uint32_t)(TMR_EncoderInitStruct->IC2ActiveInput >> 8U);
  tmpccmr1 |= (uint32_t)(TMR_EncoderInitStruct->IC2Filter >> 8U);
  tmpccmr1 |= (uint32_t)(TMR_EncoderInitStruct->IC2Prescaler >> 8U);

  /* Set TI1 and TI2 polarity and enable TI1 and TI2 */
  tmpccer &= (uint32_t)~(GTMR_CCEN_CC0POL | GTMR_CCEN_CC1POL);
  tmpccer |= (uint32_t)(TMR_EncoderInitStruct->IC1Polarity);
  tmpccer |= (uint32_t)(TMR_EncoderInitStruct->IC2Polarity << 4U);
  tmpccer |= (uint32_t)(GTMR_CCEN_CC0EN | GTMR_CCEN_CC1EN);

  /* Set encoder mode */
  DDL_GTMR_SetEncoderMode(TMRx, TMR_EncoderInitStruct->EncoderMode);

  /* Write to TMRx CCM1 */
  DDL_GTMR_WriteReg(TMRx, CCM1, tmpccmr1);

  /* Write to TMRx CCEN */
  DDL_GTMR_WriteReg(TMRx, CCEN, tmpccer);

  return SUCCESS;
}

/**
  * @brief  Set the fields of the TMRx Hall sensor interface configuration data
  *         structure to their default values.
  * @param  TMR_HallSensorInitStruct pointer to a @ref DDL_GTMR_HALLSENSOR_InitTypeDef structure (HALL sensor interface
  *         configuration data structure)
  * @retval None
  */
void DDL_GTMR_HALLSENSOR_StructInit(DDL_GTMR_HALLSENSOR_InitTypeDef *TMR_HallSensorInitStruct)
{
  /* Set the default configuration */
  TMR_HallSensorInitStruct->IC1Polarity       = DDL_GTMR_IC_POLARITY_RISING;
  TMR_HallSensorInitStruct->IC1Prescaler      = DDL_GTMR_ICPSC_DIV1;
  TMR_HallSensorInitStruct->IC1Filter         = DDL_GTMR_IC_FILTER_FDIV1;
  TMR_HallSensorInitStruct->CommutationDelay  = 0U;
}

/**
  * @brief  Configure the Hall sensor interface of the timer instance.
  * @note TMRx CH0, CH1 and CH2 inputs connected through a XOR
  *       to the TI1 input channel
  * @note TMRx slave mode controller is configured in reset mode.
          Selected internal trigger is TI1F_ED.
  * @note Channel 0 is configured as input, IC0 is mapped on TRC.
  * @note Captured value stored in TMRx_CCR0 correspond to the time elapsed
  *       between 2 changes on the inputs. It gives information about motor speed.
  * @note Channel 1 is configured in output PWM 2 mode.
  * @note Compare value stored in TMRx_CCR1 corresponds to the commutation delay.
  * @note OC1REF is selected as trigger output on TRGO.
  * @param  TMRx Timer Instance
  * @param  TMR_HallSensorInitStruct pointer to a @ref DDL_GTMR_HALLSENSOR_InitTypeDef structure (TMRx DALL sensor
  *         interface configuration data structure)
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: TMRx registers are de-initialized
  *          - ERROR: not applicable
  */
ErrorStatus DDL_GTMR_HALLSENSOR_Init(GTMR_TypeDef *TMRx, DDL_GTMR_HALLSENSOR_InitTypeDef *TMR_HallSensorInitStruct)
{
  uint32_t tmpcr2;
  uint32_t tmpccmr1;
  uint32_t tmpccer;
  uint32_t tmpsmcr;

  /* Check the parameters */
//   ASSERT_PARAM(IS_GTMR_HALL_SENSOR_INTERFACE_INSTANCE(TMRx));
  ASSERT_PARAM(IS_DDL_GTMR_IC_POLARITY_ENCODER(TMR_HallSensorInitStruct->IC1Polarity));
  ASSERT_PARAM(IS_DDL_GTMR_ICPSC(TMR_HallSensorInitStruct->IC1Prescaler));
  ASSERT_PARAM(IS_DDL_GTMR_IC_FILTER(TMR_HallSensorInitStruct->IC1Filter));

  /* Disable the CC0 and CC1: Reset the CC0E and CC1E Bits */
  TMRx->CCEN &= (uint32_t)~(GTMR_CCEN_CC0EN | GTMR_CCEN_CC1EN);

  /* Get the TMRx CTRL2 register value */
  tmpcr2 = DDL_GTMR_ReadReg(TMRx, CR2);

  /* Get the TMRx CCM1 register value */
  tmpccmr1 = DDL_GTMR_ReadReg(TMRx, CCM1);

  /* Get the TMRx CCEN register value */
  tmpccer = DDL_GTMR_ReadReg(TMRx, CCEN);

  /* Get the TMRx SMCTRL register value */
  tmpsmcr = DDL_GTMR_ReadReg(TMRx, SMCR);

  /* Connect TMRx_CH0, CH1 and CH2 pins to the TI1 input */
  tmpcr2 |= GTMR_CR2_TI0SEL;

  /* OC2REF signal is used as trigger output (TRGO) */
  tmpcr2 |= DDL_GTMR_TRGO_OC2REF;

  /* Configure the slave mode controller */
  tmpsmcr &= (uint32_t)~(GTMR_SMCR_TRGSEL | GTMR_SMCR_SMFSEL);
  tmpsmcr |= DDL_GTMR_TS_TI1F_ED;
  tmpsmcr |= DDL_GTMR_SLAVEMODE_RESET;

  /* Configure input channel 0 */
  tmpccmr1 &= (uint32_t)~(GTMR_CCM1_CC0SEL | GTMR_CCM1_IC0F | GTMR_CCM1_IC0PSC);
  tmpccmr1 |= (uint32_t)(DDL_GTMR_ACTIVEINPUT_TRC >> 16U);
  tmpccmr1 |= (uint32_t)(TMR_HallSensorInitStruct->IC1Filter >> 16U);
  tmpccmr1 |= (uint32_t)(TMR_HallSensorInitStruct->IC1Prescaler >> 16U);

  /* Configure output channel 1 */
  tmpccmr1 &= (uint32_t)~(GTMR_CCM1_OC1MOD | GTMR_CCM1_OC1PEN | GTMR_CCM1_OC1CEN);
  tmpccmr1 |= (uint32_t)(DDL_GTMR_OCMODE_PWM2 << 8U);

  /* Set Channel 0 polarity and enable Channel 0 and Channel 1 */
  tmpccer &= (uint32_t)~(GTMR_CCEN_CC0POL | GTMR_CCEN_CC1POL);
  tmpccer |= (uint32_t)(TMR_HallSensorInitStruct->IC1Polarity);
  tmpccer |= (uint32_t)(GTMR_CCEN_CC0EN | GTMR_CCEN_CC1EN);

  /* Write to TMRx CTRL2 */
  DDL_GTMR_WriteReg(TMRx, CR2, tmpcr2);

  /* Write to TMRx SMCTRL */
  DDL_GTMR_WriteReg(TMRx, SMCR, tmpsmcr);

  /* Write to TMRx CCM1 */
  DDL_GTMR_WriteReg(TMRx, CCM1, tmpccmr1);

  /* Write to TMRx CCEN */
  DDL_GTMR_WriteReg(TMRx, CCEN, tmpccer);

  /* Write to TMRx CC1 */
  DDL_GTMR_OC_SetCompareCH1(TMRx, TMR_HallSensorInitStruct->CommutationDelay);

  return SUCCESS;
}

/**
  * @}
  */

/**
  * @}
  */

/** @addtogroup GTMR_DDL_Private_Functions TMR Private Functions
  *  @brief   Private functions
  * @{
  */
/**
  * @brief  Configure the TMRx output channel 1.
  * @param  TMRx Timer Instance
  * @param  TMR_OCInitStruct pointer to the the TMRx output channel 1 configuration data structure
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: TMRx registers are de-initialized
  *          - ERROR: not applicable
  */
static ErrorStatus OC0Config(GTMR_TypeDef *TMRx, DDL_GTMR_OC_InitTypeDef *TMR_OCInitStruct)
{
  uint32_t tmpccmr1;
  uint32_t tmpccer;
  uint32_t tmpcr2;

  /* Check the parameters */
//   ASSERT_PARAM(IS_GTMR_CC0_INSTANCE(TMRx));
  ASSERT_PARAM(IS_DDL_GTMR_OCMODE(TMR_OCInitStruct->OCMode));
  ASSERT_PARAM(IS_DDL_GTMR_OCSTATE(TMR_OCInitStruct->OCState));
  ASSERT_PARAM(IS_DDL_GTMR_OCPOLARITY(TMR_OCInitStruct->OCPolarity));

  /* Disable the Channel 0: Reset the CC0E Bit */
  CLEAR_BIT(TMRx->CCEN, GTMR_CCEN_CC0EN);

  /* Get the TMRx CCEN register value */
  tmpccer = DDL_GTMR_ReadReg(TMRx, CCEN);

  /* Get the TMRx CTRL2 register value */
  tmpcr2 = DDL_GTMR_ReadReg(TMRx, CR2);

  /* Get the TMRx CCM1 register value */
  tmpccmr1 = DDL_GTMR_ReadReg(TMRx, CCM1);

  /* Reset Capture/Compare selection Bits */
  CLEAR_BIT(tmpccmr1, GTMR_CCM1_CC0SEL);

  /* Set the Output Compare Mode */
  MODIFY_REG(tmpccmr1, GTMR_CCM1_OC0MOD, TMR_OCInitStruct->OCMode << (GTMR_CCM1_OC0MOD_Pos-GTMR_CCM1_OC0MOD_Pos));

  /* Set the Output Compare Polarity */
  MODIFY_REG(tmpccer, GTMR_CCEN_CC0POL, TMR_OCInitStruct->OCPolarity << (GTMR_CCEN_CC0POL_Pos-GTMR_CCEN_CC0POL_Pos));

  /* Set the Output State */
  MODIFY_REG(tmpccer, GTMR_CCEN_CC0EN, TMR_OCInitStruct->OCState << (GTMR_CCEN_CC0EN_Pos-GTMR_CCEN_CC0EN_Pos));

  /* Write to TMRx CTRL2 */
  DDL_GTMR_WriteReg(TMRx, CR2, tmpcr2);

  /* Write to TMRx CCM1 */
  DDL_GTMR_WriteReg(TMRx, CCM1, tmpccmr1);

  /* Set the Capture Compare Register value */
  DDL_GTMR_OC_SetCompareCH0(TMRx, TMR_OCInitStruct->CompareValue);

  /* Write to TMRx CCEN */
  DDL_GTMR_WriteReg(TMRx, CCEN, tmpccer);

  return SUCCESS;
}

/**
  * @brief  Configure the TMRx output channel 1.
  * @param  TMRx Timer Instance
  * @param  TMR_OCInitStruct pointer to the the TMRx output channel 1 configuration data structure
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: TMRx registers are de-initialized
  *          - ERROR: not applicable
  */
static ErrorStatus OC1Config(GTMR_TypeDef *TMRx, DDL_GTMR_OC_InitTypeDef *TMR_OCInitStruct)
{
  uint32_t tmpccmr1;
  uint32_t tmpccer;
  uint32_t tmpcr2;

  /* Check the parameters */
//   ASSERT_PARAM(IS_GTMR_CC1_INSTANCE(TMRx));
  ASSERT_PARAM(IS_DDL_GTMR_OCMODE(TMR_OCInitStruct->OCMode));
  ASSERT_PARAM(IS_DDL_GTMR_OCSTATE(TMR_OCInitStruct->OCState));
  ASSERT_PARAM(IS_DDL_GTMR_OCPOLARITY(TMR_OCInitStruct->OCPolarity));

  /* Disable the Channel 2: Reset the CC1E Bit */
  CLEAR_BIT(TMRx->CCEN, GTMR_CCEN_CC1EN);

  /* Get the TMRx CCEN register value */
  tmpccer =  DDL_GTMR_ReadReg(TMRx, CCEN);

  /* Get the TMRx CTRL2 register value */
  tmpcr2 = DDL_GTMR_ReadReg(TMRx, CR2);

  /* Get the TMRx CCM1 register value */
  tmpccmr1 = DDL_GTMR_ReadReg(TMRx, CCM1);

  /* Reset Capture/Compare selection Bits */
  CLEAR_BIT(tmpccmr1, GTMR_CCM1_CC1SEL);

  /* Select the Output Compare Mode */
  MODIFY_REG(tmpccmr1, GTMR_CCM1_OC1MOD, TMR_OCInitStruct->OCMode << (GTMR_CCM1_OC1MOD_Pos-GTMR_CCM1_OC0MOD_Pos));

  /* Set the Output Compare Polarity */
  MODIFY_REG(tmpccer, GTMR_CCEN_CC1POL, TMR_OCInitStruct->OCPolarity << (GTMR_CCEN_CC1POL_Pos-GTMR_CCEN_CC0POL_Pos));

  /* Set the Output State */
  MODIFY_REG(tmpccer, GTMR_CCEN_CC1EN, TMR_OCInitStruct->OCState << (GTMR_CCEN_CC1EN_Pos-GTMR_CCEN_CC0EN_Pos));

  /* Write to TMRx CTRL2 */
  DDL_GTMR_WriteReg(TMRx, CR2, tmpcr2);

  /* Write to TMRx CCM1 */
  DDL_GTMR_WriteReg(TMRx, CCM1, tmpccmr1);

  /* Set the Capture Compare Register value */
  DDL_GTMR_OC_SetCompareCH1(TMRx, TMR_OCInitStruct->CompareValue);

  /* Write to TMRx CCEN */
  DDL_GTMR_WriteReg(TMRx, CCEN, tmpccer);

  return SUCCESS;
}

/**
  * @brief  Configure the TMRx output channel 2.
  * @param  TMRx Timer Instance
  * @param  TMR_OCInitStruct pointer to the the TMRx output channel 2 configuration data structure
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: TMRx registers are de-initialized
  *          - ERROR: not applicable
  */
static ErrorStatus OC2Config(GTMR_TypeDef *TMRx, DDL_GTMR_OC_InitTypeDef *TMR_OCInitStruct)
{
  uint32_t tmpccmr2;
  uint32_t tmpccer;
  uint32_t tmpcr2;

  /* Check the parameters */
//   ASSERT_PARAM(IS_GTMR_CC2_INSTANCE(TMRx));
  ASSERT_PARAM(IS_DDL_GTMR_OCMODE(TMR_OCInitStruct->OCMode));
  ASSERT_PARAM(IS_DDL_GTMR_OCSTATE(TMR_OCInitStruct->OCState));
  ASSERT_PARAM(IS_DDL_GTMR_OCPOLARITY(TMR_OCInitStruct->OCPolarity));

  /* Disable the Channel 3: Reset the CC2E Bit */
  CLEAR_BIT(TMRx->CCEN, GTMR_CCEN_CC2EN);

  /* Get the TMRx CCEN register value */
  tmpccer =  DDL_GTMR_ReadReg(TMRx, CCEN);

  /* Get the TMRx CTRL2 register value */
  tmpcr2 = DDL_GTMR_ReadReg(TMRx, CR2);

  /* Get the TMRx CCM2 register value */
  tmpccmr2 = DDL_GTMR_ReadReg(TMRx, CCM2);

  /* Reset Capture/Compare selection Bits */
  CLEAR_BIT(tmpccmr2, GTMR_CCM2_CC2SEL);

  /* Select the Output Compare Mode */
  MODIFY_REG(tmpccmr2, GTMR_CCM2_OC2MOD, TMR_OCInitStruct->OCMode << (GTMR_CCM2_OC2MOD_Pos-GTMR_CCM2_OC2MOD_Pos));

  /* Set the Output Compare Polarity */
  MODIFY_REG(tmpccer, GTMR_CCEN_CC2POL, TMR_OCInitStruct->OCPolarity << (GTMR_CCEN_CC2POL_Pos-GTMR_CCEN_CC0POL_Pos));

  /* Set the Output State */
  MODIFY_REG(tmpccer, GTMR_CCEN_CC2EN, TMR_OCInitStruct->OCState << (GTMR_CCEN_CC2EN_Pos-GTMR_CCEN_CC0EN_Pos));

  /* Write to TMRx CTRL2 */
  DDL_GTMR_WriteReg(TMRx, CR2, tmpcr2);

  /* Write to TMRx CCM2 */
  DDL_GTMR_WriteReg(TMRx, CCM2, tmpccmr2);

  /* Set the Capture Compare Register value */
  DDL_GTMR_OC_SetCompareCH2(TMRx, TMR_OCInitStruct->CompareValue);

  /* Write to TMRx CCEN */
  DDL_GTMR_WriteReg(TMRx, CCEN, tmpccer);

  return SUCCESS;
}

/**
  * @brief  Configure the TMRx output channel 3.
  * @param  TMRx Timer Instance
  * @param  TMR_OCInitStruct pointer to the the TMRx output channel 3 configuration data structure
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: TMRx registers are de-initialized
  *          - ERROR: not applicable
  */
static ErrorStatus OC3Config(GTMR_TypeDef *TMRx, DDL_GTMR_OC_InitTypeDef *TMR_OCInitStruct)
{
  uint32_t tmpccmr2;
  uint32_t tmpccer;
  uint32_t tmpcr2;

  /* Check the parameters */
//   ASSERT_PARAM(IS_GTMR_CC3_INSTANCE(TMRx));
  ASSERT_PARAM(IS_DDL_GTMR_OCMODE(TMR_OCInitStruct->OCMode));
  ASSERT_PARAM(IS_DDL_GTMR_OCSTATE(TMR_OCInitStruct->OCState));
  ASSERT_PARAM(IS_DDL_GTMR_OCPOLARITY(TMR_OCInitStruct->OCPolarity));

  /* Disable the Channel 3: Reset the CC3E Bit */
  CLEAR_BIT(TMRx->CCEN, GTMR_CCEN_CC3EN);

  /* Get the TMRx CCEN register value */
  tmpccer = DDL_GTMR_ReadReg(TMRx, CCEN);

  /* Get the TMRx CTRL2 register value */
  tmpcr2 =  DDL_GTMR_ReadReg(TMRx, CR2);

  /* Get the TMRx CCM2 register value */
  tmpccmr2 = DDL_GTMR_ReadReg(TMRx, CCM2);

  /* Reset Capture/Compare selection Bits */
  CLEAR_BIT(tmpccmr2, GTMR_CCM2_CC3SEL);

  /* Select the Output Compare Mode */
  MODIFY_REG(tmpccmr2, GTMR_CCM2_OC3MOD, TMR_OCInitStruct->OCMode << (GTMR_CCM2_OC3MOD_Pos-GTMR_CCM2_OC2MOD_Pos));

  /* Set the Output Compare Polarity */
  MODIFY_REG(tmpccer, GTMR_CCEN_CC3POL, TMR_OCInitStruct->OCPolarity << (GTMR_CCEN_CC3POL_Pos-GTMR_CCEN_CC0POL_Pos));

  /* Set the Output State */
  MODIFY_REG(tmpccer, GTMR_CCEN_CC3EN, TMR_OCInitStruct->OCState << (GTMR_CCEN_CC3EN_Pos-GTMR_CCEN_CC0EN_Pos));

  /* Write to TMRx CTRL2 */
  DDL_GTMR_WriteReg(TMRx, CR2, tmpcr2);

  /* Write to TMRx CCM2 */
  DDL_GTMR_WriteReg(TMRx, CCM2, tmpccmr2);

  /* Set the Capture Compare Register value */
  DDL_GTMR_OC_SetCompareCH3(TMRx, TMR_OCInitStruct->CompareValue);

  /* Write to TMRx CCEN */
  DDL_GTMR_WriteReg(TMRx, CCEN , tmpccer);

  return SUCCESS;
}

/**
  * @brief  Configure the TMRx input channel 0.
  * @param  TMRx Timer Instance
  * @param  TMR_ICInitStruct pointer to the the TMRx input channel 0 configuration data structure
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: TMRx registers are de-initialized
  *          - ERROR: not applicable
  */
static ErrorStatus IC0Config(GTMR_TypeDef *TMRx, DDL_GTMR_IC_InitTypeDef *TMR_ICInitStruct)
{
  /* Check the parameters */
//   ASSERT_PARAM(IS_GTMR_CC0_INSTANCE(TMRx));
  ASSERT_PARAM(IS_DDL_GTMR_IC_POLARITY(TMR_ICInitStruct->ICPolarity));
  ASSERT_PARAM(IS_DDL_GTMR_ACTIVEINPUT(TMR_ICInitStruct->ICActiveInput));
  ASSERT_PARAM(IS_DDL_GTMR_ICPSC(TMR_ICInitStruct->ICPrescaler));
  ASSERT_PARAM(IS_DDL_GTMR_IC_FILTER(TMR_ICInitStruct->ICFilter));

  /* Disable the Channel 0: Reset the CC0E Bit */
  TMRx->CCEN &= (uint32_t)~GTMR_CCEN_CC0EN;

  /* Select the Input and set the filter and the prescaler value */
  MODIFY_REG(TMRx->CCM1,
             (GTMR_CCM1_CC0SEL | GTMR_CCM1_IC0F | GTMR_CCM1_IC0PSC),
             (TMR_ICInitStruct->ICActiveInput | TMR_ICInitStruct->ICFilter | TMR_ICInitStruct->ICPrescaler) >> 16U);

  /* Select the Polarity and set the CC0E Bit */
  MODIFY_REG(TMRx->CCEN,
             (GTMR_CCEN_CC0POL),
             (TMR_ICInitStruct->ICPolarity<<(GTMR_CCEN_CC0POL_Pos-GTMR_CCEN_CC0POL_Pos) | GTMR_CCEN_CC0EN));

  return SUCCESS;
}

/**
  * @brief  Configure the TMRx input channel 1.
  * @param  TMRx Timer Instance
  * @param  TMR_ICInitStruct pointer to the the TMRx input channel 1 configuration data structure
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: TMRx registers are de-initialized
  *          - ERROR: not applicable
  */
static ErrorStatus IC1Config(GTMR_TypeDef *TMRx, DDL_GTMR_IC_InitTypeDef *TMR_ICInitStruct)
{
  /* Check the parameters */
//   ASSERT_PARAM(IS_GTMR_CC1_INSTANCE(TMRx));
  ASSERT_PARAM(IS_DDL_GTMR_IC_POLARITY(TMR_ICInitStruct->ICPolarity));
  ASSERT_PARAM(IS_DDL_GTMR_ACTIVEINPUT(TMR_ICInitStruct->ICActiveInput));
  ASSERT_PARAM(IS_DDL_GTMR_ICPSC(TMR_ICInitStruct->ICPrescaler));
  ASSERT_PARAM(IS_DDL_GTMR_IC_FILTER(TMR_ICInitStruct->ICFilter));

  /* Disable the Channel 1: Reset the CC1E Bit */
  TMRx->CCEN &= (uint32_t)~GTMR_CCEN_CC1EN;

  /* Select the Input and set the filter and the prescaler value */
  MODIFY_REG(TMRx->CCM1,
             (GTMR_CCM1_CC1SEL | GTMR_CCM1_IC1F | GTMR_CCM1_IC1PSC),
             (TMR_ICInitStruct->ICActiveInput | TMR_ICInitStruct->ICFilter | TMR_ICInitStruct->ICPrescaler) >> 8U);

  /* Select the Polarity and set the CC1E Bit */
  MODIFY_REG(TMRx->CCEN,
             (GTMR_CCEN_CC1POL),
             ((TMR_ICInitStruct->ICPolarity << (GTMR_CCEN_CC1POL_Pos-GTMR_CCEN_CC0POL_Pos)) | GTMR_CCEN_CC1EN));

  return SUCCESS;
}

/**
  * @brief  Configure the TMRx input channel 2.
  * @param  TMRx Timer Instance
  * @param  TMR_ICInitStruct pointer to the the TMRx input channel 2 configuration data structure
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: TMRx registers are de-initialized
  *          - ERROR: not applicable
  */
static ErrorStatus IC2Config(GTMR_TypeDef *TMRx, DDL_GTMR_IC_InitTypeDef *TMR_ICInitStruct)
{
  /* Check the parameters */
//   ASSERT_PARAM(IS_GTMR_CC2_INSTANCE(TMRx));
  ASSERT_PARAM(IS_DDL_GTMR_IC_POLARITY(TMR_ICInitStruct->ICPolarity));
  ASSERT_PARAM(IS_DDL_GTMR_ACTIVEINPUT(TMR_ICInitStruct->ICActiveInput));
  ASSERT_PARAM(IS_DDL_GTMR_ICPSC(TMR_ICInitStruct->ICPrescaler));
  ASSERT_PARAM(IS_DDL_GTMR_IC_FILTER(TMR_ICInitStruct->ICFilter));

  /* Disable the Channel 2: Reset the CC2E Bit */
  TMRx->CCEN &= (uint32_t)~GTMR_CCEN_CC2EN;

  /* Select the Input and set the filter and the prescaler value */
  MODIFY_REG(TMRx->CCM2,
             (GTMR_CCM2_CC2SEL | GTMR_CCM2_IC2F | GTMR_CCM2_IC2PSC),
             (TMR_ICInitStruct->ICActiveInput | TMR_ICInitStruct->ICFilter | TMR_ICInitStruct->ICPrescaler) >> 16U);

  /* Select the Polarity and set the CC2E Bit */
  MODIFY_REG(TMRx->CCEN,
             (GTMR_CCEN_CC2POL),
             ((TMR_ICInitStruct->ICPolarity << (GTMR_CCEN_CC2POL_Pos-GTMR_CCEN_CC0POL_Pos)) | GTMR_CCEN_CC2EN));

  return SUCCESS;
}

/**
  * @brief  Configure the TMRx input channel 3.
  * @param  TMRx Timer Instance
  * @param  TMR_ICInitStruct pointer to the the TMRx input channel 3 configuration data structure
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: TMRx registers are de-initialized
  *          - ERROR: not applicable
  */
static ErrorStatus IC3Config(GTMR_TypeDef *TMRx, DDL_GTMR_IC_InitTypeDef *TMR_ICInitStruct)
{
  /* Check the parameters */
//   ASSERT_PARAM(IS_GTMR_CC3_INSTANCE(TMRx));
  ASSERT_PARAM(IS_DDL_GTMR_IC_POLARITY(TMR_ICInitStruct->ICPolarity));
  ASSERT_PARAM(IS_DDL_GTMR_ACTIVEINPUT(TMR_ICInitStruct->ICActiveInput));
  ASSERT_PARAM(IS_DDL_GTMR_ICPSC(TMR_ICInitStruct->ICPrescaler));
  ASSERT_PARAM(IS_DDL_GTMR_IC_FILTER(TMR_ICInitStruct->ICFilter));

  /* Disable the Channel 3: Reset the CC3E Bit */
  TMRx->CCEN &= (uint32_t)~GTMR_CCEN_CC3EN;

  /* Select the Input and set the filter and the prescaler value */
  MODIFY_REG(TMRx->CCM2,
             (GTMR_CCM2_CC3SEL | GTMR_CCM2_IC3F | GTMR_CCM2_IC3PSC),
             (TMR_ICInitStruct->ICActiveInput | TMR_ICInitStruct->ICFilter | TMR_ICInitStruct->ICPrescaler) >> 8U);

  /* Select the Polarity and set the CC3E Bit */
  MODIFY_REG(TMRx->CCEN,
             (GTMR_CCEN_CC3POL),
             ((TMR_ICInitStruct->ICPolarity << (GTMR_CCEN_CC3POL_Pos-GTMR_CCEN_CC0POL_Pos)) | GTMR_CCEN_CC3EN));

  return SUCCESS;
}

/**
  * @}
  */

/**
  * @}
  */

#endif /* GTMR */

/**
  * @}
  */

#endif /* USE_FULL_DDL_DRIVER */

