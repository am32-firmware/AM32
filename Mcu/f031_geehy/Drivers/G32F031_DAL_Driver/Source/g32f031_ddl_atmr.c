/**
  *
  * @file    g32f031_ddl_atmr.c
  * @brief   ATMR DDL module driver.
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
#include "g32f031_ddl_atmr.h"
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

#if defined (ATMR)

/** @addtogroup ATMR_DDL ATMR
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/** @addtogroup ATMR_DDL_Private_Macros ATMR Private Macros
  * @{
  */
#define IS_ATMR_COUNTER_MODE_SELECT_INSTANCE(__VALUE__) ((__VALUE__) == ATMR)
#define IS_ATMR_CLOCK_DIVISION_INSTANCE(__VALUE__) ((__VALUE__) == ATMR)
#define IS_DDL_ATMR_COUNTERMODE(__VALUE__) (((__VALUE__) == DDL_ATMR_COUNTERMODE_UP) \
                                          || ((__VALUE__) == DDL_ATMR_COUNTERMODE_DOWN) \
                                          || ((__VALUE__) == DDL_ATMR_COUNTERMODE_CENTER_UP) \
                                          || ((__VALUE__) == DDL_ATMR_COUNTERMODE_CENTER_DOWN) \
                                          || ((__VALUE__) == DDL_ATMR_COUNTERMODE_CENTER_UP_DOWN))

#define IS_DDL_ATMR_CLOCKDIVISION(__VALUE__) (((__VALUE__) == DDL_ATMR_CLOCKDIVISION_DIV1) \
                                            || ((__VALUE__) == DDL_ATMR_CLOCKDIVISION_DIV2) \
                                            || ((__VALUE__) == DDL_ATMR_CLOCKDIVISION_DIV4))

#define IS_DDL_ATMR_OCMODE(__VALUE__) (((__VALUE__) == DDL_ATMR_OCMODE_FROZEN) \
                                     || ((__VALUE__) == DDL_ATMR_OCMODE_ACTIVE) \
                                     || ((__VALUE__) == DDL_ATMR_OCMODE_INACTIVE) \
                                     || ((__VALUE__) == DDL_ATMR_OCMODE_TOGGLE) \
                                     || ((__VALUE__) == DDL_ATMR_OCMODE_FORCED_INACTIVE) \
                                     || ((__VALUE__) == DDL_ATMR_OCMODE_FORCED_ACTIVE) \
                                     || ((__VALUE__) == DDL_ATMR_OCMODE_PWM1) \
                                     || ((__VALUE__) == DDL_ATMR_OCMODE_PWM2))

#define IS_DDL_ATMR_OCSTATE(__VALUE__) (((__VALUE__) == DDL_ATMR_OCSTATE_DISABLE) \
                                      || ((__VALUE__) == DDL_ATMR_OCSTATE_ENABLE))

#define IS_DDL_ATMR_OCPOLARITY(__VALUE__) (((__VALUE__) == DDL_ATMR_OCPOLARITY_HIGH) \
                                         || ((__VALUE__) == DDL_ATMR_OCPOLARITY_LOW))

#define IS_DDL_ATMR_OCIDLESTATE(__VALUE__) (((__VALUE__) == DDL_ATMR_OCIDLESTATE_LOW) \
                                          || ((__VALUE__) == DDL_ATMR_OCIDLESTATE_HIGH))

#define IS_DDL_ATMR_OSSR_STATE(__VALUE__) (((__VALUE__) == DDL_ATMR_OSSR_DISABLE) \
                                         || ((__VALUE__) == DDL_ATMR_OSSR_ENABLE))

#define IS_DDL_ATMR_OSSI_STATE(__VALUE__) (((__VALUE__) == DDL_ATMR_OSSI_DISABLE) \
                                         || ((__VALUE__) == DDL_ATMR_OSSI_ENABLE))

#define IS_DDL_ATMR_LOCK_LEVEL(__VALUE__) (((__VALUE__) == DDL_ATMR_LOCKLEVEL_OFF) \
                                         || ((__VALUE__) == DDL_ATMR_LOCKLEVEL_1)   \
                                         || ((__VALUE__) == DDL_ATMR_LOCKLEVEL_2)   \
                                         || ((__VALUE__) == DDL_ATMR_LOCKLEVEL_3))

#define IS_DDL_ATMR_BREAK_STATE(__VALUE__) (((__VALUE__) == DDL_ATMR_BREAK_DISABLE) \
                                          || ((__VALUE__) == DDL_ATMR_BREAK_ENABLE))

#define IS_DDL_ATMR_BREAK_POLARITY(__VALUE__) (((__VALUE__) == DDL_ATMR_BREAK_POLARITY_LOW) \
                                             || ((__VALUE__) == DDL_ATMR_BREAK_POLARITY_HIGH))

#define IS_DDL_ATMR_AUTOMATIC_OUTPUT_STATE(__VALUE__) (((__VALUE__) == DDL_ATMR_AUTOMATICOUTPUT_DISABLE) \
                                                     || ((__VALUE__) == DDL_ATMR_AUTOMATICOUTPUT_ENABLE))
/**
  * @}
  */


/* Private function prototypes -----------------------------------------------*/
/** @defgroup ATMR_DDL_Private_Functions ATMR Private Functions
  * @{
  */
static ErrorStatus OC0Config(ATMR_TypeDef *TMRx, DDL_ATMR_OC_InitTypeDef *TMR_OCInitStruct);
static ErrorStatus OC1Config(ATMR_TypeDef *TMRx, DDL_ATMR_OC_InitTypeDef *TMR_OCInitStruct);
static ErrorStatus OC2Config(ATMR_TypeDef *TMRx, DDL_ATMR_OC_InitTypeDef *TMR_OCInitStruct);
static ErrorStatus OC3Config(ATMR_TypeDef *TMRx, DDL_ATMR_OC_InitTypeDef *TMR_OCInitStruct);
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup ATMR_DDL_Exported_Functions ATMR Exported Functions
  * @{
  */

/** @addtogroup ATMR_DDL_EF_Init
  * @{
  */

/**
  * @brief  Set TMRx registers to their reset values.
  * @param  TMRx Timer instance
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: TMRx registers are de-initialized
  *          - ERROR: invalid TMRx instance
  */
ErrorStatus DDL_ATMR_DeInit(ATMR_TypeDef *TMRx)
{
  ErrorStatus result = SUCCESS;

  /* Check the parameters */
  ASSERT_PARAM(IS_ATMR_ALL_INSTANCE(TMRx));

  DDL_RCC_Unlock();
  if (TMRx == ATMR)
  {
    DDL_APB_GRP1_ForceReset(DDL_APB_GRP1_PERIPH_ATMR);
    DDL_APB_GRP1_ReleaseReset(DDL_APB_GRP1_PERIPH_ATMR);
  }
  else
  {
    result = ERROR;
  }

  DDL_RCC_Unlock();
  return result;
}

/**
  * @brief  Set the fields of the time base unit configuration data structure
  *         to their default values.
  * @param  TMR_InitStruct pointer to a @ref DDL_ATMR_InitTypeDef structure (time base unit configuration data structure)
  * @retval None
  */
void DDL_ATMR_StructInit(DDL_ATMR_InitTypeDef *TMR_InitStruct)
{
  /* Set the default configuration */
  TMR_InitStruct->Prescaler         = (uint16_t)0x0000;
  TMR_InitStruct->CounterMode       = DDL_ATMR_COUNTERMODE_UP;
  TMR_InitStruct->Autoreload        = 0xFFFFFFFFU;
  TMR_InitStruct->ClockDivision     = DDL_ATMR_CLOCKDIVISION_DIV1;
  TMR_InitStruct->RepetitionCounter = 0x00000000U;
}

/**
  * @brief  Configure the TMRx time base unit.
  * @param  TMRx Timer Instance
  * @param  TMR_InitStruct pointer to a @ref DDL_ATMR_InitTypeDef structure
  *         (TMRx time base unit configuration data structure)
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: TMRx registers are de-initialized
  *          - ERROR: not applicable
  */
ErrorStatus DDL_ATMR_Init(ATMR_TypeDef *TMRx, DDL_ATMR_InitTypeDef *TMR_InitStruct)
{
  uint32_t tmpcr1;

  /* Check the parameters */
  ASSERT_PARAM(IS_ATMR_ALL_INSTANCE(TMRx));
  ASSERT_PARAM(IS_DDL_ATMR_COUNTERMODE(TMR_InitStruct->CounterMode));
  ASSERT_PARAM(IS_DDL_ATMR_CLOCKDIVISION(TMR_InitStruct->ClockDivision));

  tmpcr1 = DDL_ATMR_ReadReg(TMRx, CR1);

  if (IS_ATMR_COUNTER_MODE_SELECT_INSTANCE(TMRx))
  {
    /* Select the Counter Mode */
    MODIFY_REG(tmpcr1, (ATMR_CR1_CNTDIR | ATMR_CR1_CAMSEL), TMR_InitStruct->CounterMode);
  }

  if (IS_ATMR_CLOCK_DIVISION_INSTANCE(TMRx))
  {
    /* Set the clock division */
    MODIFY_REG(tmpcr1, ATMR_CR1_CLKDIV, TMR_InitStruct->ClockDivision);
  }

  /* Write to TMRx CTRL1 */
  DDL_ATMR_WriteReg(TMRx, CR1, tmpcr1);

  /* Set the Autoreload value */
  DDL_ATMR_SetAutoReload(TMRx, TMR_InitStruct->Autoreload);

  /* Set the Prescaler value */
  DDL_ATMR_SetPrescaler(TMRx, TMR_InitStruct->Prescaler);

  if (IS_TMR_REPETITION_COUNTER_INSTANCE(TMRx))
  {
    /* Set the Repetition Counter value */
    DDL_ATMR_SetRepetitionCounter(TMRx, TMR_InitStruct->RepetitionCounter);
  }

  /* Generate an update event to reload the Prescaler
     and the repetition counter value (if applicable) immediately */
  DDL_ATMR_GenerateEvent_UPDATE(TMRx);

  return SUCCESS;
}

/**
  * @brief  Set the fields of the TMRx output channel configuration data
  *         structure to their default values.
  * @param  TMR_OC_InitStruct pointer to a @ref DDL_ATMR_OC_InitTypeDef structure
  *         (the output channel configuration data structure)
  * @retval None
  */
void DDL_ATMR_OC_StructInit(DDL_ATMR_OC_InitTypeDef *TMR_OC_InitStruct)
{
  /* Set the default configuration */
  TMR_OC_InitStruct->OCMode       = DDL_ATMR_OCMODE_FROZEN;
  TMR_OC_InitStruct->OCState      = DDL_ATMR_OCSTATE_DISABLE;
  TMR_OC_InitStruct->OCNState     = DDL_ATMR_OCSTATE_DISABLE;
  TMR_OC_InitStruct->CompareValue = 0x00000000U;
  TMR_OC_InitStruct->OCPolarity   = DDL_ATMR_OCPOLARITY_HIGH;
  TMR_OC_InitStruct->OCNPolarity  = DDL_ATMR_OCPOLARITY_HIGH;
  TMR_OC_InitStruct->OCIdleState  = DDL_ATMR_OCIDLESTATE_LOW;
  TMR_OC_InitStruct->OCNIdleState = DDL_ATMR_OCIDLESTATE_LOW;
}

/**
  * @brief  Configure the TMRx output channel.
  * @param  TMRx Timer Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_ATMR_CHANNEL_CH0
  *         @arg @ref DDL_ATMR_CHANNEL_CH1
  *         @arg @ref DDL_ATMR_CHANNEL_CH2
  *         @arg @ref DDL_ATMR_CHANNEL_CH3
  * @param  TMR_OC_InitStruct pointer to a @ref DDL_ATMR_OC_InitTypeDef structure (TMRx output channel configuration
  *         data structure)
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: TMRx output channel is initialized
  *          - ERROR: TMRx output channel is not initialized
  */
ErrorStatus DDL_ATMR_OC_Init(ATMR_TypeDef *TMRx, uint32_t Channel, DDL_ATMR_OC_InitTypeDef *TMR_OC_InitStruct)
{
  ErrorStatus result = ERROR;

  switch (Channel)
  {
    case DDL_ATMR_CHANNEL_CH0:
      result = OC0Config(TMRx, TMR_OC_InitStruct);
      break;
    case DDL_ATMR_CHANNEL_CH1:
      result = OC1Config(TMRx, TMR_OC_InitStruct);
      break;
    case DDL_ATMR_CHANNEL_CH2:
      result = OC2Config(TMRx, TMR_OC_InitStruct);
      break;
    case DDL_ATMR_CHANNEL_CH3:
      result = OC3Config(TMRx, TMR_OC_InitStruct);
      break;
    default:
      break;
  }

  return result;
}

/**
  * @brief  Set the fields of the Break and Dead Time configuration data structure
  *         to their default values.
  * @param  TMR_BDTInitStruct pointer to a @ref DDL_ATMR_BDT_InitTypeDef structure (Break and Dead Time configuration
  *         data structure)
  * @retval None
  */
void DDL_ATMR_BDT_StructInit(DDL_ATMR_BDT_InitTypeDef *TMR_BDTInitStruct)
{
  /* Set the default configuration */
  TMR_BDTInitStruct->OSSRState       = DDL_ATMR_OSSR_DISABLE;
  TMR_BDTInitStruct->OSSIState       = DDL_ATMR_OSSI_DISABLE;
  TMR_BDTInitStruct->LockLevel       = DDL_ATMR_LOCKLEVEL_OFF;
  TMR_BDTInitStruct->DeadTime0        = (uint8_t)0x00;
  TMR_BDTInitStruct->DeadTime1        = (uint8_t)0x00;
  TMR_BDTInitStruct->BreakState      = DDL_ATMR_BREAK_DISABLE;
  TMR_BDTInitStruct->BreakPolarity   = DDL_ATMR_BREAK_POLARITY_LOW;
  TMR_BDTInitStruct->AutomaticOutput = DDL_ATMR_AUTOMATICOUTPUT_DISABLE;
}

/**
  * @brief  Configure the Break and Dead Time feature of the timer instance.
  * @note As the bits AOE, BKP, BKE, OSSR, OSSI and DTG[7:0] can be write-locked
  *  depending on the LOCK configuration, it can be necessary to configure all of
  *  them during the first write access to the TMRx_BDTR register.
  * @note Macro IS_TMR_BREAK_INSTANCE(TMRx) can be used to check whether or not
  *       a timer instance provides a break input.
  * @param  TMRx Timer Instance(ATMR)
  * @param  TMR_BDTInitStruct pointer to a @ref DDL_ATMR_BDT_InitTypeDef structure (Break and Dead Time configuration
  *         data structure)
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: Break and Dead Time is initialized
  *          - ERROR: not applicable
  */
ErrorStatus DDL_ATMR_BDT_Init(ATMR_TypeDef *TMRx, DDL_ATMR_BDT_InitTypeDef *TMR_BDTInitStruct)
{
  uint32_t tmpbdtr = 0;

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_BREAK_INSTANCE(TMRx));
  ASSERT_PARAM(IS_DDL_ATMR_OSSR_STATE(TMR_BDTInitStruct->OSSRState));
  ASSERT_PARAM(IS_DDL_ATMR_OSSI_STATE(TMR_BDTInitStruct->OSSIState));
  ASSERT_PARAM(IS_DDL_ATMR_LOCK_LEVEL(TMR_BDTInitStruct->LockLevel));
  ASSERT_PARAM(IS_DDL_ATMR_BREAK_STATE(TMR_BDTInitStruct->BreakState));
  ASSERT_PARAM(IS_DDL_ATMR_BREAK_POLARITY(TMR_BDTInitStruct->BreakPolarity));
  ASSERT_PARAM(IS_DDL_ATMR_AUTOMATIC_OUTPUT_STATE(TMR_BDTInitStruct->AutomaticOutput));

  /* Set the Lock level, the Break enable Bit and the Polarity, the OSSR State,
  the OSSI State, the dead time value and the Automatic Output Enable Bit */

  /* Set the BDT bits */
  MODIFY_REG(tmpbdtr, ATMR_BDT_DTS0, TMR_BDTInitStruct->DeadTime0);
  MODIFY_REG(tmpbdtr, ATMR_BDT_DTS1, TMR_BDTInitStruct->DeadTime1 << 16);
  MODIFY_REG(tmpbdtr, ATMR_BDT_LOCKCFG, TMR_BDTInitStruct->LockLevel);
  MODIFY_REG(tmpbdtr, ATMR_BDT_IMOS, TMR_BDTInitStruct->OSSIState);
  MODIFY_REG(tmpbdtr, ATMR_BDT_RMOS, TMR_BDTInitStruct->OSSRState);
  MODIFY_REG(tmpbdtr, ATMR_BDT_BRKEN, TMR_BDTInitStruct->BreakState);
  MODIFY_REG(tmpbdtr, ATMR_BDT_BRKPOL, TMR_BDTInitStruct->BreakPolarity);
  MODIFY_REG(tmpbdtr, ATMR_BDT_AOEN, TMR_BDTInitStruct->AutomaticOutput);

  /* Set TMRx_BDT */
  DDL_ATMR_WriteReg(TMRx, BDT, tmpbdtr);

  return SUCCESS;
}
/**
  * @}
  */

/**
  * @}
  */

/** @addtogroup ATMR_DDL_Private_Functions ATMR Private Functions
  *  @brief   Private functions
  * @{
  */
/**
  * @brief  Configure the TMRx output channel 0.
  * @param  TMRx Timer Instance
  * @param  TMR_OCInitStruct pointer to the the TMRx output channel 0 configuration data structure
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: TMRx registers are de-initialized
  *          - ERROR: not applicable
  */
static ErrorStatus OC0Config(ATMR_TypeDef *TMRx, DDL_ATMR_OC_InitTypeDef *TMR_OCInitStruct)
{
  uint32_t tmpccmr1;
  uint32_t tmpccer;
  uint32_t tmpcr2;

  /* Check the parameters */
  ASSERT_PARAM(IS_ATMR_CC0_INSTANCE(TMRx));
  ASSERT_PARAM(IS_DDL_ATMR_OCMODE(TMR_OCInitStruct->OCMode));
  ASSERT_PARAM(IS_DDL_ATMR_OCSTATE(TMR_OCInitStruct->OCState));
  ASSERT_PARAM(IS_DDL_ATMR_OCPOLARITY(TMR_OCInitStruct->OCPolarity));
  ASSERT_PARAM(IS_DDL_ATMR_OCSTATE(TMR_OCInitStruct->OCNState));
  ASSERT_PARAM(IS_DDL_ATMR_OCPOLARITY(TMR_OCInitStruct->OCNPolarity));

  /* Disable the Channel 0: Reset the CC0E Bit */
  CLEAR_BIT(TMRx->CCEN, ATMR_CCEN_CC0EN);

  /* Get the TMRx CCEN register value */
  tmpccer = DDL_ATMR_ReadReg(TMRx, CCEN);

  /* Get the TMRx CTRL2 register value */
  tmpcr2 = DDL_ATMR_ReadReg(TMRx, CR2);

  /* Get the TMRx CCM1 register value */
  tmpccmr1 = DDL_ATMR_ReadReg(TMRx, CCM1);

  /* Set the Output Compare Mode */
  MODIFY_REG(tmpccmr1, ATMR_CCM1_OC0MOD, TMR_OCInitStruct->OCMode << (ATMR_CCM1_OC0MOD_Pos-ATMR_CCM1_OC0MOD_Pos));

  /* Set the Output Compare Polarity */
  MODIFY_REG(tmpccer, ATMR_CCEN_CC0POL, TMR_OCInitStruct->OCPolarity << (ATMR_CCEN_CC0POL_Pos-ATMR_CCEN_CC0POL_Pos));

  /* Set the Output State */
  MODIFY_REG(tmpccer, ATMR_CCEN_CC0EN, TMR_OCInitStruct->OCState << (ATMR_CCEN_CC0EN_Pos-ATMR_CCEN_CC0EN_Pos));

  if (IS_TMR_BREAK_INSTANCE(TMRx))
  {
    ASSERT_PARAM(IS_DDL_ATMR_OCIDLESTATE(TMR_OCInitStruct->OCNIdleState));
    ASSERT_PARAM(IS_DDL_ATMR_OCIDLESTATE(TMR_OCInitStruct->OCIdleState));

    /* Set the complementary output Polarity */
    MODIFY_REG(tmpccer, ATMR_CCEN_CC0NPOL, TMR_OCInitStruct->OCNPolarity << (ATMR_CCEN_CC0NPOL_Pos-ATMR_CCEN_CC0POL_Pos));

    /* Set the complementary output State */
    MODIFY_REG(tmpccer, ATMR_CCEN_CC0NEN, TMR_OCInitStruct->OCNState << (ATMR_CCEN_CC0NEN_Pos-ATMR_CCEN_CC0EN_Pos));

    /* Set the Output Idle state */
    MODIFY_REG(tmpcr2, ATMR_CR2_OC0OIS, TMR_OCInitStruct->OCIdleState << (ATMR_CR2_OC0OIS_Pos-ATMR_CR2_OC0OIS_Pos));

    /* Set the complementary output Idle state */
    MODIFY_REG(tmpcr2, ATMR_CR2_OC0NOIS, TMR_OCInitStruct->OCNIdleState << (ATMR_CR2_OC0NOIS_Pos-ATMR_CR2_OC0OIS_Pos));
  }

  /* Write to TMRx CTRL2 */
  DDL_ATMR_WriteReg(TMRx, CR2, tmpcr2);

  /* Write to TMRx CCM1 */
  DDL_ATMR_WriteReg(TMRx, CCM1, tmpccmr1);

  /* Set the Capture Compare Register value */
  DDL_ATMR_OC_SetCompareCH0(TMRx, TMR_OCInitStruct->CompareValue);

  /* Write to TMRx CCEN */
  DDL_ATMR_WriteReg(TMRx, CCEN, tmpccer);

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
static ErrorStatus OC1Config(ATMR_TypeDef *TMRx, DDL_ATMR_OC_InitTypeDef *TMR_OCInitStruct)
{
  uint32_t tmpccmr1;
  uint32_t tmpccer;
  uint32_t tmpcr2;

  /* Check the parameters */
  ASSERT_PARAM(IS_ATMR_CC1_INSTANCE(TMRx));
  ASSERT_PARAM(IS_DDL_ATMR_OCMODE(TMR_OCInitStruct->OCMode));
  ASSERT_PARAM(IS_DDL_ATMR_OCSTATE(TMR_OCInitStruct->OCState));
  ASSERT_PARAM(IS_DDL_ATMR_OCPOLARITY(TMR_OCInitStruct->OCPolarity));
  ASSERT_PARAM(IS_DDL_ATMR_OCSTATE(TMR_OCInitStruct->OCNState));
  ASSERT_PARAM(IS_DDL_ATMR_OCPOLARITY(TMR_OCInitStruct->OCNPolarity));

  /* Disable the Channel 1: Reset the CC1E Bit */
  CLEAR_BIT(TMRx->CCEN, ATMR_CCEN_CC1EN);

  /* Get the TMRx CCEN register value */
  tmpccer =  DDL_ATMR_ReadReg(TMRx, CCEN);

  /* Get the TMRx CTRL2 register value */
  tmpcr2 = DDL_ATMR_ReadReg(TMRx, CR2);

  /* Get the TMRx CCM1 register value */
  tmpccmr1 = DDL_ATMR_ReadReg(TMRx, CCM1);

  /* Select the Output Compare Mode */
  MODIFY_REG(tmpccmr1, ATMR_CCM1_OC1MOD, TMR_OCInitStruct->OCMode << (ATMR_CCM1_OC1MOD_Pos-ATMR_CCM1_OC0MOD_Pos));

  /* Set the Output Compare Polarity */
  MODIFY_REG(tmpccer, ATMR_CCEN_CC1POL, TMR_OCInitStruct->OCPolarity << (ATMR_CCEN_CC1POL_Pos-ATMR_CCEN_CC0POL_Pos));

  /* Set the Output State */
  MODIFY_REG(tmpccer, ATMR_CCEN_CC1EN, TMR_OCInitStruct->OCState << (ATMR_CCEN_CC1EN_Pos-ATMR_CCEN_CC0EN_Pos));

  if (IS_TMR_BREAK_INSTANCE(TMRx))
  {
    ASSERT_PARAM(IS_DDL_ATMR_OCIDLESTATE(TMR_OCInitStruct->OCNIdleState));
    ASSERT_PARAM(IS_DDL_ATMR_OCIDLESTATE(TMR_OCInitStruct->OCIdleState));

    /* Set the complementary output Polarity */
    MODIFY_REG(tmpccer, ATMR_CCEN_CC1NPOL, TMR_OCInitStruct->OCNPolarity << (ATMR_CCEN_CC1NPOL_Pos-ATMR_CCEN_CC0POL_Pos));

    /* Set the complementary output State */
    MODIFY_REG(tmpccer, ATMR_CCEN_CC1NEN, TMR_OCInitStruct->OCNState << (ATMR_CCEN_CC1NEN_Pos-ATMR_CCEN_CC0EN_Pos));

    /* Set the Output Idle state */
    MODIFY_REG(tmpcr2, ATMR_CR2_OC1OIS, TMR_OCInitStruct->OCIdleState << (ATMR_CR2_OC1OIS_Pos-ATMR_CR2_OC0OIS_Pos));

    /* Set the complementary output Idle state */
    MODIFY_REG(tmpcr2, ATMR_CR2_OC1NOIS, TMR_OCInitStruct->OCNIdleState << (ATMR_CR2_OC1NOIS_Pos-ATMR_CR2_OC0OIS_Pos));
  }

  /* Write to TMRx CTRL2 */
  DDL_ATMR_WriteReg(TMRx, CR2, tmpcr2);

  /* Write to TMRx CCM1 */
  DDL_ATMR_WriteReg(TMRx, CCM1, tmpccmr1);

  /* Set the Capture Compare Register value */
  DDL_ATMR_OC_SetCompareCH1(TMRx, TMR_OCInitStruct->CompareValue);

  /* Write to TMRx CCEN */
  DDL_ATMR_WriteReg(TMRx, CCEN, tmpccer);

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
static ErrorStatus OC2Config(ATMR_TypeDef *TMRx, DDL_ATMR_OC_InitTypeDef *TMR_OCInitStruct)
{
  uint32_t tmpccmr2;
  uint32_t tmpccer;
  uint32_t tmpcr2;

  /* Check the parameters */
  ASSERT_PARAM(IS_ATMR_CC2_INSTANCE(TMRx));
  ASSERT_PARAM(IS_DDL_ATMR_OCMODE(TMR_OCInitStruct->OCMode));
  ASSERT_PARAM(IS_DDL_ATMR_OCSTATE(TMR_OCInitStruct->OCState));
  ASSERT_PARAM(IS_DDL_ATMR_OCPOLARITY(TMR_OCInitStruct->OCPolarity));
  ASSERT_PARAM(IS_DDL_ATMR_OCSTATE(TMR_OCInitStruct->OCNState));
  ASSERT_PARAM(IS_DDL_ATMR_OCPOLARITY(TMR_OCInitStruct->OCNPolarity));

  /* Disable the Channel 2: Reset the CC2E Bit */
  CLEAR_BIT(TMRx->CCEN, ATMR_CCEN_CC2EN);

  /* Get the TMRx CCEN register value */
  tmpccer =  DDL_ATMR_ReadReg(TMRx, CCEN);

  /* Get the TMRx CTRL2 register value */
  tmpcr2 = DDL_ATMR_ReadReg(TMRx, CR2);

  /* Get the TMRx CCM2 register value */
  tmpccmr2 = DDL_ATMR_ReadReg(TMRx, CCM2);

  /* Select the Output Compare Mode */
  MODIFY_REG(tmpccmr2, ATMR_CCM2_OC2MOD, TMR_OCInitStruct->OCMode << (ATMR_CCM2_OC2MOD_Pos-ATMR_CCM2_OC2MOD_Pos));

  /* Set the Output Compare Polarity */
  MODIFY_REG(tmpccer, ATMR_CCEN_CC2POL, TMR_OCInitStruct->OCPolarity << (ATMR_CCEN_CC2POL_Pos-ATMR_CCEN_CC0POL_Pos));

  /* Set the Output State */
  MODIFY_REG(tmpccer, ATMR_CCEN_CC2EN, TMR_OCInitStruct->OCState << (ATMR_CCEN_CC2EN_Pos-ATMR_CCEN_CC0EN_Pos));

  if (IS_TMR_BREAK_INSTANCE(TMRx))
  {
    ASSERT_PARAM(IS_DDL_ATMR_OCIDLESTATE(TMR_OCInitStruct->OCNIdleState));
    ASSERT_PARAM(IS_DDL_ATMR_OCIDLESTATE(TMR_OCInitStruct->OCIdleState));

    /* Set the complementary output Polarity */
    MODIFY_REG(tmpccer, ATMR_CCEN_CC2NPOL, TMR_OCInitStruct->OCNPolarity << (ATMR_CCEN_CC2NPOL_Pos-ATMR_CCEN_CC0POL_Pos));

    /* Set the complementary output State */
    MODIFY_REG(tmpccer, ATMR_CCEN_CC2NEN, TMR_OCInitStruct->OCNState << (ATMR_CCEN_CC2NEN_Pos-ATMR_CCEN_CC0EN_Pos));

    /* Set the Output Idle state */
    MODIFY_REG(tmpcr2, ATMR_CR2_OC2OIS, TMR_OCInitStruct->OCIdleState << (ATMR_CR2_OC2OIS_Pos-ATMR_CR2_OC0OIS_Pos));

    /* Set the complementary output Idle state */
    MODIFY_REG(tmpcr2, ATMR_CR2_OC2NOIS, TMR_OCInitStruct->OCNIdleState << (ATMR_CR2_OC2NOIS_Pos-ATMR_CR2_OC0OIS_Pos));
  }

  /* Write to TMRx CTRL2 */
  DDL_ATMR_WriteReg(TMRx, CR2, tmpcr2);

  /* Write to TMRx CCM2 */
  DDL_ATMR_WriteReg(TMRx, CCM2, tmpccmr2);

  /* Set the Capture Compare Register value */
  DDL_ATMR_OC_SetCompareCH2(TMRx, TMR_OCInitStruct->CompareValue);

  /* Write to TMRx CCEN */
  DDL_ATMR_WriteReg(TMRx, CCEN, tmpccer);

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
static ErrorStatus OC3Config(ATMR_TypeDef *TMRx, DDL_ATMR_OC_InitTypeDef *TMR_OCInitStruct)
{
  uint32_t tmpccmr2;
  uint32_t tmpccer;
  uint32_t tmpcr2;

  /* Check the parameters */
  ASSERT_PARAM(IS_ATMR_CC3_INSTANCE(TMRx));
  ASSERT_PARAM(IS_DDL_ATMR_OCMODE(TMR_OCInitStruct->OCMode));
  ASSERT_PARAM(IS_DDL_ATMR_OCSTATE(TMR_OCInitStruct->OCState));
  ASSERT_PARAM(IS_DDL_ATMR_OCPOLARITY(TMR_OCInitStruct->OCPolarity));
  ASSERT_PARAM(IS_DDL_ATMR_OCPOLARITY(TMR_OCInitStruct->OCNPolarity));
  ASSERT_PARAM(IS_DDL_ATMR_OCSTATE(TMR_OCInitStruct->OCNState));

  /* Disable the Channel 3: Reset the CC3E Bit */
  CLEAR_BIT(TMRx->CCEN, ATMR_CCEN_CC3EN);

  /* Get the TMRx CCEN register value */
  tmpccer = DDL_ATMR_ReadReg(TMRx, CCEN);

  /* Get the TMRx CTRL2 register value */
  tmpcr2 =  DDL_ATMR_ReadReg(TMRx, CR2);

  /* Get the TMRx CCM2 register value */
  tmpccmr2 = DDL_ATMR_ReadReg(TMRx, CCM2);

  /* Select the Output Compare Mode */
  MODIFY_REG(tmpccmr2, ATMR_CCM2_OC3MOD, TMR_OCInitStruct->OCMode << (ATMR_CCM2_OC3MOD_Pos-ATMR_CCM2_OC2MOD_Pos));

  /* Set the Output Compare Polarity */
  MODIFY_REG(tmpccer, ATMR_CCEN_CC3POL, TMR_OCInitStruct->OCPolarity << (ATMR_CCEN_CC3POL_Pos-ATMR_CCEN_CC0POL_Pos));

  /* Set the Output State */
  MODIFY_REG(tmpccer, ATMR_CCEN_CC3EN, TMR_OCInitStruct->OCState << (ATMR_CCEN_CC3EN_Pos-ATMR_CCEN_CC0EN_Pos));

  if (IS_TMR_BREAK_INSTANCE(TMRx))
  {
    ASSERT_PARAM(IS_DDL_ATMR_OCIDLESTATE(TMR_OCInitStruct->OCNIdleState));
    ASSERT_PARAM(IS_DDL_ATMR_OCIDLESTATE(TMR_OCInitStruct->OCIdleState));

    /* Set the complementary output Polarity */
    MODIFY_REG(tmpccer, ATMR_CCEN_CC3NPOL, TMR_OCInitStruct->OCNPolarity << (ATMR_CCEN_CC3NPOL_Pos-ATMR_CCEN_CC0POL_Pos));

    /* Set the complementary output State */
    MODIFY_REG(tmpccer, ATMR_CCEN_CC3NEN, TMR_OCInitStruct->OCNState << (ATMR_CCEN_CC3NEN_Pos-ATMR_CCEN_CC0EN_Pos));

    /* Set the Output Idle state */
    MODIFY_REG(tmpcr2, ATMR_CR2_OC3OIS, TMR_OCInitStruct->OCIdleState << (ATMR_CR2_OC3OIS_Pos-ATMR_CR2_OC0OIS_Pos));

    /* Set the complementary output Idle state */
    MODIFY_REG(tmpcr2, ATMR_CR2_OC3NOIS, TMR_OCInitStruct->OCNIdleState << (ATMR_CR2_OC3NOIS_Pos-ATMR_CR2_OC0OIS_Pos));
  }

  /* Write to TMRx CTRL2 */
  DDL_ATMR_WriteReg(TMRx, CR2, tmpcr2);

  /* Write to TMRx CCM2 */
  DDL_ATMR_WriteReg(TMRx, CCM2, tmpccmr2);

  /* Set the Capture Compare Register value */
  DDL_ATMR_OC_SetCompareCH3(TMRx, TMR_OCInitStruct->CompareValue);

  /* Write to TMRx CCEN */
  DDL_ATMR_WriteReg(TMRx, CCEN, tmpccer);

  return SUCCESS;
}

/**
  * @}
  */

/**
  * @}
  */

#endif /* ATMR */

/**
  * @}
  */

#endif /* USE_FULL_DDL_DRIVER */

