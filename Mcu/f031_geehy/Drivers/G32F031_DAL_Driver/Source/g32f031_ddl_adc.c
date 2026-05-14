/**
  *
  * @file    g32f031_ddl_adc.c
  * @brief   ADC DDL module driver
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
  ******************************************************************************
  */

  #if defined(USE_FULL_DDL_DRIVER)
/* Includes ------------------------------------------------------------------*/
#include "g32f031_ddl_adc.h"
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

#if defined (ADC)

/** @defgroup ADC_DDL ADC
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/** @addtogroup ADC_DDL_Private_Macros ADC Private Macros
  * @{
  */

/* Definitions of ADC hardware constraints delays */
/* Note: Only ADC peripheral HW delays are defined in ADC LL driver driver,   */
/*       not timeout values:                                                  */
/*       Timeout values for ADC operations are dependent to device clock      */
/*       configuration (system clock versus ADC clock),                       */
/*       and therefore must be defined in user application.                   */
/*       Refer to @ref ADC_LL_EC_HW_DELAYS for description of ADC timeout     */
/*       values definition.                                                   */
/* Note: ADC timeout values are defined here in CPU cycles to be independent  */
/*       of device clock setting.                                             */
/*       In user application, ADC timeout values should be defined with       */
/*       temporal values, in function of device clock settings.               */
/*       Highest ratio CPU clock frequency vs ADC clock frequency:            */
/*        - ADC clock from synchronous clock with AHB prescaler 512,          */
/*          ADC prescaler 4.                                                  */
/*           Ratio max = 512 *4 = 2048                                        */
/*        - ADC clock from asynchronous clock (PLLP) with prescaler 256.      */
/*          Highest CPU clock PLL (PLLR).                                     */
/*           Ratio max = PLLRmax /PPLPmin * 256 = (VCO/2) / (VCO/31) * 256    */
/*                     = 3968                                                 */
/* Unit: CPU cycles.                                                          */
#define ADC_CLOCK_RATIO_VS_CPU_HIGHEST          (3968UL)
#define ADC_TIMEOUT_DISABLE_CPU_CYCLES          (ADC_CLOCK_RATIO_VS_CPU_HIGHEST * 1UL)
#define ADC_TIMEOUT_STOP_CONVERSION_CPU_CYCLES  (ADC_CLOCK_RATIO_VS_CPU_HIGHEST * 1UL)

#define IS_DDL_ADC_REG_TRIG_EDGE(__REG_TRIG_EDGE__)                             \
  (((__REG_TRIG_EDGE__) == DDL_ADC_REG_TRIG_EXTEDGE_RISING)                     \
   || ((__REG_TRIG_EDGE__) == DDL_ADC_REG_TRIG_EXTEDGE_FALLING)                 \
   || ((__REG_TRIG_EDGE__) == DDL_ADC_REG_TRIG_EXTEDGE_RISING_FALLING))

#define IS_DDL_ADC_REG_TRIG_SOURCE(__ADC_INSTANCE__, __REG_TRIG_SOURCE__)         \
  (((__REG_TRIG_SOURCE__) == DDL_ADC_REG_TRIG_SOFTWARE)                           \
   || ((__REG_TRIG_SOURCE__) == DDL_ADC_REG_TRIG_EXTSEL_ATMR_TRGO0)                 \
   || ((__REG_TRIG_SOURCE__) == DDL_ADC_REG_TRIG_EXTSEL_ATMR_TRGO1)                 \
   || ((__REG_TRIG_SOURCE__) == DDL_ADC_REG_TRIG_EXTSEL_ATMR_TRGO2)                 \
   || ((__REG_TRIG_SOURCE__) == DDL_ADC_REG_TRIG_EXTSEL_GTMR_TRGO))

#define IS_DDL_ADC_REG_SEQ_SCAN_LENGTH(__REG_SEQ_SCAN_LENGTH__)                 \
  (((__REG_SEQ_SCAN_LENGTH__) == DDL_ADC_REG_SEQ_SCAN_DISABLE)                  \
   || ((__REG_SEQ_SCAN_LENGTH__) == DDL_ADC_REG_SEQ_SCAN_ENABLE_2RANKS)         \
   || ((__REG_SEQ_SCAN_LENGTH__) == DDL_ADC_REG_SEQ_SCAN_ENABLE_3RANKS)         \
   || ((__REG_SEQ_SCAN_LENGTH__) == DDL_ADC_REG_SEQ_SCAN_ENABLE_4RANKS)         \
   || ((__REG_SEQ_SCAN_LENGTH__) == DDL_ADC_REG_SEQ_SCAN_ENABLE_5RANKS)         \
   || ((__REG_SEQ_SCAN_LENGTH__) == DDL_ADC_REG_SEQ_SCAN_ENABLE_6RANKS)         \
   || ((__REG_SEQ_SCAN_LENGTH__) == DDL_ADC_REG_SEQ_SCAN_ENABLE_7RANKS)         \
   || ((__REG_SEQ_SCAN_LENGTH__) == DDL_ADC_REG_SEQ_SCAN_ENABLE_8RANKS)         \
   || ((__REG_SEQ_SCAN_LENGTH__) == DDL_ADC_REG_SEQ_SCAN_ENABLE_9RANKS)         \
   || ((__REG_SEQ_SCAN_LENGTH__) == DDL_ADC_REG_SEQ_SCAN_ENABLE_10RANKS)        \
   || ((__REG_SEQ_SCAN_LENGTH__) == DDL_ADC_REG_SEQ_SCAN_ENABLE_11RANKS)        \
   || ((__REG_SEQ_SCAN_LENGTH__) == DDL_ADC_REG_SEQ_SCAN_ENABLE_12RANKS)        \
   || ((__REG_SEQ_SCAN_LENGTH__) == DDL_ADC_REG_SEQ_SCAN_ENABLE_13RANKS)        \
   || ((__REG_SEQ_SCAN_LENGTH__) == DDL_ADC_REG_SEQ_SCAN_ENABLE_14RANKS)        \
   || ((__REG_SEQ_SCAN_LENGTH__) == DDL_ADC_REG_SEQ_SCAN_ENABLE_15RANKS)        \
   || ((__REG_SEQ_SCAN_LENGTH__) == DDL_ADC_REG_SEQ_SCAN_ENABLE_16RANKS)        \
  )

#define IS_DDL_ADC_REG_SEQ_SCAN_DISCONT_MODE(__REG_SEQ_DISCONT_MODE__)          \
  (((__REG_SEQ_DISCONT_MODE__) == DDL_ADC_REG_SEQ_DISCONT_DISABLE)              \
   || ((__REG_SEQ_DISCONT_MODE__) == DDL_ADC_REG_SEQ_DISCONT_1RANK)             \
   || ((__REG_SEQ_DISCONT_MODE__) == DDL_ADC_REG_SEQ_DISCONT_2RANKS)            \
   || ((__REG_SEQ_DISCONT_MODE__) == DDL_ADC_REG_SEQ_DISCONT_3RANKS)            \
   || ((__REG_SEQ_DISCONT_MODE__) == DDL_ADC_REG_SEQ_DISCONT_4RANKS)            \
   || ((__REG_SEQ_DISCONT_MODE__) == DDL_ADC_REG_SEQ_DISCONT_5RANKS)            \
   || ((__REG_SEQ_DISCONT_MODE__) == DDL_ADC_REG_SEQ_DISCONT_6RANKS)            \
   || ((__REG_SEQ_DISCONT_MODE__) == DDL_ADC_REG_SEQ_DISCONT_7RANKS)            \
   || ((__REG_SEQ_DISCONT_MODE__) == DDL_ADC_REG_SEQ_DISCONT_8RANKS)            \
  )

#define IS_DDL_ADC_REG_CONTINUOUS_MODE(__REG_CONTINUOUS_MODE__)                 \
  (((__REG_CONTINUOUS_MODE__) == DDL_ADC_REG_CONV_SINGLE)                       \
   || ((__REG_CONTINUOUS_MODE__) == DDL_ADC_REG_CONV_CONTINUOUS)                \
  )

#define IS_DDL_ADC_REG_DMA_TRANSFER(__REG_DMA_TRANSFER__)                       \
  (((__REG_DMA_TRANSFER__) == DDL_ADC_REG_DMA_TRANSFER_DISABLE)                 \
   || ((__REG_DMA_TRANSFER__) == DDL_ADC_REG_DMA_TRANSFER_ONESHOT_MODE)         \
   || ((__REG_DMA_TRANSFER__) == DDL_ADC_REG_DMA_TRANSFER_CIRCULAR_MODE)        \
  )

#define IS_DDL_ADC_REG_OVR_DATA_BEHAVIOR(__OVR_DATA_BEHAVIOR__)       \
  (((__OVR_DATA_BEHAVIOR__) == DDL_ADC_OVERMODE_KEEP)                 \
   || ((__OVR_DATA_BEHAVIOR__) == DDL_ADC_OVERMODE_OVER)              \
  )

#define IS_DDL_ADC_SEQ_SECTIONNUMBER(__SEQ_NUM__)          \
  (((__SEQ_NUM__) == DDL_ADC_SEQ_NUMBER_1)                 \
   || ((__SEQ_NUM__) == DDL_ADC_SEQ_NUMBER_2)              \
   || ((__SEQ_NUM__) == DDL_ADC_SEQ_NUMBER_3)              \
  )

#define IS_DDL_ADC_SEQ_TRS_TIME(__SEQ_TRS_TIME__)          \
  (((__SEQ_TRS_TIME__) == DDL_ADC_SEQ_TRS_TIME_1)                 \
   || ((__SEQ_TRS_TIME__) == DDL_ADC_SEQ_TRS_TIME_2)              \
   || ((__SEQ_TRS_TIME__) == DDL_ADC_SEQ_TRS_TIME_3)              \
   || ((__SEQ_TRS_TIME__) == DDL_ADC_SEQ_TRS_TIME_4)              \
   || ((__SEQ_TRS_TIME__) == DDL_ADC_SEQ_TRS_TIME_5)              \
   || ((__SEQ_TRS_TIME__) == DDL_ADC_SEQ_TRS_TIME_6)              \
   || ((__SEQ_TRS_TIME__) == DDL_ADC_SEQ_TRS_TIME_7)              \
   || ((__SEQ_TRS_TIME__) == DDL_ADC_SEQ_TRS_TIME_8)              \
  )

#define IS_DDL_ADC_SEQ_GAPTIME(__SEQ_GAPTIME__)                               \
  (((__SEQ_GAPTIME__) == DDL_ADC_GAPTIME_ADCCLK_0)                            \
   || ((__SEQ_GAPTIME__) == DDL_ADC_GAPTIME_ADCCLK_2)                         \
   || ((__SEQ_GAPTIME__) == DDL_ADC_GAPTIME_ADCCLK_4)                         \
   || ((__SEQ_GAPTIME__) == DDL_ADC_GAPTIME_ADCCLK_8)                         \
   || ((__SEQ_GAPTIME__) == DDL_ADC_GAPTIME_ADCCLK_16)                        \
   || ((__SEQ_GAPTIME__) == DDL_ADC_GAPTIME_ADCCLK_32)                        \
   || ((__SEQ_GAPTIME__) == DDL_ADC_GAPTIME_ADCCLK_64)                        \
   || ((__SEQ_GAPTIME__) == DDL_ADC_GAPTIME_ADCCLK_128)                       \
   || ((__SEQ_GAPTIME__) == DDL_ADC_GAPTIME_ADCCLK_256)                       \
   || ((__SEQ_GAPTIME__) == DDL_ADC_GAPTIME_ADCCLK_512)                       \
   || ((__SEQ_GAPTIME__) == DDL_ADC_GAPTIME_ADCCLK_1024)                      \
   || ((__SEQ_GAPTIME__) == DDL_ADC_GAPTIME_ADCCLK_2048)                      \
   || ((__SEQ_GAPTIME__) == DDL_ADC_GAPTIME_ADCCLK_4096)                      \
   || ((__SEQ_GAPTIME__) == DDL_ADC_GAPTIME_ADCCLK_8192)                      \
   || ((__SEQ_GAPTIME__) == DDL_ADC_GAPTIME_ADCCLK_16384)                     \
   || ((__SEQ_GAPTIME__) == DDL_ADC_GAPTIME_ADCCLK_32768)                     \
  )

#define IS_DDL_ADC_SEQ_TRIG_SOURCE(__ADC_INSTANCE__, __SEQ_TRIG_SOURCE__)      \
  (((__SEQ_TRIG_SOURCE__) == DDL_ADC_SEQ_TRIG_EXTSEL_ATMR_TRGO0)                 \
   || ((__SEQ_TRIG_SOURCE__) == DDL_ADC_SEQ_TRIG_EXTSEL_ATMR_TRGO1)              \
   || ((__SEQ_TRIG_SOURCE__) == DDL_ADC_SEQ_TRIG_EXTSEL_ATMR_TRGO2)              \
   || ((__SEQ_TRIG_SOURCE__) == DDL_ADC_SEQ_TRIG_EXTSEL_GTMR_TRGO))

#define IS_DDL_ADC_SEQ_TRIG_EDGE(__SEQ_TRIG_EDGE__)                             \
  (((__SEQ_TRIG_EDGE__) == DDL_ADC_SEQ_TRIG_EXTEDGE_RISING)                     \
   || ((__SEQ_TRIG_EDGE__) == DDL_ADC_SEQ_TRIG_EXTEDGE_FALLING)                 \
   || ((__SEQ_TRIG_EDGE__) == DDL_ADC_SEQ_TRIG_EXTEDGE_RISING_FALLING))


#define IS_DDL_ADC_CHANNEL(__CHANNEL__)               \
  (((__CHANNEL__) == DDL_ADC_CHANNEL_0)               \
  || ((__CHANNEL__) == DDL_ADC_CHANNEL_1)             \
  || ((__CHANNEL__) == DDL_ADC_CHANNEL_2)             \
  || ((__CHANNEL__) == DDL_ADC_CHANNEL_3)             \
  || ((__CHANNEL__) == DDL_ADC_CHANNEL_4)             \
  || ((__CHANNEL__) == DDL_ADC_CHANNEL_5)             \
  || ((__CHANNEL__) == DDL_ADC_CHANNEL_6)             \
  || ((__CHANNEL__) == DDL_ADC_CHANNEL_7)             \
  || ((__CHANNEL__) == DDL_ADC_CHANNEL_8)             \
  || ((__CHANNEL__) == DDL_ADC_CHANNEL_9)             \
  || ((__CHANNEL__) == DDL_ADC_CHANNEL_10)            \
  || ((__CHANNEL__) == DDL_ADC_CHANNEL_11)            \
  || ((__CHANNEL__) == DDL_ADC_CHANNEL_12)            \
  )

  /**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/** @addtogroup ADC_DDL_Exported_Functions ADC Exported Functions
  * @{
  */

/**
  * @brief  De-initialize registers of the selected ADC instance
  *         to their default reset values (perform a hard reset).
  * @param  ADCx ADC instance
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: ADC registers are de-initialized
  *          - ERROR: ADC registers are not de-initialized
  */
ErrorStatus DDL_ADC_DeInit(ADC_TypeDef *ADCx)
{
  ErrorStatus status = SUCCESS;

  /* Check the parameters */
  ASSERT_PARAM(IS_ADC_ALL_INSTANCE(ADCx));

  if (ADCx == ADC)
  {
    DDL_RCC_Unlock();

    DDL_APB_GRP1_ForceReset(DDL_APB_GRP1_PERIPH_ADC);
    DDL_APB_GRP1_ReleaseReset(DDL_APB_GRP1_PERIPH_ADC);

    DDL_RCC_Lock();
  }
  else
  {
    status = ERROR;
  }

  return (status);
}

/**
  * @brief  Initialize some features of ADC instance.
  * @note   These parameters have an impact on ADC scope: ADC instance.
  *         Affects both group regular and group injected (availability
  *         of ADC group injected depends on G32 families).
  *         Refer to corresponding unitary functions into
  *         @ref ADC_DDL_EF_Configuration_ADC_Instance .
  * @note   The setting of these parameters by function @ref DDL_ADC_Init()
  *         is conditioned to ADC state:
  *         ADC instance must be disabled.
  *         This condition is applied to all ADC features, for efficiency
  *         and compatibility over all G32 families. However, the different
  *         features can be set under different ADC state conditions
  *         (setting possible with ADC enabled without conversion on going,
  *         ADC enabled with conversion on going, ...)
  *         Each feature can be updated afterwards with a unitary function
  *         and potentially with ADC in a different state than disabled,
  *         refer to description of each function for setting
  *         conditioned to ADC state.
  * @param  ADCx ADC instance
  * @param  ADC_InitStruct Pointer to a @ref DDL_ADC_InitTypeDef structure
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: ADC registers are initialized
  *          - ERROR: ADC registers are not initialized
  */
ErrorStatus DDL_ADC_Init(ADC_TypeDef *ADCx, DDL_ADC_InitTypeDef *ADC_InitStruct)
{
  ErrorStatus status = SUCCESS;

  /* Check the parameters */
  ASSERT_PARAM(IS_ADC_ALL_INSTANCE(ADCx));

  /* Note:
     For the control bits related to regular conversion configuration, software is allowed to write to these bits only
     when the ADC is enabled (ADEN=1) and no regular conversion is ongoing (ADSTART must be 0).       */
  if ((DDL_ADC_IsEnabled(ADCx) == 1UL) && (DDL_ADC_IsConversionOngoing(ADCx) == 0U))
  {
    /* Configuration of ADC hierarchical scope:                               */
    /*  - ADC instance                                                        */
    /*  - Set ADC Data Alignment                                              */
    DDL_ADC_SetDataAlignment(ADCx, ADC_InitStruct->DataAlignment);

  }
  else
  {
    /* Initialization error: ADC instance is not disabled. */
    status = ERROR;
  }

  return status;
}

/**
  * @brief  Set each @ref DDL_ADC_InitTypeDef field to default value.
  * @param  ADC_InitStruct Pointer to a @ref DDL_ADC_InitTypeDef structure
  *                        whose fields will be set to default values.
  * @retval None
  */
void DDL_ADC_StructInit(DDL_ADC_InitTypeDef *ADC_InitStruct)
{
  /* Set ADC_InitStruct fields to default values */
  /* Set fields of ADC instance */
  ADC_InitStruct->DataAlignment = DDL_ADC_ALIGNMENT_RIGHT;
}

/**
  * @brief  Initialize some features of ADC group regular.
  * @note   These parameters have an impact on ADC scope: ADC group regular.
  *         Refer to corresponding unitary functions into
  *         @ref ADC_DDL_EF_Configuration_ADC_Group_Regular
  *         (functions with prefix "REG").
  * @note   The setting of these parameters by function @ref DDL_ADC_Init()
  *         is conditioned to ADC state:
  *         ADC instance must be disabled.
  *         This condition is applied to all ADC features, for efficiency
  *         and compatibility over all G32 families. However, the different
  *         features can be set under different ADC state conditions
  *         (setting possible with ADC enabled without conversion on going,
  *         ADC enabled with conversion on going, ...)
  *         Each feature can be updated afterwards with a unitary function
  *         and potentially with ADC in a different state than disabled,
  *         refer to description of each function for setting
  *         conditioned to ADC state.
  * @param  ADCx ADC instance
  * @param  ADC_REG_InitStruct Pointer to a @ref DDL_ADC_REG_InitTypeDef structure
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: ADC registers are initialized
  *          - ERROR: ADC registers are not initialized
  */
ErrorStatus DDL_ADC_REG_Init(ADC_TypeDef *ADCx, DDL_ADC_REG_InitTypeDef *ADC_REG_InitStruct)
{
  ErrorStatus status = SUCCESS;

  /* Check the parameters */
  ASSERT_PARAM(IS_ADC_ALL_INSTANCE(ADCx));
  ASSERT_PARAM(IS_DDL_ADC_REG_TRIG_SOURCE(ADCx, ADC_REG_InitStruct->TriggerSource));
  ASSERT_PARAM(IS_DDL_ADC_REG_TRIG_EDGE(ADC_REG_InitStruct->TriggerEdge));
  ASSERT_PARAM(IS_DDL_ADC_REG_SEQ_SCAN_LENGTH(ADC_REG_InitStruct->SequencerLength));
  if (ADC_REG_InitStruct->SequencerLength != DDL_ADC_REG_SEQ_SCAN_DISABLE)
  {
    ASSERT_PARAM(IS_DDL_ADC_REG_SEQ_SCAN_DISCONT_MODE(ADC_REG_InitStruct->SequencerDiscont));
  }
  ASSERT_PARAM(IS_DDL_ADC_REG_CONTINUOUS_MODE(ADC_REG_InitStruct->ContinuousMode));
  ASSERT_PARAM(IS_DDL_ADC_REG_DMA_TRANSFER(ADC_REG_InitStruct->DMATransfer));
  ASSERT_PARAM(IS_DDL_ADC_REG_OVR_DATA_BEHAVIOR(ADC_REG_InitStruct->Overrun));

  /* ADC group regular continuous mode and discontinuous mode                 */
  /* can not be enabled simultenaeously                                       */
  ASSERT_PARAM((ADC_REG_InitStruct->ContinuousMode == DDL_ADC_REG_CONV_SINGLE)
               || (ADC_REG_InitStruct->SequencerDiscont == DDL_ADC_REG_SEQ_DISCONT_DISABLE));

  /* Note:
     For the control bits related to regular conversion configuration, software is allowed to write to these bits only
     when the ADC is enabled (ADEN=1) and no regular conversion is ongoing (ADSTART must be 0).       */
  if ((DDL_ADC_IsEnabled(ADCx) == 1UL) && (DDL_ADC_IsConversionOngoing(ADCx) == 0U))
  {
    /* Configuration of ADC hierarchical scope:                               */
    /*  - ADC group regular                                                   */
    /*    - Set ADC group regular trigger source                              */
    /*    - Set ADC group regular sequencer length                            */
    /*    - Set ADC group regular sequencer discontinuous mode                */
    /*    - Set ADC group regular continuous mode                             */
    /*    - Set ADC group regular conversion data transfer: no transfer or    */
    /*      transfer by DMA, and DMA requests mode                            */
    /*    - Set ADC group regular overrun behavior                            */
    /* Note: On this G32 series, ADC trigger edge is set to value 0x0 by    */
    /*       setting of trigger source to SW start.                           */
    if (ADC_REG_InitStruct->SequencerLength != DDL_ADC_REG_SEQ_SCAN_DISABLE)
    {
      DDL_ADC_REG_SetTriggerSource(ADCx, ADC_REG_InitStruct->TriggerSource);
      DDL_ADC_REG_SetSequencerDiscont(ADCx, ADC_REG_InitStruct->SequencerDiscont);
      DDL_ADC_REG_SetContinuousMode(ADCx, ADC_REG_InitStruct->ContinuousMode);
      DDL_ADC_REG_SetDMATransfer(ADCx, ADC_REG_InitStruct->DMATransfer);
      DDL_ADC_SetOverflowMode(ADCx, ADC_REG_InitStruct->Overrun);
    }
    else
    {
      DDL_ADC_REG_SetTriggerSource(ADCx, ADC_REG_InitStruct->TriggerSource);
      DDL_ADC_REG_SetSequencerDiscont(ADCx, DDL_ADC_REG_SEQ_DISCONT_DISABLE);
      DDL_ADC_REG_SetContinuousMode(ADCx, ADC_REG_InitStruct->ContinuousMode);
      DDL_ADC_REG_SetDMATransfer(ADCx, ADC_REG_InitStruct->DMATransfer);
      DDL_ADC_SetOverflowMode(ADCx, ADC_REG_InitStruct->Overrun);
    }
    if(ADC_REG_InitStruct->TriggerSource != DDL_ADC_REG_TRIG_SOFTWARE)
    {
      DDL_ADC_REG_SetTriggerEdge(ADCx, ADC_REG_InitStruct->TriggerEdge);
    }
    /* Set ADC group regular sequencer length and scan direction */
    DDL_ADC_REG_SetSequencerLength(ADCx, ADC_REG_InitStruct->SequencerLength);

  }
  else
  {
    /* Initialization error: ADC instance is not disabled. */
    status = ERROR;
  }
  return status;
}

/**
  * @brief  Set each @ref DDL_ADC_REG_InitTypeDef field to default value.
  * @param  ADC_REG_InitStruct Pointer to a @ref DDL_ADC_REG_InitTypeDef structure
  *                            whose fields will be set to default values.
  * @retval None
  */
void DDL_ADC_REG_StructInit(DDL_ADC_REG_InitTypeDef *ADC_REG_InitStruct)
{
  /* Set ADC_REG_InitStruct fields to default values */
  /* Set fields of ADC group regular */
  /* Note: On this G32 series, ADC trigger edge is set to value 0x0 by      */
  /*       setting of trigger source to SW start.                             */
  ADC_REG_InitStruct->TriggerSource    = DDL_ADC_REG_TRIG_SOFTWARE;
  ADC_REG_InitStruct->TriggerEdge      = DDL_ADC_REG_TRIG_EXTEDGE_RISING;
  ADC_REG_InitStruct->SequencerLength  = DDL_ADC_REG_SEQ_SCAN_DISABLE;
  ADC_REG_InitStruct->SequencerDiscont = DDL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct->ContinuousMode   = DDL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct->DMATransfer      = DDL_ADC_REG_DMA_TRANSFER_DISABLE;
  ADC_REG_InitStruct->Overrun          = DDL_ADC_OVERMODE_KEEP;
}

/**
  * @brief  Initialize some features of ADC sequential section.
  * @param  ADCx ADC instance
  * @param  ADC_SEQ_InitStruct Pointer to a @ref DDL_ADC_SEQ_InitTypeDef structure
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: ADC registers are initialized
  *          - ERROR: ADC registers are not initialized
  */
ErrorStatus DDL_ADC_SEQ_Init(ADC_TypeDef *ADCx, DDL_ADC_SEQ_InitTypeDef *ADC_SEQ_InitStruct)
{
  ErrorStatus status = SUCCESS;

  /* Check the parameters */
  ASSERT_PARAM(IS_ADC_ALL_INSTANCE(ADCx));
  ASSERT_PARAM(IS_DDL_ADC_SEQ_SECTIONNUMBER(ADC_SEQ_InitStruct->NumSec));
  ASSERT_PARAM(IS_DDL_ADC_SEQ_TRS_TIME(ADC_SEQ_InitStruct->TimesSec1));
  ASSERT_PARAM(IS_DDL_ADC_SEQ_TRS_TIME(ADC_SEQ_InitStruct->TimesSec2));
  ASSERT_PARAM(IS_DDL_ADC_SEQ_TRS_TIME(ADC_SEQ_InitStruct->TimesSec3));
  ASSERT_PARAM(IS_DDL_ADC_SEQ_GAPTIME(ADC_SEQ_InitStruct->GapTime));
  ASSERT_PARAM(IS_DDL_ADC_SEQ_TRIG_SOURCE(ADCx, ADC_SEQ_InitStruct->TriggerSourceSec1));
  ASSERT_PARAM(IS_DDL_ADC_SEQ_TRIG_EDGE(ADC_SEQ_InitStruct->TriggerEdgeSec1));
  ASSERT_PARAM(IS_DDL_ADC_SEQ_TRIG_SOURCE(ADCx, ADC_SEQ_InitStruct->TriggerSourceSec2));
  ASSERT_PARAM(IS_DDL_ADC_SEQ_TRIG_EDGE(ADC_SEQ_InitStruct->TriggerEdgeSec2));
  ASSERT_PARAM(IS_DDL_ADC_SEQ_TRIG_SOURCE(ADCx, ADC_SEQ_InitStruct->TriggerSourceSec3));
  ASSERT_PARAM(IS_DDL_ADC_SEQ_TRIG_EDGE(ADC_SEQ_InitStruct->TriggerEdgeSec3));

  /* Note:
     For the control bits related to regular conversion configuration, software is allowed to write to these bits only
     when the ADC is enabled (ADEN=1) and no regular conversion is ongoing (ADSTART must be 0).       */
  if ((DDL_ADC_IsEnabled(ADCx) == 1UL) && (DDL_ADC_IsConversionOngoing(ADCx) == 0U))
  {
    /* Configuration of ADC hierarchical scope:                               */
    /*  - ADC sequential section                                                   */
    /*    - Set ADC sequential section sequencer gap time                          */
    /*    - Set ADC sequential section1 trigger source                             */
    /*    - Set ADC sequential section1 trigger edge                               */
    /*    - Set ADC sequential section2 trigger source                             */
    /*    - Set ADC sequential section2 trigger edge                               */
    /*    - Set ADC sequential section3 trigger source                             */
    /*    - Set ADC sequential section3 trigger edge                               */
    /*    - Set ADC sequential section sampling time                               */
    DDL_ADC_SEQ_SetSectionNumber(ADCx, ADC_SEQ_InitStruct->NumSec);
    DDL_ADC_SEQ_SetSectionTimes(ADCx, DDL_ADC_SEQ_SECTION_1, ADC_SEQ_InitStruct->TimesSec1);
    DDL_ADC_SEQ_SetSectionTimes(ADCx, DDL_ADC_SEQ_SECTION_2, ADC_SEQ_InitStruct->TimesSec2);
    DDL_ADC_SEQ_SetSectionTimes(ADCx, DDL_ADC_SEQ_SECTION_3, ADC_SEQ_InitStruct->TimesSec3);
    DDL_ADC_SEQ_SetGapTime(ADCx, ADC_SEQ_InitStruct->GapTime);
    DDL_ADC_SEQ_SetTriggerSource(ADCx, DDL_ADC_SEQ_SECTION_1, ADC_SEQ_InitStruct->TriggerSourceSec1);
    DDL_ADC_SEQ_SetTriggerEdge(ADCx, DDL_ADC_SEQ_SECTION_1, ADC_SEQ_InitStruct->TriggerEdgeSec1);
    DDL_ADC_SEQ_SetTriggerSource(ADCx, DDL_ADC_SEQ_SECTION_2, ADC_SEQ_InitStruct->TriggerSourceSec2);
    DDL_ADC_SEQ_SetTriggerEdge(ADCx, DDL_ADC_SEQ_SECTION_2, ADC_SEQ_InitStruct->TriggerEdgeSec2);
    DDL_ADC_SEQ_SetTriggerSource(ADCx, DDL_ADC_SEQ_SECTION_3, ADC_SEQ_InitStruct->TriggerSourceSec3);
    DDL_ADC_SEQ_SetTriggerEdge(ADCx, DDL_ADC_SEQ_SECTION_3, ADC_SEQ_InitStruct->TriggerEdgeSec3);
  }
  else
  {
    /* Initialization error: ADC instance is not disabled. */
    status = ERROR;
  }
  return status;
}

/**
  * @brief  Set each @ref DDL_ADC_SEQ_InitTypeDef field to default value.
  * @param  ADC_SEQ_InitStruct Pointer to a @ref DDL_ADC_SEQ_InitTypeDef structure
  *                            whose fields will be set to default values.
  * @retval None
  */
void DDL_ADC_SEQ_StructInit(DDL_ADC_SEQ_InitTypeDef *ADC_SEQ_InitStruct)
{
  /* Set ADC_SEQ_InitStruct fields to default values */
  ADC_SEQ_InitStruct->NumSec               = DDL_ADC_SEQ_NUMBER_3;
  ADC_SEQ_InitStruct->TimesSec1            = DDL_ADC_SEQ_TRS_TIME_1;
  ADC_SEQ_InitStruct->TimesSec2            = DDL_ADC_SEQ_TRS_TIME_1;
  ADC_SEQ_InitStruct->TimesSec3            = DDL_ADC_SEQ_TRS_TIME_1;
  ADC_SEQ_InitStruct->GapTime              = DDL_ADC_GAPTIME_ADCCLK_0;
  ADC_SEQ_InitStruct->TriggerSourceSec1    = DDL_ADC_SEQ_TRIG_EXTSEL_ATMR_TRGO0;
  ADC_SEQ_InitStruct->TriggerSourceSec2    = DDL_ADC_SEQ_TRIG_EXTSEL_ATMR_TRGO0;
  ADC_SEQ_InitStruct->TriggerSourceSec3    = DDL_ADC_SEQ_TRIG_EXTSEL_ATMR_TRGO0;
  ADC_SEQ_InitStruct->TriggerEdgeSec1      = DDL_ADC_SEQ_TRIG_EXTEDGE_RISING;
  ADC_SEQ_InitStruct->TriggerEdgeSec2      = DDL_ADC_SEQ_TRIG_EXTEDGE_RISING;
  ADC_SEQ_InitStruct->TriggerEdgeSec3      = DDL_ADC_SEQ_TRIG_EXTEDGE_RISING;
}

/**
  * @}
  */

#endif /* ADC */

/**
  * @}
  */

#endif /* USE_FULL_DDL_DRIVER */
