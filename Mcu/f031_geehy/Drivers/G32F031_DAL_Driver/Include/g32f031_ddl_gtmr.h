/**
 *
 * @file    g32f031_ddl_gtmr.h
 * @brief   Header file of GTMR DDL module.
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
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef G32F031_DDL_GTMR_H
#define G32F031_DDL_GTMR_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "g32f0xx.h"

/** @addtogroup G32F031_DDL_Driver
 * @{
 */

#if defined(GTMR)

/** @defgroup GTMR_DDL GTMR
 * @{
 */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/** @defgroup GTMR_DDL_Private_Variables GTMR Private Variables
 * @{
 */
static const uint8_t DDL_GTMR_OFFSET_TAB_CCMRx[] =
{
    0x00U, /* 0: TMRx_CH1  */
    0x00U, /* 1: TMRx_CH2  */
    0x04U, /* 2: TMRx_CH3  */
    0x04U  /* 3: TMRx_CH4  */
};

static const uint8_t DDL_GTMR_SHIFT_TAB_OCxx[] =
{
    0U, /* 0: OC1M, OC1FE, OC1PE */
    8U, /* 1: OC2M, OC2FE, OC2PE */
    0U, /* 2: OC3M, OC3FE, OC3PE */
    8U  /* 3: OC4M, OC4FE, OC4PE */
};

static const uint8_t DDL_GTMR_SHIFT_TAB_ICxx[] =
{
    0U, /* 0: CC1S, IC1PSC, IC1F */
    8U, /* 1: CC2S, IC2PSC, IC2F */
    0U, /* 2: CC3S, IC3PSC, IC3F */
    8U  /* 3: CC4S, IC4PSC, IC4F */
};

static const uint8_t DDL_GTMR_SHIFT_TAB_ICSEL[] =
{
    0U, /* 0: IC1SEL */
    3U, /* 1: IC2SEL */
    6U, /* 2: IC3SEL */
    9U, /* 3: IC4SEL */
    12U /* 4: ETRSEL */
};

static const uint8_t DDL_GTMR_SHIFT_TAB_CCxP[] =
{
    0U,  /* 0: CC1P */
    4U,  /* 1: CC2P */
    8U,  /* 2: CC3P */
    12U  /* 3: CC4P */
};
/**
 * @}
 */

/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/** @defgroup GTMR_DDL_Private_Macros GTMR Private Macros
 * @{
 */
/** @brief  Convert channel id into channel index.
 * @param  __CHANNEL__ This parameter can be one of the following values:
 *         @arg @ref DDL_GTMR_CHANNEL_CH0
 *         @arg @ref DDL_GTMR_CHANNEL_CH1
 *         @arg @ref DDL_GTMR_CHANNEL_CH2
 *         @arg @ref DDL_GTMR_CHANNEL_CH3
 *         @arg @ref DDL_GTMR_CHANNEL_ETR
 * @retval none
 */
#define GTMR_GET_CHANNEL_INDEX(__CHANNEL__)                                                       \
    (((__CHANNEL__) == DDL_GTMR_CHANNEL_CH0)  ? 0U : \
     ((__CHANNEL__) == DDL_GTMR_CHANNEL_CH1)  ? 1U : \
     ((__CHANNEL__) == DDL_GTMR_CHANNEL_CH2)  ? 2U : \
     ((__CHANNEL__) == DDL_GTMR_CHANNEL_CH3)  ? 3U : 4U)

/** @brief  Calculate the deadtime sampling period(in ps).
 * @param  __TMRCLK__ timer input clock frequency (in Hz).
 * @param  __CKD__ This parameter can be one of the following values:
 *         @arg @ref DDL_GTMR_CLOCKDIVISION_DIV1
 *         @arg @ref DDL_GTMR_CLOCKDIVISION_DIV2
 *         @arg @ref DDL_GTMR_CLOCKDIVISION_DIV4
 * @retval none
 */
#define GTMR_CALC_DTS(__TMRCLK__, __CKD__)                                                                                                                                                   \
    (((__CKD__) == DDL_GTMR_CLOCKDIVISION_DIV1) ? ((uint64_t)1000000000000U / (__TMRCLK__)) : \
     ((__CKD__) == DDL_GTMR_CLOCKDIVISION_DIV2) ? ((uint64_t)1000000000000U / ((__TMRCLK__) >> 1U)) : ((uint64_t)1000000000000U / ((__TMRCLK__) >> 2U)))
/**
 * @}
 */

/* Exported types ------------------------------------------------------------*/
#if defined(USE_FULL_DDL_DRIVER)
/** @defgroup GTMR_DDL_ES_INIT GTMR Exported Init structure
 * @{
 */

/**
 * @brief  TMR Time Base configuration structure definition.
 */
typedef struct
{
    uint16_t Prescaler; /*!< Specifies the prescaler value used to divide the TMR clock.
                                This parameter can be a number between Min_Data=0x0000 and Max_Data=0xFFFF.

                                This feature can be modified afterwards using unitary function
                                @ref DDL_GTMR_SetPrescaler().*/

    uint32_t CounterMode; /*!< Specifies the counter mode.
                                This parameter can be a value of @ref GTMR_DDL_EC_COUNTERMODE.

                                This feature can be modified afterwards using unitary function
                                @ref DDL_GTMR_SetCounterMode().*/

    uint32_t Autoreload; /*!< Specifies the auto reload value to be loaded into the active
                                Auto-Reload Register at the next update event.
                                This parameter must be a number between Min_Data=0x0000 and Max_Data=0xFFFF.
                                Some timer instances may support 32 bits counters. In that case this parameter must
                                be a number between 0x0000 and 0xFFFFFFFF.

                                This feature can be modified afterwards using unitary function
                                @ref DDL_GTMR_SetAutoReload().*/

    uint32_t ClockDivision; /*!< Specifies the clock division.
                                    This parameter can be a value of @ref GTMR_DDL_EC_CLOCKDIVISION.

                                    This feature can be modified afterwards using unitary function
                                    @ref DDL_GTMR_SetClockDivision().*/
} DDL_GTMR_InitTypeDef;

/**
 * @brief  TMR Output Compare configuration structure definition.
 */
typedef struct
{
    uint32_t OCMode; /*!< Specifies the output mode.
                            This parameter can be a value of @ref GTMR_DDL_EC_OCMODE.

                            This feature can be modified afterwards using unitary function
                            @ref DDL_GTMR_OC_SetMode().*/

    uint32_t OCState; /*!< Specifies the TMR Output Compare state.
                            This parameter can be a value of @ref GTMR_DDL_EC_OCSTATE.

                            This feature can be modified afterwards using unitary functions
                            @ref DDL_GTMR_CC_EnableChannel() or @ref DDL_GTMR_CC_DisableChannel().*/

    uint32_t CompareValue; /*!< Specifies the Compare value to be loaded into the Capture Compare Register.
                                This parameter can be a number between Min_Data=0x0000 and Max_Data=0xFFFF.

                                This feature can be modified afterwards using unitary function
                                DDL_GTMR_OC_SetCompareCHx (x=1..6).*/

    uint32_t OCPolarity; /*!< Specifies the output polarity.
                                This parameter can be a value of @ref GTMR_DDL_EC_OCPOLARITY.

                                This feature can be modified afterwards using unitary function
                                @ref DDL_GTMR_OC_SetPolarity().*/
} DDL_GTMR_OC_InitTypeDef;

/**
 * @brief  TMR Input Capture configuration structure definition.
 */

typedef struct
{

    uint32_t ICPolarity;    /*!< Specifies the active edge of the input signal.
                                 This parameter can be a value of @ref GTMR_DDL_EC_IC_POLARITY.

                                 This feature can be modified afterwards using unitary function
                                 @ref DDL_GTMR_IC_SetPolarity().*/

    uint32_t ICActiveInput; /*!< Specifies the input.
                                 This parameter can be a value of @ref GTMR_DDL_EC_ACTIVEINPUT.

                                 This feature can be modified afterwards using unitary function
                                 @ref DDL_GTMR_IC_SetActiveInput().*/

    uint32_t ICPrescaler;   /*!< Specifies the Input Capture Prescaler.
                                 This parameter can be a value of @ref GTMR_DDL_EC_ICPSC.

                                 This feature can be modified afterwards using unitary function
                                 @ref DDL_GTMR_IC_SetPrescaler().*/

    uint32_t ICFilter;      /*!< Specifies the input capture filter.
                                 This parameter can be a value of @ref GTMR_DDL_EC_IC_FILTER.

                                 This feature can be modified afterwards using unitary function
                                 @ref DDL_GTMR_IC_SetFilter().*/
} DDL_GTMR_IC_InitTypeDef;

/**
 * @brief  TMR Encoder interface configuration structure definition.
 */
typedef struct
{
    uint32_t EncoderMode;  /*!< Specifies the encoder resolution (x2 or x4).
                                This parameter can be a value of @ref GTMR_DDL_EC_ENCODERMODE.

                                This feature can be modified afterwards using unitary function
                                @ref DDL_GTMR_SetEncoderMode().*/

    uint32_t IC1Polarity; /*!< Specifies the active edge of TI1 input.
                                This parameter can be a value of @ref GTMR_DDL_EC_IC_POLARITY.

                                This feature can be modified afterwards using unitary function
                                @ref DDL_GTMR_IC_SetPolarity().*/

    uint32_t IC1ActiveInput;   /*!< Specifies the TI1 input source
                                    This parameter can be a value of @ref GTMR_DDL_EC_ACTIVEINPUT.

                                    This feature can be modified afterwards using unitary function
                                    @ref DDL_GTMR_IC_SetActiveInput().*/

    uint32_t IC1Prescaler; /*!< Specifies the TI1 input prescaler value.
                                This parameter can be a value of @ref GTMR_DDL_EC_ICPSC.

                                This feature can be modified afterwards using unitary function
                                @ref DDL_GTMR_IC_SetPrescaler().*/

    uint32_t IC1Filter;    /*!< Specifies the TI1 input filter.
                                This parameter can be a value of @ref GTMR_DDL_EC_IC_FILTER.

                                This feature can be modified afterwards using unitary function
                                @ref DDL_GTMR_IC_SetFilter().*/

    uint32_t IC2Polarity;  /*!< Specifies the active edge of TI2 input.
                                This parameter can be a value of @ref GTMR_DDL_EC_IC_POLARITY.

                                This feature can be modified afterwards using unitary function
                                @ref DDL_GTMR_IC_SetPolarity().*/

    uint32_t IC2ActiveInput; /*!< Specifies the TI2 input source
                                    This parameter can be a value of @ref GTMR_DDL_EC_ACTIVEINPUT.

                                    This feature can be modified afterwards using unitary function
                                    @ref DDL_GTMR_IC_SetActiveInput().*/

    uint32_t IC2Prescaler; /*!< Specifies the TI2 input prescaler value.
                                This parameter can be a value of @ref GTMR_DDL_EC_ICPSC.

                                This feature can be modified afterwards using unitary function
                                @ref DDL_GTMR_IC_SetPrescaler().*/

    uint32_t IC2Filter;    /*!< Specifies the TI2 input filter.
                                This parameter can be a value of @ref GTMR_DDL_EC_IC_FILTER.

                                This feature can be modified afterwards using unitary function
                                @ref DDL_GTMR_IC_SetFilter().*/

} DDL_GTMR_ENCODER_InitTypeDef;

/**
 * @brief  TMR Hall sensor interface configuration structure definition.
 */
typedef struct
{

    uint32_t IC1Polarity; /*!< Specifies the active edge of TI1 input.
                                This parameter can be a value of @ref GTMR_DDL_EC_IC_POLARITY.

                                This feature can be modified afterwards using unitary function
                                @ref DDL_GTMR_IC_SetPolarity().*/

    uint32_t IC1Prescaler; /*!< Specifies the TI1 input prescaler value.
                                Prescaler must be set to get a maximum counter period longer than the
                                time interval between 2 consecutive changes on the Hall inputs.
                                This parameter can be a value of @ref GTMR_DDL_EC_ICPSC.

                                This feature can be modified afterwards using unitary function
                                @ref DDL_GTMR_IC_SetPrescaler().*/

    uint32_t IC1Filter; /*!< Specifies the TI1 input filter.
                                This parameter can be a value of
                                @ref GTMR_DDL_EC_IC_FILTER.

                                This feature can be modified afterwards using unitary function
                                @ref DDL_GTMR_IC_SetFilter().*/

    uint32_t CommutationDelay; /*!< Specifies the compare value to be loaded into the Capture Compare Register.
                                    A positive pulse (TRGO event) is generated with a programmable delay every time
                                    a change occurs on the Hall inputs.
                                    This parameter can be a number between Min_Data = 0x0000 and Max_Data = 0xFFFF.

                                    This feature can be modified afterwards using unitary function
                                    @ref DDL_GTMR_OC_SetCompareCH2().*/
} DDL_GTMR_HALLSENSOR_InitTypeDef;

/**
 * @}
 */
#endif /* USE_FULL_DDL_DRIVER */

/* Exported constants --------------------------------------------------------*/
/** @defgroup GTMR_DDL_Exported_Constants GTMR Exported Constants
 * @{
 */

/** @defgroup GTMR_DDL_EC_UPDATESOURCE Update Source
 * @{
 */
#define DDL_GTMR_UPDATESOURCE_REGULAR           0x00000000U           /*!< Counter overflow/underflow, Setting the UG bit or Update generation through the slave mode controller generates an update request */
#define DDL_GTMR_UPDATESOURCE_COUNTER           GTMR_CR1_URSSEL     /*!< Only counter overflow/underflow generates an update request */
/**
 * @}
 */

/** @defgroup GTMR_DDL_EC_ONEPULSEMODE One Pulse Mode
 * @{
 */
#define DDL_GTMR_ONEPULSEMODE_SINGLE            GTMR_CR1_SPMEN      /*!< Counter stops counting at the next update event */
#define DDL_GTMR_ONEPULSEMODE_REPETITIVE        0x00000000U           /*!< Counter is not stopped at update event */
/**
 * @}
 */

/** @defgroup GTMR_DDL_EC_COUNTERMODE Counter Mode
 * @{
 */
#define DDL_GTMR_COUNTERMODE_UP                 0x00000000U           /*!< Counter used as upcounter */
#define DDL_GTMR_COUNTERMODE_DOWN               GTMR_CR1_CNTDIR     /*!< Counter used as downcounter */
#define DDL_GTMR_COUNTERMODE_CENTER_DOWN        GTMR_CR1_CAMSEL_0   /*!< The counter counts up and down alternatively. Output compare interrupt flags of output channels  are set only when the counter is counting down. */
#define DDL_GTMR_COUNTERMODE_CENTER_UP          GTMR_CR1_CAMSEL_1   /*!< The counter counts up and down alternatively. Output compare interrupt flags of output channels  are set only when the counter is counting up */
#define DDL_GTMR_COUNTERMODE_CENTER_UP_DOWN     GTMR_CR1_CAMSEL     /*!< The counter counts up and down alternatively. Output compare interrupt flags of output channels  are set only when the counter is counting up or down. */
/**
 * @}
 */

/** @defgroup GTMR_DDL_EC_CLOCKDIVISION Clock Division
 * @{
 */
#define DDL_GTMR_CLOCKDIVISION_DIV1             0x00000000U           /*!< tDTS=tCK_INT */
#define DDL_GTMR_CLOCKDIVISION_DIV2             GTMR_CR1_CLKDIV_0   /*!< tDTS=2*tCK_INT */
#define DDL_GTMR_CLOCKDIVISION_DIV4             GTMR_CR1_CLKDIV_1   /*!< tDTS=4*tCK_INT */
/**
 * @}
 */

/** @defgroup GTMR_DDL_EC_COUNTERDIRECTION Counter Direction
 * @{
 */
#define DDL_GTMR_COUNTERDIRECTION_UP            0x00000000U           /*!< Timer counter counts up */
#define DDL_GTMR_COUNTERDIRECTION_DOWN          GTMR_CR1_CNTDIR     /*!< Timer counter counts down */
/**
 * @}
 */

/** @defgroup GTMR_DDL_EC_CCDMAREQUEST Capture Compare DMA Request
 * @{
 */
#define DDL_GTMR_CCDMAREQUEST_CC                0x00000000U           /*!< CCx DMA request sent when CCx event occurs */
#define DDL_GTMR_CCDMAREQUEST_UPDATE            GTMR_CR2_CCDSEL     /*!< CCx DMA requests sent when update event occurs */
/**
 * @}
 */

/** @defgroup GTMR_DDL_EC_CHANNEL Channel
 * @{
 */
#define DDL_GTMR_CHANNEL_CH0                    GTMR_CCEN_CC0EN       /*!< Timer input/output channel 1 */
#define DDL_GTMR_CHANNEL_CH1                    GTMR_CCEN_CC1EN       /*!< Timer input/output channel 2 */
#define DDL_GTMR_CHANNEL_CH2                    GTMR_CCEN_CC2EN       /*!< Timer input/output channel 3 */
#define DDL_GTMR_CHANNEL_CH3                    GTMR_CCEN_CC3EN       /*!< Timer input/output channel 4 */
#define DDL_GTMR_CHANNEL_ETR                    0x00000000U
/**
 * @}
 */

#if defined(USE_FULL_DDL_DRIVER)
/** @defgroup GTMR_DDL_EC_OCSTATE Output Configuration State
 * @{
 */
#define DDL_GTMR_OCSTATE_DISABLE                0x00000000U           /*!< OCx is not active */
#define DDL_GTMR_OCSTATE_ENABLE                 GTMR_CCEN_CC0EN       /*!< OCx signal is output on the corresponding output pin */
/**
 * @}
 */
#endif /* USE_FULL_DDL_DRIVER */

/** @defgroup GTMR_DDL_EC_OCMODE Output Configuration Mode
 * @{
 */
#define DDL_GTMR_OCMODE_FROZEN                  0x00000000U                                                 /*!< The comparison between the output compare register TMRx_CCRy and the counter TMRx_CNT has no effect on the output channel level */
#define DDL_GTMR_OCMODE_ACTIVE                  GTMR_CCM1_OC0MOD_0                                  /*!< OCyREF is forced high on compare match */
#define DDL_GTMR_OCMODE_INACTIVE                GTMR_CCM1_OC0MOD_1                                  /*!< OCyREF is forced low on compare match */
#define DDL_GTMR_OCMODE_TOGGLE                 (GTMR_CCM1_OC0MOD_1 | GTMR_CCM1_OC0MOD_0)    /*!< OCyREF toggles on compare match */
#define DDL_GTMR_OCMODE_FORCED_INACTIVE         GTMR_CCM1_OC0MOD_2                                  /*!< OCyREF is forced low */
#define DDL_GTMR_OCMODE_FORCED_ACTIVE          (GTMR_CCM1_OC0MOD_2 | GTMR_CCM1_OC0MOD_0)    /*!< OCyREF is forced high */
#define DDL_GTMR_OCMODE_PWM1                   (GTMR_CCM1_OC0MOD_2 | GTMR_CCM1_OC0MOD_1)    /*!< In upcounting, channel y is active as long as TMRx_CNT<TMRx_CCRy else inactive.  In downcounting, channel y is inactive as long as TMRx_CNT>TMRx_CCRy else active */
#define DDL_GTMR_OCMODE_PWM2                    GTMR_CCM1_OC0MOD                                    /*!< In upcounting, channel y is inactive as long as TMRx_CNT<TMRx_CCRy else active.  In downcounting, channel y is active as long as TMRx_CNT>TMRx_CCRy else inactive */
/**
 * @}
 */

/** @defgroup GTMR_DDL_EC_OCPOLARITY Output Configuration Polarity
 * @{
 */
#define DDL_GTMR_OCPOLARITY_HIGH                0x00000000U           /*!< OCxactive high */
#define DDL_GTMR_OCPOLARITY_LOW                 GTMR_CCEN_CC0POL      /*!< OCxactive low */
/**
 * @}
 */

/** @defgroup GTMR_DDL_EC_ACTIVEINPUT Active Input Selection
 * @{
 */
#define DDL_GTMR_ACTIVEINPUT_DIRECTTI          (GTMR_CCM1_CC0SEL_0 << 16U)  /*!< ICx is mapped on TIx */
#define DDL_GTMR_ACTIVEINPUT_INDIRECTTI        (GTMR_CCM1_CC0SEL_1 << 16U)  /*!< ICx is mapped on TIy */
#define DDL_GTMR_ACTIVEINPUT_TRC               (GTMR_CCM1_CC0SEL << 16U)    /*!< ICx is mapped on TRC */
/**
 * @}
 */

/** @defgroup GTMR_DDL_EC_ICPSC Input Configuration Prescaler
 * @{
 */
#define DDL_GTMR_ICPSC_DIV1                     0x00000000U                         /*!< No prescaler, capture is done each time an edge is detected on the capture input */
#define DDL_GTMR_ICPSC_DIV2                    (GTMR_CCM1_IC0PSC_0 << 16U)  /*!< Capture is done once every 2 events */
#define DDL_GTMR_ICPSC_DIV4                    (GTMR_CCM1_IC0PSC_1 << 16U)  /*!< Capture is done once every 4 events */
#define DDL_GTMR_ICPSC_DIV8                    (GTMR_CCM1_IC0PSC << 16U)    /*!< Capture is done once every 8 events */
/**
 * @}
 */

/** @defgroup GTMR_DDL_EC_IC_FILTER Input Configuration Filter
 * @{
 */
#define DDL_GTMR_IC_FILTER_FDIV1                0x00000000U                                                                             /*!< No filter, sampling is done at fDTS */
#define DDL_GTMR_IC_FILTER_FDIV1_N2            (GTMR_CCM1_IC0F_0 << 16U)                                                        /*!< fSAMPLING=fCK_INT, N=2 */
#define DDL_GTMR_IC_FILTER_FDIV1_N4            (GTMR_CCM1_IC0F_1 << 16U)                                                        /*!< fSAMPLING=fCK_INT, N=4 */
#define DDL_GTMR_IC_FILTER_FDIV1_N8           ((GTMR_CCM1_IC0F_1 | GTMR_CCM1_IC0F_0) << 16U)                            /*!< fSAMPLING=fCK_INT, N=8 */
#define DDL_GTMR_IC_FILTER_FDIV2_N6            (GTMR_CCM1_IC0F_2 << 16U)                                                        /*!< fSAMPLING=fDTS/2, N=6 */
#define DDL_GTMR_IC_FILTER_FDIV2_N8           ((GTMR_CCM1_IC0F_2 | GTMR_CCM1_IC0F_0) << 16U)                            /*!< fSAMPLING=fDTS/2, N=8 */
#define DDL_GTMR_IC_FILTER_FDIV4_N6           ((GTMR_CCM1_IC0F_2 | GTMR_CCM1_IC0F_1) << 16U)                            /*!< fSAMPLING=fDTS/4, N=6 */
#define DDL_GTMR_IC_FILTER_FDIV4_N8           ((GTMR_CCM1_IC0F_2 | GTMR_CCM1_IC0F_1 | GTMR_CCM1_IC0F_0) << 16U) /*!< fSAMPLING=fDTS/4, N=8 */
#define DDL_GTMR_IC_FILTER_FDIV8_N6            (GTMR_CCM1_IC0F_3 << 16U)                                                        /*!< fSAMPLING=fDTS/8, N=6 */
#define DDL_GTMR_IC_FILTER_FDIV8_N8           ((GTMR_CCM1_IC0F_3 | GTMR_CCM1_IC0F_0) << 16U)                            /*!< fSAMPLING=fDTS/8, N=8 */
#define DDL_GTMR_IC_FILTER_FDIV16_N5          ((GTMR_CCM1_IC0F_3 | GTMR_CCM1_IC0F_1) << 16U)                            /*!< fSAMPLING=fDTS/16, N=5 */
#define DDL_GTMR_IC_FILTER_FDIV16_N6          ((GTMR_CCM1_IC0F_3 | GTMR_CCM1_IC0F_1 | GTMR_CCM1_IC0F_0) << 16U) /*!< fSAMPLING=fDTS/16, N=6 */
#define DDL_GTMR_IC_FILTER_FDIV16_N8          ((GTMR_CCM1_IC0F_3 | GTMR_CCM1_IC0F_2) << 16U)                            /*!< fSAMPLING=fDTS/16, N=8 */
#define DDL_GTMR_IC_FILTER_FDIV32_N5          ((GTMR_CCM1_IC0F_3 | GTMR_CCM1_IC0F_2 | GTMR_CCM1_IC0F_0) << 16U) /*!< fSAMPLING=fDTS/32, N=5 */
#define DDL_GTMR_IC_FILTER_FDIV32_N6          ((GTMR_CCM1_IC0F_3 | GTMR_CCM1_IC0F_2 | GTMR_CCM1_IC0F_1) << 16U) /*!< fSAMPLING=fDTS/32, N=6 */
#define DDL_GTMR_IC_FILTER_FDIV32_N8           (GTMR_CCM1_IC0F << 16U)                                                          /*!< fSAMPLING=fDTS/32, N=8 */
/**
 * @}
 */

/** @defgroup GTMR_DDL_EC_IC_POLARITY Input Configuration Polarity
 * @{
 */
#define DDL_GTMR_IC_POLARITY_RISING             0x00000000U           /*!< The circuit is sensitive to TIxFP1 rising edge, TIxFP1 is not inverted */
#define DDL_GTMR_IC_POLARITY_FALLING            GTMR_CCEN_CC0POL      /*!< The circuit is sensitive to TIxFP1 falling edge, TIxFP1 is inverted */
/**
 * @}
 */

/** @defgroup GTMR_DDL_EC_IC_POLARITY Input Configuration Polarity
 * @{
 */
#define DDL_GTMR_IC_INTPUTSOURCE_CHANNEL        0x00000000U                                         /*!< The input source is channel */
#define DDL_GTMR_IC_INTPUTSOURCE_COMP0          GTMR_CR2_COMPCH0SEL_0                             /*!< The input source is COMP 0  */
#define DDL_GTMR_IC_INTPUTSOURCE_COMP1          GTMR_CR2_COMPCH0SEL_1                             /*!< The input source is COMP 1  */
#define DDL_GTMR_IC_INTPUTSOURCE_COMP2          (GTMR_CR2_COMPCH0SEL_1 | GTMR_CR2_COMPCH0SEL_0) /*!< The input source is COMP 2  */
#define DDL_GTMR_IC_INTPUTSOURCE_COMP3          GTMR_CR2_COMPCH0SEL_2                             /*!< The input source is COMP 3  */
/**
 * @}
 */

/** @defgroup GTMR_DDL_EC_CLOCKSOURCE Clock Source
 * @{
 */
#define DDL_GTMR_CLOCKSOURCE_INTERNAL           0x00000000U           /*!< The timer is clocked by the internal clock provided from the RCC */
#define DDL_GTMR_CLOCKSOURCE_EXT_MODE1          GTMR_SMCR_SMFSEL    /*!< Counter counts at each rising or falling edge on a selected input */
#define DDL_GTMR_CLOCKSOURCE_EXT_MODE2          GTMR_SMCR_ECEN      /*!< Counter counts at each rising or falling edge on the external trigger input ETR */
/**
 * @}
 */

/** @defgroup GTMR_DDL_EC_ENCODERMODE Encoder Mode
 * @{
 */
#define DDL_GTMR_ENCODERMODE_X2_TI1             GTMR_SMCR_SMFSEL_0                           /*!< Quadrature encoder mode 1, x2 mode - Counter counts up/down on TI1FP1 edge depending on TI2FP2 level */
#define DDL_GTMR_ENCODERMODE_X2_TI2             GTMR_SMCR_SMFSEL_1                           /*!< Quadrature encoder mode 2, x2 mode - Counter counts up/down on TI2FP2 edge depending on TI1FP1 level */
#define DDL_GTMR_ENCODERMODE_X4_TI12           (GTMR_SMCR_SMFSEL_1 | GTMR_SMCR_SMFSEL_0)   /*!< Quadrature encoder mode 3, x4 mode - Counter counts up/down on both TI1FP1 and TI2FP2 edges depending on the level of the other input */
/**
 * @}
 */

/** @defgroup GTMR_DDL_EC_TRGO Trigger Output
 * @{
 */
#define DDL_GTMR_TRGO_RESET                     0x00000000U                                 /*!< UG bit from the TMRx_EGR register is used as trigger output */
#define DDL_GTMR_TRGO_ENABLE                    GTMR_CR2_MMSEL_0                          /*!< Counter Enable signal (CNT_EN) is used as trigger output */
#define DDL_GTMR_TRGO_UPDATE                    GTMR_CR2_MMSEL_1                          /*!< Update event is used as trigger output */
#define DDL_GTMR_TRGO_CC1IF                    (GTMR_CR2_MMSEL_1 | GTMR_CR2_MMSEL_0)    /*!< CC1 capture or a compare match is used as trigger output */
#define DDL_GTMR_TRGO_OC0REF                    GTMR_CR2_MMSEL_2                          /*!< OC1REF signal is used as trigger output */
#define DDL_GTMR_TRGO_OC1REF                   (GTMR_CR2_MMSEL_2 | GTMR_CR2_MMSEL_0)    /*!< OC2REF signal is used as trigger output */
#define DDL_GTMR_TRGO_OC2REF                   (GTMR_CR2_MMSEL_2 | GTMR_CR2_MMSEL_1)    /*!< OC3REF signal is used as trigger output */
#define DDL_GTMR_TRGO_OC3REF                    GTMR_CR2_MMSEL                            /*!< OC4REF signal is used as trigger output */
/**
 * @}
 */

/** @defgroup GTMR_DDL_EC_SLAVEMODE Slave Mode
 * @{
 */
#define DDL_GTMR_SLAVEMODE_DISABLED             0x00000000U                                  /*!< Slave mode disabled */
#define DDL_GTMR_SLAVEMODE_RESET                GTMR_SMCR_SMFSEL_2                         /*!< Reset Mode - Rising edge of the selected trigger input (TRGI) reinitializes the counter */
#define DDL_GTMR_SLAVEMODE_GATED               (GTMR_SMCR_SMFSEL_2 | GTMR_SMCR_SMFSEL_0) /*!< Gated Mode - The counter clock is enabled when the trigger input (TRGI) is high */
#define DDL_GTMR_SLAVEMODE_TRIGGER             (GTMR_SMCR_SMFSEL_2 | GTMR_SMCR_SMFSEL_1) /*!< Trigger Mode - The counter starts at a rising edge of the trigger TRGI */
/**
 * @}
 */

/** @defgroup GTMR_DDL_EC_TS Trigger Selection
 * @{
 */
#define DDL_GTMR_TS_ITR0                        0x00000000U                                     /*!< Internal Trigger 0 (ITR0) is used as trigger input */
#define DDL_GTMR_TS_ITR1                        GTMR_SMCR_TRGSEL_0                            /*!< Internal Trigger 1 (ITR1) is used as trigger input */
#define DDL_GTMR_TS_ITR2                        GTMR_SMCR_TRGSEL_1                            /*!< Internal Trigger 2 (ITR2) is used as trigger input */
#define DDL_GTMR_TS_ITR3                       (GTMR_SMCR_TRGSEL_1 | GTMR_SMCR_TRGSEL_0)    /*!< Internal Trigger 3 (ITR3) is used as trigger input */
#define DDL_GTMR_TS_TI1F_ED                     GTMR_SMCR_TRGSEL_2                            /*!< TI1 Edge Detector (TI1F_ED) is used as trigger input */
#define DDL_GTMR_TS_TI1FP1                     (GTMR_SMCR_TRGSEL_2 | GTMR_SMCR_TRGSEL_0)    /*!< Filtered Timer Input 1 (TI1FP1) is used as trigger input */
#define DDL_GTMR_TS_TI2FP2                     (GTMR_SMCR_TRGSEL_2 | GTMR_SMCR_TRGSEL_1)    /*!< Filtered Timer Input 2 (TI12P2) is used as trigger input */
#define DDL_GTMR_TS_ETRF                        GTMR_SMCR_TRGSEL                              /*!< Filtered external Trigger (ETRF) is used as trigger input */
/**
 * @}
 */

/** @defgroup GTMR_DDL_EC_ETR_POLARITY External Trigger Polarity
 * @{
 */
#define DDL_GTMR_ETR_POLARITY_NONINVERTED       0x00000000U           /*!< ETR is non-inverted, active at high level or rising edge */
#define DDL_GTMR_ETR_POLARITY_INVERTED          GTMR_SMCR_ETPOL     /*!< ETR is inverted, active at low level or falling edge */
/**
 * @}
 */

/** @defgroup GTMR_DDL_EC_ETR_PRESCALER External Trigger Prescaler
 * @{
 */
#define DDL_GTMR_ETR_PRESCALER_DIV1             0x00000000U           /*!< ETR prescaler OFF */
#define DDL_GTMR_ETR_PRESCALER_DIV2             GTMR_SMCR_ETPCFG_0  /*!< ETR frequency is divided by 2 */
#define DDL_GTMR_ETR_PRESCALER_DIV4             GTMR_SMCR_ETPCFG_1  /*!< ETR frequency is divided by 4 */
#define DDL_GTMR_ETR_PRESCALER_DIV8             GTMR_SMCR_ETPCFG    /*!< ETR frequency is divided by 8 */
/**
 * @}
 */

/** @defgroup GTMR_DDL_EC_ETR_FILTER External Trigger Filter
 * @{
 */
#define DDL_GTMR_ETR_FILTER_FDIV1               0x00000000U                                                         /*!< No filter, sampling is done at fDTS */
#define DDL_GTMR_ETR_FILTER_FDIV1_N2            GTMR_SMCR_ETFCFG_0                                                /*!< fSAMPLING=fCK_INT, N=2 */
#define DDL_GTMR_ETR_FILTER_FDIV1_N4            GTMR_SMCR_ETFCFG_1                                                /*!< fSAMPLING=fCK_INT, N=4 */
#define DDL_GTMR_ETR_FILTER_FDIV1_N8           (GTMR_SMCR_ETFCFG_1 | GTMR_SMCR_ETFCFG_0)                        /*!< fSAMPLING=fCK_INT, N=8 */
#define DDL_GTMR_ETR_FILTER_FDIV2_N6            GTMR_SMCR_ETFCFG_2                                                /*!< fSAMPLING=fDTS/2, N=6 */
#define DDL_GTMR_ETR_FILTER_FDIV2_N8           (GTMR_SMCR_ETFCFG_2 | GTMR_SMCR_ETFCFG_0)                        /*!< fSAMPLING=fDTS/2, N=8 */
#define DDL_GTMR_ETR_FILTER_FDIV4_N6           (GTMR_SMCR_ETFCFG_2 | GTMR_SMCR_ETFCFG_1)                        /*!< fSAMPLING=fDTS/4, N=6 */
#define DDL_GTMR_ETR_FILTER_FDIV4_N8           (GTMR_SMCR_ETFCFG_2 | GTMR_SMCR_ETFCFG_1 | GTMR_SMCR_ETFCFG_0) /*!< fSAMPLING=fDTS/4, N=8 */
#define DDL_GTMR_ETR_FILTER_FDIV8_N6            GTMR_SMCR_ETFCFG_3                                                /*!< fSAMPLING=fDTS/8, N=8 */
#define DDL_GTMR_ETR_FILTER_FDIV8_N8           (GTMR_SMCR_ETFCFG_3 | GTMR_SMCR_ETFCFG_0)                        /*!< fSAMPLING=fDTS/16, N=5 */
#define DDL_GTMR_ETR_FILTER_FDIV16_N5          (GTMR_SMCR_ETFCFG_3 | GTMR_SMCR_ETFCFG_1)                        /*!< fSAMPLING=fDTS/16, N=6 */
#define DDL_GTMR_ETR_FILTER_FDIV16_N6          (GTMR_SMCR_ETFCFG_3 | GTMR_SMCR_ETFCFG_1 | GTMR_SMCR_ETFCFG_0) /*!< fSAMPLING=fDTS/16, N=8 */
#define DDL_GTMR_ETR_FILTER_FDIV16_N8          (GTMR_SMCR_ETFCFG_3 | GTMR_SMCR_ETFCFG_2)                        /*!< fSAMPLING=fDTS/16, N=5 */
#define DDL_GTMR_ETR_FILTER_FDIV32_N5          (GTMR_SMCR_ETFCFG_3 | GTMR_SMCR_ETFCFG_2 | GTMR_SMCR_ETFCFG_0) /*!< fSAMPLING=fDTS/32, N=5 */
#define DDL_GTMR_ETR_FILTER_FDIV32_N6          (GTMR_SMCR_ETFCFG_3 | GTMR_SMCR_ETFCFG_2 | GTMR_SMCR_ETFCFG_1) /*!< fSAMPLING=fDTS/32, N=6 */
#define DDL_GTMR_ETR_FILTER_FDIV32_N8           GTMR_SMCR_ETFCFG                                                  /*!< fSAMPLING=fDTS/32, N=8 */
/**
 * @}
 */

/** @defgroup GTMR_DDL_EC_DMABURST_BASEADDR DMA Burst Base Address
 * @{
 */
#define DDL_GTMR_DMABURST_BASEADDR_CTRL1        0x00000000U                                                                            /*!< TMRx_CTRL1 register is the DMA base address for DMA burst */
#define DDL_GTMR_DMABURST_BASEADDR_CTRL2        GTMR_DCTRL_DBADDR_0                                                                    /*!< TMRx_CTRL2 register is the DMA base address for DMA burst */
#define DDL_GTMR_DMABURST_BASEADDR_SMCTRL       GTMR_DCTRL_DBADDR_1                                                                    /*!< TMRx_SMCTRL register is the DMA base address for DMA burst */
#define DDL_GTMR_DMABURST_BASEADDR_DIER        (GTMR_DCTRL_DBADDR_1 | GTMR_DCTRL_DBADDR_0)                                             /*!< TMRx_DIER register is the DMA base address for DMA burst */
#define DDL_GTMR_DMABURST_BASEADDR_STS          GTMR_DCTRL_DBADDR_2                                                                    /*!< TMRx_STS register is the DMA base address for DMA burst */
#define DDL_GTMR_DMABURST_BASEADDR_CEG         (GTMR_DCTRL_DBADDR_2 | GTMR_DCTRL_DBADDR_0)                                             /*!< TMRx_CEG register is the DMA base address for DMA burst */
#define DDL_GTMR_DMABURST_BASEADDR_CCM1        (GTMR_DCTRL_DBADDR_2 | GTMR_DCTRL_DBADDR_1)                                             /*!< TMRx_CCM1 register is the DMA base address for DMA burst */
#define DDL_GTMR_DMABURST_BASEADDR_CCM2        (GTMR_DCTRL_DBADDR_2 | GTMR_DCTRL_DBADDR_1 | GTMR_DCTRL_DBADDR_0)                       /*!< TMRx_CCM2 register is the DMA base address for DMA burst */
#define DDL_GTMR_DMABURST_BASEADDR_CCEN         GTMR_DCTRL_DBADDR_3                                                                    /*!< TMRx_CCEN register is the DMA base address for DMA burst */
#define DDL_GTMR_DMABURST_BASEADDR_CNT         (GTMR_DCTRL_DBADDR_3 | GTMR_DCTRL_DBADDR_0)                                             /*!< TMRx_CNT register is the DMA base address for DMA burst */
#define DDL_GTMR_DMABURST_BASEADDR_PSC         (GTMR_DCTRL_DBADDR_3 | GTMR_DCTRL_DBADDR_1)                                             /*!< TMRx_PSC register is the DMA base address for DMA burst */
#define DDL_GTMR_DMABURST_BASEADDR_AUTORLD     (GTMR_DCTRL_DBADDR_3 | GTMR_DCTRL_DBADDR_1 | GTMR_DCTRL_DBADDR_0)                       /*!< TMRx_AUTORLD register is the DMA base address for DMA burst */
#define DDL_GTMR_DMABURST_BASEADDR_REPCNT      (GTMR_DCTRL_DBADDR_3 | GTMR_DCTRL_DBADDR_2)                                             /*!< TMRx_REPCNT register is the DMA base address for DMA burst */
#define DDL_GTMR_DMABURST_BASEADDR_CC1         (GTMR_DCTRL_DBADDR_3 | GTMR_DCTRL_DBADDR_2 | GTMR_DCTRL_DBADDR_0)                       /*!< TMRx_CC1 register is the DMA base address for DMA burst */
#define DDL_GTMR_DMABURST_BASEADDR_CC2         (GTMR_DCTRL_DBADDR_3 | GTMR_DCTRL_DBADDR_2 | GTMR_DCTRL_DBADDR_1)                       /*!< TMRx_CC2 register is the DMA base address for DMA burst */
#define DDL_GTMR_DMABURST_BASEADDR_CC3         (GTMR_DCTRL_DBADDR_3 | GTMR_DCTRL_DBADDR_2 | GTMR_DCTRL_DBADDR_1 | GTMR_DCTRL_DBADDR_0) /*!< TMRx_CC3 register is the DMA base address for DMA burst */
#define DDL_GTMR_DMABURST_BASEADDR_CC4          GTMR_DCTRL_DBADDR_4                                                                    /*!< TMRx_CC4 register is the DMA base address for DMA burst */
#define DDL_GTMR_DMABURST_BASEADDR_BDT         (GTMR_DCTRL_DBADDR_4 | GTMR_DCTRL_DBADDR_0)                                             /*!< TMRx_BDT register is the DMA base address for DMA burst */
/**
 * @}
 */

/** @defgroup GTMR_DDL_EC_DMABURST_LENGTH DMA Burst Length
 * @{
 */
#define DDL_GTMR_DMABURST_LENGTH_1TRANSFER      0x00000000U                                                                        /*!< Transfer is done to 1 register starting from the DMA burst base address */
#define DDL_GTMR_DMABURST_LENGTH_2TRANSFERS     GTMR_DCTRL_DBLEN_0                                                                 /*!< Transfer is done to 2 registers starting from the DMA burst base address */
#define DDL_GTMR_DMABURST_LENGTH_3TRANSFERS     GTMR_DCTRL_DBLEN_1                                                                 /*!< Transfer is done to 3 registers starting from the DMA burst base address */
#define DDL_GTMR_DMABURST_LENGTH_4TRANSFERS    (GTMR_DCTRL_DBLEN_1 | GTMR_DCTRL_DBLEN_0)                                           /*!< Transfer is done to 4 registers starting from the DMA burst base address */
#define DDL_GTMR_DMABURST_LENGTH_5TRANSFERS     GTMR_DCTRL_DBLEN_2                                                                 /*!< Transfer is done to 5 registers starting from the DMA burst base address */
#define DDL_GTMR_DMABURST_LENGTH_6TRANSFERS    (GTMR_DCTRL_DBLEN_2 | GTMR_DCTRL_DBLEN_0)                                           /*!< Transfer is done to 6 registers starting from the DMA burst base address */
#define DDL_GTMR_DMABURST_LENGTH_7TRANSFERS    (GTMR_DCTRL_DBLEN_2 | GTMR_DCTRL_DBLEN_1)                                           /*!< Transfer is done to 7 registers starting from the DMA burst base address */
#define DDL_GTMR_DMABURST_LENGTH_8TRANSFERS    (GTMR_DCTRL_DBLEN_2 | GTMR_DCTRL_DBLEN_1 | GTMR_DCTRL_DBLEN_0)                      /*!< Transfer is done to 1 registers starting from the DMA burst base address */
#define DDL_GTMR_DMABURST_LENGTH_9TRANSFERS     GTMR_DCTRL_DBLEN_3                                                                 /*!< Transfer is done to 9 registers starting from the DMA burst base address */
#define DDL_GTMR_DMABURST_LENGTH_10TRANSFERS   (GTMR_DCTRL_DBLEN_3 | GTMR_DCTRL_DBLEN_0)                                           /*!< Transfer is done to 10 registers starting from the DMA burst base address */
#define DDL_GTMR_DMABURST_LENGTH_11TRANSFERS   (GTMR_DCTRL_DBLEN_3 | GTMR_DCTRL_DBLEN_1)                                           /*!< Transfer is done to 11 registers starting from the DMA burst base address */
#define DDL_GTMR_DMABURST_LENGTH_12TRANSFERS   (GTMR_DCTRL_DBLEN_3 | GTMR_DCTRL_DBLEN_1 | GTMR_DCTRL_DBLEN_0)                      /*!< Transfer is done to 12 registers starting from the DMA burst base address */
#define DDL_GTMR_DMABURST_LENGTH_13TRANSFERS   (GTMR_DCTRL_DBLEN_3 | GTMR_DCTRL_DBLEN_2)                                           /*!< Transfer is done to 13 registers starting from the DMA burst base address */
#define DDL_GTMR_DMABURST_LENGTH_14TRANSFERS   (GTMR_DCTRL_DBLEN_3 | GTMR_DCTRL_DBLEN_2 | GTMR_DCTRL_DBLEN_0)                      /*!< Transfer is done to 14 registers starting from the DMA burst base address */
#define DDL_GTMR_DMABURST_LENGTH_15TRANSFERS   (GTMR_DCTRL_DBLEN_3 | GTMR_DCTRL_DBLEN_2 | GTMR_DCTRL_DBLEN_1)                      /*!< Transfer is done to 15 registers starting from the DMA burst base address */
#define DDL_GTMR_DMABURST_LENGTH_16TRANSFERS   (GTMR_DCTRL_DBLEN_3 | GTMR_DCTRL_DBLEN_2 | GTMR_DCTRL_DBLEN_1 | GTMR_DCTRL_DBLEN_0) /*!< Transfer is done to 16 registers starting from the DMA burst base address */
#define DDL_GTMR_DMABURST_LENGTH_17TRANSFERS    GTMR_DCTRL_DBLEN_4                                                                 /*!< Transfer is done to 17 registers starting from the DMA burst base address */
#define DDL_GTMR_DMABURST_LENGTH_18TRANSFERS   (GTMR_DCTRL_DBLEN_4 | GTMR_DCTRL_DBLEN_0)                                           /*!< Transfer is done to 18 registers starting from the DMA burst base address */
/**
 * @}
 */

/** @defgroup GTMR_DDL_EC_TRIGGLE_OUTPUT_MODE Triggle Output Configuration
 * @{
 */
#define DDL_GTMR_TRIGGLE_OUTPUT_RESET           0x00000000U
#define DDL_GTMR_TRIGGLE_OUTPUT_ENABLE          0x00000001U
#define DDL_GTMR_TRIGGLE_OUTPUT_UPDATE          0x00000002U
#define DDL_GTMR_TRIGGLE_OUTPUT_CCIF_EVENT      0x00000003U
#define DDL_GTMR_TRIGGLE_OUTPUT_OC1REF          0x00000004U
#define DDL_GTMR_TRIGGLE_OUTPUT_OC2REF          0x00000005U
#define DDL_GTMR_TRIGGLE_OUTPUT_OC3REF          0x00000006U
#define DDL_GTMR_TRIGGLE_OUTPUT_OC4REF          0x00000007U
#define DDL_GTMR_TRIGGLE_OUTPUT_OC5REF          0x00000008U

/**
 * @}
 */

/**
 * @}
 */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup GTMR_DDL_Exported_Macros GTMR Exported Macros
 * @{
 */

/** @defgroup GTMR_DDL_EM_WRITE_READ Common Write and read registers Macros
 * @{
 */
/**
 * @brief  Write a value in TMR register.
 * @param  __INSTANCE__ TMR Instance
 * @param  __REG__ Register to be written
 * @param  __VALUE__ Value to be written in the register
 * @retval None
 */
#define DDL_GTMR_WriteReg(__INSTANCE__, __REG__, __VALUE__) WRITE_REG((__INSTANCE__)->__REG__, (__VALUE__))

/**
 * @brief  Read a value in TMR register.
 * @param  __INSTANCE__ TMR Instance
 * @param  __REG__ Register to be read
 * @retval Register value
 */
#define DDL_GTMR_ReadReg(__INSTANCE__, __REG__) READ_REG((__INSTANCE__)->__REG__)
/**
 * @}
 */

/** @defgroup GTMR_DDL_EM_Exported_Macros Exported_Macros
 * @{
 */

/**
 * @brief  HELPER macro calculating the prescaler value to achieve the required counter clock frequency.
 * @note ex: @ref __DDL_GTMR_CALC_PSC (80000000, 1000000);
 * @param  __TMRCLK__ timer input clock frequency (in Hz)
 * @param  __CNTCLK__ counter clock frequency (in Hz)
 * @retval Prescaler value  (between Min_Data=0 and Max_Data=65535)
 */
#define __DDL_GTMR_CALC_PSC(__TMRCLK__, __CNTCLK__) \
    (((__TMRCLK__) >= (__CNTCLK__)) ? (uint32_t)((((__TMRCLK__) + (__CNTCLK__) / 2U) / (__CNTCLK__)) - 1U) : 0U)

/**
 * @brief  HELPER macro calculating the auto-reload value to achieve the required output signal frequency.
 * @note ex: @ref __DDL_GTMR_CALC_ARR (1000000, @ref DDL_GTMR_GetPrescaler (), 10000);
 * @param  __TMRCLK__ timer input clock frequency (in Hz)
 * @param  __PSC__ prescaler
 * @param  __FREQ__ output signal frequency (in Hz)
 * @retval  Auto-reload value  (between Min_Data=0 and Max_Data=65535)
 */
#define __DDL_GTMR_CALC_ARR(__TMRCLK__, __PSC__, __FREQ__) \
    ((((__TMRCLK__) / ((__PSC__) + 1U)) >= (__FREQ__)) ? (((__TMRCLK__) / ((__FREQ__) * ((__PSC__) + 1U))) - 1U) : 0U)

/**
 * @brief  HELPER macro calculating the compare value required to achieve the required timer output compare
 *         active/inactive delay.
 * @note ex: @ref __DDL_GTMR_CALC_DELAY (1000000, @ref DDL_GTMR_GetPrescaler (), 10);
 * @param  __TMRCLK__ timer input clock frequency (in Hz)
 * @param  __PSC__ prescaler
 * @param  __DELAY__ timer output compare active/inactive delay (in us)
 * @retval Compare value  (between Min_Data=0 and Max_Data=65535)
 */
#define __DDL_GTMR_CALC_DELAY(__TMRCLK__, __PSC__, __DELAY__) \
    ((uint32_t)(((uint64_t)(__TMRCLK__) * (uint64_t)(__DELAY__)) / ((uint64_t)1000000U * (uint64_t)((__PSC__) + 1U))))

/**
 * @brief  HELPER macro calculating the auto-reload value to achieve the required pulse duration
 *         (when the timer operates in one pulse mode).
 * @note ex: @ref __DDL_GTMR_CALC_PULSE (1000000, @ref DDL_GTMR_GetPrescaler (), 10, 20);
 * @param  __TMRCLK__ timer input clock frequency (in Hz)
 * @param  __PSC__ prescaler
 * @param  __DELAY__ timer output compare active/inactive delay (in us)
 * @param  __PULSE__ pulse duration (in us)
 * @retval Auto-reload value  (between Min_Data=0 and Max_Data=65535)
 */
#define __DDL_GTMR_CALC_PULSE(__TMRCLK__, __PSC__, __DELAY__, __PULSE__) \
    ((uint32_t)(__DDL_GTMR_CALC_DELAY((__TMRCLK__), (__PSC__), (__PULSE__)) + __DDL_GTMR_CALC_DELAY((__TMRCLK__), (__PSC__), (__DELAY__))))

/**
 * @}
 */

/**
 * @}
 */

/* Exported functions --------------------------------------------------------*/
/** @defgroup GTMR_DDL_Exported_Functions GTMR Exported Functions
 * @{
 */

/** @defgroup GTMR_DDL_EF_Time_Base Time Base configuration
 * @{
 */
/**
 * @brief  Enable timer counter.
 * @param  TMRx Timer instance
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_EnableCounter(GTMR_TypeDef *TMRx)
{
    SET_BIT(TMRx->CR1, GTMR_CR1_CNTEN);
}

/**
 * @brief  Disable timer counter.
 * @param  TMRx Timer instance
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_DisableCounter(GTMR_TypeDef *TMRx)
{
    CLEAR_BIT(TMRx->CR1, GTMR_CR1_CNTEN);
}

/**
 * @brief  Indicates whether the timer counter is enabled.
 * @param  TMRx Timer instance
 * @retval State of bit (1 or 0).
 */
__STATIC_INLINE uint32_t DDL_GTMR_IsEnabledCounter(GTMR_TypeDef *TMRx)
{
    return ((READ_BIT(TMRx->CR1, GTMR_CR1_CNTEN) == (GTMR_CR1_CNTEN)) ? 1UL : 0UL);
}

/**
 * @brief  Enable update event generation.
 * @param  TMRx Timer instance
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_EnableUpdateEvent(GTMR_TypeDef *TMRx)
{
    CLEAR_BIT(TMRx->CR1, GTMR_CR1_UD);
}

/**
 * @brief  Disable update event generation.
 * @param  TMRx Timer instance
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_DisableUpdateEvent(GTMR_TypeDef *TMRx)
{
    SET_BIT(TMRx->CR1, GTMR_CR1_UD);
}

/**
 * @brief  Indicates whether update event generation is enabled.
 * @param  TMRx Timer instance
 * @retval Inverted state of bit (0 or 1).
 */
__STATIC_INLINE uint32_t DDL_GTMR_IsEnabledUpdateEvent(GTMR_TypeDef *TMRx)
{
    return ((READ_BIT(TMRx->CR1, GTMR_CR1_UD) == (uint32_t)RESET) ? 1UL : 0UL);
}

/**
 * @brief  Set update event source
 * @note Update event source set to DDL_GTMR_UPDATESOURCE_REGULAR: any of the following events
 *       generate an update interrupt or DMA request if enabled:
 *        - Counter overflow/underflow
 *        - Setting the UG bit
 *        - Update generation through the slave mode controller
 * @note Update event source set to DDL_GTMR_UPDATESOURCE_COUNTER: only counter
 *       overflow/underflow generates an update interrupt or DMA request if enabled.
 * @param  TMRx Timer instance
 * @param  UpdateSource This parameter can be one of the following values:
 *         @arg @ref DDL_GTMR_UPDATESOURCE_REGULAR
 *         @arg @ref DDL_GTMR_UPDATESOURCE_COUNTER
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_SetUpdateSource(GTMR_TypeDef *TMRx, uint32_t UpdateSource)
{
    MODIFY_REG(TMRx->CR1, GTMR_CR1_URSSEL, UpdateSource);
}

/**
 * @brief  Get actual event update source
 * @param  TMRx Timer instance
 * @retval Returned value can be one of the following values:
 *         @arg @ref DDL_GTMR_UPDATESOURCE_REGULAR
 *         @arg @ref DDL_GTMR_UPDATESOURCE_COUNTER
 */
__STATIC_INLINE uint32_t DDL_GTMR_GetUpdateSource(GTMR_TypeDef *TMRx)
{
    return (uint32_t)(READ_BIT(TMRx->CR1, GTMR_CR1_URSSEL));
}

/**
 * @brief  Set one pulse mode (one shot v.s. repetitive).
 * @param  TMRx Timer instance
 * @param  OnePulseMode This parameter can be one of the following values:
 *         @arg @ref DDL_GTMR_ONEPULSEMODE_SINGLE
 *         @arg @ref DDL_GTMR_ONEPULSEMODE_REPETITIVE
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_SetOnePulseMode(GTMR_TypeDef *TMRx, uint32_t OnePulseMode)
{
    MODIFY_REG(TMRx->CR1, GTMR_CR1_SPMEN, OnePulseMode);
}

/**
 * @brief  Get actual one pulse mode.
 * @param  TMRx Timer instance
 * @retval Returned value can be one of the following values:
 *         @arg @ref DDL_GTMR_ONEPULSEMODE_SINGLE
 *         @arg @ref DDL_GTMR_ONEPULSEMODE_REPETITIVE
 */
__STATIC_INLINE uint32_t DDL_GTMR_GetOnePulseMode(GTMR_TypeDef *TMRx)
{
    return (uint32_t)(READ_BIT(TMRx->CR1, GTMR_CR1_SPMEN));
}

/**
 * @brief  Set the timer counter counting mode.
 * @note Macro IS_GTMR_COUNTER_MODE_SELECT_INSTANCE(TMRx) can be used to
 *       check whether or not the counter mode selection feature is supported
 *       by a timer instance.
 * @note Switching from Center Aligned counter mode to Edge counter mode (or reverse)
 *       requires a timer reset to avoid unexpected direction
 *       due to DIR bit readonly in center aligned mode.
 * @param  TMRx Timer instance
 * @param  CounterMode This parameter can be one of the following values:
 *         @arg @ref DDL_GTMR_COUNTERMODE_UP
 *         @arg @ref DDL_GTMR_COUNTERMODE_DOWN
 *         @arg @ref DDL_GTMR_COUNTERMODE_CENTER_UP
 *         @arg @ref DDL_GTMR_COUNTERMODE_CENTER_DOWN
 *         @arg @ref DDL_GTMR_COUNTERMODE_CENTER_UP_DOWN
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_SetCounterMode(GTMR_TypeDef *TMRx, uint32_t CounterMode)
{
    MODIFY_REG(TMRx->CR1, (GTMR_CR1_CNTDIR | GTMR_CR1_CAMSEL), CounterMode);
}

/**
 * @brief  Get actual counter mode.
 * @note Macro IS_GTMR_COUNTER_MODE_SELECT_INSTANCE(TMRx) can be used to
 *       check whether or not the counter mode selection feature is supported
 *       by a timer instance.
 * @param  TMRx Timer instance
 * @retval Returned value can be one of the following values:
 *         @arg @ref DDL_GTMR_COUNTERMODE_UP
 *         @arg @ref DDL_GTMR_COUNTERMODE_DOWN
 *         @arg @ref DDL_GTMR_COUNTERMODE_CENTER_UP
 *         @arg @ref DDL_GTMR_COUNTERMODE_CENTER_DOWN
 *         @arg @ref DDL_GTMR_COUNTERMODE_CENTER_UP_DOWN
 */
__STATIC_INLINE uint32_t DDL_GTMR_GetCounterMode(GTMR_TypeDef *TMRx)
{
    uint32_t counter_mode;

    counter_mode = (uint32_t)(READ_BIT(TMRx->CR1, GTMR_CR1_CAMSEL));

    if (counter_mode == 0U)
    {
        counter_mode = (uint32_t)(READ_BIT(TMRx->CR1, GTMR_CR1_CNTDIR));
    }

    return counter_mode;
}

/**
 * @brief  Get the current direction of the counter
 * @param  TMRx Timer instance
 * @retval Returned value can be one of the following values:
 *         @arg @ref DDL_GTMR_COUNTERDIRECTION_UP
 *         @arg @ref DDL_GTMR_COUNTERDIRECTION_DOWN
 */
__STATIC_INLINE uint32_t DDL_GTMR_GetDirection(GTMR_TypeDef *TMRx)
{
    return (uint32_t)(READ_BIT(TMRx->CR1, GTMR_CR1_CNTDIR));
}

/**
 * @brief  Enable auto-reload (ARR) preload.
 * @param  TMRx Timer instance
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_EnableARRPreload(GTMR_TypeDef *TMRx)
{
    SET_BIT(TMRx->CR1, GTMR_CR1_ARPEN);
}

/**
 * @brief  Disable auto-reload (ARR) preload.
 * @param  TMRx Timer instance
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_DisableARRPreload(GTMR_TypeDef *TMRx)
{
    CLEAR_BIT(TMRx->CR1, GTMR_CR1_ARPEN);
}

/**
 * @brief  Indicates whether auto-reload (ARR) preload is enabled.
 * @param  TMRx Timer instance
 * @retval State of bit (1 or 0).
 */
__STATIC_INLINE uint32_t DDL_GTMR_IsEnabledARRPreload(GTMR_TypeDef *TMRx)
{
    return ((READ_BIT(TMRx->CR1, GTMR_CR1_ARPEN) == (GTMR_CR1_ARPEN)) ? 1UL : 0UL);
}

/**
 * @brief  Set the division ratio between the timer clock  and the sampling clock used by the dead-time generators
 *         (when supported) and the digital filters.
 * @note Macro IS_GTMR_CLOCK_DIVISION_INSTANCE(TMRx) can be used to check
 *       whether or not the clock division feature is supported by the timer
 *       instance.
 * @param  TMRx Timer instance
 * @param  ClockDivision This parameter can be one of the following values:
 *         @arg @ref DDL_GTMR_CLOCKDIVISION_DIV1
 *         @arg @ref DDL_GTMR_CLOCKDIVISION_DIV2
 *         @arg @ref DDL_GTMR_CLOCKDIVISION_DIV4
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_SetClockDivision(GTMR_TypeDef *TMRx, uint32_t ClockDivision)
{
    MODIFY_REG(TMRx->CR1, GTMR_CR1_CLKDIV, ClockDivision);
}

/**
 * @brief  Get the actual division ratio between the timer clock  and the sampling clock used by the dead-time
 *         generators (when supported) and the digital filters.
 * @note Macro IS_GTMR_CLOCK_DIVISION_INSTANCE(TMRx) can be used to check
 *       whether or not the clock division feature is supported by the timer
 *       instance.
 * @param  TMRx Timer instance
 * @retval Returned value can be one of the following values:
 *         @arg @ref DDL_GTMR_CLOCKDIVISION_DIV1
 *         @arg @ref DDL_GTMR_CLOCKDIVISION_DIV2
 *         @arg @ref DDL_GTMR_CLOCKDIVISION_DIV4
 */
__STATIC_INLINE uint32_t DDL_GTMR_GetClockDivision(GTMR_TypeDef *TMRx)
{
    return (uint32_t)(READ_BIT(TMRx->CR1, GTMR_CR1_CLKDIV));
}

/**
 * @brief  Set the counter value.
 * @note Macro IS_GTMR_32B_COUNTER_INSTANCE(TMRx) can be used to check
 *       whether or not a timer instance supports a 32 bits counter.
 * @param  TMRx Timer instance
 * @param  Counter Counter value (between Min_Data=0 and Max_Data=0xFFFF)
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_SetCounter(GTMR_TypeDef *TMRx, uint32_t Counter)
{
    WRITE_REG(TMRx->CNT, Counter);
}

/**
 * @brief  Get the counter value.
 * @note Macro IS_GTMR_32B_COUNTER_INSTANCE(TMRx) can be used to check
 *       whether or not a timer instance supports a 32 bits counter.
 * @param  TMRx Timer instance
 * @retval Counter value (between Min_Data=0 and Max_Data=0xFFFF)
 */
__STATIC_INLINE uint32_t DDL_GTMR_GetCounter(GTMR_TypeDef *TMRx)
{
    return (uint32_t)(READ_REG(TMRx->CNT));
}

/**
 * @brief  Set the prescaler value.
 * @note The counter clock frequency CK_CNT is equal to fCK_PSC / (PSC[15:0] + 1).
 * @note The prescaler can be changed on the fly as this control register is buffered. The new
 *       prescaler ratio is taken into account at the next update event.
 * @note Helper macro @ref __DDL_GTMR_CALC_PSC can be used to calculate the Prescaler parameter
 * @param  TMRx Timer instance
 * @param  Prescaler between Min_Data=0 and Max_Data=65535
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_SetPrescaler(GTMR_TypeDef *TMRx, uint32_t Prescaler)
{
    WRITE_REG(TMRx->PSC, Prescaler);
}

/**
 * @brief  Get the prescaler value.
 * @param  TMRx Timer instance
 * @retval  Prescaler value between Min_Data=0 and Max_Data=65535
 */
__STATIC_INLINE uint32_t DDL_GTMR_GetPrescaler(GTMR_TypeDef *TMRx)
{
    return (uint32_t)(READ_REG(TMRx->PSC));
}

/**
 * @brief  Set the auto-reload value.
 * @note The counter is blocked while the auto-reload value is null.
 * @note Macro IS_GTMR_32B_COUNTER_INSTANCE(TMRx) can be used to check
 *       whether or not a timer instance supports a 32 bits counter.
 * @note Helper macro @ref __DDL_GTMR_CALC_ARR can be used to calculate the AutoReload parameter
 * @param  TMRx Timer instance
 * @param  AutoReload between Min_Data=0 and Max_Data=65535
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_SetAutoReload(GTMR_TypeDef *TMRx, uint32_t AutoReload)
{
    WRITE_REG(TMRx->AUTORLD, AutoReload);
}

/**
 * @brief  Get the auto-reload value.
 * @note Macro IS_GTMR_32B_COUNTER_INSTANCE(TMRx) can be used to check
 *       whether or not a timer instance supports a 32 bits counter.
 * @param  TMRx Timer instance
 * @retval Auto-reload value
 */
__STATIC_INLINE uint32_t DDL_GTMR_GetAutoReload(GTMR_TypeDef *TMRx)
{
    return (uint32_t)(READ_REG(TMRx->AUTORLD));
}

/**
 * @}
 */

/** @defgroup GTMR_DDL_EF_Capture_Compare Capture Compare configuration
 * @{
 */
/**
 * @brief  Set the trigger of the capture/compare DMA request.
 * @param  TMRx Timer instance
 * @param  DMAReqTrigger This parameter can be one of the following values:
 *         @arg @ref DDL_GTMR_CCDMAREQUEST_CC
 *         @arg @ref DDL_GTMR_CCDMAREQUEST_UPDATE
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_CC_SetDMAReqTrigger(GTMR_TypeDef *TMRx, uint32_t DMAReqTrigger)
{
    MODIFY_REG(TMRx->CR2, GTMR_CR2_CCDSEL, DMAReqTrigger);
}

/**
 * @brief  Get actual trigger of the capture/compare DMA request.
 * @param  TMRx Timer instance
 * @retval Returned value can be one of the following values:
 *         @arg @ref DDL_GTMR_CCDMAREQUEST_CC
 *         @arg @ref DDL_GTMR_CCDMAREQUEST_UPDATE
 */
__STATIC_INLINE uint32_t DDL_GTMR_CC_GetDMAReqTrigger(GTMR_TypeDef *TMRx)
{
    return (uint32_t)(READ_BIT(TMRx->CR2, GTMR_CR2_CCDSEL));
}

/**
 * @brief  Enable capture/compare channels.
 * @param  TMRx Timer instance
 * @param  Channels This parameter can be a combination of the following values:
 *         @arg @ref DDL_GTMR_CHANNEL_CH0
 *         @arg @ref DDL_GTMR_CHANNEL_CH1
 *         @arg @ref DDL_GTMR_CHANNEL_CH2
 *         @arg @ref DDL_GTMR_CHANNEL_CH3
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_CC_EnableChannel(GTMR_TypeDef *TMRx, uint32_t Channels)
{
    SET_BIT(TMRx->CCEN, Channels);
}

/**
 * @brief  Disable capture/compare channels.
 * @param  TMRx Timer instance
 * @param  Channels This parameter can be a combination of the following values:
 *         @arg @ref DDL_GTMR_CHANNEL_CH0
 *         @arg @ref DDL_GTMR_CHANNEL_CH1
 *         @arg @ref DDL_GTMR_CHANNEL_CH2
 *         @arg @ref DDL_GTMR_CHANNEL_CH3
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_CC_DisableChannel(GTMR_TypeDef *TMRx, uint32_t Channels)
{
    CLEAR_BIT(TMRx->CCEN, Channels);
}

/**
 * @brief  Indicate whether channel(s) is(are) enabled.
 * @param  TMRx Timer instance
 * @param  Channels This parameter can be a combination of the following values:
 *         @arg @ref DDL_GTMR_CHANNEL_CH0
 *         @arg @ref DDL_GTMR_CHANNEL_CH1
 *         @arg @ref DDL_GTMR_CHANNEL_CH2
 *         @arg @ref DDL_GTMR_CHANNEL_CH3
 * @retval State of bit (1 or 0).
 */
__STATIC_INLINE uint32_t DDL_GTMR_CC_IsEnabledChannel(GTMR_TypeDef *TMRx, uint32_t Channels)
{
    return ((READ_BIT(TMRx->CCEN, Channels) == (Channels)) ? 1UL : 0UL);
}

/**
 * @}
 */

/** @defgroup GTMR_DDL_EF_Output_Channel Output channel configuration
 * @{
 */
/**
 * @brief  Configure an output channel.
 * @param  TMRx Timer instance
 * @param  Channel This parameter can be one of the following values:
 *         @arg @ref DDL_GTMR_CHANNEL_CH0
 *         @arg @ref DDL_GTMR_CHANNEL_CH1
 *         @arg @ref DDL_GTMR_CHANNEL_CH2
 *         @arg @ref DDL_GTMR_CHANNEL_CH3
 * @param  Configuration This parameter must be a combination of all the following values:
 *         @arg @ref DDL_GTMR_OCPOLARITY_HIGH or @ref DDL_GTMR_OCPOLARITY_LOW
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_OC_ConfigOutput(GTMR_TypeDef *TMRx, uint32_t Channel, uint32_t Configuration)
{
    uint8_t iChannel = GTMR_GET_CHANNEL_INDEX(Channel);
    __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&TMRx->CCM1) + DDL_GTMR_OFFSET_TAB_CCMRx[iChannel]));
    CLEAR_BIT(*pReg, (GTMR_CCM1_CC0SEL << DDL_GTMR_SHIFT_TAB_OCxx[iChannel]));
    MODIFY_REG(TMRx->CCEN, (GTMR_CCEN_CC0POL << DDL_GTMR_SHIFT_TAB_CCxP[iChannel]),
                (Configuration & GTMR_CCEN_CC0POL) << DDL_GTMR_SHIFT_TAB_CCxP[iChannel]);
}

/**
 * @brief  Define the behavior of the output reference signal OCxREF from which
 *         OCx and OCxN (when relevant) are derived.
 * @param  TMRx Timer instance
 * @param  Channel This parameter can be one of the following values:
 *         @arg @ref DDL_GTMR_CHANNEL_CH0
 *         @arg @ref DDL_GTMR_CHANNEL_CH1
 *         @arg @ref DDL_GTMR_CHANNEL_CH2
 *         @arg @ref DDL_GTMR_CHANNEL_CH3
 * @param  Mode This parameter can be one of the following values:
 *         @arg @ref DDL_GTMR_OCMODE_FROZEN
 *         @arg @ref DDL_GTMR_OCMODE_ACTIVE
 *         @arg @ref DDL_GTMR_OCMODE_INACTIVE
 *         @arg @ref DDL_GTMR_OCMODE_TOGGLE
 *         @arg @ref DDL_GTMR_OCMODE_FORCED_INACTIVE
 *         @arg @ref DDL_GTMR_OCMODE_FORCED_ACTIVE
 *         @arg @ref DDL_GTMR_OCMODE_PWM1
 *         @arg @ref DDL_GTMR_OCMODE_PWM2
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_OC_SetMode(GTMR_TypeDef *TMRx, uint32_t Channel, uint32_t Mode)
{
    uint8_t iChannel = GTMR_GET_CHANNEL_INDEX(Channel);
    __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&TMRx->CCM1) + DDL_GTMR_OFFSET_TAB_CCMRx[iChannel]));
    MODIFY_REG(*pReg, ((GTMR_CCM1_OC0MOD | GTMR_CCM1_CC0SEL) << DDL_GTMR_SHIFT_TAB_OCxx[iChannel]), Mode << DDL_GTMR_SHIFT_TAB_OCxx[iChannel]);
}

/**
 * @brief  Get the output compare mode of an output channel.
 * @param  TMRx Timer instance
 * @param  Channel This parameter can be one of the following values:
 *         @arg @ref DDL_GTMR_CHANNEL_CH0
 *         @arg @ref DDL_GTMR_CHANNEL_CH1
 *         @arg @ref DDL_GTMR_CHANNEL_CH2
 *         @arg @ref DDL_GTMR_CHANNEL_CH3
 * @retval Returned value can be one of the following values:
 *         @arg @ref DDL_GTMR_OCMODE_FROZEN
 *         @arg @ref DDL_GTMR_OCMODE_ACTIVE
 *         @arg @ref DDL_GTMR_OCMODE_INACTIVE
 *         @arg @ref DDL_GTMR_OCMODE_TOGGLE
 *         @arg @ref DDL_GTMR_OCMODE_FORCED_INACTIVE
 *         @arg @ref DDL_GTMR_OCMODE_FORCED_ACTIVE
 *         @arg @ref DDL_GTMR_OCMODE_PWM1
 *         @arg @ref DDL_GTMR_OCMODE_PWM2
 */
__STATIC_INLINE uint32_t DDL_GTMR_OC_GetMode(GTMR_TypeDef *TMRx, uint32_t Channel)
{
    uint8_t iChannel = GTMR_GET_CHANNEL_INDEX(Channel);
    const __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&TMRx->CCM1) + DDL_GTMR_OFFSET_TAB_CCMRx[iChannel]));
    return (READ_BIT(*pReg, ((GTMR_CCM1_OC0MOD | GTMR_CCM1_CC0SEL) << DDL_GTMR_SHIFT_TAB_OCxx[iChannel])) >> DDL_GTMR_SHIFT_TAB_OCxx[iChannel]);
}

/**
 * @brief  Set the polarity of an output channel.
 * @param  TMRx Timer instance
 * @param  Channel This parameter can be one of the following values:
 *         @arg @ref DDL_GTMR_CHANNEL_CH0
 *         @arg @ref DDL_GTMR_CHANNEL_CH1
 *         @arg @ref DDL_GTMR_CHANNEL_CH2
 *         @arg @ref DDL_GTMR_CHANNEL_CH3
 * @param  Polarity This parameter can be one of the following values:
 *         @arg @ref DDL_GTMR_OCPOLARITY_HIGH
 *         @arg @ref DDL_GTMR_OCPOLARITY_LOW
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_OC_SetPolarity(GTMR_TypeDef *TMRx, uint32_t Channel, uint32_t Polarity)
{
    uint8_t iChannel = GTMR_GET_CHANNEL_INDEX(Channel);
    MODIFY_REG(TMRx->CCEN, (GTMR_CCEN_CC0POL << DDL_GTMR_SHIFT_TAB_CCxP[iChannel]), Polarity << DDL_GTMR_SHIFT_TAB_CCxP[iChannel]);
}

/**
 * @brief  Get the polarity of an output channel.
 * @param  TMRx Timer instance
 * @param  Channel This parameter can be one of the following values:
 *         @arg @ref DDL_GTMR_CHANNEL_CH0
 *         @arg @ref DDL_GTMR_CHANNEL_CH1
 *         @arg @ref DDL_GTMR_CHANNEL_CH2
 *         @arg @ref DDL_GTMR_CHANNEL_CH3
 * @retval Returned value can be one of the following values:
 *         @arg @ref DDL_GTMR_OCPOLARITY_HIGH
 *         @arg @ref DDL_GTMR_OCPOLARITY_LOW
 */
__STATIC_INLINE uint32_t DDL_GTMR_OC_GetPolarity(GTMR_TypeDef *TMRx, uint32_t Channel)
{
    uint8_t iChannel = GTMR_GET_CHANNEL_INDEX(Channel);
    return (READ_BIT(TMRx->CCEN, (GTMR_CCEN_CC0POL << DDL_GTMR_SHIFT_TAB_CCxP[iChannel])) >> DDL_GTMR_SHIFT_TAB_CCxP[iChannel]);
}

/**
 * @brief  Enable compare register (TMRx_CCx) preload for the output channel.
 * @param  TMRx Timer instance
 * @param  Channel This parameter can be one of the following values:
 *         @arg @ref DDL_GTMR_CHANNEL_CH0
 *         @arg @ref DDL_GTMR_CHANNEL_CH1
 *         @arg @ref DDL_GTMR_CHANNEL_CH2
 *         @arg @ref DDL_GTMR_CHANNEL_CH3
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_OC_EnablePreload(GTMR_TypeDef *TMRx, uint32_t Channel)
{
    uint8_t iChannel = GTMR_GET_CHANNEL_INDEX(Channel);
    __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&TMRx->CCM1) + DDL_GTMR_OFFSET_TAB_CCMRx[iChannel]));
    SET_BIT(*pReg, (GTMR_CCM1_OC0PEN << DDL_GTMR_SHIFT_TAB_OCxx[iChannel]));
}

/**
 * @brief  Disable compare register (TMRx_CCx) preload for the output channel.
 * @param  TMRx Timer instance
 * @param  Channel This parameter can be one of the following values:
 *         @arg @ref DDL_GTMR_CHANNEL_CH0
 *         @arg @ref DDL_GTMR_CHANNEL_CH1
 *         @arg @ref DDL_GTMR_CHANNEL_CH2
 *         @arg @ref DDL_GTMR_CHANNEL_CH3
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_OC_DisablePreload(GTMR_TypeDef *TMRx, uint32_t Channel)
{
    uint8_t iChannel = GTMR_GET_CHANNEL_INDEX(Channel);
    __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&TMRx->CCM1) + DDL_GTMR_OFFSET_TAB_CCMRx[iChannel]));
    CLEAR_BIT(*pReg, (GTMR_CCM1_OC0PEN << DDL_GTMR_SHIFT_TAB_OCxx[iChannel]));
}

/**
 * @brief  Indicates whether compare register (TMRx_CCx) preload is enabled for the output channel.
 * @param  TMRx Timer instance
 * @param  Channel This parameter can be one of the following values:
 *         @arg @ref DDL_GTMR_CHANNEL_CH0
 *         @arg @ref DDL_GTMR_CHANNEL_CH1
 *         @arg @ref DDL_GTMR_CHANNEL_CH2
 *         @arg @ref DDL_GTMR_CHANNEL_CH3
 * @retval State of bit (1 or 0).
 */
__STATIC_INLINE uint32_t DDL_GTMR_OC_IsEnabledPreload(GTMR_TypeDef *TMRx, uint32_t Channel)
{
    uint8_t iChannel = GTMR_GET_CHANNEL_INDEX(Channel);
    const __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&TMRx->CCM1) + DDL_GTMR_OFFSET_TAB_CCMRx[iChannel]));
    uint32_t bitfield = GTMR_CCM1_OC0PEN << DDL_GTMR_SHIFT_TAB_OCxx[iChannel];
    return ((READ_BIT(*pReg, bitfield) == bitfield) ? 1UL : 0UL);
}

/**
 * @brief  Enable clearing the output channel on an external event.
 * @note This function can only be used in Output compare and PWM modes. It does not work in Forced mode.
 * @note Macro IS_GTMR_OCXREF_CLEAR_INSTANCE(TMRx) can be used to check whether
 *       or not a timer instance can clear the OCxREF signal on an external event.
 * @param  TMRx Timer instance
 * @param  Channel This parameter can be one of the following values:
 *         @arg @ref DDL_GTMR_CHANNEL_CH0
 *         @arg @ref DDL_GTMR_CHANNEL_CH1
 *         @arg @ref DDL_GTMR_CHANNEL_CH2
 *         @arg @ref DDL_GTMR_CHANNEL_CH3
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_OC_EnableClear(GTMR_TypeDef *TMRx, uint32_t Channel)
{
    uint8_t iChannel = GTMR_GET_CHANNEL_INDEX(Channel);
    __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&TMRx->CCM1) + DDL_GTMR_OFFSET_TAB_CCMRx[iChannel]));
    SET_BIT(*pReg, (GTMR_CCM1_OC0CEN << DDL_GTMR_SHIFT_TAB_OCxx[iChannel]));
}

/**
 * @brief  Disable clearing the output channel on an external event.
 * @note Macro IS_GTMR_OCXREF_CLEAR_INSTANCE(TMRx) can be used to check whether
 *       or not a timer instance can clear the OCxREF signal on an external event.
 * @param  TMRx Timer instance
 * @param  Channel This parameter can be one of the following values:
 *         @arg @ref DDL_GTMR_CHANNEL_CH0
 *         @arg @ref DDL_GTMR_CHANNEL_CH1
 *         @arg @ref DDL_GTMR_CHANNEL_CH2
 *         @arg @ref DDL_GTMR_CHANNEL_CH3
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_OC_DisableClear(GTMR_TypeDef *TMRx, uint32_t Channel)
{
    uint8_t iChannel = GTMR_GET_CHANNEL_INDEX(Channel);
    __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&TMRx->CCM1) + DDL_GTMR_OFFSET_TAB_CCMRx[iChannel]));
    CLEAR_BIT(*pReg, (GTMR_CCM1_OC0CEN << DDL_GTMR_SHIFT_TAB_OCxx[iChannel]));
}

/**
 * @brief  Indicates clearing the output channel on an external event is enabled for the output channel.
 * @note This function enables clearing the output channel on an external event.
 * @note This function can only be used in Output compare and PWM modes. It does not work in Forced mode.
 * @note Macro IS_GTMR_OCXREF_CLEAR_INSTANCE(TMRx) can be used to check whether
 *       or not a timer instance can clear the OCxREF signal on an external event.
 * @param  TMRx Timer instance
 * @param  Channel This parameter can be one of the following values:
 *         @arg @ref DDL_GTMR_CHANNEL_CH0
 *         @arg @ref DDL_GTMR_CHANNEL_CH1
 *         @arg @ref DDL_GTMR_CHANNEL_CH2
 *         @arg @ref DDL_GTMR_CHANNEL_CH3
 * @retval State of bit (1 or 0).
 */
__STATIC_INLINE uint32_t DDL_GTMR_OC_IsEnabledClear(GTMR_TypeDef *TMRx, uint32_t Channel)
{
    uint8_t iChannel = GTMR_GET_CHANNEL_INDEX(Channel);
    const __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&TMRx->CCM1) + DDL_GTMR_OFFSET_TAB_CCMRx[iChannel]));
    uint32_t bitfield = GTMR_CCM1_OC0CEN << DDL_GTMR_SHIFT_TAB_OCxx[iChannel];
    return ((READ_BIT(*pReg, bitfield) == bitfield) ? 1UL : 0UL);
}

/**
 * @brief  Set compare value for output channel 0 (TMRx_CC0).
 * @note In 32-bit timer implementations compare value can be between 0x00000000 and 0xFFFFFFFF.
 * @note Macro IS_GTMR_32B_COUNTER_INSTANCE(TMRx) can be used to check
 *       whether or not a timer instance supports a 32 bits counter.
 * @note Macro IS_GTMR_CC0_INSTANCE(TMRx) can be used to check whether or not
 *       output channel 0 is supported by a timer instance.
 * @param  TMRx Timer instance
 * @param  CompareValue between Min_Data=0 and Max_Data=65535
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_OC_SetCompareCH0(GTMR_TypeDef *TMRx, uint32_t CompareValue)
{
    WRITE_REG(TMRx->CC0, CompareValue);
}

/**
 * @brief  Set compare value for output channel 1 (TMRx_CC1).
 * @note In 32-bit timer implementations compare value can be between 0x00000000 and 0xFFFFFFFF.
 * @note Macro IS_GTMR_32B_COUNTER_INSTANCE(TMRx) can be used to check
 *       whether or not a timer instance supports a 32 bits counter.
 * @note Macro IS_GTMR_CC1_INSTANCE(TMRx) can be used to check whether or not
 *       output channel 1 is supported by a timer instance.
 * @param  TMRx Timer instance
 * @param  CompareValue between Min_Data=0 and Max_Data=65535
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_OC_SetCompareCH1(GTMR_TypeDef *TMRx, uint32_t CompareValue)
{
    WRITE_REG(TMRx->CC1, CompareValue);
}

/**
 * @brief  Set compare value for output channel 2 (TMRx_CC2).
 * @note In 32-bit timer implementations compare value can be between 0x00000000 and 0xFFFFFFFF.
 * @note Macro IS_GTMR_32B_COUNTER_INSTANCE(TMRx) can be used to check
 *       whether or not a timer instance supports a 32 bits counter.
 * @note Macro IS_GTMR_CC2_INSTANCE(TMRx) can be used to check whether or not
 *       output channel is supported by a timer instance.
 * @param  TMRx Timer instance
 * @param  CompareValue between Min_Data=0 and Max_Data=65535
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_OC_SetCompareCH2(GTMR_TypeDef *TMRx, uint32_t CompareValue)
{
    WRITE_REG(TMRx->CC2, CompareValue);
}

/**
 * @brief  Set compare value for output channel 3 (TMRx_CC3).
 * @note In 32-bit timer implementations compare value can be between 0x00000000 and 0xFFFFFFFF.
 * @note Macro IS_GTMR_32B_COUNTER_INSTANCE(TMRx) can be used to check
 *       whether or not a timer instance supports a 32 bits counter.
 * @note Macro IS_GTMR_CC3_INSTANCE(TMRx) can be used to check whether or not
 *       output channel 3 is supported by a timer instance.
 * @param  TMRx Timer instance
 * @param  CompareValue between Min_Data=0 and Max_Data=65535
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_OC_SetCompareCH3(GTMR_TypeDef *TMRx, uint32_t CompareValue)
{
    WRITE_REG(TMRx->CC3, CompareValue);
}

/**
 * @brief  Get compare value (TMRx_CC0) set for  output channel 0.
 * @note In 32-bit timer implementations returned compare value can be between 0x00000000 and 0xFFFFFFFF.
 * @note Macro IS_GTMR_32B_COUNTER_INSTANCE(TMRx) can be used to check
 *       whether or not a timer instance supports a 32 bits counter.
 * @note Macro IS_GTMR_CC0_INSTANCE(TMRx) can be used to check whether or not
 *       output channel 0 is supported by a timer instance.
 * @param  TMRx Timer instance
 * @retval CompareValue (between Min_Data=0 and Max_Data=65535)
 */
__STATIC_INLINE uint32_t DDL_GTMR_OC_GetCompareCH0(GTMR_TypeDef *TMRx)
{
    return (uint32_t)(READ_REG(TMRx->CC0));
}

/**
 * @brief  Get compare value (TMRx_CC1) set for  output channel 1.
 * @note In 32-bit timer implementations returned compare value can be between 0x00000000 and 0xFFFFFFFF.
 * @note Macro IS_GTMR_32B_COUNTER_INSTANCE(TMRx) can be used to check
 *       whether or not a timer instance supports a 32 bits counter.
 * @note Macro IS_GTMR_CC1_INSTANCE(TMRx) can be used to check whether or not
 *       output channel 1 is supported by a timer instance.
 * @param  TMRx Timer instance
 * @retval CompareValue (between Min_Data=0 and Max_Data=65535)
 */
__STATIC_INLINE uint32_t DDL_GTMR_OC_GetCompareCH1(GTMR_TypeDef *TMRx)
{
    return (uint32_t)(READ_REG(TMRx->CC1));
}

/**
 * @brief  Get compare value (TMRx_CC2) set for  output channel 2.
 * @note In 32-bit timer implementations returned compare value can be between 0x00000000 and 0xFFFFFFFF.
 * @note Macro IS_GTMR_32B_COUNTER_INSTANCE(TMRx) can be used to check
 *       whether or not a timer instance supports a 32 bits counter.
 * @note Macro IS_GTMR_CC2_INSTANCE(TMRx) can be used to check whether or not
 *       output channel 2 is supported by a timer instance.
 * @param  TMRx Timer instance
 * @retval CompareValue (between Min_Data=0 and Max_Data=65535)
 */
__STATIC_INLINE uint32_t DDL_GTMR_OC_GetCompareCH2(GTMR_TypeDef *TMRx)
{
    return (uint32_t)(READ_REG(TMRx->CC2));
}

/**
 * @brief  Get compare value (TMRx_CC3) set for  output channel 3.
 * @note In 32-bit timer implementations returned compare value can be between 0x00000000 and 0xFFFFFFFF.
 * @note Macro IS_GTMR_32B_COUNTER_INSTANCE(TMRx) can be used to check
 *       whether or not a timer instance supports a 32 bits counter.
 * @note Macro IS_GTMR_CC3_INSTANCE(TMRx) can be used to check whether or not
 *       output channel 3 is supported by a timer instance.
 * @param  TMRx Timer instance
 * @retval CompareValue (between Min_Data=0 and Max_Data=65535)
 */
__STATIC_INLINE uint32_t DDL_GTMR_OC_GetCompareCH3(GTMR_TypeDef *TMRx)
{
    return (uint32_t)(READ_REG(TMRx->CC3));
}

/**
 * @}
 */

/** @defgroup GTMR_DDL_EF_Input_Channel Input channel configuration
 * @{
 */
/**
 * @brief  Configure input channel.
 * @param  TMRx Timer instance
 * @param  Channel This parameter can be one of the following values:
 *         @arg @ref DDL_GTMR_CHANNEL_CH0
 *         @arg @ref DDL_GTMR_CHANNEL_CH1
 *         @arg @ref DDL_GTMR_CHANNEL_CH2
 *         @arg @ref DDL_GTMR_CHANNEL_CH3
 * @param  Configuration This parameter must be a combination of all the following values:
 *         @arg @ref DDL_GTMR_ACTIVEINPUT_DIRECTTI or @ref DDL_GTMR_ACTIVEINPUT_INDIRECTTI or @ref DDL_GTMR_ACTIVEINPUT_TRC
 *         @arg @ref DDL_GTMR_ICPSC_DIV1 or ... or @ref DDL_GTMR_ICPSC_DIV8
 *         @arg @ref DDL_GTMR_IC_FILTER_FDIV1 or ... or @ref DDL_GTMR_IC_FILTER_FDIV32_N8
 *         @arg @ref DDL_GTMR_IC_POLARITY_RISING or @ref DDL_GTMR_IC_POLARITY_FALLING
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_IC_Config(GTMR_TypeDef *TMRx, uint32_t Channel, uint32_t Configuration)
{
    uint8_t iChannel = GTMR_GET_CHANNEL_INDEX(Channel);
    __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&TMRx->CCM1) + DDL_GTMR_OFFSET_TAB_CCMRx[iChannel]));
    MODIFY_REG(*pReg, ((GTMR_CCM1_IC0F | GTMR_CCM1_IC0PSC | GTMR_CCM1_CC0SEL) << DDL_GTMR_SHIFT_TAB_ICxx[iChannel]),
                ((Configuration >> 16U) & (GTMR_CCM1_IC0F | GTMR_CCM1_IC0PSC | GTMR_CCM1_CC0SEL))
                    << DDL_GTMR_SHIFT_TAB_ICxx[iChannel]);
    MODIFY_REG(TMRx->CCEN, (GTMR_CCEN_CC0POL << DDL_GTMR_SHIFT_TAB_CCxP[iChannel]),
                (Configuration & GTMR_CCEN_CC0POL) << DDL_GTMR_SHIFT_TAB_CCxP[iChannel]);
}

/**
 * @brief  Set the active input.
 * @param  TMRx Timer instance
 * @param  Channel This parameter can be one of the following values:
 *         @arg @ref DDL_GTMR_CHANNEL_CH0
 *         @arg @ref DDL_GTMR_CHANNEL_CH1
 *         @arg @ref DDL_GTMR_CHANNEL_CH2
 *         @arg @ref DDL_GTMR_CHANNEL_CH3
 * @param  ICActiveInput This parameter can be one of the following values:
 *         @arg @ref DDL_GTMR_ACTIVEINPUT_DIRECTTI
 *         @arg @ref DDL_GTMR_ACTIVEINPUT_INDIRECTTI
 *         @arg @ref DDL_GTMR_ACTIVEINPUT_TRC
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_IC_SetActiveInput(GTMR_TypeDef *TMRx, uint32_t Channel, uint32_t ICActiveInput)
{
    uint8_t iChannel = GTMR_GET_CHANNEL_INDEX(Channel);
    __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&TMRx->CCM1) + DDL_GTMR_OFFSET_TAB_CCMRx[iChannel]));
    MODIFY_REG(*pReg, ((GTMR_CCM1_CC0SEL) << DDL_GTMR_SHIFT_TAB_ICxx[iChannel]), (ICActiveInput >> 16U) << DDL_GTMR_SHIFT_TAB_ICxx[iChannel]);
}

/**
 * @brief  Get the current active input.
 * @param  TMRx Timer instance
 * @param  Channel This parameter can be one of the following values:
 *         @arg @ref DDL_GTMR_CHANNEL_CH0
 *         @arg @ref DDL_GTMR_CHANNEL_CH1
 *         @arg @ref DDL_GTMR_CHANNEL_CH2
 *         @arg @ref DDL_GTMR_CHANNEL_CH3
 * @retval Returned value can be one of the following values:
 *         @arg @ref DDL_GTMR_ACTIVEINPUT_DIRECTTI
 *         @arg @ref DDL_GTMR_ACTIVEINPUT_INDIRECTTI
 *         @arg @ref DDL_GTMR_ACTIVEINPUT_TRC
 */
__STATIC_INLINE uint32_t DDL_GTMR_IC_GetActiveInput(GTMR_TypeDef *TMRx, uint32_t Channel)
{
    uint8_t iChannel = GTMR_GET_CHANNEL_INDEX(Channel);
    const __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&TMRx->CCM1) + DDL_GTMR_OFFSET_TAB_CCMRx[iChannel]));
    return ((READ_BIT(*pReg, ((GTMR_CCM1_CC0SEL) << DDL_GTMR_SHIFT_TAB_ICxx[iChannel])) >> DDL_GTMR_SHIFT_TAB_ICxx[iChannel]) << 16U);
}

/**
 * @brief  Set the prescaler of input channel.
 * @param  TMRx Timer instance
 * @param  Channel This parameter can be one of the following values:
 *         @arg @ref DDL_GTMR_CHANNEL_CH0
 *         @arg @ref DDL_GTMR_CHANNEL_CH1
 *         @arg @ref DDL_GTMR_CHANNEL_CH2
 *         @arg @ref DDL_GTMR_CHANNEL_CH3
 * @param  ICPrescaler This parameter can be one of the following values:
 *         @arg @ref DDL_GTMR_ICPSC_DIV1
 *         @arg @ref DDL_GTMR_ICPSC_DIV2
 *         @arg @ref DDL_GTMR_ICPSC_DIV4
 *         @arg @ref DDL_GTMR_ICPSC_DIV8
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_IC_SetPrescaler(GTMR_TypeDef *TMRx, uint32_t Channel, uint32_t ICPrescaler)
{
    uint8_t iChannel = GTMR_GET_CHANNEL_INDEX(Channel);
    __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&TMRx->CCM1) + DDL_GTMR_OFFSET_TAB_CCMRx[iChannel]));
    MODIFY_REG(*pReg, ((GTMR_CCM1_IC0PSC) << DDL_GTMR_SHIFT_TAB_ICxx[iChannel]), (ICPrescaler >> 16U) << DDL_GTMR_SHIFT_TAB_ICxx[iChannel]);
}

/**
 * @brief  Get the current prescaler value acting on an  input channel.
 * @param  TMRx Timer instance
 * @param  Channel This parameter can be one of the following values:
 *         @arg @ref DDL_GTMR_CHANNEL_CH0
 *         @arg @ref DDL_GTMR_CHANNEL_CH1
 *         @arg @ref DDL_GTMR_CHANNEL_CH2
 *         @arg @ref DDL_GTMR_CHANNEL_CH3
 * @retval Returned value can be one of the following values:
 *         @arg @ref DDL_GTMR_ICPSC_DIV1
 *         @arg @ref DDL_GTMR_ICPSC_DIV2
 *         @arg @ref DDL_GTMR_ICPSC_DIV4
 *         @arg @ref DDL_GTMR_ICPSC_DIV8
 */
__STATIC_INLINE uint32_t DDL_GTMR_IC_GetPrescaler(GTMR_TypeDef *TMRx, uint32_t Channel)
{
    uint8_t iChannel = GTMR_GET_CHANNEL_INDEX(Channel);
    const __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&TMRx->CCM1) + DDL_GTMR_OFFSET_TAB_CCMRx[iChannel]));
    return ((READ_BIT(*pReg, ((GTMR_CCM1_IC0PSC) << DDL_GTMR_SHIFT_TAB_ICxx[iChannel])) >> DDL_GTMR_SHIFT_TAB_ICxx[iChannel]) << 16U);
}

/**
 * @brief  Set the input filter duration.
 * @param  TMRx Timer instance
 * @param  Channel This parameter can be one of the following values:
 *         @arg @ref DDL_GTMR_CHANNEL_CH0
 *         @arg @ref DDL_GTMR_CHANNEL_CH1
 *         @arg @ref DDL_GTMR_CHANNEL_CH2
 *         @arg @ref DDL_GTMR_CHANNEL_CH3
 * @param  ICFilter This parameter can be one of the following values:
 *         @arg @ref DDL_GTMR_IC_FILTER_FDIV1
 *         @arg @ref DDL_GTMR_IC_FILTER_FDIV1_N2
 *         @arg @ref DDL_GTMR_IC_FILTER_FDIV1_N4
 *         @arg @ref DDL_GTMR_IC_FILTER_FDIV1_N8
 *         @arg @ref DDL_GTMR_IC_FILTER_FDIV2_N6
 *         @arg @ref DDL_GTMR_IC_FILTER_FDIV2_N8
 *         @arg @ref DDL_GTMR_IC_FILTER_FDIV4_N6
 *         @arg @ref DDL_GTMR_IC_FILTER_FDIV4_N8
 *         @arg @ref DDL_GTMR_IC_FILTER_FDIV8_N6
 *         @arg @ref DDL_GTMR_IC_FILTER_FDIV8_N8
 *         @arg @ref DDL_GTMR_IC_FILTER_FDIV16_N5
 *         @arg @ref DDL_GTMR_IC_FILTER_FDIV16_N6
 *         @arg @ref DDL_GTMR_IC_FILTER_FDIV16_N8
 *         @arg @ref DDL_GTMR_IC_FILTER_FDIV32_N5
 *         @arg @ref DDL_GTMR_IC_FILTER_FDIV32_N6
 *         @arg @ref DDL_GTMR_IC_FILTER_FDIV32_N8
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_IC_SetFilter(GTMR_TypeDef *TMRx, uint32_t Channel, uint32_t ICFilter)
{
    uint8_t iChannel = GTMR_GET_CHANNEL_INDEX(Channel);
    __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&TMRx->CCM1) + DDL_GTMR_OFFSET_TAB_CCMRx[iChannel]));
    MODIFY_REG(*pReg, ((GTMR_CCM1_IC0F) << DDL_GTMR_SHIFT_TAB_ICxx[iChannel]), (ICFilter >> 16U) << DDL_GTMR_SHIFT_TAB_ICxx[iChannel]);
}

/**
 * @brief  Get the input filter duration.
 * @param  TMRx Timer instance
 * @param  Channel This parameter can be one of the following values:
 *         @arg @ref DDL_GTMR_CHANNEL_CH0
 *         @arg @ref DDL_GTMR_CHANNEL_CH1
 *         @arg @ref DDL_GTMR_CHANNEL_CH2
 *         @arg @ref DDL_GTMR_CHANNEL_CH3
 * @retval Returned value can be one of the following values:
 *         @arg @ref DDL_GTMR_IC_FILTER_FDIV1
 *         @arg @ref DDL_GTMR_IC_FILTER_FDIV1_N2
 *         @arg @ref DDL_GTMR_IC_FILTER_FDIV1_N4
 *         @arg @ref DDL_GTMR_IC_FILTER_FDIV1_N8
 *         @arg @ref DDL_GTMR_IC_FILTER_FDIV2_N6
 *         @arg @ref DDL_GTMR_IC_FILTER_FDIV2_N8
 *         @arg @ref DDL_GTMR_IC_FILTER_FDIV4_N6
 *         @arg @ref DDL_GTMR_IC_FILTER_FDIV4_N8
 *         @arg @ref DDL_GTMR_IC_FILTER_FDIV8_N6
 *         @arg @ref DDL_GTMR_IC_FILTER_FDIV8_N8
 *         @arg @ref DDL_GTMR_IC_FILTER_FDIV16_N5
 *         @arg @ref DDL_GTMR_IC_FILTER_FDIV16_N6
 *         @arg @ref DDL_GTMR_IC_FILTER_FDIV16_N8
 *         @arg @ref DDL_GTMR_IC_FILTER_FDIV32_N5
 *         @arg @ref DDL_GTMR_IC_FILTER_FDIV32_N6
 *         @arg @ref DDL_GTMR_IC_FILTER_FDIV32_N8
 */
__STATIC_INLINE uint32_t DDL_GTMR_IC_GetFilter(GTMR_TypeDef *TMRx, uint32_t Channel)
{
    uint8_t iChannel = GTMR_GET_CHANNEL_INDEX(Channel);
    const __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&TMRx->CCM1) + DDL_GTMR_OFFSET_TAB_CCMRx[iChannel]));
    return ((READ_BIT(*pReg, ((GTMR_CCM1_IC0F) << DDL_GTMR_SHIFT_TAB_ICxx[iChannel])) >> DDL_GTMR_SHIFT_TAB_ICxx[iChannel]) << 16U);
}

/**
 * @brief  Set the input channel polarity.
 * @param  TMRx Timer instance
 * @param  Channel This parameter can be one of the following values:
 *         @arg @ref DDL_GTMR_CHANNEL_CH0
 *         @arg @ref DDL_GTMR_CHANNEL_CH1
 *         @arg @ref DDL_GTMR_CHANNEL_CH2
 *         @arg @ref DDL_GTMR_CHANNEL_CH3
 * @param  ICPolarity This parameter can be one of the following values:
 *         @arg @ref DDL_GTMR_IC_POLARITY_RISING
 *         @arg @ref DDL_GTMR_IC_POLARITY_FALLING
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_IC_SetPolarity(GTMR_TypeDef *TMRx, uint32_t Channel, uint32_t ICPolarity)
{
    uint8_t iChannel = GTMR_GET_CHANNEL_INDEX(Channel);
    MODIFY_REG(TMRx->CCEN, (GTMR_CCEN_CC0POL << DDL_GTMR_SHIFT_TAB_CCxP[iChannel]),
                ICPolarity << DDL_GTMR_SHIFT_TAB_CCxP[iChannel]);
}

/**
 * @brief  Get the current input channel polarity.
 * @param  TMRx Timer instance
 * @param  Channel This parameter can be one of the following values:
 *         @arg @ref DDL_GTMR_CHANNEL_CH0
 *         @arg @ref DDL_GTMR_CHANNEL_CH1
 *         @arg @ref DDL_GTMR_CHANNEL_CH2
 *         @arg @ref DDL_GTMR_CHANNEL_CH3
 * @retval Returned value can be one of the following values:
 *         @arg @ref DDL_GTMR_IC_POLARITY_RISING
 *         @arg @ref DDL_GTMR_IC_POLARITY_FALLING
 */
__STATIC_INLINE uint32_t DDL_GTMR_IC_GetPolarity(GTMR_TypeDef *TMRx, uint32_t Channel)
{
    uint8_t iChannel = GTMR_GET_CHANNEL_INDEX(Channel);
    return (READ_BIT(TMRx->CCEN, (GTMR_CCEN_CC0POL << DDL_GTMR_SHIFT_TAB_CCxP[iChannel])) >>
            DDL_GTMR_SHIFT_TAB_CCxP[iChannel]);
}

/**
 * @brief  Set the channel and ETR input source.
 * @param  TMRx Timer instance
 * @param  Channel This parameter can be one of the following values:
 *         @arg @ref DDL_GTMR_CHANNEL_CH0
 *         @arg @ref DDL_GTMR_CHANNEL_CH1
 *         @arg @ref DDL_GTMR_CHANNEL_CH2
 *         @arg @ref DDL_GTMR_CHANNEL_CH3
 *         @arg @ref DDL_GTMR_CHANNEL_ETR
 * @param  Source This parameter can be one of the following values:
 *         @arg @ref DDL_GTMR_IC_INTPUTSOURCE_CHANNEL
 *         @arg @ref DDL_GTMR_IC_INTPUTSOURCE_COMP0
 *         @arg @ref DDL_GTMR_IC_INTPUTSOURCE_COMP1
 *         @arg @ref DDL_GTMR_IC_INTPUTSOURCE_COMP2
 *         @arg @ref DDL_GTMR_IC_INTPUTSOURCE_COMP3
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_IC_SetInputSource(GTMR_TypeDef *TMRx, uint32_t Channel, uint32_t Source)
{
    uint8_t iChannel = GTMR_GET_CHANNEL_INDEX(Channel);
    MODIFY_REG(TMRx->CR2, (GTMR_CR2_COMPCH0SEL << DDL_GTMR_SHIFT_TAB_ICSEL[iChannel]),
               Source << DDL_GTMR_SHIFT_TAB_ICSEL[iChannel]);
}

/**
 * @brief  Get the channel and ETR input source.
 * @param  TMRx Timer instance
 * @param  Channel This parameter can be one of the following values:
 *         @arg @ref DDL_GTMR_CHANNEL_CH0
 *         @arg @ref DDL_GTMR_CHANNEL_CH1
 *         @arg @ref DDL_GTMR_CHANNEL_CH2
 *         @arg @ref DDL_GTMR_CHANNEL_CH3
 *         @arg @ref DDL_GTMR_CHANNEL_ETR
 * @retval Returned value can be one of the following values:
 *         @arg @ref DDL_GTMR_IC_INTPUTSOURCE_CHANNEL
 *         @arg @ref DDL_GTMR_IC_INTPUTSOURCE_COMP0
 *         @arg @ref DDL_GTMR_IC_INTPUTSOURCE_COMP1
 *         @arg @ref DDL_GTMR_IC_INTPUTSOURCE_COMP2
 *         @arg @ref DDL_GTMR_IC_INTPUTSOURCE_COMP3
 */
__STATIC_INLINE uint32_t DDL_GTMR_IC_GetInputSource(GTMR_TypeDef *TMRx, uint32_t Channel)
{
    uint8_t iChannel = GTMR_GET_CHANNEL_INDEX(Channel);
    return (READ_BIT(TMRx->CR2, (GTMR_CR2_COMPCH0SEL << DDL_GTMR_SHIFT_TAB_ICSEL[iChannel])) >>
            DDL_GTMR_SHIFT_TAB_ICSEL[iChannel]);
}

/**
 * @brief  Connect the TMRx_CH1, CH2 and CH3 pins  to the TI1 input (XOR combination).
 * @note Macro IS_GTMR_XOR_INSTANCE(TMRx) can be used to check whether or not
 *       a timer instance provides an XOR input.
 * @param  TMRx Timer instance
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_IC_EnableXORCombination(GTMR_TypeDef *TMRx)
{
    SET_BIT(TMRx->CR2, GTMR_CR2_TI0SEL);
}

/**
 * @brief  Disconnect the TMRx_CH1, CH2 and CH3 pins  from the TI1 input.
 * @note Macro IS_GTMR_XOR_INSTANCE(TMRx) can be used to check whether or not
 *       a timer instance provides an XOR input.
 * @param  TMRx Timer instance
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_IC_DisableXORCombination(GTMR_TypeDef *TMRx)
{
    CLEAR_BIT(TMRx->CR2, GTMR_CR2_TI0SEL);
}

/**
 * @brief  Indicates whether the TMRx_CH1, CH2 and CH3 pins are connectected to the TI1 input.
 * @note Macro IS_GTMR_XOR_INSTANCE(TMRx) can be used to check whether or not
 * a timer instance provides an XOR input.
 * @param  TMRx Timer instance
 * @retval State of bit (1 or 0).
 */
__STATIC_INLINE uint32_t DDL_GTMR_IC_IsEnabledXORCombination(GTMR_TypeDef *TMRx)
{
    return ((READ_BIT(TMRx->CR2, GTMR_CR2_TI0SEL) == (GTMR_CR2_TI0SEL)) ? 1UL : 0UL);
}

/**
 * @brief  Get captured value for input channel 0.
 * @note In 32-bit timer implementations returned captured value can be between 0x00000000 and 0xFFFFFFFF.
 * @note Macro IS_GTMR_32B_COUNTER_INSTANCE(TMRx) can be used to check
 *       whether or not a timer instance supports a 32 bits counter.
 * @note Macro IS_GTMR_CC0_INSTANCE(TMRx) can be used to check whether or not
 *       input channel 0 is supported by a timer instance.
 * @param  TMRx Timer instance
 * @retval CapturedValue (between Min_Data=0 and Max_Data=65535)
 */
__STATIC_INLINE uint32_t DDL_GTMR_IC_GetCaptureCH0(GTMR_TypeDef *TMRx)
{
    return (uint32_t)(READ_REG(TMRx->CC0));
}

/**
 * @brief  Get captured value for input channel 1.
 * @note In 32-bit timer implementations returned captured value can be between 0x00000000 and 0xFFFFFFFF.
 * @note Macro IS_GTMR_32B_COUNTER_INSTANCE(TMRx) can be used to check
 *       whether or not a timer instance supports a 32 bits counter.
 * @note Macro IS_GTMR_CC1_INSTANCE(TMRx) can be used to check whether or not
 *       input channel 1 is supported by a timer instance.
 * @param  TMRx Timer instance
 * @retval CapturedValue (between Min_Data=0 and Max_Data=65535)
 */
__STATIC_INLINE uint32_t DDL_GTMR_IC_GetCaptureCH1(GTMR_TypeDef *TMRx)
{
    return (uint32_t)(READ_REG(TMRx->CC1));
}

/**
 * @brief  Get captured value for input channel 2.
 * @note In 32-bit timer implementations returned captured value can be between 0x00000000 and 0xFFFFFFFF.
 * @note Macro IS_GTMR_32B_COUNTER_INSTANCE(TMRx) can be used to check
 *       whether or not a timer instance supports a 32 bits counter.
 * @note Macro IS_GTMR_CC2_INSTANCE(TMRx) can be used to check whether or not
 *       input channel 2 is supported by a timer instance.
 * @param  TMRx Timer instance
 * @retval CapturedValue (between Min_Data=0 and Max_Data=65535)
 */
__STATIC_INLINE uint32_t DDL_GTMR_IC_GetCaptureCH2(GTMR_TypeDef *TMRx)
{
    return (uint32_t)(READ_REG(TMRx->CC2));
}

/**
 * @brief  Get captured value for input channel 3.
 * @note In 32-bit timer implementations returned captured value can be between 0x00000000 and 0xFFFFFFFF.
 * @note Macro IS_GTMR_32B_COUNTER_INSTANCE(TMRx) can be used to check
 *       whether or not a timer instance supports a 32 bits counter.
 * @note Macro IS_GTMR_CC3_INSTANCE(TMRx) can be used to check whether or not
 *       input channel 3 is supported by a timer instance.
 * @param  TMRx Timer instance
 * @retval CapturedValue (between Min_Data=0 and Max_Data=65535)
 */
__STATIC_INLINE uint32_t DDL_GTMR_IC_GetCaptureCH3(GTMR_TypeDef *TMRx)
{
    return (uint32_t)(READ_REG(TMRx->CC3));
}

/**
 * @}
 */

/** @defgroup GTMR_DDL_EF_Clock_Selection Counter clock selection
 * @{
 */
/**
 * @brief  Enable external clock mode 2.
 * @note When external clock mode 2 is enabled the counter is clocked by any active edge on the ETRF signal.
 * @note Macro IS_GTMR_CLOCKSOURCE_ETRMODE2_INSTANCE(TMRx) can be used to check
 *       whether or not a timer instance supports external clock mode2.
 * @param  TMRx Timer instance
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_EnableExternalClock(GTMR_TypeDef *TMRx)
{
    SET_BIT(TMRx->SMCR, GTMR_SMCR_ECEN);
}

/**
 * @brief  Disable external clock mode 2.
 * @note Macro IS_GTMR_CLOCKSOURCE_ETRMODE2_INSTANCE(TMRx) can be used to check
 *       whether or not a timer instance supports external clock mode2.
 * @param  TMRx Timer instance
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_DisableExternalClock(GTMR_TypeDef *TMRx)
{
    CLEAR_BIT(TMRx->SMCR, GTMR_SMCR_ECEN);
}

/**
 * @brief  Indicate whether external clock mode 2 is enabled.
 * @note Macro IS_GTMR_CLOCKSOURCE_ETRMODE2_INSTANCE(TMRx) can be used to check
 *       whether or not a timer instance supports external clock mode2.
 * @param  TMRx Timer instance
 * @retval State of bit (1 or 0).
 */
__STATIC_INLINE uint32_t DDL_GTMR_IsEnabledExternalClock(GTMR_TypeDef *TMRx)
{
    return ((READ_BIT(TMRx->SMCR, GTMR_SMCR_ECEN) == (GTMR_SMCR_ECEN)) ? 1UL : 0UL);
}

/**
 * @brief  Set the clock source of the counter clock.
 * @note when selected clock source is external clock mode 1, the timer input
 *       the external clock is applied is selected by calling the @ref DDL_GTMR_SetTriggerInput()
 *       function. This timer input must be configured by calling
 *       the @ref DDL_GTMR_IC_Config() function.
 * @note Macro IS_GTMR_CLOCKSOURCE_ETRMODE1_INSTANCE(TMRx) can be used to check
 *       whether or not a timer instance supports external clock mode1.
 * @note Macro IS_GTMR_CLOCKSOURCE_ETRMODE2_INSTANCE(TMRx) can be used to check
 *       whether or not a timer instance supports external clock mode2.
 * @param  TMRx Timer instance
 * @param  ClockSource This parameter can be one of the following values:
 *         @arg @ref DDL_GTMR_CLOCKSOURCE_INTERNAL
 *         @arg @ref DDL_GTMR_CLOCKSOURCE_EXT_MODE1
 *         @arg @ref DDL_GTMR_CLOCKSOURCE_EXT_MODE2
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_SetClockSource(GTMR_TypeDef *TMRx, uint32_t ClockSource)
{
    MODIFY_REG(TMRx->SMCR, GTMR_SMCR_SMFSEL | GTMR_SMCR_ECEN, ClockSource);
}

/**
 * @brief  Set the encoder interface mode.
 * @note Macro IS_GTMR_ENCODER_INTERFACE_INSTANCE(TMRx) can be used to check
 *       whether or not a timer instance supports the encoder mode.
 * @param  TMRx Timer instance
 * @param  EncoderMode This parameter can be one of the following values:
 *         @arg @ref DDL_GTMR_ENCODERMODE_X2_TI1
 *         @arg @ref DDL_GTMR_ENCODERMODE_X2_TI2
 *         @arg @ref DDL_GTMR_ENCODERMODE_X4_TI12
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_SetEncoderMode(GTMR_TypeDef *TMRx, uint32_t EncoderMode)
{
    MODIFY_REG(TMRx->SMCR, GTMR_SMCR_SMFSEL, EncoderMode);
}

/**
 * @}
 */

/** @defgroup GTMR_DDL_EF_Timer_Synchronization Timer synchronisation configuration
 * @{
 */
/**
 * @brief  Set the trigger output (TRGO) used for timer synchronization .
 * @note Macro IS_GTMR_MASTER_INSTANCE(TMRx) can be used to check
 *       whether or not a timer instance can operate as a master timer.
 * @param  TMRx Timer instance
 * @param  TimerSynchronization This parameter can be one of the following values:
 *         @arg @ref DDL_GTMR_TRGO_RESET
 *         @arg @ref DDL_GTMR_TRGO_ENABLE
 *         @arg @ref DDL_GTMR_TRGO_UPDATE
 *         @arg @ref DDL_GTMR_TRGO_CC1IF
 *         @arg @ref DDL_GTMR_TRGO_OC0REF
 *         @arg @ref DDL_GTMR_TRGO_OC1REF
 *         @arg @ref DDL_GTMR_TRGO_OC2REF
 *         @arg @ref DDL_GTMR_TRGO_OC3REF
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_SetTriggerOutput(GTMR_TypeDef *TMRx, uint32_t TimerSynchronization)
{
    MODIFY_REG(TMRx->CR2, GTMR_CR2_MMSEL, TimerSynchronization);
}

/**
 * @brief  Set the synchronization mode of a slave timer.
 * @note Macro IS_GTMR_SLAVE_INSTANCE(TMRx) can be used to check whether or not
 *       a timer instance can operate as a slave timer.
 * @param  TMRx Timer instance
 * @param  SlaveMode This parameter can be one of the following values:
 *         @arg @ref DDL_GTMR_SLAVEMODE_DISABLED
 *         @arg @ref DDL_GTMR_SLAVEMODE_RESET
 *         @arg @ref DDL_GTMR_SLAVEMODE_GATED
 *         @arg @ref DDL_GTMR_SLAVEMODE_TRIGGER
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_SetSlaveMode(GTMR_TypeDef *TMRx, uint32_t SlaveMode)
{
    MODIFY_REG(TMRx->SMCR, GTMR_SMCR_SMFSEL, SlaveMode);
}

/**
 * @brief  Set the selects the trigger input to be used to synchronize the counter.
 * @note Macro IS_GTMR_SLAVE_INSTANCE(TMRx) can be used to check whether or not
 *       a timer instance can operate as a slave timer.
 * @param  TMRx Timer instance
 * @param  TriggerInput This parameter can be one of the following values:
 *         @arg @ref DDL_GTMR_TS_ITR0
 *         @arg @ref DDL_GTMR_TS_ITR1
 *         @arg @ref DDL_GTMR_TS_ITR2
 *         @arg @ref DDL_GTMR_TS_ITR3
 *         @arg @ref DDL_GTMR_TS_TI1F_ED
 *         @arg @ref DDL_GTMR_TS_TI1FP1
 *         @arg @ref DDL_GTMR_TS_TI2FP2
 *         @arg @ref DDL_GTMR_TS_ETRF
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_SetTriggerInput(GTMR_TypeDef *TMRx, uint32_t TriggerInput)
{
    MODIFY_REG(TMRx->SMCR, GTMR_SMCR_TRGSEL, TriggerInput);
}

/**
 * @brief  Enable the Master/Slave mode.
 * @note Macro IS_GTMR_SLAVE_INSTANCE(TMRx) can be used to check whether or not
 *       a timer instance can operate as a slave timer.
 * @param  TMRx Timer instance
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_EnableMasterSlaveMode(GTMR_TypeDef *TMRx)
{
    SET_BIT(TMRx->SMCR, GTMR_SMCR_MSMEN);
}

/**
 * @brief  Disable the Master/Slave mode.
 * @note Macro IS_GTMR_SLAVE_INSTANCE(TMRx) can be used to check whether or not
 *       a timer instance can operate as a slave timer.
 * @param  TMRx Timer instance
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_DisableMasterSlaveMode(GTMR_TypeDef *TMRx)
{
    CLEAR_BIT(TMRx->SMCR, GTMR_SMCR_MSMEN);
}

/**
 * @brief Indicates whether the Master/Slave mode is enabled.
 * @note Macro IS_GTMR_SLAVE_INSTANCE(TMRx) can be used to check whether or not
 * a timer instance can operate as a slave timer.
 * @param  TMRx Timer instance
 * @retval State of bit (1 or 0).
 */
__STATIC_INLINE uint32_t DDL_GTMR_IsEnabledMasterSlaveMode(GTMR_TypeDef *TMRx)
{
    return ((READ_BIT(TMRx->SMCR, GTMR_SMCR_MSMEN) == (GTMR_SMCR_MSMEN)) ? 1UL : 0UL);
}

/**
 * @brief  Configure the external trigger (ETR) input.
 * @note Macro IS_GTMR_ETR_INSTANCE(TMRx) can be used to check whether or not
 *       a timer instance provides an external trigger input.
 * @param  TMRx Timer instance
 * @param  ETRPolarity This parameter can be one of the following values:
 *         @arg @ref DDL_GTMR_ETR_POLARITY_NONINVERTED
 *         @arg @ref DDL_GTMR_ETR_POLARITY_INVERTED
 * @param  ETRPrescaler This parameter can be one of the following values:
 *         @arg @ref DDL_GTMR_ETR_PRESCALER_DIV1
 *         @arg @ref DDL_GTMR_ETR_PRESCALER_DIV2
 *         @arg @ref DDL_GTMR_ETR_PRESCALER_DIV4
 *         @arg @ref DDL_GTMR_ETR_PRESCALER_DIV8
 * @param  ETRFilter This parameter can be one of the following values:
 *         @arg @ref DDL_GTMR_ETR_FILTER_FDIV1
 *         @arg @ref DDL_GTMR_ETR_FILTER_FDIV1_N2
 *         @arg @ref DDL_GTMR_ETR_FILTER_FDIV1_N4
 *         @arg @ref DDL_GTMR_ETR_FILTER_FDIV1_N8
 *         @arg @ref DDL_GTMR_ETR_FILTER_FDIV2_N6
 *         @arg @ref DDL_GTMR_ETR_FILTER_FDIV2_N8
 *         @arg @ref DDL_GTMR_ETR_FILTER_FDIV4_N6
 *         @arg @ref DDL_GTMR_ETR_FILTER_FDIV4_N8
 *         @arg @ref DDL_GTMR_ETR_FILTER_FDIV8_N6
 *         @arg @ref DDL_GTMR_ETR_FILTER_FDIV8_N8
 *         @arg @ref DDL_GTMR_ETR_FILTER_FDIV16_N5
 *         @arg @ref DDL_GTMR_ETR_FILTER_FDIV16_N6
 *         @arg @ref DDL_GTMR_ETR_FILTER_FDIV16_N8
 *         @arg @ref DDL_GTMR_ETR_FILTER_FDIV32_N5
 *         @arg @ref DDL_GTMR_ETR_FILTER_FDIV32_N6
 *         @arg @ref DDL_GTMR_ETR_FILTER_FDIV32_N8
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_ConfigETR(GTMR_TypeDef *TMRx, uint32_t ETRPolarity, uint32_t ETRPrescaler,
                                        uint32_t ETRFilter)
{
    MODIFY_REG(TMRx->SMCR, GTMR_SMCR_ETPOL | GTMR_SMCR_ETPCFG | GTMR_SMCR_ETFCFG, ETRPolarity | ETRPrescaler | ETRFilter);
}

/**
 * @}
 */

/** @defgroup GTMR_DDL_EF_FLAG_Management FLAG-Management
 * @{
 */
/**
 * @brief  Clear the update interrupt flag (UIF).
 * @param  TMRx Timer instance
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_ClearFlag_UPDATE(GTMR_TypeDef *TMRx)
{
    CLEAR_BIT(TMRx->SR, GTMR_SR_UIFLG);
}

/**
 * @brief  Indicate whether update interrupt flag (UIF) is set (update interrupt is pending).
 * @param  TMRx Timer instance
 * @retval State of bit (1 or 0).
 */
__STATIC_INLINE uint32_t DDL_GTMR_IsActiveFlag_UPDATE(GTMR_TypeDef *TMRx)
{
    return ((READ_BIT(TMRx->SR, GTMR_SR_UIFLG) == (GTMR_SR_UIFLG)) ? 1UL : 0UL);
}

/**
 * @brief  Clear the Capture/Compare 0 interrupt flag (CC0F).
 * @param  TMRx Timer instance
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_ClearFlag_CC0(GTMR_TypeDef *TMRx)
{
    CLEAR_BIT(TMRx->SR, GTMR_SR_CC0IFLG);
}

/**
 * @brief  Indicate whether Capture/Compare 0 interrupt flag (CC0F) is set (Capture/Compare 0 interrupt is pending).
 * @param  TMRx Timer instance
 * @retval State of bit (1 or 0).
 */
__STATIC_INLINE uint32_t DDL_GTMR_IsActiveFlag_CC0(GTMR_TypeDef *TMRx)
{
    return ((READ_BIT(TMRx->SR, GTMR_SR_CC0IFLG) == (GTMR_SR_CC0IFLG)) ? 1UL : 0UL);
}

/**
 * @brief  Clear the Capture/Compare 1 interrupt flag (CC1F).
 * @param  TMRx Timer instance
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_ClearFlag_CC1(GTMR_TypeDef *TMRx)
{
    CLEAR_BIT(TMRx->SR, GTMR_SR_CC1IFLG);
}

/**
 * @brief  Indicate whether Capture/Compare 1 interrupt flag (CC1F) is set (Capture/Compare 1 interrupt is pending).
 * @param  TMRx Timer instance
 * @retval State of bit (1 or 0).
 */
__STATIC_INLINE uint32_t DDL_GTMR_IsActiveFlag_CC1(GTMR_TypeDef *TMRx)
{
    return ((READ_BIT(TMRx->SR, GTMR_SR_CC1IFLG) == (GTMR_SR_CC1IFLG)) ? 1UL : 0UL);
}

/**
 * @brief  Clear the Capture/Compare 2 interrupt flag (CC2F).
 * @param  TMRx Timer instance
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_ClearFlag_CC2(GTMR_TypeDef *TMRx)
{
    CLEAR_BIT(TMRx->SR, GTMR_SR_CC2IFLG);
}

/**
 * @brief  Indicate whether Capture/Compare 2 interrupt flag (CC2F) is set (Capture/Compare 2 interrupt is pending).
 * @param  TMRx Timer instance
 * @retval State of bit (1 or 0).
 */
__STATIC_INLINE uint32_t DDL_GTMR_IsActiveFlag_CC2(GTMR_TypeDef *TMRx)
{
    return ((READ_BIT(TMRx->SR, GTMR_SR_CC2IFLG) == (GTMR_SR_CC2IFLG)) ? 1UL : 0UL);
}

/**
 * @brief  Clear the Capture/Compare 3 interrupt flag (CC3F).
 * @param  TMRx Timer instance
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_ClearFlag_CC3(GTMR_TypeDef *TMRx)
{
    CLEAR_BIT(TMRx->SR, GTMR_SR_CC3IFLG);
}

/**
 * @brief  Indicate whether Capture/Compare 3 interrupt flag (CC3F) is set (Capture/Compare 3 interrupt is pending).
 * @param  TMRx Timer instance
 * @retval State of bit (1 or 0).
 */
__STATIC_INLINE uint32_t DDL_GTMR_IsActiveFlag_CC3(GTMR_TypeDef *TMRx)
{
    return ((READ_BIT(TMRx->SR, GTMR_SR_CC3IFLG) == (GTMR_SR_CC3IFLG)) ? 1UL : 0UL);
}

/**
 * @brief  Clear the trigger interrupt flag (TIF).
 * @param  TMRx Timer instance
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_ClearFlag_TRIG(GTMR_TypeDef *TMRx)
{
    CLEAR_BIT(TMRx->SR, GTMR_SR_TRGIFLG);
}

/**
 * @brief  Indicate whether trigger interrupt flag (TIF) is set (trigger interrupt is pending).
 * @param  TMRx Timer instance
 * @retval State of bit (1 or 0).
 */
__STATIC_INLINE uint32_t DDL_GTMR_IsActiveFlag_TRIG(GTMR_TypeDef *TMRx)
{
    return ((READ_BIT(TMRx->SR, GTMR_SR_TRGIFLG) == (GTMR_SR_TRGIFLG)) ? 1UL : 0UL);
}

/**
 * @brief  Clear the Capture/Compare 0 over-capture interrupt flag (CC0OF).
 * @param  TMRx Timer instance
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_ClearFlag_CC0OVR(GTMR_TypeDef *TMRx)
{
    CLEAR_BIT(TMRx->SR, GTMR_SR_CC0RCFLG);
}

/**
 * @brief  Indicate whether Capture/Compare 0 over-capture interrupt flag (CC0OF) is set
 *         (Capture/Compare 0 interrupt is pending).
 * @param  TMRx Timer instance
 * @retval State of bit (1 or 0).
 */
__STATIC_INLINE uint32_t DDL_GTMR_IsActiveFlag_CC0OVR(GTMR_TypeDef *TMRx)
{
    return ((READ_BIT(TMRx->SR, GTMR_SR_CC0RCFLG) == (GTMR_SR_CC0RCFLG)) ? 1UL : 0UL);
}

/**
 * @brief  Clear the Capture/Compare 1 over-capture interrupt flag (CC1OF).
 * @param  TMRx Timer instance
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_ClearFlag_CC1OVR(GTMR_TypeDef *TMRx)
{
    CLEAR_BIT(TMRx->SR, GTMR_SR_CC1RCFLG);
}

/**
 * @brief  Indicate whether Capture/Compare 1 over-capture interrupt flag (CC1OF) is set
 *         (Capture/Compare 1 over-capture interrupt is pending).
 * @param  TMRx Timer instance
 * @retval State of bit (1 or 0).
 */
__STATIC_INLINE uint32_t DDL_GTMR_IsActiveFlag_CC1OVR(GTMR_TypeDef *TMRx)
{
    return ((READ_BIT(TMRx->SR, GTMR_SR_CC1RCFLG) == (GTMR_SR_CC1RCFLG)) ? 1UL : 0UL);
}

/**
 * @brief  Clear the Capture/Compare 2 over-capture interrupt flag (CC2OF).
 * @param  TMRx Timer instance
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_ClearFlag_CC2OVR(GTMR_TypeDef *TMRx)
{
    CLEAR_BIT(TMRx->SR, GTMR_SR_CC2RCFLG);
}

/**
 * @brief  Indicate whether Capture/Compare 2 over-capture interrupt flag (CC2OF) is set
 *         (Capture/Compare 2 over-capture interrupt is pending).
 * @param  TMRx Timer instance
 * @retval State of bit (1 or 0).
 */
__STATIC_INLINE uint32_t DDL_GTMR_IsActiveFlag_CC2OVR(GTMR_TypeDef *TMRx)
{
    return ((READ_BIT(TMRx->SR, GTMR_SR_CC2RCFLG) == (GTMR_SR_CC2RCFLG)) ? 1UL : 0UL);
}

/**
 * @brief  Clear the Capture/Compare 3 over-capture interrupt flag (CC3OF).
 * @param  TMRx Timer instance
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_ClearFlag_CC3OVR(GTMR_TypeDef *TMRx)
{
    CLEAR_BIT(TMRx->SR, GTMR_SR_CC3RCFLG);
}

/**
 * @brief  Indicate whether Capture/Compare 3 over-capture interrupt flag (CC3OF) is set
 *         (Capture/Compare 3 over-capture interrupt is pending).
 * @param  TMRx Timer instance
 * @retval State of bit (1 or 0).
 */
__STATIC_INLINE uint32_t DDL_GTMR_IsActiveFlag_CC3OVR(GTMR_TypeDef *TMRx)
{
    return ((READ_BIT(TMRx->SR, GTMR_SR_CC3RCFLG) == (GTMR_SR_CC3RCFLG)) ? 1UL : 0UL);
}

/**
 * @}
 */

/** @defgroup GTMR_DDL_EF_IT_Management IT-Management
 * @{
 */
/**
 * @brief  Enable update interrupt (UIE).
 * @param  TMRx Timer instance
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_EnableIT_UPDATE(GTMR_TypeDef *TMRx)
{
    SET_BIT(TMRx->DIER, GTMR_DIER_UIEN);
}

/**
 * @brief  Disable update interrupt (UIE).
 * @param  TMRx Timer instance
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_DisableIT_UPDATE(GTMR_TypeDef *TMRx)
{
    CLEAR_BIT(TMRx->DIER, GTMR_DIER_UIEN);
}

/**
 * @brief  Indicates whether the update interrupt (UIE) is enabled.
 * @param  TMRx Timer instance
 * @retval State of bit (1 or 0).
 */
__STATIC_INLINE uint32_t DDL_GTMR_IsEnabledIT_UPDATE(GTMR_TypeDef *TMRx)
{
    return ((READ_BIT(TMRx->DIER, GTMR_DIER_UIEN) == (GTMR_DIER_UIEN)) ? 1UL : 0UL);
}

/**
 * @brief  Enable capture/compare 0 interrupt (CC0IE).
 * @param  TMRx Timer instance
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_EnableIT_CC0(GTMR_TypeDef *TMRx)
{
    SET_BIT(TMRx->DIER, GTMR_DIER_CC0IEN);
}

/**
 * @brief  Disable capture/compare 0 interrupt (CC0IE).
 * @param  TMRx Timer instance
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_DisableIT_CC0(GTMR_TypeDef *TMRx)
{
    CLEAR_BIT(TMRx->DIER, GTMR_DIER_CC0IEN);
}

/**
 * @brief  Indicates whether the capture/compare 0 interrupt (CC0IE) is enabled.
 * @param  TMRx Timer instance
 * @retval State of bit (1 or 0).
 */
__STATIC_INLINE uint32_t DDL_GTMR_IsEnabledIT_CC0(GTMR_TypeDef *TMRx)
{
    return ((READ_BIT(TMRx->DIER, GTMR_DIER_CC0IEN) == (GTMR_DIER_CC0IEN)) ? 1UL : 0UL);
}

/**
 * @brief  Enable capture/compare 1 interrupt (CC1IE).
 * @param  TMRx Timer instance
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_EnableIT_CC1(GTMR_TypeDef *TMRx)
{
    SET_BIT(TMRx->DIER, GTMR_DIER_CC1IEN);
}

/**
 * @brief  Disable capture/compare 1 interrupt (CC1IE).
 * @param  TMRx Timer instance
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_DisableIT_CC1(GTMR_TypeDef *TMRx)
{
    CLEAR_BIT(TMRx->DIER, GTMR_DIER_CC1IEN);
}

/**
 * @brief  Indicates whether the capture/compare 1 interrupt (CC1IE) is enabled.
 * @param  TMRx Timer instance
 * @retval State of bit (1 or 0).
 */
__STATIC_INLINE uint32_t DDL_GTMR_IsEnabledIT_CC1(GTMR_TypeDef *TMRx)
{
    return ((READ_BIT(TMRx->DIER, GTMR_DIER_CC1IEN) == (GTMR_DIER_CC1IEN)) ? 1UL : 0UL);
}

/**
 * @brief  Enable capture/compare 2 interrupt (CC2IE).
 * @param  TMRx Timer instance
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_EnableIT_CC2(GTMR_TypeDef *TMRx)
{
    SET_BIT(TMRx->DIER, GTMR_DIER_CC2IEN);
}

/**
 * @brief  Disable capture/compare 2 interrupt (CC2IE).
 * @param  TMRx Timer instance
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_DisableIT_CC2(GTMR_TypeDef *TMRx)
{
    CLEAR_BIT(TMRx->DIER, GTMR_DIER_CC2IEN);
}

/**
 * @brief  Indicates whether the capture/compare 2 interrupt (CC2IE) is enabled.
 * @param  TMRx Timer instance
 * @retval State of bit (1 or 0).
 */
__STATIC_INLINE uint32_t DDL_GTMR_IsEnabledIT_CC2(GTMR_TypeDef *TMRx)
{
    return ((READ_BIT(TMRx->DIER, GTMR_DIER_CC2IEN) == (GTMR_DIER_CC2IEN)) ? 1UL : 0UL);
}

/**
 * @brief  Enable capture/compare 3 interrupt (CC3IE).
 * @param  TMRx Timer instance
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_EnableIT_CC3(GTMR_TypeDef *TMRx)
{
    SET_BIT(TMRx->DIER, GTMR_DIER_CC3IEN);
}

/**
 * @brief  Disable capture/compare 3 interrupt (CC3IE).
 * @param  TMRx Timer instance
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_DisableIT_CC3(GTMR_TypeDef *TMRx)
{
    CLEAR_BIT(TMRx->DIER, GTMR_DIER_CC3IEN);
}

/**
 * @brief  Indicates whether the capture/compare 3 interrupt (CC3IE) is enabled.
 * @param  TMRx Timer instance
 * @retval State of bit (1 or 0).
 */
__STATIC_INLINE uint32_t DDL_GTMR_IsEnabledIT_CC3(GTMR_TypeDef *TMRx)
{
    return ((READ_BIT(TMRx->DIER, GTMR_DIER_CC3IEN) == (GTMR_DIER_CC3IEN)) ? 1UL : 0UL);
}

/**
 * @brief  Enable trigger interrupt (TIE).
 * @param  TMRx Timer instance
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_EnableIT_TRIG(GTMR_TypeDef *TMRx)
{
    SET_BIT(TMRx->DIER, GTMR_DIER_TRGIEN);
}

/**
 * @brief  Disable trigger interrupt (TIE).
 * @param  TMRx Timer instance
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_DisableIT_TRIG(GTMR_TypeDef *TMRx)
{
    CLEAR_BIT(TMRx->DIER, GTMR_DIER_TRGIEN);
}

/**
 * @brief  Indicates whether the trigger interrupt (TIE) is enabled.
 * @param  TMRx Timer instance
 * @retval State of bit (1 or 0).
 */
__STATIC_INLINE uint32_t DDL_GTMR_IsEnabledIT_TRIG(GTMR_TypeDef *TMRx)
{
    return ((READ_BIT(TMRx->DIER, GTMR_DIER_TRGIEN) == (GTMR_DIER_TRGIEN)) ? 1UL : 0UL);
}

/**
 * @}
 */

/** @defgroup GTMR_DDL_EF_DMA_Management DMA Management
 * @{
 */
/**
 * @brief  Enable update DMA request (UDE).
 * @param  TMRx Timer instance
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_EnableDMAReq_UPDATE(GTMR_TypeDef *TMRx)
{
    SET_BIT(TMRx->DIER, GTMR_DIER_UDIEN);
}

/**
 * @brief  Disable update DMA request (UDE).
 * @param  TMRx Timer instance
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_DisableDMAReq_UPDATE(GTMR_TypeDef *TMRx)
{
    CLEAR_BIT(TMRx->DIER, GTMR_DIER_UDIEN);
}

/**
 * @brief  Indicates whether the update DMA request (UDE) is enabled.
 * @param  TMRx Timer instance
 * @retval State of bit (1 or 0).
 */
__STATIC_INLINE uint32_t DDL_GTMR_IsEnabledDMAReq_UPDATE(GTMR_TypeDef *TMRx)
{
    return ((READ_BIT(TMRx->DIER, GTMR_DIER_UDIEN) == (GTMR_DIER_UDIEN)) ? 1UL : 0UL);
}

/**
 * @brief  Enable capture/compare 0 DMA request (CC0DE).
 * @param  TMRx Timer instance
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_EnableDMAReq_CC0(GTMR_TypeDef *TMRx)
{
    SET_BIT(TMRx->DIER, GTMR_DIER_CC0DEN);
}

/**
 * @brief  Disable capture/compare 0 DMA request (CC0DE).
 * @param  TMRx Timer instance
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_DisableDMAReq_CC0(GTMR_TypeDef *TMRx)
{
    CLEAR_BIT(TMRx->DIER, GTMR_DIER_CC0DEN);
}

/**
 * @brief  Indicates whether the capture/compare 0 DMA request (CC0DE) is enabled.
 * @param  TMRx Timer instance
 * @retval State of bit (1 or 0).
 */
__STATIC_INLINE uint32_t DDL_GTMR_IsEnabledDMAReq_CC0(GTMR_TypeDef *TMRx)
{
    return ((READ_BIT(TMRx->DIER, GTMR_DIER_CC0DEN) == (GTMR_DIER_CC0DEN)) ? 1UL : 0UL);
}

/**
 * @brief  Enable capture/compare 1 DMA request (CC1DE).
 * @param  TMRx Timer instance
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_EnableDMAReq_CC1(GTMR_TypeDef *TMRx)
{
    SET_BIT(TMRx->DIER, GTMR_DIER_CC1DEN);
}

/**
 * @brief  Disable capture/compare 1 DMA request (CC1DE).
 * @param  TMRx Timer instance
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_DisableDMAReq_CC1(GTMR_TypeDef *TMRx)
{
    CLEAR_BIT(TMRx->DIER, GTMR_DIER_CC1DEN);
}

/**
 * @brief  Indicates whether the capture/compare 1 DMA request (CC1DE) is enabled.
 * @param  TMRx Timer instance
 * @retval State of bit (1 or 0).
 */
__STATIC_INLINE uint32_t DDL_GTMR_IsEnabledDMAReq_CC1(GTMR_TypeDef *TMRx)
{
    return ((READ_BIT(TMRx->DIER, GTMR_DIER_CC1DEN) == (GTMR_DIER_CC1DEN)) ? 1UL : 0UL);
}

/**
 * @brief  Enable capture/compare 2 DMA request (CC2DE).
 * @param  TMRx Timer instance
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_EnableDMAReq_CC2(GTMR_TypeDef *TMRx)
{
    SET_BIT(TMRx->DIER, GTMR_DIER_CC2DEN);
}

/**
 * @brief  Disable capture/compare 2 DMA request (CC2DE).
 * @param  TMRx Timer instance
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_DisableDMAReq_CC2(GTMR_TypeDef *TMRx)
{
    CLEAR_BIT(TMRx->DIER, GTMR_DIER_CC2DEN);
}

/**
 * @brief  Indicates whether the capture/compare 2 DMA request (CC2DE) is enabled.
 * @param  TMRx Timer instance
 * @retval State of bit (1 or 0).
 */
__STATIC_INLINE uint32_t DDL_GTMR_IsEnabledDMAReq_CC2(GTMR_TypeDef *TMRx)
{
    return ((READ_BIT(TMRx->DIER, GTMR_DIER_CC2DEN) == (GTMR_DIER_CC2DEN)) ? 1UL : 0UL);
}

/**
 * @brief  Enable capture/compare 3 DMA request (CC3DE).
 * @param  TMRx Timer instance
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_EnableDMAReq_CC3(GTMR_TypeDef *TMRx)
{
    SET_BIT(TMRx->DIER, GTMR_DIER_CC3DEN);
}

/**
 * @brief  Disable capture/compare 3 DMA request (CC3DE).
 * @param  TMRx Timer instance
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_DisableDMAReq_CC3(GTMR_TypeDef *TMRx)
{
    CLEAR_BIT(TMRx->DIER, GTMR_DIER_CC3DEN);
}

/**
 * @brief  Indicates whether the capture/compare 3 DMA request (CC3DE) is enabled.
 * @param  TMRx Timer instance
 * @retval State of bit (1 or 0).
 */
__STATIC_INLINE uint32_t DDL_GTMR_IsEnabledDMAReq_CC3(GTMR_TypeDef *TMRx)
{
    return ((READ_BIT(TMRx->DIER, GTMR_DIER_CC3DEN) == (GTMR_DIER_CC3DEN)) ? 1UL : 0UL);
}

/**
 * @brief  Enable trigger interrupt (TDE).
 * @param  TMRx Timer instance
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_EnableDMAReq_TRIG(GTMR_TypeDef *TMRx)
{
    SET_BIT(TMRx->DIER, GTMR_DIER_TRGDEN);
}

/**
 * @brief  Disable trigger interrupt (TDE).
 * @param  TMRx Timer instance
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_DisableDMAReq_TRIG(GTMR_TypeDef *TMRx)
{
    CLEAR_BIT(TMRx->DIER, GTMR_DIER_TRGDEN);
}

/**
 * @brief  Indicates whether the trigger interrupt (TDE) is enabled.
 * @param  TMRx Timer instance
 * @retval State of bit (1 or 0).
 */
__STATIC_INLINE uint32_t DDL_GTMR_IsEnabledDMAReq_TRIG(GTMR_TypeDef *TMRx)
{
    return ((READ_BIT(TMRx->DIER, GTMR_DIER_TRGDEN) == (GTMR_DIER_TRGDEN)) ? 1UL : 0UL);
}

/**
 * @}
 */

/** @defgroup GTMR_DDL_EF_EVENT_Management EVENT-Management
 * @{
 */
/**
 * @brief  Generate an update event.
 * @param  TMRx Timer instance
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_GenerateEvent_UPDATE(GTMR_TypeDef *TMRx)
{
    SET_BIT(TMRx->CEG, GTMR_CEG_UEG);
}

/**
 * @brief  Generate Capture/Compare 0 event.
 * @param  TMRx Timer instance
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_GenerateEvent_CC0(GTMR_TypeDef *TMRx)
{
    SET_BIT(TMRx->CEG, GTMR_CEG_CC0EG);
}

/**
 * @brief  Generate Capture/Compare 1 event.
 * @param  TMRx Timer instance
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_GenerateEvent_CC1(GTMR_TypeDef *TMRx)
{
    SET_BIT(TMRx->CEG, GTMR_CEG_CC1EG);
}

/**
 * @brief  Generate Capture/Compare 2 event.
 * @param  TMRx Timer instance
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_GenerateEvent_CC2(GTMR_TypeDef *TMRx)
{
    SET_BIT(TMRx->CEG, GTMR_CEG_CC2EG);
}

/**
 * @brief  Generate Capture/Compare 3 event.
 * @param  TMRx Timer instance
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_GenerateEvent_CC3(GTMR_TypeDef *TMRx)
{
    SET_BIT(TMRx->CEG, GTMR_CEG_CC3EG);
}

/**
 * @brief  Generate trigger event.
 * @param  TMRx Timer instance
 * @retval None
 */
__STATIC_INLINE void DDL_GTMR_GenerateEvent_TRIG(GTMR_TypeDef *TMRx)
{
    SET_BIT(TMRx->CEG, GTMR_CEG_TEG);
}

/**
 * @}
 */

#if defined(USE_FULL_DDL_DRIVER)
/** @defgroup GTMR_DDL_EF_Init Initialisation and deinitialisation functions
 * @{
 */

ErrorStatus DDL_GTMR_DeInit(GTMR_TypeDef *TMRx);
void DDL_GTMR_StructInit(DDL_GTMR_InitTypeDef *TMR_InitStruct);
ErrorStatus DDL_GTMR_Init(GTMR_TypeDef *TMRx, DDL_GTMR_InitTypeDef *TMR_InitStruct);
void DDL_GTMR_OC_StructInit(DDL_GTMR_OC_InitTypeDef *TMR_OC_InitStruct);
ErrorStatus DDL_GTMR_OC_Init(GTMR_TypeDef *TMRx, uint32_t Channel, DDL_GTMR_OC_InitTypeDef *TMR_OC_InitStruct);
void DDL_GTMR_IC_StructInit(DDL_GTMR_IC_InitTypeDef *TMR_ICInitStruct);
ErrorStatus DDL_GTMR_IC_Init(GTMR_TypeDef *TMRx, uint32_t Channel, DDL_GTMR_IC_InitTypeDef *TMR_IC_InitStruct);
void DDL_GTMR_ENCODER_StructInit(DDL_GTMR_ENCODER_InitTypeDef *TMR_EncoderInitStruct);
ErrorStatus DDL_GTMR_ENCODER_Init(GTMR_TypeDef *TMRx, DDL_GTMR_ENCODER_InitTypeDef *TMR_EncoderInitStruct);
void DDL_GTMR_HALLSENSOR_StructInit(DDL_GTMR_HALLSENSOR_InitTypeDef *TMR_HallSensorInitStruct);
ErrorStatus DDL_GTMR_HALLSENSOR_Init(GTMR_TypeDef *TMRx, DDL_GTMR_HALLSENSOR_InitTypeDef *TMR_HallSensorInitStruct);
/**
 * @}
 */
#endif /* USE_FULL_DDL_DRIVER */

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

#ifdef __cplusplus
}
#endif

#endif /* G32F031_DDL_GTMR_H */
