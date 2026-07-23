/**
  *
  * @file    g32f031_ddl_opa.h
  * @brief   Header file of OPA DDL module.
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
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef G32F031_DDL_OPA_H
#define G32F031_DDL_OPA_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "g32f0xx.h"

/** @addtogroup G32F031_DDL_Driver
  * @{
  */

#if defined (OPA)

/** @defgroup OPA_DDL OPA
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
#if defined(USE_FULL_DDL_DRIVER)
/** @defgroup OPA_DDL_ES_INIT OPA Exported Init structure
  * @{
  */

/**
  * @brief  Structure definition of some features of OPA instance.
  */
typedef struct
{
  uint32_t Channel;                     /*!< Set OPA channel.
                                             This parameter can be a value of @ref OPA_DDL_EC_CHANNEL */

  uint32_t GainSelect;                  /*!< Set OPA Gain signal .
                                             This parameter can be a value of @ref OPA_DDL_EC_INTERNALGAIN

                                             This feature can be modified afterwards using unitary function @ref DDL_OPA_SetGain(). */

  uint32_t InputControl;                /*!< Set OPA input non-inverting connection.
                                             This parameter can be a value of @ref OPA_DDL_EC_INCTRL

                                             This feature can be modified afterwards using unitary function @ref DDL_OPA_EnableInputControl(). */

  uint32_t OutputControl;               /*!< Set OPA inverting input connection.
                                             This parameter can be a value of @ref OPA_DDL_EC_OUTCTRL

                                             This feature can be modified afterwards using unitary function @ref DDL_OPA_EnableOutputControl(). */

  uint32_t VCMSelect;                   /*!< Set OPA inverting input connection.
                                             This parameter can be a value of @ref OPA_DDL_EC_VCMSEL

                                             This feature can be modified afterwards using unitary function @ref DDL_OPA_SetVCM(). */

} DDL_OPA_InitTypeDef;

/**
  * @}
  */
#endif /* USE_FULL_LL_DRIVER */

/* Exported constants --------------------------------------------------------*/
/** @defgroup OPA_DDL_Exported_Constants OPA Exported Constants
  * @{
  */

/** @defgroup OPA_DDL_EC_CHANNEL OPA channel selection
  * @{
  */
#define DDL_OPA_CHANNEL_0                     OPA_CR_OPA0EN_Pos
#define DDL_OPA_CHANNEL_1                     OPA_CR_OPA1EN_Pos
#define OPA_CHANNEL_BITOFFSET_MASK            (OPA_CR_OPA0EN_Pos | OPA_CR_OPA1EN_Pos)
/**
  * @}
  */

/** @defgroup OPA_DDL_EC_INTERNALGAIN  OPA internal gain
  * @{
  */
#define DDL_OPA_INTERNALGAIN_DISABLE          0x00000000U                                                                 /*!< OPA external gain */
#define DDL_OPA_INTERNALGAIN_1                OPA_CR_OPA0SELGAIN_0                                                      /*!< OPA internal gain * 1 */
#define DDL_OPA_INTERNALGAIN_4                OPA_CR_OPA0SELGAIN_1                                                      /*!< OPA internal gain * 4 */
#define DDL_OPA_INTERNALGAIN_6                (OPA_CR_OPA0SELGAIN_1 | OPA_CR_OPA0SELGAIN_0)                           /*!< OPA internal gain * 6 */
#define DDL_OPA_INTERNALGAIN_8                OPA_CR_OPA0SELGAIN_2                                                      /*!< OPA internal gain * 8 */
#define DDL_OPA_INTERNALGAIN_10               (OPA_CR_OPA0SELGAIN_2 | OPA_CR_OPA0SELGAIN_0)                           /*!< OPA internal gain * 10 */
#define DDL_OPA_INTERNALGAIN_12               (OPA_CR_OPA0SELGAIN_2 | OPA_CR_OPA0SELGAIN_1)                           /*!< OPA internal gain * 12 */
#define DDL_OPA_INTERNALGAIN_16               (OPA_CR_OPA0SELGAIN_2 | OPA_CR_OPA0SELGAIN_1 | OPA_CR_OPA0SELGAIN_0)  /*!< OPA internal gain * 16 */
/**
  * @}
  */

/** @defgroup OPA_DDL_EC_VCMSEL  OPA VCM select
  * @{
  */
#define DDL_OPA_VCMSEL_DISABLE                   0x00000000U                                                              /*!< OPA VCM no select */
#define DDL_OPA_VCMSEL_AVDD_0_5                  OPA_CR_OPASELVCM_2                                                     /*!< OPA VCM = 0.5 * AVDD */
#define DDL_OPA_VCMSEL_AVDD_0_25                 (OPA_CR_OPASELVCM_0 | OPA_CR_OPASELVCM_2)                            /*!< OPA VCM = 0.25 * AVDD */
#define DDL_OPA_VCMSEL_VBG                       (OPA_CR_OPASELVCM_1 | OPA_CR_OPASELVCM_2)                            /*!< OPA VCM = VBG */
#define DDL_OPA_VCMSEL_VBG_0_25                  (OPA_CR_OPASELVCM_0 | OPA_CR_OPASELVCM_1 | OPA_CR_OPASELVCM_2)     /*!< OPA VCM = 0.25 * VBG */
/**
  * @}
  */

/** @defgroup OPA_DDL_EC_INCTRL  OPA input control
  * @{
  */
#define DDL_OPA_INCTRL_DISABLE                  0x00000000U               /*!< OPA disable input control signal */
#define DDL_OPA_INCTRL_ENABLE                   OPA_CR_OPA0INSEL        /*!< OPA enable input control signal */
/**
  * @}
  */

/** @defgroup OPA_DDL_EC_OUTCTRL  OPA output control
  * @{
  */
#define DDL_OPA_OUTCTRL_DISABLE                 0x00000000U               /*!< OPA disable output control signal */
#define DDL_OPA_OUTCTRL_ENABLE                  OPA_CR_OPA0OUTSEL       /*!< OPA enable output control signal */
/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup OPA_DDL_Exported_Macros OPA Exported Macros
  * @{
  */
/** @defgroup OPA_DDL_EM_WRITE_READ Common Write and read registers macros
  * @{
  */
/**
  * @brief  Write a value in OPA register
  * @param  __INSTANCE__ OPA Instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define DDL_OPA_WriteReg(__INSTANCE__, __REG__, __VALUE__) WRITE_REG(__INSTANCE__->__REG__, (__VALUE__))

/**
  * @brief  Read a value in OPA register
  * @param  __INSTANCE__ OPA Instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define DDL_OPA_ReadReg(__INSTANCE__, __REG__) READ_REG(__INSTANCE__->__REG__)
/**
  * @}
  */

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup OPA_DDL_Exported_Functions OPA Exported Functions
  * @{
  */

/** @defgroup OPA_DDL_EF_Configuration Configuration
  * @{
  */
/**
  * @brief  Enable OPAx(x = 0 or 1).
  * @param  OPAx OPA Instance
  * @param  OPA_Select This parameter can be one of the following values:
  *         @arg @ref DDL_OPA_CHANNEL_0
  *         @arg @ref DDL_OPA_CHANNEL_1
  * @retval None
  */
__STATIC_INLINE void DDL_OPA_Enable(OPA_TypeDef *OPAx, uint32_t OPA_Select)
{
  SET_BIT(OPAx->CR, OPA_CR_OPA0EN << (OPA_Select & OPA_CHANNEL_BITOFFSET_MASK));
}

/**
  * @brief  Disable OPAx(x = 0 or 1).
  * @param  OPAx OPA Instance
  * @param  OPA_Select This parameter can be one of the following values:
  *         @arg @ref DDL_OPA_CHANNEL_0
  *         @arg @ref DDL_OPA_CHANNEL_1
  * @retval None
  */
__STATIC_INLINE void DDL_OPA_Disable(OPA_TypeDef *OPAx, uint32_t OPA_Select)
{
  CLEAR_BIT(OPAx->CR, OPA_CR_OPA0EN << (OPA_Select & OPA_CHANNEL_BITOFFSET_MASK));
}

/**
  * @brief  Checks if OPAx(x = 0 or 1) is enabled
  * @param  OPAx OPA Instance
  * @param  OPA_Select This parameter can be one of the following values:
  *         @arg @ref DDL_OPA_CHANNEL_0
  *         @arg @ref DDL_OPA_CHANNEL_1
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_OPA_IsEnabled(OPA_TypeDef *OPAx, uint32_t OPA_Select)
{
  return ((READ_BIT(OPAx->CR, OPA_CR_OPA0EN << (OPA_Select & OPA_CHANNEL_BITOFFSET_MASK)) == \
                    (OPA_CR_OPA0EN << (OPA_Select & OPA_CHANNEL_BITOFFSET_MASK))) ? 1UL : 0UL);
}

/**
  * @brief  Set OPAx(x = 0 or 1) gain.
  * @param  OPAx OPA Instance
  * @param  OPA_Select This parameter can be one of the following values:
  *         @arg @ref DDL_OPA_CHANNEL_0
  *         @arg @ref DDL_OPA_CHANNEL_1
  * @param  Gain This parameter can be one of the following values:
  *         @arg @ref DDL_OPA_INTERNALGAIN_DISABLE
  *         @arg @ref DDL_OPA_INTERNALGAIN_1
  *         @arg @ref DDL_OPA_INTERNALGAIN_4
  *         @arg @ref DDL_OPA_INTERNALGAIN_6
  *         @arg @ref DDL_OPA_INTERNALGAIN_8
  *         @arg @ref DDL_OPA_INTERNALGAIN_10
  *         @arg @ref DDL_OPA_INTERNALGAIN_12
  *         @arg @ref DDL_OPA_INTERNALGAIN_16
  * @retval None
  */
__STATIC_INLINE void DDL_OPA_SetGain(OPA_TypeDef *OPAx, uint32_t OPA_Select, uint32_t Gain)
{
  uint32_t offset = OPA_Select & OPA_CHANNEL_BITOFFSET_MASK;
  MODIFY_REG(OPAx->CR, OPA_CR_OPA0SELGAIN << offset, Gain << offset);
}

/**
  * @brief  Return OPAx(x = 0 or 1) gain.
  * @param  OPAx OPA Instance
  * @param  OPA_Select This parameter can be one of the following values:
  *         @arg @ref DDL_OPA_CHANNEL_0
  *         @arg @ref DDL_OPA_CHANNEL_1
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_OPA_INTERNALGAIN_DISABLE
  *         @arg @ref DDL_OPA_INTERNALGAIN_1
  *         @arg @ref DDL_OPA_INTERNALGAIN_4
  *         @arg @ref DDL_OPA_INTERNALGAIN_6
  *         @arg @ref DDL_OPA_INTERNALGAIN_8
  *         @arg @ref DDL_OPA_INTERNALGAIN_10
  *         @arg @ref DDL_OPA_INTERNALGAIN_12
  *         @arg @ref DDL_OPA_INTERNALGAIN_16
  */
__STATIC_INLINE uint32_t DDL_OPA_GetGain(OPA_TypeDef *OPAx, uint32_t OPA_Select)
{
  uint32_t offset = OPA_Select & OPA_CHANNEL_BITOFFSET_MASK;
  return (READ_BIT(OPAx->CR, OPA_CR_OPA0SELGAIN << offset) >> (offset));
}

/**
  * @brief  Enable OPAx(x = 0 or 1) input control signal.
  * @param  OPAx OPA Instance
  * @param  OPA_Select This parameter can be one of the following values:
  *         @arg @ref DDL_OPA_CHANNEL_0
  *         @arg @ref DDL_OPA_CHANNEL_1
  * @retval None
  */
__STATIC_INLINE void DDL_OPA_EnableInputControl(OPA_TypeDef *OPAx, uint32_t OPA_Select)
{
  SET_BIT(OPAx->CR, OPA_CR_OPA0INSEL << (OPA_Select & OPA_CHANNEL_BITOFFSET_MASK));
}

/**
  * @brief  Disable OPAx(x = 0 or 1) input control signal.
  * @param  OPAx OPA Instance
  * @param  OPA_Select This parameter can be one of the following values:
  *         @arg @ref DDL_OPA_CHANNEL_0
  *         @arg @ref DDL_OPA_CHANNEL_1
  * @retval None
  */
__STATIC_INLINE void DDL_OPA_DisableInputControl(OPA_TypeDef *OPAx, uint32_t OPA_Select)
{
  CLEAR_BIT(OPAx->CR, OPA_CR_OPA0INSEL << (OPA_Select & OPA_CHANNEL_BITOFFSET_MASK));
}

/**
  * @brief  Checks if OPAx(x = 0 or 1) input control signal is enabled
  * @param  OPAx OPA Instance
  * @param  OPA_Select This parameter can be one of the following values:
  *         @arg @ref DDL_OPA_CHANNEL_0
  *         @arg @ref DDL_OPA_CHANNEL_1
  * @retval None
  */
__STATIC_INLINE uint32_t DDL_OPA_IsEnabledInputControl(OPA_TypeDef *OPAx, uint32_t OPA_Select)
{
  return ((READ_BIT(OPAx->CR, OPA_CR_OPA0INSEL << (OPA_Select & OPA_CHANNEL_BITOFFSET_MASK)) == \
                    (OPA_CR_OPA0INSEL << (OPA_Select & OPA_CHANNEL_BITOFFSET_MASK))) ? 1UL : 0UL);
}

/**
  * @brief  Enable OPAx(x = 0 or 1) output control signal.
  * @param  OPAx OPA Instance
  * @param  OPA_Select This parameter can be one of the following values:
  *         @arg @ref DDL_OPA_CHANNEL_0
  *         @arg @ref DDL_OPA_CHANNEL_1
  * @retval None
  */
__STATIC_INLINE void DDL_OPA_EnableOutputControl(OPA_TypeDef *OPAx, uint32_t OPA_Select)
{
  SET_BIT(OPAx->CR, OPA_CR_OPA0OUTSEL << (OPA_Select & OPA_CHANNEL_BITOFFSET_MASK));
}

/**
  * @brief  Disable OPAx(x = 0 or 1) output control signal.
  * @param  OPAx OPA Instance
  * @param  OPA_Select This parameter can be one of the following values:
  *         @arg @ref DDL_OPA_CHANNEL_0
  *         @arg @ref DDL_OPA_CHANNEL_1
  * @retval None
  */
__STATIC_INLINE void DDL_OPA_DisableOutputControl(OPA_TypeDef *OPAx, uint32_t OPA_Select)
{
  CLEAR_BIT(OPAx->CR, OPA_CR_OPA0OUTSEL << (OPA_Select & OPA_CHANNEL_BITOFFSET_MASK));
}

/**
  * @brief  Checks if OPAx(x = 0 or 1) output control signal is enabled
  * @param  OPAx OPA Instance
  * @param  OPA_Select This parameter can be one of the following values:
  *         @arg @ref DDL_OPA_CHANNEL_0
  *         @arg @ref DDL_OPA_CHANNEL_1
  * @retval None
  */
__STATIC_INLINE uint32_t DDL_OPA_IsEnabledOutputControl(OPA_TypeDef *OPAx, uint32_t OPA_Select)
{
  return ((READ_BIT(OPAx->CR, OPA_CR_OPA0OUTSEL << (OPA_Select & OPA_CHANNEL_BITOFFSET_MASK)) == \
                    (OPA_CR_OPA0OUTSEL << (OPA_Select & OPA_CHANNEL_BITOFFSET_MASK))) ? 1UL : 0UL);
}

/**
  * @brief  Set OPA0&1 built-in bias voltage signal.
  * @param  OPAx OPA Instance
  * @param  vcmselect This parameter can be one of the following values:
  *         @arg @ref DDL_OPA_VCMSEL_DISABLE
  *         @arg @ref DDL_OPA_VCMSEL_AVDD_0_5
  *         @arg @ref DDL_OPA_VCMSEL_AVDD_0_25
  *         @arg @ref DDL_OPA_VCMSEL_VBG
  *         @arg @ref DDL_OPA_VCMSEL_VBG_0_25
  * @retval None
  */
__STATIC_INLINE void DDL_OPA_SetVCM(OPA_TypeDef *OPAx, uint32_t vcmselect)
{
  MODIFY_REG(OPAx->CR, OPA_CR_OPASELVCM, vcmselect);
}

/**
  * @brief  Return OPA0&1 built-in bias voltage signal.
  * @param  OPAx OPA Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_OPA_VCMSEL_DISABLE
  *         @arg @ref DDL_OPA_VCMSEL_AVDD_0_5
  *         @arg @ref DDL_OPA_VCMSEL_AVDD_0_25
  *         @arg @ref DDL_OPA_VCMSEL_VBG
  *         @arg @ref DDL_OPA_VCMSEL_VBG_0_25
  */
__STATIC_INLINE uint32_t DDL_OPA_GetVCM(OPA_TypeDef *OPAx)
{
  return (READ_BIT(OPAx->CR, OPA_CR_OPASELVCM));
}

/**
  * @}
  */

#if defined(USE_FULL_DDL_DRIVER)
/** @defgroup OPA_DDL_EF_Init Initialization and de-initialization functions
  * @{
  */

ErrorStatus DDL_OPA_DeInit(OPA_TypeDef *OPAx);
ErrorStatus DDL_OPA_Init(OPA_TypeDef *OPAx, DDL_OPA_InitTypeDef *OPA_InitStruct);
void        DDL_OPA_StructInit(DDL_OPA_InitTypeDef *OPA_InitStruct);

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

#endif /* OPA */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* G32F031_DDL_OPA_H */
