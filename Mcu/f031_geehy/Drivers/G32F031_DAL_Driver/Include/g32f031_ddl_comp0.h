/**
  *
  * @file    g32f031_ddl_comp0.h
  * @brief   Header file of COMP0 DDL module.
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
  * Copyright (c) 2017 STMicroelectronics.
 *  Copyright (C) 2026 Geehy Semiconductor
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef G32F031_DDL_COMP0_H
#define G32F031_DDL_COMP0_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "g32f0xx.h"

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
/* Exported types ------------------------------------------------------------*/
#if defined(USE_FULL_DDL_DRIVER)
/** @defgroup COMP0_DDL_ES_INIT COMP0 Exported Init Structure
  * @{
  */

/**
 * @brief COMP0 Init structure definition
 */
typedef struct
{
    uint32_t InputPlus;            /*!< Set comparator input plus (non-inverting input).
                                        This parameter can be a value of @ref COMP0_DDL_EC_INPUT_PLUS

                                        This feature can be modified afterwards using unitary function @ref DDL_COMP0_SetInputPlus(). */

    uint32_t InputMinus;           /*!< Set comparator input minus (inverting input).
                                        This parameter can be a value of @ref COMP0_DDL_EC_INPUT_MINUS

                                        This feature can be modified afterwards using unitary function @ref DDL_COMP0_SetInputMinus(). */

    uint32_t OutputPol;            /*!< Set comparator output polarity.
                                        This parameter can be a value of @ref COMP0_DDL_EC_OUTPUT_POLARITY

                                        This feature can be modified afterwards using unitary function @ref DDL_COMP0_SetOutputPolarity(). */

    uint32_t FilterPSC;            /*!< Set comparator clock prescaler.
                                        This parameter can be a value of @ref COMP0_DDL_EC_OUTPUT_FILTERPSC

                                        This feature can be modified afterwards using unitary function @ref DDL_COMP0_SetFilterPSC(). */

    uint32_t FilterCFG;            /*!< Set comparator output filter configuration.
                                        This parameter can be a value of @ref COMP0_DDL_EC_OUTPUT_FILTERCFG

                                        This feature can be modified afterwards using unitary function @ref DDL_COMP0_SetFilterCFG(). */

    uint32_t HsyP;                 /*!< Set comparator output position hysteresis.
                                        This parameter can be a value of @ref COMP0_DDL_EC_OUTPUT_HYSP

                                        This feature can be modified afterwards using unitary function @ref DDL_COMP0_SetPositiveHysteresis(). */

    uint32_t HsyN;                 /*!< Set comparator output negative hysteresis.
                                        This parameter can be a value of @ref COMP0_DDL_EC_OUTPUT_HYSN

                                        This feature can be modified afterwards using unitary function @ref DDL_COMP0_SetNegativeHysteresis(). */

} DDL_COMP0_InitTypeDef;

/**
  * @}
  */
#endif /* USE_FULL_DDL_DRIVER */

/* Exported constants --------------------------------------------------------*/
/** @defgroup COMP0_DDL_Exported_Constants COMP0 Exported Constants
  * @{
  */

/** @defgroup COMP0_DDL_EC_INPUT_PLUS COMP0 input select
 * @{
 */
#define DDL_COMP0_INPUT_PLUS_PA7            (0x00000000UL)                           /*!< Input plus select: PA7. */
#define DDL_COMP0_INPUT_PLUS_PB9            (COMP_CR_VPSEL_0)                     /*!< Input plus select: PB9. */
#define DDL_COMP0_INPUT_PLUS_BG_2           (COMP_CR_VPSEL_1)                     /*!< Input plus select: BG/2. */
#define DDL_COMP0_INPUT_PLUS_BG             (COMP_CR_VPSEL_0 | COMP_CR_VPSEL_1)  /*!< Input plus select: BG. */
/**
  * @}
  */

/** @defgroup COMP0_DDL_EC_INPUT_MINUS COMP0 input select
 * @{
 */
#define DDL_COMP0_INPUT_MINUS_PA8           (0x00000000UL)                         /*!< Negative input select: PA8. */
#define DDL_COMP0_INPUT_MINUS_PA9           (COMP_CR_VNSEL_0)                     /*!< Negative input select: PA9. */
#define DDL_COMP0_INPUT_MINUS_PGA0          (COMP_CR_VNSEL_1)                     /*!< Negative input select: PGA0. */
#define DDL_COMP0_INPUT_MINUS_PGA1          (COMP_CR_VNSEL_0 | COMP_CR_VNSEL_1)  /*!< Negative input select: PGA1. */
/**
  * @}
  */

/** @defgroup COMP0_DDL_EC_OUTPUT_POLARITY COMP0 output polarity
 * @{
 */
#define DDL_COMP0_OUTPUTPOL_NONINVERTED     (0x00000000UL)            /*!< Comparator output is non-inverted. */
#define DDL_COMP0_OUTPUTPOL_INVERTED        (COMP_CR_POL)            /*!< Comparator output is inverted. */
/**
  * @}
  */

/** @defgroup COMP0_DDL_EC_OUTPUT_LEVEL COMP0 output level
 * @{
 */
#define DDL_COMP0_OUTPUT_LEVEL_LOW          (0x00000000UL)            /*!< Comparator output level is low. */
#define DDL_COMP0_OUTPUT_LEVEL_HIGH         (COMP_CR_VAL)          /*!< Comparator output level is high. */
/**
  * @}
  */

/** @defgroup COMP0_DDL_EC_OUTPUT_HYSP COMP0 hysteresis positive
 * @{
 */
#define DDL_COMP0_HYSP_DISABLE              (0x00000000UL)            /*!< Comparator positive hysteresis disable. */
#define DDL_COMP0_HYSP_20MV                 (COMP_CR_HYSPEN_0)      /*!< Comparator positive hysteresis 20mv. */
#define DDL_COMP0_HYSP_40MV                 (COMP_CR_HYSPEN_1)      /*!< Comparator positive hysteresis 40mv. */
#define DDL_COMP0_HYSP_60MV                 (COMP_CR_HYSPEN_Msk)    /*!< Comparator positive hysteresis 60mv. */
/**
  * @}
  */

/** @defgroup COMP0_DDL_EC_OUTPUT_HYSN COMP0 hysteresis negative
 * @{
 */
#define DDL_COMP0_HYSN_DISABLE              (0x00000000UL)            /*!< Comparator negative hysteresis disable. */
#define DDL_COMP0_HYSN_20MV                 (COMP_CR_HYSNEN_0)      /*!< Comparator negative hysteresis 20mv. */
#define DDL_COMP0_HYSN_40MV                 (COMP_CR_HYSNEN_1)      /*!< Comparator negative hysteresis 40mv. */
#define DDL_COMP0_HYSN_60MV                 (COMP_CR_HYSNEN_Msk)    /*!< Comparator negative hysteresis 60mv. */
/**
  * @}
  */

/** @defgroup COMP0_DDL_EC_OUTPUT_INTSEL COMP0 output interrupt select
 * @{
 */
#define DDL_COMP0_EDGE_INT_DISABLE          (0x00000000UL)            /*!< Output interrupt occur at none edge. */
#define DDL_COMP0_EDGE_INT_RISING           (COMP_CR_REN)           /*!< Output interrupt occur at rising edge. */
#define DDL_COMP0_EDGE_INT_FALLING          (COMP_CR_FEN)           /*!< Output interrupt occur at falling edge. */
#define DDL_COMP0_EDGE_INT_RISING_FALLING   (COMP_CR_RFEN)          /*!< Output interrupt occur at rising edge and falling edge. */
/**
  * @}
  */

/** @defgroup COMP0_DDL_EC_OUTPUT_FILTERCFG COMP0 Filter Configuration
 * @{
 */
#define DDL_COMP0_FILTERCFG_1               (0x00000000UL)                                        /*!< Comparator filter Configuration 1 x CLK. */
#define DDL_COMP0_FILTERCFG_2               (COMP_CR_CFG_0)                                      /*!< Comparator filter Configuration 2 x CLK. */
#define DDL_COMP0_FILTERCFG_4               (COMP_CR_CFG_1)                                      /*!< Comparator filter Configuration 4 x CLK. */
#define DDL_COMP0_FILTERCFG_8               (COMP_CR_CFG_0 | COMP_CR_CFG_1)                     /*!< Comparator filter Configuration 8 x CLK. */
#define DDL_COMP0_FILTERCFG_16              (COMP_CR_CFG_2)                                      /*!< Comparator filter Configuration 16 x CLK. */
#define DDL_COMP0_FILTERCFG_32              (COMP_CR_CFG_2 | COMP_CR_CFG_0)                     /*!< Comparator filter Configuration 32 x CLK. */
#define DDL_COMP0_FILTERCFG_64              (COMP_CR_CFG_2 | COMP_CR_CFG_1)                     /*!< Comparator filter Configuration 64 x CLK. */
#define DDL_COMP0_FILTERCFG_128             (COMP_CR_CFG_2 | COMP_CR_CFG_1 | COMP_CR_CFG_0)    /*!< Comparator filter Configuration 128 x CLK. */
#define DDL_COMP0_FILTERCFG_256             (COMP_CR_CFG_3)                                      /*!< Comparator filter Configuration 256 x CLK. */
#define DDL_COMP0_FILTERCFG_512             (COMP_CR_CFG_3 | COMP_CR_CFG_0)                     /*!< Comparator filter Configuration 512 x CLK. */
/**
  * @}
  */

/** @defgroup COMP0_DDL_EC_OUTPUT_FILTERPSC COMP0 filter clock prescaler
 * @{
 */
#define DDL_COMP0_FILTERPSC_1               (0x00000000UL)                                        /*!< Comparator filter prescaler PCLK/1. */
#define DDL_COMP0_FILTERPSC_2               (COMP_CR_PSC_0)                                      /*!< Comparator filter prescaler PCLK/2. */
#define DDL_COMP0_FILTERPSC_4               (COMP_CR_PSC_1)                                      /*!< Comparator filter prescaler PCLK/4 */
#define DDL_COMP0_FILTERPSC_8               (COMP_CR_PSC_0 | COMP_CR_PSC_1)                     /*!< Comparator filter prescaler PCLK/8 */
#define DDL_COMP0_FILTERPSC_16              (COMP_CR_PSC_2)                                      /*!< Comparator filter prescaler PCLK/16. */
#define DDL_COMP0_FILTERPSC_32              (COMP_CR_PSC_2 | COMP_CR_PSC_0)                     /*!< Comparator filter prescaler PCLK/32. */
#define DDL_COMP0_FILTERPSC_64              (COMP_CR_PSC_2 | COMP_CR_PSC_1)                     /*!< Comparator filter prescaler PCLK/64. */
#define DDL_COMP0_FILTERPSC_128             (COMP_CR_PSC_2 | COMP_CR_PSC_1 | COMP_CR_PSC_0)    /*!< Comparator filter prescaler PCLK/128. */
/**
  * @}
  */

/** @defgroup COMP0_DDL_EC_HW_DELAYS COMP0 hardware delay
 * @{
 */

/* Delay for comparator startup time */
#define DDL_COMP0_DELAY_STARTUP_US          (80UL)                    /*!< Comparator startup time. */
/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup COMP0_DDL_Exported_Macros COMP0 Exported Macros
  * @{
  */
/** @defgroup COMP0_DDL_EM_WRITE_READ Common write and read registers macro
  * @{
  */

/**
 * @brief  Write a value in COMP0 register.
 * @param  __INSTANCE__ COMP0 submodule instance.
 * @param  __REG__ COMP0 register.
 * @param  __VALUE__ Value to be written.
 * @retval None
 */
#define DDL_COMP0_WriteReg(__INSTANCE__, __REG__, __VALUE__)     WRITE_REG(__INSTANCE__->__REG__, (__VALUE__))

/**
 * @brief  Read a value in COMP0 register.
 * @param  __INSTANCE__ COMP0 submodule instance.
 * @param  __REG__ COMP0 register.
 * @retval Register value
 */
#define DDL_COMP0_ReadReg(__INSTANCE__, __REG__)                 READ_REG(__INSTANCE__->__REG__)
/**
  * @}
  */

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup COMP0_DDL_Exported_Functions COMP0 Exported Functions
  * @{
  */

/** @defgroup COMP0_DDL_EF_Configuration_comparator_modes Configuration of comparator instance mode
 * @{
 */

/**
 * @brief  Set comparator filter clock prescaler.
 * @param  COMP0x COMP0 submodule instance.
 * @param  psc This parameter can be one of the following values:
 *         @arg @ref DDL_COMP0_FILTERPSC_1
 *         @arg @ref DDL_COMP0_FILTERPSC_2
 *         @arg @ref DDL_COMP0_FILTERPSC_4
 *         @arg @ref DDL_COMP0_FILTERPSC_8
 *         @arg @ref DDL_COMP0_FILTERPSC_16
 *         @arg @ref DDL_COMP0_FILTERPSC_32
 *         @arg @ref DDL_COMP0_FILTERPSC_64
 *         @arg @ref DDL_COMP0_FILTERPSC_128
 * @retval None
 */
__STATIC_INLINE void DDL_COMP0_SetFilterPSC(COMP0_TypeDef *COMP0x, uint32_t psc)
{
  MODIFY_REG(COMP0x->CR, COMP_CR_PSC, psc);
}

/**
 * @brief Get comparator clock prescaler.
 * @param  COMP0x COMP0 submodule instance.
 * @retval Returned value can be one of the following values:
 *         @arg @ref DDL_COMP0_FILTERPSC_1
 *         @arg @ref DDL_COMP0_FILTERPSC_2
 *         @arg @ref DDL_COMP0_FILTERPSC_4
 *         @arg @ref DDL_COMP0_FILTERPSC_8
 *         @arg @ref DDL_COMP0_FILTERPSC_16
 *         @arg @ref DDL_COMP0_FILTERPSC_32
 *         @arg @ref DDL_COMP0_FILTERPSC_64
 *         @arg @ref DDL_COMP0_FILTERPSC_128
 */
__STATIC_INLINE uint32_t DDL_COMP0_GetFilterPSC(COMP0_TypeDef *COMP0x)
{
  return (uint32_t)(READ_BIT(COMP0x->CR, COMP_CR_PSC));
}

/**
 * @brief  Set comparator filter configuration.
 * @param  COMP0x COMP0 submodule instance.
 * @param  Filter This parameter can be one of the following values:
 *         @arg @ref DDL_COMP0_FILTERCFG_1
 *         @arg @ref DDL_COMP0_FILTERCFG_2
 *         @arg @ref DDL_COMP0_FILTERCFG_4
 *         @arg @ref DDL_COMP0_FILTERCFG_8
 *         @arg @ref DDL_COMP0_FILTERCFG_16
 *         @arg @ref DDL_COMP0_FILTERCFG_32
 *         @arg @ref DDL_COMP0_FILTERCFG_64
 *         @arg @ref DDL_COMP0_FILTERCFG_128
 *         @arg @ref DDL_COMP0_FILTERCFG_256
 *         @arg @ref DDL_COMP0_FILTERCFG_512
 * @retval None
 */
__STATIC_INLINE void DDL_COMP0_SetFilterCFG(COMP0_TypeDef *COMP0x, uint32_t cfg)
{
  MODIFY_REG(COMP0x->CR, COMP_CR_CFG, cfg);
}

/**
 * @brief Get comparator filter configuration.
 * @param  COMP0x COMP0 submodule instance.
 * @retval Returned value can be one of the following values:
 *         @arg @ref DDL_COMP0_FILTERCFG_1
 *         @arg @ref DDL_COMP0_FILTERCFG_2
 *         @arg @ref DDL_COMP0_FILTERCFG_4
 *         @arg @ref DDL_COMP0_FILTERCFG_8
 *         @arg @ref DDL_COMP0_FILTERCFG_16
 *         @arg @ref DDL_COMP0_FILTERCFG_32
 *         @arg @ref DDL_COMP0_FILTERCFG_64
 *         @arg @ref DDL_COMP0_FILTERCFG_128
 *         @arg @ref DDL_COMP0_FILTERCFG_256
 *         @arg @ref DDL_COMP0_FILTERCFG_512
 */
__STATIC_INLINE uint32_t DDL_COMP0_GetFilterCFG(COMP0_TypeDef *COMP0x)
{
  return (uint32_t)(READ_BIT(COMP0x->CR, COMP_CR_CFG));
}


/**
 * @brief  Set comparator hysteresis.
 * @param  COMP0x COMP0 submodule instance.
 * @param  Hsy This parameter can be one of the following values:
 *         @arg @ref DDL_COMP0_HYSP_DISABLE
 *         @arg @ref DDL_COMP0_HYSP_20MV
 *         @arg @ref DDL_COMP0_HYSP_40MV
 *         @arg @ref DDL_COMP0_HYSP_60MV
 * @retval None
 */
__STATIC_INLINE void DDL_COMP0_SetPositiveHysteresis(COMP0_TypeDef *COMP0x, uint32_t HsyP)
{
  MODIFY_REG(COMP0x->CR, COMP_CR_HYSPEN, HsyP);
}

/**
 * @brief  Get comparator hysteresis.
 * @param  COMP0x COMP0 submodule instance.
 * @retval Returned value can be one of the following values:
 *         @arg @ref DDL_COMP0_HYSP_DISABLE
 *         @arg @ref DDL_COMP0_HYSP_20MV
 *         @arg @ref DDL_COMP0_HYSP_40MV
 *         @arg @ref DDL_COMP0_HYSP_60MV
 */
__STATIC_INLINE uint32_t DDL_COMP0_GetPositiveHysteresis(COMP0_TypeDef *COMP0x)
{
  return (uint32_t)(READ_BIT(COMP0x->CR, COMP_CR_HYSPEN));
}

/**
 * @brief  Set comparator hysteresis.
 * @param  COMP0x COMP0 submodule instance.
 * @param  Hsy This parameter can be one of the following values:
 *         @arg @ref DDL_COMP0_HYSN_DISABLE
 *         @arg @ref DDL_COMP0_HYSN_20MV
 *         @arg @ref DDL_COMP0_HYSN_40MV
 *         @arg @ref DDL_COMP0_HYSN_60MV
 * @retval None
 */
__STATIC_INLINE void DDL_COMP0_SetNegativeHysteresis(COMP0_TypeDef *COMP0x, uint32_t HsyN)
{
  MODIFY_REG(COMP0x->CR, COMP_CR_HYSNEN, HsyN);
}

/**
 * @brief  Get comparator hysteresis.
 * @param  COMP0x COMP0 submodule instance.
 * @retval Returned value can be one of the following values:
 *         @arg @ref DDL_COMP0_HYSN_DISABLE
 *         @arg @ref DDL_COMP0_HYSN_20MV
 *         @arg @ref DDL_COMP0_HYSN_40MV
 *         @arg @ref DDL_COMP0_HYSN_60MV
 */
__STATIC_INLINE uint32_t DDL_COMP0_GetNegativeHysteresis(COMP0_TypeDef *COMP0x)
{
  return (uint32_t)(READ_BIT(COMP0x->CR, COMP_CR_HYSNEN));
}

/**
  * @}
  */

/** @defgroup COMP0_DDL_EF_Configuration_comparator_inputs Configuration of comparator inputs
 * @{
 */

/**
 * @brief  Set comparator input plus.
 * @param  COMP0x COMP0 submodule instance.
 * @param  InputPlus This parameter can be one of the following values:
 *         @arg @ref DDL_COMP0_INPUT_PLUS_PA7
 *         @arg @ref DDL_COMP0_INPUT_PLUS_PB9
 *         @arg @ref DDL_COMP0_INPUT_PLUS_BG_2
 *         @arg @ref DDL_COMP0_INPUT_PLUS_BG
 * @retval None
 */
__STATIC_INLINE void DDL_COMP0_SetInputPlus(COMP0_TypeDef *COMP0x, uint32_t InputPlus)
{
  MODIFY_REG(COMP0x->CR, COMP_CR_VPSEL, InputPlus);
}

/**
 * @brief Get comparator input plus.
 * @param  COMP0x COMP0 submodule instance.
 * @retval Returned value can be one of the following values:
 *         @arg @ref DDL_COMP0_INPUT_PLUS_PA7
 *         @arg @ref DDL_COMP0_INPUT_PLUS_PB9
 *         @arg @ref DDL_COMP0_INPUT_PLUS_BG_2
 *         @arg @ref DDL_COMP0_INPUT_PLUS_BG
 */
__STATIC_INLINE uint32_t DDL_COMP0_GetInputPlus(COMP0_TypeDef *COMP0x)
{
  return (uint32_t)(READ_BIT(COMP0x->CR, COMP_CR_VPSEL));
}

/**
 * @brief  Set comparator input minus.
 * @param  COMP0x COMP0 submodule instance.
 * @param  InputSel This parameter can be one of the following values:
 *         @arg @ref DDL_COMP0_INPUT_MINUS_PA8
 *         @arg @ref DDL_COMP0_INPUT_MINUS_PA9
 *         @arg @ref DDL_COMP0_INPUT_MINUS_PGA0
 *         @arg @ref DDL_COMP0_INPUT_MINUS_PGA1
 * @retval None
 */
__STATIC_INLINE void DDL_COMP0_SetInputMinus(COMP0_TypeDef *COMP0x, uint32_t InputSel)
{
  MODIFY_REG(COMP0x->CR, COMP_CR_VNSEL, InputSel);
}

/**
 * @brief Get comparator input minus.
 * @param  COMP0x COMP0 submodule instance.
 * @retval Returned value can be one of the following values:
 *         @arg @ref DDL_COMP0_INPUT_MINUS_PA8
 *         @arg @ref DDL_COMP0_INPUT_MINUS_PA9
 *         @arg @ref DDL_COMP0_INPUT_MINUS_PGA0
 *         @arg @ref DDL_COMP0_INPUT_MINUS_PGA1
 */
__STATIC_INLINE uint32_t DDL_COMP0_GetInputMinus(COMP0_TypeDef *COMP0x)
{
  return (uint32_t)(READ_BIT(COMP0x->CR, COMP_CR_VNSEL));
}

/**
  * @}
  */

/** @defgroup COMP0_DDL_EF_Configuration_comparator_output Configuration of comparator output
 * @{
 */

/**
 * @brief  Set comparator output polarity.
 * @param  COMP0x COMP0 submodule instance.
 * @param  OutputPol This parameter can be one of the following values:
 *        @arg @ref DDL_COMP0_OUTPUTPOL_NONINVERTED
 *        @arg @ref DDL_COMP0_OUTPUTPOL_INVERTED
 * @retval None
 */
__STATIC_INLINE void DDL_COMP0_SetOutputPolarity(COMP0_TypeDef *COMP0x, uint32_t OutputPol)
{
  MODIFY_REG(COMP0x->CR, COMP_CR_POL, OutputPol);
}

/**
 * @brief Get comparator output polarity.
 * @param  COMP0x COMP0 submodule instance.
 * @retval Returned value can be one of the following values:
 *       @arg @ref DDL_COMP0_OUTPUTPOL_NONINVERTED
 *       @arg @ref DDL_COMP0_OUTPUTPOL_INVERTED
 */
__STATIC_INLINE uint32_t DDL_COMP0_GetOutputPolarity(COMP0_TypeDef *COMP0x)
{
  return (uint32_t)(READ_BIT(COMP0x->CR, COMP_CR_POL));
}

/**
  * @}
  */

 /** @defgroup COMP0_DDL_EF_Operation Operation on comparator instance
  * @{
  */

/**
 * @brief  Enable comparator instance.
 * @param  COMP0x COMP0 submodule instance.
 * @retval None
 * @note   After enable from off state, comparator startup time is needed.
 */
__STATIC_INLINE void DDL_COMP0_Enable(COMP0_TypeDef *COMP0x)
{
  SET_BIT(COMP0x->CR, COMP_CR_COMPEN);
}

/**
 * @brief  Disable comparator instance.
 * @param  COMP0x COMP0 submodule instance.
 * @retval None
 */
__STATIC_INLINE void DDL_COMP0_Disable(COMP0_TypeDef *COMP0x)
{
  CLEAR_BIT(COMP0x->CR, COMP_CR_COMPEN);
}

/**
 * @brief  Get comparator enable state.
 * @param  COMP0x COMP0 submodule instance.
 * @retval State of bit (1 or 0).
 *        - 0: comparator instance is disabled.
 *        - 1: comparator instance is enabled.
 */
__STATIC_INLINE uint32_t DDL_COMP0_IsEnabled(COMP0_TypeDef *COMP0x)
{
  return ((READ_BIT(COMP0x->CR, COMP_CR_COMPEN) == (COMP_CR_COMPEN)) ? 1UL : 0UL);
}

/**
 * @brief  Lock comparator instance.
 * @param  COMP0x COMP0 submodule instance.
 * @retval None
 * @note   Once locked, comparator configuration can no longer be modified until next reset.
 * @note   The only way to unlock the comparator is a device hardware reset.
 */
__STATIC_INLINE void DDL_COMP0_Lock(COMP0_TypeDef *COMP0x)
{
  SET_BIT(COMP0x->CR, COMP_CR_LOCK);
}

/**
 * @brief  Get comparator lock state.
 * @param  COMP0x COMP0 submodule instance.
 * @retval State of bit (1 or 0).
 *       - 0: comparator instance is unlocked.
 *       - 1: comparator instance is locked.
 */
__STATIC_INLINE uint32_t DDL_COMP0_IsLocked(COMP0_TypeDef *COMP0x)
{
  return ((READ_BIT(COMP0x->CR, COMP_CR_LOCK) == (COMP_CR_LOCK)) ? 1UL : 0UL);
}

/**
 * @brief  Read comparator output level.
 * @param  COMP0x COMP0 submodule instance.
 * @retval Returned value can be one of the following values:
 *       @arg @ref DDL_COMP0_OUTPUT_LEVEL_LOW
 *       @arg @ref DDL_COMP0_OUTPUT_LEVEL_HIGH
 * @note   The comparator output level depends on the selected polarity.
 *         If the polarity is not inverted:
 *         - Comparator output is low level when the input plus is lower than the input minus.
 *         - Comparator output is high level when the input plus is higher than the input minus.
 *         If the polarity is inverted:
 *         - Comparator output is high level when the input plus is lower than the input minus.
 *         - Comparator output is low level when the input plus is higher than the input minus.
 */
__STATIC_INLINE uint32_t DDL_COMP0_ReadOutputLevel(COMP0_TypeDef *COMP0x)
{
  return (uint32_t)(READ_BIT(COMP0x->CR, COMP_CR_VAL));
}

/**
  * @}
  */

/** @defgroup COMP0_DDL_EF_Interrupt Interrupt
 * @{
 */

/**
 * @brief  Enable comparator SW Interrupt.
 * @param  COMP0x COMP0 submodule instance.
 * @retval None
 */
__STATIC_INLINE void DDL_COMP0_EnableSWINT(COMP0_TypeDef *COMP0x)
{
  SET_BIT(COMP0x->CR, COMP_CR_SWEG);
}

/**
 * @brief  Disable comparator SW Interrupt.
 * @param  COMP0x COMP0 submodule instance.
 * @retval None
 */
__STATIC_INLINE void DDL_COMP0_DisableSWINT(COMP0_TypeDef *COMP0x)
{
  CLEAR_BIT(COMP0x->CR, COMP_CR_SWEG);
}

/**
  * @brief  Set the COMP0 interrupt.
  * @param  COMP0x COMP0 submodule instance.
  * @param  InputSel This parameter can be one of the following values:
  *         @arg @ref DDL_COMP0_EDGE_INT_DISABLE
  *         @arg @ref DDL_COMP0_EDGE_INT_RISING
  *         @arg @ref DDL_COMP0_EDGE_INT_FALLING
  *         @arg @ref DDL_COMP0_EDGE_INT_RISING_FALLING
  * @retval None
  */
__STATIC_INLINE void DDL_COMP0_SetInterrupt(COMP0_TypeDef *COMP0x, uint32_t Interrupt)
{
  MODIFY_REG(COMP0x->CR, (COMP_CR_REN | COMP_CR_FEN | COMP_CR_RFEN), Interrupt);
}

/**
  * @brief  Get the COMP0 interrupt.
  * @param  COMP0x COMP0 submodule instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_COMP0_EDGE_INT_DISABLE
  *         @arg @ref DDL_COMP0_EDGE_INT_RISING
  *         @arg @ref DDL_COMP0_EDGE_INT_FALLING
  *         @arg @ref DDL_COMP0_EDGE_INT_RISING_FALLING
  */
__STATIC_INLINE uint32_t DDL_COMP0_GetInterrupt(COMP0_TypeDef *COMP0x)
{
  return (uint32_t)(READ_BIT(COMP0x->CR, (COMP_CR_REN | COMP_CR_FEN | COMP_CR_RFEN)));
}

/**
  * @}
  */

/** @defgroup COMP0_DDL_EF_FLAG_Management FLAG-Management
 * @{
 */

/**
  * @brief  Clear the interrupt flag.
  * @param  COMP0x COMP0 submodule instance.
  * @retval None
  */
__STATIC_INLINE void DDL_COMP0_ClearFlag_IT(COMP0_TypeDef *COMP0x)
{
  SET_BIT(COMP0x->ISR, COMP_ISR_IFLG);
}

/**
  * @brief  Indicate whether interrupt flag is set.
  * @param  COMP0x COMP0 submodule instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_COMP0_IsActiveFlag_IT(COMP0_TypeDef *COMP0x)
{
  return ((READ_BIT(COMP0x->ISR, COMP_ISR_IFLG) == (COMP_ISR_IFLG)) ? 1UL : 0UL);
}


/**
  * @}
  */


#if defined(USE_FULL_DDL_DRIVER)
/** @defgroup COMP0_DDL_EF_Init Initialization and de-initialization functions
 * @{
 */

ErrorStatus DDL_COMP0_Init(COMP0_TypeDef *COMP0x, DDL_COMP0_InitTypeDef *COMP0_InitStruct);
ErrorStatus DDL_COMP0_DeInit(COMP0_TypeDef *COMP0x);
void        DDL_COMP0_StructInit(DDL_COMP0_InitTypeDef *COMP0_InitStruct);

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

 #endif /* COMP0 */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* G32F037_DDL_COMP0_H */

