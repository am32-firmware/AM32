/**
  *
  * @file    g32f031_ddl_comp1.h
  * @brief   Header file of COMP1 DDL module.
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
#ifndef G32F031_DDL_COMP1_H
#define G32F031_DDL_COMP1_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "g32f0xx.h"

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
/* Exported types ------------------------------------------------------------*/
#if defined(USE_FULL_DDL_DRIVER)
/** @defgroup COMP1_DDL_ES_INIT COMP1 Exported Init Structure
  * @{
  */

/**
 * @brief COMP1 Init structure definition
 */
typedef struct
{
    uint32_t InputPlus;                 /*!< Set comparator input plus (non-inverting input).
                                        This parameter can be a value of @ref COMP1_DDL_EC_INPUT_PLUS

                                        This feature can be modified afterwards using unitary function @ref DDL_COMP1_SetInputPlus(). */

    uint32_t InputMinus;                /*!< Set comparator input minus (inverting input).
                                        This parameter can be a value of @ref COMP1_DDL_EC_INPUT_MINUS

                                        This feature can be modified afterwards using unitary function @ref DDL_COMP1_SetInputMinus(). */

    uint32_t OutputPol;             /*!< Set comparator output polarity.
                                        This parameter can be a value of @ref COMP1_DDL_EC_OUTPUT_POLARITY

                                        This feature can be modified afterwards using unitary function @ref DDL_COMP1_SetOutputPolarity(). */

    uint32_t FilterPSC;                /*!< Set comparator clock prescaler.
                                        This parameter can be a value of @ref COMP1_DDL_EC_OUTPUT_FILTERPSC

                                        This feature can be modified afterwards using unitary function @ref DDL_COMP1_SetFilterPSC(). */

    uint32_t FilterCFG;                /*!< Set comparator output filter configuration.
                                        This parameter can be a value of @ref COMP1_DDL_EC_OUTPUT_FILTERCFG

                                        This feature can be modified afterwards using unitary function @ref DDL_COMP1_SetFilterCFG(). */

    uint32_t HsyP;                /*!< Set comparator output position hysteresis.
                                        This parameter can be a value of @ref COMP1_DDL_EC_OUTPUT_HYSP

                                        This feature can be modified afterwards using unitary function @ref DDL_COMP1_SetPositiveHysteresis(). */

    uint32_t HsyN;                /*!< Set comparator output negative hysteresis.
                                        This parameter can be a value of @ref COMP1_DDL_EC_OUTPUT_HYSN

                                        This feature can be modified afterwards using unitary function @ref DDL_COMP1_SetNegativeHysteresis(). */

} DDL_COMP1_InitTypeDef;

/**
  * @}
  */
#endif /* USE_FULL_DDL_DRIVER */

/* Exported constants --------------------------------------------------------*/
/** @defgroup COMP1_DDL_Exported_Constants COMP1 Exported Constants
  * @{
  */

/** @defgroup COMP1_DDL_EC_INPUT_PLUS COMP1 input select
 * @{
 */
#define DDL_COMP1_INPUT_PLUS_PB6            (0x00000000UL)                           /*!< Input plus select: PA7. */
#define DDL_COMP1_INPUT_PLUS_VIRTUAL        (COMP_CR_VPSEL_0)        /*!< Input plus select: Internal virtual. */
/**
 * @}
 */

/** @defgroup COMP1_DDL_EC_INPUT_MINUS COMP1 input select
 * @{
 */
#define DDL_COMP1_INPUT_MINUS_PA1           (0x00000000UL)            /*!< Negative Input select: PA1. */
#define DDL_COMP1_INPUT_MINUS_PB1           (COMP_CR_VNSEL_0)        /*!< Negative Input select: PB1. */
#define DDL_COMP1_INPUT_MINUS_PA2           (0x00000000UL)            /*!< Negative Input select: PA2. */
#define DDL_COMP1_INPUT_MINUS_PB2           (COMP_CR_VNSEL_0)        /*!< Negative Input select: PB2. */
#define DDL_COMP1_INPUT_MINUS_PA0           (0x00000000UL)            /*!< Negative Input select: PA0. */
#define DDL_COMP1_INPUT_MINUS_PB5           (COMP_CR_VNSEL_0)        /*!< Negative Input select: PB5. */
/**
 * @}
 */

/** @defgroup COMP1_DDL_EC_OUTPUT_POLARITY COMP1 output polarity
 * @{
 */
#define DDL_COMP1_OUTPUTPOL_NONINVERTED     (0x00000000UL)            /*!< Comparator output is non-inverted. */
#define DDL_COMP1_OUTPUTPOL_INVERTED        (COMP_CR_POL)            /*!< Comparator output is inverted. */
/**
 * @}
 */

/** @defgroup COMP1_DDL_EC_OUTPUT_LEVEL COMP1 output level
 * @{
 */
#define DDL_COMP1_OUTPUT_LEVEL_LOW          (0x00000000UL)            /*!< Comparator output level is low. */
#define DDL_COMP1_OUTPUT_LEVEL_HIGH         (COMP_CR_VAL)          /*!< Comparator output level is high. */
/**
 * @}
 */

/** @defgroup COMP1_DDL_EC_OUTPUT_HYSP COMP1 hysteresis positive
 * @{
 */
#define DDL_COMP1_HYSP_DISABLE              (0x00000000UL)            /*!< Comparator positive hysteresis disable. */
#define DDL_COMP1_HYSP_20MV                 (COMP_CR_HYSPEN_0)      /*!< Comparator positive hysteresis 20mv. */
#define DDL_COMP1_HYSP_40MV                 (COMP_CR_HYSPEN_1)      /*!< Comparator positive hysteresis 40mv. */
#define DDL_COMP1_HYSP_60MV                 (COMP_CR_HYSPEN_Msk)    /*!< Comparator positive hysteresis 60mv. */
/**
 * @}
 */

/** @defgroup COMP1_DDL_EC_OUTPUT_HYSN COMP1 hysteresis negative
 * @{
 */
#define DDL_COMP1_HYSN_DISABLE              (0x00000000UL)            /*!< Comparator negative hysteresis disable. */
#define DDL_COMP1_HYSN_20MV                 (COMP_CR_HYSNEN_0)      /*!< Comparator negative hysteresis 20mv. */
#define DDL_COMP1_HYSN_40MV                 (COMP_CR_HYSNEN_1)      /*!< Comparator negative hysteresis 40mv. */
#define DDL_COMP1_HYSN_60MV                 (COMP_CR_HYSNEN_Msk)    /*!< Comparator negative hysteresis 60mv. */
/**
 * @}
 */

/** @defgroup COMP1_DDL_EC_OUTPUT_INTSEL COMP1 output interrupt select
 * @{
 */
#define DDL_COMP1_EDGE_INT_DISABLE          (0x00000000UL)            /*!< Comparator negative hysteresis disable. */
#define DDL_COMP1_EDGE_INT_RISING           (COMP_CR_REN)           /*!< Comparator negative hysteresis 20mv. */
#define DDL_COMP1_EDGE_INT_FALLING          (COMP_CR_FEN)           /*!< Comparator negative hysteresis 40mv. */
#define DDL_COMP1_EDGE_INT_RISING_FALLING   (COMP_CR_RFEN)          /*!< Comparator negative hysteresis 60mv. */
/**
 * @}
 */

/** @defgroup COMP1_DDL_EC_OUTPUT_FILTERCFG COMP1 Filter Configuration
 * @{
 */
#define DDL_COMP1_FILTERCFG_1               (0x00000000UL)                                        /*!< Comparator filter Configuration 1 x CLK. */
#define DDL_COMP1_FILTERCFG_2               (COMP_CR_CFG_0)                                      /*!< Comparator filter Configuration 2 x CLK. */
#define DDL_COMP1_FILTERCFG_4               (COMP_CR_CFG_1)                                      /*!< Comparator filter Configuration 4 x CLK. */
#define DDL_COMP1_FILTERCFG_8               (COMP_CR_CFG_0  | COMP_CR_CFG_1)                    /*!< Comparator filter Configuration 8 x CLK. */
#define DDL_COMP1_FILTERCFG_16              (COMP_CR_CFG_2)                                      /*!< Comparator filter Configuration 16 x CLK. */
#define DDL_COMP1_FILTERCFG_32              (COMP_CR_CFG_2  | COMP_CR_CFG_0)                    /*!< Comparator filter Configuration 32 x CLK. */
#define DDL_COMP1_FILTERCFG_64              (COMP_CR_CFG_2  | COMP_CR_CFG_1)                    /*!< Comparator filter Configuration 64 x CLK. */
#define DDL_COMP1_FILTERCFG_128             (COMP_CR_CFG_2  | COMP_CR_CFG_1  | COMP_CR_CFG_0)  /*!< Comparator filter Configuration 128 x CLK. */
#define DDL_COMP1_FILTERCFG_256             (COMP_CR_CFG_3)                                      /*!< Comparator filter Configuration 256 x CLK. */
#define DDL_COMP1_FILTERCFG_512             (COMP_CR_CFG_3  | COMP_CR_CFG_0)                    /*!< Comparator filter Configuration 512 x CLK. */
/**
 * @}
 */

/** @defgroup COMP1_DDL_EC_OUTPUT_FILTERPSC COMP1 filter clock prescaler
 * @{
 */
#define DDL_COMP1_FILTERPSC_1               (0x00000000UL)                                     /*!< Comparator filter prescaler PCLK/1. */
#define DDL_COMP1_FILTERPSC_2               (COMP_CR_PSC_0)                                   /*!< Comparator filter prescaler PCLK/2. */
#define DDL_COMP1_FILTERPSC_4               (COMP_CR_PSC_1)                                   /*!< Comparator filter prescaler PCLK/4 */
#define DDL_COMP1_FILTERPSC_8               (COMP_CR_PSC_0 | COMP_CR_PSC_1)                  /*!< Comparator filter prescaler PCLK/8 */
#define DDL_COMP1_FILTERPSC_16              (COMP_CR_PSC_2)                                   /*!< Comparator filter prescaler PCLK/16. */
#define DDL_COMP1_FILTERPSC_32              (COMP_CR_PSC_2 | COMP_CR_PSC_0)                  /*!< Comparator filter prescaler PCLK/32. */
#define DDL_COMP1_FILTERPSC_64              (COMP_CR_PSC_2 | COMP_CR_PSC_1)                  /*!< Comparator filter prescaler PCLK/64. */
#define DDL_COMP1_FILTERPSC_128             (COMP_CR_PSC_2 | COMP_CR_PSC_1 | COMP_CR_PSC_0) /*!< Comparator filter prescaler PCLK/128. */
/**
 * @}
 */

/** @defgroup COMP1_DDL_EC_HW_DELAYS COMP1 hardware delay
 * @{
 */

/* Delay for comparator startup time */
#define DDL_COMP1_DELAY_STARTUP_US               (80UL)                      /*!< Comparator startup time. */
/**
 * @}
 */
#define DDL_COMP1_GET_INDEX(__COMP__)       ((__COMP__) == COMP1 ? 0U : \
                                             (__COMP__) == COMP2 ? 1U : 2U)
/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup COMP1_DDL_Exported_Macros COMP1 Exported Macros
  * @{
  */
/** @defgroup COMP1_DDL_EM_WRITE_READ Common write and read registers macro
  * @{
  */

/**
 * @brief  Write a value in COMP1 register.
 * @param  __INSTANCE__ COMP1 submodule instance.
 * @param  __REG__ COMP1 register.
 * @param  __VALUE__ Value to be written.
 * @retval None
 */
#define DDL_COMP1_WriteReg(__INSTANCE__, __REG__, __VALUE__)     WRITE_REG(__INSTANCE__->__REG__, (__VALUE__))

/**
 * @brief  Read a value in COMP1 register.
 * @param  __INSTANCE__ COMP1 submodule instance.
 * @param  __REG__ COMP1 register.
 * @retval Register value
 */
#define DDL_COMP1_ReadReg(__INSTANCE__, __REG__)                 READ_REG(__INSTANCE__->__REG__)
/**
  * @}
  */

/**
 * @}
 */

/* Exported functions --------------------------------------------------------*/
/** @defgroup COMP1_DDL_Exported_Functions COMP1 Exported Functions
  * @{
  */

/** @defgroup COMP1_DDL_EF_Configuration_comparator_modes Configuration of comparator instance mode
 * @{
 */

/**
 * @brief  Set comparator filter clock prescaler.
 * @param  COMP1x COMP1 submodule instance.
 * @param  psc This parameter can be one of the following values:
 *         @arg @ref DDL_COMP1_FILTERPSC_1
 *         @arg @ref DDL_COMP1_FILTERPSC_2
 *         @arg @ref DDL_COMP1_FILTERPSC_4
 *         @arg @ref DDL_COMP1_FILTERPSC_8
 *         @arg @ref DDL_COMP1_FILTERPSC_16
 *         @arg @ref DDL_COMP1_FILTERPSC_32
 *         @arg @ref DDL_COMP1_FILTERPSC_64
 *         @arg @ref DDL_COMP1_FILTERPSC_128
 * @retval None
 */
__STATIC_INLINE void DDL_COMP1_SetFilterPSC(COMP1_TypeDef *COMP1x, uint32_t psc)
{
  MODIFY_REG(COMP1x->CR, COMP_CR_PSC, psc);
}

/**
 * @brief Get comparator clock prescaler.
 * @param  COMP1x COMP1 submodule instance.
 * @retval Returned value can be one of the following values:
 *         @arg @ref DDL_COMP1_FILTERPSC_1
 *         @arg @ref DDL_COMP1_FILTERPSC_2
 *         @arg @ref DDL_COMP1_FILTERPSC_4
 *         @arg @ref DDL_COMP1_FILTERPSC_8
 *         @arg @ref DDL_COMP1_FILTERPSC_16
 *         @arg @ref DDL_COMP1_FILTERPSC_32
 *         @arg @ref DDL_COMP1_FILTERPSC_64
 *         @arg @ref DDL_COMP1_FILTERPSC_128
 */
__STATIC_INLINE uint32_t DDL_COMP1_GetFilterPSC(COMP1_TypeDef *COMP1x)
{
  return (READ_BIT(COMP1x->CR, COMP_CR_PSC));
}

/**
 * @brief  Set comparator filter configuration.
 * @param  COMP1x COMP1 submodule instance.
 * @param  Filter This parameter can be one of the following values:
 *         @arg @ref DDL_COMP1_FILTERCFG_1
 *         @arg @ref DDL_COMP1_FILTERCFG_2
 *         @arg @ref DDL_COMP1_FILTERCFG_4
 *         @arg @ref DDL_COMP1_FILTERCFG_8
 *         @arg @ref DDL_COMP1_FILTERCFG_16
 *         @arg @ref DDL_COMP1_FILTERCFG_32
 *         @arg @ref DDL_COMP1_FILTERCFG_64
 *         @arg @ref DDL_COMP1_FILTERCFG_128
 *         @arg @ref DDL_COMP1_FILTERCFG_256
 *         @arg @ref DDL_COMP1_FILTERCFG_512
 * @retval None
 */
__STATIC_INLINE void DDL_COMP1_SetFilterCFG(COMP1_TypeDef *COMP1x, uint32_t cfg)
{
  MODIFY_REG(COMP1x->CR, COMP_CR_CFG, cfg);
}

/**
 * @brief Get comparator filter configuration.
 * @param  COMP1x COMP1 submodule instance.
 * @retval Returned value can be one of the following values:
 *         @arg @ref DDL_COMP1_FILTERCFG_1
 *         @arg @ref DDL_COMP1_FILTERCFG_2
 *         @arg @ref DDL_COMP1_FILTERCFG_4
 *         @arg @ref DDL_COMP1_FILTERCFG_8
 *         @arg @ref DDL_COMP1_FILTERCFG_16
 *         @arg @ref DDL_COMP1_FILTERCFG_32
 *         @arg @ref DDL_COMP1_FILTERCFG_64
 *         @arg @ref DDL_COMP1_FILTERCFG_128
 *         @arg @ref DDL_COMP1_FILTERCFG_256
 *         @arg @ref DDL_COMP1_FILTERCFG_512
 */
__STATIC_INLINE uint32_t DDL_COMP1_GetFilterCFG(COMP1_TypeDef *COMP1x)
{
  return (READ_BIT(COMP1x->CR, COMP_CR_CFG));
}


/**
 * @brief  Set comparator hysteresis.
 * @param  COMP1x COMP1 submodule instance.
 * @param  Hsy This parameter can be one of the following values:
 *         @arg @ref DDL_COMP1_HYSP_DISABLE
 *         @arg @ref DDL_COMP1_HYSP_20MV
 *         @arg @ref DDL_COMP1_HYSP_40MV
 *         @arg @ref DDL_COMP1_HYSP_60MV
 * @retval None
 */
__STATIC_INLINE void DDL_COMP1_SetPositiveHysteresis(COMP1_TypeDef *COMP1x, uint32_t HsyP)
{
  MODIFY_REG(COMP1x->CR, COMP_CR_HYSPEN, HsyP);
}

/**
 * @brief  Get comparator hysteresis.
 * @param  COMP1x COMP1 submodule instance.
 * @retval Returned value can be one of the following values:
 *         @arg @ref DDL_COMP1_HYSP_DISABLE
 *         @arg @ref DDL_COMP1_HYSP_20MV
 *         @arg @ref DDL_COMP1_HYSP_40MV
 *         @arg @ref DDL_COMP1_HYSP_60MV
 */
__STATIC_INLINE uint32_t DDL_COMP1_GetPositiveHysteresis(COMP1_TypeDef *COMP1x)
{
  return (READ_BIT(COMP1x->CR, COMP_CR_HYSPEN));
}

/**
 * @brief  Set comparator hysteresis.
 * @param  COMP1x COMP1 submodule instance.
 * @param  Hsy This parameter can be one of the following values:
 *         @arg @ref DDL_COMP1_HYSN_DISABLE
 *         @arg @ref DDL_COMP1_HYSN_20MV
 *         @arg @ref DDL_COMP1_HYSN_40MV
 *         @arg @ref DDL_COMP1_HYSN_60MV
 * @retval None
 */
__STATIC_INLINE void DDL_COMP1_SetNegativeHysteresis(COMP1_TypeDef *COMP1x, uint32_t HsyN)
{
  MODIFY_REG(COMP1x->CR, COMP_CR_HYSNEN, HsyN);
}

/**
 * @brief  Get comparator hysteresis.
 * @param  COMP1x COMP1 submodule instance.
 * @retval Returned value can be one of the following values:
 *         @arg @ref DDL_COMP1_HYSN_DISABLE
 *         @arg @ref DDL_COMP1_HYSN_20MV
 *         @arg @ref DDL_COMP1_HYSN_40MV
 *         @arg @ref DDL_COMP1_HYSN_60MV
 */
__STATIC_INLINE uint32_t DDL_COMP1_GetNegativeHysteresis(COMP1_TypeDef *COMP1x)
{
  return (READ_BIT(COMP1x->CR, COMP_CR_HYSNEN));
}

/**
  * @}
  */

/** @defgroup COMP1_DDL_EF_Configuration_comparator_inputs Configuration of comparator inputs
 * @{
 */

/**
 * @brief  Set comparator input plus.
 * @param  COMP1x COMP1 submodule instance.
 * @param  InputPlus This parameter can be one of the following values:
 *         @arg @ref DDL_COMP1_INPUT_PLUS_PB6
 *         @arg @ref DDL_COMP1_INPUT_PLUS_VIRTUAL
 * @retval None
 */
__STATIC_INLINE void DDL_COMP1_SetInputPlus(COMP1_TypeDef *COMP1x, uint32_t InputPlus)
{
  MODIFY_REG(COMP1x->CR, COMP_CR_VPSEL, InputPlus);
}

/**
 * @brief Get comparator input plus.
 * @param  COMP1x COMP1 submodule instance.
 * @retval Returned value can be one of the following values:
 *         @arg @ref DDL_COMP1_INPUT_PLUS_PB6
 *         @arg @ref DDL_COMP1_INPUT_PLUS_VIRTUAL
 */
__STATIC_INLINE uint32_t DDL_COMP1_GetInputPlus(COMP1_TypeDef *COMP1x)
{
  return (READ_BIT(COMP1x->CR, COMP_CR_VPSEL));
}

/**
 * @brief  Set comparator input minus.
 * @param  COMP1x COMP1 submodule instance.
 * @param  InputSel This parameter can be one of the following values:
 * COMP1:
 *         @arg @ref DDL_COMP1_INPUT_MINUS_PA1
 *         @arg @ref DDL_COMP1_INPUT_MINUS_PB1
 * COMP2:
 *         @arg @ref DDL_COMP1_INPUT_MINUS_PA2
 *         @arg @ref DDL_COMP1_INPUT_MINUS_PB2
 * COMP3:
 *         @arg @ref DDL_COMP1_INPUT_MINUS_PA0
 *         @arg @ref DDL_COMP1_INPUT_MINUS_PB5
 * @retval None
 */
__STATIC_INLINE void DDL_COMP1_SetInputMinus(COMP1_TypeDef *COMP1x, uint32_t InputSel)
{
  MODIFY_REG(COMP1x->CR, COMP_CR_VNSEL, InputSel);
}

/**
 * @brief Get comparator input minus.
 * @param  COMP1x COMP1 submodule instance.
 * @retval Returned value can be one of the following values:
 * COMP1:
 *         @arg @ref DDL_COMP1_INPUT_MINUS_PA1
 *         @arg @ref DDL_COMP1_INPUT_MINUS_PB1
 * COMP2:
 *         @arg @ref DDL_COMP1_INPUT_MINUS_PA2
 *         @arg @ref DDL_COMP1_INPUT_MINUS_PB2
 * COMP3:
 *         @arg @ref DDL_COMP1_INPUT_MINUS_PA0
 *         @arg @ref DDL_COMP1_INPUT_MINUS_PB5
 */
__STATIC_INLINE uint32_t DDL_COMP1_GetInputMinus(COMP1_TypeDef *COMP1x)
{
  return (READ_BIT(COMP1x->CR, COMP_CR_VNSEL));
}

/**
  * @}
  */

/** @defgroup COMP1_DDL_EF_Configuration_comparator_output Configuration of comparator output
 * @{
 */

/**
 * @brief  Set comparator output polarity.
 * @param  COMP1x COMP1 submodule instance.
 * @param  OutputPol This parameter can be one of the following values:
 *        @arg @ref DDL_COMP1_OUTPUTPOL_NONINVERTED
 *        @arg @ref DDL_COMP1_OUTPUTPOL_INVERTED
 * @retval None
 */
__STATIC_INLINE void DDL_COMP1_SetOutputPolarity(COMP1_TypeDef *COMP1x, uint32_t OutputPol)
{
  MODIFY_REG(COMP1x->CR, COMP_CR_POL, OutputPol);
}

/**
 * @brief Get comparator output polarity.
 * @param  COMP1x COMP1 submodule instance.
 * @retval Returned value can be one of the following values:
 *       @arg @ref DDL_COMP1_OUTPUTPOL_NONINVERTED
 *       @arg @ref DDL_COMP1_OUTPUTPOL_INVERTED
 */
__STATIC_INLINE uint32_t DDL_COMP1_GetOutputPolarity(COMP1_TypeDef *COMP1x)
{
  return (READ_BIT(COMP1x->CR, COMP_CR_POL));
}

/**
  * @}
  */

 /** @defgroup COMP1_DDL_EF_Operation Operation on comparator instance
  * @{
  */

/**
 * @brief  Enable comparator instance.
 * @param  COMP1x COMP1 submodule instance.
 * @retval None
 * @note   After enable from off state, comparator startup time is needed.
 */
__STATIC_INLINE void DDL_COMP1_Enable(COMP1_TypeDef *COMP1x)
{
  SET_BIT(COMP1x->CR, COMP_CR_COMPEN);
}

/**
 * @brief  Disable comparator instance.
 * @param  COMP1x COMP1 submodule instance.
 * @retval None
 */
__STATIC_INLINE void DDL_COMP1_Disable(COMP1_TypeDef *COMP1x)
{
  CLEAR_BIT(COMP1x->CR, COMP_CR_COMPEN);
}

/**
 * @brief  Get comparator enable state.
 * @param  COMP1x COMP1 submodule instance.
 * @retval State of bit (1 or 0).
 *        - 0: comparator instance is disabled.
 *        - 1: comparator instance is enabled.
 */
__STATIC_INLINE uint32_t DDL_COMP1_IsEnabled(COMP1_TypeDef *COMP1x)
{
  return ((READ_BIT(COMP1x->CR, COMP_CR_COMPEN) == (COMP_CR_COMPEN)) ? 1UL : 0UL);
}

/**
 * @brief  Lock comparator instance.
 * @param  COMP1x COMP1 submodule instance.
 * @retval None
 * @note   Once locked, comparator configuration can no longer be modified until next reset.
 * @note   The only way to unlock the comparator is a device hardware reset.
 */
__STATIC_INLINE void DDL_COMP1_Lock(COMP1_TypeDef *COMP1x)
{
  SET_BIT(COMP1x->CR, COMP_CR_LOCK);
}

/**
 * @brief  Get comparator lock state.
 * @param  COMP1x COMP1 submodule instance.
 * @retval State of bit (1 or 0).
 *       - 0: comparator instance is unlocked.
 *       - 1: comparator instance is locked.
 */
__STATIC_INLINE uint32_t DDL_COMP1_IsLocked(COMP1_TypeDef *COMP1x)
{
  return ((READ_BIT(COMP1x->CR, COMP_CR_LOCK) == (COMP_CR_LOCK)) ? 1UL : 0UL);
}

/**
 * @brief  Read comparator output level.
 * @param  COMP1x COMP1 submodule instance.
 * @retval Returned value can be one of the following values:
 *       @arg @ref DDL_COMP1_OUTPUT_LEVEL_LOW
 *       @arg @ref DDL_COMP1_OUTPUT_LEVEL_HIGH
 * @note   The comparator output level depends on the selected polarity.
 *         If the polarity is not inverted:
 *         - Comparator output is low level when the input plus is lower than the input minus.
 *         - Comparator output is high level when the input plus is higher than the input minus.
 *         If the polarity is inverted:
 *         - Comparator output is high level when the input plus is lower than the input minus.
 *         - Comparator output is low level when the input plus is higher than the input minus.
 */
__STATIC_INLINE uint32_t DDL_COMP1_ReadOutputLevel(COMP1_TypeDef *COMP1x)
{
  return (READ_BIT(COMP1x->CR, COMP_CR_VAL));
}

/**
  * @}
  */

/** @defgroup COMP1_DDL_EF_Interrupt Interrupt
 * @{
 */

/**
 * @brief  Enable comparator SW Interrupt.
 * @param  COMP1x COMP1 submodule instance.
 * @retval None
 */
__STATIC_INLINE void DDL_COMP1_EnableSWINT(COMP1_TypeDef *COMP1x)
{
  SET_BIT(COMP1x->CR, COMP_CR_SWEG);
}

/**
 * @brief  Disable comparator SW Interrupt.
 * @param  COMP1x COMP1 submodule instance.
 * @retval None
 */
__STATIC_INLINE void DDL_COMP1_DisableSWINT(COMP1_TypeDef *COMP1x)
{
  CLEAR_BIT(COMP1x->CR, COMP_CR_SWEG);
}

/**
  * @brief  Set the COMP1 interrupt.
  * @param  COMP1x COMP1 submodule instance.
  * @param  InputSel This parameter can be one of the following values:
  *         @arg @ref DDL_COMP1_EDGE_INT_DISABLE
  *         @arg @ref DDL_COMP1_EDGE_INT_RISING
  *         @arg @ref DDL_COMP1_EDGE_INT_FALLING
  *         @arg @ref DDL_COMP1_EDGE_INT_RISING_FALLING
  * @retval None
  */
__STATIC_INLINE void DDL_COMP1_SetInterrupt(COMP1_TypeDef *COMP1x, uint32_t Interrupt)
{
  MODIFY_REG(COMP1x->CR, (COMP_CR_REN | COMP_CR_FEN | COMP_CR_RFEN), Interrupt);
}

/**
  * @brief  Get the COMP1 interrupt.
  * @param  COMP1x COMP1 submodule instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_COMP1_EDGE_INT_DISABLE
  *         @arg @ref DDL_COMP1_EDGE_INT_RISING
  *         @arg @ref DDL_COMP1_EDGE_INT_FALLING
  *         @arg @ref DDL_COMP1_EDGE_INT_RISING_FALLING
  */
__STATIC_INLINE uint32_t DDL_COMP1_GetInterrupt(COMP1_TypeDef *COMP1x)
{
  return (READ_BIT(COMP1x->CR, (COMP_CR_REN | COMP_CR_FEN | COMP_CR_RFEN)));
}

/**
  * @}
  */

/** @defgroup COMP1_DDL_EF_FLAG_Management FLAG-Management
 * @{
 */

/**
  * @brief  Clear the interrupt flag.
  * @param  COMP1x COMP1 submodule instance.
  * @retval None
  */
__STATIC_INLINE void DDL_COMP1_ClearFlag_IT(COMP1_TypeDef *COMP1x)
{
  SET_BIT(COMP1x->ISR, COMP_ISR_IFLG);
  }

/**
  * @brief  Indicate whether interrupt flag is set.
  * @param  COMP1x COMP1 submodule instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_COMP1_IsActiveFlag_IT(COMP1_TypeDef *COMP1x)
{
  return ((READ_BIT(COMP1x->ISR, COMP_ISR_IFLG) == (COMP_ISR_IFLG)) ? 1UL : 0UL);
  }

/**
  * @}
  */

#if defined(USE_FULL_DDL_DRIVER)
/** @defgroup COMP1_DDL_EF_Init Initialization and de-initialization functions
 * @{
 */

ErrorStatus DDL_COMP1_Init(COMP1_TypeDef *COMP1x, DDL_COMP1_InitTypeDef *COMP1_InitStruct);
ErrorStatus DDL_COMP1_DeInit(COMP1_TypeDef *COMP1x);
void        DDL_COMP1_StructInit(DDL_COMP1_InitTypeDef *COMP1_InitStruct);

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

 #endif /* COMP1 || COMP2 || COMP3 */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* G32F037_DDL_COMP1_H */
