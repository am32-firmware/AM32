/**
  *
  * @file    g32f031_ddl_crc.h
  * @brief   Header file of CRC DDL module.
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
 *  Copyright (C) 2026 Geehy Semiconductor
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file in
  * the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef G32F031_DDL_CRC_H
#define G32F031_DDL_CRC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "g32f0xx.h"

/** @addtogroup G32F031_DDL_Driver
  * @{
  */
#if defined(CRC)

/** @defgroup CRC_DDL CRC
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/** @defgroup CRC_DDL_Exported_Macros CRC Exported Macros
  * @{
  */

/** @defgroup CRC_DDL_EF_Selection CRC calculation mode
 * @{
 */
#define DDL_CRC_SEL_CALCULATION_32             (0x00000000UL)                       /*!< CRC calculation mode 32bit. */
#define DDL_CRC_SEL_CALCULATION_16             (CRC_CR_CRCSEL)                     /*!< CRC calculation mode 16bit. */
/**
 * @}
 */

/** @defgroup CRC_DDL_EF_Selection CRC calculation Input Flip mode
 * @{
 */
#define DDL_CRC_INDATA_REVERSE_NONE             (0x00000000UL)                          /*!< CRC calculation Input Flip mode with none. */
#define DDL_CRC_INDATA_REVERSE_BYTE             (CRC_CR_INFLIP_0)                       /*!< CRC calculation Input Flip mode with BYTE. */
#define DDL_CRC_INDATA_REVERSE_HALFWORD         (CRC_CR_INFLIP_1)                       /*!< CRC calculation Input Flip mode with HALF WORD. */
#define DDL_CRC_INDATA_REVERSE_WORD             (CRC_CR_INFLIP)                         /*!< CRC calculation Input Flip mode with WORD. */
/**
 * @}
 */

/** @defgroup CRC_DDL_EF_Selection CRC calculation Output Flip mode
 * @{
 */
#define DDL_CRC_OUTDATA_REVERSE_DISABLE             (0x00000000UL)                          /*!< Disable CRC calculation Output Flip mode. */
#define DDL_CRC_OUTDATA_REVERSE_ENABLE              (CRC_CR_OUTFLIP)                        /*!< Enable CRC calculation Output Flip mode. */
/**
 * @}
 */

/** @defgroup CRC_DDL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in CRC register
  * @param  __INSTANCE__ CRC Instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define DDL_CRC_WriteReg(__INSTANCE__, __REG__, __VALUE__) WRITE_REG(__INSTANCE__->__REG__, __VALUE__)

/**
  * @brief  Read a value in CRC register
  * @param  __INSTANCE__ CRC Instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define DDL_CRC_ReadReg(__INSTANCE__, __REG__) READ_REG(__INSTANCE__->__REG__)
/**
  * @}
  */

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup CRC_DDL_Exported_Functions CRC Exported Functions
  * @{
  */

/** @defgroup CRC_DDL_EF_OutputToggle CRC calculation Output Flip
  * @{
  */

/**
  * @brief  Set CRC calculation output flip mode.
  * @param  CRCx CRC Instance.
  * @param  outMode This parameter can be one of the following values:
  *         @arg @ref DDL_CRC_OUTDATA_REVERSE_DISABLE
  *         @arg @ref DDL_CRC_OUTDATA_REVERSE_ENABLE
  */
__STATIC_INLINE void DDL_CRC_SetOutputDataReverseMode(CRC_TypeDef *CRCx, uint32_t outMode)
{
  MODIFY_REG(CRCx->CR, CRC_CR_OUTFLIP, outMode);
}

/**
  * @brief  Get CRC calculation output flip mode.
  * @param  CRCx CRC Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_CRC_OUTDATA_REVERSE_DISABLE
  *         @arg @ref DDL_CRC_OUTDATA_REVERSE_ENABLE
  */
__STATIC_INLINE uint32_t DDL_CRC_GetOutputDataReverseMode(CRC_TypeDef *CRCx)
{
  return (uint32_t)(READ_BIT(CRCx->CR, CRC_CR_OUTFLIP));
}

/**
  * @brief  Set CRC calculation input flip mode.
  * @param  CRCx CRC Instance.
  * @param  inMode This parameter can be one of the following values:
  *         @arg @ref DDL_CRC_INDATA_REVERSE_NONE
  *         @arg @ref DDL_CRC_INDATA_REVERSE_BYTE
  *         @arg @ref DDL_CRC_INDATA_REVERSE_HALFWORD
  *         @arg @ref DDL_CRC_INDATA_REVERSE_WORD
  */
__STATIC_INLINE void DDL_CRC_SetInputDataReverseMode(CRC_TypeDef *CRCx, uint32_t inMode)
{
  MODIFY_REG(CRCx->CR, CRC_CR_INFLIP, inMode);
}

/**
  * @brief  Get CRC calculation input flip mode.
  * @param  CRCx CRC Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_CRC_INDATA_REVERSE_NONE
  *         @arg @ref DDL_CRC_INDATA_REVERSE_BYTE
  *         @arg @ref DDL_CRC_INDATA_REVERSE_HALFWORD
  *         @arg @ref DDL_CRC_INDATA_REVERSE_WORD
  */
__STATIC_INLINE uint32_t DDL_CRC_GetInputDataReverseMode(CRC_TypeDef *CRCx)
{
  return (uint32_t)(READ_BIT(CRCx->CR, CRC_CR_INFLIP));
}

/** @defgroup CRC_DDL_EF_ResetValue Reset_Value
  * @{
  */

/**
  * @brief  Set CRC unit reset value
  * @param  CRCx CRC Instance
  * @param  RstData value to be provided for unit reset value between Min_Data=0 and Max_Data=0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void DDL_CRC_SetInitialData(CRC_TypeDef *CRCx, uint32_t RstData)
{
  WRITE_REG(CRCx->INIT, RstData);
}

/**
  * @brief  Get CRC unit reset value
  * @param  CRCx CRC Instance
  * @retval CRC unit reset value as stored in CRC_RSTDATA register (32 bits).
  */
__STATIC_INLINE uint32_t DDL_CRC_GetInitialData(CRC_TypeDef *CRCx)
{
  return (uint32_t)(READ_REG(CRCx->INIT));
}

/** @defgroup CRC_DDL_EF_Selection CRC calculation mode
  * @{
  */

/**
  * @brief  Set CRC calculation mode.
  * @param  CRCx CRC Instance.
  * @param  Sel This parameter can be one of the following values:
  *         @arg @ref DDL_CRC_SEL_CALCULATION_32
  *         @arg @ref DDL_CRC_SEL_CALCULATION_16
  */
__STATIC_INLINE void DDL_CRC_SetCalculatorMode(CRC_TypeDef *CRCx, uint32_t Sel)
{
  MODIFY_REG(CRCx->CR, CRC_CR_CRCSEL, Sel);
}

/**
  * @brief  Get CRC calculation mode.
  * @param  CRCx CRC Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_CRC_SEL_CALCULATION_32
  *         @arg @ref DDL_CRC_SEL_CALCULATION_16
  */
__STATIC_INLINE uint32_t DDL_CRC_GetCalculatorMode(CRC_TypeDef *CRCx)
{
  return (uint32_t)(READ_BIT(CRCx->CR, CRC_CR_CRCSEL));
}

/** @defgroup CRC_DDL_EF_Data_Management Data_Management
  * @{
  */

/**
  * @brief  Write given 32-bit or 16-bit data to the CRC calculator
  * @param  CRCx CRC Instance
  * @param  InData value to be provided to CRC calculator between Min_Data=0 and Max_Data=0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void DDL_CRC_FeedData32(CRC_TypeDef *CRCx, uint32_t InData)
{
  WRITE_REG(CRCx->DR, InData);
}

/**
  * @brief  Write given 32-bit or 16-bit data to the CRC calculator
  * @param  CRCx CRC Instance
  * @param  InData value to be provided to CRC calculator between Min_Data=0 and Max_Data=0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void DDL_CRC_FeedData16(CRC_TypeDef *CRCx, uint16_t InData)
{
  __IO uint16_t *pReg;

  pReg = (__IO uint16_t *)(__IO void*)(&CRCx->DR);

  *pReg = InData;
}

/**
  * @brief  Return current CRC calculation result. 32 bits value is returned.
  * @param  CRCx CRC Instance
  * @retval Current CRC calculation result as stored in CRC_DATA register (32 bits or 16-bits).
  */
__STATIC_INLINE uint32_t DDL_CRC_ReadData32(CRC_TypeDef *CRCx)
{
  return (uint32_t)(READ_REG(CRCx->DR));
}

/**
  * @brief  Return current CRC calculation result.  16-bits value is returned.
  * @param  CRCx CRC Instance
  * @retval Current CRC calculation result as stored in CRC_DATA register (32 bits or 16-bits).
  */
__STATIC_INLINE uint32_t DDL_CRC_ReadData16(CRC_TypeDef *CRCx)
{
  return (uint16_t)(READ_REG(CRCx->DR));
}

/**
  * @}
  */

#if defined(USE_FULL_DDL_DRIVER)
/** @defgroup CRC_DDL_EF_Init Initialization and de-initialization functions
  * @{
  */

ErrorStatus DDL_CRC_DeInit(CRC_TypeDef *CRCx);

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

#endif /* defined(CRC) */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* G32F031_DDL_CRC_H */
