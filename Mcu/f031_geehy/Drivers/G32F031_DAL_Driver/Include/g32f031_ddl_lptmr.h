/**
  *
  * @file    g32f031_ddl_lptmr.h
  * @brief   Header file of LPTMR DDL module.
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
#ifndef G32F031_DDL_LPTMR_H
#define G32F031_DDL_LPTMR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "g32f0xx.h"

/** @addtogroup G32F031_DDL_Driver
  * @{
  */

#if defined (LPTMR)

/** @defgroup LPTMR_DDL LPTMR
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
#if defined(USE_FULL_DDL_DRIVER)
/** @defgroup LPTMR_DDL_ES_INIT LPTMR Exported Init structure
  * @{
  */

/**
  * @brief  LPTMR Time Base configuration structure definition.
  */
typedef struct
{
  uint8_t Prescaler;          /*!< Specifies the prescaler value used to divide the LPTMR clock.
                                   This parameter can be a number between Min_Data=0x0 and Max_Data=0xF.

                                   This feature can be modified afterwards using unitary function
                                   @ref DDL_LPTMR_SetPrescaler().*/

  uint16_t WakeUpValue;       /*!< Specifies the wake up value to generate wake up interrupt.
                                   This parameter must be a number between Min_Data=0x0000 and Max_Data=0xFFFF.

                                   This feature can be modified afterwards using unitary function
                                   @ref DDL_LPTMR_SetAutoReload().*/
} DDL_LPTMR_InitTypeDef;

/**
  * @}
  */
#endif /* USE_FULL_DDL_DRIVER */

/* Exported constants --------------------------------------------------------*/
/** @defgroup LPTMR_DDL_Exported_Constants LPTMR Exported Constants
  * @{
  */

/** @defgroup LPTMR_DDL_EC_GET_FLAG Get Flags Defines
  * @brief    Flags defines which can be used with DDL_LPTMR_ReadReg function.
  * @{
  */
#define DDL_LPTMR_STS_RUF                       LPTMR_SR_RVU           /*!< Reload update flag */
#define DDL_LPTMR_STS_PUF                       LPTMR_SR_PVU           /*!< Prescaler update flag */
#define DDL_LPTMR_STS_WKF                       LPTMR_SR_WKFLG       /*!< Wake up flag */
/**
  * @}
  */

/** @defgroup LPTMR_DDL_EC_IT IT Defines
  * @brief    IT defines which can be used with DDL_LPTMR_ReadReg and DDL_LPTMR_WriteReg functions.
  * @{
  */
#define DDL_LPTMR_INT_WKEN                      LPTMR_CR_LPTIEN       /*!< Update interrupt enable */
/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup LPTMR_DDL_Exported_Macros LPTMR Exported Macros
  * @{
  */

/** @defgroup LPTMR_DDL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */
/**
  * @brief  Write a value in LPTMR register.
  * @param  __INSTANCE__ LPTMR Instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define DDL_LPTMR_WriteReg(__INSTANCE__, __REG__, __VALUE__) WRITE_REG((__INSTANCE__)->__REG__, (__VALUE__))

/**
  * @brief  Read a value in LPTMR register.
  * @param  __INSTANCE__ LPTMR Instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define DDL_LPTMR_ReadReg(__INSTANCE__, __REG__) READ_REG((__INSTANCE__)->__REG__)
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup LPTMR_DDL_Exported_Functions LPTMR Exported Functions
  * @{
  */

/** @defgroup LPTMR_DDL_EF_Time_Base Time Base configuration
  * @{
  */

/**
  * @brief  Enable low power timer.
  * @param  LPTMRx LPTimer instance
  * @retval None
  */
__STATIC_INLINE void DDL_LPTMR_Enable(LPTMR_TypeDef *LPTMRx)
{
  SET_BIT(LPTMRx->CR, LPTMR_CR_CNTEN);
}

/**
  * @brief  Disable low power timer.
  * @param  LPTMRx LPTimer instance
  * @retval None
  */
__STATIC_INLINE void DDL_LPTMR_Disable(LPTMR_TypeDef *LPTMRx)
{
  CLEAR_BIT(LPTMRx->CR, LPTMR_CR_CNTEN);
}

/**
  * @brief  Set the prescaler value.
  * @note The prescaler can be changed on the fly as this control register is buffered.
  * @param  LPTMRx LPTimer instance
  * @param  Prescaler between Min_Data=0 and Max_Data=15
  * @retval None
  */
__STATIC_INLINE void DDL_LPTMR_SetPrescaler(LPTMR_TypeDef *LPTMRx, uint32_t Prescaler)
{
  MODIFY_REG(LPTMRx->PSC, LPTMR_PSC_PSC, Prescaler);
}

/**
  * @brief  Get the prescaler value.
  * @param  LPTMRx LPTimer instance
  * @retval  Prescaler value between Min_Data=0 and Max_Data=15
  */
__STATIC_INLINE uint32_t DDL_LPTMR_GetPrescaler(LPTMR_TypeDef *LPTMRx)
{
  return (uint32_t)(READ_REG(LPTMRx->PSC));
}

/**
  * @brief  Set the wake up value.
  * @param  LPTMRx LPTimer instance
  * @param  Value between Min_Data=0 and Max_Data=65535
  * @retval None
  */
__STATIC_INLINE void DDL_LPTMR_SetWakeUpValue(LPTMR_TypeDef *LPTMRx, uint32_t Value)
{
  MODIFY_REG(LPTMRx->WKVAL, LPTMR_WKVAL_WKVAL, Value);
}

/**
  * @brief  Get the wake up value.
  * @note Macro IS_LPTMR_32B_COUNTER_INSTANCE(LPTMRx) can be used to check
  *       whether or not a timer instance supports a 32 bits counter.
  * @param  LPTMRx LPTimer instance
  * @retval Auto-reload value
  */
__STATIC_INLINE uint32_t DDL_LPTMR_GetWakeUpValue(LPTMR_TypeDef *LPTMRx)
{
  return (uint32_t)(READ_REG(LPTMRx->WKVAL));
}

/**
  * @brief  Get the counter value.
  * @param  LPTMRx LPTimer instance
  * @retval Counter value between Min_Data=0 and Max_Data=0xFFFF
  */
__STATIC_INLINE uint32_t DDL_LPTMR_GetCounterValue(LPTMR_TypeDef *LPTMRx)
{
  return (uint32_t)(READ_REG(LPTMRx->CNT));
}

/**
  * @}
  */

/** @defgroup LPTMR_DDL_EF_FLAG_Management FLAG-Management
  * @{
  */

/**
  * @brief  Indicate whether reload update flag (RUF) is set.
  * @param  LPTMRx LPTimer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_LPTMR_IsActiveFlag_RU(LPTMR_TypeDef *LPTMRx)
{
  return ((READ_BIT(LPTMRx->SR, DDL_LPTMR_STS_RUF) == (DDL_LPTMR_STS_RUF)) ? 1UL : 0UL);
}

/**
  * @brief  Indicate whether prescaler update flag (PUF) is set.
  * @param  LPTMRx LPTimer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_LPTMR_IsActiveFlag_PU(LPTMR_TypeDef *LPTMRx)
{
  return ((READ_BIT(LPTMRx->SR, DDL_LPTMR_STS_PUF) == (DDL_LPTMR_STS_PUF)) ? 1UL : 0UL);
}

/**
  * @brief  Indicate whether wake up interrupt flag (WKF) is set (wake up interrupt is pending).
  * @param  LPTMRx LPTimer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_LPTMR_IsActiveFlag_WakeUp(LPTMR_TypeDef *LPTMRx)
{
  return ((READ_BIT(LPTMRx->SR, DDL_LPTMR_STS_WKF) == (DDL_LPTMR_STS_WKF)) ? 1UL : 0UL);
}

/**
  * @}
  */

/** @defgroup LPTMR_DDL_EF_IT_Management IT-Management
  * @{
  */

/**
  * @brief  Enable wake up interrupt (WK).
  * @param  LPTMRx LPTimer instance
  * @retval None
  */
__STATIC_INLINE void DDL_LPTMR_EnableIT_WakeUp(LPTMR_TypeDef *LPTMRx)
{
  SET_BIT(LPTMRx->CR, DDL_LPTMR_INT_WKEN);
}

/**
  * @brief  Disable wake up interrupt (WK).
  * @param  LPTMRx LPTimer instance
  * @retval None
  */
__STATIC_INLINE void DDL_LPTMR_DisableIT_WakeUp(LPTMR_TypeDef *LPTMRx)
{
  CLEAR_BIT(LPTMRx->CR, DDL_LPTMR_INT_WKEN);
}

/**
  * @}
  */

#if defined(USE_FULL_DDL_DRIVER)
/** @defgroup LPTMR_DDL_EF_Init Initialisation and deinitialisation functions
  * @{
  */
ErrorStatus DDL_LPTMR_DeInit(LPTMR_TypeDef *LPTMRx);
void DDL_LPTMR_StructInit(DDL_LPTMR_InitTypeDef *LPTMR_InitStruct);
ErrorStatus DDL_LPTMR_Init(LPTMR_TypeDef *LPTMRx, DDL_LPTMR_InitTypeDef *LPTMR_InitStruct);
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

#endif /* LPTMR */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* G32F031_DDL_LPTMR_H */
