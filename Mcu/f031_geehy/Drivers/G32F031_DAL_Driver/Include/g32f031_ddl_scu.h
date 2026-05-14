/**
  *
  * @file    g32f031_ddl_scu.h
  * @brief   Header file of SCU DDL module.
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
#ifndef G32F031_DDL_SCU_H
#define G32F031_DDL_SCU_H

#ifdef __cplusplus
  extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "g32f0xx.h"

/** @addtogroup G32F031_DDL_Driver
  * @{
  */

#if defined(SCU)

/** @defgroup SCU_DDL SCU
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/** @defgroup SCU_DDL_Exported_Constants SCU Exported Constants
  * @{
  */
#define DDL_SCU_DEBUG_ATMR_CONTINUE                (0x0UL)
#define DDL_SCU_DEBUG_ATMR_PAUSE                   (SCU_DBG_ATMRDBG)
#define DDL_SCU_DEBUG_GTMR_CONTINUE                (0x0UL)
#define DDL_SCU_DEBUG_GTMR_PAUSE                   (SCU_DBG_GTMRDBG)
#define DDL_SCU_DEBUG_BTMR0_CONTINUE               (0x0UL)
#define DDL_SCU_DEBUG_BTMR0_PAUSE                  (SCU_DBG_BTMR0DBG)
#define DDL_SCU_DEBUG_BTMR1_CONTINUE               (0x0UL)
#define DDL_SCU_DEBUG_BTMR1_PAUSE                  (SCU_DBG_BTMR1DBG)
#define DDL_SCU_DEBUG_WWDT_CONTINUE                (0x0UL)
#define DDL_SCU_DEBUG_WWDT_PAUSE                   (SCU_DBG_WWDTDBG)
#define DDL_SCU_DEBUG_IWDT_CONTINUE                (0x0UL)
#define DDL_SCU_DEBUG_IWDT_PAUSE                   (SCU_DBG_IWDTDBG)
#define DDL_SCU_DEBUG_LPTMR_CONTINUE               (0x0UL)
#define DDL_SCU_DEBUG_LPTMR_PAUSE                  (SCU_DBG_LPTMRDBG)

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup SCU_DDL_Exported_Functions SCU Exported Functions
  * @{
  */

/**
  * @brief  Lock the SCU register.
  * @param  None
  * @retval None
  */
__STATIC_INLINE void DDL_SCU_Lock(void)
{
  SET_BIT(SCU->KEY, SCU_KEY_LOCKKEY);
}

/**
  * @brief  Unlock the SCU register.
  * @param  None
  * @retval None
  */
__STATIC_INLINE void DDL_SCU_Unlock(void)
{
  WRITE_REG(SCU->KEY, (0xFFFFU & SCU_KEY_VALUE));
}

/**
  * @brief  Check if the SCU KEY status is locked.
  * @param  None
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_SCU_IsActiveFlag_LOCK(void)
{
  return (READ_BIT(SCU->KEY, SCU_KEY_LOCKFLG) != (SCU_KEY_LOCKFLG));
}

/**
  * @brief  In DEBUG mode, set the ATMR counting method.
  * @param  State  the ATMR counting method in debug mode.
  *         @arg @ref DDL_SCU_DEBUG_ATMR_CONTINUE
  *         @arg @ref DDL_SCU_DEBUG_ATMR_PAUSE
  * @retval None
  */
__STATIC_INLINE void DDL_SCU_SetATMRCounter(uint32_t state)
{
  MODIFY_REG(SCU->DBG, SCU_DBG_ATMRDBG, state);
}

/**
  * @brief  In DEBUG mode, set the ATMR counting method.
  * @param  None
  * @retval State  the ATMR counting method in debug mode.
  *         @arg @ref DDL_SCU_DEBUG_ATMR_CONTINUE
  *         @arg @ref DDL_SCU_DEBUG_ATMR_PAUSE
  */
__STATIC_INLINE uint32_t DDL_SCU_GetATMRCounter(void)
{
  return (uint32_t)READ_BIT(SCU->DBG, SCU_DBG_ATMRDBG);
}

/**
  * @brief  In DEBUG mode, set the GTMR counting method.
  * @param  State  the GTMR counting method in debug mode.
  *         @arg @ref DDL_SCU_DEBUG_GTMR_CONTINUE
  *         @arg @ref DDL_SCU_DEBUG_GTMR_PAUSE
  * @retval None
  */
__STATIC_INLINE void DDL_SCU_SetGTMRCounter(uint32_t state)
{
  MODIFY_REG(SCU->DBG, SCU_DBG_GTMRDBG, state);
}

/**
  * @brief  In DEBUG mode, set the GTMR counting method.
  * @param  None
  * @retval State  the GTMR counting method in debug mode.
  *         @arg @ref DDL_SCU_DEBUG_GTMR_CONTINUE
  *         @arg @ref DDL_SCU_DEBUG_GTMR_PAUSE
  */
__STATIC_INLINE uint32_t DDL_SCU_GetGTMRCounter(void)
{
  return (uint32_t)READ_BIT(SCU->DBG, SCU_DBG_GTMRDBG);
}

/**
  * @brief  In DEBUG mode, set the BTMR0 counting method.
  * @param  State  the BTMR0 counting method in debug mode.
  *         @arg @ref DDL_SCU_DEBUG_BTMR0_CONTINUE
  *         @arg @ref DDL_SCU_DEBUG_BTMR0_PAUSE
  * @retval None
  */
__STATIC_INLINE void DDL_SCU_SetBTMR0Counter(uint32_t state)
{
  MODIFY_REG(SCU->DBG, SCU_DBG_BTMR0DBG, state);
}

/**
  * @brief  In DEBUG mode, set the BTMR0 counting method.
  * @param  None
  * @retval State  the BTMR0 counting method in debug mode.
  *         @arg @ref DDL_SCU_DEBUG_BTMR0_CONTINUE
  *         @arg @ref DDL_SCU_DEBUG_BTMR0_PAUSE
  */
__STATIC_INLINE uint32_t DDL_SCU_GetBTMR0Counter(void)
{
  return (uint32_t)READ_BIT(SCU->DBG, SCU_DBG_BTMR0DBG);
}

/**
  * @brief  In DEBUG mode, set the BTMR1 counting method.
  * @param  State  the BTMR1 counting method in debug mode.
  *         @arg @ref DDL_SCU_DEBUG_BTMR1_CONTINUE
  *         @arg @ref DDL_SCU_DEBUG_BTMR1_PAUSE
  * @retval None
  */
__STATIC_INLINE void DDL_SCU_SetBTMR1Counter(uint32_t state)
{
  MODIFY_REG(SCU->DBG, SCU_DBG_BTMR1DBG, state);
}

/**
  * @brief  In DEBUG mode, set the BTMR1 counting method.
  * @param  None
  * @retval State  the BTMR1 counting method in debug mode.
  *         @arg @ref DDL_SCU_DEBUG_BTMR1_CONTINUE
  *         @arg @ref DDL_SCU_DEBUG_BTMR1_PAUSE
  */
__STATIC_INLINE uint32_t DDL_SCU_GetBTMR1Counter(void)
{
  return (uint32_t)READ_BIT(SCU->DBG, SCU_DBG_BTMR1DBG);
}

/**
  * @brief  In DEBUG mode, set the WWDT counting method.
  * @param  State  the WWDT counting method in debug mode.
  *         @arg @ref DDL_SCU_DEBUG_WWDT_CONTINUE
  *         @arg @ref DDL_SCU_DEBUG_WWDT_PAUSE
  * @retval None
  */
__STATIC_INLINE void DDL_SCU_SetWWDTCounter(uint32_t state)
{
  MODIFY_REG(SCU->DBG, SCU_DBG_WWDTDBG, state);
}

/**
  * @brief  In DEBUG mode, set the WWDT counting method.
  * @param  None
  * @retval State  the WWDT counting method in debug mode.
  *         @arg @ref DDL_SCU_DEBUG_WWDT_CONTINUE
  *         @arg @ref DDL_SCU_DEBUG_WWDT_PAUSE
  */
__STATIC_INLINE uint32_t DDL_SCU_GetWWDTCounter(void)
{
  return (uint32_t)READ_BIT(SCU->DBG, SCU_DBG_WWDTDBG);
}

/**
  * @brief  In DEBUG mode, set the IWDT counting method.
  * @param  State  the IWDT counting method in debug mode.
  *         @arg @ref DDL_SCU_DEBUG_IWDT_CONTINUE
  *         @arg @ref DDL_SCU_DEBUG_IWDT_PAUSE
  * @retval None
  */
__STATIC_INLINE void DDL_SCU_SetIWDTCounter(uint32_t state)
{
  MODIFY_REG(SCU->DBG, SCU_DBG_IWDTDBG, state);
}

/**
  * @brief  In DEBUG mode, set the IWDT counting method.
  * @param  None
  * @retval State  the IWDT counting method in debug mode.
  *         @arg @ref DDL_SCU_DEBUG_IWDT_CONTINUE
  *         @arg @ref DDL_SCU_DEBUG_IWDT_PAUSE
  */
__STATIC_INLINE uint32_t DDL_SCU_GetIWDTCounter(void)
{
  return (uint32_t)READ_BIT(SCU->DBG, SCU_DBG_IWDTDBG);
}

/**
  * @brief  In DEBUG mode, set the LPTMR counting method.
  * @param  State  the LPTMR counting method in debug mode.
  *         @arg @ref DDL_SCU_DEBUG_LPTMR_CONTINUE
  *         @arg @ref DDL_SCU_DEBUG_LPTMR_PAUSE
  * @retval None
  */
__STATIC_INLINE void DDL_SCU_SetLPTMRCounter(uint32_t state)
{
  MODIFY_REG(SCU->DBG, SCU_DBG_LPTMRDBG, state);
}

/**
  * @brief  In DEBUG mode, set the LPTMR counting method.
  * @param  None
  * @retval State  the LPTMR counting method in debug mode.
  *         @arg @ref DDL_SCU_DEBUG_LPTMR_CONTINUE
  *         @arg @ref DDL_SCU_DEBUG_LPTMR_PAUSE
  */
__STATIC_INLINE uint32_t DDL_SCU_GetLPTMRCounter(void)
{
  return (uint32_t)READ_BIT(SCU->DBG, SCU_DBG_LPTMRDBG);
}

/**
  * @}
  */

/**
  * @}
  */

#endif /* defined(SCU) */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* G32F031_DDL_SCU_H */
