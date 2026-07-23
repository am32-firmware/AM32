/**
  *
  * @file    g32f031_ddl_pmu.h
  * @brief   Header file of PMU DDL module.
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
#ifndef G32F031_DDL_PMU_H
#define G32F031_DDL_PMU_H

#ifdef __cplusplus
  extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "g32f0xx.h"

/** @addtogroup G32F031_DDL_Driver
  * @{
  */

#if defined(PMU)

/** @defgroup PMU_DDL PMU
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/** @defgroup PMU_DDL_Exported_Constants PMU Exported Constants
  * @{
  */
#define DDL_PMU_LPM_STOP                           (0x0UL)
#define DDL_PMU_LPM_STANDBY                        (PMU_LPCR_LPCFG)

#define DDL_PMU_STANDBY_MODE_LSI_OFF               (0x0UL)
#define DDL_PMU_STANDBY_MODE_LSI_ON                (PMU_LPCR_STANDBYCFG)

#define DDL_PMU_WAKEUP_PIN0_POLARITY_RISE_EDGE     (0x0UL)
#define DDL_PMU_WAKEUP_PIN0_POLARITY_FALL_EDGE     (PMU_WKCR_WKPOL0_0)
#define DDL_PMU_WAKEUP_PIN0_POLARITY_BOTH_EDGE     (PMU_WKCR_WKPOL0_1)

#define DDL_PMU_WAKEUP_PIN1_POLARITY_RISE_EDGE     (0x0UL)
#define DDL_PMU_WAKEUP_PIN1_POLARITY_FALL_EDGE     (PMU_WKCR_WKPOL1_0)
#define DDL_PMU_WAKEUP_PIN1_POLARITY_BOTH_EDGE     (PMU_WKCR_WKPOL1_1)

#define DDL_PMU_WAKEUP_PIN0_PULL_NONE              (0x0UL)
#define DDL_PMU_WAKEUP_PIN0_PULL_UP                (PMU_WKCR_WKPUS0_1)
#define DDL_PMU_WAKEUP_PIN0_PULL_DOWN              (PMU_WKCR_WKPUS0_0 | PMU_WKCR_WKPUS0_1)

#define DDL_PMU_WAKEUP_PIN1_PULL_NONE              (0x0UL)
#define DDL_PMU_WAKEUP_PIN1_PULL_UP                (PMU_WKCR_WKPUS1_1)
#define DDL_PMU_WAKEUP_PIN1_PULL_DOWN              (PMU_WKCR_WKPUS1_0 | PMU_WKCR_WKPUS1_1)

#define DDL_PMU_PVD_THRESHOLD_1                    (0x0UL)
#define DDL_PMU_PVD_THRESHOLD_2                    (PMU_PVDCSR_PVDTHSEL_0)
#define DDL_PMU_PVD_THRESHOLD_3                    (PMU_PVDCSR_PVDTHSEL_1)
#define DDL_PMU_PVD_THRESHOLD_4                    (PMU_PVDCSR_PVDTHSEL_1 | PMU_PVDCSR_PVDTHSEL_0)
#define DDL_PMU_PVD_THRESHOLD_5                    (PMU_PVDCSR_PVDTHSEL_2)
#define DDL_PMU_PVD_THRESHOLD_6                    (PMU_PVDCSR_PVDTHSEL_2 | PMU_PVDCSR_PVDTHSEL_0)
#define DDL_PMU_PVD_THRESHOLD_7                    (PMU_PVDCSR_PVDTHSEL_2 | PMU_PVDCSR_PVDTHSEL_1)
#define DDL_PMU_PVD_THRESHOLD_8                    (PMU_PVDCSR_PVDTHSEL_2 | PMU_PVDCSR_PVDTHSEL_1 | PMU_PVDCSR_PVDTHSEL_0)

#define DDL_PMU_PVD_FILTER_LENGTH_64               (0x0UL)
#define DDL_PMU_PVD_FILTER_LENGTH_128              (PMU_PVDCSR_PVDFLTSEL_0)
#define DDL_PMU_PVD_FILTER_LENGTH_192              (PMU_PVDCSR_PVDFLTSEL_1)
#define DDL_PMU_PVD_FILTER_LENGTH_320              (PMU_PVDCSR_PVDFLTSEL_1 | PMU_PVDCSR_PVDFLTSEL_0)
#define DDL_PMU_PVD_FILTER_LENGTH_640              (PMU_PVDCSR_PVDFLTSEL_2)
#define DDL_PMU_PVD_FILTER_LENGTH_1280             (PMU_PVDCSR_PVDFLTSEL_2 | PMU_PVDCSR_PVDFLTSEL_0)
#define DDL_PMU_PVD_FILTER_LENGTH_1920             (PMU_PVDCSR_PVDFLTSEL_2 | PMU_PVDCSR_PVDFLTSEL_1)
#define DDL_PMU_PVD_FILTER_LENGTH_3200             (PMU_PVDCSR_PVDFLTSEL_2 | PMU_PVDCSR_PVDFLTSEL_1 | PMU_PVDCSR_PVDFLTSEL_0)

#define DDL_PMU_VDD_BELOW_PVD_OUTPUT               (0x0UL)
#define DDL_PMU_VDD_ABOVE_PVD_OUTPUT               (PMU_PVDCSR_PVDSTS)

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup PMU_DDL_Exported_Functions PMU Exported Functions
  * @{
  */

/**
  * @brief  Lock the PMU register.
  * @param  None
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_Lock(void)
{
  SET_BIT(PMU->KEY, PMU_KEY_LOCKKEY);
}

/**
  * @brief  Unlock the PMU register.
  * @param  None
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_Unlock(void)
{
  WRITE_REG(PMU->KEY, (0xFFFFU & PMU_KEY_VALUE));
}

/**
  * @brief Check if the PMU KEY status is locked.
  * @retval None
  */
__STATIC_INLINE uint32_t DDL_PMU_IsActiveFlag_LOCK(void)
{
  return (READ_BIT(PMU->KEY, PMU_KEY_LOCKFLG) != (PMU_KEY_LOCKFLG));
}

/**
  * @brief  Set PMU low power mode
  * @param  mode PMU low power mode:
  *         @arg @ref DDL_PMU_LPM_STOP
  *         @arg @ref DDL_PMU_LPM_STANDBY
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_SetLPMMode(uint32_t mode)
{
  MODIFY_REG(PMU->LPCR, PMU_LPCR_LPCFG, mode);
}

/**
  * @brief  Get PMU low power mode flag
  * @param  None
  * @retval PMU low power mode:
  *         @arg @ref DDL_PMU_LPM_STOP
  *         @arg @ref DDL_PMU_LPM_STANDBY
  */
__STATIC_INLINE uint32_t DDL_PMU_GetLPMMode(void)
{
  return (uint32_t)READ_BIT(PMU->LPCR, PMU_LPCR_LPCFG);
}

/**
  * @brief  Set PMU standby mode
  * @param  mode PMU standby mode:
  *         @arg @ref DDL_PMU_STANDBY_MODE_LSI_OFF
  *         @arg @ref DDL_PMU_STANDBY_MODE_LSI_ON
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_SetStandbyMode(uint32_t mode)
{
  MODIFY_REG(PMU->LPCR, PMU_LPCR_STANDBYCFG, mode);
}

/**
  * @brief  Get PMU standby mode flag
  * @param  None
  * @retval PMU standby mode:
  *         @arg @ref DDL_PMU_STANDBY_MODE_LSI_OFF
  *         @arg @ref DDL_PMU_STANDBY_MODE_LSI_ON
  */
__STATIC_INLINE uint32_t DDL_PMU_GetStandbyMode(void)
{
  return (uint32_t)READ_BIT(PMU->LPCR, PMU_LPCR_STANDBYCFG);
}

/**
  * @brief  Wake-up pin 0 enables wake-up from STANDBY
  * @param  None
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_EnableWakeUpStandbyPin0(void)
{
  MODIFY_REG(PMU->WKCR, PMU_WKCR_WKEN0, PMU_WKCR_WKEN0);
}

/**
  * @brief  Wake-up pin 0 disables wake-up from STANDBY
  * @param  None
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_DisableWakeUpStandbyPin0(void)
{
  MODIFY_REG(PMU->WKCR, PMU_WKCR_WKEN0, 0);
}

/**
  * @brief  Wake-up pin 1 enables wake-up from STANDBY
  * @param  None
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_EnableWakeUpStandbyPin1(void)
{
  MODIFY_REG(PMU->WKCR, PMU_WKCR_WKEN1, PMU_WKCR_WKEN1);
}

/**
  * @brief  Wake-up pin 1 disables wake-up from STANDBY
  * @param  None
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_DisableWakeUpStandbyPin1(void)
{
  MODIFY_REG(PMU->WKCR, PMU_WKCR_WKEN1, 0);
}

/**
  * @brief  Set Wake-up polarity of pin 0 for STANDBY
  * @param  polarity Wake-up polarity
  *         @arg @ref DDL_PMU_WAKEUP_PIN0_POLARITY_RISE_EDGE
  *         @arg @ref DDL_PMU_WAKEUP_PIN0_POLARITY_FALL_EDGE
  *         @arg @ref DDL_PMU_WAKEUP_PIN0_POLARITY_BOTH_EDGE
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_SetWakeUpStandbyPin0Polarity(uint32_t polarity)
{
  MODIFY_REG(PMU->WKCR, PMU_WKCR_WKPOL0, polarity);
}

/**
  * @brief  Get Wake-up polarity of pin 0 for STANDBY
  * @param  None
  * @retval polarity Wake-up polarity
  *         @arg @ref DDL_PMU_WAKEUP_PIN0_POLARITY_RISE_EDGE
  *         @arg @ref DDL_PMU_WAKEUP_PIN0_POLARITY_FALL_EDGE
  *         @arg @ref DDL_PMU_WAKEUP_PIN0_POLARITY_BOTH_EDGE
  */
__STATIC_INLINE uint32_t DDL_PMU_GetWakeUpStandbyPin0Polarity(void)
{
  return (uint32_t)READ_BIT(PMU->WKCR, PMU_WKCR_WKPOL0);
}

/**
  * @brief  Set Wake-up polarity of pin 1 for STANDBY
  * @param  polarity Wake-up polarity
  *         @arg @ref DDL_PMU_WAKEUP_PIN1_POLARITY_RISE_EDGE
  *         @arg @ref DDL_PMU_WAKEUP_PIN1_POLARITY_FALL_EDGE
  *         @arg @ref DDL_PMU_WAKEUP_PIN1_POLARITY_BOTH_EDGE
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_SetWakeUpStandbyPin1Polarity(uint32_t polarity)
{
  MODIFY_REG(PMU->WKCR, PMU_WKCR_WKPOL1, polarity);
}

/**
  * @brief  Get Wake-up polarity of pin 1 for STANDBY
  * @param  None
  * @retval polarity Wake-up polarity
  *         @arg @ref DDL_PMU_WAKEUP_PIN1_POLARITY_RISE_EDGE
  *         @arg @ref DDL_PMU_WAKEUP_PIN1_POLARITY_FALL_EDGE
  *         @arg @ref DDL_PMU_WAKEUP_PIN1_POLARITY_BOTH_EDGE
  */
__STATIC_INLINE uint32_t DDL_PMU_GetWakeUpStandbyPin1Polarity(void)
{
  return (uint32_t)READ_BIT(PMU->WKCR, PMU_WKCR_WKPOL1);
}

/**
  * @brief  Set Wake-up pull of pin 0 for STANDBY
  * @param  polarity Wake-up pull
  *         @arg @ref DDL_PMU_WAKEUP_PIN0_PULL_NONE
  *         @arg @ref DDL_PMU_WAKEUP_PIN0_PULL_UP
  *         @arg @ref DDL_PMU_WAKEUP_PIN0_PULL_DOWN
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_SetWakeUpStandbyPin0Pull(uint32_t pull)
{
  MODIFY_REG(PMU->WKCR, PMU_WKCR_WKPUS0, pull);
}

/**
  * @brief  Get Wake-up pull of pin 0 for STANDBY
  * @param  None
  * @retval polarity Wake-up pull
  *         @arg @ref DDL_PMU_WAKEUP_PIN0_PULL_NONE
  *         @arg @ref DDL_PMU_WAKEUP_PIN0_PULL_UP
  *         @arg @ref DDL_PMU_WAKEUP_PIN0_PULL_DOWN
  */
__STATIC_INLINE uint32_t DDL_PMU_GetWakeUpStandbyPin0Pull(void)
{
  return (uint32_t)READ_BIT(PMU->WKCR, PMU_WKCR_WKPUS0);
}

/**
  * @brief  Set Wake-up pull of pin 1 for STANDBY
  * @param  polarity Wake-up pull
  *         @arg @ref DDL_PMU_WAKEUP_PIN1_PULL_NONE
  *         @arg @ref DDL_PMU_WAKEUP_PIN1_PULL_UP
  *         @arg @ref DDL_PMU_WAKEUP_PIN1_PULL_DOWN
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_SetWakeUpStandbyPin1Pull(uint32_t pull)
{
  MODIFY_REG(PMU->WKCR, PMU_WKCR_WKPUS1, pull);
}

/**
  * @brief  Get Wake-up pull of pin 1 for STANDBY
  * @param  None
  * @retval polarity Wake-up pull
  *         @arg @ref DDL_PMU_WAKEUP_PIN1_PULL_NONE
  *         @arg @ref DDL_PMU_WAKEUP_PIN1_PULL_UP
  *         @arg @ref DDL_PMU_WAKEUP_PIN1_PULL_DOWN
  */
__STATIC_INLINE uint32_t DDL_PMU_GetWakeUpStandbyPin1Pull(void)
{
  return (uint32_t)READ_BIT(PMU->WKCR, PMU_WKCR_WKPUS1);
}

/**
  * @brief  Enable IWDT wake-up in STANDBY mode
  * @param  None
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_EnableIWDTWakeUpStandby(void)
{
  SET_BIT(PMU->WKCR, PMU_WKCR_IWDTWKEN);
}

/**
  * @brief  Disable IWDT wake-up in STANDBY mode
  * @param  None
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_DisableIWDTWakeUpStandby(void)
{
  CLEAR_BIT(PMU->WKCR, PMU_WKCR_IWDTWKEN);
}

/**
  * @brief  Enable LPTMR wake-up in STANDBY mode
  * @param  None
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_EnableLPTMRWakeUpStandby(void)
{
  SET_BIT(PMU->WKCR, PMU_WKCR_LPTMRWKEN);
}

/**
  * @brief  Disable LPTMR wake-up in STANDBY mode
  * @param  None
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_DisableLPTMRWakeUpStandby(void)
{
  CLEAR_BIT(PMU->WKCR, PMU_WKCR_LPTMRWKEN);
}

/**
  * @brief  Get wake-up pin 0 to STANDBY wake-up flag
  * @param  None
  * @retval flag: wake-up pin 0 to STANDBY wake-up
  *         @arg @ref 0: No wake-up occurred on this pin
  *         @arg @ref 1: Wake-up occurred on this pin
  */
__STATIC_INLINE uint32_t DDL_PMU_IsActiveFlag_WKUPF0(void)
{
  return (uint32_t)(READ_BIT(PMU->WKSR, PMU_WKSR_WKFLG0) == PMU_WKSR_WKFLG0);
}

/**
  * @brief  Clear wake-up pin 0 to STANDBY wake-up flag
  * @param  None
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_ClearFlag_WKUPF0(void)
{
  CLEAR_BIT(PMU->WKSR, PMU_WKSR_WKFLG0);
}

/**
  * @brief  Get wake-up pin 1 to STANDBY wake-up flag
  * @param  None
  * @retval flag: wake-up pin 1 to STANDBY wake-up
  *         @arg @ref 0: No wake-up occurred on this pin
  *         @arg @ref 1: Wake-up occurred on this pin
  */
__STATIC_INLINE uint32_t DDL_PMU_IsActiveFlag_WKUPF1(void)
{
  return (uint32_t)(READ_BIT(PMU->WKSR, PMU_WKSR_WKFLG1) == PMU_WKSR_WKFLG1);
}

/**
  * @brief  Clear wake-up pin 1 to STANDBY wake-up flag
  * @param  None
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_ClearFlag_WKUPF1(void)
{
  CLEAR_BIT(PMU->WKSR, PMU_WKSR_WKFLG1);
}

/**
  * @brief  Get STANDBY event flag
  * @param  None
  * @retval flag: STANDBY event
  *         @arg @ref 0: No entry into STANDBY event occurred
  *         @arg @ref 1: An event of entering STANDBY has occurred.
  */
__STATIC_INLINE uint32_t DDL_PMU_IsActiveFlag_SBF(void)
{
  return (uint32_t)(READ_BIT(PMU->WKSR, PMU_WKSR_STANDBYFLG) == PMU_WKSR_STANDBYFLG);
}

/**
  * @brief  Clear STANDBY event flag
  * @param  None
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_ClearFlag_SBF(void)
{
  CLEAR_BIT(PMU->WKSR, PMU_WKSR_STANDBYFLG);
}

/**
  * @brief  Get the IWDT standby wake-up flag
  * @param  None
  * @retval flag: the IWDT standby wake-up
  *         @arg @ref 0: No wake-up from IWDT occurred
  *         @arg @ref 1: An IWDT wake-up occurred
  */
__STATIC_INLINE uint32_t DDL_PMU_IsActiveFlag_IWDT_WKUPF(void)
{
  return (uint32_t)(READ_BIT(PMU->WKSR, PMU_WKSR_IWDTWKFLG) == PMU_WKSR_IWDTWKFLG);
}

/**
  * @brief  Clear the IWDT standby wake-up flag
  * @param  None
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_ClearFlag_IWDT_WKUPF(void)
{
  CLEAR_BIT(PMU->WKSR, PMU_WKSR_IWDTWKFLG);
}

/**
  * @brief  Get the LPTIMER standby wake-up flag
  * @param  None
  * @retval flag: the LPTIMER standby wake-up
  *         @arg @ref 0: No wake-up from LPTIMER occurred
  *         @arg @ref 1: An LPTIMER wake-up occurred
  */
__STATIC_INLINE uint32_t DDL_PMU_IsActiveFlag_LPTMR_WKUPF(void)
{
  return (uint32_t)(READ_BIT(PMU->WKSR, PMU_WKSR_LPTMRWKFLG) == PMU_WKSR_LPTMRWKFLG);
}

/**
  * @brief  Clear the LPTMR standby wake-up flag
  * @param  None
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_ClearFlag_LPTMR_WKUPF(void)
{
  CLEAR_BIT(PMU->WKSR, PMU_WKSR_LPTMRWKFLG);
}

/**
  * @brief  Enable PMU PVD
  * @param  None
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_EnablePVD(void)
{
  SET_BIT(PMU->PVDCSR, PMU_PVDCSR_PVDEN);
}

/**
  * @brief  Disable PMU PVD
  * @param  None
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_DisablePVD(void)
{
  CLEAR_BIT(PMU->PVDCSR, PMU_PVDCSR_PVDEN);
}

/**
  * @brief  Get PMU PVD enable status
  * @param  None
  * @retval status:
  *         @arg @ref 0: Disable
  *         @arg @ref 1: Enable
  */
__STATIC_INLINE uint32_t DDL_PMU_IsEnabledPVD(void)
{
  return (uint32_t)(READ_BIT(PMU->PVDCSR, PMU_PVDCSR_PVDEN) == PMU_PVDCSR_PVDEN);
}

/**
  * @brief  Set PVD voltage threshold
  * @param  PVD voltage threshold
  *         @arg @ref DDL_PMU_PVD_THRESHOLD_1
  *         @arg @ref DDL_PMU_PVD_THRESHOLD_2
  *         @arg @ref DDL_PMU_PVD_THRESHOLD_3
  *         @arg @ref DDL_PMU_PVD_THRESHOLD_4
  *         @arg @ref DDL_PMU_PVD_THRESHOLD_5
  *         @arg @ref DDL_PMU_PVD_THRESHOLD_6
  *         @arg @ref DDL_PMU_PVD_THRESHOLD_7
  *         @arg @ref DDL_PMU_PVD_THRESHOLD_8
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_SetPVDVoltageThreshold(uint32_t threshold)
{
  MODIFY_REG(PMU->PVDCSR, PMU_PVDCSR_PVDTHSEL, threshold);
}

/**
  * @brief  Get PVD voltage threshold
  * @param  None
  * @retval PVD voltage threshold
  *         @arg @ref DDL_PMU_PVD_THRESHOLD_1
  *         @arg @ref DDL_PMU_PVD_THRESHOLD_2
  *         @arg @ref DDL_PMU_PVD_THRESHOLD_3
  *         @arg @ref DDL_PMU_PVD_THRESHOLD_4
  *         @arg @ref DDL_PMU_PVD_THRESHOLD_5
  *         @arg @ref DDL_PMU_PVD_THRESHOLD_6
  *         @arg @ref DDL_PMU_PVD_THRESHOLD_7
  *         @arg @ref DDL_PMU_PVD_THRESHOLD_8
  */
__STATIC_INLINE uint32_t DDL_PMU_GetPVDVoltageThreshold(void)
{
  return (uint32_t)READ_BIT(PMU->PVDCSR, PMU_PVDCSR_PVDTHSEL);
}

/**
  * @brief  Enable PVD below threshold voltage event monitoring
  * @param  None
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_EnablePVDBelowThresholdMonitoring(void)
{
  SET_BIT(PMU->PVDCSR, PMU_PVDCSR_PVDLT);
}

/**
  * @brief  Disable PVD below threshold voltage event monitoring
  * @param  None
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_DisablePVDBelowThresholdMonitoring(void)
{
  CLEAR_BIT(PMU->PVDCSR, PMU_PVDCSR_PVDLT);
}

/**
  * @brief  Enable PVD above threshold voltage event monitoring
  * @param  None
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_EnablePVDAboveThresholdMonitoring(void)
{
  SET_BIT(PMU->PVDCSR, PMU_PVDCSR_PVDHT);
}

/**
  * @brief  Disable PVD above threshold voltage event monitoring
  * @param  None
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_DisablePVDAboveThresholdMonitoring(void)
{
  CLEAR_BIT(PMU->PVDCSR, PMU_PVDCSR_PVDHT);
}

/**
  * @brief  Enable PVD monitoring event interrupt
  * @param  None
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_EnableIT_PVD_IE(void)
{
  SET_BIT(PMU->PVDCSR, PMU_PVDCSR_PVDIEN);
}

/**
  * @brief  Disable PVD monitoring event interrupt
  * @param  None
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_DisableIT_PVD_IE(void)
{
  CLEAR_BIT(PMU->PVDCSR, PMU_PVDCSR_PVDIEN);
}

/**
  * @brief  Get PVD monitoring event interrupt enable status
  * @param  None
  * @retval status:
  *         @arg @ref 0: Disable
  *         @arg @ref 1: Enable
  */
__STATIC_INLINE uint32_t DDL_PMU_IsEnabledIT_PVD_IE(void)
{
  return (uint32_t)(READ_BIT(PMU->PVDCSR, PMU_PVDCSR_PVDIEN) == PMU_PVDCSR_PVDIEN);
}

/**
  * @brief  Enable PVD filter
  * @param  None
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_EnableFilter(void)
{
  SET_BIT(PMU->PVDCSR, PMU_PVDCSR_PVDFLTEN);
}

/**
  * @brief  Disable PVD filter
  * @param  None
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_DisableFilter(void)
{
  CLEAR_BIT(PMU->PVDCSR, PMU_PVDCSR_PVDFLTEN);
}

/**
  * @brief  Set PVD filter length
  * @param  PVD filter length
  *         @arg @ref DDL_PMU_PVD_FILTER_LENGTH_64
  *         @arg @ref DDL_PMU_PVD_FILTER_LENGTH_128
  *         @arg @ref DDL_PMU_PVD_FILTER_LENGTH_192
  *         @arg @ref DDL_PMU_PVD_FILTER_LENGTH_320
  *         @arg @ref DDL_PMU_PVD_FILTER_LENGTH_640
  *         @arg @ref DDL_PMU_PVD_FILTER_LENGTH_1280
  *         @arg @ref DDL_PMU_PVD_FILTER_LENGTH_1920
  *         @arg @ref DDL_PMU_PVD_FILTER_LENGTH_3200
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_SetPVDFilterLength(uint32_t length)
{
  MODIFY_REG(PMU->PVDCSR, PMU_PVDCSR_PVDFLTSEL, length);
}

/**
  * @brief  Get PVD filter length
  * @param  None
  * @retval PVD filter length
  *         @arg @ref DDL_PMU_PVD_FILTER_LENGTH_64
  *         @arg @ref DDL_PMU_PVD_FILTER_LENGTH_128
  *         @arg @ref DDL_PMU_PVD_FILTER_LENGTH_192
  *         @arg @ref DDL_PMU_PVD_FILTER_LENGTH_320
  *         @arg @ref DDL_PMU_PVD_FILTER_LENGTH_640
  *         @arg @ref DDL_PMU_PVD_FILTER_LENGTH_1280
  *         @arg @ref DDL_PMU_PVD_FILTER_LENGTH_1920
  *         @arg @ref DDL_PMU_PVD_FILTER_LENGTH_3200
  */
__STATIC_INLINE uint32_t DDL_PMU_GetPVDFilterLength(void)
{
  return (uint32_t)READ_BIT(PMU->PVDCSR, PMU_PVDCSR_PVDFLTSEL);
}

/**
  * @brief  Get PVD Monitoring Event Flag
  * @param  None
  * @retval status:
  *         @arg @ref 0: No monitoring events occurred
  *         @arg @ref 1: A monitoring event has occurred
  */
__STATIC_INLINE uint32_t DDL_PMU_IsActiveFlag_PVDF(void)
{
  return (uint32_t)(READ_BIT(PMU->PVDCSR, PMU_PVDCSR_PVDFLG) == PMU_PVDCSR_PVDFLG);
}

/**
  * @brief  Clear PVD Monitoring Event Flag
  * @param  None
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_ClearFlag_PVDF(void)
{
  CLEAR_BIT(PMU->PVDCSR, PMU_PVDCSR_PVDFLG);
}

/**
  * @brief  Get PVD monitoring result output
  * @param  None
  * @retval status:
  *         @arg @ref DDL_PMU_VDD_BELOW_PVD_OUTPUT
  *         @arg @ref DDL_PMU_VDD_ABOVE_PVD_OUTPUT
  */
__STATIC_INLINE uint32_t DDL_PMU_GetPVDMonitoringResult(void)
{
  return (uint32_t)READ_BIT(PMU->PVDCSR, PMU_PVDCSR_PVDSTS);
}

/**
  * @brief  Get PVD ready status
  * @param  None
  * @retval status:
  *         @arg @ref 0: PVD is shutdown or establishing startup
  *         @arg @ref 1: PVD has ready
  */
__STATIC_INLINE uint32_t DDL_PMU_IsActiveFlag_PVDRDY(void)
{
  return (uint32_t)(READ_BIT(PMU->PVDCSR, PMU_PVDCSR_PVDRDY) == PMU_PVDCSR_PVDRDY);
}

/**
  * @}
  */

/**
  * @}
  */

#endif /* PMU */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* G32F031_DDL_PMU_H */
