/*
** ###################################################################
**     Processors:          MCXA132VFM
**                          MCXA132VFT
**                          MCXA132VLF
**                          MCXA133VFM
**                          MCXA133VFT
**                          MCXA133VLF
**                          MCXA142VFM
**                          MCXA142VFT
**                          MCXA142VLF
**                          MCXA142VLH
**                          MCXA143VFM
**                          MCXA143VFT
**                          MCXA143VLF
**                          MCXA143VLH
**                          MCXA152VFM
**                          MCXA152VFT
**                          MCXA152VLF
**                          MCXA152VLH
**                          MCXA153VFM
**                          MCXA153VFT
**                          MCXA153VLF
**                          MCXA153VLH
**
**     Version:             rev. 2.0, 2024-10-29
**     Build:               b250521
**
**     Abstract:
**         CMSIS Peripheral Access Layer for SCG
**
**     Copyright 1997-2016 Freescale Semiconductor, Inc.
**     Copyright 2016-2025 NXP
**     SPDX-License-Identifier: BSD-3-Clause
**
**     http:                 www.nxp.com
**     mail:                 support@nxp.com
**
**     Revisions:
**     - rev. 1.0 (2022-03-29)
**         Initial version based on v0.1UM
**     - rev. 2.0 (2024-10-29)
**         Change the device header file from single flat file to multiple files based on peripherals,
**         each peripheral with dedicated header file located in periphN folder.
**
** ###################################################################
*/

/*!
 * @file PERI_SCG.h
 * @version 2.0
 * @date 2024-10-29
 * @brief CMSIS Peripheral Access Layer for SCG
 *
 * CMSIS Peripheral Access Layer for SCG
 */

#if !defined(PERI_SCG_H_)
#define PERI_SCG_H_                              /**< Symbol preventing repeated inclusion */

#if (defined(CPU_MCXA132VFM) || defined(CPU_MCXA132VFT) || defined(CPU_MCXA132VLF))
#include "MCXA132_COMMON.h"
#elif (defined(CPU_MCXA133VFM) || defined(CPU_MCXA133VFT) || defined(CPU_MCXA133VLF))
#include "MCXA133_COMMON.h"
#elif (defined(CPU_MCXA142VFM) || defined(CPU_MCXA142VFT) || defined(CPU_MCXA142VLF) || defined(CPU_MCXA142VLH))
#include "MCXA142_COMMON.h"
#elif (defined(CPU_MCXA143VFM) || defined(CPU_MCXA143VFT) || defined(CPU_MCXA143VLF) || defined(CPU_MCXA143VLH))
#include "MCXA143_COMMON.h"
#elif (defined(CPU_MCXA152VFM) || defined(CPU_MCXA152VFT) || defined(CPU_MCXA152VLF) || defined(CPU_MCXA152VLH))
#include "MCXA152_COMMON.h"
#elif (defined(CPU_MCXA153VFM) || defined(CPU_MCXA153VFT) || defined(CPU_MCXA153VLF) || defined(CPU_MCXA153VLH))
#include "MCXA153_COMMON.h"
#else
  #error "No valid CPU defined!"
#endif

/* ----------------------------------------------------------------------------
   -- Device Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup Peripheral_access_layer Device Peripheral Access Layer
 * @{
 */


/*
** Start of section using anonymous unions
*/

#if defined(__ARMCC_VERSION)
  #if (__ARMCC_VERSION >= 6010050)
    #pragma clang diagnostic push
  #else
    #pragma push
    #pragma anon_unions
  #endif
#elif defined(__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined(__IAR_SYSTEMS_ICC__)
  #pragma language=extended
#else
  #error Not supported compiler type
#endif

/* ----------------------------------------------------------------------------
   -- SCG Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SCG_Peripheral_Access_Layer SCG Peripheral Access Layer
 * @{
 */

/** SCG - Register Layout Typedef */
typedef struct {
  __I  uint32_t VERID;                             /**< Version ID Register, offset: 0x0 */
  __I  uint32_t PARAM;                             /**< Parameter Register, offset: 0x4 */
  __IO uint32_t TRIM_LOCK;                         /**< Trim Lock register, offset: 0x8 */
       uint8_t RESERVED_0[4];
  __I  uint32_t CSR;                               /**< Clock Status Register, offset: 0x10 */
  __IO uint32_t RCCR;                              /**< Run Clock Control Register, offset: 0x14 */
       uint8_t RESERVED_1[232];
  __IO uint32_t SOSCCSR;                           /**< SOSC Control Status Register, offset: 0x100 */
       uint8_t RESERVED_2[4];
  __IO uint32_t SOSCCFG;                           /**< SOSC Configuration Register, offset: 0x108 */
       uint8_t RESERVED_3[244];
  __IO uint32_t SIRCCSR;                           /**< SIRC Control Status Register, offset: 0x200 */
       uint8_t RESERVED_4[8];
  __IO uint32_t SIRCTCFG;                          /**< SIRC Trim Configuration Register, offset: 0x20C */
       uint8_t RESERVED_5[8];
  __IO uint32_t SIRCSTAT;                          /**< SIRC Auto-trimming Status Register, offset: 0x218 */
       uint8_t RESERVED_6[228];
  __IO uint32_t FIRCCSR;                           /**< FIRC Control Status Register, offset: 0x300 */
       uint8_t RESERVED_7[4];
  __IO uint32_t FIRCCFG;                           /**< FIRC Configuration Register, offset: 0x308 */
  __IO uint32_t FIRCTCFG;                          /**< FIRC Trim Configuration Register, offset: 0x30C */
  __IO uint32_t FIRCTRIM;                          /**< FIRC Trim Register, offset: 0x310 */
       uint8_t RESERVED_8[4];
  __IO uint32_t FIRCSTAT;                          /**< FIRC Auto-trimming Status Register, offset: 0x318 */
       uint8_t RESERVED_9[228];
  __IO uint32_t ROSCCSR;                           /**< ROSC Control Status Register, offset: 0x400 */
} SCG_Type;

/* ----------------------------------------------------------------------------
   -- SCG Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SCG_Register_Masks SCG Register Masks
 * @{
 */

/*! @name VERID - Version ID Register */
/*! @{ */

#define SCG_VERID_VERSION_MASK                   (0xFFFFFFFFU)
#define SCG_VERID_VERSION_SHIFT                  (0U)
/*! VERSION - SCG Version Number */
#define SCG_VERID_VERSION(x)                     (((uint32_t)(((uint32_t)(x)) << SCG_VERID_VERSION_SHIFT)) & SCG_VERID_VERSION_MASK)
/*! @} */

/*! @name PARAM - Parameter Register */
/*! @{ */

#define SCG_PARAM_SOSCCLKPRES_MASK               (0x2U)
#define SCG_PARAM_SOSCCLKPRES_SHIFT              (1U)
/*! SOSCCLKPRES - SOSC Clock Present
 *  0b0..SOSC clock source is not present
 *  0b1..SOSC clock source is present
 */
#define SCG_PARAM_SOSCCLKPRES(x)                 (((uint32_t)(((uint32_t)(x)) << SCG_PARAM_SOSCCLKPRES_SHIFT)) & SCG_PARAM_SOSCCLKPRES_MASK)

#define SCG_PARAM_SIRCCLKPRES_MASK               (0x4U)
#define SCG_PARAM_SIRCCLKPRES_SHIFT              (2U)
/*! SIRCCLKPRES - SIRC Clock Present
 *  0b0..SIRC clock source is not present
 *  0b1..SIRC clock source is present
 */
#define SCG_PARAM_SIRCCLKPRES(x)                 (((uint32_t)(((uint32_t)(x)) << SCG_PARAM_SIRCCLKPRES_SHIFT)) & SCG_PARAM_SIRCCLKPRES_MASK)

#define SCG_PARAM_FIRCCLKPRES_MASK               (0x8U)
#define SCG_PARAM_FIRCCLKPRES_SHIFT              (3U)
/*! FIRCCLKPRES - FIRC Clock Present
 *  0b0..FIRC clock source is not present
 *  0b1..FIRC clock source is present
 */
#define SCG_PARAM_FIRCCLKPRES(x)                 (((uint32_t)(((uint32_t)(x)) << SCG_PARAM_FIRCCLKPRES_SHIFT)) & SCG_PARAM_FIRCCLKPRES_MASK)

#define SCG_PARAM_ROSCCLKPRES_MASK               (0x10U)
#define SCG_PARAM_ROSCCLKPRES_SHIFT              (4U)
/*! ROSCCLKPRES - ROSC Clock Present
 *  0b0..ROSC clock source is not present
 *  0b1..ROSC clock source is present
 */
#define SCG_PARAM_ROSCCLKPRES(x)                 (((uint32_t)(((uint32_t)(x)) << SCG_PARAM_ROSCCLKPRES_SHIFT)) & SCG_PARAM_ROSCCLKPRES_MASK)
/*! @} */

/*! @name TRIM_LOCK - Trim Lock register */
/*! @{ */

#define SCG_TRIM_LOCK_TRIM_UNLOCK_MASK           (0x1U)
#define SCG_TRIM_LOCK_TRIM_UNLOCK_SHIFT          (0U)
/*! TRIM_UNLOCK - TRIM_UNLOCK
 *  0b0..SCG Trim Registers locked and not writable.
 *  0b1..SCG Trim registers unlocked and writable.
 */
#define SCG_TRIM_LOCK_TRIM_UNLOCK(x)             (((uint32_t)(((uint32_t)(x)) << SCG_TRIM_LOCK_TRIM_UNLOCK_SHIFT)) & SCG_TRIM_LOCK_TRIM_UNLOCK_MASK)

#define SCG_TRIM_LOCK_IFR_DISABLE_MASK           (0x2U)
#define SCG_TRIM_LOCK_IFR_DISABLE_SHIFT          (1U)
/*! IFR_DISABLE - IFR_DISABLE
 *  0b0..IFR write access to SCG trim registers not disabled. The SCG Trim registers are reprogrammed with the IFR values after any system reset.
 *  0b1..IFR write access to SCG trim registers during system reset is blocked.
 */
#define SCG_TRIM_LOCK_IFR_DISABLE(x)             (((uint32_t)(((uint32_t)(x)) << SCG_TRIM_LOCK_IFR_DISABLE_SHIFT)) & SCG_TRIM_LOCK_IFR_DISABLE_MASK)

#define SCG_TRIM_LOCK_TRIM_LOCK_KEY_MASK         (0xFFFF0000U)
#define SCG_TRIM_LOCK_TRIM_LOCK_KEY_SHIFT        (16U)
/*! TRIM_LOCK_KEY - TRIM_LOCK_KEY */
#define SCG_TRIM_LOCK_TRIM_LOCK_KEY(x)           (((uint32_t)(((uint32_t)(x)) << SCG_TRIM_LOCK_TRIM_LOCK_KEY_SHIFT)) & SCG_TRIM_LOCK_TRIM_LOCK_KEY_MASK)
/*! @} */

/*! @name CSR - Clock Status Register */
/*! @{ */

#define SCG_CSR_SCS_MASK                         (0x7000000U)
#define SCG_CSR_SCS_SHIFT                        (24U)
/*! SCS - System Clock Source
 *  0b000..Reserved
 *  0b001..SOSC
 *  0b010..SIRC
 *  0b011..FIRC
 *  0b100..ROSC
 *  0b101-0b111..Reserved
 */
#define SCG_CSR_SCS(x)                           (((uint32_t)(((uint32_t)(x)) << SCG_CSR_SCS_SHIFT)) & SCG_CSR_SCS_MASK)
/*! @} */

/*! @name RCCR - Run Clock Control Register */
/*! @{ */

#define SCG_RCCR_SCS_MASK                        (0x7000000U)
#define SCG_RCCR_SCS_SHIFT                       (24U)
/*! SCS - System Clock Source
 *  0b000..Reserved
 *  0b001..SOSC
 *  0b010..SIRC
 *  0b011..FIRC
 *  0b100..ROSC
 *  0b101-0b111..Reserved
 */
#define SCG_RCCR_SCS(x)                          (((uint32_t)(((uint32_t)(x)) << SCG_RCCR_SCS_SHIFT)) & SCG_RCCR_SCS_MASK)
/*! @} */

/*! @name SOSCCSR - SOSC Control Status Register */
/*! @{ */

#define SCG_SOSCCSR_SOSCEN_MASK                  (0x1U)
#define SCG_SOSCCSR_SOSCEN_SHIFT                 (0U)
/*! SOSCEN - SOSC Enable
 *  0b0..SOSC is disabled
 *  0b1..SOSC is enabled
 */
#define SCG_SOSCCSR_SOSCEN(x)                    (((uint32_t)(((uint32_t)(x)) << SCG_SOSCCSR_SOSCEN_SHIFT)) & SCG_SOSCCSR_SOSCEN_MASK)

#define SCG_SOSCCSR_SOSCSTEN_MASK                (0x2U)
#define SCG_SOSCCSR_SOSCSTEN_SHIFT               (1U)
/*! SOSCSTEN - SOSC Stop Enable
 *  0b0..SOSC is disabled in Deep Sleep mode
 *  0b1..SOSC is enabled in Deep Sleep mode only if SOSCEN is set
 */
#define SCG_SOSCCSR_SOSCSTEN(x)                  (((uint32_t)(((uint32_t)(x)) << SCG_SOSCCSR_SOSCSTEN_SHIFT)) & SCG_SOSCCSR_SOSCSTEN_MASK)

#define SCG_SOSCCSR_SOSCCM_MASK                  (0x10000U)
#define SCG_SOSCCSR_SOSCCM_SHIFT                 (16U)
/*! SOSCCM - SOSC Clock Monitor Enable
 *  0b0..SOSC Clock Monitor is disabled
 *  0b1..SOSC Clock Monitor is enabled
 */
#define SCG_SOSCCSR_SOSCCM(x)                    (((uint32_t)(((uint32_t)(x)) << SCG_SOSCCSR_SOSCCM_SHIFT)) & SCG_SOSCCSR_SOSCCM_MASK)

#define SCG_SOSCCSR_SOSCCMRE_MASK                (0x20000U)
#define SCG_SOSCCSR_SOSCCMRE_SHIFT               (17U)
/*! SOSCCMRE - SOSC Clock Monitor Reset Enable
 *  0b0..Clock monitor generates an interrupt when an error is detected
 *  0b1..Clock monitor generates a reset when an error is detected
 */
#define SCG_SOSCCSR_SOSCCMRE(x)                  (((uint32_t)(((uint32_t)(x)) << SCG_SOSCCSR_SOSCCMRE_SHIFT)) & SCG_SOSCCSR_SOSCCMRE_MASK)

#define SCG_SOSCCSR_LK_MASK                      (0x800000U)
#define SCG_SOSCCSR_LK_SHIFT                     (23U)
/*! LK - Lock Register
 *  0b0..This Control Status Register can be written
 *  0b1..This Control Status Register cannot be written
 */
#define SCG_SOSCCSR_LK(x)                        (((uint32_t)(((uint32_t)(x)) << SCG_SOSCCSR_LK_SHIFT)) & SCG_SOSCCSR_LK_MASK)

#define SCG_SOSCCSR_SOSCVLD_MASK                 (0x1000000U)
#define SCG_SOSCCSR_SOSCVLD_SHIFT                (24U)
/*! SOSCVLD - SOSC Valid
 *  0b0..SOSC is not enabled or clock is not valid
 *  0b1..SOSC is enabled and output clock is valid
 */
#define SCG_SOSCCSR_SOSCVLD(x)                   (((uint32_t)(((uint32_t)(x)) << SCG_SOSCCSR_SOSCVLD_SHIFT)) & SCG_SOSCCSR_SOSCVLD_MASK)

#define SCG_SOSCCSR_SOSCSEL_MASK                 (0x2000000U)
#define SCG_SOSCCSR_SOSCSEL_SHIFT                (25U)
/*! SOSCSEL - SOSC Selected
 *  0b0..SOSC is not the system clock source
 *  0b1..SOSC is the system clock source
 */
#define SCG_SOSCCSR_SOSCSEL(x)                   (((uint32_t)(((uint32_t)(x)) << SCG_SOSCCSR_SOSCSEL_SHIFT)) & SCG_SOSCCSR_SOSCSEL_MASK)

#define SCG_SOSCCSR_SOSCERR_MASK                 (0x4000000U)
#define SCG_SOSCCSR_SOSCERR_SHIFT                (26U)
/*! SOSCERR - SOSC Clock Error
 *  0b0..SOSC Clock Monitor is disabled or has not detected an error
 *  0b1..SOSC Clock Monitor is enabled and detected an error
 */
#define SCG_SOSCCSR_SOSCERR(x)                   (((uint32_t)(((uint32_t)(x)) << SCG_SOSCCSR_SOSCERR_SHIFT)) & SCG_SOSCCSR_SOSCERR_MASK)

#define SCG_SOSCCSR_SOSCVLD_IE_MASK              (0x40000000U)
#define SCG_SOSCCSR_SOSCVLD_IE_SHIFT             (30U)
/*! SOSCVLD_IE - SOSC Valid Interrupt Enable
 *  0b0..SOSCVLD interrupt is not enabled
 *  0b1..SOSCVLD interrupt is enabled
 */
#define SCG_SOSCCSR_SOSCVLD_IE(x)                (((uint32_t)(((uint32_t)(x)) << SCG_SOSCCSR_SOSCVLD_IE_SHIFT)) & SCG_SOSCCSR_SOSCVLD_IE_MASK)
/*! @} */

/*! @name SOSCCFG - SOSC Configuration Register */
/*! @{ */

#define SCG_SOSCCFG_EREFS_MASK                   (0x4U)
#define SCG_SOSCCFG_EREFS_SHIFT                  (2U)
/*! EREFS - External Reference Select
 *  0b0..External reference clock selected.
 *  0b1..Internal crystal oscillator of OSC selected.
 */
#define SCG_SOSCCFG_EREFS(x)                     (((uint32_t)(((uint32_t)(x)) << SCG_SOSCCFG_EREFS_SHIFT)) & SCG_SOSCCFG_EREFS_MASK)

#define SCG_SOSCCFG_RANGE_MASK                   (0x30U)
#define SCG_SOSCCFG_RANGE_SHIFT                  (4U)
/*! RANGE - SOSC Range Select
 *  0b00..Frequency range select of 8-16 MHz.
 *  0b01..Frequency range select of 16-25 MHz.
 *  0b10..Frequency range select of 25-40 MHz.
 *  0b11..Frequency range select of 40-50 MHz.
 */
#define SCG_SOSCCFG_RANGE(x)                     (((uint32_t)(((uint32_t)(x)) << SCG_SOSCCFG_RANGE_SHIFT)) & SCG_SOSCCFG_RANGE_MASK)
/*! @} */

/*! @name SIRCCSR - SIRC Control Status Register */
/*! @{ */

#define SCG_SIRCCSR_SIRCSTEN_MASK                (0x2U)
#define SCG_SIRCCSR_SIRCSTEN_SHIFT               (1U)
/*! SIRCSTEN - SIRC Stop Enable
 *  0b0..SIRC is disabled in Deep Sleep mode
 *  0b1..SIRC is enabled in Deep Sleep mode
 */
#define SCG_SIRCCSR_SIRCSTEN(x)                  (((uint32_t)(((uint32_t)(x)) << SCG_SIRCCSR_SIRCSTEN_SHIFT)) & SCG_SIRCCSR_SIRCSTEN_MASK)

#define SCG_SIRCCSR_SIRC_CLK_PERIPH_EN_MASK      (0x20U)
#define SCG_SIRCCSR_SIRC_CLK_PERIPH_EN_SHIFT     (5U)
/*! SIRC_CLK_PERIPH_EN - SIRC Clock to Peripherals Enable
 *  0b0..SIRC clock to peripherals is disabled
 *  0b1..SIRC clock to peripherals is enabled
 */
#define SCG_SIRCCSR_SIRC_CLK_PERIPH_EN(x)        (((uint32_t)(((uint32_t)(x)) << SCG_SIRCCSR_SIRC_CLK_PERIPH_EN_SHIFT)) & SCG_SIRCCSR_SIRC_CLK_PERIPH_EN_MASK)

#define SCG_SIRCCSR_SIRCTREN_MASK                (0x100U)
#define SCG_SIRCCSR_SIRCTREN_SHIFT               (8U)
/*! SIRCTREN - SIRC 12 MHz Trim Enable (SIRCCFG[RANGE]=1)
 *  0b0..Disables trimming SIRC to an external clock source
 *  0b1..Enables trimming SIRC to an external clock source
 */
#define SCG_SIRCCSR_SIRCTREN(x)                  (((uint32_t)(((uint32_t)(x)) << SCG_SIRCCSR_SIRCTREN_SHIFT)) & SCG_SIRCCSR_SIRCTREN_MASK)

#define SCG_SIRCCSR_SIRCTRUP_MASK                (0x200U)
#define SCG_SIRCCSR_SIRCTRUP_SHIFT               (9U)
/*! SIRCTRUP - SIRC Trim Update
 *  0b0..Disables SIRC trimming updates
 *  0b1..Enables SIRC trimming updates
 */
#define SCG_SIRCCSR_SIRCTRUP(x)                  (((uint32_t)(((uint32_t)(x)) << SCG_SIRCCSR_SIRCTRUP_SHIFT)) & SCG_SIRCCSR_SIRCTRUP_MASK)

#define SCG_SIRCCSR_TRIM_LOCK_MASK               (0x400U)
#define SCG_SIRCCSR_TRIM_LOCK_SHIFT              (10U)
/*! TRIM_LOCK - SIRC TRIM LOCK
 *  0b0..SIRC auto trim not locked to target frequency range
 *  0b1..SIRC auto trim locked to target frequency range
 */
#define SCG_SIRCCSR_TRIM_LOCK(x)                 (((uint32_t)(((uint32_t)(x)) << SCG_SIRCCSR_TRIM_LOCK_SHIFT)) & SCG_SIRCCSR_TRIM_LOCK_MASK)

#define SCG_SIRCCSR_COARSE_TRIM_BYPASS_MASK      (0x800U)
#define SCG_SIRCCSR_COARSE_TRIM_BYPASS_SHIFT     (11U)
/*! COARSE_TRIM_BYPASS - Coarse Auto Trim Bypass
 *  0b0..SIRC Coarse Auto Trim NOT Bypassed
 *  0b1..SIRC Coarse Auto Trim Bypassed
 */
#define SCG_SIRCCSR_COARSE_TRIM_BYPASS(x)        (((uint32_t)(((uint32_t)(x)) << SCG_SIRCCSR_COARSE_TRIM_BYPASS_SHIFT)) & SCG_SIRCCSR_COARSE_TRIM_BYPASS_MASK)

#define SCG_SIRCCSR_LK_MASK                      (0x800000U)
#define SCG_SIRCCSR_LK_SHIFT                     (23U)
/*! LK - Lock Register
 *  0b0..Control Status Register can be written
 *  0b1..Control Status Register cannot be written
 */
#define SCG_SIRCCSR_LK(x)                        (((uint32_t)(((uint32_t)(x)) << SCG_SIRCCSR_LK_SHIFT)) & SCG_SIRCCSR_LK_MASK)

#define SCG_SIRCCSR_SIRCVLD_MASK                 (0x1000000U)
#define SCG_SIRCCSR_SIRCVLD_SHIFT                (24U)
/*! SIRCVLD - SIRC Valid
 *  0b0..SIRC is not enabled or clock is not valid
 *  0b1..SIRC is enabled and output clock is valid
 */
#define SCG_SIRCCSR_SIRCVLD(x)                   (((uint32_t)(((uint32_t)(x)) << SCG_SIRCCSR_SIRCVLD_SHIFT)) & SCG_SIRCCSR_SIRCVLD_MASK)

#define SCG_SIRCCSR_SIRCSEL_MASK                 (0x2000000U)
#define SCG_SIRCCSR_SIRCSEL_SHIFT                (25U)
/*! SIRCSEL - SIRC Selected
 *  0b0..SIRC is not the system clock source
 *  0b1..SIRC is the system clock source
 */
#define SCG_SIRCCSR_SIRCSEL(x)                   (((uint32_t)(((uint32_t)(x)) << SCG_SIRCCSR_SIRCSEL_SHIFT)) & SCG_SIRCCSR_SIRCSEL_MASK)

#define SCG_SIRCCSR_SIRCERR_MASK                 (0x4000000U)
#define SCG_SIRCCSR_SIRCERR_SHIFT                (26U)
/*! SIRCERR - SIRC Clock Error
 *  0b0..Error not detected with the SIRC trimming
 *  0b1..Error detected with the SIRC trimming
 */
#define SCG_SIRCCSR_SIRCERR(x)                   (((uint32_t)(((uint32_t)(x)) << SCG_SIRCCSR_SIRCERR_SHIFT)) & SCG_SIRCCSR_SIRCERR_MASK)

#define SCG_SIRCCSR_SIRCERR_IE_MASK              (0x8000000U)
#define SCG_SIRCCSR_SIRCERR_IE_SHIFT             (27U)
/*! SIRCERR_IE - SIRC Clock Error Interrupt Enable
 *  0b0..SIRCERR interrupt is not enabled
 *  0b1..SIRCERR interrupt is enabled
 */
#define SCG_SIRCCSR_SIRCERR_IE(x)                (((uint32_t)(((uint32_t)(x)) << SCG_SIRCCSR_SIRCERR_IE_SHIFT)) & SCG_SIRCCSR_SIRCERR_IE_MASK)
/*! @} */

/*! @name SIRCTCFG - SIRC Trim Configuration Register */
/*! @{ */

#define SCG_SIRCTCFG_TRIMSRC_MASK                (0x3U)
#define SCG_SIRCTCFG_TRIMSRC_SHIFT               (0U)
/*! TRIMSRC - Trim Source
 *  0b00..Reserved
 *  0b01..Reserved
 *  0b10..SOSC. This option requires that SOSC be divided using the TRIMDIV field to get a frequency of 1 MHz.
 *  0b11..Reserved
 */
#define SCG_SIRCTCFG_TRIMSRC(x)                  (((uint32_t)(((uint32_t)(x)) << SCG_SIRCTCFG_TRIMSRC_SHIFT)) & SCG_SIRCTCFG_TRIMSRC_MASK)

#define SCG_SIRCTCFG_TRIMDIV_MASK                (0x7F0000U)
#define SCG_SIRCTCFG_TRIMDIV_SHIFT               (16U)
/*! TRIMDIV - SIRC Trim Pre-divider */
#define SCG_SIRCTCFG_TRIMDIV(x)                  (((uint32_t)(((uint32_t)(x)) << SCG_SIRCTCFG_TRIMDIV_SHIFT)) & SCG_SIRCTCFG_TRIMDIV_MASK)
/*! @} */

/*! @name SIRCSTAT - SIRC Auto-trimming Status Register */
/*! @{ */

#define SCG_SIRCSTAT_CCOTRIM_MASK                (0x3FU)
#define SCG_SIRCSTAT_CCOTRIM_SHIFT               (0U)
/*! CCOTRIM - CCO Trim */
#define SCG_SIRCSTAT_CCOTRIM(x)                  (((uint32_t)(((uint32_t)(x)) << SCG_SIRCSTAT_CCOTRIM_SHIFT)) & SCG_SIRCSTAT_CCOTRIM_MASK)

#define SCG_SIRCSTAT_CLTRIM_MASK                 (0x3F00U)
#define SCG_SIRCSTAT_CLTRIM_SHIFT                (8U)
/*! CLTRIM - CL Trim */
#define SCG_SIRCSTAT_CLTRIM(x)                   (((uint32_t)(((uint32_t)(x)) << SCG_SIRCSTAT_CLTRIM_SHIFT)) & SCG_SIRCSTAT_CLTRIM_MASK)
/*! @} */

/*! @name FIRCCSR - FIRC Control Status Register */
/*! @{ */

#define SCG_FIRCCSR_FIRCEN_MASK                  (0x1U)
#define SCG_FIRCCSR_FIRCEN_SHIFT                 (0U)
/*! FIRCEN - FIRC Enable
 *  0b0..FIRC is disabled
 *  0b1..FIRC is enabled
 */
#define SCG_FIRCCSR_FIRCEN(x)                    (((uint32_t)(((uint32_t)(x)) << SCG_FIRCCSR_FIRCEN_SHIFT)) & SCG_FIRCCSR_FIRCEN_MASK)

#define SCG_FIRCCSR_FIRCSTEN_MASK                (0x2U)
#define SCG_FIRCCSR_FIRCSTEN_SHIFT               (1U)
/*! FIRCSTEN - FIRC Stop Enable
 *  0b0..FIRC is disabled in Deep Sleep mode
 *  0b1..FIRC is enabled in Deep Sleep mode
 */
#define SCG_FIRCCSR_FIRCSTEN(x)                  (((uint32_t)(((uint32_t)(x)) << SCG_FIRCCSR_FIRCSTEN_SHIFT)) & SCG_FIRCCSR_FIRCSTEN_MASK)

#define SCG_FIRCCSR_FIRC_SCLK_PERIPH_EN_MASK     (0x10U)
#define SCG_FIRCCSR_FIRC_SCLK_PERIPH_EN_SHIFT    (4U)
/*! FIRC_SCLK_PERIPH_EN - FIRC 48 MHz Clock to peripherals Enable
 *  0b0..FIRC 48 MHz to peripherals is disabled
 *  0b1..FIRC 48 MHz to peripherals is enabled
 */
#define SCG_FIRCCSR_FIRC_SCLK_PERIPH_EN(x)       (((uint32_t)(((uint32_t)(x)) << SCG_FIRCCSR_FIRC_SCLK_PERIPH_EN_SHIFT)) & SCG_FIRCCSR_FIRC_SCLK_PERIPH_EN_MASK)

#define SCG_FIRCCSR_FIRC_FCLK_PERIPH_EN_MASK     (0x20U)
#define SCG_FIRCCSR_FIRC_FCLK_PERIPH_EN_SHIFT    (5U)
/*! FIRC_FCLK_PERIPH_EN - FRO_HF Clock to peripherals Enable
 *  0b0..FRO_HF to peripherals is disabled
 *  0b1..FRO_HF to peripherals is enabled
 */
#define SCG_FIRCCSR_FIRC_FCLK_PERIPH_EN(x)       (((uint32_t)(((uint32_t)(x)) << SCG_FIRCCSR_FIRC_FCLK_PERIPH_EN_SHIFT)) & SCG_FIRCCSR_FIRC_FCLK_PERIPH_EN_MASK)

#define SCG_FIRCCSR_FIRCTREN_MASK                (0x100U)
#define SCG_FIRCCSR_FIRCTREN_SHIFT               (8U)
/*! FIRCTREN - FRO_HF Trim Enable
 *  0b0..Disables trimming FRO_HF by an external clock source
 *  0b1..Enables trimming FRO_HF by an external clock source
 */
#define SCG_FIRCCSR_FIRCTREN(x)                  (((uint32_t)(((uint32_t)(x)) << SCG_FIRCCSR_FIRCTREN_SHIFT)) & SCG_FIRCCSR_FIRCTREN_MASK)

#define SCG_FIRCCSR_FIRCTRUP_MASK                (0x200U)
#define SCG_FIRCCSR_FIRCTRUP_SHIFT               (9U)
/*! FIRCTRUP - FIRC Trim Update
 *  0b0..Disables FIRC trimming updates
 *  0b1..Enables FIRC trimming updates
 */
#define SCG_FIRCCSR_FIRCTRUP(x)                  (((uint32_t)(((uint32_t)(x)) << SCG_FIRCCSR_FIRCTRUP_SHIFT)) & SCG_FIRCCSR_FIRCTRUP_MASK)

#define SCG_FIRCCSR_TRIM_LOCK_MASK               (0x400U)
#define SCG_FIRCCSR_TRIM_LOCK_SHIFT              (10U)
/*! TRIM_LOCK - FIRC TRIM LOCK
 *  0b0..FIRC auto trim not locked to target frequency range
 *  0b1..FIRC auto trim locked to target frequency range
 */
#define SCG_FIRCCSR_TRIM_LOCK(x)                 (((uint32_t)(((uint32_t)(x)) << SCG_FIRCCSR_TRIM_LOCK_SHIFT)) & SCG_FIRCCSR_TRIM_LOCK_MASK)

#define SCG_FIRCCSR_COARSE_TRIM_BYPASS_MASK      (0x800U)
#define SCG_FIRCCSR_COARSE_TRIM_BYPASS_SHIFT     (11U)
/*! COARSE_TRIM_BYPASS - Coarse Auto Trim Bypass
 *  0b0..FIRC Coarse Auto Trim NOT Bypassed
 *  0b1..FIRC Coarse Auto Trim Bypassed
 */
#define SCG_FIRCCSR_COARSE_TRIM_BYPASS(x)        (((uint32_t)(((uint32_t)(x)) << SCG_FIRCCSR_COARSE_TRIM_BYPASS_SHIFT)) & SCG_FIRCCSR_COARSE_TRIM_BYPASS_MASK)

#define SCG_FIRCCSR_LK_MASK                      (0x800000U)
#define SCG_FIRCCSR_LK_SHIFT                     (23U)
/*! LK - Lock Register
 *  0b0..Control Status Register can be written
 *  0b1..Control Status Register cannot be written
 */
#define SCG_FIRCCSR_LK(x)                        (((uint32_t)(((uint32_t)(x)) << SCG_FIRCCSR_LK_SHIFT)) & SCG_FIRCCSR_LK_MASK)

#define SCG_FIRCCSR_FIRCVLD_MASK                 (0x1000000U)
#define SCG_FIRCCSR_FIRCVLD_SHIFT                (24U)
/*! FIRCVLD - FIRC Valid status
 *  0b0..FIRC is not enabled or clock is not valid.
 *  0b1..FIRC is enabled and output clock is valid. The clock is valid after there is an output clock from the FIRC analog.
 */
#define SCG_FIRCCSR_FIRCVLD(x)                   (((uint32_t)(((uint32_t)(x)) << SCG_FIRCCSR_FIRCVLD_SHIFT)) & SCG_FIRCCSR_FIRCVLD_MASK)

#define SCG_FIRCCSR_FIRCSEL_MASK                 (0x2000000U)
#define SCG_FIRCCSR_FIRCSEL_SHIFT                (25U)
/*! FIRCSEL - FIRC Selected
 *  0b0..FIRC is not the system clock source
 *  0b1..FIRC is the system clock source
 */
#define SCG_FIRCCSR_FIRCSEL(x)                   (((uint32_t)(((uint32_t)(x)) << SCG_FIRCCSR_FIRCSEL_SHIFT)) & SCG_FIRCCSR_FIRCSEL_MASK)

#define SCG_FIRCCSR_FIRCERR_MASK                 (0x4000000U)
#define SCG_FIRCCSR_FIRCERR_SHIFT                (26U)
/*! FIRCERR - FIRC Clock Error
 *  0b0..Error not detected with the FIRC trimming
 *  0b1..Error detected with the FIRC trimming
 */
#define SCG_FIRCCSR_FIRCERR(x)                   (((uint32_t)(((uint32_t)(x)) << SCG_FIRCCSR_FIRCERR_SHIFT)) & SCG_FIRCCSR_FIRCERR_MASK)

#define SCG_FIRCCSR_FIRCERR_IE_MASK              (0x8000000U)
#define SCG_FIRCCSR_FIRCERR_IE_SHIFT             (27U)
/*! FIRCERR_IE - FIRC Clock Error Interrupt Enable
 *  0b0..FIRCERR interrupt is not enabled
 *  0b1..FIRCERR interrupt is enabled
 */
#define SCG_FIRCCSR_FIRCERR_IE(x)                (((uint32_t)(((uint32_t)(x)) << SCG_FIRCCSR_FIRCERR_IE_SHIFT)) & SCG_FIRCCSR_FIRCERR_IE_MASK)

#define SCG_FIRCCSR_FIRCACC_IE_MASK              (0x40000000U)
#define SCG_FIRCCSR_FIRCACC_IE_SHIFT             (30U)
/*! FIRCACC_IE - FIRC Accurate Interrupt Enable
 *  0b0..FIRCACC interrupt is not enabled
 *  0b1..FIRCACC interrupt is enabled
 */
#define SCG_FIRCCSR_FIRCACC_IE(x)                (((uint32_t)(((uint32_t)(x)) << SCG_FIRCCSR_FIRCACC_IE_SHIFT)) & SCG_FIRCCSR_FIRCACC_IE_MASK)

#define SCG_FIRCCSR_FIRCACC_MASK                 (0x80000000U)
#define SCG_FIRCCSR_FIRCACC_SHIFT                (31U)
/*! FIRCACC - FIRC Frequency Accurate
 *  0b0..FIRC is not enabled or clock is not accurate.
 *  0b1..FIRC is enabled and output clock is accurate. The clock is accurate after 4096 clock cycles of FRO_HF
 *       clock(It also takes 4096 clock cycles when FIRCCFG_FREQ_SEL[0] changes) or 1365 clock cycles of 48 MHz from
 *       the FIRC analog.
 */
#define SCG_FIRCCSR_FIRCACC(x)                   (((uint32_t)(((uint32_t)(x)) << SCG_FIRCCSR_FIRCACC_SHIFT)) & SCG_FIRCCSR_FIRCACC_MASK)
/*! @} */

/*! @name FIRCCFG - FIRC Configuration Register */
/*! @{ */

#define SCG_FIRCCFG_FREQ_SEL_MASK                (0xEU)
#define SCG_FIRCCFG_FREQ_SEL_SHIFT               (1U)
/*! FREQ_SEL - Frequency select
 *  0b000..36 MHz FIRC clock selected
 *  0b001..48 MHz FIRC clock selected, divided from 192 MHz
 *  0b010..48 MHz FIRC clock selected, divided from 144 MHz
 *  0b011..64 MHz FIRC clock selected
 *  0b100..72 MHz FIRC clock selected
 *  0b101..96 MHz FIRC clock selected
 *  0b110..144 MHz FIRC clock selected
 *  0b111..192 MHz FIRC clock selected
 */
#define SCG_FIRCCFG_FREQ_SEL(x)                  (((uint32_t)(((uint32_t)(x)) << SCG_FIRCCFG_FREQ_SEL_SHIFT)) & SCG_FIRCCFG_FREQ_SEL_MASK)
/*! @} */

/*! @name FIRCTCFG - FIRC Trim Configuration Register */
/*! @{ */

#define SCG_FIRCTCFG_TRIMSRC_MASK                (0x3U)
#define SCG_FIRCTCFG_TRIMSRC_SHIFT               (0U)
/*! TRIMSRC - Trim Source
 *  0b00..USB0 Start of Frame (1 KHz). This option does not use TRIMDIV .
 *  0b01..Reserved
 *  0b10..SOSC. This option requires that SOSC be divided using the TRIMDIV field to get a frequency of 1 MHz.
 *  0b11..Reserved
 */
#define SCG_FIRCTCFG_TRIMSRC(x)                  (((uint32_t)(((uint32_t)(x)) << SCG_FIRCTCFG_TRIMSRC_SHIFT)) & SCG_FIRCTCFG_TRIMSRC_MASK)

#define SCG_FIRCTCFG_TRIMDIV_MASK                (0x7F0000U)
#define SCG_FIRCTCFG_TRIMDIV_SHIFT               (16U)
/*! TRIMDIV - FIRC Trim Pre-divider */
#define SCG_FIRCTCFG_TRIMDIV(x)                  (((uint32_t)(((uint32_t)(x)) << SCG_FIRCTCFG_TRIMDIV_SHIFT)) & SCG_FIRCTCFG_TRIMDIV_MASK)
/*! @} */

/*! @name FIRCTRIM - FIRC Trim Register */
/*! @{ */

#define SCG_FIRCTRIM_TRIMFINE_MASK               (0xFFU)
#define SCG_FIRCTRIM_TRIMFINE_SHIFT              (0U)
/*! TRIMFINE - Trim Fine */
#define SCG_FIRCTRIM_TRIMFINE(x)                 (((uint32_t)(((uint32_t)(x)) << SCG_FIRCTRIM_TRIMFINE_SHIFT)) & SCG_FIRCTRIM_TRIMFINE_MASK)

#define SCG_FIRCTRIM_TRIMCOAR_MASK               (0x3F00U)
#define SCG_FIRCTRIM_TRIMCOAR_SHIFT              (8U)
/*! TRIMCOAR - Trim Coarse */
#define SCG_FIRCTRIM_TRIMCOAR(x)                 (((uint32_t)(((uint32_t)(x)) << SCG_FIRCTRIM_TRIMCOAR_SHIFT)) & SCG_FIRCTRIM_TRIMCOAR_MASK)

#define SCG_FIRCTRIM_TRIMTEMP1_MASK              (0x30000U)
#define SCG_FIRCTRIM_TRIMTEMP1_SHIFT             (16U)
/*! TRIMTEMP1 - Trim Temperature1 */
#define SCG_FIRCTRIM_TRIMTEMP1(x)                (((uint32_t)(((uint32_t)(x)) << SCG_FIRCTRIM_TRIMTEMP1_SHIFT)) & SCG_FIRCTRIM_TRIMTEMP1_MASK)

#define SCG_FIRCTRIM_TRIMTEMP2_MASK              (0xC0000U)
#define SCG_FIRCTRIM_TRIMTEMP2_SHIFT             (18U)
/*! TRIMTEMP2 - Trim Temperature2 */
#define SCG_FIRCTRIM_TRIMTEMP2(x)                (((uint32_t)(((uint32_t)(x)) << SCG_FIRCTRIM_TRIMTEMP2_SHIFT)) & SCG_FIRCTRIM_TRIMTEMP2_MASK)

#define SCG_FIRCTRIM_TRIMSTART_MASK              (0x3F000000U)
#define SCG_FIRCTRIM_TRIMSTART_SHIFT             (24U)
/*! TRIMSTART - Trim Start */
#define SCG_FIRCTRIM_TRIMSTART(x)                (((uint32_t)(((uint32_t)(x)) << SCG_FIRCTRIM_TRIMSTART_SHIFT)) & SCG_FIRCTRIM_TRIMSTART_MASK)
/*! @} */

/*! @name FIRCSTAT - FIRC Auto-trimming Status Register */
/*! @{ */

#define SCG_FIRCSTAT_TRIMFINE_MASK               (0xFFU)
#define SCG_FIRCSTAT_TRIMFINE_SHIFT              (0U)
/*! TRIMFINE - Trim Fine */
#define SCG_FIRCSTAT_TRIMFINE(x)                 (((uint32_t)(((uint32_t)(x)) << SCG_FIRCSTAT_TRIMFINE_SHIFT)) & SCG_FIRCSTAT_TRIMFINE_MASK)

#define SCG_FIRCSTAT_TRIMCOAR_MASK               (0x3F00U)
#define SCG_FIRCSTAT_TRIMCOAR_SHIFT              (8U)
/*! TRIMCOAR - Trim Coarse */
#define SCG_FIRCSTAT_TRIMCOAR(x)                 (((uint32_t)(((uint32_t)(x)) << SCG_FIRCSTAT_TRIMCOAR_SHIFT)) & SCG_FIRCSTAT_TRIMCOAR_MASK)
/*! @} */

/*! @name ROSCCSR - ROSC Control Status Register */
/*! @{ */

#define SCG_ROSCCSR_LK_MASK                      (0x800000U)
#define SCG_ROSCCSR_LK_SHIFT                     (23U)
/*! LK - Lock Register
 *  0b0..Control Status Register can be written
 *  0b1..Control Status Register cannot be written
 */
#define SCG_ROSCCSR_LK(x)                        (((uint32_t)(((uint32_t)(x)) << SCG_ROSCCSR_LK_SHIFT)) & SCG_ROSCCSR_LK_MASK)

#define SCG_ROSCCSR_ROSCVLD_MASK                 (0x1000000U)
#define SCG_ROSCCSR_ROSCVLD_SHIFT                (24U)
/*! ROSCVLD - ROSC Valid
 *  0b0..ROSC is not enabled or clock is not valid
 *  0b1..ROSC is enabled and output clock is valid
 */
#define SCG_ROSCCSR_ROSCVLD(x)                   (((uint32_t)(((uint32_t)(x)) << SCG_ROSCCSR_ROSCVLD_SHIFT)) & SCG_ROSCCSR_ROSCVLD_MASK)

#define SCG_ROSCCSR_ROSCSEL_MASK                 (0x2000000U)
#define SCG_ROSCCSR_ROSCSEL_SHIFT                (25U)
/*! ROSCSEL - ROSC Selected
 *  0b0..ROSC is not the system clock source
 *  0b1..ROSC is the system clock source
 */
#define SCG_ROSCCSR_ROSCSEL(x)                   (((uint32_t)(((uint32_t)(x)) << SCG_ROSCCSR_ROSCSEL_SHIFT)) & SCG_ROSCCSR_ROSCSEL_MASK)

#define SCG_ROSCCSR_ROSCERR_MASK                 (0x4000000U)
#define SCG_ROSCCSR_ROSCERR_SHIFT                (26U)
/*! ROSCERR - ROSC Clock Error
 *  0b0..ROSC Clock has not detected an error
 *  0b1..ROSC Clock has detected an error
 */
#define SCG_ROSCCSR_ROSCERR(x)                   (((uint32_t)(((uint32_t)(x)) << SCG_ROSCCSR_ROSCERR_SHIFT)) & SCG_ROSCCSR_ROSCERR_MASK)
/*! @} */


/*!
 * @}
 */ /* end of group SCG_Register_Masks */


/*!
 * @}
 */ /* end of group SCG_Peripheral_Access_Layer */


/*
** End of section using anonymous unions
*/

#if defined(__ARMCC_VERSION)
  #if (__ARMCC_VERSION >= 6010050)
    #pragma clang diagnostic pop
  #else
    #pragma pop
  #endif
#elif defined(__GNUC__)
  /* leave anonymous unions enabled */
#elif defined(__IAR_SYSTEMS_ICC__)
  #pragma language=default
#else
  #error Not supported compiler type
#endif

/*!
 * @}
 */ /* end of group Peripheral_access_layer */


#endif  /* PERI_SCG_H_ */

