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
**         CMSIS Peripheral Access Layer for CMC
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
 * @file PERI_CMC.h
 * @version 2.0
 * @date 2024-10-29
 * @brief CMSIS Peripheral Access Layer for CMC
 *
 * CMSIS Peripheral Access Layer for CMC
 */

#if !defined(PERI_CMC_H_)
#define PERI_CMC_H_                              /**< Symbol preventing repeated inclusion */

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
   -- CMC Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CMC_Peripheral_Access_Layer CMC Peripheral Access Layer
 * @{
 */

/** CMC - Size of Registers Arrays */
#define CMC_PMCTRL_COUNT                          1u
#define CMC_MR_COUNT                              1u
#define CMC_FM_COUNT                              1u

/** CMC - Register Layout Typedef */
typedef struct {
  __I  uint32_t VERID;                             /**< Version ID, offset: 0x0 */
       uint8_t RESERVED_0[12];
  __IO uint32_t CKCTRL;                            /**< Clock Control, offset: 0x10 */
  __IO uint32_t CKSTAT;                            /**< Clock Status, offset: 0x14 */
  __IO uint32_t PMPROT;                            /**< Power Mode Protection, offset: 0x18 */
  __IO uint32_t GPMCTRL;                           /**< Global Power Mode Control, offset: 0x1C */
  __IO uint32_t PMCTRL[CMC_PMCTRL_COUNT];          /**< Power Mode Control, array offset: 0x20, array step: 0x4 */
       uint8_t RESERVED_1[92];
  __I  uint32_t SRS;                               /**< System Reset Status, offset: 0x80 */
  __IO uint32_t RPC;                               /**< Reset Pin Control, offset: 0x84 */
  __IO uint32_t SSRS;                              /**< Sticky System Reset Status, offset: 0x88 */
  __IO uint32_t SRIE;                              /**< System Reset Interrupt Enable, offset: 0x8C */
  __IO uint32_t SRIF;                              /**< System Reset Interrupt Flag, offset: 0x90 */
       uint8_t RESERVED_2[8];
  __I  uint32_t RSTCNT;                            /**< Reset Count Register, offset: 0x9C */
  __IO uint32_t MR[CMC_MR_COUNT];                  /**< Mode, array offset: 0xA0, array step: 0x4 */
       uint8_t RESERVED_3[12];
  __IO uint32_t FM[CMC_FM_COUNT];                  /**< Force Mode, array offset: 0xB0, array step: 0x4 */
       uint8_t RESERVED_4[44];
  __IO uint32_t FLASHCR;                           /**< Flash Control, offset: 0xE0 */
       uint8_t RESERVED_5[44];
  __IO uint32_t CORECTL;                           /**< Core Control, offset: 0x110 */
       uint8_t RESERVED_6[12];
  __IO uint32_t DBGCTL;                            /**< Debug Control, offset: 0x120 */
} CMC_Type;

/* ----------------------------------------------------------------------------
   -- CMC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CMC_Register_Masks CMC Register Masks
 * @{
 */

/*! @name VERID - Version ID */
/*! @{ */

#define CMC_VERID_FEATURE_MASK                   (0xFFFFU)
#define CMC_VERID_FEATURE_SHIFT                  (0U)
/*! FEATURE - Feature Specification Number */
#define CMC_VERID_FEATURE(x)                     (((uint32_t)(((uint32_t)(x)) << CMC_VERID_FEATURE_SHIFT)) & CMC_VERID_FEATURE_MASK)

#define CMC_VERID_MINOR_MASK                     (0xFF0000U)
#define CMC_VERID_MINOR_SHIFT                    (16U)
/*! MINOR - Minor Version Number */
#define CMC_VERID_MINOR(x)                       (((uint32_t)(((uint32_t)(x)) << CMC_VERID_MINOR_SHIFT)) & CMC_VERID_MINOR_MASK)

#define CMC_VERID_MAJOR_MASK                     (0xFF000000U)
#define CMC_VERID_MAJOR_SHIFT                    (24U)
/*! MAJOR - Major Version Number */
#define CMC_VERID_MAJOR(x)                       (((uint32_t)(((uint32_t)(x)) << CMC_VERID_MAJOR_SHIFT)) & CMC_VERID_MAJOR_MASK)
/*! @} */

/*! @name CKCTRL - Clock Control */
/*! @{ */

#define CMC_CKCTRL_CKMODE_MASK                   (0xFU)
#define CMC_CKCTRL_CKMODE_SHIFT                  (0U)
/*! CKMODE - Clocking Mode
 *  0b0000..No clock gating
 *  0b1111..Core, platform, and peripheral clocks are gated, and core enters Low-Power mode.
 */
#define CMC_CKCTRL_CKMODE(x)                     (((uint32_t)(((uint32_t)(x)) << CMC_CKCTRL_CKMODE_SHIFT)) & CMC_CKCTRL_CKMODE_MASK)

#define CMC_CKCTRL_LOCK_MASK                     (0x80000000U)
#define CMC_CKCTRL_LOCK_SHIFT                    (31U)
/*! LOCK - Lock
 *  0b0..Allowed
 *  0b1..Blocked
 */
#define CMC_CKCTRL_LOCK(x)                       (((uint32_t)(((uint32_t)(x)) << CMC_CKCTRL_LOCK_SHIFT)) & CMC_CKCTRL_LOCK_MASK)
/*! @} */

/*! @name CKSTAT - Clock Status */
/*! @{ */

#define CMC_CKSTAT_CKMODE_MASK                   (0xFU)
#define CMC_CKSTAT_CKMODE_SHIFT                  (0U)
/*! CKMODE - Low Power Status
 *  0b0000..Core clock not gated
 *  0b1111..Core, platform, and peripheral clocks were gated, and power domain entered Low-Power mode
 */
#define CMC_CKSTAT_CKMODE(x)                     (((uint32_t)(((uint32_t)(x)) << CMC_CKSTAT_CKMODE_SHIFT)) & CMC_CKSTAT_CKMODE_MASK)

#define CMC_CKSTAT_WAKEUP_MASK                   (0xFF00U)
#define CMC_CKSTAT_WAKEUP_SHIFT                  (8U)
/*! WAKEUP - Wake-up Source */
#define CMC_CKSTAT_WAKEUP(x)                     (((uint32_t)(((uint32_t)(x)) << CMC_CKSTAT_WAKEUP_SHIFT)) & CMC_CKSTAT_WAKEUP_MASK)

#define CMC_CKSTAT_VALID_MASK                    (0x80000000U)
#define CMC_CKSTAT_VALID_SHIFT                   (31U)
/*! VALID - Clock Status Valid
 *  0b0..Core clock not gated
 *  0b1..Core clock was gated due to Low-Power mode entry
 */
#define CMC_CKSTAT_VALID(x)                      (((uint32_t)(((uint32_t)(x)) << CMC_CKSTAT_VALID_SHIFT)) & CMC_CKSTAT_VALID_MASK)
/*! @} */

/*! @name PMPROT - Power Mode Protection */
/*! @{ */

#define CMC_PMPROT_LPMODE_MASK                   (0xFU)
#define CMC_PMPROT_LPMODE_SHIFT                  (0U)
/*! LPMODE - Low-Power Mode
 *  0b0000..Not allowed
 *  0b0001..Allowed
 *  0b0010..Allowed
 *  0b0011..Allowed
 *  0b0100..Allowed
 *  0b0101..Allowed
 *  0b0110..Allowed
 *  0b0111..Allowed
 *  0b1000..Allowed
 *  0b1001..Allowed
 *  0b1010..Allowed
 *  0b1011..Allowed
 *  0b1100..Allowed
 *  0b1101..Allowed
 *  0b1110..Allowed
 *  0b1111..Allowed
 */
#define CMC_PMPROT_LPMODE(x)                     (((uint32_t)(((uint32_t)(x)) << CMC_PMPROT_LPMODE_SHIFT)) & CMC_PMPROT_LPMODE_MASK)

#define CMC_PMPROT_LOCK_MASK                     (0x80000000U)
#define CMC_PMPROT_LOCK_SHIFT                    (31U)
/*! LOCK - Lock Register
 *  0b0..Allowed
 *  0b1..Blocked
 */
#define CMC_PMPROT_LOCK(x)                       (((uint32_t)(((uint32_t)(x)) << CMC_PMPROT_LOCK_SHIFT)) & CMC_PMPROT_LOCK_MASK)
/*! @} */

/*! @name GPMCTRL - Global Power Mode Control */
/*! @{ */

#define CMC_GPMCTRL_LPMODE_MASK                  (0xFU)
#define CMC_GPMCTRL_LPMODE_SHIFT                 (0U)
/*! LPMODE - Low-Power Mode */
#define CMC_GPMCTRL_LPMODE(x)                    (((uint32_t)(((uint32_t)(x)) << CMC_GPMCTRL_LPMODE_SHIFT)) & CMC_GPMCTRL_LPMODE_MASK)
/*! @} */

/*! @name PMCTRL - Power Mode Control */
/*! @{ */

#define CMC_PMCTRL_LPMODE_MASK                   (0xFU)
#define CMC_PMCTRL_LPMODE_SHIFT                  (0U)
/*! LPMODE - Low-Power Mode
 *  0b0000..Active/Sleep
 *  0b0001..Deep Sleep
 *  0b0011..Power Down
 *  0b0111..Reserved
 *  0b1111..Deep-Power Down
 */
#define CMC_PMCTRL_LPMODE(x)                     (((uint32_t)(((uint32_t)(x)) << CMC_PMCTRL_LPMODE_SHIFT)) & CMC_PMCTRL_LPMODE_MASK)
/*! @} */

/*! @name SRS - System Reset Status */
/*! @{ */

#define CMC_SRS_WAKEUP_MASK                      (0x1U)
#define CMC_SRS_WAKEUP_SHIFT                     (0U)
/*! WAKEUP - Wake-up Reset
 *  0b0..Reset not generated
 *  0b1..Reset generated
 */
#define CMC_SRS_WAKEUP(x)                        (((uint32_t)(((uint32_t)(x)) << CMC_SRS_WAKEUP_SHIFT)) & CMC_SRS_WAKEUP_MASK)

#define CMC_SRS_POR_MASK                         (0x2U)
#define CMC_SRS_POR_SHIFT                        (1U)
/*! POR - Power-on Reset
 *  0b0..Reset not generated
 *  0b1..Reset generated
 */
#define CMC_SRS_POR(x)                           (((uint32_t)(((uint32_t)(x)) << CMC_SRS_POR_SHIFT)) & CMC_SRS_POR_MASK)

#define CMC_SRS_VD_MASK                          (0x4U)
#define CMC_SRS_VD_SHIFT                         (2U)
/*! VD - Voltage Detect Reset
 *  0b0..Reset not generated
 *  0b1..Reset generated
 */
#define CMC_SRS_VD(x)                            (((uint32_t)(((uint32_t)(x)) << CMC_SRS_VD_SHIFT)) & CMC_SRS_VD_MASK)

#define CMC_SRS_WARM_MASK                        (0x10U)
#define CMC_SRS_WARM_SHIFT                       (4U)
/*! WARM - Warm Reset
 *  0b0..Reset not generated
 *  0b1..Reset generated
 */
#define CMC_SRS_WARM(x)                          (((uint32_t)(((uint32_t)(x)) << CMC_SRS_WARM_SHIFT)) & CMC_SRS_WARM_MASK)

#define CMC_SRS_FATAL_MASK                       (0x20U)
#define CMC_SRS_FATAL_SHIFT                      (5U)
/*! FATAL - Fatal Reset
 *  0b0..Reset was not generated
 *  0b1..Reset was generated
 */
#define CMC_SRS_FATAL(x)                         (((uint32_t)(((uint32_t)(x)) << CMC_SRS_FATAL_SHIFT)) & CMC_SRS_FATAL_MASK)

#define CMC_SRS_PIN_MASK                         (0x100U)
#define CMC_SRS_PIN_SHIFT                        (8U)
/*! PIN - Pin Reset
 *  0b0..Reset was not generated
 *  0b1..Reset was generated
 */
#define CMC_SRS_PIN(x)                           (((uint32_t)(((uint32_t)(x)) << CMC_SRS_PIN_SHIFT)) & CMC_SRS_PIN_MASK)

#define CMC_SRS_DAP_MASK                         (0x200U)
#define CMC_SRS_DAP_SHIFT                        (9U)
/*! DAP - Debug Access Port Reset
 *  0b0..Reset was not generated
 *  0b1..Reset was generated
 */
#define CMC_SRS_DAP(x)                           (((uint32_t)(((uint32_t)(x)) << CMC_SRS_DAP_SHIFT)) & CMC_SRS_DAP_MASK)

#define CMC_SRS_RSTACK_MASK                      (0x400U)
#define CMC_SRS_RSTACK_SHIFT                     (10U)
/*! RSTACK - Reset Timeout
 *  0b0..Reset not generated
 *  0b1..Reset generated
 */
#define CMC_SRS_RSTACK(x)                        (((uint32_t)(((uint32_t)(x)) << CMC_SRS_RSTACK_SHIFT)) & CMC_SRS_RSTACK_MASK)

#define CMC_SRS_LPACK_MASK                       (0x800U)
#define CMC_SRS_LPACK_SHIFT                      (11U)
/*! LPACK - Low Power Acknowledge Timeout Reset
 *  0b0..Reset not generated
 *  0b1..Reset generated
 */
#define CMC_SRS_LPACK(x)                         (((uint32_t)(((uint32_t)(x)) << CMC_SRS_LPACK_SHIFT)) & CMC_SRS_LPACK_MASK)

#define CMC_SRS_SCG_MASK                         (0x1000U)
#define CMC_SRS_SCG_SHIFT                        (12U)
/*! SCG - System Clock Generation Reset
 *  0b0..Reset is not generated
 *  0b1..Reset is generated
 */
#define CMC_SRS_SCG(x)                           (((uint32_t)(((uint32_t)(x)) << CMC_SRS_SCG_SHIFT)) & CMC_SRS_SCG_MASK)

#define CMC_SRS_WWDT0_MASK                       (0x2000U)
#define CMC_SRS_WWDT0_SHIFT                      (13U)
/*! WWDT0 - Windowed Watchdog 0 Reset
 *  0b0..Reset is not generated
 *  0b1..Reset is generated
 */
#define CMC_SRS_WWDT0(x)                         (((uint32_t)(((uint32_t)(x)) << CMC_SRS_WWDT0_SHIFT)) & CMC_SRS_WWDT0_MASK)

#define CMC_SRS_SW_MASK                          (0x4000U)
#define CMC_SRS_SW_SHIFT                         (14U)
/*! SW - Software Reset
 *  0b0..Reset not generated
 *  0b1..Reset generated
 */
#define CMC_SRS_SW(x)                            (((uint32_t)(((uint32_t)(x)) << CMC_SRS_SW_SHIFT)) & CMC_SRS_SW_MASK)

#define CMC_SRS_LOCKUP_MASK                      (0x8000U)
#define CMC_SRS_LOCKUP_SHIFT                     (15U)
/*! LOCKUP - Lockup Reset
 *  0b0..Reset not generated
 *  0b1..Reset generated
 */
#define CMC_SRS_LOCKUP(x)                        (((uint32_t)(((uint32_t)(x)) << CMC_SRS_LOCKUP_SHIFT)) & CMC_SRS_LOCKUP_MASK)

#define CMC_SRS_CDOG0_MASK                       (0x4000000U)
#define CMC_SRS_CDOG0_SHIFT                      (26U)
/*! CDOG0 - Code Watchdog 0 Reset
 *  0b0..Reset is not generated
 *  0b1..Reset is generated
 */
#define CMC_SRS_CDOG0(x)                         (((uint32_t)(((uint32_t)(x)) << CMC_SRS_CDOG0_SHIFT)) & CMC_SRS_CDOG0_MASK)

#define CMC_SRS_JTAG_MASK                        (0x10000000U)
#define CMC_SRS_JTAG_SHIFT                       (28U)
/*! JTAG - JTAG System Reset
 *  0b0..Reset not generated
 *  0b1..Reset generated
 */
#define CMC_SRS_JTAG(x)                          (((uint32_t)(((uint32_t)(x)) << CMC_SRS_JTAG_SHIFT)) & CMC_SRS_JTAG_MASK)
/*! @} */

/*! @name RPC - Reset Pin Control */
/*! @{ */

#define CMC_RPC_FILTCFG_MASK                     (0x1FU)
#define CMC_RPC_FILTCFG_SHIFT                    (0U)
/*! FILTCFG - Reset Filter Configuration */
#define CMC_RPC_FILTCFG(x)                       (((uint32_t)(((uint32_t)(x)) << CMC_RPC_FILTCFG_SHIFT)) & CMC_RPC_FILTCFG_MASK)

#define CMC_RPC_FILTEN_MASK                      (0x100U)
#define CMC_RPC_FILTEN_SHIFT                     (8U)
/*! FILTEN - Filter Enable
 *  0b0..Disables
 *  0b1..Enables
 */
#define CMC_RPC_FILTEN(x)                        (((uint32_t)(((uint32_t)(x)) << CMC_RPC_FILTEN_SHIFT)) & CMC_RPC_FILTEN_MASK)

#define CMC_RPC_LPFEN_MASK                       (0x200U)
#define CMC_RPC_LPFEN_SHIFT                      (9U)
/*! LPFEN - Low-Power Filter Enable
 *  0b0..Disables
 *  0b1..Enables
 */
#define CMC_RPC_LPFEN(x)                         (((uint32_t)(((uint32_t)(x)) << CMC_RPC_LPFEN_SHIFT)) & CMC_RPC_LPFEN_MASK)
/*! @} */

/*! @name SSRS - Sticky System Reset Status */
/*! @{ */

#define CMC_SSRS_WAKEUP_MASK                     (0x1U)
#define CMC_SSRS_WAKEUP_SHIFT                    (0U)
/*! WAKEUP - Wake-up Reset
 *  0b0..Reset not generated
 *  0b1..Reset generated
 */
#define CMC_SSRS_WAKEUP(x)                       (((uint32_t)(((uint32_t)(x)) << CMC_SSRS_WAKEUP_SHIFT)) & CMC_SSRS_WAKEUP_MASK)

#define CMC_SSRS_POR_MASK                        (0x2U)
#define CMC_SSRS_POR_SHIFT                       (1U)
/*! POR - Power-on Reset
 *  0b0..Reset not generated
 *  0b1..Reset generated
 */
#define CMC_SSRS_POR(x)                          (((uint32_t)(((uint32_t)(x)) << CMC_SSRS_POR_SHIFT)) & CMC_SSRS_POR_MASK)

#define CMC_SSRS_VD_MASK                         (0x4U)
#define CMC_SSRS_VD_SHIFT                        (2U)
/*! VD - Voltage Detect Reset
 *  0b0..Reset not generated
 *  0b1..Reset generated
 */
#define CMC_SSRS_VD(x)                           (((uint32_t)(((uint32_t)(x)) << CMC_SSRS_VD_SHIFT)) & CMC_SSRS_VD_MASK)

#define CMC_SSRS_WARM_MASK                       (0x10U)
#define CMC_SSRS_WARM_SHIFT                      (4U)
/*! WARM - Warm Reset
 *  0b0..Reset not generated
 *  0b1..Reset generated
 */
#define CMC_SSRS_WARM(x)                         (((uint32_t)(((uint32_t)(x)) << CMC_SSRS_WARM_SHIFT)) & CMC_SSRS_WARM_MASK)

#define CMC_SSRS_FATAL_MASK                      (0x20U)
#define CMC_SSRS_FATAL_SHIFT                     (5U)
/*! FATAL - Fatal Reset
 *  0b0..Reset was not generated
 *  0b1..Reset was generated
 */
#define CMC_SSRS_FATAL(x)                        (((uint32_t)(((uint32_t)(x)) << CMC_SSRS_FATAL_SHIFT)) & CMC_SSRS_FATAL_MASK)

#define CMC_SSRS_PIN_MASK                        (0x100U)
#define CMC_SSRS_PIN_SHIFT                       (8U)
/*! PIN - Pin Reset
 *  0b0..Reset not generated
 *  0b1..Reset generated
 */
#define CMC_SSRS_PIN(x)                          (((uint32_t)(((uint32_t)(x)) << CMC_SSRS_PIN_SHIFT)) & CMC_SSRS_PIN_MASK)

#define CMC_SSRS_DAP_MASK                        (0x200U)
#define CMC_SSRS_DAP_SHIFT                       (9U)
/*! DAP - DAP Reset
 *  0b0..Reset not generated
 *  0b1..Reset generated
 */
#define CMC_SSRS_DAP(x)                          (((uint32_t)(((uint32_t)(x)) << CMC_SSRS_DAP_SHIFT)) & CMC_SSRS_DAP_MASK)

#define CMC_SSRS_RSTACK_MASK                     (0x400U)
#define CMC_SSRS_RSTACK_SHIFT                    (10U)
/*! RSTACK - Reset Timeout
 *  0b0..Reset not generated
 *  0b1..Reset generated
 */
#define CMC_SSRS_RSTACK(x)                       (((uint32_t)(((uint32_t)(x)) << CMC_SSRS_RSTACK_SHIFT)) & CMC_SSRS_RSTACK_MASK)

#define CMC_SSRS_LPACK_MASK                      (0x800U)
#define CMC_SSRS_LPACK_SHIFT                     (11U)
/*! LPACK - Low Power Acknowledge Timeout Reset
 *  0b0..Reset not generated
 *  0b1..Reset generated
 */
#define CMC_SSRS_LPACK(x)                        (((uint32_t)(((uint32_t)(x)) << CMC_SSRS_LPACK_SHIFT)) & CMC_SSRS_LPACK_MASK)

#define CMC_SSRS_SCG_MASK                        (0x1000U)
#define CMC_SSRS_SCG_SHIFT                       (12U)
/*! SCG - System Clock Generation Reset
 *  0b0..Reset is not generated
 *  0b1..Reset is generated
 */
#define CMC_SSRS_SCG(x)                          (((uint32_t)(((uint32_t)(x)) << CMC_SSRS_SCG_SHIFT)) & CMC_SSRS_SCG_MASK)

#define CMC_SSRS_WWDT0_MASK                      (0x2000U)
#define CMC_SSRS_WWDT0_SHIFT                     (13U)
/*! WWDT0 - Windowed Watchdog 0 Reset
 *  0b0..Reset is not generated
 *  0b1..Reset is generated
 */
#define CMC_SSRS_WWDT0(x)                        (((uint32_t)(((uint32_t)(x)) << CMC_SSRS_WWDT0_SHIFT)) & CMC_SSRS_WWDT0_MASK)

#define CMC_SSRS_SW_MASK                         (0x4000U)
#define CMC_SSRS_SW_SHIFT                        (14U)
/*! SW - Software Reset
 *  0b0..Reset not generated
 *  0b1..Reset generated
 */
#define CMC_SSRS_SW(x)                           (((uint32_t)(((uint32_t)(x)) << CMC_SSRS_SW_SHIFT)) & CMC_SSRS_SW_MASK)

#define CMC_SSRS_LOCKUP_MASK                     (0x8000U)
#define CMC_SSRS_LOCKUP_SHIFT                    (15U)
/*! LOCKUP - Lockup Reset
 *  0b0..Reset not generated
 *  0b1..Reset generated
 */
#define CMC_SSRS_LOCKUP(x)                       (((uint32_t)(((uint32_t)(x)) << CMC_SSRS_LOCKUP_SHIFT)) & CMC_SSRS_LOCKUP_MASK)

#define CMC_SSRS_CDOG0_MASK                      (0x4000000U)
#define CMC_SSRS_CDOG0_SHIFT                     (26U)
/*! CDOG0 - Code Watchdog 0 Reset
 *  0b0..Reset is not generated
 *  0b1..Reset is generated
 */
#define CMC_SSRS_CDOG0(x)                        (((uint32_t)(((uint32_t)(x)) << CMC_SSRS_CDOG0_SHIFT)) & CMC_SSRS_CDOG0_MASK)

#define CMC_SSRS_JTAG_MASK                       (0x10000000U)
#define CMC_SSRS_JTAG_SHIFT                      (28U)
/*! JTAG - JTAG System Reset
 *  0b0..Reset not generated
 *  0b1..Reset generated
 */
#define CMC_SSRS_JTAG(x)                         (((uint32_t)(((uint32_t)(x)) << CMC_SSRS_JTAG_SHIFT)) & CMC_SSRS_JTAG_MASK)
/*! @} */

/*! @name SRIE - System Reset Interrupt Enable */
/*! @{ */

#define CMC_SRIE_PIN_MASK                        (0x100U)
#define CMC_SRIE_PIN_SHIFT                       (8U)
/*! PIN - Pin Reset
 *  0b0..Interrupt disabled
 *  0b1..Interrupt enabled
 */
#define CMC_SRIE_PIN(x)                          (((uint32_t)(((uint32_t)(x)) << CMC_SRIE_PIN_SHIFT)) & CMC_SRIE_PIN_MASK)

#define CMC_SRIE_DAP_MASK                        (0x200U)
#define CMC_SRIE_DAP_SHIFT                       (9U)
/*! DAP - DAP Reset
 *  0b0..Interrupt disabled
 *  0b1..Interrupt enabled
 */
#define CMC_SRIE_DAP(x)                          (((uint32_t)(((uint32_t)(x)) << CMC_SRIE_DAP_SHIFT)) & CMC_SRIE_DAP_MASK)

#define CMC_SRIE_LPACK_MASK                      (0x800U)
#define CMC_SRIE_LPACK_SHIFT                     (11U)
/*! LPACK - Low Power Acknowledge Timeout Reset
 *  0b0..Interrupt disabled
 *  0b1..Interrupt enabled
 */
#define CMC_SRIE_LPACK(x)                        (((uint32_t)(((uint32_t)(x)) << CMC_SRIE_LPACK_SHIFT)) & CMC_SRIE_LPACK_MASK)

#define CMC_SRIE_SCG_MASK                        (0x1000U)
#define CMC_SRIE_SCG_SHIFT                       (12U)
/*! SCG - System Clock Generation Reset
 *  0b0..Interrupt disabled
 *  0b1..Interrupt enabled
 */
#define CMC_SRIE_SCG(x)                          (((uint32_t)(((uint32_t)(x)) << CMC_SRIE_SCG_SHIFT)) & CMC_SRIE_SCG_MASK)

#define CMC_SRIE_WWDT0_MASK                      (0x2000U)
#define CMC_SRIE_WWDT0_SHIFT                     (13U)
/*! WWDT0 - Windowed Watchdog 0 Reset
 *  0b0..Interrupt disabled
 *  0b1..Interrupt enabled
 */
#define CMC_SRIE_WWDT0(x)                        (((uint32_t)(((uint32_t)(x)) << CMC_SRIE_WWDT0_SHIFT)) & CMC_SRIE_WWDT0_MASK)

#define CMC_SRIE_SW_MASK                         (0x4000U)
#define CMC_SRIE_SW_SHIFT                        (14U)
/*! SW - Software Reset
 *  0b0..Interrupt disabled
 *  0b1..Interrupt enabled
 */
#define CMC_SRIE_SW(x)                           (((uint32_t)(((uint32_t)(x)) << CMC_SRIE_SW_SHIFT)) & CMC_SRIE_SW_MASK)

#define CMC_SRIE_LOCKUP_MASK                     (0x8000U)
#define CMC_SRIE_LOCKUP_SHIFT                    (15U)
/*! LOCKUP - Lockup Reset
 *  0b0..Interrupt disabled
 *  0b1..Interrupt enabled
 */
#define CMC_SRIE_LOCKUP(x)                       (((uint32_t)(((uint32_t)(x)) << CMC_SRIE_LOCKUP_SHIFT)) & CMC_SRIE_LOCKUP_MASK)

#define CMC_SRIE_CDOG0_MASK                      (0x4000000U)
#define CMC_SRIE_CDOG0_SHIFT                     (26U)
/*! CDOG0 - Code Watchdog 0 Reset
 *  0b0..Interrupt disabled
 *  0b1..Interrupt enabled
 */
#define CMC_SRIE_CDOG0(x)                        (((uint32_t)(((uint32_t)(x)) << CMC_SRIE_CDOG0_SHIFT)) & CMC_SRIE_CDOG0_MASK)
/*! @} */

/*! @name SRIF - System Reset Interrupt Flag */
/*! @{ */

#define CMC_SRIF_PIN_MASK                        (0x100U)
#define CMC_SRIF_PIN_SHIFT                       (8U)
/*! PIN - Pin Reset
 *  0b0..Reset source not pending
 *  0b1..Reset source pending
 */
#define CMC_SRIF_PIN(x)                          (((uint32_t)(((uint32_t)(x)) << CMC_SRIF_PIN_SHIFT)) & CMC_SRIF_PIN_MASK)

#define CMC_SRIF_DAP_MASK                        (0x200U)
#define CMC_SRIF_DAP_SHIFT                       (9U)
/*! DAP - DAP Reset
 *  0b0..Reset source not pending
 *  0b1..Reset source pending
 */
#define CMC_SRIF_DAP(x)                          (((uint32_t)(((uint32_t)(x)) << CMC_SRIF_DAP_SHIFT)) & CMC_SRIF_DAP_MASK)

#define CMC_SRIF_LPACK_MASK                      (0x800U)
#define CMC_SRIF_LPACK_SHIFT                     (11U)
/*! LPACK - Low Power Acknowledge Timeout Reset
 *  0b0..Reset source not pending
 *  0b1..Reset source pending
 */
#define CMC_SRIF_LPACK(x)                        (((uint32_t)(((uint32_t)(x)) << CMC_SRIF_LPACK_SHIFT)) & CMC_SRIF_LPACK_MASK)

#define CMC_SRIF_WWDT0_MASK                      (0x2000U)
#define CMC_SRIF_WWDT0_SHIFT                     (13U)
/*! WWDT0 - Windowed Watchdog 0 Reset
 *  0b0..Reset source not pending
 *  0b1..Reset source pending
 */
#define CMC_SRIF_WWDT0(x)                        (((uint32_t)(((uint32_t)(x)) << CMC_SRIF_WWDT0_SHIFT)) & CMC_SRIF_WWDT0_MASK)

#define CMC_SRIF_SW_MASK                         (0x4000U)
#define CMC_SRIF_SW_SHIFT                        (14U)
/*! SW - Software Reset
 *  0b0..Reset source not pending
 *  0b1..Reset source pending
 */
#define CMC_SRIF_SW(x)                           (((uint32_t)(((uint32_t)(x)) << CMC_SRIF_SW_SHIFT)) & CMC_SRIF_SW_MASK)

#define CMC_SRIF_LOCKUP_MASK                     (0x8000U)
#define CMC_SRIF_LOCKUP_SHIFT                    (15U)
/*! LOCKUP - Lockup Reset
 *  0b0..Reset source not pending
 *  0b1..Reset source pending
 */
#define CMC_SRIF_LOCKUP(x)                       (((uint32_t)(((uint32_t)(x)) << CMC_SRIF_LOCKUP_SHIFT)) & CMC_SRIF_LOCKUP_MASK)

#define CMC_SRIF_CDOG0_MASK                      (0x4000000U)
#define CMC_SRIF_CDOG0_SHIFT                     (26U)
/*! CDOG0 - Code Watchdog 0 Reset
 *  0b0..Reset source not pending
 *  0b1..Reset source pending
 */
#define CMC_SRIF_CDOG0(x)                        (((uint32_t)(((uint32_t)(x)) << CMC_SRIF_CDOG0_SHIFT)) & CMC_SRIF_CDOG0_MASK)
/*! @} */

/*! @name RSTCNT - Reset Count Register */
/*! @{ */

#define CMC_RSTCNT_COUNT_MASK                    (0xFFU)
#define CMC_RSTCNT_COUNT_SHIFT                   (0U)
/*! COUNT - Count */
#define CMC_RSTCNT_COUNT(x)                      (((uint32_t)(((uint32_t)(x)) << CMC_RSTCNT_COUNT_SHIFT)) & CMC_RSTCNT_COUNT_MASK)
/*! @} */

/*! @name MR - Mode */
/*! @{ */

#define CMC_MR_ISPMODE_n_MASK                    (0x1U)
#define CMC_MR_ISPMODE_n_SHIFT                   (0U)
/*! ISPMODE_n - In System Programming Mode */
#define CMC_MR_ISPMODE_n(x)                      (((uint32_t)(((uint32_t)(x)) << CMC_MR_ISPMODE_n_SHIFT)) & CMC_MR_ISPMODE_n_MASK)
/*! @} */

/*! @name FM - Force Mode */
/*! @{ */

#define CMC_FM_FORCECFG_MASK                     (0x1U)
#define CMC_FM_FORCECFG_SHIFT                    (0U)
/*! FORCECFG - Boot Configuration
 *  0b0..No effect
 *  0b1..Asserts
 */
#define CMC_FM_FORCECFG(x)                       (((uint32_t)(((uint32_t)(x)) << CMC_FM_FORCECFG_SHIFT)) & CMC_FM_FORCECFG_MASK)
/*! @} */

/*! @name FLASHCR - Flash Control */
/*! @{ */

#define CMC_FLASHCR_FLASHDIS_MASK                (0x1U)
#define CMC_FLASHCR_FLASHDIS_SHIFT               (0U)
/*! FLASHDIS - Flash Disable
 *  0b0..No effect
 *  0b1..Flash memory is disabled
 */
#define CMC_FLASHCR_FLASHDIS(x)                  (((uint32_t)(((uint32_t)(x)) << CMC_FLASHCR_FLASHDIS_SHIFT)) & CMC_FLASHCR_FLASHDIS_MASK)

#define CMC_FLASHCR_FLASHDOZE_MASK               (0x2U)
#define CMC_FLASHCR_FLASHDOZE_SHIFT              (1U)
/*! FLASHDOZE - Flash Doze
 *  0b0..No effect
 *  0b1..Flash memory is disabled when core is sleeping (CKMODE > 0)
 */
#define CMC_FLASHCR_FLASHDOZE(x)                 (((uint32_t)(((uint32_t)(x)) << CMC_FLASHCR_FLASHDOZE_SHIFT)) & CMC_FLASHCR_FLASHDOZE_MASK)

#define CMC_FLASHCR_FLASHWAKE_MASK               (0x4U)
#define CMC_FLASHCR_FLASHWAKE_SHIFT              (2U)
/*! FLASHWAKE - Flash Wake
 *  0b0..No effect
 *  0b1..Flash memory is not disabled during flash memory accesses
 */
#define CMC_FLASHCR_FLASHWAKE(x)                 (((uint32_t)(((uint32_t)(x)) << CMC_FLASHCR_FLASHWAKE_SHIFT)) & CMC_FLASHCR_FLASHWAKE_MASK)
/*! @} */

/*! @name CORECTL - Core Control */
/*! @{ */

#define CMC_CORECTL_NPIE_MASK                    (0x1U)
#define CMC_CORECTL_NPIE_SHIFT                   (0U)
/*! NPIE - Non-maskable Pin Interrupt Enable
 *  0b0..Disables
 *  0b1..Enables
 */
#define CMC_CORECTL_NPIE(x)                      (((uint32_t)(((uint32_t)(x)) << CMC_CORECTL_NPIE_SHIFT)) & CMC_CORECTL_NPIE_MASK)
/*! @} */

/*! @name DBGCTL - Debug Control */
/*! @{ */

#define CMC_DBGCTL_SOD_MASK                      (0x1U)
#define CMC_DBGCTL_SOD_SHIFT                     (0U)
/*! SOD - Sleep Or Debug
 *  0b0..Remains enabled
 *  0b1..Disabled
 */
#define CMC_DBGCTL_SOD(x)                        (((uint32_t)(((uint32_t)(x)) << CMC_DBGCTL_SOD_SHIFT)) & CMC_DBGCTL_SOD_MASK)
/*! @} */


/*!
 * @}
 */ /* end of group CMC_Register_Masks */


/*!
 * @}
 */ /* end of group CMC_Peripheral_Access_Layer */


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


#endif  /* PERI_CMC_H_ */

