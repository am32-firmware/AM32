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
**         CMSIS Peripheral Access Layer for CTIMER
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
 * @file PERI_CTIMER.h
 * @version 2.0
 * @date 2024-10-29
 * @brief CMSIS Peripheral Access Layer for CTIMER
 *
 * CMSIS Peripheral Access Layer for CTIMER
 */

#if !defined(PERI_CTIMER_H_)
#define PERI_CTIMER_H_                           /**< Symbol preventing repeated inclusion */

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
   -- CTIMER Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CTIMER_Peripheral_Access_Layer CTIMER Peripheral Access Layer
 * @{
 */

/** CTIMER - Size of Registers Arrays */
#define CTIMER_MR_COUNT                           4u
#define CTIMER_CR_COUNT                           4u
#define CTIMER_MSR_COUNT                          4u

/** CTIMER - Register Layout Typedef */
typedef struct {
  __IO uint32_t IR;                                /**< Interrupt, offset: 0x0 */
  __IO uint32_t TCR;                               /**< Timer Control, offset: 0x4 */
  __IO uint32_t TC;                                /**< Timer Counter, offset: 0x8 */
  __IO uint32_t PR;                                /**< Prescale, offset: 0xC */
  __IO uint32_t PC;                                /**< Prescale Counter, offset: 0x10 */
  __IO uint32_t MCR;                               /**< Match Control, offset: 0x14 */
  __IO uint32_t MR[CTIMER_MR_COUNT];               /**< Match, array offset: 0x18, array step: 0x4 */
  __IO uint32_t CCR;                               /**< Capture Control, offset: 0x28 */
  __I  uint32_t CR[CTIMER_CR_COUNT];               /**< Capture, array offset: 0x2C, array step: 0x4 */
  __IO uint32_t EMR;                               /**< External Match, offset: 0x3C */
       uint8_t RESERVED_0[48];
  __IO uint32_t CTCR;                              /**< Count Control, offset: 0x70 */
  __IO uint32_t PWMC;                              /**< PWM Control, offset: 0x74 */
  __IO uint32_t MSR[CTIMER_MSR_COUNT];             /**< Match Shadow, array offset: 0x78, array step: 0x4 */
} CTIMER_Type;

/* ----------------------------------------------------------------------------
   -- CTIMER Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CTIMER_Register_Masks CTIMER Register Masks
 * @{
 */

/*! @name IR - Interrupt */
/*! @{ */

#define CTIMER_IR_MR0INT_MASK                    (0x1U)
#define CTIMER_IR_MR0INT_SHIFT                   (0U)
/*! MR0INT - Interrupt Flag for Match Channel 0 Event */
#define CTIMER_IR_MR0INT(x)                      (((uint32_t)(((uint32_t)(x)) << CTIMER_IR_MR0INT_SHIFT)) & CTIMER_IR_MR0INT_MASK)

#define CTIMER_IR_MR1INT_MASK                    (0x2U)
#define CTIMER_IR_MR1INT_SHIFT                   (1U)
/*! MR1INT - Interrupt Flag for Match Channel 1 Event */
#define CTIMER_IR_MR1INT(x)                      (((uint32_t)(((uint32_t)(x)) << CTIMER_IR_MR1INT_SHIFT)) & CTIMER_IR_MR1INT_MASK)

#define CTIMER_IR_MR2INT_MASK                    (0x4U)
#define CTIMER_IR_MR2INT_SHIFT                   (2U)
/*! MR2INT - Interrupt Flag for Match Channel 2 Event */
#define CTIMER_IR_MR2INT(x)                      (((uint32_t)(((uint32_t)(x)) << CTIMER_IR_MR2INT_SHIFT)) & CTIMER_IR_MR2INT_MASK)

#define CTIMER_IR_MR3INT_MASK                    (0x8U)
#define CTIMER_IR_MR3INT_SHIFT                   (3U)
/*! MR3INT - Interrupt Flag for Match Channel 3 Event */
#define CTIMER_IR_MR3INT(x)                      (((uint32_t)(((uint32_t)(x)) << CTIMER_IR_MR3INT_SHIFT)) & CTIMER_IR_MR3INT_MASK)

#define CTIMER_IR_CR0INT_MASK                    (0x10U)
#define CTIMER_IR_CR0INT_SHIFT                   (4U)
/*! CR0INT - Interrupt Flag for Capture Channel 0 Event */
#define CTIMER_IR_CR0INT(x)                      (((uint32_t)(((uint32_t)(x)) << CTIMER_IR_CR0INT_SHIFT)) & CTIMER_IR_CR0INT_MASK)

#define CTIMER_IR_CR1INT_MASK                    (0x20U)
#define CTIMER_IR_CR1INT_SHIFT                   (5U)
/*! CR1INT - Interrupt Flag for Capture Channel 1 Event */
#define CTIMER_IR_CR1INT(x)                      (((uint32_t)(((uint32_t)(x)) << CTIMER_IR_CR1INT_SHIFT)) & CTIMER_IR_CR1INT_MASK)

#define CTIMER_IR_CR2INT_MASK                    (0x40U)
#define CTIMER_IR_CR2INT_SHIFT                   (6U)
/*! CR2INT - Interrupt Flag for Capture Channel 2 Event */
#define CTIMER_IR_CR2INT(x)                      (((uint32_t)(((uint32_t)(x)) << CTIMER_IR_CR2INT_SHIFT)) & CTIMER_IR_CR2INT_MASK)

#define CTIMER_IR_CR3INT_MASK                    (0x80U)
#define CTIMER_IR_CR3INT_SHIFT                   (7U)
/*! CR3INT - Interrupt Flag for Capture Channel 3 Event */
#define CTIMER_IR_CR3INT(x)                      (((uint32_t)(((uint32_t)(x)) << CTIMER_IR_CR3INT_SHIFT)) & CTIMER_IR_CR3INT_MASK)
/*! @} */

/*! @name TCR - Timer Control */
/*! @{ */

#define CTIMER_TCR_CEN_MASK                      (0x1U)
#define CTIMER_TCR_CEN_SHIFT                     (0U)
/*! CEN - Counter Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define CTIMER_TCR_CEN(x)                        (((uint32_t)(((uint32_t)(x)) << CTIMER_TCR_CEN_SHIFT)) & CTIMER_TCR_CEN_MASK)

#define CTIMER_TCR_CRST_MASK                     (0x2U)
#define CTIMER_TCR_CRST_SHIFT                    (1U)
/*! CRST - Counter Reset Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define CTIMER_TCR_CRST(x)                       (((uint32_t)(((uint32_t)(x)) << CTIMER_TCR_CRST_SHIFT)) & CTIMER_TCR_CRST_MASK)

#define CTIMER_TCR_AGCEN_MASK                    (0x10U)
#define CTIMER_TCR_AGCEN_SHIFT                   (4U)
/*! AGCEN - Allow Global Count Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define CTIMER_TCR_AGCEN(x)                      (((uint32_t)(((uint32_t)(x)) << CTIMER_TCR_AGCEN_SHIFT)) & CTIMER_TCR_AGCEN_MASK)

#define CTIMER_TCR_ATCEN_MASK                    (0x20U)
#define CTIMER_TCR_ATCEN_SHIFT                   (5U)
/*! ATCEN - Allow Trigger Count Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define CTIMER_TCR_ATCEN(x)                      (((uint32_t)(((uint32_t)(x)) << CTIMER_TCR_ATCEN_SHIFT)) & CTIMER_TCR_ATCEN_MASK)
/*! @} */

/*! @name TC - Timer Counter */
/*! @{ */

#define CTIMER_TC_TCVAL_MASK                     (0xFFFFFFFFU)
#define CTIMER_TC_TCVAL_SHIFT                    (0U)
/*! TCVAL - Timer Counter Value */
#define CTIMER_TC_TCVAL(x)                       (((uint32_t)(((uint32_t)(x)) << CTIMER_TC_TCVAL_SHIFT)) & CTIMER_TC_TCVAL_MASK)
/*! @} */

/*! @name PR - Prescale */
/*! @{ */

#define CTIMER_PR_PRVAL_MASK                     (0xFFFFFFFFU)
#define CTIMER_PR_PRVAL_SHIFT                    (0U)
/*! PRVAL - Prescale Reload Value */
#define CTIMER_PR_PRVAL(x)                       (((uint32_t)(((uint32_t)(x)) << CTIMER_PR_PRVAL_SHIFT)) & CTIMER_PR_PRVAL_MASK)
/*! @} */

/*! @name PC - Prescale Counter */
/*! @{ */

#define CTIMER_PC_PCVAL_MASK                     (0xFFFFFFFFU)
#define CTIMER_PC_PCVAL_SHIFT                    (0U)
/*! PCVAL - Prescale Counter Value */
#define CTIMER_PC_PCVAL(x)                       (((uint32_t)(((uint32_t)(x)) << CTIMER_PC_PCVAL_SHIFT)) & CTIMER_PC_PCVAL_MASK)
/*! @} */

/*! @name MCR - Match Control */
/*! @{ */

#define CTIMER_MCR_MR0I_MASK                     (0x1U)
#define CTIMER_MCR_MR0I_SHIFT                    (0U)
/*! MR0I - Interrupt on MR0
 *  0b0..Does not generate
 *  0b1..Generates
 */
#define CTIMER_MCR_MR0I(x)                       (((uint32_t)(((uint32_t)(x)) << CTIMER_MCR_MR0I_SHIFT)) & CTIMER_MCR_MR0I_MASK)

#define CTIMER_MCR_MR0R_MASK                     (0x2U)
#define CTIMER_MCR_MR0R_SHIFT                    (1U)
/*! MR0R - Reset on MR0
 *  0b0..Does not reset
 *  0b1..Resets
 */
#define CTIMER_MCR_MR0R(x)                       (((uint32_t)(((uint32_t)(x)) << CTIMER_MCR_MR0R_SHIFT)) & CTIMER_MCR_MR0R_MASK)

#define CTIMER_MCR_MR0S_MASK                     (0x4U)
#define CTIMER_MCR_MR0S_SHIFT                    (2U)
/*! MR0S - Stop on MR0
 *  0b0..Does not stop
 *  0b1..Stops
 */
#define CTIMER_MCR_MR0S(x)                       (((uint32_t)(((uint32_t)(x)) << CTIMER_MCR_MR0S_SHIFT)) & CTIMER_MCR_MR0S_MASK)

#define CTIMER_MCR_MR1I_MASK                     (0x8U)
#define CTIMER_MCR_MR1I_SHIFT                    (3U)
/*! MR1I - Interrupt on MR1
 *  0b0..Does not generate
 *  0b1..Generates
 */
#define CTIMER_MCR_MR1I(x)                       (((uint32_t)(((uint32_t)(x)) << CTIMER_MCR_MR1I_SHIFT)) & CTIMER_MCR_MR1I_MASK)

#define CTIMER_MCR_MR1R_MASK                     (0x10U)
#define CTIMER_MCR_MR1R_SHIFT                    (4U)
/*! MR1R - Reset on MR1
 *  0b0..Does not reset
 *  0b1..Resets
 */
#define CTIMER_MCR_MR1R(x)                       (((uint32_t)(((uint32_t)(x)) << CTIMER_MCR_MR1R_SHIFT)) & CTIMER_MCR_MR1R_MASK)

#define CTIMER_MCR_MR1S_MASK                     (0x20U)
#define CTIMER_MCR_MR1S_SHIFT                    (5U)
/*! MR1S - Stop on MR1
 *  0b0..Does not stop
 *  0b1..Stops
 */
#define CTIMER_MCR_MR1S(x)                       (((uint32_t)(((uint32_t)(x)) << CTIMER_MCR_MR1S_SHIFT)) & CTIMER_MCR_MR1S_MASK)

#define CTIMER_MCR_MR2I_MASK                     (0x40U)
#define CTIMER_MCR_MR2I_SHIFT                    (6U)
/*! MR2I - Interrupt on MR2
 *  0b0..Does not generate
 *  0b1..Generates
 */
#define CTIMER_MCR_MR2I(x)                       (((uint32_t)(((uint32_t)(x)) << CTIMER_MCR_MR2I_SHIFT)) & CTIMER_MCR_MR2I_MASK)

#define CTIMER_MCR_MR2R_MASK                     (0x80U)
#define CTIMER_MCR_MR2R_SHIFT                    (7U)
/*! MR2R - Reset on MR2
 *  0b0..Does not reset
 *  0b1..Resets
 */
#define CTIMER_MCR_MR2R(x)                       (((uint32_t)(((uint32_t)(x)) << CTIMER_MCR_MR2R_SHIFT)) & CTIMER_MCR_MR2R_MASK)

#define CTIMER_MCR_MR2S_MASK                     (0x100U)
#define CTIMER_MCR_MR2S_SHIFT                    (8U)
/*! MR2S - Stop on MR2
 *  0b0..Does not stop
 *  0b1..Stops
 */
#define CTIMER_MCR_MR2S(x)                       (((uint32_t)(((uint32_t)(x)) << CTIMER_MCR_MR2S_SHIFT)) & CTIMER_MCR_MR2S_MASK)

#define CTIMER_MCR_MR3I_MASK                     (0x200U)
#define CTIMER_MCR_MR3I_SHIFT                    (9U)
/*! MR3I - Interrupt on MR3
 *  0b0..Does not generate
 *  0b1..Generates
 */
#define CTIMER_MCR_MR3I(x)                       (((uint32_t)(((uint32_t)(x)) << CTIMER_MCR_MR3I_SHIFT)) & CTIMER_MCR_MR3I_MASK)

#define CTIMER_MCR_MR3R_MASK                     (0x400U)
#define CTIMER_MCR_MR3R_SHIFT                    (10U)
/*! MR3R - Reset on MR3
 *  0b0..Does not reset
 *  0b1..Resets
 */
#define CTIMER_MCR_MR3R(x)                       (((uint32_t)(((uint32_t)(x)) << CTIMER_MCR_MR3R_SHIFT)) & CTIMER_MCR_MR3R_MASK)

#define CTIMER_MCR_MR3S_MASK                     (0x800U)
#define CTIMER_MCR_MR3S_SHIFT                    (11U)
/*! MR3S - Stop on MR3
 *  0b0..Does not stop
 *  0b1..Stops
 */
#define CTIMER_MCR_MR3S(x)                       (((uint32_t)(((uint32_t)(x)) << CTIMER_MCR_MR3S_SHIFT)) & CTIMER_MCR_MR3S_MASK)

#define CTIMER_MCR_MR0RL_MASK                    (0x1000000U)
#define CTIMER_MCR_MR0RL_SHIFT                   (24U)
/*! MR0RL - Reload MR
 *  0b0..Does not reload
 *  0b1..Reloads
 */
#define CTIMER_MCR_MR0RL(x)                      (((uint32_t)(((uint32_t)(x)) << CTIMER_MCR_MR0RL_SHIFT)) & CTIMER_MCR_MR0RL_MASK)

#define CTIMER_MCR_MR1RL_MASK                    (0x2000000U)
#define CTIMER_MCR_MR1RL_SHIFT                   (25U)
/*! MR1RL - Reload MR
 *  0b0..Does not reload
 *  0b1..Reloads
 */
#define CTIMER_MCR_MR1RL(x)                      (((uint32_t)(((uint32_t)(x)) << CTIMER_MCR_MR1RL_SHIFT)) & CTIMER_MCR_MR1RL_MASK)

#define CTIMER_MCR_MR2RL_MASK                    (0x4000000U)
#define CTIMER_MCR_MR2RL_SHIFT                   (26U)
/*! MR2RL - Reload MR
 *  0b0..Does not reload
 *  0b1..Reloads
 */
#define CTIMER_MCR_MR2RL(x)                      (((uint32_t)(((uint32_t)(x)) << CTIMER_MCR_MR2RL_SHIFT)) & CTIMER_MCR_MR2RL_MASK)

#define CTIMER_MCR_MR3RL_MASK                    (0x8000000U)
#define CTIMER_MCR_MR3RL_SHIFT                   (27U)
/*! MR3RL - Reload MR
 *  0b0..Does not reload
 *  0b1..Reloads
 */
#define CTIMER_MCR_MR3RL(x)                      (((uint32_t)(((uint32_t)(x)) << CTIMER_MCR_MR3RL_SHIFT)) & CTIMER_MCR_MR3RL_MASK)
/*! @} */

/*! @name MR - Match */
/*! @{ */

#define CTIMER_MR_MATCH_MASK                     (0xFFFFFFFFU)
#define CTIMER_MR_MATCH_SHIFT                    (0U)
/*! MATCH - Timer Counter Match Value */
#define CTIMER_MR_MATCH(x)                       (((uint32_t)(((uint32_t)(x)) << CTIMER_MR_MATCH_SHIFT)) & CTIMER_MR_MATCH_MASK)
/*! @} */

/*! @name CCR - Capture Control */
/*! @{ */

#define CTIMER_CCR_CAP0RE_MASK                   (0x1U)
#define CTIMER_CCR_CAP0RE_SHIFT                  (0U)
/*! CAP0RE - Rising Edge of Capture Channel 0
 *  0b0..Does not load
 *  0b1..Loads
 */
#define CTIMER_CCR_CAP0RE(x)                     (((uint32_t)(((uint32_t)(x)) << CTIMER_CCR_CAP0RE_SHIFT)) & CTIMER_CCR_CAP0RE_MASK)

#define CTIMER_CCR_CAP0FE_MASK                   (0x2U)
#define CTIMER_CCR_CAP0FE_SHIFT                  (1U)
/*! CAP0FE - Falling Edge of Capture Channel 0
 *  0b0..Does not load
 *  0b1..Loads
 */
#define CTIMER_CCR_CAP0FE(x)                     (((uint32_t)(((uint32_t)(x)) << CTIMER_CCR_CAP0FE_SHIFT)) & CTIMER_CCR_CAP0FE_MASK)

#define CTIMER_CCR_CAP0I_MASK                    (0x4U)
#define CTIMER_CCR_CAP0I_SHIFT                   (2U)
/*! CAP0I - Generate Interrupt on Channel 0 Capture Event
 *  0b0..Does not generate
 *  0b1..Generates
 */
#define CTIMER_CCR_CAP0I(x)                      (((uint32_t)(((uint32_t)(x)) << CTIMER_CCR_CAP0I_SHIFT)) & CTIMER_CCR_CAP0I_MASK)

#define CTIMER_CCR_CAP1RE_MASK                   (0x8U)
#define CTIMER_CCR_CAP1RE_SHIFT                  (3U)
/*! CAP1RE - Rising Edge of Capture Channel 1
 *  0b0..Does not load
 *  0b1..Loads
 */
#define CTIMER_CCR_CAP1RE(x)                     (((uint32_t)(((uint32_t)(x)) << CTIMER_CCR_CAP1RE_SHIFT)) & CTIMER_CCR_CAP1RE_MASK)

#define CTIMER_CCR_CAP1FE_MASK                   (0x10U)
#define CTIMER_CCR_CAP1FE_SHIFT                  (4U)
/*! CAP1FE - Falling Edge of Capture Channel 1
 *  0b0..Does not load
 *  0b1..Loads
 */
#define CTIMER_CCR_CAP1FE(x)                     (((uint32_t)(((uint32_t)(x)) << CTIMER_CCR_CAP1FE_SHIFT)) & CTIMER_CCR_CAP1FE_MASK)

#define CTIMER_CCR_CAP1I_MASK                    (0x20U)
#define CTIMER_CCR_CAP1I_SHIFT                   (5U)
/*! CAP1I - Generate Interrupt on Channel 1 Capture Event
 *  0b0..Does not generates
 *  0b1..Generates
 */
#define CTIMER_CCR_CAP1I(x)                      (((uint32_t)(((uint32_t)(x)) << CTIMER_CCR_CAP1I_SHIFT)) & CTIMER_CCR_CAP1I_MASK)

#define CTIMER_CCR_CAP2RE_MASK                   (0x40U)
#define CTIMER_CCR_CAP2RE_SHIFT                  (6U)
/*! CAP2RE - Rising Edge of Capture Channel 2
 *  0b0..Does not load
 *  0b1..Loads
 */
#define CTIMER_CCR_CAP2RE(x)                     (((uint32_t)(((uint32_t)(x)) << CTIMER_CCR_CAP2RE_SHIFT)) & CTIMER_CCR_CAP2RE_MASK)

#define CTIMER_CCR_CAP2FE_MASK                   (0x80U)
#define CTIMER_CCR_CAP2FE_SHIFT                  (7U)
/*! CAP2FE - Falling Edge of Capture Channel 2
 *  0b0..Does not load
 *  0b1..Loads
 */
#define CTIMER_CCR_CAP2FE(x)                     (((uint32_t)(((uint32_t)(x)) << CTIMER_CCR_CAP2FE_SHIFT)) & CTIMER_CCR_CAP2FE_MASK)

#define CTIMER_CCR_CAP2I_MASK                    (0x100U)
#define CTIMER_CCR_CAP2I_SHIFT                   (8U)
/*! CAP2I - Generate Interrupt on Channel 2 Capture Event
 *  0b0..Does not generate
 *  0b1..Generates
 */
#define CTIMER_CCR_CAP2I(x)                      (((uint32_t)(((uint32_t)(x)) << CTIMER_CCR_CAP2I_SHIFT)) & CTIMER_CCR_CAP2I_MASK)

#define CTIMER_CCR_CAP3RE_MASK                   (0x200U)
#define CTIMER_CCR_CAP3RE_SHIFT                  (9U)
/*! CAP3RE - Rising Edge of Capture Channel 3
 *  0b0..Does not load
 *  0b1..Loads
 */
#define CTIMER_CCR_CAP3RE(x)                     (((uint32_t)(((uint32_t)(x)) << CTIMER_CCR_CAP3RE_SHIFT)) & CTIMER_CCR_CAP3RE_MASK)

#define CTIMER_CCR_CAP3FE_MASK                   (0x400U)
#define CTIMER_CCR_CAP3FE_SHIFT                  (10U)
/*! CAP3FE - Falling Edge of Capture Channel 3
 *  0b0..Does not load
 *  0b1..Loads
 */
#define CTIMER_CCR_CAP3FE(x)                     (((uint32_t)(((uint32_t)(x)) << CTIMER_CCR_CAP3FE_SHIFT)) & CTIMER_CCR_CAP3FE_MASK)

#define CTIMER_CCR_CAP3I_MASK                    (0x800U)
#define CTIMER_CCR_CAP3I_SHIFT                   (11U)
/*! CAP3I - Generate Interrupt on Channel 3 Capture Event
 *  0b0..Does not generate
 *  0b1..Generates
 */
#define CTIMER_CCR_CAP3I(x)                      (((uint32_t)(((uint32_t)(x)) << CTIMER_CCR_CAP3I_SHIFT)) & CTIMER_CCR_CAP3I_MASK)
/*! @} */

/*! @name CR - Capture */
/*! @{ */

#define CTIMER_CR_CAP_MASK                       (0xFFFFFFFFU)
#define CTIMER_CR_CAP_SHIFT                      (0U)
/*! CAP - Timer Counter Capture Value */
#define CTIMER_CR_CAP(x)                         (((uint32_t)(((uint32_t)(x)) << CTIMER_CR_CAP_SHIFT)) & CTIMER_CR_CAP_MASK)
/*! @} */

/*! @name EMR - External Match */
/*! @{ */

#define CTIMER_EMR_EM0_MASK                      (0x1U)
#define CTIMER_EMR_EM0_SHIFT                     (0U)
/*! EM0 - External Match 0
 *  0b0..Low
 *  0b1..High
 */
#define CTIMER_EMR_EM0(x)                        (((uint32_t)(((uint32_t)(x)) << CTIMER_EMR_EM0_SHIFT)) & CTIMER_EMR_EM0_MASK)

#define CTIMER_EMR_EM1_MASK                      (0x2U)
#define CTIMER_EMR_EM1_SHIFT                     (1U)
/*! EM1 - External Match 1
 *  0b0..Low
 *  0b1..High
 */
#define CTIMER_EMR_EM1(x)                        (((uint32_t)(((uint32_t)(x)) << CTIMER_EMR_EM1_SHIFT)) & CTIMER_EMR_EM1_MASK)

#define CTIMER_EMR_EM2_MASK                      (0x4U)
#define CTIMER_EMR_EM2_SHIFT                     (2U)
/*! EM2 - External Match 2
 *  0b0..Low
 *  0b1..High
 */
#define CTIMER_EMR_EM2(x)                        (((uint32_t)(((uint32_t)(x)) << CTIMER_EMR_EM2_SHIFT)) & CTIMER_EMR_EM2_MASK)

#define CTIMER_EMR_EM3_MASK                      (0x8U)
#define CTIMER_EMR_EM3_SHIFT                     (3U)
/*! EM3 - External Match 3
 *  0b0..Low
 *  0b1..High
 */
#define CTIMER_EMR_EM3(x)                        (((uint32_t)(((uint32_t)(x)) << CTIMER_EMR_EM3_SHIFT)) & CTIMER_EMR_EM3_MASK)

#define CTIMER_EMR_EMC0_MASK                     (0x30U)
#define CTIMER_EMR_EMC0_SHIFT                    (4U)
/*! EMC0 - External Match Control 0
 *  0b00..Does nothing
 *  0b01..Goes low
 *  0b10..Goes high
 *  0b11..Toggles
 */
#define CTIMER_EMR_EMC0(x)                       (((uint32_t)(((uint32_t)(x)) << CTIMER_EMR_EMC0_SHIFT)) & CTIMER_EMR_EMC0_MASK)

#define CTIMER_EMR_EMC1_MASK                     (0xC0U)
#define CTIMER_EMR_EMC1_SHIFT                    (6U)
/*! EMC1 - External Match Control 1
 *  0b00..Does nothing
 *  0b01..Goes low
 *  0b10..Goes high
 *  0b11..Toggles
 */
#define CTIMER_EMR_EMC1(x)                       (((uint32_t)(((uint32_t)(x)) << CTIMER_EMR_EMC1_SHIFT)) & CTIMER_EMR_EMC1_MASK)

#define CTIMER_EMR_EMC2_MASK                     (0x300U)
#define CTIMER_EMR_EMC2_SHIFT                    (8U)
/*! EMC2 - External Match Control 2
 *  0b00..Does nothing
 *  0b01..Goes low
 *  0b10..Goes high
 *  0b11..Toggles
 */
#define CTIMER_EMR_EMC2(x)                       (((uint32_t)(((uint32_t)(x)) << CTIMER_EMR_EMC2_SHIFT)) & CTIMER_EMR_EMC2_MASK)

#define CTIMER_EMR_EMC3_MASK                     (0xC00U)
#define CTIMER_EMR_EMC3_SHIFT                    (10U)
/*! EMC3 - External Match Control 3
 *  0b00..Does nothing
 *  0b01..Goes low
 *  0b10..Goes high
 *  0b11..Toggles
 */
#define CTIMER_EMR_EMC3(x)                       (((uint32_t)(((uint32_t)(x)) << CTIMER_EMR_EMC3_SHIFT)) & CTIMER_EMR_EMC3_MASK)
/*! @} */

/*! @name CTCR - Count Control */
/*! @{ */

#define CTIMER_CTCR_CTMODE_MASK                  (0x3U)
#define CTIMER_CTCR_CTMODE_SHIFT                 (0U)
/*! CTMODE - Counter Timer Mode
 *  0b00..Timer mode
 *  0b01..Counter mode rising edge
 *  0b10..Counter mode falling edge
 *  0b11..Counter mode dual edge
 */
#define CTIMER_CTCR_CTMODE(x)                    (((uint32_t)(((uint32_t)(x)) << CTIMER_CTCR_CTMODE_SHIFT)) & CTIMER_CTCR_CTMODE_MASK)

#define CTIMER_CTCR_CINSEL_MASK                  (0xCU)
#define CTIMER_CTCR_CINSEL_SHIFT                 (2U)
/*! CINSEL - Count Input Select
 *  0b00..Channel 0, CAPn[0] for CTIMERn
 *  0b01..Channel 1, CAPn[1] for CTIMERn
 *  0b10..Channel 2, CAPn[2] for CTIMERn
 *  0b11..Channel 3, CAPn[3] for CTIMERn
 */
#define CTIMER_CTCR_CINSEL(x)                    (((uint32_t)(((uint32_t)(x)) << CTIMER_CTCR_CINSEL_SHIFT)) & CTIMER_CTCR_CINSEL_MASK)

#define CTIMER_CTCR_ENCC_MASK                    (0x10U)
#define CTIMER_CTCR_ENCC_SHIFT                   (4U)
/*! ENCC - Capture Channel Enable */
#define CTIMER_CTCR_ENCC(x)                      (((uint32_t)(((uint32_t)(x)) << CTIMER_CTCR_ENCC_SHIFT)) & CTIMER_CTCR_ENCC_MASK)

#define CTIMER_CTCR_SELCC_MASK                   (0xE0U)
#define CTIMER_CTCR_SELCC_SHIFT                  (5U)
/*! SELCC - Edge Select
 *  0b000..Capture channel 0 rising edge
 *  0b001..Capture channel 0 falling edge
 *  0b010..Capture channel 1 rising edge
 *  0b011..Capture channel 1 falling edge
 *  0b100..Capture channel 2 rising edge
 *  0b101..Capture channel 2 falling edge
 */
#define CTIMER_CTCR_SELCC(x)                     (((uint32_t)(((uint32_t)(x)) << CTIMER_CTCR_SELCC_SHIFT)) & CTIMER_CTCR_SELCC_MASK)
/*! @} */

/*! @name PWMC - PWM Control */
/*! @{ */

#define CTIMER_PWMC_PWMEN0_MASK                  (0x1U)
#define CTIMER_PWMC_PWMEN0_SHIFT                 (0U)
/*! PWMEN0 - PWM Mode Enable for Channel 0
 *  0b0..Disable
 *  0b1..Enable
 */
#define CTIMER_PWMC_PWMEN0(x)                    (((uint32_t)(((uint32_t)(x)) << CTIMER_PWMC_PWMEN0_SHIFT)) & CTIMER_PWMC_PWMEN0_MASK)

#define CTIMER_PWMC_PWMEN1_MASK                  (0x2U)
#define CTIMER_PWMC_PWMEN1_SHIFT                 (1U)
/*! PWMEN1 - PWM Mode Enable for Channel 1
 *  0b0..Disable
 *  0b1..Enable
 */
#define CTIMER_PWMC_PWMEN1(x)                    (((uint32_t)(((uint32_t)(x)) << CTIMER_PWMC_PWMEN1_SHIFT)) & CTIMER_PWMC_PWMEN1_MASK)

#define CTIMER_PWMC_PWMEN2_MASK                  (0x4U)
#define CTIMER_PWMC_PWMEN2_SHIFT                 (2U)
/*! PWMEN2 - PWM Mode Enable for Channel 2
 *  0b0..Disable
 *  0b1..Enable
 */
#define CTIMER_PWMC_PWMEN2(x)                    (((uint32_t)(((uint32_t)(x)) << CTIMER_PWMC_PWMEN2_SHIFT)) & CTIMER_PWMC_PWMEN2_MASK)

#define CTIMER_PWMC_PWMEN3_MASK                  (0x8U)
#define CTIMER_PWMC_PWMEN3_SHIFT                 (3U)
/*! PWMEN3 - PWM Mode Enable for Channel 3
 *  0b0..Disable
 *  0b1..Enable
 */
#define CTIMER_PWMC_PWMEN3(x)                    (((uint32_t)(((uint32_t)(x)) << CTIMER_PWMC_PWMEN3_SHIFT)) & CTIMER_PWMC_PWMEN3_MASK)
/*! @} */

/*! @name MSR - Match Shadow */
/*! @{ */

#define CTIMER_MSR_MATCH_SHADOW_MASK             (0xFFFFFFFFU)
#define CTIMER_MSR_MATCH_SHADOW_SHIFT            (0U)
/*! MATCH_SHADOW - Timer Counter Match Shadow Value */
#define CTIMER_MSR_MATCH_SHADOW(x)               (((uint32_t)(((uint32_t)(x)) << CTIMER_MSR_MATCH_SHADOW_SHIFT)) & CTIMER_MSR_MATCH_SHADOW_MASK)
/*! @} */


/*!
 * @}
 */ /* end of group CTIMER_Register_Masks */


/*!
 * @}
 */ /* end of group CTIMER_Peripheral_Access_Layer */


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


#endif  /* PERI_CTIMER_H_ */

