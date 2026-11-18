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
**         CMSIS Peripheral Access Layer for GPIO
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
 * @file PERI_GPIO.h
 * @version 2.0
 * @date 2024-10-29
 * @brief CMSIS Peripheral Access Layer for GPIO
 *
 * CMSIS Peripheral Access Layer for GPIO
 */

#if !defined(PERI_GPIO_H_)
#define PERI_GPIO_H_                             /**< Symbol preventing repeated inclusion */

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
   -- GPIO Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup GPIO_Peripheral_Access_Layer GPIO Peripheral Access Layer
 * @{
 */

/** GPIO - Size of Registers Arrays */
#define GPIO_PDR_COUNT                            32u
#define GPIO_ICR_COUNT                            32u
#define GPIO_ISFR_COUNT                           1u

/** GPIO - Register Layout Typedef */
typedef struct {
  __I  uint32_t VERID;                             /**< Version ID, offset: 0x0 */
  __I  uint32_t PARAM;                             /**< Parameter, offset: 0x4 */
       uint8_t RESERVED_0[56];
  __IO uint32_t PDOR;                              /**< Port Data Output, offset: 0x40 */
  __IO uint32_t PSOR;                              /**< Port Set Output, offset: 0x44 */
  __IO uint32_t PCOR;                              /**< Port Clear Output, offset: 0x48 */
  __IO uint32_t PTOR;                              /**< Port Toggle Output, offset: 0x4C */
  __I  uint32_t PDIR;                              /**< Port Data Input, offset: 0x50 */
  __IO uint32_t PDDR;                              /**< Port Data Direction, offset: 0x54 */
  __IO uint32_t PIDR;                              /**< Port Input Disable, offset: 0x58 */
       uint8_t RESERVED_1[4];
  __IO uint8_t PDR[GPIO_PDR_COUNT];                /**< Pin Data, array offset: 0x60, array step: 0x1 */
  __IO uint32_t ICR[GPIO_ICR_COUNT];               /**< Interrupt Control 0..Interrupt Control 31, array offset: 0x80, array step: 0x4 */
  __IO uint32_t GICLR;                             /**< Global Interrupt Control Low, offset: 0x100 */
  __IO uint32_t GICHR;                             /**< Global Interrupt Control High, offset: 0x104 */
       uint8_t RESERVED_2[24];
  __IO uint32_t ISFR[GPIO_ISFR_COUNT];             /**< Interrupt Status Flag, array offset: 0x120, array step: 0x4 */
} GPIO_Type;

/* ----------------------------------------------------------------------------
   -- GPIO Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup GPIO_Register_Masks GPIO Register Masks
 * @{
 */

/*! @name VERID - Version ID */
/*! @{ */

#define GPIO_VERID_FEATURE_MASK                  (0xFFFFU)
#define GPIO_VERID_FEATURE_SHIFT                 (0U)
/*! FEATURE - Feature Specification Number
 *  0b0000000000000000..Basic implementation
 *  0b0000000000000001..Protection registers implemented
 */
#define GPIO_VERID_FEATURE(x)                    (((uint32_t)(((uint32_t)(x)) << GPIO_VERID_FEATURE_SHIFT)) & GPIO_VERID_FEATURE_MASK)

#define GPIO_VERID_MINOR_MASK                    (0xFF0000U)
#define GPIO_VERID_MINOR_SHIFT                   (16U)
/*! MINOR - Minor Version Number */
#define GPIO_VERID_MINOR(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_VERID_MINOR_SHIFT)) & GPIO_VERID_MINOR_MASK)

#define GPIO_VERID_MAJOR_MASK                    (0xFF000000U)
#define GPIO_VERID_MAJOR_SHIFT                   (24U)
/*! MAJOR - Major Version Number */
#define GPIO_VERID_MAJOR(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_VERID_MAJOR_SHIFT)) & GPIO_VERID_MAJOR_MASK)
/*! @} */

/*! @name PARAM - Parameter */
/*! @{ */

#define GPIO_PARAM_IRQNUM_MASK                   (0xFU)
#define GPIO_PARAM_IRQNUM_SHIFT                  (0U)
/*! IRQNUM - Interrupt Number */
#define GPIO_PARAM_IRQNUM(x)                     (((uint32_t)(((uint32_t)(x)) << GPIO_PARAM_IRQNUM_SHIFT)) & GPIO_PARAM_IRQNUM_MASK)
/*! @} */

/*! @name PDOR - Port Data Output */
/*! @{ */

#define GPIO_PDOR_PDO0_MASK                      (0x1U)
#define GPIO_PDOR_PDO0_SHIFT                     (0U)
/*! PDO0 - Port Data Output
 *  0b0..Logic level 0
 *  0b1..Logic level 1
 */
#define GPIO_PDOR_PDO0(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_PDOR_PDO0_SHIFT)) & GPIO_PDOR_PDO0_MASK)

#define GPIO_PDOR_PDO1_MASK                      (0x2U)
#define GPIO_PDOR_PDO1_SHIFT                     (1U)
/*! PDO1 - Port Data Output
 *  0b0..Logic level 0
 *  0b1..Logic level 1
 */
#define GPIO_PDOR_PDO1(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_PDOR_PDO1_SHIFT)) & GPIO_PDOR_PDO1_MASK)

#define GPIO_PDOR_PDO2_MASK                      (0x4U)
#define GPIO_PDOR_PDO2_SHIFT                     (2U)
/*! PDO2 - Port Data Output
 *  0b0..Logic level 0
 *  0b1..Logic level 1
 */
#define GPIO_PDOR_PDO2(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_PDOR_PDO2_SHIFT)) & GPIO_PDOR_PDO2_MASK)

#define GPIO_PDOR_PDO3_MASK                      (0x8U)
#define GPIO_PDOR_PDO3_SHIFT                     (3U)
/*! PDO3 - Port Data Output
 *  0b0..Logic level 0
 *  0b1..Logic level 1
 */
#define GPIO_PDOR_PDO3(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_PDOR_PDO3_SHIFT)) & GPIO_PDOR_PDO3_MASK)

#define GPIO_PDOR_PDO4_MASK                      (0x10U)
#define GPIO_PDOR_PDO4_SHIFT                     (4U)
/*! PDO4 - Port Data Output
 *  0b0..Logic level 0
 *  0b1..Logic level 1
 */
#define GPIO_PDOR_PDO4(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_PDOR_PDO4_SHIFT)) & GPIO_PDOR_PDO4_MASK)

#define GPIO_PDOR_PDO5_MASK                      (0x20U)
#define GPIO_PDOR_PDO5_SHIFT                     (5U)
/*! PDO5 - Port Data Output
 *  0b0..Logic level 0
 *  0b1..Logic level 1
 */
#define GPIO_PDOR_PDO5(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_PDOR_PDO5_SHIFT)) & GPIO_PDOR_PDO5_MASK)

#define GPIO_PDOR_PDO6_MASK                      (0x40U)
#define GPIO_PDOR_PDO6_SHIFT                     (6U)
/*! PDO6 - Port Data Output
 *  0b0..Logic level 0
 *  0b1..Logic level 1
 */
#define GPIO_PDOR_PDO6(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_PDOR_PDO6_SHIFT)) & GPIO_PDOR_PDO6_MASK)

#define GPIO_PDOR_PDO7_MASK                      (0x80U)
#define GPIO_PDOR_PDO7_SHIFT                     (7U)
/*! PDO7 - Port Data Output
 *  0b0..Logic level 0
 *  0b1..Logic level 1
 */
#define GPIO_PDOR_PDO7(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_PDOR_PDO7_SHIFT)) & GPIO_PDOR_PDO7_MASK)

#define GPIO_PDOR_PDO8_MASK                      (0x100U)
#define GPIO_PDOR_PDO8_SHIFT                     (8U)
/*! PDO8 - Port Data Output
 *  0b0..Logic level 0
 *  0b1..Logic level 1
 */
#define GPIO_PDOR_PDO8(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_PDOR_PDO8_SHIFT)) & GPIO_PDOR_PDO8_MASK)

#define GPIO_PDOR_PDO9_MASK                      (0x200U)
#define GPIO_PDOR_PDO9_SHIFT                     (9U)
/*! PDO9 - Port Data Output
 *  0b0..Logic level 0
 *  0b1..Logic level 1
 */
#define GPIO_PDOR_PDO9(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_PDOR_PDO9_SHIFT)) & GPIO_PDOR_PDO9_MASK)

#define GPIO_PDOR_PDO10_MASK                     (0x400U)
#define GPIO_PDOR_PDO10_SHIFT                    (10U)
/*! PDO10 - Port Data Output
 *  0b0..Logic level 0
 *  0b1..Logic level 1
 */
#define GPIO_PDOR_PDO10(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDOR_PDO10_SHIFT)) & GPIO_PDOR_PDO10_MASK)

#define GPIO_PDOR_PDO11_MASK                     (0x800U)
#define GPIO_PDOR_PDO11_SHIFT                    (11U)
/*! PDO11 - Port Data Output
 *  0b0..Logic level 0
 *  0b1..Logic level 1
 */
#define GPIO_PDOR_PDO11(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDOR_PDO11_SHIFT)) & GPIO_PDOR_PDO11_MASK)

#define GPIO_PDOR_PDO12_MASK                     (0x1000U)
#define GPIO_PDOR_PDO12_SHIFT                    (12U)
/*! PDO12 - Port Data Output
 *  0b0..Logic level 0
 *  0b1..Logic level 1
 */
#define GPIO_PDOR_PDO12(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDOR_PDO12_SHIFT)) & GPIO_PDOR_PDO12_MASK)

#define GPIO_PDOR_PDO13_MASK                     (0x2000U)
#define GPIO_PDOR_PDO13_SHIFT                    (13U)
/*! PDO13 - Port Data Output
 *  0b0..Logic level 0
 *  0b1..Logic level 1
 */
#define GPIO_PDOR_PDO13(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDOR_PDO13_SHIFT)) & GPIO_PDOR_PDO13_MASK)

#define GPIO_PDOR_PDO14_MASK                     (0x4000U)
#define GPIO_PDOR_PDO14_SHIFT                    (14U)
/*! PDO14 - Port Data Output
 *  0b0..Logic level 0
 *  0b1..Logic level 1
 */
#define GPIO_PDOR_PDO14(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDOR_PDO14_SHIFT)) & GPIO_PDOR_PDO14_MASK)

#define GPIO_PDOR_PDO15_MASK                     (0x8000U)
#define GPIO_PDOR_PDO15_SHIFT                    (15U)
/*! PDO15 - Port Data Output
 *  0b0..Logic level 0
 *  0b1..Logic level 1
 */
#define GPIO_PDOR_PDO15(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDOR_PDO15_SHIFT)) & GPIO_PDOR_PDO15_MASK)

#define GPIO_PDOR_PDO16_MASK                     (0x10000U)
#define GPIO_PDOR_PDO16_SHIFT                    (16U)
/*! PDO16 - Port Data Output
 *  0b0..Logic level 0
 *  0b1..Logic level 1
 */
#define GPIO_PDOR_PDO16(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDOR_PDO16_SHIFT)) & GPIO_PDOR_PDO16_MASK)

#define GPIO_PDOR_PDO17_MASK                     (0x20000U)
#define GPIO_PDOR_PDO17_SHIFT                    (17U)
/*! PDO17 - Port Data Output
 *  0b0..Logic level 0
 *  0b1..Logic level 1
 */
#define GPIO_PDOR_PDO17(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDOR_PDO17_SHIFT)) & GPIO_PDOR_PDO17_MASK)

#define GPIO_PDOR_PDO18_MASK                     (0x40000U)
#define GPIO_PDOR_PDO18_SHIFT                    (18U)
/*! PDO18 - Port Data Output
 *  0b0..Logic level 0
 *  0b1..Logic level 1
 */
#define GPIO_PDOR_PDO18(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDOR_PDO18_SHIFT)) & GPIO_PDOR_PDO18_MASK)

#define GPIO_PDOR_PDO19_MASK                     (0x80000U)
#define GPIO_PDOR_PDO19_SHIFT                    (19U)
/*! PDO19 - Port Data Output
 *  0b0..Logic level 0
 *  0b1..Logic level 1
 */
#define GPIO_PDOR_PDO19(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDOR_PDO19_SHIFT)) & GPIO_PDOR_PDO19_MASK)

#define GPIO_PDOR_PDO20_MASK                     (0x100000U)
#define GPIO_PDOR_PDO20_SHIFT                    (20U)
/*! PDO20 - Port Data Output
 *  0b0..Logic level 0
 *  0b1..Logic level 1
 */
#define GPIO_PDOR_PDO20(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDOR_PDO20_SHIFT)) & GPIO_PDOR_PDO20_MASK)

#define GPIO_PDOR_PDO21_MASK                     (0x200000U)
#define GPIO_PDOR_PDO21_SHIFT                    (21U)
/*! PDO21 - Port Data Output
 *  0b0..Logic level 0
 *  0b1..Logic level 1
 */
#define GPIO_PDOR_PDO21(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDOR_PDO21_SHIFT)) & GPIO_PDOR_PDO21_MASK)

#define GPIO_PDOR_PDO22_MASK                     (0x400000U)
#define GPIO_PDOR_PDO22_SHIFT                    (22U)
/*! PDO22 - Port Data Output
 *  0b0..Logic level 0
 *  0b1..Logic level 1
 */
#define GPIO_PDOR_PDO22(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDOR_PDO22_SHIFT)) & GPIO_PDOR_PDO22_MASK)

#define GPIO_PDOR_PDO23_MASK                     (0x800000U)
#define GPIO_PDOR_PDO23_SHIFT                    (23U)
/*! PDO23 - Port Data Output
 *  0b0..Logic level 0
 *  0b1..Logic level 1
 */
#define GPIO_PDOR_PDO23(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDOR_PDO23_SHIFT)) & GPIO_PDOR_PDO23_MASK)

#define GPIO_PDOR_PDO24_MASK                     (0x1000000U)
#define GPIO_PDOR_PDO24_SHIFT                    (24U)
/*! PDO24 - Port Data Output
 *  0b0..Logic level 0
 *  0b1..Logic level 1
 */
#define GPIO_PDOR_PDO24(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDOR_PDO24_SHIFT)) & GPIO_PDOR_PDO24_MASK)

#define GPIO_PDOR_PDO25_MASK                     (0x2000000U)
#define GPIO_PDOR_PDO25_SHIFT                    (25U)
/*! PDO25 - Port Data Output
 *  0b0..Logic level 0
 *  0b1..Logic level 1
 */
#define GPIO_PDOR_PDO25(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDOR_PDO25_SHIFT)) & GPIO_PDOR_PDO25_MASK)

#define GPIO_PDOR_PDO26_MASK                     (0x4000000U)
#define GPIO_PDOR_PDO26_SHIFT                    (26U)
/*! PDO26 - Port Data Output
 *  0b0..Logic level 0
 *  0b1..Logic level 1
 */
#define GPIO_PDOR_PDO26(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDOR_PDO26_SHIFT)) & GPIO_PDOR_PDO26_MASK)

#define GPIO_PDOR_PDO27_MASK                     (0x8000000U)
#define GPIO_PDOR_PDO27_SHIFT                    (27U)
/*! PDO27 - Port Data Output
 *  0b0..Logic level 0
 *  0b1..Logic level 1
 */
#define GPIO_PDOR_PDO27(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDOR_PDO27_SHIFT)) & GPIO_PDOR_PDO27_MASK)

#define GPIO_PDOR_PDO28_MASK                     (0x10000000U)
#define GPIO_PDOR_PDO28_SHIFT                    (28U)
/*! PDO28 - Port Data Output
 *  0b0..Logic level 0
 *  0b1..Logic level 1
 */
#define GPIO_PDOR_PDO28(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDOR_PDO28_SHIFT)) & GPIO_PDOR_PDO28_MASK)

#define GPIO_PDOR_PDO29_MASK                     (0x20000000U)
#define GPIO_PDOR_PDO29_SHIFT                    (29U)
/*! PDO29 - Port Data Output
 *  0b0..Logic level 0
 *  0b1..Logic level 1
 */
#define GPIO_PDOR_PDO29(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDOR_PDO29_SHIFT)) & GPIO_PDOR_PDO29_MASK)

#define GPIO_PDOR_PDO30_MASK                     (0x40000000U)
#define GPIO_PDOR_PDO30_SHIFT                    (30U)
/*! PDO30 - Port Data Output
 *  0b0..Logic level 0
 *  0b1..Logic level 1
 */
#define GPIO_PDOR_PDO30(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDOR_PDO30_SHIFT)) & GPIO_PDOR_PDO30_MASK)

#define GPIO_PDOR_PDO31_MASK                     (0x80000000U)
#define GPIO_PDOR_PDO31_SHIFT                    (31U)
/*! PDO31 - Port Data Output
 *  0b0..Logic level 0
 *  0b1..Logic level 1
 */
#define GPIO_PDOR_PDO31(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDOR_PDO31_SHIFT)) & GPIO_PDOR_PDO31_MASK)
/*! @} */

/*! @name PSOR - Port Set Output */
/*! @{ */

#define GPIO_PSOR_PTSO0_MASK                     (0x1U)
#define GPIO_PSOR_PTSO0_SHIFT                    (0U)
/*! PTSO0 - Port Set Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 1
 */
#define GPIO_PSOR_PTSO0(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PSOR_PTSO0_SHIFT)) & GPIO_PSOR_PTSO0_MASK)

#define GPIO_PSOR_PTSO1_MASK                     (0x2U)
#define GPIO_PSOR_PTSO1_SHIFT                    (1U)
/*! PTSO1 - Port Set Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 1
 */
#define GPIO_PSOR_PTSO1(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PSOR_PTSO1_SHIFT)) & GPIO_PSOR_PTSO1_MASK)

#define GPIO_PSOR_PTSO2_MASK                     (0x4U)
#define GPIO_PSOR_PTSO2_SHIFT                    (2U)
/*! PTSO2 - Port Set Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 1
 */
#define GPIO_PSOR_PTSO2(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PSOR_PTSO2_SHIFT)) & GPIO_PSOR_PTSO2_MASK)

#define GPIO_PSOR_PTSO3_MASK                     (0x8U)
#define GPIO_PSOR_PTSO3_SHIFT                    (3U)
/*! PTSO3 - Port Set Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 1
 */
#define GPIO_PSOR_PTSO3(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PSOR_PTSO3_SHIFT)) & GPIO_PSOR_PTSO3_MASK)

#define GPIO_PSOR_PTSO4_MASK                     (0x10U)
#define GPIO_PSOR_PTSO4_SHIFT                    (4U)
/*! PTSO4 - Port Set Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 1
 */
#define GPIO_PSOR_PTSO4(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PSOR_PTSO4_SHIFT)) & GPIO_PSOR_PTSO4_MASK)

#define GPIO_PSOR_PTSO5_MASK                     (0x20U)
#define GPIO_PSOR_PTSO5_SHIFT                    (5U)
/*! PTSO5 - Port Set Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 1
 */
#define GPIO_PSOR_PTSO5(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PSOR_PTSO5_SHIFT)) & GPIO_PSOR_PTSO5_MASK)

#define GPIO_PSOR_PTSO6_MASK                     (0x40U)
#define GPIO_PSOR_PTSO6_SHIFT                    (6U)
/*! PTSO6 - Port Set Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 1
 */
#define GPIO_PSOR_PTSO6(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PSOR_PTSO6_SHIFT)) & GPIO_PSOR_PTSO6_MASK)

#define GPIO_PSOR_PTSO7_MASK                     (0x80U)
#define GPIO_PSOR_PTSO7_SHIFT                    (7U)
/*! PTSO7 - Port Set Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 1
 */
#define GPIO_PSOR_PTSO7(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PSOR_PTSO7_SHIFT)) & GPIO_PSOR_PTSO7_MASK)

#define GPIO_PSOR_PTSO8_MASK                     (0x100U)
#define GPIO_PSOR_PTSO8_SHIFT                    (8U)
/*! PTSO8 - Port Set Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 1
 */
#define GPIO_PSOR_PTSO8(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PSOR_PTSO8_SHIFT)) & GPIO_PSOR_PTSO8_MASK)

#define GPIO_PSOR_PTSO9_MASK                     (0x200U)
#define GPIO_PSOR_PTSO9_SHIFT                    (9U)
/*! PTSO9 - Port Set Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 1
 */
#define GPIO_PSOR_PTSO9(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PSOR_PTSO9_SHIFT)) & GPIO_PSOR_PTSO9_MASK)

#define GPIO_PSOR_PTSO10_MASK                    (0x400U)
#define GPIO_PSOR_PTSO10_SHIFT                   (10U)
/*! PTSO10 - Port Set Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 1
 */
#define GPIO_PSOR_PTSO10(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PSOR_PTSO10_SHIFT)) & GPIO_PSOR_PTSO10_MASK)

#define GPIO_PSOR_PTSO11_MASK                    (0x800U)
#define GPIO_PSOR_PTSO11_SHIFT                   (11U)
/*! PTSO11 - Port Set Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 1
 */
#define GPIO_PSOR_PTSO11(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PSOR_PTSO11_SHIFT)) & GPIO_PSOR_PTSO11_MASK)

#define GPIO_PSOR_PTSO12_MASK                    (0x1000U)
#define GPIO_PSOR_PTSO12_SHIFT                   (12U)
/*! PTSO12 - Port Set Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 1
 */
#define GPIO_PSOR_PTSO12(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PSOR_PTSO12_SHIFT)) & GPIO_PSOR_PTSO12_MASK)

#define GPIO_PSOR_PTSO13_MASK                    (0x2000U)
#define GPIO_PSOR_PTSO13_SHIFT                   (13U)
/*! PTSO13 - Port Set Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 1
 */
#define GPIO_PSOR_PTSO13(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PSOR_PTSO13_SHIFT)) & GPIO_PSOR_PTSO13_MASK)

#define GPIO_PSOR_PTSO14_MASK                    (0x4000U)
#define GPIO_PSOR_PTSO14_SHIFT                   (14U)
/*! PTSO14 - Port Set Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 1
 */
#define GPIO_PSOR_PTSO14(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PSOR_PTSO14_SHIFT)) & GPIO_PSOR_PTSO14_MASK)

#define GPIO_PSOR_PTSO15_MASK                    (0x8000U)
#define GPIO_PSOR_PTSO15_SHIFT                   (15U)
/*! PTSO15 - Port Set Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 1
 */
#define GPIO_PSOR_PTSO15(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PSOR_PTSO15_SHIFT)) & GPIO_PSOR_PTSO15_MASK)

#define GPIO_PSOR_PTSO16_MASK                    (0x10000U)
#define GPIO_PSOR_PTSO16_SHIFT                   (16U)
/*! PTSO16 - Port Set Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 1
 */
#define GPIO_PSOR_PTSO16(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PSOR_PTSO16_SHIFT)) & GPIO_PSOR_PTSO16_MASK)

#define GPIO_PSOR_PTSO17_MASK                    (0x20000U)
#define GPIO_PSOR_PTSO17_SHIFT                   (17U)
/*! PTSO17 - Port Set Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 1
 */
#define GPIO_PSOR_PTSO17(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PSOR_PTSO17_SHIFT)) & GPIO_PSOR_PTSO17_MASK)

#define GPIO_PSOR_PTSO18_MASK                    (0x40000U)
#define GPIO_PSOR_PTSO18_SHIFT                   (18U)
/*! PTSO18 - Port Set Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 1
 */
#define GPIO_PSOR_PTSO18(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PSOR_PTSO18_SHIFT)) & GPIO_PSOR_PTSO18_MASK)

#define GPIO_PSOR_PTSO19_MASK                    (0x80000U)
#define GPIO_PSOR_PTSO19_SHIFT                   (19U)
/*! PTSO19 - Port Set Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 1
 */
#define GPIO_PSOR_PTSO19(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PSOR_PTSO19_SHIFT)) & GPIO_PSOR_PTSO19_MASK)

#define GPIO_PSOR_PTSO20_MASK                    (0x100000U)
#define GPIO_PSOR_PTSO20_SHIFT                   (20U)
/*! PTSO20 - Port Set Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 1
 */
#define GPIO_PSOR_PTSO20(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PSOR_PTSO20_SHIFT)) & GPIO_PSOR_PTSO20_MASK)

#define GPIO_PSOR_PTSO21_MASK                    (0x200000U)
#define GPIO_PSOR_PTSO21_SHIFT                   (21U)
/*! PTSO21 - Port Set Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 1
 */
#define GPIO_PSOR_PTSO21(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PSOR_PTSO21_SHIFT)) & GPIO_PSOR_PTSO21_MASK)

#define GPIO_PSOR_PTSO22_MASK                    (0x400000U)
#define GPIO_PSOR_PTSO22_SHIFT                   (22U)
/*! PTSO22 - Port Set Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 1
 */
#define GPIO_PSOR_PTSO22(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PSOR_PTSO22_SHIFT)) & GPIO_PSOR_PTSO22_MASK)

#define GPIO_PSOR_PTSO23_MASK                    (0x800000U)
#define GPIO_PSOR_PTSO23_SHIFT                   (23U)
/*! PTSO23 - Port Set Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 1
 */
#define GPIO_PSOR_PTSO23(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PSOR_PTSO23_SHIFT)) & GPIO_PSOR_PTSO23_MASK)

#define GPIO_PSOR_PTSO24_MASK                    (0x1000000U)
#define GPIO_PSOR_PTSO24_SHIFT                   (24U)
/*! PTSO24 - Port Set Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 1
 */
#define GPIO_PSOR_PTSO24(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PSOR_PTSO24_SHIFT)) & GPIO_PSOR_PTSO24_MASK)

#define GPIO_PSOR_PTSO25_MASK                    (0x2000000U)
#define GPIO_PSOR_PTSO25_SHIFT                   (25U)
/*! PTSO25 - Port Set Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 1
 */
#define GPIO_PSOR_PTSO25(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PSOR_PTSO25_SHIFT)) & GPIO_PSOR_PTSO25_MASK)

#define GPIO_PSOR_PTSO26_MASK                    (0x4000000U)
#define GPIO_PSOR_PTSO26_SHIFT                   (26U)
/*! PTSO26 - Port Set Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 1
 */
#define GPIO_PSOR_PTSO26(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PSOR_PTSO26_SHIFT)) & GPIO_PSOR_PTSO26_MASK)

#define GPIO_PSOR_PTSO27_MASK                    (0x8000000U)
#define GPIO_PSOR_PTSO27_SHIFT                   (27U)
/*! PTSO27 - Port Set Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 1
 */
#define GPIO_PSOR_PTSO27(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PSOR_PTSO27_SHIFT)) & GPIO_PSOR_PTSO27_MASK)

#define GPIO_PSOR_PTSO28_MASK                    (0x10000000U)
#define GPIO_PSOR_PTSO28_SHIFT                   (28U)
/*! PTSO28 - Port Set Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 1
 */
#define GPIO_PSOR_PTSO28(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PSOR_PTSO28_SHIFT)) & GPIO_PSOR_PTSO28_MASK)

#define GPIO_PSOR_PTSO29_MASK                    (0x20000000U)
#define GPIO_PSOR_PTSO29_SHIFT                   (29U)
/*! PTSO29 - Port Set Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 1
 */
#define GPIO_PSOR_PTSO29(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PSOR_PTSO29_SHIFT)) & GPIO_PSOR_PTSO29_MASK)

#define GPIO_PSOR_PTSO30_MASK                    (0x40000000U)
#define GPIO_PSOR_PTSO30_SHIFT                   (30U)
/*! PTSO30 - Port Set Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 1
 */
#define GPIO_PSOR_PTSO30(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PSOR_PTSO30_SHIFT)) & GPIO_PSOR_PTSO30_MASK)

#define GPIO_PSOR_PTSO31_MASK                    (0x80000000U)
#define GPIO_PSOR_PTSO31_SHIFT                   (31U)
/*! PTSO31 - Port Set Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 1
 */
#define GPIO_PSOR_PTSO31(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PSOR_PTSO31_SHIFT)) & GPIO_PSOR_PTSO31_MASK)
/*! @} */

/*! @name PCOR - Port Clear Output */
/*! @{ */

#define GPIO_PCOR_PTCO0_MASK                     (0x1U)
#define GPIO_PCOR_PTCO0_SHIFT                    (0U)
/*! PTCO0 - Port Clear Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 0
 */
#define GPIO_PCOR_PTCO0(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PCOR_PTCO0_SHIFT)) & GPIO_PCOR_PTCO0_MASK)

#define GPIO_PCOR_PTCO1_MASK                     (0x2U)
#define GPIO_PCOR_PTCO1_SHIFT                    (1U)
/*! PTCO1 - Port Clear Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 0
 */
#define GPIO_PCOR_PTCO1(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PCOR_PTCO1_SHIFT)) & GPIO_PCOR_PTCO1_MASK)

#define GPIO_PCOR_PTCO2_MASK                     (0x4U)
#define GPIO_PCOR_PTCO2_SHIFT                    (2U)
/*! PTCO2 - Port Clear Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 0
 */
#define GPIO_PCOR_PTCO2(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PCOR_PTCO2_SHIFT)) & GPIO_PCOR_PTCO2_MASK)

#define GPIO_PCOR_PTCO3_MASK                     (0x8U)
#define GPIO_PCOR_PTCO3_SHIFT                    (3U)
/*! PTCO3 - Port Clear Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 0
 */
#define GPIO_PCOR_PTCO3(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PCOR_PTCO3_SHIFT)) & GPIO_PCOR_PTCO3_MASK)

#define GPIO_PCOR_PTCO4_MASK                     (0x10U)
#define GPIO_PCOR_PTCO4_SHIFT                    (4U)
/*! PTCO4 - Port Clear Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 0
 */
#define GPIO_PCOR_PTCO4(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PCOR_PTCO4_SHIFT)) & GPIO_PCOR_PTCO4_MASK)

#define GPIO_PCOR_PTCO5_MASK                     (0x20U)
#define GPIO_PCOR_PTCO5_SHIFT                    (5U)
/*! PTCO5 - Port Clear Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 0
 */
#define GPIO_PCOR_PTCO5(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PCOR_PTCO5_SHIFT)) & GPIO_PCOR_PTCO5_MASK)

#define GPIO_PCOR_PTCO6_MASK                     (0x40U)
#define GPIO_PCOR_PTCO6_SHIFT                    (6U)
/*! PTCO6 - Port Clear Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 0
 */
#define GPIO_PCOR_PTCO6(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PCOR_PTCO6_SHIFT)) & GPIO_PCOR_PTCO6_MASK)

#define GPIO_PCOR_PTCO7_MASK                     (0x80U)
#define GPIO_PCOR_PTCO7_SHIFT                    (7U)
/*! PTCO7 - Port Clear Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 0
 */
#define GPIO_PCOR_PTCO7(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PCOR_PTCO7_SHIFT)) & GPIO_PCOR_PTCO7_MASK)

#define GPIO_PCOR_PTCO8_MASK                     (0x100U)
#define GPIO_PCOR_PTCO8_SHIFT                    (8U)
/*! PTCO8 - Port Clear Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 0
 */
#define GPIO_PCOR_PTCO8(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PCOR_PTCO8_SHIFT)) & GPIO_PCOR_PTCO8_MASK)

#define GPIO_PCOR_PTCO9_MASK                     (0x200U)
#define GPIO_PCOR_PTCO9_SHIFT                    (9U)
/*! PTCO9 - Port Clear Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 0
 */
#define GPIO_PCOR_PTCO9(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PCOR_PTCO9_SHIFT)) & GPIO_PCOR_PTCO9_MASK)

#define GPIO_PCOR_PTCO10_MASK                    (0x400U)
#define GPIO_PCOR_PTCO10_SHIFT                   (10U)
/*! PTCO10 - Port Clear Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 0
 */
#define GPIO_PCOR_PTCO10(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PCOR_PTCO10_SHIFT)) & GPIO_PCOR_PTCO10_MASK)

#define GPIO_PCOR_PTCO11_MASK                    (0x800U)
#define GPIO_PCOR_PTCO11_SHIFT                   (11U)
/*! PTCO11 - Port Clear Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 0
 */
#define GPIO_PCOR_PTCO11(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PCOR_PTCO11_SHIFT)) & GPIO_PCOR_PTCO11_MASK)

#define GPIO_PCOR_PTCO12_MASK                    (0x1000U)
#define GPIO_PCOR_PTCO12_SHIFT                   (12U)
/*! PTCO12 - Port Clear Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 0
 */
#define GPIO_PCOR_PTCO12(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PCOR_PTCO12_SHIFT)) & GPIO_PCOR_PTCO12_MASK)

#define GPIO_PCOR_PTCO13_MASK                    (0x2000U)
#define GPIO_PCOR_PTCO13_SHIFT                   (13U)
/*! PTCO13 - Port Clear Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 0
 */
#define GPIO_PCOR_PTCO13(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PCOR_PTCO13_SHIFT)) & GPIO_PCOR_PTCO13_MASK)

#define GPIO_PCOR_PTCO14_MASK                    (0x4000U)
#define GPIO_PCOR_PTCO14_SHIFT                   (14U)
/*! PTCO14 - Port Clear Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 0
 */
#define GPIO_PCOR_PTCO14(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PCOR_PTCO14_SHIFT)) & GPIO_PCOR_PTCO14_MASK)

#define GPIO_PCOR_PTCO15_MASK                    (0x8000U)
#define GPIO_PCOR_PTCO15_SHIFT                   (15U)
/*! PTCO15 - Port Clear Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 0
 */
#define GPIO_PCOR_PTCO15(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PCOR_PTCO15_SHIFT)) & GPIO_PCOR_PTCO15_MASK)

#define GPIO_PCOR_PTCO16_MASK                    (0x10000U)
#define GPIO_PCOR_PTCO16_SHIFT                   (16U)
/*! PTCO16 - Port Clear Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 0
 */
#define GPIO_PCOR_PTCO16(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PCOR_PTCO16_SHIFT)) & GPIO_PCOR_PTCO16_MASK)

#define GPIO_PCOR_PTCO17_MASK                    (0x20000U)
#define GPIO_PCOR_PTCO17_SHIFT                   (17U)
/*! PTCO17 - Port Clear Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 0
 */
#define GPIO_PCOR_PTCO17(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PCOR_PTCO17_SHIFT)) & GPIO_PCOR_PTCO17_MASK)

#define GPIO_PCOR_PTCO18_MASK                    (0x40000U)
#define GPIO_PCOR_PTCO18_SHIFT                   (18U)
/*! PTCO18 - Port Clear Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 0
 */
#define GPIO_PCOR_PTCO18(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PCOR_PTCO18_SHIFT)) & GPIO_PCOR_PTCO18_MASK)

#define GPIO_PCOR_PTCO19_MASK                    (0x80000U)
#define GPIO_PCOR_PTCO19_SHIFT                   (19U)
/*! PTCO19 - Port Clear Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 0
 */
#define GPIO_PCOR_PTCO19(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PCOR_PTCO19_SHIFT)) & GPIO_PCOR_PTCO19_MASK)

#define GPIO_PCOR_PTCO20_MASK                    (0x100000U)
#define GPIO_PCOR_PTCO20_SHIFT                   (20U)
/*! PTCO20 - Port Clear Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 0
 */
#define GPIO_PCOR_PTCO20(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PCOR_PTCO20_SHIFT)) & GPIO_PCOR_PTCO20_MASK)

#define GPIO_PCOR_PTCO21_MASK                    (0x200000U)
#define GPIO_PCOR_PTCO21_SHIFT                   (21U)
/*! PTCO21 - Port Clear Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 0
 */
#define GPIO_PCOR_PTCO21(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PCOR_PTCO21_SHIFT)) & GPIO_PCOR_PTCO21_MASK)

#define GPIO_PCOR_PTCO22_MASK                    (0x400000U)
#define GPIO_PCOR_PTCO22_SHIFT                   (22U)
/*! PTCO22 - Port Clear Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 0
 */
#define GPIO_PCOR_PTCO22(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PCOR_PTCO22_SHIFT)) & GPIO_PCOR_PTCO22_MASK)

#define GPIO_PCOR_PTCO23_MASK                    (0x800000U)
#define GPIO_PCOR_PTCO23_SHIFT                   (23U)
/*! PTCO23 - Port Clear Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 0
 */
#define GPIO_PCOR_PTCO23(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PCOR_PTCO23_SHIFT)) & GPIO_PCOR_PTCO23_MASK)

#define GPIO_PCOR_PTCO24_MASK                    (0x1000000U)
#define GPIO_PCOR_PTCO24_SHIFT                   (24U)
/*! PTCO24 - Port Clear Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 0
 */
#define GPIO_PCOR_PTCO24(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PCOR_PTCO24_SHIFT)) & GPIO_PCOR_PTCO24_MASK)

#define GPIO_PCOR_PTCO25_MASK                    (0x2000000U)
#define GPIO_PCOR_PTCO25_SHIFT                   (25U)
/*! PTCO25 - Port Clear Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 0
 */
#define GPIO_PCOR_PTCO25(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PCOR_PTCO25_SHIFT)) & GPIO_PCOR_PTCO25_MASK)

#define GPIO_PCOR_PTCO26_MASK                    (0x4000000U)
#define GPIO_PCOR_PTCO26_SHIFT                   (26U)
/*! PTCO26 - Port Clear Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 0
 */
#define GPIO_PCOR_PTCO26(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PCOR_PTCO26_SHIFT)) & GPIO_PCOR_PTCO26_MASK)

#define GPIO_PCOR_PTCO27_MASK                    (0x8000000U)
#define GPIO_PCOR_PTCO27_SHIFT                   (27U)
/*! PTCO27 - Port Clear Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 0
 */
#define GPIO_PCOR_PTCO27(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PCOR_PTCO27_SHIFT)) & GPIO_PCOR_PTCO27_MASK)

#define GPIO_PCOR_PTCO28_MASK                    (0x10000000U)
#define GPIO_PCOR_PTCO28_SHIFT                   (28U)
/*! PTCO28 - Port Clear Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 0
 */
#define GPIO_PCOR_PTCO28(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PCOR_PTCO28_SHIFT)) & GPIO_PCOR_PTCO28_MASK)

#define GPIO_PCOR_PTCO29_MASK                    (0x20000000U)
#define GPIO_PCOR_PTCO29_SHIFT                   (29U)
/*! PTCO29 - Port Clear Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 0
 */
#define GPIO_PCOR_PTCO29(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PCOR_PTCO29_SHIFT)) & GPIO_PCOR_PTCO29_MASK)

#define GPIO_PCOR_PTCO30_MASK                    (0x40000000U)
#define GPIO_PCOR_PTCO30_SHIFT                   (30U)
/*! PTCO30 - Port Clear Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 0
 */
#define GPIO_PCOR_PTCO30(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PCOR_PTCO30_SHIFT)) & GPIO_PCOR_PTCO30_MASK)

#define GPIO_PCOR_PTCO31_MASK                    (0x80000000U)
#define GPIO_PCOR_PTCO31_SHIFT                   (31U)
/*! PTCO31 - Port Clear Output
 *  0b0..No change
 *  0b1..Corresponding field in PDOR becomes 0
 */
#define GPIO_PCOR_PTCO31(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PCOR_PTCO31_SHIFT)) & GPIO_PCOR_PTCO31_MASK)
/*! @} */

/*! @name PTOR - Port Toggle Output */
/*! @{ */

#define GPIO_PTOR_PTTO0_MASK                     (0x1U)
#define GPIO_PTOR_PTTO0_SHIFT                    (0U)
/*! PTTO0 - Port Toggle Output
 *  0b0..No change
 *  0b1..Set to the inverse of its current logic state
 */
#define GPIO_PTOR_PTTO0(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PTOR_PTTO0_SHIFT)) & GPIO_PTOR_PTTO0_MASK)

#define GPIO_PTOR_PTTO1_MASK                     (0x2U)
#define GPIO_PTOR_PTTO1_SHIFT                    (1U)
/*! PTTO1 - Port Toggle Output
 *  0b0..No change
 *  0b1..Set to the inverse of its current logic state
 */
#define GPIO_PTOR_PTTO1(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PTOR_PTTO1_SHIFT)) & GPIO_PTOR_PTTO1_MASK)

#define GPIO_PTOR_PTTO2_MASK                     (0x4U)
#define GPIO_PTOR_PTTO2_SHIFT                    (2U)
/*! PTTO2 - Port Toggle Output
 *  0b0..No change
 *  0b1..Set to the inverse of its current logic state
 */
#define GPIO_PTOR_PTTO2(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PTOR_PTTO2_SHIFT)) & GPIO_PTOR_PTTO2_MASK)

#define GPIO_PTOR_PTTO3_MASK                     (0x8U)
#define GPIO_PTOR_PTTO3_SHIFT                    (3U)
/*! PTTO3 - Port Toggle Output
 *  0b0..No change
 *  0b1..Set to the inverse of its current logic state
 */
#define GPIO_PTOR_PTTO3(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PTOR_PTTO3_SHIFT)) & GPIO_PTOR_PTTO3_MASK)

#define GPIO_PTOR_PTTO4_MASK                     (0x10U)
#define GPIO_PTOR_PTTO4_SHIFT                    (4U)
/*! PTTO4 - Port Toggle Output
 *  0b0..No change
 *  0b1..Set to the inverse of its current logic state
 */
#define GPIO_PTOR_PTTO4(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PTOR_PTTO4_SHIFT)) & GPIO_PTOR_PTTO4_MASK)

#define GPIO_PTOR_PTTO5_MASK                     (0x20U)
#define GPIO_PTOR_PTTO5_SHIFT                    (5U)
/*! PTTO5 - Port Toggle Output
 *  0b0..No change
 *  0b1..Set to the inverse of its current logic state
 */
#define GPIO_PTOR_PTTO5(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PTOR_PTTO5_SHIFT)) & GPIO_PTOR_PTTO5_MASK)

#define GPIO_PTOR_PTTO6_MASK                     (0x40U)
#define GPIO_PTOR_PTTO6_SHIFT                    (6U)
/*! PTTO6 - Port Toggle Output
 *  0b0..No change
 *  0b1..Set to the inverse of its current logic state
 */
#define GPIO_PTOR_PTTO6(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PTOR_PTTO6_SHIFT)) & GPIO_PTOR_PTTO6_MASK)

#define GPIO_PTOR_PTTO7_MASK                     (0x80U)
#define GPIO_PTOR_PTTO7_SHIFT                    (7U)
/*! PTTO7 - Port Toggle Output
 *  0b0..No change
 *  0b1..Set to the inverse of its current logic state
 */
#define GPIO_PTOR_PTTO7(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PTOR_PTTO7_SHIFT)) & GPIO_PTOR_PTTO7_MASK)

#define GPIO_PTOR_PTTO8_MASK                     (0x100U)
#define GPIO_PTOR_PTTO8_SHIFT                    (8U)
/*! PTTO8 - Port Toggle Output
 *  0b0..No change
 *  0b1..Set to the inverse of its current logic state
 */
#define GPIO_PTOR_PTTO8(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PTOR_PTTO8_SHIFT)) & GPIO_PTOR_PTTO8_MASK)

#define GPIO_PTOR_PTTO9_MASK                     (0x200U)
#define GPIO_PTOR_PTTO9_SHIFT                    (9U)
/*! PTTO9 - Port Toggle Output
 *  0b0..No change
 *  0b1..Set to the inverse of its current logic state
 */
#define GPIO_PTOR_PTTO9(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PTOR_PTTO9_SHIFT)) & GPIO_PTOR_PTTO9_MASK)

#define GPIO_PTOR_PTTO10_MASK                    (0x400U)
#define GPIO_PTOR_PTTO10_SHIFT                   (10U)
/*! PTTO10 - Port Toggle Output
 *  0b0..No change
 *  0b1..Set to the inverse of its current logic state
 */
#define GPIO_PTOR_PTTO10(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PTOR_PTTO10_SHIFT)) & GPIO_PTOR_PTTO10_MASK)

#define GPIO_PTOR_PTTO11_MASK                    (0x800U)
#define GPIO_PTOR_PTTO11_SHIFT                   (11U)
/*! PTTO11 - Port Toggle Output
 *  0b0..No change
 *  0b1..Set to the inverse of its current logic state
 */
#define GPIO_PTOR_PTTO11(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PTOR_PTTO11_SHIFT)) & GPIO_PTOR_PTTO11_MASK)

#define GPIO_PTOR_PTTO12_MASK                    (0x1000U)
#define GPIO_PTOR_PTTO12_SHIFT                   (12U)
/*! PTTO12 - Port Toggle Output
 *  0b0..No change
 *  0b1..Set to the inverse of its current logic state
 */
#define GPIO_PTOR_PTTO12(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PTOR_PTTO12_SHIFT)) & GPIO_PTOR_PTTO12_MASK)

#define GPIO_PTOR_PTTO13_MASK                    (0x2000U)
#define GPIO_PTOR_PTTO13_SHIFT                   (13U)
/*! PTTO13 - Port Toggle Output
 *  0b0..No change
 *  0b1..Set to the inverse of its current logic state
 */
#define GPIO_PTOR_PTTO13(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PTOR_PTTO13_SHIFT)) & GPIO_PTOR_PTTO13_MASK)

#define GPIO_PTOR_PTTO14_MASK                    (0x4000U)
#define GPIO_PTOR_PTTO14_SHIFT                   (14U)
/*! PTTO14 - Port Toggle Output
 *  0b0..No change
 *  0b1..Set to the inverse of its current logic state
 */
#define GPIO_PTOR_PTTO14(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PTOR_PTTO14_SHIFT)) & GPIO_PTOR_PTTO14_MASK)

#define GPIO_PTOR_PTTO15_MASK                    (0x8000U)
#define GPIO_PTOR_PTTO15_SHIFT                   (15U)
/*! PTTO15 - Port Toggle Output
 *  0b0..No change
 *  0b1..Set to the inverse of its current logic state
 */
#define GPIO_PTOR_PTTO15(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PTOR_PTTO15_SHIFT)) & GPIO_PTOR_PTTO15_MASK)

#define GPIO_PTOR_PTTO16_MASK                    (0x10000U)
#define GPIO_PTOR_PTTO16_SHIFT                   (16U)
/*! PTTO16 - Port Toggle Output
 *  0b0..No change
 *  0b1..Set to the inverse of its current logic state
 */
#define GPIO_PTOR_PTTO16(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PTOR_PTTO16_SHIFT)) & GPIO_PTOR_PTTO16_MASK)

#define GPIO_PTOR_PTTO17_MASK                    (0x20000U)
#define GPIO_PTOR_PTTO17_SHIFT                   (17U)
/*! PTTO17 - Port Toggle Output
 *  0b0..No change
 *  0b1..Set to the inverse of its current logic state
 */
#define GPIO_PTOR_PTTO17(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PTOR_PTTO17_SHIFT)) & GPIO_PTOR_PTTO17_MASK)

#define GPIO_PTOR_PTTO18_MASK                    (0x40000U)
#define GPIO_PTOR_PTTO18_SHIFT                   (18U)
/*! PTTO18 - Port Toggle Output
 *  0b0..No change
 *  0b1..Set to the inverse of its current logic state
 */
#define GPIO_PTOR_PTTO18(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PTOR_PTTO18_SHIFT)) & GPIO_PTOR_PTTO18_MASK)

#define GPIO_PTOR_PTTO19_MASK                    (0x80000U)
#define GPIO_PTOR_PTTO19_SHIFT                   (19U)
/*! PTTO19 - Port Toggle Output
 *  0b0..No change
 *  0b1..Set to the inverse of its current logic state
 */
#define GPIO_PTOR_PTTO19(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PTOR_PTTO19_SHIFT)) & GPIO_PTOR_PTTO19_MASK)

#define GPIO_PTOR_PTTO20_MASK                    (0x100000U)
#define GPIO_PTOR_PTTO20_SHIFT                   (20U)
/*! PTTO20 - Port Toggle Output
 *  0b0..No change
 *  0b1..Set to the inverse of its current logic state
 */
#define GPIO_PTOR_PTTO20(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PTOR_PTTO20_SHIFT)) & GPIO_PTOR_PTTO20_MASK)

#define GPIO_PTOR_PTTO21_MASK                    (0x200000U)
#define GPIO_PTOR_PTTO21_SHIFT                   (21U)
/*! PTTO21 - Port Toggle Output
 *  0b0..No change
 *  0b1..Set to the inverse of its current logic state
 */
#define GPIO_PTOR_PTTO21(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PTOR_PTTO21_SHIFT)) & GPIO_PTOR_PTTO21_MASK)

#define GPIO_PTOR_PTTO22_MASK                    (0x400000U)
#define GPIO_PTOR_PTTO22_SHIFT                   (22U)
/*! PTTO22 - Port Toggle Output
 *  0b0..No change
 *  0b1..Set to the inverse of its current logic state
 */
#define GPIO_PTOR_PTTO22(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PTOR_PTTO22_SHIFT)) & GPIO_PTOR_PTTO22_MASK)

#define GPIO_PTOR_PTTO23_MASK                    (0x800000U)
#define GPIO_PTOR_PTTO23_SHIFT                   (23U)
/*! PTTO23 - Port Toggle Output
 *  0b0..No change
 *  0b1..Set to the inverse of its current logic state
 */
#define GPIO_PTOR_PTTO23(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PTOR_PTTO23_SHIFT)) & GPIO_PTOR_PTTO23_MASK)

#define GPIO_PTOR_PTTO24_MASK                    (0x1000000U)
#define GPIO_PTOR_PTTO24_SHIFT                   (24U)
/*! PTTO24 - Port Toggle Output
 *  0b0..No change
 *  0b1..Set to the inverse of its current logic state
 */
#define GPIO_PTOR_PTTO24(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PTOR_PTTO24_SHIFT)) & GPIO_PTOR_PTTO24_MASK)

#define GPIO_PTOR_PTTO25_MASK                    (0x2000000U)
#define GPIO_PTOR_PTTO25_SHIFT                   (25U)
/*! PTTO25 - Port Toggle Output
 *  0b0..No change
 *  0b1..Set to the inverse of its current logic state
 */
#define GPIO_PTOR_PTTO25(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PTOR_PTTO25_SHIFT)) & GPIO_PTOR_PTTO25_MASK)

#define GPIO_PTOR_PTTO26_MASK                    (0x4000000U)
#define GPIO_PTOR_PTTO26_SHIFT                   (26U)
/*! PTTO26 - Port Toggle Output
 *  0b0..No change
 *  0b1..Set to the inverse of its current logic state
 */
#define GPIO_PTOR_PTTO26(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PTOR_PTTO26_SHIFT)) & GPIO_PTOR_PTTO26_MASK)

#define GPIO_PTOR_PTTO27_MASK                    (0x8000000U)
#define GPIO_PTOR_PTTO27_SHIFT                   (27U)
/*! PTTO27 - Port Toggle Output
 *  0b0..No change
 *  0b1..Set to the inverse of its current logic state
 */
#define GPIO_PTOR_PTTO27(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PTOR_PTTO27_SHIFT)) & GPIO_PTOR_PTTO27_MASK)

#define GPIO_PTOR_PTTO28_MASK                    (0x10000000U)
#define GPIO_PTOR_PTTO28_SHIFT                   (28U)
/*! PTTO28 - Port Toggle Output
 *  0b0..No change
 *  0b1..Set to the inverse of its current logic state
 */
#define GPIO_PTOR_PTTO28(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PTOR_PTTO28_SHIFT)) & GPIO_PTOR_PTTO28_MASK)

#define GPIO_PTOR_PTTO29_MASK                    (0x20000000U)
#define GPIO_PTOR_PTTO29_SHIFT                   (29U)
/*! PTTO29 - Port Toggle Output
 *  0b0..No change
 *  0b1..Set to the inverse of its current logic state
 */
#define GPIO_PTOR_PTTO29(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PTOR_PTTO29_SHIFT)) & GPIO_PTOR_PTTO29_MASK)

#define GPIO_PTOR_PTTO30_MASK                    (0x40000000U)
#define GPIO_PTOR_PTTO30_SHIFT                   (30U)
/*! PTTO30 - Port Toggle Output
 *  0b0..No change
 *  0b1..Set to the inverse of its current logic state
 */
#define GPIO_PTOR_PTTO30(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PTOR_PTTO30_SHIFT)) & GPIO_PTOR_PTTO30_MASK)

#define GPIO_PTOR_PTTO31_MASK                    (0x80000000U)
#define GPIO_PTOR_PTTO31_SHIFT                   (31U)
/*! PTTO31 - Port Toggle Output
 *  0b0..No change
 *  0b1..Set to the inverse of its current logic state
 */
#define GPIO_PTOR_PTTO31(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_PTOR_PTTO31_SHIFT)) & GPIO_PTOR_PTTO31_MASK)
/*! @} */

/*! @name PDIR - Port Data Input */
/*! @{ */

#define GPIO_PDIR_PDI0_MASK                      (0x1U)
#define GPIO_PDIR_PDI0_SHIFT                     (0U)
/*! PDI0 - Port Data Input
 *  0b0..Logic 0
 *  0b1..Logic 1
 */
#define GPIO_PDIR_PDI0(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_PDIR_PDI0_SHIFT)) & GPIO_PDIR_PDI0_MASK)

#define GPIO_PDIR_PDI1_MASK                      (0x2U)
#define GPIO_PDIR_PDI1_SHIFT                     (1U)
/*! PDI1 - Port Data Input
 *  0b0..Logic 0
 *  0b1..Logic 1
 */
#define GPIO_PDIR_PDI1(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_PDIR_PDI1_SHIFT)) & GPIO_PDIR_PDI1_MASK)

#define GPIO_PDIR_PDI2_MASK                      (0x4U)
#define GPIO_PDIR_PDI2_SHIFT                     (2U)
/*! PDI2 - Port Data Input
 *  0b0..Logic 0
 *  0b1..Logic 1
 */
#define GPIO_PDIR_PDI2(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_PDIR_PDI2_SHIFT)) & GPIO_PDIR_PDI2_MASK)

#define GPIO_PDIR_PDI3_MASK                      (0x8U)
#define GPIO_PDIR_PDI3_SHIFT                     (3U)
/*! PDI3 - Port Data Input
 *  0b0..Logic 0
 *  0b1..Logic 1
 */
#define GPIO_PDIR_PDI3(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_PDIR_PDI3_SHIFT)) & GPIO_PDIR_PDI3_MASK)

#define GPIO_PDIR_PDI4_MASK                      (0x10U)
#define GPIO_PDIR_PDI4_SHIFT                     (4U)
/*! PDI4 - Port Data Input
 *  0b0..Logic 0
 *  0b1..Logic 1
 */
#define GPIO_PDIR_PDI4(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_PDIR_PDI4_SHIFT)) & GPIO_PDIR_PDI4_MASK)

#define GPIO_PDIR_PDI5_MASK                      (0x20U)
#define GPIO_PDIR_PDI5_SHIFT                     (5U)
/*! PDI5 - Port Data Input
 *  0b0..Logic 0
 *  0b1..Logic 1
 */
#define GPIO_PDIR_PDI5(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_PDIR_PDI5_SHIFT)) & GPIO_PDIR_PDI5_MASK)

#define GPIO_PDIR_PDI6_MASK                      (0x40U)
#define GPIO_PDIR_PDI6_SHIFT                     (6U)
/*! PDI6 - Port Data Input
 *  0b0..Logic 0
 *  0b1..Logic 1
 */
#define GPIO_PDIR_PDI6(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_PDIR_PDI6_SHIFT)) & GPIO_PDIR_PDI6_MASK)

#define GPIO_PDIR_PDI7_MASK                      (0x80U)
#define GPIO_PDIR_PDI7_SHIFT                     (7U)
/*! PDI7 - Port Data Input
 *  0b0..Logic 0
 *  0b1..Logic 1
 */
#define GPIO_PDIR_PDI7(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_PDIR_PDI7_SHIFT)) & GPIO_PDIR_PDI7_MASK)

#define GPIO_PDIR_PDI8_MASK                      (0x100U)
#define GPIO_PDIR_PDI8_SHIFT                     (8U)
/*! PDI8 - Port Data Input
 *  0b0..Logic 0
 *  0b1..Logic 1
 */
#define GPIO_PDIR_PDI8(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_PDIR_PDI8_SHIFT)) & GPIO_PDIR_PDI8_MASK)

#define GPIO_PDIR_PDI9_MASK                      (0x200U)
#define GPIO_PDIR_PDI9_SHIFT                     (9U)
/*! PDI9 - Port Data Input
 *  0b0..Logic 0
 *  0b1..Logic 1
 */
#define GPIO_PDIR_PDI9(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_PDIR_PDI9_SHIFT)) & GPIO_PDIR_PDI9_MASK)

#define GPIO_PDIR_PDI10_MASK                     (0x400U)
#define GPIO_PDIR_PDI10_SHIFT                    (10U)
/*! PDI10 - Port Data Input
 *  0b0..Logic 0
 *  0b1..Logic 1
 */
#define GPIO_PDIR_PDI10(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDIR_PDI10_SHIFT)) & GPIO_PDIR_PDI10_MASK)

#define GPIO_PDIR_PDI11_MASK                     (0x800U)
#define GPIO_PDIR_PDI11_SHIFT                    (11U)
/*! PDI11 - Port Data Input
 *  0b0..Logic 0
 *  0b1..Logic 1
 */
#define GPIO_PDIR_PDI11(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDIR_PDI11_SHIFT)) & GPIO_PDIR_PDI11_MASK)

#define GPIO_PDIR_PDI12_MASK                     (0x1000U)
#define GPIO_PDIR_PDI12_SHIFT                    (12U)
/*! PDI12 - Port Data Input
 *  0b0..Logic 0
 *  0b1..Logic 1
 */
#define GPIO_PDIR_PDI12(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDIR_PDI12_SHIFT)) & GPIO_PDIR_PDI12_MASK)

#define GPIO_PDIR_PDI13_MASK                     (0x2000U)
#define GPIO_PDIR_PDI13_SHIFT                    (13U)
/*! PDI13 - Port Data Input
 *  0b0..Logic 0
 *  0b1..Logic 1
 */
#define GPIO_PDIR_PDI13(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDIR_PDI13_SHIFT)) & GPIO_PDIR_PDI13_MASK)

#define GPIO_PDIR_PDI14_MASK                     (0x4000U)
#define GPIO_PDIR_PDI14_SHIFT                    (14U)
/*! PDI14 - Port Data Input
 *  0b0..Logic 0
 *  0b1..Logic 1
 */
#define GPIO_PDIR_PDI14(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDIR_PDI14_SHIFT)) & GPIO_PDIR_PDI14_MASK)

#define GPIO_PDIR_PDI15_MASK                     (0x8000U)
#define GPIO_PDIR_PDI15_SHIFT                    (15U)
/*! PDI15 - Port Data Input
 *  0b0..Logic 0
 *  0b1..Logic 1
 */
#define GPIO_PDIR_PDI15(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDIR_PDI15_SHIFT)) & GPIO_PDIR_PDI15_MASK)

#define GPIO_PDIR_PDI16_MASK                     (0x10000U)
#define GPIO_PDIR_PDI16_SHIFT                    (16U)
/*! PDI16 - Port Data Input
 *  0b0..Logic 0
 *  0b1..Logic 1
 */
#define GPIO_PDIR_PDI16(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDIR_PDI16_SHIFT)) & GPIO_PDIR_PDI16_MASK)

#define GPIO_PDIR_PDI17_MASK                     (0x20000U)
#define GPIO_PDIR_PDI17_SHIFT                    (17U)
/*! PDI17 - Port Data Input
 *  0b0..Logic 0
 *  0b1..Logic 1
 */
#define GPIO_PDIR_PDI17(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDIR_PDI17_SHIFT)) & GPIO_PDIR_PDI17_MASK)

#define GPIO_PDIR_PDI18_MASK                     (0x40000U)
#define GPIO_PDIR_PDI18_SHIFT                    (18U)
/*! PDI18 - Port Data Input
 *  0b0..Logic 0
 *  0b1..Logic 1
 */
#define GPIO_PDIR_PDI18(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDIR_PDI18_SHIFT)) & GPIO_PDIR_PDI18_MASK)

#define GPIO_PDIR_PDI19_MASK                     (0x80000U)
#define GPIO_PDIR_PDI19_SHIFT                    (19U)
/*! PDI19 - Port Data Input
 *  0b0..Logic 0
 *  0b1..Logic 1
 */
#define GPIO_PDIR_PDI19(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDIR_PDI19_SHIFT)) & GPIO_PDIR_PDI19_MASK)

#define GPIO_PDIR_PDI20_MASK                     (0x100000U)
#define GPIO_PDIR_PDI20_SHIFT                    (20U)
/*! PDI20 - Port Data Input
 *  0b0..Logic 0
 *  0b1..Logic 1
 */
#define GPIO_PDIR_PDI20(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDIR_PDI20_SHIFT)) & GPIO_PDIR_PDI20_MASK)

#define GPIO_PDIR_PDI21_MASK                     (0x200000U)
#define GPIO_PDIR_PDI21_SHIFT                    (21U)
/*! PDI21 - Port Data Input
 *  0b0..Logic 0
 *  0b1..Logic 1
 */
#define GPIO_PDIR_PDI21(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDIR_PDI21_SHIFT)) & GPIO_PDIR_PDI21_MASK)

#define GPIO_PDIR_PDI22_MASK                     (0x400000U)
#define GPIO_PDIR_PDI22_SHIFT                    (22U)
/*! PDI22 - Port Data Input
 *  0b0..Logic 0
 *  0b1..Logic 1
 */
#define GPIO_PDIR_PDI22(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDIR_PDI22_SHIFT)) & GPIO_PDIR_PDI22_MASK)

#define GPIO_PDIR_PDI23_MASK                     (0x800000U)
#define GPIO_PDIR_PDI23_SHIFT                    (23U)
/*! PDI23 - Port Data Input
 *  0b0..Logic 0
 *  0b1..Logic 1
 */
#define GPIO_PDIR_PDI23(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDIR_PDI23_SHIFT)) & GPIO_PDIR_PDI23_MASK)

#define GPIO_PDIR_PDI24_MASK                     (0x1000000U)
#define GPIO_PDIR_PDI24_SHIFT                    (24U)
/*! PDI24 - Port Data Input
 *  0b0..Logic 0
 *  0b1..Logic 1
 */
#define GPIO_PDIR_PDI24(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDIR_PDI24_SHIFT)) & GPIO_PDIR_PDI24_MASK)

#define GPIO_PDIR_PDI25_MASK                     (0x2000000U)
#define GPIO_PDIR_PDI25_SHIFT                    (25U)
/*! PDI25 - Port Data Input
 *  0b0..Logic 0
 *  0b1..Logic 1
 */
#define GPIO_PDIR_PDI25(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDIR_PDI25_SHIFT)) & GPIO_PDIR_PDI25_MASK)

#define GPIO_PDIR_PDI26_MASK                     (0x4000000U)
#define GPIO_PDIR_PDI26_SHIFT                    (26U)
/*! PDI26 - Port Data Input
 *  0b0..Logic 0
 *  0b1..Logic 1
 */
#define GPIO_PDIR_PDI26(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDIR_PDI26_SHIFT)) & GPIO_PDIR_PDI26_MASK)

#define GPIO_PDIR_PDI27_MASK                     (0x8000000U)
#define GPIO_PDIR_PDI27_SHIFT                    (27U)
/*! PDI27 - Port Data Input
 *  0b0..Logic 0
 *  0b1..Logic 1
 */
#define GPIO_PDIR_PDI27(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDIR_PDI27_SHIFT)) & GPIO_PDIR_PDI27_MASK)

#define GPIO_PDIR_PDI28_MASK                     (0x10000000U)
#define GPIO_PDIR_PDI28_SHIFT                    (28U)
/*! PDI28 - Port Data Input
 *  0b0..Logic 0
 *  0b1..Logic 1
 */
#define GPIO_PDIR_PDI28(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDIR_PDI28_SHIFT)) & GPIO_PDIR_PDI28_MASK)

#define GPIO_PDIR_PDI29_MASK                     (0x20000000U)
#define GPIO_PDIR_PDI29_SHIFT                    (29U)
/*! PDI29 - Port Data Input
 *  0b0..Logic 0
 *  0b1..Logic 1
 */
#define GPIO_PDIR_PDI29(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDIR_PDI29_SHIFT)) & GPIO_PDIR_PDI29_MASK)

#define GPIO_PDIR_PDI30_MASK                     (0x40000000U)
#define GPIO_PDIR_PDI30_SHIFT                    (30U)
/*! PDI30 - Port Data Input
 *  0b0..Logic 0
 *  0b1..Logic 1
 */
#define GPIO_PDIR_PDI30(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDIR_PDI30_SHIFT)) & GPIO_PDIR_PDI30_MASK)

#define GPIO_PDIR_PDI31_MASK                     (0x80000000U)
#define GPIO_PDIR_PDI31_SHIFT                    (31U)
/*! PDI31 - Port Data Input
 *  0b0..Logic 0
 *  0b1..Logic 1
 */
#define GPIO_PDIR_PDI31(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDIR_PDI31_SHIFT)) & GPIO_PDIR_PDI31_MASK)
/*! @} */

/*! @name PDDR - Port Data Direction */
/*! @{ */

#define GPIO_PDDR_PDD0_MASK                      (0x1U)
#define GPIO_PDDR_PDD0_SHIFT                     (0U)
/*! PDD0 - Port Data Direction
 *  0b0..Input
 *  0b1..Output
 */
#define GPIO_PDDR_PDD0(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_PDDR_PDD0_SHIFT)) & GPIO_PDDR_PDD0_MASK)

#define GPIO_PDDR_PDD1_MASK                      (0x2U)
#define GPIO_PDDR_PDD1_SHIFT                     (1U)
/*! PDD1 - Port Data Direction
 *  0b0..Input
 *  0b1..Output
 */
#define GPIO_PDDR_PDD1(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_PDDR_PDD1_SHIFT)) & GPIO_PDDR_PDD1_MASK)

#define GPIO_PDDR_PDD2_MASK                      (0x4U)
#define GPIO_PDDR_PDD2_SHIFT                     (2U)
/*! PDD2 - Port Data Direction
 *  0b0..Input
 *  0b1..Output
 */
#define GPIO_PDDR_PDD2(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_PDDR_PDD2_SHIFT)) & GPIO_PDDR_PDD2_MASK)

#define GPIO_PDDR_PDD3_MASK                      (0x8U)
#define GPIO_PDDR_PDD3_SHIFT                     (3U)
/*! PDD3 - Port Data Direction
 *  0b0..Input
 *  0b1..Output
 */
#define GPIO_PDDR_PDD3(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_PDDR_PDD3_SHIFT)) & GPIO_PDDR_PDD3_MASK)

#define GPIO_PDDR_PDD4_MASK                      (0x10U)
#define GPIO_PDDR_PDD4_SHIFT                     (4U)
/*! PDD4 - Port Data Direction
 *  0b0..Input
 *  0b1..Output
 */
#define GPIO_PDDR_PDD4(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_PDDR_PDD4_SHIFT)) & GPIO_PDDR_PDD4_MASK)

#define GPIO_PDDR_PDD5_MASK                      (0x20U)
#define GPIO_PDDR_PDD5_SHIFT                     (5U)
/*! PDD5 - Port Data Direction
 *  0b0..Input
 *  0b1..Output
 */
#define GPIO_PDDR_PDD5(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_PDDR_PDD5_SHIFT)) & GPIO_PDDR_PDD5_MASK)

#define GPIO_PDDR_PDD6_MASK                      (0x40U)
#define GPIO_PDDR_PDD6_SHIFT                     (6U)
/*! PDD6 - Port Data Direction
 *  0b0..Input
 *  0b1..Output
 */
#define GPIO_PDDR_PDD6(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_PDDR_PDD6_SHIFT)) & GPIO_PDDR_PDD6_MASK)

#define GPIO_PDDR_PDD7_MASK                      (0x80U)
#define GPIO_PDDR_PDD7_SHIFT                     (7U)
/*! PDD7 - Port Data Direction
 *  0b0..Input
 *  0b1..Output
 */
#define GPIO_PDDR_PDD7(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_PDDR_PDD7_SHIFT)) & GPIO_PDDR_PDD7_MASK)

#define GPIO_PDDR_PDD8_MASK                      (0x100U)
#define GPIO_PDDR_PDD8_SHIFT                     (8U)
/*! PDD8 - Port Data Direction
 *  0b0..Input
 *  0b1..Output
 */
#define GPIO_PDDR_PDD8(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_PDDR_PDD8_SHIFT)) & GPIO_PDDR_PDD8_MASK)

#define GPIO_PDDR_PDD9_MASK                      (0x200U)
#define GPIO_PDDR_PDD9_SHIFT                     (9U)
/*! PDD9 - Port Data Direction
 *  0b0..Input
 *  0b1..Output
 */
#define GPIO_PDDR_PDD9(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_PDDR_PDD9_SHIFT)) & GPIO_PDDR_PDD9_MASK)

#define GPIO_PDDR_PDD10_MASK                     (0x400U)
#define GPIO_PDDR_PDD10_SHIFT                    (10U)
/*! PDD10 - Port Data Direction
 *  0b0..Input
 *  0b1..Output
 */
#define GPIO_PDDR_PDD10(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDDR_PDD10_SHIFT)) & GPIO_PDDR_PDD10_MASK)

#define GPIO_PDDR_PDD11_MASK                     (0x800U)
#define GPIO_PDDR_PDD11_SHIFT                    (11U)
/*! PDD11 - Port Data Direction
 *  0b0..Input
 *  0b1..Output
 */
#define GPIO_PDDR_PDD11(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDDR_PDD11_SHIFT)) & GPIO_PDDR_PDD11_MASK)

#define GPIO_PDDR_PDD12_MASK                     (0x1000U)
#define GPIO_PDDR_PDD12_SHIFT                    (12U)
/*! PDD12 - Port Data Direction
 *  0b0..Input
 *  0b1..Output
 */
#define GPIO_PDDR_PDD12(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDDR_PDD12_SHIFT)) & GPIO_PDDR_PDD12_MASK)

#define GPIO_PDDR_PDD13_MASK                     (0x2000U)
#define GPIO_PDDR_PDD13_SHIFT                    (13U)
/*! PDD13 - Port Data Direction
 *  0b0..Input
 *  0b1..Output
 */
#define GPIO_PDDR_PDD13(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDDR_PDD13_SHIFT)) & GPIO_PDDR_PDD13_MASK)

#define GPIO_PDDR_PDD14_MASK                     (0x4000U)
#define GPIO_PDDR_PDD14_SHIFT                    (14U)
/*! PDD14 - Port Data Direction
 *  0b0..Input
 *  0b1..Output
 */
#define GPIO_PDDR_PDD14(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDDR_PDD14_SHIFT)) & GPIO_PDDR_PDD14_MASK)

#define GPIO_PDDR_PDD15_MASK                     (0x8000U)
#define GPIO_PDDR_PDD15_SHIFT                    (15U)
/*! PDD15 - Port Data Direction
 *  0b0..Input
 *  0b1..Output
 */
#define GPIO_PDDR_PDD15(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDDR_PDD15_SHIFT)) & GPIO_PDDR_PDD15_MASK)

#define GPIO_PDDR_PDD16_MASK                     (0x10000U)
#define GPIO_PDDR_PDD16_SHIFT                    (16U)
/*! PDD16 - Port Data Direction
 *  0b0..Input
 *  0b1..Output
 */
#define GPIO_PDDR_PDD16(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDDR_PDD16_SHIFT)) & GPIO_PDDR_PDD16_MASK)

#define GPIO_PDDR_PDD17_MASK                     (0x20000U)
#define GPIO_PDDR_PDD17_SHIFT                    (17U)
/*! PDD17 - Port Data Direction
 *  0b0..Input
 *  0b1..Output
 */
#define GPIO_PDDR_PDD17(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDDR_PDD17_SHIFT)) & GPIO_PDDR_PDD17_MASK)

#define GPIO_PDDR_PDD18_MASK                     (0x40000U)
#define GPIO_PDDR_PDD18_SHIFT                    (18U)
/*! PDD18 - Port Data Direction
 *  0b0..Input
 *  0b1..Output
 */
#define GPIO_PDDR_PDD18(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDDR_PDD18_SHIFT)) & GPIO_PDDR_PDD18_MASK)

#define GPIO_PDDR_PDD19_MASK                     (0x80000U)
#define GPIO_PDDR_PDD19_SHIFT                    (19U)
/*! PDD19 - Port Data Direction
 *  0b0..Input
 *  0b1..Output
 */
#define GPIO_PDDR_PDD19(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDDR_PDD19_SHIFT)) & GPIO_PDDR_PDD19_MASK)

#define GPIO_PDDR_PDD20_MASK                     (0x100000U)
#define GPIO_PDDR_PDD20_SHIFT                    (20U)
/*! PDD20 - Port Data Direction
 *  0b0..Input
 *  0b1..Output
 */
#define GPIO_PDDR_PDD20(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDDR_PDD20_SHIFT)) & GPIO_PDDR_PDD20_MASK)

#define GPIO_PDDR_PDD21_MASK                     (0x200000U)
#define GPIO_PDDR_PDD21_SHIFT                    (21U)
/*! PDD21 - Port Data Direction
 *  0b0..Input
 *  0b1..Output
 */
#define GPIO_PDDR_PDD21(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDDR_PDD21_SHIFT)) & GPIO_PDDR_PDD21_MASK)

#define GPIO_PDDR_PDD22_MASK                     (0x400000U)
#define GPIO_PDDR_PDD22_SHIFT                    (22U)
/*! PDD22 - Port Data Direction
 *  0b0..Input
 *  0b1..Output
 */
#define GPIO_PDDR_PDD22(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDDR_PDD22_SHIFT)) & GPIO_PDDR_PDD22_MASK)

#define GPIO_PDDR_PDD23_MASK                     (0x800000U)
#define GPIO_PDDR_PDD23_SHIFT                    (23U)
/*! PDD23 - Port Data Direction
 *  0b0..Input
 *  0b1..Output
 */
#define GPIO_PDDR_PDD23(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDDR_PDD23_SHIFT)) & GPIO_PDDR_PDD23_MASK)

#define GPIO_PDDR_PDD24_MASK                     (0x1000000U)
#define GPIO_PDDR_PDD24_SHIFT                    (24U)
/*! PDD24 - Port Data Direction
 *  0b0..Input
 *  0b1..Output
 */
#define GPIO_PDDR_PDD24(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDDR_PDD24_SHIFT)) & GPIO_PDDR_PDD24_MASK)

#define GPIO_PDDR_PDD25_MASK                     (0x2000000U)
#define GPIO_PDDR_PDD25_SHIFT                    (25U)
/*! PDD25 - Port Data Direction
 *  0b0..Input
 *  0b1..Output
 */
#define GPIO_PDDR_PDD25(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDDR_PDD25_SHIFT)) & GPIO_PDDR_PDD25_MASK)

#define GPIO_PDDR_PDD26_MASK                     (0x4000000U)
#define GPIO_PDDR_PDD26_SHIFT                    (26U)
/*! PDD26 - Port Data Direction
 *  0b0..Input
 *  0b1..Output
 */
#define GPIO_PDDR_PDD26(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDDR_PDD26_SHIFT)) & GPIO_PDDR_PDD26_MASK)

#define GPIO_PDDR_PDD27_MASK                     (0x8000000U)
#define GPIO_PDDR_PDD27_SHIFT                    (27U)
/*! PDD27 - Port Data Direction
 *  0b0..Input
 *  0b1..Output
 */
#define GPIO_PDDR_PDD27(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDDR_PDD27_SHIFT)) & GPIO_PDDR_PDD27_MASK)

#define GPIO_PDDR_PDD28_MASK                     (0x10000000U)
#define GPIO_PDDR_PDD28_SHIFT                    (28U)
/*! PDD28 - Port Data Direction
 *  0b0..Input
 *  0b1..Output
 */
#define GPIO_PDDR_PDD28(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDDR_PDD28_SHIFT)) & GPIO_PDDR_PDD28_MASK)

#define GPIO_PDDR_PDD29_MASK                     (0x20000000U)
#define GPIO_PDDR_PDD29_SHIFT                    (29U)
/*! PDD29 - Port Data Direction
 *  0b0..Input
 *  0b1..Output
 */
#define GPIO_PDDR_PDD29(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDDR_PDD29_SHIFT)) & GPIO_PDDR_PDD29_MASK)

#define GPIO_PDDR_PDD30_MASK                     (0x40000000U)
#define GPIO_PDDR_PDD30_SHIFT                    (30U)
/*! PDD30 - Port Data Direction
 *  0b0..Input
 *  0b1..Output
 */
#define GPIO_PDDR_PDD30(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDDR_PDD30_SHIFT)) & GPIO_PDDR_PDD30_MASK)

#define GPIO_PDDR_PDD31_MASK                     (0x80000000U)
#define GPIO_PDDR_PDD31_SHIFT                    (31U)
/*! PDD31 - Port Data Direction
 *  0b0..Input
 *  0b1..Output
 */
#define GPIO_PDDR_PDD31(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PDDR_PDD31_SHIFT)) & GPIO_PDDR_PDD31_MASK)
/*! @} */

/*! @name PIDR - Port Input Disable */
/*! @{ */

#define GPIO_PIDR_PID0_MASK                      (0x1U)
#define GPIO_PIDR_PID0_SHIFT                     (0U)
/*! PID0 - Port Input Disable
 *  0b0..Configured for general-purpose input
 *  0b1..Disabled for general-purpose input
 */
#define GPIO_PIDR_PID0(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_PIDR_PID0_SHIFT)) & GPIO_PIDR_PID0_MASK)

#define GPIO_PIDR_PID1_MASK                      (0x2U)
#define GPIO_PIDR_PID1_SHIFT                     (1U)
/*! PID1 - Port Input Disable
 *  0b0..Configured for general-purpose input
 *  0b1..Disabled for general-purpose input
 */
#define GPIO_PIDR_PID1(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_PIDR_PID1_SHIFT)) & GPIO_PIDR_PID1_MASK)

#define GPIO_PIDR_PID2_MASK                      (0x4U)
#define GPIO_PIDR_PID2_SHIFT                     (2U)
/*! PID2 - Port Input Disable
 *  0b0..Configured for general-purpose input
 *  0b1..Disabled for general-purpose input
 */
#define GPIO_PIDR_PID2(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_PIDR_PID2_SHIFT)) & GPIO_PIDR_PID2_MASK)

#define GPIO_PIDR_PID3_MASK                      (0x8U)
#define GPIO_PIDR_PID3_SHIFT                     (3U)
/*! PID3 - Port Input Disable
 *  0b0..Configured for general-purpose input
 *  0b1..Disabled for general-purpose input
 */
#define GPIO_PIDR_PID3(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_PIDR_PID3_SHIFT)) & GPIO_PIDR_PID3_MASK)

#define GPIO_PIDR_PID4_MASK                      (0x10U)
#define GPIO_PIDR_PID4_SHIFT                     (4U)
/*! PID4 - Port Input Disable
 *  0b0..Configured for general-purpose input
 *  0b1..Disabled for general-purpose input
 */
#define GPIO_PIDR_PID4(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_PIDR_PID4_SHIFT)) & GPIO_PIDR_PID4_MASK)

#define GPIO_PIDR_PID5_MASK                      (0x20U)
#define GPIO_PIDR_PID5_SHIFT                     (5U)
/*! PID5 - Port Input Disable
 *  0b0..Configured for general-purpose input
 *  0b1..Disabled for general-purpose input
 */
#define GPIO_PIDR_PID5(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_PIDR_PID5_SHIFT)) & GPIO_PIDR_PID5_MASK)

#define GPIO_PIDR_PID6_MASK                      (0x40U)
#define GPIO_PIDR_PID6_SHIFT                     (6U)
/*! PID6 - Port Input Disable
 *  0b0..Configured for general-purpose input
 *  0b1..Disabled for general-purpose input
 */
#define GPIO_PIDR_PID6(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_PIDR_PID6_SHIFT)) & GPIO_PIDR_PID6_MASK)

#define GPIO_PIDR_PID7_MASK                      (0x80U)
#define GPIO_PIDR_PID7_SHIFT                     (7U)
/*! PID7 - Port Input Disable
 *  0b0..Configured for general-purpose input
 *  0b1..Disabled for general-purpose input
 */
#define GPIO_PIDR_PID7(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_PIDR_PID7_SHIFT)) & GPIO_PIDR_PID7_MASK)

#define GPIO_PIDR_PID8_MASK                      (0x100U)
#define GPIO_PIDR_PID8_SHIFT                     (8U)
/*! PID8 - Port Input Disable
 *  0b0..Configured for general-purpose input
 *  0b1..Disabled for general-purpose input
 */
#define GPIO_PIDR_PID8(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_PIDR_PID8_SHIFT)) & GPIO_PIDR_PID8_MASK)

#define GPIO_PIDR_PID9_MASK                      (0x200U)
#define GPIO_PIDR_PID9_SHIFT                     (9U)
/*! PID9 - Port Input Disable
 *  0b0..Configured for general-purpose input
 *  0b1..Disabled for general-purpose input
 */
#define GPIO_PIDR_PID9(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_PIDR_PID9_SHIFT)) & GPIO_PIDR_PID9_MASK)

#define GPIO_PIDR_PID10_MASK                     (0x400U)
#define GPIO_PIDR_PID10_SHIFT                    (10U)
/*! PID10 - Port Input Disable
 *  0b0..Configured for general-purpose input
 *  0b1..Disabled for general-purpose input
 */
#define GPIO_PIDR_PID10(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PIDR_PID10_SHIFT)) & GPIO_PIDR_PID10_MASK)

#define GPIO_PIDR_PID11_MASK                     (0x800U)
#define GPIO_PIDR_PID11_SHIFT                    (11U)
/*! PID11 - Port Input Disable
 *  0b0..Configured for general-purpose input
 *  0b1..Disabled for general-purpose input
 */
#define GPIO_PIDR_PID11(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PIDR_PID11_SHIFT)) & GPIO_PIDR_PID11_MASK)

#define GPIO_PIDR_PID12_MASK                     (0x1000U)
#define GPIO_PIDR_PID12_SHIFT                    (12U)
/*! PID12 - Port Input Disable
 *  0b0..Configured for general-purpose input
 *  0b1..Disabled for general-purpose input
 */
#define GPIO_PIDR_PID12(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PIDR_PID12_SHIFT)) & GPIO_PIDR_PID12_MASK)

#define GPIO_PIDR_PID13_MASK                     (0x2000U)
#define GPIO_PIDR_PID13_SHIFT                    (13U)
/*! PID13 - Port Input Disable
 *  0b0..Configured for general-purpose input
 *  0b1..Disabled for general-purpose input
 */
#define GPIO_PIDR_PID13(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PIDR_PID13_SHIFT)) & GPIO_PIDR_PID13_MASK)

#define GPIO_PIDR_PID14_MASK                     (0x4000U)
#define GPIO_PIDR_PID14_SHIFT                    (14U)
/*! PID14 - Port Input Disable
 *  0b0..Configured for general-purpose input
 *  0b1..Disabled for general-purpose input
 */
#define GPIO_PIDR_PID14(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PIDR_PID14_SHIFT)) & GPIO_PIDR_PID14_MASK)

#define GPIO_PIDR_PID15_MASK                     (0x8000U)
#define GPIO_PIDR_PID15_SHIFT                    (15U)
/*! PID15 - Port Input Disable
 *  0b0..Configured for general-purpose input
 *  0b1..Disabled for general-purpose input
 */
#define GPIO_PIDR_PID15(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PIDR_PID15_SHIFT)) & GPIO_PIDR_PID15_MASK)

#define GPIO_PIDR_PID16_MASK                     (0x10000U)
#define GPIO_PIDR_PID16_SHIFT                    (16U)
/*! PID16 - Port Input Disable
 *  0b0..Configured for general-purpose input
 *  0b1..Disabled for general-purpose input
 */
#define GPIO_PIDR_PID16(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PIDR_PID16_SHIFT)) & GPIO_PIDR_PID16_MASK)

#define GPIO_PIDR_PID17_MASK                     (0x20000U)
#define GPIO_PIDR_PID17_SHIFT                    (17U)
/*! PID17 - Port Input Disable
 *  0b0..Configured for general-purpose input
 *  0b1..Disabled for general-purpose input
 */
#define GPIO_PIDR_PID17(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PIDR_PID17_SHIFT)) & GPIO_PIDR_PID17_MASK)

#define GPIO_PIDR_PID18_MASK                     (0x40000U)
#define GPIO_PIDR_PID18_SHIFT                    (18U)
/*! PID18 - Port Input Disable
 *  0b0..Configured for general-purpose input
 *  0b1..Disabled for general-purpose input
 */
#define GPIO_PIDR_PID18(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PIDR_PID18_SHIFT)) & GPIO_PIDR_PID18_MASK)

#define GPIO_PIDR_PID19_MASK                     (0x80000U)
#define GPIO_PIDR_PID19_SHIFT                    (19U)
/*! PID19 - Port Input Disable
 *  0b0..Configured for general-purpose input
 *  0b1..Disabled for general-purpose input
 */
#define GPIO_PIDR_PID19(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PIDR_PID19_SHIFT)) & GPIO_PIDR_PID19_MASK)

#define GPIO_PIDR_PID20_MASK                     (0x100000U)
#define GPIO_PIDR_PID20_SHIFT                    (20U)
/*! PID20 - Port Input Disable
 *  0b0..Configured for general-purpose input
 *  0b1..Disabled for general-purpose input
 */
#define GPIO_PIDR_PID20(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PIDR_PID20_SHIFT)) & GPIO_PIDR_PID20_MASK)

#define GPIO_PIDR_PID21_MASK                     (0x200000U)
#define GPIO_PIDR_PID21_SHIFT                    (21U)
/*! PID21 - Port Input Disable
 *  0b0..Configured for general-purpose input
 *  0b1..Disabled for general-purpose input
 */
#define GPIO_PIDR_PID21(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PIDR_PID21_SHIFT)) & GPIO_PIDR_PID21_MASK)

#define GPIO_PIDR_PID22_MASK                     (0x400000U)
#define GPIO_PIDR_PID22_SHIFT                    (22U)
/*! PID22 - Port Input Disable
 *  0b0..Configured for general-purpose input
 *  0b1..Disabled for general-purpose input
 */
#define GPIO_PIDR_PID22(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PIDR_PID22_SHIFT)) & GPIO_PIDR_PID22_MASK)

#define GPIO_PIDR_PID23_MASK                     (0x800000U)
#define GPIO_PIDR_PID23_SHIFT                    (23U)
/*! PID23 - Port Input Disable
 *  0b0..Configured for general-purpose input
 *  0b1..Disabled for general-purpose input
 */
#define GPIO_PIDR_PID23(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PIDR_PID23_SHIFT)) & GPIO_PIDR_PID23_MASK)

#define GPIO_PIDR_PID24_MASK                     (0x1000000U)
#define GPIO_PIDR_PID24_SHIFT                    (24U)
/*! PID24 - Port Input Disable
 *  0b0..Configured for general-purpose input
 *  0b1..Disabled for general-purpose input
 */
#define GPIO_PIDR_PID24(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PIDR_PID24_SHIFT)) & GPIO_PIDR_PID24_MASK)

#define GPIO_PIDR_PID25_MASK                     (0x2000000U)
#define GPIO_PIDR_PID25_SHIFT                    (25U)
/*! PID25 - Port Input Disable
 *  0b0..Configured for general-purpose input
 *  0b1..Disabled for general-purpose input
 */
#define GPIO_PIDR_PID25(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PIDR_PID25_SHIFT)) & GPIO_PIDR_PID25_MASK)

#define GPIO_PIDR_PID26_MASK                     (0x4000000U)
#define GPIO_PIDR_PID26_SHIFT                    (26U)
/*! PID26 - Port Input Disable
 *  0b0..Configured for general-purpose input
 *  0b1..Disabled for general-purpose input
 */
#define GPIO_PIDR_PID26(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PIDR_PID26_SHIFT)) & GPIO_PIDR_PID26_MASK)

#define GPIO_PIDR_PID27_MASK                     (0x8000000U)
#define GPIO_PIDR_PID27_SHIFT                    (27U)
/*! PID27 - Port Input Disable
 *  0b0..Configured for general-purpose input
 *  0b1..Disabled for general-purpose input
 */
#define GPIO_PIDR_PID27(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PIDR_PID27_SHIFT)) & GPIO_PIDR_PID27_MASK)

#define GPIO_PIDR_PID28_MASK                     (0x10000000U)
#define GPIO_PIDR_PID28_SHIFT                    (28U)
/*! PID28 - Port Input Disable
 *  0b0..Configured for general-purpose input
 *  0b1..Disabled for general-purpose input
 */
#define GPIO_PIDR_PID28(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PIDR_PID28_SHIFT)) & GPIO_PIDR_PID28_MASK)

#define GPIO_PIDR_PID29_MASK                     (0x20000000U)
#define GPIO_PIDR_PID29_SHIFT                    (29U)
/*! PID29 - Port Input Disable
 *  0b0..Configured for general-purpose input
 *  0b1..Disabled for general-purpose input
 */
#define GPIO_PIDR_PID29(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PIDR_PID29_SHIFT)) & GPIO_PIDR_PID29_MASK)

#define GPIO_PIDR_PID30_MASK                     (0x40000000U)
#define GPIO_PIDR_PID30_SHIFT                    (30U)
/*! PID30 - Port Input Disable
 *  0b0..Configured for general-purpose input
 *  0b1..Disabled for general-purpose input
 */
#define GPIO_PIDR_PID30(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PIDR_PID30_SHIFT)) & GPIO_PIDR_PID30_MASK)

#define GPIO_PIDR_PID31_MASK                     (0x80000000U)
#define GPIO_PIDR_PID31_SHIFT                    (31U)
/*! PID31 - Port Input Disable
 *  0b0..Configured for general-purpose input
 *  0b1..Disabled for general-purpose input
 */
#define GPIO_PIDR_PID31(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_PIDR_PID31_SHIFT)) & GPIO_PIDR_PID31_MASK)
/*! @} */

/*! @name PDR - Pin Data */
/*! @{ */

#define GPIO_PDR_PD_MASK                         (0x1U)
#define GPIO_PDR_PD_SHIFT                        (0U)
/*! PD - Pin Data (I/O)
 *  0b0..Logic zero
 *  0b1..Logic one
 */
#define GPIO_PDR_PD(x)                           (((uint8_t)(((uint8_t)(x)) << GPIO_PDR_PD_SHIFT)) & GPIO_PDR_PD_MASK)
/*! @} */

/*! @name ICR - Interrupt Control 0..Interrupt Control 31 */
/*! @{ */

#define GPIO_ICR_IRQC_MASK                       (0xF0000U)
#define GPIO_ICR_IRQC_SHIFT                      (16U)
/*! IRQC - Interrupt Configuration
 *  0b0000..ISF is disabled
 *  0b0001..ISF and DMA request on rising edge
 *  0b0010..ISF and DMA request on falling edge
 *  0b0011..ISF and DMA request on either edge
 *  0b0100..Reserved
 *  0b0101..ISF sets on rising edge
 *  0b0110..ISF sets on falling edge
 *  0b0111..ISF sets on either edge
 *  0b1000..ISF and interrupt when logic 0
 *  0b1001..ISF and interrupt on rising edge
 *  0b1010..ISF and interrupt on falling edge
 *  0b1011..ISF and Interrupt on either edge
 *  0b1100..ISF and interrupt when logic 1
 *  0b1101..Enable active-high trigger output; ISF on rising edge (pin state is ORed with other enabled triggers
 *          to generate the output trigger for use by other peripherals)
 *  0b1110..Enable active-low trigger output; ISF on falling edge (pin state is inverted and ORed with other
 *          enabled triggers to generate the output trigger for use by other peripherals)
 *  0b1111..Reserved
 */
#define GPIO_ICR_IRQC(x)                         (((uint32_t)(((uint32_t)(x)) << GPIO_ICR_IRQC_SHIFT)) & GPIO_ICR_IRQC_MASK)

#define GPIO_ICR_ISF_MASK                        (0x1000000U)
#define GPIO_ICR_ISF_SHIFT                       (24U)
/*! ISF - Interrupt Status Flag
 *  0b0..No effect
 *  0b0..Not detected
 *  0b1..Clear the flag
 *  0b1..Detected
 */
#define GPIO_ICR_ISF(x)                          (((uint32_t)(((uint32_t)(x)) << GPIO_ICR_ISF_SHIFT)) & GPIO_ICR_ISF_MASK)
/*! @} */

/*! @name GICLR - Global Interrupt Control Low */
/*! @{ */

#define GPIO_GICLR_GIWE0_MASK                    (0x1U)
#define GPIO_GICLR_GIWE0_SHIFT                   (0U)
/*! GIWE0 - Global Interrupt Write Enable
 *  0b0..Not updated
 *  0b1..Updated
 */
#define GPIO_GICLR_GIWE0(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_GICLR_GIWE0_SHIFT)) & GPIO_GICLR_GIWE0_MASK)

#define GPIO_GICLR_GIWE1_MASK                    (0x2U)
#define GPIO_GICLR_GIWE1_SHIFT                   (1U)
/*! GIWE1 - Global Interrupt Write Enable
 *  0b0..Not updated
 *  0b1..Updated
 */
#define GPIO_GICLR_GIWE1(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_GICLR_GIWE1_SHIFT)) & GPIO_GICLR_GIWE1_MASK)

#define GPIO_GICLR_GIWE2_MASK                    (0x4U)
#define GPIO_GICLR_GIWE2_SHIFT                   (2U)
/*! GIWE2 - Global Interrupt Write Enable
 *  0b0..Not updated
 *  0b1..Updated
 */
#define GPIO_GICLR_GIWE2(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_GICLR_GIWE2_SHIFT)) & GPIO_GICLR_GIWE2_MASK)

#define GPIO_GICLR_GIWE3_MASK                    (0x8U)
#define GPIO_GICLR_GIWE3_SHIFT                   (3U)
/*! GIWE3 - Global Interrupt Write Enable
 *  0b0..Not updated
 *  0b1..Updated
 */
#define GPIO_GICLR_GIWE3(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_GICLR_GIWE3_SHIFT)) & GPIO_GICLR_GIWE3_MASK)

#define GPIO_GICLR_GIWE4_MASK                    (0x10U)
#define GPIO_GICLR_GIWE4_SHIFT                   (4U)
/*! GIWE4 - Global Interrupt Write Enable
 *  0b0..Not updated
 *  0b1..Updated
 */
#define GPIO_GICLR_GIWE4(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_GICLR_GIWE4_SHIFT)) & GPIO_GICLR_GIWE4_MASK)

#define GPIO_GICLR_GIWE5_MASK                    (0x20U)
#define GPIO_GICLR_GIWE5_SHIFT                   (5U)
/*! GIWE5 - Global Interrupt Write Enable
 *  0b0..Not updated
 *  0b1..Updated
 */
#define GPIO_GICLR_GIWE5(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_GICLR_GIWE5_SHIFT)) & GPIO_GICLR_GIWE5_MASK)

#define GPIO_GICLR_GIWE6_MASK                    (0x40U)
#define GPIO_GICLR_GIWE6_SHIFT                   (6U)
/*! GIWE6 - Global Interrupt Write Enable
 *  0b0..Not updated
 *  0b1..Updated
 */
#define GPIO_GICLR_GIWE6(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_GICLR_GIWE6_SHIFT)) & GPIO_GICLR_GIWE6_MASK)

#define GPIO_GICLR_GIWE7_MASK                    (0x80U)
#define GPIO_GICLR_GIWE7_SHIFT                   (7U)
/*! GIWE7 - Global Interrupt Write Enable
 *  0b0..Not updated
 *  0b1..Updated
 */
#define GPIO_GICLR_GIWE7(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_GICLR_GIWE7_SHIFT)) & GPIO_GICLR_GIWE7_MASK)

#define GPIO_GICLR_GIWE8_MASK                    (0x100U)
#define GPIO_GICLR_GIWE8_SHIFT                   (8U)
/*! GIWE8 - Global Interrupt Write Enable
 *  0b0..Not updated
 *  0b1..Updated
 */
#define GPIO_GICLR_GIWE8(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_GICLR_GIWE8_SHIFT)) & GPIO_GICLR_GIWE8_MASK)

#define GPIO_GICLR_GIWE9_MASK                    (0x200U)
#define GPIO_GICLR_GIWE9_SHIFT                   (9U)
/*! GIWE9 - Global Interrupt Write Enable
 *  0b0..Not updated
 *  0b1..Updated
 */
#define GPIO_GICLR_GIWE9(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_GICLR_GIWE9_SHIFT)) & GPIO_GICLR_GIWE9_MASK)

#define GPIO_GICLR_GIWE10_MASK                   (0x400U)
#define GPIO_GICLR_GIWE10_SHIFT                  (10U)
/*! GIWE10 - Global Interrupt Write Enable
 *  0b0..Not updated
 *  0b1..Updated
 */
#define GPIO_GICLR_GIWE10(x)                     (((uint32_t)(((uint32_t)(x)) << GPIO_GICLR_GIWE10_SHIFT)) & GPIO_GICLR_GIWE10_MASK)

#define GPIO_GICLR_GIWE11_MASK                   (0x800U)
#define GPIO_GICLR_GIWE11_SHIFT                  (11U)
/*! GIWE11 - Global Interrupt Write Enable
 *  0b0..Not updated
 *  0b1..Updated
 */
#define GPIO_GICLR_GIWE11(x)                     (((uint32_t)(((uint32_t)(x)) << GPIO_GICLR_GIWE11_SHIFT)) & GPIO_GICLR_GIWE11_MASK)

#define GPIO_GICLR_GIWE12_MASK                   (0x1000U)
#define GPIO_GICLR_GIWE12_SHIFT                  (12U)
/*! GIWE12 - Global Interrupt Write Enable
 *  0b0..Not updated
 *  0b1..Updated
 */
#define GPIO_GICLR_GIWE12(x)                     (((uint32_t)(((uint32_t)(x)) << GPIO_GICLR_GIWE12_SHIFT)) & GPIO_GICLR_GIWE12_MASK)

#define GPIO_GICLR_GIWE13_MASK                   (0x2000U)
#define GPIO_GICLR_GIWE13_SHIFT                  (13U)
/*! GIWE13 - Global Interrupt Write Enable
 *  0b0..Not updated
 *  0b1..Updated
 */
#define GPIO_GICLR_GIWE13(x)                     (((uint32_t)(((uint32_t)(x)) << GPIO_GICLR_GIWE13_SHIFT)) & GPIO_GICLR_GIWE13_MASK)

#define GPIO_GICLR_GIWE14_MASK                   (0x4000U)
#define GPIO_GICLR_GIWE14_SHIFT                  (14U)
/*! GIWE14 - Global Interrupt Write Enable
 *  0b0..Not updated
 *  0b1..Updated
 */
#define GPIO_GICLR_GIWE14(x)                     (((uint32_t)(((uint32_t)(x)) << GPIO_GICLR_GIWE14_SHIFT)) & GPIO_GICLR_GIWE14_MASK)

#define GPIO_GICLR_GIWE15_MASK                   (0x8000U)
#define GPIO_GICLR_GIWE15_SHIFT                  (15U)
/*! GIWE15 - Global Interrupt Write Enable
 *  0b0..Not updated
 *  0b1..Updated
 */
#define GPIO_GICLR_GIWE15(x)                     (((uint32_t)(((uint32_t)(x)) << GPIO_GICLR_GIWE15_SHIFT)) & GPIO_GICLR_GIWE15_MASK)

#define GPIO_GICLR_GIWD_MASK                     (0xFFFF0000U)
#define GPIO_GICLR_GIWD_SHIFT                    (16U)
/*! GIWD - Global Interrupt Write Data */
#define GPIO_GICLR_GIWD(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_GICLR_GIWD_SHIFT)) & GPIO_GICLR_GIWD_MASK)
/*! @} */

/*! @name GICHR - Global Interrupt Control High */
/*! @{ */

#define GPIO_GICHR_GIWE16_MASK                   (0x1U)
#define GPIO_GICHR_GIWE16_SHIFT                  (0U)
/*! GIWE16 - Global Interrupt Write Enable
 *  0b0..Not updated.
 *  0b1..Updated
 */
#define GPIO_GICHR_GIWE16(x)                     (((uint32_t)(((uint32_t)(x)) << GPIO_GICHR_GIWE16_SHIFT)) & GPIO_GICHR_GIWE16_MASK)

#define GPIO_GICHR_GIWE17_MASK                   (0x2U)
#define GPIO_GICHR_GIWE17_SHIFT                  (1U)
/*! GIWE17 - Global Interrupt Write Enable
 *  0b0..Not updated.
 *  0b1..Updated
 */
#define GPIO_GICHR_GIWE17(x)                     (((uint32_t)(((uint32_t)(x)) << GPIO_GICHR_GIWE17_SHIFT)) & GPIO_GICHR_GIWE17_MASK)

#define GPIO_GICHR_GIWE18_MASK                   (0x4U)
#define GPIO_GICHR_GIWE18_SHIFT                  (2U)
/*! GIWE18 - Global Interrupt Write Enable
 *  0b0..Not updated.
 *  0b1..Updated
 */
#define GPIO_GICHR_GIWE18(x)                     (((uint32_t)(((uint32_t)(x)) << GPIO_GICHR_GIWE18_SHIFT)) & GPIO_GICHR_GIWE18_MASK)

#define GPIO_GICHR_GIWE19_MASK                   (0x8U)
#define GPIO_GICHR_GIWE19_SHIFT                  (3U)
/*! GIWE19 - Global Interrupt Write Enable
 *  0b0..Not updated.
 *  0b1..Updated
 */
#define GPIO_GICHR_GIWE19(x)                     (((uint32_t)(((uint32_t)(x)) << GPIO_GICHR_GIWE19_SHIFT)) & GPIO_GICHR_GIWE19_MASK)

#define GPIO_GICHR_GIWE20_MASK                   (0x10U)
#define GPIO_GICHR_GIWE20_SHIFT                  (4U)
/*! GIWE20 - Global Interrupt Write Enable
 *  0b0..Not updated.
 *  0b1..Updated
 */
#define GPIO_GICHR_GIWE20(x)                     (((uint32_t)(((uint32_t)(x)) << GPIO_GICHR_GIWE20_SHIFT)) & GPIO_GICHR_GIWE20_MASK)

#define GPIO_GICHR_GIWE21_MASK                   (0x20U)
#define GPIO_GICHR_GIWE21_SHIFT                  (5U)
/*! GIWE21 - Global Interrupt Write Enable
 *  0b0..Not updated.
 *  0b1..Updated
 */
#define GPIO_GICHR_GIWE21(x)                     (((uint32_t)(((uint32_t)(x)) << GPIO_GICHR_GIWE21_SHIFT)) & GPIO_GICHR_GIWE21_MASK)

#define GPIO_GICHR_GIWE22_MASK                   (0x40U)
#define GPIO_GICHR_GIWE22_SHIFT                  (6U)
/*! GIWE22 - Global Interrupt Write Enable
 *  0b0..Not updated.
 *  0b1..Updated
 */
#define GPIO_GICHR_GIWE22(x)                     (((uint32_t)(((uint32_t)(x)) << GPIO_GICHR_GIWE22_SHIFT)) & GPIO_GICHR_GIWE22_MASK)

#define GPIO_GICHR_GIWE23_MASK                   (0x80U)
#define GPIO_GICHR_GIWE23_SHIFT                  (7U)
/*! GIWE23 - Global Interrupt Write Enable
 *  0b0..Not updated.
 *  0b1..Updated
 */
#define GPIO_GICHR_GIWE23(x)                     (((uint32_t)(((uint32_t)(x)) << GPIO_GICHR_GIWE23_SHIFT)) & GPIO_GICHR_GIWE23_MASK)

#define GPIO_GICHR_GIWE24_MASK                   (0x100U)
#define GPIO_GICHR_GIWE24_SHIFT                  (8U)
/*! GIWE24 - Global Interrupt Write Enable
 *  0b0..Not updated.
 *  0b1..Updated
 */
#define GPIO_GICHR_GIWE24(x)                     (((uint32_t)(((uint32_t)(x)) << GPIO_GICHR_GIWE24_SHIFT)) & GPIO_GICHR_GIWE24_MASK)

#define GPIO_GICHR_GIWE25_MASK                   (0x200U)
#define GPIO_GICHR_GIWE25_SHIFT                  (9U)
/*! GIWE25 - Global Interrupt Write Enable
 *  0b0..Not updated.
 *  0b1..Updated
 */
#define GPIO_GICHR_GIWE25(x)                     (((uint32_t)(((uint32_t)(x)) << GPIO_GICHR_GIWE25_SHIFT)) & GPIO_GICHR_GIWE25_MASK)

#define GPIO_GICHR_GIWE26_MASK                   (0x400U)
#define GPIO_GICHR_GIWE26_SHIFT                  (10U)
/*! GIWE26 - Global Interrupt Write Enable
 *  0b0..Not updated.
 *  0b1..Updated
 */
#define GPIO_GICHR_GIWE26(x)                     (((uint32_t)(((uint32_t)(x)) << GPIO_GICHR_GIWE26_SHIFT)) & GPIO_GICHR_GIWE26_MASK)

#define GPIO_GICHR_GIWE27_MASK                   (0x800U)
#define GPIO_GICHR_GIWE27_SHIFT                  (11U)
/*! GIWE27 - Global Interrupt Write Enable
 *  0b0..Not updated.
 *  0b1..Updated
 */
#define GPIO_GICHR_GIWE27(x)                     (((uint32_t)(((uint32_t)(x)) << GPIO_GICHR_GIWE27_SHIFT)) & GPIO_GICHR_GIWE27_MASK)

#define GPIO_GICHR_GIWE28_MASK                   (0x1000U)
#define GPIO_GICHR_GIWE28_SHIFT                  (12U)
/*! GIWE28 - Global Interrupt Write Enable
 *  0b0..Not updated.
 *  0b1..Updated
 */
#define GPIO_GICHR_GIWE28(x)                     (((uint32_t)(((uint32_t)(x)) << GPIO_GICHR_GIWE28_SHIFT)) & GPIO_GICHR_GIWE28_MASK)

#define GPIO_GICHR_GIWE29_MASK                   (0x2000U)
#define GPIO_GICHR_GIWE29_SHIFT                  (13U)
/*! GIWE29 - Global Interrupt Write Enable
 *  0b0..Not updated.
 *  0b1..Updated
 */
#define GPIO_GICHR_GIWE29(x)                     (((uint32_t)(((uint32_t)(x)) << GPIO_GICHR_GIWE29_SHIFT)) & GPIO_GICHR_GIWE29_MASK)

#define GPIO_GICHR_GIWE30_MASK                   (0x4000U)
#define GPIO_GICHR_GIWE30_SHIFT                  (14U)
/*! GIWE30 - Global Interrupt Write Enable
 *  0b0..Not updated.
 *  0b1..Updated
 */
#define GPIO_GICHR_GIWE30(x)                     (((uint32_t)(((uint32_t)(x)) << GPIO_GICHR_GIWE30_SHIFT)) & GPIO_GICHR_GIWE30_MASK)

#define GPIO_GICHR_GIWE31_MASK                   (0x8000U)
#define GPIO_GICHR_GIWE31_SHIFT                  (15U)
/*! GIWE31 - Global Interrupt Write Enable
 *  0b0..Not updated.
 *  0b1..Updated
 */
#define GPIO_GICHR_GIWE31(x)                     (((uint32_t)(((uint32_t)(x)) << GPIO_GICHR_GIWE31_SHIFT)) & GPIO_GICHR_GIWE31_MASK)

#define GPIO_GICHR_GIWD_MASK                     (0xFFFF0000U)
#define GPIO_GICHR_GIWD_SHIFT                    (16U)
/*! GIWD - Global Interrupt Write Data */
#define GPIO_GICHR_GIWD(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_GICHR_GIWD_SHIFT)) & GPIO_GICHR_GIWD_MASK)
/*! @} */

/*! @name ISFR - Interrupt Status Flag */
/*! @{ */

#define GPIO_ISFR_ISF0_MASK                      (0x1U)
#define GPIO_ISFR_ISF0_SHIFT                     (0U)
/*! ISF0 - Interrupt Status Flag
 *  0b0..No effect
 *  0b0..Not detected
 *  0b1..Clear the flag
 *  0b1..Detected
 */
#define GPIO_ISFR_ISF0(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_ISFR_ISF0_SHIFT)) & GPIO_ISFR_ISF0_MASK)

#define GPIO_ISFR_ISF1_MASK                      (0x2U)
#define GPIO_ISFR_ISF1_SHIFT                     (1U)
/*! ISF1 - Interrupt Status Flag
 *  0b0..No effect
 *  0b0..Not detected
 *  0b1..Clear the flag
 *  0b1..Detected
 */
#define GPIO_ISFR_ISF1(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_ISFR_ISF1_SHIFT)) & GPIO_ISFR_ISF1_MASK)

#define GPIO_ISFR_ISF2_MASK                      (0x4U)
#define GPIO_ISFR_ISF2_SHIFT                     (2U)
/*! ISF2 - Interrupt Status Flag
 *  0b0..No effect
 *  0b0..Not detected
 *  0b1..Clear the flag
 *  0b1..Detected
 */
#define GPIO_ISFR_ISF2(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_ISFR_ISF2_SHIFT)) & GPIO_ISFR_ISF2_MASK)

#define GPIO_ISFR_ISF3_MASK                      (0x8U)
#define GPIO_ISFR_ISF3_SHIFT                     (3U)
/*! ISF3 - Interrupt Status Flag
 *  0b0..No effect
 *  0b0..Not detected
 *  0b1..Clear the flag
 *  0b1..Detected
 */
#define GPIO_ISFR_ISF3(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_ISFR_ISF3_SHIFT)) & GPIO_ISFR_ISF3_MASK)

#define GPIO_ISFR_ISF4_MASK                      (0x10U)
#define GPIO_ISFR_ISF4_SHIFT                     (4U)
/*! ISF4 - Interrupt Status Flag
 *  0b0..No effect
 *  0b0..Not detected
 *  0b1..Clear the flag
 *  0b1..Detected
 */
#define GPIO_ISFR_ISF4(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_ISFR_ISF4_SHIFT)) & GPIO_ISFR_ISF4_MASK)

#define GPIO_ISFR_ISF5_MASK                      (0x20U)
#define GPIO_ISFR_ISF5_SHIFT                     (5U)
/*! ISF5 - Interrupt Status Flag
 *  0b0..No effect
 *  0b0..Not detected
 *  0b1..Clear the flag
 *  0b1..Detected
 */
#define GPIO_ISFR_ISF5(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_ISFR_ISF5_SHIFT)) & GPIO_ISFR_ISF5_MASK)

#define GPIO_ISFR_ISF6_MASK                      (0x40U)
#define GPIO_ISFR_ISF6_SHIFT                     (6U)
/*! ISF6 - Interrupt Status Flag
 *  0b0..No effect
 *  0b0..Not detected
 *  0b1..Clear the flag
 *  0b1..Detected
 */
#define GPIO_ISFR_ISF6(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_ISFR_ISF6_SHIFT)) & GPIO_ISFR_ISF6_MASK)

#define GPIO_ISFR_ISF7_MASK                      (0x80U)
#define GPIO_ISFR_ISF7_SHIFT                     (7U)
/*! ISF7 - Interrupt Status Flag
 *  0b0..No effect
 *  0b0..Not detected
 *  0b1..Clear the flag
 *  0b1..Detected
 */
#define GPIO_ISFR_ISF7(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_ISFR_ISF7_SHIFT)) & GPIO_ISFR_ISF7_MASK)

#define GPIO_ISFR_ISF8_MASK                      (0x100U)
#define GPIO_ISFR_ISF8_SHIFT                     (8U)
/*! ISF8 - Interrupt Status Flag
 *  0b0..No effect
 *  0b0..Not detected
 *  0b1..Clear the flag
 *  0b1..Detected
 */
#define GPIO_ISFR_ISF8(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_ISFR_ISF8_SHIFT)) & GPIO_ISFR_ISF8_MASK)

#define GPIO_ISFR_ISF9_MASK                      (0x200U)
#define GPIO_ISFR_ISF9_SHIFT                     (9U)
/*! ISF9 - Interrupt Status Flag
 *  0b0..No effect
 *  0b0..Not detected
 *  0b1..Clear the flag
 *  0b1..Detected
 */
#define GPIO_ISFR_ISF9(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_ISFR_ISF9_SHIFT)) & GPIO_ISFR_ISF9_MASK)

#define GPIO_ISFR_ISF10_MASK                     (0x400U)
#define GPIO_ISFR_ISF10_SHIFT                    (10U)
/*! ISF10 - Interrupt Status Flag
 *  0b0..No effect
 *  0b0..Not detected
 *  0b1..Clear the flag
 *  0b1..Detected
 */
#define GPIO_ISFR_ISF10(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_ISFR_ISF10_SHIFT)) & GPIO_ISFR_ISF10_MASK)

#define GPIO_ISFR_ISF11_MASK                     (0x800U)
#define GPIO_ISFR_ISF11_SHIFT                    (11U)
/*! ISF11 - Interrupt Status Flag
 *  0b0..No effect
 *  0b0..Not detected
 *  0b1..Clear the flag
 *  0b1..Detected
 */
#define GPIO_ISFR_ISF11(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_ISFR_ISF11_SHIFT)) & GPIO_ISFR_ISF11_MASK)

#define GPIO_ISFR_ISF12_MASK                     (0x1000U)
#define GPIO_ISFR_ISF12_SHIFT                    (12U)
/*! ISF12 - Interrupt Status Flag
 *  0b0..No effect
 *  0b0..Not detected
 *  0b1..Clear the flag
 *  0b1..Detected
 */
#define GPIO_ISFR_ISF12(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_ISFR_ISF12_SHIFT)) & GPIO_ISFR_ISF12_MASK)

#define GPIO_ISFR_ISF13_MASK                     (0x2000U)
#define GPIO_ISFR_ISF13_SHIFT                    (13U)
/*! ISF13 - Interrupt Status Flag
 *  0b0..No effect
 *  0b0..Not detected
 *  0b1..Clear the flag
 *  0b1..Detected
 */
#define GPIO_ISFR_ISF13(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_ISFR_ISF13_SHIFT)) & GPIO_ISFR_ISF13_MASK)

#define GPIO_ISFR_ISF14_MASK                     (0x4000U)
#define GPIO_ISFR_ISF14_SHIFT                    (14U)
/*! ISF14 - Interrupt Status Flag
 *  0b0..No effect
 *  0b0..Not detected
 *  0b1..Clear the flag
 *  0b1..Detected
 */
#define GPIO_ISFR_ISF14(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_ISFR_ISF14_SHIFT)) & GPIO_ISFR_ISF14_MASK)

#define GPIO_ISFR_ISF15_MASK                     (0x8000U)
#define GPIO_ISFR_ISF15_SHIFT                    (15U)
/*! ISF15 - Interrupt Status Flag
 *  0b0..No effect
 *  0b0..Not detected
 *  0b1..Clear the flag
 *  0b1..Detected
 */
#define GPIO_ISFR_ISF15(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_ISFR_ISF15_SHIFT)) & GPIO_ISFR_ISF15_MASK)

#define GPIO_ISFR_ISF16_MASK                     (0x10000U)
#define GPIO_ISFR_ISF16_SHIFT                    (16U)
/*! ISF16 - Interrupt Status Flag
 *  0b0..No effect
 *  0b0..Not detected
 *  0b1..Clear the flag
 *  0b1..Detected
 */
#define GPIO_ISFR_ISF16(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_ISFR_ISF16_SHIFT)) & GPIO_ISFR_ISF16_MASK)

#define GPIO_ISFR_ISF17_MASK                     (0x20000U)
#define GPIO_ISFR_ISF17_SHIFT                    (17U)
/*! ISF17 - Interrupt Status Flag
 *  0b0..No effect
 *  0b0..Not detected
 *  0b1..Clear the flag
 *  0b1..Detected
 */
#define GPIO_ISFR_ISF17(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_ISFR_ISF17_SHIFT)) & GPIO_ISFR_ISF17_MASK)

#define GPIO_ISFR_ISF18_MASK                     (0x40000U)
#define GPIO_ISFR_ISF18_SHIFT                    (18U)
/*! ISF18 - Interrupt Status Flag
 *  0b0..No effect
 *  0b0..Not detected
 *  0b1..Clear the flag
 *  0b1..Detected
 */
#define GPIO_ISFR_ISF18(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_ISFR_ISF18_SHIFT)) & GPIO_ISFR_ISF18_MASK)

#define GPIO_ISFR_ISF19_MASK                     (0x80000U)
#define GPIO_ISFR_ISF19_SHIFT                    (19U)
/*! ISF19 - Interrupt Status Flag
 *  0b0..No effect
 *  0b0..Not detected
 *  0b1..Clear the flag
 *  0b1..Detected
 */
#define GPIO_ISFR_ISF19(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_ISFR_ISF19_SHIFT)) & GPIO_ISFR_ISF19_MASK)

#define GPIO_ISFR_ISF20_MASK                     (0x100000U)
#define GPIO_ISFR_ISF20_SHIFT                    (20U)
/*! ISF20 - Interrupt Status Flag
 *  0b0..No effect
 *  0b0..Not detected
 *  0b1..Clear the flag
 *  0b1..Detected
 */
#define GPIO_ISFR_ISF20(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_ISFR_ISF20_SHIFT)) & GPIO_ISFR_ISF20_MASK)

#define GPIO_ISFR_ISF21_MASK                     (0x200000U)
#define GPIO_ISFR_ISF21_SHIFT                    (21U)
/*! ISF21 - Interrupt Status Flag
 *  0b0..No effect
 *  0b0..Not detected
 *  0b1..Clear the flag
 *  0b1..Detected
 */
#define GPIO_ISFR_ISF21(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_ISFR_ISF21_SHIFT)) & GPIO_ISFR_ISF21_MASK)

#define GPIO_ISFR_ISF22_MASK                     (0x400000U)
#define GPIO_ISFR_ISF22_SHIFT                    (22U)
/*! ISF22 - Interrupt Status Flag
 *  0b0..No effect
 *  0b0..Not detected
 *  0b1..Clear the flag
 *  0b1..Detected
 */
#define GPIO_ISFR_ISF22(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_ISFR_ISF22_SHIFT)) & GPIO_ISFR_ISF22_MASK)

#define GPIO_ISFR_ISF23_MASK                     (0x800000U)
#define GPIO_ISFR_ISF23_SHIFT                    (23U)
/*! ISF23 - Interrupt Status Flag
 *  0b0..No effect
 *  0b0..Not detected
 *  0b1..Clear the flag
 *  0b1..Detected
 */
#define GPIO_ISFR_ISF23(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_ISFR_ISF23_SHIFT)) & GPIO_ISFR_ISF23_MASK)

#define GPIO_ISFR_ISF24_MASK                     (0x1000000U)
#define GPIO_ISFR_ISF24_SHIFT                    (24U)
/*! ISF24 - Interrupt Status Flag
 *  0b0..No effect
 *  0b0..Not detected
 *  0b1..Clear the flag
 *  0b1..Detected
 */
#define GPIO_ISFR_ISF24(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_ISFR_ISF24_SHIFT)) & GPIO_ISFR_ISF24_MASK)

#define GPIO_ISFR_ISF25_MASK                     (0x2000000U)
#define GPIO_ISFR_ISF25_SHIFT                    (25U)
/*! ISF25 - Interrupt Status Flag
 *  0b0..No effect
 *  0b0..Not detected
 *  0b1..Clear the flag
 *  0b1..Detected
 */
#define GPIO_ISFR_ISF25(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_ISFR_ISF25_SHIFT)) & GPIO_ISFR_ISF25_MASK)

#define GPIO_ISFR_ISF26_MASK                     (0x4000000U)
#define GPIO_ISFR_ISF26_SHIFT                    (26U)
/*! ISF26 - Interrupt Status Flag
 *  0b0..No effect
 *  0b0..Not detected
 *  0b1..Clear the flag
 *  0b1..Detected
 */
#define GPIO_ISFR_ISF26(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_ISFR_ISF26_SHIFT)) & GPIO_ISFR_ISF26_MASK)

#define GPIO_ISFR_ISF27_MASK                     (0x8000000U)
#define GPIO_ISFR_ISF27_SHIFT                    (27U)
/*! ISF27 - Interrupt Status Flag
 *  0b0..No effect
 *  0b0..Not detected
 *  0b1..Clear the flag
 *  0b1..Detected
 */
#define GPIO_ISFR_ISF27(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_ISFR_ISF27_SHIFT)) & GPIO_ISFR_ISF27_MASK)

#define GPIO_ISFR_ISF28_MASK                     (0x10000000U)
#define GPIO_ISFR_ISF28_SHIFT                    (28U)
/*! ISF28 - Interrupt Status Flag
 *  0b0..No effect
 *  0b0..Not detected
 *  0b1..Clear the flag
 *  0b1..Detected
 */
#define GPIO_ISFR_ISF28(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_ISFR_ISF28_SHIFT)) & GPIO_ISFR_ISF28_MASK)

#define GPIO_ISFR_ISF29_MASK                     (0x20000000U)
#define GPIO_ISFR_ISF29_SHIFT                    (29U)
/*! ISF29 - Interrupt Status Flag
 *  0b0..No effect
 *  0b0..Not detected
 *  0b1..Clear the flag
 *  0b1..Detected
 */
#define GPIO_ISFR_ISF29(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_ISFR_ISF29_SHIFT)) & GPIO_ISFR_ISF29_MASK)

#define GPIO_ISFR_ISF30_MASK                     (0x40000000U)
#define GPIO_ISFR_ISF30_SHIFT                    (30U)
/*! ISF30 - Interrupt Status Flag
 *  0b0..No effect
 *  0b0..Not detected
 *  0b1..Clear the flag
 *  0b1..Detected
 */
#define GPIO_ISFR_ISF30(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_ISFR_ISF30_SHIFT)) & GPIO_ISFR_ISF30_MASK)

#define GPIO_ISFR_ISF31_MASK                     (0x80000000U)
#define GPIO_ISFR_ISF31_SHIFT                    (31U)
/*! ISF31 - Interrupt Status Flag
 *  0b0..No effect
 *  0b0..Not detected
 *  0b1..Clear the flag
 *  0b1..Detected
 */
#define GPIO_ISFR_ISF31(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_ISFR_ISF31_SHIFT)) & GPIO_ISFR_ISF31_MASK)
/*! @} */


/*!
 * @}
 */ /* end of group GPIO_Register_Masks */


/*!
 * @}
 */ /* end of group GPIO_Peripheral_Access_Layer */


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


#endif  /* PERI_GPIO_H_ */

