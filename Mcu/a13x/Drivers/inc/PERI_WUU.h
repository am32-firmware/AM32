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
**         CMSIS Peripheral Access Layer for WUU
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
 * @file PERI_WUU.h
 * @version 2.0
 * @date 2024-10-29
 * @brief CMSIS Peripheral Access Layer for WUU
 *
 * CMSIS Peripheral Access Layer for WUU
 */

#if !defined(PERI_WUU_H_)
#define PERI_WUU_H_                              /**< Symbol preventing repeated inclusion */

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
   -- WUU Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup WUU_Peripheral_Access_Layer WUU Peripheral Access Layer
 * @{
 */

/** WUU - Register Layout Typedef */
typedef struct {
  __I  uint32_t VERID;                             /**< Version ID, offset: 0x0 */
  __I  uint32_t PARAM;                             /**< Parameter, offset: 0x4 */
  __IO uint32_t PE1;                               /**< Pin Enable 1, offset: 0x8 */
  __IO uint32_t PE2;                               /**< Pin Enable 2, offset: 0xC */
       uint8_t RESERVED_0[8];
  __IO uint32_t ME;                                /**< Module Interrupt Enable, offset: 0x18 */
  __IO uint32_t DE;                                /**< Module DMA/Trigger Enable, offset: 0x1C */
  __IO uint32_t PF;                                /**< Pin Flag, offset: 0x20 */
       uint8_t RESERVED_1[12];
  __IO uint32_t FILT;                              /**< Pin Filter, offset: 0x30 */
       uint8_t RESERVED_2[4];
  __IO uint32_t PDC1;                              /**< Pin DMA/Trigger Configuration 1, offset: 0x38 */
  __IO uint32_t PDC2;                              /**< Pin DMA/Trigger Configuration 2, offset: 0x3C */
       uint8_t RESERVED_3[8];
  __IO uint32_t FDC;                               /**< Pin Filter DMA/Trigger Configuration, offset: 0x48 */
       uint8_t RESERVED_4[4];
  __IO uint32_t PMC;                               /**< Pin Mode Configuration, offset: 0x50 */
       uint8_t RESERVED_5[4];
  __IO uint32_t FMC;                               /**< Pin Filter Mode Configuration, offset: 0x58 */
} WUU_Type;

/* ----------------------------------------------------------------------------
   -- WUU Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup WUU_Register_Masks WUU Register Masks
 * @{
 */

/*! @name VERID - Version ID */
/*! @{ */

#define WUU_VERID_FEATURE_MASK                   (0xFFFFU)
#define WUU_VERID_FEATURE_SHIFT                  (0U)
/*! FEATURE - Feature Specification Number
 *  0b0000000000000000..Standard features implemented
 *  0b0000000000000001..Support for DMA/Trigger generation from wake-up pins and filters enabled. Support for
 *                      external pin/filter detection during all power modes enabled.
 */
#define WUU_VERID_FEATURE(x)                     (((uint32_t)(((uint32_t)(x)) << WUU_VERID_FEATURE_SHIFT)) & WUU_VERID_FEATURE_MASK)

#define WUU_VERID_MINOR_MASK                     (0xFF0000U)
#define WUU_VERID_MINOR_SHIFT                    (16U)
/*! MINOR - Minor Version Number */
#define WUU_VERID_MINOR(x)                       (((uint32_t)(((uint32_t)(x)) << WUU_VERID_MINOR_SHIFT)) & WUU_VERID_MINOR_MASK)

#define WUU_VERID_MAJOR_MASK                     (0xFF000000U)
#define WUU_VERID_MAJOR_SHIFT                    (24U)
/*! MAJOR - Major Version Number */
#define WUU_VERID_MAJOR(x)                       (((uint32_t)(((uint32_t)(x)) << WUU_VERID_MAJOR_SHIFT)) & WUU_VERID_MAJOR_MASK)
/*! @} */

/*! @name PARAM - Parameter */
/*! @{ */

#define WUU_PARAM_FILTERS_MASK                   (0xFFU)
#define WUU_PARAM_FILTERS_SHIFT                  (0U)
/*! FILTERS - Filter Number */
#define WUU_PARAM_FILTERS(x)                     (((uint32_t)(((uint32_t)(x)) << WUU_PARAM_FILTERS_SHIFT)) & WUU_PARAM_FILTERS_MASK)

#define WUU_PARAM_DMAS_MASK                      (0xFF00U)
#define WUU_PARAM_DMAS_SHIFT                     (8U)
/*! DMAS - DMA Number */
#define WUU_PARAM_DMAS(x)                        (((uint32_t)(((uint32_t)(x)) << WUU_PARAM_DMAS_SHIFT)) & WUU_PARAM_DMAS_MASK)

#define WUU_PARAM_MODULES_MASK                   (0xFF0000U)
#define WUU_PARAM_MODULES_SHIFT                  (16U)
/*! MODULES - Module Number */
#define WUU_PARAM_MODULES(x)                     (((uint32_t)(((uint32_t)(x)) << WUU_PARAM_MODULES_SHIFT)) & WUU_PARAM_MODULES_MASK)

#define WUU_PARAM_PINS_MASK                      (0xFF000000U)
#define WUU_PARAM_PINS_SHIFT                     (24U)
/*! PINS - Pin Number */
#define WUU_PARAM_PINS(x)                        (((uint32_t)(((uint32_t)(x)) << WUU_PARAM_PINS_SHIFT)) & WUU_PARAM_PINS_MASK)
/*! @} */

/*! @name PE1 - Pin Enable 1 */
/*! @{ */

#define WUU_PE1_Reserved0_MASK                   (0x3U)
#define WUU_PE1_Reserved0_SHIFT                  (0U)
/*! Reserved0 - Reserved
 *  0b00..Not supported
 *  0b01..Not supported
 *  0b10..Not supported
 *  0b11..Not supported
 */
#define WUU_PE1_Reserved0(x)                     (((uint32_t)(((uint32_t)(x)) << WUU_PE1_Reserved0_SHIFT)) & WUU_PE1_Reserved0_MASK)

#define WUU_PE1_Reserved1_MASK                   (0xCU)
#define WUU_PE1_Reserved1_SHIFT                  (2U)
/*! Reserved1 - Reserved
 *  0b00..Not supported
 *  0b01..Not supported
 *  0b10..Not supported
 *  0b11..Not supported
 */
#define WUU_PE1_Reserved1(x)                     (((uint32_t)(((uint32_t)(x)) << WUU_PE1_Reserved1_SHIFT)) & WUU_PE1_Reserved1_MASK)

#define WUU_PE1_WUPE2_MASK                       (0x30U)
#define WUU_PE1_WUPE2_SHIFT                      (4U)
/*! WUPE2 - Wake-up Pin Enable for WUU_Pn
 *  0b00..Disable
 *  0b01..Enable (detect on rising edge or high level)
 *  0b10..Enable (detect on falling edge or low level)
 *  0b11..Enable (detect on any edge)
 */
#define WUU_PE1_WUPE2(x)                         (((uint32_t)(((uint32_t)(x)) << WUU_PE1_WUPE2_SHIFT)) & WUU_PE1_WUPE2_MASK)

#define WUU_PE1_Reserved3_MASK                   (0xC0U)
#define WUU_PE1_Reserved3_SHIFT                  (6U)
/*! Reserved3 - Reserved
 *  0b00..Not supported
 *  0b01..Not supported
 *  0b10..Not supported
 *  0b11..Not supported
 */
#define WUU_PE1_Reserved3(x)                     (((uint32_t)(((uint32_t)(x)) << WUU_PE1_Reserved3_SHIFT)) & WUU_PE1_Reserved3_MASK)

#define WUU_PE1_Reserved4_MASK                   (0x300U)
#define WUU_PE1_Reserved4_SHIFT                  (8U)
/*! Reserved4 - Reserved
 *  0b00..Not supported
 *  0b01..Not supported
 *  0b10..Not supported
 *  0b11..Not supported
 */
#define WUU_PE1_Reserved4(x)                     (((uint32_t)(((uint32_t)(x)) << WUU_PE1_Reserved4_SHIFT)) & WUU_PE1_Reserved4_MASK)

#define WUU_PE1_Reserved5_MASK                   (0xC00U)
#define WUU_PE1_Reserved5_SHIFT                  (10U)
/*! Reserved5 - Reserved
 *  0b00..Not supported
 *  0b01..Not supported
 *  0b10..Not supported
 *  0b11..Not supported
 */
#define WUU_PE1_Reserved5(x)                     (((uint32_t)(((uint32_t)(x)) << WUU_PE1_Reserved5_SHIFT)) & WUU_PE1_Reserved5_MASK)

#define WUU_PE1_WUPE6_MASK                       (0x3000U)
#define WUU_PE1_WUPE6_SHIFT                      (12U)
/*! WUPE6 - Wake-up Pin Enable for WUU_Pn
 *  0b00..Disable
 *  0b01..Enable (detect on rising edge or high level)
 *  0b10..Enable (detect on falling edge or low level)
 *  0b11..Enable (detect on any edge)
 */
#define WUU_PE1_WUPE6(x)                         (((uint32_t)(((uint32_t)(x)) << WUU_PE1_WUPE6_SHIFT)) & WUU_PE1_WUPE6_MASK)

#define WUU_PE1_WUPE7_MASK                       (0xC000U)
#define WUU_PE1_WUPE7_SHIFT                      (14U)
/*! WUPE7 - Wake-up Pin Enable for WUU_Pn
 *  0b00..Disable
 *  0b01..Enable (detect on rising edge or high level)
 *  0b10..Enable (detect on falling edge or low level)
 *  0b11..Enable (detect on any edge)
 */
#define WUU_PE1_WUPE7(x)                         (((uint32_t)(((uint32_t)(x)) << WUU_PE1_WUPE7_SHIFT)) & WUU_PE1_WUPE7_MASK)

#define WUU_PE1_WUPE8_MASK                       (0x30000U)
#define WUU_PE1_WUPE8_SHIFT                      (16U)
/*! WUPE8 - Wake-up Pin Enable for WUU_Pn
 *  0b00..Disable
 *  0b01..Enable (detect on rising edge or high level)
 *  0b10..Enable (detect on falling edge or low level)
 *  0b11..Enable (detect on any edge)
 */
#define WUU_PE1_WUPE8(x)                         (((uint32_t)(((uint32_t)(x)) << WUU_PE1_WUPE8_SHIFT)) & WUU_PE1_WUPE8_MASK)

#define WUU_PE1_WUPE9_MASK                       (0xC0000U)
#define WUU_PE1_WUPE9_SHIFT                      (18U)
/*! WUPE9 - Wake-up Pin Enable for WUU_Pn
 *  0b00..Disable
 *  0b01..Enable (detect on rising edge or high level)
 *  0b10..Enable (detect on falling edge or low level)
 *  0b11..Enable (detect on any edge)
 */
#define WUU_PE1_WUPE9(x)                         (((uint32_t)(((uint32_t)(x)) << WUU_PE1_WUPE9_SHIFT)) & WUU_PE1_WUPE9_MASK)

#define WUU_PE1_WUPE10_MASK                      (0x300000U)
#define WUU_PE1_WUPE10_SHIFT                     (20U)
/*! WUPE10 - Wake-up Pin Enable for WUU_Pn
 *  0b00..Disable
 *  0b01..Enable (detect on rising edge or high level)
 *  0b10..Enable (detect on falling edge or low level)
 *  0b11..Enable (detect on any edge)
 */
#define WUU_PE1_WUPE10(x)                        (((uint32_t)(((uint32_t)(x)) << WUU_PE1_WUPE10_SHIFT)) & WUU_PE1_WUPE10_MASK)

#define WUU_PE1_WUPE11_MASK                      (0xC00000U)
#define WUU_PE1_WUPE11_SHIFT                     (22U)
/*! WUPE11 - Wake-up Pin Enable for WUU_Pn
 *  0b00..Disable
 *  0b01..Enable (detect on rising edge or high level)
 *  0b10..Enable (detect on falling edge or low level)
 *  0b11..Enable (detect on any edge)
 */
#define WUU_PE1_WUPE11(x)                        (((uint32_t)(((uint32_t)(x)) << WUU_PE1_WUPE11_SHIFT)) & WUU_PE1_WUPE11_MASK)

#define WUU_PE1_WUPE12_MASK                      (0x3000000U)
#define WUU_PE1_WUPE12_SHIFT                     (24U)
/*! WUPE12 - Wake-up Pin Enable for WUU_Pn
 *  0b00..Disable
 *  0b01..Enable (detect on rising edge or high level)
 *  0b10..Enable (detect on falling edge or low level)
 *  0b11..Enable (detect on any edge)
 */
#define WUU_PE1_WUPE12(x)                        (((uint32_t)(((uint32_t)(x)) << WUU_PE1_WUPE12_SHIFT)) & WUU_PE1_WUPE12_MASK)

#define WUU_PE1_Reserved13_MASK                  (0xC000000U)
#define WUU_PE1_Reserved13_SHIFT                 (26U)
/*! Reserved13 - Reserved
 *  0b00..Not supported
 *  0b01..Not supported
 *  0b10..Not supported
 *  0b11..Not supported
 */
#define WUU_PE1_Reserved13(x)                    (((uint32_t)(((uint32_t)(x)) << WUU_PE1_Reserved13_SHIFT)) & WUU_PE1_Reserved13_MASK)

#define WUU_PE1_Reserved14_MASK                  (0x30000000U)
#define WUU_PE1_Reserved14_SHIFT                 (28U)
/*! Reserved14 - Reserved
 *  0b00..Not supported
 *  0b01..Not supported
 *  0b10..Not supported
 *  0b11..Not supported
 */
#define WUU_PE1_Reserved14(x)                    (((uint32_t)(((uint32_t)(x)) << WUU_PE1_Reserved14_SHIFT)) & WUU_PE1_Reserved14_MASK)

#define WUU_PE1_Reserved15_MASK                  (0xC0000000U)
#define WUU_PE1_Reserved15_SHIFT                 (30U)
/*! Reserved15 - Reserved
 *  0b00..Not supported
 *  0b01..Not supported
 *  0b10..Not supported
 *  0b11..Not supported
 */
#define WUU_PE1_Reserved15(x)                    (((uint32_t)(((uint32_t)(x)) << WUU_PE1_Reserved15_SHIFT)) & WUU_PE1_Reserved15_MASK)
/*! @} */

/*! @name PE2 - Pin Enable 2 */
/*! @{ */

#define WUU_PE2_Reserved16_MASK                  (0x3U)
#define WUU_PE2_Reserved16_SHIFT                 (0U)
/*! Reserved16 - Reserved
 *  0b00..Not supported
 *  0b01..Not supported
 *  0b10..Not supported
 *  0b11..Not supported
 */
#define WUU_PE2_Reserved16(x)                    (((uint32_t)(((uint32_t)(x)) << WUU_PE2_Reserved16_SHIFT)) & WUU_PE2_Reserved16_MASK)

#define WUU_PE2_Reserved17_MASK                  (0xCU)
#define WUU_PE2_Reserved17_SHIFT                 (2U)
/*! Reserved17 - Reserved
 *  0b00..Not supported
 *  0b01..Not supported
 *  0b10..Not supported
 *  0b11..Not supported
 */
#define WUU_PE2_Reserved17(x)                    (((uint32_t)(((uint32_t)(x)) << WUU_PE2_Reserved17_SHIFT)) & WUU_PE2_Reserved17_MASK)

#define WUU_PE2_WUPE18_MASK                      (0x30U)
#define WUU_PE2_WUPE18_SHIFT                     (4U)
/*! WUPE18 - Wake-up Pin Enable for WUU_Pn
 *  0b00..Disable
 *  0b01..Enable (detect on rising edge or high level)
 *  0b10..Enable (detect on falling edge or low level)
 *  0b11..Enable (detect on any edge)
 */
#define WUU_PE2_WUPE18(x)                        (((uint32_t)(((uint32_t)(x)) << WUU_PE2_WUPE18_SHIFT)) & WUU_PE2_WUPE18_MASK)

#define WUU_PE2_WUPE19_MASK                      (0xC0U)
#define WUU_PE2_WUPE19_SHIFT                     (6U)
/*! WUPE19 - Wake-up Pin Enable for WUU_Pn
 *  0b00..Disable
 *  0b01..Enable (detect on rising edge or high level)
 *  0b10..Enable (detect on falling edge or low level)
 *  0b11..Enable (detect on any edge)
 */
#define WUU_PE2_WUPE19(x)                        (((uint32_t)(((uint32_t)(x)) << WUU_PE2_WUPE19_SHIFT)) & WUU_PE2_WUPE19_MASK)

#define WUU_PE2_WUPE20_MASK                      (0x300U)
#define WUU_PE2_WUPE20_SHIFT                     (8U)
/*! WUPE20 - Wake-up Pin Enable for WUU_Pn
 *  0b00..Disable
 *  0b01..Enable (detect on rising edge or high level)
 *  0b10..Enable (detect on falling edge or low level)
 *  0b11..Enable (detect on any edge)
 */
#define WUU_PE2_WUPE20(x)                        (((uint32_t)(((uint32_t)(x)) << WUU_PE2_WUPE20_SHIFT)) & WUU_PE2_WUPE20_MASK)

#define WUU_PE2_Reserved21_MASK                  (0xC00U)
#define WUU_PE2_Reserved21_SHIFT                 (10U)
/*! Reserved21 - Reserved
 *  0b00..Not supported
 *  0b01..Not supported
 *  0b10..Not supported
 *  0b11..Not supported
 */
#define WUU_PE2_Reserved21(x)                    (((uint32_t)(((uint32_t)(x)) << WUU_PE2_Reserved21_SHIFT)) & WUU_PE2_Reserved21_MASK)

#define WUU_PE2_WUPE22_MASK                      (0x3000U)
#define WUU_PE2_WUPE22_SHIFT                     (12U)
/*! WUPE22 - Wake-up Pin Enable for WUU_Pn
 *  0b00..Disable
 *  0b01..Enable (detect on rising edge or high level)
 *  0b10..Enable (detect on falling edge or low level)
 *  0b11..Enable (detect on any edge)
 */
#define WUU_PE2_WUPE22(x)                        (((uint32_t)(((uint32_t)(x)) << WUU_PE2_WUPE22_SHIFT)) & WUU_PE2_WUPE22_MASK)

#define WUU_PE2_WUPE23_MASK                      (0xC000U)
#define WUU_PE2_WUPE23_SHIFT                     (14U)
/*! WUPE23 - Wake-up Pin Enable for WUU_Pn
 *  0b00..Disable
 *  0b01..Enable (detect on rising edge or high level)
 *  0b10..Enable (detect on falling edge or low level)
 *  0b11..Enable (detect on any edge)
 */
#define WUU_PE2_WUPE23(x)                        (((uint32_t)(((uint32_t)(x)) << WUU_PE2_WUPE23_SHIFT)) & WUU_PE2_WUPE23_MASK)

#define WUU_PE2_WUPE24_MASK                      (0x30000U)
#define WUU_PE2_WUPE24_SHIFT                     (16U)
/*! WUPE24 - Wake-up Pin Enable for WUU_Pn
 *  0b00..Disable
 *  0b01..Enable (detect on rising edge or high level)
 *  0b10..Enable (detect on falling edge or low level)
 *  0b11..Enable (detect on any edge)
 */
#define WUU_PE2_WUPE24(x)                        (((uint32_t)(((uint32_t)(x)) << WUU_PE2_WUPE24_SHIFT)) & WUU_PE2_WUPE24_MASK)

#define WUU_PE2_WUPE25_MASK                      (0xC0000U)
#define WUU_PE2_WUPE25_SHIFT                     (18U)
/*! WUPE25 - Wake-up Pin Enable for WUU_Pn
 *  0b00..Disable
 *  0b01..Enable (detect on rising edge or high level)
 *  0b10..Enable (detect on falling edge or low level)
 *  0b11..Enable (detect on any edge)
 */
#define WUU_PE2_WUPE25(x)                        (((uint32_t)(((uint32_t)(x)) << WUU_PE2_WUPE25_SHIFT)) & WUU_PE2_WUPE25_MASK)

#define WUU_PE2_WUPE26_MASK                      (0x300000U)
#define WUU_PE2_WUPE26_SHIFT                     (20U)
/*! WUPE26 - Wake-up Pin Enable for WUU_Pn
 *  0b00..Disable
 *  0b01..Enable (detect on rising edge or high level)
 *  0b10..Enable (detect on falling edge or low level)
 *  0b11..Enable (detect on any edge)
 */
#define WUU_PE2_WUPE26(x)                        (((uint32_t)(((uint32_t)(x)) << WUU_PE2_WUPE26_SHIFT)) & WUU_PE2_WUPE26_MASK)

#define WUU_PE2_WUPE27_MASK                      (0xC00000U)
#define WUU_PE2_WUPE27_SHIFT                     (22U)
/*! WUPE27 - Wake-up Pin Enable for WUU_Pn
 *  0b00..Disable
 *  0b01..Enable (detect on rising edge or high level)
 *  0b10..Enable (detect on falling edge or low level)
 *  0b11..Enable (detect on any edge)
 */
#define WUU_PE2_WUPE27(x)                        (((uint32_t)(((uint32_t)(x)) << WUU_PE2_WUPE27_SHIFT)) & WUU_PE2_WUPE27_MASK)

#define WUU_PE2_WUPE28_MASK                      (0x3000000U)
#define WUU_PE2_WUPE28_SHIFT                     (24U)
/*! WUPE28 - Wake-up Pin Enable for WUU_Pn
 *  0b00..Disable
 *  0b01..Enable (detect on rising edge or high level)
 *  0b10..Enable (detect on falling edge or low level)
 *  0b11..Enable (detect on any edge)
 */
#define WUU_PE2_WUPE28(x)                        (((uint32_t)(((uint32_t)(x)) << WUU_PE2_WUPE28_SHIFT)) & WUU_PE2_WUPE28_MASK)

#define WUU_PE2_WUPE29_MASK                      (0xC000000U)
#define WUU_PE2_WUPE29_SHIFT                     (26U)
/*! WUPE29 - Wake-up Pin Enable for WUU_Pn
 *  0b00..Disable
 *  0b01..Enable (detect on rising edge or high level)
 *  0b10..Enable (detect on falling edge or low level)
 *  0b11..Enable (detect on any edge)
 */
#define WUU_PE2_WUPE29(x)                        (((uint32_t)(((uint32_t)(x)) << WUU_PE2_WUPE29_SHIFT)) & WUU_PE2_WUPE29_MASK)

#define WUU_PE2_Reserved30_MASK                  (0x30000000U)
#define WUU_PE2_Reserved30_SHIFT                 (28U)
/*! Reserved30 - Reserved
 *  0b00..Not supported
 *  0b01..Not supported
 *  0b10..Not supported
 *  0b11..Not supported
 */
#define WUU_PE2_Reserved30(x)                    (((uint32_t)(((uint32_t)(x)) << WUU_PE2_Reserved30_SHIFT)) & WUU_PE2_Reserved30_MASK)

#define WUU_PE2_WUPE31_MASK                      (0xC0000000U)
#define WUU_PE2_WUPE31_SHIFT                     (30U)
/*! WUPE31 - Wake-up Pin Enable for WUU_Pn
 *  0b00..Disable
 *  0b01..Enable (detect on rising edge or high level)
 *  0b10..Enable (detect on falling edge or low level)
 *  0b11..Enable (detect on any edge)
 */
#define WUU_PE2_WUPE31(x)                        (((uint32_t)(((uint32_t)(x)) << WUU_PE2_WUPE31_SHIFT)) & WUU_PE2_WUPE31_MASK)
/*! @} */

/*! @name ME - Module Interrupt Enable */
/*! @{ */

#define WUU_ME_WUME0_MASK                        (0x1U)
#define WUU_ME_WUME0_SHIFT                       (0U)
/*! WUME0 - Module Interrupt Wake-up Enable for Module 0 */
#define WUU_ME_WUME0(x)                          (((uint32_t)(((uint32_t)(x)) << WUU_ME_WUME0_SHIFT)) & WUU_ME_WUME0_MASK)

#define WUU_ME_WUME2_MASK                        (0x4U)
#define WUU_ME_WUME2_SHIFT                       (2U)
/*! WUME2 - Module Interrupt Wake-up Enable for Module 2 */
#define WUU_ME_WUME2(x)                          (((uint32_t)(((uint32_t)(x)) << WUU_ME_WUME2_SHIFT)) & WUU_ME_WUME2_MASK)

#define WUU_ME_WUME6_MASK                        (0x40U)
#define WUU_ME_WUME6_SHIFT                       (6U)
/*! WUME6 - Module Interrupt Wake-up Enable for Module 6 */
#define WUU_ME_WUME6(x)                          (((uint32_t)(((uint32_t)(x)) << WUU_ME_WUME6_SHIFT)) & WUU_ME_WUME6_MASK)

#define WUU_ME_WUME8_MASK                        (0x100U)
#define WUU_ME_WUME8_SHIFT                       (8U)
/*! WUME8 - Module Interrupt Wake-up Enable for Module 8 */
#define WUU_ME_WUME8(x)                          (((uint32_t)(((uint32_t)(x)) << WUU_ME_WUME8_SHIFT)) & WUU_ME_WUME8_MASK)
/*! @} */

/*! @name DE - Module DMA/Trigger Enable */
/*! @{ */

#define WUU_DE_WUDE4_MASK                        (0x10U)
#define WUU_DE_WUDE4_SHIFT                       (4U)
/*! WUDE4 - DMA/Trigger Wake-up Enable for Module 4 */
#define WUU_DE_WUDE4(x)                          (((uint32_t)(((uint32_t)(x)) << WUU_DE_WUDE4_SHIFT)) & WUU_DE_WUDE4_MASK)

#define WUU_DE_WUDE6_MASK                        (0x40U)
#define WUU_DE_WUDE6_SHIFT                       (6U)
/*! WUDE6 - DMA/Trigger Wake-up Enable for Module 6 */
#define WUU_DE_WUDE6(x)                          (((uint32_t)(((uint32_t)(x)) << WUU_DE_WUDE6_SHIFT)) & WUU_DE_WUDE6_MASK)

#define WUU_DE_WUDE8_MASK                        (0x100U)
#define WUU_DE_WUDE8_SHIFT                       (8U)
/*! WUDE8 - DMA/Trigger Wake-up Enable for Module 8 */
#define WUU_DE_WUDE8(x)                          (((uint32_t)(((uint32_t)(x)) << WUU_DE_WUDE8_SHIFT)) & WUU_DE_WUDE8_MASK)
/*! @} */

/*! @name PF - Pin Flag */
/*! @{ */

#define WUU_PF_Reserved0_MASK                    (0x1U)
#define WUU_PF_Reserved0_SHIFT                   (0U)
/*! Reserved0 - Reserved
 *  0b0..Not supported
 *  0b1..Not supported
 */
#define WUU_PF_Reserved0(x)                      (((uint32_t)(((uint32_t)(x)) << WUU_PF_Reserved0_SHIFT)) & WUU_PF_Reserved0_MASK)

#define WUU_PF_Reserved1_MASK                    (0x2U)
#define WUU_PF_Reserved1_SHIFT                   (1U)
/*! Reserved1 - Reserved
 *  0b0..Not supported
 *  0b1..Not supported
 */
#define WUU_PF_Reserved1(x)                      (((uint32_t)(((uint32_t)(x)) << WUU_PF_Reserved1_SHIFT)) & WUU_PF_Reserved1_MASK)

#define WUU_PF_WUF2_MASK                         (0x4U)
#define WUU_PF_WUF2_SHIFT                        (2U)
/*! WUF2 - Wake-up Flag for WUU_Pn
 *  0b0..No
 *  0b1..Yes
 */
#define WUU_PF_WUF2(x)                           (((uint32_t)(((uint32_t)(x)) << WUU_PF_WUF2_SHIFT)) & WUU_PF_WUF2_MASK)

#define WUU_PF_Reserved3_MASK                    (0x8U)
#define WUU_PF_Reserved3_SHIFT                   (3U)
/*! Reserved3 - Reserved
 *  0b0..Not supported
 *  0b1..Not supported
 */
#define WUU_PF_Reserved3(x)                      (((uint32_t)(((uint32_t)(x)) << WUU_PF_Reserved3_SHIFT)) & WUU_PF_Reserved3_MASK)

#define WUU_PF_Reserved4_MASK                    (0x10U)
#define WUU_PF_Reserved4_SHIFT                   (4U)
/*! Reserved4 - Reserved
 *  0b0..Not supported
 *  0b1..Not supported
 */
#define WUU_PF_Reserved4(x)                      (((uint32_t)(((uint32_t)(x)) << WUU_PF_Reserved4_SHIFT)) & WUU_PF_Reserved4_MASK)

#define WUU_PF_Reserved5_MASK                    (0x20U)
#define WUU_PF_Reserved5_SHIFT                   (5U)
/*! Reserved5 - Reserved
 *  0b0..Not supported
 *  0b1..Not supported
 */
#define WUU_PF_Reserved5(x)                      (((uint32_t)(((uint32_t)(x)) << WUU_PF_Reserved5_SHIFT)) & WUU_PF_Reserved5_MASK)

#define WUU_PF_WUF6_MASK                         (0x40U)
#define WUU_PF_WUF6_SHIFT                        (6U)
/*! WUF6 - Wake-up Flag for WUU_Pn
 *  0b0..No
 *  0b1..Yes
 */
#define WUU_PF_WUF6(x)                           (((uint32_t)(((uint32_t)(x)) << WUU_PF_WUF6_SHIFT)) & WUU_PF_WUF6_MASK)

#define WUU_PF_WUF7_MASK                         (0x80U)
#define WUU_PF_WUF7_SHIFT                        (7U)
/*! WUF7 - Wake-up Flag for WUU_Pn
 *  0b0..No
 *  0b1..Yes
 */
#define WUU_PF_WUF7(x)                           (((uint32_t)(((uint32_t)(x)) << WUU_PF_WUF7_SHIFT)) & WUU_PF_WUF7_MASK)

#define WUU_PF_WUF8_MASK                         (0x100U)
#define WUU_PF_WUF8_SHIFT                        (8U)
/*! WUF8 - Wake-up Flag for WUU_Pn
 *  0b0..No
 *  0b1..Yes
 */
#define WUU_PF_WUF8(x)                           (((uint32_t)(((uint32_t)(x)) << WUU_PF_WUF8_SHIFT)) & WUU_PF_WUF8_MASK)

#define WUU_PF_WUF9_MASK                         (0x200U)
#define WUU_PF_WUF9_SHIFT                        (9U)
/*! WUF9 - Wake-up Flag for WUU_Pn
 *  0b0..No
 *  0b1..Yes
 */
#define WUU_PF_WUF9(x)                           (((uint32_t)(((uint32_t)(x)) << WUU_PF_WUF9_SHIFT)) & WUU_PF_WUF9_MASK)

#define WUU_PF_WUF10_MASK                        (0x400U)
#define WUU_PF_WUF10_SHIFT                       (10U)
/*! WUF10 - Wake-up Flag for WUU_Pn
 *  0b0..No
 *  0b1..Yes
 */
#define WUU_PF_WUF10(x)                          (((uint32_t)(((uint32_t)(x)) << WUU_PF_WUF10_SHIFT)) & WUU_PF_WUF10_MASK)

#define WUU_PF_WUF11_MASK                        (0x800U)
#define WUU_PF_WUF11_SHIFT                       (11U)
/*! WUF11 - Wake-up Flag for WUU_Pn
 *  0b0..No
 *  0b1..Yes
 */
#define WUU_PF_WUF11(x)                          (((uint32_t)(((uint32_t)(x)) << WUU_PF_WUF11_SHIFT)) & WUU_PF_WUF11_MASK)

#define WUU_PF_WUF12_MASK                        (0x1000U)
#define WUU_PF_WUF12_SHIFT                       (12U)
/*! WUF12 - Wake-up Flag for WUU_Pn
 *  0b0..No
 *  0b1..Yes
 */
#define WUU_PF_WUF12(x)                          (((uint32_t)(((uint32_t)(x)) << WUU_PF_WUF12_SHIFT)) & WUU_PF_WUF12_MASK)

#define WUU_PF_Reserved13_MASK                   (0x2000U)
#define WUU_PF_Reserved13_SHIFT                  (13U)
/*! Reserved13 - Reserved
 *  0b0..Not supported
 *  0b1..Not supported
 */
#define WUU_PF_Reserved13(x)                     (((uint32_t)(((uint32_t)(x)) << WUU_PF_Reserved13_SHIFT)) & WUU_PF_Reserved13_MASK)

#define WUU_PF_Reserved14_MASK                   (0x4000U)
#define WUU_PF_Reserved14_SHIFT                  (14U)
/*! Reserved14 - Reserved
 *  0b0..Not supported
 *  0b1..Not supported
 */
#define WUU_PF_Reserved14(x)                     (((uint32_t)(((uint32_t)(x)) << WUU_PF_Reserved14_SHIFT)) & WUU_PF_Reserved14_MASK)

#define WUU_PF_Reserved15_MASK                   (0x8000U)
#define WUU_PF_Reserved15_SHIFT                  (15U)
/*! Reserved15 - Reserved
 *  0b0..Not supported
 *  0b1..Not supported
 */
#define WUU_PF_Reserved15(x)                     (((uint32_t)(((uint32_t)(x)) << WUU_PF_Reserved15_SHIFT)) & WUU_PF_Reserved15_MASK)

#define WUU_PF_Reserved16_MASK                   (0x10000U)
#define WUU_PF_Reserved16_SHIFT                  (16U)
/*! Reserved16 - Reserved
 *  0b0..Not supported
 *  0b1..Not supported
 */
#define WUU_PF_Reserved16(x)                     (((uint32_t)(((uint32_t)(x)) << WUU_PF_Reserved16_SHIFT)) & WUU_PF_Reserved16_MASK)

#define WUU_PF_Reserved17_MASK                   (0x20000U)
#define WUU_PF_Reserved17_SHIFT                  (17U)
/*! Reserved17 - Reserved
 *  0b0..Not supported
 *  0b1..Not supported
 */
#define WUU_PF_Reserved17(x)                     (((uint32_t)(((uint32_t)(x)) << WUU_PF_Reserved17_SHIFT)) & WUU_PF_Reserved17_MASK)

#define WUU_PF_WUF18_MASK                        (0x40000U)
#define WUU_PF_WUF18_SHIFT                       (18U)
/*! WUF18 - Wake-up Flag for WUU_Pn
 *  0b0..No
 *  0b1..Yes
 */
#define WUU_PF_WUF18(x)                          (((uint32_t)(((uint32_t)(x)) << WUU_PF_WUF18_SHIFT)) & WUU_PF_WUF18_MASK)

#define WUU_PF_WUF19_MASK                        (0x80000U)
#define WUU_PF_WUF19_SHIFT                       (19U)
/*! WUF19 - Wake-up Flag for WUU_Pn
 *  0b0..No
 *  0b1..Yes
 */
#define WUU_PF_WUF19(x)                          (((uint32_t)(((uint32_t)(x)) << WUU_PF_WUF19_SHIFT)) & WUU_PF_WUF19_MASK)

#define WUU_PF_WUF20_MASK                        (0x100000U)
#define WUU_PF_WUF20_SHIFT                       (20U)
/*! WUF20 - Wake-up Flag for WUU_Pn
 *  0b0..No
 *  0b1..Yes
 */
#define WUU_PF_WUF20(x)                          (((uint32_t)(((uint32_t)(x)) << WUU_PF_WUF20_SHIFT)) & WUU_PF_WUF20_MASK)

#define WUU_PF_Reserved21_MASK                   (0x200000U)
#define WUU_PF_Reserved21_SHIFT                  (21U)
/*! Reserved21 - Reserved
 *  0b0..Not supported
 *  0b1..Not supported
 */
#define WUU_PF_Reserved21(x)                     (((uint32_t)(((uint32_t)(x)) << WUU_PF_Reserved21_SHIFT)) & WUU_PF_Reserved21_MASK)

#define WUU_PF_WUF22_MASK                        (0x400000U)
#define WUU_PF_WUF22_SHIFT                       (22U)
/*! WUF22 - Wake-up Flag for WUU_Pn
 *  0b0..No
 *  0b1..Yes
 */
#define WUU_PF_WUF22(x)                          (((uint32_t)(((uint32_t)(x)) << WUU_PF_WUF22_SHIFT)) & WUU_PF_WUF22_MASK)

#define WUU_PF_WUF23_MASK                        (0x800000U)
#define WUU_PF_WUF23_SHIFT                       (23U)
/*! WUF23 - Wake-up Flag for WUU_Pn
 *  0b0..No
 *  0b1..Yes
 */
#define WUU_PF_WUF23(x)                          (((uint32_t)(((uint32_t)(x)) << WUU_PF_WUF23_SHIFT)) & WUU_PF_WUF23_MASK)

#define WUU_PF_WUF24_MASK                        (0x1000000U)
#define WUU_PF_WUF24_SHIFT                       (24U)
/*! WUF24 - Wake-up Flag for WUU_Pn
 *  0b0..No
 *  0b1..Yes
 */
#define WUU_PF_WUF24(x)                          (((uint32_t)(((uint32_t)(x)) << WUU_PF_WUF24_SHIFT)) & WUU_PF_WUF24_MASK)

#define WUU_PF_WUF25_MASK                        (0x2000000U)
#define WUU_PF_WUF25_SHIFT                       (25U)
/*! WUF25 - Wake-up Flag for WUU_Pn
 *  0b0..No
 *  0b1..Yes
 */
#define WUU_PF_WUF25(x)                          (((uint32_t)(((uint32_t)(x)) << WUU_PF_WUF25_SHIFT)) & WUU_PF_WUF25_MASK)

#define WUU_PF_WUF26_MASK                        (0x4000000U)
#define WUU_PF_WUF26_SHIFT                       (26U)
/*! WUF26 - Wake-up Flag for WUU_Pn
 *  0b0..No
 *  0b1..Yes
 */
#define WUU_PF_WUF26(x)                          (((uint32_t)(((uint32_t)(x)) << WUU_PF_WUF26_SHIFT)) & WUU_PF_WUF26_MASK)

#define WUU_PF_WUF27_MASK                        (0x8000000U)
#define WUU_PF_WUF27_SHIFT                       (27U)
/*! WUF27 - Wake-up Flag for WUU_Pn
 *  0b0..No
 *  0b1..Yes
 */
#define WUU_PF_WUF27(x)                          (((uint32_t)(((uint32_t)(x)) << WUU_PF_WUF27_SHIFT)) & WUU_PF_WUF27_MASK)

#define WUU_PF_WUF28_MASK                        (0x10000000U)
#define WUU_PF_WUF28_SHIFT                       (28U)
/*! WUF28 - Wake-up Flag for WUU_Pn
 *  0b0..No
 *  0b1..Yes
 */
#define WUU_PF_WUF28(x)                          (((uint32_t)(((uint32_t)(x)) << WUU_PF_WUF28_SHIFT)) & WUU_PF_WUF28_MASK)

#define WUU_PF_WUF29_MASK                        (0x20000000U)
#define WUU_PF_WUF29_SHIFT                       (29U)
/*! WUF29 - Wake-up Flag for WUU_Pn
 *  0b0..No
 *  0b1..Yes
 */
#define WUU_PF_WUF29(x)                          (((uint32_t)(((uint32_t)(x)) << WUU_PF_WUF29_SHIFT)) & WUU_PF_WUF29_MASK)

#define WUU_PF_Reserved30_MASK                   (0x40000000U)
#define WUU_PF_Reserved30_SHIFT                  (30U)
/*! Reserved30 - Reserved
 *  0b0..Not supported
 *  0b1..Not supported
 */
#define WUU_PF_Reserved30(x)                     (((uint32_t)(((uint32_t)(x)) << WUU_PF_Reserved30_SHIFT)) & WUU_PF_Reserved30_MASK)

#define WUU_PF_WUF31_MASK                        (0x80000000U)
#define WUU_PF_WUF31_SHIFT                       (31U)
/*! WUF31 - Wake-up Flag for WUU_Pn
 *  0b0..No
 *  0b1..Yes
 */
#define WUU_PF_WUF31(x)                          (((uint32_t)(((uint32_t)(x)) << WUU_PF_WUF31_SHIFT)) & WUU_PF_WUF31_MASK)
/*! @} */

/*! @name FILT - Pin Filter */
/*! @{ */

#define WUU_FILT_FILTSEL1_MASK                   (0x1FU)
#define WUU_FILT_FILTSEL1_SHIFT                  (0U)
/*! FILTSEL1 - Filter 1 Pin Select */
#define WUU_FILT_FILTSEL1(x)                     (((uint32_t)(((uint32_t)(x)) << WUU_FILT_FILTSEL1_SHIFT)) & WUU_FILT_FILTSEL1_MASK)

#define WUU_FILT_FILTE1_MASK                     (0x60U)
#define WUU_FILT_FILTE1_SHIFT                    (5U)
/*! FILTE1 - Filter 1 Enable
 *  0b00..Disable
 *  0b01..Enable (Detect on rising edge or high level)
 *  0b10..Enable (Detect on falling edge or low level)
 *  0b11..Enable (Detect on any edge)
 */
#define WUU_FILT_FILTE1(x)                       (((uint32_t)(((uint32_t)(x)) << WUU_FILT_FILTE1_SHIFT)) & WUU_FILT_FILTE1_MASK)

#define WUU_FILT_FILTF1_MASK                     (0x80U)
#define WUU_FILT_FILTF1_SHIFT                    (7U)
/*! FILTF1 - Filter 1 Flag
 *  0b0..No
 *  0b1..Yes
 */
#define WUU_FILT_FILTF1(x)                       (((uint32_t)(((uint32_t)(x)) << WUU_FILT_FILTF1_SHIFT)) & WUU_FILT_FILTF1_MASK)

#define WUU_FILT_FILTSEL2_MASK                   (0x1F00U)
#define WUU_FILT_FILTSEL2_SHIFT                  (8U)
/*! FILTSEL2 - Filter 2 Pin Select */
#define WUU_FILT_FILTSEL2(x)                     (((uint32_t)(((uint32_t)(x)) << WUU_FILT_FILTSEL2_SHIFT)) & WUU_FILT_FILTSEL2_MASK)

#define WUU_FILT_FILTE2_MASK                     (0x6000U)
#define WUU_FILT_FILTE2_SHIFT                    (13U)
/*! FILTE2 - Filter 2 Enable
 *  0b00..Disable
 *  0b01..Enable (Detect on rising edge or high level)
 *  0b10..Enable (Detect on falling edge or low level)
 *  0b11..Enable (Detect on any edge)
 */
#define WUU_FILT_FILTE2(x)                       (((uint32_t)(((uint32_t)(x)) << WUU_FILT_FILTE2_SHIFT)) & WUU_FILT_FILTE2_MASK)

#define WUU_FILT_FILTF2_MASK                     (0x8000U)
#define WUU_FILT_FILTF2_SHIFT                    (15U)
/*! FILTF2 - Filter 2 Flag
 *  0b0..No
 *  0b1..Yes
 */
#define WUU_FILT_FILTF2(x)                       (((uint32_t)(((uint32_t)(x)) << WUU_FILT_FILTF2_SHIFT)) & WUU_FILT_FILTF2_MASK)
/*! @} */

/*! @name PDC1 - Pin DMA/Trigger Configuration 1 */
/*! @{ */

#define WUU_PDC1_Reserved0_MASK                  (0x3U)
#define WUU_PDC1_Reserved0_SHIFT                 (0U)
/*! Reserved0 - Reserved
 *  0b00..Not supported
 *  0b01..Not supported
 *  0b10..Not supported
 *  0b11..Not supported
 */
#define WUU_PDC1_Reserved0(x)                    (((uint32_t)(((uint32_t)(x)) << WUU_PDC1_Reserved0_SHIFT)) & WUU_PDC1_Reserved0_MASK)

#define WUU_PDC1_Reserved1_MASK                  (0xCU)
#define WUU_PDC1_Reserved1_SHIFT                 (2U)
/*! Reserved1 - Reserved
 *  0b00..Not supported
 *  0b01..Not supported
 *  0b10..Not supported
 *  0b11..Not supported
 */
#define WUU_PDC1_Reserved1(x)                    (((uint32_t)(((uint32_t)(x)) << WUU_PDC1_Reserved1_SHIFT)) & WUU_PDC1_Reserved1_MASK)

#define WUU_PDC1_WUPDC2_MASK                     (0x30U)
#define WUU_PDC1_WUPDC2_SHIFT                    (4U)
/*! WUPDC2 - Wake-up Pin Configuration for WUU_Pn
 *  0b00..Interrupt
 *  0b01..DMA request
 *  0b10..Trigger event
 *  0b11..Reserved
 */
#define WUU_PDC1_WUPDC2(x)                       (((uint32_t)(((uint32_t)(x)) << WUU_PDC1_WUPDC2_SHIFT)) & WUU_PDC1_WUPDC2_MASK)

#define WUU_PDC1_Reserved3_MASK                  (0xC0U)
#define WUU_PDC1_Reserved3_SHIFT                 (6U)
/*! Reserved3 - Reserved
 *  0b00..Not supported
 *  0b01..Not supported
 *  0b10..Not supported
 *  0b11..Not supported
 */
#define WUU_PDC1_Reserved3(x)                    (((uint32_t)(((uint32_t)(x)) << WUU_PDC1_Reserved3_SHIFT)) & WUU_PDC1_Reserved3_MASK)

#define WUU_PDC1_Reserved4_MASK                  (0x300U)
#define WUU_PDC1_Reserved4_SHIFT                 (8U)
/*! Reserved4 - Reserved
 *  0b00..Not supported
 *  0b01..Not supported
 *  0b10..Not supported
 *  0b11..Not supported
 */
#define WUU_PDC1_Reserved4(x)                    (((uint32_t)(((uint32_t)(x)) << WUU_PDC1_Reserved4_SHIFT)) & WUU_PDC1_Reserved4_MASK)

#define WUU_PDC1_Reserved5_MASK                  (0xC00U)
#define WUU_PDC1_Reserved5_SHIFT                 (10U)
/*! Reserved5 - Reserved
 *  0b00..Not supported
 *  0b01..Not supported
 *  0b10..Not supported
 *  0b11..Not supported
 */
#define WUU_PDC1_Reserved5(x)                    (((uint32_t)(((uint32_t)(x)) << WUU_PDC1_Reserved5_SHIFT)) & WUU_PDC1_Reserved5_MASK)

#define WUU_PDC1_WUPDC6_MASK                     (0x3000U)
#define WUU_PDC1_WUPDC6_SHIFT                    (12U)
/*! WUPDC6 - Wake-up Pin Configuration for WUU_Pn
 *  0b00..Interrupt
 *  0b01..DMA request
 *  0b10..Trigger event
 *  0b11..Reserved
 */
#define WUU_PDC1_WUPDC6(x)                       (((uint32_t)(((uint32_t)(x)) << WUU_PDC1_WUPDC6_SHIFT)) & WUU_PDC1_WUPDC6_MASK)

#define WUU_PDC1_WUPDC7_MASK                     (0xC000U)
#define WUU_PDC1_WUPDC7_SHIFT                    (14U)
/*! WUPDC7 - Wake-up Pin Configuration for WUU_Pn
 *  0b00..Interrupt
 *  0b01..DMA request
 *  0b10..Trigger event
 *  0b11..Reserved
 */
#define WUU_PDC1_WUPDC7(x)                       (((uint32_t)(((uint32_t)(x)) << WUU_PDC1_WUPDC7_SHIFT)) & WUU_PDC1_WUPDC7_MASK)

#define WUU_PDC1_WUPDC8_MASK                     (0x30000U)
#define WUU_PDC1_WUPDC8_SHIFT                    (16U)
/*! WUPDC8 - Wake-up Pin Configuration for WUU_Pn
 *  0b00..Interrupt
 *  0b01..DMA request
 *  0b10..Trigger event
 *  0b11..Reserved
 */
#define WUU_PDC1_WUPDC8(x)                       (((uint32_t)(((uint32_t)(x)) << WUU_PDC1_WUPDC8_SHIFT)) & WUU_PDC1_WUPDC8_MASK)

#define WUU_PDC1_WUPDC9_MASK                     (0xC0000U)
#define WUU_PDC1_WUPDC9_SHIFT                    (18U)
/*! WUPDC9 - Wake-up Pin Configuration for WUU_Pn
 *  0b00..Interrupt
 *  0b01..DMA request
 *  0b10..Trigger event
 *  0b11..Reserved
 */
#define WUU_PDC1_WUPDC9(x)                       (((uint32_t)(((uint32_t)(x)) << WUU_PDC1_WUPDC9_SHIFT)) & WUU_PDC1_WUPDC9_MASK)

#define WUU_PDC1_WUPDC10_MASK                    (0x300000U)
#define WUU_PDC1_WUPDC10_SHIFT                   (20U)
/*! WUPDC10 - Wake-up Pin Configuration for WUU_Pn
 *  0b00..Interrupt
 *  0b01..DMA request
 *  0b10..Trigger event
 *  0b11..Reserved
 */
#define WUU_PDC1_WUPDC10(x)                      (((uint32_t)(((uint32_t)(x)) << WUU_PDC1_WUPDC10_SHIFT)) & WUU_PDC1_WUPDC10_MASK)

#define WUU_PDC1_WUPDC11_MASK                    (0xC00000U)
#define WUU_PDC1_WUPDC11_SHIFT                   (22U)
/*! WUPDC11 - Wake-up Pin Configuration for WUU_Pn
 *  0b00..Interrupt
 *  0b01..DMA request
 *  0b10..Trigger event
 *  0b11..Reserved
 */
#define WUU_PDC1_WUPDC11(x)                      (((uint32_t)(((uint32_t)(x)) << WUU_PDC1_WUPDC11_SHIFT)) & WUU_PDC1_WUPDC11_MASK)

#define WUU_PDC1_WUPDC12_MASK                    (0x3000000U)
#define WUU_PDC1_WUPDC12_SHIFT                   (24U)
/*! WUPDC12 - Wake-up Pin Configuration for WUU_Pn
 *  0b00..Interrupt
 *  0b01..DMA request
 *  0b10..Trigger event
 *  0b11..Reserved
 */
#define WUU_PDC1_WUPDC12(x)                      (((uint32_t)(((uint32_t)(x)) << WUU_PDC1_WUPDC12_SHIFT)) & WUU_PDC1_WUPDC12_MASK)

#define WUU_PDC1_Reserved13_MASK                 (0xC000000U)
#define WUU_PDC1_Reserved13_SHIFT                (26U)
/*! Reserved13 - Reserved
 *  0b00..Not supported
 *  0b01..Not supported
 *  0b10..Not supported
 *  0b11..Not supported
 */
#define WUU_PDC1_Reserved13(x)                   (((uint32_t)(((uint32_t)(x)) << WUU_PDC1_Reserved13_SHIFT)) & WUU_PDC1_Reserved13_MASK)

#define WUU_PDC1_Reserved14_MASK                 (0x30000000U)
#define WUU_PDC1_Reserved14_SHIFT                (28U)
/*! Reserved14 - Reserved
 *  0b00..Not supported
 *  0b01..Not supported
 *  0b10..Not supported
 *  0b11..Not supported
 */
#define WUU_PDC1_Reserved14(x)                   (((uint32_t)(((uint32_t)(x)) << WUU_PDC1_Reserved14_SHIFT)) & WUU_PDC1_Reserved14_MASK)

#define WUU_PDC1_Reserved15_MASK                 (0xC0000000U)
#define WUU_PDC1_Reserved15_SHIFT                (30U)
/*! Reserved15 - Reserved
 *  0b00..Not supported
 *  0b01..Not supported
 *  0b10..Not supported
 *  0b11..Not supported
 */
#define WUU_PDC1_Reserved15(x)                   (((uint32_t)(((uint32_t)(x)) << WUU_PDC1_Reserved15_SHIFT)) & WUU_PDC1_Reserved15_MASK)
/*! @} */

/*! @name PDC2 - Pin DMA/Trigger Configuration 2 */
/*! @{ */

#define WUU_PDC2_Reserved16_MASK                 (0x3U)
#define WUU_PDC2_Reserved16_SHIFT                (0U)
/*! Reserved16 - Reserved
 *  0b00..Not supported
 *  0b01..Not supported
 *  0b10..Not supported
 *  0b11..Not supported
 */
#define WUU_PDC2_Reserved16(x)                   (((uint32_t)(((uint32_t)(x)) << WUU_PDC2_Reserved16_SHIFT)) & WUU_PDC2_Reserved16_MASK)

#define WUU_PDC2_Reserved17_MASK                 (0xCU)
#define WUU_PDC2_Reserved17_SHIFT                (2U)
/*! Reserved17 - Reserved
 *  0b00..Not supported
 *  0b01..Not supported
 *  0b10..Not supported
 *  0b11..Not supported
 */
#define WUU_PDC2_Reserved17(x)                   (((uint32_t)(((uint32_t)(x)) << WUU_PDC2_Reserved17_SHIFT)) & WUU_PDC2_Reserved17_MASK)

#define WUU_PDC2_WUPDC18_MASK                    (0x30U)
#define WUU_PDC2_WUPDC18_SHIFT                   (4U)
/*! WUPDC18 - Wake-up Pin Configuration for WUU_Pn
 *  0b00..Interrupt
 *  0b01..DMA request
 *  0b10..Trigger event
 *  0b11..Reserved
 */
#define WUU_PDC2_WUPDC18(x)                      (((uint32_t)(((uint32_t)(x)) << WUU_PDC2_WUPDC18_SHIFT)) & WUU_PDC2_WUPDC18_MASK)

#define WUU_PDC2_WUPDC19_MASK                    (0xC0U)
#define WUU_PDC2_WUPDC19_SHIFT                   (6U)
/*! WUPDC19 - Wake-up Pin Configuration for WUU_Pn
 *  0b00..Interrupt
 *  0b01..DMA request
 *  0b10..Trigger event
 *  0b11..Reserved
 */
#define WUU_PDC2_WUPDC19(x)                      (((uint32_t)(((uint32_t)(x)) << WUU_PDC2_WUPDC19_SHIFT)) & WUU_PDC2_WUPDC19_MASK)

#define WUU_PDC2_WUPDC20_MASK                    (0x300U)
#define WUU_PDC2_WUPDC20_SHIFT                   (8U)
/*! WUPDC20 - Wake-up Pin Configuration for WUU_Pn
 *  0b00..Interrupt
 *  0b01..DMA request
 *  0b10..Trigger event
 *  0b11..Reserved
 */
#define WUU_PDC2_WUPDC20(x)                      (((uint32_t)(((uint32_t)(x)) << WUU_PDC2_WUPDC20_SHIFT)) & WUU_PDC2_WUPDC20_MASK)

#define WUU_PDC2_Reserved21_MASK                 (0xC00U)
#define WUU_PDC2_Reserved21_SHIFT                (10U)
/*! Reserved21 - Reserved
 *  0b00..Not supported
 *  0b01..Not supported
 *  0b10..Not supported
 *  0b11..Not supported
 */
#define WUU_PDC2_Reserved21(x)                   (((uint32_t)(((uint32_t)(x)) << WUU_PDC2_Reserved21_SHIFT)) & WUU_PDC2_Reserved21_MASK)

#define WUU_PDC2_WUPDC22_MASK                    (0x3000U)
#define WUU_PDC2_WUPDC22_SHIFT                   (12U)
/*! WUPDC22 - Wake-up Pin Configuration for WUU_Pn
 *  0b00..Interrupt
 *  0b01..DMA request
 *  0b10..Trigger event
 *  0b11..Reserved
 */
#define WUU_PDC2_WUPDC22(x)                      (((uint32_t)(((uint32_t)(x)) << WUU_PDC2_WUPDC22_SHIFT)) & WUU_PDC2_WUPDC22_MASK)

#define WUU_PDC2_WUPDC23_MASK                    (0xC000U)
#define WUU_PDC2_WUPDC23_SHIFT                   (14U)
/*! WUPDC23 - Wake-up Pin Configuration for WUU_Pn
 *  0b00..Interrupt
 *  0b01..DMA request
 *  0b10..Trigger event
 *  0b11..Reserved
 */
#define WUU_PDC2_WUPDC23(x)                      (((uint32_t)(((uint32_t)(x)) << WUU_PDC2_WUPDC23_SHIFT)) & WUU_PDC2_WUPDC23_MASK)

#define WUU_PDC2_WUPDC24_MASK                    (0x30000U)
#define WUU_PDC2_WUPDC24_SHIFT                   (16U)
/*! WUPDC24 - Wake-up Pin Configuration for WUU_Pn
 *  0b00..Interrupt
 *  0b01..DMA request
 *  0b10..Trigger event
 *  0b11..Reserved
 */
#define WUU_PDC2_WUPDC24(x)                      (((uint32_t)(((uint32_t)(x)) << WUU_PDC2_WUPDC24_SHIFT)) & WUU_PDC2_WUPDC24_MASK)

#define WUU_PDC2_WUPDC25_MASK                    (0xC0000U)
#define WUU_PDC2_WUPDC25_SHIFT                   (18U)
/*! WUPDC25 - Wake-up Pin Configuration for WUU_Pn
 *  0b00..Interrupt
 *  0b01..DMA request
 *  0b10..Trigger event
 *  0b11..Reserved
 */
#define WUU_PDC2_WUPDC25(x)                      (((uint32_t)(((uint32_t)(x)) << WUU_PDC2_WUPDC25_SHIFT)) & WUU_PDC2_WUPDC25_MASK)

#define WUU_PDC2_WUPDC26_MASK                    (0x300000U)
#define WUU_PDC2_WUPDC26_SHIFT                   (20U)
/*! WUPDC26 - Wake-up Pin Configuration for WUU_Pn
 *  0b00..Interrupt
 *  0b01..DMA request
 *  0b10..Trigger event
 *  0b11..Reserved
 */
#define WUU_PDC2_WUPDC26(x)                      (((uint32_t)(((uint32_t)(x)) << WUU_PDC2_WUPDC26_SHIFT)) & WUU_PDC2_WUPDC26_MASK)

#define WUU_PDC2_WUPDC27_MASK                    (0xC00000U)
#define WUU_PDC2_WUPDC27_SHIFT                   (22U)
/*! WUPDC27 - Wake-up Pin Configuration for WUU_Pn
 *  0b00..Interrupt
 *  0b01..DMA request
 *  0b10..Trigger event
 *  0b11..Reserved
 */
#define WUU_PDC2_WUPDC27(x)                      (((uint32_t)(((uint32_t)(x)) << WUU_PDC2_WUPDC27_SHIFT)) & WUU_PDC2_WUPDC27_MASK)

#define WUU_PDC2_WUPDC28_MASK                    (0x3000000U)
#define WUU_PDC2_WUPDC28_SHIFT                   (24U)
/*! WUPDC28 - Wake-up Pin Configuration for WUU_Pn
 *  0b00..Interrupt
 *  0b01..DMA request
 *  0b10..Trigger event
 *  0b11..Reserved
 */
#define WUU_PDC2_WUPDC28(x)                      (((uint32_t)(((uint32_t)(x)) << WUU_PDC2_WUPDC28_SHIFT)) & WUU_PDC2_WUPDC28_MASK)

#define WUU_PDC2_WUPDC29_MASK                    (0xC000000U)
#define WUU_PDC2_WUPDC29_SHIFT                   (26U)
/*! WUPDC29 - Wake-up Pin Configuration for WUU_Pn
 *  0b00..Interrupt
 *  0b01..DMA request
 *  0b10..Trigger event
 *  0b11..Reserved
 */
#define WUU_PDC2_WUPDC29(x)                      (((uint32_t)(((uint32_t)(x)) << WUU_PDC2_WUPDC29_SHIFT)) & WUU_PDC2_WUPDC29_MASK)

#define WUU_PDC2_Reserved30_MASK                 (0x30000000U)
#define WUU_PDC2_Reserved30_SHIFT                (28U)
/*! Reserved30 - Reserved
 *  0b00..Not supported
 *  0b01..Not supported
 *  0b10..Not supported
 *  0b11..Not supported
 */
#define WUU_PDC2_Reserved30(x)                   (((uint32_t)(((uint32_t)(x)) << WUU_PDC2_Reserved30_SHIFT)) & WUU_PDC2_Reserved30_MASK)

#define WUU_PDC2_WUPDC31_MASK                    (0xC0000000U)
#define WUU_PDC2_WUPDC31_SHIFT                   (30U)
/*! WUPDC31 - Wake-up Pin Configuration for WUU_Pn
 *  0b00..Interrupt
 *  0b01..DMA request
 *  0b10..Trigger event
 *  0b11..Reserved
 */
#define WUU_PDC2_WUPDC31(x)                      (((uint32_t)(((uint32_t)(x)) << WUU_PDC2_WUPDC31_SHIFT)) & WUU_PDC2_WUPDC31_MASK)
/*! @} */

/*! @name FDC - Pin Filter DMA/Trigger Configuration */
/*! @{ */

#define WUU_FDC_FILTC1_MASK                      (0x3U)
#define WUU_FDC_FILTC1_SHIFT                     (0U)
/*! FILTC1 - Filter Configuration for FILTn
 *  0b00..Interrupt
 *  0b01..DMA request
 *  0b10..Trigger event
 *  0b11..Reserved
 */
#define WUU_FDC_FILTC1(x)                        (((uint32_t)(((uint32_t)(x)) << WUU_FDC_FILTC1_SHIFT)) & WUU_FDC_FILTC1_MASK)

#define WUU_FDC_FILTC2_MASK                      (0xCU)
#define WUU_FDC_FILTC2_SHIFT                     (2U)
/*! FILTC2 - Filter Configuration for FILTn
 *  0b00..Interrupt
 *  0b01..DMA request
 *  0b10..Trigger event
 *  0b11..Reserved
 */
#define WUU_FDC_FILTC2(x)                        (((uint32_t)(((uint32_t)(x)) << WUU_FDC_FILTC2_SHIFT)) & WUU_FDC_FILTC2_MASK)
/*! @} */

/*! @name PMC - Pin Mode Configuration */
/*! @{ */

#define WUU_PMC_Reserved0_MASK                   (0x1U)
#define WUU_PMC_Reserved0_SHIFT                  (0U)
/*! Reserved0 - Reserved
 *  0b0..Not supported
 *  0b1..Not supported
 */
#define WUU_PMC_Reserved0(x)                     (((uint32_t)(((uint32_t)(x)) << WUU_PMC_Reserved0_SHIFT)) & WUU_PMC_Reserved0_MASK)

#define WUU_PMC_Reserved1_MASK                   (0x2U)
#define WUU_PMC_Reserved1_SHIFT                  (1U)
/*! Reserved1 - Reserved
 *  0b0..Not supported
 *  0b1..Not supported
 */
#define WUU_PMC_Reserved1(x)                     (((uint32_t)(((uint32_t)(x)) << WUU_PMC_Reserved1_SHIFT)) & WUU_PMC_Reserved1_MASK)

#define WUU_PMC_WUPMC2_MASK                      (0x4U)
#define WUU_PMC_WUPMC2_SHIFT                     (2U)
/*! WUPMC2 - Wake-up Pin Mode Configuration for WUU_Pn
 *  0b0..Active only during a low-leakage mode. You can modify the corresponding fields within Pin Enable (PEn) or
 *       Pin DMA/Trigger Configuration (PDCn).
 *  0b1..Active during all power modes. Do not modify the corresponding fields within Pin Enable (PEn) or Pin DMA/Trigger Configuration (PDCn).
 */
#define WUU_PMC_WUPMC2(x)                        (((uint32_t)(((uint32_t)(x)) << WUU_PMC_WUPMC2_SHIFT)) & WUU_PMC_WUPMC2_MASK)

#define WUU_PMC_Reserved3_MASK                   (0x8U)
#define WUU_PMC_Reserved3_SHIFT                  (3U)
/*! Reserved3 - Reserved
 *  0b0..Not supported
 *  0b1..Not supported
 */
#define WUU_PMC_Reserved3(x)                     (((uint32_t)(((uint32_t)(x)) << WUU_PMC_Reserved3_SHIFT)) & WUU_PMC_Reserved3_MASK)

#define WUU_PMC_Reserved4_MASK                   (0x10U)
#define WUU_PMC_Reserved4_SHIFT                  (4U)
/*! Reserved4 - Reserved
 *  0b0..Not supported
 *  0b1..Not supported
 */
#define WUU_PMC_Reserved4(x)                     (((uint32_t)(((uint32_t)(x)) << WUU_PMC_Reserved4_SHIFT)) & WUU_PMC_Reserved4_MASK)

#define WUU_PMC_Reserved5_MASK                   (0x20U)
#define WUU_PMC_Reserved5_SHIFT                  (5U)
/*! Reserved5 - Reserved
 *  0b0..Not supported
 *  0b1..Not supported
 */
#define WUU_PMC_Reserved5(x)                     (((uint32_t)(((uint32_t)(x)) << WUU_PMC_Reserved5_SHIFT)) & WUU_PMC_Reserved5_MASK)

#define WUU_PMC_WUPMC6_MASK                      (0x40U)
#define WUU_PMC_WUPMC6_SHIFT                     (6U)
/*! WUPMC6 - Wake-up Pin Mode Configuration for WUU_Pn
 *  0b0..Active only during a low-leakage mode. You can modify the corresponding fields within Pin Enable (PEn) or
 *       Pin DMA/Trigger Configuration (PDCn).
 *  0b1..Active during all power modes. Do not modify the corresponding fields within Pin Enable (PEn) or Pin DMA/Trigger Configuration (PDCn).
 */
#define WUU_PMC_WUPMC6(x)                        (((uint32_t)(((uint32_t)(x)) << WUU_PMC_WUPMC6_SHIFT)) & WUU_PMC_WUPMC6_MASK)

#define WUU_PMC_WUPMC7_MASK                      (0x80U)
#define WUU_PMC_WUPMC7_SHIFT                     (7U)
/*! WUPMC7 - Wake-up Pin Mode Configuration for WUU_Pn
 *  0b0..Active only during a low-leakage mode. You can modify the corresponding fields within Pin Enable (PEn) or
 *       Pin DMA/Trigger Configuration (PDCn).
 *  0b1..Active during all power modes. Do not modify the corresponding fields within Pin Enable (PEn) or Pin DMA/Trigger Configuration (PDCn).
 */
#define WUU_PMC_WUPMC7(x)                        (((uint32_t)(((uint32_t)(x)) << WUU_PMC_WUPMC7_SHIFT)) & WUU_PMC_WUPMC7_MASK)

#define WUU_PMC_WUPMC8_MASK                      (0x100U)
#define WUU_PMC_WUPMC8_SHIFT                     (8U)
/*! WUPMC8 - Wake-up Pin Mode Configuration for WUU_Pn
 *  0b0..Active only during a low-leakage mode. You can modify the corresponding fields within Pin Enable (PEn) or
 *       Pin DMA/Trigger Configuration (PDCn).
 *  0b1..Active during all power modes. Do not modify the corresponding fields within Pin Enable (PEn) or Pin DMA/Trigger Configuration (PDCn).
 */
#define WUU_PMC_WUPMC8(x)                        (((uint32_t)(((uint32_t)(x)) << WUU_PMC_WUPMC8_SHIFT)) & WUU_PMC_WUPMC8_MASK)

#define WUU_PMC_WUPMC9_MASK                      (0x200U)
#define WUU_PMC_WUPMC9_SHIFT                     (9U)
/*! WUPMC9 - Wake-up Pin Mode Configuration for WUU_Pn
 *  0b0..Active only during a low-leakage mode. You can modify the corresponding fields within Pin Enable (PEn) or
 *       Pin DMA/Trigger Configuration (PDCn).
 *  0b1..Active during all power modes. Do not modify the corresponding fields within Pin Enable (PEn) or Pin DMA/Trigger Configuration (PDCn).
 */
#define WUU_PMC_WUPMC9(x)                        (((uint32_t)(((uint32_t)(x)) << WUU_PMC_WUPMC9_SHIFT)) & WUU_PMC_WUPMC9_MASK)

#define WUU_PMC_WUPMC10_MASK                     (0x400U)
#define WUU_PMC_WUPMC10_SHIFT                    (10U)
/*! WUPMC10 - Wake-up Pin Mode Configuration for WUU_Pn
 *  0b0..Active only during a low-leakage mode. You can modify the corresponding fields within Pin Enable (PEn) or
 *       Pin DMA/Trigger Configuration (PDCn).
 *  0b1..Active during all power modes. Do not modify the corresponding fields within Pin Enable (PEn) or Pin DMA/Trigger Configuration (PDCn).
 */
#define WUU_PMC_WUPMC10(x)                       (((uint32_t)(((uint32_t)(x)) << WUU_PMC_WUPMC10_SHIFT)) & WUU_PMC_WUPMC10_MASK)

#define WUU_PMC_WUPMC11_MASK                     (0x800U)
#define WUU_PMC_WUPMC11_SHIFT                    (11U)
/*! WUPMC11 - Wake-up Pin Mode Configuration for WUU_Pn
 *  0b0..Active only during a low-leakage mode. You can modify the corresponding fields within Pin Enable (PEn) or
 *       Pin DMA/Trigger Configuration (PDCn).
 *  0b1..Active during all power modes. Do not modify the corresponding fields within Pin Enable (PEn) or Pin DMA/Trigger Configuration (PDCn).
 */
#define WUU_PMC_WUPMC11(x)                       (((uint32_t)(((uint32_t)(x)) << WUU_PMC_WUPMC11_SHIFT)) & WUU_PMC_WUPMC11_MASK)

#define WUU_PMC_WUPMC12_MASK                     (0x1000U)
#define WUU_PMC_WUPMC12_SHIFT                    (12U)
/*! WUPMC12 - Wake-up Pin Mode Configuration for WUU_Pn
 *  0b0..Active only during a low-leakage mode. You can modify the corresponding fields within Pin Enable (PEn) or
 *       Pin DMA/Trigger Configuration (PDCn).
 *  0b1..Active during all power modes. Do not modify the corresponding fields within Pin Enable (PEn) or Pin DMA/Trigger Configuration (PDCn).
 */
#define WUU_PMC_WUPMC12(x)                       (((uint32_t)(((uint32_t)(x)) << WUU_PMC_WUPMC12_SHIFT)) & WUU_PMC_WUPMC12_MASK)

#define WUU_PMC_Reserved13_MASK                  (0x2000U)
#define WUU_PMC_Reserved13_SHIFT                 (13U)
/*! Reserved13 - Reserved
 *  0b0..Not supported
 *  0b1..Not supported
 */
#define WUU_PMC_Reserved13(x)                    (((uint32_t)(((uint32_t)(x)) << WUU_PMC_Reserved13_SHIFT)) & WUU_PMC_Reserved13_MASK)

#define WUU_PMC_Reserved14_MASK                  (0x4000U)
#define WUU_PMC_Reserved14_SHIFT                 (14U)
/*! Reserved14 - Reserved
 *  0b0..Not supported
 *  0b1..Not supported
 */
#define WUU_PMC_Reserved14(x)                    (((uint32_t)(((uint32_t)(x)) << WUU_PMC_Reserved14_SHIFT)) & WUU_PMC_Reserved14_MASK)

#define WUU_PMC_Reserved15_MASK                  (0x8000U)
#define WUU_PMC_Reserved15_SHIFT                 (15U)
/*! Reserved15 - Reserved
 *  0b0..Not supported
 *  0b1..Not supported
 */
#define WUU_PMC_Reserved15(x)                    (((uint32_t)(((uint32_t)(x)) << WUU_PMC_Reserved15_SHIFT)) & WUU_PMC_Reserved15_MASK)

#define WUU_PMC_Reserved16_MASK                  (0x10000U)
#define WUU_PMC_Reserved16_SHIFT                 (16U)
/*! Reserved16 - Reserved
 *  0b0..Not supported
 *  0b1..Not supported
 */
#define WUU_PMC_Reserved16(x)                    (((uint32_t)(((uint32_t)(x)) << WUU_PMC_Reserved16_SHIFT)) & WUU_PMC_Reserved16_MASK)

#define WUU_PMC_Reserved17_MASK                  (0x20000U)
#define WUU_PMC_Reserved17_SHIFT                 (17U)
/*! Reserved17 - Reserved
 *  0b0..Not supported
 *  0b1..Not supported
 */
#define WUU_PMC_Reserved17(x)                    (((uint32_t)(((uint32_t)(x)) << WUU_PMC_Reserved17_SHIFT)) & WUU_PMC_Reserved17_MASK)

#define WUU_PMC_WUPMC18_MASK                     (0x40000U)
#define WUU_PMC_WUPMC18_SHIFT                    (18U)
/*! WUPMC18 - Wake-up Pin Mode Configuration for WUU_Pn
 *  0b0..Active only during a low-leakage mode. You can modify the corresponding fields within Pin Enable (PEn) or
 *       Pin DMA/Trigger Configuration (PDCn).
 *  0b1..Active during all power modes. Do not modify the corresponding fields within Pin Enable (PEn) or Pin DMA/Trigger Configuration (PDCn).
 */
#define WUU_PMC_WUPMC18(x)                       (((uint32_t)(((uint32_t)(x)) << WUU_PMC_WUPMC18_SHIFT)) & WUU_PMC_WUPMC18_MASK)

#define WUU_PMC_WUPMC19_MASK                     (0x80000U)
#define WUU_PMC_WUPMC19_SHIFT                    (19U)
/*! WUPMC19 - Wake-up Pin Mode Configuration for WUU_Pn
 *  0b0..Active only during a low-leakage mode. You can modify the corresponding fields within Pin Enable (PEn) or
 *       Pin DMA/Trigger Configuration (PDCn).
 *  0b1..Active during all power modes. Do not modify the corresponding fields within Pin Enable (PEn) or Pin DMA/Trigger Configuration (PDCn).
 */
#define WUU_PMC_WUPMC19(x)                       (((uint32_t)(((uint32_t)(x)) << WUU_PMC_WUPMC19_SHIFT)) & WUU_PMC_WUPMC19_MASK)

#define WUU_PMC_WUPMC20_MASK                     (0x100000U)
#define WUU_PMC_WUPMC20_SHIFT                    (20U)
/*! WUPMC20 - Wake-up Pin Mode Configuration for WUU_Pn
 *  0b0..Active only during a low-leakage mode. You can modify the corresponding fields within Pin Enable (PEn) or
 *       Pin DMA/Trigger Configuration (PDCn).
 *  0b1..Active during all power modes. Do not modify the corresponding fields within Pin Enable (PEn) or Pin DMA/Trigger Configuration (PDCn).
 */
#define WUU_PMC_WUPMC20(x)                       (((uint32_t)(((uint32_t)(x)) << WUU_PMC_WUPMC20_SHIFT)) & WUU_PMC_WUPMC20_MASK)

#define WUU_PMC_Reserved21_MASK                  (0x200000U)
#define WUU_PMC_Reserved21_SHIFT                 (21U)
/*! Reserved21 - Reserved
 *  0b0..Not supported
 *  0b1..Not supported
 */
#define WUU_PMC_Reserved21(x)                    (((uint32_t)(((uint32_t)(x)) << WUU_PMC_Reserved21_SHIFT)) & WUU_PMC_Reserved21_MASK)

#define WUU_PMC_WUPMC22_MASK                     (0x400000U)
#define WUU_PMC_WUPMC22_SHIFT                    (22U)
/*! WUPMC22 - Wake-up Pin Mode Configuration for WUU_Pn
 *  0b0..Active only during a low-leakage mode. You can modify the corresponding fields within Pin Enable (PEn) or
 *       Pin DMA/Trigger Configuration (PDCn).
 *  0b1..Active during all power modes. Do not modify the corresponding fields within Pin Enable (PEn) or Pin DMA/Trigger Configuration (PDCn).
 */
#define WUU_PMC_WUPMC22(x)                       (((uint32_t)(((uint32_t)(x)) << WUU_PMC_WUPMC22_SHIFT)) & WUU_PMC_WUPMC22_MASK)

#define WUU_PMC_WUPMC23_MASK                     (0x800000U)
#define WUU_PMC_WUPMC23_SHIFT                    (23U)
/*! WUPMC23 - Wake-up Pin Mode Configuration for WUU_Pn
 *  0b0..Active only during a low-leakage mode. You can modify the corresponding fields within Pin Enable (PEn) or
 *       Pin DMA/Trigger Configuration (PDCn).
 *  0b1..Active during all power modes. Do not modify the corresponding fields within Pin Enable (PEn) or Pin DMA/Trigger Configuration (PDCn).
 */
#define WUU_PMC_WUPMC23(x)                       (((uint32_t)(((uint32_t)(x)) << WUU_PMC_WUPMC23_SHIFT)) & WUU_PMC_WUPMC23_MASK)

#define WUU_PMC_WUPMC24_MASK                     (0x1000000U)
#define WUU_PMC_WUPMC24_SHIFT                    (24U)
/*! WUPMC24 - Wake-up Pin Mode Configuration for WUU_Pn
 *  0b0..Active only during a low-leakage mode. You can modify the corresponding fields within Pin Enable (PEn) or
 *       Pin DMA/Trigger Configuration (PDCn).
 *  0b1..Active during all power modes. Do not modify the corresponding fields within Pin Enable (PEn) or Pin DMA/Trigger Configuration (PDCn).
 */
#define WUU_PMC_WUPMC24(x)                       (((uint32_t)(((uint32_t)(x)) << WUU_PMC_WUPMC24_SHIFT)) & WUU_PMC_WUPMC24_MASK)

#define WUU_PMC_WUPMC25_MASK                     (0x2000000U)
#define WUU_PMC_WUPMC25_SHIFT                    (25U)
/*! WUPMC25 - Wake-up Pin Mode Configuration for WUU_Pn
 *  0b0..Active only during a low-leakage mode. You can modify the corresponding fields within Pin Enable (PEn) or
 *       Pin DMA/Trigger Configuration (PDCn).
 *  0b1..Active during all power modes. Do not modify the corresponding fields within Pin Enable (PEn) or Pin DMA/Trigger Configuration (PDCn).
 */
#define WUU_PMC_WUPMC25(x)                       (((uint32_t)(((uint32_t)(x)) << WUU_PMC_WUPMC25_SHIFT)) & WUU_PMC_WUPMC25_MASK)

#define WUU_PMC_WUPMC26_MASK                     (0x4000000U)
#define WUU_PMC_WUPMC26_SHIFT                    (26U)
/*! WUPMC26 - Wake-up Pin Mode Configuration for WUU_Pn
 *  0b0..Active only during a low-leakage mode. You can modify the corresponding fields within Pin Enable (PEn) or
 *       Pin DMA/Trigger Configuration (PDCn).
 *  0b1..Active during all power modes. Do not modify the corresponding fields within Pin Enable (PEn) or Pin DMA/Trigger Configuration (PDCn).
 */
#define WUU_PMC_WUPMC26(x)                       (((uint32_t)(((uint32_t)(x)) << WUU_PMC_WUPMC26_SHIFT)) & WUU_PMC_WUPMC26_MASK)

#define WUU_PMC_WUPMC27_MASK                     (0x8000000U)
#define WUU_PMC_WUPMC27_SHIFT                    (27U)
/*! WUPMC27 - Wake-up Pin Mode Configuration for WUU_Pn
 *  0b0..Active only during a low-leakage mode. You can modify the corresponding fields within Pin Enable (PEn) or
 *       Pin DMA/Trigger Configuration (PDCn).
 *  0b1..Active during all power modes. Do not modify the corresponding fields within Pin Enable (PEn) or Pin DMA/Trigger Configuration (PDCn).
 */
#define WUU_PMC_WUPMC27(x)                       (((uint32_t)(((uint32_t)(x)) << WUU_PMC_WUPMC27_SHIFT)) & WUU_PMC_WUPMC27_MASK)

#define WUU_PMC_WUPMC28_MASK                     (0x10000000U)
#define WUU_PMC_WUPMC28_SHIFT                    (28U)
/*! WUPMC28 - Wake-up Pin Mode Configuration for WUU_Pn
 *  0b0..Active only during a low-leakage mode. You can modify the corresponding fields within Pin Enable (PEn) or
 *       Pin DMA/Trigger Configuration (PDCn).
 *  0b1..Active during all power modes. Do not modify the corresponding fields within Pin Enable (PEn) or Pin DMA/Trigger Configuration (PDCn).
 */
#define WUU_PMC_WUPMC28(x)                       (((uint32_t)(((uint32_t)(x)) << WUU_PMC_WUPMC28_SHIFT)) & WUU_PMC_WUPMC28_MASK)

#define WUU_PMC_WUPMC29_MASK                     (0x20000000U)
#define WUU_PMC_WUPMC29_SHIFT                    (29U)
/*! WUPMC29 - Wake-up Pin Mode Configuration for WUU_Pn
 *  0b0..Active only during a low-leakage mode. You can modify the corresponding fields within Pin Enable (PEn) or
 *       Pin DMA/Trigger Configuration (PDCn).
 *  0b1..Active during all power modes. Do not modify the corresponding fields within Pin Enable (PEn) or Pin DMA/Trigger Configuration (PDCn).
 */
#define WUU_PMC_WUPMC29(x)                       (((uint32_t)(((uint32_t)(x)) << WUU_PMC_WUPMC29_SHIFT)) & WUU_PMC_WUPMC29_MASK)

#define WUU_PMC_Reserved30_MASK                  (0x40000000U)
#define WUU_PMC_Reserved30_SHIFT                 (30U)
/*! Reserved30 - Reserved
 *  0b0..Not supported
 *  0b1..Not supported
 */
#define WUU_PMC_Reserved30(x)                    (((uint32_t)(((uint32_t)(x)) << WUU_PMC_Reserved30_SHIFT)) & WUU_PMC_Reserved30_MASK)

#define WUU_PMC_WUPMC31_MASK                     (0x80000000U)
#define WUU_PMC_WUPMC31_SHIFT                    (31U)
/*! WUPMC31 - Wake-up Pin Mode Configuration for WUU_Pn
 *  0b0..Active only during a low-leakage mode. You can modify the corresponding fields within Pin Enable (PEn) or
 *       Pin DMA/Trigger Configuration (PDCn).
 *  0b1..Active during all power modes. Do not modify the corresponding fields within Pin Enable (PEn) or Pin DMA/Trigger Configuration (PDCn).
 */
#define WUU_PMC_WUPMC31(x)                       (((uint32_t)(((uint32_t)(x)) << WUU_PMC_WUPMC31_SHIFT)) & WUU_PMC_WUPMC31_MASK)
/*! @} */

/*! @name FMC - Pin Filter Mode Configuration */
/*! @{ */

#define WUU_FMC_FILTM1_MASK                      (0x1U)
#define WUU_FMC_FILTM1_SHIFT                     (0U)
/*! FILTM1 - Filter Mode for FILTn
 *  0b0..Active only during Power Down/Deep Power Down mode
 *  0b1..Active during all power modes
 */
#define WUU_FMC_FILTM1(x)                        (((uint32_t)(((uint32_t)(x)) << WUU_FMC_FILTM1_SHIFT)) & WUU_FMC_FILTM1_MASK)

#define WUU_FMC_FILTM2_MASK                      (0x2U)
#define WUU_FMC_FILTM2_SHIFT                     (1U)
/*! FILTM2 - Filter Mode for FILTn
 *  0b0..Active only during Power Down/Deep Power Down mode
 *  0b1..Active during all power modes
 */
#define WUU_FMC_FILTM2(x)                        (((uint32_t)(((uint32_t)(x)) << WUU_FMC_FILTM2_SHIFT)) & WUU_FMC_FILTM2_MASK)
/*! @} */


/*!
 * @}
 */ /* end of group WUU_Register_Masks */


/*!
 * @}
 */ /* end of group WUU_Peripheral_Access_Layer */


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


#endif  /* PERI_WUU_H_ */

