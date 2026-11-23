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
**         CMSIS Peripheral Access Layer for PORT
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
 * @file PERI_PORT.h
 * @version 2.0
 * @date 2024-10-29
 * @brief CMSIS Peripheral Access Layer for PORT
 *
 * CMSIS Peripheral Access Layer for PORT
 */

#if !defined(PERI_PORT_H_)
#define PERI_PORT_H_                             /**< Symbol preventing repeated inclusion */

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
   -- PORT Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PORT_Peripheral_Access_Layer PORT Peripheral Access Layer
 * @{
 */

/** PORT - Size of Registers Arrays */
#define PORT_PCR_COUNT                            32u

/** PORT - Register Layout Typedef */
typedef struct {
  __I  uint32_t VERID;                             /**< Version ID, offset: 0x0 */
       uint8_t RESERVED_0[12];
  __IO uint32_t GPCLR;                             /**< Global Pin Control Low, offset: 0x10 */
  __IO uint32_t GPCHR;                             /**< Global Pin Control High, offset: 0x14 */
       uint8_t RESERVED_1[8];
  __IO uint32_t CONFIG;                            /**< Configuration, offset: 0x20 */
       uint8_t RESERVED_2[60];
  __IO uint32_t CALIB0;                            /**< Calibration 0, offset: 0x60, available only on: PORT1, PORT3 (missing on PORT0, PORT2) */
  __IO uint32_t CALIB1;                            /**< Calibration 1, offset: 0x64, available only on: PORT1, PORT3 (missing on PORT0, PORT2) */
       uint8_t RESERVED_3[24];
  __IO uint32_t PCR[PORT_PCR_COUNT];               /**< Pin Control 0..Pin Control 31, array offset: 0x80, array step: 0x4, irregular array, not all indices are valid */
} PORT_Type;

/* ----------------------------------------------------------------------------
   -- PORT Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PORT_Register_Masks PORT Register Masks
 * @{
 */

/*! @name VERID - Version ID */
/*! @{ */

#define PORT_VERID_FEATURE_MASK                  (0xFFFFU)
#define PORT_VERID_FEATURE_SHIFT                 (0U)
/*! FEATURE - Feature Specification Number
 *  0b0000000000000000..Basic implementation
 */
#define PORT_VERID_FEATURE(x)                    (((uint32_t)(((uint32_t)(x)) << PORT_VERID_FEATURE_SHIFT)) & PORT_VERID_FEATURE_MASK)

#define PORT_VERID_MINOR_MASK                    (0xFF0000U)
#define PORT_VERID_MINOR_SHIFT                   (16U)
/*! MINOR - Minor Version Number */
#define PORT_VERID_MINOR(x)                      (((uint32_t)(((uint32_t)(x)) << PORT_VERID_MINOR_SHIFT)) & PORT_VERID_MINOR_MASK)

#define PORT_VERID_MAJOR_MASK                    (0xFF000000U)
#define PORT_VERID_MAJOR_SHIFT                   (24U)
/*! MAJOR - Major Version Number */
#define PORT_VERID_MAJOR(x)                      (((uint32_t)(((uint32_t)(x)) << PORT_VERID_MAJOR_SHIFT)) & PORT_VERID_MAJOR_MASK)
/*! @} */

/*! @name GPCLR - Global Pin Control Low */
/*! @{ */

#define PORT_GPCLR_GPWD_MASK                     (0xFFFFU)
#define PORT_GPCLR_GPWD_SHIFT                    (0U)
/*! GPWD - Global Pin Write Data */
#define PORT_GPCLR_GPWD(x)                       (((uint32_t)(((uint32_t)(x)) << PORT_GPCLR_GPWD_SHIFT)) & PORT_GPCLR_GPWD_MASK)

#define PORT_GPCLR_GPWE0_MASK                    (0x10000U)
#define PORT_GPCLR_GPWE0_SHIFT                   (16U)
/*! GPWE0 - Global Pin Write Enable
 *  0b0..Not updated
 *  0b1..Updated
 */
#define PORT_GPCLR_GPWE0(x)                      (((uint32_t)(((uint32_t)(x)) << PORT_GPCLR_GPWE0_SHIFT)) & PORT_GPCLR_GPWE0_MASK)

#define PORT_GPCLR_GPWE1_MASK                    (0x20000U)
#define PORT_GPCLR_GPWE1_SHIFT                   (17U)
/*! GPWE1 - Global Pin Write Enable
 *  0b0..Not updated
 *  0b1..Updated
 */
#define PORT_GPCLR_GPWE1(x)                      (((uint32_t)(((uint32_t)(x)) << PORT_GPCLR_GPWE1_SHIFT)) & PORT_GPCLR_GPWE1_MASK)

#define PORT_GPCLR_GPWE2_MASK                    (0x40000U)
#define PORT_GPCLR_GPWE2_SHIFT                   (18U)
/*! GPWE2 - Global Pin Write Enable
 *  0b0..Not updated
 *  0b1..Updated
 */
#define PORT_GPCLR_GPWE2(x)                      (((uint32_t)(((uint32_t)(x)) << PORT_GPCLR_GPWE2_SHIFT)) & PORT_GPCLR_GPWE2_MASK)

#define PORT_GPCLR_GPWE3_MASK                    (0x80000U)
#define PORT_GPCLR_GPWE3_SHIFT                   (19U)
/*! GPWE3 - Global Pin Write Enable
 *  0b0..Not updated
 *  0b1..Updated
 */
#define PORT_GPCLR_GPWE3(x)                      (((uint32_t)(((uint32_t)(x)) << PORT_GPCLR_GPWE3_SHIFT)) & PORT_GPCLR_GPWE3_MASK)

#define PORT_GPCLR_GPWE4_MASK                    (0x100000U)
#define PORT_GPCLR_GPWE4_SHIFT                   (20U)
/*! GPWE4 - Global Pin Write Enable
 *  0b0..Not updated
 *  0b1..Updated
 */
#define PORT_GPCLR_GPWE4(x)                      (((uint32_t)(((uint32_t)(x)) << PORT_GPCLR_GPWE4_SHIFT)) & PORT_GPCLR_GPWE4_MASK)

#define PORT_GPCLR_GPWE5_MASK                    (0x200000U)
#define PORT_GPCLR_GPWE5_SHIFT                   (21U)
/*! GPWE5 - Global Pin Write Enable
 *  0b0..Not updated
 *  0b1..Updated
 */
#define PORT_GPCLR_GPWE5(x)                      (((uint32_t)(((uint32_t)(x)) << PORT_GPCLR_GPWE5_SHIFT)) & PORT_GPCLR_GPWE5_MASK)

#define PORT_GPCLR_GPWE6_MASK                    (0x400000U)
#define PORT_GPCLR_GPWE6_SHIFT                   (22U)
/*! GPWE6 - Global Pin Write Enable
 *  0b0..Not updated
 *  0b1..Updated
 */
#define PORT_GPCLR_GPWE6(x)                      (((uint32_t)(((uint32_t)(x)) << PORT_GPCLR_GPWE6_SHIFT)) & PORT_GPCLR_GPWE6_MASK)

#define PORT_GPCLR_GPWE7_MASK                    (0x800000U)
#define PORT_GPCLR_GPWE7_SHIFT                   (23U)
/*! GPWE7 - Global Pin Write Enable
 *  0b0..Not updated
 *  0b1..Updated
 */
#define PORT_GPCLR_GPWE7(x)                      (((uint32_t)(((uint32_t)(x)) << PORT_GPCLR_GPWE7_SHIFT)) & PORT_GPCLR_GPWE7_MASK)

#define PORT_GPCLR_GPWE8_MASK                    (0x1000000U)
#define PORT_GPCLR_GPWE8_SHIFT                   (24U)
/*! GPWE8 - Global Pin Write Enable
 *  0b0..Not updated
 *  0b1..Updated
 */
#define PORT_GPCLR_GPWE8(x)                      (((uint32_t)(((uint32_t)(x)) << PORT_GPCLR_GPWE8_SHIFT)) & PORT_GPCLR_GPWE8_MASK)

#define PORT_GPCLR_GPWE9_MASK                    (0x2000000U)
#define PORT_GPCLR_GPWE9_SHIFT                   (25U)
/*! GPWE9 - Global Pin Write Enable
 *  0b0..Not updated
 *  0b1..Updated
 */
#define PORT_GPCLR_GPWE9(x)                      (((uint32_t)(((uint32_t)(x)) << PORT_GPCLR_GPWE9_SHIFT)) & PORT_GPCLR_GPWE9_MASK)

#define PORT_GPCLR_GPWE10_MASK                   (0x4000000U)
#define PORT_GPCLR_GPWE10_SHIFT                  (26U)
/*! GPWE10 - Global Pin Write Enable
 *  0b0..Not updated
 *  0b1..Updated
 */
#define PORT_GPCLR_GPWE10(x)                     (((uint32_t)(((uint32_t)(x)) << PORT_GPCLR_GPWE10_SHIFT)) & PORT_GPCLR_GPWE10_MASK)

#define PORT_GPCLR_GPWE11_MASK                   (0x8000000U)
#define PORT_GPCLR_GPWE11_SHIFT                  (27U)
/*! GPWE11 - Global Pin Write Enable
 *  0b0..Not updated
 *  0b1..Updated
 */
#define PORT_GPCLR_GPWE11(x)                     (((uint32_t)(((uint32_t)(x)) << PORT_GPCLR_GPWE11_SHIFT)) & PORT_GPCLR_GPWE11_MASK)

#define PORT_GPCLR_GPWE12_MASK                   (0x10000000U)
#define PORT_GPCLR_GPWE12_SHIFT                  (28U)
/*! GPWE12 - Global Pin Write Enable
 *  0b0..Not updated
 *  0b1..Updated
 */
#define PORT_GPCLR_GPWE12(x)                     (((uint32_t)(((uint32_t)(x)) << PORT_GPCLR_GPWE12_SHIFT)) & PORT_GPCLR_GPWE12_MASK)

#define PORT_GPCLR_GPWE13_MASK                   (0x20000000U)
#define PORT_GPCLR_GPWE13_SHIFT                  (29U)
/*! GPWE13 - Global Pin Write Enable
 *  0b0..Not updated
 *  0b1..Updated
 */
#define PORT_GPCLR_GPWE13(x)                     (((uint32_t)(((uint32_t)(x)) << PORT_GPCLR_GPWE13_SHIFT)) & PORT_GPCLR_GPWE13_MASK)

#define PORT_GPCLR_GPWE14_MASK                   (0x40000000U)
#define PORT_GPCLR_GPWE14_SHIFT                  (30U)
/*! GPWE14 - Global Pin Write Enable
 *  0b0..Not updated
 *  0b1..Updated
 */
#define PORT_GPCLR_GPWE14(x)                     (((uint32_t)(((uint32_t)(x)) << PORT_GPCLR_GPWE14_SHIFT)) & PORT_GPCLR_GPWE14_MASK)

#define PORT_GPCLR_GPWE15_MASK                   (0x80000000U)
#define PORT_GPCLR_GPWE15_SHIFT                  (31U)
/*! GPWE15 - Global Pin Write Enable
 *  0b0..Not updated
 *  0b1..Updated
 */
#define PORT_GPCLR_GPWE15(x)                     (((uint32_t)(((uint32_t)(x)) << PORT_GPCLR_GPWE15_SHIFT)) & PORT_GPCLR_GPWE15_MASK)
/*! @} */

/*! @name GPCHR - Global Pin Control High */
/*! @{ */

#define PORT_GPCHR_GPWD_MASK                     (0xFFFFU)
#define PORT_GPCHR_GPWD_SHIFT                    (0U)
/*! GPWD - Global Pin Write Data */
#define PORT_GPCHR_GPWD(x)                       (((uint32_t)(((uint32_t)(x)) << PORT_GPCHR_GPWD_SHIFT)) & PORT_GPCHR_GPWD_MASK)

#define PORT_GPCHR_GPWE16_MASK                   (0x10000U)
#define PORT_GPCHR_GPWE16_SHIFT                  (16U)
/*! GPWE16 - Global Pin Write Enable
 *  0b0..Not updated
 *  0b1..Updated
 */
#define PORT_GPCHR_GPWE16(x)                     (((uint32_t)(((uint32_t)(x)) << PORT_GPCHR_GPWE16_SHIFT)) & PORT_GPCHR_GPWE16_MASK)

#define PORT_GPCHR_GPWE17_MASK                   (0x20000U)
#define PORT_GPCHR_GPWE17_SHIFT                  (17U)
/*! GPWE17 - Global Pin Write Enable
 *  0b0..Not updated
 *  0b1..Updated
 */
#define PORT_GPCHR_GPWE17(x)                     (((uint32_t)(((uint32_t)(x)) << PORT_GPCHR_GPWE17_SHIFT)) & PORT_GPCHR_GPWE17_MASK)

#define PORT_GPCHR_GPWE18_MASK                   (0x40000U)
#define PORT_GPCHR_GPWE18_SHIFT                  (18U)
/*! GPWE18 - Global Pin Write Enable
 *  0b0..Not updated
 *  0b1..Updated
 */
#define PORT_GPCHR_GPWE18(x)                     (((uint32_t)(((uint32_t)(x)) << PORT_GPCHR_GPWE18_SHIFT)) & PORT_GPCHR_GPWE18_MASK)

#define PORT_GPCHR_GPWE19_MASK                   (0x80000U)
#define PORT_GPCHR_GPWE19_SHIFT                  (19U)
/*! GPWE19 - Global Pin Write Enable
 *  0b0..Not updated
 *  0b1..Updated
 */
#define PORT_GPCHR_GPWE19(x)                     (((uint32_t)(((uint32_t)(x)) << PORT_GPCHR_GPWE19_SHIFT)) & PORT_GPCHR_GPWE19_MASK)

#define PORT_GPCHR_GPWE20_MASK                   (0x100000U)
#define PORT_GPCHR_GPWE20_SHIFT                  (20U)
/*! GPWE20 - Global Pin Write Enable
 *  0b0..Not updated
 *  0b1..Updated
 */
#define PORT_GPCHR_GPWE20(x)                     (((uint32_t)(((uint32_t)(x)) << PORT_GPCHR_GPWE20_SHIFT)) & PORT_GPCHR_GPWE20_MASK)

#define PORT_GPCHR_GPWE21_MASK                   (0x200000U)
#define PORT_GPCHR_GPWE21_SHIFT                  (21U)
/*! GPWE21 - Global Pin Write Enable
 *  0b0..Not updated
 *  0b1..Updated
 */
#define PORT_GPCHR_GPWE21(x)                     (((uint32_t)(((uint32_t)(x)) << PORT_GPCHR_GPWE21_SHIFT)) & PORT_GPCHR_GPWE21_MASK)

#define PORT_GPCHR_GPWE22_MASK                   (0x400000U)
#define PORT_GPCHR_GPWE22_SHIFT                  (22U)
/*! GPWE22 - Global Pin Write Enable
 *  0b0..Not updated
 *  0b1..Updated
 */
#define PORT_GPCHR_GPWE22(x)                     (((uint32_t)(((uint32_t)(x)) << PORT_GPCHR_GPWE22_SHIFT)) & PORT_GPCHR_GPWE22_MASK)

#define PORT_GPCHR_GPWE23_MASK                   (0x800000U)
#define PORT_GPCHR_GPWE23_SHIFT                  (23U)
/*! GPWE23 - Global Pin Write Enable
 *  0b0..Not updated
 *  0b1..Updated
 */
#define PORT_GPCHR_GPWE23(x)                     (((uint32_t)(((uint32_t)(x)) << PORT_GPCHR_GPWE23_SHIFT)) & PORT_GPCHR_GPWE23_MASK)

#define PORT_GPCHR_GPWE24_MASK                   (0x1000000U)
#define PORT_GPCHR_GPWE24_SHIFT                  (24U)
/*! GPWE24 - Global Pin Write Enable
 *  0b0..Not updated
 *  0b1..Updated
 */
#define PORT_GPCHR_GPWE24(x)                     (((uint32_t)(((uint32_t)(x)) << PORT_GPCHR_GPWE24_SHIFT)) & PORT_GPCHR_GPWE24_MASK)

#define PORT_GPCHR_GPWE25_MASK                   (0x2000000U)
#define PORT_GPCHR_GPWE25_SHIFT                  (25U)
/*! GPWE25 - Global Pin Write Enable
 *  0b0..Not updated
 *  0b1..Updated
 */
#define PORT_GPCHR_GPWE25(x)                     (((uint32_t)(((uint32_t)(x)) << PORT_GPCHR_GPWE25_SHIFT)) & PORT_GPCHR_GPWE25_MASK)

#define PORT_GPCHR_GPWE26_MASK                   (0x4000000U)
#define PORT_GPCHR_GPWE26_SHIFT                  (26U)
/*! GPWE26 - Global Pin Write Enable
 *  0b0..Not updated
 *  0b1..Updated
 */
#define PORT_GPCHR_GPWE26(x)                     (((uint32_t)(((uint32_t)(x)) << PORT_GPCHR_GPWE26_SHIFT)) & PORT_GPCHR_GPWE26_MASK)

#define PORT_GPCHR_GPWE27_MASK                   (0x8000000U)
#define PORT_GPCHR_GPWE27_SHIFT                  (27U)
/*! GPWE27 - Global Pin Write Enable
 *  0b0..Not updated
 *  0b1..Updated
 */
#define PORT_GPCHR_GPWE27(x)                     (((uint32_t)(((uint32_t)(x)) << PORT_GPCHR_GPWE27_SHIFT)) & PORT_GPCHR_GPWE27_MASK)

#define PORT_GPCHR_GPWE28_MASK                   (0x10000000U)
#define PORT_GPCHR_GPWE28_SHIFT                  (28U)
/*! GPWE28 - Global Pin Write Enable
 *  0b0..Not updated
 *  0b1..Updated
 */
#define PORT_GPCHR_GPWE28(x)                     (((uint32_t)(((uint32_t)(x)) << PORT_GPCHR_GPWE28_SHIFT)) & PORT_GPCHR_GPWE28_MASK)

#define PORT_GPCHR_GPWE29_MASK                   (0x20000000U)
#define PORT_GPCHR_GPWE29_SHIFT                  (29U)
/*! GPWE29 - Global Pin Write Enable
 *  0b0..Not updated
 *  0b1..Updated
 */
#define PORT_GPCHR_GPWE29(x)                     (((uint32_t)(((uint32_t)(x)) << PORT_GPCHR_GPWE29_SHIFT)) & PORT_GPCHR_GPWE29_MASK)

#define PORT_GPCHR_GPWE30_MASK                   (0x40000000U)
#define PORT_GPCHR_GPWE30_SHIFT                  (30U)
/*! GPWE30 - Global Pin Write Enable
 *  0b0..Not updated
 *  0b1..Updated
 */
#define PORT_GPCHR_GPWE30(x)                     (((uint32_t)(((uint32_t)(x)) << PORT_GPCHR_GPWE30_SHIFT)) & PORT_GPCHR_GPWE30_MASK)

#define PORT_GPCHR_GPWE31_MASK                   (0x80000000U)
#define PORT_GPCHR_GPWE31_SHIFT                  (31U)
/*! GPWE31 - Global Pin Write Enable
 *  0b0..Not updated
 *  0b1..Updated
 */
#define PORT_GPCHR_GPWE31(x)                     (((uint32_t)(((uint32_t)(x)) << PORT_GPCHR_GPWE31_SHIFT)) & PORT_GPCHR_GPWE31_MASK)
/*! @} */

/*! @name CONFIG - Configuration */
/*! @{ */

#define PORT_CONFIG_RANGE_MASK                   (0x1U)
#define PORT_CONFIG_RANGE_SHIFT                  (0U)
/*! RANGE - Port Voltage Range
 *  0b0..1.71 V-3.6 V
 *  0b1..2.70 V-3.6 V
 */
#define PORT_CONFIG_RANGE(x)                     (((uint32_t)(((uint32_t)(x)) << PORT_CONFIG_RANGE_SHIFT)) & PORT_CONFIG_RANGE_MASK)
/*! @} */

/*! @name CALIB0 - Calibration 0 */
/*! @{ */

#define PORT_CALIB0_NCAL_MASK                    (0x3FU)
#define PORT_CALIB0_NCAL_SHIFT                   (0U)
/*! NCAL - Calibration of NMOS Output Driver */
#define PORT_CALIB0_NCAL(x)                      (((uint32_t)(((uint32_t)(x)) << PORT_CALIB0_NCAL_SHIFT)) & PORT_CALIB0_NCAL_MASK)

#define PORT_CALIB0_PCAL_MASK                    (0x3F0000U)
#define PORT_CALIB0_PCAL_SHIFT                   (16U)
/*! PCAL - Calibration of PMOS Output Driver */
#define PORT_CALIB0_PCAL(x)                      (((uint32_t)(((uint32_t)(x)) << PORT_CALIB0_PCAL_SHIFT)) & PORT_CALIB0_PCAL_MASK)
/*! @} */

/*! @name CALIB1 - Calibration 1 */
/*! @{ */

#define PORT_CALIB1_NCAL_MASK                    (0x3FU)
#define PORT_CALIB1_NCAL_SHIFT                   (0U)
/*! NCAL - Calibration of NMOS Output Driver */
#define PORT_CALIB1_NCAL(x)                      (((uint32_t)(((uint32_t)(x)) << PORT_CALIB1_NCAL_SHIFT)) & PORT_CALIB1_NCAL_MASK)

#define PORT_CALIB1_PCAL_MASK                    (0x3F0000U)
#define PORT_CALIB1_PCAL_SHIFT                   (16U)
/*! PCAL - Calibration of PMOS Output Driver */
#define PORT_CALIB1_PCAL(x)                      (((uint32_t)(((uint32_t)(x)) << PORT_CALIB1_PCAL_SHIFT)) & PORT_CALIB1_PCAL_MASK)
/*! @} */

/*! @name PCR - Pin Control 0..Pin Control 31 */
/*! @{ */

#define PORT_PCR_PS_MASK                         (0x1U)
#define PORT_PCR_PS_SHIFT                        (0U)
/*! PS - Pull Select
 *  0b0..Enables internal pulldown resistor
 *  0b1..Enables internal pullup resistor
 */
#define PORT_PCR_PS(x)                           (((uint32_t)(((uint32_t)(x)) << PORT_PCR_PS_SHIFT)) & PORT_PCR_PS_MASK)

#define PORT_PCR_PE_MASK                         (0x2U)
#define PORT_PCR_PE_SHIFT                        (1U)
/*! PE - Pull Enable
 *  0b0..Disables
 *  0b1..Enables
 */
#define PORT_PCR_PE(x)                           (((uint32_t)(((uint32_t)(x)) << PORT_PCR_PE_SHIFT)) & PORT_PCR_PE_MASK)

#define PORT_PCR_PV_MASK                         (0x4U)
#define PORT_PCR_PV_SHIFT                        (2U)
/*! PV - Pull Value
 *  0b0..Low
 *  0b1..High
 */
#define PORT_PCR_PV(x)                           (((uint32_t)(((uint32_t)(x)) << PORT_PCR_PV_SHIFT)) & PORT_PCR_PV_MASK)

#define PORT_PCR_SRE_MASK                        (0x8U)
#define PORT_PCR_SRE_SHIFT                       (3U)
/*! SRE - Slew Rate Enable
 *  0b0..Fast
 *  0b1..Slow
 */
#define PORT_PCR_SRE(x)                          (((uint32_t)(((uint32_t)(x)) << PORT_PCR_SRE_SHIFT)) & PORT_PCR_SRE_MASK)

#define PORT_PCR_PFE_MASK                        (0x10U)
#define PORT_PCR_PFE_SHIFT                       (4U)
/*! PFE - Passive Filter Enable
 *  0b0..Disables
 *  0b1..Enables
 */
#define PORT_PCR_PFE(x)                          (((uint32_t)(((uint32_t)(x)) << PORT_PCR_PFE_SHIFT)) & PORT_PCR_PFE_MASK)

#define PORT_PCR_ODE_MASK                        (0x20U)
#define PORT_PCR_ODE_SHIFT                       (5U)
/*! ODE - Open Drain Enable
 *  0b0..Disables
 *  0b1..Enables
 */
#define PORT_PCR_ODE(x)                          (((uint32_t)(((uint32_t)(x)) << PORT_PCR_ODE_SHIFT)) & PORT_PCR_ODE_MASK)

#define PORT_PCR_DSE_MASK                        (0x40U)
#define PORT_PCR_DSE_SHIFT                       (6U)
/*! DSE - Drive Strength Enable
 *  0b0..Low
 *  0b1..High
 */
#define PORT_PCR_DSE(x)                          (((uint32_t)(((uint32_t)(x)) << PORT_PCR_DSE_SHIFT)) & PORT_PCR_DSE_MASK)

#define PORT_PCR_DSE1_MASK                       (0x80U)
#define PORT_PCR_DSE1_SHIFT                      (7U)
/*! DSE1 - Drive Strength Enable
 *  0b0..Normal
 *  0b1..Double
 */
#define PORT_PCR_DSE1(x)                         (((uint32_t)(((uint32_t)(x)) << PORT_PCR_DSE1_SHIFT)) & PORT_PCR_DSE1_MASK)

#define PORT_PCR_MUX_MASK                        (0xF00U)  /* Merged from fields with different position or width, of widths (2, 3, 4), largest definition used */
#define PORT_PCR_MUX_SHIFT                       (8U)
/*! MUX - Pin Multiplex Control
 *  0b0000..Alternative 0 (GPIO)
 *  0b0001..Alternative 1 (chip-specific)
 *  0b0010..Alternative 2 (chip-specific)
 *  0b0011..Alternative 3 (chip-specific)
 *  0b0100..Alternative 4 (chip-specific)
 *  0b0101..Alternative 5 (chip-specific)
 *  0b0110..Alternative 6 (chip-specific)
 *  0b0111..Alternative 7 (chip-specific)
 *  0b1000..Alternative 8 (chip-specific)
 *  0b1001..Alternative 9 (chip-specific)
 *  0b1010..Alternative 10 (chip-specific)
 *  0b1011..Alternative 11 (chip-specific)
 *  0b1100..Alternative 12 (chip-specific)
 *  0b1101..Alternative 13 (chip-specific)
 */
#define PORT_PCR_MUX(x)                          (((uint32_t)(((uint32_t)(x)) << PORT_PCR_MUX_SHIFT)) & PORT_PCR_MUX_MASK)  /* Merged from fields with different position or width, of widths (2, 3, 4), largest definition used */

#define PORT_PCR_IBE_MASK                        (0x1000U)
#define PORT_PCR_IBE_SHIFT                       (12U)
/*! IBE - Input Buffer Enable
 *  0b0..Disables
 *  0b1..Enables
 */
#define PORT_PCR_IBE(x)                          (((uint32_t)(((uint32_t)(x)) << PORT_PCR_IBE_SHIFT)) & PORT_PCR_IBE_MASK)

#define PORT_PCR_INV_MASK                        (0x2000U)
#define PORT_PCR_INV_SHIFT                       (13U)
/*! INV - Invert Input
 *  0b0..Does not invert
 *  0b1..Inverts
 */
#define PORT_PCR_INV(x)                          (((uint32_t)(((uint32_t)(x)) << PORT_PCR_INV_SHIFT)) & PORT_PCR_INV_MASK)

#define PORT_PCR_LK_MASK                         (0x8000U)
#define PORT_PCR_LK_SHIFT                        (15U)
/*! LK - Lock Register
 *  0b0..Does not lock
 *  0b1..Locks
 */
#define PORT_PCR_LK(x)                           (((uint32_t)(((uint32_t)(x)) << PORT_PCR_LK_SHIFT)) & PORT_PCR_LK_MASK)
/*! @} */


/*!
 * @}
 */ /* end of group PORT_Register_Masks */


/*!
 * @}
 */ /* end of group PORT_Peripheral_Access_Layer */


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


#endif  /* PERI_PORT_H_ */

