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
**         CMSIS Peripheral Access Layer for MRCC
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
 * @file PERI_MRCC.h
 * @version 2.0
 * @date 2024-10-29
 * @brief CMSIS Peripheral Access Layer for MRCC
 *
 * CMSIS Peripheral Access Layer for MRCC
 */

#if !defined(PERI_MRCC_H_)
#define PERI_MRCC_H_                             /**< Symbol preventing repeated inclusion */

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
   -- MRCC Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MRCC_Peripheral_Access_Layer MRCC Peripheral Access Layer
 * @{
 */

/** MRCC - Register Layout Typedef */
typedef struct {
  __IO uint32_t MRCC_GLB_RST0;                     /**< Peripheral Reset Control 0, offset: 0x0 */
  __O  uint32_t MRCC_GLB_RST0_SET;                 /**< Peripheral Reset Control Set 0, offset: 0x4 */
  __O  uint32_t MRCC_GLB_RST0_CLR;                 /**< Peripheral Reset Control Clear 0, offset: 0x8 */
       uint8_t RESERVED_0[4];
  __IO uint32_t MRCC_GLB_RST1;                     /**< Peripheral Reset Control 1, offset: 0x10 */
  __O  uint32_t MRCC_GLB_RST1_SET;                 /**< Peripheral Reset Control Set 1, offset: 0x14 */
  __O  uint32_t MRCC_GLB_RST1_CLR;                 /**< Peripheral Reset Control Clear 1, offset: 0x18 */
       uint8_t RESERVED_1[36];
  __IO uint32_t MRCC_GLB_CC0;                      /**< AHB Clock Control 0, offset: 0x40 */
  __O  uint32_t MRCC_GLB_CC0_SET;                  /**< AHB Clock Control Set 0, offset: 0x44 */
  __O  uint32_t MRCC_GLB_CC0_CLR;                  /**< AHB Clock Control Clear 0, offset: 0x48 */
       uint8_t RESERVED_2[4];
  __IO uint32_t MRCC_GLB_CC1;                      /**< AHB Clock Control 1, offset: 0x50 */
  __O  uint32_t MRCC_GLB_CC1_SET;                  /**< AHB Clock Control Set 1, offset: 0x54 */
  __O  uint32_t MRCC_GLB_CC1_CLR;                  /**< AHB Clock Control Clear 1, offset: 0x58 */
       uint8_t RESERVED_3[36];
  __IO uint32_t MRCC_GLB_ACC0;                     /**< Control Automatic Clock Gating 0, offset: 0x80 */
  __IO uint32_t MRCC_GLB_ACC1;                     /**< Control Automatic Clock Gating 1, offset: 0x84 */
       uint8_t RESERVED_4[24];
  __IO uint32_t MRCC_I3C0_FCLK_CLKSEL;             /**< I3C0_FCLK clock selection control, offset: 0xA0 */
  __IO uint32_t MRCC_I3C0_FCLK_CLKDIV;             /**< I3C0_FCLK clock divider control, offset: 0xA4 */
  __IO uint32_t MRCC_CTIMER0_CLKSEL;               /**< CTIMER0 clock selection control, offset: 0xA8 */
  __IO uint32_t MRCC_CTIMER0_CLKDIV;               /**< CTIMER0 clock divider control, offset: 0xAC */
  __IO uint32_t MRCC_CTIMER1_CLKSEL;               /**< CTIMER1 clock selection control, offset: 0xB0 */
  __IO uint32_t MRCC_CTIMER1_CLKDIV;               /**< CTIMER1 clock divider control, offset: 0xB4 */
  __IO uint32_t MRCC_CTIMER2_CLKSEL;               /**< CTIMER2 clock selection control, offset: 0xB8 */
  __IO uint32_t MRCC_CTIMER2_CLKDIV;               /**< CTIMER2 clock divider control, offset: 0xBC */
       uint8_t RESERVED_5[4];
  __IO uint32_t MRCC_WWDT0_CLKDIV;                 /**< WWDT0 clock divider control, offset: 0xC4 */
  __IO uint32_t MRCC_LPI2C0_CLKSEL;                /**< LPI2C0 clock selection control, offset: 0xC8 */
  __IO uint32_t MRCC_LPI2C0_CLKDIV;                /**< LPI2C0 clock divider control, offset: 0xCC */
  __IO uint32_t MRCC_LPSPI0_CLKSEL;                /**< LPSPI0 clock selection control, offset: 0xD0 */
  __IO uint32_t MRCC_LPSPI0_CLKDIV;                /**< LPSPI0 clock divider control, offset: 0xD4 */
  __IO uint32_t MRCC_LPSPI1_CLKSEL;                /**< LPSPI1 clock selection control, offset: 0xD8 */
  __IO uint32_t MRCC_LPSPI1_CLKDIV;                /**< LPSPI1 clock divider control, offset: 0xDC */
  __IO uint32_t MRCC_LPUART0_CLKSEL;               /**< LPUART0 clock selection control, offset: 0xE0 */
  __IO uint32_t MRCC_LPUART0_CLKDIV;               /**< LPUART0 clock divider control, offset: 0xE4 */
  __IO uint32_t MRCC_LPUART1_CLKSEL;               /**< LPUART1 clock selection control, offset: 0xE8 */
  __IO uint32_t MRCC_LPUART1_CLKDIV;               /**< LPUART1 clock divider control, offset: 0xEC */
  __IO uint32_t MRCC_LPUART2_CLKSEL;               /**< LPUART2 clock selection control, offset: 0xF0 */
  __IO uint32_t MRCC_LPUART2_CLKDIV;               /**< LPUART2 clock divider control, offset: 0xF4 */
  __IO uint32_t MRCC_USB0_CLKSEL;                  /**< USB0 clock selection control, offset: 0xF8 */
       uint8_t RESERVED_6[4];
  __IO uint32_t MRCC_LPTMR0_CLKSEL;                /**< LPTMR0 clock selection control, offset: 0x100 */
  __IO uint32_t MRCC_LPTMR0_CLKDIV;                /**< LPTMR0 clock divider control, offset: 0x104 */
  __IO uint32_t MRCC_OSTIMER0_CLKSEL;              /**< OSTIMER0 clock selection control, offset: 0x108 */
       uint8_t RESERVED_7[4];
  __IO uint32_t MRCC_ADC0_CLKSEL;                  /**< ADC0 clock selection control, offset: 0x110 */
  __IO uint32_t MRCC_ADC0_CLKDIV;                  /**< ADC0 clock divider control, offset: 0x114 */
       uint8_t RESERVED_8[4];
  __IO uint32_t MRCC_CMP0_FUNC_CLKDIV;             /**< CMP0_FUNC clock divider control, offset: 0x11C */
  __IO uint32_t MRCC_CMP0_RR_CLKSEL;               /**< CMP0_RR clock selection control, offset: 0x120 */
  __IO uint32_t MRCC_CMP0_RR_CLKDIV;               /**< CMP0_RR clock divider control, offset: 0x124 */
       uint8_t RESERVED_9[4];
  __IO uint32_t MRCC_CMP1_FUNC_CLKDIV;             /**< CMP1_FUNC clock divider control, offset: 0x12C */
  __IO uint32_t MRCC_CMP1_RR_CLKSEL;               /**< CMP1_RR clock selection control, offset: 0x130 */
  __IO uint32_t MRCC_CMP1_RR_CLKDIV;               /**< CMP1_RR clock divider control, offset: 0x134 */
  __IO uint32_t MRCC_DBG_TRACE_CLKSEL;             /**< DBG_TRACE clock selection control, offset: 0x138 */
  __IO uint32_t MRCC_DBG_TRACE_CLKDIV;             /**< DBG_TRACE clock divider control, offset: 0x13C */
  __IO uint32_t MRCC_CLKOUT_CLKSEL;                /**< CLKOUT clock selection control, offset: 0x140 */
  __IO uint32_t MRCC_CLKOUT_CLKDIV;                /**< CLKOUT clock divider control, offset: 0x144 */
  __IO uint32_t MRCC_SYSTICK_CLKSEL;               /**< SYSTICK clock selection control, offset: 0x148 */
  __IO uint32_t MRCC_SYSTICK_CLKDIV;               /**< SYSTICK clock divider control, offset: 0x14C */
       uint8_t RESERVED_10[4];
  __IO uint32_t MRCC_FRO_HF_DIV_CLKDIV;            /**< FRO_HF_DIV clock divider control, offset: 0x154 */
} MRCC_Type;

/* ----------------------------------------------------------------------------
   -- MRCC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MRCC_Register_Masks MRCC Register Masks
 * @{
 */

/*! @name MRCC_GLB_RST0 - Peripheral Reset Control 0 */
/*! @{ */

#define MRCC_MRCC_GLB_RST0_INPUTMUX0_MASK        (0x1U)
#define MRCC_MRCC_GLB_RST0_INPUTMUX0_SHIFT       (0U)
/*! INPUTMUX0 - Write to INPUTMUX0
 *  0b0..Peripheral is held in reset
 *  0b1..Peripheral is released from reset
 */
#define MRCC_MRCC_GLB_RST0_INPUTMUX0(x)          (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_RST0_INPUTMUX0_SHIFT)) & MRCC_MRCC_GLB_RST0_INPUTMUX0_MASK)

#define MRCC_MRCC_GLB_RST0_I3C0_MASK             (0x2U)
#define MRCC_MRCC_GLB_RST0_I3C0_SHIFT            (1U)
/*! I3C0 - Write to I3C0
 *  0b0..Peripheral is held in reset
 *  0b1..Peripheral is released from reset
 */
#define MRCC_MRCC_GLB_RST0_I3C0(x)               (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_RST0_I3C0_SHIFT)) & MRCC_MRCC_GLB_RST0_I3C0_MASK)

#define MRCC_MRCC_GLB_RST0_CTIMER0_MASK          (0x4U)
#define MRCC_MRCC_GLB_RST0_CTIMER0_SHIFT         (2U)
/*! CTIMER0 - Write to CTIMER0
 *  0b0..Peripheral is held in reset
 *  0b1..Peripheral is released from reset
 */
#define MRCC_MRCC_GLB_RST0_CTIMER0(x)            (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_RST0_CTIMER0_SHIFT)) & MRCC_MRCC_GLB_RST0_CTIMER0_MASK)

#define MRCC_MRCC_GLB_RST0_CTIMER1_MASK          (0x8U)
#define MRCC_MRCC_GLB_RST0_CTIMER1_SHIFT         (3U)
/*! CTIMER1 - Write to CTIMER1
 *  0b0..Peripheral is held in reset
 *  0b1..Peripheral is released from reset
 */
#define MRCC_MRCC_GLB_RST0_CTIMER1(x)            (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_RST0_CTIMER1_SHIFT)) & MRCC_MRCC_GLB_RST0_CTIMER1_MASK)

#define MRCC_MRCC_GLB_RST0_CTIMER2_MASK          (0x10U)
#define MRCC_MRCC_GLB_RST0_CTIMER2_SHIFT         (4U)
/*! CTIMER2 - Write to CTIMER2
 *  0b0..Peripheral is held in reset
 *  0b1..Peripheral is released from reset
 */
#define MRCC_MRCC_GLB_RST0_CTIMER2(x)            (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_RST0_CTIMER2_SHIFT)) & MRCC_MRCC_GLB_RST0_CTIMER2_MASK)

#define MRCC_MRCC_GLB_RST0_FREQME_MASK           (0x20U)
#define MRCC_MRCC_GLB_RST0_FREQME_SHIFT          (5U)
/*! FREQME - Write to FREQME
 *  0b0..Peripheral is held in reset
 *  0b1..Peripheral is released from reset
 */
#define MRCC_MRCC_GLB_RST0_FREQME(x)             (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_RST0_FREQME_SHIFT)) & MRCC_MRCC_GLB_RST0_FREQME_MASK)

#define MRCC_MRCC_GLB_RST0_UTICK0_MASK           (0x40U)
#define MRCC_MRCC_GLB_RST0_UTICK0_SHIFT          (6U)
/*! UTICK0 - Write to UTICK0
 *  0b0..Peripheral is held in reset
 *  0b1..Peripheral is released from reset
 */
#define MRCC_MRCC_GLB_RST0_UTICK0(x)             (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_RST0_UTICK0_SHIFT)) & MRCC_MRCC_GLB_RST0_UTICK0_MASK)

#define MRCC_MRCC_GLB_RST0_DMA_MASK              (0x100U)
#define MRCC_MRCC_GLB_RST0_DMA_SHIFT             (8U)
/*! DMA - Write to DMA
 *  0b0..Peripheral is held in reset
 *  0b1..Peripheral is released from reset
 */
#define MRCC_MRCC_GLB_RST0_DMA(x)                (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_RST0_DMA_SHIFT)) & MRCC_MRCC_GLB_RST0_DMA_MASK)

#define MRCC_MRCC_GLB_RST0_AOI0_MASK             (0x200U)
#define MRCC_MRCC_GLB_RST0_AOI0_SHIFT            (9U)
/*! AOI0 - Write to AOI0
 *  0b0..Peripheral is held in reset
 *  0b1..Peripheral is released from reset
 */
#define MRCC_MRCC_GLB_RST0_AOI0(x)               (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_RST0_AOI0_SHIFT)) & MRCC_MRCC_GLB_RST0_AOI0_MASK)

#define MRCC_MRCC_GLB_RST0_CRC_MASK              (0x400U)
#define MRCC_MRCC_GLB_RST0_CRC_SHIFT             (10U)
/*! CRC - Write to CRC
 *  0b0..Peripheral is held in reset
 *  0b1..Peripheral is released from reset
 */
#define MRCC_MRCC_GLB_RST0_CRC(x)                (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_RST0_CRC_SHIFT)) & MRCC_MRCC_GLB_RST0_CRC_MASK)

#define MRCC_MRCC_GLB_RST0_EIM_MASK              (0x800U)
#define MRCC_MRCC_GLB_RST0_EIM_SHIFT             (11U)
/*! EIM - Write to EIM
 *  0b0..Peripheral is held in reset
 *  0b1..Peripheral is released from reset
 */
#define MRCC_MRCC_GLB_RST0_EIM(x)                (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_RST0_EIM_SHIFT)) & MRCC_MRCC_GLB_RST0_EIM_MASK)

#define MRCC_MRCC_GLB_RST0_ERM_MASK              (0x1000U)
#define MRCC_MRCC_GLB_RST0_ERM_SHIFT             (12U)
/*! ERM - Write to ERM
 *  0b0..Peripheral is held in reset
 *  0b1..Peripheral is released from reset
 */
#define MRCC_MRCC_GLB_RST0_ERM(x)                (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_RST0_ERM_SHIFT)) & MRCC_MRCC_GLB_RST0_ERM_MASK)

#define MRCC_MRCC_GLB_RST0_LPI2C0_MASK           (0x10000U)
#define MRCC_MRCC_GLB_RST0_LPI2C0_SHIFT          (16U)
/*! LPI2C0 - Write to LPI2C0
 *  0b0..Peripheral is held in reset
 *  0b1..Peripheral is released from reset
 */
#define MRCC_MRCC_GLB_RST0_LPI2C0(x)             (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_RST0_LPI2C0_SHIFT)) & MRCC_MRCC_GLB_RST0_LPI2C0_MASK)

#define MRCC_MRCC_GLB_RST0_LPSPI0_MASK           (0x20000U)
#define MRCC_MRCC_GLB_RST0_LPSPI0_SHIFT          (17U)
/*! LPSPI0 - Write to LPSPI0
 *  0b0..Peripheral is held in reset
 *  0b1..Peripheral is released from reset
 */
#define MRCC_MRCC_GLB_RST0_LPSPI0(x)             (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_RST0_LPSPI0_SHIFT)) & MRCC_MRCC_GLB_RST0_LPSPI0_MASK)

#define MRCC_MRCC_GLB_RST0_LPSPI1_MASK           (0x40000U)
#define MRCC_MRCC_GLB_RST0_LPSPI1_SHIFT          (18U)
/*! LPSPI1 - Write to LPSPI1
 *  0b0..Peripheral is held in reset
 *  0b1..Peripheral is released from reset
 */
#define MRCC_MRCC_GLB_RST0_LPSPI1(x)             (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_RST0_LPSPI1_SHIFT)) & MRCC_MRCC_GLB_RST0_LPSPI1_MASK)

#define MRCC_MRCC_GLB_RST0_LPUART0_MASK          (0x80000U)
#define MRCC_MRCC_GLB_RST0_LPUART0_SHIFT         (19U)
/*! LPUART0 - Write to LPUART0
 *  0b0..Peripheral is held in reset
 *  0b1..Peripheral is released from reset
 */
#define MRCC_MRCC_GLB_RST0_LPUART0(x)            (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_RST0_LPUART0_SHIFT)) & MRCC_MRCC_GLB_RST0_LPUART0_MASK)

#define MRCC_MRCC_GLB_RST0_LPUART1_MASK          (0x100000U)
#define MRCC_MRCC_GLB_RST0_LPUART1_SHIFT         (20U)
/*! LPUART1 - Write to LPUART1
 *  0b0..Peripheral is held in reset
 *  0b1..Peripheral is released from reset
 */
#define MRCC_MRCC_GLB_RST0_LPUART1(x)            (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_RST0_LPUART1_SHIFT)) & MRCC_MRCC_GLB_RST0_LPUART1_MASK)

#define MRCC_MRCC_GLB_RST0_LPUART2_MASK          (0x200000U)
#define MRCC_MRCC_GLB_RST0_LPUART2_SHIFT         (21U)
/*! LPUART2 - Write to LPUART2
 *  0b0..Peripheral is held in reset
 *  0b1..Peripheral is released from reset
 */
#define MRCC_MRCC_GLB_RST0_LPUART2(x)            (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_RST0_LPUART2_SHIFT)) & MRCC_MRCC_GLB_RST0_LPUART2_MASK)

#define MRCC_MRCC_GLB_RST0_USB0_MASK             (0x400000U)
#define MRCC_MRCC_GLB_RST0_USB0_SHIFT            (22U)
/*! USB0 - Write to USB0
 *  0b0..Peripheral is held in reset
 *  0b1..Peripheral is released from reset
 */
#define MRCC_MRCC_GLB_RST0_USB0(x)               (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_RST0_USB0_SHIFT)) & MRCC_MRCC_GLB_RST0_USB0_MASK)

#define MRCC_MRCC_GLB_RST0_QDC0_MASK             (0x800000U)
#define MRCC_MRCC_GLB_RST0_QDC0_SHIFT            (23U)
/*! QDC0 - Write to QDC0
 *  0b0..Peripheral is held in reset
 *  0b1..Peripheral is released from reset
 */
#define MRCC_MRCC_GLB_RST0_QDC0(x)               (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_RST0_QDC0_SHIFT)) & MRCC_MRCC_GLB_RST0_QDC0_MASK)

#define MRCC_MRCC_GLB_RST0_FLEXPWM0_MASK         (0x1000000U)
#define MRCC_MRCC_GLB_RST0_FLEXPWM0_SHIFT        (24U)
/*! FLEXPWM0 - Write to FLEXPWM0
 *  0b0..Peripheral is held in reset
 *  0b1..Peripheral is released from reset
 */
#define MRCC_MRCC_GLB_RST0_FLEXPWM0(x)           (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_RST0_FLEXPWM0_SHIFT)) & MRCC_MRCC_GLB_RST0_FLEXPWM0_MASK)

#define MRCC_MRCC_GLB_RST0_OSTIMER0_MASK         (0x2000000U)
#define MRCC_MRCC_GLB_RST0_OSTIMER0_SHIFT        (25U)
/*! OSTIMER0 - Write to OSTIMER0
 *  0b0..Peripheral is held in reset
 *  0b1..Peripheral is released from reset
 */
#define MRCC_MRCC_GLB_RST0_OSTIMER0(x)           (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_RST0_OSTIMER0_SHIFT)) & MRCC_MRCC_GLB_RST0_OSTIMER0_MASK)

#define MRCC_MRCC_GLB_RST0_ADC0_MASK             (0x4000000U)
#define MRCC_MRCC_GLB_RST0_ADC0_SHIFT            (26U)
/*! ADC0 - Write to ADC0
 *  0b0..Peripheral is held in reset
 *  0b1..Peripheral is released from reset
 */
#define MRCC_MRCC_GLB_RST0_ADC0(x)               (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_RST0_ADC0_SHIFT)) & MRCC_MRCC_GLB_RST0_ADC0_MASK)

#define MRCC_MRCC_GLB_RST0_CMP1_MASK             (0x10000000U)
#define MRCC_MRCC_GLB_RST0_CMP1_SHIFT            (28U)
/*! CMP1 - Write to CMP1
 *  0b0..Peripheral is held in reset
 *  0b1..Peripheral is released from reset
 */
#define MRCC_MRCC_GLB_RST0_CMP1(x)               (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_RST0_CMP1_SHIFT)) & MRCC_MRCC_GLB_RST0_CMP1_MASK)

#define MRCC_MRCC_GLB_RST0_PORT0_MASK            (0x20000000U)
#define MRCC_MRCC_GLB_RST0_PORT0_SHIFT           (29U)
/*! PORT0 - Write to PORT0
 *  0b0..Peripheral is held in reset
 *  0b1..Peripheral is released from reset
 */
#define MRCC_MRCC_GLB_RST0_PORT0(x)              (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_RST0_PORT0_SHIFT)) & MRCC_MRCC_GLB_RST0_PORT0_MASK)

#define MRCC_MRCC_GLB_RST0_PORT1_MASK            (0x40000000U)
#define MRCC_MRCC_GLB_RST0_PORT1_SHIFT           (30U)
/*! PORT1 - Write to PORT1
 *  0b0..Peripheral is held in reset
 *  0b1..Peripheral is released from reset
 */
#define MRCC_MRCC_GLB_RST0_PORT1(x)              (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_RST0_PORT1_SHIFT)) & MRCC_MRCC_GLB_RST0_PORT1_MASK)

#define MRCC_MRCC_GLB_RST0_PORT2_MASK            (0x80000000U)
#define MRCC_MRCC_GLB_RST0_PORT2_SHIFT           (31U)
/*! PORT2 - Write to PORT2
 *  0b0..Peripheral is held in reset
 *  0b1..Peripheral is released from reset
 */
#define MRCC_MRCC_GLB_RST0_PORT2(x)              (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_RST0_PORT2_SHIFT)) & MRCC_MRCC_GLB_RST0_PORT2_MASK)
/*! @} */

/*! @name MRCC_GLB_RST0_SET - Peripheral Reset Control Set 0 */
/*! @{ */

#define MRCC_MRCC_GLB_RST0_SET_DATA_MASK         (0xFFFFFFFFU)
#define MRCC_MRCC_GLB_RST0_SET_DATA_SHIFT        (0U)
/*! DATA - Data array value, refer to corresponding position in MRCC_GLB_RSTn. */
#define MRCC_MRCC_GLB_RST0_SET_DATA(x)           (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_RST0_SET_DATA_SHIFT)) & MRCC_MRCC_GLB_RST0_SET_DATA_MASK)
/*! @} */

/*! @name MRCC_GLB_RST0_CLR - Peripheral Reset Control Clear 0 */
/*! @{ */

#define MRCC_MRCC_GLB_RST0_CLR_DATA_MASK         (0xFFFFFFFFU)
#define MRCC_MRCC_GLB_RST0_CLR_DATA_SHIFT        (0U)
/*! DATA - Data array value, refer to corresponding position in MRCC_GLB_RSTn. */
#define MRCC_MRCC_GLB_RST0_CLR_DATA(x)           (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_RST0_CLR_DATA_SHIFT)) & MRCC_MRCC_GLB_RST0_CLR_DATA_MASK)
/*! @} */

/*! @name MRCC_GLB_RST1 - Peripheral Reset Control 1 */
/*! @{ */

#define MRCC_MRCC_GLB_RST1_PORT3_MASK            (0x1U)
#define MRCC_MRCC_GLB_RST1_PORT3_SHIFT           (0U)
/*! PORT3 - Write to PORT3
 *  0b0..Peripheral is held in reset
 *  0b1..Peripheral is released from reset
 */
#define MRCC_MRCC_GLB_RST1_PORT3(x)              (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_RST1_PORT3_SHIFT)) & MRCC_MRCC_GLB_RST1_PORT3_MASK)

#define MRCC_MRCC_GLB_RST1_GPIO0_MASK            (0x20U)
#define MRCC_MRCC_GLB_RST1_GPIO0_SHIFT           (5U)
/*! GPIO0 - Write to GPIO0
 *  0b0..Peripheral is held in reset
 *  0b1..Peripheral is released from reset
 */
#define MRCC_MRCC_GLB_RST1_GPIO0(x)              (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_RST1_GPIO0_SHIFT)) & MRCC_MRCC_GLB_RST1_GPIO0_MASK)

#define MRCC_MRCC_GLB_RST1_GPIO1_MASK            (0x40U)
#define MRCC_MRCC_GLB_RST1_GPIO1_SHIFT           (6U)
/*! GPIO1 - Write to GPIO1
 *  0b0..Peripheral is held in reset
 *  0b1..Peripheral is released from reset
 */
#define MRCC_MRCC_GLB_RST1_GPIO1(x)              (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_RST1_GPIO1_SHIFT)) & MRCC_MRCC_GLB_RST1_GPIO1_MASK)

#define MRCC_MRCC_GLB_RST1_GPIO2_MASK            (0x80U)
#define MRCC_MRCC_GLB_RST1_GPIO2_SHIFT           (7U)
/*! GPIO2 - Write to GPIO2
 *  0b0..Peripheral is held in reset
 *  0b1..Peripheral is released from reset
 */
#define MRCC_MRCC_GLB_RST1_GPIO2(x)              (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_RST1_GPIO2_SHIFT)) & MRCC_MRCC_GLB_RST1_GPIO2_MASK)

#define MRCC_MRCC_GLB_RST1_GPIO3_MASK            (0x100U)
#define MRCC_MRCC_GLB_RST1_GPIO3_SHIFT           (8U)
/*! GPIO3 - Write to GPIO3
 *  0b0..Peripheral is held in reset
 *  0b1..Peripheral is released from reset
 */
#define MRCC_MRCC_GLB_RST1_GPIO3(x)              (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_RST1_GPIO3_SHIFT)) & MRCC_MRCC_GLB_RST1_GPIO3_MASK)
/*! @} */

/*! @name MRCC_GLB_RST1_SET - Peripheral Reset Control Set 1 */
/*! @{ */

#define MRCC_MRCC_GLB_RST1_SET_DATA_MASK         (0xFFFFFFFFU)
#define MRCC_MRCC_GLB_RST1_SET_DATA_SHIFT        (0U)
/*! DATA - Data array value, refer to corresponding position in MRCC_GLB_RSTn. */
#define MRCC_MRCC_GLB_RST1_SET_DATA(x)           (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_RST1_SET_DATA_SHIFT)) & MRCC_MRCC_GLB_RST1_SET_DATA_MASK)
/*! @} */

/*! @name MRCC_GLB_RST1_CLR - Peripheral Reset Control Clear 1 */
/*! @{ */

#define MRCC_MRCC_GLB_RST1_CLR_DATA_MASK         (0xFFFFFFFFU)
#define MRCC_MRCC_GLB_RST1_CLR_DATA_SHIFT        (0U)
/*! DATA - Data array value, refer to corresponding position in MRCC_GLB_RSTn. */
#define MRCC_MRCC_GLB_RST1_CLR_DATA(x)           (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_RST1_CLR_DATA_SHIFT)) & MRCC_MRCC_GLB_RST1_CLR_DATA_MASK)
/*! @} */

/*! @name MRCC_GLB_CC0 - AHB Clock Control 0 */
/*! @{ */

#define MRCC_MRCC_GLB_CC0_INPUTMUX0_MASK         (0x1U)
#define MRCC_MRCC_GLB_CC0_INPUTMUX0_SHIFT        (0U)
/*! INPUTMUX0 - write to INPUTMUX0
 *  0b0..Peripheral clock is disabled
 *  0b1..Peripheral clock is enabled
 */
#define MRCC_MRCC_GLB_CC0_INPUTMUX0(x)           (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_CC0_INPUTMUX0_SHIFT)) & MRCC_MRCC_GLB_CC0_INPUTMUX0_MASK)

#define MRCC_MRCC_GLB_CC0_I3C0_MASK              (0x2U)
#define MRCC_MRCC_GLB_CC0_I3C0_SHIFT             (1U)
/*! I3C0 - Write to I3C0
 *  0b0..Peripheral clock is disabled
 *  0b1..Peripheral clock is enabled
 */
#define MRCC_MRCC_GLB_CC0_I3C0(x)                (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_CC0_I3C0_SHIFT)) & MRCC_MRCC_GLB_CC0_I3C0_MASK)

#define MRCC_MRCC_GLB_CC0_CTIMER0_MASK           (0x4U)
#define MRCC_MRCC_GLB_CC0_CTIMER0_SHIFT          (2U)
/*! CTIMER0 - Write to CTIMER0
 *  0b0..Peripheral clock is disabled
 *  0b1..Peripheral clock is enabled
 */
#define MRCC_MRCC_GLB_CC0_CTIMER0(x)             (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_CC0_CTIMER0_SHIFT)) & MRCC_MRCC_GLB_CC0_CTIMER0_MASK)

#define MRCC_MRCC_GLB_CC0_CTIMER1_MASK           (0x8U)
#define MRCC_MRCC_GLB_CC0_CTIMER1_SHIFT          (3U)
/*! CTIMER1 - Write to CTIMER1
 *  0b0..Peripheral clock is disabled
 *  0b1..Peripheral clock is enabled
 */
#define MRCC_MRCC_GLB_CC0_CTIMER1(x)             (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_CC0_CTIMER1_SHIFT)) & MRCC_MRCC_GLB_CC0_CTIMER1_MASK)

#define MRCC_MRCC_GLB_CC0_CTIMER2_MASK           (0x10U)
#define MRCC_MRCC_GLB_CC0_CTIMER2_SHIFT          (4U)
/*! CTIMER2 - Write to CTIMER2
 *  0b0..Peripheral clock is disabled
 *  0b1..Peripheral clock is enabled
 */
#define MRCC_MRCC_GLB_CC0_CTIMER2(x)             (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_CC0_CTIMER2_SHIFT)) & MRCC_MRCC_GLB_CC0_CTIMER2_MASK)

#define MRCC_MRCC_GLB_CC0_FREQME_MASK            (0x20U)
#define MRCC_MRCC_GLB_CC0_FREQME_SHIFT           (5U)
/*! FREQME - Write to FREQME
 *  0b0..Peripheral clock is disabled
 *  0b1..Peripheral clock is enabled
 */
#define MRCC_MRCC_GLB_CC0_FREQME(x)              (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_CC0_FREQME_SHIFT)) & MRCC_MRCC_GLB_CC0_FREQME_MASK)

#define MRCC_MRCC_GLB_CC0_UTICK0_MASK            (0x40U)
#define MRCC_MRCC_GLB_CC0_UTICK0_SHIFT           (6U)
/*! UTICK0 - Write to UTICK0
 *  0b0..Peripheral clock is disabled
 *  0b1..Peripheral clock is enabled
 */
#define MRCC_MRCC_GLB_CC0_UTICK0(x)              (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_CC0_UTICK0_SHIFT)) & MRCC_MRCC_GLB_CC0_UTICK0_MASK)

#define MRCC_MRCC_GLB_CC0_WWDT0_MASK             (0x80U)
#define MRCC_MRCC_GLB_CC0_WWDT0_SHIFT            (7U)
/*! WWDT0 - Write to WWDT0
 *  0b0..Peripheral clock is disabled
 *  0b1..Peripheral clock is enabled
 */
#define MRCC_MRCC_GLB_CC0_WWDT0(x)               (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_CC0_WWDT0_SHIFT)) & MRCC_MRCC_GLB_CC0_WWDT0_MASK)

#define MRCC_MRCC_GLB_CC0_DMA_MASK               (0x100U)
#define MRCC_MRCC_GLB_CC0_DMA_SHIFT              (8U)
/*! DMA - Write to DMA
 *  0b0..Peripheral clock is disabled
 *  0b1..Peripheral clock is enabled
 */
#define MRCC_MRCC_GLB_CC0_DMA(x)                 (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_CC0_DMA_SHIFT)) & MRCC_MRCC_GLB_CC0_DMA_MASK)

#define MRCC_MRCC_GLB_CC0_AOI0_MASK              (0x200U)
#define MRCC_MRCC_GLB_CC0_AOI0_SHIFT             (9U)
/*! AOI0 - Write to AOI0
 *  0b0..Peripheral clock is disabled
 *  0b1..Peripheral clock is enabled
 */
#define MRCC_MRCC_GLB_CC0_AOI0(x)                (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_CC0_AOI0_SHIFT)) & MRCC_MRCC_GLB_CC0_AOI0_MASK)

#define MRCC_MRCC_GLB_CC0_CRC_MASK               (0x400U)
#define MRCC_MRCC_GLB_CC0_CRC_SHIFT              (10U)
/*! CRC - Write to CRC
 *  0b0..Peripheral clock is disabled
 *  0b1..Peripheral clock is enabled
 */
#define MRCC_MRCC_GLB_CC0_CRC(x)                 (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_CC0_CRC_SHIFT)) & MRCC_MRCC_GLB_CC0_CRC_MASK)

#define MRCC_MRCC_GLB_CC0_EIM_MASK               (0x800U)
#define MRCC_MRCC_GLB_CC0_EIM_SHIFT              (11U)
/*! EIM - Write to EIM
 *  0b0..Peripheral clock is disabled
 *  0b1..Peripheral clock is enabled
 */
#define MRCC_MRCC_GLB_CC0_EIM(x)                 (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_CC0_EIM_SHIFT)) & MRCC_MRCC_GLB_CC0_EIM_MASK)

#define MRCC_MRCC_GLB_CC0_ERM_MASK               (0x1000U)
#define MRCC_MRCC_GLB_CC0_ERM_SHIFT              (12U)
/*! ERM - Write to ERM
 *  0b0..Peripheral clock is disabled
 *  0b1..Peripheral clock is enabled
 */
#define MRCC_MRCC_GLB_CC0_ERM(x)                 (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_CC0_ERM_SHIFT)) & MRCC_MRCC_GLB_CC0_ERM_MASK)

#define MRCC_MRCC_GLB_CC0_LPI2C0_MASK            (0x10000U)
#define MRCC_MRCC_GLB_CC0_LPI2C0_SHIFT           (16U)
/*! LPI2C0 - Write to LPI2C0
 *  0b0..Peripheral clock is disabled
 *  0b1..Peripheral clock is enabled
 */
#define MRCC_MRCC_GLB_CC0_LPI2C0(x)              (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_CC0_LPI2C0_SHIFT)) & MRCC_MRCC_GLB_CC0_LPI2C0_MASK)

#define MRCC_MRCC_GLB_CC0_LPSPI0_MASK            (0x20000U)
#define MRCC_MRCC_GLB_CC0_LPSPI0_SHIFT           (17U)
/*! LPSPI0 - Write to LPSPI0
 *  0b0..Peripheral clock is disabled
 *  0b1..Peripheral clock is enabled
 */
#define MRCC_MRCC_GLB_CC0_LPSPI0(x)              (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_CC0_LPSPI0_SHIFT)) & MRCC_MRCC_GLB_CC0_LPSPI0_MASK)

#define MRCC_MRCC_GLB_CC0_LPSPI1_MASK            (0x40000U)
#define MRCC_MRCC_GLB_CC0_LPSPI1_SHIFT           (18U)
/*! LPSPI1 - write to LPSPI1
 *  0b0..Peripheral clock is disabled
 *  0b1..Peripheral clock is enabled
 */
#define MRCC_MRCC_GLB_CC0_LPSPI1(x)              (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_CC0_LPSPI1_SHIFT)) & MRCC_MRCC_GLB_CC0_LPSPI1_MASK)

#define MRCC_MRCC_GLB_CC0_LPUART0_MASK           (0x80000U)
#define MRCC_MRCC_GLB_CC0_LPUART0_SHIFT          (19U)
/*! LPUART0 - Write to LPUART0
 *  0b0..Peripheral clock is disabled
 *  0b1..Peripheral clock is enabled
 */
#define MRCC_MRCC_GLB_CC0_LPUART0(x)             (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_CC0_LPUART0_SHIFT)) & MRCC_MRCC_GLB_CC0_LPUART0_MASK)

#define MRCC_MRCC_GLB_CC0_LPUART1_MASK           (0x100000U)
#define MRCC_MRCC_GLB_CC0_LPUART1_SHIFT          (20U)
/*! LPUART1 - Write to LPUART1
 *  0b0..Peripheral clock is disabled
 *  0b1..Peripheral clock is enabled
 */
#define MRCC_MRCC_GLB_CC0_LPUART1(x)             (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_CC0_LPUART1_SHIFT)) & MRCC_MRCC_GLB_CC0_LPUART1_MASK)

#define MRCC_MRCC_GLB_CC0_LPUART2_MASK           (0x200000U)
#define MRCC_MRCC_GLB_CC0_LPUART2_SHIFT          (21U)
/*! LPUART2 - Write to LPUART2
 *  0b0..Peripheral clock is disabled
 *  0b1..Peripheral clock is enabled
 */
#define MRCC_MRCC_GLB_CC0_LPUART2(x)             (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_CC0_LPUART2_SHIFT)) & MRCC_MRCC_GLB_CC0_LPUART2_MASK)

#define MRCC_MRCC_GLB_CC0_USB0_MASK              (0x400000U)
#define MRCC_MRCC_GLB_CC0_USB0_SHIFT             (22U)
/*! USB0 - Write to USB0
 *  0b0..Peripheral clock is disabled
 *  0b1..Peripheral clock is enabled
 */
#define MRCC_MRCC_GLB_CC0_USB0(x)                (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_CC0_USB0_SHIFT)) & MRCC_MRCC_GLB_CC0_USB0_MASK)

#define MRCC_MRCC_GLB_CC0_QDC0_MASK              (0x800000U)
#define MRCC_MRCC_GLB_CC0_QDC0_SHIFT             (23U)
/*! QDC0 - Write to QDC0
 *  0b0..Peripheral clock is disabled
 *  0b1..Peripheral clock is enabled
 */
#define MRCC_MRCC_GLB_CC0_QDC0(x)                (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_CC0_QDC0_SHIFT)) & MRCC_MRCC_GLB_CC0_QDC0_MASK)

#define MRCC_MRCC_GLB_CC0_FLEXPWM0_MASK          (0x1000000U)
#define MRCC_MRCC_GLB_CC0_FLEXPWM0_SHIFT         (24U)
/*! FLEXPWM0 - Write to FLEXPWM0
 *  0b0..Peripheral clock is disabled
 *  0b1..Peripheral clock is enabled
 */
#define MRCC_MRCC_GLB_CC0_FLEXPWM0(x)            (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_CC0_FLEXPWM0_SHIFT)) & MRCC_MRCC_GLB_CC0_FLEXPWM0_MASK)

#define MRCC_MRCC_GLB_CC0_OSTIMER0_MASK          (0x2000000U)
#define MRCC_MRCC_GLB_CC0_OSTIMER0_SHIFT         (25U)
/*! OSTIMER0 - Write to OSTIMER0
 *  0b0..Peripheral clock is disabled
 *  0b1..Peripheral clock is enabled
 */
#define MRCC_MRCC_GLB_CC0_OSTIMER0(x)            (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_CC0_OSTIMER0_SHIFT)) & MRCC_MRCC_GLB_CC0_OSTIMER0_MASK)

#define MRCC_MRCC_GLB_CC0_ADC0_MASK              (0x4000000U)
#define MRCC_MRCC_GLB_CC0_ADC0_SHIFT             (26U)
/*! ADC0 - Write to ADC0
 *  0b0..Peripheral clock is disabled
 *  0b1..Peripheral clock is enabled
 */
#define MRCC_MRCC_GLB_CC0_ADC0(x)                (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_CC0_ADC0_SHIFT)) & MRCC_MRCC_GLB_CC0_ADC0_MASK)

#define MRCC_MRCC_GLB_CC0_CMP0_MASK              (0x8000000U)
#define MRCC_MRCC_GLB_CC0_CMP0_SHIFT             (27U)
/*! CMP0 - Write to CMP0
 *  0b0..Peripheral clock is disabled
 *  0b1..Peripheral clock is enabled
 */
#define MRCC_MRCC_GLB_CC0_CMP0(x)                (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_CC0_CMP0_SHIFT)) & MRCC_MRCC_GLB_CC0_CMP0_MASK)

#define MRCC_MRCC_GLB_CC0_CMP1_MASK              (0x10000000U)
#define MRCC_MRCC_GLB_CC0_CMP1_SHIFT             (28U)
/*! CMP1 - Write to CMP1
 *  0b0..Peripheral clock is disabled
 *  0b1..Peripheral clock is enabled
 */
#define MRCC_MRCC_GLB_CC0_CMP1(x)                (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_CC0_CMP1_SHIFT)) & MRCC_MRCC_GLB_CC0_CMP1_MASK)

#define MRCC_MRCC_GLB_CC0_PORT0_MASK             (0x20000000U)
#define MRCC_MRCC_GLB_CC0_PORT0_SHIFT            (29U)
/*! PORT0 - Write to PORT0
 *  0b0..Peripheral clock is disabled
 *  0b1..Peripheral clock is enabled
 */
#define MRCC_MRCC_GLB_CC0_PORT0(x)               (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_CC0_PORT0_SHIFT)) & MRCC_MRCC_GLB_CC0_PORT0_MASK)

#define MRCC_MRCC_GLB_CC0_PORT1_MASK             (0x40000000U)
#define MRCC_MRCC_GLB_CC0_PORT1_SHIFT            (30U)
/*! PORT1 - Write to PORT1
 *  0b0..Peripheral clock is disabled
 *  0b1..Peripheral clock is enabled
 */
#define MRCC_MRCC_GLB_CC0_PORT1(x)               (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_CC0_PORT1_SHIFT)) & MRCC_MRCC_GLB_CC0_PORT1_MASK)

#define MRCC_MRCC_GLB_CC0_PORT2_MASK             (0x80000000U)
#define MRCC_MRCC_GLB_CC0_PORT2_SHIFT            (31U)
/*! PORT2 - Write to PORT2
 *  0b0..Peripheral clock is disabled
 *  0b1..Peripheral clock is enabled
 */
#define MRCC_MRCC_GLB_CC0_PORT2(x)               (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_CC0_PORT2_SHIFT)) & MRCC_MRCC_GLB_CC0_PORT2_MASK)
/*! @} */

/*! @name MRCC_GLB_CC0_SET - AHB Clock Control Set 0 */
/*! @{ */

#define MRCC_MRCC_GLB_CC0_SET_DATA_MASK          (0xFFFFFFFFU)
#define MRCC_MRCC_GLB_CC0_SET_DATA_SHIFT         (0U)
/*! DATA - Data array value, refer to corresponding position in MRCC_GLB_CCn. */
#define MRCC_MRCC_GLB_CC0_SET_DATA(x)            (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_CC0_SET_DATA_SHIFT)) & MRCC_MRCC_GLB_CC0_SET_DATA_MASK)
/*! @} */

/*! @name MRCC_GLB_CC0_CLR - AHB Clock Control Clear 0 */
/*! @{ */

#define MRCC_MRCC_GLB_CC0_CLR_DATA_MASK          (0xFFFFFFFFU)
#define MRCC_MRCC_GLB_CC0_CLR_DATA_SHIFT         (0U)
/*! DATA - Data array value, refer to corresponding position in MRCC_GLB_CCn. */
#define MRCC_MRCC_GLB_CC0_CLR_DATA(x)            (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_CC0_CLR_DATA_SHIFT)) & MRCC_MRCC_GLB_CC0_CLR_DATA_MASK)
/*! @} */

/*! @name MRCC_GLB_CC1 - AHB Clock Control 1 */
/*! @{ */

#define MRCC_MRCC_GLB_CC1_PORT3_MASK             (0x1U)
#define MRCC_MRCC_GLB_CC1_PORT3_SHIFT            (0U)
/*! PORT3 - Write to PORT3
 *  0b0..Peripheral clock is disabled
 *  0b1..Peripheral clock is enabled
 */
#define MRCC_MRCC_GLB_CC1_PORT3(x)               (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_CC1_PORT3_SHIFT)) & MRCC_MRCC_GLB_CC1_PORT3_MASK)

#define MRCC_MRCC_GLB_CC1_MTR_MASK               (0x4U)
#define MRCC_MRCC_GLB_CC1_MTR_SHIFT              (2U)
/*! MTR - Write to MTR
 *  0b0..Peripheral clock is disabled
 *  0b1..Peripheral clock is enabled
 */
#define MRCC_MRCC_GLB_CC1_MTR(x)                 (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_CC1_MTR_SHIFT)) & MRCC_MRCC_GLB_CC1_MTR_MASK)

#define MRCC_MRCC_GLB_CC1_TCU_MASK               (0x8U)
#define MRCC_MRCC_GLB_CC1_TCU_SHIFT              (3U)
/*! TCU - Write to TCU
 *  0b0..Peripheral clock is disabled
 *  0b1..Peripheral clock is enabled
 */
#define MRCC_MRCC_GLB_CC1_TCU(x)                 (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_CC1_TCU_SHIFT)) & MRCC_MRCC_GLB_CC1_TCU_MASK)

#define MRCC_MRCC_GLB_CC1_EZRAMC_RAMA_MASK       (0x10U)
#define MRCC_MRCC_GLB_CC1_EZRAMC_RAMA_SHIFT      (4U)
/*! EZRAMC_RAMA - Write to EZRAMC_RAMA
 *  0b0..Peripheral clock is disabled
 *  0b1..Peripheral clock is enabled
 */
#define MRCC_MRCC_GLB_CC1_EZRAMC_RAMA(x)         (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_CC1_EZRAMC_RAMA_SHIFT)) & MRCC_MRCC_GLB_CC1_EZRAMC_RAMA_MASK)

#define MRCC_MRCC_GLB_CC1_GPIO0_MASK             (0x20U)
#define MRCC_MRCC_GLB_CC1_GPIO0_SHIFT            (5U)
/*! GPIO0 - Write to GPIO0
 *  0b0..Peripheral clock is disabled
 *  0b1..Peripheral clock is enabled
 */
#define MRCC_MRCC_GLB_CC1_GPIO0(x)               (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_CC1_GPIO0_SHIFT)) & MRCC_MRCC_GLB_CC1_GPIO0_MASK)

#define MRCC_MRCC_GLB_CC1_GPIO1_MASK             (0x40U)
#define MRCC_MRCC_GLB_CC1_GPIO1_SHIFT            (6U)
/*! GPIO1 - Write to GPIO1
 *  0b0..Peripheral clock is disabled
 *  0b1..Peripheral clock is enabled
 */
#define MRCC_MRCC_GLB_CC1_GPIO1(x)               (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_CC1_GPIO1_SHIFT)) & MRCC_MRCC_GLB_CC1_GPIO1_MASK)

#define MRCC_MRCC_GLB_CC1_GPIO2_MASK             (0x80U)
#define MRCC_MRCC_GLB_CC1_GPIO2_SHIFT            (7U)
/*! GPIO2 - Write to GPIO2
 *  0b0..Peripheral clock is disabled
 *  0b1..Peripheral clock is enabled
 */
#define MRCC_MRCC_GLB_CC1_GPIO2(x)               (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_CC1_GPIO2_SHIFT)) & MRCC_MRCC_GLB_CC1_GPIO2_MASK)

#define MRCC_MRCC_GLB_CC1_GPIO3_MASK             (0x100U)
#define MRCC_MRCC_GLB_CC1_GPIO3_SHIFT            (8U)
/*! GPIO3 - Write to GPIO3
 *  0b0..Peripheral clock is disabled
 *  0b1..Peripheral clock is enabled
 */
#define MRCC_MRCC_GLB_CC1_GPIO3(x)               (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_CC1_GPIO3_SHIFT)) & MRCC_MRCC_GLB_CC1_GPIO3_MASK)

#define MRCC_MRCC_GLB_CC1_ROMCP_MASK             (0x200U)
#define MRCC_MRCC_GLB_CC1_ROMCP_SHIFT            (9U)
/*! ROMCP - Write to ROMCP
 *  0b0..Peripheral clock is disabled
 *  0b1..Peripheral clock is enabled
 */
#define MRCC_MRCC_GLB_CC1_ROMCP(x)               (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_CC1_ROMCP_SHIFT)) & MRCC_MRCC_GLB_CC1_ROMCP_MASK)
/*! @} */

/*! @name MRCC_GLB_CC1_SET - AHB Clock Control Set 1 */
/*! @{ */

#define MRCC_MRCC_GLB_CC1_SET_DATA_MASK          (0xFFFFFFFFU)
#define MRCC_MRCC_GLB_CC1_SET_DATA_SHIFT         (0U)
/*! DATA - Data array value, refer to corresponding position in MRCC_GLB_CCn. */
#define MRCC_MRCC_GLB_CC1_SET_DATA(x)            (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_CC1_SET_DATA_SHIFT)) & MRCC_MRCC_GLB_CC1_SET_DATA_MASK)
/*! @} */

/*! @name MRCC_GLB_CC1_CLR - AHB Clock Control Clear 1 */
/*! @{ */

#define MRCC_MRCC_GLB_CC1_CLR_DATA_MASK          (0xFFFFFFFFU)
#define MRCC_MRCC_GLB_CC1_CLR_DATA_SHIFT         (0U)
/*! DATA - Data array value, refer to corresponding position in MRCC_GLB_CCn. */
#define MRCC_MRCC_GLB_CC1_CLR_DATA(x)            (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_CC1_CLR_DATA_SHIFT)) & MRCC_MRCC_GLB_CC1_CLR_DATA_MASK)
/*! @} */

/*! @name MRCC_GLB_ACC0 - Control Automatic Clock Gating 0 */
/*! @{ */

#define MRCC_MRCC_GLB_ACC0_INPUTMUX0_MASK        (0x1U)
#define MRCC_MRCC_GLB_ACC0_INPUTMUX0_SHIFT       (0U)
/*! INPUTMUX0 - Write to INPUTMUX0
 *  0b0..Automatic clock gating is disabled
 *  0b1..Automatic clock gating is enabled
 */
#define MRCC_MRCC_GLB_ACC0_INPUTMUX0(x)          (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_ACC0_INPUTMUX0_SHIFT)) & MRCC_MRCC_GLB_ACC0_INPUTMUX0_MASK)

#define MRCC_MRCC_GLB_ACC0_I3C0_MASK             (0x2U)
#define MRCC_MRCC_GLB_ACC0_I3C0_SHIFT            (1U)
/*! I3C0 - Write to I3C0
 *  0b0..Automatic clock gating is disabled
 *  0b1..Automatic clock gating is enabled
 */
#define MRCC_MRCC_GLB_ACC0_I3C0(x)               (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_ACC0_I3C0_SHIFT)) & MRCC_MRCC_GLB_ACC0_I3C0_MASK)

#define MRCC_MRCC_GLB_ACC0_CTIMER0_MASK          (0x4U)
#define MRCC_MRCC_GLB_ACC0_CTIMER0_SHIFT         (2U)
/*! CTIMER0 - Write to CTIMER0
 *  0b0..Automatic clock gating is disabled
 *  0b1..Automatic clock gating is enabled
 */
#define MRCC_MRCC_GLB_ACC0_CTIMER0(x)            (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_ACC0_CTIMER0_SHIFT)) & MRCC_MRCC_GLB_ACC0_CTIMER0_MASK)

#define MRCC_MRCC_GLB_ACC0_CTIMER1_MASK          (0x8U)
#define MRCC_MRCC_GLB_ACC0_CTIMER1_SHIFT         (3U)
/*! CTIMER1 - Write to CTIMER1
 *  0b0..Automatic clock gating is disabled
 *  0b1..Automatic clock gating is enabled
 */
#define MRCC_MRCC_GLB_ACC0_CTIMER1(x)            (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_ACC0_CTIMER1_SHIFT)) & MRCC_MRCC_GLB_ACC0_CTIMER1_MASK)

#define MRCC_MRCC_GLB_ACC0_CTIMER2_MASK          (0x10U)
#define MRCC_MRCC_GLB_ACC0_CTIMER2_SHIFT         (4U)
/*! CTIMER2 - Write to CTIMER2
 *  0b0..Automatic clock gating is disabled
 *  0b1..Automatic clock gating is enabled
 */
#define MRCC_MRCC_GLB_ACC0_CTIMER2(x)            (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_ACC0_CTIMER2_SHIFT)) & MRCC_MRCC_GLB_ACC0_CTIMER2_MASK)

#define MRCC_MRCC_GLB_ACC0_FREQME_MASK           (0x20U)
#define MRCC_MRCC_GLB_ACC0_FREQME_SHIFT          (5U)
/*! FREQME - Write to FREQME
 *  0b0..Automatic clock gating is disabled
 *  0b1..Automatic clock gating is enabled
 */
#define MRCC_MRCC_GLB_ACC0_FREQME(x)             (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_ACC0_FREQME_SHIFT)) & MRCC_MRCC_GLB_ACC0_FREQME_MASK)

#define MRCC_MRCC_GLB_ACC0_UTICK0_MASK           (0x40U)
#define MRCC_MRCC_GLB_ACC0_UTICK0_SHIFT          (6U)
/*! UTICK0 - Write to UTICK0
 *  0b0..Automatic clock gating is disabled
 *  0b1..Automatic clock gating is enabled
 */
#define MRCC_MRCC_GLB_ACC0_UTICK0(x)             (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_ACC0_UTICK0_SHIFT)) & MRCC_MRCC_GLB_ACC0_UTICK0_MASK)

#define MRCC_MRCC_GLB_ACC0_WWDT0_MASK            (0x80U)
#define MRCC_MRCC_GLB_ACC0_WWDT0_SHIFT           (7U)
/*! WWDT0 - Write to WWDT0
 *  0b0..Automatic clock gating is disabled
 *  0b1..Automatic clock gating is enabled
 */
#define MRCC_MRCC_GLB_ACC0_WWDT0(x)              (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_ACC0_WWDT0_SHIFT)) & MRCC_MRCC_GLB_ACC0_WWDT0_MASK)

#define MRCC_MRCC_GLB_ACC0_DMA_MASK              (0x100U)
#define MRCC_MRCC_GLB_ACC0_DMA_SHIFT             (8U)
/*! DMA - Write to DMA
 *  0b0..Automatic clock gating is disabled
 *  0b1..Automatic clock gating is enabled
 */
#define MRCC_MRCC_GLB_ACC0_DMA(x)                (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_ACC0_DMA_SHIFT)) & MRCC_MRCC_GLB_ACC0_DMA_MASK)

#define MRCC_MRCC_GLB_ACC0_AOI0_MASK             (0x200U)
#define MRCC_MRCC_GLB_ACC0_AOI0_SHIFT            (9U)
/*! AOI0 - Write to AOI0
 *  0b0..Automatic clock gating is disabled
 *  0b1..Automatic clock gating is enabled
 */
#define MRCC_MRCC_GLB_ACC0_AOI0(x)               (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_ACC0_AOI0_SHIFT)) & MRCC_MRCC_GLB_ACC0_AOI0_MASK)

#define MRCC_MRCC_GLB_ACC0_CRC_MASK              (0x400U)
#define MRCC_MRCC_GLB_ACC0_CRC_SHIFT             (10U)
/*! CRC - Write to CRC
 *  0b0..Automatic clock gating is disabled
 *  0b1..Automatic clock gating is enabled
 */
#define MRCC_MRCC_GLB_ACC0_CRC(x)                (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_ACC0_CRC_SHIFT)) & MRCC_MRCC_GLB_ACC0_CRC_MASK)

#define MRCC_MRCC_GLB_ACC0_EIM_MASK              (0x800U)
#define MRCC_MRCC_GLB_ACC0_EIM_SHIFT             (11U)
/*! EIM - Write to EIM
 *  0b0..Automatic clock gating is disabled
 *  0b1..Automatic clock gating is enabled
 */
#define MRCC_MRCC_GLB_ACC0_EIM(x)                (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_ACC0_EIM_SHIFT)) & MRCC_MRCC_GLB_ACC0_EIM_MASK)

#define MRCC_MRCC_GLB_ACC0_ERM_MASK              (0x1000U)
#define MRCC_MRCC_GLB_ACC0_ERM_SHIFT             (12U)
/*! ERM - Write to ERM
 *  0b0..Automatic clock gating is disabled
 *  0b1..Automatic clock gating is enabled
 */
#define MRCC_MRCC_GLB_ACC0_ERM(x)                (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_ACC0_ERM_SHIFT)) & MRCC_MRCC_GLB_ACC0_ERM_MASK)

#define MRCC_MRCC_GLB_ACC0_LPI2C0_MASK           (0x10000U)
#define MRCC_MRCC_GLB_ACC0_LPI2C0_SHIFT          (16U)
/*! LPI2C0 - Write to LPI2C0
 *  0b0..Automatic clock gating is disabled
 *  0b1..Automatic clock gating is enabled
 */
#define MRCC_MRCC_GLB_ACC0_LPI2C0(x)             (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_ACC0_LPI2C0_SHIFT)) & MRCC_MRCC_GLB_ACC0_LPI2C0_MASK)

#define MRCC_MRCC_GLB_ACC0_LPSPI0_MASK           (0x20000U)
#define MRCC_MRCC_GLB_ACC0_LPSPI0_SHIFT          (17U)
/*! LPSPI0 - Write to LPSPI0
 *  0b0..Automatic clock gating is disabled
 *  0b1..Automatic clock gating is enabled
 */
#define MRCC_MRCC_GLB_ACC0_LPSPI0(x)             (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_ACC0_LPSPI0_SHIFT)) & MRCC_MRCC_GLB_ACC0_LPSPI0_MASK)

#define MRCC_MRCC_GLB_ACC0_LPSPI1_MASK           (0x40000U)
#define MRCC_MRCC_GLB_ACC0_LPSPI1_SHIFT          (18U)
/*! LPSPI1 - Write to LPSPI1
 *  0b0..Automatic clock gating is disabled
 *  0b1..Automatic clock gating is enabled
 */
#define MRCC_MRCC_GLB_ACC0_LPSPI1(x)             (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_ACC0_LPSPI1_SHIFT)) & MRCC_MRCC_GLB_ACC0_LPSPI1_MASK)

#define MRCC_MRCC_GLB_ACC0_LPUART0_MASK          (0x80000U)
#define MRCC_MRCC_GLB_ACC0_LPUART0_SHIFT         (19U)
/*! LPUART0 - Write to LPUART0
 *  0b0..Automatic clock gating is disabled
 *  0b1..Automatic clock gating is enabled
 */
#define MRCC_MRCC_GLB_ACC0_LPUART0(x)            (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_ACC0_LPUART0_SHIFT)) & MRCC_MRCC_GLB_ACC0_LPUART0_MASK)

#define MRCC_MRCC_GLB_ACC0_LPUART1_MASK          (0x100000U)
#define MRCC_MRCC_GLB_ACC0_LPUART1_SHIFT         (20U)
/*! LPUART1 - Write to LPUART1
 *  0b0..Automatic clock gating is disabled
 *  0b1..Automatic clock gating is enabled
 */
#define MRCC_MRCC_GLB_ACC0_LPUART1(x)            (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_ACC0_LPUART1_SHIFT)) & MRCC_MRCC_GLB_ACC0_LPUART1_MASK)

#define MRCC_MRCC_GLB_ACC0_LPUART2_MASK          (0x200000U)
#define MRCC_MRCC_GLB_ACC0_LPUART2_SHIFT         (21U)
/*! LPUART2 - Write to LPUART2
 *  0b0..Automatic clock gating is disabled
 *  0b1..Automatic clock gating is enabled
 */
#define MRCC_MRCC_GLB_ACC0_LPUART2(x)            (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_ACC0_LPUART2_SHIFT)) & MRCC_MRCC_GLB_ACC0_LPUART2_MASK)

#define MRCC_MRCC_GLB_ACC0_USB0_MASK             (0x400000U)
#define MRCC_MRCC_GLB_ACC0_USB0_SHIFT            (22U)
/*! USB0 - Write to USB0
 *  0b0..Automatic clock gating is disabled
 *  0b1..Automatic clock gating is enabled
 */
#define MRCC_MRCC_GLB_ACC0_USB0(x)               (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_ACC0_USB0_SHIFT)) & MRCC_MRCC_GLB_ACC0_USB0_MASK)

#define MRCC_MRCC_GLB_ACC0_QDC0_MASK             (0x800000U)
#define MRCC_MRCC_GLB_ACC0_QDC0_SHIFT            (23U)
/*! QDC0 - Write to QDC0
 *  0b0..Automatic clock gating is disabled
 *  0b1..Automatic clock gating is enabled
 */
#define MRCC_MRCC_GLB_ACC0_QDC0(x)               (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_ACC0_QDC0_SHIFT)) & MRCC_MRCC_GLB_ACC0_QDC0_MASK)

#define MRCC_MRCC_GLB_ACC0_FLEXPWM0_MASK         (0x1000000U)
#define MRCC_MRCC_GLB_ACC0_FLEXPWM0_SHIFT        (24U)
/*! FLEXPWM0 - Write to FLEXPWM0
 *  0b0..Automatic clock gating is disabled
 *  0b1..Automatic clock gating is enabled
 */
#define MRCC_MRCC_GLB_ACC0_FLEXPWM0(x)           (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_ACC0_FLEXPWM0_SHIFT)) & MRCC_MRCC_GLB_ACC0_FLEXPWM0_MASK)

#define MRCC_MRCC_GLB_ACC0_OSTIMER0_MASK         (0x2000000U)
#define MRCC_MRCC_GLB_ACC0_OSTIMER0_SHIFT        (25U)
/*! OSTIMER0 - Write to OSTIMER0
 *  0b0..Automatic clock gating is disabled
 *  0b1..Automatic clock gating is enabled
 */
#define MRCC_MRCC_GLB_ACC0_OSTIMER0(x)           (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_ACC0_OSTIMER0_SHIFT)) & MRCC_MRCC_GLB_ACC0_OSTIMER0_MASK)

#define MRCC_MRCC_GLB_ACC0_ADC0_MASK             (0x4000000U)
#define MRCC_MRCC_GLB_ACC0_ADC0_SHIFT            (26U)
/*! ADC0 - Write to ADC0
 *  0b0..Automatic clock gating is disabled
 *  0b1..Automatic clock gating is enabled
 */
#define MRCC_MRCC_GLB_ACC0_ADC0(x)               (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_ACC0_ADC0_SHIFT)) & MRCC_MRCC_GLB_ACC0_ADC0_MASK)

#define MRCC_MRCC_GLB_ACC0_CMP0_MASK             (0x8000000U)
#define MRCC_MRCC_GLB_ACC0_CMP0_SHIFT            (27U)
/*! CMP0 - Write to CMP0
 *  0b0..Automatic clock gating is disabled
 *  0b1..Automatic clock gating is enabled
 */
#define MRCC_MRCC_GLB_ACC0_CMP0(x)               (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_ACC0_CMP0_SHIFT)) & MRCC_MRCC_GLB_ACC0_CMP0_MASK)

#define MRCC_MRCC_GLB_ACC0_CMP1_MASK             (0x10000000U)
#define MRCC_MRCC_GLB_ACC0_CMP1_SHIFT            (28U)
/*! CMP1 - Write to CMP1
 *  0b0..Automatic clock gating is disabled
 *  0b1..Automatic clock gating is enabled
 */
#define MRCC_MRCC_GLB_ACC0_CMP1(x)               (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_ACC0_CMP1_SHIFT)) & MRCC_MRCC_GLB_ACC0_CMP1_MASK)

#define MRCC_MRCC_GLB_ACC0_PORT0_MASK            (0x20000000U)
#define MRCC_MRCC_GLB_ACC0_PORT0_SHIFT           (29U)
/*! PORT0 - Write to PORT0
 *  0b0..Automatic clock gating is disabled
 *  0b1..Automatic clock gating is enabled
 */
#define MRCC_MRCC_GLB_ACC0_PORT0(x)              (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_ACC0_PORT0_SHIFT)) & MRCC_MRCC_GLB_ACC0_PORT0_MASK)

#define MRCC_MRCC_GLB_ACC0_PORT1_MASK            (0x40000000U)
#define MRCC_MRCC_GLB_ACC0_PORT1_SHIFT           (30U)
/*! PORT1 - Write to PORT1
 *  0b0..Automatic clock gating is disabled
 *  0b1..Automatic clock gating is enabled
 */
#define MRCC_MRCC_GLB_ACC0_PORT1(x)              (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_ACC0_PORT1_SHIFT)) & MRCC_MRCC_GLB_ACC0_PORT1_MASK)

#define MRCC_MRCC_GLB_ACC0_PORT2_MASK            (0x80000000U)
#define MRCC_MRCC_GLB_ACC0_PORT2_SHIFT           (31U)
/*! PORT2 - Write to PORT2
 *  0b0..Automatic clock gating is disabled
 *  0b1..Automatic clock gating is enabled
 */
#define MRCC_MRCC_GLB_ACC0_PORT2(x)              (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_ACC0_PORT2_SHIFT)) & MRCC_MRCC_GLB_ACC0_PORT2_MASK)
/*! @} */

/*! @name MRCC_GLB_ACC1 - Control Automatic Clock Gating 1 */
/*! @{ */

#define MRCC_MRCC_GLB_ACC1_PORT3_MASK            (0x1U)
#define MRCC_MRCC_GLB_ACC1_PORT3_SHIFT           (0U)
/*! PORT3 - Write to PORT3
 *  0b0..Automatic clock gating is disabled
 *  0b1..Automatic clock gating is enabled
 */
#define MRCC_MRCC_GLB_ACC1_PORT3(x)              (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_ACC1_PORT3_SHIFT)) & MRCC_MRCC_GLB_ACC1_PORT3_MASK)

#define MRCC_MRCC_GLB_ACC1_EZRAMC_RAMA_MASK      (0x10U)
#define MRCC_MRCC_GLB_ACC1_EZRAMC_RAMA_SHIFT     (4U)
/*! EZRAMC_RAMA - Write to EZRAMC_RAMA
 *  0b0..Automatic clock gating is disabled
 *  0b1..Automatic clock gating is enabled
 */
#define MRCC_MRCC_GLB_ACC1_EZRAMC_RAMA(x)        (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_ACC1_EZRAMC_RAMA_SHIFT)) & MRCC_MRCC_GLB_ACC1_EZRAMC_RAMA_MASK)

#define MRCC_MRCC_GLB_ACC1_GPIO0_MASK            (0x20U)
#define MRCC_MRCC_GLB_ACC1_GPIO0_SHIFT           (5U)
/*! GPIO0 - Write to GPIO0
 *  0b0..Automatic clock gating is disabled
 *  0b1..Automatic clock gating is enabled
 */
#define MRCC_MRCC_GLB_ACC1_GPIO0(x)              (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_ACC1_GPIO0_SHIFT)) & MRCC_MRCC_GLB_ACC1_GPIO0_MASK)

#define MRCC_MRCC_GLB_ACC1_GPIO1_MASK            (0x40U)
#define MRCC_MRCC_GLB_ACC1_GPIO1_SHIFT           (6U)
/*! GPIO1 - Write to GPIO1
 *  0b0..Automatic clock gating is disabled
 *  0b1..Automatic clock gating is enabled
 */
#define MRCC_MRCC_GLB_ACC1_GPIO1(x)              (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_ACC1_GPIO1_SHIFT)) & MRCC_MRCC_GLB_ACC1_GPIO1_MASK)

#define MRCC_MRCC_GLB_ACC1_GPIO2_MASK            (0x80U)
#define MRCC_MRCC_GLB_ACC1_GPIO2_SHIFT           (7U)
/*! GPIO2 - Write to GPIO2
 *  0b0..Automatic clock gating is disabled
 *  0b1..Automatic clock gating is enabled
 */
#define MRCC_MRCC_GLB_ACC1_GPIO2(x)              (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_ACC1_GPIO2_SHIFT)) & MRCC_MRCC_GLB_ACC1_GPIO2_MASK)

#define MRCC_MRCC_GLB_ACC1_GPIO3_MASK            (0x100U)
#define MRCC_MRCC_GLB_ACC1_GPIO3_SHIFT           (8U)
/*! GPIO3 - Write to GPIO3
 *  0b0..Automatic clock gating is disabled
 *  0b1..Automatic clock gating is enabled
 */
#define MRCC_MRCC_GLB_ACC1_GPIO3(x)              (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_ACC1_GPIO3_SHIFT)) & MRCC_MRCC_GLB_ACC1_GPIO3_MASK)

#define MRCC_MRCC_GLB_ACC1_ROMCP_MASK            (0x200U)
#define MRCC_MRCC_GLB_ACC1_ROMCP_SHIFT           (9U)
/*! ROMCP - Write to ROMCP
 *  0b0..Automatic clock gating is disabled
 *  0b1..Automatic clock gating is enabled
 */
#define MRCC_MRCC_GLB_ACC1_ROMCP(x)              (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_GLB_ACC1_ROMCP_SHIFT)) & MRCC_MRCC_GLB_ACC1_ROMCP_MASK)
/*! @} */

/*! @name MRCC_I3C0_FCLK_CLKSEL - I3C0_FCLK clock selection control */
/*! @{ */

#define MRCC_MRCC_I3C0_FCLK_CLKSEL_MUX_MASK      (0x7U)
#define MRCC_MRCC_I3C0_FCLK_CLKSEL_MUX_SHIFT     (0U)
/*! MUX - Functional Clock Mux Select
 *  0b000..FRO_12M
 *  0b010..FRO_HF_DIV
 *  0b011..CLK_IN
 *  0b101..CLK_1M
 */
#define MRCC_MRCC_I3C0_FCLK_CLKSEL_MUX(x)        (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_I3C0_FCLK_CLKSEL_MUX_SHIFT)) & MRCC_MRCC_I3C0_FCLK_CLKSEL_MUX_MASK)
/*! @} */

/*! @name MRCC_I3C0_FCLK_CLKDIV - I3C0_FCLK clock divider control */
/*! @{ */

#define MRCC_MRCC_I3C0_FCLK_CLKDIV_DIV_MASK      (0xFU)
#define MRCC_MRCC_I3C0_FCLK_CLKDIV_DIV_SHIFT     (0U)
/*! DIV - Functional Clock Divider */
#define MRCC_MRCC_I3C0_FCLK_CLKDIV_DIV(x)        (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_I3C0_FCLK_CLKDIV_DIV_SHIFT)) & MRCC_MRCC_I3C0_FCLK_CLKDIV_DIV_MASK)

#define MRCC_MRCC_I3C0_FCLK_CLKDIV_RESET_MASK    (0x20000000U)
#define MRCC_MRCC_I3C0_FCLK_CLKDIV_RESET_SHIFT   (29U)
/*! RESET - Reset divider counter
 *  0b0..Divider isn't reset
 *  0b1..Divider is reset
 */
#define MRCC_MRCC_I3C0_FCLK_CLKDIV_RESET(x)      (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_I3C0_FCLK_CLKDIV_RESET_SHIFT)) & MRCC_MRCC_I3C0_FCLK_CLKDIV_RESET_MASK)

#define MRCC_MRCC_I3C0_FCLK_CLKDIV_HALT_MASK     (0x40000000U)
#define MRCC_MRCC_I3C0_FCLK_CLKDIV_HALT_SHIFT    (30U)
/*! HALT - Halt divider counter
 *  0b0..Divider clock is running
 *  0b1..Divider clock is stopped
 */
#define MRCC_MRCC_I3C0_FCLK_CLKDIV_HALT(x)       (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_I3C0_FCLK_CLKDIV_HALT_SHIFT)) & MRCC_MRCC_I3C0_FCLK_CLKDIV_HALT_MASK)

#define MRCC_MRCC_I3C0_FCLK_CLKDIV_UNSTAB_MASK   (0x80000000U)
#define MRCC_MRCC_I3C0_FCLK_CLKDIV_UNSTAB_SHIFT  (31U)
/*! UNSTAB - Divider status flag
 *  0b0..Divider clock is stable
 *  0b1..Clock frequency isn't stable
 */
#define MRCC_MRCC_I3C0_FCLK_CLKDIV_UNSTAB(x)     (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_I3C0_FCLK_CLKDIV_UNSTAB_SHIFT)) & MRCC_MRCC_I3C0_FCLK_CLKDIV_UNSTAB_MASK)
/*! @} */

/*! @name MRCC_CTIMER0_CLKSEL - CTIMER0 clock selection control */
/*! @{ */

#define MRCC_MRCC_CTIMER0_CLKSEL_MUX_MASK        (0x7U)
#define MRCC_MRCC_CTIMER0_CLKSEL_MUX_SHIFT       (0U)
/*! MUX - Functional Clock Mux Select
 *  0b000..FRO_12M
 *  0b001..FRO_HF_GATED
 *  0b011..CLK_IN
 *  0b100..CLK_16K
 *  0b101..CLK_1M
 */
#define MRCC_MRCC_CTIMER0_CLKSEL_MUX(x)          (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_CTIMER0_CLKSEL_MUX_SHIFT)) & MRCC_MRCC_CTIMER0_CLKSEL_MUX_MASK)
/*! @} */

/*! @name MRCC_CTIMER0_CLKDIV - CTIMER0 clock divider control */
/*! @{ */

#define MRCC_MRCC_CTIMER0_CLKDIV_DIV_MASK        (0xFU)
#define MRCC_MRCC_CTIMER0_CLKDIV_DIV_SHIFT       (0U)
/*! DIV - Functional Clock Divider */
#define MRCC_MRCC_CTIMER0_CLKDIV_DIV(x)          (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_CTIMER0_CLKDIV_DIV_SHIFT)) & MRCC_MRCC_CTIMER0_CLKDIV_DIV_MASK)

#define MRCC_MRCC_CTIMER0_CLKDIV_RESET_MASK      (0x20000000U)
#define MRCC_MRCC_CTIMER0_CLKDIV_RESET_SHIFT     (29U)
/*! RESET - Reset divider counter
 *  0b0..Divider isn't reset
 *  0b1..Divider is reset
 */
#define MRCC_MRCC_CTIMER0_CLKDIV_RESET(x)        (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_CTIMER0_CLKDIV_RESET_SHIFT)) & MRCC_MRCC_CTIMER0_CLKDIV_RESET_MASK)

#define MRCC_MRCC_CTIMER0_CLKDIV_HALT_MASK       (0x40000000U)
#define MRCC_MRCC_CTIMER0_CLKDIV_HALT_SHIFT      (30U)
/*! HALT - Halt divider counter
 *  0b0..Divider clock is running
 *  0b1..Divider clock is stopped
 */
#define MRCC_MRCC_CTIMER0_CLKDIV_HALT(x)         (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_CTIMER0_CLKDIV_HALT_SHIFT)) & MRCC_MRCC_CTIMER0_CLKDIV_HALT_MASK)

#define MRCC_MRCC_CTIMER0_CLKDIV_UNSTAB_MASK     (0x80000000U)
#define MRCC_MRCC_CTIMER0_CLKDIV_UNSTAB_SHIFT    (31U)
/*! UNSTAB - Divider status flag
 *  0b0..Divider clock is stable
 *  0b1..Clock frequency isn't stable
 */
#define MRCC_MRCC_CTIMER0_CLKDIV_UNSTAB(x)       (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_CTIMER0_CLKDIV_UNSTAB_SHIFT)) & MRCC_MRCC_CTIMER0_CLKDIV_UNSTAB_MASK)
/*! @} */

/*! @name MRCC_CTIMER1_CLKSEL - CTIMER1 clock selection control */
/*! @{ */

#define MRCC_MRCC_CTIMER1_CLKSEL_MUX_MASK        (0x7U)
#define MRCC_MRCC_CTIMER1_CLKSEL_MUX_SHIFT       (0U)
/*! MUX - Functional Clock Mux Select
 *  0b000..FRO_12M
 *  0b001..FRO_HF_GATED
 *  0b011..CLK_IN
 *  0b100..CLK_16K
 *  0b101..CLK_1M
 */
#define MRCC_MRCC_CTIMER1_CLKSEL_MUX(x)          (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_CTIMER1_CLKSEL_MUX_SHIFT)) & MRCC_MRCC_CTIMER1_CLKSEL_MUX_MASK)
/*! @} */

/*! @name MRCC_CTIMER1_CLKDIV - CTIMER1 clock divider control */
/*! @{ */

#define MRCC_MRCC_CTIMER1_CLKDIV_DIV_MASK        (0xFU)
#define MRCC_MRCC_CTIMER1_CLKDIV_DIV_SHIFT       (0U)
/*! DIV - Functional Clock Divider */
#define MRCC_MRCC_CTIMER1_CLKDIV_DIV(x)          (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_CTIMER1_CLKDIV_DIV_SHIFT)) & MRCC_MRCC_CTIMER1_CLKDIV_DIV_MASK)

#define MRCC_MRCC_CTIMER1_CLKDIV_RESET_MASK      (0x20000000U)
#define MRCC_MRCC_CTIMER1_CLKDIV_RESET_SHIFT     (29U)
/*! RESET - Reset divider counter
 *  0b0..Divider isn't reset
 *  0b1..Divider is reset
 */
#define MRCC_MRCC_CTIMER1_CLKDIV_RESET(x)        (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_CTIMER1_CLKDIV_RESET_SHIFT)) & MRCC_MRCC_CTIMER1_CLKDIV_RESET_MASK)

#define MRCC_MRCC_CTIMER1_CLKDIV_HALT_MASK       (0x40000000U)
#define MRCC_MRCC_CTIMER1_CLKDIV_HALT_SHIFT      (30U)
/*! HALT - Halt divider counter
 *  0b0..Divider clock is running
 *  0b1..Divider clock is stopped
 */
#define MRCC_MRCC_CTIMER1_CLKDIV_HALT(x)         (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_CTIMER1_CLKDIV_HALT_SHIFT)) & MRCC_MRCC_CTIMER1_CLKDIV_HALT_MASK)

#define MRCC_MRCC_CTIMER1_CLKDIV_UNSTAB_MASK     (0x80000000U)
#define MRCC_MRCC_CTIMER1_CLKDIV_UNSTAB_SHIFT    (31U)
/*! UNSTAB - Divider status flag
 *  0b0..Divider clock is stable
 *  0b1..Clock frequency isn't stable
 */
#define MRCC_MRCC_CTIMER1_CLKDIV_UNSTAB(x)       (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_CTIMER1_CLKDIV_UNSTAB_SHIFT)) & MRCC_MRCC_CTIMER1_CLKDIV_UNSTAB_MASK)
/*! @} */

/*! @name MRCC_CTIMER2_CLKSEL - CTIMER2 clock selection control */
/*! @{ */

#define MRCC_MRCC_CTIMER2_CLKSEL_MUX_MASK        (0x7U)
#define MRCC_MRCC_CTIMER2_CLKSEL_MUX_SHIFT       (0U)
/*! MUX - Functional Clock Mux Select
 *  0b000..FRO_12M
 *  0b001..FRO_HF_GATED
 *  0b011..CLK_IN
 *  0b100..CLK_16K
 *  0b101..CLK_1M
 */
#define MRCC_MRCC_CTIMER2_CLKSEL_MUX(x)          (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_CTIMER2_CLKSEL_MUX_SHIFT)) & MRCC_MRCC_CTIMER2_CLKSEL_MUX_MASK)
/*! @} */

/*! @name MRCC_CTIMER2_CLKDIV - CTIMER2 clock divider control */
/*! @{ */

#define MRCC_MRCC_CTIMER2_CLKDIV_DIV_MASK        (0xFU)
#define MRCC_MRCC_CTIMER2_CLKDIV_DIV_SHIFT       (0U)
/*! DIV - Functional Clock Divider */
#define MRCC_MRCC_CTIMER2_CLKDIV_DIV(x)          (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_CTIMER2_CLKDIV_DIV_SHIFT)) & MRCC_MRCC_CTIMER2_CLKDIV_DIV_MASK)

#define MRCC_MRCC_CTIMER2_CLKDIV_RESET_MASK      (0x20000000U)
#define MRCC_MRCC_CTIMER2_CLKDIV_RESET_SHIFT     (29U)
/*! RESET - Reset divider counter
 *  0b0..Divider isn't reset
 *  0b1..Divider is reset
 */
#define MRCC_MRCC_CTIMER2_CLKDIV_RESET(x)        (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_CTIMER2_CLKDIV_RESET_SHIFT)) & MRCC_MRCC_CTIMER2_CLKDIV_RESET_MASK)

#define MRCC_MRCC_CTIMER2_CLKDIV_HALT_MASK       (0x40000000U)
#define MRCC_MRCC_CTIMER2_CLKDIV_HALT_SHIFT      (30U)
/*! HALT - Halt divider counter
 *  0b0..Divider clock is running
 *  0b1..Divider clock is stopped
 */
#define MRCC_MRCC_CTIMER2_CLKDIV_HALT(x)         (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_CTIMER2_CLKDIV_HALT_SHIFT)) & MRCC_MRCC_CTIMER2_CLKDIV_HALT_MASK)

#define MRCC_MRCC_CTIMER2_CLKDIV_UNSTAB_MASK     (0x80000000U)
#define MRCC_MRCC_CTIMER2_CLKDIV_UNSTAB_SHIFT    (31U)
/*! UNSTAB - Divider status flag
 *  0b0..Divider clock is stable
 *  0b1..Clock frequency isn't stable
 */
#define MRCC_MRCC_CTIMER2_CLKDIV_UNSTAB(x)       (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_CTIMER2_CLKDIV_UNSTAB_SHIFT)) & MRCC_MRCC_CTIMER2_CLKDIV_UNSTAB_MASK)
/*! @} */

/*! @name MRCC_WWDT0_CLKDIV - WWDT0 clock divider control */
/*! @{ */

#define MRCC_MRCC_WWDT0_CLKDIV_DIV_MASK          (0xFU)
#define MRCC_MRCC_WWDT0_CLKDIV_DIV_SHIFT         (0U)
/*! DIV - Functional Clock Divider */
#define MRCC_MRCC_WWDT0_CLKDIV_DIV(x)            (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_WWDT0_CLKDIV_DIV_SHIFT)) & MRCC_MRCC_WWDT0_CLKDIV_DIV_MASK)

#define MRCC_MRCC_WWDT0_CLKDIV_RESET_MASK        (0x20000000U)
#define MRCC_MRCC_WWDT0_CLKDIV_RESET_SHIFT       (29U)
/*! RESET - Reset divider counter
 *  0b0..Divider isn't reset
 *  0b1..Divider is reset
 */
#define MRCC_MRCC_WWDT0_CLKDIV_RESET(x)          (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_WWDT0_CLKDIV_RESET_SHIFT)) & MRCC_MRCC_WWDT0_CLKDIV_RESET_MASK)

#define MRCC_MRCC_WWDT0_CLKDIV_HALT_MASK         (0x40000000U)
#define MRCC_MRCC_WWDT0_CLKDIV_HALT_SHIFT        (30U)
/*! HALT - Halt divider counter
 *  0b0..Divider clock is running
 *  0b1..Divider clock is stopped
 */
#define MRCC_MRCC_WWDT0_CLKDIV_HALT(x)           (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_WWDT0_CLKDIV_HALT_SHIFT)) & MRCC_MRCC_WWDT0_CLKDIV_HALT_MASK)

#define MRCC_MRCC_WWDT0_CLKDIV_UNSTAB_MASK       (0x80000000U)
#define MRCC_MRCC_WWDT0_CLKDIV_UNSTAB_SHIFT      (31U)
/*! UNSTAB - Divider status flag
 *  0b0..Divider clock is stable
 *  0b1..Clock frequency isn't stable
 */
#define MRCC_MRCC_WWDT0_CLKDIV_UNSTAB(x)         (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_WWDT0_CLKDIV_UNSTAB_SHIFT)) & MRCC_MRCC_WWDT0_CLKDIV_UNSTAB_MASK)
/*! @} */

/*! @name MRCC_LPI2C0_CLKSEL - LPI2C0 clock selection control */
/*! @{ */

#define MRCC_MRCC_LPI2C0_CLKSEL_MUX_MASK         (0x7U)
#define MRCC_MRCC_LPI2C0_CLKSEL_MUX_SHIFT        (0U)
/*! MUX - Functional Clock Mux Select
 *  0b000..FRO_12M
 *  0b010..FRO_HF_DIV
 *  0b011..CLK_IN
 *  0b101..CLK_1M
 */
#define MRCC_MRCC_LPI2C0_CLKSEL_MUX(x)           (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_LPI2C0_CLKSEL_MUX_SHIFT)) & MRCC_MRCC_LPI2C0_CLKSEL_MUX_MASK)
/*! @} */

/*! @name MRCC_LPI2C0_CLKDIV - LPI2C0 clock divider control */
/*! @{ */

#define MRCC_MRCC_LPI2C0_CLKDIV_DIV_MASK         (0xFU)
#define MRCC_MRCC_LPI2C0_CLKDIV_DIV_SHIFT        (0U)
/*! DIV - Functional Clock Divider */
#define MRCC_MRCC_LPI2C0_CLKDIV_DIV(x)           (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_LPI2C0_CLKDIV_DIV_SHIFT)) & MRCC_MRCC_LPI2C0_CLKDIV_DIV_MASK)

#define MRCC_MRCC_LPI2C0_CLKDIV_RESET_MASK       (0x20000000U)
#define MRCC_MRCC_LPI2C0_CLKDIV_RESET_SHIFT      (29U)
/*! RESET - Reset divider counter
 *  0b0..Divider isn't reset
 *  0b1..Divider is reset
 */
#define MRCC_MRCC_LPI2C0_CLKDIV_RESET(x)         (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_LPI2C0_CLKDIV_RESET_SHIFT)) & MRCC_MRCC_LPI2C0_CLKDIV_RESET_MASK)

#define MRCC_MRCC_LPI2C0_CLKDIV_HALT_MASK        (0x40000000U)
#define MRCC_MRCC_LPI2C0_CLKDIV_HALT_SHIFT       (30U)
/*! HALT - Halt divider counter
 *  0b0..Divider clock is running
 *  0b1..Divider clock is stopped
 */
#define MRCC_MRCC_LPI2C0_CLKDIV_HALT(x)          (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_LPI2C0_CLKDIV_HALT_SHIFT)) & MRCC_MRCC_LPI2C0_CLKDIV_HALT_MASK)

#define MRCC_MRCC_LPI2C0_CLKDIV_UNSTAB_MASK      (0x80000000U)
#define MRCC_MRCC_LPI2C0_CLKDIV_UNSTAB_SHIFT     (31U)
/*! UNSTAB - Divider status flag
 *  0b0..Divider clock is stable
 *  0b1..Clock frequency isn't stable
 */
#define MRCC_MRCC_LPI2C0_CLKDIV_UNSTAB(x)        (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_LPI2C0_CLKDIV_UNSTAB_SHIFT)) & MRCC_MRCC_LPI2C0_CLKDIV_UNSTAB_MASK)
/*! @} */

/*! @name MRCC_LPSPI0_CLKSEL - LPSPI0 clock selection control */
/*! @{ */

#define MRCC_MRCC_LPSPI0_CLKSEL_MUX_MASK         (0x7U)
#define MRCC_MRCC_LPSPI0_CLKSEL_MUX_SHIFT        (0U)
/*! MUX - Functional Clock Mux Select
 *  0b000..FRO_12M
 *  0b010..FRO_HF_DIV
 *  0b011..CLK_IN
 *  0b101..CLK_1M
 */
#define MRCC_MRCC_LPSPI0_CLKSEL_MUX(x)           (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_LPSPI0_CLKSEL_MUX_SHIFT)) & MRCC_MRCC_LPSPI0_CLKSEL_MUX_MASK)
/*! @} */

/*! @name MRCC_LPSPI0_CLKDIV - LPSPI0 clock divider control */
/*! @{ */

#define MRCC_MRCC_LPSPI0_CLKDIV_DIV_MASK         (0xFU)
#define MRCC_MRCC_LPSPI0_CLKDIV_DIV_SHIFT        (0U)
/*! DIV - Functional Clock Divider */
#define MRCC_MRCC_LPSPI0_CLKDIV_DIV(x)           (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_LPSPI0_CLKDIV_DIV_SHIFT)) & MRCC_MRCC_LPSPI0_CLKDIV_DIV_MASK)

#define MRCC_MRCC_LPSPI0_CLKDIV_RESET_MASK       (0x20000000U)
#define MRCC_MRCC_LPSPI0_CLKDIV_RESET_SHIFT      (29U)
/*! RESET - Reset divider counter
 *  0b0..Divider isn't reset
 *  0b1..Divider is reset
 */
#define MRCC_MRCC_LPSPI0_CLKDIV_RESET(x)         (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_LPSPI0_CLKDIV_RESET_SHIFT)) & MRCC_MRCC_LPSPI0_CLKDIV_RESET_MASK)

#define MRCC_MRCC_LPSPI0_CLKDIV_HALT_MASK        (0x40000000U)
#define MRCC_MRCC_LPSPI0_CLKDIV_HALT_SHIFT       (30U)
/*! HALT - Halt divider counter
 *  0b0..Divider clock is running
 *  0b1..Divider clock is stopped
 */
#define MRCC_MRCC_LPSPI0_CLKDIV_HALT(x)          (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_LPSPI0_CLKDIV_HALT_SHIFT)) & MRCC_MRCC_LPSPI0_CLKDIV_HALT_MASK)

#define MRCC_MRCC_LPSPI0_CLKDIV_UNSTAB_MASK      (0x80000000U)
#define MRCC_MRCC_LPSPI0_CLKDIV_UNSTAB_SHIFT     (31U)
/*! UNSTAB - Divider status flag
 *  0b0..Divider clock is stable
 *  0b1..Clock frequency isn't stable
 */
#define MRCC_MRCC_LPSPI0_CLKDIV_UNSTAB(x)        (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_LPSPI0_CLKDIV_UNSTAB_SHIFT)) & MRCC_MRCC_LPSPI0_CLKDIV_UNSTAB_MASK)
/*! @} */

/*! @name MRCC_LPSPI1_CLKSEL - LPSPI1 clock selection control */
/*! @{ */

#define MRCC_MRCC_LPSPI1_CLKSEL_MUX_MASK         (0x7U)
#define MRCC_MRCC_LPSPI1_CLKSEL_MUX_SHIFT        (0U)
/*! MUX - Functional Clock Mux Select
 *  0b000..FRO_12M
 *  0b010..FRO_HF_DIV
 *  0b011..CLK_IN
 *  0b101..CLK_1M
 */
#define MRCC_MRCC_LPSPI1_CLKSEL_MUX(x)           (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_LPSPI1_CLKSEL_MUX_SHIFT)) & MRCC_MRCC_LPSPI1_CLKSEL_MUX_MASK)
/*! @} */

/*! @name MRCC_LPSPI1_CLKDIV - LPSPI1 clock divider control */
/*! @{ */

#define MRCC_MRCC_LPSPI1_CLKDIV_DIV_MASK         (0xFU)
#define MRCC_MRCC_LPSPI1_CLKDIV_DIV_SHIFT        (0U)
/*! DIV - Functional Clock Divider */
#define MRCC_MRCC_LPSPI1_CLKDIV_DIV(x)           (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_LPSPI1_CLKDIV_DIV_SHIFT)) & MRCC_MRCC_LPSPI1_CLKDIV_DIV_MASK)

#define MRCC_MRCC_LPSPI1_CLKDIV_RESET_MASK       (0x20000000U)
#define MRCC_MRCC_LPSPI1_CLKDIV_RESET_SHIFT      (29U)
/*! RESET - Reset divider counter
 *  0b0..Divider isn't reset
 *  0b1..Divider is reset
 */
#define MRCC_MRCC_LPSPI1_CLKDIV_RESET(x)         (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_LPSPI1_CLKDIV_RESET_SHIFT)) & MRCC_MRCC_LPSPI1_CLKDIV_RESET_MASK)

#define MRCC_MRCC_LPSPI1_CLKDIV_HALT_MASK        (0x40000000U)
#define MRCC_MRCC_LPSPI1_CLKDIV_HALT_SHIFT       (30U)
/*! HALT - Halt divider counter
 *  0b0..Divider clock is running
 *  0b1..Divider clock is stopped
 */
#define MRCC_MRCC_LPSPI1_CLKDIV_HALT(x)          (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_LPSPI1_CLKDIV_HALT_SHIFT)) & MRCC_MRCC_LPSPI1_CLKDIV_HALT_MASK)

#define MRCC_MRCC_LPSPI1_CLKDIV_UNSTAB_MASK      (0x80000000U)
#define MRCC_MRCC_LPSPI1_CLKDIV_UNSTAB_SHIFT     (31U)
/*! UNSTAB - Divider status flag
 *  0b0..Divider clock is stable
 *  0b1..Clock frequency isn't stable
 */
#define MRCC_MRCC_LPSPI1_CLKDIV_UNSTAB(x)        (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_LPSPI1_CLKDIV_UNSTAB_SHIFT)) & MRCC_MRCC_LPSPI1_CLKDIV_UNSTAB_MASK)
/*! @} */

/*! @name MRCC_LPUART0_CLKSEL - LPUART0 clock selection control */
/*! @{ */

#define MRCC_MRCC_LPUART0_CLKSEL_MUX_MASK        (0x7U)
#define MRCC_MRCC_LPUART0_CLKSEL_MUX_SHIFT       (0U)
/*! MUX - Functional Clock Mux Select
 *  0b000..FRO_12M
 *  0b010..FRO_HF_DIV
 *  0b011..CLK_IN
 *  0b100..CLK_16K
 *  0b101..CLK_1M
 */
#define MRCC_MRCC_LPUART0_CLKSEL_MUX(x)          (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_LPUART0_CLKSEL_MUX_SHIFT)) & MRCC_MRCC_LPUART0_CLKSEL_MUX_MASK)
/*! @} */

/*! @name MRCC_LPUART0_CLKDIV - LPUART0 clock divider control */
/*! @{ */

#define MRCC_MRCC_LPUART0_CLKDIV_DIV_MASK        (0xFU)
#define MRCC_MRCC_LPUART0_CLKDIV_DIV_SHIFT       (0U)
/*! DIV - Functional Clock Divider */
#define MRCC_MRCC_LPUART0_CLKDIV_DIV(x)          (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_LPUART0_CLKDIV_DIV_SHIFT)) & MRCC_MRCC_LPUART0_CLKDIV_DIV_MASK)

#define MRCC_MRCC_LPUART0_CLKDIV_RESET_MASK      (0x20000000U)
#define MRCC_MRCC_LPUART0_CLKDIV_RESET_SHIFT     (29U)
/*! RESET - Reset divider counter
 *  0b0..Divider isn't reset
 *  0b1..Divider is reset
 */
#define MRCC_MRCC_LPUART0_CLKDIV_RESET(x)        (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_LPUART0_CLKDIV_RESET_SHIFT)) & MRCC_MRCC_LPUART0_CLKDIV_RESET_MASK)

#define MRCC_MRCC_LPUART0_CLKDIV_HALT_MASK       (0x40000000U)
#define MRCC_MRCC_LPUART0_CLKDIV_HALT_SHIFT      (30U)
/*! HALT - Halt divider counter
 *  0b0..Divider clock is running
 *  0b1..Divider clock is stopped
 */
#define MRCC_MRCC_LPUART0_CLKDIV_HALT(x)         (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_LPUART0_CLKDIV_HALT_SHIFT)) & MRCC_MRCC_LPUART0_CLKDIV_HALT_MASK)

#define MRCC_MRCC_LPUART0_CLKDIV_UNSTAB_MASK     (0x80000000U)
#define MRCC_MRCC_LPUART0_CLKDIV_UNSTAB_SHIFT    (31U)
/*! UNSTAB - Divider status flag
 *  0b0..Divider clock is stable
 *  0b1..Clock frequency isn't stable
 */
#define MRCC_MRCC_LPUART0_CLKDIV_UNSTAB(x)       (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_LPUART0_CLKDIV_UNSTAB_SHIFT)) & MRCC_MRCC_LPUART0_CLKDIV_UNSTAB_MASK)
/*! @} */

/*! @name MRCC_LPUART1_CLKSEL - LPUART1 clock selection control */
/*! @{ */

#define MRCC_MRCC_LPUART1_CLKSEL_MUX_MASK        (0x7U)
#define MRCC_MRCC_LPUART1_CLKSEL_MUX_SHIFT       (0U)
/*! MUX - Functional Clock Mux Select
 *  0b000..FRO_12M
 *  0b010..FRO_HF_DIV
 *  0b011..CLK_IN
 *  0b100..CLK_16K
 *  0b101..CLK_1M
 */
#define MRCC_MRCC_LPUART1_CLKSEL_MUX(x)          (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_LPUART1_CLKSEL_MUX_SHIFT)) & MRCC_MRCC_LPUART1_CLKSEL_MUX_MASK)
/*! @} */

/*! @name MRCC_LPUART1_CLKDIV - LPUART1 clock divider control */
/*! @{ */

#define MRCC_MRCC_LPUART1_CLKDIV_DIV_MASK        (0xFU)
#define MRCC_MRCC_LPUART1_CLKDIV_DIV_SHIFT       (0U)
/*! DIV - Functional Clock Divider */
#define MRCC_MRCC_LPUART1_CLKDIV_DIV(x)          (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_LPUART1_CLKDIV_DIV_SHIFT)) & MRCC_MRCC_LPUART1_CLKDIV_DIV_MASK)

#define MRCC_MRCC_LPUART1_CLKDIV_RESET_MASK      (0x20000000U)
#define MRCC_MRCC_LPUART1_CLKDIV_RESET_SHIFT     (29U)
/*! RESET - Reset divider counter
 *  0b0..Divider isn't reset
 *  0b1..Divider is reset
 */
#define MRCC_MRCC_LPUART1_CLKDIV_RESET(x)        (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_LPUART1_CLKDIV_RESET_SHIFT)) & MRCC_MRCC_LPUART1_CLKDIV_RESET_MASK)

#define MRCC_MRCC_LPUART1_CLKDIV_HALT_MASK       (0x40000000U)
#define MRCC_MRCC_LPUART1_CLKDIV_HALT_SHIFT      (30U)
/*! HALT - Halt divider counter
 *  0b0..Divider clock is running
 *  0b1..Divider clock is stopped
 */
#define MRCC_MRCC_LPUART1_CLKDIV_HALT(x)         (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_LPUART1_CLKDIV_HALT_SHIFT)) & MRCC_MRCC_LPUART1_CLKDIV_HALT_MASK)

#define MRCC_MRCC_LPUART1_CLKDIV_UNSTAB_MASK     (0x80000000U)
#define MRCC_MRCC_LPUART1_CLKDIV_UNSTAB_SHIFT    (31U)
/*! UNSTAB - Divider status flag
 *  0b0..Divider clock is stable
 *  0b1..Clock frequency isn't stable
 */
#define MRCC_MRCC_LPUART1_CLKDIV_UNSTAB(x)       (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_LPUART1_CLKDIV_UNSTAB_SHIFT)) & MRCC_MRCC_LPUART1_CLKDIV_UNSTAB_MASK)
/*! @} */

/*! @name MRCC_LPUART2_CLKSEL - LPUART2 clock selection control */
/*! @{ */

#define MRCC_MRCC_LPUART2_CLKSEL_MUX_MASK        (0x7U)
#define MRCC_MRCC_LPUART2_CLKSEL_MUX_SHIFT       (0U)
/*! MUX - Functional Clock Mux Select
 *  0b000..FRO_12M
 *  0b010..FRO_HF_DIV
 *  0b011..CLK_IN
 *  0b100..CLK_16K
 *  0b101..CLK_1M
 */
#define MRCC_MRCC_LPUART2_CLKSEL_MUX(x)          (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_LPUART2_CLKSEL_MUX_SHIFT)) & MRCC_MRCC_LPUART2_CLKSEL_MUX_MASK)
/*! @} */

/*! @name MRCC_LPUART2_CLKDIV - LPUART2 clock divider control */
/*! @{ */

#define MRCC_MRCC_LPUART2_CLKDIV_DIV_MASK        (0xFU)
#define MRCC_MRCC_LPUART2_CLKDIV_DIV_SHIFT       (0U)
/*! DIV - Functional Clock Divider */
#define MRCC_MRCC_LPUART2_CLKDIV_DIV(x)          (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_LPUART2_CLKDIV_DIV_SHIFT)) & MRCC_MRCC_LPUART2_CLKDIV_DIV_MASK)

#define MRCC_MRCC_LPUART2_CLKDIV_RESET_MASK      (0x20000000U)
#define MRCC_MRCC_LPUART2_CLKDIV_RESET_SHIFT     (29U)
/*! RESET - Reset divider counter
 *  0b0..Divider isn't reset
 *  0b1..Divider is reset
 */
#define MRCC_MRCC_LPUART2_CLKDIV_RESET(x)        (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_LPUART2_CLKDIV_RESET_SHIFT)) & MRCC_MRCC_LPUART2_CLKDIV_RESET_MASK)

#define MRCC_MRCC_LPUART2_CLKDIV_HALT_MASK       (0x40000000U)
#define MRCC_MRCC_LPUART2_CLKDIV_HALT_SHIFT      (30U)
/*! HALT - Halt divider counter
 *  0b0..Divider clock is running
 *  0b1..Divider clock is stopped
 */
#define MRCC_MRCC_LPUART2_CLKDIV_HALT(x)         (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_LPUART2_CLKDIV_HALT_SHIFT)) & MRCC_MRCC_LPUART2_CLKDIV_HALT_MASK)

#define MRCC_MRCC_LPUART2_CLKDIV_UNSTAB_MASK     (0x80000000U)
#define MRCC_MRCC_LPUART2_CLKDIV_UNSTAB_SHIFT    (31U)
/*! UNSTAB - Divider status flag
 *  0b0..Divider clock is stable
 *  0b1..Clock frequency isn't stable
 */
#define MRCC_MRCC_LPUART2_CLKDIV_UNSTAB(x)       (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_LPUART2_CLKDIV_UNSTAB_SHIFT)) & MRCC_MRCC_LPUART2_CLKDIV_UNSTAB_MASK)
/*! @} */

/*! @name MRCC_USB0_CLKSEL - USB0 clock selection control */
/*! @{ */

#define MRCC_MRCC_USB0_CLKSEL_MUX_MASK           (0x3U)
#define MRCC_MRCC_USB0_CLKSEL_MUX_SHIFT          (0U)
/*! MUX - Functional Clock Mux Select
 *  0b01..scg_scg_firc_48mhz_clk
 *  0b10..clkroot_sosc
 */
#define MRCC_MRCC_USB0_CLKSEL_MUX(x)             (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_USB0_CLKSEL_MUX_SHIFT)) & MRCC_MRCC_USB0_CLKSEL_MUX_MASK)
/*! @} */

/*! @name MRCC_LPTMR0_CLKSEL - LPTMR0 clock selection control */
/*! @{ */

#define MRCC_MRCC_LPTMR0_CLKSEL_MUX_MASK         (0x7U)
#define MRCC_MRCC_LPTMR0_CLKSEL_MUX_SHIFT        (0U)
/*! MUX - Functional Clock Mux Select
 *  0b000..FRO_12M
 *  0b010..FRO_HF_DIV
 *  0b011..CLK_IN
 *  0b101..CLK_1M
 */
#define MRCC_MRCC_LPTMR0_CLKSEL_MUX(x)           (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_LPTMR0_CLKSEL_MUX_SHIFT)) & MRCC_MRCC_LPTMR0_CLKSEL_MUX_MASK)
/*! @} */

/*! @name MRCC_LPTMR0_CLKDIV - LPTMR0 clock divider control */
/*! @{ */

#define MRCC_MRCC_LPTMR0_CLKDIV_DIV_MASK         (0xFU)
#define MRCC_MRCC_LPTMR0_CLKDIV_DIV_SHIFT        (0U)
/*! DIV - Functional Clock Divider */
#define MRCC_MRCC_LPTMR0_CLKDIV_DIV(x)           (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_LPTMR0_CLKDIV_DIV_SHIFT)) & MRCC_MRCC_LPTMR0_CLKDIV_DIV_MASK)

#define MRCC_MRCC_LPTMR0_CLKDIV_RESET_MASK       (0x20000000U)
#define MRCC_MRCC_LPTMR0_CLKDIV_RESET_SHIFT      (29U)
/*! RESET - Reset divider counter
 *  0b0..Divider isn't reset
 *  0b1..Divider is reset
 */
#define MRCC_MRCC_LPTMR0_CLKDIV_RESET(x)         (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_LPTMR0_CLKDIV_RESET_SHIFT)) & MRCC_MRCC_LPTMR0_CLKDIV_RESET_MASK)

#define MRCC_MRCC_LPTMR0_CLKDIV_HALT_MASK        (0x40000000U)
#define MRCC_MRCC_LPTMR0_CLKDIV_HALT_SHIFT       (30U)
/*! HALT - Halt divider counter
 *  0b0..Divider clock is running
 *  0b1..Divider clock is stopped
 */
#define MRCC_MRCC_LPTMR0_CLKDIV_HALT(x)          (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_LPTMR0_CLKDIV_HALT_SHIFT)) & MRCC_MRCC_LPTMR0_CLKDIV_HALT_MASK)

#define MRCC_MRCC_LPTMR0_CLKDIV_UNSTAB_MASK      (0x80000000U)
#define MRCC_MRCC_LPTMR0_CLKDIV_UNSTAB_SHIFT     (31U)
/*! UNSTAB - Divider status flag
 *  0b0..Divider clock is stable
 *  0b1..Clock frequency isn't stable
 */
#define MRCC_MRCC_LPTMR0_CLKDIV_UNSTAB(x)        (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_LPTMR0_CLKDIV_UNSTAB_SHIFT)) & MRCC_MRCC_LPTMR0_CLKDIV_UNSTAB_MASK)
/*! @} */

/*! @name MRCC_OSTIMER0_CLKSEL - OSTIMER0 clock selection control */
/*! @{ */

#define MRCC_MRCC_OSTIMER0_CLKSEL_MUX_MASK       (0x3U)
#define MRCC_MRCC_OSTIMER0_CLKSEL_MUX_SHIFT      (0U)
/*! MUX - Functional Clock Mux Select
 *  0b00..clkroot_16k
 *  0b10..clkroot_1m
 */
#define MRCC_MRCC_OSTIMER0_CLKSEL_MUX(x)         (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_OSTIMER0_CLKSEL_MUX_SHIFT)) & MRCC_MRCC_OSTIMER0_CLKSEL_MUX_MASK)
/*! @} */

/*! @name MRCC_ADC0_CLKSEL - ADC0 clock selection control */
/*! @{ */

#define MRCC_MRCC_ADC0_CLKSEL_MUX_MASK           (0x7U)
#define MRCC_MRCC_ADC0_CLKSEL_MUX_SHIFT          (0U)
/*! MUX - Functional Clock Mux Select
 *  0b000..FRO_12M
 *  0b001..FRO_HF_GATED
 *  0b011..CLK_IN
 *  0b101..CLK_1M
 */
#define MRCC_MRCC_ADC0_CLKSEL_MUX(x)             (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_ADC0_CLKSEL_MUX_SHIFT)) & MRCC_MRCC_ADC0_CLKSEL_MUX_MASK)
/*! @} */

/*! @name MRCC_ADC0_CLKDIV - ADC0 clock divider control */
/*! @{ */

#define MRCC_MRCC_ADC0_CLKDIV_DIV_MASK           (0xFU)
#define MRCC_MRCC_ADC0_CLKDIV_DIV_SHIFT          (0U)
/*! DIV - Functional Clock Divider */
#define MRCC_MRCC_ADC0_CLKDIV_DIV(x)             (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_ADC0_CLKDIV_DIV_SHIFT)) & MRCC_MRCC_ADC0_CLKDIV_DIV_MASK)

#define MRCC_MRCC_ADC0_CLKDIV_RESET_MASK         (0x20000000U)
#define MRCC_MRCC_ADC0_CLKDIV_RESET_SHIFT        (29U)
/*! RESET - Reset divider counter
 *  0b0..Divider isn't reset
 *  0b1..Divider is reset
 */
#define MRCC_MRCC_ADC0_CLKDIV_RESET(x)           (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_ADC0_CLKDIV_RESET_SHIFT)) & MRCC_MRCC_ADC0_CLKDIV_RESET_MASK)

#define MRCC_MRCC_ADC0_CLKDIV_HALT_MASK          (0x40000000U)
#define MRCC_MRCC_ADC0_CLKDIV_HALT_SHIFT         (30U)
/*! HALT - Halt divider counter
 *  0b0..Divider clock is running
 *  0b1..Divider clock is stopped
 */
#define MRCC_MRCC_ADC0_CLKDIV_HALT(x)            (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_ADC0_CLKDIV_HALT_SHIFT)) & MRCC_MRCC_ADC0_CLKDIV_HALT_MASK)

#define MRCC_MRCC_ADC0_CLKDIV_UNSTAB_MASK        (0x80000000U)
#define MRCC_MRCC_ADC0_CLKDIV_UNSTAB_SHIFT       (31U)
/*! UNSTAB - Divider status flag
 *  0b0..Divider clock is stable
 *  0b1..Clock frequency isn't stable
 */
#define MRCC_MRCC_ADC0_CLKDIV_UNSTAB(x)          (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_ADC0_CLKDIV_UNSTAB_SHIFT)) & MRCC_MRCC_ADC0_CLKDIV_UNSTAB_MASK)
/*! @} */

/*! @name MRCC_CMP0_FUNC_CLKDIV - CMP0_FUNC clock divider control */
/*! @{ */

#define MRCC_MRCC_CMP0_FUNC_CLKDIV_DIV_MASK      (0xFU)
#define MRCC_MRCC_CMP0_FUNC_CLKDIV_DIV_SHIFT     (0U)
/*! DIV - Functional Clock Divider */
#define MRCC_MRCC_CMP0_FUNC_CLKDIV_DIV(x)        (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_CMP0_FUNC_CLKDIV_DIV_SHIFT)) & MRCC_MRCC_CMP0_FUNC_CLKDIV_DIV_MASK)

#define MRCC_MRCC_CMP0_FUNC_CLKDIV_RESET_MASK    (0x20000000U)
#define MRCC_MRCC_CMP0_FUNC_CLKDIV_RESET_SHIFT   (29U)
/*! RESET - Reset divider counter
 *  0b0..Divider isn't reset
 *  0b1..Divider is reset
 */
#define MRCC_MRCC_CMP0_FUNC_CLKDIV_RESET(x)      (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_CMP0_FUNC_CLKDIV_RESET_SHIFT)) & MRCC_MRCC_CMP0_FUNC_CLKDIV_RESET_MASK)

#define MRCC_MRCC_CMP0_FUNC_CLKDIV_HALT_MASK     (0x40000000U)
#define MRCC_MRCC_CMP0_FUNC_CLKDIV_HALT_SHIFT    (30U)
/*! HALT - Halt divider counter
 *  0b0..Divider clock is running
 *  0b1..Divider clock is stopped
 */
#define MRCC_MRCC_CMP0_FUNC_CLKDIV_HALT(x)       (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_CMP0_FUNC_CLKDIV_HALT_SHIFT)) & MRCC_MRCC_CMP0_FUNC_CLKDIV_HALT_MASK)

#define MRCC_MRCC_CMP0_FUNC_CLKDIV_UNSTAB_MASK   (0x80000000U)
#define MRCC_MRCC_CMP0_FUNC_CLKDIV_UNSTAB_SHIFT  (31U)
/*! UNSTAB - Divider status flag
 *  0b0..Divider clock is stable
 *  0b1..Clock frequency isn't stable
 */
#define MRCC_MRCC_CMP0_FUNC_CLKDIV_UNSTAB(x)     (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_CMP0_FUNC_CLKDIV_UNSTAB_SHIFT)) & MRCC_MRCC_CMP0_FUNC_CLKDIV_UNSTAB_MASK)
/*! @} */

/*! @name MRCC_CMP0_RR_CLKSEL - CMP0_RR clock selection control */
/*! @{ */

#define MRCC_MRCC_CMP0_RR_CLKSEL_MUX_MASK        (0x7U)
#define MRCC_MRCC_CMP0_RR_CLKSEL_MUX_SHIFT       (0U)
/*! MUX - Functional Clock Mux Select
 *  0b000..FRO_12M
 *  0b010..FRO_HF_DIV
 *  0b011..CLK_IN
 *  0b101..CLK_1M
 */
#define MRCC_MRCC_CMP0_RR_CLKSEL_MUX(x)          (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_CMP0_RR_CLKSEL_MUX_SHIFT)) & MRCC_MRCC_CMP0_RR_CLKSEL_MUX_MASK)
/*! @} */

/*! @name MRCC_CMP0_RR_CLKDIV - CMP0_RR clock divider control */
/*! @{ */

#define MRCC_MRCC_CMP0_RR_CLKDIV_DIV_MASK        (0x3U)
#define MRCC_MRCC_CMP0_RR_CLKDIV_DIV_SHIFT       (0U)
/*! DIV - Functional Clock Divider */
#define MRCC_MRCC_CMP0_RR_CLKDIV_DIV(x)          (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_CMP0_RR_CLKDIV_DIV_SHIFT)) & MRCC_MRCC_CMP0_RR_CLKDIV_DIV_MASK)

#define MRCC_MRCC_CMP0_RR_CLKDIV_RESET_MASK      (0x20000000U)
#define MRCC_MRCC_CMP0_RR_CLKDIV_RESET_SHIFT     (29U)
/*! RESET - Reset divider counter
 *  0b0..Divider isn't reset
 *  0b1..Divider is reset
 */
#define MRCC_MRCC_CMP0_RR_CLKDIV_RESET(x)        (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_CMP0_RR_CLKDIV_RESET_SHIFT)) & MRCC_MRCC_CMP0_RR_CLKDIV_RESET_MASK)

#define MRCC_MRCC_CMP0_RR_CLKDIV_HALT_MASK       (0x40000000U)
#define MRCC_MRCC_CMP0_RR_CLKDIV_HALT_SHIFT      (30U)
/*! HALT - Halt divider counter
 *  0b0..Divider clock is running
 *  0b1..Divider clock is stopped
 */
#define MRCC_MRCC_CMP0_RR_CLKDIV_HALT(x)         (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_CMP0_RR_CLKDIV_HALT_SHIFT)) & MRCC_MRCC_CMP0_RR_CLKDIV_HALT_MASK)

#define MRCC_MRCC_CMP0_RR_CLKDIV_UNSTAB_MASK     (0x80000000U)
#define MRCC_MRCC_CMP0_RR_CLKDIV_UNSTAB_SHIFT    (31U)
/*! UNSTAB - Divider status flag
 *  0b0..Divider clock is stable
 *  0b1..Clock frequency isn't stable
 */
#define MRCC_MRCC_CMP0_RR_CLKDIV_UNSTAB(x)       (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_CMP0_RR_CLKDIV_UNSTAB_SHIFT)) & MRCC_MRCC_CMP0_RR_CLKDIV_UNSTAB_MASK)
/*! @} */

/*! @name MRCC_CMP1_FUNC_CLKDIV - CMP1_FUNC clock divider control */
/*! @{ */

#define MRCC_MRCC_CMP1_FUNC_CLKDIV_DIV_MASK      (0xFU)
#define MRCC_MRCC_CMP1_FUNC_CLKDIV_DIV_SHIFT     (0U)
/*! DIV - Functional Clock Divider */
#define MRCC_MRCC_CMP1_FUNC_CLKDIV_DIV(x)        (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_CMP1_FUNC_CLKDIV_DIV_SHIFT)) & MRCC_MRCC_CMP1_FUNC_CLKDIV_DIV_MASK)

#define MRCC_MRCC_CMP1_FUNC_CLKDIV_RESET_MASK    (0x20000000U)
#define MRCC_MRCC_CMP1_FUNC_CLKDIV_RESET_SHIFT   (29U)
/*! RESET - Reset divider counter
 *  0b0..Divider isn't reset
 *  0b1..Divider is reset
 */
#define MRCC_MRCC_CMP1_FUNC_CLKDIV_RESET(x)      (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_CMP1_FUNC_CLKDIV_RESET_SHIFT)) & MRCC_MRCC_CMP1_FUNC_CLKDIV_RESET_MASK)

#define MRCC_MRCC_CMP1_FUNC_CLKDIV_HALT_MASK     (0x40000000U)
#define MRCC_MRCC_CMP1_FUNC_CLKDIV_HALT_SHIFT    (30U)
/*! HALT - Halt divider counter
 *  0b0..Divider clock is running
 *  0b1..Divider clock is stopped
 */
#define MRCC_MRCC_CMP1_FUNC_CLKDIV_HALT(x)       (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_CMP1_FUNC_CLKDIV_HALT_SHIFT)) & MRCC_MRCC_CMP1_FUNC_CLKDIV_HALT_MASK)

#define MRCC_MRCC_CMP1_FUNC_CLKDIV_UNSTAB_MASK   (0x80000000U)
#define MRCC_MRCC_CMP1_FUNC_CLKDIV_UNSTAB_SHIFT  (31U)
/*! UNSTAB - Divider status flag
 *  0b0..Divider clock is stable
 *  0b1..Clock frequency isn't stable
 */
#define MRCC_MRCC_CMP1_FUNC_CLKDIV_UNSTAB(x)     (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_CMP1_FUNC_CLKDIV_UNSTAB_SHIFT)) & MRCC_MRCC_CMP1_FUNC_CLKDIV_UNSTAB_MASK)
/*! @} */

/*! @name MRCC_CMP1_RR_CLKSEL - CMP1_RR clock selection control */
/*! @{ */

#define MRCC_MRCC_CMP1_RR_CLKSEL_MUX_MASK        (0x7U)
#define MRCC_MRCC_CMP1_RR_CLKSEL_MUX_SHIFT       (0U)
/*! MUX - Functional Clock Mux Select
 *  0b000..FRO_12M
 *  0b010..FRO_HF_DIV
 *  0b011..CLK_IN
 *  0b101..CLK_1M
 */
#define MRCC_MRCC_CMP1_RR_CLKSEL_MUX(x)          (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_CMP1_RR_CLKSEL_MUX_SHIFT)) & MRCC_MRCC_CMP1_RR_CLKSEL_MUX_MASK)
/*! @} */

/*! @name MRCC_CMP1_RR_CLKDIV - CMP1_RR clock divider control */
/*! @{ */

#define MRCC_MRCC_CMP1_RR_CLKDIV_DIV_MASK        (0x3U)
#define MRCC_MRCC_CMP1_RR_CLKDIV_DIV_SHIFT       (0U)
/*! DIV - Functional Clock Divider */
#define MRCC_MRCC_CMP1_RR_CLKDIV_DIV(x)          (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_CMP1_RR_CLKDIV_DIV_SHIFT)) & MRCC_MRCC_CMP1_RR_CLKDIV_DIV_MASK)

#define MRCC_MRCC_CMP1_RR_CLKDIV_RESET_MASK      (0x20000000U)
#define MRCC_MRCC_CMP1_RR_CLKDIV_RESET_SHIFT     (29U)
/*! RESET - Reset divider counter
 *  0b0..Divider isn't reset
 *  0b1..Divider is reset
 */
#define MRCC_MRCC_CMP1_RR_CLKDIV_RESET(x)        (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_CMP1_RR_CLKDIV_RESET_SHIFT)) & MRCC_MRCC_CMP1_RR_CLKDIV_RESET_MASK)

#define MRCC_MRCC_CMP1_RR_CLKDIV_HALT_MASK       (0x40000000U)
#define MRCC_MRCC_CMP1_RR_CLKDIV_HALT_SHIFT      (30U)
/*! HALT - Halt divider counter
 *  0b0..Divider clock is running
 *  0b1..Divider clock is stopped
 */
#define MRCC_MRCC_CMP1_RR_CLKDIV_HALT(x)         (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_CMP1_RR_CLKDIV_HALT_SHIFT)) & MRCC_MRCC_CMP1_RR_CLKDIV_HALT_MASK)

#define MRCC_MRCC_CMP1_RR_CLKDIV_UNSTAB_MASK     (0x80000000U)
#define MRCC_MRCC_CMP1_RR_CLKDIV_UNSTAB_SHIFT    (31U)
/*! UNSTAB - Divider status flag
 *  0b0..Divider clock is stable
 *  0b1..Clock frequency isn't stable
 */
#define MRCC_MRCC_CMP1_RR_CLKDIV_UNSTAB(x)       (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_CMP1_RR_CLKDIV_UNSTAB_SHIFT)) & MRCC_MRCC_CMP1_RR_CLKDIV_UNSTAB_MASK)
/*! @} */

/*! @name MRCC_DBG_TRACE_CLKSEL - DBG_TRACE clock selection control */
/*! @{ */

#define MRCC_MRCC_DBG_TRACE_CLKSEL_MUX_MASK      (0x3U)
#define MRCC_MRCC_DBG_TRACE_CLKSEL_MUX_SHIFT     (0U)
/*! MUX - Functional Clock Mux Select
 *  0b00..clkroot_cpu
 *  0b01..clkroot_1m
 *  0b10..clkroot_16k
 */
#define MRCC_MRCC_DBG_TRACE_CLKSEL_MUX(x)        (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_DBG_TRACE_CLKSEL_MUX_SHIFT)) & MRCC_MRCC_DBG_TRACE_CLKSEL_MUX_MASK)
/*! @} */

/*! @name MRCC_DBG_TRACE_CLKDIV - DBG_TRACE clock divider control */
/*! @{ */

#define MRCC_MRCC_DBG_TRACE_CLKDIV_DIV_MASK      (0xFU)
#define MRCC_MRCC_DBG_TRACE_CLKDIV_DIV_SHIFT     (0U)
/*! DIV - Functional Clock Divider */
#define MRCC_MRCC_DBG_TRACE_CLKDIV_DIV(x)        (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_DBG_TRACE_CLKDIV_DIV_SHIFT)) & MRCC_MRCC_DBG_TRACE_CLKDIV_DIV_MASK)

#define MRCC_MRCC_DBG_TRACE_CLKDIV_RESET_MASK    (0x20000000U)
#define MRCC_MRCC_DBG_TRACE_CLKDIV_RESET_SHIFT   (29U)
/*! RESET - Reset divider counter
 *  0b0..Divider isn't reset
 *  0b1..Divider is reset
 */
#define MRCC_MRCC_DBG_TRACE_CLKDIV_RESET(x)      (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_DBG_TRACE_CLKDIV_RESET_SHIFT)) & MRCC_MRCC_DBG_TRACE_CLKDIV_RESET_MASK)

#define MRCC_MRCC_DBG_TRACE_CLKDIV_HALT_MASK     (0x40000000U)
#define MRCC_MRCC_DBG_TRACE_CLKDIV_HALT_SHIFT    (30U)
/*! HALT - Halt divider counter
 *  0b0..Divider clock is running
 *  0b1..Divider clock is stopped
 */
#define MRCC_MRCC_DBG_TRACE_CLKDIV_HALT(x)       (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_DBG_TRACE_CLKDIV_HALT_SHIFT)) & MRCC_MRCC_DBG_TRACE_CLKDIV_HALT_MASK)

#define MRCC_MRCC_DBG_TRACE_CLKDIV_UNSTAB_MASK   (0x80000000U)
#define MRCC_MRCC_DBG_TRACE_CLKDIV_UNSTAB_SHIFT  (31U)
/*! UNSTAB - Divider status flag
 *  0b0..Divider clock is stable
 *  0b1..Clock frequency isn't stable
 */
#define MRCC_MRCC_DBG_TRACE_CLKDIV_UNSTAB(x)     (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_DBG_TRACE_CLKDIV_UNSTAB_SHIFT)) & MRCC_MRCC_DBG_TRACE_CLKDIV_UNSTAB_MASK)
/*! @} */

/*! @name MRCC_CLKOUT_CLKSEL - CLKOUT clock selection control */
/*! @{ */

#define MRCC_MRCC_CLKOUT_CLKSEL_MUX_MASK         (0x7U)
#define MRCC_MRCC_CLKOUT_CLKSEL_MUX_SHIFT        (0U)
/*! MUX - Functional Clock Mux Select
 *  0b000..clkroot_12m
 *  0b001..clkroot_firc_div
 *  0b010..clkroot_sosc
 *  0b011..clkroot_16k
 *  0b110..clkroot_slow
 */
#define MRCC_MRCC_CLKOUT_CLKSEL_MUX(x)           (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_CLKOUT_CLKSEL_MUX_SHIFT)) & MRCC_MRCC_CLKOUT_CLKSEL_MUX_MASK)
/*! @} */

/*! @name MRCC_CLKOUT_CLKDIV - CLKOUT clock divider control */
/*! @{ */

#define MRCC_MRCC_CLKOUT_CLKDIV_DIV_MASK         (0xFU)
#define MRCC_MRCC_CLKOUT_CLKDIV_DIV_SHIFT        (0U)
/*! DIV - Functional Clock Divider */
#define MRCC_MRCC_CLKOUT_CLKDIV_DIV(x)           (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_CLKOUT_CLKDIV_DIV_SHIFT)) & MRCC_MRCC_CLKOUT_CLKDIV_DIV_MASK)

#define MRCC_MRCC_CLKOUT_CLKDIV_RESET_MASK       (0x20000000U)
#define MRCC_MRCC_CLKOUT_CLKDIV_RESET_SHIFT      (29U)
/*! RESET - Reset divider counter
 *  0b0..Divider isn't reset
 *  0b1..Divider is reset
 */
#define MRCC_MRCC_CLKOUT_CLKDIV_RESET(x)         (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_CLKOUT_CLKDIV_RESET_SHIFT)) & MRCC_MRCC_CLKOUT_CLKDIV_RESET_MASK)

#define MRCC_MRCC_CLKOUT_CLKDIV_HALT_MASK        (0x40000000U)
#define MRCC_MRCC_CLKOUT_CLKDIV_HALT_SHIFT       (30U)
/*! HALT - Halt divider counter
 *  0b0..Divider clock is running
 *  0b1..Divider clock is stopped
 */
#define MRCC_MRCC_CLKOUT_CLKDIV_HALT(x)          (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_CLKOUT_CLKDIV_HALT_SHIFT)) & MRCC_MRCC_CLKOUT_CLKDIV_HALT_MASK)

#define MRCC_MRCC_CLKOUT_CLKDIV_UNSTAB_MASK      (0x80000000U)
#define MRCC_MRCC_CLKOUT_CLKDIV_UNSTAB_SHIFT     (31U)
/*! UNSTAB - Divider status flag
 *  0b0..Divider clock is stable
 *  0b1..Clock frequency is not stable
 */
#define MRCC_MRCC_CLKOUT_CLKDIV_UNSTAB(x)        (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_CLKOUT_CLKDIV_UNSTAB_SHIFT)) & MRCC_MRCC_CLKOUT_CLKDIV_UNSTAB_MASK)
/*! @} */

/*! @name MRCC_SYSTICK_CLKSEL - SYSTICK clock selection control */
/*! @{ */

#define MRCC_MRCC_SYSTICK_CLKSEL_MUX_MASK        (0x3U)
#define MRCC_MRCC_SYSTICK_CLKSEL_MUX_SHIFT       (0U)
/*! MUX - Functional Clock Mux Select
 *  0b00..clkroot_cpu
 *  0b01..clkroot_1m
 *  0b10..clkroot_16k
 */
#define MRCC_MRCC_SYSTICK_CLKSEL_MUX(x)          (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_SYSTICK_CLKSEL_MUX_SHIFT)) & MRCC_MRCC_SYSTICK_CLKSEL_MUX_MASK)
/*! @} */

/*! @name MRCC_SYSTICK_CLKDIV - SYSTICK clock divider control */
/*! @{ */

#define MRCC_MRCC_SYSTICK_CLKDIV_DIV_MASK        (0xFU)
#define MRCC_MRCC_SYSTICK_CLKDIV_DIV_SHIFT       (0U)
/*! DIV - Functional Clock Divider */
#define MRCC_MRCC_SYSTICK_CLKDIV_DIV(x)          (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_SYSTICK_CLKDIV_DIV_SHIFT)) & MRCC_MRCC_SYSTICK_CLKDIV_DIV_MASK)

#define MRCC_MRCC_SYSTICK_CLKDIV_RESET_MASK      (0x20000000U)
#define MRCC_MRCC_SYSTICK_CLKDIV_RESET_SHIFT     (29U)
/*! RESET - Reset divider counter
 *  0b0..Divider isn't reset
 *  0b1..Divider is reset
 */
#define MRCC_MRCC_SYSTICK_CLKDIV_RESET(x)        (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_SYSTICK_CLKDIV_RESET_SHIFT)) & MRCC_MRCC_SYSTICK_CLKDIV_RESET_MASK)

#define MRCC_MRCC_SYSTICK_CLKDIV_HALT_MASK       (0x40000000U)
#define MRCC_MRCC_SYSTICK_CLKDIV_HALT_SHIFT      (30U)
/*! HALT - Halt divider counter
 *  0b0..Divider clock is running
 *  0b1..Divider clock is stopped
 */
#define MRCC_MRCC_SYSTICK_CLKDIV_HALT(x)         (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_SYSTICK_CLKDIV_HALT_SHIFT)) & MRCC_MRCC_SYSTICK_CLKDIV_HALT_MASK)

#define MRCC_MRCC_SYSTICK_CLKDIV_UNSTAB_MASK     (0x80000000U)
#define MRCC_MRCC_SYSTICK_CLKDIV_UNSTAB_SHIFT    (31U)
/*! UNSTAB - Divider status flag
 *  0b0..Divider clock is stable
 *  0b1..Clock frequency isn't stable
 */
#define MRCC_MRCC_SYSTICK_CLKDIV_UNSTAB(x)       (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_SYSTICK_CLKDIV_UNSTAB_SHIFT)) & MRCC_MRCC_SYSTICK_CLKDIV_UNSTAB_MASK)
/*! @} */

/*! @name MRCC_FRO_HF_DIV_CLKDIV - FRO_HF_DIV clock divider control */
/*! @{ */

#define MRCC_MRCC_FRO_HF_DIV_CLKDIV_DIV_MASK     (0xFU)
#define MRCC_MRCC_FRO_HF_DIV_CLKDIV_DIV_SHIFT    (0U)
/*! DIV - Functional Clock Divider */
#define MRCC_MRCC_FRO_HF_DIV_CLKDIV_DIV(x)       (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_FRO_HF_DIV_CLKDIV_DIV_SHIFT)) & MRCC_MRCC_FRO_HF_DIV_CLKDIV_DIV_MASK)

#define MRCC_MRCC_FRO_HF_DIV_CLKDIV_UNSTAB_MASK  (0x80000000U)
#define MRCC_MRCC_FRO_HF_DIV_CLKDIV_UNSTAB_SHIFT (31U)
/*! UNSTAB - Divider status flag
 *  0b0..Divider clock is stable
 *  0b1..Clock frequency isn't stable
 */
#define MRCC_MRCC_FRO_HF_DIV_CLKDIV_UNSTAB(x)    (((uint32_t)(((uint32_t)(x)) << MRCC_MRCC_FRO_HF_DIV_CLKDIV_UNSTAB_SHIFT)) & MRCC_MRCC_FRO_HF_DIV_CLKDIV_UNSTAB_MASK)
/*! @} */


/*!
 * @}
 */ /* end of group MRCC_Register_Masks */


/*!
 * @}
 */ /* end of group MRCC_Peripheral_Access_Layer */


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


#endif  /* PERI_MRCC_H_ */

