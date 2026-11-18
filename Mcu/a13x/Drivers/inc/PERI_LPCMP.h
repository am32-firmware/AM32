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
**         CMSIS Peripheral Access Layer for LPCMP
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
 * @file PERI_LPCMP.h
 * @version 2.0
 * @date 2024-10-29
 * @brief CMSIS Peripheral Access Layer for LPCMP
 *
 * CMSIS Peripheral Access Layer for LPCMP
 */

#if !defined(PERI_LPCMP_H_)
#define PERI_LPCMP_H_                            /**< Symbol preventing repeated inclusion */

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
   -- LPCMP Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LPCMP_Peripheral_Access_Layer LPCMP Peripheral Access Layer
 * @{
 */

/** LPCMP - Register Layout Typedef */
typedef struct {
  __I  uint32_t VERID;                             /**< Version ID, offset: 0x0 */
  __I  uint32_t PARAM;                             /**< Parameter, offset: 0x4 */
  __IO uint32_t CCR0;                              /**< Comparator Control Register 0, offset: 0x8 */
  __IO uint32_t CCR1;                              /**< Comparator Control Register 1, offset: 0xC */
  __IO uint32_t CCR2;                              /**< Comparator Control Register 2, offset: 0x10 */
       uint8_t RESERVED_0[4];
  __IO uint32_t DCR;                               /**< DAC Control, offset: 0x18 */
  __IO uint32_t IER;                               /**< Interrupt Enable, offset: 0x1C */
  __IO uint32_t CSR;                               /**< Comparator Status, offset: 0x20 */
  __IO uint32_t RRCR0;                             /**< Round Robin Control Register 0, offset: 0x24 */
  __IO uint32_t RRCR1;                             /**< Round Robin Control Register 1, offset: 0x28 */
  __IO uint32_t RRCSR;                             /**< Round Robin Control and Status, offset: 0x2C */
  __IO uint32_t RRSR;                              /**< Round Robin Status, offset: 0x30 */
       uint8_t RESERVED_1[4];
  __IO uint32_t RRCR2;                             /**< Round Robin Control Register 2, offset: 0x38 */
} LPCMP_Type;

/* ----------------------------------------------------------------------------
   -- LPCMP Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LPCMP_Register_Masks LPCMP Register Masks
 * @{
 */

/*! @name VERID - Version ID */
/*! @{ */

#define LPCMP_VERID_FEATURE_MASK                 (0xFFFFU)
#define LPCMP_VERID_FEATURE_SHIFT                (0U)
/*! FEATURE - Feature Specification Number
 *  0b0000000000000001..Round robin feature
 */
#define LPCMP_VERID_FEATURE(x)                   (((uint32_t)(((uint32_t)(x)) << LPCMP_VERID_FEATURE_SHIFT)) & LPCMP_VERID_FEATURE_MASK)

#define LPCMP_VERID_MINOR_MASK                   (0xFF0000U)
#define LPCMP_VERID_MINOR_SHIFT                  (16U)
/*! MINOR - Minor Version Number */
#define LPCMP_VERID_MINOR(x)                     (((uint32_t)(((uint32_t)(x)) << LPCMP_VERID_MINOR_SHIFT)) & LPCMP_VERID_MINOR_MASK)

#define LPCMP_VERID_MAJOR_MASK                   (0xFF000000U)
#define LPCMP_VERID_MAJOR_SHIFT                  (24U)
/*! MAJOR - Major Version Number */
#define LPCMP_VERID_MAJOR(x)                     (((uint32_t)(((uint32_t)(x)) << LPCMP_VERID_MAJOR_SHIFT)) & LPCMP_VERID_MAJOR_MASK)
/*! @} */

/*! @name PARAM - Parameter */
/*! @{ */

#define LPCMP_PARAM_DAC_RES_MASK                 (0xFU)
#define LPCMP_PARAM_DAC_RES_SHIFT                (0U)
/*! DAC_RES - DAC Resolution
 *  0b0000..4-bit DAC
 *  0b0001..6-bit DAC
 *  0b0010..8-bit DAC
 *  0b0011..10-bit DAC
 *  0b0100..12-bit DAC
 *  0b0101..14-bit DAC
 *  0b0110..16-bit DAC
 */
#define LPCMP_PARAM_DAC_RES(x)                   (((uint32_t)(((uint32_t)(x)) << LPCMP_PARAM_DAC_RES_SHIFT)) & LPCMP_PARAM_DAC_RES_MASK)
/*! @} */

/*! @name CCR0 - Comparator Control Register 0 */
/*! @{ */

#define LPCMP_CCR0_CMP_EN_MASK                   (0x1U)
#define LPCMP_CCR0_CMP_EN_SHIFT                  (0U)
/*! CMP_EN - Comparator Enable
 *  0b0..Disables (The analog logic remains off and consumes no power.)
 *  0b1..Enables
 */
#define LPCMP_CCR0_CMP_EN(x)                     (((uint32_t)(((uint32_t)(x)) << LPCMP_CCR0_CMP_EN_SHIFT)) & LPCMP_CCR0_CMP_EN_MASK)

#define LPCMP_CCR0_CMP_STOP_EN_MASK              (0x2U)
#define LPCMP_CCR0_CMP_STOP_EN_SHIFT             (1U)
/*! CMP_STOP_EN - Comparator Deep sleep Mode Enable
 *  0b0..Disable the analog comparator regardless of CMP_EN.
 *  0b1..Allows CMP_EN to enable the analog comparator.
 */
#define LPCMP_CCR0_CMP_STOP_EN(x)                (((uint32_t)(((uint32_t)(x)) << LPCMP_CCR0_CMP_STOP_EN_SHIFT)) & LPCMP_CCR0_CMP_STOP_EN_MASK)
/*! @} */

/*! @name CCR1 - Comparator Control Register 1 */
/*! @{ */

#define LPCMP_CCR1_WINDOW_EN_MASK                (0x1U)
#define LPCMP_CCR1_WINDOW_EN_SHIFT               (0U)
/*! WINDOW_EN - Windowing Enable
 *  0b0..Disables
 *  0b1..Enables
 */
#define LPCMP_CCR1_WINDOW_EN(x)                  (((uint32_t)(((uint32_t)(x)) << LPCMP_CCR1_WINDOW_EN_SHIFT)) & LPCMP_CCR1_WINDOW_EN_MASK)

#define LPCMP_CCR1_SAMPLE_EN_MASK                (0x2U)
#define LPCMP_CCR1_SAMPLE_EN_SHIFT               (1U)
/*! SAMPLE_EN - Sampling Enable
 *  0b0..Disables
 *  0b1..Enables
 */
#define LPCMP_CCR1_SAMPLE_EN(x)                  (((uint32_t)(((uint32_t)(x)) << LPCMP_CCR1_SAMPLE_EN_SHIFT)) & LPCMP_CCR1_SAMPLE_EN_MASK)

#define LPCMP_CCR1_DMA_EN_MASK                   (0x4U)
#define LPCMP_CCR1_DMA_EN_SHIFT                  (2U)
/*! DMA_EN - DMA Enable
 *  0b0..Disables
 *  0b1..Enables
 */
#define LPCMP_CCR1_DMA_EN(x)                     (((uint32_t)(((uint32_t)(x)) << LPCMP_CCR1_DMA_EN_SHIFT)) & LPCMP_CCR1_DMA_EN_MASK)

#define LPCMP_CCR1_COUT_INV_MASK                 (0x8U)
#define LPCMP_CCR1_COUT_INV_SHIFT                (3U)
/*! COUT_INV - Comparator Invert
 *  0b0..Do not invert
 *  0b1..Invert
 */
#define LPCMP_CCR1_COUT_INV(x)                   (((uint32_t)(((uint32_t)(x)) << LPCMP_CCR1_COUT_INV_SHIFT)) & LPCMP_CCR1_COUT_INV_MASK)

#define LPCMP_CCR1_COUT_SEL_MASK                 (0x10U)
#define LPCMP_CCR1_COUT_SEL_SHIFT                (4U)
/*! COUT_SEL - Comparator Output Select
 *  0b0..Use COUT (filtered)
 *  0b1..Use COUTA (unfiltered)
 */
#define LPCMP_CCR1_COUT_SEL(x)                   (((uint32_t)(((uint32_t)(x)) << LPCMP_CCR1_COUT_SEL_SHIFT)) & LPCMP_CCR1_COUT_SEL_MASK)

#define LPCMP_CCR1_COUT_PEN_MASK                 (0x20U)
#define LPCMP_CCR1_COUT_PEN_SHIFT                (5U)
/*! COUT_PEN - Comparator Output Pin Enable
 *  0b0..Not available
 *  0b1..Available
 */
#define LPCMP_CCR1_COUT_PEN(x)                   (((uint32_t)(((uint32_t)(x)) << LPCMP_CCR1_COUT_PEN_SHIFT)) & LPCMP_CCR1_COUT_PEN_MASK)

#define LPCMP_CCR1_COUTA_OWEN_MASK               (0x40U)
#define LPCMP_CCR1_COUTA_OWEN_SHIFT              (6U)
/*! COUTA_OWEN - COUTA_OW Enable
 *  0b0..COUTA holds the last sampled value.
 *  0b1..Enables the COUTA signal value to be defined by COUTA_OW.
 */
#define LPCMP_CCR1_COUTA_OWEN(x)                 (((uint32_t)(((uint32_t)(x)) << LPCMP_CCR1_COUTA_OWEN_SHIFT)) & LPCMP_CCR1_COUTA_OWEN_MASK)

#define LPCMP_CCR1_COUTA_OW_MASK                 (0x80U)
#define LPCMP_CCR1_COUTA_OW_SHIFT                (7U)
/*! COUTA_OW - COUTA Output Level for Closed Window
 *  0b0..COUTA is 0
 *  0b1..COUTA is 1
 */
#define LPCMP_CCR1_COUTA_OW(x)                   (((uint32_t)(((uint32_t)(x)) << LPCMP_CCR1_COUTA_OW_SHIFT)) & LPCMP_CCR1_COUTA_OW_MASK)

#define LPCMP_CCR1_WINDOW_INV_MASK               (0x100U)
#define LPCMP_CCR1_WINDOW_INV_SHIFT              (8U)
/*! WINDOW_INV - WINDOW/SAMPLE Signal Invert
 *  0b0..Do not invert
 *  0b1..Invert
 */
#define LPCMP_CCR1_WINDOW_INV(x)                 (((uint32_t)(((uint32_t)(x)) << LPCMP_CCR1_WINDOW_INV_SHIFT)) & LPCMP_CCR1_WINDOW_INV_MASK)

#define LPCMP_CCR1_WINDOW_CLS_MASK               (0x200U)
#define LPCMP_CCR1_WINDOW_CLS_SHIFT              (9U)
/*! WINDOW_CLS - COUT Event Window Close
 *  0b0..COUT event cannot close the window
 *  0b1..COUT event can close the window
 */
#define LPCMP_CCR1_WINDOW_CLS(x)                 (((uint32_t)(((uint32_t)(x)) << LPCMP_CCR1_WINDOW_CLS_SHIFT)) & LPCMP_CCR1_WINDOW_CLS_MASK)

#define LPCMP_CCR1_EVT_SEL_MASK                  (0xC00U)
#define LPCMP_CCR1_EVT_SEL_SHIFT                 (10U)
/*! EVT_SEL - COUT Event Select
 *  0b00..Rising edge
 *  0b01..Falling edge
 *  0b1x..Both edges
 */
#define LPCMP_CCR1_EVT_SEL(x)                    (((uint32_t)(((uint32_t)(x)) << LPCMP_CCR1_EVT_SEL_SHIFT)) & LPCMP_CCR1_EVT_SEL_MASK)

#define LPCMP_CCR1_FUNC_CLK_SEL_MASK             (0x3000U)
#define LPCMP_CCR1_FUNC_CLK_SEL_SHIFT            (12U)
/*! FUNC_CLK_SEL - Functional Clock Source Select
 *  0b00..Select functional clock source 0
 *  0b01..Select functional clock source 1
 *  0b10..Select functional clock source 2
 *  0b11..Select functional clock source 3
 */
#define LPCMP_CCR1_FUNC_CLK_SEL(x)               (((uint32_t)(((uint32_t)(x)) << LPCMP_CCR1_FUNC_CLK_SEL_SHIFT)) & LPCMP_CCR1_FUNC_CLK_SEL_MASK)

#define LPCMP_CCR1_FILT_CNT_MASK                 (0x70000U)
#define LPCMP_CCR1_FILT_CNT_SHIFT                (16U)
/*! FILT_CNT - Filter Sample Count
 *  0b000..Filter is bypassed: COUT = COUTA
 *  0b001..1 consecutive sample (Comparator output is simply sampled.)
 *  0b010..2 consecutive samples
 *  0b011..3 consecutive samples
 *  0b100..4 consecutive samples
 *  0b101..5 consecutive samples
 *  0b110..6 consecutive samples
 *  0b111..7 consecutive samples
 */
#define LPCMP_CCR1_FILT_CNT(x)                   (((uint32_t)(((uint32_t)(x)) << LPCMP_CCR1_FILT_CNT_SHIFT)) & LPCMP_CCR1_FILT_CNT_MASK)

#define LPCMP_CCR1_FILT_PER_MASK                 (0xFF000000U)
#define LPCMP_CCR1_FILT_PER_SHIFT                (24U)
/*! FILT_PER - Filter Sample Period */
#define LPCMP_CCR1_FILT_PER(x)                   (((uint32_t)(((uint32_t)(x)) << LPCMP_CCR1_FILT_PER_SHIFT)) & LPCMP_CCR1_FILT_PER_MASK)
/*! @} */

/*! @name CCR2 - Comparator Control Register 2 */
/*! @{ */

#define LPCMP_CCR2_CMP_HPMD_MASK                 (0x1U)
#define LPCMP_CCR2_CMP_HPMD_SHIFT                (0U)
/*! CMP_HPMD - CMP High Power Mode Select
 *  0b0..Low power (speed) comparison mode
 *  0b1..High power (speed) comparison mode
 */
#define LPCMP_CCR2_CMP_HPMD(x)                   (((uint32_t)(((uint32_t)(x)) << LPCMP_CCR2_CMP_HPMD_SHIFT)) & LPCMP_CCR2_CMP_HPMD_MASK)

#define LPCMP_CCR2_CMP_NPMD_MASK                 (0x2U)
#define LPCMP_CCR2_CMP_NPMD_SHIFT                (1U)
/*! CMP_NPMD - CMP Nano Power Mode Select
 *  0b0..Disables CMP Nano power mode. CCR2[CMP_HPMD] determines the mode for the comparator.
 *  0b1..Enables CMP Nano power mode.
 */
#define LPCMP_CCR2_CMP_NPMD(x)                   (((uint32_t)(((uint32_t)(x)) << LPCMP_CCR2_CMP_NPMD_SHIFT)) & LPCMP_CCR2_CMP_NPMD_MASK)

#define LPCMP_CCR2_HYSTCTR_MASK                  (0x30U)
#define LPCMP_CCR2_HYSTCTR_SHIFT                 (4U)
/*! HYSTCTR - Comparator Hysteresis Control
 *  0b00..Level 0
 *  0b01..Level 1
 *  0b10..Level 2
 *  0b11..Level 3
 */
#define LPCMP_CCR2_HYSTCTR(x)                    (((uint32_t)(((uint32_t)(x)) << LPCMP_CCR2_HYSTCTR_SHIFT)) & LPCMP_CCR2_HYSTCTR_MASK)

#define LPCMP_CCR2_PSEL_MASK                     (0x70000U)
#define LPCMP_CCR2_PSEL_SHIFT                    (16U)
/*! PSEL - Plus Input MUX Select
 *  0b000..Input 0p
 *  0b001..Input 1p
 *  0b010..Input 2p
 *  0b011..Input 3p
 *  0b100..Input 4p
 *  0b101..Input 5p
 *  0b110..Reserved
 *  0b111..Internal DAC output
 */
#define LPCMP_CCR2_PSEL(x)                       (((uint32_t)(((uint32_t)(x)) << LPCMP_CCR2_PSEL_SHIFT)) & LPCMP_CCR2_PSEL_MASK)

#define LPCMP_CCR2_MSEL_MASK                     (0x700000U)
#define LPCMP_CCR2_MSEL_SHIFT                    (20U)
/*! MSEL - Minus Input MUX Select
 *  0b000..Input 0m
 *  0b001..Input 1m
 *  0b010..Input 2m
 *  0b011..Input 3m
 *  0b100..Input 4m
 *  0b101..Input 5m
 *  0b110..Reserved
 *  0b111..Internal DAC output
 */
#define LPCMP_CCR2_MSEL(x)                       (((uint32_t)(((uint32_t)(x)) << LPCMP_CCR2_MSEL_SHIFT)) & LPCMP_CCR2_MSEL_MASK)
/*! @} */

/*! @name DCR - DAC Control */
/*! @{ */

#define LPCMP_DCR_DAC_EN_MASK                    (0x1U)
#define LPCMP_DCR_DAC_EN_SHIFT                   (0U)
/*! DAC_EN - DAC Enable
 *  0b0..Disables
 *  0b1..Enables
 */
#define LPCMP_DCR_DAC_EN(x)                      (((uint32_t)(((uint32_t)(x)) << LPCMP_DCR_DAC_EN_SHIFT)) & LPCMP_DCR_DAC_EN_MASK)

#define LPCMP_DCR_DAC_HPMD_MASK                  (0x2U)
#define LPCMP_DCR_DAC_HPMD_SHIFT                 (1U)
/*! DAC_HPMD - DAC High Power Mode Select
 *  0b0..Disables
 *  0b1..Enables
 */
#define LPCMP_DCR_DAC_HPMD(x)                    (((uint32_t)(((uint32_t)(x)) << LPCMP_DCR_DAC_HPMD_SHIFT)) & LPCMP_DCR_DAC_HPMD_MASK)

#define LPCMP_DCR_VRSEL_MASK                     (0x100U)
#define LPCMP_DCR_VRSEL_SHIFT                    (8U)
/*! VRSEL - DAC Reference High Voltage Source Select
 *  0b0..vrefh0
 *  0b1..vrefh1
 */
#define LPCMP_DCR_VRSEL(x)                       (((uint32_t)(((uint32_t)(x)) << LPCMP_DCR_VRSEL_SHIFT)) & LPCMP_DCR_VRSEL_MASK)

#define LPCMP_DCR_DAC_DATA_MASK                  (0xFF0000U)
#define LPCMP_DCR_DAC_DATA_SHIFT                 (16U)
/*! DAC_DATA - DAC Output Voltage Select */
#define LPCMP_DCR_DAC_DATA(x)                    (((uint32_t)(((uint32_t)(x)) << LPCMP_DCR_DAC_DATA_SHIFT)) & LPCMP_DCR_DAC_DATA_MASK)
/*! @} */

/*! @name IER - Interrupt Enable */
/*! @{ */

#define LPCMP_IER_CFR_IE_MASK                    (0x1U)
#define LPCMP_IER_CFR_IE_SHIFT                   (0U)
/*! CFR_IE - Comparator Flag Rising Interrupt Enable
 *  0b0..Disables the comparator flag rising interrupt.
 *  0b1..Enables the comparator flag rising interrupt when CFR is set.
 */
#define LPCMP_IER_CFR_IE(x)                      (((uint32_t)(((uint32_t)(x)) << LPCMP_IER_CFR_IE_SHIFT)) & LPCMP_IER_CFR_IE_MASK)

#define LPCMP_IER_CFF_IE_MASK                    (0x2U)
#define LPCMP_IER_CFF_IE_SHIFT                   (1U)
/*! CFF_IE - Comparator Flag Falling Interrupt Enable
 *  0b0..Disables the comparator flag falling interrupt.
 *  0b1..Enables the comparator flag falling interrupt when CFF is set.
 */
#define LPCMP_IER_CFF_IE(x)                      (((uint32_t)(((uint32_t)(x)) << LPCMP_IER_CFF_IE_SHIFT)) & LPCMP_IER_CFF_IE_MASK)

#define LPCMP_IER_RRF_IE_MASK                    (0x4U)
#define LPCMP_IER_RRF_IE_SHIFT                   (2U)
/*! RRF_IE - Round-Robin Flag Interrupt Enable
 *  0b0..Disables the round-robin flag interrupt.
 *  0b1..Enables the round-robin flag interrupt when the comparison result changes for a given channel.
 */
#define LPCMP_IER_RRF_IE(x)                      (((uint32_t)(((uint32_t)(x)) << LPCMP_IER_RRF_IE_SHIFT)) & LPCMP_IER_RRF_IE_MASK)
/*! @} */

/*! @name CSR - Comparator Status */
/*! @{ */

#define LPCMP_CSR_CFR_MASK                       (0x1U)
#define LPCMP_CSR_CFR_SHIFT                      (0U)
/*! CFR - Analog Comparator Flag Rising
 *  0b0..Not detected
 *  0b1..Detected
 */
#define LPCMP_CSR_CFR(x)                         (((uint32_t)(((uint32_t)(x)) << LPCMP_CSR_CFR_SHIFT)) & LPCMP_CSR_CFR_MASK)

#define LPCMP_CSR_CFF_MASK                       (0x2U)
#define LPCMP_CSR_CFF_SHIFT                      (1U)
/*! CFF - Analog Comparator Flag Falling
 *  0b0..Not detected
 *  0b1..Detected
 */
#define LPCMP_CSR_CFF(x)                         (((uint32_t)(((uint32_t)(x)) << LPCMP_CSR_CFF_SHIFT)) & LPCMP_CSR_CFF_MASK)

#define LPCMP_CSR_RRF_MASK                       (0x4U)
#define LPCMP_CSR_RRF_SHIFT                      (2U)
/*! RRF - Round-Robin Flag
 *  0b0..Not detected
 *  0b1..Detected
 */
#define LPCMP_CSR_RRF(x)                         (((uint32_t)(((uint32_t)(x)) << LPCMP_CSR_RRF_SHIFT)) & LPCMP_CSR_RRF_MASK)

#define LPCMP_CSR_COUT_MASK                      (0x100U)
#define LPCMP_CSR_COUT_SHIFT                     (8U)
/*! COUT - Analog Comparator Output */
#define LPCMP_CSR_COUT(x)                        (((uint32_t)(((uint32_t)(x)) << LPCMP_CSR_COUT_SHIFT)) & LPCMP_CSR_COUT_MASK)
/*! @} */

/*! @name RRCR0 - Round Robin Control Register 0 */
/*! @{ */

#define LPCMP_RRCR0_RR_EN_MASK                   (0x1U)
#define LPCMP_RRCR0_RR_EN_SHIFT                  (0U)
/*! RR_EN - Round-Robin Enable
 *  0b0..Disables
 *  0b1..Enables
 */
#define LPCMP_RRCR0_RR_EN(x)                     (((uint32_t)(((uint32_t)(x)) << LPCMP_RRCR0_RR_EN_SHIFT)) & LPCMP_RRCR0_RR_EN_MASK)

#define LPCMP_RRCR0_RR_TRG_SEL_MASK              (0x2U)
#define LPCMP_RRCR0_RR_TRG_SEL_SHIFT             (1U)
/*! RR_TRG_SEL - Round-Robin Trigger Select
 *  0b0..External trigger
 *  0b1..Internal trigger
 */
#define LPCMP_RRCR0_RR_TRG_SEL(x)                (((uint32_t)(((uint32_t)(x)) << LPCMP_RRCR0_RR_TRG_SEL_SHIFT)) & LPCMP_RRCR0_RR_TRG_SEL_MASK)

#define LPCMP_RRCR0_RR_EXTTRG_SEL_MASK           (0x3CU)
#define LPCMP_RRCR0_RR_EXTTRG_SEL_SHIFT          (2U)
/*! RR_EXTTRG_SEL - Select the External Trigger Source
 *  0b0000..Select external trigger source 0
 *  0b0001..Select external trigger source 1
 *  0b0010..Select external trigger source 2
 *  0b0011..Select external trigger source 3
 *  0b0100..Select external trigger source 4
 *  0b0101..Select external trigger source 5
 *  0b0110..Select external trigger source 6
 *  0b0111..Select external trigger source 7
 *  0b1000..Select external trigger source 8
 *  0b1001..Select external trigger source 9
 *  0b1010..Select external trigger source 10
 *  0b1011..Select external trigger source 11
 *  0b1100..Select external trigger source 12
 *  0b1101..Select external trigger source 13
 *  0b1110..Select external trigger source 14
 *  0b1111..Select external trigger source 15
 */
#define LPCMP_RRCR0_RR_EXTTRG_SEL(x)             (((uint32_t)(((uint32_t)(x)) << LPCMP_RRCR0_RR_EXTTRG_SEL_SHIFT)) & LPCMP_RRCR0_RR_EXTTRG_SEL_MASK)

#define LPCMP_RRCR0_RR_NSAM_MASK                 (0x300U)
#define LPCMP_RRCR0_RR_NSAM_SHIFT                (8U)
/*! RR_NSAM - Number of Sample Clocks
 *  0b00..0 clock
 *  0b01..1 clock
 *  0b10..2 clocks
 *  0b11..3 clocks
 */
#define LPCMP_RRCR0_RR_NSAM(x)                   (((uint32_t)(((uint32_t)(x)) << LPCMP_RRCR0_RR_NSAM_SHIFT)) & LPCMP_RRCR0_RR_NSAM_MASK)

#define LPCMP_RRCR0_RR_CLK_SEL_MASK              (0x3000U)
#define LPCMP_RRCR0_RR_CLK_SEL_SHIFT             (12U)
/*! RR_CLK_SEL - Round Robin Clock Source Select
 *  0b00..Select Round Robin clock Source 0
 *  0b01..Select Round Robin clock Source 1
 *  0b10..Select Round Robin clock Source 2
 *  0b11..Select Round Robin clock Source 3
 */
#define LPCMP_RRCR0_RR_CLK_SEL(x)                (((uint32_t)(((uint32_t)(x)) << LPCMP_RRCR0_RR_CLK_SEL_SHIFT)) & LPCMP_RRCR0_RR_CLK_SEL_MASK)

#define LPCMP_RRCR0_RR_INITMOD_MASK              (0x3F0000U)
#define LPCMP_RRCR0_RR_INITMOD_SHIFT             (16U)
/*! RR_INITMOD - Initialization Delay Modulus
 *  0b000000..63 cycles (same as 111111b)
 *  0b000001-0b111111..1 to 63 cycles
 */
#define LPCMP_RRCR0_RR_INITMOD(x)                (((uint32_t)(((uint32_t)(x)) << LPCMP_RRCR0_RR_INITMOD_SHIFT)) & LPCMP_RRCR0_RR_INITMOD_MASK)

#define LPCMP_RRCR0_RR_SAMPLE_CNT_MASK           (0xF000000U)
#define LPCMP_RRCR0_RR_SAMPLE_CNT_SHIFT          (24U)
/*! RR_SAMPLE_CNT - Number of Sample for One Channel
 *  0b0000..1 samples
 *  0b0001..2 samples
 *  0b0010..3 samples
 *  0b0011..4 samples
 *  0b0100..5 samples
 *  0b0101..6 samples
 *  0b0110..7 samples
 *  0b0111..8 samples
 *  0b1000..9 samples
 *  0b1001..10 samples
 *  0b1010..11 samples
 *  0b1011..12 samples
 *  0b1100..13 samples
 *  0b1101..14 samples
 *  0b1110..15 samples
 *  0b1111..16 samples
 */
#define LPCMP_RRCR0_RR_SAMPLE_CNT(x)             (((uint32_t)(((uint32_t)(x)) << LPCMP_RRCR0_RR_SAMPLE_CNT_SHIFT)) & LPCMP_RRCR0_RR_SAMPLE_CNT_MASK)

#define LPCMP_RRCR0_RR_SAMPLE_THRESHOLD_MASK     (0xF0000000U)
#define LPCMP_RRCR0_RR_SAMPLE_THRESHOLD_SHIFT    (28U)
/*! RR_SAMPLE_THRESHOLD - Sample Time Threshold
 *  0b0000..At least 1 sampled "1", the final result is "1"
 *  0b0001..At least 2 sampled "1", the final result is "1"
 *  0b0010..At least 3 sampled "1", the final result is "1"
 *  0b0011..At least 4 sampled "1", the final result is "1"
 *  0b0100..At least 5 sampled "1", the final result is "1"
 *  0b0101..At least 6 sampled "1", the final result is "1"
 *  0b0110..At least 7 sampled "1", the final result is "1"
 *  0b0111..At least 8 sampled "1", the final result is "1"
 *  0b1000..At least 9 sampled "1", the final result is "1"
 *  0b1001..At least 10 sampled "1", the final result is "1"
 *  0b1010..At least 11 sampled "1", the final result is "1"
 *  0b1011..At least 12 sampled "1", the final result is "1"
 *  0b1100..At least 13 sampled "1", the final result is "1"
 *  0b1101..At least 14 sampled "1", the final result is "1"
 *  0b1110..At least 15 sampled "1", the final result is "1"
 *  0b1111..At least 16 sampled "1", the final result is "1"
 */
#define LPCMP_RRCR0_RR_SAMPLE_THRESHOLD(x)       (((uint32_t)(((uint32_t)(x)) << LPCMP_RRCR0_RR_SAMPLE_THRESHOLD_SHIFT)) & LPCMP_RRCR0_RR_SAMPLE_THRESHOLD_MASK)
/*! @} */

/*! @name RRCR1 - Round Robin Control Register 1 */
/*! @{ */

#define LPCMP_RRCR1_RR_CH0EN_MASK                (0x1U)
#define LPCMP_RRCR1_RR_CH0EN_SHIFT               (0U)
/*! RR_CH0EN - Channel 0 Input Enable in Trigger Mode
 *  0b0..Disables
 *  0b1..Enables
 */
#define LPCMP_RRCR1_RR_CH0EN(x)                  (((uint32_t)(((uint32_t)(x)) << LPCMP_RRCR1_RR_CH0EN_SHIFT)) & LPCMP_RRCR1_RR_CH0EN_MASK)

#define LPCMP_RRCR1_RR_CH1EN_MASK                (0x2U)
#define LPCMP_RRCR1_RR_CH1EN_SHIFT               (1U)
/*! RR_CH1EN - Channel 1 Input Enable in Trigger Mode
 *  0b0..Disables
 *  0b1..Enables
 */
#define LPCMP_RRCR1_RR_CH1EN(x)                  (((uint32_t)(((uint32_t)(x)) << LPCMP_RRCR1_RR_CH1EN_SHIFT)) & LPCMP_RRCR1_RR_CH1EN_MASK)

#define LPCMP_RRCR1_RR_CH2EN_MASK                (0x4U)
#define LPCMP_RRCR1_RR_CH2EN_SHIFT               (2U)
/*! RR_CH2EN - Channel 2 Input Enable in Trigger Mode
 *  0b0..Disables
 *  0b1..Enables
 */
#define LPCMP_RRCR1_RR_CH2EN(x)                  (((uint32_t)(((uint32_t)(x)) << LPCMP_RRCR1_RR_CH2EN_SHIFT)) & LPCMP_RRCR1_RR_CH2EN_MASK)

#define LPCMP_RRCR1_RR_CH3EN_MASK                (0x8U)
#define LPCMP_RRCR1_RR_CH3EN_SHIFT               (3U)
/*! RR_CH3EN - Channel 3 Input Enable in Trigger Mode
 *  0b0..Disables
 *  0b1..Enables
 */
#define LPCMP_RRCR1_RR_CH3EN(x)                  (((uint32_t)(((uint32_t)(x)) << LPCMP_RRCR1_RR_CH3EN_SHIFT)) & LPCMP_RRCR1_RR_CH3EN_MASK)

#define LPCMP_RRCR1_RR_CH4EN_MASK                (0x10U)
#define LPCMP_RRCR1_RR_CH4EN_SHIFT               (4U)
/*! RR_CH4EN - Channel 4 Input Enable in Trigger Mode
 *  0b0..Disables
 *  0b1..Enables
 */
#define LPCMP_RRCR1_RR_CH4EN(x)                  (((uint32_t)(((uint32_t)(x)) << LPCMP_RRCR1_RR_CH4EN_SHIFT)) & LPCMP_RRCR1_RR_CH4EN_MASK)

#define LPCMP_RRCR1_RR_CH5EN_MASK                (0x20U)
#define LPCMP_RRCR1_RR_CH5EN_SHIFT               (5U)
/*! RR_CH5EN - Channel 5 Input Enable in Trigger Mode
 *  0b0..Disables
 *  0b1..Enables
 */
#define LPCMP_RRCR1_RR_CH5EN(x)                  (((uint32_t)(((uint32_t)(x)) << LPCMP_RRCR1_RR_CH5EN_SHIFT)) & LPCMP_RRCR1_RR_CH5EN_MASK)

#define LPCMP_RRCR1_RR_CH6EN_MASK                (0x40U)
#define LPCMP_RRCR1_RR_CH6EN_SHIFT               (6U)
/*! RR_CH6EN - Channel 6 Input Enable in Trigger Mode
 *  0b0..Disables
 *  0b1..Enables
 */
#define LPCMP_RRCR1_RR_CH6EN(x)                  (((uint32_t)(((uint32_t)(x)) << LPCMP_RRCR1_RR_CH6EN_SHIFT)) & LPCMP_RRCR1_RR_CH6EN_MASK)

#define LPCMP_RRCR1_RR_CH7EN_MASK                (0x80U)
#define LPCMP_RRCR1_RR_CH7EN_SHIFT               (7U)
/*! RR_CH7EN - Channel 7 Input Enable in Trigger Mode
 *  0b0..Disables
 *  0b1..Enables
 */
#define LPCMP_RRCR1_RR_CH7EN(x)                  (((uint32_t)(((uint32_t)(x)) << LPCMP_RRCR1_RR_CH7EN_SHIFT)) & LPCMP_RRCR1_RR_CH7EN_MASK)

#define LPCMP_RRCR1_FIXP_MASK                    (0x10000U)
#define LPCMP_RRCR1_FIXP_SHIFT                   (16U)
/*! FIXP - Fixed Port
 *  0b0..Fix the plus port. Sweep only the inputs to the minus port.
 *  0b1..Fix the minus port. Sweep only the inputs to the plus port.
 */
#define LPCMP_RRCR1_FIXP(x)                      (((uint32_t)(((uint32_t)(x)) << LPCMP_RRCR1_FIXP_SHIFT)) & LPCMP_RRCR1_FIXP_MASK)

#define LPCMP_RRCR1_FIXCH_MASK                   (0x700000U)
#define LPCMP_RRCR1_FIXCH_SHIFT                  (20U)
/*! FIXCH - Fixed Channel Select
 *  0b000..Channel 0
 *  0b001..Channel 1
 *  0b010..Channel 2
 *  0b011..Channel 3
 *  0b100..Channel 4
 *  0b101..Channel 5
 *  0b110..Channel 6
 *  0b111..Channel 7
 */
#define LPCMP_RRCR1_FIXCH(x)                     (((uint32_t)(((uint32_t)(x)) << LPCMP_RRCR1_FIXCH_SHIFT)) & LPCMP_RRCR1_FIXCH_MASK)
/*! @} */

/*! @name RRCSR - Round Robin Control and Status */
/*! @{ */

#define LPCMP_RRCSR_RR_CH0OUT_MASK               (0x1U)
#define LPCMP_RRCSR_RR_CH0OUT_SHIFT              (0U)
/*! RR_CH0OUT - Comparison Result for Channel 0 */
#define LPCMP_RRCSR_RR_CH0OUT(x)                 (((uint32_t)(((uint32_t)(x)) << LPCMP_RRCSR_RR_CH0OUT_SHIFT)) & LPCMP_RRCSR_RR_CH0OUT_MASK)

#define LPCMP_RRCSR_RR_CH1OUT_MASK               (0x2U)
#define LPCMP_RRCSR_RR_CH1OUT_SHIFT              (1U)
/*! RR_CH1OUT - Comparison Result for Channel 1 */
#define LPCMP_RRCSR_RR_CH1OUT(x)                 (((uint32_t)(((uint32_t)(x)) << LPCMP_RRCSR_RR_CH1OUT_SHIFT)) & LPCMP_RRCSR_RR_CH1OUT_MASK)

#define LPCMP_RRCSR_RR_CH2OUT_MASK               (0x4U)
#define LPCMP_RRCSR_RR_CH2OUT_SHIFT              (2U)
/*! RR_CH2OUT - Comparison Result for Channel 2 */
#define LPCMP_RRCSR_RR_CH2OUT(x)                 (((uint32_t)(((uint32_t)(x)) << LPCMP_RRCSR_RR_CH2OUT_SHIFT)) & LPCMP_RRCSR_RR_CH2OUT_MASK)

#define LPCMP_RRCSR_RR_CH3OUT_MASK               (0x8U)
#define LPCMP_RRCSR_RR_CH3OUT_SHIFT              (3U)
/*! RR_CH3OUT - Comparison Result for Channel 3 */
#define LPCMP_RRCSR_RR_CH3OUT(x)                 (((uint32_t)(((uint32_t)(x)) << LPCMP_RRCSR_RR_CH3OUT_SHIFT)) & LPCMP_RRCSR_RR_CH3OUT_MASK)

#define LPCMP_RRCSR_RR_CH4OUT_MASK               (0x10U)
#define LPCMP_RRCSR_RR_CH4OUT_SHIFT              (4U)
/*! RR_CH4OUT - Comparison Result for Channel 4 */
#define LPCMP_RRCSR_RR_CH4OUT(x)                 (((uint32_t)(((uint32_t)(x)) << LPCMP_RRCSR_RR_CH4OUT_SHIFT)) & LPCMP_RRCSR_RR_CH4OUT_MASK)

#define LPCMP_RRCSR_RR_CH5OUT_MASK               (0x20U)
#define LPCMP_RRCSR_RR_CH5OUT_SHIFT              (5U)
/*! RR_CH5OUT - Comparison Result for Channel 5 */
#define LPCMP_RRCSR_RR_CH5OUT(x)                 (((uint32_t)(((uint32_t)(x)) << LPCMP_RRCSR_RR_CH5OUT_SHIFT)) & LPCMP_RRCSR_RR_CH5OUT_MASK)

#define LPCMP_RRCSR_RR_CH6OUT_MASK               (0x40U)
#define LPCMP_RRCSR_RR_CH6OUT_SHIFT              (6U)
/*! RR_CH6OUT - Comparison Result for Channel 6 */
#define LPCMP_RRCSR_RR_CH6OUT(x)                 (((uint32_t)(((uint32_t)(x)) << LPCMP_RRCSR_RR_CH6OUT_SHIFT)) & LPCMP_RRCSR_RR_CH6OUT_MASK)

#define LPCMP_RRCSR_RR_CH7OUT_MASK               (0x80U)
#define LPCMP_RRCSR_RR_CH7OUT_SHIFT              (7U)
/*! RR_CH7OUT - Comparison Result for Channel 7 */
#define LPCMP_RRCSR_RR_CH7OUT(x)                 (((uint32_t)(((uint32_t)(x)) << LPCMP_RRCSR_RR_CH7OUT_SHIFT)) & LPCMP_RRCSR_RR_CH7OUT_MASK)
/*! @} */

/*! @name RRSR - Round Robin Status */
/*! @{ */

#define LPCMP_RRSR_RR_CH0F_MASK                  (0x1U)
#define LPCMP_RRSR_RR_CH0F_SHIFT                 (0U)
/*! RR_CH0F - Channel 0 Input Changed Flag
 *  0b0..Not different
 *  0b1..Different
 */
#define LPCMP_RRSR_RR_CH0F(x)                    (((uint32_t)(((uint32_t)(x)) << LPCMP_RRSR_RR_CH0F_SHIFT)) & LPCMP_RRSR_RR_CH0F_MASK)

#define LPCMP_RRSR_RR_CH1F_MASK                  (0x2U)
#define LPCMP_RRSR_RR_CH1F_SHIFT                 (1U)
/*! RR_CH1F - Channel 1 Input Changed Flag
 *  0b0..Not different
 *  0b1..Different
 */
#define LPCMP_RRSR_RR_CH1F(x)                    (((uint32_t)(((uint32_t)(x)) << LPCMP_RRSR_RR_CH1F_SHIFT)) & LPCMP_RRSR_RR_CH1F_MASK)

#define LPCMP_RRSR_RR_CH2F_MASK                  (0x4U)
#define LPCMP_RRSR_RR_CH2F_SHIFT                 (2U)
/*! RR_CH2F - Channel 2 Input Changed Flag
 *  0b0..Not different
 *  0b1..Different
 */
#define LPCMP_RRSR_RR_CH2F(x)                    (((uint32_t)(((uint32_t)(x)) << LPCMP_RRSR_RR_CH2F_SHIFT)) & LPCMP_RRSR_RR_CH2F_MASK)

#define LPCMP_RRSR_RR_CH3F_MASK                  (0x8U)
#define LPCMP_RRSR_RR_CH3F_SHIFT                 (3U)
/*! RR_CH3F - Channel 3 Input Changed Flag
 *  0b0..Not different
 *  0b1..Different
 */
#define LPCMP_RRSR_RR_CH3F(x)                    (((uint32_t)(((uint32_t)(x)) << LPCMP_RRSR_RR_CH3F_SHIFT)) & LPCMP_RRSR_RR_CH3F_MASK)

#define LPCMP_RRSR_RR_CH4F_MASK                  (0x10U)
#define LPCMP_RRSR_RR_CH4F_SHIFT                 (4U)
/*! RR_CH4F - Channel 4 Input Changed Flag
 *  0b0..Not different
 *  0b1..Different
 */
#define LPCMP_RRSR_RR_CH4F(x)                    (((uint32_t)(((uint32_t)(x)) << LPCMP_RRSR_RR_CH4F_SHIFT)) & LPCMP_RRSR_RR_CH4F_MASK)

#define LPCMP_RRSR_RR_CH5F_MASK                  (0x20U)
#define LPCMP_RRSR_RR_CH5F_SHIFT                 (5U)
/*! RR_CH5F - Channel 5 Input Changed Flag
 *  0b0..Not different
 *  0b1..Different
 */
#define LPCMP_RRSR_RR_CH5F(x)                    (((uint32_t)(((uint32_t)(x)) << LPCMP_RRSR_RR_CH5F_SHIFT)) & LPCMP_RRSR_RR_CH5F_MASK)

#define LPCMP_RRSR_RR_CH6F_MASK                  (0x40U)
#define LPCMP_RRSR_RR_CH6F_SHIFT                 (6U)
/*! RR_CH6F - Channel 6 Input Changed Flag
 *  0b0..Not different
 *  0b1..Different
 */
#define LPCMP_RRSR_RR_CH6F(x)                    (((uint32_t)(((uint32_t)(x)) << LPCMP_RRSR_RR_CH6F_SHIFT)) & LPCMP_RRSR_RR_CH6F_MASK)

#define LPCMP_RRSR_RR_CH7F_MASK                  (0x80U)
#define LPCMP_RRSR_RR_CH7F_SHIFT                 (7U)
/*! RR_CH7F - Channel 7 Input Changed Flag
 *  0b0..Not different
 *  0b1..Different
 */
#define LPCMP_RRSR_RR_CH7F(x)                    (((uint32_t)(((uint32_t)(x)) << LPCMP_RRSR_RR_CH7F_SHIFT)) & LPCMP_RRSR_RR_CH7F_MASK)
/*! @} */

/*! @name RRCR2 - Round Robin Control Register 2 */
/*! @{ */

#define LPCMP_RRCR2_RR_TIMER_RELOAD_MASK         (0xFFFFFFFU)
#define LPCMP_RRCR2_RR_TIMER_RELOAD_SHIFT        (0U)
/*! RR_TIMER_RELOAD - Number of Sample Clocks */
#define LPCMP_RRCR2_RR_TIMER_RELOAD(x)           (((uint32_t)(((uint32_t)(x)) << LPCMP_RRCR2_RR_TIMER_RELOAD_SHIFT)) & LPCMP_RRCR2_RR_TIMER_RELOAD_MASK)

#define LPCMP_RRCR2_RR_TIMER_EN_MASK             (0x80000000U)
#define LPCMP_RRCR2_RR_TIMER_EN_SHIFT            (31U)
/*! RR_TIMER_EN - Round-Robin Internal Timer Enable
 *  0b0..Disables
 *  0b1..Enables
 */
#define LPCMP_RRCR2_RR_TIMER_EN(x)               (((uint32_t)(((uint32_t)(x)) << LPCMP_RRCR2_RR_TIMER_EN_SHIFT)) & LPCMP_RRCR2_RR_TIMER_EN_MASK)
/*! @} */


/*!
 * @}
 */ /* end of group LPCMP_Register_Masks */


/*!
 * @}
 */ /* end of group LPCMP_Peripheral_Access_Layer */


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


#endif  /* PERI_LPCMP_H_ */

