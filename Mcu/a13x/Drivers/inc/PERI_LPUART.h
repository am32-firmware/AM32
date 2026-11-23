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
**         CMSIS Peripheral Access Layer for LPUART
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
 * @file PERI_LPUART.h
 * @version 2.0
 * @date 2024-10-29
 * @brief CMSIS Peripheral Access Layer for LPUART
 *
 * CMSIS Peripheral Access Layer for LPUART
 */

#if !defined(PERI_LPUART_H_)
#define PERI_LPUART_H_                           /**< Symbol preventing repeated inclusion */

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
   -- LPUART Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LPUART_Peripheral_Access_Layer LPUART Peripheral Access Layer
 * @{
 */

/** LPUART - Register Layout Typedef */
typedef struct {
  __I  uint32_t VERID;                             /**< Version ID, offset: 0x0 */
  __I  uint32_t PARAM;                             /**< Parameter, offset: 0x4 */
  __IO uint32_t GLOBAL;                            /**< Global, offset: 0x8 */
  __IO uint32_t PINCFG;                            /**< Pin Configuration, offset: 0xC */
  __IO uint32_t BAUD;                              /**< Baud Rate, offset: 0x10 */
  __IO uint32_t STAT;                              /**< Status, offset: 0x14 */
  __IO uint32_t CTRL;                              /**< Control, offset: 0x18 */
  __IO uint32_t DATA;                              /**< Data, offset: 0x1C */
  __IO uint32_t MATCH;                             /**< Match Address, offset: 0x20 */
  __IO uint32_t MODIR;                             /**< MODEM IrDA, offset: 0x24 */
  __IO uint32_t FIFO;                              /**< FIFO, offset: 0x28 */
  __IO uint32_t WATER;                             /**< Watermark, offset: 0x2C */
  __I  uint32_t DATARO;                            /**< Data Read-Only, offset: 0x30 */
} LPUART_Type;

/* ----------------------------------------------------------------------------
   -- LPUART Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LPUART_Register_Masks LPUART Register Masks
 * @{
 */

/*! @name VERID - Version ID */
/*! @{ */

#define LPUART_VERID_FEATURE_MASK                (0xFFFFU)
#define LPUART_VERID_FEATURE_SHIFT               (0U)
/*! FEATURE - Feature Identification Number
 *  0b0000000000000001..Standard feature set
 *  0b0000000000000011..Standard feature set with MODEM and IrDA support
 */
#define LPUART_VERID_FEATURE(x)                  (((uint32_t)(((uint32_t)(x)) << LPUART_VERID_FEATURE_SHIFT)) & LPUART_VERID_FEATURE_MASK)

#define LPUART_VERID_MINOR_MASK                  (0xFF0000U)
#define LPUART_VERID_MINOR_SHIFT                 (16U)
/*! MINOR - Minor Version Number */
#define LPUART_VERID_MINOR(x)                    (((uint32_t)(((uint32_t)(x)) << LPUART_VERID_MINOR_SHIFT)) & LPUART_VERID_MINOR_MASK)

#define LPUART_VERID_MAJOR_MASK                  (0xFF000000U)
#define LPUART_VERID_MAJOR_SHIFT                 (24U)
/*! MAJOR - Major Version Number */
#define LPUART_VERID_MAJOR(x)                    (((uint32_t)(((uint32_t)(x)) << LPUART_VERID_MAJOR_SHIFT)) & LPUART_VERID_MAJOR_MASK)
/*! @} */

/*! @name PARAM - Parameter */
/*! @{ */

#define LPUART_PARAM_TXFIFO_MASK                 (0xFFU)
#define LPUART_PARAM_TXFIFO_SHIFT                (0U)
/*! TXFIFO - Transmit FIFO Size */
#define LPUART_PARAM_TXFIFO(x)                   (((uint32_t)(((uint32_t)(x)) << LPUART_PARAM_TXFIFO_SHIFT)) & LPUART_PARAM_TXFIFO_MASK)

#define LPUART_PARAM_RXFIFO_MASK                 (0xFF00U)
#define LPUART_PARAM_RXFIFO_SHIFT                (8U)
/*! RXFIFO - Receive FIFO Size */
#define LPUART_PARAM_RXFIFO(x)                   (((uint32_t)(((uint32_t)(x)) << LPUART_PARAM_RXFIFO_SHIFT)) & LPUART_PARAM_RXFIFO_MASK)
/*! @} */

/*! @name GLOBAL - Global */
/*! @{ */

#define LPUART_GLOBAL_RST_MASK                   (0x2U)
#define LPUART_GLOBAL_RST_SHIFT                  (1U)
/*! RST - Software Reset
 *  0b0..Not reset
 *  0b1..Reset
 */
#define LPUART_GLOBAL_RST(x)                     (((uint32_t)(((uint32_t)(x)) << LPUART_GLOBAL_RST_SHIFT)) & LPUART_GLOBAL_RST_MASK)
/*! @} */

/*! @name PINCFG - Pin Configuration */
/*! @{ */

#define LPUART_PINCFG_TRGSEL_MASK                (0x3U)
#define LPUART_PINCFG_TRGSEL_SHIFT               (0U)
/*! TRGSEL - Trigger Select
 *  0b00..Input trigger disabled
 *  0b01..Input trigger used instead of the RXD pin input
 *  0b10..Input trigger used instead of the CTS_B pin input
 *  0b11..Input trigger used to modulate the TXD pin output, which (after TXINV configuration) is internally ANDed with the input trigger
 */
#define LPUART_PINCFG_TRGSEL(x)                  (((uint32_t)(((uint32_t)(x)) << LPUART_PINCFG_TRGSEL_SHIFT)) & LPUART_PINCFG_TRGSEL_MASK)
/*! @} */

/*! @name BAUD - Baud Rate */
/*! @{ */

#define LPUART_BAUD_SBR_MASK                     (0x1FFFU)
#define LPUART_BAUD_SBR_SHIFT                    (0U)
/*! SBR - Baud Rate Modulo Divisor */
#define LPUART_BAUD_SBR(x)                       (((uint32_t)(((uint32_t)(x)) << LPUART_BAUD_SBR_SHIFT)) & LPUART_BAUD_SBR_MASK)

#define LPUART_BAUD_SBNS_MASK                    (0x2000U)
#define LPUART_BAUD_SBNS_SHIFT                   (13U)
/*! SBNS - Stop Bit Number Select
 *  0b0..One stop bit
 *  0b1..Two stop bits
 */
#define LPUART_BAUD_SBNS(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_BAUD_SBNS_SHIFT)) & LPUART_BAUD_SBNS_MASK)

#define LPUART_BAUD_RXEDGIE_MASK                 (0x4000U)
#define LPUART_BAUD_RXEDGIE_SHIFT                (14U)
/*! RXEDGIE - RX Input Active Edge Interrupt Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPUART_BAUD_RXEDGIE(x)                   (((uint32_t)(((uint32_t)(x)) << LPUART_BAUD_RXEDGIE_SHIFT)) & LPUART_BAUD_RXEDGIE_MASK)

#define LPUART_BAUD_LBKDIE_MASK                  (0x8000U)
#define LPUART_BAUD_LBKDIE_SHIFT                 (15U)
/*! LBKDIE - LIN Break Detect Interrupt Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPUART_BAUD_LBKDIE(x)                    (((uint32_t)(((uint32_t)(x)) << LPUART_BAUD_LBKDIE_SHIFT)) & LPUART_BAUD_LBKDIE_MASK)

#define LPUART_BAUD_RESYNCDIS_MASK               (0x10000U)
#define LPUART_BAUD_RESYNCDIS_SHIFT              (16U)
/*! RESYNCDIS - Resynchronization Disable
 *  0b0..Enable
 *  0b1..Disable
 */
#define LPUART_BAUD_RESYNCDIS(x)                 (((uint32_t)(((uint32_t)(x)) << LPUART_BAUD_RESYNCDIS_SHIFT)) & LPUART_BAUD_RESYNCDIS_MASK)

#define LPUART_BAUD_BOTHEDGE_MASK                (0x20000U)
#define LPUART_BAUD_BOTHEDGE_SHIFT               (17U)
/*! BOTHEDGE - Both Edge Sampling
 *  0b0..Rising edge
 *  0b1..Both rising and falling edges
 */
#define LPUART_BAUD_BOTHEDGE(x)                  (((uint32_t)(((uint32_t)(x)) << LPUART_BAUD_BOTHEDGE_SHIFT)) & LPUART_BAUD_BOTHEDGE_MASK)

#define LPUART_BAUD_MATCFG_MASK                  (0xC0000U)
#define LPUART_BAUD_MATCFG_SHIFT                 (18U)
/*! MATCFG - Match Configuration
 *  0b00..Address match wake-up
 *  0b01..Idle match wake-up
 *  0b10..Match on and match off
 *  0b11..Enables RWU on data match and match on or off for the transmitter CTS input
 */
#define LPUART_BAUD_MATCFG(x)                    (((uint32_t)(((uint32_t)(x)) << LPUART_BAUD_MATCFG_SHIFT)) & LPUART_BAUD_MATCFG_MASK)

#define LPUART_BAUD_RIDMAE_MASK                  (0x100000U)
#define LPUART_BAUD_RIDMAE_SHIFT                 (20U)
/*! RIDMAE - Receiver Idle DMA Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPUART_BAUD_RIDMAE(x)                    (((uint32_t)(((uint32_t)(x)) << LPUART_BAUD_RIDMAE_SHIFT)) & LPUART_BAUD_RIDMAE_MASK)

#define LPUART_BAUD_RDMAE_MASK                   (0x200000U)
#define LPUART_BAUD_RDMAE_SHIFT                  (21U)
/*! RDMAE - Receiver Full DMA Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPUART_BAUD_RDMAE(x)                     (((uint32_t)(((uint32_t)(x)) << LPUART_BAUD_RDMAE_SHIFT)) & LPUART_BAUD_RDMAE_MASK)

#define LPUART_BAUD_TDMAE_MASK                   (0x800000U)
#define LPUART_BAUD_TDMAE_SHIFT                  (23U)
/*! TDMAE - Transmitter DMA Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPUART_BAUD_TDMAE(x)                     (((uint32_t)(((uint32_t)(x)) << LPUART_BAUD_TDMAE_SHIFT)) & LPUART_BAUD_TDMAE_MASK)

#define LPUART_BAUD_OSR_MASK                     (0x1F000000U)
#define LPUART_BAUD_OSR_SHIFT                    (24U)
/*! OSR - Oversampling Ratio
 *  0b00000..Results in an OSR of 16
 *  0b00001..Reserved
 *  0b00010..Reserved
 *  0b00011..Results in an OSR of 4 (requires BAUD[BOTHEDGE] to be 1)
 *  0b00100..Results in an OSR of 5 (requires BAUD[BOTHEDGE] to be 1)
 *  0b00101..Results in an OSR of 6 (requires BAUD[BOTHEDGE] to be 1)
 *  0b00110..Results in an OSR of 7 (requires BAUD[BOTHEDGE] to be 1)
 *  0b00111..Results in an OSR of 8
 *  0b01000..Results in an OSR of 9
 *  0b01001..Results in an OSR of 10
 *  0b01010..Results in an OSR of 11
 *  0b01011..Results in an OSR of 12
 *  0b01100..Results in an OSR of 13
 *  0b01101..Results in an OSR of 14
 *  0b01110..Results in an OSR of 15
 *  0b01111..Results in an OSR of 16
 *  0b10000..Results in an OSR of 17
 *  0b10001..Results in an OSR of 18
 *  0b10010..Results in an OSR of 19
 *  0b10011..Results in an OSR of 20
 *  0b10100..Results in an OSR of 21
 *  0b10101..Results in an OSR of 22
 *  0b10110..Results in an OSR of 23
 *  0b10111..Results in an OSR of 24
 *  0b11000..Results in an OSR of 25
 *  0b11001..Results in an OSR of 26
 *  0b11010..Results in an OSR of 27
 *  0b11011..Results in an OSR of 28
 *  0b11100..Results in an OSR of 29
 *  0b11101..Results in an OSR of 30
 *  0b11110..Results in an OSR of 31
 *  0b11111..Results in an OSR of 32
 */
#define LPUART_BAUD_OSR(x)                       (((uint32_t)(((uint32_t)(x)) << LPUART_BAUD_OSR_SHIFT)) & LPUART_BAUD_OSR_MASK)

#define LPUART_BAUD_M10_MASK                     (0x20000000U)
#define LPUART_BAUD_M10_SHIFT                    (29U)
/*! M10 - 10-Bit Mode Select
 *  0b0..Receiver and transmitter use 7-bit to 9-bit data characters
 *  0b1..Receiver and transmitter use 10-bit data characters
 */
#define LPUART_BAUD_M10(x)                       (((uint32_t)(((uint32_t)(x)) << LPUART_BAUD_M10_SHIFT)) & LPUART_BAUD_M10_MASK)

#define LPUART_BAUD_MAEN2_MASK                   (0x40000000U)
#define LPUART_BAUD_MAEN2_SHIFT                  (30U)
/*! MAEN2 - Match Address Mode Enable 2
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPUART_BAUD_MAEN2(x)                     (((uint32_t)(((uint32_t)(x)) << LPUART_BAUD_MAEN2_SHIFT)) & LPUART_BAUD_MAEN2_MASK)

#define LPUART_BAUD_MAEN1_MASK                   (0x80000000U)
#define LPUART_BAUD_MAEN1_SHIFT                  (31U)
/*! MAEN1 - Match Address Mode Enable 1
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPUART_BAUD_MAEN1(x)                     (((uint32_t)(((uint32_t)(x)) << LPUART_BAUD_MAEN1_SHIFT)) & LPUART_BAUD_MAEN1_MASK)
/*! @} */

/*! @name STAT - Status */
/*! @{ */

#define LPUART_STAT_LBKFE_MASK                   (0x1U)
#define LPUART_STAT_LBKFE_SHIFT                  (0U)
/*! LBKFE - LIN Break Flag Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPUART_STAT_LBKFE(x)                     (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_LBKFE_SHIFT)) & LPUART_STAT_LBKFE_MASK)

#define LPUART_STAT_AME_MASK                     (0x2U)
#define LPUART_STAT_AME_SHIFT                    (1U)
/*! AME - Address Mark Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPUART_STAT_AME(x)                       (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_AME_SHIFT)) & LPUART_STAT_AME_MASK)

#define LPUART_STAT_MA2F_MASK                    (0x4000U)
#define LPUART_STAT_MA2F_SHIFT                   (14U)
/*! MA2F - Match 2 Flag
 *  0b0..No effect
 *  0b0..Not equal to MA2
 *  0b1..Clear the flag
 *  0b1..Equal to MA2
 */
#define LPUART_STAT_MA2F(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_MA2F_SHIFT)) & LPUART_STAT_MA2F_MASK)

#define LPUART_STAT_MA1F_MASK                    (0x8000U)
#define LPUART_STAT_MA1F_SHIFT                   (15U)
/*! MA1F - Match 1 Flag
 *  0b0..No effect
 *  0b0..Not equal to MA1
 *  0b1..Clear the flag
 *  0b1..Equal to MA1
 */
#define LPUART_STAT_MA1F(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_MA1F_SHIFT)) & LPUART_STAT_MA1F_MASK)

#define LPUART_STAT_PF_MASK                      (0x10000U)
#define LPUART_STAT_PF_SHIFT                     (16U)
/*! PF - Parity Error Flag
 *  0b0..No effect
 *  0b0..No parity error detected
 *  0b1..Clear the flag
 *  0b1..Parity error detected
 */
#define LPUART_STAT_PF(x)                        (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_PF_SHIFT)) & LPUART_STAT_PF_MASK)

#define LPUART_STAT_FE_MASK                      (0x20000U)
#define LPUART_STAT_FE_SHIFT                     (17U)
/*! FE - Framing Error Flag
 *  0b0..No effect
 *  0b0..No framing error detected (this does not guarantee that the framing is correct)
 *  0b1..Clear the flag
 *  0b1..Framing error detected
 */
#define LPUART_STAT_FE(x)                        (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_FE_SHIFT)) & LPUART_STAT_FE_MASK)

#define LPUART_STAT_NF_MASK                      (0x40000U)
#define LPUART_STAT_NF_SHIFT                     (18U)
/*! NF - Noise Flag
 *  0b0..No effect
 *  0b0..No noise detected
 *  0b1..Clear the flag
 *  0b1..Noise detected
 */
#define LPUART_STAT_NF(x)                        (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_NF_SHIFT)) & LPUART_STAT_NF_MASK)

#define LPUART_STAT_OR_MASK                      (0x80000U)
#define LPUART_STAT_OR_SHIFT                     (19U)
/*! OR - Receiver Overrun Flag
 *  0b0..No effect
 *  0b0..No overrun
 *  0b1..Clear the flag
 *  0b1..Receive overrun (new LPUART data is lost)
 */
#define LPUART_STAT_OR(x)                        (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_OR_SHIFT)) & LPUART_STAT_OR_MASK)

#define LPUART_STAT_IDLE_MASK                    (0x100000U)
#define LPUART_STAT_IDLE_SHIFT                   (20U)
/*! IDLE - Idle Line Flag
 *  0b0..Idle line detected
 *  0b0..No effect
 *  0b1..Clear the flag
 *  0b1..Idle line not detected
 */
#define LPUART_STAT_IDLE(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_IDLE_SHIFT)) & LPUART_STAT_IDLE_MASK)

#define LPUART_STAT_RDRF_MASK                    (0x200000U)
#define LPUART_STAT_RDRF_SHIFT                   (21U)
/*! RDRF - Receive Data Register Full Flag
 *  0b0..Equal to or less than watermark
 *  0b1..Greater than watermark
 */
#define LPUART_STAT_RDRF(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_RDRF_SHIFT)) & LPUART_STAT_RDRF_MASK)

#define LPUART_STAT_TC_MASK                      (0x400000U)
#define LPUART_STAT_TC_SHIFT                     (22U)
/*! TC - Transmission Complete Flag
 *  0b0..Transmitter active
 *  0b1..Transmitter idle
 */
#define LPUART_STAT_TC(x)                        (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_TC_SHIFT)) & LPUART_STAT_TC_MASK)

#define LPUART_STAT_TDRE_MASK                    (0x800000U)
#define LPUART_STAT_TDRE_SHIFT                   (23U)
/*! TDRE - Transmit Data Register Empty Flag
 *  0b0..Greater than watermark
 *  0b1..Equal to or less than watermark
 */
#define LPUART_STAT_TDRE(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_TDRE_SHIFT)) & LPUART_STAT_TDRE_MASK)

#define LPUART_STAT_RAF_MASK                     (0x1000000U)
#define LPUART_STAT_RAF_SHIFT                    (24U)
/*! RAF - Receiver Active Flag
 *  0b0..Idle, waiting for a start bit
 *  0b1..Receiver active (RXD pin input not idle)
 */
#define LPUART_STAT_RAF(x)                       (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_RAF_SHIFT)) & LPUART_STAT_RAF_MASK)

#define LPUART_STAT_LBKDE_MASK                   (0x2000000U)
#define LPUART_STAT_LBKDE_SHIFT                  (25U)
/*! LBKDE - LIN Break Detection Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPUART_STAT_LBKDE(x)                     (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_LBKDE_SHIFT)) & LPUART_STAT_LBKDE_MASK)

#define LPUART_STAT_BRK13_MASK                   (0x4000000U)
#define LPUART_STAT_BRK13_SHIFT                  (26U)
/*! BRK13 - Break Character Generation Length
 *  0b0..9 to 13 bit times
 *  0b1..12 to 15 bit times
 */
#define LPUART_STAT_BRK13(x)                     (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_BRK13_SHIFT)) & LPUART_STAT_BRK13_MASK)

#define LPUART_STAT_RWUID_MASK                   (0x8000000U)
#define LPUART_STAT_RWUID_SHIFT                  (27U)
/*! RWUID - Receive Wake Up Idle Detect
 *  0b0..STAT[IDLE] does not become 1
 *  0b1..STAT[IDLE] becomes 1
 */
#define LPUART_STAT_RWUID(x)                     (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_RWUID_SHIFT)) & LPUART_STAT_RWUID_MASK)

#define LPUART_STAT_RXINV_MASK                   (0x10000000U)
#define LPUART_STAT_RXINV_SHIFT                  (28U)
/*! RXINV - Receive Data Inversion
 *  0b0..Inverted
 *  0b1..Not inverted
 */
#define LPUART_STAT_RXINV(x)                     (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_RXINV_SHIFT)) & LPUART_STAT_RXINV_MASK)

#define LPUART_STAT_MSBF_MASK                    (0x20000000U)
#define LPUART_STAT_MSBF_SHIFT                   (29U)
/*! MSBF - MSB First
 *  0b0..LSB
 *  0b1..MSB
 */
#define LPUART_STAT_MSBF(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_MSBF_SHIFT)) & LPUART_STAT_MSBF_MASK)

#define LPUART_STAT_RXEDGIF_MASK                 (0x40000000U)
#define LPUART_STAT_RXEDGIF_SHIFT                (30U)
/*! RXEDGIF - RXD Pin Active Edge Interrupt Flag
 *  0b0..No effect
 *  0b0..Not occurred
 *  0b1..Clear the flag
 *  0b1..Occurred
 */
#define LPUART_STAT_RXEDGIF(x)                   (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_RXEDGIF_SHIFT)) & LPUART_STAT_RXEDGIF_MASK)

#define LPUART_STAT_LBKDIF_MASK                  (0x80000000U)
#define LPUART_STAT_LBKDIF_SHIFT                 (31U)
/*! LBKDIF - LIN Break Detect Interrupt Flag
 *  0b0..No effect
 *  0b0..Not detected
 *  0b1..Clear the flag
 *  0b1..Detected
 */
#define LPUART_STAT_LBKDIF(x)                    (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_LBKDIF_SHIFT)) & LPUART_STAT_LBKDIF_MASK)
/*! @} */

/*! @name CTRL - Control */
/*! @{ */

#define LPUART_CTRL_PT_MASK                      (0x1U)
#define LPUART_CTRL_PT_SHIFT                     (0U)
/*! PT - Parity Type
 *  0b0..Even parity
 *  0b1..Odd parity
 */
#define LPUART_CTRL_PT(x)                        (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_PT_SHIFT)) & LPUART_CTRL_PT_MASK)

#define LPUART_CTRL_PE_MASK                      (0x2U)
#define LPUART_CTRL_PE_SHIFT                     (1U)
/*! PE - Parity Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPUART_CTRL_PE(x)                        (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_PE_SHIFT)) & LPUART_CTRL_PE_MASK)

#define LPUART_CTRL_ILT_MASK                     (0x4U)
#define LPUART_CTRL_ILT_SHIFT                    (2U)
/*! ILT - Idle Line Type Select
 *  0b0..After the start bit
 *  0b1..After the stop bit
 */
#define LPUART_CTRL_ILT(x)                       (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_ILT_SHIFT)) & LPUART_CTRL_ILT_MASK)

#define LPUART_CTRL_WAKE_MASK                    (0x8U)
#define LPUART_CTRL_WAKE_SHIFT                   (3U)
/*! WAKE - Receiver Wake-Up Method Select
 *  0b0..Idle
 *  0b1..Mark
 */
#define LPUART_CTRL_WAKE(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_WAKE_SHIFT)) & LPUART_CTRL_WAKE_MASK)

#define LPUART_CTRL_M_MASK                       (0x10U)
#define LPUART_CTRL_M_SHIFT                      (4U)
/*! M - 9-Bit Or 8-Bit Mode Select
 *  0b0..8-bit
 *  0b1..9-bit
 */
#define LPUART_CTRL_M(x)                         (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_M_SHIFT)) & LPUART_CTRL_M_MASK)

#define LPUART_CTRL_RSRC_MASK                    (0x20U)
#define LPUART_CTRL_RSRC_SHIFT                   (5U)
/*! RSRC - Receiver Source Select
 *  0b0..Internal Loopback mode
 *  0b1..Single-wire mode
 */
#define LPUART_CTRL_RSRC(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_RSRC_SHIFT)) & LPUART_CTRL_RSRC_MASK)

#define LPUART_CTRL_DOZEEN_MASK                  (0x40U)
#define LPUART_CTRL_DOZEEN_SHIFT                 (6U)
/*! DOZEEN - Doze Mode
 *  0b0..Enable
 *  0b1..Disable
 */
#define LPUART_CTRL_DOZEEN(x)                    (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_DOZEEN_SHIFT)) & LPUART_CTRL_DOZEEN_MASK)

#define LPUART_CTRL_LOOPS_MASK                   (0x80U)
#define LPUART_CTRL_LOOPS_SHIFT                  (7U)
/*! LOOPS - Loop Mode Select
 *  0b0..Normal operation: RXD and TXD use separate pins
 *  0b1..Loop mode or Single-Wire mode
 */
#define LPUART_CTRL_LOOPS(x)                     (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_LOOPS_SHIFT)) & LPUART_CTRL_LOOPS_MASK)

#define LPUART_CTRL_IDLECFG_MASK                 (0x700U)
#define LPUART_CTRL_IDLECFG_SHIFT                (8U)
/*! IDLECFG - Idle Configuration
 *  0b000..1
 *  0b001..2
 *  0b010..4
 *  0b011..8
 *  0b100..16
 *  0b101..32
 *  0b110..64
 *  0b111..128
 */
#define LPUART_CTRL_IDLECFG(x)                   (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_IDLECFG_SHIFT)) & LPUART_CTRL_IDLECFG_MASK)

#define LPUART_CTRL_M7_MASK                      (0x800U)
#define LPUART_CTRL_M7_SHIFT                     (11U)
/*! M7 - 7-Bit Mode Select
 *  0b0..8-bit to 10-bit
 *  0b1..7-bit
 */
#define LPUART_CTRL_M7(x)                        (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_M7_SHIFT)) & LPUART_CTRL_M7_MASK)

#define LPUART_CTRL_MA2IE_MASK                   (0x4000U)
#define LPUART_CTRL_MA2IE_SHIFT                  (14U)
/*! MA2IE - Match 2 (MA2F) Interrupt Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPUART_CTRL_MA2IE(x)                     (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_MA2IE_SHIFT)) & LPUART_CTRL_MA2IE_MASK)

#define LPUART_CTRL_MA1IE_MASK                   (0x8000U)
#define LPUART_CTRL_MA1IE_SHIFT                  (15U)
/*! MA1IE - Match 1 (MA1F) Interrupt Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPUART_CTRL_MA1IE(x)                     (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_MA1IE_SHIFT)) & LPUART_CTRL_MA1IE_MASK)

#define LPUART_CTRL_SBK_MASK                     (0x10000U)
#define LPUART_CTRL_SBK_SHIFT                    (16U)
/*! SBK - Send Break
 *  0b0..Normal transmitter operation
 *  0b1..Queue break character(s) to be sent
 */
#define LPUART_CTRL_SBK(x)                       (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_SBK_SHIFT)) & LPUART_CTRL_SBK_MASK)

#define LPUART_CTRL_RWU_MASK                     (0x20000U)
#define LPUART_CTRL_RWU_SHIFT                    (17U)
/*! RWU - Receiver Wake-Up Control
 *  0b0..Normal receiver operation
 *  0b1..LPUART receiver in standby, waiting for a wake-up condition
 */
#define LPUART_CTRL_RWU(x)                       (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_RWU_SHIFT)) & LPUART_CTRL_RWU_MASK)

#define LPUART_CTRL_RE_MASK                      (0x40000U)
#define LPUART_CTRL_RE_SHIFT                     (18U)
/*! RE - Receiver Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPUART_CTRL_RE(x)                        (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_RE_SHIFT)) & LPUART_CTRL_RE_MASK)

#define LPUART_CTRL_TE_MASK                      (0x80000U)
#define LPUART_CTRL_TE_SHIFT                     (19U)
/*! TE - Transmitter Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPUART_CTRL_TE(x)                        (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_TE_SHIFT)) & LPUART_CTRL_TE_MASK)

#define LPUART_CTRL_ILIE_MASK                    (0x100000U)
#define LPUART_CTRL_ILIE_SHIFT                   (20U)
/*! ILIE - Idle Line Interrupt Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPUART_CTRL_ILIE(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_ILIE_SHIFT)) & LPUART_CTRL_ILIE_MASK)

#define LPUART_CTRL_RIE_MASK                     (0x200000U)
#define LPUART_CTRL_RIE_SHIFT                    (21U)
/*! RIE - Receiver Interrupt Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPUART_CTRL_RIE(x)                       (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_RIE_SHIFT)) & LPUART_CTRL_RIE_MASK)

#define LPUART_CTRL_TCIE_MASK                    (0x400000U)
#define LPUART_CTRL_TCIE_SHIFT                   (22U)
/*! TCIE - Transmission Complete Interrupt Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPUART_CTRL_TCIE(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_TCIE_SHIFT)) & LPUART_CTRL_TCIE_MASK)

#define LPUART_CTRL_TIE_MASK                     (0x800000U)
#define LPUART_CTRL_TIE_SHIFT                    (23U)
/*! TIE - Transmit Interrupt Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPUART_CTRL_TIE(x)                       (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_TIE_SHIFT)) & LPUART_CTRL_TIE_MASK)

#define LPUART_CTRL_PEIE_MASK                    (0x1000000U)
#define LPUART_CTRL_PEIE_SHIFT                   (24U)
/*! PEIE - Parity Error Interrupt Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPUART_CTRL_PEIE(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_PEIE_SHIFT)) & LPUART_CTRL_PEIE_MASK)

#define LPUART_CTRL_FEIE_MASK                    (0x2000000U)
#define LPUART_CTRL_FEIE_SHIFT                   (25U)
/*! FEIE - Framing Error Interrupt Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPUART_CTRL_FEIE(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_FEIE_SHIFT)) & LPUART_CTRL_FEIE_MASK)

#define LPUART_CTRL_NEIE_MASK                    (0x4000000U)
#define LPUART_CTRL_NEIE_SHIFT                   (26U)
/*! NEIE - Noise Error Interrupt Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPUART_CTRL_NEIE(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_NEIE_SHIFT)) & LPUART_CTRL_NEIE_MASK)

#define LPUART_CTRL_ORIE_MASK                    (0x8000000U)
#define LPUART_CTRL_ORIE_SHIFT                   (27U)
/*! ORIE - Overrun Interrupt Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPUART_CTRL_ORIE(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_ORIE_SHIFT)) & LPUART_CTRL_ORIE_MASK)

#define LPUART_CTRL_TXINV_MASK                   (0x10000000U)
#define LPUART_CTRL_TXINV_SHIFT                  (28U)
/*! TXINV - Transmit Data Inversion
 *  0b0..Not inverted
 *  0b1..Inverted
 */
#define LPUART_CTRL_TXINV(x)                     (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_TXINV_SHIFT)) & LPUART_CTRL_TXINV_MASK)

#define LPUART_CTRL_TXDIR_MASK                   (0x20000000U)
#define LPUART_CTRL_TXDIR_SHIFT                  (29U)
/*! TXDIR - TXD Pin Direction in Single-Wire Mode
 *  0b0..Input
 *  0b1..Output
 */
#define LPUART_CTRL_TXDIR(x)                     (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_TXDIR_SHIFT)) & LPUART_CTRL_TXDIR_MASK)

#define LPUART_CTRL_R9T8_MASK                    (0x40000000U)
#define LPUART_CTRL_R9T8_SHIFT                   (30U)
/*! R9T8 - Receive Bit 9 Transmit Bit 8 */
#define LPUART_CTRL_R9T8(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_R9T8_SHIFT)) & LPUART_CTRL_R9T8_MASK)

#define LPUART_CTRL_R8T9_MASK                    (0x80000000U)
#define LPUART_CTRL_R8T9_SHIFT                   (31U)
/*! R8T9 - Receive Bit 8 Transmit Bit 9 */
#define LPUART_CTRL_R8T9(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_R8T9_SHIFT)) & LPUART_CTRL_R8T9_MASK)
/*! @} */

/*! @name DATA - Data */
/*! @{ */

#define LPUART_DATA_R0T0_MASK                    (0x1U)
#define LPUART_DATA_R0T0_SHIFT                   (0U)
/*! R0T0 - Read receive FIFO bit 0 or write transmit FIFO bit 0 */
#define LPUART_DATA_R0T0(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_DATA_R0T0_SHIFT)) & LPUART_DATA_R0T0_MASK)

#define LPUART_DATA_R1T1_MASK                    (0x2U)
#define LPUART_DATA_R1T1_SHIFT                   (1U)
/*! R1T1 - Read receive FIFO bit 1 or write transmit FIFO bit 1 */
#define LPUART_DATA_R1T1(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_DATA_R1T1_SHIFT)) & LPUART_DATA_R1T1_MASK)

#define LPUART_DATA_R2T2_MASK                    (0x4U)
#define LPUART_DATA_R2T2_SHIFT                   (2U)
/*! R2T2 - Read receive FIFO bit 2 or write transmit FIFO bit 2 */
#define LPUART_DATA_R2T2(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_DATA_R2T2_SHIFT)) & LPUART_DATA_R2T2_MASK)

#define LPUART_DATA_R3T3_MASK                    (0x8U)
#define LPUART_DATA_R3T3_SHIFT                   (3U)
/*! R3T3 - Read receive FIFO bit 3 or write transmit FIFO bit 3 */
#define LPUART_DATA_R3T3(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_DATA_R3T3_SHIFT)) & LPUART_DATA_R3T3_MASK)

#define LPUART_DATA_R4T4_MASK                    (0x10U)
#define LPUART_DATA_R4T4_SHIFT                   (4U)
/*! R4T4 - Read receive FIFO bit 4 or write transmit FIFO bit 4 */
#define LPUART_DATA_R4T4(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_DATA_R4T4_SHIFT)) & LPUART_DATA_R4T4_MASK)

#define LPUART_DATA_R5T5_MASK                    (0x20U)
#define LPUART_DATA_R5T5_SHIFT                   (5U)
/*! R5T5 - Read receive FIFO bit 5 or write transmit FIFO bit 5 */
#define LPUART_DATA_R5T5(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_DATA_R5T5_SHIFT)) & LPUART_DATA_R5T5_MASK)

#define LPUART_DATA_R6T6_MASK                    (0x40U)
#define LPUART_DATA_R6T6_SHIFT                   (6U)
/*! R6T6 - Read receive FIFO bit 6 or write transmit FIFO bit 6 */
#define LPUART_DATA_R6T6(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_DATA_R6T6_SHIFT)) & LPUART_DATA_R6T6_MASK)

#define LPUART_DATA_R7T7_MASK                    (0x80U)
#define LPUART_DATA_R7T7_SHIFT                   (7U)
/*! R7T7 - Read receive FIFO bit 7 or write transmit FIFO bit 7 */
#define LPUART_DATA_R7T7(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_DATA_R7T7_SHIFT)) & LPUART_DATA_R7T7_MASK)

#define LPUART_DATA_R8T8_MASK                    (0x100U)
#define LPUART_DATA_R8T8_SHIFT                   (8U)
/*! R8T8 - Read receive FIFO bit 8 or write transmit FIFO bit 8 */
#define LPUART_DATA_R8T8(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_DATA_R8T8_SHIFT)) & LPUART_DATA_R8T8_MASK)

#define LPUART_DATA_R9T9_MASK                    (0x200U)
#define LPUART_DATA_R9T9_SHIFT                   (9U)
/*! R9T9 - Read receive FIFO bit 9 or write transmit FIFO bit 9 */
#define LPUART_DATA_R9T9(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_DATA_R9T9_SHIFT)) & LPUART_DATA_R9T9_MASK)

#define LPUART_DATA_LINBRK_MASK                  (0x400U)
#define LPUART_DATA_LINBRK_SHIFT                 (10U)
/*! LINBRK - LIN Break
 *  0b0..Not detected
 *  0b1..Detected
 */
#define LPUART_DATA_LINBRK(x)                    (((uint32_t)(((uint32_t)(x)) << LPUART_DATA_LINBRK_SHIFT)) & LPUART_DATA_LINBRK_MASK)

#define LPUART_DATA_IDLINE_MASK                  (0x800U)
#define LPUART_DATA_IDLINE_SHIFT                 (11U)
/*! IDLINE - Idle Line
 *  0b0..Not idle
 *  0b1..Idle
 */
#define LPUART_DATA_IDLINE(x)                    (((uint32_t)(((uint32_t)(x)) << LPUART_DATA_IDLINE_SHIFT)) & LPUART_DATA_IDLINE_MASK)

#define LPUART_DATA_RXEMPT_MASK                  (0x1000U)
#define LPUART_DATA_RXEMPT_SHIFT                 (12U)
/*! RXEMPT - Receive Buffer Empty
 *  0b0..Valid data
 *  0b1..Invalid data and empty
 */
#define LPUART_DATA_RXEMPT(x)                    (((uint32_t)(((uint32_t)(x)) << LPUART_DATA_RXEMPT_SHIFT)) & LPUART_DATA_RXEMPT_MASK)

#define LPUART_DATA_FRETSC_MASK                  (0x2000U)
#define LPUART_DATA_FRETSC_SHIFT                 (13U)
/*! FRETSC - Frame Error Transmit Special Character
 *  0b0..Received without a frame error on reads or transmits a normal character on writes
 *  0b1..Received with a frame error on reads or transmits an idle or break character on writes
 */
#define LPUART_DATA_FRETSC(x)                    (((uint32_t)(((uint32_t)(x)) << LPUART_DATA_FRETSC_SHIFT)) & LPUART_DATA_FRETSC_MASK)

#define LPUART_DATA_PARITYE_MASK                 (0x4000U)
#define LPUART_DATA_PARITYE_SHIFT                (14U)
/*! PARITYE - Parity Error
 *  0b0..Received without a parity error
 *  0b1..Received with a parity error
 */
#define LPUART_DATA_PARITYE(x)                   (((uint32_t)(((uint32_t)(x)) << LPUART_DATA_PARITYE_SHIFT)) & LPUART_DATA_PARITYE_MASK)

#define LPUART_DATA_NOISY_MASK                   (0x8000U)
#define LPUART_DATA_NOISY_SHIFT                  (15U)
/*! NOISY - Noisy Data Received
 *  0b0..Received without noise
 *  0b1..Received with noise
 */
#define LPUART_DATA_NOISY(x)                     (((uint32_t)(((uint32_t)(x)) << LPUART_DATA_NOISY_SHIFT)) & LPUART_DATA_NOISY_MASK)
/*! @} */

/*! @name MATCH - Match Address */
/*! @{ */

#define LPUART_MATCH_MA1_MASK                    (0x3FFU)
#define LPUART_MATCH_MA1_SHIFT                   (0U)
/*! MA1 - Match Address 1 */
#define LPUART_MATCH_MA1(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_MATCH_MA1_SHIFT)) & LPUART_MATCH_MA1_MASK)

#define LPUART_MATCH_MA2_MASK                    (0x3FF0000U)
#define LPUART_MATCH_MA2_SHIFT                   (16U)
/*! MA2 - Match Address 2 */
#define LPUART_MATCH_MA2(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_MATCH_MA2_SHIFT)) & LPUART_MATCH_MA2_MASK)
/*! @} */

/*! @name MODIR - MODEM IrDA */
/*! @{ */

#define LPUART_MODIR_TXCTSE_MASK                 (0x1U)
#define LPUART_MODIR_TXCTSE_SHIFT                (0U)
/*! TXCTSE - Transmitter CTS Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPUART_MODIR_TXCTSE(x)                   (((uint32_t)(((uint32_t)(x)) << LPUART_MODIR_TXCTSE_SHIFT)) & LPUART_MODIR_TXCTSE_MASK)

#define LPUART_MODIR_TXRTSE_MASK                 (0x2U)
#define LPUART_MODIR_TXRTSE_SHIFT                (1U)
/*! TXRTSE - Transmitter RTS Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPUART_MODIR_TXRTSE(x)                   (((uint32_t)(((uint32_t)(x)) << LPUART_MODIR_TXRTSE_SHIFT)) & LPUART_MODIR_TXRTSE_MASK)

#define LPUART_MODIR_TXRTSPOL_MASK               (0x4U)
#define LPUART_MODIR_TXRTSPOL_SHIFT              (2U)
/*! TXRTSPOL - Transmitter RTS Polarity
 *  0b0..Active low
 *  0b1..Active high
 */
#define LPUART_MODIR_TXRTSPOL(x)                 (((uint32_t)(((uint32_t)(x)) << LPUART_MODIR_TXRTSPOL_SHIFT)) & LPUART_MODIR_TXRTSPOL_MASK)

#define LPUART_MODIR_RXRTSE_MASK                 (0x8U)
#define LPUART_MODIR_RXRTSE_SHIFT                (3U)
/*! RXRTSE - Receiver RTS Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPUART_MODIR_RXRTSE(x)                   (((uint32_t)(((uint32_t)(x)) << LPUART_MODIR_RXRTSE_SHIFT)) & LPUART_MODIR_RXRTSE_MASK)

#define LPUART_MODIR_TXCTSC_MASK                 (0x10U)
#define LPUART_MODIR_TXCTSC_SHIFT                (4U)
/*! TXCTSC - Transmit CTS Configuration
 *  0b0..Sampled at the start of each character
 *  0b1..Sampled when the transmitter is idle
 */
#define LPUART_MODIR_TXCTSC(x)                   (((uint32_t)(((uint32_t)(x)) << LPUART_MODIR_TXCTSC_SHIFT)) & LPUART_MODIR_TXCTSC_MASK)

#define LPUART_MODIR_TXCTSSRC_MASK               (0x20U)
#define LPUART_MODIR_TXCTSSRC_SHIFT              (5U)
/*! TXCTSSRC - Transmit CTS Source
 *  0b0..The CTS_B pin
 *  0b1..An internal connection to the receiver address match result
 */
#define LPUART_MODIR_TXCTSSRC(x)                 (((uint32_t)(((uint32_t)(x)) << LPUART_MODIR_TXCTSSRC_SHIFT)) & LPUART_MODIR_TXCTSSRC_MASK)

#define LPUART_MODIR_RTSWATER_MASK               (0x300U)
#define LPUART_MODIR_RTSWATER_SHIFT              (8U)
/*! RTSWATER - Receive RTS Configuration */
#define LPUART_MODIR_RTSWATER(x)                 (((uint32_t)(((uint32_t)(x)) << LPUART_MODIR_RTSWATER_SHIFT)) & LPUART_MODIR_RTSWATER_MASK)

#define LPUART_MODIR_TNP_MASK                    (0x30000U)
#define LPUART_MODIR_TNP_SHIFT                   (16U)
/*! TNP - Transmitter Narrow Pulse
 *  0b00..1 / OSR
 *  0b01..2 / OSR
 *  0b10..3 / OSR
 *  0b11..4 / OSR
 */
#define LPUART_MODIR_TNP(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_MODIR_TNP_SHIFT)) & LPUART_MODIR_TNP_MASK)

#define LPUART_MODIR_IREN_MASK                   (0x40000U)
#define LPUART_MODIR_IREN_SHIFT                  (18U)
/*! IREN - IR Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPUART_MODIR_IREN(x)                     (((uint32_t)(((uint32_t)(x)) << LPUART_MODIR_IREN_SHIFT)) & LPUART_MODIR_IREN_MASK)
/*! @} */

/*! @name FIFO - FIFO */
/*! @{ */

#define LPUART_FIFO_RXFIFOSIZE_MASK              (0x7U)
#define LPUART_FIFO_RXFIFOSIZE_SHIFT             (0U)
/*! RXFIFOSIZE - Receive FIFO Buffer Depth
 *  0b000..1
 *  0b001..4
 *  0b010..8
 *  0b011..16
 *  0b100..32
 *  0b101..64
 *  0b110..128
 *  0b111..256
 */
#define LPUART_FIFO_RXFIFOSIZE(x)                (((uint32_t)(((uint32_t)(x)) << LPUART_FIFO_RXFIFOSIZE_SHIFT)) & LPUART_FIFO_RXFIFOSIZE_MASK)

#define LPUART_FIFO_RXFE_MASK                    (0x8U)
#define LPUART_FIFO_RXFE_SHIFT                   (3U)
/*! RXFE - Receive FIFO Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPUART_FIFO_RXFE(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_FIFO_RXFE_SHIFT)) & LPUART_FIFO_RXFE_MASK)

#define LPUART_FIFO_TXFIFOSIZE_MASK              (0x70U)
#define LPUART_FIFO_TXFIFOSIZE_SHIFT             (4U)
/*! TXFIFOSIZE - Transmit FIFO Buffer Depth
 *  0b000..1
 *  0b001..4
 *  0b010..8
 *  0b011..16
 *  0b100..32
 *  0b101..64
 *  0b110..128
 *  0b111..256
 */
#define LPUART_FIFO_TXFIFOSIZE(x)                (((uint32_t)(((uint32_t)(x)) << LPUART_FIFO_TXFIFOSIZE_SHIFT)) & LPUART_FIFO_TXFIFOSIZE_MASK)

#define LPUART_FIFO_TXFE_MASK                    (0x80U)
#define LPUART_FIFO_TXFE_SHIFT                   (7U)
/*! TXFE - Transmit FIFO Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPUART_FIFO_TXFE(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_FIFO_TXFE_SHIFT)) & LPUART_FIFO_TXFE_MASK)

#define LPUART_FIFO_RXUFE_MASK                   (0x100U)
#define LPUART_FIFO_RXUFE_SHIFT                  (8U)
/*! RXUFE - Receive FIFO Underflow Interrupt Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPUART_FIFO_RXUFE(x)                     (((uint32_t)(((uint32_t)(x)) << LPUART_FIFO_RXUFE_SHIFT)) & LPUART_FIFO_RXUFE_MASK)

#define LPUART_FIFO_TXOFE_MASK                   (0x200U)
#define LPUART_FIFO_TXOFE_SHIFT                  (9U)
/*! TXOFE - Transmit FIFO Overflow Interrupt Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPUART_FIFO_TXOFE(x)                     (((uint32_t)(((uint32_t)(x)) << LPUART_FIFO_TXOFE_SHIFT)) & LPUART_FIFO_TXOFE_MASK)

#define LPUART_FIFO_RXIDEN_MASK                  (0x1C00U)
#define LPUART_FIFO_RXIDEN_SHIFT                 (10U)
/*! RXIDEN - Receiver Idle Empty Enable
 *  0b000..Disable STAT[RDRF] to become 1 because of partially filled FIFO when the receiver is idle
 *  0b001..Enable STAT[RDRF] to become 1 because of partially filled FIFO when the receiver is idle for one character
 *  0b010..Enable STAT[RDRF] to become 1 because of partially filled FIFO when the receiver is idle for two characters
 *  0b011..Enable STAT[RDRF] to become 1 because of partially filled FIFO when the receiver is idle for four characters
 *  0b100..Enable STAT[RDRF] to become 1 because of partially filled FIFO when the receiver is idle for eight characters
 *  0b101..Enable STAT[RDRF] to become 1 because of partially filled FIFO when the receiver is idle for 16 characters
 *  0b110..Enable STAT[RDRF] to become 1 because of partially filled FIFO when the receiver is idle for 32 characters
 *  0b111..Enable STAT[RDRF] to become 1 because of partially filled FIFO when the receiver is idle for 64 characters
 */
#define LPUART_FIFO_RXIDEN(x)                    (((uint32_t)(((uint32_t)(x)) << LPUART_FIFO_RXIDEN_SHIFT)) & LPUART_FIFO_RXIDEN_MASK)

#define LPUART_FIFO_RXFLUSH_MASK                 (0x4000U)
#define LPUART_FIFO_RXFLUSH_SHIFT                (14U)
/*! RXFLUSH - Receive FIFO Flush
 *  0b0..No effect
 *  0b1..All data flushed out
 */
#define LPUART_FIFO_RXFLUSH(x)                   (((uint32_t)(((uint32_t)(x)) << LPUART_FIFO_RXFLUSH_SHIFT)) & LPUART_FIFO_RXFLUSH_MASK)

#define LPUART_FIFO_TXFLUSH_MASK                 (0x8000U)
#define LPUART_FIFO_TXFLUSH_SHIFT                (15U)
/*! TXFLUSH - Transmit FIFO Flush
 *  0b0..No effect
 *  0b1..All data flushed out
 */
#define LPUART_FIFO_TXFLUSH(x)                   (((uint32_t)(((uint32_t)(x)) << LPUART_FIFO_TXFLUSH_SHIFT)) & LPUART_FIFO_TXFLUSH_MASK)

#define LPUART_FIFO_RXUF_MASK                    (0x10000U)
#define LPUART_FIFO_RXUF_SHIFT                   (16U)
/*! RXUF - Receiver FIFO Underflow Flag
 *  0b0..No effect
 *  0b0..No underflow
 *  0b1..Clear the flag
 *  0b1..Underflow
 */
#define LPUART_FIFO_RXUF(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_FIFO_RXUF_SHIFT)) & LPUART_FIFO_RXUF_MASK)

#define LPUART_FIFO_TXOF_MASK                    (0x20000U)
#define LPUART_FIFO_TXOF_SHIFT                   (17U)
/*! TXOF - Transmitter FIFO Overflow Flag
 *  0b0..No effect
 *  0b0..No overflow
 *  0b1..Clear the flag
 *  0b1..Overflow
 */
#define LPUART_FIFO_TXOF(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_FIFO_TXOF_SHIFT)) & LPUART_FIFO_TXOF_MASK)

#define LPUART_FIFO_RXEMPT_MASK                  (0x400000U)
#define LPUART_FIFO_RXEMPT_SHIFT                 (22U)
/*! RXEMPT - Receive FIFO Or Buffer Empty
 *  0b0..Not empty
 *  0b1..Empty
 */
#define LPUART_FIFO_RXEMPT(x)                    (((uint32_t)(((uint32_t)(x)) << LPUART_FIFO_RXEMPT_SHIFT)) & LPUART_FIFO_RXEMPT_MASK)

#define LPUART_FIFO_TXEMPT_MASK                  (0x800000U)
#define LPUART_FIFO_TXEMPT_SHIFT                 (23U)
/*! TXEMPT - Transmit FIFO Or Buffer Empty
 *  0b0..Not empty
 *  0b1..Empty
 */
#define LPUART_FIFO_TXEMPT(x)                    (((uint32_t)(((uint32_t)(x)) << LPUART_FIFO_TXEMPT_SHIFT)) & LPUART_FIFO_TXEMPT_MASK)
/*! @} */

/*! @name WATER - Watermark */
/*! @{ */

#define LPUART_WATER_TXWATER_MASK                (0x3U)
#define LPUART_WATER_TXWATER_SHIFT               (0U)
/*! TXWATER - Transmit Watermark */
#define LPUART_WATER_TXWATER(x)                  (((uint32_t)(((uint32_t)(x)) << LPUART_WATER_TXWATER_SHIFT)) & LPUART_WATER_TXWATER_MASK)

#define LPUART_WATER_TXCOUNT_MASK                (0x700U)
#define LPUART_WATER_TXCOUNT_SHIFT               (8U)
/*! TXCOUNT - Transmit Counter */
#define LPUART_WATER_TXCOUNT(x)                  (((uint32_t)(((uint32_t)(x)) << LPUART_WATER_TXCOUNT_SHIFT)) & LPUART_WATER_TXCOUNT_MASK)

#define LPUART_WATER_RXWATER_MASK                (0x30000U)
#define LPUART_WATER_RXWATER_SHIFT               (16U)
/*! RXWATER - Receive Watermark */
#define LPUART_WATER_RXWATER(x)                  (((uint32_t)(((uint32_t)(x)) << LPUART_WATER_RXWATER_SHIFT)) & LPUART_WATER_RXWATER_MASK)

#define LPUART_WATER_RXCOUNT_MASK                (0x7000000U)
#define LPUART_WATER_RXCOUNT_SHIFT               (24U)
/*! RXCOUNT - Receive Counter */
#define LPUART_WATER_RXCOUNT(x)                  (((uint32_t)(((uint32_t)(x)) << LPUART_WATER_RXCOUNT_SHIFT)) & LPUART_WATER_RXCOUNT_MASK)
/*! @} */

/*! @name DATARO - Data Read-Only */
/*! @{ */

#define LPUART_DATARO_DATA_MASK                  (0xFFFFU)
#define LPUART_DATARO_DATA_SHIFT                 (0U)
/*! DATA - Receive Data */
#define LPUART_DATARO_DATA(x)                    (((uint32_t)(((uint32_t)(x)) << LPUART_DATARO_DATA_SHIFT)) & LPUART_DATARO_DATA_MASK)
/*! @} */


/*!
 * @}
 */ /* end of group LPUART_Register_Masks */


/*!
 * @}
 */ /* end of group LPUART_Peripheral_Access_Layer */


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


#endif  /* PERI_LPUART_H_ */

