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
**     Build:               b260810
**
**     Abstract:
**         CMSIS Peripheral Access Layer for USB
**
**     Copyright 1997-2016 Freescale Semiconductor, Inc.
**     Copyright 2016-2026 NXP
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
 * @file PERI_USB.h
 * @version 2.0
 * @date 2024-10-29
 * @brief CMSIS Peripheral Access Layer for USB
 *
 * CMSIS Peripheral Access Layer for USB
 */

#if !defined(PERI_USB_H_)
#define PERI_USB_H_                              /**< Symbol preventing repeated inclusion */

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
   -- USB Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup USB_Peripheral_Access_Layer USB Peripheral Access Layer
 * @{
 */

/** USB - Size of Registers Arrays */
#define USB_ENDPOINT_COUNT                        16u

/** USB - Register Layout Typedef */
typedef struct {
  __I  uint8_t PERID;                              /**< Peripheral ID, offset: 0x0 */
       uint8_t RESERVED_0[3];
  __I  uint8_t IDCOMP;                             /**< Peripheral ID Complement, offset: 0x4 */
       uint8_t RESERVED_1[3];
  __I  uint8_t REV;                                /**< Peripheral Revision, offset: 0x8 */
       uint8_t RESERVED_2[19];
  __IO uint8_t OTGCTL;                             /**< OTG Control, offset: 0x1C */
       uint8_t RESERVED_3[99];
  __IO uint8_t ISTAT;                              /**< Interrupt Status, offset: 0x80 */
       uint8_t RESERVED_4[3];
  __IO uint8_t INTEN;                              /**< Interrupt Enable, offset: 0x84 */
       uint8_t RESERVED_5[3];
  __IO uint8_t ERRSTAT;                            /**< Error Interrupt Status, offset: 0x88 */
       uint8_t RESERVED_6[3];
  __IO uint8_t ERREN;                              /**< Error Interrupt Enable, offset: 0x8C */
       uint8_t RESERVED_7[3];
  __I  uint8_t STAT;                               /**< Status, offset: 0x90 */
       uint8_t RESERVED_8[3];
  __IO uint8_t CTL;                                /**< Control, offset: 0x94 */
       uint8_t RESERVED_9[3];
  __IO uint8_t ADDR;                               /**< Address, offset: 0x98 */
       uint8_t RESERVED_10[3];
  __IO uint8_t BDTPAGE1;                           /**< BDT Page 1, offset: 0x9C */
       uint8_t RESERVED_11[3];
  __I  uint8_t FRMNUML;                            /**< Frame Number Register Low, offset: 0xA0 */
       uint8_t RESERVED_12[3];
  __I  uint8_t FRMNUMH;                            /**< Frame Number Register High, offset: 0xA4 */
       uint8_t RESERVED_13[11];
  __IO uint8_t BDTPAGE2;                           /**< BDT Page 2, offset: 0xB0 */
       uint8_t RESERVED_14[3];
  __IO uint8_t BDTPAGE3;                           /**< BDT Page 3, offset: 0xB4 */
       uint8_t RESERVED_15[11];
  struct {                                         /* offset: 0xC0, array step: 0x4 */
    __IO uint8_t ENDPT;                              /**< Endpoint Control, array offset: 0xC0, array step: 0x4 */
         uint8_t RESERVED_0[3];
  } ENDPOINT[USB_ENDPOINT_COUNT];
  __IO uint8_t USBCTRL;                            /**< USB Control, offset: 0x100 */
       uint8_t RESERVED_16[3];
  __I  uint8_t OBSERVE;                            /**< USB OTG Observe, offset: 0x104 */
       uint8_t RESERVED_17[3];
  __IO uint8_t CONTROL;                            /**< USB OTG Control, offset: 0x108 */
       uint8_t RESERVED_18[3];
  __IO uint8_t USBTRC0;                            /**< USB Transceiver Control 0, offset: 0x10C */
       uint8_t RESERVED_19[23];
  __I  uint8_t KEEP_ALIVE_CTRL_RSVD;               /**< Reserved, offset: 0x124 */
       uint8_t RESERVED_20[3];
  __I  uint8_t KEEP_ALIVE_WKCTRL_RSVD;             /**< Reserved, offset: 0x128 */
       uint8_t RESERVED_21[3];
  __IO uint8_t MISCCTRL;                           /**< Miscellaneous Control, offset: 0x12C */
       uint8_t RESERVED_22[3];
  __IO uint8_t STALL_IL_DIS;                       /**< Peripheral Mode Stall Disable for Endpoints 7 to 0 in IN Direction, offset: 0x130 */
       uint8_t RESERVED_23[3];
  __IO uint8_t STALL_IH_DIS;                       /**< Peripheral Mode Stall Disable for Endpoints 15 to 8 in IN Direction, offset: 0x134 */
       uint8_t RESERVED_24[3];
  __IO uint8_t STALL_OL_DIS;                       /**< Peripheral Mode Stall Disable for Endpoints 7 to 0 in OUT Direction, offset: 0x138 */
       uint8_t RESERVED_25[3];
  __IO uint8_t STALL_OH_DIS;                       /**< Peripheral Mode Stall Disable for Endpoints 15 to 8 in OUT Direction, offset: 0x13C */
       uint8_t RESERVED_26[3];
  __IO uint8_t CLK_RECOVER_CTRL;                   /**< USB Clock Recovery Control, offset: 0x140 */
       uint8_t RESERVED_27[3];
  __IO uint8_t CLK_RECOVER_IRC_EN;                 /**< FIRC Oscillator Enable, offset: 0x144 */
       uint8_t RESERVED_28[15];
  __IO uint8_t CLK_RECOVER_INT_EN;                 /**< Clock Recovery Combined Interrupt Enable, offset: 0x154 */
       uint8_t RESERVED_29[7];
  __IO uint8_t CLK_RECOVER_INT_STATUS;             /**< Clock Recovery Separated Interrupt Status, offset: 0x15C */
} USB_Type;

/* ----------------------------------------------------------------------------
   -- USB Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup USB_Register_Masks USB Register Masks
 * @{
 */

/*! @name PERID - Peripheral ID */
/*! @{ */

#define USB_PERID_ID_MASK                        (0x3FU)
#define USB_PERID_ID_SHIFT                       (0U)
/*! ID - Peripheral Identification */
#define USB_PERID_ID(x)                          (((uint8_t)(((uint8_t)(x)) << USB_PERID_ID_SHIFT)) & USB_PERID_ID_MASK)
/*! @} */

/*! @name IDCOMP - Peripheral ID Complement */
/*! @{ */

#define USB_IDCOMP_NID_MASK                      (0x3FU)
#define USB_IDCOMP_NID_SHIFT                     (0U)
/*! NID - Negative Peripheral ID */
#define USB_IDCOMP_NID(x)                        (((uint8_t)(((uint8_t)(x)) << USB_IDCOMP_NID_SHIFT)) & USB_IDCOMP_NID_MASK)
/*! @} */

/*! @name REV - Peripheral Revision */
/*! @{ */

#define USB_REV_REV_MASK                         (0xFFU)
#define USB_REV_REV_SHIFT                        (0U)
/*! REV - Revision */
#define USB_REV_REV(x)                           (((uint8_t)(((uint8_t)(x)) << USB_REV_REV_SHIFT)) & USB_REV_REV_MASK)
/*! @} */

/*! @name OTGCTL - OTG Control */
/*! @{ */

#define USB_OTGCTL_DPHIGH_MASK                   (0x80U)
#define USB_OTGCTL_DPHIGH_SHIFT                  (7U)
/*! DPHIGH - D+ Data Line Pullup Resistor Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define USB_OTGCTL_DPHIGH(x)                     (((uint8_t)(((uint8_t)(x)) << USB_OTGCTL_DPHIGH_SHIFT)) & USB_OTGCTL_DPHIGH_MASK)
/*! @} */

/*! @name ISTAT - Interrupt Status */
/*! @{ */

#define USB_ISTAT_USBRST_MASK                    (0x1U)
#define USB_ISTAT_USBRST_SHIFT                   (0U)
/*! USBRST - USB Reset Flag
 *  0b0..No effect
 *  0b0..Not detected
 *  0b1..Clear the flag
 *  0b1..Detected
 */
#define USB_ISTAT_USBRST(x)                      (((uint8_t)(((uint8_t)(x)) << USB_ISTAT_USBRST_SHIFT)) & USB_ISTAT_USBRST_MASK)

#define USB_ISTAT_ERROR_MASK                     (0x2U)
#define USB_ISTAT_ERROR_SHIFT                    (1U)
/*! ERROR - Error Flag
 *  0b0..Error did not occur
 *  0b0..No effect
 *  0b1..Clear the flag
 *  0b1..Error occurred
 */
#define USB_ISTAT_ERROR(x)                       (((uint8_t)(((uint8_t)(x)) << USB_ISTAT_ERROR_SHIFT)) & USB_ISTAT_ERROR_MASK)

#define USB_ISTAT_SOFTOK_MASK                    (0x4U)
#define USB_ISTAT_SOFTOK_SHIFT                   (2U)
/*! SOFTOK - Start Of Frame (SOF) Token Flag
 *  0b0..Did not receive
 *  0b0..No effect
 *  0b1..Clear the flag
 *  0b1..Received
 */
#define USB_ISTAT_SOFTOK(x)                      (((uint8_t)(((uint8_t)(x)) << USB_ISTAT_SOFTOK_SHIFT)) & USB_ISTAT_SOFTOK_MASK)

#define USB_ISTAT_TOKDNE_MASK                    (0x8U)
#define USB_ISTAT_TOKDNE_SHIFT                   (3U)
/*! TOKDNE - Current Token Processing Flag
 *  0b0..No effect
 *  0b0..Not processed
 *  0b1..Clear the flag
 *  0b1..Processed
 */
#define USB_ISTAT_TOKDNE(x)                      (((uint8_t)(((uint8_t)(x)) << USB_ISTAT_TOKDNE_SHIFT)) & USB_ISTAT_TOKDNE_MASK)

#define USB_ISTAT_SLEEP_MASK                     (0x10U)
#define USB_ISTAT_SLEEP_SHIFT                    (4U)
/*! SLEEP - Sleep Flag
 *  0b0..Interrupt did not occur
 *  0b0..No effect
 *  0b1..Clear the flag
 *  0b1..Interrupt occurred
 */
#define USB_ISTAT_SLEEP(x)                       (((uint8_t)(((uint8_t)(x)) << USB_ISTAT_SLEEP_SHIFT)) & USB_ISTAT_SLEEP_MASK)

#define USB_ISTAT_RESUME_MASK                    (0x20U)
#define USB_ISTAT_RESUME_SHIFT                   (5U)
/*! RESUME - Resume Flag
 *  0b0..Interrupt did not occur
 *  0b0..No effect
 *  0b1..Clear the flag
 *  0b1..Interrupt occurred
 */
#define USB_ISTAT_RESUME(x)                      (((uint8_t)(((uint8_t)(x)) << USB_ISTAT_RESUME_SHIFT)) & USB_ISTAT_RESUME_MASK)

#define USB_ISTAT_STALL_MASK                     (0x80U)
#define USB_ISTAT_STALL_SHIFT                    (7U)
/*! STALL - Stall Interrupt Flag
 *  0b0..Interrupt did not occur
 *  0b0..No effect
 *  0b1..Clear the flag
 *  0b1..Interrupt occurred
 */
#define USB_ISTAT_STALL(x)                       (((uint8_t)(((uint8_t)(x)) << USB_ISTAT_STALL_SHIFT)) & USB_ISTAT_STALL_MASK)
/*! @} */

/*! @name INTEN - Interrupt Enable */
/*! @{ */

#define USB_INTEN_USBRSTEN_MASK                  (0x1U)
#define USB_INTEN_USBRSTEN_SHIFT                 (0U)
/*! USBRSTEN - USBRST Interrupt Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define USB_INTEN_USBRSTEN(x)                    (((uint8_t)(((uint8_t)(x)) << USB_INTEN_USBRSTEN_SHIFT)) & USB_INTEN_USBRSTEN_MASK)

#define USB_INTEN_ERROREN_MASK                   (0x2U)
#define USB_INTEN_ERROREN_SHIFT                  (1U)
/*! ERROREN - ERROR Interrupt Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define USB_INTEN_ERROREN(x)                     (((uint8_t)(((uint8_t)(x)) << USB_INTEN_ERROREN_SHIFT)) & USB_INTEN_ERROREN_MASK)

#define USB_INTEN_SOFTOKEN_MASK                  (0x4U)
#define USB_INTEN_SOFTOKEN_SHIFT                 (2U)
/*! SOFTOKEN - SOFTOK Interrupt Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define USB_INTEN_SOFTOKEN(x)                    (((uint8_t)(((uint8_t)(x)) << USB_INTEN_SOFTOKEN_SHIFT)) & USB_INTEN_SOFTOKEN_MASK)

#define USB_INTEN_TOKDNEEN_MASK                  (0x8U)
#define USB_INTEN_TOKDNEEN_SHIFT                 (3U)
/*! TOKDNEEN - TOKDNE Interrupt Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define USB_INTEN_TOKDNEEN(x)                    (((uint8_t)(((uint8_t)(x)) << USB_INTEN_TOKDNEEN_SHIFT)) & USB_INTEN_TOKDNEEN_MASK)

#define USB_INTEN_SLEEPEN_MASK                   (0x10U)
#define USB_INTEN_SLEEPEN_SHIFT                  (4U)
/*! SLEEPEN - SLEEP Interrupt Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define USB_INTEN_SLEEPEN(x)                     (((uint8_t)(((uint8_t)(x)) << USB_INTEN_SLEEPEN_SHIFT)) & USB_INTEN_SLEEPEN_MASK)

#define USB_INTEN_RESUMEEN_MASK                  (0x20U)
#define USB_INTEN_RESUMEEN_SHIFT                 (5U)
/*! RESUMEEN - RESUME Interrupt Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define USB_INTEN_RESUMEEN(x)                    (((uint8_t)(((uint8_t)(x)) << USB_INTEN_RESUMEEN_SHIFT)) & USB_INTEN_RESUMEEN_MASK)

#define USB_INTEN_STALLEN_MASK                   (0x80U)
#define USB_INTEN_STALLEN_SHIFT                  (7U)
/*! STALLEN - STALL Interrupt Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define USB_INTEN_STALLEN(x)                     (((uint8_t)(((uint8_t)(x)) << USB_INTEN_STALLEN_SHIFT)) & USB_INTEN_STALLEN_MASK)
/*! @} */

/*! @name ERRSTAT - Error Interrupt Status */
/*! @{ */

#define USB_ERRSTAT_PIDERR_MASK                  (0x1U)
#define USB_ERRSTAT_PIDERR_SHIFT                 (0U)
/*! PIDERR - PID Error Flag
 *  0b0..Did not fail
 *  0b0..No effect
 *  0b1..Clear the flag
 *  0b1..Failed
 */
#define USB_ERRSTAT_PIDERR(x)                    (((uint8_t)(((uint8_t)(x)) << USB_ERRSTAT_PIDERR_SHIFT)) & USB_ERRSTAT_PIDERR_MASK)

#define USB_ERRSTAT_CRC5EOF_MASK                 (0x2U)
#define USB_ERRSTAT_CRC5EOF_SHIFT                (1U)
/*! CRC5EOF - CRC5 Error or End of Frame Error Flag
 *  0b0..Interrupt did not occur
 *  0b0..No effect
 *  0b1..Clear the flag
 *  0b1..Interrupt occurred
 */
#define USB_ERRSTAT_CRC5EOF(x)                   (((uint8_t)(((uint8_t)(x)) << USB_ERRSTAT_CRC5EOF_SHIFT)) & USB_ERRSTAT_CRC5EOF_MASK)

#define USB_ERRSTAT_CRC16_MASK                   (0x4U)
#define USB_ERRSTAT_CRC16_SHIFT                  (2U)
/*! CRC16 - CRC16 Error Flag
 *  0b0..No effect
 *  0b0..Not rejected
 *  0b1..Clear the flag
 *  0b1..Rejected
 */
#define USB_ERRSTAT_CRC16(x)                     (((uint8_t)(((uint8_t)(x)) << USB_ERRSTAT_CRC16_SHIFT)) & USB_ERRSTAT_CRC16_MASK)

#define USB_ERRSTAT_DFN8_MASK                    (0x8U)
#define USB_ERRSTAT_DFN8_SHIFT                   (3U)
/*! DFN8 - Data Field Not 8 Bits Flag
 *  0b0..Integer number of bytes
 *  0b0..No effect
 *  0b1..Clear the flag
 *  0b1..Not an integer number of bytes
 */
#define USB_ERRSTAT_DFN8(x)                      (((uint8_t)(((uint8_t)(x)) << USB_ERRSTAT_DFN8_SHIFT)) & USB_ERRSTAT_DFN8_MASK)

#define USB_ERRSTAT_BTOERR_MASK                  (0x10U)
#define USB_ERRSTAT_BTOERR_SHIFT                 (4U)
/*! BTOERR - Bus Turnaround Timeout Error Flag
 *  0b0..No effect
 *  0b0..Not timed out
 *  0b1..Clear the flag
 *  0b1..Timed out
 */
#define USB_ERRSTAT_BTOERR(x)                    (((uint8_t)(((uint8_t)(x)) << USB_ERRSTAT_BTOERR_SHIFT)) & USB_ERRSTAT_BTOERR_MASK)

#define USB_ERRSTAT_DMAERR_MASK                  (0x20U)
#define USB_ERRSTAT_DMAERR_SHIFT                 (5U)
/*! DMAERR - DMA Access Error Flag
 *  0b0..Interrupt did not occur
 *  0b0..No effect
 *  0b1..Clear the flag
 *  0b1..Interrupt occurred
 */
#define USB_ERRSTAT_DMAERR(x)                    (((uint8_t)(((uint8_t)(x)) << USB_ERRSTAT_DMAERR_SHIFT)) & USB_ERRSTAT_DMAERR_MASK)

#define USB_ERRSTAT_OWNERR_MASK                  (0x40U)
#define USB_ERRSTAT_OWNERR_SHIFT                 (6U)
/*! OWNERR - BD Unavailable Error Flag
 *  0b0..Interrupt did not occur
 *  0b0..No effect
 *  0b1..Clear the flag
 *  0b1..Interrupt occurred
 */
#define USB_ERRSTAT_OWNERR(x)                    (((uint8_t)(((uint8_t)(x)) << USB_ERRSTAT_OWNERR_SHIFT)) & USB_ERRSTAT_OWNERR_MASK)

#define USB_ERRSTAT_BTSERR_MASK                  (0x80U)
#define USB_ERRSTAT_BTSERR_SHIFT                 (7U)
/*! BTSERR - Bit Stuff Error Flag
 *  0b0..No effect
 *  0b0..Packet not rejected due to the error
 *  0b1..Clear the flag
 *  0b1..Packet rejected due to the error
 */
#define USB_ERRSTAT_BTSERR(x)                    (((uint8_t)(((uint8_t)(x)) << USB_ERRSTAT_BTSERR_SHIFT)) & USB_ERRSTAT_BTSERR_MASK)
/*! @} */

/*! @name ERREN - Error Interrupt Enable */
/*! @{ */

#define USB_ERREN_PIDERREN_MASK                  (0x1U)
#define USB_ERREN_PIDERREN_SHIFT                 (0U)
/*! PIDERREN - PIDERR Interrupt Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define USB_ERREN_PIDERREN(x)                    (((uint8_t)(((uint8_t)(x)) << USB_ERREN_PIDERREN_SHIFT)) & USB_ERREN_PIDERREN_MASK)

#define USB_ERREN_CRC5EOFEN_MASK                 (0x2U)
#define USB_ERREN_CRC5EOFEN_SHIFT                (1U)
/*! CRC5EOFEN - CRC5/EOF Interrupt Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define USB_ERREN_CRC5EOFEN(x)                   (((uint8_t)(((uint8_t)(x)) << USB_ERREN_CRC5EOFEN_SHIFT)) & USB_ERREN_CRC5EOFEN_MASK)

#define USB_ERREN_CRC16EN_MASK                   (0x4U)
#define USB_ERREN_CRC16EN_SHIFT                  (2U)
/*! CRC16EN - CRC16 Interrupt Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define USB_ERREN_CRC16EN(x)                     (((uint8_t)(((uint8_t)(x)) << USB_ERREN_CRC16EN_SHIFT)) & USB_ERREN_CRC16EN_MASK)

#define USB_ERREN_DFN8EN_MASK                    (0x8U)
#define USB_ERREN_DFN8EN_SHIFT                   (3U)
/*! DFN8EN - DFN8 (Data Field Not Integer Number of Bytes) Interrupt Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define USB_ERREN_DFN8EN(x)                      (((uint8_t)(((uint8_t)(x)) << USB_ERREN_DFN8EN_SHIFT)) & USB_ERREN_DFN8EN_MASK)

#define USB_ERREN_BTOERREN_MASK                  (0x10U)
#define USB_ERREN_BTOERREN_SHIFT                 (4U)
/*! BTOERREN - BTOERR (Bus Timeout Error) Interrupt Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define USB_ERREN_BTOERREN(x)                    (((uint8_t)(((uint8_t)(x)) << USB_ERREN_BTOERREN_SHIFT)) & USB_ERREN_BTOERREN_MASK)

#define USB_ERREN_DMAERREN_MASK                  (0x20U)
#define USB_ERREN_DMAERREN_SHIFT                 (5U)
/*! DMAERREN - DMAERR Interrupt Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define USB_ERREN_DMAERREN(x)                    (((uint8_t)(((uint8_t)(x)) << USB_ERREN_DMAERREN_SHIFT)) & USB_ERREN_DMAERREN_MASK)

#define USB_ERREN_OWNERREN_MASK                  (0x40U)
#define USB_ERREN_OWNERREN_SHIFT                 (6U)
/*! OWNERREN - OWNERR Interrupt Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define USB_ERREN_OWNERREN(x)                    (((uint8_t)(((uint8_t)(x)) << USB_ERREN_OWNERREN_SHIFT)) & USB_ERREN_OWNERREN_MASK)

#define USB_ERREN_BTSERREN_MASK                  (0x80U)
#define USB_ERREN_BTSERREN_SHIFT                 (7U)
/*! BTSERREN - BTSERR (Bit Stuff Error) Interrupt Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define USB_ERREN_BTSERREN(x)                    (((uint8_t)(((uint8_t)(x)) << USB_ERREN_BTSERREN_SHIFT)) & USB_ERREN_BTSERREN_MASK)
/*! @} */

/*! @name STAT - Status */
/*! @{ */

#define USB_STAT_ODD_MASK                        (0x4U)
#define USB_STAT_ODD_SHIFT                       (2U)
/*! ODD - Odd Bank
 *  0b0..Not in the odd bank
 *  0b1..In the odd bank
 */
#define USB_STAT_ODD(x)                          (((uint8_t)(((uint8_t)(x)) << USB_STAT_ODD_SHIFT)) & USB_STAT_ODD_MASK)

#define USB_STAT_TX_MASK                         (0x8U)
#define USB_STAT_TX_SHIFT                        (3U)
/*! TX - Transmit Indicator
 *  0b0..Receive
 *  0b1..Transmit
 */
#define USB_STAT_TX(x)                           (((uint8_t)(((uint8_t)(x)) << USB_STAT_TX_SHIFT)) & USB_STAT_TX_MASK)

#define USB_STAT_ENDP_MASK                       (0xF0U)
#define USB_STAT_ENDP_SHIFT                      (4U)
/*! ENDP - Endpoint address */
#define USB_STAT_ENDP(x)                         (((uint8_t)(((uint8_t)(x)) << USB_STAT_ENDP_SHIFT)) & USB_STAT_ENDP_MASK)
/*! @} */

/*! @name CTL - Control */
/*! @{ */

#define USB_CTL_USBENSOFEN_MASK                  (0x1U)
#define USB_CTL_USBENSOFEN_SHIFT                 (0U)
/*! USBENSOFEN - USB Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define USB_CTL_USBENSOFEN(x)                    (((uint8_t)(((uint8_t)(x)) << USB_CTL_USBENSOFEN_SHIFT)) & USB_CTL_USBENSOFEN_MASK)

#define USB_CTL_ODDRST_MASK                      (0x2U)
#define USB_CTL_ODDRST_SHIFT                     (1U)
/*! ODDRST - Odd Reset */
#define USB_CTL_ODDRST(x)                        (((uint8_t)(((uint8_t)(x)) << USB_CTL_ODDRST_SHIFT)) & USB_CTL_ODDRST_MASK)

#define USB_CTL_RESUME_MASK                      (0x4U)
#define USB_CTL_RESUME_SHIFT                     (2U)
/*! RESUME - Resume */
#define USB_CTL_RESUME(x)                        (((uint8_t)(((uint8_t)(x)) << USB_CTL_RESUME_SHIFT)) & USB_CTL_RESUME_MASK)

#define USB_CTL_TXSUSPENDTOKENBUSY_MASK          (0x20U)
#define USB_CTL_TXSUSPENDTOKENBUSY_SHIFT         (5U)
/*! TXSUSPENDTOKENBUSY - TXD Suspend And Token Busy */
#define USB_CTL_TXSUSPENDTOKENBUSY(x)            (((uint8_t)(((uint8_t)(x)) << USB_CTL_TXSUSPENDTOKENBUSY_SHIFT)) & USB_CTL_TXSUSPENDTOKENBUSY_MASK)

#define USB_CTL_SE0_MASK                         (0x40U)
#define USB_CTL_SE0_SHIFT                        (6U)
/*! SE0 - Live USB Single-Ended Zero signal */
#define USB_CTL_SE0(x)                           (((uint8_t)(((uint8_t)(x)) << USB_CTL_SE0_SHIFT)) & USB_CTL_SE0_MASK)
/*! @} */

/*! @name ADDR - Address */
/*! @{ */

#define USB_ADDR_ADDR_MASK                       (0x7FU)
#define USB_ADDR_ADDR_SHIFT                      (0U)
/*! ADDR - USB Address */
#define USB_ADDR_ADDR(x)                         (((uint8_t)(((uint8_t)(x)) << USB_ADDR_ADDR_SHIFT)) & USB_ADDR_ADDR_MASK)
/*! @} */

/*! @name BDTPAGE1 - BDT Page 1 */
/*! @{ */

#define USB_BDTPAGE1_BDTBA_MASK                  (0xFEU)
#define USB_BDTPAGE1_BDTBA_SHIFT                 (1U)
/*! BDTBA - BDT Base Address */
#define USB_BDTPAGE1_BDTBA(x)                    (((uint8_t)(((uint8_t)(x)) << USB_BDTPAGE1_BDTBA_SHIFT)) & USB_BDTPAGE1_BDTBA_MASK)
/*! @} */

/*! @name FRMNUML - Frame Number Register Low */
/*! @{ */

#define USB_FRMNUML_FRM_MASK                     (0xFFU)
#define USB_FRMNUML_FRM_SHIFT                    (0U)
/*! FRM - Frame Number, Bits 0-7 */
#define USB_FRMNUML_FRM(x)                       (((uint8_t)(((uint8_t)(x)) << USB_FRMNUML_FRM_SHIFT)) & USB_FRMNUML_FRM_MASK)
/*! @} */

/*! @name FRMNUMH - Frame Number Register High */
/*! @{ */

#define USB_FRMNUMH_FRM_MASK                     (0x7U)
#define USB_FRMNUMH_FRM_SHIFT                    (0U)
/*! FRM - Frame Number, Bits 8-10 */
#define USB_FRMNUMH_FRM(x)                       (((uint8_t)(((uint8_t)(x)) << USB_FRMNUMH_FRM_SHIFT)) & USB_FRMNUMH_FRM_MASK)
/*! @} */

/*! @name BDTPAGE2 - BDT Page 2 */
/*! @{ */

#define USB_BDTPAGE2_BDTBA_MASK                  (0xFFU)
#define USB_BDTPAGE2_BDTBA_SHIFT                 (0U)
/*! BDTBA - BDT Base Address */
#define USB_BDTPAGE2_BDTBA(x)                    (((uint8_t)(((uint8_t)(x)) << USB_BDTPAGE2_BDTBA_SHIFT)) & USB_BDTPAGE2_BDTBA_MASK)
/*! @} */

/*! @name BDTPAGE3 - BDT Page 3 */
/*! @{ */

#define USB_BDTPAGE3_BDTBA_MASK                  (0xFFU)
#define USB_BDTPAGE3_BDTBA_SHIFT                 (0U)
/*! BDTBA - BDT Base Address */
#define USB_BDTPAGE3_BDTBA(x)                    (((uint8_t)(((uint8_t)(x)) << USB_BDTPAGE3_BDTBA_SHIFT)) & USB_BDTPAGE3_BDTBA_MASK)
/*! @} */

/*! @name ENDPT - Endpoint Control */
/*! @{ */

#define USB_ENDPT_EPHSHK_MASK                    (0x1U)
#define USB_ENDPT_EPHSHK_SHIFT                   (0U)
/*! EPHSHK - Endpoint Handshaking Enable */
#define USB_ENDPT_EPHSHK(x)                      (((uint8_t)(((uint8_t)(x)) << USB_ENDPT_EPHSHK_SHIFT)) & USB_ENDPT_EPHSHK_MASK)

#define USB_ENDPT_EPSTALL_MASK                   (0x2U)
#define USB_ENDPT_EPSTALL_SHIFT                  (1U)
/*! EPSTALL - Endpoint Stalled */
#define USB_ENDPT_EPSTALL(x)                     (((uint8_t)(((uint8_t)(x)) << USB_ENDPT_EPSTALL_SHIFT)) & USB_ENDPT_EPSTALL_MASK)

#define USB_ENDPT_EPTXEN_MASK                    (0x4U)
#define USB_ENDPT_EPTXEN_SHIFT                   (2U)
/*! EPTXEN - Endpoint for TX transfers enable */
#define USB_ENDPT_EPTXEN(x)                      (((uint8_t)(((uint8_t)(x)) << USB_ENDPT_EPTXEN_SHIFT)) & USB_ENDPT_EPTXEN_MASK)

#define USB_ENDPT_EPRXEN_MASK                    (0x8U)
#define USB_ENDPT_EPRXEN_SHIFT                   (3U)
/*! EPRXEN - Endpoint for RX transfers enable */
#define USB_ENDPT_EPRXEN(x)                      (((uint8_t)(((uint8_t)(x)) << USB_ENDPT_EPRXEN_SHIFT)) & USB_ENDPT_EPRXEN_MASK)

#define USB_ENDPT_EPCTLDIS_MASK                  (0x10U)
#define USB_ENDPT_EPCTLDIS_SHIFT                 (4U)
/*! EPCTLDIS - Control Transfer Disable
 *  0b0..Enable
 *  0b1..Disable
 */
#define USB_ENDPT_EPCTLDIS(x)                    (((uint8_t)(((uint8_t)(x)) << USB_ENDPT_EPCTLDIS_SHIFT)) & USB_ENDPT_EPCTLDIS_MASK)
/*! @} */

/* The count of USB_ENDPT */
#define USB_ENDPT_COUNT                          (16U)

/*! @name USBCTRL - USB Control */
/*! @{ */

#define USB_USBCTRL_DPDM_LANE_REVERSE_MASK       (0x4U)
#define USB_USBCTRL_DPDM_LANE_REVERSE_SHIFT      (2U)
/*! DPDM_LANE_REVERSE - DP and DM Lane Reversal Control
 *  0b0..Standard USB DP and DM package pin assignment
 *  0b1..Reverse roles of USB DP and DM package pins
 */
#define USB_USBCTRL_DPDM_LANE_REVERSE(x)         (((uint8_t)(((uint8_t)(x)) << USB_USBCTRL_DPDM_LANE_REVERSE_SHIFT)) & USB_USBCTRL_DPDM_LANE_REVERSE_MASK)

#define USB_USBCTRL_UARTSEL_MASK                 (0x10U)
#define USB_USBCTRL_UARTSEL_SHIFT                (4U)
/*! UARTSEL - UART Select
 *  0b0..USB DP and DM external package pins are used for USB signaling.
 *  0b1..USB DP and DM external package pins are used for UART signaling.
 */
#define USB_USBCTRL_UARTSEL(x)                   (((uint8_t)(((uint8_t)(x)) << USB_USBCTRL_UARTSEL_SHIFT)) & USB_USBCTRL_UARTSEL_MASK)

#define USB_USBCTRL_UARTCHLS_MASK                (0x20U)
#define USB_USBCTRL_UARTCHLS_SHIFT               (5U)
/*! UARTCHLS - UART Signal Channel Select
 *  0b0..USB DP and DM signals are used as UART TX/RX.
 *  0b1..USB DP and DM signals are used as UART RX/TX.
 */
#define USB_USBCTRL_UARTCHLS(x)                  (((uint8_t)(((uint8_t)(x)) << USB_USBCTRL_UARTCHLS_SHIFT)) & USB_USBCTRL_UARTCHLS_MASK)

#define USB_USBCTRL_PDE_MASK                     (0x40U)
#define USB_USBCTRL_PDE_SHIFT                    (6U)
/*! PDE - Pulldown Enable
 *  0b0..Disable on D+ and D-
 *  0b1..Enable on D+ and D-
 */
#define USB_USBCTRL_PDE(x)                       (((uint8_t)(((uint8_t)(x)) << USB_USBCTRL_PDE_SHIFT)) & USB_USBCTRL_PDE_MASK)

#define USB_USBCTRL_SUSP_MASK                    (0x80U)
#define USB_USBCTRL_SUSP_SHIFT                   (7U)
/*! SUSP - Suspend
 *  0b0..Not in Suspend state
 *  0b1..In Suspend state
 */
#define USB_USBCTRL_SUSP(x)                      (((uint8_t)(((uint8_t)(x)) << USB_USBCTRL_SUSP_SHIFT)) & USB_USBCTRL_SUSP_MASK)
/*! @} */

/*! @name OBSERVE - USB OTG Observe */
/*! @{ */

#define USB_OBSERVE_DMPD_MASK                    (0x10U)
#define USB_OBSERVE_DMPD_SHIFT                   (4U)
/*! DMPD - D- Pulldown
 *  0b0..Disabled
 *  0b1..Enabled
 */
#define USB_OBSERVE_DMPD(x)                      (((uint8_t)(((uint8_t)(x)) << USB_OBSERVE_DMPD_SHIFT)) & USB_OBSERVE_DMPD_MASK)

#define USB_OBSERVE_DPPD_MASK                    (0x40U)
#define USB_OBSERVE_DPPD_SHIFT                   (6U)
/*! DPPD - D+ Pulldown
 *  0b0..Disabled
 *  0b1..Enabled
 */
#define USB_OBSERVE_DPPD(x)                      (((uint8_t)(((uint8_t)(x)) << USB_OBSERVE_DPPD_SHIFT)) & USB_OBSERVE_DPPD_MASK)

#define USB_OBSERVE_DPPU_MASK                    (0x80U)
#define USB_OBSERVE_DPPU_SHIFT                   (7U)
/*! DPPU - D+ Pullup
 *  0b0..Disabled
 *  0b1..Enabled
 */
#define USB_OBSERVE_DPPU(x)                      (((uint8_t)(((uint8_t)(x)) << USB_OBSERVE_DPPU_SHIFT)) & USB_OBSERVE_DPPU_MASK)
/*! @} */

/*! @name CONTROL - USB OTG Control */
/*! @{ */

#define USB_CONTROL_VBUS_SOURCE_SEL_MASK         (0x1U)
#define USB_CONTROL_VBUS_SOURCE_SEL_SHIFT        (0U)
/*! VBUS_SOURCE_SEL - VBUS Monitoring Source Select
 *  0b0..Reserved
 *  0b1..Resistive divider attached to a GPIO pin
 */
#define USB_CONTROL_VBUS_SOURCE_SEL(x)           (((uint8_t)(((uint8_t)(x)) << USB_CONTROL_VBUS_SOURCE_SEL_SHIFT)) & USB_CONTROL_VBUS_SOURCE_SEL_MASK)

#define USB_CONTROL_SESS_VLD_MASK                (0x2U)
#define USB_CONTROL_SESS_VLD_SHIFT               (1U)
/*! SESS_VLD - VBUS Session Valid status
 *  0b0..Below
 *  0b1..Above
 */
#define USB_CONTROL_SESS_VLD(x)                  (((uint8_t)(((uint8_t)(x)) << USB_CONTROL_SESS_VLD_SHIFT)) & USB_CONTROL_SESS_VLD_MASK)

#define USB_CONTROL_DPPULLUPNONOTG_MASK          (0x10U)
#define USB_CONTROL_DPPULLUPNONOTG_SHIFT         (4U)
/*! DPPULLUPNONOTG - DP Pullup in Non-OTG Device Mode
 *  0b0..Disable
 *  0b1..Enabled
 */
#define USB_CONTROL_DPPULLUPNONOTG(x)            (((uint8_t)(((uint8_t)(x)) << USB_CONTROL_DPPULLUPNONOTG_SHIFT)) & USB_CONTROL_DPPULLUPNONOTG_MASK)
/*! @} */

/*! @name USBTRC0 - USB Transceiver Control 0 */
/*! @{ */

#define USB_USBTRC0_USB_RESUME_INT_MASK          (0x1U)
#define USB_USBTRC0_USB_RESUME_INT_SHIFT         (0U)
/*! USB_RESUME_INT - USB Asynchronous Interrupt
 *  0b0..Not generated
 *  0b1..Generated because of the USB asynchronous interrupt
 */
#define USB_USBTRC0_USB_RESUME_INT(x)            (((uint8_t)(((uint8_t)(x)) << USB_USBTRC0_USB_RESUME_INT_SHIFT)) & USB_USBTRC0_USB_RESUME_INT_MASK)

#define USB_USBTRC0_SYNC_DET_MASK                (0x2U)
#define USB_USBTRC0_SYNC_DET_SHIFT               (1U)
/*! SYNC_DET - Synchronous USB Interrupt Detect
 *  0b0..Not detected
 *  0b1..Detected
 */
#define USB_USBTRC0_SYNC_DET(x)                  (((uint8_t)(((uint8_t)(x)) << USB_USBTRC0_SYNC_DET_SHIFT)) & USB_USBTRC0_SYNC_DET_MASK)

#define USB_USBTRC0_USB_CLK_RECOVERY_INT_MASK    (0x4U)
#define USB_USBTRC0_USB_CLK_RECOVERY_INT_SHIFT   (2U)
/*! USB_CLK_RECOVERY_INT - Combined USB Clock Recovery interrupt status */
#define USB_USBTRC0_USB_CLK_RECOVERY_INT(x)      (((uint8_t)(((uint8_t)(x)) << USB_USBTRC0_USB_CLK_RECOVERY_INT_SHIFT)) & USB_USBTRC0_USB_CLK_RECOVERY_INT_MASK)

#define USB_USBTRC0_VREDG_DET_MASK               (0x8U)
#define USB_USBTRC0_VREDG_DET_SHIFT              (3U)
/*! VREDG_DET - VREGIN Rising Edge Interrupt Detect
 *  0b0..Not detected
 *  0b1..Detected
 */
#define USB_USBTRC0_VREDG_DET(x)                 (((uint8_t)(((uint8_t)(x)) << USB_USBTRC0_VREDG_DET_SHIFT)) & USB_USBTRC0_VREDG_DET_MASK)

#define USB_USBTRC0_VFEDG_DET_MASK               (0x10U)
#define USB_USBTRC0_VFEDG_DET_SHIFT              (4U)
/*! VFEDG_DET - VREGIN Falling Edge Interrupt Detect
 *  0b0..Not detected
 *  0b1..Detected
 */
#define USB_USBTRC0_VFEDG_DET(x)                 (((uint8_t)(((uint8_t)(x)) << USB_USBTRC0_VFEDG_DET_SHIFT)) & USB_USBTRC0_VFEDG_DET_MASK)

#define USB_USBTRC0_USBRESMEN_MASK               (0x20U)
#define USB_USBTRC0_USBRESMEN_SHIFT              (5U)
/*! USBRESMEN - Asynchronous Resume Interrupt Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define USB_USBTRC0_USBRESMEN(x)                 (((uint8_t)(((uint8_t)(x)) << USB_USBTRC0_USBRESMEN_SHIFT)) & USB_USBTRC0_USBRESMEN_MASK)

#define USB_USBTRC0_VREGIN_STS_MASK              (0x40U)
#define USB_USBTRC0_VREGIN_STS_SHIFT             (6U)
/*! VREGIN_STS - VREGIN Status */
#define USB_USBTRC0_VREGIN_STS(x)                (((uint8_t)(((uint8_t)(x)) << USB_USBTRC0_VREGIN_STS_SHIFT)) & USB_USBTRC0_VREGIN_STS_MASK)

#define USB_USBTRC0_USBRESET_MASK                (0x80U)
#define USB_USBTRC0_USBRESET_SHIFT               (7U)
/*! USBRESET - USB Reset
 *  0b0..Normal USBFS operation
 *  0b1..Returns USBFS to its reset state
 */
#define USB_USBTRC0_USBRESET(x)                  (((uint8_t)(((uint8_t)(x)) << USB_USBTRC0_USBRESET_SHIFT)) & USB_USBTRC0_USBRESET_MASK)
/*! @} */

/*! @name MISCCTRL - Miscellaneous Control */
/*! @{ */

#define USB_MISCCTRL_OWNERRISODIS_MASK           (0x4U)
#define USB_MISCCTRL_OWNERRISODIS_SHIFT          (2U)
/*! OWNERRISODIS - OWN Error Detect for ISO IN and ISO OUT Disable
 *  0b0..Enable
 *  0b1..Disable
 */
#define USB_MISCCTRL_OWNERRISODIS(x)             (((uint8_t)(((uint8_t)(x)) << USB_MISCCTRL_OWNERRISODIS_SHIFT)) & USB_MISCCTRL_OWNERRISODIS_MASK)

#define USB_MISCCTRL_VREDG_EN_MASK               (0x8U)
#define USB_MISCCTRL_VREDG_EN_SHIFT              (3U)
/*! VREDG_EN - VREGIN Rising Edge Interrupt Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define USB_MISCCTRL_VREDG_EN(x)                 (((uint8_t)(((uint8_t)(x)) << USB_MISCCTRL_VREDG_EN_SHIFT)) & USB_MISCCTRL_VREDG_EN_MASK)

#define USB_MISCCTRL_VFEDG_EN_MASK               (0x10U)
#define USB_MISCCTRL_VFEDG_EN_SHIFT              (4U)
/*! VFEDG_EN - VREGIN Falling Edge Interrupt Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define USB_MISCCTRL_VFEDG_EN(x)                 (((uint8_t)(((uint8_t)(x)) << USB_MISCCTRL_VFEDG_EN_SHIFT)) & USB_MISCCTRL_VFEDG_EN_MASK)

#define USB_MISCCTRL_STL_ADJ_EN_MASK             (0x80U)
#define USB_MISCCTRL_STL_ADJ_EN_SHIFT            (7U)
/*! STL_ADJ_EN - USB Peripheral Mode Stall Adjust Enable
 *  0b0..If ENDPTn[END_STALL] = 1, both IN and OUT directions for the associated endpoint stalls.
 *  0b1..If ENDPTn[END_STALL] = 1, the STALL_xx_DIS registers control which directions for the associated endpoint stalls.
 */
#define USB_MISCCTRL_STL_ADJ_EN(x)               (((uint8_t)(((uint8_t)(x)) << USB_MISCCTRL_STL_ADJ_EN_SHIFT)) & USB_MISCCTRL_STL_ADJ_EN_MASK)
/*! @} */

/*! @name STALL_IL_DIS - Peripheral Mode Stall Disable for Endpoints 7 to 0 in IN Direction */
/*! @{ */

#define USB_STALL_IL_DIS_STALL_I_DIS0_MASK       (0x1U)
#define USB_STALL_IL_DIS_STALL_I_DIS0_SHIFT      (0U)
/*! STALL_I_DIS0 - Disable Endpoint 0 IN Direction
 *  0b0..Enable
 *  0b1..Disable
 */
#define USB_STALL_IL_DIS_STALL_I_DIS0(x)         (((uint8_t)(((uint8_t)(x)) << USB_STALL_IL_DIS_STALL_I_DIS0_SHIFT)) & USB_STALL_IL_DIS_STALL_I_DIS0_MASK)

#define USB_STALL_IL_DIS_STALL_I_DIS1_MASK       (0x2U)
#define USB_STALL_IL_DIS_STALL_I_DIS1_SHIFT      (1U)
/*! STALL_I_DIS1 - Disable Endpoint 1 IN Direction
 *  0b0..Enable
 *  0b1..Disable
 */
#define USB_STALL_IL_DIS_STALL_I_DIS1(x)         (((uint8_t)(((uint8_t)(x)) << USB_STALL_IL_DIS_STALL_I_DIS1_SHIFT)) & USB_STALL_IL_DIS_STALL_I_DIS1_MASK)

#define USB_STALL_IL_DIS_STALL_I_DIS2_MASK       (0x4U)
#define USB_STALL_IL_DIS_STALL_I_DIS2_SHIFT      (2U)
/*! STALL_I_DIS2 - Disable Endpoint 2 IN Direction
 *  0b0..Enable
 *  0b1..Disable
 */
#define USB_STALL_IL_DIS_STALL_I_DIS2(x)         (((uint8_t)(((uint8_t)(x)) << USB_STALL_IL_DIS_STALL_I_DIS2_SHIFT)) & USB_STALL_IL_DIS_STALL_I_DIS2_MASK)

#define USB_STALL_IL_DIS_STALL_I_DIS3_MASK       (0x8U)
#define USB_STALL_IL_DIS_STALL_I_DIS3_SHIFT      (3U)
/*! STALL_I_DIS3 - Disable Endpoint 3 IN Direction
 *  0b0..Enable
 *  0b1..Disable
 */
#define USB_STALL_IL_DIS_STALL_I_DIS3(x)         (((uint8_t)(((uint8_t)(x)) << USB_STALL_IL_DIS_STALL_I_DIS3_SHIFT)) & USB_STALL_IL_DIS_STALL_I_DIS3_MASK)

#define USB_STALL_IL_DIS_STALL_I_DIS4_MASK       (0x10U)
#define USB_STALL_IL_DIS_STALL_I_DIS4_SHIFT      (4U)
/*! STALL_I_DIS4 - Disable Endpoint 4 IN Direction
 *  0b0..Enable
 *  0b1..Disable
 */
#define USB_STALL_IL_DIS_STALL_I_DIS4(x)         (((uint8_t)(((uint8_t)(x)) << USB_STALL_IL_DIS_STALL_I_DIS4_SHIFT)) & USB_STALL_IL_DIS_STALL_I_DIS4_MASK)

#define USB_STALL_IL_DIS_STALL_I_DIS5_MASK       (0x20U)
#define USB_STALL_IL_DIS_STALL_I_DIS5_SHIFT      (5U)
/*! STALL_I_DIS5 - Disable Endpoint 5 IN Direction
 *  0b0..Enable
 *  0b1..Disable
 */
#define USB_STALL_IL_DIS_STALL_I_DIS5(x)         (((uint8_t)(((uint8_t)(x)) << USB_STALL_IL_DIS_STALL_I_DIS5_SHIFT)) & USB_STALL_IL_DIS_STALL_I_DIS5_MASK)

#define USB_STALL_IL_DIS_STALL_I_DIS6_MASK       (0x40U)
#define USB_STALL_IL_DIS_STALL_I_DIS6_SHIFT      (6U)
/*! STALL_I_DIS6 - Disable Endpoint 6 IN Direction
 *  0b0..Enable
 *  0b1..Disable
 */
#define USB_STALL_IL_DIS_STALL_I_DIS6(x)         (((uint8_t)(((uint8_t)(x)) << USB_STALL_IL_DIS_STALL_I_DIS6_SHIFT)) & USB_STALL_IL_DIS_STALL_I_DIS6_MASK)

#define USB_STALL_IL_DIS_STALL_I_DIS7_MASK       (0x80U)
#define USB_STALL_IL_DIS_STALL_I_DIS7_SHIFT      (7U)
/*! STALL_I_DIS7 - Disable Endpoint 7 IN Direction
 *  0b0..Enable
 *  0b1..Disable
 */
#define USB_STALL_IL_DIS_STALL_I_DIS7(x)         (((uint8_t)(((uint8_t)(x)) << USB_STALL_IL_DIS_STALL_I_DIS7_SHIFT)) & USB_STALL_IL_DIS_STALL_I_DIS7_MASK)
/*! @} */

/*! @name STALL_IH_DIS - Peripheral Mode Stall Disable for Endpoints 15 to 8 in IN Direction */
/*! @{ */

#define USB_STALL_IH_DIS_STALL_I_DIS8_MASK       (0x1U)
#define USB_STALL_IH_DIS_STALL_I_DIS8_SHIFT      (0U)
/*! STALL_I_DIS8 - Disable Endpoint 8 IN Direction
 *  0b0..Enable
 *  0b1..Disable
 */
#define USB_STALL_IH_DIS_STALL_I_DIS8(x)         (((uint8_t)(((uint8_t)(x)) << USB_STALL_IH_DIS_STALL_I_DIS8_SHIFT)) & USB_STALL_IH_DIS_STALL_I_DIS8_MASK)

#define USB_STALL_IH_DIS_STALL_I_DIS9_MASK       (0x2U)
#define USB_STALL_IH_DIS_STALL_I_DIS9_SHIFT      (1U)
/*! STALL_I_DIS9 - Disable Endpoint 9 IN Direction
 *  0b0..Enable
 *  0b1..Disable
 */
#define USB_STALL_IH_DIS_STALL_I_DIS9(x)         (((uint8_t)(((uint8_t)(x)) << USB_STALL_IH_DIS_STALL_I_DIS9_SHIFT)) & USB_STALL_IH_DIS_STALL_I_DIS9_MASK)

#define USB_STALL_IH_DIS_STALL_I_DIS10_MASK      (0x4U)
#define USB_STALL_IH_DIS_STALL_I_DIS10_SHIFT     (2U)
/*! STALL_I_DIS10 - Disable Endpoint 10 IN Direction
 *  0b0..Enable
 *  0b1..Disable
 */
#define USB_STALL_IH_DIS_STALL_I_DIS10(x)        (((uint8_t)(((uint8_t)(x)) << USB_STALL_IH_DIS_STALL_I_DIS10_SHIFT)) & USB_STALL_IH_DIS_STALL_I_DIS10_MASK)

#define USB_STALL_IH_DIS_STALL_I_DIS11_MASK      (0x8U)
#define USB_STALL_IH_DIS_STALL_I_DIS11_SHIFT     (3U)
/*! STALL_I_DIS11 - Disable Endpoint 11 IN Direction
 *  0b0..Enable
 *  0b1..Disable
 */
#define USB_STALL_IH_DIS_STALL_I_DIS11(x)        (((uint8_t)(((uint8_t)(x)) << USB_STALL_IH_DIS_STALL_I_DIS11_SHIFT)) & USB_STALL_IH_DIS_STALL_I_DIS11_MASK)

#define USB_STALL_IH_DIS_STALL_I_DIS12_MASK      (0x10U)
#define USB_STALL_IH_DIS_STALL_I_DIS12_SHIFT     (4U)
/*! STALL_I_DIS12 - Disable Endpoint 12 IN Direction
 *  0b0..Enable
 *  0b1..Disable
 */
#define USB_STALL_IH_DIS_STALL_I_DIS12(x)        (((uint8_t)(((uint8_t)(x)) << USB_STALL_IH_DIS_STALL_I_DIS12_SHIFT)) & USB_STALL_IH_DIS_STALL_I_DIS12_MASK)

#define USB_STALL_IH_DIS_STALL_I_DIS13_MASK      (0x20U)
#define USB_STALL_IH_DIS_STALL_I_DIS13_SHIFT     (5U)
/*! STALL_I_DIS13 - Disable Endpoint 13 IN Direction
 *  0b0..Enable
 *  0b1..Disable
 */
#define USB_STALL_IH_DIS_STALL_I_DIS13(x)        (((uint8_t)(((uint8_t)(x)) << USB_STALL_IH_DIS_STALL_I_DIS13_SHIFT)) & USB_STALL_IH_DIS_STALL_I_DIS13_MASK)

#define USB_STALL_IH_DIS_STALL_I_DIS14_MASK      (0x40U)
#define USB_STALL_IH_DIS_STALL_I_DIS14_SHIFT     (6U)
/*! STALL_I_DIS14 - Disable Endpoint 14 IN Direction
 *  0b0..Enable
 *  0b1..Disable
 */
#define USB_STALL_IH_DIS_STALL_I_DIS14(x)        (((uint8_t)(((uint8_t)(x)) << USB_STALL_IH_DIS_STALL_I_DIS14_SHIFT)) & USB_STALL_IH_DIS_STALL_I_DIS14_MASK)

#define USB_STALL_IH_DIS_STALL_I_DIS15_MASK      (0x80U)
#define USB_STALL_IH_DIS_STALL_I_DIS15_SHIFT     (7U)
/*! STALL_I_DIS15 - Disable Endpoint 15 IN Direction
 *  0b0..Enable
 *  0b1..Disable
 */
#define USB_STALL_IH_DIS_STALL_I_DIS15(x)        (((uint8_t)(((uint8_t)(x)) << USB_STALL_IH_DIS_STALL_I_DIS15_SHIFT)) & USB_STALL_IH_DIS_STALL_I_DIS15_MASK)
/*! @} */

/*! @name STALL_OL_DIS - Peripheral Mode Stall Disable for Endpoints 7 to 0 in OUT Direction */
/*! @{ */

#define USB_STALL_OL_DIS_STALL_O_DIS0_MASK       (0x1U)
#define USB_STALL_OL_DIS_STALL_O_DIS0_SHIFT      (0U)
/*! STALL_O_DIS0 - Disable Endpoint 0 OUT Direction
 *  0b0..Enable
 *  0b1..Disable
 */
#define USB_STALL_OL_DIS_STALL_O_DIS0(x)         (((uint8_t)(((uint8_t)(x)) << USB_STALL_OL_DIS_STALL_O_DIS0_SHIFT)) & USB_STALL_OL_DIS_STALL_O_DIS0_MASK)

#define USB_STALL_OL_DIS_STALL_O_DIS1_MASK       (0x2U)
#define USB_STALL_OL_DIS_STALL_O_DIS1_SHIFT      (1U)
/*! STALL_O_DIS1 - Disable Endpoint 1 OUT Direction
 *  0b0..Enable
 *  0b1..Disable
 */
#define USB_STALL_OL_DIS_STALL_O_DIS1(x)         (((uint8_t)(((uint8_t)(x)) << USB_STALL_OL_DIS_STALL_O_DIS1_SHIFT)) & USB_STALL_OL_DIS_STALL_O_DIS1_MASK)

#define USB_STALL_OL_DIS_STALL_O_DIS2_MASK       (0x4U)
#define USB_STALL_OL_DIS_STALL_O_DIS2_SHIFT      (2U)
/*! STALL_O_DIS2 - Disable Endpoint 2 OUT Direction
 *  0b0..Enable
 *  0b1..Disable
 */
#define USB_STALL_OL_DIS_STALL_O_DIS2(x)         (((uint8_t)(((uint8_t)(x)) << USB_STALL_OL_DIS_STALL_O_DIS2_SHIFT)) & USB_STALL_OL_DIS_STALL_O_DIS2_MASK)

#define USB_STALL_OL_DIS_STALL_O_DIS3_MASK       (0x8U)
#define USB_STALL_OL_DIS_STALL_O_DIS3_SHIFT      (3U)
/*! STALL_O_DIS3 - Disable Endpoint 3 OUT Direction
 *  0b0..Enable
 *  0b1..Disable
 */
#define USB_STALL_OL_DIS_STALL_O_DIS3(x)         (((uint8_t)(((uint8_t)(x)) << USB_STALL_OL_DIS_STALL_O_DIS3_SHIFT)) & USB_STALL_OL_DIS_STALL_O_DIS3_MASK)

#define USB_STALL_OL_DIS_STALL_O_DIS4_MASK       (0x10U)
#define USB_STALL_OL_DIS_STALL_O_DIS4_SHIFT      (4U)
/*! STALL_O_DIS4 - Disable Endpoint 4 OUT Direction
 *  0b0..Enable
 *  0b1..Disable
 */
#define USB_STALL_OL_DIS_STALL_O_DIS4(x)         (((uint8_t)(((uint8_t)(x)) << USB_STALL_OL_DIS_STALL_O_DIS4_SHIFT)) & USB_STALL_OL_DIS_STALL_O_DIS4_MASK)

#define USB_STALL_OL_DIS_STALL_O_DIS5_MASK       (0x20U)
#define USB_STALL_OL_DIS_STALL_O_DIS5_SHIFT      (5U)
/*! STALL_O_DIS5 - Disable Endpoint 5 OUT Direction
 *  0b0..Enable
 *  0b1..Disable
 */
#define USB_STALL_OL_DIS_STALL_O_DIS5(x)         (((uint8_t)(((uint8_t)(x)) << USB_STALL_OL_DIS_STALL_O_DIS5_SHIFT)) & USB_STALL_OL_DIS_STALL_O_DIS5_MASK)

#define USB_STALL_OL_DIS_STALL_O_DIS6_MASK       (0x40U)
#define USB_STALL_OL_DIS_STALL_O_DIS6_SHIFT      (6U)
/*! STALL_O_DIS6 - Disable Endpoint 6 OUT Direction
 *  0b0..Enable
 *  0b1..Disable
 */
#define USB_STALL_OL_DIS_STALL_O_DIS6(x)         (((uint8_t)(((uint8_t)(x)) << USB_STALL_OL_DIS_STALL_O_DIS6_SHIFT)) & USB_STALL_OL_DIS_STALL_O_DIS6_MASK)

#define USB_STALL_OL_DIS_STALL_O_DIS7_MASK       (0x80U)
#define USB_STALL_OL_DIS_STALL_O_DIS7_SHIFT      (7U)
/*! STALL_O_DIS7 - Disable Endpoint 7 OUT Direction
 *  0b0..Enable
 *  0b1..Disable
 */
#define USB_STALL_OL_DIS_STALL_O_DIS7(x)         (((uint8_t)(((uint8_t)(x)) << USB_STALL_OL_DIS_STALL_O_DIS7_SHIFT)) & USB_STALL_OL_DIS_STALL_O_DIS7_MASK)
/*! @} */

/*! @name STALL_OH_DIS - Peripheral Mode Stall Disable for Endpoints 15 to 8 in OUT Direction */
/*! @{ */

#define USB_STALL_OH_DIS_STALL_O_DIS8_MASK       (0x1U)
#define USB_STALL_OH_DIS_STALL_O_DIS8_SHIFT      (0U)
/*! STALL_O_DIS8 - Disable Endpoint 8 OUT Direction
 *  0b0..Enable
 *  0b1..Disable
 */
#define USB_STALL_OH_DIS_STALL_O_DIS8(x)         (((uint8_t)(((uint8_t)(x)) << USB_STALL_OH_DIS_STALL_O_DIS8_SHIFT)) & USB_STALL_OH_DIS_STALL_O_DIS8_MASK)

#define USB_STALL_OH_DIS_STALL_O_DIS9_MASK       (0x2U)
#define USB_STALL_OH_DIS_STALL_O_DIS9_SHIFT      (1U)
/*! STALL_O_DIS9 - Disable Endpoint 9 OUT Direction
 *  0b0..Enable
 *  0b1..Disable
 */
#define USB_STALL_OH_DIS_STALL_O_DIS9(x)         (((uint8_t)(((uint8_t)(x)) << USB_STALL_OH_DIS_STALL_O_DIS9_SHIFT)) & USB_STALL_OH_DIS_STALL_O_DIS9_MASK)

#define USB_STALL_OH_DIS_STALL_O_DIS10_MASK      (0x4U)
#define USB_STALL_OH_DIS_STALL_O_DIS10_SHIFT     (2U)
/*! STALL_O_DIS10 - Disable Endpoint 10 OUT Direction
 *  0b0..Enable
 *  0b1..Disable
 */
#define USB_STALL_OH_DIS_STALL_O_DIS10(x)        (((uint8_t)(((uint8_t)(x)) << USB_STALL_OH_DIS_STALL_O_DIS10_SHIFT)) & USB_STALL_OH_DIS_STALL_O_DIS10_MASK)

#define USB_STALL_OH_DIS_STALL_O_DIS11_MASK      (0x8U)
#define USB_STALL_OH_DIS_STALL_O_DIS11_SHIFT     (3U)
/*! STALL_O_DIS11 - Disable Endpoint 11 OUT Direction
 *  0b0..Enable
 *  0b1..Disable
 */
#define USB_STALL_OH_DIS_STALL_O_DIS11(x)        (((uint8_t)(((uint8_t)(x)) << USB_STALL_OH_DIS_STALL_O_DIS11_SHIFT)) & USB_STALL_OH_DIS_STALL_O_DIS11_MASK)

#define USB_STALL_OH_DIS_STALL_O_DIS12_MASK      (0x10U)
#define USB_STALL_OH_DIS_STALL_O_DIS12_SHIFT     (4U)
/*! STALL_O_DIS12 - Disable endpoint 12 OUT direction
 *  0b0..Enable
 *  0b1..Disable
 */
#define USB_STALL_OH_DIS_STALL_O_DIS12(x)        (((uint8_t)(((uint8_t)(x)) << USB_STALL_OH_DIS_STALL_O_DIS12_SHIFT)) & USB_STALL_OH_DIS_STALL_O_DIS12_MASK)

#define USB_STALL_OH_DIS_STALL_O_DIS13_MASK      (0x20U)
#define USB_STALL_OH_DIS_STALL_O_DIS13_SHIFT     (5U)
/*! STALL_O_DIS13 - Disable Endpoint 13 OUT Direction
 *  0b0..Enable
 *  0b1..Disable
 */
#define USB_STALL_OH_DIS_STALL_O_DIS13(x)        (((uint8_t)(((uint8_t)(x)) << USB_STALL_OH_DIS_STALL_O_DIS13_SHIFT)) & USB_STALL_OH_DIS_STALL_O_DIS13_MASK)

#define USB_STALL_OH_DIS_STALL_O_DIS14_MASK      (0x40U)
#define USB_STALL_OH_DIS_STALL_O_DIS14_SHIFT     (6U)
/*! STALL_O_DIS14 - Disable Endpoint 14 OUT Direction
 *  0b0..Enable
 *  0b1..Disable
 */
#define USB_STALL_OH_DIS_STALL_O_DIS14(x)        (((uint8_t)(((uint8_t)(x)) << USB_STALL_OH_DIS_STALL_O_DIS14_SHIFT)) & USB_STALL_OH_DIS_STALL_O_DIS14_MASK)

#define USB_STALL_OH_DIS_STALL_O_DIS15_MASK      (0x80U)
#define USB_STALL_OH_DIS_STALL_O_DIS15_SHIFT     (7U)
/*! STALL_O_DIS15 - Disable Endpoint 15 OUT Direction
 *  0b0..Enable
 *  0b1..Disable
 */
#define USB_STALL_OH_DIS_STALL_O_DIS15(x)        (((uint8_t)(((uint8_t)(x)) << USB_STALL_OH_DIS_STALL_O_DIS15_SHIFT)) & USB_STALL_OH_DIS_STALL_O_DIS15_MASK)
/*! @} */

/*! @name CLK_RECOVER_CTRL - USB Clock Recovery Control */
/*! @{ */

#define USB_CLK_RECOVER_CTRL_TRIM_INIT_VAL_SEL_MASK (0x8U)
#define USB_CLK_RECOVER_CTRL_TRIM_INIT_VAL_SEL_SHIFT (3U)
/*! TRIM_INIT_VAL_SEL - Selects the source for the initial FIRC trim fine value used after a reset.
 *  0b0..Mid-scale
 *  0b1..IFR
 */
#define USB_CLK_RECOVER_CTRL_TRIM_INIT_VAL_SEL(x) (((uint8_t)(((uint8_t)(x)) << USB_CLK_RECOVER_CTRL_TRIM_INIT_VAL_SEL_SHIFT)) & USB_CLK_RECOVER_CTRL_TRIM_INIT_VAL_SEL_MASK)

#define USB_CLK_RECOVER_CTRL_RESTART_IFRTRIM_EN_MASK (0x20U)
#define USB_CLK_RECOVER_CTRL_RESTART_IFRTRIM_EN_SHIFT (5U)
/*! RESTART_IFRTRIM_EN - Restart from IFR Trim Value
 *  0b0..Trim fine adjustment always works based on the previous updated trim fine value.
 *  0b1..Trim fine restarts from the IFR trim value whenever you detect bus_reset or bus_resume or deassert module enable.
 */
#define USB_CLK_RECOVER_CTRL_RESTART_IFRTRIM_EN(x) (((uint8_t)(((uint8_t)(x)) << USB_CLK_RECOVER_CTRL_RESTART_IFRTRIM_EN_SHIFT)) & USB_CLK_RECOVER_CTRL_RESTART_IFRTRIM_EN_MASK)

#define USB_CLK_RECOVER_CTRL_RESET_RESUME_ROUGH_EN_MASK (0x40U)
#define USB_CLK_RECOVER_CTRL_RESET_RESUME_ROUGH_EN_SHIFT (6U)
/*! RESET_RESUME_ROUGH_EN - Reset or Resume to Rough Phase Enable
 *  0b0..Always works in tracking phase after the first time rough phase, to track transition.
 *  0b1..Go back to rough stage whenever a bus reset or bus resume occurs.
 */
#define USB_CLK_RECOVER_CTRL_RESET_RESUME_ROUGH_EN(x) (((uint8_t)(((uint8_t)(x)) << USB_CLK_RECOVER_CTRL_RESET_RESUME_ROUGH_EN_SHIFT)) & USB_CLK_RECOVER_CTRL_RESET_RESUME_ROUGH_EN_MASK)

#define USB_CLK_RECOVER_CTRL_CLOCK_RECOVER_EN_MASK (0x80U)
#define USB_CLK_RECOVER_CTRL_CLOCK_RECOVER_EN_SHIFT (7U)
/*! CLOCK_RECOVER_EN - Crystal-Less USB Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define USB_CLK_RECOVER_CTRL_CLOCK_RECOVER_EN(x) (((uint8_t)(((uint8_t)(x)) << USB_CLK_RECOVER_CTRL_CLOCK_RECOVER_EN_SHIFT)) & USB_CLK_RECOVER_CTRL_CLOCK_RECOVER_EN_MASK)
/*! @} */

/*! @name CLK_RECOVER_IRC_EN - FIRC Oscillator Enable */
/*! @{ */

#define USB_CLK_RECOVER_IRC_EN_IRC_EN_MASK       (0x2U)
#define USB_CLK_RECOVER_IRC_EN_IRC_EN_SHIFT      (1U)
/*! IRC_EN - Fast IRC enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define USB_CLK_RECOVER_IRC_EN_IRC_EN(x)         (((uint8_t)(((uint8_t)(x)) << USB_CLK_RECOVER_IRC_EN_IRC_EN_SHIFT)) & USB_CLK_RECOVER_IRC_EN_IRC_EN_MASK)
/*! @} */

/*! @name CLK_RECOVER_INT_EN - Clock Recovery Combined Interrupt Enable */
/*! @{ */

#define USB_CLK_RECOVER_INT_EN_OVF_ERROR_EN_MASK (0x10U)
#define USB_CLK_RECOVER_INT_EN_OVF_ERROR_EN_SHIFT (4U)
/*! OVF_ERROR_EN - Overflow error interrupt enable
 *  0b0..The interrupt is masked
 *  0b1..The interrupt is enabled
 */
#define USB_CLK_RECOVER_INT_EN_OVF_ERROR_EN(x)   (((uint8_t)(((uint8_t)(x)) << USB_CLK_RECOVER_INT_EN_OVF_ERROR_EN_SHIFT)) & USB_CLK_RECOVER_INT_EN_OVF_ERROR_EN_MASK)
/*! @} */

/*! @name CLK_RECOVER_INT_STATUS - Clock Recovery Separated Interrupt Status */
/*! @{ */

#define USB_CLK_RECOVER_INT_STATUS_OVF_ERROR_MASK (0x10U)
#define USB_CLK_RECOVER_INT_STATUS_OVF_ERROR_SHIFT (4U)
/*! OVF_ERROR - Overflow Error Interrupt Status Flag
 *  0b0..Interrupt did not occur
 *  0b0..No effect
 *  0b1..Clear the flag
 *  0b1..Unmasked interrupt occurred
 */
#define USB_CLK_RECOVER_INT_STATUS_OVF_ERROR(x)  (((uint8_t)(((uint8_t)(x)) << USB_CLK_RECOVER_INT_STATUS_OVF_ERROR_SHIFT)) & USB_CLK_RECOVER_INT_STATUS_OVF_ERROR_MASK)
/*! @} */


/*!
 * @}
 */ /* end of group USB_Register_Masks */

/* Backward compatibility */
#define USBFS_IRQS                               USB_IRQS
#define USBFS_IRQHandler                         USB0_IRQHandler


/*!
 * @}
 */ /* end of group USB_Peripheral_Access_Layer */


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


#endif  /* PERI_USB_H_ */

