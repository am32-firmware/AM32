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
**         CMSIS Peripheral Access Layer for LPI2C
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
 * @file PERI_LPI2C.h
 * @version 2.0
 * @date 2024-10-29
 * @brief CMSIS Peripheral Access Layer for LPI2C
 *
 * CMSIS Peripheral Access Layer for LPI2C
 */

#if !defined(PERI_LPI2C_H_)
#define PERI_LPI2C_H_                            /**< Symbol preventing repeated inclusion */

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
   -- LPI2C Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LPI2C_Peripheral_Access_Layer LPI2C Peripheral Access Layer
 * @{
 */

/** LPI2C - Register Layout Typedef */
typedef struct {
  __I  uint32_t VERID;                             /**< Version ID, offset: 0x0 */
  __I  uint32_t PARAM;                             /**< Parameter, offset: 0x4 */
       uint8_t RESERVED_0[8];
  __IO uint32_t MCR;                               /**< Controller Control, offset: 0x10 */
  __IO uint32_t MSR;                               /**< Controller Status, offset: 0x14 */
  __IO uint32_t MIER;                              /**< Controller Interrupt Enable, offset: 0x18 */
  __IO uint32_t MDER;                              /**< Controller DMA Enable, offset: 0x1C */
  __IO uint32_t MCFGR0;                            /**< Controller Configuration 0, offset: 0x20 */
  __IO uint32_t MCFGR1;                            /**< Controller Configuration 1, offset: 0x24 */
  __IO uint32_t MCFGR2;                            /**< Controller Configuration 2, offset: 0x28 */
  __IO uint32_t MCFGR3;                            /**< Controller Configuration 3, offset: 0x2C */
       uint8_t RESERVED_1[16];
  __IO uint32_t MDMR;                              /**< Controller Data Match, offset: 0x40 */
       uint8_t RESERVED_2[4];
  __IO uint32_t MCCR0;                             /**< Controller Clock Configuration 0, offset: 0x48 */
       uint8_t RESERVED_3[4];
  __IO uint32_t MCCR1;                             /**< Controller Clock Configuration 1, offset: 0x50 */
       uint8_t RESERVED_4[4];
  __IO uint32_t MFCR;                              /**< Controller FIFO Control, offset: 0x58 */
  __I  uint32_t MFSR;                              /**< Controller FIFO Status, offset: 0x5C */
  __O  uint32_t MTDR;                              /**< Controller Transmit Data, offset: 0x60 */
       uint8_t RESERVED_5[12];
  __I  uint32_t MRDR;                              /**< Controller Receive Data, offset: 0x70 */
       uint8_t RESERVED_6[4];
  __I  uint32_t MRDROR;                            /**< Controller Receive Data Read Only, offset: 0x78 */
       uint8_t RESERVED_7[148];
  __IO uint32_t SCR;                               /**< Target Control, offset: 0x110 */
  __IO uint32_t SSR;                               /**< Target Status, offset: 0x114 */
  __IO uint32_t SIER;                              /**< Target Interrupt Enable, offset: 0x118 */
  __IO uint32_t SDER;                              /**< Target DMA Enable, offset: 0x11C */
  __IO uint32_t SCFGR0;                            /**< Target Configuration 0, offset: 0x120 */
  __IO uint32_t SCFGR1;                            /**< Target Configuration 1, offset: 0x124 */
  __IO uint32_t SCFGR2;                            /**< Target Configuration 2, offset: 0x128 */
       uint8_t RESERVED_8[20];
  __IO uint32_t SAMR;                              /**< Target Address Match, offset: 0x140 */
       uint8_t RESERVED_9[12];
  __I  uint32_t SASR;                              /**< Target Address Status, offset: 0x150 */
  __IO uint32_t STAR;                              /**< Target Transmit ACK, offset: 0x154 */
       uint8_t RESERVED_10[8];
  __O  uint32_t STDR;                              /**< Target Transmit Data, offset: 0x160 */
       uint8_t RESERVED_11[12];
  __I  uint32_t SRDR;                              /**< Target Receive Data, offset: 0x170 */
       uint8_t RESERVED_12[4];
  __I  uint32_t SRDROR;                            /**< Target Receive Data Read Only, offset: 0x178 */
} LPI2C_Type;

/* ----------------------------------------------------------------------------
   -- LPI2C Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LPI2C_Register_Masks LPI2C Register Masks
 * @{
 */

/*! @name VERID - Version ID */
/*! @{ */

#define LPI2C_VERID_FEATURE_MASK                 (0xFFFFU)
#define LPI2C_VERID_FEATURE_SHIFT                (0U)
/*! FEATURE - Feature Specification Number
 *  0b0000000000000010..Controller only, with standard feature set
 *  0b0000000000000011..Controller and target, with standard feature set
 */
#define LPI2C_VERID_FEATURE(x)                   (((uint32_t)(((uint32_t)(x)) << LPI2C_VERID_FEATURE_SHIFT)) & LPI2C_VERID_FEATURE_MASK)

#define LPI2C_VERID_MINOR_MASK                   (0xFF0000U)
#define LPI2C_VERID_MINOR_SHIFT                  (16U)
/*! MINOR - Minor Version Number */
#define LPI2C_VERID_MINOR(x)                     (((uint32_t)(((uint32_t)(x)) << LPI2C_VERID_MINOR_SHIFT)) & LPI2C_VERID_MINOR_MASK)

#define LPI2C_VERID_MAJOR_MASK                   (0xFF000000U)
#define LPI2C_VERID_MAJOR_SHIFT                  (24U)
/*! MAJOR - Major Version Number */
#define LPI2C_VERID_MAJOR(x)                     (((uint32_t)(((uint32_t)(x)) << LPI2C_VERID_MAJOR_SHIFT)) & LPI2C_VERID_MAJOR_MASK)
/*! @} */

/*! @name PARAM - Parameter */
/*! @{ */

#define LPI2C_PARAM_MTXFIFO_MASK                 (0xFU)
#define LPI2C_PARAM_MTXFIFO_SHIFT                (0U)
/*! MTXFIFO - Controller Transmit FIFO Size */
#define LPI2C_PARAM_MTXFIFO(x)                   (((uint32_t)(((uint32_t)(x)) << LPI2C_PARAM_MTXFIFO_SHIFT)) & LPI2C_PARAM_MTXFIFO_MASK)

#define LPI2C_PARAM_MRXFIFO_MASK                 (0xF00U)
#define LPI2C_PARAM_MRXFIFO_SHIFT                (8U)
/*! MRXFIFO - Controller Receive FIFO Size */
#define LPI2C_PARAM_MRXFIFO(x)                   (((uint32_t)(((uint32_t)(x)) << LPI2C_PARAM_MRXFIFO_SHIFT)) & LPI2C_PARAM_MRXFIFO_MASK)
/*! @} */

/*! @name MCR - Controller Control */
/*! @{ */

#define LPI2C_MCR_MEN_MASK                       (0x1U)
#define LPI2C_MCR_MEN_SHIFT                      (0U)
/*! MEN - Controller Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPI2C_MCR_MEN(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_MCR_MEN_SHIFT)) & LPI2C_MCR_MEN_MASK)

#define LPI2C_MCR_RST_MASK                       (0x2U)
#define LPI2C_MCR_RST_SHIFT                      (1U)
/*! RST - Software Reset
 *  0b0..No effect
 *  0b1..Reset
 */
#define LPI2C_MCR_RST(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_MCR_RST_SHIFT)) & LPI2C_MCR_RST_MASK)

#define LPI2C_MCR_DOZEN_MASK                     (0x4U)
#define LPI2C_MCR_DOZEN_SHIFT                    (2U)
/*! DOZEN - Doze Mode Enable
 *  0b0..Enable
 *  0b1..Disable
 */
#define LPI2C_MCR_DOZEN(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_MCR_DOZEN_SHIFT)) & LPI2C_MCR_DOZEN_MASK)

#define LPI2C_MCR_DBGEN_MASK                     (0x8U)
#define LPI2C_MCR_DBGEN_SHIFT                    (3U)
/*! DBGEN - Debug Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPI2C_MCR_DBGEN(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_MCR_DBGEN_SHIFT)) & LPI2C_MCR_DBGEN_MASK)

#define LPI2C_MCR_RTF_MASK                       (0x100U)
#define LPI2C_MCR_RTF_SHIFT                      (8U)
/*! RTF - Reset Transmit FIFO
 *  0b0..No effect
 *  0b1..Reset transmit FIFO
 */
#define LPI2C_MCR_RTF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_MCR_RTF_SHIFT)) & LPI2C_MCR_RTF_MASK)

#define LPI2C_MCR_RRF_MASK                       (0x200U)
#define LPI2C_MCR_RRF_SHIFT                      (9U)
/*! RRF - Reset Receive FIFO
 *  0b0..No effect
 *  0b1..Reset receive FIFO
 */
#define LPI2C_MCR_RRF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_MCR_RRF_SHIFT)) & LPI2C_MCR_RRF_MASK)
/*! @} */

/*! @name MSR - Controller Status */
/*! @{ */

#define LPI2C_MSR_TDF_MASK                       (0x1U)
#define LPI2C_MSR_TDF_SHIFT                      (0U)
/*! TDF - Transmit Data Flag
 *  0b0..Transmit data not requested
 *  0b1..Transmit data requested
 */
#define LPI2C_MSR_TDF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_MSR_TDF_SHIFT)) & LPI2C_MSR_TDF_MASK)

#define LPI2C_MSR_RDF_MASK                       (0x2U)
#define LPI2C_MSR_RDF_SHIFT                      (1U)
/*! RDF - Receive Data Flag
 *  0b0..Receive data not ready
 *  0b1..Receive data ready
 */
#define LPI2C_MSR_RDF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_MSR_RDF_SHIFT)) & LPI2C_MSR_RDF_MASK)

#define LPI2C_MSR_EPF_MASK                       (0x100U)
#define LPI2C_MSR_EPF_SHIFT                      (8U)
/*! EPF - End Packet Flag
 *  0b0..No Stop or repeated Start generated
 *  0b0..No effect
 *  0b1..Clear the flag
 *  0b1..Stop or repeated Start generated
 */
#define LPI2C_MSR_EPF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_MSR_EPF_SHIFT)) & LPI2C_MSR_EPF_MASK)

#define LPI2C_MSR_SDF_MASK                       (0x200U)
#define LPI2C_MSR_SDF_SHIFT                      (9U)
/*! SDF - Stop Detect Flag
 *  0b0..No Stop condition generated
 *  0b0..No effect
 *  0b1..Clear the flag
 *  0b1..Stop condition generated
 */
#define LPI2C_MSR_SDF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_MSR_SDF_SHIFT)) & LPI2C_MSR_SDF_MASK)

#define LPI2C_MSR_NDF_MASK                       (0x400U)
#define LPI2C_MSR_NDF_SHIFT                      (10U)
/*! NDF - NACK Detect Flag
 *  0b0..No effect
 *  0b0..No unexpected NACK detected
 *  0b1..Clear the flag
 *  0b1..Unexpected NACK detected
 */
#define LPI2C_MSR_NDF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_MSR_NDF_SHIFT)) & LPI2C_MSR_NDF_MASK)

#define LPI2C_MSR_ALF_MASK                       (0x800U)
#define LPI2C_MSR_ALF_SHIFT                      (11U)
/*! ALF - Arbitration Lost Flag
 *  0b0..Controller did not lose arbitration
 *  0b0..No effect
 *  0b1..Clear the flag
 *  0b1..Controller lost arbitration
 */
#define LPI2C_MSR_ALF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_MSR_ALF_SHIFT)) & LPI2C_MSR_ALF_MASK)

#define LPI2C_MSR_FEF_MASK                       (0x1000U)
#define LPI2C_MSR_FEF_SHIFT                      (12U)
/*! FEF - FIFO Error Flag
 *  0b0..No FIFO error
 *  0b0..No effect
 *  0b1..Clear the flag
 *  0b1..FIFO error
 */
#define LPI2C_MSR_FEF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_MSR_FEF_SHIFT)) & LPI2C_MSR_FEF_MASK)

#define LPI2C_MSR_PLTF_MASK                      (0x2000U)
#define LPI2C_MSR_PLTF_SHIFT                     (13U)
/*! PLTF - Pin Low Timeout Flag
 *  0b0..No effect
 *  0b0..Pin low timeout did not occur
 *  0b1..Clear the flag
 *  0b1..Pin low timeout occurred
 */
#define LPI2C_MSR_PLTF(x)                        (((uint32_t)(((uint32_t)(x)) << LPI2C_MSR_PLTF_SHIFT)) & LPI2C_MSR_PLTF_MASK)

#define LPI2C_MSR_DMF_MASK                       (0x4000U)
#define LPI2C_MSR_DMF_SHIFT                      (14U)
/*! DMF - Data Match Flag
 *  0b0..Matching data not received
 *  0b0..No effect
 *  0b1..Clear the flag
 *  0b1..Matching data received
 */
#define LPI2C_MSR_DMF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_MSR_DMF_SHIFT)) & LPI2C_MSR_DMF_MASK)

#define LPI2C_MSR_STF_MASK                       (0x8000U)
#define LPI2C_MSR_STF_SHIFT                      (15U)
/*! STF - Start Flag
 *  0b0..No effect
 *  0b0..Start condition not detected
 *  0b1..Clear the flag
 *  0b1..Start condition detected
 */
#define LPI2C_MSR_STF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_MSR_STF_SHIFT)) & LPI2C_MSR_STF_MASK)

#define LPI2C_MSR_MBF_MASK                       (0x1000000U)
#define LPI2C_MSR_MBF_SHIFT                      (24U)
/*! MBF - Controller Busy Flag
 *  0b0..Idle
 *  0b1..Busy
 */
#define LPI2C_MSR_MBF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_MSR_MBF_SHIFT)) & LPI2C_MSR_MBF_MASK)

#define LPI2C_MSR_BBF_MASK                       (0x2000000U)
#define LPI2C_MSR_BBF_SHIFT                      (25U)
/*! BBF - Bus Busy Flag
 *  0b0..Idle
 *  0b1..Busy
 */
#define LPI2C_MSR_BBF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_MSR_BBF_SHIFT)) & LPI2C_MSR_BBF_MASK)
/*! @} */

/*! @name MIER - Controller Interrupt Enable */
/*! @{ */

#define LPI2C_MIER_TDIE_MASK                     (0x1U)
#define LPI2C_MIER_TDIE_SHIFT                    (0U)
/*! TDIE - Transmit Data Interrupt Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPI2C_MIER_TDIE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_MIER_TDIE_SHIFT)) & LPI2C_MIER_TDIE_MASK)

#define LPI2C_MIER_RDIE_MASK                     (0x2U)
#define LPI2C_MIER_RDIE_SHIFT                    (1U)
/*! RDIE - Receive Data Interrupt Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPI2C_MIER_RDIE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_MIER_RDIE_SHIFT)) & LPI2C_MIER_RDIE_MASK)

#define LPI2C_MIER_EPIE_MASK                     (0x100U)
#define LPI2C_MIER_EPIE_SHIFT                    (8U)
/*! EPIE - End Packet Interrupt Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPI2C_MIER_EPIE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_MIER_EPIE_SHIFT)) & LPI2C_MIER_EPIE_MASK)

#define LPI2C_MIER_SDIE_MASK                     (0x200U)
#define LPI2C_MIER_SDIE_SHIFT                    (9U)
/*! SDIE - Stop Detect Interrupt Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPI2C_MIER_SDIE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_MIER_SDIE_SHIFT)) & LPI2C_MIER_SDIE_MASK)

#define LPI2C_MIER_NDIE_MASK                     (0x400U)
#define LPI2C_MIER_NDIE_SHIFT                    (10U)
/*! NDIE - NACK Detect Interrupt Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPI2C_MIER_NDIE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_MIER_NDIE_SHIFT)) & LPI2C_MIER_NDIE_MASK)

#define LPI2C_MIER_ALIE_MASK                     (0x800U)
#define LPI2C_MIER_ALIE_SHIFT                    (11U)
/*! ALIE - Arbitration Lost Interrupt Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPI2C_MIER_ALIE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_MIER_ALIE_SHIFT)) & LPI2C_MIER_ALIE_MASK)

#define LPI2C_MIER_FEIE_MASK                     (0x1000U)
#define LPI2C_MIER_FEIE_SHIFT                    (12U)
/*! FEIE - FIFO Error Interrupt Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPI2C_MIER_FEIE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_MIER_FEIE_SHIFT)) & LPI2C_MIER_FEIE_MASK)

#define LPI2C_MIER_PLTIE_MASK                    (0x2000U)
#define LPI2C_MIER_PLTIE_SHIFT                   (13U)
/*! PLTIE - Pin Low Timeout Interrupt Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPI2C_MIER_PLTIE(x)                      (((uint32_t)(((uint32_t)(x)) << LPI2C_MIER_PLTIE_SHIFT)) & LPI2C_MIER_PLTIE_MASK)

#define LPI2C_MIER_DMIE_MASK                     (0x4000U)
#define LPI2C_MIER_DMIE_SHIFT                    (14U)
/*! DMIE - Data Match Interrupt Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPI2C_MIER_DMIE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_MIER_DMIE_SHIFT)) & LPI2C_MIER_DMIE_MASK)

#define LPI2C_MIER_STIE_MASK                     (0x8000U)
#define LPI2C_MIER_STIE_SHIFT                    (15U)
/*! STIE - Start Interrupt Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPI2C_MIER_STIE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_MIER_STIE_SHIFT)) & LPI2C_MIER_STIE_MASK)
/*! @} */

/*! @name MDER - Controller DMA Enable */
/*! @{ */

#define LPI2C_MDER_TDDE_MASK                     (0x1U)
#define LPI2C_MDER_TDDE_SHIFT                    (0U)
/*! TDDE - Transmit Data DMA Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPI2C_MDER_TDDE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_MDER_TDDE_SHIFT)) & LPI2C_MDER_TDDE_MASK)

#define LPI2C_MDER_RDDE_MASK                     (0x2U)
#define LPI2C_MDER_RDDE_SHIFT                    (1U)
/*! RDDE - Receive Data DMA Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPI2C_MDER_RDDE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_MDER_RDDE_SHIFT)) & LPI2C_MDER_RDDE_MASK)
/*! @} */

/*! @name MCFGR0 - Controller Configuration 0 */
/*! @{ */

#define LPI2C_MCFGR0_HREN_MASK                   (0x1U)
#define LPI2C_MCFGR0_HREN_SHIFT                  (0U)
/*! HREN - Host Request Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPI2C_MCFGR0_HREN(x)                     (((uint32_t)(((uint32_t)(x)) << LPI2C_MCFGR0_HREN_SHIFT)) & LPI2C_MCFGR0_HREN_MASK)

#define LPI2C_MCFGR0_HRPOL_MASK                  (0x2U)
#define LPI2C_MCFGR0_HRPOL_SHIFT                 (1U)
/*! HRPOL - Host Request Polarity
 *  0b0..Active low
 *  0b1..Active high
 */
#define LPI2C_MCFGR0_HRPOL(x)                    (((uint32_t)(((uint32_t)(x)) << LPI2C_MCFGR0_HRPOL_SHIFT)) & LPI2C_MCFGR0_HRPOL_MASK)

#define LPI2C_MCFGR0_HRSEL_MASK                  (0x4U)
#define LPI2C_MCFGR0_HRSEL_SHIFT                 (2U)
/*! HRSEL - Host Request Select
 *  0b0..Host request input is pin HREQ
 *  0b1..Host request input is input trigger
 */
#define LPI2C_MCFGR0_HRSEL(x)                    (((uint32_t)(((uint32_t)(x)) << LPI2C_MCFGR0_HRSEL_SHIFT)) & LPI2C_MCFGR0_HRSEL_MASK)

#define LPI2C_MCFGR0_HRDIR_MASK                  (0x8U)
#define LPI2C_MCFGR0_HRDIR_SHIFT                 (3U)
/*! HRDIR - Host Request Direction
 *  0b0..HREQ pin is input (for LPI2C controller)
 *  0b1..HREQ pin is output (for LPI2C target)
 */
#define LPI2C_MCFGR0_HRDIR(x)                    (((uint32_t)(((uint32_t)(x)) << LPI2C_MCFGR0_HRDIR_SHIFT)) & LPI2C_MCFGR0_HRDIR_MASK)

#define LPI2C_MCFGR0_CIRFIFO_MASK                (0x100U)
#define LPI2C_MCFGR0_CIRFIFO_SHIFT               (8U)
/*! CIRFIFO - Circular FIFO Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPI2C_MCFGR0_CIRFIFO(x)                  (((uint32_t)(((uint32_t)(x)) << LPI2C_MCFGR0_CIRFIFO_SHIFT)) & LPI2C_MCFGR0_CIRFIFO_MASK)

#define LPI2C_MCFGR0_RDMO_MASK                   (0x200U)
#define LPI2C_MCFGR0_RDMO_SHIFT                  (9U)
/*! RDMO - Receive Data Match Only
 *  0b0..Received data is stored in the receive FIFO
 *  0b1..Received data is discarded unless MSR[DMF] is set
 */
#define LPI2C_MCFGR0_RDMO(x)                     (((uint32_t)(((uint32_t)(x)) << LPI2C_MCFGR0_RDMO_SHIFT)) & LPI2C_MCFGR0_RDMO_MASK)

#define LPI2C_MCFGR0_RELAX_MASK                  (0x10000U)
#define LPI2C_MCFGR0_RELAX_SHIFT                 (16U)
/*! RELAX - Relaxed Mode
 *  0b0..Normal transfer
 *  0b1..Relaxed transfer
 */
#define LPI2C_MCFGR0_RELAX(x)                    (((uint32_t)(((uint32_t)(x)) << LPI2C_MCFGR0_RELAX_SHIFT)) & LPI2C_MCFGR0_RELAX_MASK)

#define LPI2C_MCFGR0_ABORT_MASK                  (0x20000U)
#define LPI2C_MCFGR0_ABORT_SHIFT                 (17U)
/*! ABORT - Abort Transfer
 *  0b0..Normal transfer
 *  0b1..Abort existing transfer and do not start a new one
 */
#define LPI2C_MCFGR0_ABORT(x)                    (((uint32_t)(((uint32_t)(x)) << LPI2C_MCFGR0_ABORT_SHIFT)) & LPI2C_MCFGR0_ABORT_MASK)
/*! @} */

/*! @name MCFGR1 - Controller Configuration 1 */
/*! @{ */

#define LPI2C_MCFGR1_PRESCALE_MASK               (0x7U)
#define LPI2C_MCFGR1_PRESCALE_SHIFT              (0U)
/*! PRESCALE - Prescaler
 *  0b000..Divide by 1
 *  0b001..Divide by 2
 *  0b010..Divide by 4
 *  0b011..Divide by 8
 *  0b100..Divide by 16
 *  0b101..Divide by 32
 *  0b110..Divide by 64
 *  0b111..Divide by 128
 */
#define LPI2C_MCFGR1_PRESCALE(x)                 (((uint32_t)(((uint32_t)(x)) << LPI2C_MCFGR1_PRESCALE_SHIFT)) & LPI2C_MCFGR1_PRESCALE_MASK)

#define LPI2C_MCFGR1_AUTOSTOP_MASK               (0x100U)
#define LPI2C_MCFGR1_AUTOSTOP_SHIFT              (8U)
/*! AUTOSTOP - Automatic Stop Generation
 *  0b0..No effect
 *  0b1..Stop automatically generated
 */
#define LPI2C_MCFGR1_AUTOSTOP(x)                 (((uint32_t)(((uint32_t)(x)) << LPI2C_MCFGR1_AUTOSTOP_SHIFT)) & LPI2C_MCFGR1_AUTOSTOP_MASK)

#define LPI2C_MCFGR1_IGNACK_MASK                 (0x200U)
#define LPI2C_MCFGR1_IGNACK_SHIFT                (9U)
/*! IGNACK - Ignore NACK
 *  0b0..No effect
 *  0b1..Treat a received NACK as an ACK
 */
#define LPI2C_MCFGR1_IGNACK(x)                   (((uint32_t)(((uint32_t)(x)) << LPI2C_MCFGR1_IGNACK_SHIFT)) & LPI2C_MCFGR1_IGNACK_MASK)

#define LPI2C_MCFGR1_TIMECFG_MASK                (0x400U)
#define LPI2C_MCFGR1_TIMECFG_SHIFT               (10U)
/*! TIMECFG - Timeout Configuration
 *  0b0..SCL
 *  0b1..SCL or SDA
 */
#define LPI2C_MCFGR1_TIMECFG(x)                  (((uint32_t)(((uint32_t)(x)) << LPI2C_MCFGR1_TIMECFG_SHIFT)) & LPI2C_MCFGR1_TIMECFG_MASK)

#define LPI2C_MCFGR1_STOPCFG_MASK                (0x800U)
#define LPI2C_MCFGR1_STOPCFG_SHIFT               (11U)
/*! STOPCFG - Stop Configuration
 *  0b0..Any Stop condition
 *  0b1..Last Stop condition
 */
#define LPI2C_MCFGR1_STOPCFG(x)                  (((uint32_t)(((uint32_t)(x)) << LPI2C_MCFGR1_STOPCFG_SHIFT)) & LPI2C_MCFGR1_STOPCFG_MASK)

#define LPI2C_MCFGR1_STARTCFG_MASK               (0x1000U)
#define LPI2C_MCFGR1_STARTCFG_SHIFT              (12U)
/*! STARTCFG - Start Configuration
 *  0b0..Sets when both I2C bus and LPI2C controller are idle
 *  0b1..Sets when I2C bus is idle
 */
#define LPI2C_MCFGR1_STARTCFG(x)                 (((uint32_t)(((uint32_t)(x)) << LPI2C_MCFGR1_STARTCFG_SHIFT)) & LPI2C_MCFGR1_STARTCFG_MASK)

#define LPI2C_MCFGR1_MATCFG_MASK                 (0x70000U)
#define LPI2C_MCFGR1_MATCFG_SHIFT                (16U)
/*! MATCFG - Match Configuration
 *  0b000..Match is disabled
 *  0b001..Reserved
 *  0b010..Match is enabled: first data word equals MDMR[MATCH0] OR MDMR[MATCH1]
 *  0b011..Match is enabled: any data word equals MDMR[MATCH0] OR MDMR[MATCH1]
 *  0b100..Match is enabled: (first data word equals MDMR[MATCH0]) AND (second data word equals MDMR[MATCH1)
 *  0b101..Match is enabled: (any data word equals MDMR[MATCH0]) AND (next data word equals MDMR[MATCH1)
 *  0b110..Match is enabled: (first data word AND MDMR[MATCH1]) equals (MDMR[MATCH0] AND MDMR[MATCH1])
 *  0b111..Match is enabled: (any data word AND MDMR[MATCH1]) equals (MDMR[MATCH0] AND MDMR[MATCH1])
 */
#define LPI2C_MCFGR1_MATCFG(x)                   (((uint32_t)(((uint32_t)(x)) << LPI2C_MCFGR1_MATCFG_SHIFT)) & LPI2C_MCFGR1_MATCFG_MASK)

#define LPI2C_MCFGR1_PINCFG_MASK                 (0x7000000U)
#define LPI2C_MCFGR1_PINCFG_SHIFT                (24U)
/*! PINCFG - Pin Configuration
 *  0b000..Two-pin open drain mode
 *  0b001..Two-pin output only mode (Ultra-Fast mode)
 *  0b010..Two-pin push-pull mode
 *  0b011..Four-pin push-pull mode
 *  0b100..Two-pin open-drain mode with separate LPI2C target
 *  0b101..Two-pin output only mode (Ultra-Fast mode) with separate LPI2C target
 *  0b110..Two-pin push-pull mode with separate LPI2C target
 *  0b111..Four-pin push-pull mode (inverted outputs)
 */
#define LPI2C_MCFGR1_PINCFG(x)                   (((uint32_t)(((uint32_t)(x)) << LPI2C_MCFGR1_PINCFG_SHIFT)) & LPI2C_MCFGR1_PINCFG_MASK)
/*! @} */

/*! @name MCFGR2 - Controller Configuration 2 */
/*! @{ */

#define LPI2C_MCFGR2_BUSIDLE_MASK                (0xFFFU)
#define LPI2C_MCFGR2_BUSIDLE_SHIFT               (0U)
/*! BUSIDLE - Bus Idle Timeout */
#define LPI2C_MCFGR2_BUSIDLE(x)                  (((uint32_t)(((uint32_t)(x)) << LPI2C_MCFGR2_BUSIDLE_SHIFT)) & LPI2C_MCFGR2_BUSIDLE_MASK)

#define LPI2C_MCFGR2_FILTSCL_MASK                (0xF0000U)
#define LPI2C_MCFGR2_FILTSCL_SHIFT               (16U)
/*! FILTSCL - Glitch Filter SCL */
#define LPI2C_MCFGR2_FILTSCL(x)                  (((uint32_t)(((uint32_t)(x)) << LPI2C_MCFGR2_FILTSCL_SHIFT)) & LPI2C_MCFGR2_FILTSCL_MASK)

#define LPI2C_MCFGR2_FILTSDA_MASK                (0xF000000U)
#define LPI2C_MCFGR2_FILTSDA_SHIFT               (24U)
/*! FILTSDA - Glitch Filter SDA */
#define LPI2C_MCFGR2_FILTSDA(x)                  (((uint32_t)(((uint32_t)(x)) << LPI2C_MCFGR2_FILTSDA_SHIFT)) & LPI2C_MCFGR2_FILTSDA_MASK)
/*! @} */

/*! @name MCFGR3 - Controller Configuration 3 */
/*! @{ */

#define LPI2C_MCFGR3_PINLOW_MASK                 (0xFFF00U)
#define LPI2C_MCFGR3_PINLOW_SHIFT                (8U)
/*! PINLOW - Pin Low Timeout */
#define LPI2C_MCFGR3_PINLOW(x)                   (((uint32_t)(((uint32_t)(x)) << LPI2C_MCFGR3_PINLOW_SHIFT)) & LPI2C_MCFGR3_PINLOW_MASK)
/*! @} */

/*! @name MDMR - Controller Data Match */
/*! @{ */

#define LPI2C_MDMR_MATCH0_MASK                   (0xFFU)
#define LPI2C_MDMR_MATCH0_SHIFT                  (0U)
/*! MATCH0 - Match 0 Value */
#define LPI2C_MDMR_MATCH0(x)                     (((uint32_t)(((uint32_t)(x)) << LPI2C_MDMR_MATCH0_SHIFT)) & LPI2C_MDMR_MATCH0_MASK)

#define LPI2C_MDMR_MATCH1_MASK                   (0xFF0000U)
#define LPI2C_MDMR_MATCH1_SHIFT                  (16U)
/*! MATCH1 - Match 1 Value */
#define LPI2C_MDMR_MATCH1(x)                     (((uint32_t)(((uint32_t)(x)) << LPI2C_MDMR_MATCH1_SHIFT)) & LPI2C_MDMR_MATCH1_MASK)
/*! @} */

/*! @name MCCR0 - Controller Clock Configuration 0 */
/*! @{ */

#define LPI2C_MCCR0_CLKLO_MASK                   (0x3FU)
#define LPI2C_MCCR0_CLKLO_SHIFT                  (0U)
/*! CLKLO - Clock Low Period */
#define LPI2C_MCCR0_CLKLO(x)                     (((uint32_t)(((uint32_t)(x)) << LPI2C_MCCR0_CLKLO_SHIFT)) & LPI2C_MCCR0_CLKLO_MASK)

#define LPI2C_MCCR0_CLKHI_MASK                   (0x3F00U)
#define LPI2C_MCCR0_CLKHI_SHIFT                  (8U)
/*! CLKHI - Clock High Period */
#define LPI2C_MCCR0_CLKHI(x)                     (((uint32_t)(((uint32_t)(x)) << LPI2C_MCCR0_CLKHI_SHIFT)) & LPI2C_MCCR0_CLKHI_MASK)

#define LPI2C_MCCR0_SETHOLD_MASK                 (0x3F0000U)
#define LPI2C_MCCR0_SETHOLD_SHIFT                (16U)
/*! SETHOLD - Setup Hold Delay */
#define LPI2C_MCCR0_SETHOLD(x)                   (((uint32_t)(((uint32_t)(x)) << LPI2C_MCCR0_SETHOLD_SHIFT)) & LPI2C_MCCR0_SETHOLD_MASK)

#define LPI2C_MCCR0_DATAVD_MASK                  (0x3F000000U)
#define LPI2C_MCCR0_DATAVD_SHIFT                 (24U)
/*! DATAVD - Data Valid Delay */
#define LPI2C_MCCR0_DATAVD(x)                    (((uint32_t)(((uint32_t)(x)) << LPI2C_MCCR0_DATAVD_SHIFT)) & LPI2C_MCCR0_DATAVD_MASK)
/*! @} */

/*! @name MCCR1 - Controller Clock Configuration 1 */
/*! @{ */

#define LPI2C_MCCR1_CLKLO_MASK                   (0x3FU)
#define LPI2C_MCCR1_CLKLO_SHIFT                  (0U)
/*! CLKLO - Clock Low Period */
#define LPI2C_MCCR1_CLKLO(x)                     (((uint32_t)(((uint32_t)(x)) << LPI2C_MCCR1_CLKLO_SHIFT)) & LPI2C_MCCR1_CLKLO_MASK)

#define LPI2C_MCCR1_CLKHI_MASK                   (0x3F00U)
#define LPI2C_MCCR1_CLKHI_SHIFT                  (8U)
/*! CLKHI - Clock High Period */
#define LPI2C_MCCR1_CLKHI(x)                     (((uint32_t)(((uint32_t)(x)) << LPI2C_MCCR1_CLKHI_SHIFT)) & LPI2C_MCCR1_CLKHI_MASK)

#define LPI2C_MCCR1_SETHOLD_MASK                 (0x3F0000U)
#define LPI2C_MCCR1_SETHOLD_SHIFT                (16U)
/*! SETHOLD - Setup Hold Delay */
#define LPI2C_MCCR1_SETHOLD(x)                   (((uint32_t)(((uint32_t)(x)) << LPI2C_MCCR1_SETHOLD_SHIFT)) & LPI2C_MCCR1_SETHOLD_MASK)

#define LPI2C_MCCR1_DATAVD_MASK                  (0x3F000000U)
#define LPI2C_MCCR1_DATAVD_SHIFT                 (24U)
/*! DATAVD - Data Valid Delay */
#define LPI2C_MCCR1_DATAVD(x)                    (((uint32_t)(((uint32_t)(x)) << LPI2C_MCCR1_DATAVD_SHIFT)) & LPI2C_MCCR1_DATAVD_MASK)
/*! @} */

/*! @name MFCR - Controller FIFO Control */
/*! @{ */

#define LPI2C_MFCR_TXWATER_MASK                  (0x3U)
#define LPI2C_MFCR_TXWATER_SHIFT                 (0U)
/*! TXWATER - Transmit FIFO Watermark */
#define LPI2C_MFCR_TXWATER(x)                    (((uint32_t)(((uint32_t)(x)) << LPI2C_MFCR_TXWATER_SHIFT)) & LPI2C_MFCR_TXWATER_MASK)

#define LPI2C_MFCR_RXWATER_MASK                  (0x30000U)
#define LPI2C_MFCR_RXWATER_SHIFT                 (16U)
/*! RXWATER - Receive FIFO Watermark */
#define LPI2C_MFCR_RXWATER(x)                    (((uint32_t)(((uint32_t)(x)) << LPI2C_MFCR_RXWATER_SHIFT)) & LPI2C_MFCR_RXWATER_MASK)
/*! @} */

/*! @name MFSR - Controller FIFO Status */
/*! @{ */

#define LPI2C_MFSR_TXCOUNT_MASK                  (0x7U)
#define LPI2C_MFSR_TXCOUNT_SHIFT                 (0U)
/*! TXCOUNT - Transmit FIFO Count */
#define LPI2C_MFSR_TXCOUNT(x)                    (((uint32_t)(((uint32_t)(x)) << LPI2C_MFSR_TXCOUNT_SHIFT)) & LPI2C_MFSR_TXCOUNT_MASK)

#define LPI2C_MFSR_RXCOUNT_MASK                  (0x70000U)
#define LPI2C_MFSR_RXCOUNT_SHIFT                 (16U)
/*! RXCOUNT - Receive FIFO Count */
#define LPI2C_MFSR_RXCOUNT(x)                    (((uint32_t)(((uint32_t)(x)) << LPI2C_MFSR_RXCOUNT_SHIFT)) & LPI2C_MFSR_RXCOUNT_MASK)
/*! @} */

/*! @name MTDR - Controller Transmit Data */
/*! @{ */

#define LPI2C_MTDR_DATA_MASK                     (0xFFU)
#define LPI2C_MTDR_DATA_SHIFT                    (0U)
/*! DATA - Transmit Data */
#define LPI2C_MTDR_DATA(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_MTDR_DATA_SHIFT)) & LPI2C_MTDR_DATA_MASK)

#define LPI2C_MTDR_CMD_MASK                      (0x700U)
#define LPI2C_MTDR_CMD_SHIFT                     (8U)
/*! CMD - Command Data
 *  0b000..Transmit the value in DATA[7:0]
 *  0b001..Receive (DATA[7:0] + 1) bytes
 *  0b010..Generate Stop condition on I2C bus
 *  0b011..Receive and discard (DATA[7:0] + 1) bytes
 *  0b100..Generate (repeated) Start on the I2C bus and transmit the address in DATA[7:0]
 *  0b101..Generate (repeated) Start on the I2C bus and transmit the address in DATA[7:0] (this transfer expects a NACK to be returned)
 *  0b110..Generate (repeated) Start on the I2C bus and transmit the address in DATA[7:0] using HS mode
 *  0b111..Generate (repeated) Start on the I2C bus and transmit the address in DATA[7:0] using HS mode (this transfer expects a NACK to be returned)
 */
#define LPI2C_MTDR_CMD(x)                        (((uint32_t)(((uint32_t)(x)) << LPI2C_MTDR_CMD_SHIFT)) & LPI2C_MTDR_CMD_MASK)
/*! @} */

/*! @name MRDR - Controller Receive Data */
/*! @{ */

#define LPI2C_MRDR_DATA_MASK                     (0xFFU)
#define LPI2C_MRDR_DATA_SHIFT                    (0U)
/*! DATA - Receive Data */
#define LPI2C_MRDR_DATA(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_MRDR_DATA_SHIFT)) & LPI2C_MRDR_DATA_MASK)

#define LPI2C_MRDR_RXEMPTY_MASK                  (0x4000U)
#define LPI2C_MRDR_RXEMPTY_SHIFT                 (14U)
/*! RXEMPTY - Receive Empty
 *  0b0..Not empty
 *  0b1..Empty
 */
#define LPI2C_MRDR_RXEMPTY(x)                    (((uint32_t)(((uint32_t)(x)) << LPI2C_MRDR_RXEMPTY_SHIFT)) & LPI2C_MRDR_RXEMPTY_MASK)
/*! @} */

/*! @name MRDROR - Controller Receive Data Read Only */
/*! @{ */

#define LPI2C_MRDROR_DATA_MASK                   (0xFFU)
#define LPI2C_MRDROR_DATA_SHIFT                  (0U)
/*! DATA - Receive Data */
#define LPI2C_MRDROR_DATA(x)                     (((uint32_t)(((uint32_t)(x)) << LPI2C_MRDROR_DATA_SHIFT)) & LPI2C_MRDROR_DATA_MASK)

#define LPI2C_MRDROR_RXEMPTY_MASK                (0x4000U)
#define LPI2C_MRDROR_RXEMPTY_SHIFT               (14U)
/*! RXEMPTY - RX Empty
 *  0b0..Not empty
 *  0b1..Empty
 */
#define LPI2C_MRDROR_RXEMPTY(x)                  (((uint32_t)(((uint32_t)(x)) << LPI2C_MRDROR_RXEMPTY_SHIFT)) & LPI2C_MRDROR_RXEMPTY_MASK)
/*! @} */

/*! @name SCR - Target Control */
/*! @{ */

#define LPI2C_SCR_SEN_MASK                       (0x1U)
#define LPI2C_SCR_SEN_SHIFT                      (0U)
/*! SEN - Target Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPI2C_SCR_SEN(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_SCR_SEN_SHIFT)) & LPI2C_SCR_SEN_MASK)

#define LPI2C_SCR_RST_MASK                       (0x2U)
#define LPI2C_SCR_RST_SHIFT                      (1U)
/*! RST - Software Reset
 *  0b0..Not reset
 *  0b1..Reset
 */
#define LPI2C_SCR_RST(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_SCR_RST_SHIFT)) & LPI2C_SCR_RST_MASK)

#define LPI2C_SCR_FILTEN_MASK                    (0x10U)
#define LPI2C_SCR_FILTEN_SHIFT                   (4U)
/*! FILTEN - Filter Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPI2C_SCR_FILTEN(x)                      (((uint32_t)(((uint32_t)(x)) << LPI2C_SCR_FILTEN_SHIFT)) & LPI2C_SCR_FILTEN_MASK)

#define LPI2C_SCR_FILTDZ_MASK                    (0x20U)
#define LPI2C_SCR_FILTDZ_SHIFT                   (5U)
/*! FILTDZ - Filter Doze Enable
 *  0b0..Enable
 *  0b1..Disable
 */
#define LPI2C_SCR_FILTDZ(x)                      (((uint32_t)(((uint32_t)(x)) << LPI2C_SCR_FILTDZ_SHIFT)) & LPI2C_SCR_FILTDZ_MASK)

#define LPI2C_SCR_RTF_MASK                       (0x100U)
#define LPI2C_SCR_RTF_SHIFT                      (8U)
/*! RTF - Reset Transmit FIFO
 *  0b0..No effect
 *  0b1..STDR is now empty
 */
#define LPI2C_SCR_RTF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_SCR_RTF_SHIFT)) & LPI2C_SCR_RTF_MASK)

#define LPI2C_SCR_RRF_MASK                       (0x200U)
#define LPI2C_SCR_RRF_SHIFT                      (9U)
/*! RRF - Reset Receive FIFO
 *  0b0..No effect
 *  0b1..SRDR is now empty
 */
#define LPI2C_SCR_RRF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_SCR_RRF_SHIFT)) & LPI2C_SCR_RRF_MASK)
/*! @} */

/*! @name SSR - Target Status */
/*! @{ */

#define LPI2C_SSR_TDF_MASK                       (0x1U)
#define LPI2C_SSR_TDF_SHIFT                      (0U)
/*! TDF - Transmit Data Flag
 *  0b0..Transmit data not requested
 *  0b1..Transmit data is requested
 */
#define LPI2C_SSR_TDF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_SSR_TDF_SHIFT)) & LPI2C_SSR_TDF_MASK)

#define LPI2C_SSR_RDF_MASK                       (0x2U)
#define LPI2C_SSR_RDF_SHIFT                      (1U)
/*! RDF - Receive Data Flag
 *  0b0..Not ready
 *  0b1..Ready
 */
#define LPI2C_SSR_RDF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_SSR_RDF_SHIFT)) & LPI2C_SSR_RDF_MASK)

#define LPI2C_SSR_AVF_MASK                       (0x4U)
#define LPI2C_SSR_AVF_SHIFT                      (2U)
/*! AVF - Address Valid Flag
 *  0b0..Not valid
 *  0b1..Valid
 */
#define LPI2C_SSR_AVF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_SSR_AVF_SHIFT)) & LPI2C_SSR_AVF_MASK)

#define LPI2C_SSR_TAF_MASK                       (0x8U)
#define LPI2C_SSR_TAF_SHIFT                      (3U)
/*! TAF - Transmit ACK Flag
 *  0b0..Not required
 *  0b1..Required
 */
#define LPI2C_SSR_TAF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_SSR_TAF_SHIFT)) & LPI2C_SSR_TAF_MASK)

#define LPI2C_SSR_RSF_MASK                       (0x100U)
#define LPI2C_SSR_RSF_SHIFT                      (8U)
/*! RSF - Repeated Start Flag
 *  0b0..No effect
 *  0b0..No repeated Start detected
 *  0b1..Clear the flag
 *  0b1..Repeated Start detected
 */
#define LPI2C_SSR_RSF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_SSR_RSF_SHIFT)) & LPI2C_SSR_RSF_MASK)

#define LPI2C_SSR_SDF_MASK                       (0x200U)
#define LPI2C_SSR_SDF_SHIFT                      (9U)
/*! SDF - Stop Detect Flag
 *  0b0..No Stop detected
 *  0b0..No effect
 *  0b1..Clear the flag
 *  0b1..Stop detected
 */
#define LPI2C_SSR_SDF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_SSR_SDF_SHIFT)) & LPI2C_SSR_SDF_MASK)

#define LPI2C_SSR_BEF_MASK                       (0x400U)
#define LPI2C_SSR_BEF_SHIFT                      (10U)
/*! BEF - Bit Error Flag
 *  0b0..No bit error occurred
 *  0b0..No effect
 *  0b1..Bit error occurred
 *  0b1..Clear the flag
 */
#define LPI2C_SSR_BEF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_SSR_BEF_SHIFT)) & LPI2C_SSR_BEF_MASK)

#define LPI2C_SSR_FEF_MASK                       (0x800U)
#define LPI2C_SSR_FEF_SHIFT                      (11U)
/*! FEF - FIFO Error Flag
 *  0b0..No FIFO error
 *  0b0..No effect
 *  0b1..Clear the flag
 *  0b1..FIFO error
 */
#define LPI2C_SSR_FEF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_SSR_FEF_SHIFT)) & LPI2C_SSR_FEF_MASK)

#define LPI2C_SSR_AM0F_MASK                      (0x1000U)
#define LPI2C_SSR_AM0F_SHIFT                     (12U)
/*! AM0F - Address Match 0 Flag
 *  0b0..ADDR0 matching address not received
 *  0b1..ADDR0 matching address received
 */
#define LPI2C_SSR_AM0F(x)                        (((uint32_t)(((uint32_t)(x)) << LPI2C_SSR_AM0F_SHIFT)) & LPI2C_SSR_AM0F_MASK)

#define LPI2C_SSR_AM1F_MASK                      (0x2000U)
#define LPI2C_SSR_AM1F_SHIFT                     (13U)
/*! AM1F - Address Match 1 Flag
 *  0b0..Matching address not received
 *  0b1..Matching address received
 */
#define LPI2C_SSR_AM1F(x)                        (((uint32_t)(((uint32_t)(x)) << LPI2C_SSR_AM1F_SHIFT)) & LPI2C_SSR_AM1F_MASK)

#define LPI2C_SSR_GCF_MASK                       (0x4000U)
#define LPI2C_SSR_GCF_SHIFT                      (14U)
/*! GCF - General Call Flag
 *  0b0..General call address disabled or not detected
 *  0b1..General call address detected
 */
#define LPI2C_SSR_GCF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_SSR_GCF_SHIFT)) & LPI2C_SSR_GCF_MASK)

#define LPI2C_SSR_SARF_MASK                      (0x8000U)
#define LPI2C_SSR_SARF_SHIFT                     (15U)
/*! SARF - SMBus Alert Response Flag
 *  0b0..Disabled or not detected
 *  0b1..Enabled and detected
 */
#define LPI2C_SSR_SARF(x)                        (((uint32_t)(((uint32_t)(x)) << LPI2C_SSR_SARF_SHIFT)) & LPI2C_SSR_SARF_MASK)

#define LPI2C_SSR_SBF_MASK                       (0x1000000U)
#define LPI2C_SSR_SBF_SHIFT                      (24U)
/*! SBF - Target Busy Flag
 *  0b0..Idle
 *  0b1..Busy
 */
#define LPI2C_SSR_SBF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_SSR_SBF_SHIFT)) & LPI2C_SSR_SBF_MASK)

#define LPI2C_SSR_BBF_MASK                       (0x2000000U)
#define LPI2C_SSR_BBF_SHIFT                      (25U)
/*! BBF - Bus Busy Flag
 *  0b0..Idle
 *  0b1..Busy
 */
#define LPI2C_SSR_BBF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_SSR_BBF_SHIFT)) & LPI2C_SSR_BBF_MASK)
/*! @} */

/*! @name SIER - Target Interrupt Enable */
/*! @{ */

#define LPI2C_SIER_TDIE_MASK                     (0x1U)
#define LPI2C_SIER_TDIE_SHIFT                    (0U)
/*! TDIE - Transmit Data Interrupt Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPI2C_SIER_TDIE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_SIER_TDIE_SHIFT)) & LPI2C_SIER_TDIE_MASK)

#define LPI2C_SIER_RDIE_MASK                     (0x2U)
#define LPI2C_SIER_RDIE_SHIFT                    (1U)
/*! RDIE - Receive Data Interrupt Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPI2C_SIER_RDIE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_SIER_RDIE_SHIFT)) & LPI2C_SIER_RDIE_MASK)

#define LPI2C_SIER_AVIE_MASK                     (0x4U)
#define LPI2C_SIER_AVIE_SHIFT                    (2U)
/*! AVIE - Address Valid Interrupt Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPI2C_SIER_AVIE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_SIER_AVIE_SHIFT)) & LPI2C_SIER_AVIE_MASK)

#define LPI2C_SIER_TAIE_MASK                     (0x8U)
#define LPI2C_SIER_TAIE_SHIFT                    (3U)
/*! TAIE - Transmit ACK Interrupt Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPI2C_SIER_TAIE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_SIER_TAIE_SHIFT)) & LPI2C_SIER_TAIE_MASK)

#define LPI2C_SIER_RSIE_MASK                     (0x100U)
#define LPI2C_SIER_RSIE_SHIFT                    (8U)
/*! RSIE - Repeated Start Interrupt Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPI2C_SIER_RSIE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_SIER_RSIE_SHIFT)) & LPI2C_SIER_RSIE_MASK)

#define LPI2C_SIER_SDIE_MASK                     (0x200U)
#define LPI2C_SIER_SDIE_SHIFT                    (9U)
/*! SDIE - Stop Detect Interrupt Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPI2C_SIER_SDIE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_SIER_SDIE_SHIFT)) & LPI2C_SIER_SDIE_MASK)

#define LPI2C_SIER_BEIE_MASK                     (0x400U)
#define LPI2C_SIER_BEIE_SHIFT                    (10U)
/*! BEIE - Bit Error Interrupt Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPI2C_SIER_BEIE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_SIER_BEIE_SHIFT)) & LPI2C_SIER_BEIE_MASK)

#define LPI2C_SIER_FEIE_MASK                     (0x800U)
#define LPI2C_SIER_FEIE_SHIFT                    (11U)
/*! FEIE - FIFO Error Interrupt Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPI2C_SIER_FEIE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_SIER_FEIE_SHIFT)) & LPI2C_SIER_FEIE_MASK)

#define LPI2C_SIER_AM0IE_MASK                    (0x1000U)
#define LPI2C_SIER_AM0IE_SHIFT                   (12U)
/*! AM0IE - Address Match 0 Interrupt Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPI2C_SIER_AM0IE(x)                      (((uint32_t)(((uint32_t)(x)) << LPI2C_SIER_AM0IE_SHIFT)) & LPI2C_SIER_AM0IE_MASK)

#define LPI2C_SIER_AM1IE_MASK                    (0x2000U)
#define LPI2C_SIER_AM1IE_SHIFT                   (13U)
/*! AM1IE - Address Match 1 Interrupt Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPI2C_SIER_AM1IE(x)                      (((uint32_t)(((uint32_t)(x)) << LPI2C_SIER_AM1IE_SHIFT)) & LPI2C_SIER_AM1IE_MASK)

#define LPI2C_SIER_GCIE_MASK                     (0x4000U)
#define LPI2C_SIER_GCIE_SHIFT                    (14U)
/*! GCIE - General Call Interrupt Enable
 *  0b0..Disabled
 *  0b1..Enabled
 */
#define LPI2C_SIER_GCIE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_SIER_GCIE_SHIFT)) & LPI2C_SIER_GCIE_MASK)

#define LPI2C_SIER_SARIE_MASK                    (0x8000U)
#define LPI2C_SIER_SARIE_SHIFT                   (15U)
/*! SARIE - SMBus Alert Response Interrupt Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPI2C_SIER_SARIE(x)                      (((uint32_t)(((uint32_t)(x)) << LPI2C_SIER_SARIE_SHIFT)) & LPI2C_SIER_SARIE_MASK)
/*! @} */

/*! @name SDER - Target DMA Enable */
/*! @{ */

#define LPI2C_SDER_TDDE_MASK                     (0x1U)
#define LPI2C_SDER_TDDE_SHIFT                    (0U)
/*! TDDE - Transmit Data DMA Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPI2C_SDER_TDDE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_SDER_TDDE_SHIFT)) & LPI2C_SDER_TDDE_MASK)

#define LPI2C_SDER_RDDE_MASK                     (0x2U)
#define LPI2C_SDER_RDDE_SHIFT                    (1U)
/*! RDDE - Receive Data DMA Enable
 *  0b0..Disable DMA request
 *  0b1..Enable DMA request
 */
#define LPI2C_SDER_RDDE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_SDER_RDDE_SHIFT)) & LPI2C_SDER_RDDE_MASK)

#define LPI2C_SDER_AVDE_MASK                     (0x4U)
#define LPI2C_SDER_AVDE_SHIFT                    (2U)
/*! AVDE - Address Valid DMA Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPI2C_SDER_AVDE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_SDER_AVDE_SHIFT)) & LPI2C_SDER_AVDE_MASK)

#define LPI2C_SDER_RSDE_MASK                     (0x100U)
#define LPI2C_SDER_RSDE_SHIFT                    (8U)
/*! RSDE - Repeated Start DMA Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPI2C_SDER_RSDE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_SDER_RSDE_SHIFT)) & LPI2C_SDER_RSDE_MASK)

#define LPI2C_SDER_SDDE_MASK                     (0x200U)
#define LPI2C_SDER_SDDE_SHIFT                    (9U)
/*! SDDE - Stop Detect DMA Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPI2C_SDER_SDDE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_SDER_SDDE_SHIFT)) & LPI2C_SDER_SDDE_MASK)
/*! @} */

/*! @name SCFGR0 - Target Configuration 0 */
/*! @{ */

#define LPI2C_SCFGR0_RDREQ_MASK                  (0x1U)
#define LPI2C_SCFGR0_RDREQ_SHIFT                 (0U)
/*! RDREQ - Read Request
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPI2C_SCFGR0_RDREQ(x)                    (((uint32_t)(((uint32_t)(x)) << LPI2C_SCFGR0_RDREQ_SHIFT)) & LPI2C_SCFGR0_RDREQ_MASK)

#define LPI2C_SCFGR0_RDACK_MASK                  (0x2U)
#define LPI2C_SCFGR0_RDACK_SHIFT                 (1U)
/*! RDACK - Read Acknowledge Flag
 *  0b0..Read Request not acknowledged
 *  0b1..Read Request acknowledged
 */
#define LPI2C_SCFGR0_RDACK(x)                    (((uint32_t)(((uint32_t)(x)) << LPI2C_SCFGR0_RDACK_SHIFT)) & LPI2C_SCFGR0_RDACK_MASK)
/*! @} */

/*! @name SCFGR1 - Target Configuration 1 */
/*! @{ */

#define LPI2C_SCFGR1_ADRSTALL_MASK               (0x1U)
#define LPI2C_SCFGR1_ADRSTALL_SHIFT              (0U)
/*! ADRSTALL - Address SCL Stall
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPI2C_SCFGR1_ADRSTALL(x)                 (((uint32_t)(((uint32_t)(x)) << LPI2C_SCFGR1_ADRSTALL_SHIFT)) & LPI2C_SCFGR1_ADRSTALL_MASK)

#define LPI2C_SCFGR1_RXSTALL_MASK                (0x2U)
#define LPI2C_SCFGR1_RXSTALL_SHIFT               (1U)
/*! RXSTALL - RX SCL Stall
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPI2C_SCFGR1_RXSTALL(x)                  (((uint32_t)(((uint32_t)(x)) << LPI2C_SCFGR1_RXSTALL_SHIFT)) & LPI2C_SCFGR1_RXSTALL_MASK)

#define LPI2C_SCFGR1_TXDSTALL_MASK               (0x4U)
#define LPI2C_SCFGR1_TXDSTALL_SHIFT              (2U)
/*! TXDSTALL - Transmit Data SCL Stall
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPI2C_SCFGR1_TXDSTALL(x)                 (((uint32_t)(((uint32_t)(x)) << LPI2C_SCFGR1_TXDSTALL_SHIFT)) & LPI2C_SCFGR1_TXDSTALL_MASK)

#define LPI2C_SCFGR1_ACKSTALL_MASK               (0x8U)
#define LPI2C_SCFGR1_ACKSTALL_SHIFT              (3U)
/*! ACKSTALL - ACK SCL Stall
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPI2C_SCFGR1_ACKSTALL(x)                 (((uint32_t)(((uint32_t)(x)) << LPI2C_SCFGR1_ACKSTALL_SHIFT)) & LPI2C_SCFGR1_ACKSTALL_MASK)

#define LPI2C_SCFGR1_RXNACK_MASK                 (0x10U)
#define LPI2C_SCFGR1_RXNACK_SHIFT                (4U)
/*! RXNACK - Receive NACK
 *  0b0..ACK or NACK always determined by STAR[TXNACK]
 *  0b1..NACK always generated on address overrun or receive data overrun, otherwise ACK or NACK is determined by STAR[TXNACK]
 */
#define LPI2C_SCFGR1_RXNACK(x)                   (((uint32_t)(((uint32_t)(x)) << LPI2C_SCFGR1_RXNACK_SHIFT)) & LPI2C_SCFGR1_RXNACK_MASK)

#define LPI2C_SCFGR1_GCEN_MASK                   (0x100U)
#define LPI2C_SCFGR1_GCEN_SHIFT                  (8U)
/*! GCEN - General Call Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPI2C_SCFGR1_GCEN(x)                     (((uint32_t)(((uint32_t)(x)) << LPI2C_SCFGR1_GCEN_SHIFT)) & LPI2C_SCFGR1_GCEN_MASK)

#define LPI2C_SCFGR1_SAEN_MASK                   (0x200U)
#define LPI2C_SCFGR1_SAEN_SHIFT                  (9U)
/*! SAEN - SMBus Alert Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPI2C_SCFGR1_SAEN(x)                     (((uint32_t)(((uint32_t)(x)) << LPI2C_SCFGR1_SAEN_SHIFT)) & LPI2C_SCFGR1_SAEN_MASK)

#define LPI2C_SCFGR1_TXCFG_MASK                  (0x400U)
#define LPI2C_SCFGR1_TXCFG_SHIFT                 (10U)
/*! TXCFG - Transmit Flag Configuration
 *  0b0..MSR[TDF] is set only during a target-transmit transfer when STDR is empty
 *  0b1..MSR[TDF] is set whenever STDR is empty
 */
#define LPI2C_SCFGR1_TXCFG(x)                    (((uint32_t)(((uint32_t)(x)) << LPI2C_SCFGR1_TXCFG_SHIFT)) & LPI2C_SCFGR1_TXCFG_MASK)

#define LPI2C_SCFGR1_RXCFG_MASK                  (0x800U)
#define LPI2C_SCFGR1_RXCFG_SHIFT                 (11U)
/*! RXCFG - Receive Data Configuration
 *  0b0..Return received data, clear MSR[RDF]
 *  0b1..Return SASR and clear SSR[AVF] when SSR[AVF] is set, return received data and clear MSR[RDF] when SSR[AFV] is not set
 */
#define LPI2C_SCFGR1_RXCFG(x)                    (((uint32_t)(((uint32_t)(x)) << LPI2C_SCFGR1_RXCFG_SHIFT)) & LPI2C_SCFGR1_RXCFG_MASK)

#define LPI2C_SCFGR1_IGNACK_MASK                 (0x1000U)
#define LPI2C_SCFGR1_IGNACK_SHIFT                (12U)
/*! IGNACK - Ignore NACK
 *  0b0..End transfer on NACK
 *  0b1..Do not end transfer on NACK
 */
#define LPI2C_SCFGR1_IGNACK(x)                   (((uint32_t)(((uint32_t)(x)) << LPI2C_SCFGR1_IGNACK_SHIFT)) & LPI2C_SCFGR1_IGNACK_MASK)

#define LPI2C_SCFGR1_HSMEN_MASK                  (0x2000U)
#define LPI2C_SCFGR1_HSMEN_SHIFT                 (13U)
/*! HSMEN - HS Mode Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPI2C_SCFGR1_HSMEN(x)                    (((uint32_t)(((uint32_t)(x)) << LPI2C_SCFGR1_HSMEN_SHIFT)) & LPI2C_SCFGR1_HSMEN_MASK)

#define LPI2C_SCFGR1_ADDRCFG_MASK                (0x70000U)
#define LPI2C_SCFGR1_ADDRCFG_SHIFT               (16U)
/*! ADDRCFG - Address Configuration
 *  0b000..Address match 0 (7-bit)
 *  0b001..Address match 0 (10-bit)
 *  0b010..Address match 0 (7-bit) or address match 1 (7-bit)
 *  0b011..Address match 0 (10-bit) or address match 1 (10-bit)
 *  0b100..Address match 0 (7-bit) or address match 1 (10-bit)
 *  0b101..Address match 0 (10-bit) or address match 1 (7-bit)
 *  0b110..From address match 0 (7-bit) to address match 1 (7-bit)
 *  0b111..From address match 0 (10-bit) to address match 1 (10-bit)
 */
#define LPI2C_SCFGR1_ADDRCFG(x)                  (((uint32_t)(((uint32_t)(x)) << LPI2C_SCFGR1_ADDRCFG_SHIFT)) & LPI2C_SCFGR1_ADDRCFG_MASK)

#define LPI2C_SCFGR1_RXALL_MASK                  (0x1000000U)
#define LPI2C_SCFGR1_RXALL_SHIFT                 (24U)
/*! RXALL - Receive All
 *  0b0..Disable
 *  0b1..Enable
 */
#define LPI2C_SCFGR1_RXALL(x)                    (((uint32_t)(((uint32_t)(x)) << LPI2C_SCFGR1_RXALL_SHIFT)) & LPI2C_SCFGR1_RXALL_MASK)

#define LPI2C_SCFGR1_RSCFG_MASK                  (0x2000000U)
#define LPI2C_SCFGR1_RSCFG_SHIFT                 (25U)
/*! RSCFG - Repeated Start Configuration
 *  0b0..Any repeated Start condition following an address match
 *  0b1..Any repeated Start condition
 */
#define LPI2C_SCFGR1_RSCFG(x)                    (((uint32_t)(((uint32_t)(x)) << LPI2C_SCFGR1_RSCFG_SHIFT)) & LPI2C_SCFGR1_RSCFG_MASK)

#define LPI2C_SCFGR1_SDCFG_MASK                  (0x4000000U)
#define LPI2C_SCFGR1_SDCFG_SHIFT                 (26U)
/*! SDCFG - Stop Detect Configuration
 *  0b0..Any Stop condition following an address match
 *  0b1..Any Stop condition
 */
#define LPI2C_SCFGR1_SDCFG(x)                    (((uint32_t)(((uint32_t)(x)) << LPI2C_SCFGR1_SDCFG_SHIFT)) & LPI2C_SCFGR1_SDCFG_MASK)
/*! @} */

/*! @name SCFGR2 - Target Configuration 2 */
/*! @{ */

#define LPI2C_SCFGR2_CLKHOLD_MASK                (0xFU)
#define LPI2C_SCFGR2_CLKHOLD_SHIFT               (0U)
/*! CLKHOLD - Clock Hold Time */
#define LPI2C_SCFGR2_CLKHOLD(x)                  (((uint32_t)(((uint32_t)(x)) << LPI2C_SCFGR2_CLKHOLD_SHIFT)) & LPI2C_SCFGR2_CLKHOLD_MASK)

#define LPI2C_SCFGR2_DATAVD_MASK                 (0x3F00U)
#define LPI2C_SCFGR2_DATAVD_SHIFT                (8U)
/*! DATAVD - Data Valid Delay */
#define LPI2C_SCFGR2_DATAVD(x)                   (((uint32_t)(((uint32_t)(x)) << LPI2C_SCFGR2_DATAVD_SHIFT)) & LPI2C_SCFGR2_DATAVD_MASK)

#define LPI2C_SCFGR2_FILTSCL_MASK                (0xF0000U)
#define LPI2C_SCFGR2_FILTSCL_SHIFT               (16U)
/*! FILTSCL - Glitch Filter SCL */
#define LPI2C_SCFGR2_FILTSCL(x)                  (((uint32_t)(((uint32_t)(x)) << LPI2C_SCFGR2_FILTSCL_SHIFT)) & LPI2C_SCFGR2_FILTSCL_MASK)

#define LPI2C_SCFGR2_FILTSDA_MASK                (0xF000000U)
#define LPI2C_SCFGR2_FILTSDA_SHIFT               (24U)
/*! FILTSDA - Glitch Filter SDA */
#define LPI2C_SCFGR2_FILTSDA(x)                  (((uint32_t)(((uint32_t)(x)) << LPI2C_SCFGR2_FILTSDA_SHIFT)) & LPI2C_SCFGR2_FILTSDA_MASK)
/*! @} */

/*! @name SAMR - Target Address Match */
/*! @{ */

#define LPI2C_SAMR_ADDR0_MASK                    (0x7FEU)
#define LPI2C_SAMR_ADDR0_SHIFT                   (1U)
/*! ADDR0 - Address 0 Value */
#define LPI2C_SAMR_ADDR0(x)                      (((uint32_t)(((uint32_t)(x)) << LPI2C_SAMR_ADDR0_SHIFT)) & LPI2C_SAMR_ADDR0_MASK)

#define LPI2C_SAMR_ADDR1_MASK                    (0x7FE0000U)
#define LPI2C_SAMR_ADDR1_SHIFT                   (17U)
/*! ADDR1 - Address 1 Value */
#define LPI2C_SAMR_ADDR1(x)                      (((uint32_t)(((uint32_t)(x)) << LPI2C_SAMR_ADDR1_SHIFT)) & LPI2C_SAMR_ADDR1_MASK)
/*! @} */

/*! @name SASR - Target Address Status */
/*! @{ */

#define LPI2C_SASR_RADDR_MASK                    (0x7FFU)
#define LPI2C_SASR_RADDR_SHIFT                   (0U)
/*! RADDR - Received Address */
#define LPI2C_SASR_RADDR(x)                      (((uint32_t)(((uint32_t)(x)) << LPI2C_SASR_RADDR_SHIFT)) & LPI2C_SASR_RADDR_MASK)

#define LPI2C_SASR_ANV_MASK                      (0x4000U)
#define LPI2C_SASR_ANV_SHIFT                     (14U)
/*! ANV - Address Not Valid
 *  0b0..Valid
 *  0b1..Not valid
 */
#define LPI2C_SASR_ANV(x)                        (((uint32_t)(((uint32_t)(x)) << LPI2C_SASR_ANV_SHIFT)) & LPI2C_SASR_ANV_MASK)
/*! @} */

/*! @name STAR - Target Transmit ACK */
/*! @{ */

#define LPI2C_STAR_TXNACK_MASK                   (0x1U)
#define LPI2C_STAR_TXNACK_SHIFT                  (0U)
/*! TXNACK - Transmit NACK
 *  0b0..Transmit ACK
 *  0b1..Transmit NACK
 */
#define LPI2C_STAR_TXNACK(x)                     (((uint32_t)(((uint32_t)(x)) << LPI2C_STAR_TXNACK_SHIFT)) & LPI2C_STAR_TXNACK_MASK)
/*! @} */

/*! @name STDR - Target Transmit Data */
/*! @{ */

#define LPI2C_STDR_DATA_MASK                     (0xFFU)
#define LPI2C_STDR_DATA_SHIFT                    (0U)
/*! DATA - Transmit Data */
#define LPI2C_STDR_DATA(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_STDR_DATA_SHIFT)) & LPI2C_STDR_DATA_MASK)
/*! @} */

/*! @name SRDR - Target Receive Data */
/*! @{ */

#define LPI2C_SRDR_DATA_MASK                     (0xFFU)
#define LPI2C_SRDR_DATA_SHIFT                    (0U)
/*! DATA - Received Data */
#define LPI2C_SRDR_DATA(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_SRDR_DATA_SHIFT)) & LPI2C_SRDR_DATA_MASK)

#define LPI2C_SRDR_RADDR_MASK                    (0x700U)
#define LPI2C_SRDR_RADDR_SHIFT                   (8U)
/*! RADDR - Received Address */
#define LPI2C_SRDR_RADDR(x)                      (((uint32_t)(((uint32_t)(x)) << LPI2C_SRDR_RADDR_SHIFT)) & LPI2C_SRDR_RADDR_MASK)

#define LPI2C_SRDR_RXEMPTY_MASK                  (0x4000U)
#define LPI2C_SRDR_RXEMPTY_SHIFT                 (14U)
/*! RXEMPTY - Receive Empty
 *  0b0..Not empty
 *  0b1..Empty
 */
#define LPI2C_SRDR_RXEMPTY(x)                    (((uint32_t)(((uint32_t)(x)) << LPI2C_SRDR_RXEMPTY_SHIFT)) & LPI2C_SRDR_RXEMPTY_MASK)

#define LPI2C_SRDR_SOF_MASK                      (0x8000U)
#define LPI2C_SRDR_SOF_SHIFT                     (15U)
/*! SOF - Start of Frame
 *  0b0..Not first
 *  0b1..First
 */
#define LPI2C_SRDR_SOF(x)                        (((uint32_t)(((uint32_t)(x)) << LPI2C_SRDR_SOF_SHIFT)) & LPI2C_SRDR_SOF_MASK)
/*! @} */

/*! @name SRDROR - Target Receive Data Read Only */
/*! @{ */

#define LPI2C_SRDROR_DATA_MASK                   (0xFFU)
#define LPI2C_SRDROR_DATA_SHIFT                  (0U)
/*! DATA - Receive Data */
#define LPI2C_SRDROR_DATA(x)                     (((uint32_t)(((uint32_t)(x)) << LPI2C_SRDROR_DATA_SHIFT)) & LPI2C_SRDROR_DATA_MASK)

#define LPI2C_SRDROR_RADDR_MASK                  (0x700U)
#define LPI2C_SRDROR_RADDR_SHIFT                 (8U)
/*! RADDR - Received Address */
#define LPI2C_SRDROR_RADDR(x)                    (((uint32_t)(((uint32_t)(x)) << LPI2C_SRDROR_RADDR_SHIFT)) & LPI2C_SRDROR_RADDR_MASK)

#define LPI2C_SRDROR_RXEMPTY_MASK                (0x4000U)
#define LPI2C_SRDROR_RXEMPTY_SHIFT               (14U)
/*! RXEMPTY - Receive Empty
 *  0b0..Not empty
 *  0b1..Empty
 */
#define LPI2C_SRDROR_RXEMPTY(x)                  (((uint32_t)(((uint32_t)(x)) << LPI2C_SRDROR_RXEMPTY_SHIFT)) & LPI2C_SRDROR_RXEMPTY_MASK)

#define LPI2C_SRDROR_SOF_MASK                    (0x8000U)
#define LPI2C_SRDROR_SOF_SHIFT                   (15U)
/*! SOF - Start of Frame
 *  0b0..Not the first
 *  0b1..First
 */
#define LPI2C_SRDROR_SOF(x)                      (((uint32_t)(((uint32_t)(x)) << LPI2C_SRDROR_SOF_SHIFT)) & LPI2C_SRDROR_SOF_MASK)
/*! @} */


/*!
 * @}
 */ /* end of group LPI2C_Register_Masks */


/*!
 * @}
 */ /* end of group LPI2C_Peripheral_Access_Layer */


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


#endif  /* PERI_LPI2C_H_ */

