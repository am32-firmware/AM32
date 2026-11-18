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
**         CMSIS Peripheral Access Layer for SPC
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
 * @file PERI_SPC.h
 * @version 2.0
 * @date 2024-10-29
 * @brief CMSIS Peripheral Access Layer for SPC
 *
 * CMSIS Peripheral Access Layer for SPC
 */

#if !defined(PERI_SPC_H_)
#define PERI_SPC_H_                              /**< Symbol preventing repeated inclusion */

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
   -- SPC Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SPC_Peripheral_Access_Layer SPC Peripheral Access Layer
 * @{
 */

/** SPC - Size of Registers Arrays */
#define SPC_PD_STATUS_COUNT                       1u

/** SPC - Register Layout Typedef */
typedef struct {
  __I  uint32_t VERID;                             /**< Version ID, offset: 0x0 */
       uint8_t RESERVED_0[12];
  __IO uint32_t SC;                                /**< Status Control, offset: 0x10 */
       uint8_t RESERVED_1[8];
  __IO uint32_t LPREQ_CFG;                         /**< Low-Power Request Configuration, offset: 0x1C */
  __IO uint32_t CFG;                               /**< SPC Configuration, offset: 0x20 */
       uint8_t RESERVED_2[12];
  __IO uint32_t PD_STATUS[SPC_PD_STATUS_COUNT];    /**< SPC Power Domain Mode Status, array offset: 0x30, array step: 0x4 */
       uint8_t RESERVED_3[12];
  __IO uint32_t SRAMCTL;                           /**< SRAM Control, offset: 0x40 */
       uint8_t RESERVED_4[16];
  __IO uint32_t SRAMRETLDO_REFTRIM;                /**< SRAM Retention Reference Trim, offset: 0x54 */
  __IO uint32_t SRAMRETLDO_CNTRL;                  /**< SRAM Retention LDO Control, offset: 0x58 */
       uint8_t RESERVED_5[164];
  __IO uint32_t ACTIVE_CFG;                        /**< Active Power Mode Configuration, offset: 0x100 */
  __IO uint32_t ACTIVE_CFG1;                       /**< Active Power Mode Configuration 1, offset: 0x104 */
  __IO uint32_t LP_CFG;                            /**< Low-Power Mode Configuration, offset: 0x108 */
  __IO uint32_t LP_CFG1;                           /**< Low Power Mode Configuration 1, offset: 0x10C */
       uint8_t RESERVED_6[16];
  __IO uint32_t LPWKUP_DELAY;                      /**< Low Power Wake-Up Delay, offset: 0x120 */
  __IO uint32_t ACTIVE_VDELAY;                     /**< Active Voltage Trim Delay, offset: 0x124 */
       uint8_t RESERVED_7[8];
  __IO uint32_t VD_STAT;                           /**< Voltage Detect Status, offset: 0x130 */
  __IO uint32_t VD_CORE_CFG;                       /**< Core Voltage Detect Configuration, offset: 0x134 */
  __IO uint32_t VD_SYS_CFG;                        /**< System Voltage Detect Configuration, offset: 0x138 */
       uint8_t RESERVED_8[4];
  __IO uint32_t EVD_CFG;                           /**< External Voltage Domain Configuration, offset: 0x140 */
       uint8_t RESERVED_9[444];
       uint32_t CORELDO_CFG;                       /**< LDO_CORE Configuration, offset: 0x300 */
} SPC_Type;

/* ----------------------------------------------------------------------------
   -- SPC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SPC_Register_Masks SPC Register Masks
 * @{
 */

/*! @name VERID - Version ID */
/*! @{ */

#define SPC_VERID_FEATURE_MASK                   (0xFFFFU)
#define SPC_VERID_FEATURE_SHIFT                  (0U)
/*! FEATURE - Feature Specification Number
 *  0b0000000000000000..Standard features
 */
#define SPC_VERID_FEATURE(x)                     (((uint32_t)(((uint32_t)(x)) << SPC_VERID_FEATURE_SHIFT)) & SPC_VERID_FEATURE_MASK)

#define SPC_VERID_MINOR_MASK                     (0xFF0000U)
#define SPC_VERID_MINOR_SHIFT                    (16U)
/*! MINOR - Minor Version Number */
#define SPC_VERID_MINOR(x)                       (((uint32_t)(((uint32_t)(x)) << SPC_VERID_MINOR_SHIFT)) & SPC_VERID_MINOR_MASK)

#define SPC_VERID_MAJOR_MASK                     (0xFF000000U)
#define SPC_VERID_MAJOR_SHIFT                    (24U)
/*! MAJOR - Major Version Number */
#define SPC_VERID_MAJOR(x)                       (((uint32_t)(((uint32_t)(x)) << SPC_VERID_MAJOR_SHIFT)) & SPC_VERID_MAJOR_MASK)
/*! @} */

/*! @name SC - Status Control */
/*! @{ */

#define SPC_SC_BUSY_MASK                         (0x1U)
#define SPC_SC_BUSY_SHIFT                        (0U)
/*! BUSY - SPC Busy Status Flag
 *  0b0..Not busy
 *  0b1..Busy
 */
#define SPC_SC_BUSY(x)                           (((uint32_t)(((uint32_t)(x)) << SPC_SC_BUSY_SHIFT)) & SPC_SC_BUSY_MASK)

#define SPC_SC_SPC_LP_REQ_MASK                   (0x2U)
#define SPC_SC_SPC_LP_REQ_SHIFT                  (1U)
/*! SPC_LP_REQ - SPC Power Mode Configuration Status Flag
 *  0b0..No effect
 *  0b0..SPC is in Active mode; the ACTIVE_CFG register has control
 *  0b1..All power domains requested low-power mode; SPC entered a low-power state; power-mode configuration based on the LP_CFG register
 *  0b1..Clear the flag
 */
#define SPC_SC_SPC_LP_REQ(x)                     (((uint32_t)(((uint32_t)(x)) << SPC_SC_SPC_LP_REQ_SHIFT)) & SPC_SC_SPC_LP_REQ_MASK)

#define SPC_SC_SPC_LP_MODE_MASK                  (0xF0U)
#define SPC_SC_SPC_LP_MODE_SHIFT                 (4U)
/*! SPC_LP_MODE - Power Domain Low-Power Mode Request
 *  0b0000..Sleep mode with system clock running
 *  0b0001..DSLEEP with system clock off
 *  0b0010..PDOWN with system clock off
 *  0b0100..
 *  0b1000..DPDOWN with system clock off
 */
#define SPC_SC_SPC_LP_MODE(x)                    (((uint32_t)(((uint32_t)(x)) << SPC_SC_SPC_LP_MODE_SHIFT)) & SPC_SC_SPC_LP_MODE_MASK)

#define SPC_SC_ISO_CLR_MASK                      (0x10000U)
#define SPC_SC_ISO_CLR_SHIFT                     (16U)
/*! ISO_CLR - Isolation Clear Flags */
#define SPC_SC_ISO_CLR(x)                        (((uint32_t)(((uint32_t)(x)) << SPC_SC_ISO_CLR_SHIFT)) & SPC_SC_ISO_CLR_MASK)

#define SPC_SC_SWITCH_STATE_MASK                 (0x80000000U)
#define SPC_SC_SWITCH_STATE_SHIFT                (31U)
/*! SWITCH_STATE - Power Switch State
 *  0b0..Off
 *  0b1..On
 */
#define SPC_SC_SWITCH_STATE(x)                   (((uint32_t)(((uint32_t)(x)) << SPC_SC_SWITCH_STATE_SHIFT)) & SPC_SC_SWITCH_STATE_MASK)
/*! @} */

/*! @name LPREQ_CFG - Low-Power Request Configuration */
/*! @{ */

#define SPC_LPREQ_CFG_LPREQOE_MASK               (0x1U)
#define SPC_LPREQ_CFG_LPREQOE_SHIFT              (0U)
/*! LPREQOE - Low-Power Request Output Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define SPC_LPREQ_CFG_LPREQOE(x)                 (((uint32_t)(((uint32_t)(x)) << SPC_LPREQ_CFG_LPREQOE_SHIFT)) & SPC_LPREQ_CFG_LPREQOE_MASK)

#define SPC_LPREQ_CFG_LPREQPOL_MASK              (0x2U)
#define SPC_LPREQ_CFG_LPREQPOL_SHIFT             (1U)
/*! LPREQPOL - Low-Power Request Output Pin Polarity Control
 *  0b0..High
 *  0b1..Low
 */
#define SPC_LPREQ_CFG_LPREQPOL(x)                (((uint32_t)(((uint32_t)(x)) << SPC_LPREQ_CFG_LPREQPOL_SHIFT)) & SPC_LPREQ_CFG_LPREQPOL_MASK)

#define SPC_LPREQ_CFG_LPREQOV_MASK               (0xCU)
#define SPC_LPREQ_CFG_LPREQOV_SHIFT              (2U)
/*! LPREQOV - Low-Power Request Output Override
 *  0b00..Not forced
 *  0b01..
 *  0b10..Forced low (ignore LPREQPOL settings)
 *  0b11..Forced high (ignore LPREQPOL settings)
 */
#define SPC_LPREQ_CFG_LPREQOV(x)                 (((uint32_t)(((uint32_t)(x)) << SPC_LPREQ_CFG_LPREQOV_SHIFT)) & SPC_LPREQ_CFG_LPREQOV_MASK)
/*! @} */

/*! @name CFG - SPC Configuration */
/*! @{ */

#define SPC_CFG_INTG_PWSWTCH_SLEEP_EN_MASK       (0x1U)
#define SPC_CFG_INTG_PWSWTCH_SLEEP_EN_SHIFT      (0U)
/*! INTG_PWSWTCH_SLEEP_EN - Integrated Power Switch Sleep Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define SPC_CFG_INTG_PWSWTCH_SLEEP_EN(x)         (((uint32_t)(((uint32_t)(x)) << SPC_CFG_INTG_PWSWTCH_SLEEP_EN_SHIFT)) & SPC_CFG_INTG_PWSWTCH_SLEEP_EN_MASK)

#define SPC_CFG_INTG_PWSWTCH_WKUP_EN_MASK        (0x2U)
#define SPC_CFG_INTG_PWSWTCH_WKUP_EN_SHIFT       (1U)
/*! INTG_PWSWTCH_WKUP_EN - Integrated Power Switch Wake-up Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define SPC_CFG_INTG_PWSWTCH_WKUP_EN(x)          (((uint32_t)(((uint32_t)(x)) << SPC_CFG_INTG_PWSWTCH_WKUP_EN_SHIFT)) & SPC_CFG_INTG_PWSWTCH_WKUP_EN_MASK)

#define SPC_CFG_INTG_PWSWTCH_SLEEP_ACTIVE_EN_MASK (0x4U)
#define SPC_CFG_INTG_PWSWTCH_SLEEP_ACTIVE_EN_SHIFT (2U)
/*! INTG_PWSWTCH_SLEEP_ACTIVE_EN - Integrated Power Switch Active Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define SPC_CFG_INTG_PWSWTCH_SLEEP_ACTIVE_EN(x)  (((uint32_t)(((uint32_t)(x)) << SPC_CFG_INTG_PWSWTCH_SLEEP_ACTIVE_EN_SHIFT)) & SPC_CFG_INTG_PWSWTCH_SLEEP_ACTIVE_EN_MASK)

#define SPC_CFG_INTG_PWSWTCH_WKUP_ACTIVE_EN_MASK (0x8U)
#define SPC_CFG_INTG_PWSWTCH_WKUP_ACTIVE_EN_SHIFT (3U)
/*! INTG_PWSWTCH_WKUP_ACTIVE_EN - Integrated Power Switch Wake-up Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define SPC_CFG_INTG_PWSWTCH_WKUP_ACTIVE_EN(x)   (((uint32_t)(((uint32_t)(x)) << SPC_CFG_INTG_PWSWTCH_WKUP_ACTIVE_EN_SHIFT)) & SPC_CFG_INTG_PWSWTCH_WKUP_ACTIVE_EN_MASK)
/*! @} */

/*! @name PD_STATUS - SPC Power Domain Mode Status */
/*! @{ */

#define SPC_PD_STATUS_PWR_REQ_STATUS_MASK        (0x1U)
#define SPC_PD_STATUS_PWR_REQ_STATUS_SHIFT       (0U)
/*! PWR_REQ_STATUS - Power Request Status Flag
 *  0b0..Did not request
 *  0b1..Requested
 */
#define SPC_PD_STATUS_PWR_REQ_STATUS(x)          (((uint32_t)(((uint32_t)(x)) << SPC_PD_STATUS_PWR_REQ_STATUS_SHIFT)) & SPC_PD_STATUS_PWR_REQ_STATUS_MASK)

#define SPC_PD_STATUS_PD_LP_REQ_MASK             (0x10U)
#define SPC_PD_STATUS_PD_LP_REQ_SHIFT            (4U)
/*! PD_LP_REQ - Power Domain Low Power Request Flag
 *  0b0..Did not request
 *  0b1..Requested
 */
#define SPC_PD_STATUS_PD_LP_REQ(x)               (((uint32_t)(((uint32_t)(x)) << SPC_PD_STATUS_PD_LP_REQ_SHIFT)) & SPC_PD_STATUS_PD_LP_REQ_MASK)

#define SPC_PD_STATUS_LP_MODE_MASK               (0xF00U)
#define SPC_PD_STATUS_LP_MODE_SHIFT              (8U)
/*! LP_MODE - Power Domain Low Power Mode Request
 *  0b0000..SLEEP with system clock running
 *  0b0001..DSLEEP with system clock off
 *  0b0010..PDOWN with system clock off
 *  0b0100..
 *  0b1000..DPDOWN with system clock off
 */
#define SPC_PD_STATUS_LP_MODE(x)                 (((uint32_t)(((uint32_t)(x)) << SPC_PD_STATUS_LP_MODE_SHIFT)) & SPC_PD_STATUS_LP_MODE_MASK)
/*! @} */

/*! @name SRAMCTL - SRAM Control */
/*! @{ */

#define SPC_SRAMCTL_VSM_MASK                     (0x3U)
#define SPC_SRAMCTL_VSM_SHIFT                    (0U)
/*! VSM - Voltage Select Margin
 *  0b00..
 *  0b01..1.0 V
 *  0b10..1.1 V
 *  0b11..SRAM configured for 1.2 V operation
 */
#define SPC_SRAMCTL_VSM(x)                       (((uint32_t)(((uint32_t)(x)) << SPC_SRAMCTL_VSM_SHIFT)) & SPC_SRAMCTL_VSM_MASK)

#define SPC_SRAMCTL_REQ_MASK                     (0x40000000U)
#define SPC_SRAMCTL_REQ_SHIFT                    (30U)
/*! REQ - SRAM Voltage Update Request
 *  0b0..Do not request
 *  0b1..Request
 */
#define SPC_SRAMCTL_REQ(x)                       (((uint32_t)(((uint32_t)(x)) << SPC_SRAMCTL_REQ_SHIFT)) & SPC_SRAMCTL_REQ_MASK)

#define SPC_SRAMCTL_ACK_MASK                     (0x80000000U)
#define SPC_SRAMCTL_ACK_SHIFT                    (31U)
/*! ACK - SRAM Voltage Update Request Acknowledge
 *  0b0..Not acknowledged
 *  0b1..Acknowledged
 */
#define SPC_SRAMCTL_ACK(x)                       (((uint32_t)(((uint32_t)(x)) << SPC_SRAMCTL_ACK_SHIFT)) & SPC_SRAMCTL_ACK_MASK)
/*! @} */

/*! @name SRAMRETLDO_REFTRIM - SRAM Retention Reference Trim */
/*! @{ */

#define SPC_SRAMRETLDO_REFTRIM_REFTRIM_MASK      (0x1FU)
#define SPC_SRAMRETLDO_REFTRIM_REFTRIM_SHIFT     (0U)
/*! REFTRIM - Reference Trim. Voltage range is around 0.48V - 0.85V. Trim step is 12 mV. */
#define SPC_SRAMRETLDO_REFTRIM_REFTRIM(x)        (((uint32_t)(((uint32_t)(x)) << SPC_SRAMRETLDO_REFTRIM_REFTRIM_SHIFT)) & SPC_SRAMRETLDO_REFTRIM_REFTRIM_MASK)
/*! @} */

/*! @name SRAMRETLDO_CNTRL - SRAM Retention LDO Control */
/*! @{ */

#define SPC_SRAMRETLDO_CNTRL_SRAMLDO_ON_MASK     (0x1U)
#define SPC_SRAMRETLDO_CNTRL_SRAMLDO_ON_SHIFT    (0U)
/*! SRAMLDO_ON - SRAM LDO Regulator Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define SPC_SRAMRETLDO_CNTRL_SRAMLDO_ON(x)       (((uint32_t)(((uint32_t)(x)) << SPC_SRAMRETLDO_CNTRL_SRAMLDO_ON_SHIFT)) & SPC_SRAMRETLDO_CNTRL_SRAMLDO_ON_MASK)

#define SPC_SRAMRETLDO_CNTRL_SRAM_RET_EN_MASK    (0xF00U)
#define SPC_SRAMRETLDO_CNTRL_SRAM_RET_EN_SHIFT   (8U)
/*! SRAM_RET_EN - SRAM Retention */
#define SPC_SRAMRETLDO_CNTRL_SRAM_RET_EN(x)      (((uint32_t)(((uint32_t)(x)) << SPC_SRAMRETLDO_CNTRL_SRAM_RET_EN_SHIFT)) & SPC_SRAMRETLDO_CNTRL_SRAM_RET_EN_MASK)
/*! @} */

/*! @name ACTIVE_CFG - Active Power Mode Configuration */
/*! @{ */

#define SPC_ACTIVE_CFG_CORELDO_VDD_DS_MASK       (0x1U)
#define SPC_ACTIVE_CFG_CORELDO_VDD_DS_SHIFT      (0U)
/*! CORELDO_VDD_DS - LDO_CORE VDD Drive Strength
 *  0b0..Low
 *  0b1..Normal
 */
#define SPC_ACTIVE_CFG_CORELDO_VDD_DS(x)         (((uint32_t)(((uint32_t)(x)) << SPC_ACTIVE_CFG_CORELDO_VDD_DS_SHIFT)) & SPC_ACTIVE_CFG_CORELDO_VDD_DS_MASK)

#define SPC_ACTIVE_CFG_CORELDO_VDD_LVL_MASK      (0xCU)
#define SPC_ACTIVE_CFG_CORELDO_VDD_LVL_SHIFT     (2U)
/*! CORELDO_VDD_LVL - LDO_CORE VDD Regulator Voltage Level
 *  0b00..
 *  0b01..Regulate to mid voltage (1.0 V)
 *  0b10..Regulate to normal voltage (1.1 V)
 *  0b11..Regulate to overdrive voltage (1.15 V)
 */
#define SPC_ACTIVE_CFG_CORELDO_VDD_LVL(x)        (((uint32_t)(((uint32_t)(x)) << SPC_ACTIVE_CFG_CORELDO_VDD_LVL_SHIFT)) & SPC_ACTIVE_CFG_CORELDO_VDD_LVL_MASK)

#define SPC_ACTIVE_CFG_BGMODE_MASK               (0x300000U)
#define SPC_ACTIVE_CFG_BGMODE_SHIFT              (20U)
/*! BGMODE - Bandgap Mode
 *  0b00..Bandgap disabled
 *  0b01..Bandgap enabled, buffer disabled
 *  0b10..Bandgap enabled, buffer enabled
 *  0b11..
 */
#define SPC_ACTIVE_CFG_BGMODE(x)                 (((uint32_t)(((uint32_t)(x)) << SPC_ACTIVE_CFG_BGMODE_SHIFT)) & SPC_ACTIVE_CFG_BGMODE_MASK)

#define SPC_ACTIVE_CFG_VDD_VD_DISABLE_MASK       (0x800000U)
#define SPC_ACTIVE_CFG_VDD_VD_DISABLE_SHIFT      (23U)
/*! VDD_VD_DISABLE - VDD Voltage Detect Disable
 *  0b0..Enable
 *  0b1..Disable
 */
#define SPC_ACTIVE_CFG_VDD_VD_DISABLE(x)         (((uint32_t)(((uint32_t)(x)) << SPC_ACTIVE_CFG_VDD_VD_DISABLE_SHIFT)) & SPC_ACTIVE_CFG_VDD_VD_DISABLE_MASK)

#define SPC_ACTIVE_CFG_CORE_LVDE_MASK            (0x1000000U)
#define SPC_ACTIVE_CFG_CORE_LVDE_SHIFT           (24U)
/*! CORE_LVDE - Core Low-Voltage Detection Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define SPC_ACTIVE_CFG_CORE_LVDE(x)              (((uint32_t)(((uint32_t)(x)) << SPC_ACTIVE_CFG_CORE_LVDE_SHIFT)) & SPC_ACTIVE_CFG_CORE_LVDE_MASK)

#define SPC_ACTIVE_CFG_SYS_LVDE_MASK             (0x2000000U)
#define SPC_ACTIVE_CFG_SYS_LVDE_SHIFT            (25U)
/*! SYS_LVDE - System Low-Voltage Detection Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define SPC_ACTIVE_CFG_SYS_LVDE(x)               (((uint32_t)(((uint32_t)(x)) << SPC_ACTIVE_CFG_SYS_LVDE_SHIFT)) & SPC_ACTIVE_CFG_SYS_LVDE_MASK)

#define SPC_ACTIVE_CFG_SYS_HVDE_MASK             (0x10000000U)
#define SPC_ACTIVE_CFG_SYS_HVDE_SHIFT            (28U)
/*! SYS_HVDE - System High-Voltage Detection Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define SPC_ACTIVE_CFG_SYS_HVDE(x)               (((uint32_t)(((uint32_t)(x)) << SPC_ACTIVE_CFG_SYS_HVDE_SHIFT)) & SPC_ACTIVE_CFG_SYS_HVDE_MASK)
/*! @} */

/*! @name ACTIVE_CFG1 - Active Power Mode Configuration 1 */
/*! @{ */

#define SPC_ACTIVE_CFG1_SOC_CNTRL_MASK           (0xFFFFFFFFU)
#define SPC_ACTIVE_CFG1_SOC_CNTRL_SHIFT          (0U)
/*! SOC_CNTRL - Active Config Chip Control */
#define SPC_ACTIVE_CFG1_SOC_CNTRL(x)             (((uint32_t)(((uint32_t)(x)) << SPC_ACTIVE_CFG1_SOC_CNTRL_SHIFT)) & SPC_ACTIVE_CFG1_SOC_CNTRL_MASK)
/*! @} */

/*! @name LP_CFG - Low-Power Mode Configuration */
/*! @{ */

#define SPC_LP_CFG_CORELDO_VDD_DS_MASK           (0x1U)
#define SPC_LP_CFG_CORELDO_VDD_DS_SHIFT          (0U)
/*! CORELDO_VDD_DS - LDO_CORE VDD Drive Strength
 *  0b0..Low
 *  0b1..Normal
 */
#define SPC_LP_CFG_CORELDO_VDD_DS(x)             (((uint32_t)(((uint32_t)(x)) << SPC_LP_CFG_CORELDO_VDD_DS_SHIFT)) & SPC_LP_CFG_CORELDO_VDD_DS_MASK)

#define SPC_LP_CFG_CORELDO_VDD_LVL_MASK          (0xCU)
#define SPC_LP_CFG_CORELDO_VDD_LVL_SHIFT         (2U)
/*! CORELDO_VDD_LVL - LDO_CORE VDD Regulator Voltage Level
 *  0b00..Reserved
 *  0b01..Mid voltage (1.0 V)
 *  0b10..Normal voltage (1.1 V)
 *  0b11..Overdrive voltage (1.15 V)
 */
#define SPC_LP_CFG_CORELDO_VDD_LVL(x)            (((uint32_t)(((uint32_t)(x)) << SPC_LP_CFG_CORELDO_VDD_LVL_SHIFT)) & SPC_LP_CFG_CORELDO_VDD_LVL_MASK)

#define SPC_LP_CFG_SRAMLDO_DPD_ON_MASK           (0x80000U)
#define SPC_LP_CFG_SRAMLDO_DPD_ON_SHIFT          (19U)
/*! SRAMLDO_DPD_ON - SRAM_LDO Deep Power Low Power IREF Enable
 *  0b0..Low Power IREF is disabled for power saving in Deep Power Down mode
 *  0b1..Low Power IREF is enabled
 */
#define SPC_LP_CFG_SRAMLDO_DPD_ON(x)             (((uint32_t)(((uint32_t)(x)) << SPC_LP_CFG_SRAMLDO_DPD_ON_SHIFT)) & SPC_LP_CFG_SRAMLDO_DPD_ON_MASK)

#define SPC_LP_CFG_BGMODE_MASK                   (0x300000U)
#define SPC_LP_CFG_BGMODE_SHIFT                  (20U)
/*! BGMODE - Bandgap Mode
 *  0b00..Bandgap disabled
 *  0b01..Bandgap enabled, buffer disabled
 *  0b10..Bandgap enabled, buffer enabled
 *  0b11..
 */
#define SPC_LP_CFG_BGMODE(x)                     (((uint32_t)(((uint32_t)(x)) << SPC_LP_CFG_BGMODE_SHIFT)) & SPC_LP_CFG_BGMODE_MASK)

#define SPC_LP_CFG_LP_IREFEN_MASK                (0x800000U)
#define SPC_LP_CFG_LP_IREFEN_SHIFT               (23U)
/*! LP_IREFEN - Low-Power IREF Enable
 *  0b0..Disable for power saving in Deep Power Down mode
 *  0b1..Enable
 */
#define SPC_LP_CFG_LP_IREFEN(x)                  (((uint32_t)(((uint32_t)(x)) << SPC_LP_CFG_LP_IREFEN_SHIFT)) & SPC_LP_CFG_LP_IREFEN_MASK)

#define SPC_LP_CFG_CORE_LVDE_MASK                (0x1000000U)
#define SPC_LP_CFG_CORE_LVDE_SHIFT               (24U)
/*! CORE_LVDE - Core Low Voltage Detect Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define SPC_LP_CFG_CORE_LVDE(x)                  (((uint32_t)(((uint32_t)(x)) << SPC_LP_CFG_CORE_LVDE_SHIFT)) & SPC_LP_CFG_CORE_LVDE_MASK)

#define SPC_LP_CFG_SYS_LVDE_MASK                 (0x2000000U)
#define SPC_LP_CFG_SYS_LVDE_SHIFT                (25U)
/*! SYS_LVDE - System Low Voltage Detect Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define SPC_LP_CFG_SYS_LVDE(x)                   (((uint32_t)(((uint32_t)(x)) << SPC_LP_CFG_SYS_LVDE_SHIFT)) & SPC_LP_CFG_SYS_LVDE_MASK)

#define SPC_LP_CFG_SYS_HVDE_MASK                 (0x10000000U)
#define SPC_LP_CFG_SYS_HVDE_SHIFT                (28U)
/*! SYS_HVDE - System High Voltage Detect Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define SPC_LP_CFG_SYS_HVDE(x)                   (((uint32_t)(((uint32_t)(x)) << SPC_LP_CFG_SYS_HVDE_SHIFT)) & SPC_LP_CFG_SYS_HVDE_MASK)
/*! @} */

/*! @name LP_CFG1 - Low Power Mode Configuration 1 */
/*! @{ */

#define SPC_LP_CFG1_SOC_CNTRL_MASK               (0xFFFFFFFFU)
#define SPC_LP_CFG1_SOC_CNTRL_SHIFT              (0U)
/*! SOC_CNTRL - Low-Power Configuration Chip Control */
#define SPC_LP_CFG1_SOC_CNTRL(x)                 (((uint32_t)(((uint32_t)(x)) << SPC_LP_CFG1_SOC_CNTRL_SHIFT)) & SPC_LP_CFG1_SOC_CNTRL_MASK)
/*! @} */

/*! @name LPWKUP_DELAY - Low Power Wake-Up Delay */
/*! @{ */

#define SPC_LPWKUP_DELAY_LPWKUP_DELAY_MASK       (0xFFFFU)
#define SPC_LPWKUP_DELAY_LPWKUP_DELAY_SHIFT      (0U)
/*! LPWKUP_DELAY - Low-Power Wake-Up Delay */
#define SPC_LPWKUP_DELAY_LPWKUP_DELAY(x)         (((uint32_t)(((uint32_t)(x)) << SPC_LPWKUP_DELAY_LPWKUP_DELAY_SHIFT)) & SPC_LPWKUP_DELAY_LPWKUP_DELAY_MASK)
/*! @} */

/*! @name ACTIVE_VDELAY - Active Voltage Trim Delay */
/*! @{ */

#define SPC_ACTIVE_VDELAY_ACTIVE_VDELAY_MASK     (0xFFFFU)
#define SPC_ACTIVE_VDELAY_ACTIVE_VDELAY_SHIFT    (0U)
/*! ACTIVE_VDELAY - Active Voltage Delay */
#define SPC_ACTIVE_VDELAY_ACTIVE_VDELAY(x)       (((uint32_t)(((uint32_t)(x)) << SPC_ACTIVE_VDELAY_ACTIVE_VDELAY_SHIFT)) & SPC_ACTIVE_VDELAY_ACTIVE_VDELAY_MASK)
/*! @} */

/*! @name VD_STAT - Voltage Detect Status */
/*! @{ */

#define SPC_VD_STAT_COREVDD_LVDF_MASK            (0x1U)
#define SPC_VD_STAT_COREVDD_LVDF_SHIFT           (0U)
/*! COREVDD_LVDF - Core Low-Voltage Detect Flag
 *  0b0..Event not detected
 *  0b0..No effect
 *  0b1..Clear the flag
 *  0b1..Event detected
 */
#define SPC_VD_STAT_COREVDD_LVDF(x)              (((uint32_t)(((uint32_t)(x)) << SPC_VD_STAT_COREVDD_LVDF_SHIFT)) & SPC_VD_STAT_COREVDD_LVDF_MASK)

#define SPC_VD_STAT_SYSVDD_LVDF_MASK             (0x2U)
#define SPC_VD_STAT_SYSVDD_LVDF_SHIFT            (1U)
/*! SYSVDD_LVDF - System Low-Voltage Detect Flag
 *  0b0..Event not detected
 *  0b0..No effect
 *  0b1..Clear the flag
 *  0b1..Event detected
 */
#define SPC_VD_STAT_SYSVDD_LVDF(x)               (((uint32_t)(((uint32_t)(x)) << SPC_VD_STAT_SYSVDD_LVDF_SHIFT)) & SPC_VD_STAT_SYSVDD_LVDF_MASK)

#define SPC_VD_STAT_SYSVDD_HVDF_MASK             (0x20U)
#define SPC_VD_STAT_SYSVDD_HVDF_SHIFT            (5U)
/*! SYSVDD_HVDF - System HVD Flag
 *  0b0..Event not detected
 *  0b0..No effect
 *  0b1..Clear the flag
 *  0b1..Event detected
 */
#define SPC_VD_STAT_SYSVDD_HVDF(x)               (((uint32_t)(((uint32_t)(x)) << SPC_VD_STAT_SYSVDD_HVDF_SHIFT)) & SPC_VD_STAT_SYSVDD_HVDF_MASK)
/*! @} */

/*! @name VD_CORE_CFG - Core Voltage Detect Configuration */
/*! @{ */

#define SPC_VD_CORE_CFG_LVDRE_MASK               (0x1U)
#define SPC_VD_CORE_CFG_LVDRE_SHIFT              (0U)
/*! LVDRE - Core LVD Reset Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define SPC_VD_CORE_CFG_LVDRE(x)                 (((uint32_t)(((uint32_t)(x)) << SPC_VD_CORE_CFG_LVDRE_SHIFT)) & SPC_VD_CORE_CFG_LVDRE_MASK)

#define SPC_VD_CORE_CFG_LVDIE_MASK               (0x2U)
#define SPC_VD_CORE_CFG_LVDIE_SHIFT              (1U)
/*! LVDIE - Core LVD Interrupt Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define SPC_VD_CORE_CFG_LVDIE(x)                 (((uint32_t)(((uint32_t)(x)) << SPC_VD_CORE_CFG_LVDIE_SHIFT)) & SPC_VD_CORE_CFG_LVDIE_MASK)

#define SPC_VD_CORE_CFG_LOCK_MASK                (0x10000U)
#define SPC_VD_CORE_CFG_LOCK_SHIFT               (16U)
/*! LOCK - Core Voltage Detect Reset Enable Lock
 *  0b0..Allow
 *  0b1..Deny
 */
#define SPC_VD_CORE_CFG_LOCK(x)                  (((uint32_t)(((uint32_t)(x)) << SPC_VD_CORE_CFG_LOCK_SHIFT)) & SPC_VD_CORE_CFG_LOCK_MASK)
/*! @} */

/*! @name VD_SYS_CFG - System Voltage Detect Configuration */
/*! @{ */

#define SPC_VD_SYS_CFG_LVDRE_MASK                (0x1U)
#define SPC_VD_SYS_CFG_LVDRE_SHIFT               (0U)
/*! LVDRE - System LVD Reset Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define SPC_VD_SYS_CFG_LVDRE(x)                  (((uint32_t)(((uint32_t)(x)) << SPC_VD_SYS_CFG_LVDRE_SHIFT)) & SPC_VD_SYS_CFG_LVDRE_MASK)

#define SPC_VD_SYS_CFG_LVDIE_MASK                (0x2U)
#define SPC_VD_SYS_CFG_LVDIE_SHIFT               (1U)
/*! LVDIE - System LVD Interrupt Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define SPC_VD_SYS_CFG_LVDIE(x)                  (((uint32_t)(((uint32_t)(x)) << SPC_VD_SYS_CFG_LVDIE_SHIFT)) & SPC_VD_SYS_CFG_LVDIE_MASK)

#define SPC_VD_SYS_CFG_HVDRE_MASK                (0x4U)
#define SPC_VD_SYS_CFG_HVDRE_SHIFT               (2U)
/*! HVDRE - System HVD Reset Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define SPC_VD_SYS_CFG_HVDRE(x)                  (((uint32_t)(((uint32_t)(x)) << SPC_VD_SYS_CFG_HVDRE_SHIFT)) & SPC_VD_SYS_CFG_HVDRE_MASK)

#define SPC_VD_SYS_CFG_HVDIE_MASK                (0x8U)
#define SPC_VD_SYS_CFG_HVDIE_SHIFT               (3U)
/*! HVDIE - System HVD Interrupt Enable
 *  0b0..Disable
 *  0b1..Enable
 */
#define SPC_VD_SYS_CFG_HVDIE(x)                  (((uint32_t)(((uint32_t)(x)) << SPC_VD_SYS_CFG_HVDIE_SHIFT)) & SPC_VD_SYS_CFG_HVDIE_MASK)

#define SPC_VD_SYS_CFG_LVSEL_MASK                (0x100U)
#define SPC_VD_SYS_CFG_LVSEL_SHIFT               (8U)
/*! LVSEL - System Low-Voltage Level Select
 *  0b0..Normal
 *  0b1..Safe
 */
#define SPC_VD_SYS_CFG_LVSEL(x)                  (((uint32_t)(((uint32_t)(x)) << SPC_VD_SYS_CFG_LVSEL_SHIFT)) & SPC_VD_SYS_CFG_LVSEL_MASK)

#define SPC_VD_SYS_CFG_LOCK_MASK                 (0x10000U)
#define SPC_VD_SYS_CFG_LOCK_SHIFT                (16U)
/*! LOCK - System Voltage Detect Reset Enable Lock
 *  0b0..Allow
 *  0b1..Deny
 */
#define SPC_VD_SYS_CFG_LOCK(x)                   (((uint32_t)(((uint32_t)(x)) << SPC_VD_SYS_CFG_LOCK_SHIFT)) & SPC_VD_SYS_CFG_LOCK_MASK)
/*! @} */

/*! @name EVD_CFG - External Voltage Domain Configuration */
/*! @{ */

#define SPC_EVD_CFG_EVDISO_MASK                  (0x7U)
#define SPC_EVD_CFG_EVDISO_SHIFT                 (0U)
/*! EVDISO - External Voltage Domain Isolation */
#define SPC_EVD_CFG_EVDISO(x)                    (((uint32_t)(((uint32_t)(x)) << SPC_EVD_CFG_EVDISO_SHIFT)) & SPC_EVD_CFG_EVDISO_MASK)

#define SPC_EVD_CFG_EVDLPISO_MASK                (0x700U)
#define SPC_EVD_CFG_EVDLPISO_SHIFT               (8U)
/*! EVDLPISO - External Voltage Domain Low-Power Isolation */
#define SPC_EVD_CFG_EVDLPISO(x)                  (((uint32_t)(((uint32_t)(x)) << SPC_EVD_CFG_EVDLPISO_SHIFT)) & SPC_EVD_CFG_EVDLPISO_MASK)

#define SPC_EVD_CFG_EVDSTAT_MASK                 (0x70000U)
#define SPC_EVD_CFG_EVDSTAT_SHIFT                (16U)
/*! EVDSTAT - External Voltage Domain Status */
#define SPC_EVD_CFG_EVDSTAT(x)                   (((uint32_t)(((uint32_t)(x)) << SPC_EVD_CFG_EVDSTAT_SHIFT)) & SPC_EVD_CFG_EVDSTAT_MASK)
/*! @} */


/*!
 * @}
 */ /* end of group SPC_Register_Masks */


/*!
 * @}
 */ /* end of group SPC_Peripheral_Access_Layer */


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


#endif  /* PERI_SPC_H_ */

