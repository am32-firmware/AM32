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
**     Build:               b260805
**
**     Abstract:
**         CMSIS Peripheral Access Layer for SYSCON
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
 * @file PERI_SYSCON.h
 * @version 2.0
 * @date 2024-10-29
 * @brief CMSIS Peripheral Access Layer for SYSCON
 *
 * CMSIS Peripheral Access Layer for SYSCON
 */

#if !defined(PERI_SYSCON_H_)
#define PERI_SYSCON_H_                           /**< Symbol preventing repeated inclusion */

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
   -- SYSCON Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SYSCON_Peripheral_Access_Layer SYSCON Peripheral Access Layer
 * @{
 */

/** SYSCON - Register Layout Typedef */
typedef struct {
       uint8_t RESERVED_0[512];
  __IO uint32_t REMAP;                             /**< AHB Matrix Remap Control, offset: 0x200 */
       uint8_t RESERVED_1[12];
  __IO uint32_t AHBMATPRIO;                        /**< AHB Matrix Priority Control, offset: 0x210 */
       uint8_t RESERVED_2[40];
  __IO uint32_t CPU0NSTCKCAL;                      /**< Non-Secure CPU0 System Tick Calibration, offset: 0x23C */
       uint8_t RESERVED_3[8];
  __IO uint32_t NMISRC;                            /**< NMI Source Select, offset: 0x248 */
       uint8_t RESERVED_4[300];
  __IO uint32_t SLOWCLKDIV;                        /**< SLOW_CLK Clock Divider, offset: 0x378 */
       uint8_t RESERVED_5[4];
  __IO uint32_t AHBCLKDIV;                         /**< System Clock Divider, offset: 0x380 */
       uint8_t RESERVED_6[120];
  __IO uint32_t CLKUNLOCK;                         /**< Clock Configuration Unlock, offset: 0x3FC */
  __IO uint32_t NVM_CTRL;                          /**< NVM Control, offset: 0x400 */
  __I  uint32_t ROMCR;                             /**< ROM Wait State, offset: 0x404 */
       uint8_t RESERVED_7[1028];
  __I  uint32_t CPUSTAT;                           /**< CPU Status, offset: 0x80C */
       uint8_t RESERVED_8[20];
  __IO uint32_t LPCAC_CTRL;                        /**< LPCAC Control, offset: 0x824 */
       uint8_t RESERVED_9[272];
  __IO uint32_t PWM0SUBCTL;                        /**< PWM0 Submodule Control, offset: 0x938 */
       uint8_t RESERVED_10[4];
  __IO uint32_t CTIMERGLOBALSTARTEN;               /**< CTIMER Global Start Enable, offset: 0x940 */
  __IO uint32_t RAM_CTRL;                          /**< RAM Control, offset: 0x944 */
       uint8_t RESERVED_11[536];
  __IO uint32_t GRAY_CODE_LSB;                     /**< Gray to Binary Converter Gray Code [31:0], offset: 0xB60 */
  __IO uint32_t GRAY_CODE_MSB;                     /**< Gray to Binary Converter Gray Code [41:32], offset: 0xB64 */
  __I  uint32_t BINARY_CODE_LSB;                   /**< Gray to Binary Converter Binary Code [31:0], offset: 0xB68 */
  __I  uint32_t BINARY_CODE_MSB;                   /**< Gray to Binary Converter Binary Code [41:32], offset: 0xB6C */
       uint8_t RESERVED_12[720];
  __I  uint32_t OVP_PAD_STATE;                     /**< OVP_PAD_STATE, offset: 0xE40 */
  __I  uint32_t PROBE_STATE;                       /**< PROBE_STATE, offset: 0xE44 */
  __I  uint32_t FT_STATE_A;                        /**< FT_STATE_A, offset: 0xE48 */
  __I  uint32_t ROP_STATE;                         /**< ROP State Register, offset: 0xE4C */
       uint8_t RESERVED_13[8];
  __IO uint32_t SRAM_XEN;                          /**< RAM XEN Control, offset: 0xE58 */
  __IO uint32_t SRAM_XEN_DP;                       /**< RAM XEN Control (Duplicate), offset: 0xE5C */
       uint8_t RESERVED_14[32];
  __I  uint32_t ELS_OTP_LC_STATE;                  /**< Life Cycle State Register, offset: 0xE80 */
  __I  uint32_t ELS_OTP_LC_STATE_DP;               /**< Life Cycle State Register (Duplicate), offset: 0xE84 */
       uint8_t RESERVED_15[280];
  __IO uint32_t DEBUG_LOCK_EN;                     /**< Control Write Access to Security, offset: 0xFA0 */
  __IO uint32_t DEBUG_FEATURES;                    /**< Cortex Debug Features Control, offset: 0xFA4 */
  __IO uint32_t DEBUG_FEATURES_DP;                 /**< Cortex Debug Features Control (Duplicate), offset: 0xFA8 */
       uint8_t RESERVED_16[8];
  __IO uint32_t SWD_ACCESS_CPU0;                   /**< CPU0 Software Debug Access, offset: 0xFB4 */
       uint8_t RESERVED_17[8];
  __IO uint32_t DEBUG_AUTH_BEACON;                 /**< Debug Authentication BEACON, offset: 0xFC0 */
       uint8_t RESERVED_18[44];
  __I  uint32_t JTAG_ID;                           /**< JTAG Chip ID, offset: 0xFF0 */
  __I  uint32_t DEVICE_TYPE;                       /**< Device Type, offset: 0xFF4 */
  __I  uint32_t DEVICE_ID0;                        /**< Device ID, offset: 0xFF8 */
  __I  uint32_t DIEID;                             /**< Chip Revision ID and Number, offset: 0xFFC */
} SYSCON_Type;

/* ----------------------------------------------------------------------------
   -- SYSCON Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SYSCON_Register_Masks SYSCON Register Masks
 * @{
 */

/*! @name REMAP - AHB Matrix Remap Control */
/*! @{ */

#define SYSCON_REMAP_CPU0_SBUS_MASK              (0x3U)
#define SYSCON_REMAP_CPU0_SBUS_SHIFT             (0U)
/*! CPU0_SBUS - RAMX0 address remap for CPU System bus
 *  0b00..RAMX0: 0x04000000 - 0x04001fff
 *  0b01..RAMX0: 0x20006000 - 0x20007fff
 */
#define SYSCON_REMAP_CPU0_SBUS(x)                (((uint32_t)(((uint32_t)(x)) << SYSCON_REMAP_CPU0_SBUS_SHIFT)) & SYSCON_REMAP_CPU0_SBUS_MASK)

#define SYSCON_REMAP_DMA0_MASK                   (0xCU)
#define SYSCON_REMAP_DMA0_SHIFT                  (2U)
/*! DMA0 - RAMX0 address remap for DMA0
 *  0b00..RAMX0: 0x04000000 - 0x04001fff
 *  0b01..RAMX0: same alias space as CPU0_SBUS
 */
#define SYSCON_REMAP_DMA0(x)                     (((uint32_t)(((uint32_t)(x)) << SYSCON_REMAP_DMA0_SHIFT)) & SYSCON_REMAP_DMA0_MASK)

#define SYSCON_REMAP_USB0_MASK                   (0x30U)
#define SYSCON_REMAP_USB0_SHIFT                  (4U)
/*! USB0 - RAMX0 address remap for USB0
 *  0b00..RAMX0: 0x04000000 - 0x04001fff
 *  0b01..RAMX0: same alias space as CPU0_SBUS
 */
#define SYSCON_REMAP_USB0(x)                     (((uint32_t)(((uint32_t)(x)) << SYSCON_REMAP_USB0_SHIFT)) & SYSCON_REMAP_USB0_MASK)

#define SYSCON_REMAP_LOCK_MASK                   (0x80000000U)
#define SYSCON_REMAP_LOCK_SHIFT                  (31U)
/*! LOCK - This 1-bit field provides a mechanism to limit writes to the this register to protect its
 *    contents. Once set, this bit remains asserted until the next reset.
 *  0b0..This register is not locked and can be altered.
 *  0b1..This register is locked and cannot be altered.
 */
#define SYSCON_REMAP_LOCK(x)                     (((uint32_t)(((uint32_t)(x)) << SYSCON_REMAP_LOCK_SHIFT)) & SYSCON_REMAP_LOCK_MASK)
/*! @} */

/*! @name AHBMATPRIO - AHB Matrix Priority Control */
/*! @{ */

#define SYSCON_AHBMATPRIO_CPU0_CBUS_MASK         (0x3U)
#define SYSCON_AHBMATPRIO_CPU0_CBUS_SHIFT        (0U)
/*! CPU0_CBUS - CPU0 C-AHB bus master priority level
 *  0b00..level 0
 *  0b01..level 1
 *  0b10..level 2
 *  0b11..level 3
 */
#define SYSCON_AHBMATPRIO_CPU0_CBUS(x)           (((uint32_t)(((uint32_t)(x)) << SYSCON_AHBMATPRIO_CPU0_CBUS_SHIFT)) & SYSCON_AHBMATPRIO_CPU0_CBUS_MASK)

#define SYSCON_AHBMATPRIO_CPU0_SBUS_MASK         (0xCU)
#define SYSCON_AHBMATPRIO_CPU0_SBUS_SHIFT        (2U)
/*! CPU0_SBUS - CPU0 S-AHB bus master priority level
 *  0b00..level 0
 *  0b01..level 1
 *  0b10..level 2
 *  0b11..level 3
 */
#define SYSCON_AHBMATPRIO_CPU0_SBUS(x)           (((uint32_t)(((uint32_t)(x)) << SYSCON_AHBMATPRIO_CPU0_SBUS_SHIFT)) & SYSCON_AHBMATPRIO_CPU0_SBUS_MASK)

#define SYSCON_AHBMATPRIO_DMA0_MASK              (0x300U)
#define SYSCON_AHBMATPRIO_DMA0_SHIFT             (8U)
/*! DMA0 - DMA0 controller bus master priority level
 *  0b00..level 0
 *  0b01..level 1
 *  0b10..level 2
 *  0b11..level 3
 */
#define SYSCON_AHBMATPRIO_DMA0(x)                (((uint32_t)(((uint32_t)(x)) << SYSCON_AHBMATPRIO_DMA0_SHIFT)) & SYSCON_AHBMATPRIO_DMA0_MASK)

#define SYSCON_AHBMATPRIO_USB_FS_ENET_MASK       (0x3000000U)
#define SYSCON_AHBMATPRIO_USB_FS_ENET_SHIFT      (24U)
/*! USB_FS_ENET - USB-FS bus master priority level
 *  0b00..level 0
 *  0b01..level 1
 *  0b10..level 2
 *  0b11..level 3
 */
#define SYSCON_AHBMATPRIO_USB_FS_ENET(x)         (((uint32_t)(((uint32_t)(x)) << SYSCON_AHBMATPRIO_USB_FS_ENET_SHIFT)) & SYSCON_AHBMATPRIO_USB_FS_ENET_MASK)
/*! @} */

/*! @name CPU0NSTCKCAL - Non-Secure CPU0 System Tick Calibration */
/*! @{ */

#define SYSCON_CPU0NSTCKCAL_TENMS_MASK           (0xFFFFFFU)
#define SYSCON_CPU0NSTCKCAL_TENMS_SHIFT          (0U)
/*! TENMS - Reload value for 10 ms (100 Hz) timing, subject to system clock skew errors. If the
 *    value reads as zero, the calibration value is not known.
 */
#define SYSCON_CPU0NSTCKCAL_TENMS(x)             (((uint32_t)(((uint32_t)(x)) << SYSCON_CPU0NSTCKCAL_TENMS_SHIFT)) & SYSCON_CPU0NSTCKCAL_TENMS_MASK)

#define SYSCON_CPU0NSTCKCAL_SKEW_MASK            (0x1000000U)
#define SYSCON_CPU0NSTCKCAL_SKEW_SHIFT           (24U)
/*! SKEW - Indicates whether the TENMS value is exact.
 *  0b0..TENMS value is exact
 *  0b1..TENMS value is not exact or not given
 */
#define SYSCON_CPU0NSTCKCAL_SKEW(x)              (((uint32_t)(((uint32_t)(x)) << SYSCON_CPU0NSTCKCAL_SKEW_SHIFT)) & SYSCON_CPU0NSTCKCAL_SKEW_MASK)

#define SYSCON_CPU0NSTCKCAL_NOREF_MASK           (0x2000000U)
#define SYSCON_CPU0NSTCKCAL_NOREF_SHIFT          (25U)
/*! NOREF - Indicates whether the device provides a reference clock to the processor.
 *  0b0..Reference clock is provided
 *  0b1..No reference clock is provided
 */
#define SYSCON_CPU0NSTCKCAL_NOREF(x)             (((uint32_t)(((uint32_t)(x)) << SYSCON_CPU0NSTCKCAL_NOREF_SHIFT)) & SYSCON_CPU0NSTCKCAL_NOREF_MASK)
/*! @} */

/*! @name NMISRC - NMI Source Select */
/*! @{ */

#define SYSCON_NMISRC_IRQCPU0_MASK               (0xFFU)
#define SYSCON_NMISRC_IRQCPU0_SHIFT              (0U)
/*! IRQCPU0 - The IRQ number of the interrupt that acts as the Non-Maskable Interrupt (NMI) for CPU0, if enabled by NMIENCPU0. */
#define SYSCON_NMISRC_IRQCPU0(x)                 (((uint32_t)(((uint32_t)(x)) << SYSCON_NMISRC_IRQCPU0_SHIFT)) & SYSCON_NMISRC_IRQCPU0_MASK)

#define SYSCON_NMISRC_NMIENCPU0_MASK             (0x80000000U)
#define SYSCON_NMISRC_NMIENCPU0_SHIFT            (31U)
/*! NMIENCPU0 - Enables the Non-Maskable Interrupt (NMI) source selected by IRQCPU0.
 *  0b0..Disable.
 *  0b1..Enable.
 */
#define SYSCON_NMISRC_NMIENCPU0(x)               (((uint32_t)(((uint32_t)(x)) << SYSCON_NMISRC_NMIENCPU0_SHIFT)) & SYSCON_NMISRC_NMIENCPU0_MASK)
/*! @} */

/*! @name SLOWCLKDIV - SLOW_CLK Clock Divider */
/*! @{ */

#define SYSCON_SLOWCLKDIV_RESET_MASK             (0x20000000U)
#define SYSCON_SLOWCLKDIV_RESET_SHIFT            (29U)
/*! RESET - Resets the divider counter
 *  0b0..Divider is not reset
 *  0b1..Divider is reset
 */
#define SYSCON_SLOWCLKDIV_RESET(x)               (((uint32_t)(((uint32_t)(x)) << SYSCON_SLOWCLKDIV_RESET_SHIFT)) & SYSCON_SLOWCLKDIV_RESET_MASK)

#define SYSCON_SLOWCLKDIV_HALT_MASK              (0x40000000U)
#define SYSCON_SLOWCLKDIV_HALT_SHIFT             (30U)
/*! HALT - Halts the divider counter
 *  0b0..Divider clock is running
 *  0b1..Divider clock is stopped
 */
#define SYSCON_SLOWCLKDIV_HALT(x)                (((uint32_t)(((uint32_t)(x)) << SYSCON_SLOWCLKDIV_HALT_SHIFT)) & SYSCON_SLOWCLKDIV_HALT_MASK)

#define SYSCON_SLOWCLKDIV_UNSTAB_MASK            (0x80000000U)
#define SYSCON_SLOWCLKDIV_UNSTAB_SHIFT           (31U)
/*! UNSTAB - Divider status flag
 *  0b0..Divider clock is stable
 *  0b1..Clock frequency is not stable
 */
#define SYSCON_SLOWCLKDIV_UNSTAB(x)              (((uint32_t)(((uint32_t)(x)) << SYSCON_SLOWCLKDIV_UNSTAB_SHIFT)) & SYSCON_SLOWCLKDIV_UNSTAB_MASK)
/*! @} */

/*! @name AHBCLKDIV - System Clock Divider */
/*! @{ */

#define SYSCON_AHBCLKDIV_DIV_MASK                (0xFFU)
#define SYSCON_AHBCLKDIV_DIV_SHIFT               (0U)
/*! DIV - Clock divider value */
#define SYSCON_AHBCLKDIV_DIV(x)                  (((uint32_t)(((uint32_t)(x)) << SYSCON_AHBCLKDIV_DIV_SHIFT)) & SYSCON_AHBCLKDIV_DIV_MASK)

#define SYSCON_AHBCLKDIV_UNSTAB_MASK             (0x80000000U)
#define SYSCON_AHBCLKDIV_UNSTAB_SHIFT            (31U)
/*! UNSTAB - Divider status flag
 *  0b0..Divider clock is stable
 *  0b1..Clock frequency is not stable
 */
#define SYSCON_AHBCLKDIV_UNSTAB(x)               (((uint32_t)(((uint32_t)(x)) << SYSCON_AHBCLKDIV_UNSTAB_SHIFT)) & SYSCON_AHBCLKDIV_UNSTAB_MASK)
/*! @} */

/*! @name CLKUNLOCK - Clock Configuration Unlock */
/*! @{ */

#define SYSCON_CLKUNLOCK_UNLOCK_MASK             (0x1U)
#define SYSCON_CLKUNLOCK_UNLOCK_SHIFT            (0U)
/*! UNLOCK - Controls clock configuration registers access (for example, MRCC_xxx_CLKDIV, MRCC_xxx_CLKSEL, MRCC_GLB_xxx)
 *  0b0..Updates are allowed to all clock configuration registers
 *  0b1..Freezes all clock configuration registers update.
 */
#define SYSCON_CLKUNLOCK_UNLOCK(x)               (((uint32_t)(((uint32_t)(x)) << SYSCON_CLKUNLOCK_UNLOCK_SHIFT)) & SYSCON_CLKUNLOCK_UNLOCK_MASK)
/*! @} */

/*! @name NVM_CTRL - NVM Control */
/*! @{ */

#define SYSCON_NVM_CTRL_DIS_FLASH_SPEC_MASK      (0x1U)
#define SYSCON_NVM_CTRL_DIS_FLASH_SPEC_SHIFT     (0U)
/*! DIS_FLASH_SPEC - Flash speculation control
 *  0b0..Enables flash speculation
 *  0b1..Disables flash speculation
 */
#define SYSCON_NVM_CTRL_DIS_FLASH_SPEC(x)        (((uint32_t)(((uint32_t)(x)) << SYSCON_NVM_CTRL_DIS_FLASH_SPEC_SHIFT)) & SYSCON_NVM_CTRL_DIS_FLASH_SPEC_MASK)

#define SYSCON_NVM_CTRL_DIS_DATA_SPEC_MASK       (0x2U)
#define SYSCON_NVM_CTRL_DIS_DATA_SPEC_SHIFT      (1U)
/*! DIS_DATA_SPEC - Flash data speculation control
 *  0b0..Enables data speculation
 *  0b1..Disables data speculation
 */
#define SYSCON_NVM_CTRL_DIS_DATA_SPEC(x)         (((uint32_t)(((uint32_t)(x)) << SYSCON_NVM_CTRL_DIS_DATA_SPEC_SHIFT)) & SYSCON_NVM_CTRL_DIS_DATA_SPEC_MASK)

#define SYSCON_NVM_CTRL_FLASH_STALL_EN_MASK      (0x400U)
#define SYSCON_NVM_CTRL_FLASH_STALL_EN_SHIFT     (10U)
/*! FLASH_STALL_EN - FLASH stall on busy control
 *  0b0..No stall on FLASH busy
 *  0b1..Stall on FLASH busy
 */
#define SYSCON_NVM_CTRL_FLASH_STALL_EN(x)        (((uint32_t)(((uint32_t)(x)) << SYSCON_NVM_CTRL_FLASH_STALL_EN_SHIFT)) & SYSCON_NVM_CTRL_FLASH_STALL_EN_MASK)

#define SYSCON_NVM_CTRL_DIS_MBECC_ERR_INST_MASK  (0x10000U)
#define SYSCON_NVM_CTRL_DIS_MBECC_ERR_INST_SHIFT (16U)
/*! DIS_MBECC_ERR_INST
 *  0b0..Enables bus error on multi-bit ECC error for instruction
 *  0b1..Disables bus error on multi-bit ECC error for instruction
 */
#define SYSCON_NVM_CTRL_DIS_MBECC_ERR_INST(x)    (((uint32_t)(((uint32_t)(x)) << SYSCON_NVM_CTRL_DIS_MBECC_ERR_INST_SHIFT)) & SYSCON_NVM_CTRL_DIS_MBECC_ERR_INST_MASK)

#define SYSCON_NVM_CTRL_DIS_MBECC_ERR_DATA_MASK  (0x20000U)
#define SYSCON_NVM_CTRL_DIS_MBECC_ERR_DATA_SHIFT (17U)
/*! DIS_MBECC_ERR_DATA
 *  0b0..Enables bus error on multi-bit ECC error for data
 *  0b1..Disables bus error on multi-bit ECC error for data
 */
#define SYSCON_NVM_CTRL_DIS_MBECC_ERR_DATA(x)    (((uint32_t)(((uint32_t)(x)) << SYSCON_NVM_CTRL_DIS_MBECC_ERR_DATA_SHIFT)) & SYSCON_NVM_CTRL_DIS_MBECC_ERR_DATA_MASK)
/*! @} */

/*! @name CPUSTAT - CPU Status */
/*! @{ */

#define SYSCON_CPUSTAT_CPU0SLEEPING_MASK         (0x1U)
#define SYSCON_CPUSTAT_CPU0SLEEPING_SHIFT        (0U)
/*! CPU0SLEEPING - CPU0 sleeping state
 *  0b0..CPU is not sleeping
 *  0b1..CPU is sleeping
 */
#define SYSCON_CPUSTAT_CPU0SLEEPING(x)           (((uint32_t)(((uint32_t)(x)) << SYSCON_CPUSTAT_CPU0SLEEPING_SHIFT)) & SYSCON_CPUSTAT_CPU0SLEEPING_MASK)

#define SYSCON_CPUSTAT_CPU0LOCKUP_MASK           (0x4U)
#define SYSCON_CPUSTAT_CPU0LOCKUP_SHIFT          (2U)
/*! CPU0LOCKUP - CPU0 lockup state
 *  0b0..CPU is not in lockup
 *  0b1..CPU is in lockup
 */
#define SYSCON_CPUSTAT_CPU0LOCKUP(x)             (((uint32_t)(((uint32_t)(x)) << SYSCON_CPUSTAT_CPU0LOCKUP_SHIFT)) & SYSCON_CPUSTAT_CPU0LOCKUP_MASK)
/*! @} */

/*! @name LPCAC_CTRL - LPCAC Control */
/*! @{ */

#define SYSCON_LPCAC_CTRL_DIS_LPCAC_MASK         (0x1U)
#define SYSCON_LPCAC_CTRL_DIS_LPCAC_SHIFT        (0U)
/*! DIS_LPCAC - Disables/enables the cache function.
 *  0b0..Enabled
 *  0b1..Disabled
 */
#define SYSCON_LPCAC_CTRL_DIS_LPCAC(x)           (((uint32_t)(((uint32_t)(x)) << SYSCON_LPCAC_CTRL_DIS_LPCAC_SHIFT)) & SYSCON_LPCAC_CTRL_DIS_LPCAC_MASK)

#define SYSCON_LPCAC_CTRL_CLR_LPCAC_MASK         (0x2U)
#define SYSCON_LPCAC_CTRL_CLR_LPCAC_SHIFT        (1U)
/*! CLR_LPCAC - Clears the cache function.
 *  0b0..Unclears the cache
 *  0b1..Clears the cache
 */
#define SYSCON_LPCAC_CTRL_CLR_LPCAC(x)           (((uint32_t)(((uint32_t)(x)) << SYSCON_LPCAC_CTRL_CLR_LPCAC_SHIFT)) & SYSCON_LPCAC_CTRL_CLR_LPCAC_MASK)

#define SYSCON_LPCAC_CTRL_FRC_NO_ALLOC_MASK      (0x4U)
#define SYSCON_LPCAC_CTRL_FRC_NO_ALLOC_SHIFT     (2U)
/*! FRC_NO_ALLOC - Forces no allocation.
 *  0b0..Forces allocation
 *  0b1..Forces no allocation
 */
#define SYSCON_LPCAC_CTRL_FRC_NO_ALLOC(x)        (((uint32_t)(((uint32_t)(x)) << SYSCON_LPCAC_CTRL_FRC_NO_ALLOC_SHIFT)) & SYSCON_LPCAC_CTRL_FRC_NO_ALLOC_MASK)

#define SYSCON_LPCAC_CTRL_DIS_LPCAC_WTBF_MASK    (0x10U)
#define SYSCON_LPCAC_CTRL_DIS_LPCAC_WTBF_SHIFT   (4U)
/*! DIS_LPCAC_WTBF - Disable LPCAC Write Through Buffer.
 *  0b0..Enables write through buffer
 *  0b1..Disables write through buffer
 */
#define SYSCON_LPCAC_CTRL_DIS_LPCAC_WTBF(x)      (((uint32_t)(((uint32_t)(x)) << SYSCON_LPCAC_CTRL_DIS_LPCAC_WTBF_SHIFT)) & SYSCON_LPCAC_CTRL_DIS_LPCAC_WTBF_MASK)

#define SYSCON_LPCAC_CTRL_LIM_LPCAC_WTBF_MASK    (0x20U)
#define SYSCON_LPCAC_CTRL_LIM_LPCAC_WTBF_SHIFT   (5U)
/*! LIM_LPCAC_WTBF - Limit LPCAC Write Through Buffer.
 *  0b0..Write buffer enabled when transaction is bufferable.
 *  0b1..Write buffer enabled when transaction is cacheable and bufferable
 */
#define SYSCON_LPCAC_CTRL_LIM_LPCAC_WTBF(x)      (((uint32_t)(((uint32_t)(x)) << SYSCON_LPCAC_CTRL_LIM_LPCAC_WTBF_SHIFT)) & SYSCON_LPCAC_CTRL_LIM_LPCAC_WTBF_MASK)

#define SYSCON_LPCAC_CTRL_LPCAC_XOM_MASK         (0x80U)
#define SYSCON_LPCAC_CTRL_LPCAC_XOM_SHIFT        (7U)
/*! LPCAC_XOM - LPCAC XOM(eXecute-Only-Memory) attribute control
 *  0b0..Disabled.
 *  0b1..Enabled.
 */
#define SYSCON_LPCAC_CTRL_LPCAC_XOM(x)           (((uint32_t)(((uint32_t)(x)) << SYSCON_LPCAC_CTRL_LPCAC_XOM_SHIFT)) & SYSCON_LPCAC_CTRL_LPCAC_XOM_MASK)

#define SYSCON_LPCAC_CTRL_LPCAC_MEM_REQ_MASK     (0x100U)
#define SYSCON_LPCAC_CTRL_LPCAC_MEM_REQ_SHIFT    (8U)
/*! LPCAC_MEM_REQ - Request LPCAC memories.
 *  0b0..Configure shared memories RAMX1 as general memories.
 *  0b1..Configure shared memories RAMX1 as LPCAC memories, write one lock.
 */
#define SYSCON_LPCAC_CTRL_LPCAC_MEM_REQ(x)       (((uint32_t)(((uint32_t)(x)) << SYSCON_LPCAC_CTRL_LPCAC_MEM_REQ_SHIFT)) & SYSCON_LPCAC_CTRL_LPCAC_MEM_REQ_MASK)
/*! @} */

/*! @name PWM0SUBCTL - PWM0 Submodule Control */
/*! @{ */

#define SYSCON_PWM0SUBCTL_CLK0_EN_MASK           (0x1U)
#define SYSCON_PWM0SUBCTL_CLK0_EN_SHIFT          (0U)
/*! CLK0_EN - Enables PWM0 SUB Clock0
 *  0b0..Disable
 *  0b1..Enable
 */
#define SYSCON_PWM0SUBCTL_CLK0_EN(x)             (((uint32_t)(((uint32_t)(x)) << SYSCON_PWM0SUBCTL_CLK0_EN_SHIFT)) & SYSCON_PWM0SUBCTL_CLK0_EN_MASK)

#define SYSCON_PWM0SUBCTL_CLK1_EN_MASK           (0x2U)
#define SYSCON_PWM0SUBCTL_CLK1_EN_SHIFT          (1U)
/*! CLK1_EN - Enables PWM0 SUB Clock1
 *  0b0..Disable
 *  0b1..Enable
 */
#define SYSCON_PWM0SUBCTL_CLK1_EN(x)             (((uint32_t)(((uint32_t)(x)) << SYSCON_PWM0SUBCTL_CLK1_EN_SHIFT)) & SYSCON_PWM0SUBCTL_CLK1_EN_MASK)

#define SYSCON_PWM0SUBCTL_CLK2_EN_MASK           (0x4U)
#define SYSCON_PWM0SUBCTL_CLK2_EN_SHIFT          (2U)
/*! CLK2_EN - Enables PWM0 SUB Clock2
 *  0b0..Disable
 *  0b1..Enable
 */
#define SYSCON_PWM0SUBCTL_CLK2_EN(x)             (((uint32_t)(((uint32_t)(x)) << SYSCON_PWM0SUBCTL_CLK2_EN_SHIFT)) & SYSCON_PWM0SUBCTL_CLK2_EN_MASK)

#define SYSCON_PWM0SUBCTL_CLK3_EN_MASK           (0x8U)
#define SYSCON_PWM0SUBCTL_CLK3_EN_SHIFT          (3U)
/*! CLK3_EN - Enables PWM0 SUB Clock3
 *  0b0..Disable
 *  0b1..Enable
 */
#define SYSCON_PWM0SUBCTL_CLK3_EN(x)             (((uint32_t)(((uint32_t)(x)) << SYSCON_PWM0SUBCTL_CLK3_EN_SHIFT)) & SYSCON_PWM0SUBCTL_CLK3_EN_MASK)
/*! @} */

/*! @name CTIMERGLOBALSTARTEN - CTIMER Global Start Enable */
/*! @{ */

#define SYSCON_CTIMERGLOBALSTARTEN_CTIMER0_CLK_EN_MASK (0x1U)
#define SYSCON_CTIMERGLOBALSTARTEN_CTIMER0_CLK_EN_SHIFT (0U)
/*! CTIMER0_CLK_EN - Enables the CTIMER0 function clock
 *  0b0..Disable
 *  0b1..Enable
 */
#define SYSCON_CTIMERGLOBALSTARTEN_CTIMER0_CLK_EN(x) (((uint32_t)(((uint32_t)(x)) << SYSCON_CTIMERGLOBALSTARTEN_CTIMER0_CLK_EN_SHIFT)) & SYSCON_CTIMERGLOBALSTARTEN_CTIMER0_CLK_EN_MASK)

#define SYSCON_CTIMERGLOBALSTARTEN_CTIMER1_CLK_EN_MASK (0x2U)
#define SYSCON_CTIMERGLOBALSTARTEN_CTIMER1_CLK_EN_SHIFT (1U)
/*! CTIMER1_CLK_EN - Enables the CTIMER1 function clock
 *  0b0..Disable
 *  0b1..Enable
 */
#define SYSCON_CTIMERGLOBALSTARTEN_CTIMER1_CLK_EN(x) (((uint32_t)(((uint32_t)(x)) << SYSCON_CTIMERGLOBALSTARTEN_CTIMER1_CLK_EN_SHIFT)) & SYSCON_CTIMERGLOBALSTARTEN_CTIMER1_CLK_EN_MASK)

#define SYSCON_CTIMERGLOBALSTARTEN_CTIMER2_CLK_EN_MASK (0x4U)
#define SYSCON_CTIMERGLOBALSTARTEN_CTIMER2_CLK_EN_SHIFT (2U)
/*! CTIMER2_CLK_EN - Enables the CTIMER2 function clock
 *  0b0..Disable
 *  0b1..Enable
 */
#define SYSCON_CTIMERGLOBALSTARTEN_CTIMER2_CLK_EN(x) (((uint32_t)(((uint32_t)(x)) << SYSCON_CTIMERGLOBALSTARTEN_CTIMER2_CLK_EN_SHIFT)) & SYSCON_CTIMERGLOBALSTARTEN_CTIMER2_CLK_EN_MASK)
/*! @} */

/*! @name RAM_CTRL - RAM Control */
/*! @{ */

#define SYSCON_RAM_CTRL_RAMA_ECC_ENABLE_MASK     (0x1U)
#define SYSCON_RAM_CTRL_RAMA_ECC_ENABLE_SHIFT    (0U)
/*! RAMA_ECC_ENABLE - RAMA ECC enable
 *  0b0..ECC is disabled
 *  0b1..ECC is enabled
 */
#define SYSCON_RAM_CTRL_RAMA_ECC_ENABLE(x)       (((uint32_t)(((uint32_t)(x)) << SYSCON_RAM_CTRL_RAMA_ECC_ENABLE_SHIFT)) & SYSCON_RAM_CTRL_RAMA_ECC_ENABLE_MASK)

#define SYSCON_RAM_CTRL_RAMA_CG_OVERRIDE_MASK    (0x10000U)
#define SYSCON_RAM_CTRL_RAMA_CG_OVERRIDE_SHIFT   (16U)
/*! RAMA_CG_OVERRIDE - RAMA bank clock gating control, only avaiable when RAMA_ECC_ENABLE = 0.
 *  0b0..Memory bank clock is gated automatically if no access more than 16 clock cycles
 *  0b1..Auto clock gating feature is disabled
 */
#define SYSCON_RAM_CTRL_RAMA_CG_OVERRIDE(x)      (((uint32_t)(((uint32_t)(x)) << SYSCON_RAM_CTRL_RAMA_CG_OVERRIDE_SHIFT)) & SYSCON_RAM_CTRL_RAMA_CG_OVERRIDE_MASK)

#define SYSCON_RAM_CTRL_RAMX_CG_OVERRIDE_MASK    (0x20000U)
#define SYSCON_RAM_CTRL_RAMX_CG_OVERRIDE_SHIFT   (17U)
/*! RAMX_CG_OVERRIDE - RAMX bank clock gating control
 *  0b0..Memory bank clock is gated automatically if no access more than 16 clock cycles
 *  0b1..Auto clock gating feature is disabled
 */
#define SYSCON_RAM_CTRL_RAMX_CG_OVERRIDE(x)      (((uint32_t)(((uint32_t)(x)) << SYSCON_RAM_CTRL_RAMX_CG_OVERRIDE_SHIFT)) & SYSCON_RAM_CTRL_RAMX_CG_OVERRIDE_MASK)
/*! @} */

/*! @name GRAY_CODE_LSB - Gray to Binary Converter Gray Code [31:0] */
/*! @{ */

#define SYSCON_GRAY_CODE_LSB_code_gray_31_0_MASK (0xFFFFFFFFU)
#define SYSCON_GRAY_CODE_LSB_code_gray_31_0_SHIFT (0U)
/*! code_gray_31_0 - Gray code [31:0] */
#define SYSCON_GRAY_CODE_LSB_code_gray_31_0(x)   (((uint32_t)(((uint32_t)(x)) << SYSCON_GRAY_CODE_LSB_code_gray_31_0_SHIFT)) & SYSCON_GRAY_CODE_LSB_code_gray_31_0_MASK)
/*! @} */

/*! @name GRAY_CODE_MSB - Gray to Binary Converter Gray Code [41:32] */
/*! @{ */

#define SYSCON_GRAY_CODE_MSB_code_gray_41_32_MASK (0x3FFU)
#define SYSCON_GRAY_CODE_MSB_code_gray_41_32_SHIFT (0U)
/*! code_gray_41_32 - Gray code [41:32] */
#define SYSCON_GRAY_CODE_MSB_code_gray_41_32(x)  (((uint32_t)(((uint32_t)(x)) << SYSCON_GRAY_CODE_MSB_code_gray_41_32_SHIFT)) & SYSCON_GRAY_CODE_MSB_code_gray_41_32_MASK)
/*! @} */

/*! @name BINARY_CODE_LSB - Gray to Binary Converter Binary Code [31:0] */
/*! @{ */

#define SYSCON_BINARY_CODE_LSB_code_bin_31_0_MASK (0xFFFFFFFFU)
#define SYSCON_BINARY_CODE_LSB_code_bin_31_0_SHIFT (0U)
/*! code_bin_31_0 - Binary code [31:0] */
#define SYSCON_BINARY_CODE_LSB_code_bin_31_0(x)  (((uint32_t)(((uint32_t)(x)) << SYSCON_BINARY_CODE_LSB_code_bin_31_0_SHIFT)) & SYSCON_BINARY_CODE_LSB_code_bin_31_0_MASK)
/*! @} */

/*! @name BINARY_CODE_MSB - Gray to Binary Converter Binary Code [41:32] */
/*! @{ */

#define SYSCON_BINARY_CODE_MSB_code_bin_41_32_MASK (0x3FFU)
#define SYSCON_BINARY_CODE_MSB_code_bin_41_32_SHIFT (0U)
/*! code_bin_41_32 - Binary code [41:32] */
#define SYSCON_BINARY_CODE_MSB_code_bin_41_32(x) (((uint32_t)(((uint32_t)(x)) << SYSCON_BINARY_CODE_MSB_code_bin_41_32_SHIFT)) & SYSCON_BINARY_CODE_MSB_code_bin_41_32_MASK)
/*! @} */

/*! @name OVP_PAD_STATE - OVP_PAD_STATE */
/*! @{ */

#define SYSCON_OVP_PAD_STATE_OVP_PAD_STATE_MASK  (0xFFFFFFFFU)
#define SYSCON_OVP_PAD_STATE_OVP_PAD_STATE_SHIFT (0U)
/*! OVP_PAD_STATE - OVP_PAD_STATE */
#define SYSCON_OVP_PAD_STATE_OVP_PAD_STATE(x)    (((uint32_t)(((uint32_t)(x)) << SYSCON_OVP_PAD_STATE_OVP_PAD_STATE_SHIFT)) & SYSCON_OVP_PAD_STATE_OVP_PAD_STATE_MASK)
/*! @} */

/*! @name PROBE_STATE - PROBE_STATE */
/*! @{ */

#define SYSCON_PROBE_STATE_PROBE_STATE_MASK      (0xFFFFFFFFU)
#define SYSCON_PROBE_STATE_PROBE_STATE_SHIFT     (0U)
/*! PROBE_STATE - PROBE_STATE */
#define SYSCON_PROBE_STATE_PROBE_STATE(x)        (((uint32_t)(((uint32_t)(x)) << SYSCON_PROBE_STATE_PROBE_STATE_SHIFT)) & SYSCON_PROBE_STATE_PROBE_STATE_MASK)
/*! @} */

/*! @name FT_STATE_A - FT_STATE_A */
/*! @{ */

#define SYSCON_FT_STATE_A_FT_STATE_A_MASK        (0xFFFFFFFFU)
#define SYSCON_FT_STATE_A_FT_STATE_A_SHIFT       (0U)
/*! FT_STATE_A - FT_STATE_A */
#define SYSCON_FT_STATE_A_FT_STATE_A(x)          (((uint32_t)(((uint32_t)(x)) << SYSCON_FT_STATE_A_FT_STATE_A_SHIFT)) & SYSCON_FT_STATE_A_FT_STATE_A_MASK)
/*! @} */

/*! @name ROP_STATE - ROP State Register */
/*! @{ */

#define SYSCON_ROP_STATE_ROP_STATE_MASK          (0xFFFFFFFFU)
#define SYSCON_ROP_STATE_ROP_STATE_SHIFT         (0U)
/*! ROP_STATE - ROP state */
#define SYSCON_ROP_STATE_ROP_STATE(x)            (((uint32_t)(((uint32_t)(x)) << SYSCON_ROP_STATE_ROP_STATE_SHIFT)) & SYSCON_ROP_STATE_ROP_STATE_MASK)
/*! @} */

/*! @name SRAM_XEN - RAM XEN Control */
/*! @{ */

#define SYSCON_SRAM_XEN_RAMX0_XEN_MASK           (0x1U)
#define SYSCON_SRAM_XEN_RAMX0_XEN_SHIFT          (0U)
/*! RAMX0_XEN - RAMX0 Execute permission control.
 *  0b0..Execute permission is disabled, R/W are enabled.
 *  0b1..Execute permission is enabled, R/W/X are enabled.
 */
#define SYSCON_SRAM_XEN_RAMX0_XEN(x)             (((uint32_t)(((uint32_t)(x)) << SYSCON_SRAM_XEN_RAMX0_XEN_SHIFT)) & SYSCON_SRAM_XEN_RAMX0_XEN_MASK)

#define SYSCON_SRAM_XEN_RAMX1_XEN_MASK           (0x2U)
#define SYSCON_SRAM_XEN_RAMX1_XEN_SHIFT          (1U)
/*! RAMX1_XEN - RAMX1 Execute permission control.
 *  0b0..Execute permission is disabled, R/W are enabled.
 *  0b1..Execute permission is enabled, R/W/X are enabled.
 */
#define SYSCON_SRAM_XEN_RAMX1_XEN(x)             (((uint32_t)(((uint32_t)(x)) << SYSCON_SRAM_XEN_RAMX1_XEN_SHIFT)) & SYSCON_SRAM_XEN_RAMX1_XEN_MASK)

#define SYSCON_SRAM_XEN_RAMA0_XEN_MASK           (0x4U)
#define SYSCON_SRAM_XEN_RAMA0_XEN_SHIFT          (2U)
/*! RAMA0_XEN - RAMA0 Execute permission control.
 *  0b0..Execute permission is disabled, R/W are enabled.
 *  0b1..Execute permission is enabled, R/W/X are enabled.
 */
#define SYSCON_SRAM_XEN_RAMA0_XEN(x)             (((uint32_t)(((uint32_t)(x)) << SYSCON_SRAM_XEN_RAMA0_XEN_SHIFT)) & SYSCON_SRAM_XEN_RAMA0_XEN_MASK)

#define SYSCON_SRAM_XEN_RAMA1_XEN_MASK           (0x8U)
#define SYSCON_SRAM_XEN_RAMA1_XEN_SHIFT          (3U)
/*! RAMA1_XEN - RAMAx (excepts RAMA0) Execute permission control.
 *  0b0..Execute permission is disabled, R/W are enabled.
 *  0b1..Execute permission is enabled, R/W/X are enabled.
 */
#define SYSCON_SRAM_XEN_RAMA1_XEN(x)             (((uint32_t)(((uint32_t)(x)) << SYSCON_SRAM_XEN_RAMA1_XEN_SHIFT)) & SYSCON_SRAM_XEN_RAMA1_XEN_MASK)

#define SYSCON_SRAM_XEN_LOCK_MASK                (0x80000000U)
#define SYSCON_SRAM_XEN_LOCK_SHIFT               (31U)
/*! LOCK - This 1-bit field provides a mechanism to limit writes to the this register (and
 *    SRAM_XEN_DP) to protect its contents. Once set, this bit remains asserted until the next reset.
 *  0b0..This register is not locked and can be altered.
 *  0b1..This register is locked and cannot be altered.
 */
#define SYSCON_SRAM_XEN_LOCK(x)                  (((uint32_t)(((uint32_t)(x)) << SYSCON_SRAM_XEN_LOCK_SHIFT)) & SYSCON_SRAM_XEN_LOCK_MASK)
/*! @} */

/*! @name SRAM_XEN_DP - RAM XEN Control (Duplicate) */
/*! @{ */

#define SYSCON_SRAM_XEN_DP_RAMX0_XEN_MASK        (0x1U)
#define SYSCON_SRAM_XEN_DP_RAMX0_XEN_SHIFT       (0U)
/*! RAMX0_XEN - Refer to SRAM_XEN for more details. */
#define SYSCON_SRAM_XEN_DP_RAMX0_XEN(x)          (((uint32_t)(((uint32_t)(x)) << SYSCON_SRAM_XEN_DP_RAMX0_XEN_SHIFT)) & SYSCON_SRAM_XEN_DP_RAMX0_XEN_MASK)

#define SYSCON_SRAM_XEN_DP_RAMX1_XEN_MASK        (0x2U)
#define SYSCON_SRAM_XEN_DP_RAMX1_XEN_SHIFT       (1U)
/*! RAMX1_XEN - Refer to SRAM_XEN for more details. */
#define SYSCON_SRAM_XEN_DP_RAMX1_XEN(x)          (((uint32_t)(((uint32_t)(x)) << SYSCON_SRAM_XEN_DP_RAMX1_XEN_SHIFT)) & SYSCON_SRAM_XEN_DP_RAMX1_XEN_MASK)

#define SYSCON_SRAM_XEN_DP_RAMA0_XEN_MASK        (0x4U)
#define SYSCON_SRAM_XEN_DP_RAMA0_XEN_SHIFT       (2U)
/*! RAMA0_XEN - Refer to SRAM_XEN for more details. */
#define SYSCON_SRAM_XEN_DP_RAMA0_XEN(x)          (((uint32_t)(((uint32_t)(x)) << SYSCON_SRAM_XEN_DP_RAMA0_XEN_SHIFT)) & SYSCON_SRAM_XEN_DP_RAMA0_XEN_MASK)

#define SYSCON_SRAM_XEN_DP_RAMA1_XEN_MASK        (0x8U)
#define SYSCON_SRAM_XEN_DP_RAMA1_XEN_SHIFT       (3U)
/*! RAMA1_XEN - Refer to SRAM_XEN for more details. */
#define SYSCON_SRAM_XEN_DP_RAMA1_XEN(x)          (((uint32_t)(((uint32_t)(x)) << SYSCON_SRAM_XEN_DP_RAMA1_XEN_SHIFT)) & SYSCON_SRAM_XEN_DP_RAMA1_XEN_MASK)
/*! @} */

/*! @name ELS_OTP_LC_STATE - Life Cycle State Register */
/*! @{ */

#define SYSCON_ELS_OTP_LC_STATE_OTP_LC_STATE_MASK (0xFFU)
#define SYSCON_ELS_OTP_LC_STATE_OTP_LC_STATE_SHIFT (0U)
/*! OTP_LC_STATE - OTP life cycle state */
#define SYSCON_ELS_OTP_LC_STATE_OTP_LC_STATE(x)  (((uint32_t)(((uint32_t)(x)) << SYSCON_ELS_OTP_LC_STATE_OTP_LC_STATE_SHIFT)) & SYSCON_ELS_OTP_LC_STATE_OTP_LC_STATE_MASK)
/*! @} */

/*! @name ELS_OTP_LC_STATE_DP - Life Cycle State Register (Duplicate) */
/*! @{ */

#define SYSCON_ELS_OTP_LC_STATE_DP_OTP_LC_STATE_DP_MASK (0xFFU)
#define SYSCON_ELS_OTP_LC_STATE_DP_OTP_LC_STATE_DP_SHIFT (0U)
/*! OTP_LC_STATE_DP - OTP life cycle state */
#define SYSCON_ELS_OTP_LC_STATE_DP_OTP_LC_STATE_DP(x) (((uint32_t)(((uint32_t)(x)) << SYSCON_ELS_OTP_LC_STATE_DP_OTP_LC_STATE_DP_SHIFT)) & SYSCON_ELS_OTP_LC_STATE_DP_OTP_LC_STATE_DP_MASK)
/*! @} */

/*! @name DEBUG_LOCK_EN - Control Write Access to Security */
/*! @{ */

#define SYSCON_DEBUG_LOCK_EN_LOCK_ALL_MASK       (0xFU)
#define SYSCON_DEBUG_LOCK_EN_LOCK_ALL_SHIFT      (0U)
/*! LOCK_ALL - Controls write access to the security registers
 *  0b0000..Any other value than b1010: disables write access to all registers
 *  0b1010..Enables write access to all registers
 */
#define SYSCON_DEBUG_LOCK_EN_LOCK_ALL(x)         (((uint32_t)(((uint32_t)(x)) << SYSCON_DEBUG_LOCK_EN_LOCK_ALL_SHIFT)) & SYSCON_DEBUG_LOCK_EN_LOCK_ALL_MASK)
/*! @} */

/*! @name DEBUG_FEATURES - Cortex Debug Features Control */
/*! @{ */

#define SYSCON_DEBUG_FEATURES_CPU0_DBGEN_MASK    (0x3U)
#define SYSCON_DEBUG_FEATURES_CPU0_DBGEN_SHIFT   (0U)
/*! CPU0_DBGEN - CPU0 invasive debug control
 *  0b01..Disables debug
 *  0b10..Enables debug
 */
#define SYSCON_DEBUG_FEATURES_CPU0_DBGEN(x)      (((uint32_t)(((uint32_t)(x)) << SYSCON_DEBUG_FEATURES_CPU0_DBGEN_SHIFT)) & SYSCON_DEBUG_FEATURES_CPU0_DBGEN_MASK)

#define SYSCON_DEBUG_FEATURES_CPU0_NIDEN_MASK    (0xCU)
#define SYSCON_DEBUG_FEATURES_CPU0_NIDEN_SHIFT   (2U)
/*! CPU0_NIDEN - CPU0 non-invasive debug control
 *  0b01..Disables debug
 *  0b10..Enables debug
 */
#define SYSCON_DEBUG_FEATURES_CPU0_NIDEN(x)      (((uint32_t)(((uint32_t)(x)) << SYSCON_DEBUG_FEATURES_CPU0_NIDEN_SHIFT)) & SYSCON_DEBUG_FEATURES_CPU0_NIDEN_MASK)
/*! @} */

/*! @name DEBUG_FEATURES_DP - Cortex Debug Features Control (Duplicate) */
/*! @{ */

#define SYSCON_DEBUG_FEATURES_DP_CPU0_DBGEN_MASK (0x3U)
#define SYSCON_DEBUG_FEATURES_DP_CPU0_DBGEN_SHIFT (0U)
/*! CPU0_DBGEN - CPU0 invasive debug control
 *  0b01..Disables debug
 *  0b10..Enables debug
 */
#define SYSCON_DEBUG_FEATURES_DP_CPU0_DBGEN(x)   (((uint32_t)(((uint32_t)(x)) << SYSCON_DEBUG_FEATURES_DP_CPU0_DBGEN_SHIFT)) & SYSCON_DEBUG_FEATURES_DP_CPU0_DBGEN_MASK)

#define SYSCON_DEBUG_FEATURES_DP_CPU0_NIDEN_MASK (0xCU)
#define SYSCON_DEBUG_FEATURES_DP_CPU0_NIDEN_SHIFT (2U)
/*! CPU0_NIDEN - CPU0 non-invasive debug control
 *  0b01..Disables debug
 *  0b10..Enables debug
 */
#define SYSCON_DEBUG_FEATURES_DP_CPU0_NIDEN(x)   (((uint32_t)(((uint32_t)(x)) << SYSCON_DEBUG_FEATURES_DP_CPU0_NIDEN_SHIFT)) & SYSCON_DEBUG_FEATURES_DP_CPU0_NIDEN_MASK)
/*! @} */

/*! @name SWD_ACCESS_CPU0 - CPU0 Software Debug Access */
/*! @{ */

#define SYSCON_SWD_ACCESS_CPU0_SEC_CODE_MASK     (0xFFFFFFFFU)
#define SYSCON_SWD_ACCESS_CPU0_SEC_CODE_SHIFT    (0U)
/*! SEC_CODE - CPU0 SWD-AP: 0x12345678
 *  0b00000000000000000000000000000000..CPU0 DAP is not allowed. Reading back register is read as 0x5.
 *  0b00010010001101000101011001111000..Value to write to enable CPU0 SWD access. Reading back register is read as 0xA.
 */
#define SYSCON_SWD_ACCESS_CPU0_SEC_CODE(x)       (((uint32_t)(((uint32_t)(x)) << SYSCON_SWD_ACCESS_CPU0_SEC_CODE_SHIFT)) & SYSCON_SWD_ACCESS_CPU0_SEC_CODE_MASK)
/*! @} */

/*! @name DEBUG_AUTH_BEACON - Debug Authentication BEACON */
/*! @{ */

#define SYSCON_DEBUG_AUTH_BEACON_BEACON_MASK     (0xFFFFFFFFU)
#define SYSCON_DEBUG_AUTH_BEACON_BEACON_SHIFT    (0U)
/*! BEACON - Sets by the debug authentication code in ROM to pass the debug beacons (Credential
 *    Beacon and Authentication Beacon) to the application code.
 */
#define SYSCON_DEBUG_AUTH_BEACON_BEACON(x)       (((uint32_t)(((uint32_t)(x)) << SYSCON_DEBUG_AUTH_BEACON_BEACON_SHIFT)) & SYSCON_DEBUG_AUTH_BEACON_BEACON_MASK)
/*! @} */

/*! @name JTAG_ID - JTAG Chip ID */
/*! @{ */

#define SYSCON_JTAG_ID_JTAG_ID_MASK              (0xFFFFFFFFU)
#define SYSCON_JTAG_ID_JTAG_ID_SHIFT             (0U)
/*! JTAG_ID - Indicates the device ID */
#define SYSCON_JTAG_ID_JTAG_ID(x)                (((uint32_t)(((uint32_t)(x)) << SYSCON_JTAG_ID_JTAG_ID_SHIFT)) & SYSCON_JTAG_ID_JTAG_ID_MASK)
/*! @} */

/*! @name DEVICE_TYPE - Device Type */
/*! @{ */

#define SYSCON_DEVICE_TYPE_DEVICE_TYPE_MASK      (0xFFFFFFFFU)
#define SYSCON_DEVICE_TYPE_DEVICE_TYPE_SHIFT     (0U)
/*! DEVICE_TYPE - Indicates DEVICE TYPE. */
#define SYSCON_DEVICE_TYPE_DEVICE_TYPE(x)        (((uint32_t)(((uint32_t)(x)) << SYSCON_DEVICE_TYPE_DEVICE_TYPE_SHIFT)) & SYSCON_DEVICE_TYPE_DEVICE_TYPE_MASK)
/*! @} */

/*! @name DEVICE_ID0 - Device ID */
/*! @{ */

#define SYSCON_DEVICE_ID0_ROM_REV_MINOR_MASK     (0xF00000U)
#define SYSCON_DEVICE_ID0_ROM_REV_MINOR_SHIFT    (20U)
/*! ROM_REV_MINOR - ROM revision. */
#define SYSCON_DEVICE_ID0_ROM_REV_MINOR(x)       (((uint32_t)(((uint32_t)(x)) << SYSCON_DEVICE_ID0_ROM_REV_MINOR_SHIFT)) & SYSCON_DEVICE_ID0_ROM_REV_MINOR_MASK)
/*! @} */

/*! @name DIEID - Chip Revision ID and Number */
/*! @{ */

#define SYSCON_DIEID_MINOR_REVISION_MASK         (0xFU)
#define SYSCON_DIEID_MINOR_REVISION_SHIFT        (0U)
/*! MINOR_REVISION - Chip minor revision */
#define SYSCON_DIEID_MINOR_REVISION(x)           (((uint32_t)(((uint32_t)(x)) << SYSCON_DIEID_MINOR_REVISION_SHIFT)) & SYSCON_DIEID_MINOR_REVISION_MASK)

#define SYSCON_DIEID_MAJOR_REVISION_MASK         (0xF0U)
#define SYSCON_DIEID_MAJOR_REVISION_SHIFT        (4U)
/*! MAJOR_REVISION - Chip major revision */
#define SYSCON_DIEID_MAJOR_REVISION(x)           (((uint32_t)(((uint32_t)(x)) << SYSCON_DIEID_MAJOR_REVISION_SHIFT)) & SYSCON_DIEID_MAJOR_REVISION_MASK)

#define SYSCON_DIEID_MCO_NUM_IN_DIE_ID_MASK      (0xFFFFF00U)
#define SYSCON_DIEID_MCO_NUM_IN_DIE_ID_SHIFT     (8U)
/*! MCO_NUM_IN_DIE_ID - Chip number */
#define SYSCON_DIEID_MCO_NUM_IN_DIE_ID(x)        (((uint32_t)(((uint32_t)(x)) << SYSCON_DIEID_MCO_NUM_IN_DIE_ID_SHIFT)) & SYSCON_DIEID_MCO_NUM_IN_DIE_ID_MASK)
/*! @} */


/*!
 * @}
 */ /* end of group SYSCON_Register_Masks */


/*!
 * @}
 */ /* end of group SYSCON_Peripheral_Access_Layer */


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


#endif  /* PERI_SYSCON_H_ */

