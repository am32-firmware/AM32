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
**         CMSIS Peripheral Access Layer for FMUTEST
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
 * @file PERI_FMUTEST.h
 * @version 2.0
 * @date 2024-10-29
 * @brief CMSIS Peripheral Access Layer for FMUTEST
 *
 * CMSIS Peripheral Access Layer for FMUTEST
 */

#if !defined(PERI_FMUTEST_H_)
#define PERI_FMUTEST_H_                          /**< Symbol preventing repeated inclusion */

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
   -- FMUTEST Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FMUTEST_Peripheral_Access_Layer FMUTEST Peripheral Access Layer
 * @{
 */

/** FMUTEST - Register Layout Typedef */
typedef struct {
  __IO uint32_t FSTAT;                             /**< Flash Status Register, offset: 0x0 */
  __IO uint32_t FCNFG;                             /**< Flash Configuration Register, offset: 0x4 */
  __IO uint32_t FCTRL;                             /**< Flash Control Register, offset: 0x8 */
  __I  uint32_t FTEST;                             /**< Flash Test Register, offset: 0xC */
  __IO uint32_t FCCOB0;                            /**< Flash Command Control 0 Register, offset: 0x10 */
  __IO uint32_t FCCOB1;                            /**< Flash Command Control 1 Register, offset: 0x14 */
  __IO uint32_t FCCOB2;                            /**< Flash Command Control 2 Register, offset: 0x18 */
  __IO uint32_t FCCOB3;                            /**< Flash Command Control 3 Register, offset: 0x1C */
  __IO uint32_t FCCOB4;                            /**< Flash Command Control 4 Register, offset: 0x20 */
  __IO uint32_t FCCOB5;                            /**< Flash Command Control 5 Register, offset: 0x24 */
  __IO uint32_t FCCOB6;                            /**< Flash Command Control 6 Register, offset: 0x28 */
  __IO uint32_t FCCOB7;                            /**< Flash Command Control 7 Register, offset: 0x2C */
       uint8_t RESERVED_0[208];
  __IO uint32_t RESET_STATUS;                      /**< FMU Initialization Tracking Register, offset: 0x100 */
  __IO uint32_t MCTL;                              /**< FMU Control Register, offset: 0x104 */
  __I  uint32_t BSEL_GEN;                          /**< FMU Block Select Generation Register, offset: 0x108 */
  __IO uint32_t PWR_OPT;                           /**< Power Mode Options Register, offset: 0x10C */
  __I  uint32_t CMD_CHECK;                         /**< FMU Command Check Register, offset: 0x110 */
       uint8_t RESERVED_1[12];
  __IO uint32_t BSEL;                              /**< FMU Block Select Register, offset: 0x120 */
  __IO uint32_t MSIZE;                             /**< FMU Memory Size Register, offset: 0x124 */
  __IO uint32_t FLASH_RD_ADD;                      /**< Flash Read Address Register, offset: 0x128 */
       uint8_t RESERVED_2[4];
  __IO uint32_t FLASH_STOP_ADD;                    /**< Flash Stop Address Register, offset: 0x130 */
  __IO uint32_t FLASH_RD_CTRL;                     /**< Flash Read Control Register, offset: 0x134 */
  __IO uint32_t MM_ADDR;                           /**< Memory Map Address Register, offset: 0x138 */
       uint8_t RESERVED_3[4];
  __IO uint32_t MM_WDATA;                          /**< Memory Map Write Data Register, offset: 0x140 */
  __IO uint32_t MM_CTL;                            /**< Memory Map Control Register, offset: 0x144 */
  __IO uint32_t UINT_CTL;                          /**< User Interface Control Register, offset: 0x148 */
  __IO uint32_t RD_DATA0;                          /**< Read Data 0 Register, offset: 0x14C */
  __IO uint32_t RD_DATA1;                          /**< Read Data 1 Register, offset: 0x150 */
  __IO uint32_t RD_DATA2;                          /**< Read Data 2 Register, offset: 0x154 */
  __IO uint32_t RD_DATA3;                          /**< Read Data 3 Register, offset: 0x158 */
  __IO uint32_t PARITY;                            /**< Parity Register, offset: 0x15C */
  __IO uint32_t RD_PATH_CTRL_STATUS;               /**< Read Path Control and Status Register, offset: 0x160 */
  __IO uint32_t SMW_DIN0;                          /**< SMW DIN 0 Register, offset: 0x164 */
  __IO uint32_t SMW_DIN1;                          /**< SMW DIN 1 Register, offset: 0x168 */
  __IO uint32_t SMW_DIN2;                          /**< SMW DIN 2 Register, offset: 0x16C */
  __IO uint32_t SMW_DIN3;                          /**< SMW DIN 3 Register, offset: 0x170 */
  __IO uint32_t SMW_ADDR;                          /**< SMW Address Register, offset: 0x174 */
  __IO uint32_t SMW_CMD_WAIT;                      /**< SMW Command and Wait Register, offset: 0x178 */
  __I  uint32_t SMW_STATUS;                        /**< SMW Status Register, offset: 0x17C */
  __IO uint32_t SOCTRIM0_0;                        /**< SoC Trim Phrase 0 Word 0 Register, offset: 0x180 */
  __IO uint32_t SOCTRIM0_1;                        /**< SoC Trim Phrase 0 Word 1 Register, offset: 0x184 */
  __IO uint32_t SOCTRIM0_2;                        /**< SoC Trim Phrase 0 Word 2 Register, offset: 0x188 */
  __IO uint32_t SOCTRIM0_3;                        /**< SoC Trim Phrase 0 Word 3 Register, offset: 0x18C */
  __IO uint32_t SOCTRIM1_0;                        /**< SoC Trim Phrase 1 Word 0 Register, offset: 0x190 */
  __IO uint32_t SOCTRIM1_1;                        /**< SoC Trim Phrase 1 Word 1 Register, offset: 0x194 */
  __IO uint32_t SOCTRIM1_2;                        /**< SoC Trim Phrase 1 Word 2 Register, offset: 0x198 */
  __IO uint32_t SOCTRIM1_3;                        /**< SoC Trim Phrase 1 Word 3 Register, offset: 0x19C */
  __IO uint32_t SOCTRIM2_0;                        /**< SoC Trim Phrase 2 Word 0 Register, offset: 0x1A0 */
  __IO uint32_t SOCTRIM2_1;                        /**< SoC Trim Phrase 2 Word 1 Register, offset: 0x1A4 */
  __IO uint32_t SOCTRIM2_2;                        /**< SoC Trim Phrase 2 Word 2 Register, offset: 0x1A8 */
  __IO uint32_t SOCTRIM2_3;                        /**< SoC Trim Phrase 2 Word 3 Register, offset: 0x1AC */
  __IO uint32_t SOCTRIM3_0;                        /**< SoC Trim Phrase 3 Word 0 Register, offset: 0x1B0 */
  __IO uint32_t SOCTRIM3_1;                        /**< SoC Trim Phrase 3 Word 1 Register, offset: 0x1B4 */
  __IO uint32_t SOCTRIM3_2;                        /**< SoC Trim Phrase 3 Word 2 Register, offset: 0x1B8 */
  __IO uint32_t SOCTRIM3_3;                        /**< SoC Trim Phrase 3 Word 3 Register, offset: 0x1BC */
  __IO uint32_t SOCTRIM4_0;                        /**< SoC Trim Phrase 4 Word 0 Register, offset: 0x1C0 */
  __IO uint32_t SOCTRIM4_1;                        /**< SoC Trim Phrase 4 Word 1 Register, offset: 0x1C4 */
  __IO uint32_t SOCTRIM4_2;                        /**< SoC Trim Phrase 4 Word 2 Register, offset: 0x1C8 */
  __IO uint32_t SOCTRIM4_3;                        /**< SoC Trim Phrase 4 Word 3 Register, offset: 0x1CC */
  __IO uint32_t SOCTRIM5_0;                        /**< SoC Trim Phrase 5 Word 0 Register, offset: 0x1D0 */
  __IO uint32_t SOCTRIM5_1;                        /**< SoC Trim Phrase 5 Word 1 Register, offset: 0x1D4 */
  __IO uint32_t SOCTRIM5_2;                        /**< SoC Trim Phrase 5 Word 2 Register, offset: 0x1D8 */
  __IO uint32_t SOCTRIM5_3;                        /**< SoC Trim Phrase 5 Word 3 Register, offset: 0x1DC */
  __IO uint32_t SOCTRIM6_0;                        /**< SoC Trim Phrase 6 Word 0 Register, offset: 0x1E0 */
  __IO uint32_t SOCTRIM6_1;                        /**< SoC Trim Phrase 6 Word 1 Register, offset: 0x1E4 */
  __IO uint32_t SOCTRIM6_2;                        /**< SoC Trim Phrase 6 Word 2 Register, offset: 0x1E8 */
  __IO uint32_t SOCTRIM6_3;                        /**< SoC Trim Phrase 6 Word 3 Register, offset: 0x1EC */
  __IO uint32_t SOCTRIM7_0;                        /**< SoC Trim Phrase 7 Word 0 Register, offset: 0x1F0 */
  __IO uint32_t SOCTRIM7_1;                        /**< SoC Trim Phrase 7 Word 1 Register, offset: 0x1F4 */
  __IO uint32_t SOCTRIM7_2;                        /**< SoC Trim Phrase 7 Word 2 Register, offset: 0x1F8 */
  __IO uint32_t SOCTRIM7_3;                        /**< SoC Trim Phrase 7 Word 3 Register, offset: 0x1FC */
       uint8_t RESERVED_4[4];
  __IO uint32_t R_IP_CONFIG;                       /**< BIST Configuration Register, offset: 0x204 */
  __IO uint32_t R_TESTCODE;                        /**< BIST Test Code Register, offset: 0x208 */
  __IO uint32_t R_DFT_CTRL;                        /**< BIST DFT Control Register, offset: 0x20C */
  __IO uint32_t R_ADR_CTRL;                        /**< BIST Address Control Register, offset: 0x210 */
  __IO uint32_t R_DATA_CTRL0;                      /**< BIST Data Control 0 Register, offset: 0x214 */
  __IO uint32_t R_PIN_CTRL;                        /**< BIST Pin Control Register, offset: 0x218 */
  __IO uint32_t R_CNT_LOOP_CTRL;                   /**< BIST Loop Count Control Register, offset: 0x21C */
  __IO uint32_t R_TIMER_CTRL;                      /**< BIST Timer Control Register, offset: 0x220 */
  __IO uint32_t R_TEST_CTRL;                       /**< BIST Test Control Register, offset: 0x224 */
  __IO uint32_t R_ABORT_LOOP;                      /**< BIST Abort Loop Register, offset: 0x228 */
  __I  uint32_t R_ADR_QUERY;                       /**< BIST Address Query Register, offset: 0x22C */
  __I  uint32_t R_DOUT_QUERY0;                     /**< BIST DOUT Query 0 Register, offset: 0x230 */
       uint8_t RESERVED_5[8];
  __I  uint32_t R_SMW_QUERY;                       /**< BIST SMW Query Register, offset: 0x23C */
  __IO uint32_t R_SMW_SETTING0;                    /**< BIST SMW Setting 0 Register, offset: 0x240 */
  __IO uint32_t R_SMW_SETTING1;                    /**< BIST SMW Setting 1 Register, offset: 0x244 */
  __IO uint32_t R_SMP_WHV0;                        /**< BIST SMP WHV Setting 0 Register, offset: 0x248 */
  __IO uint32_t R_SMP_WHV1;                        /**< BIST SMP WHV Setting 1 Register, offset: 0x24C */
  __IO uint32_t R_SME_WHV0;                        /**< BIST SME WHV Setting 0 Register, offset: 0x250 */
  __IO uint32_t R_SME_WHV1;                        /**< BIST SME WHV Setting 1 Register, offset: 0x254 */
  __IO uint32_t R_SMW_SETTING2;                    /**< BIST SMW Setting 2 Register, offset: 0x258 */
  __I  uint32_t R_D_MISR0;                         /**< BIST DIN MISR 0 Register, offset: 0x25C */
  __I  uint32_t R_A_MISR0;                         /**< BIST Address MISR 0 Register, offset: 0x260 */
  __I  uint32_t R_C_MISR0;                         /**< BIST Control MISR 0 Register, offset: 0x264 */
  __IO uint32_t R_SMW_SETTING3;                    /**< BIST SMW Setting 3 Register, offset: 0x268 */
  __IO uint32_t R_DATA_CTRL1;                      /**< BIST Data Control 1 Register, offset: 0x26C */
  __IO uint32_t R_DATA_CTRL2;                      /**< BIST Data Control 2 Register, offset: 0x270 */
  __IO uint32_t R_DATA_CTRL3;                      /**< BIST Data Control 3 Register, offset: 0x274 */
       uint8_t RESERVED_6[8];
  __I  uint32_t R_REPAIR0_0;                       /**< BIST Repair 0 for Block 0 Register, offset: 0x280 */
  __I  uint32_t R_REPAIR0_1;                       /**< BIST Repair 1 Block 0 Register, offset: 0x284 */
  __I  uint32_t R_REPAIR1_0;                       /**< BIST Repair 0 Block 1 Register, offset: 0x288 */
  __I  uint32_t R_REPAIR1_1;                       /**< BIST Repair 1 Block 1 Register, offset: 0x28C */
       uint8_t RESERVED_7[132];
  __IO uint32_t R_DATA_CTRL0_EX;                   /**< BIST Data Control 0 Extension Register, offset: 0x314 */
       uint8_t RESERVED_8[8];
  __IO uint32_t R_TIMER_CTRL_EX;                   /**< BIST Timer Control Extension Register, offset: 0x320 */
       uint8_t RESERVED_9[12];
  __I  uint32_t R_DOUT_QUERY1;                     /**< BIST DOUT Query 1 Register, offset: 0x330 */
       uint8_t RESERVED_10[40];
  __I  uint32_t R_D_MISR1;                         /**< BIST DIN MISR 1 Register, offset: 0x35C */
  __I  uint32_t R_A_MISR1;                         /**< BIST Address MISR 1 Register, offset: 0x360 */
  __I  uint32_t R_C_MISR1;                         /**< BIST Control MISR 1 Register, offset: 0x364 */
       uint8_t RESERVED_11[4];
  __IO uint32_t R_DATA_CTRL1_EX;                   /**< BIST Data Control 1 Extension Register, offset: 0x36C */
  __IO uint32_t R_DATA_CTRL2_EX;                   /**< BIST Data Control 2 Extension Register, offset: 0x370 */
  __IO uint32_t R_DATA_CTRL3_EX;                   /**< BIST Data Control 3 Extension Register, offset: 0x374 */
       uint8_t RESERVED_12[136];
  __IO uint32_t SMW_TIMER_OPTION;                  /**< SMW Timer Option Register, offset: 0x400 */
  __IO uint32_t SMW_SETTING_OPTION0;               /**< SMW Setting Option 0 Register, offset: 0x404 */
  __IO uint32_t SMW_SETTING_OPTION2;               /**< SMW Setting Option 2 Register, offset: 0x408 */
  __IO uint32_t SMW_SETTING_OPTION3;               /**< SMW Setting Option 3 Register, offset: 0x40C */
  __IO uint32_t SMW_SMP_WHV_OPTION0;               /**< SMW SMP WHV Option 0 Register, offset: 0x410 */
  __IO uint32_t SMW_SME_WHV_OPTION0;               /**< SMW SME WHV Option 0 Register, offset: 0x414 */
  __IO uint32_t SMW_SETTING_OPTION1;               /**< SMW Setting Option 1 Register, offset: 0x418 */
  __IO uint32_t SMW_SMP_WHV_OPTION1;               /**< SMW SMP WHV Option 1 Register, offset: 0x41C */
  __IO uint32_t SMW_SME_WHV_OPTION1;               /**< SMW SME WHV Option 1 Register, offset: 0x420 */
       uint8_t RESERVED_13[220];
  __IO uint32_t REPAIR0_0;                         /**< FMU Repair 0 Block 0 Register, offset: 0x500 */
  __IO uint32_t REPAIR0_1;                         /**< FMU Repair 1 Block 0 Register, offset: 0x504 */
  __IO uint32_t REPAIR1_0;                         /**< FMU Repair 0 Block 1 Register, offset: 0x508 */
  __IO uint32_t REPAIR1_1;                         /**< FMU Repair 1 Block 1 Register, offset: 0x50C */
       uint8_t RESERVED_14[240];
  __IO uint32_t SMW_HB_SIGNALS;                    /**< SMW HB Signals Register, offset: 0x600 */
  __IO uint32_t BIST_DUMP_CTRL;                    /**< BIST Datadump Control Register, offset: 0x604 */
       uint8_t RESERVED_15[4];
  __IO uint32_t ATX_PIN_CTRL;                      /**< ATX Pin Control Register, offset: 0x60C */
  __IO uint32_t FAILCNT;                           /**< Fail Count Register, offset: 0x610 */
  __IO uint32_t PGM_PULSE_CNT0;                    /**< Block 0 Program Pulse Count Register, offset: 0x614 */
  __IO uint32_t PGM_PULSE_CNT1;                    /**< Block 1 Program Pulse Count Register, offset: 0x618 */
  __IO uint32_t ERS_PULSE_CNT;                     /**< Erase Pulse Count Register, offset: 0x61C */
  __IO uint32_t MAX_PULSE_CNT;                     /**< Maximum Pulse Count Register, offset: 0x620 */
  __IO uint32_t PORT_CTRL;                         /**< Port Control Register, offset: 0x624 */
} FMUTEST_Type;

/* ----------------------------------------------------------------------------
   -- FMUTEST Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FMUTEST_Register_Masks FMUTEST Register Masks
 * @{
 */

/*! @name FSTAT - Flash Status Register */
/*! @{ */

#define FMUTEST_FSTAT_FAIL_MASK                  (0x1U)
#define FMUTEST_FSTAT_FAIL_SHIFT                 (0U)
/*! FAIL - Command Fail Flag
 *  0b0..Error not detected
 *  0b1..Error detected
 */
#define FMUTEST_FSTAT_FAIL(x)                    (((uint32_t)(((uint32_t)(x)) << FMUTEST_FSTAT_FAIL_SHIFT)) & FMUTEST_FSTAT_FAIL_MASK)

#define FMUTEST_FSTAT_CMDABT_MASK                (0x4U)
#define FMUTEST_FSTAT_CMDABT_SHIFT               (2U)
/*! CMDABT - Command Abort Flag
 *  0b0..No command abort detected
 *  0b1..Command abort detected
 */
#define FMUTEST_FSTAT_CMDABT(x)                  (((uint32_t)(((uint32_t)(x)) << FMUTEST_FSTAT_CMDABT_SHIFT)) & FMUTEST_FSTAT_CMDABT_MASK)

#define FMUTEST_FSTAT_PVIOL_MASK                 (0x10U)
#define FMUTEST_FSTAT_PVIOL_SHIFT                (4U)
/*! PVIOL - Command Protection Violation Flag
 *  0b0..No protection violation detected
 *  0b1..Protection violation detected
 */
#define FMUTEST_FSTAT_PVIOL(x)                   (((uint32_t)(((uint32_t)(x)) << FMUTEST_FSTAT_PVIOL_SHIFT)) & FMUTEST_FSTAT_PVIOL_MASK)

#define FMUTEST_FSTAT_ACCERR_MASK                (0x20U)
#define FMUTEST_FSTAT_ACCERR_SHIFT               (5U)
/*! ACCERR - Command Access Error Flag
 *  0b0..No access error detected
 *  0b1..Access error detected
 */
#define FMUTEST_FSTAT_ACCERR(x)                  (((uint32_t)(((uint32_t)(x)) << FMUTEST_FSTAT_ACCERR_SHIFT)) & FMUTEST_FSTAT_ACCERR_MASK)

#define FMUTEST_FSTAT_CWSABT_MASK                (0x40U)
#define FMUTEST_FSTAT_CWSABT_SHIFT               (6U)
/*! CWSABT - Command Write Sequence Abort Flag
 *  0b0..Command write sequence not aborted
 *  0b1..Command write sequence aborted
 */
#define FMUTEST_FSTAT_CWSABT(x)                  (((uint32_t)(((uint32_t)(x)) << FMUTEST_FSTAT_CWSABT_SHIFT)) & FMUTEST_FSTAT_CWSABT_MASK)

#define FMUTEST_FSTAT_CCIF_MASK                  (0x80U)
#define FMUTEST_FSTAT_CCIF_SHIFT                 (7U)
/*! CCIF - Command Complete Interrupt Flag
 *  0b0..Flash command or initialization in progress
 *  0b1..Flash command or initialization has completed
 */
#define FMUTEST_FSTAT_CCIF(x)                    (((uint32_t)(((uint32_t)(x)) << FMUTEST_FSTAT_CCIF_SHIFT)) & FMUTEST_FSTAT_CCIF_MASK)

#define FMUTEST_FSTAT_CMDPRT_MASK                (0x300U)
#define FMUTEST_FSTAT_CMDPRT_SHIFT               (8U)
/*! CMDPRT - Command Protection Level
 *  0b00..Secure, normal access
 *  0b01..Secure, privileged access
 *  0b10..Nonsecure, normal access
 *  0b11..Nonsecure, privileged access
 */
#define FMUTEST_FSTAT_CMDPRT(x)                  (((uint32_t)(((uint32_t)(x)) << FMUTEST_FSTAT_CMDPRT_SHIFT)) & FMUTEST_FSTAT_CMDPRT_MASK)

#define FMUTEST_FSTAT_CMDP_MASK                  (0x800U)
#define FMUTEST_FSTAT_CMDP_SHIFT                 (11U)
/*! CMDP - Command Protection Status Flag
 *  0b0..Command protection level and domain ID are stale
 *  0b1..Command protection level (CMDPRT) and domain ID (CMDDID) are set
 */
#define FMUTEST_FSTAT_CMDP(x)                    (((uint32_t)(((uint32_t)(x)) << FMUTEST_FSTAT_CMDP_SHIFT)) & FMUTEST_FSTAT_CMDP_MASK)

#define FMUTEST_FSTAT_CMDDID_MASK                (0xF000U)
#define FMUTEST_FSTAT_CMDDID_SHIFT               (12U)
/*! CMDDID - Command Domain ID */
#define FMUTEST_FSTAT_CMDDID(x)                  (((uint32_t)(((uint32_t)(x)) << FMUTEST_FSTAT_CMDDID_SHIFT)) & FMUTEST_FSTAT_CMDDID_MASK)

#define FMUTEST_FSTAT_DFDIF_MASK                 (0x10000U)
#define FMUTEST_FSTAT_DFDIF_SHIFT                (16U)
/*! DFDIF - Double Bit Fault Detect Interrupt Flag
 *  0b0..Double bit fault not detected during a valid flash read access from the FMC
 *  0b1..Double bit fault detected (or FCTRL[FDFD] is set) during a valid flash read access from the FMC
 */
#define FMUTEST_FSTAT_DFDIF(x)                   (((uint32_t)(((uint32_t)(x)) << FMUTEST_FSTAT_DFDIF_SHIFT)) & FMUTEST_FSTAT_DFDIF_MASK)

#define FMUTEST_FSTAT_SALV_USED_MASK             (0x20000U)
#define FMUTEST_FSTAT_SALV_USED_SHIFT            (17U)
/*! SALV_USED - Salvage Used for Erase operation
 *  0b0..Salvage not used during the last operation
 *  0b1..Salvage used during the last erase operation
 */
#define FMUTEST_FSTAT_SALV_USED(x)               (((uint32_t)(((uint32_t)(x)) << FMUTEST_FSTAT_SALV_USED_SHIFT)) & FMUTEST_FSTAT_SALV_USED_MASK)

#define FMUTEST_FSTAT_PEWEN_MASK                 (0x3000000U)
#define FMUTEST_FSTAT_PEWEN_SHIFT                (24U)
/*! PEWEN - Program-Erase Write Enable Control
 *  0b00..Writes are not enabled
 *  0b01..Writes are enabled for one flash or IFR phrase (phrase programming, sector erase)
 *  0b10..Writes are enabled for one flash or IFR page (page programming)
 *  0b11..Reserved
 */
#define FMUTEST_FSTAT_PEWEN(x)                   (((uint32_t)(((uint32_t)(x)) << FMUTEST_FSTAT_PEWEN_SHIFT)) & FMUTEST_FSTAT_PEWEN_MASK)

#define FMUTEST_FSTAT_PERDY_MASK                 (0x80000000U)
#define FMUTEST_FSTAT_PERDY_SHIFT                (31U)
/*! PERDY - Program/Erase Ready Control/Status Flag
 *  0b0..Program or sector erase command operation is not stalled
 *  0b1..Program or sector erase command operation is stalled
 */
#define FMUTEST_FSTAT_PERDY(x)                   (((uint32_t)(((uint32_t)(x)) << FMUTEST_FSTAT_PERDY_SHIFT)) & FMUTEST_FSTAT_PERDY_MASK)
/*! @} */

/*! @name FCNFG - Flash Configuration Register */
/*! @{ */

#define FMUTEST_FCNFG_CCIE_MASK                  (0x80U)
#define FMUTEST_FCNFG_CCIE_SHIFT                 (7U)
/*! CCIE - Command Complete Interrupt Enable
 *  0b0..Command complete interrupt disabled
 *  0b1..Command complete interrupt enabled. An interrupt request is generated whenever the FSTAT[CCIF] flag is set.
 */
#define FMUTEST_FCNFG_CCIE(x)                    (((uint32_t)(((uint32_t)(x)) << FMUTEST_FCNFG_CCIE_SHIFT)) & FMUTEST_FCNFG_CCIE_MASK)

#define FMUTEST_FCNFG_ERSREQ_MASK                (0x100U)
#define FMUTEST_FCNFG_ERSREQ_SHIFT               (8U)
/*! ERSREQ - Mass Erase (Erase All) Request
 *  0b0..No request or request complete
 *  0b1..Request to run the Mass Erase operation
 */
#define FMUTEST_FCNFG_ERSREQ(x)                  (((uint32_t)(((uint32_t)(x)) << FMUTEST_FCNFG_ERSREQ_SHIFT)) & FMUTEST_FCNFG_ERSREQ_MASK)

#define FMUTEST_FCNFG_DFDIE_MASK                 (0x10000U)
#define FMUTEST_FCNFG_DFDIE_SHIFT                (16U)
/*! DFDIE - Double Bit Fault Detect Interrupt Enable
 *  0b0..Double bit fault detect interrupt disabled
 *  0b1..Double bit fault detect interrupt enabled; an interrupt request is generated whenever the FSTAT[DFDIF] flag is set
 */
#define FMUTEST_FCNFG_DFDIE(x)                   (((uint32_t)(((uint32_t)(x)) << FMUTEST_FCNFG_DFDIE_SHIFT)) & FMUTEST_FCNFG_DFDIE_MASK)

#define FMUTEST_FCNFG_ERSIEN0_MASK               (0xF000000U)
#define FMUTEST_FCNFG_ERSIEN0_SHIFT              (24U)
/*! ERSIEN0 - Erase IFR Sector Enable - Block 0
 *  0b0000..Block 0 IFR Sector X is protected from erase by ERSSCR command
 *  0b0001..Block 0 IFR Sector X is not protected from erase by ERSSCR command
 */
#define FMUTEST_FCNFG_ERSIEN0(x)                 (((uint32_t)(((uint32_t)(x)) << FMUTEST_FCNFG_ERSIEN0_SHIFT)) & FMUTEST_FCNFG_ERSIEN0_MASK)

#define FMUTEST_FCNFG_ERSIEN1_MASK               (0xF0000000U)
#define FMUTEST_FCNFG_ERSIEN1_SHIFT              (28U)
/*! ERSIEN1 - Erase IFR Sector Enable - Block 1 (for dual block configs)
 *  0b0000..Block 1 IFR Sector X is protected from erase by ERSSCR command
 *  0b0001..Block 1 IFR Sector X is not protected from erase by ERSSCR command
 */
#define FMUTEST_FCNFG_ERSIEN1(x)                 (((uint32_t)(((uint32_t)(x)) << FMUTEST_FCNFG_ERSIEN1_SHIFT)) & FMUTEST_FCNFG_ERSIEN1_MASK)
/*! @} */

/*! @name FCTRL - Flash Control Register */
/*! @{ */

#define FMUTEST_FCTRL_RWSC_MASK                  (0xFU)
#define FMUTEST_FCTRL_RWSC_SHIFT                 (0U)
/*! RWSC - Read Wait-State Control
 *  0b0000..no additional wait-states are added (single cycle access)
 *  0b0001..1 additional wait-state is added
 *  0b0010..2 additional wait-states are added
 *  0b0011..3 additional wait-states are added
 *  0b0100..4 additional wait-states are added
 *  0b0101..5 additional wait-states are added
 *  0b0110..6 additional wait-states are added
 *  0b0111..7 additional wait-states are added
 *  0b1000..8 additional wait-states are added
 *  0b1001..9 additional wait-states are added
 *  0b1010..10 additional wait-states are added
 *  0b1011..11 additional wait-states are added
 *  0b1100..12 additional wait-states are added
 *  0b1101..13 additional wait-states are added
 *  0b1110..14 additional wait-states are added
 *  0b1111..15 additional wait-states are added
 */
#define FMUTEST_FCTRL_RWSC(x)                    (((uint32_t)(((uint32_t)(x)) << FMUTEST_FCTRL_RWSC_SHIFT)) & FMUTEST_FCTRL_RWSC_MASK)

#define FMUTEST_FCTRL_LSACTIVE_MASK              (0x100U)
#define FMUTEST_FCTRL_LSACTIVE_SHIFT             (8U)
/*! LSACTIVE - Low Speed Active Mode
 *  0b0..Full speed active mode requested
 *  0b1..Low speed active mode requested
 */
#define FMUTEST_FCTRL_LSACTIVE(x)                (((uint32_t)(((uint32_t)(x)) << FMUTEST_FCTRL_LSACTIVE_SHIFT)) & FMUTEST_FCTRL_LSACTIVE_MASK)

#define FMUTEST_FCTRL_FDFD_MASK                  (0x10000U)
#define FMUTEST_FCTRL_FDFD_SHIFT                 (16U)
/*! FDFD - Force Double Bit Fault Detect
 *  0b0..FSTAT[DFDIF] sets only if a double bit fault is detected during a valid flash read access from the FMC
 *  0b1..FSTAT[DFDIF] sets during any valid flash read access from the FMC; an interrupt request is generated if the DFDIE bit is set
 */
#define FMUTEST_FCTRL_FDFD(x)                    (((uint32_t)(((uint32_t)(x)) << FMUTEST_FCTRL_FDFD_SHIFT)) & FMUTEST_FCTRL_FDFD_MASK)

#define FMUTEST_FCTRL_ABTREQ_MASK                (0x1000000U)
#define FMUTEST_FCTRL_ABTREQ_SHIFT               (24U)
/*! ABTREQ - Abort Request
 *  0b0..No request to abort a command write sequence
 *  0b1..Request to abort a command write sequence
 */
#define FMUTEST_FCTRL_ABTREQ(x)                  (((uint32_t)(((uint32_t)(x)) << FMUTEST_FCTRL_ABTREQ_SHIFT)) & FMUTEST_FCTRL_ABTREQ_MASK)
/*! @} */

/*! @name FTEST - Flash Test Register */
/*! @{ */

#define FMUTEST_FTEST_TMECTL_MASK                (0x1U)
#define FMUTEST_FTEST_TMECTL_SHIFT               (0U)
/*! TMECTL - Test Mode Entry Control
 *  0b0..FTEST register always reads 0 and writes to FTEST are ignored
 *  0b1..FTEST register is readable and can be written to enable writability of TME
 */
#define FMUTEST_FTEST_TMECTL(x)                  (((uint32_t)(((uint32_t)(x)) << FMUTEST_FTEST_TMECTL_SHIFT)) & FMUTEST_FTEST_TMECTL_MASK)

#define FMUTEST_FTEST_TMEWR_MASK                 (0x2U)
#define FMUTEST_FTEST_TMEWR_SHIFT                (1U)
/*! TMEWR - Test Mode Entry Writable
 *  0b0..TME bit is not writable
 *  0b1..TME bit is writable
 */
#define FMUTEST_FTEST_TMEWR(x)                   (((uint32_t)(((uint32_t)(x)) << FMUTEST_FTEST_TMEWR_SHIFT)) & FMUTEST_FTEST_TMEWR_MASK)

#define FMUTEST_FTEST_TME_MASK                   (0x4U)
#define FMUTEST_FTEST_TME_SHIFT                  (2U)
/*! TME - Test Mode Entry
 *  0b0..Test mode entry not requested
 *  0b1..Test mode entry requested
 */
#define FMUTEST_FTEST_TME(x)                     (((uint32_t)(((uint32_t)(x)) << FMUTEST_FTEST_TME_SHIFT)) & FMUTEST_FTEST_TME_MASK)

#define FMUTEST_FTEST_TMODE_MASK                 (0x8U)
#define FMUTEST_FTEST_TMODE_SHIFT                (3U)
/*! TMODE - Test Mode Status
 *  0b0..Test mode not active
 *  0b1..Test mode active
 */
#define FMUTEST_FTEST_TMODE(x)                   (((uint32_t)(((uint32_t)(x)) << FMUTEST_FTEST_TMODE_SHIFT)) & FMUTEST_FTEST_TMODE_MASK)

#define FMUTEST_FTEST_TMELOCK_MASK               (0x10U)
#define FMUTEST_FTEST_TMELOCK_SHIFT              (4U)
/*! TMELOCK - Test Mode Entry Lock
 *  0b0..FTEST register not locked from accepting writes
 *  0b1..FTEST register locked from accepting writes
 */
#define FMUTEST_FTEST_TMELOCK(x)                 (((uint32_t)(((uint32_t)(x)) << FMUTEST_FTEST_TMELOCK_SHIFT)) & FMUTEST_FTEST_TMELOCK_MASK)
/*! @} */

/*! @name FCCOB0 - Flash Command Control 0 Register */
/*! @{ */

#define FMUTEST_FCCOB0_CMDCODE_MASK              (0xFFU)
#define FMUTEST_FCCOB0_CMDCODE_SHIFT             (0U)
/*! CMDCODE - Command code */
#define FMUTEST_FCCOB0_CMDCODE(x)                (((uint32_t)(((uint32_t)(x)) << FMUTEST_FCCOB0_CMDCODE_SHIFT)) & FMUTEST_FCCOB0_CMDCODE_MASK)
/*! @} */

/*! @name FCCOB1 - Flash Command Control 1 Register */
/*! @{ */

#define FMUTEST_FCCOB1_CMDOPT_MASK               (0xFFU)
#define FMUTEST_FCCOB1_CMDOPT_SHIFT              (0U)
/*! CMDOPT - Command options */
#define FMUTEST_FCCOB1_CMDOPT(x)                 (((uint32_t)(((uint32_t)(x)) << FMUTEST_FCCOB1_CMDOPT_SHIFT)) & FMUTEST_FCCOB1_CMDOPT_MASK)
/*! @} */

/*! @name FCCOB2 - Flash Command Control 2 Register */
/*! @{ */

#define FMUTEST_FCCOB2_CMDADDR_MASK              (0xFFFFFFFFU)
#define FMUTEST_FCCOB2_CMDADDR_SHIFT             (0U)
/*! CMDADDR - Command starting address */
#define FMUTEST_FCCOB2_CMDADDR(x)                (((uint32_t)(((uint32_t)(x)) << FMUTEST_FCCOB2_CMDADDR_SHIFT)) & FMUTEST_FCCOB2_CMDADDR_MASK)
/*! @} */

/*! @name FCCOB3 - Flash Command Control 3 Register */
/*! @{ */

#define FMUTEST_FCCOB3_CMDADDRE_MASK             (0xFFFFFFFFU)
#define FMUTEST_FCCOB3_CMDADDRE_SHIFT            (0U)
/*! CMDADDRE - Command ending address */
#define FMUTEST_FCCOB3_CMDADDRE(x)               (((uint32_t)(((uint32_t)(x)) << FMUTEST_FCCOB3_CMDADDRE_SHIFT)) & FMUTEST_FCCOB3_CMDADDRE_MASK)
/*! @} */

/*! @name FCCOB4 - Flash Command Control 4 Register */
/*! @{ */

#define FMUTEST_FCCOB4_CMDDATA0_MASK             (0xFFFFFFFFU)
#define FMUTEST_FCCOB4_CMDDATA0_SHIFT            (0U)
/*! CMDDATA0 - Command data word 0 */
#define FMUTEST_FCCOB4_CMDDATA0(x)               (((uint32_t)(((uint32_t)(x)) << FMUTEST_FCCOB4_CMDDATA0_SHIFT)) & FMUTEST_FCCOB4_CMDDATA0_MASK)
/*! @} */

/*! @name FCCOB5 - Flash Command Control 5 Register */
/*! @{ */

#define FMUTEST_FCCOB5_CMDDATA1_MASK             (0xFFFFFFFFU)
#define FMUTEST_FCCOB5_CMDDATA1_SHIFT            (0U)
/*! CMDDATA1 - Command data word 1 */
#define FMUTEST_FCCOB5_CMDDATA1(x)               (((uint32_t)(((uint32_t)(x)) << FMUTEST_FCCOB5_CMDDATA1_SHIFT)) & FMUTEST_FCCOB5_CMDDATA1_MASK)
/*! @} */

/*! @name FCCOB6 - Flash Command Control 6 Register */
/*! @{ */

#define FMUTEST_FCCOB6_CMDDATA2_MASK             (0xFFFFFFFFU)
#define FMUTEST_FCCOB6_CMDDATA2_SHIFT            (0U)
/*! CMDDATA2 - Command data word 2 */
#define FMUTEST_FCCOB6_CMDDATA2(x)               (((uint32_t)(((uint32_t)(x)) << FMUTEST_FCCOB6_CMDDATA2_SHIFT)) & FMUTEST_FCCOB6_CMDDATA2_MASK)
/*! @} */

/*! @name FCCOB7 - Flash Command Control 7 Register */
/*! @{ */

#define FMUTEST_FCCOB7_CMDDATA3_MASK             (0xFFFFFFFFU)
#define FMUTEST_FCCOB7_CMDDATA3_SHIFT            (0U)
/*! CMDDATA3 - Command data word 3 */
#define FMUTEST_FCCOB7_CMDDATA3(x)               (((uint32_t)(((uint32_t)(x)) << FMUTEST_FCCOB7_CMDDATA3_SHIFT)) & FMUTEST_FCCOB7_CMDDATA3_MASK)
/*! @} */

/*! @name RESET_STATUS - FMU Initialization Tracking Register */
/*! @{ */

#define FMUTEST_RESET_STATUS_ARY_TRIM_DONE_MASK  (0x1U)
#define FMUTEST_RESET_STATUS_ARY_TRIM_DONE_SHIFT (0U)
/*! ARY_TRIM_DONE - Array Trim Complete
 *  0b0..Recall register load operation has not been completed
 *  0b1..Recall register load operation has completed
 */
#define FMUTEST_RESET_STATUS_ARY_TRIM_DONE(x)    (((uint32_t)(((uint32_t)(x)) << FMUTEST_RESET_STATUS_ARY_TRIM_DONE_SHIFT)) & FMUTEST_RESET_STATUS_ARY_TRIM_DONE_MASK)

#define FMUTEST_RESET_STATUS_FMU_PARM_EN_MASK    (0x2U)
#define FMUTEST_RESET_STATUS_FMU_PARM_EN_SHIFT   (1U)
/*! FMU_PARM_EN - Status of the C0DE_C0DEh check to enable loading of the FMU parameters
 *  0b0..C0DE_C0DEh check not attempted
 *  0b1..C0DE_C0DEh check completed
 */
#define FMUTEST_RESET_STATUS_FMU_PARM_EN(x)      (((uint32_t)(((uint32_t)(x)) << FMUTEST_RESET_STATUS_FMU_PARM_EN_SHIFT)) & FMUTEST_RESET_STATUS_FMU_PARM_EN_MASK)

#define FMUTEST_RESET_STATUS_FMU_PARM_DONE_MASK  (0x4U)
#define FMUTEST_RESET_STATUS_FMU_PARM_DONE_SHIFT (2U)
/*! FMU_PARM_DONE - FMU Register Load Complete
 *  0b0..FMU registers have not been loaded
 *  0b1..FMU registers have been loaded
 */
#define FMUTEST_RESET_STATUS_FMU_PARM_DONE(x)    (((uint32_t)(((uint32_t)(x)) << FMUTEST_RESET_STATUS_FMU_PARM_DONE_SHIFT)) & FMUTEST_RESET_STATUS_FMU_PARM_DONE_MASK)

#define FMUTEST_RESET_STATUS_SOC_TRIM_EN_MASK    (0x8U)
#define FMUTEST_RESET_STATUS_SOC_TRIM_EN_SHIFT   (3U)
/*! SOC_TRIM_EN - Status of the C0DE_C0DEh check to enable loading of the SoC trim settings
 *  0b0..C0DE_C0DEh check not attempted
 *  0b1..C0DE_C0DEh check completed
 */
#define FMUTEST_RESET_STATUS_SOC_TRIM_EN(x)      (((uint32_t)(((uint32_t)(x)) << FMUTEST_RESET_STATUS_SOC_TRIM_EN_SHIFT)) & FMUTEST_RESET_STATUS_SOC_TRIM_EN_MASK)

#define FMUTEST_RESET_STATUS_SOC_TRIM_ECC_MASK   (0x10U)
#define FMUTEST_RESET_STATUS_SOC_TRIM_ECC_SHIFT  (4U)
/*! SOC_TRIM_ECC - Status of the C0DE_C0DEh check for enabling ECC decoder during reads of SoC trim settings
 *  0b0..C0DE_C0DEh check failed
 *  0b1..C0DE_C0DEh check passed
 */
#define FMUTEST_RESET_STATUS_SOC_TRIM_ECC(x)     (((uint32_t)(((uint32_t)(x)) << FMUTEST_RESET_STATUS_SOC_TRIM_ECC_SHIFT)) & FMUTEST_RESET_STATUS_SOC_TRIM_ECC_MASK)

#define FMUTEST_RESET_STATUS_SOC_TRIM_DONE_MASK  (0x20U)
#define FMUTEST_RESET_STATUS_SOC_TRIM_DONE_SHIFT (5U)
/*! SOC_TRIM_DONE - SoC Trim Complete
 *  0b0..SoC Trim registers have not been updated
 *  0b1..All SoC Trim registers have been updated
 */
#define FMUTEST_RESET_STATUS_SOC_TRIM_DONE(x)    (((uint32_t)(((uint32_t)(x)) << FMUTEST_RESET_STATUS_SOC_TRIM_DONE_SHIFT)) & FMUTEST_RESET_STATUS_SOC_TRIM_DONE_MASK)

#define FMUTEST_RESET_STATUS_RPR_DONE_MASK       (0x40U)
#define FMUTEST_RESET_STATUS_RPR_DONE_SHIFT      (6U)
/*! RPR_DONE - Array Repair Complete
 *  0b0..Repair registers have not been loaded
 *  0b1..Repair registers have been loaded
 */
#define FMUTEST_RESET_STATUS_RPR_DONE(x)         (((uint32_t)(((uint32_t)(x)) << FMUTEST_RESET_STATUS_RPR_DONE_SHIFT)) & FMUTEST_RESET_STATUS_RPR_DONE_MASK)

#define FMUTEST_RESET_STATUS_INIT_DONE_MASK      (0x80U)
#define FMUTEST_RESET_STATUS_INIT_DONE_SHIFT     (7U)
/*! INIT_DONE - Initialization Done
 *  0b0..All initialization steps did not complete
 *  0b1..All initialization steps completed
 */
#define FMUTEST_RESET_STATUS_INIT_DONE(x)        (((uint32_t)(((uint32_t)(x)) << FMUTEST_RESET_STATUS_INIT_DONE_SHIFT)) & FMUTEST_RESET_STATUS_INIT_DONE_MASK)

#define FMUTEST_RESET_STATUS_RST_SF_ERR_MASK     (0x100U)
#define FMUTEST_RESET_STATUS_RST_SF_ERR_SHIFT    (8U)
/*! RST_SF_ERR - ECC Single Fault during Reset Recovery
 *  0b0..No single-bit faults detected during initialization
 *  0b1..At least one single ECC fault was detected during initialization
 */
#define FMUTEST_RESET_STATUS_RST_SF_ERR(x)       (((uint32_t)(((uint32_t)(x)) << FMUTEST_RESET_STATUS_RST_SF_ERR_SHIFT)) & FMUTEST_RESET_STATUS_RST_SF_ERR_MASK)

#define FMUTEST_RESET_STATUS_RST_DF_ERR_MASK     (0x200U)
#define FMUTEST_RESET_STATUS_RST_DF_ERR_SHIFT    (9U)
/*! RST_DF_ERR - ECC Double Fault during Reset Recovery
 *  0b0..No double-bit faults detected during initialization
 *  0b1..Double-bit ECC fault was detected during initialization
 */
#define FMUTEST_RESET_STATUS_RST_DF_ERR(x)       (((uint32_t)(((uint32_t)(x)) << FMUTEST_RESET_STATUS_RST_DF_ERR_SHIFT)) & FMUTEST_RESET_STATUS_RST_DF_ERR_MASK)

#define FMUTEST_RESET_STATUS_SOC_TRIM_DF_ERR_MASK (0x3FC00U)
#define FMUTEST_RESET_STATUS_SOC_TRIM_DF_ERR_SHIFT (10U)
/*! SOC_TRIM_DF_ERR - ECC Double Fault during load of SoC Trim phrases */
#define FMUTEST_RESET_STATUS_SOC_TRIM_DF_ERR(x)  (((uint32_t)(((uint32_t)(x)) << FMUTEST_RESET_STATUS_SOC_TRIM_DF_ERR_SHIFT)) & FMUTEST_RESET_STATUS_SOC_TRIM_DF_ERR_MASK)

#define FMUTEST_RESET_STATUS_RST_PATCH_LD_MASK   (0x40000U)
#define FMUTEST_RESET_STATUS_RST_PATCH_LD_SHIFT  (18U)
/*! RST_PATCH_LD - Reset Patch Required
 *  0b0..No patch required to be loaded during reset
 *  0b1..Patch loaded during reset
 */
#define FMUTEST_RESET_STATUS_RST_PATCH_LD(x)     (((uint32_t)(((uint32_t)(x)) << FMUTEST_RESET_STATUS_RST_PATCH_LD_SHIFT)) & FMUTEST_RESET_STATUS_RST_PATCH_LD_MASK)

#define FMUTEST_RESET_STATUS_RECALL_DATA_MISMATCH_MASK (0x80000U)
#define FMUTEST_RESET_STATUS_RECALL_DATA_MISMATCH_SHIFT (19U)
/*! RECALL_DATA_MISMATCH - Recall Data Mismatch
 *  0b0..Data read towards end of reset matched data read for Recall
 *  0b1..Data read towards end of reset did not match data read for recall
 */
#define FMUTEST_RESET_STATUS_RECALL_DATA_MISMATCH(x) (((uint32_t)(((uint32_t)(x)) << FMUTEST_RESET_STATUS_RECALL_DATA_MISMATCH_SHIFT)) & FMUTEST_RESET_STATUS_RECALL_DATA_MISMATCH_MASK)
/*! @} */

/*! @name MCTL - FMU Control Register */
/*! @{ */

#define FMUTEST_MCTL_COREHLD_MASK                (0x1U)
#define FMUTEST_MCTL_COREHLD_SHIFT               (0U)
/*! COREHLD - Core Hold
 *  0b0..CPU access is allowed
 *  0b1..CPU access must be blocked
 */
#define FMUTEST_MCTL_COREHLD(x)                  (((uint32_t)(((uint32_t)(x)) << FMUTEST_MCTL_COREHLD_SHIFT)) & FMUTEST_MCTL_COREHLD_MASK)

#define FMUTEST_MCTL_LSACT_EN_MASK               (0x4U)
#define FMUTEST_MCTL_LSACT_EN_SHIFT              (2U)
/*! LSACT_EN - LSACTIVE Feature Enable
 *  0b0..LSACTIVE feature disabled completely: FCTRL[LSACTIVE] is forced low and no longer writable, LVE cannot assert at the TSMC array interface.
 *  0b1..LSACTIVE feature fully enabled and controllable by SoC and internal UINT SM.
 */
#define FMUTEST_MCTL_LSACT_EN(x)                 (((uint32_t)(((uint32_t)(x)) << FMUTEST_MCTL_LSACT_EN_SHIFT)) & FMUTEST_MCTL_LSACT_EN_MASK)

#define FMUTEST_MCTL_LSACTWREN_MASK              (0x8U)
#define FMUTEST_MCTL_LSACTWREN_SHIFT             (3U)
/*! LSACTWREN - LSACTIVE Write Enable
 *  0b0..Unrestricted write access allowed
 *  0b1..Write access while CMP set must match CMDDID and CMDPRT
 */
#define FMUTEST_MCTL_LSACTWREN(x)                (((uint32_t)(((uint32_t)(x)) << FMUTEST_MCTL_LSACTWREN_SHIFT)) & FMUTEST_MCTL_LSACTWREN_MASK)

#define FMUTEST_MCTL_MASTER_REPAIR_EN_MASK       (0x10U)
#define FMUTEST_MCTL_MASTER_REPAIR_EN_SHIFT      (4U)
/*! MASTER_REPAIR_EN - Master Repair Enable
 *  0b0..Repair disabled
 *  0b1..Repair enable determined by bit 0 of each REPAIR register
 */
#define FMUTEST_MCTL_MASTER_REPAIR_EN(x)         (((uint32_t)(((uint32_t)(x)) << FMUTEST_MCTL_MASTER_REPAIR_EN_SHIFT)) & FMUTEST_MCTL_MASTER_REPAIR_EN_MASK)

#define FMUTEST_MCTL_RFCMDEN_MASK                (0x20U)
#define FMUTEST_MCTL_RFCMDEN_SHIFT               (5U)
/*! RFCMDEN - RF Active Command Enable Control
 *  0b0..Flash commands blocked (CCIF not writable)
 *  0b1..Flash commands allowed
 */
#define FMUTEST_MCTL_RFCMDEN(x)                  (((uint32_t)(((uint32_t)(x)) << FMUTEST_MCTL_RFCMDEN_SHIFT)) & FMUTEST_MCTL_RFCMDEN_MASK)

#define FMUTEST_MCTL_CWSABTEN_MASK               (0x40U)
#define FMUTEST_MCTL_CWSABTEN_SHIFT              (6U)
/*! CWSABTEN - Command Write Sequence Abort Enable
 *  0b0..CWS abort feature is disabled
 *  0b1..CWS abort feature is enabled
 */
#define FMUTEST_MCTL_CWSABTEN(x)                 (((uint32_t)(((uint32_t)(x)) << FMUTEST_MCTL_CWSABTEN_SHIFT)) & FMUTEST_MCTL_CWSABTEN_MASK)

#define FMUTEST_MCTL_MRGRDDIS_MASK               (0x80U)
#define FMUTEST_MCTL_MRGRDDIS_SHIFT              (7U)
/*! MRGRDDIS - Margin Read Disable
 *  0b0..Margin Read Settings are enabled
 *  0b1..Margin Read Settings are disabled
 */
#define FMUTEST_MCTL_MRGRDDIS(x)                 (((uint32_t)(((uint32_t)(x)) << FMUTEST_MCTL_MRGRDDIS_SHIFT)) & FMUTEST_MCTL_MRGRDDIS_MASK)

#define FMUTEST_MCTL_MRGRD0_MASK                 (0xF00U)
#define FMUTEST_MCTL_MRGRD0_SHIFT                (8U)
/*! MRGRD0 - Margin Read Setting for Program */
#define FMUTEST_MCTL_MRGRD0(x)                   (((uint32_t)(((uint32_t)(x)) << FMUTEST_MCTL_MRGRD0_SHIFT)) & FMUTEST_MCTL_MRGRD0_MASK)

#define FMUTEST_MCTL_MRGRD1_MASK                 (0xF000U)
#define FMUTEST_MCTL_MRGRD1_SHIFT                (12U)
/*! MRGRD1 - Margin Read Setting for Erase */
#define FMUTEST_MCTL_MRGRD1(x)                   (((uint32_t)(((uint32_t)(x)) << FMUTEST_MCTL_MRGRD1_SHIFT)) & FMUTEST_MCTL_MRGRD1_MASK)

#define FMUTEST_MCTL_ERSAACK_MASK                (0x10000U)
#define FMUTEST_MCTL_ERSAACK_SHIFT               (16U)
/*! ERSAACK - Mass Erase (Erase All) Acknowledge
 *  0b0..Mass Erase operation is not active (operation has completed or has not started)
 *  0b1..Mass Erase operation is active (controller acknowledges that the soc_ersall_req input is asserted and will continue with the operation)
 */
#define FMUTEST_MCTL_ERSAACK(x)                  (((uint32_t)(((uint32_t)(x)) << FMUTEST_MCTL_ERSAACK_SHIFT)) & FMUTEST_MCTL_ERSAACK_MASK)

#define FMUTEST_MCTL_SCAN_OBS_MASK               (0x80000U)
#define FMUTEST_MCTL_SCAN_OBS_SHIFT              (19U)
/*! SCAN_OBS - Scan Observability Control
 *  0b0..Normal functional behavior
 *  0b1..Enables observation of signals that may otherwise be ATPG untestable
 */
#define FMUTEST_MCTL_SCAN_OBS(x)                 (((uint32_t)(((uint32_t)(x)) << FMUTEST_MCTL_SCAN_OBS_SHIFT)) & FMUTEST_MCTL_SCAN_OBS_MASK)

#define FMUTEST_MCTL_BIST_CTL_MASK               (0x100000U)
#define FMUTEST_MCTL_BIST_CTL_SHIFT              (20U)
/*! BIST_CTL - BIST IP Control
 *  0b0..BIST IP disabled
 *  0b1..BIST IP enabled
 */
#define FMUTEST_MCTL_BIST_CTL(x)                 (((uint32_t)(((uint32_t)(x)) << FMUTEST_MCTL_BIST_CTL_SHIFT)) & FMUTEST_MCTL_BIST_CTL_MASK)

#define FMUTEST_MCTL_SMWR_CTL_MASK               (0x200000U)
#define FMUTEST_MCTL_SMWR_CTL_SHIFT              (21U)
/*! SMWR_CTL - SMWR IP Control
 *  0b0..SMWR IP disabled
 *  0b1..SMWR IP enabled
 */
#define FMUTEST_MCTL_SMWR_CTL(x)                 (((uint32_t)(((uint32_t)(x)) << FMUTEST_MCTL_SMWR_CTL_SHIFT)) & FMUTEST_MCTL_SMWR_CTL_MASK)

#define FMUTEST_MCTL_SALV_DIS_MASK               (0x1000000U)
#define FMUTEST_MCTL_SALV_DIS_SHIFT              (24U)
/*! SALV_DIS - Salvage Disable
 *  0b0..Salvage enabled (ECC used during erase verify)
 *  0b1..Salvage disabled (ECC not used during erase verify)
 */
#define FMUTEST_MCTL_SALV_DIS(x)                 (((uint32_t)(((uint32_t)(x)) << FMUTEST_MCTL_SALV_DIS_SHIFT)) & FMUTEST_MCTL_SALV_DIS_MASK)

#define FMUTEST_MCTL_SOC_ECC_CTL_MASK            (0x2000000U)
#define FMUTEST_MCTL_SOC_ECC_CTL_SHIFT           (25U)
/*! SOC_ECC_CTL - SOC ECC Control
 *  0b0..ECC is enabled for SOC read access
 *  0b1..ECC is disabled for SOC read access
 */
#define FMUTEST_MCTL_SOC_ECC_CTL(x)              (((uint32_t)(((uint32_t)(x)) << FMUTEST_MCTL_SOC_ECC_CTL_SHIFT)) & FMUTEST_MCTL_SOC_ECC_CTL_MASK)

#define FMUTEST_MCTL_FMU_ECC_CTL_MASK            (0x4000000U)
#define FMUTEST_MCTL_FMU_ECC_CTL_SHIFT           (26U)
/*! FMU_ECC_CTL - FMU ECC Control
 *  0b0..ECC is enabled for FMU program operations
 *  0b1..ECC is disabled for FMU program operations
 */
#define FMUTEST_MCTL_FMU_ECC_CTL(x)              (((uint32_t)(((uint32_t)(x)) << FMUTEST_MCTL_FMU_ECC_CTL_SHIFT)) & FMUTEST_MCTL_FMU_ECC_CTL_MASK)

#define FMUTEST_MCTL_BIST_PWR_DIS_MASK           (0x20000000U)
#define FMUTEST_MCTL_BIST_PWR_DIS_SHIFT          (29U)
/*! BIST_PWR_DIS - BIST Power Mode Disable
 *  0b0..BIST DFT logic has full control of SLM and LVE when BIST is enabled (including during commands)
 *  0b1..BIST DFT logic has no control of SLM and LVE; power mode RTL is in complete control of SLM and LVE values
 */
#define FMUTEST_MCTL_BIST_PWR_DIS(x)             (((uint32_t)(((uint32_t)(x)) << FMUTEST_MCTL_BIST_PWR_DIS_SHIFT)) & FMUTEST_MCTL_BIST_PWR_DIS_MASK)

#define FMUTEST_MCTL_OSC_H_MASK                  (0x80000000U)
#define FMUTEST_MCTL_OSC_H_SHIFT                 (31U)
/*! OSC_H - Oscillator control
 *  0b0..Use APB clock
 *  0b1..Use a known fixed-frequency clock, e.g. 12 MHz
 */
#define FMUTEST_MCTL_OSC_H(x)                    (((uint32_t)(((uint32_t)(x)) << FMUTEST_MCTL_OSC_H_SHIFT)) & FMUTEST_MCTL_OSC_H_MASK)
/*! @} */

/*! @name BSEL_GEN - FMU Block Select Generation Register */
/*! @{ */

#define FMUTEST_BSEL_GEN_SBSEL_GEN_MASK          (0x3U)
#define FMUTEST_BSEL_GEN_SBSEL_GEN_SHIFT         (0U)
/*! SBSEL_GEN - Generated SBSEL */
#define FMUTEST_BSEL_GEN_SBSEL_GEN(x)            (((uint32_t)(((uint32_t)(x)) << FMUTEST_BSEL_GEN_SBSEL_GEN_SHIFT)) & FMUTEST_BSEL_GEN_SBSEL_GEN_MASK)

#define FMUTEST_BSEL_GEN_MBSEL_GEN_MASK          (0x300U)
#define FMUTEST_BSEL_GEN_MBSEL_GEN_SHIFT         (8U)
/*! MBSEL_GEN - Generated MBSEL */
#define FMUTEST_BSEL_GEN_MBSEL_GEN(x)            (((uint32_t)(((uint32_t)(x)) << FMUTEST_BSEL_GEN_MBSEL_GEN_SHIFT)) & FMUTEST_BSEL_GEN_MBSEL_GEN_MASK)
/*! @} */

/*! @name PWR_OPT - Power Mode Options Register */
/*! @{ */

#define FMUTEST_PWR_OPT_PD_CDIV_MASK             (0xFFU)
#define FMUTEST_PWR_OPT_PD_CDIV_SHIFT            (0U)
/*! PD_CDIV - Power Down Clock Divider Setting */
#define FMUTEST_PWR_OPT_PD_CDIV(x)               (((uint32_t)(((uint32_t)(x)) << FMUTEST_PWR_OPT_PD_CDIV_SHIFT)) & FMUTEST_PWR_OPT_PD_CDIV_MASK)

#define FMUTEST_PWR_OPT_SLM_COUNT_MASK           (0x3FF0000U)
#define FMUTEST_PWR_OPT_SLM_COUNT_SHIFT          (16U)
/*! SLM_COUNT - Sleep Recovery Timer Count */
#define FMUTEST_PWR_OPT_SLM_COUNT(x)             (((uint32_t)(((uint32_t)(x)) << FMUTEST_PWR_OPT_SLM_COUNT_SHIFT)) & FMUTEST_PWR_OPT_SLM_COUNT_MASK)

#define FMUTEST_PWR_OPT_PD_TIMER_EN_MASK         (0x80000000U)
#define FMUTEST_PWR_OPT_PD_TIMER_EN_SHIFT        (31U)
/*! PD_TIMER_EN - Power Down BIST Timer Enable
 *  0b0..BIST timer is not triggered during Power Down recovery
 *  0b1..BIST timer is triggered during Power Down recovery (default behavior)
 */
#define FMUTEST_PWR_OPT_PD_TIMER_EN(x)           (((uint32_t)(((uint32_t)(x)) << FMUTEST_PWR_OPT_PD_TIMER_EN_SHIFT)) & FMUTEST_PWR_OPT_PD_TIMER_EN_MASK)
/*! @} */

/*! @name CMD_CHECK - FMU Command Check Register */
/*! @{ */

#define FMUTEST_CMD_CHECK_ALIGNFAIL_PHR_MASK     (0x1U)
#define FMUTEST_CMD_CHECK_ALIGNFAIL_PHR_SHIFT    (0U)
/*! ALIGNFAIL_PHR - Phrase Alignment Fail
 *  0b0..The address is phrase-aligned
 *  0b1..The address is not phrase-aligned
 */
#define FMUTEST_CMD_CHECK_ALIGNFAIL_PHR(x)       (((uint32_t)(((uint32_t)(x)) << FMUTEST_CMD_CHECK_ALIGNFAIL_PHR_SHIFT)) & FMUTEST_CMD_CHECK_ALIGNFAIL_PHR_MASK)

#define FMUTEST_CMD_CHECK_ALIGNFAIL_PG_MASK      (0x2U)
#define FMUTEST_CMD_CHECK_ALIGNFAIL_PG_SHIFT     (1U)
/*! ALIGNFAIL_PG - Page Alignment Fail
 *  0b0..The address is page-aligned
 *  0b1..The address is not page-aligned
 */
#define FMUTEST_CMD_CHECK_ALIGNFAIL_PG(x)        (((uint32_t)(((uint32_t)(x)) << FMUTEST_CMD_CHECK_ALIGNFAIL_PG_SHIFT)) & FMUTEST_CMD_CHECK_ALIGNFAIL_PG_MASK)

#define FMUTEST_CMD_CHECK_ALIGNFAIL_SCR_MASK     (0x4U)
#define FMUTEST_CMD_CHECK_ALIGNFAIL_SCR_SHIFT    (2U)
/*! ALIGNFAIL_SCR - Sector Alignment Fail
 *  0b0..The address is sector-aligned
 *  0b1..The address is not sector-aligned
 */
#define FMUTEST_CMD_CHECK_ALIGNFAIL_SCR(x)       (((uint32_t)(((uint32_t)(x)) << FMUTEST_CMD_CHECK_ALIGNFAIL_SCR_SHIFT)) & FMUTEST_CMD_CHECK_ALIGNFAIL_SCR_MASK)

#define FMUTEST_CMD_CHECK_ALIGNFAIL_BLK_MASK     (0x8U)
#define FMUTEST_CMD_CHECK_ALIGNFAIL_BLK_SHIFT    (3U)
/*! ALIGNFAIL_BLK - Block Alignment Fail
 *  0b0..The address is block-aligned
 *  0b1..The address is not block-aligned
 */
#define FMUTEST_CMD_CHECK_ALIGNFAIL_BLK(x)       (((uint32_t)(((uint32_t)(x)) << FMUTEST_CMD_CHECK_ALIGNFAIL_BLK_SHIFT)) & FMUTEST_CMD_CHECK_ALIGNFAIL_BLK_MASK)

#define FMUTEST_CMD_CHECK_ADDR_FAIL_MASK         (0x10U)
#define FMUTEST_CMD_CHECK_ADDR_FAIL_SHIFT        (4U)
/*! ADDR_FAIL - Address Fail
 *  0b0..The address is within the flash or IFR address space
 *  0b1..The address is outside the flash or IFR address space
 */
#define FMUTEST_CMD_CHECK_ADDR_FAIL(x)           (((uint32_t)(((uint32_t)(x)) << FMUTEST_CMD_CHECK_ADDR_FAIL_SHIFT)) & FMUTEST_CMD_CHECK_ADDR_FAIL_MASK)

#define FMUTEST_CMD_CHECK_IFR_CMD_MASK           (0x20U)
#define FMUTEST_CMD_CHECK_IFR_CMD_SHIFT          (5U)
/*! IFR_CMD - IFR Command
 *  0b0..The command operates on a main flash address
 *  0b1..The command operates on an IFR address
 */
#define FMUTEST_CMD_CHECK_IFR_CMD(x)             (((uint32_t)(((uint32_t)(x)) << FMUTEST_CMD_CHECK_IFR_CMD_SHIFT)) & FMUTEST_CMD_CHECK_IFR_CMD_MASK)

#define FMUTEST_CMD_CHECK_ALL_CMD_MASK           (0x40U)
#define FMUTEST_CMD_CHECK_ALL_CMD_SHIFT          (6U)
/*! ALL_CMD - All Blocks Command
 *  0b0..The command operates on a single flash block
 *  0b1..The command operates on all flash blocks
 */
#define FMUTEST_CMD_CHECK_ALL_CMD(x)             (((uint32_t)(((uint32_t)(x)) << FMUTEST_CMD_CHECK_ALL_CMD_SHIFT)) & FMUTEST_CMD_CHECK_ALL_CMD_MASK)

#define FMUTEST_CMD_CHECK_RANGE_FAIL_MASK        (0x80U)
#define FMUTEST_CMD_CHECK_RANGE_FAIL_SHIFT       (7U)
/*! RANGE_FAIL - Address Range Fail
 *  0b0..The address range is valid
 *  0b1..The address range is invalid
 */
#define FMUTEST_CMD_CHECK_RANGE_FAIL(x)          (((uint32_t)(((uint32_t)(x)) << FMUTEST_CMD_CHECK_RANGE_FAIL_SHIFT)) & FMUTEST_CMD_CHECK_RANGE_FAIL_MASK)

#define FMUTEST_CMD_CHECK_SCR_ALIGN_CHK_MASK     (0x100U)
#define FMUTEST_CMD_CHECK_SCR_ALIGN_CHK_SHIFT    (8U)
/*! SCR_ALIGN_CHK - Sector Alignment Check
 *  0b0..No sector alignment check
 *  0b1..Sector alignment check
 */
#define FMUTEST_CMD_CHECK_SCR_ALIGN_CHK(x)       (((uint32_t)(((uint32_t)(x)) << FMUTEST_CMD_CHECK_SCR_ALIGN_CHK_SHIFT)) & FMUTEST_CMD_CHECK_SCR_ALIGN_CHK_MASK)

#define FMUTEST_CMD_CHECK_OPTION_FAIL_MASK       (0x200U)
#define FMUTEST_CMD_CHECK_OPTION_FAIL_SHIFT      (9U)
/*! OPTION_FAIL - Option Check Fail
 *  0b0..Option check passes for read command or command is not a read command
 *  0b1..Option check fails for read command
 */
#define FMUTEST_CMD_CHECK_OPTION_FAIL(x)         (((uint32_t)(((uint32_t)(x)) << FMUTEST_CMD_CHECK_OPTION_FAIL_SHIFT)) & FMUTEST_CMD_CHECK_OPTION_FAIL_MASK)

#define FMUTEST_CMD_CHECK_ILLEGAL_CMD_MASK       (0x400U)
#define FMUTEST_CMD_CHECK_ILLEGAL_CMD_SHIFT      (10U)
/*! ILLEGAL_CMD - Illegal Command
 *  0b0..Command is legal
 *  0b1..Command is illegal
 */
#define FMUTEST_CMD_CHECK_ILLEGAL_CMD(x)         (((uint32_t)(((uint32_t)(x)) << FMUTEST_CMD_CHECK_ILLEGAL_CMD_SHIFT)) & FMUTEST_CMD_CHECK_ILLEGAL_CMD_MASK)
/*! @} */

/*! @name BSEL - FMU Block Select Register */
/*! @{ */

#define FMUTEST_BSEL_SBSEL_MASK                  (0x3U)
#define FMUTEST_BSEL_SBSEL_SHIFT                 (0U)
/*! SBSEL - Slave Block Select */
#define FMUTEST_BSEL_SBSEL(x)                    (((uint32_t)(((uint32_t)(x)) << FMUTEST_BSEL_SBSEL_SHIFT)) & FMUTEST_BSEL_SBSEL_MASK)

#define FMUTEST_BSEL_MBSEL_MASK                  (0x300U)
#define FMUTEST_BSEL_MBSEL_SHIFT                 (8U)
/*! MBSEL - Master Block Select */
#define FMUTEST_BSEL_MBSEL(x)                    (((uint32_t)(((uint32_t)(x)) << FMUTEST_BSEL_MBSEL_SHIFT)) & FMUTEST_BSEL_MBSEL_MASK)
/*! @} */

/*! @name MSIZE - FMU Memory Size Register */
/*! @{ */

#define FMUTEST_MSIZE_MAXADDR0_MASK              (0xFFU)
#define FMUTEST_MSIZE_MAXADDR0_SHIFT             (0U)
/*! MAXADDR0 - Size of Flash Block 0 */
#define FMUTEST_MSIZE_MAXADDR0(x)                (((uint32_t)(((uint32_t)(x)) << FMUTEST_MSIZE_MAXADDR0_SHIFT)) & FMUTEST_MSIZE_MAXADDR0_MASK)
/*! @} */

/*! @name FLASH_RD_ADD - Flash Read Address Register */
/*! @{ */

#define FMUTEST_FLASH_RD_ADD_FLASH_RD_ADD_MASK   (0xFFFFFFFFU)
#define FMUTEST_FLASH_RD_ADD_FLASH_RD_ADD_SHIFT  (0U)
/*! FLASH_RD_ADD - Flash Read Address */
#define FMUTEST_FLASH_RD_ADD_FLASH_RD_ADD(x)     (((uint32_t)(((uint32_t)(x)) << FMUTEST_FLASH_RD_ADD_FLASH_RD_ADD_SHIFT)) & FMUTEST_FLASH_RD_ADD_FLASH_RD_ADD_MASK)
/*! @} */

/*! @name FLASH_STOP_ADD - Flash Stop Address Register */
/*! @{ */

#define FMUTEST_FLASH_STOP_ADD_FLASH_STOP_ADD_MASK (0xFFFFFFFFU)
#define FMUTEST_FLASH_STOP_ADD_FLASH_STOP_ADD_SHIFT (0U)
/*! FLASH_STOP_ADD - Flash Stop Address */
#define FMUTEST_FLASH_STOP_ADD_FLASH_STOP_ADD(x) (((uint32_t)(((uint32_t)(x)) << FMUTEST_FLASH_STOP_ADD_FLASH_STOP_ADD_SHIFT)) & FMUTEST_FLASH_STOP_ADD_FLASH_STOP_ADD_MASK)
/*! @} */

/*! @name FLASH_RD_CTRL - Flash Read Control Register */
/*! @{ */

#define FMUTEST_FLASH_RD_CTRL_FLASH_RD_MASK      (0x1U)
#define FMUTEST_FLASH_RD_CTRL_FLASH_RD_SHIFT     (0U)
/*! FLASH_RD - Flash Read Enable
 *  0b0..Manual flash read not enabled.(default)
 *  0b1..Manual flash read enabled
 */
#define FMUTEST_FLASH_RD_CTRL_FLASH_RD(x)        (((uint32_t)(((uint32_t)(x)) << FMUTEST_FLASH_RD_CTRL_FLASH_RD_SHIFT)) & FMUTEST_FLASH_RD_CTRL_FLASH_RD_MASK)

#define FMUTEST_FLASH_RD_CTRL_WIDE_LOAD_MASK     (0x2U)
#define FMUTEST_FLASH_RD_CTRL_WIDE_LOAD_SHIFT    (1U)
/*! WIDE_LOAD - Wide Load Enable
 *  0b0..Wide load mode disabled (default)
 *  0b1..Wide load mode enabled
 */
#define FMUTEST_FLASH_RD_CTRL_WIDE_LOAD(x)       (((uint32_t)(((uint32_t)(x)) << FMUTEST_FLASH_RD_CTRL_WIDE_LOAD_SHIFT)) & FMUTEST_FLASH_RD_CTRL_WIDE_LOAD_MASK)

#define FMUTEST_FLASH_RD_CTRL_SINGLE_RD_MASK     (0x4U)
#define FMUTEST_FLASH_RD_CTRL_SINGLE_RD_SHIFT    (2U)
/*! SINGLE_RD - Single Flash Read
 *  0b0..Normal UINT operation
 *  0b1..UINT configured for single cycle reads
 */
#define FMUTEST_FLASH_RD_CTRL_SINGLE_RD(x)       (((uint32_t)(((uint32_t)(x)) << FMUTEST_FLASH_RD_CTRL_SINGLE_RD_SHIFT)) & FMUTEST_FLASH_RD_CTRL_SINGLE_RD_MASK)
/*! @} */

/*! @name MM_ADDR - Memory Map Address Register */
/*! @{ */

#define FMUTEST_MM_ADDR_MM_ADDR_MASK             (0xFFFFFFFFU)
#define FMUTEST_MM_ADDR_MM_ADDR_SHIFT            (0U)
/*! MM_ADDR - Memory Map Address */
#define FMUTEST_MM_ADDR_MM_ADDR(x)               (((uint32_t)(((uint32_t)(x)) << FMUTEST_MM_ADDR_MM_ADDR_SHIFT)) & FMUTEST_MM_ADDR_MM_ADDR_MASK)
/*! @} */

/*! @name MM_WDATA - Memory Map Write Data Register */
/*! @{ */

#define FMUTEST_MM_WDATA_MM_WDATA_MASK           (0xFFFFFFFFU)
#define FMUTEST_MM_WDATA_MM_WDATA_SHIFT          (0U)
/*! MM_WDATA - Memory Map Write Data */
#define FMUTEST_MM_WDATA_MM_WDATA(x)             (((uint32_t)(((uint32_t)(x)) << FMUTEST_MM_WDATA_MM_WDATA_SHIFT)) & FMUTEST_MM_WDATA_MM_WDATA_MASK)
/*! @} */

/*! @name MM_CTL - Memory Map Control Register */
/*! @{ */

#define FMUTEST_MM_CTL_MM_SEL_MASK               (0x1U)
#define FMUTEST_MM_CTL_MM_SEL_SHIFT              (0U)
/*! MM_SEL - Register Access Enable */
#define FMUTEST_MM_CTL_MM_SEL(x)                 (((uint32_t)(((uint32_t)(x)) << FMUTEST_MM_CTL_MM_SEL_SHIFT)) & FMUTEST_MM_CTL_MM_SEL_MASK)

#define FMUTEST_MM_CTL_MM_RD_MASK                (0x2U)
#define FMUTEST_MM_CTL_MM_RD_SHIFT               (1U)
/*! MM_RD - Register R/W Control
 *  0b0..Write to register
 *  0b1..Read register
 */
#define FMUTEST_MM_CTL_MM_RD(x)                  (((uint32_t)(((uint32_t)(x)) << FMUTEST_MM_CTL_MM_RD_SHIFT)) & FMUTEST_MM_CTL_MM_RD_MASK)

#define FMUTEST_MM_CTL_BIST_ON_MASK              (0x4U)
#define FMUTEST_MM_CTL_BIST_ON_SHIFT             (2U)
/*! BIST_ON - BIST on
 *  0b0..BIST enable not forced by user interface
 *  0b1..BIST enable control by user interface
 */
#define FMUTEST_MM_CTL_BIST_ON(x)                (((uint32_t)(((uint32_t)(x)) << FMUTEST_MM_CTL_BIST_ON_SHIFT)) & FMUTEST_MM_CTL_BIST_ON_MASK)

#define FMUTEST_MM_CTL_FORCE_SW_CLK_MASK         (0x8U)
#define FMUTEST_MM_CTL_FORCE_SW_CLK_SHIFT        (3U)
/*! FORCE_SW_CLK - Force Switch Clock
 *  0b0..Switch clock not forced on (gated normally)
 *  0b1..Switch clock forced on
 */
#define FMUTEST_MM_CTL_FORCE_SW_CLK(x)           (((uint32_t)(((uint32_t)(x)) << FMUTEST_MM_CTL_FORCE_SW_CLK_SHIFT)) & FMUTEST_MM_CTL_FORCE_SW_CLK_MASK)
/*! @} */

/*! @name UINT_CTL - User Interface Control Register */
/*! @{ */

#define FMUTEST_UINT_CTL_SET_FAIL_MASK           (0x1U)
#define FMUTEST_UINT_CTL_SET_FAIL_SHIFT          (0U)
/*! SET_FAIL - Set Fail On Exit
 *  0b0..FAIL flag should not be set on command exit (no failure detected)
 *  0b1..FAIL flag should be set on command exit
 */
#define FMUTEST_UINT_CTL_SET_FAIL(x)             (((uint32_t)(((uint32_t)(x)) << FMUTEST_UINT_CTL_SET_FAIL_SHIFT)) & FMUTEST_UINT_CTL_SET_FAIL_MASK)

#define FMUTEST_UINT_CTL_DBERR_MASK              (0x2U)
#define FMUTEST_UINT_CTL_DBERR_SHIFT             (1U)
/*! DBERR - Double-Bit ECC Fault Detect
 *  0b0..No double-bit fault detected during UINT-driven read sequence
 *  0b1..Double-bit fault detected during UINT-driven read sequence
 */
#define FMUTEST_UINT_CTL_DBERR(x)                (((uint32_t)(((uint32_t)(x)) << FMUTEST_UINT_CTL_DBERR_SHIFT)) & FMUTEST_UINT_CTL_DBERR_MASK)
/*! @} */

/*! @name RD_DATA0 - Read Data 0 Register */
/*! @{ */

#define FMUTEST_RD_DATA0_RD_DATA0_MASK           (0xFFFFFFFFU)
#define FMUTEST_RD_DATA0_RD_DATA0_SHIFT          (0U)
/*! RD_DATA0 - Read Data 0 */
#define FMUTEST_RD_DATA0_RD_DATA0(x)             (((uint32_t)(((uint32_t)(x)) << FMUTEST_RD_DATA0_RD_DATA0_SHIFT)) & FMUTEST_RD_DATA0_RD_DATA0_MASK)
/*! @} */

/*! @name RD_DATA1 - Read Data 1 Register */
/*! @{ */

#define FMUTEST_RD_DATA1_RD_DATA1_MASK           (0xFFFFFFFFU)
#define FMUTEST_RD_DATA1_RD_DATA1_SHIFT          (0U)
/*! RD_DATA1 - Read Data 1 */
#define FMUTEST_RD_DATA1_RD_DATA1(x)             (((uint32_t)(((uint32_t)(x)) << FMUTEST_RD_DATA1_RD_DATA1_SHIFT)) & FMUTEST_RD_DATA1_RD_DATA1_MASK)
/*! @} */

/*! @name RD_DATA2 - Read Data 2 Register */
/*! @{ */

#define FMUTEST_RD_DATA2_RD_DATA2_MASK           (0xFFFFFFFFU)
#define FMUTEST_RD_DATA2_RD_DATA2_SHIFT          (0U)
/*! RD_DATA2 - Read Data 2 */
#define FMUTEST_RD_DATA2_RD_DATA2(x)             (((uint32_t)(((uint32_t)(x)) << FMUTEST_RD_DATA2_RD_DATA2_SHIFT)) & FMUTEST_RD_DATA2_RD_DATA2_MASK)
/*! @} */

/*! @name RD_DATA3 - Read Data 3 Register */
/*! @{ */

#define FMUTEST_RD_DATA3_RD_DATA3_MASK           (0xFFFFFFFFU)
#define FMUTEST_RD_DATA3_RD_DATA3_SHIFT          (0U)
/*! RD_DATA3 - Read Data 3 */
#define FMUTEST_RD_DATA3_RD_DATA3(x)             (((uint32_t)(((uint32_t)(x)) << FMUTEST_RD_DATA3_RD_DATA3_SHIFT)) & FMUTEST_RD_DATA3_RD_DATA3_MASK)
/*! @} */

/*! @name PARITY - Parity Register */
/*! @{ */

#define FMUTEST_PARITY_PARITY_MASK               (0x1FFU)
#define FMUTEST_PARITY_PARITY_SHIFT              (0U)
/*! PARITY - Read data [136:128] */
#define FMUTEST_PARITY_PARITY(x)                 (((uint32_t)(((uint32_t)(x)) << FMUTEST_PARITY_PARITY_SHIFT)) & FMUTEST_PARITY_PARITY_MASK)
/*! @} */

/*! @name RD_PATH_CTRL_STATUS - Read Path Control and Status Register */
/*! @{ */

#define FMUTEST_RD_PATH_CTRL_STATUS_RD_CAPT_MASK (0xFFU)
#define FMUTEST_RD_PATH_CTRL_STATUS_RD_CAPT_SHIFT (0U)
/*! RD_CAPT - Read Capture Clock Periods */
#define FMUTEST_RD_PATH_CTRL_STATUS_RD_CAPT(x)   (((uint32_t)(((uint32_t)(x)) << FMUTEST_RD_PATH_CTRL_STATUS_RD_CAPT_SHIFT)) & FMUTEST_RD_PATH_CTRL_STATUS_RD_CAPT_MASK)

#define FMUTEST_RD_PATH_CTRL_STATUS_SE_SIZE_MASK (0xFF00U)
#define FMUTEST_RD_PATH_CTRL_STATUS_SE_SIZE_SHIFT (8U)
/*! SE_SIZE - SE Clock Periods */
#define FMUTEST_RD_PATH_CTRL_STATUS_SE_SIZE(x)   (((uint32_t)(((uint32_t)(x)) << FMUTEST_RD_PATH_CTRL_STATUS_SE_SIZE_SHIFT)) & FMUTEST_RD_PATH_CTRL_STATUS_SE_SIZE_MASK)

#define FMUTEST_RD_PATH_CTRL_STATUS_ECC_ENABLEB_MASK (0x10000U)
#define FMUTEST_RD_PATH_CTRL_STATUS_ECC_ENABLEB_SHIFT (16U)
/*! ECC_ENABLEB - ECC Decoder Control
 *  0b0..ECC decoder enabled (default)
 *  0b1..ECC decoder disabled
 */
#define FMUTEST_RD_PATH_CTRL_STATUS_ECC_ENABLEB(x) (((uint32_t)(((uint32_t)(x)) << FMUTEST_RD_PATH_CTRL_STATUS_ECC_ENABLEB_SHIFT)) & FMUTEST_RD_PATH_CTRL_STATUS_ECC_ENABLEB_MASK)

#define FMUTEST_RD_PATH_CTRL_STATUS_MISR_EN_MASK (0x20000U)
#define FMUTEST_RD_PATH_CTRL_STATUS_MISR_EN_SHIFT (17U)
/*! MISR_EN - MISR Enable
 *  0b0..MISR option disabled (default)
 *  0b1..MISR option enabled
 */
#define FMUTEST_RD_PATH_CTRL_STATUS_MISR_EN(x)   (((uint32_t)(((uint32_t)(x)) << FMUTEST_RD_PATH_CTRL_STATUS_MISR_EN_SHIFT)) & FMUTEST_RD_PATH_CTRL_STATUS_MISR_EN_MASK)

#define FMUTEST_RD_PATH_CTRL_STATUS_CPY_PAR_EN_MASK (0x40000U)
#define FMUTEST_RD_PATH_CTRL_STATUS_CPY_PAR_EN_SHIFT (18U)
/*! CPY_PAR_EN - Copy Parity Enable
 *  0b0..Copy parity disabled
 *  0b1..Copy parity enabled
 */
#define FMUTEST_RD_PATH_CTRL_STATUS_CPY_PAR_EN(x) (((uint32_t)(((uint32_t)(x)) << FMUTEST_RD_PATH_CTRL_STATUS_CPY_PAR_EN_SHIFT)) & FMUTEST_RD_PATH_CTRL_STATUS_CPY_PAR_EN_MASK)

#define FMUTEST_RD_PATH_CTRL_STATUS_BIST_MUX_TO_SMW_MASK (0x80000U)
#define FMUTEST_RD_PATH_CTRL_STATUS_BIST_MUX_TO_SMW_SHIFT (19U)
/*! BIST_MUX_TO_SMW - BIST Mux to SMW
 *  0b0..BIST drives fields
 *  0b1..SMW registers drive fields
 */
#define FMUTEST_RD_PATH_CTRL_STATUS_BIST_MUX_TO_SMW(x) (((uint32_t)(((uint32_t)(x)) << FMUTEST_RD_PATH_CTRL_STATUS_BIST_MUX_TO_SMW_SHIFT)) & FMUTEST_RD_PATH_CTRL_STATUS_BIST_MUX_TO_SMW_MASK)

#define FMUTEST_RD_PATH_CTRL_STATUS_AD_SET_MASK  (0xF00000U)
#define FMUTEST_RD_PATH_CTRL_STATUS_AD_SET_SHIFT (20U)
/*! AD_SET - Multi-Cycle Address Setup Time */
#define FMUTEST_RD_PATH_CTRL_STATUS_AD_SET(x)    (((uint32_t)(((uint32_t)(x)) << FMUTEST_RD_PATH_CTRL_STATUS_AD_SET_SHIFT)) & FMUTEST_RD_PATH_CTRL_STATUS_AD_SET_MASK)

#define FMUTEST_RD_PATH_CTRL_STATUS_WR_PATH_EN_MASK (0x1000000U)
#define FMUTEST_RD_PATH_CTRL_STATUS_WR_PATH_EN_SHIFT (24U)
/*! WR_PATH_EN - Write Path Enable
 *  0b0..Writes to BIST setting registers driven by MM_WDATA
 *  0b1..Writes to BIST setting registers driven by SMW_DIN
 */
#define FMUTEST_RD_PATH_CTRL_STATUS_WR_PATH_EN(x) (((uint32_t)(((uint32_t)(x)) << FMUTEST_RD_PATH_CTRL_STATUS_WR_PATH_EN_SHIFT)) & FMUTEST_RD_PATH_CTRL_STATUS_WR_PATH_EN_MASK)

#define FMUTEST_RD_PATH_CTRL_STATUS_WR_PATH_ECC_EN_MASK (0x2000000U)
#define FMUTEST_RD_PATH_CTRL_STATUS_WR_PATH_ECC_EN_SHIFT (25U)
/*! WR_PATH_ECC_EN - Write Path ECC Enable
 *  0b0..ECC encoding disabled
 *  0b1..ECC encoding enabled
 */
#define FMUTEST_RD_PATH_CTRL_STATUS_WR_PATH_ECC_EN(x) (((uint32_t)(((uint32_t)(x)) << FMUTEST_RD_PATH_CTRL_STATUS_WR_PATH_ECC_EN_SHIFT)) & FMUTEST_RD_PATH_CTRL_STATUS_WR_PATH_ECC_EN_MASK)

#define FMUTEST_RD_PATH_CTRL_STATUS_DBERR_REG_MASK (0x4000000U)
#define FMUTEST_RD_PATH_CTRL_STATUS_DBERR_REG_SHIFT (26U)
/*! DBERR_REG - Double-Bit Error
 *  0b0..Double-bit fault not detected
 *  0b1..Double-bit fault detected on previous UINT flash read
 */
#define FMUTEST_RD_PATH_CTRL_STATUS_DBERR_REG(x) (((uint32_t)(((uint32_t)(x)) << FMUTEST_RD_PATH_CTRL_STATUS_DBERR_REG_SHIFT)) & FMUTEST_RD_PATH_CTRL_STATUS_DBERR_REG_MASK)

#define FMUTEST_RD_PATH_CTRL_STATUS_SBERR_REG_MASK (0x8000000U)
#define FMUTEST_RD_PATH_CTRL_STATUS_SBERR_REG_SHIFT (27U)
/*! SBERR_REG - Single-Bit Error
 *  0b0..Single-bit fault not detected
 *  0b1..Single-bit fault detected on previous UINT flash read
 */
#define FMUTEST_RD_PATH_CTRL_STATUS_SBERR_REG(x) (((uint32_t)(((uint32_t)(x)) << FMUTEST_RD_PATH_CTRL_STATUS_SBERR_REG_SHIFT)) & FMUTEST_RD_PATH_CTRL_STATUS_SBERR_REG_MASK)

#define FMUTEST_RD_PATH_CTRL_STATUS_CPY_PHRASE_EN_MASK (0x10000000U)
#define FMUTEST_RD_PATH_CTRL_STATUS_CPY_PHRASE_EN_SHIFT (28U)
/*! CPY_PHRASE_EN - Copy Phrase Enable
 *  0b0..Copy Flash read data disabled
 *  0b1..Copy Flash read data enabled
 */
#define FMUTEST_RD_PATH_CTRL_STATUS_CPY_PHRASE_EN(x) (((uint32_t)(((uint32_t)(x)) << FMUTEST_RD_PATH_CTRL_STATUS_CPY_PHRASE_EN_SHIFT)) & FMUTEST_RD_PATH_CTRL_STATUS_CPY_PHRASE_EN_MASK)

#define FMUTEST_RD_PATH_CTRL_STATUS_SMW_ARRAY1_SMW0_SEL_MASK (0x20000000U)
#define FMUTEST_RD_PATH_CTRL_STATUS_SMW_ARRAY1_SMW0_SEL_SHIFT (29U)
/*! SMW_ARRAY1_SMW0_SEL - SMW_ARRAY1_SMW0_SEL
 *  0b0..Select block 0
 *  0b1..Select block 1
 */
#define FMUTEST_RD_PATH_CTRL_STATUS_SMW_ARRAY1_SMW0_SEL(x) (((uint32_t)(((uint32_t)(x)) << FMUTEST_RD_PATH_CTRL_STATUS_SMW_ARRAY1_SMW0_SEL_SHIFT)) & FMUTEST_RD_PATH_CTRL_STATUS_SMW_ARRAY1_SMW0_SEL_MASK)

#define FMUTEST_RD_PATH_CTRL_STATUS_BIST_ECC_EN_MASK (0x40000000U)
#define FMUTEST_RD_PATH_CTRL_STATUS_BIST_ECC_EN_SHIFT (30U)
/*! BIST_ECC_EN - BIST ECC Enable
 *  0b0..ECC correction disabled
 *  0b1..ECC correction enabled
 */
#define FMUTEST_RD_PATH_CTRL_STATUS_BIST_ECC_EN(x) (((uint32_t)(((uint32_t)(x)) << FMUTEST_RD_PATH_CTRL_STATUS_BIST_ECC_EN_SHIFT)) & FMUTEST_RD_PATH_CTRL_STATUS_BIST_ECC_EN_MASK)

#define FMUTEST_RD_PATH_CTRL_STATUS_LAST_READ_MASK (0x80000000U)
#define FMUTEST_RD_PATH_CTRL_STATUS_LAST_READ_SHIFT (31U)
/*! LAST_READ - Last Read
 *  0b0..Latest read not last in multi-address operation
 *  0b1..Latest read last in multi-address operation
 */
#define FMUTEST_RD_PATH_CTRL_STATUS_LAST_READ(x) (((uint32_t)(((uint32_t)(x)) << FMUTEST_RD_PATH_CTRL_STATUS_LAST_READ_SHIFT)) & FMUTEST_RD_PATH_CTRL_STATUS_LAST_READ_MASK)
/*! @} */

/*! @name SMW_DIN0 - SMW DIN 0 Register */
/*! @{ */

#define FMUTEST_SMW_DIN0_SMW_DIN0_MASK           (0xFFFFFFFFU)
#define FMUTEST_SMW_DIN0_SMW_DIN0_SHIFT          (0U)
/*! SMW_DIN0 - SMW DIN 0 */
#define FMUTEST_SMW_DIN0_SMW_DIN0(x)             (((uint32_t)(((uint32_t)(x)) << FMUTEST_SMW_DIN0_SMW_DIN0_SHIFT)) & FMUTEST_SMW_DIN0_SMW_DIN0_MASK)
/*! @} */

/*! @name SMW_DIN1 - SMW DIN 1 Register */
/*! @{ */

#define FMUTEST_SMW_DIN1_SMW_DIN1_MASK           (0xFFFFFFFFU)
#define FMUTEST_SMW_DIN1_SMW_DIN1_SHIFT          (0U)
/*! SMW_DIN1 - SMW DIN 1 */
#define FMUTEST_SMW_DIN1_SMW_DIN1(x)             (((uint32_t)(((uint32_t)(x)) << FMUTEST_SMW_DIN1_SMW_DIN1_SHIFT)) & FMUTEST_SMW_DIN1_SMW_DIN1_MASK)
/*! @} */

/*! @name SMW_DIN2 - SMW DIN 2 Register */
/*! @{ */

#define FMUTEST_SMW_DIN2_SMW_DIN2_MASK           (0xFFFFFFFFU)
#define FMUTEST_SMW_DIN2_SMW_DIN2_SHIFT          (0U)
/*! SMW_DIN2 - SMW DIN 2 */
#define FMUTEST_SMW_DIN2_SMW_DIN2(x)             (((uint32_t)(((uint32_t)(x)) << FMUTEST_SMW_DIN2_SMW_DIN2_SHIFT)) & FMUTEST_SMW_DIN2_SMW_DIN2_MASK)
/*! @} */

/*! @name SMW_DIN3 - SMW DIN 3 Register */
/*! @{ */

#define FMUTEST_SMW_DIN3_SMW_DIN3_MASK           (0xFFFFFFFFU)
#define FMUTEST_SMW_DIN3_SMW_DIN3_SHIFT          (0U)
/*! SMW_DIN3 - SMW DIN 3 */
#define FMUTEST_SMW_DIN3_SMW_DIN3(x)             (((uint32_t)(((uint32_t)(x)) << FMUTEST_SMW_DIN3_SMW_DIN3_SHIFT)) & FMUTEST_SMW_DIN3_SMW_DIN3_MASK)
/*! @} */

/*! @name SMW_ADDR - SMW Address Register */
/*! @{ */

#define FMUTEST_SMW_ADDR_SMW_ADDR_MASK           (0xFFFFFFFFU)
#define FMUTEST_SMW_ADDR_SMW_ADDR_SHIFT          (0U)
/*! SMW_ADDR - SMW Address */
#define FMUTEST_SMW_ADDR_SMW_ADDR(x)             (((uint32_t)(((uint32_t)(x)) << FMUTEST_SMW_ADDR_SMW_ADDR_SHIFT)) & FMUTEST_SMW_ADDR_SMW_ADDR_MASK)
/*! @} */

/*! @name SMW_CMD_WAIT - SMW Command and Wait Register */
/*! @{ */

#define FMUTEST_SMW_CMD_WAIT_CMD_MASK            (0x7U)
#define FMUTEST_SMW_CMD_WAIT_CMD_SHIFT           (0U)
/*! CMD - SMW Command
 *  0b000..IDLE
 *  0b001..ABORT
 *  0b010..SME2 to one-shot mass erase
 *  0b011..SME3 to sector erase on selected array
 *  0b100..SMP1 to program phrase or page on selected array with shot disabled on previously programmed bit
 *  0b101..Reserved for SME4 (multi-sector erase)
 *  0b110..SMP2 to program phrase or page on selected array to repair cells of weak program after power loss
 *  0b111..Reserved
 */
#define FMUTEST_SMW_CMD_WAIT_CMD(x)              (((uint32_t)(((uint32_t)(x)) << FMUTEST_SMW_CMD_WAIT_CMD_SHIFT)) & FMUTEST_SMW_CMD_WAIT_CMD_MASK)

#define FMUTEST_SMW_CMD_WAIT_WAIT_EN_MASK        (0x8U)
#define FMUTEST_SMW_CMD_WAIT_WAIT_EN_SHIFT       (3U)
/*! WAIT_EN - SMW Wait Enable
 *  0b0..Wait feature disabled
 *  0b1..Wait feature enabled
 */
#define FMUTEST_SMW_CMD_WAIT_WAIT_EN(x)          (((uint32_t)(((uint32_t)(x)) << FMUTEST_SMW_CMD_WAIT_WAIT_EN_SHIFT)) & FMUTEST_SMW_CMD_WAIT_WAIT_EN_MASK)

#define FMUTEST_SMW_CMD_WAIT_WAIT_AUTO_SET_MASK  (0x10U)
#define FMUTEST_SMW_CMD_WAIT_WAIT_AUTO_SET_SHIFT (4U)
/*! WAIT_AUTO_SET - SMW Wait Auto Set */
#define FMUTEST_SMW_CMD_WAIT_WAIT_AUTO_SET(x)    (((uint32_t)(((uint32_t)(x)) << FMUTEST_SMW_CMD_WAIT_WAIT_AUTO_SET_SHIFT)) & FMUTEST_SMW_CMD_WAIT_WAIT_AUTO_SET_MASK)
/*! @} */

/*! @name SMW_STATUS - SMW Status Register */
/*! @{ */

#define FMUTEST_SMW_STATUS_SMW_ERR_MASK          (0x1U)
#define FMUTEST_SMW_STATUS_SMW_ERR_SHIFT         (0U)
/*! SMW_ERR - SMW Error
 *  0b0..Error not detected
 *  0b1..Error detected
 */
#define FMUTEST_SMW_STATUS_SMW_ERR(x)            (((uint32_t)(((uint32_t)(x)) << FMUTEST_SMW_STATUS_SMW_ERR_SHIFT)) & FMUTEST_SMW_STATUS_SMW_ERR_MASK)

#define FMUTEST_SMW_STATUS_SMW_BUSY_MASK         (0x2U)
#define FMUTEST_SMW_STATUS_SMW_BUSY_SHIFT        (1U)
/*! SMW_BUSY - SMW Busy
 *  0b0..SMW command not active
 *  0b1..SMW command is active
 */
#define FMUTEST_SMW_STATUS_SMW_BUSY(x)           (((uint32_t)(((uint32_t)(x)) << FMUTEST_SMW_STATUS_SMW_BUSY_SHIFT)) & FMUTEST_SMW_STATUS_SMW_BUSY_MASK)

#define FMUTEST_SMW_STATUS_BIST_BUSY_MASK        (0x4U)
#define FMUTEST_SMW_STATUS_BIST_BUSY_SHIFT       (2U)
/*! BIST_BUSY - BIST Busy
 *  0b0..BIST Command not active
 *  0b1..BIST Command is active
 */
#define FMUTEST_SMW_STATUS_BIST_BUSY(x)          (((uint32_t)(((uint32_t)(x)) << FMUTEST_SMW_STATUS_BIST_BUSY_SHIFT)) & FMUTEST_SMW_STATUS_BIST_BUSY_MASK)
/*! @} */

/*! @name SOCTRIM0_0 - SoC Trim Phrase 0 Word 0 Register */
/*! @{ */

#define FMUTEST_SOCTRIM0_0_TRIM0_0_MASK          (0xFFFFFFFFU)
#define FMUTEST_SOCTRIM0_0_TRIM0_0_SHIFT         (0U)
/*! TRIM0_0 - TRIM0_0 */
#define FMUTEST_SOCTRIM0_0_TRIM0_0(x)            (((uint32_t)(((uint32_t)(x)) << FMUTEST_SOCTRIM0_0_TRIM0_0_SHIFT)) & FMUTEST_SOCTRIM0_0_TRIM0_0_MASK)
/*! @} */

/*! @name SOCTRIM0_1 - SoC Trim Phrase 0 Word 1 Register */
/*! @{ */

#define FMUTEST_SOCTRIM0_1_TRIM0_1_MASK          (0xFFFFFFFFU)
#define FMUTEST_SOCTRIM0_1_TRIM0_1_SHIFT         (0U)
/*! TRIM0_1 - TRIM0_1 */
#define FMUTEST_SOCTRIM0_1_TRIM0_1(x)            (((uint32_t)(((uint32_t)(x)) << FMUTEST_SOCTRIM0_1_TRIM0_1_SHIFT)) & FMUTEST_SOCTRIM0_1_TRIM0_1_MASK)
/*! @} */

/*! @name SOCTRIM0_2 - SoC Trim Phrase 0 Word 2 Register */
/*! @{ */

#define FMUTEST_SOCTRIM0_2_TRIM0_2_MASK          (0xFFFFFFFFU)
#define FMUTEST_SOCTRIM0_2_TRIM0_2_SHIFT         (0U)
/*! TRIM0_2 - TRIM0_2 */
#define FMUTEST_SOCTRIM0_2_TRIM0_2(x)            (((uint32_t)(((uint32_t)(x)) << FMUTEST_SOCTRIM0_2_TRIM0_2_SHIFT)) & FMUTEST_SOCTRIM0_2_TRIM0_2_MASK)
/*! @} */

/*! @name SOCTRIM0_3 - SoC Trim Phrase 0 Word 3 Register */
/*! @{ */

#define FMUTEST_SOCTRIM0_3_TRIM0_3_MASK          (0xFFFFFFFFU)
#define FMUTEST_SOCTRIM0_3_TRIM0_3_SHIFT         (0U)
/*! TRIM0_3 - TRIM0_3 */
#define FMUTEST_SOCTRIM0_3_TRIM0_3(x)            (((uint32_t)(((uint32_t)(x)) << FMUTEST_SOCTRIM0_3_TRIM0_3_SHIFT)) & FMUTEST_SOCTRIM0_3_TRIM0_3_MASK)
/*! @} */

/*! @name SOCTRIM1_0 - SoC Trim Phrase 1 Word 0 Register */
/*! @{ */

#define FMUTEST_SOCTRIM1_0_TRIM1_0_MASK          (0xFFFFFFFFU)
#define FMUTEST_SOCTRIM1_0_TRIM1_0_SHIFT         (0U)
/*! TRIM1_0 - TRIM1_0 */
#define FMUTEST_SOCTRIM1_0_TRIM1_0(x)            (((uint32_t)(((uint32_t)(x)) << FMUTEST_SOCTRIM1_0_TRIM1_0_SHIFT)) & FMUTEST_SOCTRIM1_0_TRIM1_0_MASK)
/*! @} */

/*! @name SOCTRIM1_1 - SoC Trim Phrase 1 Word 1 Register */
/*! @{ */

#define FMUTEST_SOCTRIM1_1_TRIM1_1_MASK          (0xFFFFFFFFU)
#define FMUTEST_SOCTRIM1_1_TRIM1_1_SHIFT         (0U)
/*! TRIM1_1 - TRIM1_1 */
#define FMUTEST_SOCTRIM1_1_TRIM1_1(x)            (((uint32_t)(((uint32_t)(x)) << FMUTEST_SOCTRIM1_1_TRIM1_1_SHIFT)) & FMUTEST_SOCTRIM1_1_TRIM1_1_MASK)
/*! @} */

/*! @name SOCTRIM1_2 - SoC Trim Phrase 1 Word 2 Register */
/*! @{ */

#define FMUTEST_SOCTRIM1_2_TRIM1_2_MASK          (0xFFFFFFFFU)
#define FMUTEST_SOCTRIM1_2_TRIM1_2_SHIFT         (0U)
/*! TRIM1_2 - TRIM1_2 */
#define FMUTEST_SOCTRIM1_2_TRIM1_2(x)            (((uint32_t)(((uint32_t)(x)) << FMUTEST_SOCTRIM1_2_TRIM1_2_SHIFT)) & FMUTEST_SOCTRIM1_2_TRIM1_2_MASK)
/*! @} */

/*! @name SOCTRIM1_3 - SoC Trim Phrase 1 Word 3 Register */
/*! @{ */

#define FMUTEST_SOCTRIM1_3_TRIM1_3_MASK          (0xFFFFFFFFU)
#define FMUTEST_SOCTRIM1_3_TRIM1_3_SHIFT         (0U)
/*! TRIM1_3 - TRIM1_3 */
#define FMUTEST_SOCTRIM1_3_TRIM1_3(x)            (((uint32_t)(((uint32_t)(x)) << FMUTEST_SOCTRIM1_3_TRIM1_3_SHIFT)) & FMUTEST_SOCTRIM1_3_TRIM1_3_MASK)
/*! @} */

/*! @name SOCTRIM2_0 - SoC Trim Phrase 2 Word 0 Register */
/*! @{ */

#define FMUTEST_SOCTRIM2_0_TRIM2_0_MASK          (0xFFFFFFFFU)
#define FMUTEST_SOCTRIM2_0_TRIM2_0_SHIFT         (0U)
/*! TRIM2_0 - TRIM2_0 */
#define FMUTEST_SOCTRIM2_0_TRIM2_0(x)            (((uint32_t)(((uint32_t)(x)) << FMUTEST_SOCTRIM2_0_TRIM2_0_SHIFT)) & FMUTEST_SOCTRIM2_0_TRIM2_0_MASK)
/*! @} */

/*! @name SOCTRIM2_1 - SoC Trim Phrase 2 Word 1 Register */
/*! @{ */

#define FMUTEST_SOCTRIM2_1_TRIM2_1_MASK          (0xFFFFFFFFU)
#define FMUTEST_SOCTRIM2_1_TRIM2_1_SHIFT         (0U)
/*! TRIM2_1 - TRIM2_1 */
#define FMUTEST_SOCTRIM2_1_TRIM2_1(x)            (((uint32_t)(((uint32_t)(x)) << FMUTEST_SOCTRIM2_1_TRIM2_1_SHIFT)) & FMUTEST_SOCTRIM2_1_TRIM2_1_MASK)
/*! @} */

/*! @name SOCTRIM2_2 - SoC Trim Phrase 2 Word 2 Register */
/*! @{ */

#define FMUTEST_SOCTRIM2_2_TRIM2_2_MASK          (0xFFFFFFFFU)
#define FMUTEST_SOCTRIM2_2_TRIM2_2_SHIFT         (0U)
/*! TRIM2_2 - TRIM2_2 */
#define FMUTEST_SOCTRIM2_2_TRIM2_2(x)            (((uint32_t)(((uint32_t)(x)) << FMUTEST_SOCTRIM2_2_TRIM2_2_SHIFT)) & FMUTEST_SOCTRIM2_2_TRIM2_2_MASK)
/*! @} */

/*! @name SOCTRIM2_3 - SoC Trim Phrase 2 Word 3 Register */
/*! @{ */

#define FMUTEST_SOCTRIM2_3_TRIM2_3_MASK          (0xFFFFFFFFU)
#define FMUTEST_SOCTRIM2_3_TRIM2_3_SHIFT         (0U)
/*! TRIM2_3 - TRIM2_3 */
#define FMUTEST_SOCTRIM2_3_TRIM2_3(x)            (((uint32_t)(((uint32_t)(x)) << FMUTEST_SOCTRIM2_3_TRIM2_3_SHIFT)) & FMUTEST_SOCTRIM2_3_TRIM2_3_MASK)
/*! @} */

/*! @name SOCTRIM3_0 - SoC Trim Phrase 3 Word 0 Register */
/*! @{ */

#define FMUTEST_SOCTRIM3_0_TRIM3_0_MASK          (0xFFFFFFFFU)
#define FMUTEST_SOCTRIM3_0_TRIM3_0_SHIFT         (0U)
/*! TRIM3_0 - TRIM3_0 */
#define FMUTEST_SOCTRIM3_0_TRIM3_0(x)            (((uint32_t)(((uint32_t)(x)) << FMUTEST_SOCTRIM3_0_TRIM3_0_SHIFT)) & FMUTEST_SOCTRIM3_0_TRIM3_0_MASK)
/*! @} */

/*! @name SOCTRIM3_1 - SoC Trim Phrase 3 Word 1 Register */
/*! @{ */

#define FMUTEST_SOCTRIM3_1_TRIM3_1_MASK          (0xFFFFFFFFU)
#define FMUTEST_SOCTRIM3_1_TRIM3_1_SHIFT         (0U)
/*! TRIM3_1 - TRIM3_1 */
#define FMUTEST_SOCTRIM3_1_TRIM3_1(x)            (((uint32_t)(((uint32_t)(x)) << FMUTEST_SOCTRIM3_1_TRIM3_1_SHIFT)) & FMUTEST_SOCTRIM3_1_TRIM3_1_MASK)
/*! @} */

/*! @name SOCTRIM3_2 - SoC Trim Phrase 3 Word 2 Register */
/*! @{ */

#define FMUTEST_SOCTRIM3_2_TRIM3_2_MASK          (0xFFFFFFFFU)
#define FMUTEST_SOCTRIM3_2_TRIM3_2_SHIFT         (0U)
/*! TRIM3_2 - TRIM3_2 */
#define FMUTEST_SOCTRIM3_2_TRIM3_2(x)            (((uint32_t)(((uint32_t)(x)) << FMUTEST_SOCTRIM3_2_TRIM3_2_SHIFT)) & FMUTEST_SOCTRIM3_2_TRIM3_2_MASK)
/*! @} */

/*! @name SOCTRIM3_3 - SoC Trim Phrase 3 Word 3 Register */
/*! @{ */

#define FMUTEST_SOCTRIM3_3_TRIM3_3_MASK          (0xFFFFFFFFU)
#define FMUTEST_SOCTRIM3_3_TRIM3_3_SHIFT         (0U)
/*! TRIM3_3 - TRIM3_3 */
#define FMUTEST_SOCTRIM3_3_TRIM3_3(x)            (((uint32_t)(((uint32_t)(x)) << FMUTEST_SOCTRIM3_3_TRIM3_3_SHIFT)) & FMUTEST_SOCTRIM3_3_TRIM3_3_MASK)
/*! @} */

/*! @name SOCTRIM4_0 - SoC Trim Phrase 4 Word 0 Register */
/*! @{ */

#define FMUTEST_SOCTRIM4_0_TRIM4_0_MASK          (0xFFFFFFFFU)
#define FMUTEST_SOCTRIM4_0_TRIM4_0_SHIFT         (0U)
/*! TRIM4_0 - TRIM4_0 */
#define FMUTEST_SOCTRIM4_0_TRIM4_0(x)            (((uint32_t)(((uint32_t)(x)) << FMUTEST_SOCTRIM4_0_TRIM4_0_SHIFT)) & FMUTEST_SOCTRIM4_0_TRIM4_0_MASK)
/*! @} */

/*! @name SOCTRIM4_1 - SoC Trim Phrase 4 Word 1 Register */
/*! @{ */

#define FMUTEST_SOCTRIM4_1_TRIM4_1_MASK          (0xFFFFFFFFU)
#define FMUTEST_SOCTRIM4_1_TRIM4_1_SHIFT         (0U)
/*! TRIM4_1 - TRIM4_1 */
#define FMUTEST_SOCTRIM4_1_TRIM4_1(x)            (((uint32_t)(((uint32_t)(x)) << FMUTEST_SOCTRIM4_1_TRIM4_1_SHIFT)) & FMUTEST_SOCTRIM4_1_TRIM4_1_MASK)
/*! @} */

/*! @name SOCTRIM4_2 - SoC Trim Phrase 4 Word 2 Register */
/*! @{ */

#define FMUTEST_SOCTRIM4_2_TRIM4_2_MASK          (0xFFFFFFFFU)
#define FMUTEST_SOCTRIM4_2_TRIM4_2_SHIFT         (0U)
/*! TRIM4_2 - TRIM4_2 */
#define FMUTEST_SOCTRIM4_2_TRIM4_2(x)            (((uint32_t)(((uint32_t)(x)) << FMUTEST_SOCTRIM4_2_TRIM4_2_SHIFT)) & FMUTEST_SOCTRIM4_2_TRIM4_2_MASK)
/*! @} */

/*! @name SOCTRIM4_3 - SoC Trim Phrase 4 Word 3 Register */
/*! @{ */

#define FMUTEST_SOCTRIM4_3_TRIM4_3_MASK          (0xFFFFFFFFU)
#define FMUTEST_SOCTRIM4_3_TRIM4_3_SHIFT         (0U)
/*! TRIM4_3 - TRIM4_3 */
#define FMUTEST_SOCTRIM4_3_TRIM4_3(x)            (((uint32_t)(((uint32_t)(x)) << FMUTEST_SOCTRIM4_3_TRIM4_3_SHIFT)) & FMUTEST_SOCTRIM4_3_TRIM4_3_MASK)
/*! @} */

/*! @name SOCTRIM5_0 - SoC Trim Phrase 5 Word 0 Register */
/*! @{ */

#define FMUTEST_SOCTRIM5_0_TRIM5_0_MASK          (0xFFFFFFFFU)
#define FMUTEST_SOCTRIM5_0_TRIM5_0_SHIFT         (0U)
/*! TRIM5_0 - TRIM5_0 */
#define FMUTEST_SOCTRIM5_0_TRIM5_0(x)            (((uint32_t)(((uint32_t)(x)) << FMUTEST_SOCTRIM5_0_TRIM5_0_SHIFT)) & FMUTEST_SOCTRIM5_0_TRIM5_0_MASK)
/*! @} */

/*! @name SOCTRIM5_1 - SoC Trim Phrase 5 Word 1 Register */
/*! @{ */

#define FMUTEST_SOCTRIM5_1_TRIM5_1_MASK          (0xFFFFFFFFU)
#define FMUTEST_SOCTRIM5_1_TRIM5_1_SHIFT         (0U)
/*! TRIM5_1 - TRIM5_1 */
#define FMUTEST_SOCTRIM5_1_TRIM5_1(x)            (((uint32_t)(((uint32_t)(x)) << FMUTEST_SOCTRIM5_1_TRIM5_1_SHIFT)) & FMUTEST_SOCTRIM5_1_TRIM5_1_MASK)
/*! @} */

/*! @name SOCTRIM5_2 - SoC Trim Phrase 5 Word 2 Register */
/*! @{ */

#define FMUTEST_SOCTRIM5_2_TRIM5_2_MASK          (0xFFFFFFFFU)
#define FMUTEST_SOCTRIM5_2_TRIM5_2_SHIFT         (0U)
/*! TRIM5_2 - TRIM5_2 */
#define FMUTEST_SOCTRIM5_2_TRIM5_2(x)            (((uint32_t)(((uint32_t)(x)) << FMUTEST_SOCTRIM5_2_TRIM5_2_SHIFT)) & FMUTEST_SOCTRIM5_2_TRIM5_2_MASK)
/*! @} */

/*! @name SOCTRIM5_3 - SoC Trim Phrase 5 Word 3 Register */
/*! @{ */

#define FMUTEST_SOCTRIM5_3_TRIM5_3_MASK          (0xFFFFFFFFU)
#define FMUTEST_SOCTRIM5_3_TRIM5_3_SHIFT         (0U)
/*! TRIM5_3 - TRIM5_3 */
#define FMUTEST_SOCTRIM5_3_TRIM5_3(x)            (((uint32_t)(((uint32_t)(x)) << FMUTEST_SOCTRIM5_3_TRIM5_3_SHIFT)) & FMUTEST_SOCTRIM5_3_TRIM5_3_MASK)
/*! @} */

/*! @name SOCTRIM6_0 - SoC Trim Phrase 6 Word 0 Register */
/*! @{ */

#define FMUTEST_SOCTRIM6_0_TRIM6_0_MASK          (0xFFFFFFFFU)
#define FMUTEST_SOCTRIM6_0_TRIM6_0_SHIFT         (0U)
/*! TRIM6_0 - TRIM6_0 */
#define FMUTEST_SOCTRIM6_0_TRIM6_0(x)            (((uint32_t)(((uint32_t)(x)) << FMUTEST_SOCTRIM6_0_TRIM6_0_SHIFT)) & FMUTEST_SOCTRIM6_0_TRIM6_0_MASK)
/*! @} */

/*! @name SOCTRIM6_1 - SoC Trim Phrase 6 Word 1 Register */
/*! @{ */

#define FMUTEST_SOCTRIM6_1_TRIM6_1_MASK          (0xFFFFFFFFU)
#define FMUTEST_SOCTRIM6_1_TRIM6_1_SHIFT         (0U)
/*! TRIM6_1 - TRIM6_1 */
#define FMUTEST_SOCTRIM6_1_TRIM6_1(x)            (((uint32_t)(((uint32_t)(x)) << FMUTEST_SOCTRIM6_1_TRIM6_1_SHIFT)) & FMUTEST_SOCTRIM6_1_TRIM6_1_MASK)
/*! @} */

/*! @name SOCTRIM6_2 - SoC Trim Phrase 6 Word 2 Register */
/*! @{ */

#define FMUTEST_SOCTRIM6_2_TRIM6_2_MASK          (0xFFFFFFFFU)
#define FMUTEST_SOCTRIM6_2_TRIM6_2_SHIFT         (0U)
/*! TRIM6_2 - TRIM6_2 */
#define FMUTEST_SOCTRIM6_2_TRIM6_2(x)            (((uint32_t)(((uint32_t)(x)) << FMUTEST_SOCTRIM6_2_TRIM6_2_SHIFT)) & FMUTEST_SOCTRIM6_2_TRIM6_2_MASK)
/*! @} */

/*! @name SOCTRIM6_3 - SoC Trim Phrase 6 Word 3 Register */
/*! @{ */

#define FMUTEST_SOCTRIM6_3_TRIM6_3_MASK          (0xFFFFFFFFU)
#define FMUTEST_SOCTRIM6_3_TRIM6_3_SHIFT         (0U)
/*! TRIM6_3 - TRIM6_3 */
#define FMUTEST_SOCTRIM6_3_TRIM6_3(x)            (((uint32_t)(((uint32_t)(x)) << FMUTEST_SOCTRIM6_3_TRIM6_3_SHIFT)) & FMUTEST_SOCTRIM6_3_TRIM6_3_MASK)
/*! @} */

/*! @name SOCTRIM7_0 - SoC Trim Phrase 7 Word 0 Register */
/*! @{ */

#define FMUTEST_SOCTRIM7_0_TRIM7_0_MASK          (0xFFFFFFFFU)
#define FMUTEST_SOCTRIM7_0_TRIM7_0_SHIFT         (0U)
/*! TRIM7_0 - TRIM7_0 */
#define FMUTEST_SOCTRIM7_0_TRIM7_0(x)            (((uint32_t)(((uint32_t)(x)) << FMUTEST_SOCTRIM7_0_TRIM7_0_SHIFT)) & FMUTEST_SOCTRIM7_0_TRIM7_0_MASK)
/*! @} */

/*! @name SOCTRIM7_1 - SoC Trim Phrase 7 Word 1 Register */
/*! @{ */

#define FMUTEST_SOCTRIM7_1_TRIM7_1_MASK          (0xFFFFFFFFU)
#define FMUTEST_SOCTRIM7_1_TRIM7_1_SHIFT         (0U)
/*! TRIM7_1 - TRIM7_1 */
#define FMUTEST_SOCTRIM7_1_TRIM7_1(x)            (((uint32_t)(((uint32_t)(x)) << FMUTEST_SOCTRIM7_1_TRIM7_1_SHIFT)) & FMUTEST_SOCTRIM7_1_TRIM7_1_MASK)
/*! @} */

/*! @name SOCTRIM7_2 - SoC Trim Phrase 7 Word 2 Register */
/*! @{ */

#define FMUTEST_SOCTRIM7_2_TRIM7_2_MASK          (0xFFFFFFFFU)
#define FMUTEST_SOCTRIM7_2_TRIM7_2_SHIFT         (0U)
/*! TRIM7_2 - TRIM7_2 */
#define FMUTEST_SOCTRIM7_2_TRIM7_2(x)            (((uint32_t)(((uint32_t)(x)) << FMUTEST_SOCTRIM7_2_TRIM7_2_SHIFT)) & FMUTEST_SOCTRIM7_2_TRIM7_2_MASK)
/*! @} */

/*! @name SOCTRIM7_3 - SoC Trim Phrase 7 Word 3 Register */
/*! @{ */

#define FMUTEST_SOCTRIM7_3_TRIM7_3_MASK          (0xFFFFFFFFU)
#define FMUTEST_SOCTRIM7_3_TRIM7_3_SHIFT         (0U)
/*! TRIM7_3 - TRIM7_3 */
#define FMUTEST_SOCTRIM7_3_TRIM7_3(x)            (((uint32_t)(((uint32_t)(x)) << FMUTEST_SOCTRIM7_3_TRIM7_3_SHIFT)) & FMUTEST_SOCTRIM7_3_TRIM7_3_MASK)
/*! @} */

/*! @name R_IP_CONFIG - BIST Configuration Register */
/*! @{ */

#define FMUTEST_R_IP_CONFIG_IPSEL0_MASK          (0x3U)
#define FMUTEST_R_IP_CONFIG_IPSEL0_SHIFT         (0U)
/*! IPSEL0 - Block 0 Select Control
 *  0b00..Unselect block 0
 *  0b01..not used, reserved
 *  0b10..Enable block 0 test, repair off (default)
 *  0b11..Enable block 0 test, repair on
 */
#define FMUTEST_R_IP_CONFIG_IPSEL0(x)            (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_IP_CONFIG_IPSEL0_SHIFT)) & FMUTEST_R_IP_CONFIG_IPSEL0_MASK)

#define FMUTEST_R_IP_CONFIG_IPSEL1_MASK          (0xCU)
#define FMUTEST_R_IP_CONFIG_IPSEL1_SHIFT         (2U)
/*! IPSEL1 - Block 1 Select Control
 *  0b00..Unselect block 1
 *  0b01..not used, reserved
 *  0b10..Enable block 1 test, repair off (default)
 *  0b11..Enable block 1 test, repair on
 */
#define FMUTEST_R_IP_CONFIG_IPSEL1(x)            (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_IP_CONFIG_IPSEL1_SHIFT)) & FMUTEST_R_IP_CONFIG_IPSEL1_MASK)

#define FMUTEST_R_IP_CONFIG_BIST_CDIVL_MASK      (0xFF0U)
#define FMUTEST_R_IP_CONFIG_BIST_CDIVL_SHIFT     (4U)
/*! BIST_CDIVL - Clock Divide Scalar for Long Pulse */
#define FMUTEST_R_IP_CONFIG_BIST_CDIVL(x)        (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_IP_CONFIG_BIST_CDIVL_SHIFT)) & FMUTEST_R_IP_CONFIG_BIST_CDIVL_MASK)

#define FMUTEST_R_IP_CONFIG_CDIVS_MASK           (0x7000U)
#define FMUTEST_R_IP_CONFIG_CDIVS_SHIFT          (12U)
/*! CDIVS - Number of clock cycles to generate short pulse */
#define FMUTEST_R_IP_CONFIG_CDIVS(x)             (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_IP_CONFIG_CDIVS_SHIFT)) & FMUTEST_R_IP_CONFIG_CDIVS_MASK)

#define FMUTEST_R_IP_CONFIG_BIST_TVFY_MASK       (0xF8000U)
#define FMUTEST_R_IP_CONFIG_BIST_TVFY_SHIFT      (15U)
/*! BIST_TVFY - Timer adjust for verify */
#define FMUTEST_R_IP_CONFIG_BIST_TVFY(x)         (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_IP_CONFIG_BIST_TVFY_SHIFT)) & FMUTEST_R_IP_CONFIG_BIST_TVFY_MASK)

#define FMUTEST_R_IP_CONFIG_TSTCTL_MASK          (0x300000U)
#define FMUTEST_R_IP_CONFIG_TSTCTL_SHIFT         (20U)
/*! TSTCTL - BIST self-test control
 *  0b00..Default, disable both BIST self-test and MISR
 *  0b01..Enable BIST self-test mode DOUT from macro will be forced to '0', and disable MISR.
 *  0b10..Enable MISR
 *  0b11..Enable both BIST self-test mode and MISR
 */
#define FMUTEST_R_IP_CONFIG_TSTCTL(x)            (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_IP_CONFIG_TSTCTL_SHIFT)) & FMUTEST_R_IP_CONFIG_TSTCTL_MASK)

#define FMUTEST_R_IP_CONFIG_DBGCTL_MASK          (0x400000U)
#define FMUTEST_R_IP_CONFIG_DBGCTL_SHIFT         (22U)
/*! DBGCTL - Debug feature control
 *  0b0..Default
 *  0b1..Enable debug feature to collect failure address and data.
 */
#define FMUTEST_R_IP_CONFIG_DBGCTL(x)            (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_IP_CONFIG_DBGCTL_SHIFT)) & FMUTEST_R_IP_CONFIG_DBGCTL_MASK)

#define FMUTEST_R_IP_CONFIG_BIST_CLK_SEL_MASK    (0x800000U)
#define FMUTEST_R_IP_CONFIG_BIST_CLK_SEL_SHIFT   (23U)
/*! BIST_CLK_SEL - BIST Clock Select */
#define FMUTEST_R_IP_CONFIG_BIST_CLK_SEL(x)      (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_IP_CONFIG_BIST_CLK_SEL_SHIFT)) & FMUTEST_R_IP_CONFIG_BIST_CLK_SEL_MASK)

#define FMUTEST_R_IP_CONFIG_SMWTST_MASK          (0x3000000U)
#define FMUTEST_R_IP_CONFIG_SMWTST_SHIFT         (24U)
/*! SMWTST - SMWR DOUT Function Control
 *  0b00..Default
 *  0b01..Enable SMWR self-test mode, DOUT from macro will be forced to all 0
 *  0b10..Enable SMWR self-test mode, DOUT from macro will be forced to all 1
 *  0b11..Reserved (unused)
 */
#define FMUTEST_R_IP_CONFIG_SMWTST(x)            (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_IP_CONFIG_SMWTST_SHIFT)) & FMUTEST_R_IP_CONFIG_SMWTST_MASK)

#define FMUTEST_R_IP_CONFIG_ECCEN_MASK           (0x4000000U)
#define FMUTEST_R_IP_CONFIG_ECCEN_SHIFT          (26U)
/*! ECCEN - BIST ECC Control
 *  0b0..Default mode (no ECC encode or decode)
 *  0b1..Enable ECC encode/decode
 */
#define FMUTEST_R_IP_CONFIG_ECCEN(x)             (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_IP_CONFIG_ECCEN_SHIFT)) & FMUTEST_R_IP_CONFIG_ECCEN_MASK)
/*! @} */

/*! @name R_TESTCODE - BIST Test Code Register */
/*! @{ */

#define FMUTEST_R_TESTCODE_TESTCODE_MASK         (0x3FU)
#define FMUTEST_R_TESTCODE_TESTCODE_SHIFT        (0U)
/*! TESTCODE - Used to store test code information before running TMR-RST/TMRSET BIST command */
#define FMUTEST_R_TESTCODE_TESTCODE(x)           (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_TESTCODE_TESTCODE_SHIFT)) & FMUTEST_R_TESTCODE_TESTCODE_MASK)
/*! @} */

/*! @name R_DFT_CTRL - BIST DFT Control Register */
/*! @{ */

#define FMUTEST_R_DFT_CTRL_DFT_XADR_MASK         (0xFU)
#define FMUTEST_R_DFT_CTRL_DFT_XADR_SHIFT        (0U)
/*! DFT_XADR - DFT XADR Pattern
 *  0b0000..XADR fixed, no change at all
 *  0b0001..XADR increased by 1 after row. For READ operation, XADR increases by 1 after reading the last word of
 *          row. For PROG operation, XADR increases by 1 after NVSTR falls.
 *  0b0010..XADR increased for diagonal. For PROG-DIAGONAL operation, XADR is increased to create diagonal pattern.
 *  0b0011..XADR increased by sector. During ERASE operation, XADR increased by number of rows in a sector when NVSTR falls.
 *  0b0100..XADR inversed. XADR is inversed after reading one word or after programming one row when NVSTR falls.
 *  0b0101..XADR increased by 2 after row. For READ operation, XADR is increased by 2 after reading the last word
 *          of a row. For PROG operation, XADR is increased by 2 when NVSTR falls.
 *  0b0110..XADR[0] inversed. XADR[0] is inversed after reading one word or after programming one row when NVSTR falls.
 *  0b0111..XADR increased by 1. For READ operations only, XADR increased by 1 after each read cycle.
 *  0b1000..XADR decreased by 1 after row. For READ operations only, XADR is decreased by 1 after YADR decreases to 0.
 *  0b1001..XADR decreased by 1. For READ operations only, XADR is decreased by 1 after each read cycle.
 */
#define FMUTEST_R_DFT_CTRL_DFT_XADR(x)           (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_DFT_CTRL_DFT_XADR_SHIFT)) & FMUTEST_R_DFT_CTRL_DFT_XADR_MASK)

#define FMUTEST_R_DFT_CTRL_DFT_YADR_MASK         (0xF0U)
#define FMUTEST_R_DFT_CTRL_DFT_YADR_SHIFT        (4U)
/*! DFT_YADR - DFT YADR Pattern
 *  0b0000..YADR fixed, no change at all
 *  0b0001..YADR for ICKBD. For PROG and READ operations, YADR changed to generate inverse checkerboard pattern.
 *  0b0010..YADR for CKBD. For PROG and READ operations, YADR changed to generate checkerboard pattern.
 *  0b0011..YADR increased by 1. For READ operations, YADR increased by 1 after each read cycle. For PROG
 *          operations, YADR increased by 1 after YE falls.
 *  0b0100..YADR increased for diagonal. For PROG-DIAGONAL operation, YADR is increased to create diagonal pattern.
 *  0b0101..YADR inversed. YADR is inversed after reading one word or after programming one word when YE falls.
 *  0b0110..YADR[0] inversed. YADR[0] is inversed after reading one word or after programming one word when YE falls.
 *  0b0111..YADR increased by 1 after last row. For READ operations only, YADR is increased by 1 after XADR reaches last row.
 *  0b1000..YADR decreased by 1. For READ operations only, YADR is decreased by 1 after each read cycle.
 *  0b1001..YADR decreased by 1 after first row. For READ operations only, YADR is decreased by 1 after XADR decreases to 0.
 */
#define FMUTEST_R_DFT_CTRL_DFT_YADR(x)           (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_DFT_CTRL_DFT_YADR_SHIFT)) & FMUTEST_R_DFT_CTRL_DFT_YADR_MASK)

#define FMUTEST_R_DFT_CTRL_DFT_DATA_MASK         (0xF00U)
#define FMUTEST_R_DFT_CTRL_DFT_DATA_SHIFT        (8U)
/*! DFT_DATA - DFT Data Pattern
 *  0b0000..CKBD pattern. For READ operations only, compare DOUT with checkerboard data pattern for each read cycle.
 *  0b0001..ICKBD pattern. For READ operations only, compare DOUT with inverse checkerboard data pattern for each read cycle.
 *  0b0010..Diagonal pattern. Used for READ operations only, compare DOUT to diagonal pattern.
 *  0b0011..Fixed data pattern. For READ operations, comparison to DOUT for selected groups; refer to
 *          R_ADR_CTRL[GRPSEL] for modules with multiple groups.
 *  0b0100..Random data pattern which will be generated based on the initial seed set in R_DATA; for READ
 *          operations, used for DOUT comparison of selected groups. For PROG operations, used to control DIN of selected
 *          groups.
 *  0b0101..DOUT based pattern. For READ operations only, DOUT of selected group will be latched in R_DATA. If
 *          more than one group is selected in R_ADR_CTRL[GRPSEL], the group with the lower index will be latched.
 *  0b0110..R_DATA based pattern. For READ operations, expected DOUT value of selected groups equals to R_DATA
 *          when XADR[0]==YADR[0] or ~R_DATA when XADR[0]!=YADR[0]. For PROG operations, DIN of selected groups equals
 *          R_DATA when XADR[0]==YADR[0] or ~R_DATA when XADR[0]!=YADR[0].
 *  0b0111..SCAN-IO pattern. For READ operations, control expected DOUT value of selected groups to SCAN-IO data
 *          pattern. For PROG operations, control DIN of selected groups to SCAN-IO data pattern.
 *  0b1000..REPAIR set. For PROG operation to IFR1(7,1) and IFR1(7,2), R_REPAIR0_0 and R_REPAIR0_1 or R_REPAIR1_0
 *          and R_REPAIR1_1 will control DIN. For READ operation on IFR1(7,1) and IFR1(7,2), DOUT will be compared
 *          against R_REPAIR0_0 and R_REPAIR0_1 or R_REPAIR1_0 andR_REPAIR1_1. When this option is selected, only
 *          one flash block can be selected.
 *  0b1001..REPAIR load. For READ operation only, DOUT from IFR1(7,1) and IFR1(7,2) is loaded to R_REPAIR0 and R_REPAIR1.
 */
#define FMUTEST_R_DFT_CTRL_DFT_DATA(x)           (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_DFT_CTRL_DFT_DATA_SHIFT)) & FMUTEST_R_DFT_CTRL_DFT_DATA_MASK)

#define FMUTEST_R_DFT_CTRL_CMP_MASK_MASK         (0x3000U)
#define FMUTEST_R_DFT_CTRL_CMP_MASK_SHIFT        (12U)
/*! CMP_MASK - Data Compare Mask
 *  0b00..Expected data is compared to DOUT
 *  0b01..Expected data (only 0s are considered) are compared to DOUT
 *  0b10..Expected data (only 1s are considered) are compared to DOUT
 */
#define FMUTEST_R_DFT_CTRL_CMP_MASK(x)           (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_DFT_CTRL_CMP_MASK_SHIFT)) & FMUTEST_R_DFT_CTRL_CMP_MASK_MASK)

#define FMUTEST_R_DFT_CTRL_DFT_DATA_SRC_MASK     (0x4000U)
#define FMUTEST_R_DFT_CTRL_DFT_DATA_SRC_SHIFT    (14U)
/*! DFT_DATA_SRC - DFT Data Source
 *  0b0..{R_DATA_CTRL0,R_DATA_CTRL_EX[2:0],R_DATA_CTRL0,R_DATA_CTRL_EX[2:0],R_DATA_CTRL0,R_DATA_CTRL_EX[2:0],R_DATA_CTRL0} is used
 *  0b1..{R_DATA_CTRL3,R_DATA_CTRL2_EX[2:0],R_DATA_CTRL2,R_DATA_CTRL1_EX[2:0],R_DATA_CTRL1,R_DATA_CTRL_EX[2:0],R_DATA_CTRL0} is used
 */
#define FMUTEST_R_DFT_CTRL_DFT_DATA_SRC(x)       (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_DFT_CTRL_DFT_DATA_SRC_SHIFT)) & FMUTEST_R_DFT_CTRL_DFT_DATA_SRC_MASK)
/*! @} */

/*! @name R_ADR_CTRL - BIST Address Control Register */
/*! @{ */

#define FMUTEST_R_ADR_CTRL_GRPSEL_MASK           (0xFU)
#define FMUTEST_R_ADR_CTRL_GRPSEL_SHIFT          (0U)
/*! GRPSEL - Data Group Select
 *  0b0000..Select no data
 *  0b0001..Select data slice [34:0]
 *  0b0010..Select data slice [69:35]
 *  0b0100..Select data slice [104:70]
 *  0b1000..Select data slice [136:105]
 *  0b1111..Select data [136:0]
 */
#define FMUTEST_R_ADR_CTRL_GRPSEL(x)             (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_ADR_CTRL_GRPSEL_SHIFT)) & FMUTEST_R_ADR_CTRL_GRPSEL_MASK)

#define FMUTEST_R_ADR_CTRL_XADR_MASK             (0xFFF0U)
#define FMUTEST_R_ADR_CTRL_XADR_SHIFT            (4U)
/*! XADR - BIST XADR */
#define FMUTEST_R_ADR_CTRL_XADR(x)               (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_ADR_CTRL_XADR_SHIFT)) & FMUTEST_R_ADR_CTRL_XADR_MASK)

#define FMUTEST_R_ADR_CTRL_YADR_MASK             (0x1F0000U)
#define FMUTEST_R_ADR_CTRL_YADR_SHIFT            (16U)
/*! YADR - BIST YADR */
#define FMUTEST_R_ADR_CTRL_YADR(x)               (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_ADR_CTRL_YADR_SHIFT)) & FMUTEST_R_ADR_CTRL_YADR_MASK)

#define FMUTEST_R_ADR_CTRL_PROG_ATTR_MASK        (0xE00000U)
#define FMUTEST_R_ADR_CTRL_PROG_ATTR_SHIFT       (21U)
/*! PROG_ATTR - Program Attribute
 *  0b000..One YE pulse will program one data slice group
 *  0b001..One YE pulse will program two data slice groups
 *  0b010..One YE pulse will program three data slice groups (reserved)
 *  0b011..One YE pulse will program four data slice groups
 *  0b100..One YE pulse will program five data slice groups (reserved)
 *  0b101..One YE pulse will program six data slice groups (reserved)
 *  0b110..One YE pulse will program seven data slice groups (reserved)
 *  0b111..One YE pulse will program eight data slice groups (reserved)
 */
#define FMUTEST_R_ADR_CTRL_PROG_ATTR(x)          (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_ADR_CTRL_PROG_ATTR_SHIFT)) & FMUTEST_R_ADR_CTRL_PROG_ATTR_MASK)
/*! @} */

/*! @name R_DATA_CTRL0 - BIST Data Control 0 Register */
/*! @{ */

#define FMUTEST_R_DATA_CTRL0_DATA0_MASK          (0xFFFFFFFFU)
#define FMUTEST_R_DATA_CTRL0_DATA0_SHIFT         (0U)
/*! DATA0 - BIST Data 0 Low */
#define FMUTEST_R_DATA_CTRL0_DATA0(x)            (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_DATA_CTRL0_DATA0_SHIFT)) & FMUTEST_R_DATA_CTRL0_DATA0_MASK)
/*! @} */

/*! @name R_PIN_CTRL - BIST Pin Control Register */
/*! @{ */

#define FMUTEST_R_PIN_CTRL_MAS1_MASK             (0x1U)
#define FMUTEST_R_PIN_CTRL_MAS1_SHIFT            (0U)
/*! MAS1 - Mass Erase */
#define FMUTEST_R_PIN_CTRL_MAS1(x)               (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_PIN_CTRL_MAS1_SHIFT)) & FMUTEST_R_PIN_CTRL_MAS1_MASK)

#define FMUTEST_R_PIN_CTRL_IFREN_MASK            (0x2U)
#define FMUTEST_R_PIN_CTRL_IFREN_SHIFT           (1U)
/*! IFREN - IFR Enable */
#define FMUTEST_R_PIN_CTRL_IFREN(x)              (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_PIN_CTRL_IFREN_SHIFT)) & FMUTEST_R_PIN_CTRL_IFREN_MASK)

#define FMUTEST_R_PIN_CTRL_IFREN1_MASK           (0x4U)
#define FMUTEST_R_PIN_CTRL_IFREN1_SHIFT          (2U)
/*! IFREN1 - IFR1 Enable */
#define FMUTEST_R_PIN_CTRL_IFREN1(x)             (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_PIN_CTRL_IFREN1_SHIFT)) & FMUTEST_R_PIN_CTRL_IFREN1_MASK)

#define FMUTEST_R_PIN_CTRL_REDEN_MASK            (0x8U)
#define FMUTEST_R_PIN_CTRL_REDEN_SHIFT           (3U)
/*! REDEN - Redundancy Block Enable */
#define FMUTEST_R_PIN_CTRL_REDEN(x)              (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_PIN_CTRL_REDEN_SHIFT)) & FMUTEST_R_PIN_CTRL_REDEN_MASK)

#define FMUTEST_R_PIN_CTRL_LVE_MASK              (0x10U)
#define FMUTEST_R_PIN_CTRL_LVE_SHIFT             (4U)
/*! LVE - Low Voltage Enable */
#define FMUTEST_R_PIN_CTRL_LVE(x)                (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_PIN_CTRL_LVE_SHIFT)) & FMUTEST_R_PIN_CTRL_LVE_MASK)

#define FMUTEST_R_PIN_CTRL_PV_MASK               (0x20U)
#define FMUTEST_R_PIN_CTRL_PV_SHIFT              (5U)
/*! PV - Program Verify Enable */
#define FMUTEST_R_PIN_CTRL_PV(x)                 (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_PIN_CTRL_PV_SHIFT)) & FMUTEST_R_PIN_CTRL_PV_MASK)

#define FMUTEST_R_PIN_CTRL_EV_MASK               (0x40U)
#define FMUTEST_R_PIN_CTRL_EV_SHIFT              (6U)
/*! EV - Erase Verify Enable */
#define FMUTEST_R_PIN_CTRL_EV(x)                 (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_PIN_CTRL_EV_SHIFT)) & FMUTEST_R_PIN_CTRL_EV_MASK)

#define FMUTEST_R_PIN_CTRL_WIPGM_MASK            (0x180U)
#define FMUTEST_R_PIN_CTRL_WIPGM_SHIFT           (7U)
/*! WIPGM - Program Current */
#define FMUTEST_R_PIN_CTRL_WIPGM(x)              (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_PIN_CTRL_WIPGM_SHIFT)) & FMUTEST_R_PIN_CTRL_WIPGM_MASK)

#define FMUTEST_R_PIN_CTRL_WHV_MASK              (0x1E00U)
#define FMUTEST_R_PIN_CTRL_WHV_SHIFT             (9U)
/*! WHV - High Voltage Level */
#define FMUTEST_R_PIN_CTRL_WHV(x)                (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_PIN_CTRL_WHV_SHIFT)) & FMUTEST_R_PIN_CTRL_WHV_MASK)

#define FMUTEST_R_PIN_CTRL_WMV_MASK              (0xE000U)
#define FMUTEST_R_PIN_CTRL_WMV_SHIFT             (13U)
/*! WMV - Medium Voltage Level */
#define FMUTEST_R_PIN_CTRL_WMV(x)                (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_PIN_CTRL_WMV_SHIFT)) & FMUTEST_R_PIN_CTRL_WMV_MASK)

#define FMUTEST_R_PIN_CTRL_XE_MASK               (0x10000U)
#define FMUTEST_R_PIN_CTRL_XE_SHIFT              (16U)
/*! XE - X Address Enable */
#define FMUTEST_R_PIN_CTRL_XE(x)                 (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_PIN_CTRL_XE_SHIFT)) & FMUTEST_R_PIN_CTRL_XE_MASK)

#define FMUTEST_R_PIN_CTRL_YE_MASK               (0x20000U)
#define FMUTEST_R_PIN_CTRL_YE_SHIFT              (17U)
/*! YE - Y Address Enable */
#define FMUTEST_R_PIN_CTRL_YE(x)                 (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_PIN_CTRL_YE_SHIFT)) & FMUTEST_R_PIN_CTRL_YE_MASK)

#define FMUTEST_R_PIN_CTRL_SE_MASK               (0x40000U)
#define FMUTEST_R_PIN_CTRL_SE_SHIFT              (18U)
/*! SE - Sense Amp Enable */
#define FMUTEST_R_PIN_CTRL_SE(x)                 (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_PIN_CTRL_SE_SHIFT)) & FMUTEST_R_PIN_CTRL_SE_MASK)

#define FMUTEST_R_PIN_CTRL_ERASE_MASK            (0x80000U)
#define FMUTEST_R_PIN_CTRL_ERASE_SHIFT           (19U)
/*! ERASE - Erase Mode */
#define FMUTEST_R_PIN_CTRL_ERASE(x)              (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_PIN_CTRL_ERASE_SHIFT)) & FMUTEST_R_PIN_CTRL_ERASE_MASK)

#define FMUTEST_R_PIN_CTRL_PROG_MASK             (0x100000U)
#define FMUTEST_R_PIN_CTRL_PROG_SHIFT            (20U)
/*! PROG - Program Mode */
#define FMUTEST_R_PIN_CTRL_PROG(x)               (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_PIN_CTRL_PROG_SHIFT)) & FMUTEST_R_PIN_CTRL_PROG_MASK)

#define FMUTEST_R_PIN_CTRL_NVSTR_MASK            (0x200000U)
#define FMUTEST_R_PIN_CTRL_NVSTR_SHIFT           (21U)
/*! NVSTR - NVM Store */
#define FMUTEST_R_PIN_CTRL_NVSTR(x)              (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_PIN_CTRL_NVSTR_SHIFT)) & FMUTEST_R_PIN_CTRL_NVSTR_MASK)

#define FMUTEST_R_PIN_CTRL_SLM_MASK              (0x400000U)
#define FMUTEST_R_PIN_CTRL_SLM_SHIFT             (22U)
/*! SLM - Sleep Mode Enable */
#define FMUTEST_R_PIN_CTRL_SLM(x)                (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_PIN_CTRL_SLM_SHIFT)) & FMUTEST_R_PIN_CTRL_SLM_MASK)

#define FMUTEST_R_PIN_CTRL_RECALL_MASK           (0x800000U)
#define FMUTEST_R_PIN_CTRL_RECALL_SHIFT          (23U)
/*! RECALL - Recall Trim Code */
#define FMUTEST_R_PIN_CTRL_RECALL(x)             (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_PIN_CTRL_RECALL_SHIFT)) & FMUTEST_R_PIN_CTRL_RECALL_MASK)

#define FMUTEST_R_PIN_CTRL_HEM_MASK              (0x1000000U)
#define FMUTEST_R_PIN_CTRL_HEM_SHIFT             (24U)
/*! HEM - HEM Control */
#define FMUTEST_R_PIN_CTRL_HEM(x)                (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_PIN_CTRL_HEM_SHIFT)) & FMUTEST_R_PIN_CTRL_HEM_MASK)
/*! @} */

/*! @name R_CNT_LOOP_CTRL - BIST Loop Count Control Register */
/*! @{ */

#define FMUTEST_R_CNT_LOOP_CTRL_LOOPCNT_MASK     (0xFFFU)
#define FMUTEST_R_CNT_LOOP_CTRL_LOOPCNT_SHIFT    (0U)
/*! LOOPCNT - Loop Count Control */
#define FMUTEST_R_CNT_LOOP_CTRL_LOOPCNT(x)       (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_CNT_LOOP_CTRL_LOOPCNT_SHIFT)) & FMUTEST_R_CNT_LOOP_CTRL_LOOPCNT_MASK)

#define FMUTEST_R_CNT_LOOP_CTRL_LOOPOPT_MASK     (0x7000U)
#define FMUTEST_R_CNT_LOOP_CTRL_LOOPOPT_SHIFT    (12U)
/*! LOOPOPT - Loop Option
 *  0b000..Loop is disabled; selected BIST operation is run once
 *  0b001..Loop is enabled; XADR increments by 1 XADR increments by 1 for each new loop. Stops when total loop count meets LOOPCNT+1.
 *  0b010..Loop is enabled; YADR increments by 1 YADR increments by 1 for each new loop. Stops when total loop count meets LOOPCNT+1.
 *  0b011..Loop is enabled; XADR increments by 2 XADR increments by 2 for each new loop. Stops when total loop count meets LOOPCNT+1.
 *  0b100..Loop is enabled; XADR increments by sector XADR increments by 16 for each new loop. Stops when total loop count meets LOOPCNT+1.
 */
#define FMUTEST_R_CNT_LOOP_CTRL_LOOPOPT(x)       (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_CNT_LOOP_CTRL_LOOPOPT_SHIFT)) & FMUTEST_R_CNT_LOOP_CTRL_LOOPOPT_MASK)

#define FMUTEST_R_CNT_LOOP_CTRL_LOOPUNIT_MASK    (0x38000U)
#define FMUTEST_R_CNT_LOOP_CTRL_LOOPUNIT_SHIFT   (15U)
/*! LOOPUNIT - Loop Time Unit
 *  0b000..Clock cycles
 *  0b001..0.5 usec
 *  0b010..1 usec
 *  0b011..10 usec
 *  0b100..100 usec
 *  0b101..1 msec
 *  0b110..10 msec
 *  0b111..100 msec
 */
#define FMUTEST_R_CNT_LOOP_CTRL_LOOPUNIT(x)      (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_CNT_LOOP_CTRL_LOOPUNIT_SHIFT)) & FMUTEST_R_CNT_LOOP_CTRL_LOOPUNIT_MASK)

#define FMUTEST_R_CNT_LOOP_CTRL_LOOPDLY_MASK     (0x1FC0000U)
#define FMUTEST_R_CNT_LOOP_CTRL_LOOPDLY_SHIFT    (18U)
/*! LOOPDLY - Loop Time Delay Scalar */
#define FMUTEST_R_CNT_LOOP_CTRL_LOOPDLY(x)       (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_CNT_LOOP_CTRL_LOOPDLY_SHIFT)) & FMUTEST_R_CNT_LOOP_CTRL_LOOPDLY_MASK)
/*! @} */

/*! @name R_TIMER_CTRL - BIST Timer Control Register */
/*! @{ */

#define FMUTEST_R_TIMER_CTRL_TNVSUNIT_MASK       (0x7U)
#define FMUTEST_R_TIMER_CTRL_TNVSUNIT_SHIFT      (0U)
/*! TNVSUNIT - Tnvs Time Unit
 *  0b000..Clock cycles
 *  0b001..0.5 usec
 *  0b010..1 usec
 *  0b011..10 usec
 *  0b100..100 usec
 *  0b101..1 msec
 *  0b110..10 msec
 *  0b111..100 msec
 */
#define FMUTEST_R_TIMER_CTRL_TNVSUNIT(x)         (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_TIMER_CTRL_TNVSUNIT_SHIFT)) & FMUTEST_R_TIMER_CTRL_TNVSUNIT_MASK)

#define FMUTEST_R_TIMER_CTRL_TNVSDLY_MASK        (0x78U)
#define FMUTEST_R_TIMER_CTRL_TNVSDLY_SHIFT       (3U)
/*! TNVSDLY - Tnvs Time Delay Scalar */
#define FMUTEST_R_TIMER_CTRL_TNVSDLY(x)          (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_TIMER_CTRL_TNVSDLY_SHIFT)) & FMUTEST_R_TIMER_CTRL_TNVSDLY_MASK)

#define FMUTEST_R_TIMER_CTRL_TNVHUNIT_MASK       (0x380U)
#define FMUTEST_R_TIMER_CTRL_TNVHUNIT_SHIFT      (7U)
/*! TNVHUNIT - Tnvh Time Unit
 *  0b000..Clock cycles
 *  0b001..0.5 usec
 *  0b010..1 usec
 *  0b011..10 usec
 *  0b100..100 usec
 *  0b101..1 msec
 *  0b110..10 msec
 *  0b111..100 msec
 */
#define FMUTEST_R_TIMER_CTRL_TNVHUNIT(x)         (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_TIMER_CTRL_TNVHUNIT_SHIFT)) & FMUTEST_R_TIMER_CTRL_TNVHUNIT_MASK)

#define FMUTEST_R_TIMER_CTRL_TNVHDLY_MASK        (0x3C00U)
#define FMUTEST_R_TIMER_CTRL_TNVHDLY_SHIFT       (10U)
/*! TNVHDLY - Tnvh Time Delay Scalar */
#define FMUTEST_R_TIMER_CTRL_TNVHDLY(x)          (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_TIMER_CTRL_TNVHDLY_SHIFT)) & FMUTEST_R_TIMER_CTRL_TNVHDLY_MASK)

#define FMUTEST_R_TIMER_CTRL_TPGSUNIT_MASK       (0x1C000U)
#define FMUTEST_R_TIMER_CTRL_TPGSUNIT_SHIFT      (14U)
/*! TPGSUNIT - Tpgs Time Unit
 *  0b000..Clock cycles
 *  0b001..0.5 usec
 *  0b010..1 usec
 *  0b011..10 usec
 *  0b100..100 usec
 *  0b101..1 msec
 *  0b110..10 msec
 *  0b111..100 msec
 */
#define FMUTEST_R_TIMER_CTRL_TPGSUNIT(x)         (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_TIMER_CTRL_TPGSUNIT_SHIFT)) & FMUTEST_R_TIMER_CTRL_TPGSUNIT_MASK)

#define FMUTEST_R_TIMER_CTRL_TPGSDLY_MASK        (0x1E0000U)
#define FMUTEST_R_TIMER_CTRL_TPGSDLY_SHIFT       (17U)
/*! TPGSDLY - Tpgs Time Delay Scalar */
#define FMUTEST_R_TIMER_CTRL_TPGSDLY(x)          (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_TIMER_CTRL_TPGSDLY_SHIFT)) & FMUTEST_R_TIMER_CTRL_TPGSDLY_MASK)

#define FMUTEST_R_TIMER_CTRL_TRCVUNIT_MASK       (0xE00000U)
#define FMUTEST_R_TIMER_CTRL_TRCVUNIT_SHIFT      (21U)
/*! TRCVUNIT - Trcv Time Unit
 *  0b000..Clock cycles
 *  0b001..0.5 usec
 *  0b010..1 usec
 *  0b011..10 usec
 *  0b100..100 usec
 *  0b101..1 msec
 *  0b110..10 msec
 *  0b111..100 msec
 */
#define FMUTEST_R_TIMER_CTRL_TRCVUNIT(x)         (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_TIMER_CTRL_TRCVUNIT_SHIFT)) & FMUTEST_R_TIMER_CTRL_TRCVUNIT_MASK)

#define FMUTEST_R_TIMER_CTRL_TRCVDLY_MASK        (0xF000000U)
#define FMUTEST_R_TIMER_CTRL_TRCVDLY_SHIFT       (24U)
/*! TRCVDLY - Trcv Time Delay Scalar */
#define FMUTEST_R_TIMER_CTRL_TRCVDLY(x)          (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_TIMER_CTRL_TRCVDLY_SHIFT)) & FMUTEST_R_TIMER_CTRL_TRCVDLY_MASK)

#define FMUTEST_R_TIMER_CTRL_TLVSUNIT_MASK       (0x70000000U)
#define FMUTEST_R_TIMER_CTRL_TLVSUNIT_SHIFT      (28U)
/*! TLVSUNIT - Tlvs Time Unit
 *  0b000..Clock cycles
 *  0b001..0.5 usec
 *  0b010..1 usec
 *  0b011..10 usec
 *  0b100..100 usec
 *  0b101..1 msec
 *  0b110..10 msec
 *  0b111..100 msec
 */
#define FMUTEST_R_TIMER_CTRL_TLVSUNIT(x)         (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_TIMER_CTRL_TLVSUNIT_SHIFT)) & FMUTEST_R_TIMER_CTRL_TLVSUNIT_MASK)

#define FMUTEST_R_TIMER_CTRL_TLVSDLY_L_MASK      (0x80000000U)
#define FMUTEST_R_TIMER_CTRL_TLVSDLY_L_SHIFT     (31U)
/*! TLVSDLY_L - Tlvs Time Delay Scalar Low */
#define FMUTEST_R_TIMER_CTRL_TLVSDLY_L(x)        (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_TIMER_CTRL_TLVSDLY_L_SHIFT)) & FMUTEST_R_TIMER_CTRL_TLVSDLY_L_MASK)
/*! @} */

/*! @name R_TEST_CTRL - BIST Test Control Register */
/*! @{ */

#define FMUTEST_R_TEST_CTRL_BUSY_MASK            (0x1U)
#define FMUTEST_R_TEST_CTRL_BUSY_SHIFT           (0U)
/*! BUSY - BIST Busy Status
 *  0b0..BIST is idle
 *  0b1..BIST is busy
 */
#define FMUTEST_R_TEST_CTRL_BUSY(x)              (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_TEST_CTRL_BUSY_SHIFT)) & FMUTEST_R_TEST_CTRL_BUSY_MASK)

#define FMUTEST_R_TEST_CTRL_DEBUG_MASK           (0x2U)
#define FMUTEST_R_TEST_CTRL_DEBUG_SHIFT          (1U)
/*! DEBUG - BIST Debug Status */
#define FMUTEST_R_TEST_CTRL_DEBUG(x)             (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_TEST_CTRL_DEBUG_SHIFT)) & FMUTEST_R_TEST_CTRL_DEBUG_MASK)

#define FMUTEST_R_TEST_CTRL_STATUS0_MASK         (0x4U)
#define FMUTEST_R_TEST_CTRL_STATUS0_SHIFT        (2U)
/*! STATUS0 - BIST Status 0
 *  0b0..BIST test passed on flash block 0
 *  0b1..BIST test failed on flash block 0
 */
#define FMUTEST_R_TEST_CTRL_STATUS0(x)           (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_TEST_CTRL_STATUS0_SHIFT)) & FMUTEST_R_TEST_CTRL_STATUS0_MASK)

#define FMUTEST_R_TEST_CTRL_STATUS1_MASK         (0x8U)
#define FMUTEST_R_TEST_CTRL_STATUS1_SHIFT        (3U)
/*! STATUS1 - BIST status 1
 *  0b0..BIST test passed on flash block 1
 *  0b1..BIST test failed on flash block 1
 */
#define FMUTEST_R_TEST_CTRL_STATUS1(x)           (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_TEST_CTRL_STATUS1_SHIFT)) & FMUTEST_R_TEST_CTRL_STATUS1_MASK)

#define FMUTEST_R_TEST_CTRL_DEBUGRUN_MASK        (0x10U)
#define FMUTEST_R_TEST_CTRL_DEBUGRUN_SHIFT       (4U)
/*! DEBUGRUN - BIST Continue Debug Run */
#define FMUTEST_R_TEST_CTRL_DEBUGRUN(x)          (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_TEST_CTRL_DEBUGRUN_SHIFT)) & FMUTEST_R_TEST_CTRL_DEBUGRUN_MASK)

#define FMUTEST_R_TEST_CTRL_STARTRUN_MASK        (0x20U)
#define FMUTEST_R_TEST_CTRL_STARTRUN_SHIFT       (5U)
/*! STARTRUN - Run New BIST Operation */
#define FMUTEST_R_TEST_CTRL_STARTRUN(x)          (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_TEST_CTRL_STARTRUN_SHIFT)) & FMUTEST_R_TEST_CTRL_STARTRUN_MASK)

#define FMUTEST_R_TEST_CTRL_CMDINDEX_MASK        (0xFFC0U)
#define FMUTEST_R_TEST_CTRL_CMDINDEX_SHIFT       (6U)
/*! CMDINDEX - BIST Command Index (code) */
#define FMUTEST_R_TEST_CTRL_CMDINDEX(x)          (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_TEST_CTRL_CMDINDEX_SHIFT)) & FMUTEST_R_TEST_CTRL_CMDINDEX_MASK)

#define FMUTEST_R_TEST_CTRL_DISABLE_IP1_MASK     (0x10000U)
#define FMUTEST_R_TEST_CTRL_DISABLE_IP1_SHIFT    (16U)
/*! DISABLE_IP1 - BIST Disable IP1 */
#define FMUTEST_R_TEST_CTRL_DISABLE_IP1(x)       (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_TEST_CTRL_DISABLE_IP1_SHIFT)) & FMUTEST_R_TEST_CTRL_DISABLE_IP1_MASK)
/*! @} */

/*! @name R_ABORT_LOOP - BIST Abort Loop Register */
/*! @{ */

#define FMUTEST_R_ABORT_LOOP_ABORT_LOOP_MASK     (0x1U)
#define FMUTEST_R_ABORT_LOOP_ABORT_LOOP_SHIFT    (0U)
/*! ABORT_LOOP - Abort Loop
 *  0b0..No effect
 *  0b1..Abort BIST loop commands and force the loop counter to return to 0x0
 */
#define FMUTEST_R_ABORT_LOOP_ABORT_LOOP(x)       (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_ABORT_LOOP_ABORT_LOOP_SHIFT)) & FMUTEST_R_ABORT_LOOP_ABORT_LOOP_MASK)
/*! @} */

/*! @name R_ADR_QUERY - BIST Address Query Register */
/*! @{ */

#define FMUTEST_R_ADR_QUERY_YADRFAIL_MASK        (0x1FU)
#define FMUTEST_R_ADR_QUERY_YADRFAIL_SHIFT       (0U)
/*! YADRFAIL - Failing YADR */
#define FMUTEST_R_ADR_QUERY_YADRFAIL(x)          (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_ADR_QUERY_YADRFAIL_SHIFT)) & FMUTEST_R_ADR_QUERY_YADRFAIL_MASK)

#define FMUTEST_R_ADR_QUERY_XADRFAIL_MASK        (0x1FFE0U)
#define FMUTEST_R_ADR_QUERY_XADRFAIL_SHIFT       (5U)
/*! XADRFAIL - Failing XADR */
#define FMUTEST_R_ADR_QUERY_XADRFAIL(x)          (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_ADR_QUERY_XADRFAIL_SHIFT)) & FMUTEST_R_ADR_QUERY_XADRFAIL_MASK)
/*! @} */

/*! @name R_DOUT_QUERY0 - BIST DOUT Query 0 Register */
/*! @{ */

#define FMUTEST_R_DOUT_QUERY0_DOUTFAIL_MASK      (0xFFFFFFFFU)
#define FMUTEST_R_DOUT_QUERY0_DOUTFAIL_SHIFT     (0U)
/*! DOUTFAIL - Failing DOUT Low */
#define FMUTEST_R_DOUT_QUERY0_DOUTFAIL(x)        (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_DOUT_QUERY0_DOUTFAIL_SHIFT)) & FMUTEST_R_DOUT_QUERY0_DOUTFAIL_MASK)
/*! @} */

/*! @name R_SMW_QUERY - BIST SMW Query Register */
/*! @{ */

#define FMUTEST_R_SMW_QUERY_SMWLOOP_MASK         (0x3FFU)
#define FMUTEST_R_SMW_QUERY_SMWLOOP_SHIFT        (0U)
/*! SMWLOOP - SMW Total Loop Count */
#define FMUTEST_R_SMW_QUERY_SMWLOOP(x)           (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_SMW_QUERY_SMWLOOP_SHIFT)) & FMUTEST_R_SMW_QUERY_SMWLOOP_MASK)

#define FMUTEST_R_SMW_QUERY_SMWLAST_MASK         (0x7FC00U)
#define FMUTEST_R_SMW_QUERY_SMWLAST_SHIFT        (10U)
/*! SMWLAST - SMW Last Voltage Setting */
#define FMUTEST_R_SMW_QUERY_SMWLAST(x)           (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_SMW_QUERY_SMWLAST_SHIFT)) & FMUTEST_R_SMW_QUERY_SMWLAST_MASK)
/*! @} */

/*! @name R_SMW_SETTING0 - BIST SMW Setting 0 Register */
/*! @{ */

#define FMUTEST_R_SMW_SETTING0_SMWPARM0_MASK     (0x7FFFFFFFU)
#define FMUTEST_R_SMW_SETTING0_SMWPARM0_SHIFT    (0U)
/*! SMWPARM0 - SMW Parameter Set 0 */
#define FMUTEST_R_SMW_SETTING0_SMWPARM0(x)       (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_SMW_SETTING0_SMWPARM0_SHIFT)) & FMUTEST_R_SMW_SETTING0_SMWPARM0_MASK)
/*! @} */

/*! @name R_SMW_SETTING1 - BIST SMW Setting 1 Register */
/*! @{ */

#define FMUTEST_R_SMW_SETTING1_SMWPARM1_MASK     (0xFFFFFFFU)
#define FMUTEST_R_SMW_SETTING1_SMWPARM1_SHIFT    (0U)
/*! SMWPARM1 - SMW Parameter Set 1 */
#define FMUTEST_R_SMW_SETTING1_SMWPARM1(x)       (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_SMW_SETTING1_SMWPARM1_SHIFT)) & FMUTEST_R_SMW_SETTING1_SMWPARM1_MASK)
/*! @} */

/*! @name R_SMP_WHV0 - BIST SMP WHV Setting 0 Register */
/*! @{ */

#define FMUTEST_R_SMP_WHV0_SMPWHV0_MASK          (0xFFFFFFFFU)
#define FMUTEST_R_SMP_WHV0_SMPWHV0_SHIFT         (0U)
/*! SMPWHV0 - SMP WHV Parameter Set 0 */
#define FMUTEST_R_SMP_WHV0_SMPWHV0(x)            (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_SMP_WHV0_SMPWHV0_SHIFT)) & FMUTEST_R_SMP_WHV0_SMPWHV0_MASK)
/*! @} */

/*! @name R_SMP_WHV1 - BIST SMP WHV Setting 1 Register */
/*! @{ */

#define FMUTEST_R_SMP_WHV1_SMPWHV1_MASK          (0xFFFFFFFFU)
#define FMUTEST_R_SMP_WHV1_SMPWHV1_SHIFT         (0U)
/*! SMPWHV1 - SMP WHV Parameter Set 1 */
#define FMUTEST_R_SMP_WHV1_SMPWHV1(x)            (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_SMP_WHV1_SMPWHV1_SHIFT)) & FMUTEST_R_SMP_WHV1_SMPWHV1_MASK)
/*! @} */

/*! @name R_SME_WHV0 - BIST SME WHV Setting 0 Register */
/*! @{ */

#define FMUTEST_R_SME_WHV0_SMEWHV0_MASK          (0xFFFFFFFFU)
#define FMUTEST_R_SME_WHV0_SMEWHV0_SHIFT         (0U)
/*! SMEWHV0 - SME WHV Parameter Set 0 */
#define FMUTEST_R_SME_WHV0_SMEWHV0(x)            (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_SME_WHV0_SMEWHV0_SHIFT)) & FMUTEST_R_SME_WHV0_SMEWHV0_MASK)
/*! @} */

/*! @name R_SME_WHV1 - BIST SME WHV Setting 1 Register */
/*! @{ */

#define FMUTEST_R_SME_WHV1_SMEWHV1_MASK          (0xFFFFFFFFU)
#define FMUTEST_R_SME_WHV1_SMEWHV1_SHIFT         (0U)
/*! SMEWHV1 - SME WHV Parameter Set 1 */
#define FMUTEST_R_SME_WHV1_SMEWHV1(x)            (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_SME_WHV1_SMEWHV1_SHIFT)) & FMUTEST_R_SME_WHV1_SMEWHV1_MASK)
/*! @} */

/*! @name R_SMW_SETTING2 - BIST SMW Setting 2 Register */
/*! @{ */

#define FMUTEST_R_SMW_SETTING2_SMWPARM2_MASK     (0x1FFFFFFFU)
#define FMUTEST_R_SMW_SETTING2_SMWPARM2_SHIFT    (0U)
/*! SMWPARM2 - SMW Parameter Set 2 */
#define FMUTEST_R_SMW_SETTING2_SMWPARM2(x)       (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_SMW_SETTING2_SMWPARM2_SHIFT)) & FMUTEST_R_SMW_SETTING2_SMWPARM2_MASK)
/*! @} */

/*! @name R_D_MISR0 - BIST DIN MISR 0 Register */
/*! @{ */

#define FMUTEST_R_D_MISR0_DATASIG0_MASK          (0xFFFFFFFFU)
#define FMUTEST_R_D_MISR0_DATASIG0_SHIFT         (0U)
/*! DATASIG0 - Data Signature */
#define FMUTEST_R_D_MISR0_DATASIG0(x)            (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_D_MISR0_DATASIG0_SHIFT)) & FMUTEST_R_D_MISR0_DATASIG0_MASK)
/*! @} */

/*! @name R_A_MISR0 - BIST Address MISR 0 Register */
/*! @{ */

#define FMUTEST_R_A_MISR0_ADRSIG0_MASK           (0xFFFFFFFFU)
#define FMUTEST_R_A_MISR0_ADRSIG0_SHIFT          (0U)
/*! ADRSIG0 - Address Signature */
#define FMUTEST_R_A_MISR0_ADRSIG0(x)             (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_A_MISR0_ADRSIG0_SHIFT)) & FMUTEST_R_A_MISR0_ADRSIG0_MASK)
/*! @} */

/*! @name R_C_MISR0 - BIST Control MISR 0 Register */
/*! @{ */

#define FMUTEST_R_C_MISR0_CTRLSIG0_MASK          (0xFFFFFFFFU)
#define FMUTEST_R_C_MISR0_CTRLSIG0_SHIFT         (0U)
/*! CTRLSIG0 - Control Signature */
#define FMUTEST_R_C_MISR0_CTRLSIG0(x)            (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_C_MISR0_CTRLSIG0_SHIFT)) & FMUTEST_R_C_MISR0_CTRLSIG0_MASK)
/*! @} */

/*! @name R_SMW_SETTING3 - BIST SMW Setting 3 Register */
/*! @{ */

#define FMUTEST_R_SMW_SETTING3_SMWPARM3_MASK     (0x1FFFFU)
#define FMUTEST_R_SMW_SETTING3_SMWPARM3_SHIFT    (0U)
/*! SMWPARM3 - SMW Parameter Set 3 */
#define FMUTEST_R_SMW_SETTING3_SMWPARM3(x)       (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_SMW_SETTING3_SMWPARM3_SHIFT)) & FMUTEST_R_SMW_SETTING3_SMWPARM3_MASK)
/*! @} */

/*! @name R_DATA_CTRL1 - BIST Data Control 1 Register */
/*! @{ */

#define FMUTEST_R_DATA_CTRL1_DATA1_MASK          (0xFFFFFFFFU)
#define FMUTEST_R_DATA_CTRL1_DATA1_SHIFT         (0U)
/*! DATA1 - BIST Data 1 Low */
#define FMUTEST_R_DATA_CTRL1_DATA1(x)            (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_DATA_CTRL1_DATA1_SHIFT)) & FMUTEST_R_DATA_CTRL1_DATA1_MASK)
/*! @} */

/*! @name R_DATA_CTRL2 - BIST Data Control 2 Register */
/*! @{ */

#define FMUTEST_R_DATA_CTRL2_DATA2_MASK          (0xFFFFFFFFU)
#define FMUTEST_R_DATA_CTRL2_DATA2_SHIFT         (0U)
/*! DATA2 - BIST Data 2 Low */
#define FMUTEST_R_DATA_CTRL2_DATA2(x)            (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_DATA_CTRL2_DATA2_SHIFT)) & FMUTEST_R_DATA_CTRL2_DATA2_MASK)
/*! @} */

/*! @name R_DATA_CTRL3 - BIST Data Control 3 Register */
/*! @{ */

#define FMUTEST_R_DATA_CTRL3_DATA3_MASK          (0xFFFFFFFFU)
#define FMUTEST_R_DATA_CTRL3_DATA3_SHIFT         (0U)
/*! DATA3 - BIST Data 3 Low */
#define FMUTEST_R_DATA_CTRL3_DATA3(x)            (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_DATA_CTRL3_DATA3_SHIFT)) & FMUTEST_R_DATA_CTRL3_DATA3_MASK)
/*! @} */

/*! @name R_REPAIR0_0 - BIST Repair 0 for Block 0 Register */
/*! @{ */

#define FMUTEST_R_REPAIR0_0_RDIS0_0_MASK         (0x1U)
#define FMUTEST_R_REPAIR0_0_RDIS0_0_SHIFT        (0U)
/*! RDIS0_0 - Control Repair 0 in Block 0.
 *  0b0..Repair address is valid
 *  0b1..Repair address is not valid
 */
#define FMUTEST_R_REPAIR0_0_RDIS0_0(x)           (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_REPAIR0_0_RDIS0_0_SHIFT)) & FMUTEST_R_REPAIR0_0_RDIS0_0_MASK)

#define FMUTEST_R_REPAIR0_0_RADR0_0_MASK         (0x1FEU)
#define FMUTEST_R_REPAIR0_0_RADR0_0_SHIFT        (1U)
/*! RADR0_0 - XADR for Repair 0 in Block 0 */
#define FMUTEST_R_REPAIR0_0_RADR0_0(x)           (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_REPAIR0_0_RADR0_0_SHIFT)) & FMUTEST_R_REPAIR0_0_RADR0_0_MASK)
/*! @} */

/*! @name R_REPAIR0_1 - BIST Repair 1 Block 0 Register */
/*! @{ */

#define FMUTEST_R_REPAIR0_1_RDIS0_1_MASK         (0x1U)
#define FMUTEST_R_REPAIR0_1_RDIS0_1_SHIFT        (0U)
/*! RDIS0_1 - Control Repair 1 in Block 0.
 *  0b0..Repair address is valid
 *  0b1..Repair address is not valid
 */
#define FMUTEST_R_REPAIR0_1_RDIS0_1(x)           (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_REPAIR0_1_RDIS0_1_SHIFT)) & FMUTEST_R_REPAIR0_1_RDIS0_1_MASK)

#define FMUTEST_R_REPAIR0_1_RADR0_1_MASK         (0x1FEU)
#define FMUTEST_R_REPAIR0_1_RADR0_1_SHIFT        (1U)
/*! RADR0_1 - XADR for Repair 1 in Block 0. */
#define FMUTEST_R_REPAIR0_1_RADR0_1(x)           (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_REPAIR0_1_RADR0_1_SHIFT)) & FMUTEST_R_REPAIR0_1_RADR0_1_MASK)
/*! @} */

/*! @name R_REPAIR1_0 - BIST Repair 0 Block 1 Register */
/*! @{ */

#define FMUTEST_R_REPAIR1_0_RDIS1_0_MASK         (0x1U)
#define FMUTEST_R_REPAIR1_0_RDIS1_0_SHIFT        (0U)
/*! RDIS1_0 - Control Repair 0 in Block 1.
 *  0b0..Repair address is valid
 *  0b1..Repair address is not valid
 */
#define FMUTEST_R_REPAIR1_0_RDIS1_0(x)           (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_REPAIR1_0_RDIS1_0_SHIFT)) & FMUTEST_R_REPAIR1_0_RDIS1_0_MASK)

#define FMUTEST_R_REPAIR1_0_RADR1_0_MASK         (0x1FEU)
#define FMUTEST_R_REPAIR1_0_RADR1_0_SHIFT        (1U)
/*! RADR1_0 - XADR for Repair 0 in Block 1. */
#define FMUTEST_R_REPAIR1_0_RADR1_0(x)           (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_REPAIR1_0_RADR1_0_SHIFT)) & FMUTEST_R_REPAIR1_0_RADR1_0_MASK)
/*! @} */

/*! @name R_REPAIR1_1 - BIST Repair 1 Block 1 Register */
/*! @{ */

#define FMUTEST_R_REPAIR1_1_RDIS1_1_MASK         (0x1U)
#define FMUTEST_R_REPAIR1_1_RDIS1_1_SHIFT        (0U)
/*! RDIS1_1 - Control Repair 1 in Block 1.
 *  0b0..Repair address is valid
 *  0b1..Repair address is not valid
 */
#define FMUTEST_R_REPAIR1_1_RDIS1_1(x)           (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_REPAIR1_1_RDIS1_1_SHIFT)) & FMUTEST_R_REPAIR1_1_RDIS1_1_MASK)

#define FMUTEST_R_REPAIR1_1_RADR1_1_MASK         (0x1FEU)
#define FMUTEST_R_REPAIR1_1_RADR1_1_SHIFT        (1U)
/*! RADR1_1 - XADR for Repair 1 in Block 1. */
#define FMUTEST_R_REPAIR1_1_RADR1_1(x)           (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_REPAIR1_1_RADR1_1_SHIFT)) & FMUTEST_R_REPAIR1_1_RADR1_1_MASK)
/*! @} */

/*! @name R_DATA_CTRL0_EX - BIST Data Control 0 Extension Register */
/*! @{ */

#define FMUTEST_R_DATA_CTRL0_EX_DATA0X_MASK      (0x7U)
#define FMUTEST_R_DATA_CTRL0_EX_DATA0X_SHIFT     (0U)
/*! DATA0X - BIST Data 0 High */
#define FMUTEST_R_DATA_CTRL0_EX_DATA0X(x)        (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_DATA_CTRL0_EX_DATA0X_SHIFT)) & FMUTEST_R_DATA_CTRL0_EX_DATA0X_MASK)
/*! @} */

/*! @name R_TIMER_CTRL_EX - BIST Timer Control Extension Register */
/*! @{ */

#define FMUTEST_R_TIMER_CTRL_EX_TLVSDLY_H_MASK   (0x7U)
#define FMUTEST_R_TIMER_CTRL_EX_TLVSDLY_H_SHIFT  (0U)
/*! TLVSDLY_H - Tlvs Time Delay Scalar High */
#define FMUTEST_R_TIMER_CTRL_EX_TLVSDLY_H(x)     (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_TIMER_CTRL_EX_TLVSDLY_H_SHIFT)) & FMUTEST_R_TIMER_CTRL_EX_TLVSDLY_H_MASK)
/*! @} */

/*! @name R_DOUT_QUERY1 - BIST DOUT Query 1 Register */
/*! @{ */

#define FMUTEST_R_DOUT_QUERY1_DOUT_MASK          (0x7U)
#define FMUTEST_R_DOUT_QUERY1_DOUT_SHIFT         (0U)
/*! DOUT - Failing DOUT High */
#define FMUTEST_R_DOUT_QUERY1_DOUT(x)            (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_DOUT_QUERY1_DOUT_SHIFT)) & FMUTEST_R_DOUT_QUERY1_DOUT_MASK)
/*! @} */

/*! @name R_D_MISR1 - BIST DIN MISR 1 Register */
/*! @{ */

#define FMUTEST_R_D_MISR1_DATASIG1_MASK          (0xFFU)
#define FMUTEST_R_D_MISR1_DATASIG1_SHIFT         (0U)
/*! DATASIG1 - MISR Data Signature High */
#define FMUTEST_R_D_MISR1_DATASIG1(x)            (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_D_MISR1_DATASIG1_SHIFT)) & FMUTEST_R_D_MISR1_DATASIG1_MASK)
/*! @} */

/*! @name R_A_MISR1 - BIST Address MISR 1 Register */
/*! @{ */

#define FMUTEST_R_A_MISR1_ADRSIG1_MASK           (0xFFU)
#define FMUTEST_R_A_MISR1_ADRSIG1_SHIFT          (0U)
/*! ADRSIG1 - MISR Address Signature High */
#define FMUTEST_R_A_MISR1_ADRSIG1(x)             (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_A_MISR1_ADRSIG1_SHIFT)) & FMUTEST_R_A_MISR1_ADRSIG1_MASK)
/*! @} */

/*! @name R_C_MISR1 - BIST Control MISR 1 Register */
/*! @{ */

#define FMUTEST_R_C_MISR1_CTRLSIG1_MASK          (0xFFU)
#define FMUTEST_R_C_MISR1_CTRLSIG1_SHIFT         (0U)
/*! CTRLSIG1 - MISR Control Signature High */
#define FMUTEST_R_C_MISR1_CTRLSIG1(x)            (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_C_MISR1_CTRLSIG1_SHIFT)) & FMUTEST_R_C_MISR1_CTRLSIG1_MASK)
/*! @} */

/*! @name R_DATA_CTRL1_EX - BIST Data Control 1 Extension Register */
/*! @{ */

#define FMUTEST_R_DATA_CTRL1_EX_DATA1X_MASK      (0x7U)
#define FMUTEST_R_DATA_CTRL1_EX_DATA1X_SHIFT     (0U)
/*! DATA1X - BIST Data 1 High */
#define FMUTEST_R_DATA_CTRL1_EX_DATA1X(x)        (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_DATA_CTRL1_EX_DATA1X_SHIFT)) & FMUTEST_R_DATA_CTRL1_EX_DATA1X_MASK)
/*! @} */

/*! @name R_DATA_CTRL2_EX - BIST Data Control 2 Extension Register */
/*! @{ */

#define FMUTEST_R_DATA_CTRL2_EX_DATA2X_MASK      (0x7U)
#define FMUTEST_R_DATA_CTRL2_EX_DATA2X_SHIFT     (0U)
/*! DATA2X - BIST Data 2 High */
#define FMUTEST_R_DATA_CTRL2_EX_DATA2X(x)        (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_DATA_CTRL2_EX_DATA2X_SHIFT)) & FMUTEST_R_DATA_CTRL2_EX_DATA2X_MASK)
/*! @} */

/*! @name R_DATA_CTRL3_EX - BIST Data Control 3 Extension Register */
/*! @{ */

#define FMUTEST_R_DATA_CTRL3_EX_DATA3X_MASK      (0x7U)
#define FMUTEST_R_DATA_CTRL3_EX_DATA3X_SHIFT     (0U)
/*! DATA3X - BIST Data 3 High */
#define FMUTEST_R_DATA_CTRL3_EX_DATA3X(x)        (((uint32_t)(((uint32_t)(x)) << FMUTEST_R_DATA_CTRL3_EX_DATA3X_SHIFT)) & FMUTEST_R_DATA_CTRL3_EX_DATA3X_MASK)
/*! @} */

/*! @name SMW_TIMER_OPTION - SMW Timer Option Register */
/*! @{ */

#define FMUTEST_SMW_TIMER_OPTION_SMW_CDIVL_MASK  (0xFFU)
#define FMUTEST_SMW_TIMER_OPTION_SMW_CDIVL_SHIFT (0U)
/*! SMW_CDIVL - Clock Divide Scalar for Long Pulse */
#define FMUTEST_SMW_TIMER_OPTION_SMW_CDIVL(x)    (((uint32_t)(((uint32_t)(x)) << FMUTEST_SMW_TIMER_OPTION_SMW_CDIVL_SHIFT)) & FMUTEST_SMW_TIMER_OPTION_SMW_CDIVL_MASK)

#define FMUTEST_SMW_TIMER_OPTION_SMW_TVFY_MASK   (0x1F00U)
#define FMUTEST_SMW_TIMER_OPTION_SMW_TVFY_SHIFT  (8U)
/*! SMW_TVFY - Timer Adjust for Verify */
#define FMUTEST_SMW_TIMER_OPTION_SMW_TVFY(x)     (((uint32_t)(((uint32_t)(x)) << FMUTEST_SMW_TIMER_OPTION_SMW_TVFY_SHIFT)) & FMUTEST_SMW_TIMER_OPTION_SMW_TVFY_MASK)
/*! @} */

/*! @name SMW_SETTING_OPTION0 - SMW Setting Option 0 Register */
/*! @{ */

#define FMUTEST_SMW_SETTING_OPTION0_MV_INIT_MASK (0x1C000U)
#define FMUTEST_SMW_SETTING_OPTION0_MV_INIT_SHIFT (14U)
/*! MV_INIT - Medium Voltage Level Select Initial */
#define FMUTEST_SMW_SETTING_OPTION0_MV_INIT(x)   (((uint32_t)(((uint32_t)(x)) << FMUTEST_SMW_SETTING_OPTION0_MV_INIT_SHIFT)) & FMUTEST_SMW_SETTING_OPTION0_MV_INIT_MASK)

#define FMUTEST_SMW_SETTING_OPTION0_MV_END_MASK  (0xE0000U)
#define FMUTEST_SMW_SETTING_OPTION0_MV_END_SHIFT (17U)
/*! MV_END - Medium Voltage Level Select Final */
#define FMUTEST_SMW_SETTING_OPTION0_MV_END(x)    (((uint32_t)(((uint32_t)(x)) << FMUTEST_SMW_SETTING_OPTION0_MV_END_SHIFT)) & FMUTEST_SMW_SETTING_OPTION0_MV_END_MASK)

#define FMUTEST_SMW_SETTING_OPTION0_MV_MISC_MASK (0xF00000U)
#define FMUTEST_SMW_SETTING_OPTION0_MV_MISC_SHIFT (20U)
/*! MV_MISC - Medium Voltage Control Misc */
#define FMUTEST_SMW_SETTING_OPTION0_MV_MISC(x)   (((uint32_t)(((uint32_t)(x)) << FMUTEST_SMW_SETTING_OPTION0_MV_MISC_SHIFT)) & FMUTEST_SMW_SETTING_OPTION0_MV_MISC_MASK)

#define FMUTEST_SMW_SETTING_OPTION0_IPGM_INIT_MASK (0x3000000U)
#define FMUTEST_SMW_SETTING_OPTION0_IPGM_INIT_SHIFT (24U)
/*! IPGM_INIT - Program Current Control Initial */
#define FMUTEST_SMW_SETTING_OPTION0_IPGM_INIT(x) (((uint32_t)(((uint32_t)(x)) << FMUTEST_SMW_SETTING_OPTION0_IPGM_INIT_SHIFT)) & FMUTEST_SMW_SETTING_OPTION0_IPGM_INIT_MASK)

#define FMUTEST_SMW_SETTING_OPTION0_IPGM_END_MASK (0xC000000U)
#define FMUTEST_SMW_SETTING_OPTION0_IPGM_END_SHIFT (26U)
/*! IPGM_END - Program Current Control Final */
#define FMUTEST_SMW_SETTING_OPTION0_IPGM_END(x)  (((uint32_t)(((uint32_t)(x)) << FMUTEST_SMW_SETTING_OPTION0_IPGM_END_SHIFT)) & FMUTEST_SMW_SETTING_OPTION0_IPGM_END_MASK)

#define FMUTEST_SMW_SETTING_OPTION0_IPGM_MISC_MASK (0x70000000U)
#define FMUTEST_SMW_SETTING_OPTION0_IPGM_MISC_SHIFT (28U)
/*! IPGM_MISC - Program Current Control Misc */
#define FMUTEST_SMW_SETTING_OPTION0_IPGM_MISC(x) (((uint32_t)(((uint32_t)(x)) << FMUTEST_SMW_SETTING_OPTION0_IPGM_MISC_SHIFT)) & FMUTEST_SMW_SETTING_OPTION0_IPGM_MISC_MASK)
/*! @} */

/*! @name SMW_SETTING_OPTION2 - SMW Setting Option 2 Register */
/*! @{ */

#define FMUTEST_SMW_SETTING_OPTION2_THVS_CTRL_MASK (0x7U)
#define FMUTEST_SMW_SETTING_OPTION2_THVS_CTRL_SHIFT (0U)
/*! THVS_CTRL - Thvs control */
#define FMUTEST_SMW_SETTING_OPTION2_THVS_CTRL(x) (((uint32_t)(((uint32_t)(x)) << FMUTEST_SMW_SETTING_OPTION2_THVS_CTRL_SHIFT)) & FMUTEST_SMW_SETTING_OPTION2_THVS_CTRL_MASK)

#define FMUTEST_SMW_SETTING_OPTION2_TRCV_CTRL_MASK (0x38U)
#define FMUTEST_SMW_SETTING_OPTION2_TRCV_CTRL_SHIFT (3U)
/*! TRCV_CTRL - Trcv Control */
#define FMUTEST_SMW_SETTING_OPTION2_TRCV_CTRL(x) (((uint32_t)(((uint32_t)(x)) << FMUTEST_SMW_SETTING_OPTION2_TRCV_CTRL_SHIFT)) & FMUTEST_SMW_SETTING_OPTION2_TRCV_CTRL_MASK)

#define FMUTEST_SMW_SETTING_OPTION2_XTRA_ERS_MASK (0xC0U)
#define FMUTEST_SMW_SETTING_OPTION2_XTRA_ERS_SHIFT (6U)
/*! XTRA_ERS - Number of Post Shots for SME */
#define FMUTEST_SMW_SETTING_OPTION2_XTRA_ERS(x)  (((uint32_t)(((uint32_t)(x)) << FMUTEST_SMW_SETTING_OPTION2_XTRA_ERS_SHIFT)) & FMUTEST_SMW_SETTING_OPTION2_XTRA_ERS_MASK)

#define FMUTEST_SMW_SETTING_OPTION2_XTRA_PGM_MASK (0x300U)
#define FMUTEST_SMW_SETTING_OPTION2_XTRA_PGM_SHIFT (8U)
/*! XTRA_PGM - Number of Post Shots for SMP */
#define FMUTEST_SMW_SETTING_OPTION2_XTRA_PGM(x)  (((uint32_t)(((uint32_t)(x)) << FMUTEST_SMW_SETTING_OPTION2_XTRA_PGM_SHIFT)) & FMUTEST_SMW_SETTING_OPTION2_XTRA_PGM_MASK)

#define FMUTEST_SMW_SETTING_OPTION2_WHV_CNTR_MASK (0x3FC00U)
#define FMUTEST_SMW_SETTING_OPTION2_WHV_CNTR_SHIFT (10U)
/*! WHV_CNTR - WHV Counter */
#define FMUTEST_SMW_SETTING_OPTION2_WHV_CNTR(x)  (((uint32_t)(((uint32_t)(x)) << FMUTEST_SMW_SETTING_OPTION2_WHV_CNTR_SHIFT)) & FMUTEST_SMW_SETTING_OPTION2_WHV_CNTR_MASK)

#define FMUTEST_SMW_SETTING_OPTION2_POST_TERS_MASK (0x1C0000U)
#define FMUTEST_SMW_SETTING_OPTION2_POST_TERS_SHIFT (18U)
/*! POST_TERS - Post Ters Time
 *  0b000..50 usec
 *  0b001..100 usec
 *  0b010..200 usec
 *  0b011..300 usec
 *  0b100..500 usec
 *  0b101..1 msec
 *  0b110..1.5 msec
 *  0b111..2 msec
 */
#define FMUTEST_SMW_SETTING_OPTION2_POST_TERS(x) (((uint32_t)(((uint32_t)(x)) << FMUTEST_SMW_SETTING_OPTION2_POST_TERS_SHIFT)) & FMUTEST_SMW_SETTING_OPTION2_POST_TERS_MASK)

#define FMUTEST_SMW_SETTING_OPTION2_POST_TPGM_MASK (0x600000U)
#define FMUTEST_SMW_SETTING_OPTION2_POST_TPGM_SHIFT (21U)
/*! POST_TPGM - Post Tpgm Time
 *  0b00..1 usec
 *  0b01..2 usec
 *  0b10..4 usec
 *  0b11..8 usec
 */
#define FMUTEST_SMW_SETTING_OPTION2_POST_TPGM(x) (((uint32_t)(((uint32_t)(x)) << FMUTEST_SMW_SETTING_OPTION2_POST_TPGM_SHIFT)) & FMUTEST_SMW_SETTING_OPTION2_POST_TPGM_MASK)

#define FMUTEST_SMW_SETTING_OPTION2_VFY_OPT_MASK (0x1800000U)
#define FMUTEST_SMW_SETTING_OPTION2_VFY_OPT_SHIFT (23U)
/*! VFY_OPT - Verify Option
 *  0b00..Skip verify for post shot only, verify for all other shots
 *  0b01..Skip verify for the 1st and post shots
 *  0b10..Skip the 1st, 2nd, and post shots
 *  0b11..Skip verify for all shots
 */
#define FMUTEST_SMW_SETTING_OPTION2_VFY_OPT(x)   (((uint32_t)(((uint32_t)(x)) << FMUTEST_SMW_SETTING_OPTION2_VFY_OPT_SHIFT)) & FMUTEST_SMW_SETTING_OPTION2_VFY_OPT_MASK)

#define FMUTEST_SMW_SETTING_OPTION2_TPGM_OPT_MASK (0x6000000U)
#define FMUTEST_SMW_SETTING_OPTION2_TPGM_OPT_SHIFT (25U)
/*! TPGM_OPT - Tpgm Option
 *  0b00..Fixed Tpgm for all shots, except post shot
 *  0b01..Increase Tpgm option by 1 for each loop until Tpgm reaches 4 usec
 *  0b10..Increase Tpgm option by 1 for each loop until Tpgm reaches 8 usec
 *  0b11..Unused
 */
#define FMUTEST_SMW_SETTING_OPTION2_TPGM_OPT(x)  (((uint32_t)(((uint32_t)(x)) << FMUTEST_SMW_SETTING_OPTION2_TPGM_OPT_SHIFT)) & FMUTEST_SMW_SETTING_OPTION2_TPGM_OPT_MASK)

#define FMUTEST_SMW_SETTING_OPTION2_MASK0_OPT_MASK (0x8000000U)
#define FMUTEST_SMW_SETTING_OPTION2_MASK0_OPT_SHIFT (27U)
/*! MASK0_OPT - MASK0_OPT
 *  0b0..Mask programmed bits passing PV until extra shot
 *  0b1..Always program bits even if they pass PV
 */
#define FMUTEST_SMW_SETTING_OPTION2_MASK0_OPT(x) (((uint32_t)(((uint32_t)(x)) << FMUTEST_SMW_SETTING_OPTION2_MASK0_OPT_SHIFT)) & FMUTEST_SMW_SETTING_OPTION2_MASK0_OPT_MASK)

#define FMUTEST_SMW_SETTING_OPTION2_DIS_PRER_MASK (0x10000000U)
#define FMUTEST_SMW_SETTING_OPTION2_DIS_PRER_SHIFT (28U)
/*! DIS_PRER - Disable pre-PV Read before First Program Shot
 *  0b0..Enable pre-PV read before first program shot
 *  0b1..Disable pre-PV read before first program shot
 */
#define FMUTEST_SMW_SETTING_OPTION2_DIS_PRER(x)  (((uint32_t)(((uint32_t)(x)) << FMUTEST_SMW_SETTING_OPTION2_DIS_PRER_SHIFT)) & FMUTEST_SMW_SETTING_OPTION2_DIS_PRER_MASK)
/*! @} */

/*! @name SMW_SETTING_OPTION3 - SMW Setting Option 3 Register */
/*! @{ */

#define FMUTEST_SMW_SETTING_OPTION3_HEM_WHV_CNTR_MASK (0xFFU)
#define FMUTEST_SMW_SETTING_OPTION3_HEM_WHV_CNTR_SHIFT (0U)
/*! HEM_WHV_CNTR - WHV_COUNTER for HEM-erase Cycle */
#define FMUTEST_SMW_SETTING_OPTION3_HEM_WHV_CNTR(x) (((uint32_t)(((uint32_t)(x)) << FMUTEST_SMW_SETTING_OPTION3_HEM_WHV_CNTR_SHIFT)) & FMUTEST_SMW_SETTING_OPTION3_HEM_WHV_CNTR_MASK)

#define FMUTEST_SMW_SETTING_OPTION3_HEM_MAX_ERS_MASK (0x1FF00U)
#define FMUTEST_SMW_SETTING_OPTION3_HEM_MAX_ERS_SHIFT (8U)
/*! HEM_MAX_ERS - HEM Max Erase Shot Count */
#define FMUTEST_SMW_SETTING_OPTION3_HEM_MAX_ERS(x) (((uint32_t)(((uint32_t)(x)) << FMUTEST_SMW_SETTING_OPTION3_HEM_MAX_ERS_SHIFT)) & FMUTEST_SMW_SETTING_OPTION3_HEM_MAX_ERS_MASK)
/*! @} */

/*! @name SMW_SMP_WHV_OPTION0 - SMW SMP WHV Option 0 Register */
/*! @{ */

#define FMUTEST_SMW_SMP_WHV_OPTION0_SMP_WHV_OPT0_MASK (0xFFFFFFFFU)
#define FMUTEST_SMW_SMP_WHV_OPTION0_SMP_WHV_OPT0_SHIFT (0U)
/*! SMP_WHV_OPT0 - Smart Program WHV Option Low */
#define FMUTEST_SMW_SMP_WHV_OPTION0_SMP_WHV_OPT0(x) (((uint32_t)(((uint32_t)(x)) << FMUTEST_SMW_SMP_WHV_OPTION0_SMP_WHV_OPT0_SHIFT)) & FMUTEST_SMW_SMP_WHV_OPTION0_SMP_WHV_OPT0_MASK)
/*! @} */

/*! @name SMW_SME_WHV_OPTION0 - SMW SME WHV Option 0 Register */
/*! @{ */

#define FMUTEST_SMW_SME_WHV_OPTION0_SME_WHV_OPT0_MASK (0xFFFFFFFFU)
#define FMUTEST_SMW_SME_WHV_OPTION0_SME_WHV_OPT0_SHIFT (0U)
/*! SME_WHV_OPT0 - Smart Erase WHV Option Low */
#define FMUTEST_SMW_SME_WHV_OPTION0_SME_WHV_OPT0(x) (((uint32_t)(((uint32_t)(x)) << FMUTEST_SMW_SME_WHV_OPTION0_SME_WHV_OPT0_SHIFT)) & FMUTEST_SMW_SME_WHV_OPTION0_SME_WHV_OPT0_MASK)
/*! @} */

/*! @name SMW_SETTING_OPTION1 - SMW Setting Option 1 Register */
/*! @{ */

#define FMUTEST_SMW_SETTING_OPTION1_TERS_CTRL0_MASK (0x7U)
#define FMUTEST_SMW_SETTING_OPTION1_TERS_CTRL0_SHIFT (0U)
/*! TERS_CTRL0 - Ters Control
 *  0b000..50 usec
 *  0b001..100 usec
 *  0b010..200 usec
 *  0b011..300 usec
 *  0b100..500 usec
 *  0b101..1 msec
 *  0b110..1.5 msec
 *  0b111..2 msec
 */
#define FMUTEST_SMW_SETTING_OPTION1_TERS_CTRL0(x) (((uint32_t)(((uint32_t)(x)) << FMUTEST_SMW_SETTING_OPTION1_TERS_CTRL0_SHIFT)) & FMUTEST_SMW_SETTING_OPTION1_TERS_CTRL0_MASK)

#define FMUTEST_SMW_SETTING_OPTION1_TPGM_CTRL_MASK (0x18U)
#define FMUTEST_SMW_SETTING_OPTION1_TPGM_CTRL_SHIFT (3U)
/*! TPGM_CTRL - Tpgm Control
 *  0b00..1 usec
 *  0b01..2 usec
 *  0b10..4 usec
 *  0b11..8 usec
 */
#define FMUTEST_SMW_SETTING_OPTION1_TPGM_CTRL(x) (((uint32_t)(((uint32_t)(x)) << FMUTEST_SMW_SETTING_OPTION1_TPGM_CTRL_SHIFT)) & FMUTEST_SMW_SETTING_OPTION1_TPGM_CTRL_MASK)

#define FMUTEST_SMW_SETTING_OPTION1_TNVS_CTRL_MASK (0xE0U)
#define FMUTEST_SMW_SETTING_OPTION1_TNVS_CTRL_SHIFT (5U)
/*! TNVS_CTRL - Tnvs Control
 *  0b000..5 usec
 *  0b001..8 usec
 *  0b010..11 usec
 *  0b011..14 usec
 *  0b100..17 usec
 *  0b101..20 usec
 *  0b110..23 usec
 *  0b111..26 usec
 */
#define FMUTEST_SMW_SETTING_OPTION1_TNVS_CTRL(x) (((uint32_t)(((uint32_t)(x)) << FMUTEST_SMW_SETTING_OPTION1_TNVS_CTRL_SHIFT)) & FMUTEST_SMW_SETTING_OPTION1_TNVS_CTRL_MASK)

#define FMUTEST_SMW_SETTING_OPTION1_TNVH_CTRL_MASK (0x700U)
#define FMUTEST_SMW_SETTING_OPTION1_TNVH_CTRL_SHIFT (8U)
/*! TNVH_CTRL - Tnvh Control
 *  0b000..2 usec
 *  0b001..2.5 usec
 *  0b010..3 usec
 *  0b011..3.5 usec
 *  0b100..4 usec
 *  0b101..4.5 usec
 *  0b110..5 usec
 *  0b111..5.5 usec
 */
#define FMUTEST_SMW_SETTING_OPTION1_TNVH_CTRL(x) (((uint32_t)(((uint32_t)(x)) << FMUTEST_SMW_SETTING_OPTION1_TNVH_CTRL_SHIFT)) & FMUTEST_SMW_SETTING_OPTION1_TNVH_CTRL_MASK)

#define FMUTEST_SMW_SETTING_OPTION1_TPGS_CTRL_MASK (0x3800U)
#define FMUTEST_SMW_SETTING_OPTION1_TPGS_CTRL_SHIFT (11U)
/*! TPGS_CTRL - Tpgs Control
 *  0b000..1 usec
 *  0b001..2 usec
 *  0b010..3 usec
 *  0b011..4 usec
 *  0b100..5 usec
 *  0b101..6 usec
 *  0b110..7 usec
 *  0b111..8 usec
 */
#define FMUTEST_SMW_SETTING_OPTION1_TPGS_CTRL(x) (((uint32_t)(((uint32_t)(x)) << FMUTEST_SMW_SETTING_OPTION1_TPGS_CTRL_SHIFT)) & FMUTEST_SMW_SETTING_OPTION1_TPGS_CTRL_MASK)

#define FMUTEST_SMW_SETTING_OPTION1_MAX_ERASE_MASK (0x7FC000U)
#define FMUTEST_SMW_SETTING_OPTION1_MAX_ERASE_SHIFT (14U)
/*! MAX_ERASE - Number of Erase Shots */
#define FMUTEST_SMW_SETTING_OPTION1_MAX_ERASE(x) (((uint32_t)(((uint32_t)(x)) << FMUTEST_SMW_SETTING_OPTION1_MAX_ERASE_SHIFT)) & FMUTEST_SMW_SETTING_OPTION1_MAX_ERASE_MASK)

#define FMUTEST_SMW_SETTING_OPTION1_MAX_PROG_MASK (0xF800000U)
#define FMUTEST_SMW_SETTING_OPTION1_MAX_PROG_SHIFT (23U)
/*! MAX_PROG - Number of Program Shots */
#define FMUTEST_SMW_SETTING_OPTION1_MAX_PROG(x)  (((uint32_t)(((uint32_t)(x)) << FMUTEST_SMW_SETTING_OPTION1_MAX_PROG_SHIFT)) & FMUTEST_SMW_SETTING_OPTION1_MAX_PROG_MASK)
/*! @} */

/*! @name SMW_SMP_WHV_OPTION1 - SMW SMP WHV Option 1 Register */
/*! @{ */

#define FMUTEST_SMW_SMP_WHV_OPTION1_SMP_WHV_OPT1_MASK (0xFFFFFFFFU)
#define FMUTEST_SMW_SMP_WHV_OPTION1_SMP_WHV_OPT1_SHIFT (0U)
/*! SMP_WHV_OPT1 - Smart Program WHV Option High */
#define FMUTEST_SMW_SMP_WHV_OPTION1_SMP_WHV_OPT1(x) (((uint32_t)(((uint32_t)(x)) << FMUTEST_SMW_SMP_WHV_OPTION1_SMP_WHV_OPT1_SHIFT)) & FMUTEST_SMW_SMP_WHV_OPTION1_SMP_WHV_OPT1_MASK)
/*! @} */

/*! @name SMW_SME_WHV_OPTION1 - SMW SME WHV Option 1 Register */
/*! @{ */

#define FMUTEST_SMW_SME_WHV_OPTION1_SME_WHV_OPT1_MASK (0xFFFFFFFFU)
#define FMUTEST_SMW_SME_WHV_OPTION1_SME_WHV_OPT1_SHIFT (0U)
/*! SME_WHV_OPT1 - Smart Erase WHV Option High */
#define FMUTEST_SMW_SME_WHV_OPTION1_SME_WHV_OPT1(x) (((uint32_t)(((uint32_t)(x)) << FMUTEST_SMW_SME_WHV_OPTION1_SME_WHV_OPT1_SHIFT)) & FMUTEST_SMW_SME_WHV_OPTION1_SME_WHV_OPT1_MASK)
/*! @} */

/*! @name REPAIR0_0 - FMU Repair 0 Block 0 Register */
/*! @{ */

#define FMUTEST_REPAIR0_0_RDIS0_0_MASK           (0x1U)
#define FMUTEST_REPAIR0_0_RDIS0_0_SHIFT          (0U)
/*! RDIS0_0 - RDIS0_0
 *  0b0..Repair address is valid
 *  0b1..Repair address is not valid
 */
#define FMUTEST_REPAIR0_0_RDIS0_0(x)             (((uint32_t)(((uint32_t)(x)) << FMUTEST_REPAIR0_0_RDIS0_0_SHIFT)) & FMUTEST_REPAIR0_0_RDIS0_0_MASK)

#define FMUTEST_REPAIR0_0_RADR0_0_MASK           (0x1FEU)
#define FMUTEST_REPAIR0_0_RADR0_0_SHIFT          (1U)
/*! RADR0_0 - RADR0_0 */
#define FMUTEST_REPAIR0_0_RADR0_0(x)             (((uint32_t)(((uint32_t)(x)) << FMUTEST_REPAIR0_0_RADR0_0_SHIFT)) & FMUTEST_REPAIR0_0_RADR0_0_MASK)
/*! @} */

/*! @name REPAIR0_1 - FMU Repair 1 Block 0 Register */
/*! @{ */

#define FMUTEST_REPAIR0_1_RDIS0_1_MASK           (0x1U)
#define FMUTEST_REPAIR0_1_RDIS0_1_SHIFT          (0U)
/*! RDIS0_1 - RDIS0_1
 *  0b0..Repair address is valid
 *  0b1..Repair address is not valid
 */
#define FMUTEST_REPAIR0_1_RDIS0_1(x)             (((uint32_t)(((uint32_t)(x)) << FMUTEST_REPAIR0_1_RDIS0_1_SHIFT)) & FMUTEST_REPAIR0_1_RDIS0_1_MASK)

#define FMUTEST_REPAIR0_1_RADR0_1_MASK           (0x1FEU)
#define FMUTEST_REPAIR0_1_RADR0_1_SHIFT          (1U)
/*! RADR0_1 - RADR0_1 */
#define FMUTEST_REPAIR0_1_RADR0_1(x)             (((uint32_t)(((uint32_t)(x)) << FMUTEST_REPAIR0_1_RADR0_1_SHIFT)) & FMUTEST_REPAIR0_1_RADR0_1_MASK)
/*! @} */

/*! @name REPAIR1_0 - FMU Repair 0 Block 1 Register */
/*! @{ */

#define FMUTEST_REPAIR1_0_RDIS1_0_MASK           (0x1U)
#define FMUTEST_REPAIR1_0_RDIS1_0_SHIFT          (0U)
/*! RDIS1_0 - RDIS1_0
 *  0b0..Repair address is valid
 *  0b1..Repair address is not valid
 */
#define FMUTEST_REPAIR1_0_RDIS1_0(x)             (((uint32_t)(((uint32_t)(x)) << FMUTEST_REPAIR1_0_RDIS1_0_SHIFT)) & FMUTEST_REPAIR1_0_RDIS1_0_MASK)

#define FMUTEST_REPAIR1_0_RADR1_0_MASK           (0x1FEU)
#define FMUTEST_REPAIR1_0_RADR1_0_SHIFT          (1U)
/*! RADR1_0 - RADR1_0 */
#define FMUTEST_REPAIR1_0_RADR1_0(x)             (((uint32_t)(((uint32_t)(x)) << FMUTEST_REPAIR1_0_RADR1_0_SHIFT)) & FMUTEST_REPAIR1_0_RADR1_0_MASK)
/*! @} */

/*! @name REPAIR1_1 - FMU Repair 1 Block 1 Register */
/*! @{ */

#define FMUTEST_REPAIR1_1_RDIS1_1_MASK           (0x1U)
#define FMUTEST_REPAIR1_1_RDIS1_1_SHIFT          (0U)
/*! RDIS1_1 - RDIS1_1
 *  0b0..Repair address is valid
 *  0b1..Repair address is not valid
 */
#define FMUTEST_REPAIR1_1_RDIS1_1(x)             (((uint32_t)(((uint32_t)(x)) << FMUTEST_REPAIR1_1_RDIS1_1_SHIFT)) & FMUTEST_REPAIR1_1_RDIS1_1_MASK)

#define FMUTEST_REPAIR1_1_RADR1_1_MASK           (0x1FEU)
#define FMUTEST_REPAIR1_1_RADR1_1_SHIFT          (1U)
/*! RADR1_1 - RADR1_1 */
#define FMUTEST_REPAIR1_1_RADR1_1(x)             (((uint32_t)(((uint32_t)(x)) << FMUTEST_REPAIR1_1_RADR1_1_SHIFT)) & FMUTEST_REPAIR1_1_RADR1_1_MASK)
/*! @} */

/*! @name SMW_HB_SIGNALS - SMW HB Signals Register */
/*! @{ */

#define FMUTEST_SMW_HB_SIGNALS_SMW_ARRAY_MASK    (0x7U)
#define FMUTEST_SMW_HB_SIGNALS_SMW_ARRAY_SHIFT   (0U)
/*! SMW_ARRAY - SMW Region Select
 *  0b000..Main array
 *  0b001..IFR space only or main (and REDEN space) with IFR space for mass erase
 *  0b010..IFR1 space
 *  0b100..REDEN space
 */
#define FMUTEST_SMW_HB_SIGNALS_SMW_ARRAY(x)      (((uint32_t)(((uint32_t)(x)) << FMUTEST_SMW_HB_SIGNALS_SMW_ARRAY_SHIFT)) & FMUTEST_SMW_HB_SIGNALS_SMW_ARRAY_MASK)

#define FMUTEST_SMW_HB_SIGNALS_USER_IFREN1_MASK  (0x8U)
#define FMUTEST_SMW_HB_SIGNALS_USER_IFREN1_SHIFT (3U)
/*! USER_IFREN1 - IFR1 Enable
 *  0b0..IFREN1 input to the flash array is driven LOW
 *  0b1..IFREN1 input to the flash array is driven HIGH
 */
#define FMUTEST_SMW_HB_SIGNALS_USER_IFREN1(x)    (((uint32_t)(((uint32_t)(x)) << FMUTEST_SMW_HB_SIGNALS_USER_IFREN1_SHIFT)) & FMUTEST_SMW_HB_SIGNALS_USER_IFREN1_MASK)

#define FMUTEST_SMW_HB_SIGNALS_USER_PV_MASK      (0x10U)
#define FMUTEST_SMW_HB_SIGNALS_USER_PV_SHIFT     (4U)
/*! USER_PV - Program Verify
 *  0b0..PV input to the flash array is driven LOW
 *  0b1..PV input to the flash array is driven HIGH
 */
#define FMUTEST_SMW_HB_SIGNALS_USER_PV(x)        (((uint32_t)(((uint32_t)(x)) << FMUTEST_SMW_HB_SIGNALS_USER_PV_SHIFT)) & FMUTEST_SMW_HB_SIGNALS_USER_PV_MASK)

#define FMUTEST_SMW_HB_SIGNALS_USER_EV_MASK      (0x20U)
#define FMUTEST_SMW_HB_SIGNALS_USER_EV_SHIFT     (5U)
/*! USER_EV - Erase Verify
 *  0b0..EV input to the flash array is driven LOW
 *  0b1..EV input to the flash array is driven HIGH
 */
#define FMUTEST_SMW_HB_SIGNALS_USER_EV(x)        (((uint32_t)(((uint32_t)(x)) << FMUTEST_SMW_HB_SIGNALS_USER_EV_SHIFT)) & FMUTEST_SMW_HB_SIGNALS_USER_EV_MASK)

#define FMUTEST_SMW_HB_SIGNALS_USER_IFREN_MASK   (0x40U)
#define FMUTEST_SMW_HB_SIGNALS_USER_IFREN_SHIFT  (6U)
/*! USER_IFREN - IFR Enable
 *  0b0..IFREN input to the flash array is driven LOW
 *  0b1..IFREN input to the flash array is driven HIGH
 */
#define FMUTEST_SMW_HB_SIGNALS_USER_IFREN(x)     (((uint32_t)(((uint32_t)(x)) << FMUTEST_SMW_HB_SIGNALS_USER_IFREN_SHIFT)) & FMUTEST_SMW_HB_SIGNALS_USER_IFREN_MASK)

#define FMUTEST_SMW_HB_SIGNALS_USER_REDEN_MASK   (0x80U)
#define FMUTEST_SMW_HB_SIGNALS_USER_REDEN_SHIFT  (7U)
/*! USER_REDEN - Repair Read Enable
 *  0b0..REDEN input to the flash array is driven LOW
 *  0b1..REDEN input to the flash array is driven HIGH
 */
#define FMUTEST_SMW_HB_SIGNALS_USER_REDEN(x)     (((uint32_t)(((uint32_t)(x)) << FMUTEST_SMW_HB_SIGNALS_USER_REDEN_SHIFT)) & FMUTEST_SMW_HB_SIGNALS_USER_REDEN_MASK)

#define FMUTEST_SMW_HB_SIGNALS_USER_HEM_MASK     (0x100U)
#define FMUTEST_SMW_HB_SIGNALS_USER_HEM_SHIFT    (8U)
/*! USER_HEM - High Endurance Enable
 *  0b0..HEM input to SMW / BIST PIN_CTRL[24] is driven LOW
 *  0b1..HEM input to SMW / BIST PIN_CTRL[24] is driven HIGH
 */
#define FMUTEST_SMW_HB_SIGNALS_USER_HEM(x)       (((uint32_t)(((uint32_t)(x)) << FMUTEST_SMW_HB_SIGNALS_USER_HEM_SHIFT)) & FMUTEST_SMW_HB_SIGNALS_USER_HEM_MASK)
/*! @} */

/*! @name BIST_DUMP_CTRL - BIST Datadump Control Register */
/*! @{ */

#define FMUTEST_BIST_DUMP_CTRL_BIST_DONE_MASK    (0x10000U)
#define FMUTEST_BIST_DUMP_CTRL_BIST_DONE_SHIFT   (16U)
/*! BIST_DONE - BIST Done
 *  0b0..The BIST (or data dump) is running
 *  0b1..The BIST (or data dump) has completed
 */
#define FMUTEST_BIST_DUMP_CTRL_BIST_DONE(x)      (((uint32_t)(((uint32_t)(x)) << FMUTEST_BIST_DUMP_CTRL_BIST_DONE_SHIFT)) & FMUTEST_BIST_DUMP_CTRL_BIST_DONE_MASK)

#define FMUTEST_BIST_DUMP_CTRL_BIST_FAIL_MASK    (0x20000U)
#define FMUTEST_BIST_DUMP_CTRL_BIST_FAIL_SHIFT   (17U)
/*! BIST_FAIL - BIST Fail
 *  0b0..The last BIST operation completed successfully (or could not fail)
 *  0b1..The last BIST operation failed
 */
#define FMUTEST_BIST_DUMP_CTRL_BIST_FAIL(x)      (((uint32_t)(((uint32_t)(x)) << FMUTEST_BIST_DUMP_CTRL_BIST_FAIL_SHIFT)) & FMUTEST_BIST_DUMP_CTRL_BIST_FAIL_MASK)

#define FMUTEST_BIST_DUMP_CTRL_DATADUMP_MASK     (0x40000U)
#define FMUTEST_BIST_DUMP_CTRL_DATADUMP_SHIFT    (18U)
/*! DATADUMP - Data Dump Enable */
#define FMUTEST_BIST_DUMP_CTRL_DATADUMP(x)       (((uint32_t)(((uint32_t)(x)) << FMUTEST_BIST_DUMP_CTRL_DATADUMP_SHIFT)) & FMUTEST_BIST_DUMP_CTRL_DATADUMP_MASK)

#define FMUTEST_BIST_DUMP_CTRL_DATADUMP_TRIG_MASK (0x80000U)
#define FMUTEST_BIST_DUMP_CTRL_DATADUMP_TRIG_SHIFT (19U)
/*! DATADUMP_TRIG - Data Dump Trigger */
#define FMUTEST_BIST_DUMP_CTRL_DATADUMP_TRIG(x)  (((uint32_t)(((uint32_t)(x)) << FMUTEST_BIST_DUMP_CTRL_DATADUMP_TRIG_SHIFT)) & FMUTEST_BIST_DUMP_CTRL_DATADUMP_TRIG_MASK)

#define FMUTEST_BIST_DUMP_CTRL_DATADUMP_PATT_MASK (0x300000U)
#define FMUTEST_BIST_DUMP_CTRL_DATADUMP_PATT_SHIFT (20U)
/*! DATADUMP_PATT - Data Dump Pattern Select
 *  0b00..All ones
 *  0b01..All zeroes
 *  0b10..Checkerboard
 *  0b11..Inverse checkerboard
 */
#define FMUTEST_BIST_DUMP_CTRL_DATADUMP_PATT(x)  (((uint32_t)(((uint32_t)(x)) << FMUTEST_BIST_DUMP_CTRL_DATADUMP_PATT_SHIFT)) & FMUTEST_BIST_DUMP_CTRL_DATADUMP_PATT_MASK)

#define FMUTEST_BIST_DUMP_CTRL_DATADUMP_MRGEN_MASK (0x400000U)
#define FMUTEST_BIST_DUMP_CTRL_DATADUMP_MRGEN_SHIFT (22U)
/*! DATADUMP_MRGEN - Data Dump Margin Enable
 *  0b0..Normal read pulse shape
 *  0b1..Margin read pulse shape
 */
#define FMUTEST_BIST_DUMP_CTRL_DATADUMP_MRGEN(x) (((uint32_t)(((uint32_t)(x)) << FMUTEST_BIST_DUMP_CTRL_DATADUMP_MRGEN_SHIFT)) & FMUTEST_BIST_DUMP_CTRL_DATADUMP_MRGEN_MASK)

#define FMUTEST_BIST_DUMP_CTRL_DATADUMP_MRGTYPE_MASK (0x800000U)
#define FMUTEST_BIST_DUMP_CTRL_DATADUMP_MRGTYPE_SHIFT (23U)
/*! DATADUMP_MRGTYPE - Data Dump Margin Type
 *  0b0..DIN method used
 *  0b1..TM method used
 */
#define FMUTEST_BIST_DUMP_CTRL_DATADUMP_MRGTYPE(x) (((uint32_t)(((uint32_t)(x)) << FMUTEST_BIST_DUMP_CTRL_DATADUMP_MRGTYPE_SHIFT)) & FMUTEST_BIST_DUMP_CTRL_DATADUMP_MRGTYPE_MASK)
/*! @} */

/*! @name ATX_PIN_CTRL - ATX Pin Control Register */
/*! @{ */

#define FMUTEST_ATX_PIN_CTRL_TM_TO_ATX_MASK      (0xFFU)
#define FMUTEST_ATX_PIN_CTRL_TM_TO_ATX_SHIFT     (0U)
/*! TM_TO_ATX - TM to ATX
 *  0b00000001..TM[0] to ATX0
 *  0b00000010..TM[1] to ATX0
 *  0b00000100..TM[2] to ATX0
 *  0b00001000..TM[3] to ATX0
 *  0b00010000..TM[0] to ATX1
 *  0b00100000..TM[1] to ATX1
 *  0b01000000..TM[2] to ATX1
 *  0b10000000..TM[3] to ATX1
 */
#define FMUTEST_ATX_PIN_CTRL_TM_TO_ATX(x)        (((uint32_t)(((uint32_t)(x)) << FMUTEST_ATX_PIN_CTRL_TM_TO_ATX_SHIFT)) & FMUTEST_ATX_PIN_CTRL_TM_TO_ATX_MASK)
/*! @} */

/*! @name FAILCNT - Fail Count Register */
/*! @{ */

#define FMUTEST_FAILCNT_FAILCNT_MASK             (0xFFFFFFFFU)
#define FMUTEST_FAILCNT_FAILCNT_SHIFT            (0U)
/*! FAILCNT - Fail Count */
#define FMUTEST_FAILCNT_FAILCNT(x)               (((uint32_t)(((uint32_t)(x)) << FMUTEST_FAILCNT_FAILCNT_SHIFT)) & FMUTEST_FAILCNT_FAILCNT_MASK)
/*! @} */

/*! @name PGM_PULSE_CNT0 - Block 0 Program Pulse Count Register */
/*! @{ */

#define FMUTEST_PGM_PULSE_CNT0_PGM_CNT0_MASK     (0xFFFFFFFFU)
#define FMUTEST_PGM_PULSE_CNT0_PGM_CNT0_SHIFT    (0U)
/*! PGM_CNT0 - Program Pulse Count */
#define FMUTEST_PGM_PULSE_CNT0_PGM_CNT0(x)       (((uint32_t)(((uint32_t)(x)) << FMUTEST_PGM_PULSE_CNT0_PGM_CNT0_SHIFT)) & FMUTEST_PGM_PULSE_CNT0_PGM_CNT0_MASK)
/*! @} */

/*! @name PGM_PULSE_CNT1 - Block 1 Program Pulse Count Register */
/*! @{ */

#define FMUTEST_PGM_PULSE_CNT1_PGM_CNT1_MASK     (0xFFFFFFFFU)
#define FMUTEST_PGM_PULSE_CNT1_PGM_CNT1_SHIFT    (0U)
/*! PGM_CNT1 - Program Pulse Count */
#define FMUTEST_PGM_PULSE_CNT1_PGM_CNT1(x)       (((uint32_t)(((uint32_t)(x)) << FMUTEST_PGM_PULSE_CNT1_PGM_CNT1_SHIFT)) & FMUTEST_PGM_PULSE_CNT1_PGM_CNT1_MASK)
/*! @} */

/*! @name ERS_PULSE_CNT - Erase Pulse Count Register */
/*! @{ */

#define FMUTEST_ERS_PULSE_CNT_ERS_CNT0_MASK      (0xFFFFU)
#define FMUTEST_ERS_PULSE_CNT_ERS_CNT0_SHIFT     (0U)
/*! ERS_CNT0 - Block 0 Erase Pulse Count */
#define FMUTEST_ERS_PULSE_CNT_ERS_CNT0(x)        (((uint32_t)(((uint32_t)(x)) << FMUTEST_ERS_PULSE_CNT_ERS_CNT0_SHIFT)) & FMUTEST_ERS_PULSE_CNT_ERS_CNT0_MASK)

#define FMUTEST_ERS_PULSE_CNT_ERS_CNT1_MASK      (0xFFFF0000U)
#define FMUTEST_ERS_PULSE_CNT_ERS_CNT1_SHIFT     (16U)
/*! ERS_CNT1 - Block 1 Erase Pulse Count */
#define FMUTEST_ERS_PULSE_CNT_ERS_CNT1(x)        (((uint32_t)(((uint32_t)(x)) << FMUTEST_ERS_PULSE_CNT_ERS_CNT1_SHIFT)) & FMUTEST_ERS_PULSE_CNT_ERS_CNT1_MASK)
/*! @} */

/*! @name MAX_PULSE_CNT - Maximum Pulse Count Register */
/*! @{ */

#define FMUTEST_MAX_PULSE_CNT_LAST_PCNT_MASK     (0x1FFU)
#define FMUTEST_MAX_PULSE_CNT_LAST_PCNT_SHIFT    (0U)
/*! LAST_PCNT - Last SMW Operation's Pulse Count */
#define FMUTEST_MAX_PULSE_CNT_LAST_PCNT(x)       (((uint32_t)(((uint32_t)(x)) << FMUTEST_MAX_PULSE_CNT_LAST_PCNT_SHIFT)) & FMUTEST_MAX_PULSE_CNT_LAST_PCNT_MASK)

#define FMUTEST_MAX_PULSE_CNT_MAX_ERS_CNT_MASK   (0x1FF0000U)
#define FMUTEST_MAX_PULSE_CNT_MAX_ERS_CNT_SHIFT  (16U)
/*! MAX_ERS_CNT - Maximum Erase Pulse Count */
#define FMUTEST_MAX_PULSE_CNT_MAX_ERS_CNT(x)     (((uint32_t)(((uint32_t)(x)) << FMUTEST_MAX_PULSE_CNT_MAX_ERS_CNT_SHIFT)) & FMUTEST_MAX_PULSE_CNT_MAX_ERS_CNT_MASK)

#define FMUTEST_MAX_PULSE_CNT_MAX_PGM_CNT_MASK   (0xF8000000U)
#define FMUTEST_MAX_PULSE_CNT_MAX_PGM_CNT_SHIFT  (27U)
/*! MAX_PGM_CNT - Maximum Program Pulse Count */
#define FMUTEST_MAX_PULSE_CNT_MAX_PGM_CNT(x)     (((uint32_t)(((uint32_t)(x)) << FMUTEST_MAX_PULSE_CNT_MAX_PGM_CNT_SHIFT)) & FMUTEST_MAX_PULSE_CNT_MAX_PGM_CNT_MASK)
/*! @} */

/*! @name PORT_CTRL - Port Control Register */
/*! @{ */

#define FMUTEST_PORT_CTRL_BDONE_SEL_MASK         (0x3U)
#define FMUTEST_PORT_CTRL_BDONE_SEL_SHIFT        (0U)
/*! BDONE_SEL - BIST Done Select
 *  0b00..Select internal bist_done signal from current module instantiation
 *  0b01..Select ipt_bist_fail signal from current module instantiation
 *  0b10..Select ipt_bist_done signal from other module instantiation
 *  0b11..Select AND of internal bist_done signal from current module instantiation with ipt_bist_done signal from other module instantiation
 */
#define FMUTEST_PORT_CTRL_BDONE_SEL(x)           (((uint32_t)(((uint32_t)(x)) << FMUTEST_PORT_CTRL_BDONE_SEL_SHIFT)) & FMUTEST_PORT_CTRL_BDONE_SEL_MASK)

#define FMUTEST_PORT_CTRL_BSDO_SEL_MASK          (0xCU)
#define FMUTEST_PORT_CTRL_BSDO_SEL_SHIFT         (2U)
/*! BSDO_SEL - BIST Serial Data Output Select
 *  0b00..Select internal bist_sdo signal from current module instantiation
 *  0b01..Select ipt_bist_done signal from current module instantiation
 *  0b10..Select ipt_bist_sdo signal from other module instantiation
 *  0b11..Select ipt_bist_done signal from other module instantiation
 */
#define FMUTEST_PORT_CTRL_BSDO_SEL(x)            (((uint32_t)(((uint32_t)(x)) << FMUTEST_PORT_CTRL_BSDO_SEL_SHIFT)) & FMUTEST_PORT_CTRL_BSDO_SEL_MASK)
/*! @} */


/*!
 * @}
 */ /* end of group FMUTEST_Register_Masks */


/*!
 * @}
 */ /* end of group FMUTEST_Peripheral_Access_Layer */


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


#endif  /* PERI_FMUTEST_H_ */

