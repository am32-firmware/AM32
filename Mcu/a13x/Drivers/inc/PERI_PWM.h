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
**     Build:               b251210
**
**     Abstract:
**         CMSIS Peripheral Access Layer for PWM
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
 * @file PERI_PWM.h
 * @version 2.0
 * @date 2024-10-29
 * @brief CMSIS Peripheral Access Layer for PWM
 *
 * CMSIS Peripheral Access Layer for PWM
 */

#if !defined(PERI_PWM_H_)
#define PERI_PWM_H_                              /**< Symbol preventing repeated inclusion */

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
   -- PWM Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PWM_Peripheral_Access_Layer PWM Peripheral Access Layer
 * @{
 */

/** PWM - Size of Registers Arrays */
#define PWM_SM_DISMAP_COUNT                       1u
#define PWM_SM_COUNT                              3u

/** PWM - Register Layout Typedef */
typedef struct {
  struct {                                         /* offset: 0x0, array step: 0x60 */
    __I  uint16_t CNT;                               /**< Counter Register, array offset: 0x0, array step: 0x60 */
    __IO uint16_t INIT;                              /**< Initial Count Register, array offset: 0x2, array step: 0x60 */
    __IO uint16_t CTRL2;                             /**< Control 2 Register, array offset: 0x4, array step: 0x60 */
    __IO uint16_t CTRL;                              /**< Control Register, array offset: 0x6, array step: 0x60 */
         uint8_t RESERVED_0[2];
    __IO uint16_t VAL0;                              /**< Value Register 0, array offset: 0xA, array step: 0x60 */
         uint8_t RESERVED_1[2];
    __IO uint16_t VAL1;                              /**< Value Register 1, array offset: 0xE, array step: 0x60 */
         uint8_t RESERVED_2[2];
    __IO uint16_t VAL2;                              /**< Value Register 2, array offset: 0x12, array step: 0x60 */
         uint8_t RESERVED_3[2];
    __IO uint16_t VAL3;                              /**< Value Register 3, array offset: 0x16, array step: 0x60 */
         uint8_t RESERVED_4[2];
    __IO uint16_t VAL4;                              /**< Value Register 4, array offset: 0x1A, array step: 0x60 */
         uint8_t RESERVED_5[2];
    __IO uint16_t VAL5;                              /**< Value Register 5, array offset: 0x1E, array step: 0x60 */
         uint8_t RESERVED_6[2];
    __IO uint16_t OCTRL;                             /**< Output Control Register, array offset: 0x22, array step: 0x60 */
    __IO uint16_t STS;                               /**< Status Register, array offset: 0x24, array step: 0x60 */
    __IO uint16_t INTEN;                             /**< Interrupt Enable Register, array offset: 0x26, array step: 0x60 */
    __IO uint16_t DMAEN;                             /**< DMA Enable Register, array offset: 0x28, array step: 0x60 */
    __IO uint16_t TCTRL;                             /**< Output Trigger Control Register, array offset: 0x2A, array step: 0x60 */
    __IO uint16_t DISMAP[PWM_SM_DISMAP_COUNT];       /**< Fault Disable Mapping Register 0, array offset: 0x2C, array step: index*0x60, index2*0x2 */
         uint8_t RESERVED_7[2];
    __IO uint16_t DTCNT0;                            /**< Deadtime Count Register 0, array offset: 0x30, array step: 0x60 */
    __IO uint16_t DTCNT1;                            /**< Deadtime Count Register 1, array offset: 0x32, array step: 0x60 */
         uint8_t RESERVED_8[8];
    __IO uint16_t CAPTCTRLX;                         /**< Capture Control X Register, array offset: 0x3C, array step: 0x60 */
    __IO uint16_t CAPTCOMPX;                         /**< Capture Compare X Register, array offset: 0x3E, array step: 0x60 */
    __I  uint16_t CVAL0;                             /**< Capture Value 0 Register, array offset: 0x40, array step: 0x60 */
    __I  uint16_t CVAL0CYC;                          /**< Capture Value 0 Cycle Register, array offset: 0x42, array step: 0x60 */
    __I  uint16_t CVAL1;                             /**< Capture Value 1 Register, array offset: 0x44, array step: 0x60 */
    __I  uint16_t CVAL1CYC;                          /**< Capture Value 1 Cycle Register, array offset: 0x46, array step: 0x60 */
         uint8_t RESERVED_9[16];
    __IO uint16_t PHASEDLY;                          /**< Phase Delay Register, array offset: 0x58, array step: 0x60, valid indices: [1-2] */
         uint8_t RESERVED_10[4];
    __IO uint16_t CAPTFILTX;                         /**< Capture PWM_X Input Filter Register, array offset: 0x5E, array step: 0x60 */
  } SM[PWM_SM_COUNT];
       uint8_t RESERVED_0[96];
  __IO uint16_t OUTEN;                             /**< Output Enable Register, offset: 0x180 */
  __IO uint16_t MASK;                              /**< Mask Register, offset: 0x182 */
  __IO uint16_t SWCOUT;                            /**< Software Controlled Output Register, offset: 0x184 */
  __IO uint16_t DTSRCSEL;                          /**< PWM Source Select Register, offset: 0x186 */
  __IO uint16_t MCTRL;                             /**< Master Control Register, offset: 0x188 */
  __IO uint16_t MCTRL2;                            /**< Master Control 2 Register, offset: 0x18A */
  __IO uint16_t FCTRL;                             /**< Fault Control Register, offset: 0x18C */
  __IO uint16_t FSTS;                              /**< Fault Status Register, offset: 0x18E */
  __IO uint16_t FFILT;                             /**< Fault Filter Register, offset: 0x190 */
  __IO uint16_t FTST;                              /**< Fault Test Register, offset: 0x192 */
  __IO uint16_t FCTRL2;                            /**< Fault Control 2 Register, offset: 0x194 */
} PWM_Type;

/* ----------------------------------------------------------------------------
   -- PWM Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PWM_Register_Masks PWM Register Masks
 * @{
 */

/*! @name CNT - Counter Register */
/*! @{ */

#define PWM_CNT_CNT_MASK                         (0xFFFFU)
#define PWM_CNT_CNT_SHIFT                        (0U)
/*! CNT - Counter Register Bits */
#define PWM_CNT_CNT(x)                           (((uint16_t)(((uint16_t)(x)) << PWM_CNT_CNT_SHIFT)) & PWM_CNT_CNT_MASK)
/*! @} */

/* The count of PWM_CNT */
#define PWM_CNT_COUNT                            (3U)

/*! @name INIT - Initial Count Register */
/*! @{ */

#define PWM_INIT_INIT_MASK                       (0xFFFFU)
#define PWM_INIT_INIT_SHIFT                      (0U)
/*! INIT - Initial Count Register Bits */
#define PWM_INIT_INIT(x)                         (((uint16_t)(((uint16_t)(x)) << PWM_INIT_INIT_SHIFT)) & PWM_INIT_INIT_MASK)
/*! @} */

/* The count of PWM_INIT */
#define PWM_INIT_COUNT                           (3U)

/*! @name CTRL2 - Control 2 Register */
/*! @{ */

#define PWM_CTRL2_CLK_SEL_MASK                   (0x3U)
#define PWM_CTRL2_CLK_SEL_SHIFT                  (0U)
/*! CLK_SEL - Clock Source Select
 *  0b00..The IPBus clock is used as the clock for the local prescaler and counter.
 *  0b01..EXT_CLK is used as the clock for the local prescaler and counter.
 *  0b10..Submodule 0's clock (AUX_CLK) is used as the source clock for the local prescaler and counter. This
 *        setting should not be used in submodule 0 as it forces the clock to logic 0.
 *  0b11..Reserved
 */
#define PWM_CTRL2_CLK_SEL(x)                     (((uint16_t)(((uint16_t)(x)) << PWM_CTRL2_CLK_SEL_SHIFT)) & PWM_CTRL2_CLK_SEL_MASK)

#define PWM_CTRL2_RELOAD_SEL_MASK                (0x4U)
#define PWM_CTRL2_RELOAD_SEL_SHIFT               (2U)
/*! RELOAD_SEL - Reload Source Select
 *  0b0..The local RELOAD signal is used to reload registers.
 *  0b1..The master RELOAD signal (from submodule 0) is used to reload registers. This setting should not be used
 *       in submodule 0 as it forces the RELOAD signal to logic 0.
 */
#define PWM_CTRL2_RELOAD_SEL(x)                  (((uint16_t)(((uint16_t)(x)) << PWM_CTRL2_RELOAD_SEL_SHIFT)) & PWM_CTRL2_RELOAD_SEL_MASK)

#define PWM_CTRL2_FORCE_SEL_MASK                 (0x38U)
#define PWM_CTRL2_FORCE_SEL_SHIFT                (3U)
/*! FORCE_SEL - Force Select
 *  0b000..The local force signal, CTRL2[FORCE], from this submodule is used to force updates.
 *  0b001..The master force signal from submodule 0 is used to force updates. This setting should not be used in
 *         submodule 0 as it holds the FORCE OUTPUT signal to logic 0.
 *  0b010..The local reload signal from this submodule is used to force updates without regard to the state of LDOK.
 *  0b011..The master reload signal from submodule0 is used to force updates if LDOK is set. This setting should
 *         not be used in submodule0 as it holds the FORCE OUTPUT signal to logic 0.
 *  0b100..The local sync signal from this submodule is used to force updates.
 *  0b101..The master sync signal from submodule0 is used to force updates. This setting should not be used in
 *         submodule0 as it holds the FORCE OUTPUT signal to logic 0.
 *  0b110..The external force signal, EXT_FORCE, from outside the PWM module causes updates.
 *  0b111..The external sync signal, EXT_SYNC, from outside the PWM module causes updates.
 */
#define PWM_CTRL2_FORCE_SEL(x)                   (((uint16_t)(((uint16_t)(x)) << PWM_CTRL2_FORCE_SEL_SHIFT)) & PWM_CTRL2_FORCE_SEL_MASK)

#define PWM_CTRL2_FORCE_MASK                     (0x40U)
#define PWM_CTRL2_FORCE_SHIFT                    (6U)
/*! FORCE - Force Initialization */
#define PWM_CTRL2_FORCE(x)                       (((uint16_t)(((uint16_t)(x)) << PWM_CTRL2_FORCE_SHIFT)) & PWM_CTRL2_FORCE_MASK)

#define PWM_CTRL2_FRCEN_MASK                     (0x80U)
#define PWM_CTRL2_FRCEN_SHIFT                    (7U)
/*! FRCEN - Force Enable
 *  0b0..Initialization from a FORCE_OUT is disabled.
 *  0b1..Initialization from a FORCE_OUT is enabled.
 */
#define PWM_CTRL2_FRCEN(x)                       (((uint16_t)(((uint16_t)(x)) << PWM_CTRL2_FRCEN_SHIFT)) & PWM_CTRL2_FRCEN_MASK)

#define PWM_CTRL2_INIT_SEL_MASK                  (0x300U)
#define PWM_CTRL2_INIT_SEL_SHIFT                 (8U)
/*! INIT_SEL - Initialization Control Select
 *  0b00..Local sync (PWM_X) causes initialization.
 *  0b01..Master reload from submodule 0 causes initialization. This setting should not be used in submodule 0 as
 *        it forces the INIT signal to logic 0. The submodule counter will only re-initialize when a master reload
 *        occurs.
 *  0b10..Master sync from submodule 0 causes initialization. This setting should not be used in submodule 0 as it forces the INIT signal to logic 0.
 *  0b11..EXT_SYNC causes initialization.
 */
#define PWM_CTRL2_INIT_SEL(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_CTRL2_INIT_SEL_SHIFT)) & PWM_CTRL2_INIT_SEL_MASK)

#define PWM_CTRL2_PWMX_INIT_MASK                 (0x400U)
#define PWM_CTRL2_PWMX_INIT_SHIFT                (10U)
/*! PWMX_INIT - PWM_X Initial Value */
#define PWM_CTRL2_PWMX_INIT(x)                   (((uint16_t)(((uint16_t)(x)) << PWM_CTRL2_PWMX_INIT_SHIFT)) & PWM_CTRL2_PWMX_INIT_MASK)

#define PWM_CTRL2_PWM45_INIT_MASK                (0x800U)
#define PWM_CTRL2_PWM45_INIT_SHIFT               (11U)
/*! PWM45_INIT - PWM45 Initial Value */
#define PWM_CTRL2_PWM45_INIT(x)                  (((uint16_t)(((uint16_t)(x)) << PWM_CTRL2_PWM45_INIT_SHIFT)) & PWM_CTRL2_PWM45_INIT_MASK)

#define PWM_CTRL2_PWM23_INIT_MASK                (0x1000U)
#define PWM_CTRL2_PWM23_INIT_SHIFT               (12U)
/*! PWM23_INIT - PWM23 Initial Value */
#define PWM_CTRL2_PWM23_INIT(x)                  (((uint16_t)(((uint16_t)(x)) << PWM_CTRL2_PWM23_INIT_SHIFT)) & PWM_CTRL2_PWM23_INIT_MASK)

#define PWM_CTRL2_INDEP_MASK                     (0x2000U)
#define PWM_CTRL2_INDEP_SHIFT                    (13U)
/*! INDEP - Independent or Complementary Pair Operation
 *  0b0..PWM_A and PWM_B form a complementary PWM pair.
 *  0b1..PWM_A and PWM_B outputs are independent PWMs.
 */
#define PWM_CTRL2_INDEP(x)                       (((uint16_t)(((uint16_t)(x)) << PWM_CTRL2_INDEP_SHIFT)) & PWM_CTRL2_INDEP_MASK)

#define PWM_CTRL2_WAITEN_MASK                    (0x4000U)
#define PWM_CTRL2_WAITEN_SHIFT                   (14U)
/*! WAITEN - Sleep Enable */
#define PWM_CTRL2_WAITEN(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_CTRL2_WAITEN_SHIFT)) & PWM_CTRL2_WAITEN_MASK)

#define PWM_CTRL2_DBGEN_MASK                     (0x8000U)
#define PWM_CTRL2_DBGEN_SHIFT                    (15U)
/*! DBGEN - Debug Enable */
#define PWM_CTRL2_DBGEN(x)                       (((uint16_t)(((uint16_t)(x)) << PWM_CTRL2_DBGEN_SHIFT)) & PWM_CTRL2_DBGEN_MASK)
/*! @} */

/* The count of PWM_CTRL2 */
#define PWM_CTRL2_COUNT                          (3U)

/*! @name CTRL - Control Register */
/*! @{ */

#define PWM_CTRL_DBLEN_MASK                      (0x1U)
#define PWM_CTRL_DBLEN_SHIFT                     (0U)
/*! DBLEN - Double Switching Enable
 *  0b0..Double switching disabled.
 *  0b1..Double switching enabled.
 */
#define PWM_CTRL_DBLEN(x)                        (((uint16_t)(((uint16_t)(x)) << PWM_CTRL_DBLEN_SHIFT)) & PWM_CTRL_DBLEN_MASK)

#define PWM_CTRL_DBLX_MASK                       (0x2U)
#define PWM_CTRL_DBLX_SHIFT                      (1U)
/*! DBLX - PWM_X Double Switching Enable
 *  0b0..PWM_X double pulse disabled.
 *  0b1..PWM_X double pulse enabled.
 */
#define PWM_CTRL_DBLX(x)                         (((uint16_t)(((uint16_t)(x)) << PWM_CTRL_DBLX_SHIFT)) & PWM_CTRL_DBLX_MASK)

#define PWM_CTRL_LDMOD_MASK                      (0x4U)
#define PWM_CTRL_LDMOD_SHIFT                     (2U)
/*! LDMOD - Load Mode Select
 *  0b0..Buffered registers of this submodule are loaded and take effect at the next PWM reload if MCTRL[LDOK] is set.
 *  0b1..Buffered registers of this submodule are loaded and take effect immediately upon MCTRL[LDOK] being set.
 *       In this case, it is not necessary to set CTRL[FULL] or CTRL[HALF].
 */
#define PWM_CTRL_LDMOD(x)                        (((uint16_t)(((uint16_t)(x)) << PWM_CTRL_LDMOD_SHIFT)) & PWM_CTRL_LDMOD_MASK)

#define PWM_CTRL_SPLIT_MASK                      (0x8U)
#define PWM_CTRL_SPLIT_SHIFT                     (3U)
/*! SPLIT - Split the DBLPWM signal to PWM_A and PWM_B
 *  0b0..DBLPWM is not split. PWM_A and PWM_B each have double pulses.
 *  0b1..DBLPWM is split to PWM_A and PWM_B.
 */
#define PWM_CTRL_SPLIT(x)                        (((uint16_t)(((uint16_t)(x)) << PWM_CTRL_SPLIT_SHIFT)) & PWM_CTRL_SPLIT_MASK)

#define PWM_CTRL_PRSC_MASK                       (0x70U)
#define PWM_CTRL_PRSC_SHIFT                      (4U)
/*! PRSC - Prescaler
 *  0b000..Prescaler 1
 *  0b001..Prescaler 2
 *  0b010..Prescaler 4
 *  0b011..Prescaler 8
 *  0b100..Prescaler 16
 *  0b101..Prescaler 32
 *  0b110..Prescaler 64
 *  0b111..Prescaler 128
 */
#define PWM_CTRL_PRSC(x)                         (((uint16_t)(((uint16_t)(x)) << PWM_CTRL_PRSC_SHIFT)) & PWM_CTRL_PRSC_MASK)

#define PWM_CTRL_COMPMODE_MASK                   (0x80U)
#define PWM_CTRL_COMPMODE_SHIFT                  (7U)
/*! COMPMODE - Compare Mode
 *  0b0..The VAL* registers and the PWM counter are compared using an "equal to" method. This means that PWM edges
 *       are only produced when the counter is equal to one of the VAL* register values. This implies that a PWM_A
 *       output that is high at the end of a period maintains this state until a match with VAL3 clears the output
 *       in the following period.
 *  0b1..The VAL* registers and the PWM counter are compared using an "equal to or greater than" method. This
 *       means that PWM edges are produced when the counter is equal to or greater than one of the VAL* register
 *       values. This implies that a PWM_A output that is high at the end of a period could go low at the start of the
 *       next period if the starting counter value is greater than (but not necessarily equal to) the new VAL3 value.
 */
#define PWM_CTRL_COMPMODE(x)                     (((uint16_t)(((uint16_t)(x)) << PWM_CTRL_COMPMODE_SHIFT)) & PWM_CTRL_COMPMODE_MASK)

#define PWM_CTRL_DT_MASK                         (0x300U)
#define PWM_CTRL_DT_SHIFT                        (8U)
/*! DT - Deadtime */
#define PWM_CTRL_DT(x)                           (((uint16_t)(((uint16_t)(x)) << PWM_CTRL_DT_SHIFT)) & PWM_CTRL_DT_MASK)

#define PWM_CTRL_FULL_MASK                       (0x400U)
#define PWM_CTRL_FULL_SHIFT                      (10U)
/*! FULL - Full Cycle Reload
 *  0b0..Full-cycle reloads disabled.
 *  0b1..Full-cycle reloads enabled.
 */
#define PWM_CTRL_FULL(x)                         (((uint16_t)(((uint16_t)(x)) << PWM_CTRL_FULL_SHIFT)) & PWM_CTRL_FULL_MASK)

#define PWM_CTRL_HALF_MASK                       (0x800U)
#define PWM_CTRL_HALF_SHIFT                      (11U)
/*! HALF - Half Cycle Reload
 *  0b0..Half-cycle reloads disabled.
 *  0b1..Half-cycle reloads enabled.
 */
#define PWM_CTRL_HALF(x)                         (((uint16_t)(((uint16_t)(x)) << PWM_CTRL_HALF_SHIFT)) & PWM_CTRL_HALF_MASK)

#define PWM_CTRL_LDFQ_MASK                       (0xF000U)
#define PWM_CTRL_LDFQ_SHIFT                      (12U)
/*! LDFQ - Load Frequency
 *  0b0000..Every PWM opportunity
 *  0b0001..Every 2 PWM opportunities
 *  0b0010..Every 3 PWM opportunities
 *  0b0011..Every 4 PWM opportunities
 *  0b0100..Every 5 PWM opportunities
 *  0b0101..Every 6 PWM opportunities
 *  0b0110..Every 7 PWM opportunities
 *  0b0111..Every 8 PWM opportunities
 *  0b1000..Every 9 PWM opportunities
 *  0b1001..Every 10 PWM opportunities
 *  0b1010..Every 11 PWM opportunities
 *  0b1011..Every 12 PWM opportunities
 *  0b1100..Every 13 PWM opportunities
 *  0b1101..Every 14 PWM opportunities
 *  0b1110..Every 15 PWM opportunities
 *  0b1111..Every 16 PWM opportunities
 */
#define PWM_CTRL_LDFQ(x)                         (((uint16_t)(((uint16_t)(x)) << PWM_CTRL_LDFQ_SHIFT)) & PWM_CTRL_LDFQ_MASK)
/*! @} */

/* The count of PWM_CTRL */
#define PWM_CTRL_COUNT                           (3U)

/*! @name VAL0 - Value Register 0 */
/*! @{ */

#define PWM_VAL0_VAL0_MASK                       (0xFFFFU)
#define PWM_VAL0_VAL0_SHIFT                      (0U)
/*! VAL0 - Value 0 */
#define PWM_VAL0_VAL0(x)                         (((uint16_t)(((uint16_t)(x)) << PWM_VAL0_VAL0_SHIFT)) & PWM_VAL0_VAL0_MASK)
/*! @} */

/* The count of PWM_VAL0 */
#define PWM_VAL0_COUNT                           (3U)

/*! @name VAL1 - Value Register 1 */
/*! @{ */

#define PWM_VAL1_VAL1_MASK                       (0xFFFFU)
#define PWM_VAL1_VAL1_SHIFT                      (0U)
/*! VAL1 - Value 1 */
#define PWM_VAL1_VAL1(x)                         (((uint16_t)(((uint16_t)(x)) << PWM_VAL1_VAL1_SHIFT)) & PWM_VAL1_VAL1_MASK)
/*! @} */

/* The count of PWM_VAL1 */
#define PWM_VAL1_COUNT                           (3U)

/*! @name VAL2 - Value Register 2 */
/*! @{ */

#define PWM_VAL2_VAL2_MASK                       (0xFFFFU)
#define PWM_VAL2_VAL2_SHIFT                      (0U)
/*! VAL2 - Value 2 */
#define PWM_VAL2_VAL2(x)                         (((uint16_t)(((uint16_t)(x)) << PWM_VAL2_VAL2_SHIFT)) & PWM_VAL2_VAL2_MASK)
/*! @} */

/* The count of PWM_VAL2 */
#define PWM_VAL2_COUNT                           (3U)

/*! @name VAL3 - Value Register 3 */
/*! @{ */

#define PWM_VAL3_VAL3_MASK                       (0xFFFFU)
#define PWM_VAL3_VAL3_SHIFT                      (0U)
/*! VAL3 - Value 3 */
#define PWM_VAL3_VAL3(x)                         (((uint16_t)(((uint16_t)(x)) << PWM_VAL3_VAL3_SHIFT)) & PWM_VAL3_VAL3_MASK)
/*! @} */

/* The count of PWM_VAL3 */
#define PWM_VAL3_COUNT                           (3U)

/*! @name VAL4 - Value Register 4 */
/*! @{ */

#define PWM_VAL4_VAL4_MASK                       (0xFFFFU)
#define PWM_VAL4_VAL4_SHIFT                      (0U)
/*! VAL4 - Value 4 */
#define PWM_VAL4_VAL4(x)                         (((uint16_t)(((uint16_t)(x)) << PWM_VAL4_VAL4_SHIFT)) & PWM_VAL4_VAL4_MASK)
/*! @} */

/* The count of PWM_VAL4 */
#define PWM_VAL4_COUNT                           (3U)

/*! @name VAL5 - Value Register 5 */
/*! @{ */

#define PWM_VAL5_VAL5_MASK                       (0xFFFFU)
#define PWM_VAL5_VAL5_SHIFT                      (0U)
/*! VAL5 - Value 5 */
#define PWM_VAL5_VAL5(x)                         (((uint16_t)(((uint16_t)(x)) << PWM_VAL5_VAL5_SHIFT)) & PWM_VAL5_VAL5_MASK)
/*! @} */

/* The count of PWM_VAL5 */
#define PWM_VAL5_COUNT                           (3U)

/*! @name OCTRL - Output Control Register */
/*! @{ */

#define PWM_OCTRL_PWMXFS_MASK                    (0x3U)
#define PWM_OCTRL_PWMXFS_SHIFT                   (0U)
/*! PWMXFS - PWM_X Fault State
 *  0b00..Output is forced to logic 0 state prior to consideration of output polarity control.
 *  0b01..Output is forced to logic 1 state prior to consideration of output polarity control.
 *  0b10, 0b11..Output is put in a high-impedance state.
 */
#define PWM_OCTRL_PWMXFS(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_OCTRL_PWMXFS_SHIFT)) & PWM_OCTRL_PWMXFS_MASK)

#define PWM_OCTRL_PWMBFS_MASK                    (0xCU)
#define PWM_OCTRL_PWMBFS_SHIFT                   (2U)
/*! PWMBFS - PWM_B Fault State
 *  0b00..Output is forced to logic 0 state prior to consideration of output polarity control.
 *  0b01..Output is forced to logic 1 state prior to consideration of output polarity control.
 *  0b10, 0b11..Output is put in a high-impedance state.
 */
#define PWM_OCTRL_PWMBFS(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_OCTRL_PWMBFS_SHIFT)) & PWM_OCTRL_PWMBFS_MASK)

#define PWM_OCTRL_PWMAFS_MASK                    (0x30U)
#define PWM_OCTRL_PWMAFS_SHIFT                   (4U)
/*! PWMAFS - PWM_A Fault State
 *  0b00..Output is forced to logic 0 state prior to consideration of output polarity control.
 *  0b01..Output is forced to logic 1 state prior to consideration of output polarity control.
 *  0b10, 0b11..Output is put in a high-impedance state.
 */
#define PWM_OCTRL_PWMAFS(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_OCTRL_PWMAFS_SHIFT)) & PWM_OCTRL_PWMAFS_MASK)

#define PWM_OCTRL_POLX_MASK                      (0x100U)
#define PWM_OCTRL_POLX_SHIFT                     (8U)
/*! POLX - PWM_X Output Polarity
 *  0b0..PWM_X output not inverted. A high level on the PWM_X pin represents the "on" or "active" state.
 *  0b1..PWM_X output inverted. A low level on the PWM_X pin represents the "on" or "active" state.
 */
#define PWM_OCTRL_POLX(x)                        (((uint16_t)(((uint16_t)(x)) << PWM_OCTRL_POLX_SHIFT)) & PWM_OCTRL_POLX_MASK)

#define PWM_OCTRL_POLB_MASK                      (0x200U)
#define PWM_OCTRL_POLB_SHIFT                     (9U)
/*! POLB - PWM_B Output Polarity
 *  0b0..PWM_B output not inverted. A high level on the PWM_B pin represents the "on" or "active" state.
 *  0b1..PWM_B output inverted. A low level on the PWM_B pin represents the "on" or "active" state.
 */
#define PWM_OCTRL_POLB(x)                        (((uint16_t)(((uint16_t)(x)) << PWM_OCTRL_POLB_SHIFT)) & PWM_OCTRL_POLB_MASK)

#define PWM_OCTRL_POLA_MASK                      (0x400U)
#define PWM_OCTRL_POLA_SHIFT                     (10U)
/*! POLA - PWM_A Output Polarity
 *  0b0..PWM_A output not inverted. A high level on the PWM_A pin represents the "on" or "active" state.
 *  0b1..PWM_A output inverted. A low level on the PWM_A pin represents the "on" or "active" state.
 */
#define PWM_OCTRL_POLA(x)                        (((uint16_t)(((uint16_t)(x)) << PWM_OCTRL_POLA_SHIFT)) & PWM_OCTRL_POLA_MASK)

#define PWM_OCTRL_PWMX_IN_MASK                   (0x2000U)
#define PWM_OCTRL_PWMX_IN_SHIFT                  (13U)
/*! PWMX_IN - PWM_X Input */
#define PWM_OCTRL_PWMX_IN(x)                     (((uint16_t)(((uint16_t)(x)) << PWM_OCTRL_PWMX_IN_SHIFT)) & PWM_OCTRL_PWMX_IN_MASK)

#define PWM_OCTRL_PWMB_IN_MASK                   (0x4000U)
#define PWM_OCTRL_PWMB_IN_SHIFT                  (14U)
/*! PWMB_IN - PWM_B Input */
#define PWM_OCTRL_PWMB_IN(x)                     (((uint16_t)(((uint16_t)(x)) << PWM_OCTRL_PWMB_IN_SHIFT)) & PWM_OCTRL_PWMB_IN_MASK)

#define PWM_OCTRL_PWMA_IN_MASK                   (0x8000U)
#define PWM_OCTRL_PWMA_IN_SHIFT                  (15U)
/*! PWMA_IN - PWM_A Input */
#define PWM_OCTRL_PWMA_IN(x)                     (((uint16_t)(((uint16_t)(x)) << PWM_OCTRL_PWMA_IN_SHIFT)) & PWM_OCTRL_PWMA_IN_MASK)
/*! @} */

/* The count of PWM_OCTRL */
#define PWM_OCTRL_COUNT                          (3U)

/*! @name STS - Status Register */
/*! @{ */

#define PWM_STS_CMPF_MASK                        (0x3FU)
#define PWM_STS_CMPF_SHIFT                       (0U)
/*! CMPF - Compare Flags
 *  0b000000..No compare event has occurred for a particular VALx value.
 *  0b000001..A compare event has occurred for a particular VALx value.
 */
#define PWM_STS_CMPF(x)                          (((uint16_t)(((uint16_t)(x)) << PWM_STS_CMPF_SHIFT)) & PWM_STS_CMPF_MASK)

#define PWM_STS_CFX0_MASK                        (0x40U)
#define PWM_STS_CFX0_SHIFT                       (6U)
/*! CFX0 - Capture Flag X0 */
#define PWM_STS_CFX0(x)                          (((uint16_t)(((uint16_t)(x)) << PWM_STS_CFX0_SHIFT)) & PWM_STS_CFX0_MASK)

#define PWM_STS_CFX1_MASK                        (0x80U)
#define PWM_STS_CFX1_SHIFT                       (7U)
/*! CFX1 - Capture Flag X1 */
#define PWM_STS_CFX1(x)                          (((uint16_t)(((uint16_t)(x)) << PWM_STS_CFX1_SHIFT)) & PWM_STS_CFX1_MASK)

#define PWM_STS_RF_MASK                          (0x1000U)
#define PWM_STS_RF_SHIFT                         (12U)
/*! RF - Reload Flag
 *  0b0..No new reload cycle since last STS[RF] clearing
 *  0b1..New reload cycle since last STS[RF] clearing
 */
#define PWM_STS_RF(x)                            (((uint16_t)(((uint16_t)(x)) << PWM_STS_RF_SHIFT)) & PWM_STS_RF_MASK)

#define PWM_STS_REF_MASK                         (0x2000U)
#define PWM_STS_REF_SHIFT                        (13U)
/*! REF - Reload Error Flag
 *  0b0..No reload error occurred.
 *  0b1..Reload signal occurred with non-coherent data and MCTRL[LDOK] = 0.
 */
#define PWM_STS_REF(x)                           (((uint16_t)(((uint16_t)(x)) << PWM_STS_REF_SHIFT)) & PWM_STS_REF_MASK)

#define PWM_STS_RUF_MASK                         (0x4000U)
#define PWM_STS_RUF_SHIFT                        (14U)
/*! RUF - Registers Updated Flag
 *  0b0..No register update has occurred since last reload.
 *  0b1..At least one of the double buffered registers has been updated since the last reload.
 */
#define PWM_STS_RUF(x)                           (((uint16_t)(((uint16_t)(x)) << PWM_STS_RUF_SHIFT)) & PWM_STS_RUF_MASK)
/*! @} */

/* The count of PWM_STS */
#define PWM_STS_COUNT                            (3U)

/*! @name INTEN - Interrupt Enable Register */
/*! @{ */

#define PWM_INTEN_CMPIE_MASK                     (0x3FU)
#define PWM_INTEN_CMPIE_SHIFT                    (0U)
/*! CMPIE - Compare Interrupt Enables
 *  0b000000..The corresponding STS[CMPF] bit will not cause an interrupt request.
 *  0b000001..The corresponding STS[CMPF] bit will cause an interrupt request.
 */
#define PWM_INTEN_CMPIE(x)                       (((uint16_t)(((uint16_t)(x)) << PWM_INTEN_CMPIE_SHIFT)) & PWM_INTEN_CMPIE_MASK)

#define PWM_INTEN_CX0IE_MASK                     (0x40U)
#define PWM_INTEN_CX0IE_SHIFT                    (6U)
/*! CX0IE - Capture X 0 Interrupt Enable
 *  0b0..Interrupt request disabled for STS[CFX0].
 *  0b1..Interrupt request enabled for STS[CFX0].
 */
#define PWM_INTEN_CX0IE(x)                       (((uint16_t)(((uint16_t)(x)) << PWM_INTEN_CX0IE_SHIFT)) & PWM_INTEN_CX0IE_MASK)

#define PWM_INTEN_CX1IE_MASK                     (0x80U)
#define PWM_INTEN_CX1IE_SHIFT                    (7U)
/*! CX1IE - Capture X 1 Interrupt Enable
 *  0b0..Interrupt request disabled for STS[CFX1].
 *  0b1..Interrupt request enabled for STS[CFX1].
 */
#define PWM_INTEN_CX1IE(x)                       (((uint16_t)(((uint16_t)(x)) << PWM_INTEN_CX1IE_SHIFT)) & PWM_INTEN_CX1IE_MASK)

#define PWM_INTEN_RIE_MASK                       (0x1000U)
#define PWM_INTEN_RIE_SHIFT                      (12U)
/*! RIE - Reload Interrupt Enable
 *  0b0..STS[RF] CPU interrupt requests disabled
 *  0b1..STS[RF] CPU interrupt requests enabled
 */
#define PWM_INTEN_RIE(x)                         (((uint16_t)(((uint16_t)(x)) << PWM_INTEN_RIE_SHIFT)) & PWM_INTEN_RIE_MASK)

#define PWM_INTEN_REIE_MASK                      (0x2000U)
#define PWM_INTEN_REIE_SHIFT                     (13U)
/*! REIE - Reload Error Interrupt Enable
 *  0b0..STS[REF] CPU interrupt requests disabled
 *  0b1..STS[REF] CPU interrupt requests enabled
 */
#define PWM_INTEN_REIE(x)                        (((uint16_t)(((uint16_t)(x)) << PWM_INTEN_REIE_SHIFT)) & PWM_INTEN_REIE_MASK)
/*! @} */

/* The count of PWM_INTEN */
#define PWM_INTEN_COUNT                          (3U)

/*! @name DMAEN - DMA Enable Register */
/*! @{ */

#define PWM_DMAEN_CX0DE_MASK                     (0x1U)
#define PWM_DMAEN_CX0DE_SHIFT                    (0U)
/*! CX0DE - Capture X0 FIFO DMA Enable */
#define PWM_DMAEN_CX0DE(x)                       (((uint16_t)(((uint16_t)(x)) << PWM_DMAEN_CX0DE_SHIFT)) & PWM_DMAEN_CX0DE_MASK)

#define PWM_DMAEN_CX1DE_MASK                     (0x2U)
#define PWM_DMAEN_CX1DE_SHIFT                    (1U)
/*! CX1DE - Capture X1 FIFO DMA Enable */
#define PWM_DMAEN_CX1DE(x)                       (((uint16_t)(((uint16_t)(x)) << PWM_DMAEN_CX1DE_SHIFT)) & PWM_DMAEN_CX1DE_MASK)

#define PWM_DMAEN_CAPTDE_MASK                    (0xC0U)
#define PWM_DMAEN_CAPTDE_SHIFT                   (6U)
/*! CAPTDE - Capture DMA Enable Source Select
 *  0b00..Read DMA requests disabled.
 *  0b01..Exceeding a FIFO watermark sets the DMA read request. This requires at least one of DMAEN[CA1DE],
 *        DMAEN[CA0DE], DMAEN[CB1DE], DMAEN[CB0DE], DMAEN[CX1DE], or DMAEN[CX0DE] to be set to determine which
 *        watermark(s) the DMA request is sensitive.
 *  0b10..A local synchronization (VAL1 matches counter) sets the read DMA request.
 *  0b11..A local reload (STS[RF] being set) sets the read DMA request.
 */
#define PWM_DMAEN_CAPTDE(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_DMAEN_CAPTDE_SHIFT)) & PWM_DMAEN_CAPTDE_MASK)

#define PWM_DMAEN_FAND_MASK                      (0x100U)
#define PWM_DMAEN_FAND_SHIFT                     (8U)
/*! FAND - FIFO Watermark AND Control
 *  0b0..Selected FIFO watermarks are OR'ed together.
 *  0b1..Selected FIFO watermarks are AND'ed together.
 */
#define PWM_DMAEN_FAND(x)                        (((uint16_t)(((uint16_t)(x)) << PWM_DMAEN_FAND_SHIFT)) & PWM_DMAEN_FAND_MASK)

#define PWM_DMAEN_VALDE_MASK                     (0x200U)
#define PWM_DMAEN_VALDE_SHIFT                    (9U)
/*! VALDE - Value Registers DMA Enable
 *  0b0..DMA write requests disabled
 *  0b1..Enabled
 */
#define PWM_DMAEN_VALDE(x)                       (((uint16_t)(((uint16_t)(x)) << PWM_DMAEN_VALDE_SHIFT)) & PWM_DMAEN_VALDE_MASK)
/*! @} */

/* The count of PWM_DMAEN */
#define PWM_DMAEN_COUNT                          (3U)

/*! @name TCTRL - Output Trigger Control Register */
/*! @{ */

#define PWM_TCTRL_OUT_TRIG_EN_MASK               (0x3FU)
#define PWM_TCTRL_OUT_TRIG_EN_SHIFT              (0U)
/*! OUT_TRIG_EN - Output Trigger Enables
 *  0b1xxxxx..PWM_OUT_TRIG1 will set when the counter value matches the VAL5 value.
 *  0bx1xxxx..PWM_OUT_TRIG0 will set when the counter value matches the VAL4 value.
 *  0bxx1xxx..PWM_OUT_TRIG1 will set when the counter value matches the VAL3 value.
 *  0bxxx1xx..PWM_OUT_TRIG0 will set when the counter value matches the VAL2 value.
 *  0bxxxx1x..PWM_OUT_TRIG1 will set when the counter value matches the VAL1 value.
 *  0bxxxxx1..PWM_OUT_TRIG0 will set when the counter value matches the VAL0 value.
 */
#define PWM_TCTRL_OUT_TRIG_EN(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_TCTRL_OUT_TRIG_EN_SHIFT)) & PWM_TCTRL_OUT_TRIG_EN_MASK)

#define PWM_TCTRL_TRGFRQ_MASK                    (0x1000U)
#define PWM_TCTRL_TRGFRQ_SHIFT                   (12U)
/*! TRGFRQ - Trigger Frequency
 *  0b0..Trigger outputs are generated during every PWM period even if the PWM is not reloaded every period due to CTRL[LDFQ] being non-zero.
 *  0b1..Trigger outputs are generated only during the final PWM period prior to a reload opportunity when the PWM
 *       is not reloaded every period due to CTRL[LDFQ] being non-zero.
 */
#define PWM_TCTRL_TRGFRQ(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_TCTRL_TRGFRQ_SHIFT)) & PWM_TCTRL_TRGFRQ_MASK)

#define PWM_TCTRL_PWBOT1_MASK                    (0x4000U)
#define PWM_TCTRL_PWBOT1_SHIFT                   (14U)
/*! PWBOT1 - Mux Output Trigger 1 Source Select
 *  0b0..Route the PWM_OUT_TRIG1 signal to PWM_MUX_TRIG1 port.
 *  0b1..Route the PWM_B output to the PWM_MUX_TRIG1 port.
 */
#define PWM_TCTRL_PWBOT1(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_TCTRL_PWBOT1_SHIFT)) & PWM_TCTRL_PWBOT1_MASK)

#define PWM_TCTRL_PWAOT0_MASK                    (0x8000U)
#define PWM_TCTRL_PWAOT0_SHIFT                   (15U)
/*! PWAOT0 - Mux Output Trigger 0 Source Select
 *  0b0..Route the PWM_OUT_TRIG0 signal to PWM_MUX_TRIG0 port.
 *  0b1..Route the PWM_A output to the PWM_MUX_TRIG0 port.
 */
#define PWM_TCTRL_PWAOT0(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_TCTRL_PWAOT0_SHIFT)) & PWM_TCTRL_PWAOT0_MASK)
/*! @} */

/* The count of PWM_TCTRL */
#define PWM_TCTRL_COUNT                          (3U)

/*! @name DISMAP - Fault Disable Mapping Register 0 */
/*! @{ */

#define PWM_DISMAP_DIS0A_MASK                    (0xFU)
#define PWM_DISMAP_DIS0A_SHIFT                   (0U)
/*! DIS0A - PWM_A Fault Disable Mask 0 */
#define PWM_DISMAP_DIS0A(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_DISMAP_DIS0A_SHIFT)) & PWM_DISMAP_DIS0A_MASK)

#define PWM_DISMAP_DIS0B_MASK                    (0xF0U)
#define PWM_DISMAP_DIS0B_SHIFT                   (4U)
/*! DIS0B - PWM_B Fault Disable Mask 0 */
#define PWM_DISMAP_DIS0B(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_DISMAP_DIS0B_SHIFT)) & PWM_DISMAP_DIS0B_MASK)

#define PWM_DISMAP_DIS0X_MASK                    (0xF00U)
#define PWM_DISMAP_DIS0X_SHIFT                   (8U)
/*! DIS0X - PWM_X Fault Disable Mask 0 */
#define PWM_DISMAP_DIS0X(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_DISMAP_DIS0X_SHIFT)) & PWM_DISMAP_DIS0X_MASK)
/*! @} */

/* The count of PWM_DISMAP */
#define PWM_DISMAP_COUNT                         (3U)

/* The count of PWM_DISMAP */
#define PWM_DISMAP_COUNT2                        (1U)

/*! @name DTCNT0 - Deadtime Count Register 0 */
/*! @{ */

#define PWM_DTCNT0_DTCNT0_MASK                   (0x7FFU)
#define PWM_DTCNT0_DTCNT0_SHIFT                  (0U)
/*! DTCNT0 - Deadtime Count Register 0 */
#define PWM_DTCNT0_DTCNT0(x)                     (((uint16_t)(((uint16_t)(x)) << PWM_DTCNT0_DTCNT0_SHIFT)) & PWM_DTCNT0_DTCNT0_MASK)
/*! @} */

/* The count of PWM_DTCNT0 */
#define PWM_DTCNT0_COUNT                         (3U)

/*! @name DTCNT1 - Deadtime Count Register 1 */
/*! @{ */

#define PWM_DTCNT1_DTCNT1_MASK                   (0x7FFU)
#define PWM_DTCNT1_DTCNT1_SHIFT                  (0U)
/*! DTCNT1 - Deadtime Count Register 1 */
#define PWM_DTCNT1_DTCNT1(x)                     (((uint16_t)(((uint16_t)(x)) << PWM_DTCNT1_DTCNT1_SHIFT)) & PWM_DTCNT1_DTCNT1_MASK)
/*! @} */

/* The count of PWM_DTCNT1 */
#define PWM_DTCNT1_COUNT                         (3U)

/*! @name CAPTCTRLX - Capture Control X Register */
/*! @{ */

#define PWM_CAPTCTRLX_ARMX_MASK                  (0x1U)
#define PWM_CAPTCTRLX_ARMX_SHIFT                 (0U)
/*! ARMX - Arm X
 *  0b0..Input capture operation is disabled.
 *  0b1..Input capture operation as specified by CAPTCTRLX[EDGXx] is enabled.
 */
#define PWM_CAPTCTRLX_ARMX(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_CAPTCTRLX_ARMX_SHIFT)) & PWM_CAPTCTRLX_ARMX_MASK)

#define PWM_CAPTCTRLX_ONESHOTX_MASK              (0x2U)
#define PWM_CAPTCTRLX_ONESHOTX_SHIFT             (1U)
/*! ONESHOTX - One Shot Mode Aux
 *  0b0..Free Running
 *  0b1..One Shot
 */
#define PWM_CAPTCTRLX_ONESHOTX(x)                (((uint16_t)(((uint16_t)(x)) << PWM_CAPTCTRLX_ONESHOTX_SHIFT)) & PWM_CAPTCTRLX_ONESHOTX_MASK)

#define PWM_CAPTCTRLX_EDGX0_MASK                 (0xCU)
#define PWM_CAPTCTRLX_EDGX0_SHIFT                (2U)
/*! EDGX0 - Edge X 0
 *  0b00..Disabled
 *  0b01..Capture falling edges
 *  0b10..Capture rising edges
 *  0b11..Capture any edge
 */
#define PWM_CAPTCTRLX_EDGX0(x)                   (((uint16_t)(((uint16_t)(x)) << PWM_CAPTCTRLX_EDGX0_SHIFT)) & PWM_CAPTCTRLX_EDGX0_MASK)

#define PWM_CAPTCTRLX_EDGX1_MASK                 (0x30U)
#define PWM_CAPTCTRLX_EDGX1_SHIFT                (4U)
/*! EDGX1 - Edge X 1
 *  0b00..Disabled
 *  0b01..Capture falling edges
 *  0b10..Capture rising edges
 *  0b11..Capture any edge
 */
#define PWM_CAPTCTRLX_EDGX1(x)                   (((uint16_t)(((uint16_t)(x)) << PWM_CAPTCTRLX_EDGX1_SHIFT)) & PWM_CAPTCTRLX_EDGX1_MASK)

#define PWM_CAPTCTRLX_INP_SELX_MASK              (0x40U)
#define PWM_CAPTCTRLX_INP_SELX_SHIFT             (6U)
/*! INP_SELX - Input Select X
 *  0b0..Raw PWM_X input signal selected as source.
 *  0b1..Edge Counter
 */
#define PWM_CAPTCTRLX_INP_SELX(x)                (((uint16_t)(((uint16_t)(x)) << PWM_CAPTCTRLX_INP_SELX_SHIFT)) & PWM_CAPTCTRLX_INP_SELX_MASK)

#define PWM_CAPTCTRLX_EDGCNTX_EN_MASK            (0x80U)
#define PWM_CAPTCTRLX_EDGCNTX_EN_SHIFT           (7U)
/*! EDGCNTX_EN - Edge Counter X Enable
 *  0b0..Edge counter disabled and held in reset
 *  0b1..Edge counter enabled
 */
#define PWM_CAPTCTRLX_EDGCNTX_EN(x)              (((uint16_t)(((uint16_t)(x)) << PWM_CAPTCTRLX_EDGCNTX_EN_SHIFT)) & PWM_CAPTCTRLX_EDGCNTX_EN_MASK)

#define PWM_CAPTCTRLX_CFXWM_MASK                 (0x300U)
#define PWM_CAPTCTRLX_CFXWM_SHIFT                (8U)
/*! CFXWM - Capture X FIFOs Water Mark */
#define PWM_CAPTCTRLX_CFXWM(x)                   (((uint16_t)(((uint16_t)(x)) << PWM_CAPTCTRLX_CFXWM_SHIFT)) & PWM_CAPTCTRLX_CFXWM_MASK)

#define PWM_CAPTCTRLX_CX0CNT_MASK                (0x1C00U)
#define PWM_CAPTCTRLX_CX0CNT_SHIFT               (10U)
/*! CX0CNT - Capture X0 FIFO Word Count */
#define PWM_CAPTCTRLX_CX0CNT(x)                  (((uint16_t)(((uint16_t)(x)) << PWM_CAPTCTRLX_CX0CNT_SHIFT)) & PWM_CAPTCTRLX_CX0CNT_MASK)

#define PWM_CAPTCTRLX_CX1CNT_MASK                (0xE000U)
#define PWM_CAPTCTRLX_CX1CNT_SHIFT               (13U)
/*! CX1CNT - Capture X1 FIFO Word Count */
#define PWM_CAPTCTRLX_CX1CNT(x)                  (((uint16_t)(((uint16_t)(x)) << PWM_CAPTCTRLX_CX1CNT_SHIFT)) & PWM_CAPTCTRLX_CX1CNT_MASK)
/*! @} */

/* The count of PWM_CAPTCTRLX */
#define PWM_CAPTCTRLX_COUNT                      (3U)

/*! @name CAPTCOMPX - Capture Compare X Register */
/*! @{ */

#define PWM_CAPTCOMPX_EDGCMPX_MASK               (0xFFU)
#define PWM_CAPTCOMPX_EDGCMPX_SHIFT              (0U)
/*! EDGCMPX - Edge Compare X */
#define PWM_CAPTCOMPX_EDGCMPX(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_CAPTCOMPX_EDGCMPX_SHIFT)) & PWM_CAPTCOMPX_EDGCMPX_MASK)

#define PWM_CAPTCOMPX_EDGCNTX_MASK               (0xFF00U)
#define PWM_CAPTCOMPX_EDGCNTX_SHIFT              (8U)
/*! EDGCNTX - Edge Counter X */
#define PWM_CAPTCOMPX_EDGCNTX(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_CAPTCOMPX_EDGCNTX_SHIFT)) & PWM_CAPTCOMPX_EDGCNTX_MASK)
/*! @} */

/* The count of PWM_CAPTCOMPX */
#define PWM_CAPTCOMPX_COUNT                      (3U)

/*! @name CVAL0 - Capture Value 0 Register */
/*! @{ */

#define PWM_CVAL0_CAPTVAL0_MASK                  (0xFFFFU)
#define PWM_CVAL0_CAPTVAL0_SHIFT                 (0U)
/*! CAPTVAL0 - Capture Value 0 */
#define PWM_CVAL0_CAPTVAL0(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_CVAL0_CAPTVAL0_SHIFT)) & PWM_CVAL0_CAPTVAL0_MASK)
/*! @} */

/* The count of PWM_CVAL0 */
#define PWM_CVAL0_COUNT                          (3U)

/*! @name CVAL0CYC - Capture Value 0 Cycle Register */
/*! @{ */

#define PWM_CVAL0CYC_CVAL0CYC_MASK               (0xFU)
#define PWM_CVAL0CYC_CVAL0CYC_SHIFT              (0U)
/*! CVAL0CYC - Capture Value 0 Cycle */
#define PWM_CVAL0CYC_CVAL0CYC(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_CVAL0CYC_CVAL0CYC_SHIFT)) & PWM_CVAL0CYC_CVAL0CYC_MASK)
/*! @} */

/* The count of PWM_CVAL0CYC */
#define PWM_CVAL0CYC_COUNT                       (3U)

/*! @name CVAL1 - Capture Value 1 Register */
/*! @{ */

#define PWM_CVAL1_CAPTVAL1_MASK                  (0xFFFFU)
#define PWM_CVAL1_CAPTVAL1_SHIFT                 (0U)
/*! CAPTVAL1 - Capture Value 1 */
#define PWM_CVAL1_CAPTVAL1(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_CVAL1_CAPTVAL1_SHIFT)) & PWM_CVAL1_CAPTVAL1_MASK)
/*! @} */

/* The count of PWM_CVAL1 */
#define PWM_CVAL1_COUNT                          (3U)

/*! @name CVAL1CYC - Capture Value 1 Cycle Register */
/*! @{ */

#define PWM_CVAL1CYC_CVAL1CYC_MASK               (0xFU)
#define PWM_CVAL1CYC_CVAL1CYC_SHIFT              (0U)
/*! CVAL1CYC - Capture Value 1 Cycle */
#define PWM_CVAL1CYC_CVAL1CYC(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_CVAL1CYC_CVAL1CYC_SHIFT)) & PWM_CVAL1CYC_CVAL1CYC_MASK)
/*! @} */

/* The count of PWM_CVAL1CYC */
#define PWM_CVAL1CYC_COUNT                       (3U)

/*! @name PHASEDLY - Phase Delay Register */
/*! @{ */

#define PWM_PHASEDLY_PHASEDLY_MASK               (0xFFFFU)
#define PWM_PHASEDLY_PHASEDLY_SHIFT              (0U)
/*! PHASEDLY - Initial Count Register Bits */
#define PWM_PHASEDLY_PHASEDLY(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_PHASEDLY_PHASEDLY_SHIFT)) & PWM_PHASEDLY_PHASEDLY_MASK)
/*! @} */

/* The count of PWM_PHASEDLY */
#define PWM_PHASEDLY_COUNT                       (3U)

/*! @name CAPTFILTX - Capture PWM_X Input Filter Register */
/*! @{ */

#define PWM_CAPTFILTX_CAPTX_FILT_PER_MASK        (0xFFU)
#define PWM_CAPTFILTX_CAPTX_FILT_PER_SHIFT       (0U)
/*! CAPTX_FILT_PER - Input Capture Filter Period */
#define PWM_CAPTFILTX_CAPTX_FILT_PER(x)          (((uint16_t)(((uint16_t)(x)) << PWM_CAPTFILTX_CAPTX_FILT_PER_SHIFT)) & PWM_CAPTFILTX_CAPTX_FILT_PER_MASK)

#define PWM_CAPTFILTX_CAPTX_FILT_CNT_MASK        (0x700U)
#define PWM_CAPTFILTX_CAPTX_FILT_CNT_SHIFT       (8U)
/*! CAPTX_FILT_CNT - Input Capture Filter Count */
#define PWM_CAPTFILTX_CAPTX_FILT_CNT(x)          (((uint16_t)(((uint16_t)(x)) << PWM_CAPTFILTX_CAPTX_FILT_CNT_SHIFT)) & PWM_CAPTFILTX_CAPTX_FILT_CNT_MASK)
/*! @} */

/* The count of PWM_CAPTFILTX */
#define PWM_CAPTFILTX_COUNT                      (3U)

/*! @name OUTEN - Output Enable Register */
/*! @{ */

#define PWM_OUTEN_PWMX_EN_MASK                   (0x7U)
#define PWM_OUTEN_PWMX_EN_SHIFT                  (0U)
/*! PWMX_EN - PWM_X Output Enables */
#define PWM_OUTEN_PWMX_EN(x)                     (((uint16_t)(((uint16_t)(x)) << PWM_OUTEN_PWMX_EN_SHIFT)) & PWM_OUTEN_PWMX_EN_MASK)

#define PWM_OUTEN_PWMB_EN_MASK                   (0x70U)
#define PWM_OUTEN_PWMB_EN_SHIFT                  (4U)
/*! PWMB_EN - PWM_B Output Enables */
#define PWM_OUTEN_PWMB_EN(x)                     (((uint16_t)(((uint16_t)(x)) << PWM_OUTEN_PWMB_EN_SHIFT)) & PWM_OUTEN_PWMB_EN_MASK)

#define PWM_OUTEN_PWMA_EN_MASK                   (0x700U)
#define PWM_OUTEN_PWMA_EN_SHIFT                  (8U)
/*! PWMA_EN - PWM_A Output Enables */
#define PWM_OUTEN_PWMA_EN(x)                     (((uint16_t)(((uint16_t)(x)) << PWM_OUTEN_PWMA_EN_SHIFT)) & PWM_OUTEN_PWMA_EN_MASK)
/*! @} */

/*! @name MASK - Mask Register */
/*! @{ */

#define PWM_MASK_MASKX_MASK                      (0x7U)
#define PWM_MASK_MASKX_SHIFT                     (0U)
/*! MASKX - PWM_X Masks */
#define PWM_MASK_MASKX(x)                        (((uint16_t)(((uint16_t)(x)) << PWM_MASK_MASKX_SHIFT)) & PWM_MASK_MASKX_MASK)

#define PWM_MASK_MASKB_MASK                      (0x70U)
#define PWM_MASK_MASKB_SHIFT                     (4U)
/*! MASKB - PWM_B Masks */
#define PWM_MASK_MASKB(x)                        (((uint16_t)(((uint16_t)(x)) << PWM_MASK_MASKB_SHIFT)) & PWM_MASK_MASKB_MASK)

#define PWM_MASK_MASKA_MASK                      (0x700U)
#define PWM_MASK_MASKA_SHIFT                     (8U)
/*! MASKA - PWM_A Masks */
#define PWM_MASK_MASKA(x)                        (((uint16_t)(((uint16_t)(x)) << PWM_MASK_MASKA_SHIFT)) & PWM_MASK_MASKA_MASK)

#define PWM_MASK_UPDATE_MASK_MASK                (0x7000U)
#define PWM_MASK_UPDATE_MASK_SHIFT               (12U)
/*! UPDATE_MASK - Update Mask Bits Immediately */
#define PWM_MASK_UPDATE_MASK(x)                  (((uint16_t)(((uint16_t)(x)) << PWM_MASK_UPDATE_MASK_SHIFT)) & PWM_MASK_UPDATE_MASK_MASK)
/*! @} */

/*! @name SWCOUT - Software Controlled Output Register */
/*! @{ */

#define PWM_SWCOUT_SM0OUT45_MASK                 (0x1U)
#define PWM_SWCOUT_SM0OUT45_SHIFT                (0U)
/*! SM0OUT45 - Submodule 0 Software Controlled Output 45
 *  0b0..A logic 0 is supplied to the deadtime generator of submodule 0 instead of PWM45.
 *  0b1..A logic 1 is supplied to the deadtime generator of submodule 0 instead of PWM45.
 */
#define PWM_SWCOUT_SM0OUT45(x)                   (((uint16_t)(((uint16_t)(x)) << PWM_SWCOUT_SM0OUT45_SHIFT)) & PWM_SWCOUT_SM0OUT45_MASK)

#define PWM_SWCOUT_SM0OUT23_MASK                 (0x2U)
#define PWM_SWCOUT_SM0OUT23_SHIFT                (1U)
/*! SM0OUT23 - Submodule 0 Software Controlled Output 23
 *  0b0..A logic 0 is supplied to the deadtime generator of submodule 0 instead of PWM23.
 *  0b1..A logic 1 is supplied to the deadtime generator of submodule 0 instead of PWM23.
 */
#define PWM_SWCOUT_SM0OUT23(x)                   (((uint16_t)(((uint16_t)(x)) << PWM_SWCOUT_SM0OUT23_SHIFT)) & PWM_SWCOUT_SM0OUT23_MASK)

#define PWM_SWCOUT_SM1OUT45_MASK                 (0x4U)
#define PWM_SWCOUT_SM1OUT45_SHIFT                (2U)
/*! SM1OUT45 - Submodule 1 Software Controlled Output 45
 *  0b0..A logic 0 is supplied to the deadtime generator of submodule 1 instead of PWM45.
 *  0b1..A logic 1 is supplied to the deadtime generator of submodule 1 instead of PWM45.
 */
#define PWM_SWCOUT_SM1OUT45(x)                   (((uint16_t)(((uint16_t)(x)) << PWM_SWCOUT_SM1OUT45_SHIFT)) & PWM_SWCOUT_SM1OUT45_MASK)

#define PWM_SWCOUT_SM1OUT23_MASK                 (0x8U)
#define PWM_SWCOUT_SM1OUT23_SHIFT                (3U)
/*! SM1OUT23 - Submodule 1 Software Controlled Output 23
 *  0b0..A logic 0 is supplied to the deadtime generator of submodule 1 instead of PWM23.
 *  0b1..A logic 1 is supplied to the deadtime generator of submodule 1 instead of PWM23.
 */
#define PWM_SWCOUT_SM1OUT23(x)                   (((uint16_t)(((uint16_t)(x)) << PWM_SWCOUT_SM1OUT23_SHIFT)) & PWM_SWCOUT_SM1OUT23_MASK)

#define PWM_SWCOUT_SM2OUT45_MASK                 (0x10U)
#define PWM_SWCOUT_SM2OUT45_SHIFT                (4U)
/*! SM2OUT45 - Submodule 2 Software Controlled Output 45
 *  0b0..A logic 0 is supplied to the deadtime generator of submodule 2 instead of PWM45.
 *  0b1..A logic 1 is supplied to the deadtime generator of submodule 2 instead of PWM45.
 */
#define PWM_SWCOUT_SM2OUT45(x)                   (((uint16_t)(((uint16_t)(x)) << PWM_SWCOUT_SM2OUT45_SHIFT)) & PWM_SWCOUT_SM2OUT45_MASK)

#define PWM_SWCOUT_SM2OUT23_MASK                 (0x20U)
#define PWM_SWCOUT_SM2OUT23_SHIFT                (5U)
/*! SM2OUT23 - Submodule 2 Software Controlled Output 23
 *  0b0..A logic 0 is supplied to the deadtime generator of submodule 2 instead of PWM23.
 *  0b1..A logic 1 is supplied to the deadtime generator of submodule 2 instead of PWM23.
 */
#define PWM_SWCOUT_SM2OUT23(x)                   (((uint16_t)(((uint16_t)(x)) << PWM_SWCOUT_SM2OUT23_SHIFT)) & PWM_SWCOUT_SM2OUT23_MASK)
/*! @} */

/*! @name DTSRCSEL - PWM Source Select Register */
/*! @{ */

#define PWM_DTSRCSEL_SM0SEL45_MASK               (0x3U)
#define PWM_DTSRCSEL_SM0SEL45_SHIFT              (0U)
/*! SM0SEL45 - Submodule 0 PWM45 Control Select
 *  0b00..Generated SM0PWM45 signal used by the deadtime logic.
 *  0b01..Inverted generated SM0PWM45 signal used by the deadtime logic.
 *  0b10..SWCOUT[SM0OUT45] used by the deadtime logic.
 *  0b11..Reserved
 */
#define PWM_DTSRCSEL_SM0SEL45(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_DTSRCSEL_SM0SEL45_SHIFT)) & PWM_DTSRCSEL_SM0SEL45_MASK)

#define PWM_DTSRCSEL_SM0SEL23_MASK               (0xCU)
#define PWM_DTSRCSEL_SM0SEL23_SHIFT              (2U)
/*! SM0SEL23 - Submodule 0 PWM23 Control Select
 *  0b00..Generated SM0PWM23 signal used by the deadtime logic.
 *  0b01..Inverted generated SM0PWM23 signal used by the deadtime logic.
 *  0b10..SWCOUT[SM0OUT23] used by the deadtime logic.
 *  0b11..PWM0_EXTA signal used by the deadtime logic.
 */
#define PWM_DTSRCSEL_SM0SEL23(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_DTSRCSEL_SM0SEL23_SHIFT)) & PWM_DTSRCSEL_SM0SEL23_MASK)

#define PWM_DTSRCSEL_SM1SEL45_MASK               (0x30U)
#define PWM_DTSRCSEL_SM1SEL45_SHIFT              (4U)
/*! SM1SEL45 - Submodule 1 PWM45 Control Select
 *  0b00..Generated SM1PWM45 signal used by the deadtime logic.
 *  0b01..Inverted generated SM1PWM45 signal used by the deadtime logic.
 *  0b10..SWCOUT[SM1OUT45] used by the deadtime logic.
 *  0b11..Reserved
 */
#define PWM_DTSRCSEL_SM1SEL45(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_DTSRCSEL_SM1SEL45_SHIFT)) & PWM_DTSRCSEL_SM1SEL45_MASK)

#define PWM_DTSRCSEL_SM1SEL23_MASK               (0xC0U)
#define PWM_DTSRCSEL_SM1SEL23_SHIFT              (6U)
/*! SM1SEL23 - Submodule 1 PWM23 Control Select
 *  0b00..Generated SM1PWM23 signal used by the deadtime logic.
 *  0b01..Inverted generated SM1PWM23 signal used by the deadtime logic.
 *  0b10..SWCOUT[SM1OUT23] used by the deadtime logic.
 *  0b11..PWM1_EXTA signal used by the deadtime logic.
 */
#define PWM_DTSRCSEL_SM1SEL23(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_DTSRCSEL_SM1SEL23_SHIFT)) & PWM_DTSRCSEL_SM1SEL23_MASK)

#define PWM_DTSRCSEL_SM2SEL45_MASK               (0x300U)
#define PWM_DTSRCSEL_SM2SEL45_SHIFT              (8U)
/*! SM2SEL45 - Submodule 2 PWM45 Control Select
 *  0b00..Generated SM2PWM45 signal used by the deadtime logic.
 *  0b01..Inverted generated SM2PWM45 signal used by the deadtime logic.
 *  0b10..SWCOUT[SM2OUT45] used by the deadtime logic.
 *  0b11..Reserved
 */
#define PWM_DTSRCSEL_SM2SEL45(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_DTSRCSEL_SM2SEL45_SHIFT)) & PWM_DTSRCSEL_SM2SEL45_MASK)

#define PWM_DTSRCSEL_SM2SEL23_MASK               (0xC00U)
#define PWM_DTSRCSEL_SM2SEL23_SHIFT              (10U)
/*! SM2SEL23 - Submodule 2 PWM23 Control Select
 *  0b00..Generated SM2PWM23 signal used by the deadtime logic.
 *  0b01..Inverted generated SM2PWM23 signal used by the deadtime logic.
 *  0b10..SWCOUT[SM2OUT23] used by the deadtime logic.
 *  0b11..PWM2_EXTA signal used by the deadtime logic.
 */
#define PWM_DTSRCSEL_SM2SEL23(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_DTSRCSEL_SM2SEL23_SHIFT)) & PWM_DTSRCSEL_SM2SEL23_MASK)
/*! @} */

/*! @name MCTRL - Master Control Register */
/*! @{ */

#define PWM_MCTRL_LDOK_MASK                      (0x7U)
#define PWM_MCTRL_LDOK_SHIFT                     (0U)
/*! LDOK - Load Okay
 *  0b000..Do not load new values.
 *  0b001..Load prescaler, modulus, and PWM values of the corresponding submodule.
 */
#define PWM_MCTRL_LDOK(x)                        (((uint16_t)(((uint16_t)(x)) << PWM_MCTRL_LDOK_SHIFT)) & PWM_MCTRL_LDOK_MASK)

#define PWM_MCTRL_CLDOK_MASK                     (0x70U)
#define PWM_MCTRL_CLDOK_SHIFT                    (4U)
/*! CLDOK - Clear Load Okay */
#define PWM_MCTRL_CLDOK(x)                       (((uint16_t)(((uint16_t)(x)) << PWM_MCTRL_CLDOK_SHIFT)) & PWM_MCTRL_CLDOK_MASK)

#define PWM_MCTRL_RUN_MASK                       (0x700U)
#define PWM_MCTRL_RUN_SHIFT                      (8U)
/*! RUN - Run
 *  0b000..PWM counter is stopped, but PWM outputs hold the current state.
 *  0b001..PWM counter is started in the corresponding submodule.
 */
#define PWM_MCTRL_RUN(x)                         (((uint16_t)(((uint16_t)(x)) << PWM_MCTRL_RUN_SHIFT)) & PWM_MCTRL_RUN_MASK)

#define PWM_MCTRL_IPOL_MASK                      (0x7000U)
#define PWM_MCTRL_IPOL_SHIFT                     (12U)
/*! IPOL - Current Polarity
 *  0b000..PWM23 is used to generate complementary PWM pair in the corresponding submodule.
 *  0b001..PWM45 is used to generate complementary PWM pair in the corresponding submodule.
 */
#define PWM_MCTRL_IPOL(x)                        (((uint16_t)(((uint16_t)(x)) << PWM_MCTRL_IPOL_SHIFT)) & PWM_MCTRL_IPOL_MASK)
/*! @} */

/*! @name MCTRL2 - Master Control 2 Register */
/*! @{ */

#define PWM_MCTRL2_WRPROT_MASK                   (0xCU)
#define PWM_MCTRL2_WRPROT_SHIFT                  (2U)
/*! WRPROT - Write protect
 *  0b00..Write protection off (default).
 *  0b01..Write protection on.
 *  0b10..Write protection off and locked until chip reset.
 *  0b11..Write protection on and locked until chip reset.
 */
#define PWM_MCTRL2_WRPROT(x)                     (((uint16_t)(((uint16_t)(x)) << PWM_MCTRL2_WRPROT_SHIFT)) & PWM_MCTRL2_WRPROT_MASK)

#define PWM_MCTRL2_STRETCH_CNT_PRSC_MASK         (0xC0U)
#define PWM_MCTRL2_STRETCH_CNT_PRSC_SHIFT        (6U)
/*! STRETCH_CNT_PRSC - Stretch IPBus clock count prescaler for mux0_trig/mux1_trig/out0_trig/out1_trig/pwma_trig/pwmb_trig
 *  0b00..Stretch count is zero, no stretch.
 *  0b01..Stretch mux0_trig/mux1_trig/out0_trig/out1_trig/pwma_trig/pwmb_trig for 2 IPBus clock period.
 *  0b10..Stretch mux0_trig/mux1_trig/out0_trig/out1_trig/pwma_trig/pwmb_trig for 4 IPBus clock period.
 *  0b11..Stretch mux0_trig/mux1_trig/out0_trig/out1_trig/pwma_trig/pwmb_trig for 8 IPBus clock period.
 */
#define PWM_MCTRL2_STRETCH_CNT_PRSC(x)           (((uint16_t)(((uint16_t)(x)) << PWM_MCTRL2_STRETCH_CNT_PRSC_SHIFT)) & PWM_MCTRL2_STRETCH_CNT_PRSC_MASK)
/*! @} */

/*! @name FCTRL - Fault Control Register */
/*! @{ */

#define PWM_FCTRL_FIE_MASK                       (0xFU)
#define PWM_FCTRL_FIE_SHIFT                      (0U)
/*! FIE - Fault Interrupt Enables
 *  0b0000..FAULTx CPU interrupt requests disabled.
 *  0b0001..FAULTx CPU interrupt requests enabled.
 */
#define PWM_FCTRL_FIE(x)                         (((uint16_t)(((uint16_t)(x)) << PWM_FCTRL_FIE_SHIFT)) & PWM_FCTRL_FIE_MASK)

#define PWM_FCTRL_FSAFE_MASK                     (0xF0U)
#define PWM_FCTRL_FSAFE_SHIFT                    (4U)
/*! FSAFE - Fault Safety Mode
 *  0b0000..Normal mode. PWM outputs disabled by this fault are not enabled until FSTS[FFLAGx] is clear at the
 *          start of a half cycle or full cycle depending on the states of FSTS[FHALF] and FSTS[FFULL] without regard
 *          to the state of FSTS[FFPINx]. If neither FHALF nor FFULL is set, then the fault condition cannot be
 *          cleared. The PWM outputs disabled by this fault input will not be re-enabled until the actual FAULTx input
 *          signal de-asserts since the fault input will combinationally disable the PWM outputs (as programmed in
 *          DISMAPn).
 *  0b0001..Safe mode. PWM outputs disabled by this fault are not enabled until FSTS[FFLAGx] is clear and
 *          FSTS[FFPINx] is clear at the start of a half cycle or full cycle depending on the states of FSTS[FHALF] and
 *          FSTS[FFULL]. If neither FHLAF nor FFULL is set, then the fault condition cannot be cleared.
 */
#define PWM_FCTRL_FSAFE(x)                       (((uint16_t)(((uint16_t)(x)) << PWM_FCTRL_FSAFE_SHIFT)) & PWM_FCTRL_FSAFE_MASK)

#define PWM_FCTRL_FAUTO_MASK                     (0xF00U)
#define PWM_FCTRL_FAUTO_SHIFT                    (8U)
/*! FAUTO - Automatic Fault Clearing
 *  0b0000..Manual fault clearing. PWM outputs disabled by this fault are not enabled until FSTS[FFLAGx] is clear
 *          at the start of a half cycle or full cycle depending on the states of FSTS[FHALF] and FSTS[FFULL]. If
 *          neither FFULL nor FHALF is set, then the fault condition cannot be cleared. This is further controlled
 *          by FCTRL[FSAFE].
 *  0b0001..Automatic fault clearing. PWM outputs disabled by this fault are enabled when FSTS[FFPINx] is clear at
 *          the start of a half cycle or full cycle depending on the states of FSTS[FHALF] and FSTS[FFULL] without
 *          regard to the state of FSTS[FFLAGx]. If neither FFULL nor FHALF is set, then the fault condition
 *          cannot be cleared.
 */
#define PWM_FCTRL_FAUTO(x)                       (((uint16_t)(((uint16_t)(x)) << PWM_FCTRL_FAUTO_SHIFT)) & PWM_FCTRL_FAUTO_MASK)

#define PWM_FCTRL_FLVL_MASK                      (0xF000U)
#define PWM_FCTRL_FLVL_SHIFT                     (12U)
/*! FLVL - Fault Level
 *  0b0000..A logic 0 on the fault input indicates a fault condition.
 *  0b0001..A logic 1 on the fault input indicates a fault condition.
 */
#define PWM_FCTRL_FLVL(x)                        (((uint16_t)(((uint16_t)(x)) << PWM_FCTRL_FLVL_SHIFT)) & PWM_FCTRL_FLVL_MASK)
/*! @} */

/*! @name FSTS - Fault Status Register */
/*! @{ */

#define PWM_FSTS_FFLAG_MASK                      (0xFU)
#define PWM_FSTS_FFLAG_SHIFT                     (0U)
/*! FFLAG - Fault Flags
 *  0b0000..No fault on the FAULTx pin.
 *  0b0001..Fault on the FAULTx pin.
 */
#define PWM_FSTS_FFLAG(x)                        (((uint16_t)(((uint16_t)(x)) << PWM_FSTS_FFLAG_SHIFT)) & PWM_FSTS_FFLAG_MASK)

#define PWM_FSTS_FFULL_MASK                      (0xF0U)
#define PWM_FSTS_FFULL_SHIFT                     (4U)
/*! FFULL - Full Cycle
 *  0b0000..PWM outputs are not re-enabled at the start of a full cycle
 *  0b0001..PWM outputs are re-enabled at the start of a full cycle
 */
#define PWM_FSTS_FFULL(x)                        (((uint16_t)(((uint16_t)(x)) << PWM_FSTS_FFULL_SHIFT)) & PWM_FSTS_FFULL_MASK)

#define PWM_FSTS_FFPIN_MASK                      (0xF00U)
#define PWM_FSTS_FFPIN_SHIFT                     (8U)
/*! FFPIN - Filtered Fault Pins */
#define PWM_FSTS_FFPIN(x)                        (((uint16_t)(((uint16_t)(x)) << PWM_FSTS_FFPIN_SHIFT)) & PWM_FSTS_FFPIN_MASK)

#define PWM_FSTS_FHALF_MASK                      (0xF000U)
#define PWM_FSTS_FHALF_SHIFT                     (12U)
/*! FHALF - Half Cycle Fault Recovery
 *  0b0000..PWM outputs are not re-enabled at the start of a half cycle.
 *  0b0001..PWM outputs are re-enabled at the start of a half cycle (as defined by VAL0).
 */
#define PWM_FSTS_FHALF(x)                        (((uint16_t)(((uint16_t)(x)) << PWM_FSTS_FHALF_SHIFT)) & PWM_FSTS_FHALF_MASK)
/*! @} */

/*! @name FFILT - Fault Filter Register */
/*! @{ */

#define PWM_FFILT_FILT_PER_MASK                  (0xFFU)
#define PWM_FFILT_FILT_PER_SHIFT                 (0U)
/*! FILT_PER - Fault Filter Period */
#define PWM_FFILT_FILT_PER(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_FFILT_FILT_PER_SHIFT)) & PWM_FFILT_FILT_PER_MASK)

#define PWM_FFILT_FILT_CNT_MASK                  (0x700U)
#define PWM_FFILT_FILT_CNT_SHIFT                 (8U)
/*! FILT_CNT - Fault Filter Count */
#define PWM_FFILT_FILT_CNT(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_FFILT_FILT_CNT_SHIFT)) & PWM_FFILT_FILT_CNT_MASK)

#define PWM_FFILT_GSTR_MASK                      (0x8000U)
#define PWM_FFILT_GSTR_SHIFT                     (15U)
/*! GSTR - Fault Glitch Stretch Enable
 *  0b0..Fault input glitch stretching is disabled.
 *  0b1..Input fault signals are stretched to at least 2 IPBus clock cycles.
 */
#define PWM_FFILT_GSTR(x)                        (((uint16_t)(((uint16_t)(x)) << PWM_FFILT_GSTR_SHIFT)) & PWM_FFILT_GSTR_MASK)
/*! @} */

/*! @name FTST - Fault Test Register */
/*! @{ */

#define PWM_FTST_FTEST_MASK                      (0x1U)
#define PWM_FTST_FTEST_SHIFT                     (0U)
/*! FTEST - Fault Test
 *  0b0..No fault
 *  0b1..Cause a simulated fault
 */
#define PWM_FTST_FTEST(x)                        (((uint16_t)(((uint16_t)(x)) << PWM_FTST_FTEST_SHIFT)) & PWM_FTST_FTEST_MASK)
/*! @} */

/*! @name FCTRL2 - Fault Control 2 Register */
/*! @{ */

#define PWM_FCTRL2_NOCOMB_MASK                   (0xFU)
#define PWM_FCTRL2_NOCOMB_SHIFT                  (0U)
/*! NOCOMB - No Combinational Path From Fault Input To PWM Output
 *  0b0000..There is a combinational link from the fault inputs to the PWM outputs. The fault inputs are combined
 *          with the filtered and latched fault signals to disable the PWM outputs.
 *  0b0001..The direct combinational path from the fault inputs to the PWM outputs is disabled and the filtered
 *          and latched fault signals are used to disable the PWM outputs.
 */
#define PWM_FCTRL2_NOCOMB(x)                     (((uint16_t)(((uint16_t)(x)) << PWM_FCTRL2_NOCOMB_SHIFT)) & PWM_FCTRL2_NOCOMB_MASK)
/*! @} */


/*!
 * @}
 */ /* end of group PWM_Register_Masks */


/*!
 * @}
 */ /* end of group PWM_Peripheral_Access_Layer */


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


#endif  /* PERI_PWM_H_ */

