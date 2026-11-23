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
**         CMSIS Peripheral Access Layer for EQDC
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
 * @file PERI_EQDC.h
 * @version 2.0
 * @date 2024-10-29
 * @brief CMSIS Peripheral Access Layer for EQDC
 *
 * CMSIS Peripheral Access Layer for EQDC
 */

#if !defined(PERI_EQDC_H_)
#define PERI_EQDC_H_                             /**< Symbol preventing repeated inclusion */

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
   -- EQDC Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup EQDC_Peripheral_Access_Layer EQDC Peripheral Access Layer
 * @{
 */

/** EQDC - Register Layout Typedef */
typedef struct {
  __IO uint16_t CTRL;                              /**< Control Register, offset: 0x0 */
  __IO uint16_t CTRL2;                             /**< Control 2 Register, offset: 0x2 */
  __IO uint16_t FILT;                              /**< Input Filter Register, offset: 0x4 */
  __I  uint16_t LASTEDGE;                          /**< Last Edge Time Register, offset: 0x6 */
  __I  uint16_t POSDPER;                           /**< Position Difference Period Counter Register, offset: 0x8 */
  __I  uint16_t POSDPERBFR;                        /**< Position Difference Period Buffer Register, offset: 0xA */
  __IO uint16_t UPOS;                              /**< Upper Position Counter Register, offset: 0xC */
  __IO uint16_t LPOS;                              /**< Lower Position Counter Register, offset: 0xE */
  __IO uint16_t POSD;                              /**< Position Difference Counter Register, offset: 0x10 */
  __I  uint16_t POSDH;                             /**< Position Difference Hold Register, offset: 0x12 */
  __I  uint16_t UPOSH;                             /**< Upper Position Hold Register, offset: 0x14 */
  __I  uint16_t LPOSH;                             /**< Lower Position Hold Register, offset: 0x16 */
  __I  uint16_t LASTEDGEH;                         /**< Last Edge Time Hold Register, offset: 0x18 */
  __I  uint16_t POSDPERH;                          /**< Position Difference Period Hold Register, offset: 0x1A */
  __I  uint16_t REVH;                              /**< Revolution Hold Register, offset: 0x1C */
  __IO uint16_t REV;                               /**< Revolution Counter Register, offset: 0x1E */
  __IO uint16_t UINIT;                             /**< Upper Initialization Register, offset: 0x20 */
  __IO uint16_t LINIT;                             /**< Lower Initialization Register, offset: 0x22 */
  __IO uint16_t UMOD;                              /**< Upper Modulus Register, offset: 0x24 */
  __IO uint16_t LMOD;                              /**< Lower Modulus Register, offset: 0x26 */
  __IO uint16_t UCOMP0;                            /**< Upper Position Compare Register 0, offset: 0x28 */
  __IO uint16_t LCOMP0;                            /**< Lower Position Compare Register 0, offset: 0x2A */
  union {                                          /* offset: 0x2C */
    __O  uint16_t UCOMP1;                            /**< Upper Position Compare 1, offset: 0x2C */
    __I  uint16_t UPOSH1;                            /**< Upper Position Holder Register 1, offset: 0x2C */
  };
  union {                                          /* offset: 0x2E */
    __O  uint16_t LCOMP1;                            /**< Lower Position Compare 1, offset: 0x2E */
    __I  uint16_t LPOSH1;                            /**< Lower Position Holder Register 1, offset: 0x2E */
  };
  union {                                          /* offset: 0x30 */
    __O  uint16_t UCOMP2;                            /**< Upper Position Compare 2, offset: 0x30 */
    __I  uint16_t UPOSH2;                            /**< Upper Position Holder Register 3, offset: 0x30 */
  };
  union {                                          /* offset: 0x32 */
    __O  uint16_t LCOMP2;                            /**< Lower Position Compare 2, offset: 0x32 */
    __I  uint16_t LPOSH2;                            /**< Lower Position Holder Register 2, offset: 0x32 */
  };
  union {                                          /* offset: 0x34 */
    __O  uint16_t UCOMP3;                            /**< Upper Position Compare 3, offset: 0x34 */
    __I  uint16_t UPOSH3;                            /**< Upper Position Holder Register 3, offset: 0x34 */
  };
  union {                                          /* offset: 0x36 */
    __O  uint16_t LCOMP3;                            /**< Lower Position Compare 3, offset: 0x36 */
    __I  uint16_t LPOSH3;                            /**< Lower Position Holder Register 3, offset: 0x36 */
  };
  __IO uint16_t INTCTRL;                           /**< Interrupt Control Register, offset: 0x38 */
  __IO uint16_t WTR;                               /**< Watchdog Timeout Register, offset: 0x3A */
  __IO uint16_t IMR;                               /**< Input Monitor Register, offset: 0x3C */
  __IO uint16_t TST;                               /**< Test Register, offset: 0x3E */
       uint8_t RESERVED_0[16];
  __I  uint16_t UVERID;                            /**< Upper VERID, offset: 0x50 */
  __I  uint16_t LVERID;                            /**< Lower VERID, offset: 0x52 */
} EQDC_Type;

/* ----------------------------------------------------------------------------
   -- EQDC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup EQDC_Register_Masks EQDC Register Masks
 * @{
 */

/*! @name CTRL - Control Register */
/*! @{ */

#define EQDC_CTRL_LDOK_MASK                      (0x1U)
#define EQDC_CTRL_LDOK_SHIFT                     (0U)
/*! LDOK - Load Okay
 *  0b0..No loading action taken. Users can write new values to buffered registers (writing into outer-set of these buffered registers)
 *  0b1..Outer-set values are ready to be loaded into inner-set and take effect. The loading time point depends on CTRL2[LDMOD].
 */
#define EQDC_CTRL_LDOK(x)                        (((uint16_t)(((uint16_t)(x)) << EQDC_CTRL_LDOK_SHIFT)) & EQDC_CTRL_LDOK_MASK)

#define EQDC_CTRL_DMAEN_MASK                     (0x2U)
#define EQDC_CTRL_DMAEN_SHIFT                    (1U)
/*! DMAEN - DMA Enable
 *  0b0..DMA is disabled
 *  0b1..DMA is enabled. DMA request asserts automatically when the values in the outer-set of buffered compare
 *       registers (UCOMP0/LCOMP0;UCOMP1/LCOMP1;UCOMP2/LCOMP2;UCOMP3/LCOMP3), initial registers(UINIT/LINIT) and
 *       modulus registers (UMOD/LMOD) are loaded into the inner-set of buffer and then LDOK is cleared automatically.
 *       After the completion of this DMA transfer, LDOK is set automatically, it ensures outer-set values can be
 *       loaded into inner-set which in turn triggers DMA again.
 */
#define EQDC_CTRL_DMAEN(x)                       (((uint16_t)(((uint16_t)(x)) << EQDC_CTRL_DMAEN_SHIFT)) & EQDC_CTRL_DMAEN_MASK)

#define EQDC_CTRL_WDE_MASK                       (0x4U)
#define EQDC_CTRL_WDE_SHIFT                      (2U)
/*! WDE - Watchdog Enable
 *  0b0..Disabled
 *  0b1..Enabled
 */
#define EQDC_CTRL_WDE(x)                         (((uint16_t)(((uint16_t)(x)) << EQDC_CTRL_WDE_SHIFT)) & EQDC_CTRL_WDE_MASK)

#define EQDC_CTRL_WDIE_MASK                      (0x8U)
#define EQDC_CTRL_WDIE_SHIFT                     (3U)
/*! WDIE - Watchdog Timeout Interrupt Enable
 *  0b0..Disabled
 *  0b1..Enabled
 */
#define EQDC_CTRL_WDIE(x)                        (((uint16_t)(((uint16_t)(x)) << EQDC_CTRL_WDIE_SHIFT)) & EQDC_CTRL_WDIE_MASK)

#define EQDC_CTRL_WDIRQ_MASK                     (0x10U)
#define EQDC_CTRL_WDIRQ_SHIFT                    (4U)
/*! WDIRQ - Watchdog Timeout Interrupt Request
 *  0b0..No Watchdog timeout interrupt has occurred
 *  0b1..Watchdog timeout interrupt has occurred
 */
#define EQDC_CTRL_WDIRQ(x)                       (((uint16_t)(((uint16_t)(x)) << EQDC_CTRL_WDIRQ_SHIFT)) & EQDC_CTRL_WDIRQ_MASK)

#define EQDC_CTRL_XNE_MASK                       (0x20U)
#define EQDC_CTRL_XNE_SHIFT                      (5U)
/*! XNE - Select Positive/Negative Edge of INDEX/PRESET Pulse
 *  0b0..Use positive edge of INDEX/PRESET pulse
 *  0b1..Use negative edge of INDEX/PRESET pulse
 */
#define EQDC_CTRL_XNE(x)                         (((uint16_t)(((uint16_t)(x)) << EQDC_CTRL_XNE_SHIFT)) & EQDC_CTRL_XNE_MASK)

#define EQDC_CTRL_XIP_MASK                       (0x40U)
#define EQDC_CTRL_XIP_SHIFT                      (6U)
/*! XIP - INDEX Triggered Initialization of Position Counters UPOS and LPOS
 *  0b0..INDEX pulse does not initialize the position counter
 *  0b1..INDEX pulse initializes the position counter
 */
#define EQDC_CTRL_XIP(x)                         (((uint16_t)(((uint16_t)(x)) << EQDC_CTRL_XIP_SHIFT)) & EQDC_CTRL_XIP_MASK)

#define EQDC_CTRL_XIE_MASK                       (0x80U)
#define EQDC_CTRL_XIE_SHIFT                      (7U)
/*! XIE - INDEX/PRESET Pulse Interrupt Enable
 *  0b0..Disabled
 *  0b1..Enabled
 */
#define EQDC_CTRL_XIE(x)                         (((uint16_t)(((uint16_t)(x)) << EQDC_CTRL_XIE_SHIFT)) & EQDC_CTRL_XIE_MASK)

#define EQDC_CTRL_XIRQ_MASK                      (0x100U)
#define EQDC_CTRL_XIRQ_SHIFT                     (8U)
/*! XIRQ - INDEX/PRESET Pulse Interrupt Request
 *  0b0..INDEX/PRESET pulse has not occurred
 *  0b1..INDEX/PRESET pulse has occurred
 */
#define EQDC_CTRL_XIRQ(x)                        (((uint16_t)(((uint16_t)(x)) << EQDC_CTRL_XIRQ_SHIFT)) & EQDC_CTRL_XIRQ_MASK)

#define EQDC_CTRL_PH1_MASK                       (0x200U)
#define EQDC_CTRL_PH1_SHIFT                      (9U)
/*! PH1 - Enable Single Phase Mode
 *  0b0..Standard quadrature decoder, where PHASEA and PHASEB represent a two-phase quadrature signal.
 *  0b1..Single phase mode, bypass the quadrature decoder, refer to CTRL2[CMODE] description
 */
#define EQDC_CTRL_PH1(x)                         (((uint16_t)(((uint16_t)(x)) << EQDC_CTRL_PH1_SHIFT)) & EQDC_CTRL_PH1_MASK)

#define EQDC_CTRL_REV_MASK                       (0x400U)
#define EQDC_CTRL_REV_SHIFT                      (10U)
/*! REV - Enable Reverse Direction Counting
 *  0b0..Count normally and the position counter initialization uses upper/lower initialization register UINIT/LINIT
 *  0b1..Count in the reverse direction and the position counter initialization uses upper/lower modulus register UMOD/LMOD
 */
#define EQDC_CTRL_REV(x)                         (((uint16_t)(((uint16_t)(x)) << EQDC_CTRL_REV_SHIFT)) & EQDC_CTRL_REV_MASK)

#define EQDC_CTRL_SWIP_MASK                      (0x800U)
#define EQDC_CTRL_SWIP_SHIFT                     (11U)
/*! SWIP - Software-Triggered Initialization of Position Counters UPOS and LPOS
 *  0b0..No action
 *  0b1..Initialize position counter
 */
#define EQDC_CTRL_SWIP(x)                        (((uint16_t)(((uint16_t)(x)) << EQDC_CTRL_SWIP_SHIFT)) & EQDC_CTRL_SWIP_MASK)

#define EQDC_CTRL_HNE_MASK                       (0x1000U)
#define EQDC_CTRL_HNE_SHIFT                      (12U)
/*! HNE - Use Negative Edge of HOME/ENABLE Input
 *  0b0..When CTRL[OPMODE] = 0,use HOME positive edge to trigger initialization of position counters. When
 *       CTRL[OPMODE] = 1,use ENABLE high level to enable POS/POSD/WDG/REV counters
 *  0b1..When CTRL[OPMODE] = 0,use HOME negative edge to trigger initialization of position counters. When
 *       CTRL[OPMODE] = 1,use ENABLE low level to enable POS/POSD/WDG/REV counters
 */
#define EQDC_CTRL_HNE(x)                         (((uint16_t)(((uint16_t)(x)) << EQDC_CTRL_HNE_SHIFT)) & EQDC_CTRL_HNE_MASK)

#define EQDC_CTRL_HIP_MASK                       (0x2000U)
#define EQDC_CTRL_HIP_SHIFT                      (13U)
/*! HIP - Enable HOME to Initialize Position Counter UPOS/LPOS
 *  0b0..No action
 *  0b1..HOME signal initializes the position counter
 */
#define EQDC_CTRL_HIP(x)                         (((uint16_t)(((uint16_t)(x)) << EQDC_CTRL_HIP_SHIFT)) & EQDC_CTRL_HIP_MASK)

#define EQDC_CTRL_HIE_MASK                       (0x4000U)
#define EQDC_CTRL_HIE_SHIFT                      (14U)
/*! HIE - HOME/ENABLE Interrupt Enable
 *  0b0..Disabled
 *  0b1..Enabled
 */
#define EQDC_CTRL_HIE(x)                         (((uint16_t)(((uint16_t)(x)) << EQDC_CTRL_HIE_SHIFT)) & EQDC_CTRL_HIE_MASK)

#define EQDC_CTRL_HIRQ_MASK                      (0x8000U)
#define EQDC_CTRL_HIRQ_SHIFT                     (15U)
/*! HIRQ - HOME/ENABLE Signal Transition Interrupt Request
 *  0b0..No transition on the HOME/ENABLE signal has occurred
 *  0b1..A transition on the HOME/ENABLE signal has occurred
 */
#define EQDC_CTRL_HIRQ(x)                        (((uint16_t)(((uint16_t)(x)) << EQDC_CTRL_HIRQ_SHIFT)) & EQDC_CTRL_HIRQ_MASK)
/*! @} */

/*! @name CTRL2 - Control 2 Register */
/*! @{ */

#define EQDC_CTRL2_UPDHLD_MASK                   (0x1U)
#define EQDC_CTRL2_UPDHLD_SHIFT                  (0U)
/*! UPDHLD - Update Hold Registers */
#define EQDC_CTRL2_UPDHLD(x)                     (((uint16_t)(((uint16_t)(x)) << EQDC_CTRL2_UPDHLD_SHIFT)) & EQDC_CTRL2_UPDHLD_MASK)

#define EQDC_CTRL2_UPDPOS_MASK                   (0x2U)
#define EQDC_CTRL2_UPDPOS_SHIFT                  (1U)
/*! UPDPOS - Update Position Registers */
#define EQDC_CTRL2_UPDPOS(x)                     (((uint16_t)(((uint16_t)(x)) << EQDC_CTRL2_UPDPOS_SHIFT)) & EQDC_CTRL2_UPDPOS_MASK)

#define EQDC_CTRL2_OPMODE_MASK                   (0x4U)
#define EQDC_CTRL2_OPMODE_SHIFT                  (2U)
/*! OPMODE - Operation Mode Select
 *  0b0..Decode Mode: Input nodes INDEX/PRESET and HOME/ENABLE are assigned to function of INDEX and HOME.
 *  0b1..Count Mode: Input nodes INDEX/PRESET and HOME/ENABLE are assigned to functions of PRESET and ENABLE. In
 *       this mode: (1)only when ENABLE=1, all counters (position/position difference/revolution/watchdog) can run,
 *       when ENABLE=0, all counters (position/position difference/revolution/watchdog) can't run. (2) the rising
 *       edge of PRESET input can initialize position/revolution/watchdog counters (position counter initialization
 *       also need referring to bit CTRL[REV]).
 */
#define EQDC_CTRL2_OPMODE(x)                     (((uint16_t)(((uint16_t)(x)) << EQDC_CTRL2_OPMODE_SHIFT)) & EQDC_CTRL2_OPMODE_MASK)

#define EQDC_CTRL2_LDMOD_MASK                    (0x8U)
#define EQDC_CTRL2_LDMOD_SHIFT                   (3U)
/*! LDMOD - Buffered Register Load (Update) Mode Select
 *  0b0..Buffered registers are loaded and take effect immediately upon CTRL[LDOK] is set.
 *  0b1..Buffered registers are loaded and take effect at the next roll-over or roll-under if CTRL[LDOK] is set.
 */
#define EQDC_CTRL2_LDMOD(x)                      (((uint16_t)(((uint16_t)(x)) << EQDC_CTRL2_LDMOD_SHIFT)) & EQDC_CTRL2_LDMOD_MASK)

#define EQDC_CTRL2_REVMOD_MASK                   (0x100U)
#define EQDC_CTRL2_REVMOD_SHIFT                  (8U)
/*! REVMOD - Revolution Counter Modulus Enable
 *  0b0..Use INDEX pulse to increment/decrement revolution counter (REV)
 *  0b1..Use modulus counting roll-over/under to increment/decrement revolution counter (REV)
 */
#define EQDC_CTRL2_REVMOD(x)                     (((uint16_t)(((uint16_t)(x)) << EQDC_CTRL2_REVMOD_SHIFT)) & EQDC_CTRL2_REVMOD_MASK)

#define EQDC_CTRL2_OUTCTL_MASK                   (0x200U)
#define EQDC_CTRL2_OUTCTL_SHIFT                  (9U)
/*! OUTCTL - Output Control
 *  0b0..POS_MATCH[x](x range is 0-3) is asserted when the Position Counter is equal to according compare value
 *       (UCOMPx/LCOMPx)(x range is 0-3), and de-asserted when the Position Counter not equal to the compare value
 *       (UCOMPx/LCOMPx)(x range is 0-3)
 *  0b1..All POS_MATCH[x](x range is 0-3) are asserted a pulse, when the UPOS, LPOS, REV, or POSD registers are read
 */
#define EQDC_CTRL2_OUTCTL(x)                     (((uint16_t)(((uint16_t)(x)) << EQDC_CTRL2_OUTCTL_SHIFT)) & EQDC_CTRL2_OUTCTL_MASK)

#define EQDC_CTRL2_PMEN_MASK                     (0x400U)
#define EQDC_CTRL2_PMEN_SHIFT                    (10U)
/*! PMEN - Period measurement function enable
 *  0b0..Period measurement functions are not used. POSD is loaded to POSDH and then cleared whenever POSD, UPOS, LPOS or REV is read.
 *  0b1..Period measurement functions are used. POSD is loaded into POSDH and then cleared only when POSD is read.
 */
#define EQDC_CTRL2_PMEN(x)                       (((uint16_t)(((uint16_t)(x)) << EQDC_CTRL2_PMEN_SHIFT)) & EQDC_CTRL2_PMEN_MASK)

#define EQDC_CTRL2_EMIP_MASK                     (0x800U)
#define EQDC_CTRL2_EMIP_SHIFT                    (11U)
/*! EMIP - Enables/disables the position counter to be initialized by Index Event Edge Mark
 *  0b0..disables the position counter to be initialized by Index Event Edge Mark
 *  0b1..enables the position counter to be initialized by Index Event Edge Mark.
 */
#define EQDC_CTRL2_EMIP(x)                       (((uint16_t)(((uint16_t)(x)) << EQDC_CTRL2_EMIP_SHIFT)) & EQDC_CTRL2_EMIP_MASK)

#define EQDC_CTRL2_INITPOS_MASK                  (0x1000U)
#define EQDC_CTRL2_INITPOS_SHIFT                 (12U)
/*! INITPOS - Initial Position Register
 *  0b0..Don't initialize position counter on rising edge of TRIGGER
 *  0b1..Initialize position counter on rising edge of TRIGGER
 */
#define EQDC_CTRL2_INITPOS(x)                    (((uint16_t)(((uint16_t)(x)) << EQDC_CTRL2_INITPOS_SHIFT)) & EQDC_CTRL2_INITPOS_MASK)

#define EQDC_CTRL2_ONCE_MASK                     (0x2000U)
#define EQDC_CTRL2_ONCE_SHIFT                    (13U)
/*! ONCE - Count Once
 *  0b0..Position counter counts repeatedly
 *  0b1..Position counter counts until roll-over or roll-under, then stop.
 */
#define EQDC_CTRL2_ONCE(x)                       (((uint16_t)(((uint16_t)(x)) << EQDC_CTRL2_ONCE_SHIFT)) & EQDC_CTRL2_ONCE_MASK)

#define EQDC_CTRL2_CMODE_MASK                    (0xC000U)
#define EQDC_CTRL2_CMODE_SHIFT                   (14U)
/*! CMODE - Counting Mode */
#define EQDC_CTRL2_CMODE(x)                      (((uint16_t)(((uint16_t)(x)) << EQDC_CTRL2_CMODE_SHIFT)) & EQDC_CTRL2_CMODE_MASK)
/*! @} */

/*! @name FILT - Input Filter Register */
/*! @{ */

#define EQDC_FILT_FILT_PER_MASK                  (0xFFU)
#define EQDC_FILT_FILT_PER_SHIFT                 (0U)
/*! FILT_PER - Input Filter Sample Period */
#define EQDC_FILT_FILT_PER(x)                    (((uint16_t)(((uint16_t)(x)) << EQDC_FILT_FILT_PER_SHIFT)) & EQDC_FILT_FILT_PER_MASK)

#define EQDC_FILT_FILT_CNT_MASK                  (0x700U)
#define EQDC_FILT_FILT_CNT_SHIFT                 (8U)
/*! FILT_CNT - Input Filter Sample Count */
#define EQDC_FILT_FILT_CNT(x)                    (((uint16_t)(((uint16_t)(x)) << EQDC_FILT_FILT_CNT_SHIFT)) & EQDC_FILT_FILT_CNT_MASK)

#define EQDC_FILT_FILT_CS_MASK                   (0x800U)
#define EQDC_FILT_FILT_CS_SHIFT                  (11U)
/*! FILT_CS - Filter Clock Source selection
 *  0b0..Peripheral Clock
 *  0b1..Prescaled peripheral clock by PRSC
 */
#define EQDC_FILT_FILT_CS(x)                     (((uint16_t)(((uint16_t)(x)) << EQDC_FILT_FILT_CS_SHIFT)) & EQDC_FILT_FILT_CS_MASK)

#define EQDC_FILT_PRSC_MASK                      (0xF000U)
#define EQDC_FILT_PRSC_SHIFT                     (12U)
/*! PRSC - Prescaler */
#define EQDC_FILT_PRSC(x)                        (((uint16_t)(((uint16_t)(x)) << EQDC_FILT_PRSC_SHIFT)) & EQDC_FILT_PRSC_MASK)
/*! @} */

/*! @name LASTEDGE - Last Edge Time Register */
/*! @{ */

#define EQDC_LASTEDGE_LASTEDGE_MASK              (0xFFFFU)
#define EQDC_LASTEDGE_LASTEDGE_SHIFT             (0U)
/*! LASTEDGE - Last Edge Time Counter */
#define EQDC_LASTEDGE_LASTEDGE(x)                (((uint16_t)(((uint16_t)(x)) << EQDC_LASTEDGE_LASTEDGE_SHIFT)) & EQDC_LASTEDGE_LASTEDGE_MASK)
/*! @} */

/*! @name POSDPER - Position Difference Period Counter Register */
/*! @{ */

#define EQDC_POSDPER_POSDPER_MASK                (0xFFFFU)
#define EQDC_POSDPER_POSDPER_SHIFT               (0U)
/*! POSDPER - Position difference period */
#define EQDC_POSDPER_POSDPER(x)                  (((uint16_t)(((uint16_t)(x)) << EQDC_POSDPER_POSDPER_SHIFT)) & EQDC_POSDPER_POSDPER_MASK)
/*! @} */

/*! @name POSDPERBFR - Position Difference Period Buffer Register */
/*! @{ */

#define EQDC_POSDPERBFR_POSDPERBFR_MASK          (0xFFFFU)
#define EQDC_POSDPERBFR_POSDPERBFR_SHIFT         (0U)
/*! POSDPERBFR - Position difference period buffer */
#define EQDC_POSDPERBFR_POSDPERBFR(x)            (((uint16_t)(((uint16_t)(x)) << EQDC_POSDPERBFR_POSDPERBFR_SHIFT)) & EQDC_POSDPERBFR_POSDPERBFR_MASK)
/*! @} */

/*! @name UPOS - Upper Position Counter Register */
/*! @{ */

#define EQDC_UPOS_POS_MASK                       (0xFFFFU)
#define EQDC_UPOS_POS_SHIFT                      (0U)
/*! POS - POS */
#define EQDC_UPOS_POS(x)                         (((uint16_t)(((uint16_t)(x)) << EQDC_UPOS_POS_SHIFT)) & EQDC_UPOS_POS_MASK)
/*! @} */

/*! @name LPOS - Lower Position Counter Register */
/*! @{ */

#define EQDC_LPOS_POS_MASK                       (0xFFFFU)
#define EQDC_LPOS_POS_SHIFT                      (0U)
/*! POS - POS */
#define EQDC_LPOS_POS(x)                         (((uint16_t)(((uint16_t)(x)) << EQDC_LPOS_POS_SHIFT)) & EQDC_LPOS_POS_MASK)
/*! @} */

/*! @name POSD - Position Difference Counter Register */
/*! @{ */

#define EQDC_POSD_POSD_MASK                      (0xFFFFU)
#define EQDC_POSD_POSD_SHIFT                     (0U)
/*! POSD - POSD */
#define EQDC_POSD_POSD(x)                        (((uint16_t)(((uint16_t)(x)) << EQDC_POSD_POSD_SHIFT)) & EQDC_POSD_POSD_MASK)
/*! @} */

/*! @name POSDH - Position Difference Hold Register */
/*! @{ */

#define EQDC_POSDH_POSDH_MASK                    (0xFFFFU)
#define EQDC_POSDH_POSDH_SHIFT                   (0U)
/*! POSDH - POSDH */
#define EQDC_POSDH_POSDH(x)                      (((uint16_t)(((uint16_t)(x)) << EQDC_POSDH_POSDH_SHIFT)) & EQDC_POSDH_POSDH_MASK)
/*! @} */

/*! @name UPOSH - Upper Position Hold Register */
/*! @{ */

#define EQDC_UPOSH_POSH_MASK                     (0xFFFFU)
#define EQDC_UPOSH_POSH_SHIFT                    (0U)
/*! POSH - POSH */
#define EQDC_UPOSH_POSH(x)                       (((uint16_t)(((uint16_t)(x)) << EQDC_UPOSH_POSH_SHIFT)) & EQDC_UPOSH_POSH_MASK)
/*! @} */

/*! @name LPOSH - Lower Position Hold Register */
/*! @{ */

#define EQDC_LPOSH_LPOSH_MASK                    (0xFFFFU)
#define EQDC_LPOSH_LPOSH_SHIFT                   (0U)
/*! LPOSH - POSH */
#define EQDC_LPOSH_LPOSH(x)                      (((uint16_t)(((uint16_t)(x)) << EQDC_LPOSH_LPOSH_SHIFT)) & EQDC_LPOSH_LPOSH_MASK)
/*! @} */

/*! @name LASTEDGEH - Last Edge Time Hold Register */
/*! @{ */

#define EQDC_LASTEDGEH_LASTEDGEH_MASK            (0xFFFFU)
#define EQDC_LASTEDGEH_LASTEDGEH_SHIFT           (0U)
/*! LASTEDGEH - Last Edge Time Hold */
#define EQDC_LASTEDGEH_LASTEDGEH(x)              (((uint16_t)(((uint16_t)(x)) << EQDC_LASTEDGEH_LASTEDGEH_SHIFT)) & EQDC_LASTEDGEH_LASTEDGEH_MASK)
/*! @} */

/*! @name POSDPERH - Position Difference Period Hold Register */
/*! @{ */

#define EQDC_POSDPERH_POSDPERH_MASK              (0xFFFFU)
#define EQDC_POSDPERH_POSDPERH_SHIFT             (0U)
/*! POSDPERH - Position difference period hold */
#define EQDC_POSDPERH_POSDPERH(x)                (((uint16_t)(((uint16_t)(x)) << EQDC_POSDPERH_POSDPERH_SHIFT)) & EQDC_POSDPERH_POSDPERH_MASK)
/*! @} */

/*! @name REVH - Revolution Hold Register */
/*! @{ */

#define EQDC_REVH_REVH_MASK                      (0xFFFFU)
#define EQDC_REVH_REVH_SHIFT                     (0U)
/*! REVH - REVH */
#define EQDC_REVH_REVH(x)                        (((uint16_t)(((uint16_t)(x)) << EQDC_REVH_REVH_SHIFT)) & EQDC_REVH_REVH_MASK)
/*! @} */

/*! @name REV - Revolution Counter Register */
/*! @{ */

#define EQDC_REV_REV_MASK                        (0xFFFFU)
#define EQDC_REV_REV_SHIFT                       (0U)
/*! REV - REV */
#define EQDC_REV_REV(x)                          (((uint16_t)(((uint16_t)(x)) << EQDC_REV_REV_SHIFT)) & EQDC_REV_REV_MASK)
/*! @} */

/*! @name UINIT - Upper Initialization Register */
/*! @{ */

#define EQDC_UINIT_INIT_MASK                     (0xFFFFU)
#define EQDC_UINIT_INIT_SHIFT                    (0U)
/*! INIT - INIT */
#define EQDC_UINIT_INIT(x)                       (((uint16_t)(((uint16_t)(x)) << EQDC_UINIT_INIT_SHIFT)) & EQDC_UINIT_INIT_MASK)
/*! @} */

/*! @name LINIT - Lower Initialization Register */
/*! @{ */

#define EQDC_LINIT_INIT_MASK                     (0xFFFFU)
#define EQDC_LINIT_INIT_SHIFT                    (0U)
/*! INIT - INIT */
#define EQDC_LINIT_INIT(x)                       (((uint16_t)(((uint16_t)(x)) << EQDC_LINIT_INIT_SHIFT)) & EQDC_LINIT_INIT_MASK)
/*! @} */

/*! @name UMOD - Upper Modulus Register */
/*! @{ */

#define EQDC_UMOD_MOD_MASK                       (0xFFFFU)
#define EQDC_UMOD_MOD_SHIFT                      (0U)
/*! MOD - MOD */
#define EQDC_UMOD_MOD(x)                         (((uint16_t)(((uint16_t)(x)) << EQDC_UMOD_MOD_SHIFT)) & EQDC_UMOD_MOD_MASK)
/*! @} */

/*! @name LMOD - Lower Modulus Register */
/*! @{ */

#define EQDC_LMOD_MOD_MASK                       (0xFFFFU)
#define EQDC_LMOD_MOD_SHIFT                      (0U)
/*! MOD - MOD */
#define EQDC_LMOD_MOD(x)                         (((uint16_t)(((uint16_t)(x)) << EQDC_LMOD_MOD_SHIFT)) & EQDC_LMOD_MOD_MASK)
/*! @} */

/*! @name UCOMP0 - Upper Position Compare Register 0 */
/*! @{ */

#define EQDC_UCOMP0_UCOMP0_MASK                  (0xFFFFU)
#define EQDC_UCOMP0_UCOMP0_SHIFT                 (0U)
/*! UCOMP0 - UCOMP0 */
#define EQDC_UCOMP0_UCOMP0(x)                    (((uint16_t)(((uint16_t)(x)) << EQDC_UCOMP0_UCOMP0_SHIFT)) & EQDC_UCOMP0_UCOMP0_MASK)
/*! @} */

/*! @name LCOMP0 - Lower Position Compare Register 0 */
/*! @{ */

#define EQDC_LCOMP0_LCOMP0_MASK                  (0xFFFFU)
#define EQDC_LCOMP0_LCOMP0_SHIFT                 (0U)
/*! LCOMP0 - LCOMP0 */
#define EQDC_LCOMP0_LCOMP0(x)                    (((uint16_t)(((uint16_t)(x)) << EQDC_LCOMP0_LCOMP0_SHIFT)) & EQDC_LCOMP0_LCOMP0_MASK)
/*! @} */

/*! @name UCOMP1 - Upper Position Compare 1 */
/*! @{ */

#define EQDC_UCOMP1_UCOMP1_MASK                  (0xFFFFU)
#define EQDC_UCOMP1_UCOMP1_SHIFT                 (0U)
/*! UCOMP1 - UCOMP1 */
#define EQDC_UCOMP1_UCOMP1(x)                    (((uint16_t)(((uint16_t)(x)) << EQDC_UCOMP1_UCOMP1_SHIFT)) & EQDC_UCOMP1_UCOMP1_MASK)
/*! @} */

/*! @name UPOSH1 - Upper Position Holder Register 1 */
/*! @{ */

#define EQDC_UPOSH1_UPOSH1_MASK                  (0xFFFFU)
#define EQDC_UPOSH1_UPOSH1_SHIFT                 (0U)
/*! UPOSH1 - UPOSH1 */
#define EQDC_UPOSH1_UPOSH1(x)                    (((uint16_t)(((uint16_t)(x)) << EQDC_UPOSH1_UPOSH1_SHIFT)) & EQDC_UPOSH1_UPOSH1_MASK)
/*! @} */

/*! @name LCOMP1 - Lower Position Compare 1 */
/*! @{ */

#define EQDC_LCOMP1_LCOMP1_MASK                  (0xFFFFU)
#define EQDC_LCOMP1_LCOMP1_SHIFT                 (0U)
/*! LCOMP1 - LCOMP1 */
#define EQDC_LCOMP1_LCOMP1(x)                    (((uint16_t)(((uint16_t)(x)) << EQDC_LCOMP1_LCOMP1_SHIFT)) & EQDC_LCOMP1_LCOMP1_MASK)
/*! @} */

/*! @name LPOSH1 - Lower Position Holder Register 1 */
/*! @{ */

#define EQDC_LPOSH1_LPOSH1_MASK                  (0xFFFFU)
#define EQDC_LPOSH1_LPOSH1_SHIFT                 (0U)
/*! LPOSH1 - LPOSH1 */
#define EQDC_LPOSH1_LPOSH1(x)                    (((uint16_t)(((uint16_t)(x)) << EQDC_LPOSH1_LPOSH1_SHIFT)) & EQDC_LPOSH1_LPOSH1_MASK)
/*! @} */

/*! @name UCOMP2 - Upper Position Compare 2 */
/*! @{ */

#define EQDC_UCOMP2_UCOMP2_MASK                  (0xFFFFU)
#define EQDC_UCOMP2_UCOMP2_SHIFT                 (0U)
/*! UCOMP2 - UCOMP2 */
#define EQDC_UCOMP2_UCOMP2(x)                    (((uint16_t)(((uint16_t)(x)) << EQDC_UCOMP2_UCOMP2_SHIFT)) & EQDC_UCOMP2_UCOMP2_MASK)
/*! @} */

/*! @name UPOSH2 - Upper Position Holder Register 3 */
/*! @{ */

#define EQDC_UPOSH2_UPOSH2_MASK                  (0xFFFFU)
#define EQDC_UPOSH2_UPOSH2_SHIFT                 (0U)
/*! UPOSH2 - UPOSH2 */
#define EQDC_UPOSH2_UPOSH2(x)                    (((uint16_t)(((uint16_t)(x)) << EQDC_UPOSH2_UPOSH2_SHIFT)) & EQDC_UPOSH2_UPOSH2_MASK)
/*! @} */

/*! @name LCOMP2 - Lower Position Compare 2 */
/*! @{ */

#define EQDC_LCOMP2_LCOMP2_MASK                  (0xFFFFU)
#define EQDC_LCOMP2_LCOMP2_SHIFT                 (0U)
/*! LCOMP2 - LCOMP2 */
#define EQDC_LCOMP2_LCOMP2(x)                    (((uint16_t)(((uint16_t)(x)) << EQDC_LCOMP2_LCOMP2_SHIFT)) & EQDC_LCOMP2_LCOMP2_MASK)
/*! @} */

/*! @name LPOSH2 - Lower Position Holder Register 2 */
/*! @{ */

#define EQDC_LPOSH2_LPOSH2_MASK                  (0xFFFFU)
#define EQDC_LPOSH2_LPOSH2_SHIFT                 (0U)
/*! LPOSH2 - LPOSH2 */
#define EQDC_LPOSH2_LPOSH2(x)                    (((uint16_t)(((uint16_t)(x)) << EQDC_LPOSH2_LPOSH2_SHIFT)) & EQDC_LPOSH2_LPOSH2_MASK)
/*! @} */

/*! @name UCOMP3 - Upper Position Compare 3 */
/*! @{ */

#define EQDC_UCOMP3_UCOMP3_MASK                  (0xFFFFU)
#define EQDC_UCOMP3_UCOMP3_SHIFT                 (0U)
/*! UCOMP3 - UCOMP3 */
#define EQDC_UCOMP3_UCOMP3(x)                    (((uint16_t)(((uint16_t)(x)) << EQDC_UCOMP3_UCOMP3_SHIFT)) & EQDC_UCOMP3_UCOMP3_MASK)
/*! @} */

/*! @name UPOSH3 - Upper Position Holder Register 3 */
/*! @{ */

#define EQDC_UPOSH3_UPOSH3_MASK                  (0xFFFFU)
#define EQDC_UPOSH3_UPOSH3_SHIFT                 (0U)
/*! UPOSH3 - UPOSH3 */
#define EQDC_UPOSH3_UPOSH3(x)                    (((uint16_t)(((uint16_t)(x)) << EQDC_UPOSH3_UPOSH3_SHIFT)) & EQDC_UPOSH3_UPOSH3_MASK)
/*! @} */

/*! @name LCOMP3 - Lower Position Compare 3 */
/*! @{ */

#define EQDC_LCOMP3_LCOMP3_MASK                  (0xFFFFU)
#define EQDC_LCOMP3_LCOMP3_SHIFT                 (0U)
/*! LCOMP3 - LCOMP3 */
#define EQDC_LCOMP3_LCOMP3(x)                    (((uint16_t)(((uint16_t)(x)) << EQDC_LCOMP3_LCOMP3_SHIFT)) & EQDC_LCOMP3_LCOMP3_MASK)
/*! @} */

/*! @name LPOSH3 - Lower Position Holder Register 3 */
/*! @{ */

#define EQDC_LPOSH3_LPOSH3_MASK                  (0xFFFFU)
#define EQDC_LPOSH3_LPOSH3_SHIFT                 (0U)
/*! LPOSH3 - LPOSH3 */
#define EQDC_LPOSH3_LPOSH3(x)                    (((uint16_t)(((uint16_t)(x)) << EQDC_LPOSH3_LPOSH3_SHIFT)) & EQDC_LPOSH3_LPOSH3_MASK)
/*! @} */

/*! @name INTCTRL - Interrupt Control Register */
/*! @{ */

#define EQDC_INTCTRL_SABIE_MASK                  (0x1U)
#define EQDC_INTCTRL_SABIE_SHIFT                 (0U)
/*! SABIE - Simultaneous PHASEA and PHASEB Change Interrupt Enable
 *  0b0..Disabled
 *  0b1..Enabled
 */
#define EQDC_INTCTRL_SABIE(x)                    (((uint16_t)(((uint16_t)(x)) << EQDC_INTCTRL_SABIE_SHIFT)) & EQDC_INTCTRL_SABIE_MASK)

#define EQDC_INTCTRL_SABIRQ_MASK                 (0x2U)
#define EQDC_INTCTRL_SABIRQ_SHIFT                (1U)
/*! SABIRQ - Simultaneous PHASEA and PHASEB Change Interrupt Request
 *  0b0..No simultaneous change of PHASEA and PHASEB has occurred
 *  0b1..A simultaneous change of PHASEA and PHASEB has occurred
 */
#define EQDC_INTCTRL_SABIRQ(x)                   (((uint16_t)(((uint16_t)(x)) << EQDC_INTCTRL_SABIRQ_SHIFT)) & EQDC_INTCTRL_SABIRQ_MASK)

#define EQDC_INTCTRL_DIRIE_MASK                  (0x4U)
#define EQDC_INTCTRL_DIRIE_SHIFT                 (2U)
/*! DIRIE - Count direction change interrupt enable
 *  0b0..Disabled
 *  0b1..Enabled
 */
#define EQDC_INTCTRL_DIRIE(x)                    (((uint16_t)(((uint16_t)(x)) << EQDC_INTCTRL_DIRIE_SHIFT)) & EQDC_INTCTRL_DIRIE_MASK)

#define EQDC_INTCTRL_DIRIRQ_MASK                 (0x8U)
#define EQDC_INTCTRL_DIRIRQ_SHIFT                (3U)
/*! DIRIRQ - Count direction change interrupt
 *  0b0..Count direction unchanged
 *  0b1..Count direction changed
 */
#define EQDC_INTCTRL_DIRIRQ(x)                   (((uint16_t)(((uint16_t)(x)) << EQDC_INTCTRL_DIRIRQ_SHIFT)) & EQDC_INTCTRL_DIRIRQ_MASK)

#define EQDC_INTCTRL_RUIE_MASK                   (0x10U)
#define EQDC_INTCTRL_RUIE_SHIFT                  (4U)
/*! RUIE - Roll-under Interrupt Enable
 *  0b0..Disabled
 *  0b1..Enabled
 */
#define EQDC_INTCTRL_RUIE(x)                     (((uint16_t)(((uint16_t)(x)) << EQDC_INTCTRL_RUIE_SHIFT)) & EQDC_INTCTRL_RUIE_MASK)

#define EQDC_INTCTRL_RUIRQ_MASK                  (0x20U)
#define EQDC_INTCTRL_RUIRQ_SHIFT                 (5U)
/*! RUIRQ - Roll-under Interrupt Request
 *  0b0..No roll-under has occurred
 *  0b1..Roll-under has occurred
 */
#define EQDC_INTCTRL_RUIRQ(x)                    (((uint16_t)(((uint16_t)(x)) << EQDC_INTCTRL_RUIRQ_SHIFT)) & EQDC_INTCTRL_RUIRQ_MASK)

#define EQDC_INTCTRL_ROIE_MASK                   (0x40U)
#define EQDC_INTCTRL_ROIE_SHIFT                  (6U)
/*! ROIE - Roll-over Interrupt Enable
 *  0b0..Disabled
 *  0b1..Enabled
 */
#define EQDC_INTCTRL_ROIE(x)                     (((uint16_t)(((uint16_t)(x)) << EQDC_INTCTRL_ROIE_SHIFT)) & EQDC_INTCTRL_ROIE_MASK)

#define EQDC_INTCTRL_ROIRQ_MASK                  (0x80U)
#define EQDC_INTCTRL_ROIRQ_SHIFT                 (7U)
/*! ROIRQ - Roll-over Interrupt Request
 *  0b0..No roll-over has occurred
 *  0b1..Roll-over has occurred
 */
#define EQDC_INTCTRL_ROIRQ(x)                    (((uint16_t)(((uint16_t)(x)) << EQDC_INTCTRL_ROIRQ_SHIFT)) & EQDC_INTCTRL_ROIRQ_MASK)

#define EQDC_INTCTRL_CMP0IE_MASK                 (0x100U)
#define EQDC_INTCTRL_CMP0IE_SHIFT                (8U)
/*! CMP0IE - Compare 0 Interrupt Enable
 *  0b0..Disabled
 *  0b1..Enabled
 */
#define EQDC_INTCTRL_CMP0IE(x)                   (((uint16_t)(((uint16_t)(x)) << EQDC_INTCTRL_CMP0IE_SHIFT)) & EQDC_INTCTRL_CMP0IE_MASK)

#define EQDC_INTCTRL_CMP0IRQ_MASK                (0x200U)
#define EQDC_INTCTRL_CMP0IRQ_SHIFT               (9U)
/*! CMP0IRQ - Compare 0 Interrupt Request
 *  0b0..No match has occurred (the position counter does not match the COMP0 value)
 *  0b1..COMP match has occurred (the position counter matches the COMP0 value)
 */
#define EQDC_INTCTRL_CMP0IRQ(x)                  (((uint16_t)(((uint16_t)(x)) << EQDC_INTCTRL_CMP0IRQ_SHIFT)) & EQDC_INTCTRL_CMP0IRQ_MASK)

#define EQDC_INTCTRL_CMP1IE_MASK                 (0x400U)
#define EQDC_INTCTRL_CMP1IE_SHIFT                (10U)
/*! CMP1IE - Compare1 Interrupt Enable
 *  0b0..Disabled
 *  0b1..Enabled
 */
#define EQDC_INTCTRL_CMP1IE(x)                   (((uint16_t)(((uint16_t)(x)) << EQDC_INTCTRL_CMP1IE_SHIFT)) & EQDC_INTCTRL_CMP1IE_MASK)

#define EQDC_INTCTRL_CMP1IRQ_MASK                (0x800U)
#define EQDC_INTCTRL_CMP1IRQ_SHIFT               (11U)
/*! CMP1IRQ - Compare1 Interrupt Request
 *  0b0..No match has occurred (the position counter does not match the COMP1 value)
 *  0b1..COMP1 match has occurred (the position counter matches the COMP1 value)
 */
#define EQDC_INTCTRL_CMP1IRQ(x)                  (((uint16_t)(((uint16_t)(x)) << EQDC_INTCTRL_CMP1IRQ_SHIFT)) & EQDC_INTCTRL_CMP1IRQ_MASK)

#define EQDC_INTCTRL_CMP2IE_MASK                 (0x1000U)
#define EQDC_INTCTRL_CMP2IE_SHIFT                (12U)
/*! CMP2IE - Compare2 Interrupt Enable
 *  0b0..Disabled
 *  0b1..Enabled
 */
#define EQDC_INTCTRL_CMP2IE(x)                   (((uint16_t)(((uint16_t)(x)) << EQDC_INTCTRL_CMP2IE_SHIFT)) & EQDC_INTCTRL_CMP2IE_MASK)

#define EQDC_INTCTRL_CMP2IRQ_MASK                (0x2000U)
#define EQDC_INTCTRL_CMP2IRQ_SHIFT               (13U)
/*! CMP2IRQ - Compare2 Interrupt Request
 *  0b0..No match has occurred (the position counter does not match the COMP2 value)
 *  0b1..COMP2 match has occurred (the position counter matches the COMP2 value)
 */
#define EQDC_INTCTRL_CMP2IRQ(x)                  (((uint16_t)(((uint16_t)(x)) << EQDC_INTCTRL_CMP2IRQ_SHIFT)) & EQDC_INTCTRL_CMP2IRQ_MASK)

#define EQDC_INTCTRL_CMP3IE_MASK                 (0x4000U)
#define EQDC_INTCTRL_CMP3IE_SHIFT                (14U)
/*! CMP3IE - Compare3 Interrupt Enable
 *  0b0..Disabled
 *  0b1..Enabled
 */
#define EQDC_INTCTRL_CMP3IE(x)                   (((uint16_t)(((uint16_t)(x)) << EQDC_INTCTRL_CMP3IE_SHIFT)) & EQDC_INTCTRL_CMP3IE_MASK)

#define EQDC_INTCTRL_CMP3IRQ_MASK                (0x8000U)
#define EQDC_INTCTRL_CMP3IRQ_SHIFT               (15U)
/*! CMP3IRQ - Compare3 Interrupt Request
 *  0b0..No match has occurred (the position counter does not match the COMP3 value)
 *  0b1..COMP3 match has occurred (the position counter matches the COMP3 value)
 */
#define EQDC_INTCTRL_CMP3IRQ(x)                  (((uint16_t)(((uint16_t)(x)) << EQDC_INTCTRL_CMP3IRQ_SHIFT)) & EQDC_INTCTRL_CMP3IRQ_MASK)
/*! @} */

/*! @name WTR - Watchdog Timeout Register */
/*! @{ */

#define EQDC_WTR_WDOG_MASK                       (0xFFFFU)
#define EQDC_WTR_WDOG_SHIFT                      (0U)
/*! WDOG - WDOG */
#define EQDC_WTR_WDOG(x)                         (((uint16_t)(((uint16_t)(x)) << EQDC_WTR_WDOG_SHIFT)) & EQDC_WTR_WDOG_MASK)
/*! @} */

/*! @name IMR - Input Monitor Register */
/*! @{ */

#define EQDC_IMR_HOME_ENABLE_MASK                (0x1U)
#define EQDC_IMR_HOME_ENABLE_SHIFT               (0U)
/*! HOME_ENABLE - HOME_ENABLE */
#define EQDC_IMR_HOME_ENABLE(x)                  (((uint16_t)(((uint16_t)(x)) << EQDC_IMR_HOME_ENABLE_SHIFT)) & EQDC_IMR_HOME_ENABLE_MASK)

#define EQDC_IMR_INDEX_PRESET_MASK               (0x2U)
#define EQDC_IMR_INDEX_PRESET_SHIFT              (1U)
/*! INDEX_PRESET - INDEX_PRESET */
#define EQDC_IMR_INDEX_PRESET(x)                 (((uint16_t)(((uint16_t)(x)) << EQDC_IMR_INDEX_PRESET_SHIFT)) & EQDC_IMR_INDEX_PRESET_MASK)

#define EQDC_IMR_PHB_MASK                        (0x4U)
#define EQDC_IMR_PHB_SHIFT                       (2U)
/*! PHB - PHB */
#define EQDC_IMR_PHB(x)                          (((uint16_t)(((uint16_t)(x)) << EQDC_IMR_PHB_SHIFT)) & EQDC_IMR_PHB_MASK)

#define EQDC_IMR_PHA_MASK                        (0x8U)
#define EQDC_IMR_PHA_SHIFT                       (3U)
/*! PHA - PHA */
#define EQDC_IMR_PHA(x)                          (((uint16_t)(((uint16_t)(x)) << EQDC_IMR_PHA_SHIFT)) & EQDC_IMR_PHA_MASK)

#define EQDC_IMR_FHOM_ENA_MASK                   (0x10U)
#define EQDC_IMR_FHOM_ENA_SHIFT                  (4U)
/*! FHOM_ENA - filter operation on HOME/ENABLE input */
#define EQDC_IMR_FHOM_ENA(x)                     (((uint16_t)(((uint16_t)(x)) << EQDC_IMR_FHOM_ENA_SHIFT)) & EQDC_IMR_FHOM_ENA_MASK)

#define EQDC_IMR_FIND_PRE_MASK                   (0x20U)
#define EQDC_IMR_FIND_PRE_SHIFT                  (5U)
/*! FIND_PRE - filter operation on INDEX/PRESET input */
#define EQDC_IMR_FIND_PRE(x)                     (((uint16_t)(((uint16_t)(x)) << EQDC_IMR_FIND_PRE_SHIFT)) & EQDC_IMR_FIND_PRE_MASK)

#define EQDC_IMR_FPHB_MASK                       (0x40U)
#define EQDC_IMR_FPHB_SHIFT                      (6U)
/*! FPHB - filter operation on PHASEB input */
#define EQDC_IMR_FPHB(x)                         (((uint16_t)(((uint16_t)(x)) << EQDC_IMR_FPHB_SHIFT)) & EQDC_IMR_FPHB_MASK)

#define EQDC_IMR_FPHA_MASK                       (0x80U)
#define EQDC_IMR_FPHA_SHIFT                      (7U)
/*! FPHA - filter operation on PHASEA input */
#define EQDC_IMR_FPHA(x)                         (((uint16_t)(((uint16_t)(x)) << EQDC_IMR_FPHA_SHIFT)) & EQDC_IMR_FPHA_MASK)

#define EQDC_IMR_CMPF0_MASK                      (0x100U)
#define EQDC_IMR_CMPF0_SHIFT                     (8U)
/*! CMPF0 - Position Compare 0 Flag Output
 *  0b0..When the position counter is less than value of COMP0 register
 *  0b1..When the position counter is greater or equal than value of COMP0 register
 */
#define EQDC_IMR_CMPF0(x)                        (((uint16_t)(((uint16_t)(x)) << EQDC_IMR_CMPF0_SHIFT)) & EQDC_IMR_CMPF0_MASK)

#define EQDC_IMR_CMP1F_MASK                      (0x200U)
#define EQDC_IMR_CMP1F_SHIFT                     (9U)
/*! CMP1F - Position Compare1 Flag Output
 *  0b0..When the position counter is less than value of COMP1 register
 *  0b1..When the position counter is greater or equal than value of COMP1 register
 */
#define EQDC_IMR_CMP1F(x)                        (((uint16_t)(((uint16_t)(x)) << EQDC_IMR_CMP1F_SHIFT)) & EQDC_IMR_CMP1F_MASK)

#define EQDC_IMR_CMP2F_MASK                      (0x400U)
#define EQDC_IMR_CMP2F_SHIFT                     (10U)
/*! CMP2F - Position Compare2 Flag Output
 *  0b0..When the position counter is less than value of COMP2 register
 *  0b1..When the position counter is greater or equal than value of COMP2 register
 */
#define EQDC_IMR_CMP2F(x)                        (((uint16_t)(((uint16_t)(x)) << EQDC_IMR_CMP2F_SHIFT)) & EQDC_IMR_CMP2F_MASK)

#define EQDC_IMR_CMP3F_MASK                      (0x800U)
#define EQDC_IMR_CMP3F_SHIFT                     (11U)
/*! CMP3F - Position Compare3 Flag Output
 *  0b0..When the position counter value is less than value of COMP3 register
 *  0b1..When the position counter is greater or equal than value of COMP3 register
 */
#define EQDC_IMR_CMP3F(x)                        (((uint16_t)(((uint16_t)(x)) << EQDC_IMR_CMP3F_SHIFT)) & EQDC_IMR_CMP3F_MASK)

#define EQDC_IMR_DIRH_MASK                       (0x4000U)
#define EQDC_IMR_DIRH_SHIFT                      (14U)
/*! DIRH - Count Direction Flag Hold */
#define EQDC_IMR_DIRH(x)                         (((uint16_t)(((uint16_t)(x)) << EQDC_IMR_DIRH_SHIFT)) & EQDC_IMR_DIRH_MASK)

#define EQDC_IMR_DIR_MASK                        (0x8000U)
#define EQDC_IMR_DIR_SHIFT                       (15U)
/*! DIR - Count Direction Flag Output
 *  0b0..Current count was in the down direction
 *  0b1..Current count was in the up direction
 */
#define EQDC_IMR_DIR(x)                          (((uint16_t)(((uint16_t)(x)) << EQDC_IMR_DIR_SHIFT)) & EQDC_IMR_DIR_MASK)
/*! @} */

/*! @name TST - Test Register */
/*! @{ */

#define EQDC_TST_TEST_COUNT_MASK                 (0xFFU)
#define EQDC_TST_TEST_COUNT_SHIFT                (0U)
/*! TEST_COUNT - TEST_COUNT */
#define EQDC_TST_TEST_COUNT(x)                   (((uint16_t)(((uint16_t)(x)) << EQDC_TST_TEST_COUNT_SHIFT)) & EQDC_TST_TEST_COUNT_MASK)

#define EQDC_TST_TEST_PERIOD_MASK                (0x1F00U)
#define EQDC_TST_TEST_PERIOD_SHIFT               (8U)
/*! TEST_PERIOD - TEST_PERIOD */
#define EQDC_TST_TEST_PERIOD(x)                  (((uint16_t)(((uint16_t)(x)) << EQDC_TST_TEST_PERIOD_SHIFT)) & EQDC_TST_TEST_PERIOD_MASK)

#define EQDC_TST_QDN_MASK                        (0x2000U)
#define EQDC_TST_QDN_SHIFT                       (13U)
/*! QDN - Quadrature Decoder Negative Signal
 *  0b0..Generates a positive quadrature decoder signal
 *  0b1..Generates a negative quadrature decoder signal
 */
#define EQDC_TST_QDN(x)                          (((uint16_t)(((uint16_t)(x)) << EQDC_TST_QDN_SHIFT)) & EQDC_TST_QDN_MASK)

#define EQDC_TST_TCE_MASK                        (0x4000U)
#define EQDC_TST_TCE_SHIFT                       (14U)
/*! TCE - Test Counter Enable
 *  0b0..Disabled
 *  0b1..Enabled
 */
#define EQDC_TST_TCE(x)                          (((uint16_t)(((uint16_t)(x)) << EQDC_TST_TCE_SHIFT)) & EQDC_TST_TCE_MASK)

#define EQDC_TST_TEN_MASK                        (0x8000U)
#define EQDC_TST_TEN_SHIFT                       (15U)
/*! TEN - Test Mode Enable
 *  0b0..Disabled
 *  0b1..Enabled
 */
#define EQDC_TST_TEN(x)                          (((uint16_t)(((uint16_t)(x)) << EQDC_TST_TEN_SHIFT)) & EQDC_TST_TEN_MASK)
/*! @} */

/*! @name UVERID - Upper VERID */
/*! @{ */

#define EQDC_UVERID_UVERID_MASK                  (0xFFFFU)
#define EQDC_UVERID_UVERID_SHIFT                 (0U)
/*! UVERID - UVERID */
#define EQDC_UVERID_UVERID(x)                    (((uint16_t)(((uint16_t)(x)) << EQDC_UVERID_UVERID_SHIFT)) & EQDC_UVERID_UVERID_MASK)
/*! @} */

/*! @name LVERID - Lower VERID */
/*! @{ */

#define EQDC_LVERID_LVERID_MASK                  (0xFFFFU)
#define EQDC_LVERID_LVERID_SHIFT                 (0U)
/*! LVERID - LVERID */
#define EQDC_LVERID_LVERID(x)                    (((uint16_t)(((uint16_t)(x)) << EQDC_LVERID_LVERID_SHIFT)) & EQDC_LVERID_LVERID_MASK)
/*! @} */


/*!
 * @}
 */ /* end of group EQDC_Register_Masks */


/*!
 * @}
 */ /* end of group EQDC_Peripheral_Access_Layer */


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


#endif  /* PERI_EQDC_H_ */

