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
**         CMSIS Peripheral Access Layer for OSTIMER
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
 * @file PERI_OSTIMER.h
 * @version 2.0
 * @date 2024-10-29
 * @brief CMSIS Peripheral Access Layer for OSTIMER
 *
 * CMSIS Peripheral Access Layer for OSTIMER
 */

#if !defined(PERI_OSTIMER_H_)
#define PERI_OSTIMER_H_                          /**< Symbol preventing repeated inclusion */

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
   -- OSTIMER Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup OSTIMER_Peripheral_Access_Layer OSTIMER Peripheral Access Layer
 * @{
 */

/** OSTIMER - Register Layout Typedef */
typedef struct {
  __I  uint32_t EVTIMERL;                          /**< EVTIMER Low, offset: 0x0 */
  __I  uint32_t EVTIMERH;                          /**< EVTIMER High, offset: 0x4 */
  __I  uint32_t CAPTURE_L;                         /**< Local Capture Low for CPU, offset: 0x8 */
  __I  uint32_t CAPTURE_H;                         /**< Local Capture High for CPU, offset: 0xC */
  __IO uint32_t MATCH_L;                           /**< Local Match Low for CPU, offset: 0x10 */
  __IO uint32_t MATCH_H;                           /**< Local Match High for CPU, offset: 0x14 */
       uint8_t RESERVED_0[4];
  __IO uint32_t OSEVENT_CTRL;                      /**< OSTIMER Control for CPU, offset: 0x1C */
} OSTIMER_Type;

/* ----------------------------------------------------------------------------
   -- OSTIMER Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup OSTIMER_Register_Masks OSTIMER Register Masks
 * @{
 */

/*! @name EVTIMERL - EVTIMER Low */
/*! @{ */

#define OSTIMER_EVTIMERL_EVTIMER_COUNT_VALUE_MASK (0xFFFFFFFFU)
#define OSTIMER_EVTIMERL_EVTIMER_COUNT_VALUE_SHIFT (0U)
/*! EVTIMER_COUNT_VALUE - EVTimer Count Value */
#define OSTIMER_EVTIMERL_EVTIMER_COUNT_VALUE(x)  (((uint32_t)(((uint32_t)(x)) << OSTIMER_EVTIMERL_EVTIMER_COUNT_VALUE_SHIFT)) & OSTIMER_EVTIMERL_EVTIMER_COUNT_VALUE_MASK)
/*! @} */

/*! @name EVTIMERH - EVTIMER High */
/*! @{ */

#define OSTIMER_EVTIMERH_EVTIMER_COUNT_VALUE_MASK (0x3FFU)
#define OSTIMER_EVTIMERH_EVTIMER_COUNT_VALUE_SHIFT (0U)
/*! EVTIMER_COUNT_VALUE - EVTimer Count Value */
#define OSTIMER_EVTIMERH_EVTIMER_COUNT_VALUE(x)  (((uint32_t)(((uint32_t)(x)) << OSTIMER_EVTIMERH_EVTIMER_COUNT_VALUE_SHIFT)) & OSTIMER_EVTIMERH_EVTIMER_COUNT_VALUE_MASK)
/*! @} */

/*! @name CAPTURE_L - Local Capture Low for CPU */
/*! @{ */

#define OSTIMER_CAPTURE_L_CAPTURE_VALUE_MASK     (0xFFFFFFFFU)
#define OSTIMER_CAPTURE_L_CAPTURE_VALUE_SHIFT    (0U)
/*! CAPTURE_VALUE - EVTimer Capture Value */
#define OSTIMER_CAPTURE_L_CAPTURE_VALUE(x)       (((uint32_t)(((uint32_t)(x)) << OSTIMER_CAPTURE_L_CAPTURE_VALUE_SHIFT)) & OSTIMER_CAPTURE_L_CAPTURE_VALUE_MASK)
/*! @} */

/*! @name CAPTURE_H - Local Capture High for CPU */
/*! @{ */

#define OSTIMER_CAPTURE_H_CAPTURE_VALUE_MASK     (0x3FFU)
#define OSTIMER_CAPTURE_H_CAPTURE_VALUE_SHIFT    (0U)
/*! CAPTURE_VALUE - EVTimer Capture Value */
#define OSTIMER_CAPTURE_H_CAPTURE_VALUE(x)       (((uint32_t)(((uint32_t)(x)) << OSTIMER_CAPTURE_H_CAPTURE_VALUE_SHIFT)) & OSTIMER_CAPTURE_H_CAPTURE_VALUE_MASK)
/*! @} */

/*! @name MATCH_L - Local Match Low for CPU */
/*! @{ */

#define OSTIMER_MATCH_L_MATCH_VALUE_MASK         (0xFFFFFFFFU)
#define OSTIMER_MATCH_L_MATCH_VALUE_SHIFT        (0U)
/*! MATCH_VALUE - EVTimer Match Value */
#define OSTIMER_MATCH_L_MATCH_VALUE(x)           (((uint32_t)(((uint32_t)(x)) << OSTIMER_MATCH_L_MATCH_VALUE_SHIFT)) & OSTIMER_MATCH_L_MATCH_VALUE_MASK)
/*! @} */

/*! @name MATCH_H - Local Match High for CPU */
/*! @{ */

#define OSTIMER_MATCH_H_MATCH_VALUE_MASK         (0x3FFU)
#define OSTIMER_MATCH_H_MATCH_VALUE_SHIFT        (0U)
/*! MATCH_VALUE - EVTimer Match Value */
#define OSTIMER_MATCH_H_MATCH_VALUE(x)           (((uint32_t)(((uint32_t)(x)) << OSTIMER_MATCH_H_MATCH_VALUE_SHIFT)) & OSTIMER_MATCH_H_MATCH_VALUE_MASK)
/*! @} */

/*! @name OSEVENT_CTRL - OSTIMER Control for CPU */
/*! @{ */

#define OSTIMER_OSEVENT_CTRL_OSTIMER_INTRFLAG_MASK (0x1U)
#define OSTIMER_OSEVENT_CTRL_OSTIMER_INTRFLAG_SHIFT (0U)
/*! OSTIMER_INTRFLAG - Interrupt Flag */
#define OSTIMER_OSEVENT_CTRL_OSTIMER_INTRFLAG(x) (((uint32_t)(((uint32_t)(x)) << OSTIMER_OSEVENT_CTRL_OSTIMER_INTRFLAG_SHIFT)) & OSTIMER_OSEVENT_CTRL_OSTIMER_INTRFLAG_MASK)

#define OSTIMER_OSEVENT_CTRL_OSTIMER_INTENA_MASK (0x2U)
#define OSTIMER_OSEVENT_CTRL_OSTIMER_INTENA_SHIFT (1U)
/*! OSTIMER_INTENA - Interrupt or Wake-Up Request
 *  0b0..Interrupts blocked
 *  0b1..Interrupts enabled
 */
#define OSTIMER_OSEVENT_CTRL_OSTIMER_INTENA(x)   (((uint32_t)(((uint32_t)(x)) << OSTIMER_OSEVENT_CTRL_OSTIMER_INTENA_SHIFT)) & OSTIMER_OSEVENT_CTRL_OSTIMER_INTENA_MASK)

#define OSTIMER_OSEVENT_CTRL_MATCH_WR_RDY_MASK   (0x4U)
#define OSTIMER_OSEVENT_CTRL_MATCH_WR_RDY_SHIFT  (2U)
/*! MATCH_WR_RDY - EVTimer Match Write Ready */
#define OSTIMER_OSEVENT_CTRL_MATCH_WR_RDY(x)     (((uint32_t)(((uint32_t)(x)) << OSTIMER_OSEVENT_CTRL_MATCH_WR_RDY_SHIFT)) & OSTIMER_OSEVENT_CTRL_MATCH_WR_RDY_MASK)

#define OSTIMER_OSEVENT_CTRL_DEBUG_EN_MASK       (0x8U)
#define OSTIMER_OSEVENT_CTRL_DEBUG_EN_SHIFT      (3U)
/*! DEBUG_EN - Debug Enable
 *  0b0..Disables
 *  0b1..Enables
 */
#define OSTIMER_OSEVENT_CTRL_DEBUG_EN(x)         (((uint32_t)(((uint32_t)(x)) << OSTIMER_OSEVENT_CTRL_DEBUG_EN_SHIFT)) & OSTIMER_OSEVENT_CTRL_DEBUG_EN_MASK)
/*! @} */


/*!
 * @}
 */ /* end of group OSTIMER_Register_Masks */


/*!
 * @}
 */ /* end of group OSTIMER_Peripheral_Access_Layer */


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


#endif  /* PERI_OSTIMER_H_ */

