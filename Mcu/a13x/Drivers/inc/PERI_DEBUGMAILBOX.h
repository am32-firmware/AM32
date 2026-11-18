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
**         CMSIS Peripheral Access Layer for DEBUGMAILBOX
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
 * @file PERI_DEBUGMAILBOX.h
 * @version 2.0
 * @date 2024-10-29
 * @brief CMSIS Peripheral Access Layer for DEBUGMAILBOX
 *
 * CMSIS Peripheral Access Layer for DEBUGMAILBOX
 */

#if !defined(PERI_DEBUGMAILBOX_H_)
#define PERI_DEBUGMAILBOX_H_                     /**< Symbol preventing repeated inclusion */

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
   -- DEBUGMAILBOX Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DEBUGMAILBOX_Peripheral_Access_Layer DEBUGMAILBOX Peripheral Access Layer
 * @{
 */

/** DEBUGMAILBOX - Register Layout Typedef */
typedef struct {
  __IO uint32_t CSW;                               /**< Command and Status Word, offset: 0x0 */
  __IO uint32_t REQUEST;                           /**< Request Value, offset: 0x4 */
  __IO uint32_t RETURN;                            /**< Return Value, offset: 0x8 */
       uint8_t RESERVED_0[240];
  __I  uint32_t ID;                                /**< Identification, offset: 0xFC */
} DEBUGMAILBOX_Type;

/* ----------------------------------------------------------------------------
   -- DEBUGMAILBOX Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DEBUGMAILBOX_Register_Masks DEBUGMAILBOX Register Masks
 * @{
 */

/*! @name CSW - Command and Status Word */
/*! @{ */

#define DEBUGMAILBOX_CSW_RESYNCH_REQ_MASK        (0x1U)
#define DEBUGMAILBOX_CSW_RESYNCH_REQ_SHIFT       (0U)
/*! RESYNCH_REQ - Resynchronization Request
 *  0b0..No request
 *  0b1..Request for resynchronization
 */
#define DEBUGMAILBOX_CSW_RESYNCH_REQ(x)          (((uint32_t)(((uint32_t)(x)) << DEBUGMAILBOX_CSW_RESYNCH_REQ_SHIFT)) & DEBUGMAILBOX_CSW_RESYNCH_REQ_MASK)

#define DEBUGMAILBOX_CSW_REQ_PENDING_MASK        (0x2U)
#define DEBUGMAILBOX_CSW_REQ_PENDING_SHIFT       (1U)
/*! REQ_PENDING - Request Pending
 *  0b0..No request pending
 *  0b1..Request for resynchronization pending
 */
#define DEBUGMAILBOX_CSW_REQ_PENDING(x)          (((uint32_t)(((uint32_t)(x)) << DEBUGMAILBOX_CSW_REQ_PENDING_SHIFT)) & DEBUGMAILBOX_CSW_REQ_PENDING_MASK)

#define DEBUGMAILBOX_CSW_DBG_OR_ERR_MASK         (0x4U)
#define DEBUGMAILBOX_CSW_DBG_OR_ERR_SHIFT        (2U)
/*! DBG_OR_ERR - DBGMB Overrun Error
 *  0b0..No DBGMB Overrun error
 *  0b1..DBGMB overrun error. A DBGMB overrun occurred.
 */
#define DEBUGMAILBOX_CSW_DBG_OR_ERR(x)           (((uint32_t)(((uint32_t)(x)) << DEBUGMAILBOX_CSW_DBG_OR_ERR_SHIFT)) & DEBUGMAILBOX_CSW_DBG_OR_ERR_MASK)

#define DEBUGMAILBOX_CSW_AHB_OR_ERR_MASK         (0x8U)
#define DEBUGMAILBOX_CSW_AHB_OR_ERR_SHIFT        (3U)
/*! AHB_OR_ERR - AHB Overrun Error
 *  0b0..No AHB Overrun Error
 *  0b1..AHB Overrun Error. An AHB overrun occurred.
 */
#define DEBUGMAILBOX_CSW_AHB_OR_ERR(x)           (((uint32_t)(((uint32_t)(x)) << DEBUGMAILBOX_CSW_AHB_OR_ERR_SHIFT)) & DEBUGMAILBOX_CSW_AHB_OR_ERR_MASK)

#define DEBUGMAILBOX_CSW_SOFT_RESET_MASK         (0x10U)
#define DEBUGMAILBOX_CSW_SOFT_RESET_SHIFT        (4U)
/*! SOFT_RESET - Soft Reset */
#define DEBUGMAILBOX_CSW_SOFT_RESET(x)           (((uint32_t)(((uint32_t)(x)) << DEBUGMAILBOX_CSW_SOFT_RESET_SHIFT)) & DEBUGMAILBOX_CSW_SOFT_RESET_MASK)

#define DEBUGMAILBOX_CSW_CHIP_RESET_REQ_MASK     (0x20U)
#define DEBUGMAILBOX_CSW_CHIP_RESET_REQ_SHIFT    (5U)
/*! CHIP_RESET_REQ - Chip Reset Request */
#define DEBUGMAILBOX_CSW_CHIP_RESET_REQ(x)       (((uint32_t)(((uint32_t)(x)) << DEBUGMAILBOX_CSW_CHIP_RESET_REQ_SHIFT)) & DEBUGMAILBOX_CSW_CHIP_RESET_REQ_MASK)
/*! @} */

/*! @name REQUEST - Request Value */
/*! @{ */

#define DEBUGMAILBOX_REQUEST_REQUEST_MASK        (0xFFFFFFFFU)
#define DEBUGMAILBOX_REQUEST_REQUEST_SHIFT       (0U)
/*! REQUEST - Request Value */
#define DEBUGMAILBOX_REQUEST_REQUEST(x)          (((uint32_t)(((uint32_t)(x)) << DEBUGMAILBOX_REQUEST_REQUEST_SHIFT)) & DEBUGMAILBOX_REQUEST_REQUEST_MASK)
/*! @} */

/*! @name RETURN - Return Value */
/*! @{ */

#define DEBUGMAILBOX_RETURN_RET_MASK             (0xFFFFFFFFU)
#define DEBUGMAILBOX_RETURN_RET_SHIFT            (0U)
/*! RET - Return Value */
#define DEBUGMAILBOX_RETURN_RET(x)               (((uint32_t)(((uint32_t)(x)) << DEBUGMAILBOX_RETURN_RET_SHIFT)) & DEBUGMAILBOX_RETURN_RET_MASK)
/*! @} */

/*! @name ID - Identification */
/*! @{ */

#define DEBUGMAILBOX_ID_ID_MASK                  (0xFFFFFFFFU)
#define DEBUGMAILBOX_ID_ID_SHIFT                 (0U)
/*! ID - Identification Value */
#define DEBUGMAILBOX_ID_ID(x)                    (((uint32_t)(((uint32_t)(x)) << DEBUGMAILBOX_ID_ID_SHIFT)) & DEBUGMAILBOX_ID_ID_MASK)
/*! @} */


/*!
 * @}
 */ /* end of group DEBUGMAILBOX_Register_Masks */


/*!
 * @}
 */ /* end of group DEBUGMAILBOX_Peripheral_Access_Layer */


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


#endif  /* PERI_DEBUGMAILBOX_H_ */

