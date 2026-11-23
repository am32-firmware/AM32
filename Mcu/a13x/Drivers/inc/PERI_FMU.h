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
**         CMSIS Peripheral Access Layer for FMU
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
 * @file PERI_FMU.h
 * @version 2.0
 * @date 2024-10-29
 * @brief CMSIS Peripheral Access Layer for FMU
 *
 * CMSIS Peripheral Access Layer for FMU
 */

#if !defined(PERI_FMU_H_)
#define PERI_FMU_H_                              /**< Symbol preventing repeated inclusion */

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
   -- FMU Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FMU_Peripheral_Access_Layer FMU Peripheral Access Layer
 * @{
 */

/** FMU - Size of Registers Arrays */
#define FMU_FCCOB_COUNT                           8u

/** FMU - Register Layout Typedef */
typedef struct {
  __IO uint32_t FSTAT;                             /**< Flash Status Register, offset: 0x0 */
  __IO uint32_t FCNFG;                             /**< Flash Configuration Register, offset: 0x4 */
  __IO uint32_t FCTRL;                             /**< Flash Control Register, offset: 0x8 */
       uint8_t RESERVED_0[4];
  __IO uint32_t FCCOB[FMU_FCCOB_COUNT];            /**< Flash Common Command Object Registers, array offset: 0x10, array step: 0x4 */
} FMU_Type;

/* ----------------------------------------------------------------------------
   -- FMU Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FMU_Register_Masks FMU Register Masks
 * @{
 */

/*! @name FSTAT - Flash Status Register */
/*! @{ */

#define FMU_FSTAT_FAIL_MASK                      (0x1U)
#define FMU_FSTAT_FAIL_SHIFT                     (0U)
/*! FAIL - Command Fail Flag
 *  0b0..Error not detected
 *  0b1..Error detected
 */
#define FMU_FSTAT_FAIL(x)                        (((uint32_t)(((uint32_t)(x)) << FMU_FSTAT_FAIL_SHIFT)) & FMU_FSTAT_FAIL_MASK)

#define FMU_FSTAT_CMDABT_MASK                    (0x4U)
#define FMU_FSTAT_CMDABT_SHIFT                   (2U)
/*! CMDABT - Command Abort Flag
 *  0b0..No command abort detected
 *  0b1..Command abort detected
 */
#define FMU_FSTAT_CMDABT(x)                      (((uint32_t)(((uint32_t)(x)) << FMU_FSTAT_CMDABT_SHIFT)) & FMU_FSTAT_CMDABT_MASK)

#define FMU_FSTAT_PVIOL_MASK                     (0x10U)
#define FMU_FSTAT_PVIOL_SHIFT                    (4U)
/*! PVIOL - Command Protection Violation Flag
 *  0b0..No protection violation detected
 *  0b1..Protection violation detected
 */
#define FMU_FSTAT_PVIOL(x)                       (((uint32_t)(((uint32_t)(x)) << FMU_FSTAT_PVIOL_SHIFT)) & FMU_FSTAT_PVIOL_MASK)

#define FMU_FSTAT_ACCERR_MASK                    (0x20U)
#define FMU_FSTAT_ACCERR_SHIFT                   (5U)
/*! ACCERR - Command Access Error Flag
 *  0b0..No access error detected
 *  0b1..Access error detected
 */
#define FMU_FSTAT_ACCERR(x)                      (((uint32_t)(((uint32_t)(x)) << FMU_FSTAT_ACCERR_SHIFT)) & FMU_FSTAT_ACCERR_MASK)

#define FMU_FSTAT_CWSABT_MASK                    (0x40U)
#define FMU_FSTAT_CWSABT_SHIFT                   (6U)
/*! CWSABT - Command Write Sequence Abort Flag
 *  0b0..Command write sequence not aborted
 *  0b1..Command write sequence aborted
 */
#define FMU_FSTAT_CWSABT(x)                      (((uint32_t)(((uint32_t)(x)) << FMU_FSTAT_CWSABT_SHIFT)) & FMU_FSTAT_CWSABT_MASK)

#define FMU_FSTAT_CCIF_MASK                      (0x80U)
#define FMU_FSTAT_CCIF_SHIFT                     (7U)
/*! CCIF - Command Complete Interrupt Flag
 *  0b0..Flash command, initialization, or power mode recovery in progress
 *  0b1..Flash command, initialization, or power mode recovery has completed
 */
#define FMU_FSTAT_CCIF(x)                        (((uint32_t)(((uint32_t)(x)) << FMU_FSTAT_CCIF_SHIFT)) & FMU_FSTAT_CCIF_MASK)

#define FMU_FSTAT_CMDPRT_MASK                    (0x300U)
#define FMU_FSTAT_CMDPRT_SHIFT                   (8U)
/*! CMDPRT - Command protection level
 *  0b00..Secure, normal access
 *  0b01..Secure, privileged access
 *  0b10..Nonsecure, normal access
 *  0b11..Nonsecure, privileged access
 */
#define FMU_FSTAT_CMDPRT(x)                      (((uint32_t)(((uint32_t)(x)) << FMU_FSTAT_CMDPRT_SHIFT)) & FMU_FSTAT_CMDPRT_MASK)

#define FMU_FSTAT_CMDP_MASK                      (0x800U)
#define FMU_FSTAT_CMDP_SHIFT                     (11U)
/*! CMDP - Command protection status flag
 *  0b0..Command protection level and domain ID are stale
 *  0b1..Command protection level (CMDPRT) and domain ID (CMDDID) are set
 */
#define FMU_FSTAT_CMDP(x)                        (((uint32_t)(((uint32_t)(x)) << FMU_FSTAT_CMDP_SHIFT)) & FMU_FSTAT_CMDP_MASK)

#define FMU_FSTAT_CMDDID_MASK                    (0xF000U)
#define FMU_FSTAT_CMDDID_SHIFT                   (12U)
/*! CMDDID - Command domain ID */
#define FMU_FSTAT_CMDDID(x)                      (((uint32_t)(((uint32_t)(x)) << FMU_FSTAT_CMDDID_SHIFT)) & FMU_FSTAT_CMDDID_MASK)

#define FMU_FSTAT_DFDIF_MASK                     (0x10000U)
#define FMU_FSTAT_DFDIF_SHIFT                    (16U)
/*! DFDIF - Double Bit Fault Detect Interrupt Flag
 *  0b0..Double bit fault not detected during a valid flash read access
 *  0b1..Double bit fault detected (or FCTRL[FDFD] is set) during a valid flash read access
 */
#define FMU_FSTAT_DFDIF(x)                       (((uint32_t)(((uint32_t)(x)) << FMU_FSTAT_DFDIF_SHIFT)) & FMU_FSTAT_DFDIF_MASK)

#define FMU_FSTAT_SALV_USED_MASK                 (0x20000U)
#define FMU_FSTAT_SALV_USED_SHIFT                (17U)
/*! SALV_USED - Salvage Used for Erase operation
 *  0b0..Salvage not used during last operation
 *  0b1..Salvage used during the last erase operation
 */
#define FMU_FSTAT_SALV_USED(x)                   (((uint32_t)(((uint32_t)(x)) << FMU_FSTAT_SALV_USED_SHIFT)) & FMU_FSTAT_SALV_USED_MASK)

#define FMU_FSTAT_PEWEN_MASK                     (0x3000000U)
#define FMU_FSTAT_PEWEN_SHIFT                    (24U)
/*! PEWEN - Program-Erase Write Enable Control
 *  0b00..Writes are not enabled
 *  0b01..Writes are enabled for one flash or IFR phrase (phrase programming, sector erase)
 *  0b10..Writes are enabled for one flash or IFR page (page programming)
 *  0b11..Reserved
 */
#define FMU_FSTAT_PEWEN(x)                       (((uint32_t)(((uint32_t)(x)) << FMU_FSTAT_PEWEN_SHIFT)) & FMU_FSTAT_PEWEN_MASK)

#define FMU_FSTAT_PERDY_MASK                     (0x80000000U)
#define FMU_FSTAT_PERDY_SHIFT                    (31U)
/*! PERDY - Program-Erase Ready Control/Status Flag
 *  0b0..Program or sector erase command operation not stalled
 *  0b1..Program or sector erase command operation ready to execute
 */
#define FMU_FSTAT_PERDY(x)                       (((uint32_t)(((uint32_t)(x)) << FMU_FSTAT_PERDY_SHIFT)) & FMU_FSTAT_PERDY_MASK)
/*! @} */

/*! @name FCNFG - Flash Configuration Register */
/*! @{ */

#define FMU_FCNFG_CCIE_MASK                      (0x80U)
#define FMU_FCNFG_CCIE_SHIFT                     (7U)
/*! CCIE - Command Complete Interrupt Enable
 *  0b0..Command complete interrupt disabled
 *  0b1..Command complete interrupt enabled
 */
#define FMU_FCNFG_CCIE(x)                        (((uint32_t)(((uint32_t)(x)) << FMU_FCNFG_CCIE_SHIFT)) & FMU_FCNFG_CCIE_MASK)

#define FMU_FCNFG_ERSREQ_MASK                    (0x100U)
#define FMU_FCNFG_ERSREQ_SHIFT                   (8U)
/*! ERSREQ - Mass Erase Request
 *  0b0..No request or request complete
 *  0b1..Request to run the Mass Erase operation
 */
#define FMU_FCNFG_ERSREQ(x)                      (((uint32_t)(((uint32_t)(x)) << FMU_FCNFG_ERSREQ_SHIFT)) & FMU_FCNFG_ERSREQ_MASK)

#define FMU_FCNFG_DFDIE_MASK                     (0x10000U)
#define FMU_FCNFG_DFDIE_SHIFT                    (16U)
/*! DFDIE - Double Bit Fault Detect Interrupt Enable
 *  0b0..Double bit fault detect interrupt disabled
 *  0b1..Double bit fault detect interrupt enabled
 */
#define FMU_FCNFG_DFDIE(x)                       (((uint32_t)(((uint32_t)(x)) << FMU_FCNFG_DFDIE_SHIFT)) & FMU_FCNFG_DFDIE_MASK)

#define FMU_FCNFG_ERSIEN0_MASK                   (0xF000000U)
#define FMU_FCNFG_ERSIEN0_SHIFT                  (24U)
/*! ERSIEN0 - Erase IFR Sector Enable - Block 0
 *  0b0000..Block 0 IFR Sector X is protected from erase by ERSSCR command
 *  0b0001..Block 0 IFR Sector X is not protected from erase by ERSSCR command
 */
#define FMU_FCNFG_ERSIEN0(x)                     (((uint32_t)(((uint32_t)(x)) << FMU_FCNFG_ERSIEN0_SHIFT)) & FMU_FCNFG_ERSIEN0_MASK)

#define FMU_FCNFG_ERSIEN1_MASK                   (0xF0000000U)
#define FMU_FCNFG_ERSIEN1_SHIFT                  (28U)
/*! ERSIEN1 - Erase IFR Sector Enable - Block 1 (for dual block configs)
 *  0b0000..Block 1 IFR Sector X is protected from erase by ERSSCR command
 *  0b0001..Block 1 IFR Sector X is not protected from erase by ERSSCR command
 */
#define FMU_FCNFG_ERSIEN1(x)                     (((uint32_t)(((uint32_t)(x)) << FMU_FCNFG_ERSIEN1_SHIFT)) & FMU_FCNFG_ERSIEN1_MASK)
/*! @} */

/*! @name FCTRL - Flash Control Register */
/*! @{ */

#define FMU_FCTRL_RWSC_MASK                      (0xFU)
#define FMU_FCTRL_RWSC_SHIFT                     (0U)
/*! RWSC - Read Wait-State Control */
#define FMU_FCTRL_RWSC(x)                        (((uint32_t)(((uint32_t)(x)) << FMU_FCTRL_RWSC_SHIFT)) & FMU_FCTRL_RWSC_MASK)

#define FMU_FCTRL_LSACTIVE_MASK                  (0x100U)
#define FMU_FCTRL_LSACTIVE_SHIFT                 (8U)
/*! LSACTIVE - Low speed active mode
 *  0b0..Full speed active mode requested
 *  0b1..Low speed active mode requested
 */
#define FMU_FCTRL_LSACTIVE(x)                    (((uint32_t)(((uint32_t)(x)) << FMU_FCTRL_LSACTIVE_SHIFT)) & FMU_FCTRL_LSACTIVE_MASK)

#define FMU_FCTRL_FDFD_MASK                      (0x10000U)
#define FMU_FCTRL_FDFD_SHIFT                     (16U)
/*! FDFD - Force Double Bit Fault Detect
 *  0b0..FSTAT[DFDIF] sets only if a double bit fault is detected during a valid flash read access from the platform flash controller
 *  0b1..FSTAT[DFDIF] sets during any valid flash read access from the platform flash controller. An interrupt
 *       request is generated if the DFDIE bit is set.
 */
#define FMU_FCTRL_FDFD(x)                        (((uint32_t)(((uint32_t)(x)) << FMU_FCTRL_FDFD_SHIFT)) & FMU_FCTRL_FDFD_MASK)

#define FMU_FCTRL_ABTREQ_MASK                    (0x1000000U)
#define FMU_FCTRL_ABTREQ_SHIFT                   (24U)
/*! ABTREQ - Abort Request
 *  0b0..No request to abort a command write sequence
 *  0b1..Request to abort a command write sequence
 */
#define FMU_FCTRL_ABTREQ(x)                      (((uint32_t)(((uint32_t)(x)) << FMU_FCTRL_ABTREQ_SHIFT)) & FMU_FCTRL_ABTREQ_MASK)
/*! @} */

/*! @name FCCOB - Flash Common Command Object Registers */
/*! @{ */

#define FMU_FCCOB_CCOBn_MASK                     (0xFFFFFFFFU)
#define FMU_FCCOB_CCOBn_SHIFT                    (0U)
/*! CCOBn - CCOBn */
#define FMU_FCCOB_CCOBn(x)                       (((uint32_t)(((uint32_t)(x)) << FMU_FCCOB_CCOBn_SHIFT)) & FMU_FCCOB_CCOBn_MASK)
/*! @} */


/*!
 * @}
 */ /* end of group FMU_Register_Masks */


/*!
 * @}
 */ /* end of group FMU_Peripheral_Access_Layer */


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


#endif  /* PERI_FMU_H_ */

