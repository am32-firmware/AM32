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
**         CMSIS Peripheral Access Layer for TRDC
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
 * @file PERI_TRDC.h
 * @version 2.0
 * @date 2024-10-29
 * @brief CMSIS Peripheral Access Layer for TRDC
 *
 * CMSIS Peripheral Access Layer for TRDC
 */

#if !defined(PERI_TRDC_H_)
#define PERI_TRDC_H_                             /**< Symbol preventing repeated inclusion */

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
   -- TRDC Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup TRDC_Peripheral_Access_Layer TRDC Peripheral Access Layer
 * @{
 */

/** TRDC - Size of Registers Arrays */
#define MBC_MEM_GLBCFG_COUNT                      4u
#define MBC_MEMN_GLBAC_COUNT                      8u
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_COUNT 2u
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_COUNT 1u
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_COUNT 1u
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_COUNT 1u
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_COUNT 1u
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_COUNT 1u
#define TRDC_MBC_INDEX_COUNT                      1u

/** TRDC - Register Layout Typedef */
typedef struct {
  struct {                                         /* offset: 0x0, array step: 0x1CC */
    __IO uint32_t MBC_MEM_GLBCFG[MBC_MEM_GLBCFG_COUNT];   /**< MBC Global Configuration Register, array offset: 0x0, array step: index*0x1CC, index2*0x4 */
    __IO uint32_t MBC_NSE_BLK_INDEX;                 /**< MBC NonSecure Enable Block Index, array offset: 0x10, array step: 0x1CC */
    __IO uint32_t MBC_NSE_BLK_SET;                   /**< MBC NonSecure Enable Block Set, array offset: 0x14, array step: 0x1CC */
    __IO uint32_t MBC_NSE_BLK_CLR;                   /**< MBC NonSecure Enable Block Clear, array offset: 0x18, array step: 0x1CC */
    __IO uint32_t MBC_NSE_BLK_CLR_ALL;               /**< MBC NonSecure Enable Block Clear All, array offset: 0x1C, array step: 0x1CC */
    __IO uint32_t MBC_MEMN_GLBAC[MBC_MEMN_GLBAC_COUNT];   /**< MBC Global Access Control, array offset: 0x20, array step: index*0x1CC, index2*0x4 */
    __IO uint32_t MBC_DOM0_MEM0_BLK_CFG_W[TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_COUNT];   /**< MBC Memory Block Configuration Word, array offset: 0x40, array step: index*0x1CC, index2*0x4 */
         uint8_t RESERVED_0[248];
    __IO uint32_t MBC_DOM0_MEM0_BLK_NSE_W[TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_COUNT];   /**< MBC Memory Block NonSecure Enable Word, array offset: 0x140, array step: index*0x1CC, index2*0x4 */
         uint8_t RESERVED_1[60];
    __IO uint32_t MBC_DOM0_MEM1_BLK_CFG_W[TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_COUNT];   /**< MBC Memory Block Configuration Word, array offset: 0x180, array step: index*0x1CC, index2*0x4 */
         uint8_t RESERVED_2[28];
    __IO uint32_t MBC_DOM0_MEM1_BLK_NSE_W[TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_COUNT];   /**< MBC Memory Block NonSecure Enable Word, array offset: 0x1A0, array step: index*0x1CC, index2*0x4 */
         uint8_t RESERVED_3[4];
    __IO uint32_t MBC_DOM0_MEM2_BLK_CFG_W[TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_COUNT];   /**< MBC Memory Block Configuration Word, array offset: 0x1A8, array step: index*0x1CC, index2*0x4 */
         uint8_t RESERVED_4[28];
    __IO uint32_t MBC_DOM0_MEM2_BLK_NSE_W[TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_COUNT];   /**< MBC Memory Block NonSecure Enable Word, array offset: 0x1C8, array step: index*0x1CC, index2*0x4 */
  } MBC_INDEX[TRDC_MBC_INDEX_COUNT];
} TRDC_Type;

/* ----------------------------------------------------------------------------
   -- TRDC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup TRDC_Register_Masks TRDC Register Masks
 * @{
 */

/*! @name MBC_INDEX_MBC_MEM_GLBCFG - MBC Global Configuration Register */
/*! @{ */

#define TRDC_MBC_INDEX_MBC_MEM_GLBCFG_NBLKS_MASK (0x3FFU)
#define TRDC_MBC_INDEX_MBC_MEM_GLBCFG_NBLKS_SHIFT (0U)
/*! NBLKS - Number of blocks in this memory */
#define TRDC_MBC_INDEX_MBC_MEM_GLBCFG_NBLKS(x)   (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_MEM_GLBCFG_NBLKS_SHIFT)) & TRDC_MBC_INDEX_MBC_MEM_GLBCFG_NBLKS_MASK)

#define TRDC_MBC_INDEX_MBC_MEM_GLBCFG_SIZE_LOG2_MASK (0x1F0000U)
#define TRDC_MBC_INDEX_MBC_MEM_GLBCFG_SIZE_LOG2_SHIFT (16U)
/*! SIZE_LOG2 - Log2 size per block */
#define TRDC_MBC_INDEX_MBC_MEM_GLBCFG_SIZE_LOG2(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_MEM_GLBCFG_SIZE_LOG2_SHIFT)) & TRDC_MBC_INDEX_MBC_MEM_GLBCFG_SIZE_LOG2_MASK)

#define TRDC_MBC_INDEX_MBC_MEM_GLBCFG_CLRE_MASK  (0xC0000000U)
#define TRDC_MBC_INDEX_MBC_MEM_GLBCFG_CLRE_SHIFT (30U)
/*! CLRE - Clear Error */
#define TRDC_MBC_INDEX_MBC_MEM_GLBCFG_CLRE(x)    (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_MEM_GLBCFG_CLRE_SHIFT)) & TRDC_MBC_INDEX_MBC_MEM_GLBCFG_CLRE_MASK)
/*! @} */

/* The count of TRDC_MBC_INDEX_MBC_MEM_GLBCFG */
#define TRDC_MBC_INDEX_MBC_MEM_GLBCFG_COUNT      (1U)

/* The count of TRDC_MBC_INDEX_MBC_MEM_GLBCFG */
#define TRDC_MBC_INDEX_MBC_MEM_GLBCFG_COUNT2     (4U)

/*! @name MBC_INDEX_MBC_NSE_BLK_INDEX - MBC NonSecure Enable Block Index */
/*! @{ */

#define TRDC_MBC_INDEX_MBC_NSE_BLK_INDEX_WNDX_MASK (0x3CU)
#define TRDC_MBC_INDEX_MBC_NSE_BLK_INDEX_WNDX_SHIFT (2U)
/*! WNDX - Word index into the block NSE bitmap. It selects the BLK_NSE_Wn register, where WNDX determines the value of n. */
#define TRDC_MBC_INDEX_MBC_NSE_BLK_INDEX_WNDX(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_NSE_BLK_INDEX_WNDX_SHIFT)) & TRDC_MBC_INDEX_MBC_NSE_BLK_INDEX_WNDX_MASK)

#define TRDC_MBC_INDEX_MBC_NSE_BLK_INDEX_MEM_SEL_MASK (0xF00U)
#define TRDC_MBC_INDEX_MBC_NSE_BLK_INDEX_MEM_SEL_SHIFT (8U)
/*! MEM_SEL - Memory Select */
#define TRDC_MBC_INDEX_MBC_NSE_BLK_INDEX_MEM_SEL(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_NSE_BLK_INDEX_MEM_SEL_SHIFT)) & TRDC_MBC_INDEX_MBC_NSE_BLK_INDEX_MEM_SEL_MASK)

#define TRDC_MBC_INDEX_MBC_NSE_BLK_INDEX_DID_SEL0_MASK (0x10000U)
#define TRDC_MBC_INDEX_MBC_NSE_BLK_INDEX_DID_SEL0_SHIFT (16U)
/*! DID_SEL0 - DID Select
 *  0b0..No effect.
 *  0b1..Selects NSE bits for this domain.
 */
#define TRDC_MBC_INDEX_MBC_NSE_BLK_INDEX_DID_SEL0(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_NSE_BLK_INDEX_DID_SEL0_SHIFT)) & TRDC_MBC_INDEX_MBC_NSE_BLK_INDEX_DID_SEL0_MASK)

#define TRDC_MBC_INDEX_MBC_NSE_BLK_INDEX_AI_MASK (0x80000000U)
#define TRDC_MBC_INDEX_MBC_NSE_BLK_INDEX_AI_SHIFT (31U)
/*! AI - Auto Increment
 *  0b0..No effect.
 *  0b1..Add 1 to the WNDX field after the register write.
 */
#define TRDC_MBC_INDEX_MBC_NSE_BLK_INDEX_AI(x)   (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_NSE_BLK_INDEX_AI_SHIFT)) & TRDC_MBC_INDEX_MBC_NSE_BLK_INDEX_AI_MASK)
/*! @} */

/* The count of TRDC_MBC_INDEX_MBC_NSE_BLK_INDEX */
#define TRDC_MBC_INDEX_MBC_NSE_BLK_INDEX_COUNT   (1U)

/*! @name MBC_INDEX_MBC_NSE_BLK_SET - MBC NonSecure Enable Block Set */
/*! @{ */

#define TRDC_MBC_INDEX_MBC_NSE_BLK_SET_W1SET_MASK (0xFFFFFFFFU)
#define TRDC_MBC_INDEX_MBC_NSE_BLK_SET_W1SET_SHIFT (0U)
/*! W1SET - Write-1 Set */
#define TRDC_MBC_INDEX_MBC_NSE_BLK_SET_W1SET(x)  (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_NSE_BLK_SET_W1SET_SHIFT)) & TRDC_MBC_INDEX_MBC_NSE_BLK_SET_W1SET_MASK)
/*! @} */

/* The count of TRDC_MBC_INDEX_MBC_NSE_BLK_SET */
#define TRDC_MBC_INDEX_MBC_NSE_BLK_SET_COUNT     (1U)

/*! @name MBC_INDEX_MBC_NSE_BLK_CLR - MBC NonSecure Enable Block Clear */
/*! @{ */

#define TRDC_MBC_INDEX_MBC_NSE_BLK_CLR_W1CLR_MASK (0xFFFFFFFFU)
#define TRDC_MBC_INDEX_MBC_NSE_BLK_CLR_W1CLR_SHIFT (0U)
/*! W1CLR - Write-1 Clear */
#define TRDC_MBC_INDEX_MBC_NSE_BLK_CLR_W1CLR(x)  (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_NSE_BLK_CLR_W1CLR_SHIFT)) & TRDC_MBC_INDEX_MBC_NSE_BLK_CLR_W1CLR_MASK)
/*! @} */

/* The count of TRDC_MBC_INDEX_MBC_NSE_BLK_CLR */
#define TRDC_MBC_INDEX_MBC_NSE_BLK_CLR_COUNT     (1U)

/*! @name MBC_INDEX_MBC_NSE_BLK_CLR_ALL - MBC NonSecure Enable Block Clear All */
/*! @{ */

#define TRDC_MBC_INDEX_MBC_NSE_BLK_CLR_ALL_MEMSEL_MASK (0xF00U)
#define TRDC_MBC_INDEX_MBC_NSE_BLK_CLR_ALL_MEMSEL_SHIFT (8U)
/*! MEMSEL - Memory Select */
#define TRDC_MBC_INDEX_MBC_NSE_BLK_CLR_ALL_MEMSEL(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_NSE_BLK_CLR_ALL_MEMSEL_SHIFT)) & TRDC_MBC_INDEX_MBC_NSE_BLK_CLR_ALL_MEMSEL_MASK)

#define TRDC_MBC_INDEX_MBC_NSE_BLK_CLR_ALL_DID_SEL0_MASK (0x10000U)
#define TRDC_MBC_INDEX_MBC_NSE_BLK_CLR_ALL_DID_SEL0_SHIFT (16U)
/*! DID_SEL0 - DID Select
 *  0b0..No effect.
 *  0b1..Clear all NSE bits for this domain.
 */
#define TRDC_MBC_INDEX_MBC_NSE_BLK_CLR_ALL_DID_SEL0(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_NSE_BLK_CLR_ALL_DID_SEL0_SHIFT)) & TRDC_MBC_INDEX_MBC_NSE_BLK_CLR_ALL_DID_SEL0_MASK)
/*! @} */

/* The count of TRDC_MBC_INDEX_MBC_NSE_BLK_CLR_ALL */
#define TRDC_MBC_INDEX_MBC_NSE_BLK_CLR_ALL_COUNT (1U)

/*! @name MBC_INDEX_MBC_MEMN_GLBAC - MBC Global Access Control */
/*! @{ */

#define TRDC_MBC_INDEX_MBC_MEMN_GLBAC_NUX_MASK   (0x1U)
#define TRDC_MBC_INDEX_MBC_MEMN_GLBAC_NUX_SHIFT  (0U)
/*! NUX - NonsecureUser Execute
 *  0b0..Execute access is not allowed in Nonsecure User mode.
 *  0b1..Execute access is allowed in Nonsecure User mode.
 */
#define TRDC_MBC_INDEX_MBC_MEMN_GLBAC_NUX(x)     (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_MEMN_GLBAC_NUX_SHIFT)) & TRDC_MBC_INDEX_MBC_MEMN_GLBAC_NUX_MASK)

#define TRDC_MBC_INDEX_MBC_MEMN_GLBAC_NUW_MASK   (0x2U)
#define TRDC_MBC_INDEX_MBC_MEMN_GLBAC_NUW_SHIFT  (1U)
/*! NUW - NonsecureUser Write
 *  0b0..Write access is not allowed in Nonsecure User mode.
 *  0b1..Write access is allowed in Nonsecure User mode.
 */
#define TRDC_MBC_INDEX_MBC_MEMN_GLBAC_NUW(x)     (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_MEMN_GLBAC_NUW_SHIFT)) & TRDC_MBC_INDEX_MBC_MEMN_GLBAC_NUW_MASK)

#define TRDC_MBC_INDEX_MBC_MEMN_GLBAC_NUR_MASK   (0x4U)
#define TRDC_MBC_INDEX_MBC_MEMN_GLBAC_NUR_SHIFT  (2U)
/*! NUR - NonsecureUser Read
 *  0b0..Read access is not allowed in Nonsecure User mode.
 *  0b1..Read access is allowed in Nonsecure User mode.
 */
#define TRDC_MBC_INDEX_MBC_MEMN_GLBAC_NUR(x)     (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_MEMN_GLBAC_NUR_SHIFT)) & TRDC_MBC_INDEX_MBC_MEMN_GLBAC_NUR_MASK)

#define TRDC_MBC_INDEX_MBC_MEMN_GLBAC_NPX_MASK   (0x10U)
#define TRDC_MBC_INDEX_MBC_MEMN_GLBAC_NPX_SHIFT  (4U)
/*! NPX - NonsecurePriv Execute
 *  0b0..Execute access is not allowed in Nonsecure Privilege mode.
 *  0b1..Execute access is allowed in Nonsecure Privilege mode.
 */
#define TRDC_MBC_INDEX_MBC_MEMN_GLBAC_NPX(x)     (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_MEMN_GLBAC_NPX_SHIFT)) & TRDC_MBC_INDEX_MBC_MEMN_GLBAC_NPX_MASK)

#define TRDC_MBC_INDEX_MBC_MEMN_GLBAC_NPW_MASK   (0x20U)
#define TRDC_MBC_INDEX_MBC_MEMN_GLBAC_NPW_SHIFT  (5U)
/*! NPW - NonsecurePriv Write
 *  0b0..Write access is not allowed in Nonsecure Privilege mode.
 *  0b1..Write access is allowed in Nonsecure Privilege mode.
 */
#define TRDC_MBC_INDEX_MBC_MEMN_GLBAC_NPW(x)     (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_MEMN_GLBAC_NPW_SHIFT)) & TRDC_MBC_INDEX_MBC_MEMN_GLBAC_NPW_MASK)

#define TRDC_MBC_INDEX_MBC_MEMN_GLBAC_NPR_MASK   (0x40U)
#define TRDC_MBC_INDEX_MBC_MEMN_GLBAC_NPR_SHIFT  (6U)
/*! NPR - NonsecurePriv Read
 *  0b0..Read access is not allowed in Nonsecure Privilege mode.
 *  0b1..Read access is allowed in Nonsecure Privilege mode.
 */
#define TRDC_MBC_INDEX_MBC_MEMN_GLBAC_NPR(x)     (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_MEMN_GLBAC_NPR_SHIFT)) & TRDC_MBC_INDEX_MBC_MEMN_GLBAC_NPR_MASK)

#define TRDC_MBC_INDEX_MBC_MEMN_GLBAC_SUX_MASK   (0x100U)
#define TRDC_MBC_INDEX_MBC_MEMN_GLBAC_SUX_SHIFT  (8U)
/*! SUX - SecureUser Execute
 *  0b0..Execute access is not allowed in Secure User mode.
 *  0b1..Execute access is allowed in Secure User mode.
 */
#define TRDC_MBC_INDEX_MBC_MEMN_GLBAC_SUX(x)     (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_MEMN_GLBAC_SUX_SHIFT)) & TRDC_MBC_INDEX_MBC_MEMN_GLBAC_SUX_MASK)

#define TRDC_MBC_INDEX_MBC_MEMN_GLBAC_SUW_MASK   (0x200U)
#define TRDC_MBC_INDEX_MBC_MEMN_GLBAC_SUW_SHIFT  (9U)
/*! SUW - SecureUser Write
 *  0b0..Write access is not allowed in Secure User mode.
 *  0b1..Write access is allowed in Secure User mode.
 */
#define TRDC_MBC_INDEX_MBC_MEMN_GLBAC_SUW(x)     (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_MEMN_GLBAC_SUW_SHIFT)) & TRDC_MBC_INDEX_MBC_MEMN_GLBAC_SUW_MASK)

#define TRDC_MBC_INDEX_MBC_MEMN_GLBAC_SUR_MASK   (0x400U)
#define TRDC_MBC_INDEX_MBC_MEMN_GLBAC_SUR_SHIFT  (10U)
/*! SUR - SecureUser Read
 *  0b0..Read access is not allowed in Secure User mode.
 *  0b1..Read access is allowed in Secure User mode.
 */
#define TRDC_MBC_INDEX_MBC_MEMN_GLBAC_SUR(x)     (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_MEMN_GLBAC_SUR_SHIFT)) & TRDC_MBC_INDEX_MBC_MEMN_GLBAC_SUR_MASK)

#define TRDC_MBC_INDEX_MBC_MEMN_GLBAC_SPX_MASK   (0x1000U)
#define TRDC_MBC_INDEX_MBC_MEMN_GLBAC_SPX_SHIFT  (12U)
/*! SPX - SecurePriv Execute
 *  0b0..Execute access is not allowed in Secure Privilege mode.
 *  0b1..Execute access is allowed in Secure Privilege mode.
 */
#define TRDC_MBC_INDEX_MBC_MEMN_GLBAC_SPX(x)     (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_MEMN_GLBAC_SPX_SHIFT)) & TRDC_MBC_INDEX_MBC_MEMN_GLBAC_SPX_MASK)

#define TRDC_MBC_INDEX_MBC_MEMN_GLBAC_SPW_MASK   (0x2000U)
#define TRDC_MBC_INDEX_MBC_MEMN_GLBAC_SPW_SHIFT  (13U)
/*! SPW - SecurePriv Write
 *  0b0..Write access is not allowed in Secure Privilege mode.
 *  0b1..Write access is allowed in Secure Privilege mode.
 */
#define TRDC_MBC_INDEX_MBC_MEMN_GLBAC_SPW(x)     (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_MEMN_GLBAC_SPW_SHIFT)) & TRDC_MBC_INDEX_MBC_MEMN_GLBAC_SPW_MASK)

#define TRDC_MBC_INDEX_MBC_MEMN_GLBAC_SPR_MASK   (0x4000U)
#define TRDC_MBC_INDEX_MBC_MEMN_GLBAC_SPR_SHIFT  (14U)
/*! SPR - SecurePriv Read
 *  0b0..Read access is not allowed in Secure Privilege mode.
 *  0b1..Read access is allowed in Secure Privilege mode.
 */
#define TRDC_MBC_INDEX_MBC_MEMN_GLBAC_SPR(x)     (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_MEMN_GLBAC_SPR_SHIFT)) & TRDC_MBC_INDEX_MBC_MEMN_GLBAC_SPR_MASK)

#define TRDC_MBC_INDEX_MBC_MEMN_GLBAC_LK_MASK    (0x80000000U)
#define TRDC_MBC_INDEX_MBC_MEMN_GLBAC_LK_SHIFT   (31U)
/*! LK - LOCK
 *  0b0..This register is not locked and can be altered.
 *  0b1..This register is locked and cannot be altered.
 */
#define TRDC_MBC_INDEX_MBC_MEMN_GLBAC_LK(x)      (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_MEMN_GLBAC_LK_SHIFT)) & TRDC_MBC_INDEX_MBC_MEMN_GLBAC_LK_MASK)
/*! @} */

/* The count of TRDC_MBC_INDEX_MBC_MEMN_GLBAC */
#define TRDC_MBC_INDEX_MBC_MEMN_GLBAC_COUNT      (1U)

/* The count of TRDC_MBC_INDEX_MBC_MEMN_GLBAC */
#define TRDC_MBC_INDEX_MBC_MEMN_GLBAC_COUNT2     (8U)

/*! @name MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W - MBC Memory Block Configuration Word */
/*! @{ */

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_MBACSEL0_MASK (0x7U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_MBACSEL0_SHIFT (0U)
/*! MBACSEL0 - Memory Block Access Control Select for block B
 *  0b000..select MBC_MEMN_GLBAC0 access control policy for block B
 *  0b001..select MBC_MEMN_GLBAC1 access control policy for block B
 *  0b010..select MBC_MEMN_GLBAC2 access control policy for block B
 *  0b011..select MBC_MEMN_GLBAC3 access control policy for block B
 *  0b100..select MBC_MEMN_GLBAC4 access control policy for block B
 *  0b101..select MBC_MEMN_GLBAC5 access control policy for block B
 *  0b110..select MBC_MEMN_GLBAC6 access control policy for block B
 *  0b111..select MBC_MEMN_GLBAC7 access control policy for block B
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_MBACSEL0(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_MBACSEL0_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_MBACSEL0_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_NSE0_MASK (0x8U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_NSE0_SHIFT (3U)
/*! NSE0 - NonSecure Enable for block B
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in this register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in this register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_NSE0(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_NSE0_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_NSE0_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_MBACSEL1_MASK (0x70U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_MBACSEL1_SHIFT (4U)
/*! MBACSEL1 - Memory Block Access Control Select for block B
 *  0b000..select MBC_MEMN_GLBAC0 access control policy for block B
 *  0b001..select MBC_MEMN_GLBAC1 access control policy for block B
 *  0b010..select MBC_MEMN_GLBAC2 access control policy for block B
 *  0b011..select MBC_MEMN_GLBAC3 access control policy for block B
 *  0b100..select MBC_MEMN_GLBAC4 access control policy for block B
 *  0b101..select MBC_MEMN_GLBAC5 access control policy for block B
 *  0b110..select MBC_MEMN_GLBAC6 access control policy for block B
 *  0b111..select MBC_MEMN_GLBAC7 access control policy for block B
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_MBACSEL1(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_MBACSEL1_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_MBACSEL1_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_NSE1_MASK (0x80U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_NSE1_SHIFT (7U)
/*! NSE1 - NonSecure Enable for block B
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in this register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in this register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_NSE1(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_NSE1_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_NSE1_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_MBACSEL2_MASK (0x700U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_MBACSEL2_SHIFT (8U)
/*! MBACSEL2 - Memory Block Access Control Select for block B
 *  0b000..select MBC_MEMN_GLBAC0 access control policy for block B
 *  0b001..select MBC_MEMN_GLBAC1 access control policy for block B
 *  0b010..select MBC_MEMN_GLBAC2 access control policy for block B
 *  0b011..select MBC_MEMN_GLBAC3 access control policy for block B
 *  0b100..select MBC_MEMN_GLBAC4 access control policy for block B
 *  0b101..select MBC_MEMN_GLBAC5 access control policy for block B
 *  0b110..select MBC_MEMN_GLBAC6 access control policy for block B
 *  0b111..select MBC_MEMN_GLBAC7 access control policy for block B
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_MBACSEL2(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_MBACSEL2_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_MBACSEL2_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_NSE2_MASK (0x800U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_NSE2_SHIFT (11U)
/*! NSE2 - NonSecure Enable for block B
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in this register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in this register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_NSE2(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_NSE2_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_NSE2_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_MBACSEL3_MASK (0x7000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_MBACSEL3_SHIFT (12U)
/*! MBACSEL3 - Memory Block Access Control Select for block B
 *  0b000..select MBC_MEMN_GLBAC0 access control policy for block B
 *  0b001..select MBC_MEMN_GLBAC1 access control policy for block B
 *  0b010..select MBC_MEMN_GLBAC2 access control policy for block B
 *  0b011..select MBC_MEMN_GLBAC3 access control policy for block B
 *  0b100..select MBC_MEMN_GLBAC4 access control policy for block B
 *  0b101..select MBC_MEMN_GLBAC5 access control policy for block B
 *  0b110..select MBC_MEMN_GLBAC6 access control policy for block B
 *  0b111..select MBC_MEMN_GLBAC7 access control policy for block B
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_MBACSEL3(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_MBACSEL3_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_MBACSEL3_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_NSE3_MASK (0x8000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_NSE3_SHIFT (15U)
/*! NSE3 - NonSecure Enable for block B
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in this register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in this register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_NSE3(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_NSE3_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_NSE3_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_MBACSEL4_MASK (0x70000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_MBACSEL4_SHIFT (16U)
/*! MBACSEL4 - Memory Block Access Control Select for block B
 *  0b000..select MBC_MEMN_GLBAC0 access control policy for block B
 *  0b001..select MBC_MEMN_GLBAC1 access control policy for block B
 *  0b010..select MBC_MEMN_GLBAC2 access control policy for block B
 *  0b011..select MBC_MEMN_GLBAC3 access control policy for block B
 *  0b100..select MBC_MEMN_GLBAC4 access control policy for block B
 *  0b101..select MBC_MEMN_GLBAC5 access control policy for block B
 *  0b110..select MBC_MEMN_GLBAC6 access control policy for block B
 *  0b111..select MBC_MEMN_GLBAC7 access control policy for block B
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_MBACSEL4(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_MBACSEL4_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_MBACSEL4_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_NSE4_MASK (0x80000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_NSE4_SHIFT (19U)
/*! NSE4 - NonSecure Enable for block B
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in this register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in this register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_NSE4(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_NSE4_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_NSE4_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_MBACSEL5_MASK (0x700000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_MBACSEL5_SHIFT (20U)
/*! MBACSEL5 - Memory Block Access Control Select for block B
 *  0b000..select MBC_MEMN_GLBAC0 access control policy for block B
 *  0b001..select MBC_MEMN_GLBAC1 access control policy for block B
 *  0b010..select MBC_MEMN_GLBAC2 access control policy for block B
 *  0b011..select MBC_MEMN_GLBAC3 access control policy for block B
 *  0b100..select MBC_MEMN_GLBAC4 access control policy for block B
 *  0b101..select MBC_MEMN_GLBAC5 access control policy for block B
 *  0b110..select MBC_MEMN_GLBAC6 access control policy for block B
 *  0b111..select MBC_MEMN_GLBAC7 access control policy for block B
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_MBACSEL5(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_MBACSEL5_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_MBACSEL5_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_NSE5_MASK (0x800000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_NSE5_SHIFT (23U)
/*! NSE5 - NonSecure Enable for block B
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in this register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in this register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_NSE5(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_NSE5_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_NSE5_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_MBACSEL6_MASK (0x7000000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_MBACSEL6_SHIFT (24U)
/*! MBACSEL6 - Memory Block Access Control Select for block B
 *  0b000..select MBC_MEMN_GLBAC0 access control policy for block B
 *  0b001..select MBC_MEMN_GLBAC1 access control policy for block B
 *  0b010..select MBC_MEMN_GLBAC2 access control policy for block B
 *  0b011..select MBC_MEMN_GLBAC3 access control policy for block B
 *  0b100..select MBC_MEMN_GLBAC4 access control policy for block B
 *  0b101..select MBC_MEMN_GLBAC5 access control policy for block B
 *  0b110..select MBC_MEMN_GLBAC6 access control policy for block B
 *  0b111..select MBC_MEMN_GLBAC7 access control policy for block B
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_MBACSEL6(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_MBACSEL6_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_MBACSEL6_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_NSE6_MASK (0x8000000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_NSE6_SHIFT (27U)
/*! NSE6 - NonSecure Enable for block B
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in this register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in this register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_NSE6(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_NSE6_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_NSE6_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_MBACSEL7_MASK (0x70000000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_MBACSEL7_SHIFT (28U)
/*! MBACSEL7 - Memory Block Access Control Select for block B
 *  0b000..select MBC_MEMN_GLBAC0 access control policy for block B
 *  0b001..select MBC_MEMN_GLBAC1 access control policy for block B
 *  0b010..select MBC_MEMN_GLBAC2 access control policy for block B
 *  0b011..select MBC_MEMN_GLBAC3 access control policy for block B
 *  0b100..select MBC_MEMN_GLBAC4 access control policy for block B
 *  0b101..select MBC_MEMN_GLBAC5 access control policy for block B
 *  0b110..select MBC_MEMN_GLBAC6 access control policy for block B
 *  0b111..select MBC_MEMN_GLBAC7 access control policy for block B
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_MBACSEL7(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_MBACSEL7_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_MBACSEL7_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_NSE7_MASK (0x80000000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_NSE7_SHIFT (31U)
/*! NSE7 - NonSecure Enable for block B
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in this register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in this register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_NSE7(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_NSE7_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_NSE7_MASK)
/*! @} */

/* The count of TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_COUNT (1U)

/* The count of TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_CFG_W_MBC_DOM0_MEM0_BLK_CFG_W_COUNT2 (2U)

/*! @name MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W - MBC Memory Block NonSecure Enable Word */
/*! @{ */

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT0_MASK (0x1U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT0_SHIFT (0U)
/*! BIT0 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT0(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT0_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT0_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT1_MASK (0x2U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT1_SHIFT (1U)
/*! BIT1 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT1(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT1_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT1_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT2_MASK (0x4U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT2_SHIFT (2U)
/*! BIT2 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT2(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT2_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT2_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT3_MASK (0x8U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT3_SHIFT (3U)
/*! BIT3 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT3(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT3_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT3_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT4_MASK (0x10U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT4_SHIFT (4U)
/*! BIT4 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT4(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT4_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT4_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT5_MASK (0x20U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT5_SHIFT (5U)
/*! BIT5 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT5(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT5_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT5_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT6_MASK (0x40U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT6_SHIFT (6U)
/*! BIT6 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT6(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT6_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT6_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT7_MASK (0x80U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT7_SHIFT (7U)
/*! BIT7 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT7(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT7_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT7_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT8_MASK (0x100U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT8_SHIFT (8U)
/*! BIT8 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT8(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT8_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT8_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT9_MASK (0x200U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT9_SHIFT (9U)
/*! BIT9 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT9(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT9_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT9_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT10_MASK (0x400U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT10_SHIFT (10U)
/*! BIT10 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT10(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT10_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT10_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT11_MASK (0x800U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT11_SHIFT (11U)
/*! BIT11 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT11(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT11_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT11_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT12_MASK (0x1000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT12_SHIFT (12U)
/*! BIT12 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT12(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT12_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT12_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT13_MASK (0x2000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT13_SHIFT (13U)
/*! BIT13 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT13(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT13_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT13_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT14_MASK (0x4000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT14_SHIFT (14U)
/*! BIT14 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT14(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT14_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT14_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT15_MASK (0x8000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT15_SHIFT (15U)
/*! BIT15 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT15(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT15_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT15_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT16_MASK (0x10000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT16_SHIFT (16U)
/*! BIT16 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT16(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT16_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT16_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT17_MASK (0x20000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT17_SHIFT (17U)
/*! BIT17 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT17(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT17_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT17_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT18_MASK (0x40000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT18_SHIFT (18U)
/*! BIT18 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT18(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT18_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT18_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT19_MASK (0x80000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT19_SHIFT (19U)
/*! BIT19 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT19(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT19_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT19_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT20_MASK (0x100000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT20_SHIFT (20U)
/*! BIT20 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT20(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT20_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT20_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT21_MASK (0x200000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT21_SHIFT (21U)
/*! BIT21 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT21(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT21_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT21_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT22_MASK (0x400000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT22_SHIFT (22U)
/*! BIT22 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT22(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT22_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT22_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT23_MASK (0x800000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT23_SHIFT (23U)
/*! BIT23 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT23(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT23_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT23_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT24_MASK (0x1000000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT24_SHIFT (24U)
/*! BIT24 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT24(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT24_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT24_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT25_MASK (0x2000000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT25_SHIFT (25U)
/*! BIT25 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT25(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT25_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT25_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT26_MASK (0x4000000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT26_SHIFT (26U)
/*! BIT26 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT26(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT26_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT26_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT27_MASK (0x8000000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT27_SHIFT (27U)
/*! BIT27 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT27(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT27_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT27_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT28_MASK (0x10000000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT28_SHIFT (28U)
/*! BIT28 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT28(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT28_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT28_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT29_MASK (0x20000000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT29_SHIFT (29U)
/*! BIT29 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT29(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT29_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT29_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT30_MASK (0x40000000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT30_SHIFT (30U)
/*! BIT30 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT30(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT30_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT30_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT31_MASK (0x80000000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT31_SHIFT (31U)
/*! BIT31 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT31(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT31_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_BIT31_MASK)
/*! @} */

/* The count of TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_COUNT (1U)

/* The count of TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM0_BLK_NSE_W_MBC_DOM0_MEM0_BLK_NSE_W_COUNT2 (1U)

/*! @name MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W - MBC Memory Block Configuration Word */
/*! @{ */

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_MBACSEL0_MASK (0x7U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_MBACSEL0_SHIFT (0U)
/*! MBACSEL0 - Memory Block Access Control Select for block B
 *  0b000..select MBC_MEMN_GLBAC0 access control policy for block B
 *  0b001..select MBC_MEMN_GLBAC1 access control policy for block B
 *  0b010..select MBC_MEMN_GLBAC2 access control policy for block B
 *  0b011..select MBC_MEMN_GLBAC3 access control policy for block B
 *  0b100..select MBC_MEMN_GLBAC4 access control policy for block B
 *  0b101..select MBC_MEMN_GLBAC5 access control policy for block B
 *  0b110..select MBC_MEMN_GLBAC6 access control policy for block B
 *  0b111..select MBC_MEMN_GLBAC7 access control policy for block B
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_MBACSEL0(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_MBACSEL0_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_MBACSEL0_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_NSE0_MASK (0x8U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_NSE0_SHIFT (3U)
/*! NSE0 - NonSecure Enable for block B
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in this register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in this register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_NSE0(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_NSE0_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_NSE0_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_MBACSEL1_MASK (0x70U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_MBACSEL1_SHIFT (4U)
/*! MBACSEL1 - Memory Block Access Control Select for block B
 *  0b000..select MBC_MEMN_GLBAC0 access control policy for block B
 *  0b001..select MBC_MEMN_GLBAC1 access control policy for block B
 *  0b010..select MBC_MEMN_GLBAC2 access control policy for block B
 *  0b011..select MBC_MEMN_GLBAC3 access control policy for block B
 *  0b100..select MBC_MEMN_GLBAC4 access control policy for block B
 *  0b101..select MBC_MEMN_GLBAC5 access control policy for block B
 *  0b110..select MBC_MEMN_GLBAC6 access control policy for block B
 *  0b111..select MBC_MEMN_GLBAC7 access control policy for block B
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_MBACSEL1(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_MBACSEL1_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_MBACSEL1_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_NSE1_MASK (0x80U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_NSE1_SHIFT (7U)
/*! NSE1 - NonSecure Enable for block B
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in this register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in this register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_NSE1(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_NSE1_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_NSE1_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_MBACSEL2_MASK (0x700U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_MBACSEL2_SHIFT (8U)
/*! MBACSEL2 - Memory Block Access Control Select for block B
 *  0b000..select MBC_MEMN_GLBAC0 access control policy for block B
 *  0b001..select MBC_MEMN_GLBAC1 access control policy for block B
 *  0b010..select MBC_MEMN_GLBAC2 access control policy for block B
 *  0b011..select MBC_MEMN_GLBAC3 access control policy for block B
 *  0b100..select MBC_MEMN_GLBAC4 access control policy for block B
 *  0b101..select MBC_MEMN_GLBAC5 access control policy for block B
 *  0b110..select MBC_MEMN_GLBAC6 access control policy for block B
 *  0b111..select MBC_MEMN_GLBAC7 access control policy for block B
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_MBACSEL2(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_MBACSEL2_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_MBACSEL2_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_NSE2_MASK (0x800U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_NSE2_SHIFT (11U)
/*! NSE2 - NonSecure Enable for block B
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in this register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in this register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_NSE2(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_NSE2_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_NSE2_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_MBACSEL3_MASK (0x7000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_MBACSEL3_SHIFT (12U)
/*! MBACSEL3 - Memory Block Access Control Select for block B
 *  0b000..select MBC_MEMN_GLBAC0 access control policy for block B
 *  0b001..select MBC_MEMN_GLBAC1 access control policy for block B
 *  0b010..select MBC_MEMN_GLBAC2 access control policy for block B
 *  0b011..select MBC_MEMN_GLBAC3 access control policy for block B
 *  0b100..select MBC_MEMN_GLBAC4 access control policy for block B
 *  0b101..select MBC_MEMN_GLBAC5 access control policy for block B
 *  0b110..select MBC_MEMN_GLBAC6 access control policy for block B
 *  0b111..select MBC_MEMN_GLBAC7 access control policy for block B
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_MBACSEL3(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_MBACSEL3_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_MBACSEL3_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_NSE3_MASK (0x8000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_NSE3_SHIFT (15U)
/*! NSE3 - NonSecure Enable for block B
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in this register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in this register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_NSE3(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_NSE3_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_NSE3_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_MBACSEL4_MASK (0x70000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_MBACSEL4_SHIFT (16U)
/*! MBACSEL4 - Memory Block Access Control Select for block B
 *  0b000..select MBC_MEMN_GLBAC0 access control policy for block B
 *  0b001..select MBC_MEMN_GLBAC1 access control policy for block B
 *  0b010..select MBC_MEMN_GLBAC2 access control policy for block B
 *  0b011..select MBC_MEMN_GLBAC3 access control policy for block B
 *  0b100..select MBC_MEMN_GLBAC4 access control policy for block B
 *  0b101..select MBC_MEMN_GLBAC5 access control policy for block B
 *  0b110..select MBC_MEMN_GLBAC6 access control policy for block B
 *  0b111..select MBC_MEMN_GLBAC7 access control policy for block B
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_MBACSEL4(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_MBACSEL4_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_MBACSEL4_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_NSE4_MASK (0x80000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_NSE4_SHIFT (19U)
/*! NSE4 - NonSecure Enable for block B
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in this register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in this register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_NSE4(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_NSE4_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_NSE4_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_MBACSEL5_MASK (0x700000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_MBACSEL5_SHIFT (20U)
/*! MBACSEL5 - Memory Block Access Control Select for block B
 *  0b000..select MBC_MEMN_GLBAC0 access control policy for block B
 *  0b001..select MBC_MEMN_GLBAC1 access control policy for block B
 *  0b010..select MBC_MEMN_GLBAC2 access control policy for block B
 *  0b011..select MBC_MEMN_GLBAC3 access control policy for block B
 *  0b100..select MBC_MEMN_GLBAC4 access control policy for block B
 *  0b101..select MBC_MEMN_GLBAC5 access control policy for block B
 *  0b110..select MBC_MEMN_GLBAC6 access control policy for block B
 *  0b111..select MBC_MEMN_GLBAC7 access control policy for block B
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_MBACSEL5(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_MBACSEL5_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_MBACSEL5_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_NSE5_MASK (0x800000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_NSE5_SHIFT (23U)
/*! NSE5 - NonSecure Enable for block B
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in this register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in this register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_NSE5(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_NSE5_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_NSE5_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_MBACSEL6_MASK (0x7000000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_MBACSEL6_SHIFT (24U)
/*! MBACSEL6 - Memory Block Access Control Select for block B
 *  0b000..select MBC_MEMN_GLBAC0 access control policy for block B
 *  0b001..select MBC_MEMN_GLBAC1 access control policy for block B
 *  0b010..select MBC_MEMN_GLBAC2 access control policy for block B
 *  0b011..select MBC_MEMN_GLBAC3 access control policy for block B
 *  0b100..select MBC_MEMN_GLBAC4 access control policy for block B
 *  0b101..select MBC_MEMN_GLBAC5 access control policy for block B
 *  0b110..select MBC_MEMN_GLBAC6 access control policy for block B
 *  0b111..select MBC_MEMN_GLBAC7 access control policy for block B
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_MBACSEL6(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_MBACSEL6_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_MBACSEL6_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_NSE6_MASK (0x8000000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_NSE6_SHIFT (27U)
/*! NSE6 - NonSecure Enable for block B
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in this register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in this register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_NSE6(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_NSE6_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_NSE6_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_MBACSEL7_MASK (0x70000000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_MBACSEL7_SHIFT (28U)
/*! MBACSEL7 - Memory Block Access Control Select for block B
 *  0b000..select MBC_MEMN_GLBAC0 access control policy for block B
 *  0b001..select MBC_MEMN_GLBAC1 access control policy for block B
 *  0b010..select MBC_MEMN_GLBAC2 access control policy for block B
 *  0b011..select MBC_MEMN_GLBAC3 access control policy for block B
 *  0b100..select MBC_MEMN_GLBAC4 access control policy for block B
 *  0b101..select MBC_MEMN_GLBAC5 access control policy for block B
 *  0b110..select MBC_MEMN_GLBAC6 access control policy for block B
 *  0b111..select MBC_MEMN_GLBAC7 access control policy for block B
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_MBACSEL7(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_MBACSEL7_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_MBACSEL7_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_NSE7_MASK (0x80000000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_NSE7_SHIFT (31U)
/*! NSE7 - NonSecure Enable for block B
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in this register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in this register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_NSE7(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_NSE7_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_NSE7_MASK)
/*! @} */

/* The count of TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_COUNT (1U)

/* The count of TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_CFG_W_MBC_DOM0_MEM1_BLK_CFG_W_COUNT2 (1U)

/*! @name MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W - MBC Memory Block NonSecure Enable Word */
/*! @{ */

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT0_MASK (0x1U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT0_SHIFT (0U)
/*! BIT0 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT0(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT0_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT0_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT1_MASK (0x2U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT1_SHIFT (1U)
/*! BIT1 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT1(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT1_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT1_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT2_MASK (0x4U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT2_SHIFT (2U)
/*! BIT2 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT2(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT2_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT2_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT3_MASK (0x8U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT3_SHIFT (3U)
/*! BIT3 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT3(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT3_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT3_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT4_MASK (0x10U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT4_SHIFT (4U)
/*! BIT4 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT4(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT4_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT4_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT5_MASK (0x20U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT5_SHIFT (5U)
/*! BIT5 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT5(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT5_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT5_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT6_MASK (0x40U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT6_SHIFT (6U)
/*! BIT6 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT6(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT6_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT6_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT7_MASK (0x80U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT7_SHIFT (7U)
/*! BIT7 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT7(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT7_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT7_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT8_MASK (0x100U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT8_SHIFT (8U)
/*! BIT8 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT8(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT8_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT8_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT9_MASK (0x200U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT9_SHIFT (9U)
/*! BIT9 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT9(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT9_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT9_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT10_MASK (0x400U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT10_SHIFT (10U)
/*! BIT10 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT10(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT10_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT10_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT11_MASK (0x800U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT11_SHIFT (11U)
/*! BIT11 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT11(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT11_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT11_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT12_MASK (0x1000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT12_SHIFT (12U)
/*! BIT12 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT12(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT12_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT12_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT13_MASK (0x2000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT13_SHIFT (13U)
/*! BIT13 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT13(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT13_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT13_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT14_MASK (0x4000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT14_SHIFT (14U)
/*! BIT14 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT14(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT14_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT14_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT15_MASK (0x8000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT15_SHIFT (15U)
/*! BIT15 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT15(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT15_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT15_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT16_MASK (0x10000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT16_SHIFT (16U)
/*! BIT16 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT16(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT16_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT16_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT17_MASK (0x20000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT17_SHIFT (17U)
/*! BIT17 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT17(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT17_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT17_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT18_MASK (0x40000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT18_SHIFT (18U)
/*! BIT18 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT18(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT18_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT18_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT19_MASK (0x80000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT19_SHIFT (19U)
/*! BIT19 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT19(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT19_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT19_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT20_MASK (0x100000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT20_SHIFT (20U)
/*! BIT20 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT20(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT20_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT20_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT21_MASK (0x200000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT21_SHIFT (21U)
/*! BIT21 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT21(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT21_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT21_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT22_MASK (0x400000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT22_SHIFT (22U)
/*! BIT22 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT22(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT22_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT22_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT23_MASK (0x800000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT23_SHIFT (23U)
/*! BIT23 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT23(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT23_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT23_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT24_MASK (0x1000000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT24_SHIFT (24U)
/*! BIT24 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT24(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT24_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT24_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT25_MASK (0x2000000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT25_SHIFT (25U)
/*! BIT25 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT25(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT25_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT25_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT26_MASK (0x4000000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT26_SHIFT (26U)
/*! BIT26 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT26(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT26_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT26_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT27_MASK (0x8000000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT27_SHIFT (27U)
/*! BIT27 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT27(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT27_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT27_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT28_MASK (0x10000000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT28_SHIFT (28U)
/*! BIT28 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT28(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT28_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT28_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT29_MASK (0x20000000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT29_SHIFT (29U)
/*! BIT29 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT29(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT29_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT29_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT30_MASK (0x40000000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT30_SHIFT (30U)
/*! BIT30 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT30(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT30_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT30_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT31_MASK (0x80000000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT31_SHIFT (31U)
/*! BIT31 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT31(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT31_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_BIT31_MASK)
/*! @} */

/* The count of TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_COUNT (1U)

/* The count of TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM1_BLK_NSE_W_MBC_DOM0_MEM1_BLK_NSE_W_COUNT2 (1U)

/*! @name MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W - MBC Memory Block Configuration Word */
/*! @{ */

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_MBACSEL0_MASK (0x7U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_MBACSEL0_SHIFT (0U)
/*! MBACSEL0 - Memory Block Access Control Select for block B
 *  0b000..select MBC_MEMN_GLBAC0 access control policy for block B
 *  0b001..select MBC_MEMN_GLBAC1 access control policy for block B
 *  0b010..select MBC_MEMN_GLBAC2 access control policy for block B
 *  0b011..select MBC_MEMN_GLBAC3 access control policy for block B
 *  0b100..select MBC_MEMN_GLBAC4 access control policy for block B
 *  0b101..select MBC_MEMN_GLBAC5 access control policy for block B
 *  0b110..select MBC_MEMN_GLBAC6 access control policy for block B
 *  0b111..select MBC_MEMN_GLBAC7 access control policy for block B
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_MBACSEL0(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_MBACSEL0_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_MBACSEL0_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_NSE0_MASK (0x8U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_NSE0_SHIFT (3U)
/*! NSE0 - NonSecure Enable for block B
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in this register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in this register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_NSE0(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_NSE0_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_NSE0_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_MBACSEL1_MASK (0x70U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_MBACSEL1_SHIFT (4U)
/*! MBACSEL1 - Memory Block Access Control Select for block B
 *  0b000..select MBC_MEMN_GLBAC0 access control policy for block B
 *  0b001..select MBC_MEMN_GLBAC1 access control policy for block B
 *  0b010..select MBC_MEMN_GLBAC2 access control policy for block B
 *  0b011..select MBC_MEMN_GLBAC3 access control policy for block B
 *  0b100..select MBC_MEMN_GLBAC4 access control policy for block B
 *  0b101..select MBC_MEMN_GLBAC5 access control policy for block B
 *  0b110..select MBC_MEMN_GLBAC6 access control policy for block B
 *  0b111..select MBC_MEMN_GLBAC7 access control policy for block B
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_MBACSEL1(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_MBACSEL1_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_MBACSEL1_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_NSE1_MASK (0x80U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_NSE1_SHIFT (7U)
/*! NSE1 - NonSecure Enable for block B
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in this register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in this register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_NSE1(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_NSE1_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_NSE1_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_MBACSEL2_MASK (0x700U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_MBACSEL2_SHIFT (8U)
/*! MBACSEL2 - Memory Block Access Control Select for block B
 *  0b000..select MBC_MEMN_GLBAC0 access control policy for block B
 *  0b001..select MBC_MEMN_GLBAC1 access control policy for block B
 *  0b010..select MBC_MEMN_GLBAC2 access control policy for block B
 *  0b011..select MBC_MEMN_GLBAC3 access control policy for block B
 *  0b100..select MBC_MEMN_GLBAC4 access control policy for block B
 *  0b101..select MBC_MEMN_GLBAC5 access control policy for block B
 *  0b110..select MBC_MEMN_GLBAC6 access control policy for block B
 *  0b111..select MBC_MEMN_GLBAC7 access control policy for block B
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_MBACSEL2(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_MBACSEL2_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_MBACSEL2_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_NSE2_MASK (0x800U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_NSE2_SHIFT (11U)
/*! NSE2 - NonSecure Enable for block B
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in this register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in this register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_NSE2(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_NSE2_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_NSE2_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_MBACSEL3_MASK (0x7000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_MBACSEL3_SHIFT (12U)
/*! MBACSEL3 - Memory Block Access Control Select for block B
 *  0b000..select MBC_MEMN_GLBAC0 access control policy for block B
 *  0b001..select MBC_MEMN_GLBAC1 access control policy for block B
 *  0b010..select MBC_MEMN_GLBAC2 access control policy for block B
 *  0b011..select MBC_MEMN_GLBAC3 access control policy for block B
 *  0b100..select MBC_MEMN_GLBAC4 access control policy for block B
 *  0b101..select MBC_MEMN_GLBAC5 access control policy for block B
 *  0b110..select MBC_MEMN_GLBAC6 access control policy for block B
 *  0b111..select MBC_MEMN_GLBAC7 access control policy for block B
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_MBACSEL3(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_MBACSEL3_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_MBACSEL3_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_NSE3_MASK (0x8000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_NSE3_SHIFT (15U)
/*! NSE3 - NonSecure Enable for block B
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in this register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in this register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_NSE3(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_NSE3_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_NSE3_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_MBACSEL4_MASK (0x70000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_MBACSEL4_SHIFT (16U)
/*! MBACSEL4 - Memory Block Access Control Select for block B
 *  0b000..select MBC_MEMN_GLBAC0 access control policy for block B
 *  0b001..select MBC_MEMN_GLBAC1 access control policy for block B
 *  0b010..select MBC_MEMN_GLBAC2 access control policy for block B
 *  0b011..select MBC_MEMN_GLBAC3 access control policy for block B
 *  0b100..select MBC_MEMN_GLBAC4 access control policy for block B
 *  0b101..select MBC_MEMN_GLBAC5 access control policy for block B
 *  0b110..select MBC_MEMN_GLBAC6 access control policy for block B
 *  0b111..select MBC_MEMN_GLBAC7 access control policy for block B
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_MBACSEL4(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_MBACSEL4_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_MBACSEL4_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_NSE4_MASK (0x80000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_NSE4_SHIFT (19U)
/*! NSE4 - NonSecure Enable for block B
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in this register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in this register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_NSE4(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_NSE4_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_NSE4_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_MBACSEL5_MASK (0x700000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_MBACSEL5_SHIFT (20U)
/*! MBACSEL5 - Memory Block Access Control Select for block B
 *  0b000..select MBC_MEMN_GLBAC0 access control policy for block B
 *  0b001..select MBC_MEMN_GLBAC1 access control policy for block B
 *  0b010..select MBC_MEMN_GLBAC2 access control policy for block B
 *  0b011..select MBC_MEMN_GLBAC3 access control policy for block B
 *  0b100..select MBC_MEMN_GLBAC4 access control policy for block B
 *  0b101..select MBC_MEMN_GLBAC5 access control policy for block B
 *  0b110..select MBC_MEMN_GLBAC6 access control policy for block B
 *  0b111..select MBC_MEMN_GLBAC7 access control policy for block B
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_MBACSEL5(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_MBACSEL5_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_MBACSEL5_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_NSE5_MASK (0x800000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_NSE5_SHIFT (23U)
/*! NSE5 - NonSecure Enable for block B
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in this register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in this register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_NSE5(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_NSE5_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_NSE5_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_MBACSEL6_MASK (0x7000000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_MBACSEL6_SHIFT (24U)
/*! MBACSEL6 - Memory Block Access Control Select for block B
 *  0b000..select MBC_MEMN_GLBAC0 access control policy for block B
 *  0b001..select MBC_MEMN_GLBAC1 access control policy for block B
 *  0b010..select MBC_MEMN_GLBAC2 access control policy for block B
 *  0b011..select MBC_MEMN_GLBAC3 access control policy for block B
 *  0b100..select MBC_MEMN_GLBAC4 access control policy for block B
 *  0b101..select MBC_MEMN_GLBAC5 access control policy for block B
 *  0b110..select MBC_MEMN_GLBAC6 access control policy for block B
 *  0b111..select MBC_MEMN_GLBAC7 access control policy for block B
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_MBACSEL6(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_MBACSEL6_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_MBACSEL6_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_NSE6_MASK (0x8000000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_NSE6_SHIFT (27U)
/*! NSE6 - NonSecure Enable for block B
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in this register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in this register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_NSE6(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_NSE6_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_NSE6_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_MBACSEL7_MASK (0x70000000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_MBACSEL7_SHIFT (28U)
/*! MBACSEL7 - Memory Block Access Control Select for block B
 *  0b000..select MBC_MEMN_GLBAC0 access control policy for block B
 *  0b001..select MBC_MEMN_GLBAC1 access control policy for block B
 *  0b010..select MBC_MEMN_GLBAC2 access control policy for block B
 *  0b011..select MBC_MEMN_GLBAC3 access control policy for block B
 *  0b100..select MBC_MEMN_GLBAC4 access control policy for block B
 *  0b101..select MBC_MEMN_GLBAC5 access control policy for block B
 *  0b110..select MBC_MEMN_GLBAC6 access control policy for block B
 *  0b111..select MBC_MEMN_GLBAC7 access control policy for block B
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_MBACSEL7(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_MBACSEL7_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_MBACSEL7_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_NSE7_MASK (0x80000000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_NSE7_SHIFT (31U)
/*! NSE7 - NonSecure Enable for block B
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in this register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in this register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_NSE7(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_NSE7_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_NSE7_MASK)
/*! @} */

/* The count of TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_COUNT (1U)

/* The count of TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_CFG_W_MBC_DOM0_MEM2_BLK_CFG_W_COUNT2 (1U)

/*! @name MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W - MBC Memory Block NonSecure Enable Word */
/*! @{ */

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT0_MASK (0x1U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT0_SHIFT (0U)
/*! BIT0 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT0(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT0_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT0_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT1_MASK (0x2U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT1_SHIFT (1U)
/*! BIT1 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT1(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT1_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT1_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT2_MASK (0x4U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT2_SHIFT (2U)
/*! BIT2 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT2(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT2_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT2_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT3_MASK (0x8U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT3_SHIFT (3U)
/*! BIT3 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT3(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT3_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT3_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT4_MASK (0x10U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT4_SHIFT (4U)
/*! BIT4 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT4(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT4_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT4_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT5_MASK (0x20U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT5_SHIFT (5U)
/*! BIT5 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT5(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT5_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT5_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT6_MASK (0x40U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT6_SHIFT (6U)
/*! BIT6 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT6(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT6_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT6_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT7_MASK (0x80U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT7_SHIFT (7U)
/*! BIT7 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT7(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT7_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT7_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT8_MASK (0x100U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT8_SHIFT (8U)
/*! BIT8 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT8(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT8_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT8_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT9_MASK (0x200U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT9_SHIFT (9U)
/*! BIT9 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT9(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT9_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT9_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT10_MASK (0x400U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT10_SHIFT (10U)
/*! BIT10 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT10(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT10_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT10_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT11_MASK (0x800U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT11_SHIFT (11U)
/*! BIT11 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT11(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT11_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT11_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT12_MASK (0x1000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT12_SHIFT (12U)
/*! BIT12 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT12(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT12_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT12_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT13_MASK (0x2000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT13_SHIFT (13U)
/*! BIT13 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT13(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT13_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT13_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT14_MASK (0x4000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT14_SHIFT (14U)
/*! BIT14 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT14(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT14_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT14_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT15_MASK (0x8000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT15_SHIFT (15U)
/*! BIT15 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT15(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT15_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT15_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT16_MASK (0x10000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT16_SHIFT (16U)
/*! BIT16 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT16(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT16_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT16_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT17_MASK (0x20000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT17_SHIFT (17U)
/*! BIT17 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT17(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT17_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT17_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT18_MASK (0x40000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT18_SHIFT (18U)
/*! BIT18 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT18(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT18_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT18_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT19_MASK (0x80000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT19_SHIFT (19U)
/*! BIT19 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT19(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT19_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT19_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT20_MASK (0x100000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT20_SHIFT (20U)
/*! BIT20 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT20(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT20_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT20_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT21_MASK (0x200000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT21_SHIFT (21U)
/*! BIT21 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT21(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT21_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT21_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT22_MASK (0x400000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT22_SHIFT (22U)
/*! BIT22 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT22(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT22_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT22_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT23_MASK (0x800000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT23_SHIFT (23U)
/*! BIT23 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT23(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT23_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT23_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT24_MASK (0x1000000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT24_SHIFT (24U)
/*! BIT24 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT24(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT24_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT24_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT25_MASK (0x2000000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT25_SHIFT (25U)
/*! BIT25 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT25(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT25_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT25_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT26_MASK (0x4000000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT26_SHIFT (26U)
/*! BIT26 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT26(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT26_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT26_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT27_MASK (0x8000000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT27_SHIFT (27U)
/*! BIT27 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT27(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT27_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT27_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT28_MASK (0x10000000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT28_SHIFT (28U)
/*! BIT28 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT28(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT28_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT28_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT29_MASK (0x20000000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT29_SHIFT (29U)
/*! BIT29 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT29(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT29_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT29_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT30_MASK (0x40000000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT30_SHIFT (30U)
/*! BIT30 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT30(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT30_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT30_MASK)

#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT31_MASK (0x80000000U)
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT31_SHIFT (31U)
/*! BIT31 - Bit b NonSecure Enable [b = 0 - 31]
 *  0b0..Secure accesses to block B are based on corresponding MBACSEL field in register
 *       (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]), nonsecure accesses to block B are not allowed.
 *  0b1..Secure accesses to block B are not allowed, nonsecure accesses to block B are based on corresponding
 *       MBACSEL field in register (MBCm_DOMd_MEMs_BLK_CFG_Ww[MBACSEL]).
 */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT31(x) (((uint32_t)(((uint32_t)(x)) << TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT31_SHIFT)) & TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_BIT31_MASK)
/*! @} */

/* The count of TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_COUNT (1U)

/* The count of TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W */
#define TRDC_MBC_INDEX_MBC_INDEX_DOM0_MEM2_BLK_NSE_W_MBC_DOM0_MEM2_BLK_NSE_W_COUNT2 (1U)


/*!
 * @}
 */ /* end of group TRDC_Register_Masks */


/*!
 * @}
 */ /* end of group TRDC_Peripheral_Access_Layer */


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


#endif  /* PERI_TRDC_H_ */

