/**
  ******************************************************************************
  * @file    iss66wvh8m8.h
  * @author  MCD Application Team
  * @brief   This file contains all the description of the ISS66WVH8M8 Octal memory.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ISS66WVH8M8_H
#define __ISS66WVH8M8_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/

/** @addtogroup BSP
  * @{
  */ 

/** @addtogroup Components
  * @{
  */ 
  
/** @addtogroup ISS66WVH8M8
  * @{
  */

/** @defgroup ISS66WVH8M8_Exported_Types
  * @{
  */
   
/**
  * @}
  */ 

/** @defgroup ISS66WVH8M8_Exported_Constants
  * @{
  */
   
/** 
  * @brief  ISS66WVH8M8 Configuration  
  */  
#define ISS66WVH8M8_RAM_SIZE          0x800000               /* 64 MBits => 8 MBytes */

#define ISS66WVH8M8_LATENCY_83M       3
#define ISS66WVH8M8_LATENCY_100M      4
#define ISS66WVH8M8_LATENCY_133M      5
#define ISS66WVH8M8_LATENCY_166M      6

#define ISS66WVH8M8_CR0_LATENCY_83M   ISS66WVH8M8_CR0_IL_3_CLOCK
#define ISS66WVH8M8_CR0_LATENCY_100M  ISS66WVH8M8_CR0_IL_4_CLOCK
#define ISS66WVH8M8_CR0_LATENCY_133M  ISS66WVH8M8_CR0_IL_5_CLOCK
#define ISS66WVH8M8_CR0_LATENCY_166M  ISS66WVH8M8_CR0_IL_6_CLOCK

/** 
  * @brief  ISS66WVH8M8 Registers  
  */ 
/* Device Identification Register 0 */
#define ISS66WVH8M8_DIR0_ADDRESS      ((uint32_t)0x00000000)

#define ISS66WVH8M8_DIR0_MANUFACTURER ((uint16_t)0x000F)     /*!< Manufacturer */
#define ISS66WVH8M8_DIR0_CADC         ((uint16_t)0x00F0)     /*!< Column Address Bit Count */
#define ISS66WVH8M8_DIR0_RABC         ((uint16_t)0x1F00)     /*!< Row Address Bit Count */
#define ISS66WVH8M8_DIR0_DADDR        ((uint16_t)0xC000)     /*!< Die Address */

/* Device Identification Register 1 */
#define ISS66WVH8M8_DIR1_ADDRESS      ((uint32_t)0x00000001)

#define ISS66WVH8M8_DIR1_DTYPE        ((uint16_t)0x000F)     /*!< Device Type */

/* Configuration Register 0 */
#define ISS66WVH8M8_CR0_ADDRESS       ((uint32_t)0x00000800)

#define ISS66WVH8M8_CR0_BLENGTH       ((uint16_t)0x0003)     /*!< Burst length */
#define ISS66WVH8M8_CR0_HBE           ((uint16_t)0x0004)     /*!< Hybrid burst enable */
#define ISS66WVH8M8_CR0_FLE           ((uint16_t)0x0008)     /*!< Fixed latency enable */
#define ISS66WVH8M8_CR0_IL            ((uint16_t)0x00F0)     /*!< Initial latency */
#define ISS66WVH8M8_CR0_IL_5_CLOCK    ((uint16_t)0x0000)     /*!< 5 clock latency */
#define ISS66WVH8M8_CR0_IL_6_CLOCK    ((uint16_t)0x0010)     /*!< 6 clock latency */
#define ISS66WVH8M8_CR0_IL_3_CLOCK    ((uint16_t)0x00E0)     /*!< 3 clock latency */
#define ISS66WVH8M8_CR0_IL_4_CLOCK    ((uint16_t)0x00F0)     /*!< 4 clock latency */
#define ISS66WVH8M8_CR0_DS            ((uint16_t)0x7000)     /*!< Drive strength */
#define ISS66WVH8M8_CR0_DPDE          ((uint16_t)0x8000)     /*!< Deep Power down enable */

/* Configuration Register 1 */
#define ISS66WVH8M8_CR1_ADDRESS       ((uint32_t)0x00000801)

#define ISS66WVH8M8_CR1_DRI           ((uint16_t)0x0003)     /*!< Distributed refresh interval */

/**
  * @}
  */
  
/** @defgroup ISS66WVH8M8_Exported_Functions
  * @{
  */ 
/**
  * @}
  */ 
      
#ifdef __cplusplus
}
#endif

#endif /* __ISS66WVH8M8_H */

/**
  * @}
  */ 

/**
  * @}
  */ 

/**
  * @}
  */
  
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
