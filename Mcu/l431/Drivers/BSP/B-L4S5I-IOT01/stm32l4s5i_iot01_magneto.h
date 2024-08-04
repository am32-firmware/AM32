/**
  ******************************************************************************
  * @file    stm32l4s5i_iot01_magneto.h
  * @author  MCD Application Team
  * @brief   This file provides a set of functions needed to manage the magnetometer sensor
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32L4S5I_IOT01_MAGNETO_H
#define __STM32L4S5I_IOT01_MAGNETO_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4s5i_iot01.h"   
/* Include Magnetometer component driver */
#include "../Components/lis3mdl/lis3mdl.h"  
   
/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L4S5I_IOT01
  * @{
  */
      
/** @addtogroup STM32L4S5I_IOT01_MAGNETO
  * @{
  */ 

/** @defgroup STM32L4S5_IOT01_MAGNETO_Exported_Types MAGNETO Exported Types
  * @{
  */   
/* Exported types ------------------------------------------------------------*/
typedef enum 
{
  MAGNETO_OK = 0,
  MAGNETO_ERROR = 1,
  MAGNETO_TIMEOUT = 2
} 
MAGNETO_StatusTypeDef;
/**
  * @}
  */

/** @defgroup STM32L4S5I_IOT01_MAGNETO_Exported_Functions MAGNETO Exported Functions
  * @{
  */ 
MAGNETO_StatusTypeDef BSP_MAGNETO_Init(void);
void BSP_MAGNETO_DeInit(void);
void BSP_MAGNETO_LowPower(uint16_t status); /* 0 Means Disable Low Power Mode, otherwise Low Power Mode is enabled */
void BSP_MAGNETO_GetXYZ(int16_t *pDataXYZ);
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __STM32L4S5I_IOT01_MAGNETO_H */


