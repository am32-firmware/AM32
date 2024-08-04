/**
  ******************************************************************************
  * @file    stm32l476g_discovery_gyroscope.h
  * @author  MCD Application Team
  * @brief   This file contains definitions for stm32l476g_discovery_gyroscope.c
  *          firmware driver.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32L476G_DISCOVERY_GYROSCOPE_H
#define __STM32L476G_DISCOVERY_GYROSCOPE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l476g_discovery.h"
/* Include Gyroscope component driver */
#include "../Components/l3gd20/l3gd20.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L476G_DISCOVERY
  * @{
  */

/** @addtogroup STM32L476G_DISCOVERY_GYROSCOPE
  * @{
  */

/** @defgroup STM32L476G_DISCOVERY_GYROSCOPE_Exported_Types Exported Types
  * @{
  */
typedef enum
{
  GYRO_OK = 0,
  GYRO_ERROR = 1,
  GYRO_TIMEOUT = 2
}
GYRO_StatusTypeDef;

/**
  * @}
  */

/** @defgroup STM32L476G_DISCOVERY_GYROSCOPE_Exported_Constants Exported Constants
  * @{
  */

/**
  * @}
  */

/** @defgroup STM32L476G_DISCOVERY_GYROSCOPE_Exported_Macros Exported Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup STM32L476G_DISCOVERY_GYROSCOPE_Exported_Functions  Exported Functions
  * @{
  */
/* Sensor Configuration Functions */
uint8_t BSP_GYRO_Init(void);
void    BSP_GYRO_DeInit(void);
void    BSP_GYRO_LowPower(void);
void    BSP_GYRO_Reset(void);
uint8_t BSP_GYRO_ReadID(void);
void    BSP_GYRO_ITConfig(GYRO_InterruptConfigTypeDef *pIntConfigStruct);
void    BSP_GYRO_EnableIT(uint8_t IntPin);
void    BSP_GYRO_DisableIT(uint8_t IntPin);
void    BSP_GYRO_GetXYZ(float *pfData);

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

#endif /* __STM32L476G_DISCOVERY_GYROSCOPE_H */


