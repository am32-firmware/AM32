/**
  ******************************************************************************
  * @file    stm32l476g_discovery_compass.h
  * @author  MCD Application Team
  * @brief   This file contains definitions for stm32l476g_discovery_compass.c
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
#ifndef __STM32L476G_DISCOVERY_COMPASS_H
#define __STM32L476G_DISCOVERY_COMPASS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l476g_discovery.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L476G_DISCOVERY
  * @{
  */

/** @addtogroup STM32L476G_DISCOVERY_COMPASS
  * @{
  */

/** @defgroup STM32L476G_DISCOVERY_COMPASS_Exported_Types Exported Types
  * @{
  */

/**
  * @}
  */

/** @defgroup STM32L476G_DISCOVERY_COMPASS_Exported_Constants Exported Constants
  * @{
  */
typedef enum
{
  COMPASS_OK = 0,
  COMPASS_ERROR = 1,
  COMPASS_TIMEOUT = 2
}
COMPASS_StatusTypeDef;

/**
  * @}
  */

/** @defgroup STM32L476G_DISCOVERY_COMPASS_Exported_Macros Exported Macros
  * @{
  */

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup STM32L476G_DISCOVERY_COMPASS_Exported_Functions Exported Functions
  * @{
  */
/* COMPASS functions */
COMPASS_StatusTypeDef   BSP_COMPASS_Init(void);
void                    BSP_COMPASS_DeInit(void);
void                    BSP_COMPASS_LowPower(void);
void                    BSP_COMPASS_MagGetXYZ(int16_t *pDataXYZ);
void                    BSP_COMPASS_AccGetXYZ(int16_t *pDataXYZ);

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

#endif /* __STM32L476G_DISCOVERY_COMPASS_H */


