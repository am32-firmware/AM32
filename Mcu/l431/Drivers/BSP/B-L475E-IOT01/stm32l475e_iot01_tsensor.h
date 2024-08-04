/**
  ******************************************************************************
  * @file    stm32l475e_iot01_tsensor.h
  * @author  MCD Application Team
  * @brief   This file provides a set of functions needed to manage the temperature sensor
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
#ifndef __STM32L475E_IOT01_TSENSOR_H
#define __STM32L475E_IOT01_TSENSOR_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l475e_iot01.h"
#ifdef USE_LPS22HB_TEMP
#include "../Components/lps22hb/lps22hb.h"
#else /* USE_HTS221_TEMP */
#include "../Components/hts221/hts221.h"
#endif

/** @addtogroup BSP
  * @{
  */ 

/** @addtogroup STM32L475E_IOT01
  * @{
  */

/** @addtogroup STM32L475E_IOT01_TEMPERATURE 
  * @{
  */
   
/** @defgroup STM32L475E_IOT01_TEMPERATURE_Exported_Types TEMPERATURE Exported Types
  * @{
  */
   
/** 
  * @brief  TSENSOR Status  
  */ 
typedef enum
{
  TSENSOR_OK = 0,
  TSENSOR_ERROR
}TSENSOR_Status_TypDef;

/**
  * @}
  */


/** @defgroup STM32L475E_IOT01_TEMPERATURE_Exported_Functions TEMPERATURE Exported Constants
  * @{
  */
/* Exported macros -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/* Sensor Configuration Functions */
uint32_t BSP_TSENSOR_Init(void);
float BSP_TSENSOR_ReadTemp(void);
/**
  * @}
  */ 

#ifdef __cplusplus
}
#endif

/**
  * @}
  */ 

/**
  * @}
  */ 

/**
  * @}
  */

#endif /* __STM32L475E_IOT01_TSENSOR_H */


