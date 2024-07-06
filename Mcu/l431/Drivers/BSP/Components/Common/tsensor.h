/**
  ******************************************************************************
  * @file    tsensor.h
  * @author  MCD Application Team
  * @brief   This header file contains the functions prototypes for the
  *          Temperature Sensor driver. 
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
#ifndef __TSENSOR_H
#define __TSENSOR_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/** @addtogroup BSP
  * @{
  */

/** @addtogroup Components
  * @{
  */
    
/** @addtogroup TSENSOR
  * @{
  */

/** @defgroup TSENSOR_Exported_Types
  * @{
  */ 

/** @defgroup TSENSOR_Config_structure  Temperature Sensor Configuration structure
  * @{
  */
typedef struct
{
  uint8_t AlertMode;            /* Alert Mode Temperature out of range*/
  uint8_t ConversionMode;       /* Continuous/One Shot Mode */
  uint8_t ConversionResolution; /* Temperature Resolution */
  uint8_t ConversionRate;       /* Number of measure per second */
  int8_t TemperatureLimitHigh; /* High Temperature Limit Range */
  int8_t TemperatureLimitLow;  /* Low Temperature Limit Range */
}TSENSOR_InitTypeDef;
/**
  * @}
  */

/** @defgroup TSENSOR_Driver_structure  Temperature Sensor Driver structure
  * @{
  */
typedef struct
{  
  void       (*Init)(uint16_t, TSENSOR_InitTypeDef *);
  uint8_t    (*IsReady)(uint16_t, uint32_t);
  uint8_t    (*ReadStatus)(uint16_t);
  float      (*ReadTemp)(uint16_t); 
}TSENSOR_DrvTypeDef;
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

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __TSENSOR_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
