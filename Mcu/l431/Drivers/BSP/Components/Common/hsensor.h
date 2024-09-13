/**
  ******************************************************************************
  * @file    hsensor.h
  * @author  MCD Application Team
  * @brief   This header file contains the functions prototypes for the
  *          Humidity Sensor driver.
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
#ifndef __HSENSOR_H
#define __HSENSOR_H

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
    
/** @addtogroup HSENSOR
  * @{
  */

/** @defgroup HSENSOR_Exported_Types
  * @{
  */ 

/** @defgroup HSENSOR_Driver_structure  Humidity Sensor Driver structure
  * @{
  */
typedef struct
{
  void       (*Init)(uint16_t); 
  uint8_t    (*ReadID)(uint16_t);
  float      (*ReadHumidity)(uint16_t); 
}HSENSOR_DrvTypeDef;
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

#endif /* __HSENSOR_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
