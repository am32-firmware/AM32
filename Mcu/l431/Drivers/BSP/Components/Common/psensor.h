/**
  ******************************************************************************
  * @file    psensor.h
  * @author  MCD Application Team
  * @brief   This header file contains the functions prototypes for the
  *          Pressure Sensor driver.
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
#ifndef __PSENSOR_H
#define __PSENSOR_H

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
    
/** @addtogroup PSENSOR
  * @{
  */

/** @defgroup PSENSOR_Exported_Types
  * @{
  */ 

/** @defgroup PSENSOR_Driver_structure  Pressure Sensor Driver structure
  * @{
  */
typedef struct
{  
  void       (*Init)(uint16_t);
  uint8_t    (*ReadID)(uint16_t);
  float      (*ReadPressure)(uint16_t);
}PSENSOR_DrvTypeDef;
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

#endif /* __PSENSOR_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
