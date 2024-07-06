/**
  ******************************************************************************
  * @file    magneto.h
  * @author  MCD Application Team
  * @brief   This header file contains the functions prototypes for the MAGNETO driver.
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
#ifndef __MAGNETO_H
#define __MAGNETO_H

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
    
/** @addtogroup MAGNETO
  * @{
  */

/** @defgroup MAGNETO_Exported_Types
  * @{
  */ 

/** @defgroup MAGNETO_Config_structure  Magnetometer Configuration structure
  * @{
  */
typedef struct
{
  uint8_t Register1;
  uint8_t Register2;
  uint8_t Register3;
  uint8_t Register4;
  uint8_t Register5;
}MAGNETO_InitTypeDef;
/**
  * @}
  */

/** @defgroup MAGNETO_Driver_structure  Magnetometer Driver structure
  * @{
  */
typedef struct
{  
  void      (*Init)(MAGNETO_InitTypeDef);
  void      (*DeInit)(void); 
  uint8_t   (*ReadID)(void);
  void      (*Reset)(void);
  void      (*LowPower)(uint16_t);
  void      (*ConfigIT)(void);
  void      (*EnableIT)(uint8_t);
  void      (*DisableIT)(uint8_t);
  uint8_t   (*ITStatus)(uint16_t);
  void      (*ClearIT)(void);
  void      (*FilterConfig)(uint8_t);
  void      (*FilterCmd)(uint8_t);
  void      (*GetXYZ)(int16_t *);
}MAGNETO_DrvTypeDef;
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

#endif /* __MAGNETO_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
