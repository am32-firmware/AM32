/**
  ******************************************************************************
  * @file    camera.h
  * @author  MCD Application Team
  * @brief   This header file contains the common defines and functions prototypes
  *          for the camera driver.   
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
#ifndef __CAMERA_H
#define __CAMERA_H

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
    
/** @addtogroup CAMERA
  * @{
  */


/** @defgroup CAMERA_Exported_Types
  * @{
  */ 

/** @defgroup CAMERA_Driver_structure  Camera Driver structure
  * @{
  */
typedef struct
{
  void     (*Init)(uint16_t, uint32_t);
  uint16_t (*ReadID)(uint16_t);  
  void     (*Config)(uint16_t, uint32_t, uint32_t, uint32_t);
}CAMERA_DrvTypeDef;
/**
  * @}
  */

/**
  * @}
  */

/** @defgroup CAMERA_Exported_Constants
  * @{
  */
#define CAMERA_R160x120                 0x00   /* QQVGA Resolution                     */
#define CAMERA_R320x240                 0x01   /* QVGA Resolution                      */
#define CAMERA_R480x272                 0x02   /* 480x272 Resolution                   */
#define CAMERA_R640x480                 0x03   /* VGA Resolution                       */  

#define CAMERA_CONTRAST_BRIGHTNESS      0x00   /* Camera contrast brightness features  */
#define CAMERA_BLACK_WHITE              0x01   /* Camera black white feature           */
#define CAMERA_COLOR_EFFECT             0x03   /* Camera color effect feature          */

#define CAMERA_BRIGHTNESS_LEVEL0        0x00   /* Brightness level -2         */
#define CAMERA_BRIGHTNESS_LEVEL1        0x01   /* Brightness level -1         */
#define CAMERA_BRIGHTNESS_LEVEL2        0x02   /* Brightness level 0          */
#define CAMERA_BRIGHTNESS_LEVEL3        0x03   /* Brightness level +1         */
#define CAMERA_BRIGHTNESS_LEVEL4        0x04   /* Brightness level +2         */

#define CAMERA_CONTRAST_LEVEL0          0x05   /* Contrast level -2           */
#define CAMERA_CONTRAST_LEVEL1          0x06   /* Contrast level -1           */
#define CAMERA_CONTRAST_LEVEL2          0x07   /* Contrast level  0           */
#define CAMERA_CONTRAST_LEVEL3          0x08   /* Contrast level +1           */
#define CAMERA_CONTRAST_LEVEL4          0x09   /* Contrast level +2           */    
    
#define CAMERA_BLACK_WHITE_BW           0x00   /* Black and white effect      */
#define CAMERA_BLACK_WHITE_NEGATIVE     0x01   /* Negative effect             */
#define CAMERA_BLACK_WHITE_BW_NEGATIVE  0x02   /* BW and Negative effect      */
#define CAMERA_BLACK_WHITE_NORMAL       0x03   /* Normal effect               */
                                        
#define CAMERA_COLOR_EFFECT_NONE        0x00   /* No effects                  */
#define CAMERA_COLOR_EFFECT_BLUE        0x01   /* Blue effect                 */
#define CAMERA_COLOR_EFFECT_GREEN       0x02   /* Green effect                */
#define CAMERA_COLOR_EFFECT_RED         0x03   /* Red effect                  */
#define CAMERA_COLOR_EFFECT_ANTIQUE     0x04   /* Antique effect              */

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

#endif /* __CAMERA_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
