/**
  ******************************************************************************
  * @file    stm32l4r9i_discovery_camera.h
  * @author  MCD Application Team
  * @brief   This file contains the common defines and functions prototypes for
  *          the stm32l4r9i_discovery_camera.c driver.
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
#ifndef __STM32L4R9I_DISCOVERY_CAMERA_H
#define __STM32L4R9I_DISCOVERY_CAMERA_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Include Camera component Driver */
#include "../Components/ov9655/ov9655.h"
#include "stm32l4r9i_discovery.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L4R9I_DISCOVERY
  * @{
  */

/** @addtogroup STM32L4R9I_DISCOVERY_CAMERA
  * @{
  */

/** @defgroup STM32L4R9I_DISCOVERY_CAMERA_Exported_Types STM32L4R9I_DISCOVERY_CAMERA Exported Types
  * @{
  */

/**
  * @brief  Camera State structures definition
  */
typedef enum
{
  CAMERA_OK            = 0x00,
  CAMERA_ERROR         = 0x01,
  CAMERA_TIMEOUT       = 0x02,
  CAMERA_NOT_DETECTED  = 0x03,
  CAMERA_NOT_SUPPORTED = 0x04

} Camera_StatusTypeDef;

#define RESOLUTION_R160x120      CAMERA_R160x120      /* QQVGA Resolution     */
#define RESOLUTION_R320x240      CAMERA_R320x240      /* QVGA Resolution      */
#define RESOLUTION_R480x272      CAMERA_R480x272      /* 480x272 Resolution   */
#define RESOLUTION_R640x480      CAMERA_R640x480      /* VGA Resolution       */
/**
  * @}
  */

/** @defgroup STM32L4R9I_DISCOVERY_CAMERA_Exported_Constants STM32L4R9I_DISCOVERY_CAMERA Exported Constants
  * @{
  */
#define BSP_CAMERA_IRQHandler      DCMI_IRQHandler
#define BSP_CAMERA_DMA_IRQHandler  DMA2_Channel6_IRQHandler
#define BSP_CAMERA_DMA_IRQn        DMA2_Channel6_IRQn
#define BSP_CAMERA_DMA_INSTANCE    DMA2_Channel6

/**
  * @brief Camera special pins
  */
/* Camera power up pin */
#define CAMERA_PWR_EN_PIN                  IO_PIN_12

/**
  * @}
  */

/** @defgroup STM32L4R9I_DISCOVERY_CAMERA_Exported_Variables STM32L4R9I_DISCOVERY_CAMERA Exported Variables
  * @{
  */
extern DCMI_HandleTypeDef  hDcmiHandler;

/**
  * @}
  */

/** @addtogroup STM32L4R9I_DISCOVERY_CAMERA_Exported_Functions
  * @{
  */

uint8_t BSP_CAMERA_Init(uint32_t Resolution);
uint8_t BSP_CAMERA_DeInit(void);
void    BSP_CAMERA_ContinuousStart(uint8_t *buff);
void    BSP_CAMERA_SnapshotStart(uint8_t *buff);
void    BSP_CAMERA_Suspend(void);
void    BSP_CAMERA_Resume(void);
uint8_t BSP_CAMERA_Stop(void);
void    BSP_CAMERA_PwrUp(void);
void    BSP_CAMERA_PwrDown(void);

/* Camera features functions prototype */
void    BSP_CAMERA_ContrastBrightnessConfig(uint32_t contrast_level, uint32_t brightness_level);
void    BSP_CAMERA_BlackWhiteConfig(uint32_t Mode);
void    BSP_CAMERA_ColorEffectConfig(uint32_t Effect);

/* These __weak functions can be surcharged by application code in case the current settings
   (eg. interrupt priority, callbacks implementation) need to be changed for specific application needs */
void    BSP_CAMERA_MspInit(DCMI_HandleTypeDef *hdcmi, void *Params);
void    BSP_CAMERA_MspDeInit(DCMI_HandleTypeDef *hdcmi, void *Params);
void    BSP_CAMERA_LineEventCallback(void);
void    BSP_CAMERA_VsyncEventCallback(void);
void    BSP_CAMERA_FrameEventCallback(void);
void    BSP_CAMERA_ErrorCallback(void);

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

#endif /* __STM32L4R9I_DISCOVERY_CAMERA_H */


