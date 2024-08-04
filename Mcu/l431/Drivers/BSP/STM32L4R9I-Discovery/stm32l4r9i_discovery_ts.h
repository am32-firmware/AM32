/**
  ******************************************************************************
  * @file    stm32l4r9i_discovery_ts.h
  * @author  MCD Application Team
  * @brief   This file contains the common defines and functions prototypes for
  *          the stm32l4r9i_discovery_ts.c driver.
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
#ifndef __STM32L4R9I_DISCOVERY_TS_H
#define __STM32L4R9I_DISCOVERY_TS_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4r9i_discovery.h"
#include "stm32l4r9i_discovery_lcd.h"
#include "stm32l4r9i_discovery_io.h"

/* Include TouchScreen component driver */
#include "../Components/ft3x67/ft3x67.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L4R9I_DISCOVERY
  * @{
  */

/** @defgroup STM32L4R9I_DISCOVERY_TS STM32L4R9I_DISCOVERY TS
  * @{
  */

 /** @defgroup STM32L4R9I_DISCOVERY_TS_Exported_Constants Exported Constants
   * @{
   */
/** @brief With FT3X67 : maximum 2 touches detected simultaneously
  */
#define TS_MAX_NB_TOUCH                 ((uint32_t) FT3X67_MAX_DETECTABLE_TOUCH)

#define TS_NO_IRQ_PENDING               ((uint8_t) 0)
#define TS_IRQ_PENDING                  ((uint8_t) 1)

#define TS_SWAP_NONE                    ((uint8_t) 0x01)
#define TS_SWAP_X                       ((uint8_t) 0x02)
#define TS_SWAP_Y                       ((uint8_t) 0x04)
#define TS_SWAP_XY                      ((uint8_t) 0x08)

#define TS_ORIENTATION_PORTRAIT         ((uint8_t) 0)
#define TS_ORIENTATION_LANDSCAPE        ((uint8_t) 1)

/**
  * @}
  */

/** @defgroup STM32L4R9I_DISCOVERY_TS_Exported_Types  Exported Types
  * @{
  */
/**
*  @brief TS_StateTypeDef
*  Define TS State structure
*/
typedef struct
{
  uint8_t  touchDetected;                 /*!< Total number of active touches detected at last scan */
  uint16_t touchX[TS_MAX_NB_TOUCH];       /*!< Touch X[0], X[1] coordinates on 12 bits */
  uint16_t touchY[TS_MAX_NB_TOUCH];       /*!< Touch Y[0], Y[1] coordinates on 12 bits */
  uint8_t  touchWeight[TS_MAX_NB_TOUCH];  /*!< Touch_Weight[0], Touch_Weight[1] : weight property of touches */
  uint8_t  touchEventId[TS_MAX_NB_TOUCH]; /*!< Touch_EventId[0], Touch_EventId[1] : take value of type @ref TS_TouchEventTypeDef */
  uint8_t  touchArea[TS_MAX_NB_TOUCH];    /*!< Touch_Area[0], Touch_Area[1] : touch area of each touch */
  uint32_t gestureId;                     /*!< type of gesture detected : take value of type @ref TS_GestureIdTypeDef */
} TS_StateTypeDef;

/**
 *  @brief TS_StatusTypeDef
 *  Define BSP_TS_xxx() functions possible return value,
 *  when status is returned by those functions.
 */
typedef enum
{
  TS_OK                = 0x00, /*!< Touch Ok */
  TS_ERROR             = 0x01, /*!< Touch Error */
  TS_TIMEOUT           = 0x02, /*!< Touch Timeout */
  TS_DEVICE_NOT_FOUND  = 0x03  /*!< Touchscreen device not found */
} TS_StatusTypeDef;

/**
 *  @brief TS_GestureIdTypeDef
 *  Define Possible managed gesture identification values returned by touch screen
 *  driver.
 */
typedef enum
{
  GEST_ID_NO_GESTURE   = 0x00, /*!< Gesture not defined / recognized */
  GEST_ID_MOVE_UP      = 0x01, /*!< Gesture Move Up */
  GEST_ID_MOVE_RIGHT   = 0x02, /*!< Gesture Move Right */
  GEST_ID_MOVE_DOWN    = 0x03, /*!< Gesture Move Down */
  GEST_ID_MOVE_LEFT    = 0x04, /*!< Gesture Move Left */
  GEST_ID_ZOOM_IN      = 0x05, /*!< Gesture Zoom In */
  GEST_ID_ZOOM_OUT     = 0x06, /*!< Gesture Zoom Out */
  GEST_ID_SINGLE_CLICK = 0x07, /*!< Gesture Single Click */
  GEST_ID_DOUBLE_CLICK = 0x08, /*!< Gesture Double Click */
  GEST_ID_NB_MAX       = 0x09 /*!< max number of gesture id */
} TS_GestureIdTypeDef;

/**
 *  @brief TS_TouchEventTypeDef
 *  Define Possible touch events kind as returned values
 *  by touch screen IC Driver.
 */
typedef enum
{
  TOUCH_EVENT_NO_EVT        = 0x00, /*!< Touch Event : undetermined */
  TOUCH_EVENT_PRESS_DOWN    = 0x01, /*!< Touch Event Press Down */
  TOUCH_EVENT_LIFT_UP       = 0x02, /*!< Touch Event Lift Up */
  TOUCH_EVENT_CONTACT       = 0x03, /*!< Touch Event Contact */
  TOUCH_EVENT_NB_MAX        = 0x04  /*!< max number of touch events kind */
} TS_TouchEventTypeDef;

/**
  * @}
  */

/** @defgroup STM32L4R9I_DISCOVERY_TS_Exported_Functions Exported Functions
  * @{
  */

uint8_t BSP_TS_Init(uint16_t ts_SizeX, uint16_t ts_SizeY);
uint8_t BSP_TS_DeInit(void);

uint8_t BSP_TS_GetState(TS_StateTypeDef *TS_State);
uint8_t BSP_TS_GestureConfig(FunctionalState State);
uint8_t BSP_TS_Get_GestureId(TS_StateTypeDef *TS_State);
uint8_t BSP_TS_ResetTouchData(TS_StateTypeDef *TS_State);

uint8_t BSP_TS_ITConfig(void);
uint8_t BSP_TS_ITDisable(void);
uint8_t BSP_TS_ITGetStatus(void);
void    BSP_TS_ITClear(void);

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

#endif /* __STM32L4R9I_DISCOVERY_TS_H */


