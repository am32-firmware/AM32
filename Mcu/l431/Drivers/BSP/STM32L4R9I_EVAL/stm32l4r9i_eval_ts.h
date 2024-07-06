/**
  ******************************************************************************
  * @file    stm32l4r9i_eval_ts.h
  * @author  MCD Application Team
  * @brief   This file contains the common defines and functions prototypes for
  *          the stm32l4r9i_eval_ts.c driver.
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
#ifndef __STM32L4R9I_EVAL_TS_H
#define __STM32L4R9I_EVAL_TS_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4r9i_eval.h"
#include "stm32l4r9i_eval_lcd.h"
#include "stm32l4r9i_eval_io.h"

/* Include TouchScreen component driver */
#if defined(USE_GVO_390x390)
/* Include touch screen FT3X67 component Driver */
#include "../Components/ft3x67/ft3x67.h"
#endif

#if defined(USE_ROCKTECH_480x272)
/* Include touch screen FT5336 component Driver */
#include "../Components/ft5336/ft5336.h"
#endif

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L4R9I_EVAL
  * @{
  */

/** @defgroup STM32L4R9I_EVAL_TS STM32L4R9I_EVAL TS
  * @{
  */

 /** @defgroup STM32L4R9I_EVAL_TS_Exported_Constants Exported Constants
   * @{
   */
#if defined(USE_GVO_390x390)
/** @brief With FT3X67 : maximum 2 touches detected simultaneously
  */
#define TS_MAX_NB_TOUCH                 ((uint32_t) FT3X67_MAX_DETECTABLE_TOUCH)
#endif

#if defined(USE_ROCKTECH_480x272)
/** @brief With FT5336 : maximum 5 touches detected simultaneously
  */
#define TS_MAX_NB_TOUCH                 ((uint32_t) FT5336_MAX_DETECTABLE_TOUCH)
#endif

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

/** @defgroup STM32L4R9I_EVAL_TS_Exported_Types  Exported Types
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

/** @defgroup STM32L4R9I_EVAL_TS_Exported_Functions Exported Functions
  * @{
  */

#if defined(USE_GVO_390x390)

#include "stm32l4r9i_eval_dsi_ts.h"

#define BSP_TS_Init                BSP_DSI_TS_Init
#define BSP_TS_DeInit              BSP_DSI_TS_DeInit

#define BSP_TS_GetState            BSP_DSI_TS_GetState
#define BSP_TS_GestureConfig       BSP_DSI_TS_GestureConfig
#define BSP_TS_Get_GestureId       BSP_DSI_TS_Get_GestureId
#define BSP_TS_ResetTouchData      BSP_DSI_TS_ResetTouchData

#define BSP_TS_ITConfig            BSP_DSI_TS_ITConfig
#define BSP_TS_ITGetStatus         BSP_DSI_TS_ITGetStatus
#define BSP_TS_ITClear             BSP_DSI_TS_ITClear

#endif /* USE_GVO_390x390 */


#if defined(USE_ROCKTECH_480x272)

#include "stm32l4r9i_eval_rgb_ts.h"

#define BSP_TS_Init                BSP_RGB_TS_Init
#define BSP_TS_DeInit              BSP_RGB_TS_DeInit

#define BSP_TS_GetState            BSP_RGB_TS_GetState
#define BSP_TS_Get_GestureId       BSP_RGB_TS_Get_GestureId
#define BSP_TS_ResetTouchData      BSP_RGB_TS_ResetTouchData

#define BSP_TS_ITConfig            BSP_RGB_TS_ITConfig
#define BSP_TS_ITGetStatus         BSP_RGB_TS_ITGetStatus
#define BSP_TS_ITClear             BSP_RGB_TS_ITClear

#endif /* USE_ROCKTECH_480x272 */

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

#endif /* __STM32L4R9I_EVAL_TS_H */


