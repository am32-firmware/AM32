/**
  ******************************************************************************
  * @file    stm32l4r9i_eval_dsi_ts.h
  * @author  MCD Application Team
  * @brief   This file contains the common defines and functions prototypes for
  *          the stm32l4r9i_eval_dsi_ts.c driver.
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
#ifndef __STM32L4R9I_EVAL_DSI_TS_H
#define __STM32L4R9I_EVAL_DSI_TS_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4r9i_eval.h"

/* Include TouchScreen component driver */
#include "../Components/ft3x67/ft3x67.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L4R9I_EVAL
  * @{
  */

/** @defgroup STM32L4R9I_EVAL_TS STM32L4R9I_EVAL TS
  * @{
  */

/** @defgroup STM32L4R9I-EVAL_DSI_TS STM32L4R9I_EVAL DSI TS
  * @{
  */

/** @defgroup STM32L4R9I-EVAL_DSI_TS_Exported_Functions Exported Functions
  * @{
  */

uint8_t BSP_DSI_TS_Init(uint16_t ts_SizeX, uint16_t ts_SizeY);
uint8_t BSP_DSI_TS_DeInit(void);

uint8_t BSP_DSI_TS_GetState(TS_StateTypeDef *TS_State);
uint8_t BSP_DSI_TS_GestureConfig(FunctionalState State);
uint8_t BSP_DSI_TS_Get_GestureId(TS_StateTypeDef *TS_State);
uint8_t BSP_DSI_TS_ResetTouchData(TS_StateTypeDef *TS_State);

uint8_t BSP_DSI_TS_ITConfig(void);
uint8_t BSP_DSI_TS_ITGetStatus(void);
void    BSP_DSI_TS_ITClear(void);

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

#endif /* __STM32L4R9I_EVAL_DSI_TS_H */


