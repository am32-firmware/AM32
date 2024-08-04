/**
  ******************************************************************************
  * @file    stm32l476g_eval_ts.h
  * @author  MCD Application Team
  * @brief   This file contains the common defines and functions prototypes for
  *          the stm32l476g_eval_ts.c driver.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32L476G_EVAL_TS_H
#define __STM32L476G_EVAL_TS_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l476g_eval.h"  
/* Include IOExpander(STMPE811) component Driver */ 
#include "../Components/stmpe811/stmpe811.h" 

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L476G_EVAL
  * @{
  */

/** @addtogroup STM32L476G_EVAL_TS
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/** @defgroup STM32L476G_EVAL_TS_Exported_Types Exported Types
  * @{
  */
typedef struct
{
  uint16_t TouchDetected;
  uint16_t x;
  uint16_t y;
  uint16_t z;
}TS_StateTypeDef; 
/**
  * @}
  */

/** @defgroup STM32L476G_EVAL_TS_Exported_Constants Exported Constants
  * @{
  */
#define TS_I2C_ADDRESS                  0x82

#define TS_SWAP_NONE                    0x00
#define TS_SWAP_X                       0x01
#define TS_SWAP_Y                       0x02
#define TS_SWAP_XY                      0x04

typedef enum 
{
  TS_OK       = 0x00,
  TS_ERROR    = 0x01,
  TS_TIMEOUT  = 0x02
}TS_StatusTypeDef;
/**
  * @}
  */

/** @defgroup STM32L476G_EVAL_TS_Exported_Macros Exported Macros
  * @{
  */
/**
  * @}
  */ 

/* Exported functions --------------------------------------------------------*/

/** @defgroup STM32L476G_EVAL_TS_Exported_Functions Exported Functions
  * @{
  */
uint8_t BSP_TS_Init(uint16_t xSize, uint16_t ySize);
uint8_t BSP_TS_GetState(TS_StateTypeDef *TS_State);
uint8_t BSP_TS_ITConfig(void);
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

#endif /* __STM32L476G_EVAL_TS_H */


