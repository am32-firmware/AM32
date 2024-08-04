/**
  ******************************************************************************
  * @file    stm32l4r9i_eval_sd.h
  * @author  MCD Application Team
  * @brief   This file contains the common defines and functions prototypes for
  *          the stm32l4r9i_eval_sd.c driver.
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
#ifndef __STM32L4R9I_EVAL_SD_H
#define __STM32L4R9I_EVAL_SD_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L4R9I_EVAL
  * @{
  */

/** @addtogroup STM32L4R9I_EVAL_SD
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/** @defgroup STM32L4R9I_EVAL_SD_Exported_Types Exported Types
  * @{
  */

/**
  * @brief SD Card information structure
  */
#define BSP_SD_CardInfo HAL_SD_CardInfoTypeDef
/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/

/** @defgroup STM32L4R9I_EVAL_SD_Exported_Constants  Exported Constants
  * @{
  */
/**
  * @brief  SD status structure definition
  */
#define MSD_OK                   ((uint8_t)0x00)
#define MSD_ERROR                ((uint8_t)0x01)
#define MSD_ERROR_SD_NOT_PRESENT ((uint8_t)0x02)

/**
  * @brief  SD transfer state definition
  */
#define SD_TRANSFER_OK           ((uint8_t)0x00)
#define SD_TRANSFER_BUSY         ((uint8_t)0x01)
#define SD_TRANSFER_ERROR        ((uint8_t)0x02)

#define SD_DATATIMEOUT           ((uint32_t)100000000)

#define SD_PRESENT               ((uint8_t)0x01)
#define SD_NOT_PRESENT           ((uint8_t)0x00)

/* SD IRQ handler */
#define SDMMCx_IRQHandler        SDMMC1_IRQHandler
#define SDMMCx_IRQn              SDMMC1_IRQn

/**
  * @}
  */

/* Exported variables --------------------------------------------------------*/

/** @defgroup STM32L4R9I_EVAL_SD_Exported_Variables Exported Variables
  * @{
  */
extern SD_HandleTypeDef           hsd_eval;

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/

/** @defgroup STM32L4R9I_EVAL_SD_Exported_Functions Exported Functions
  * @{
  */
uint8_t BSP_SD_Init(void);
uint8_t BSP_SD_DeInit(void);
uint8_t BSP_SD_ITConfig(void);
uint8_t BSP_SD_ReadBlocks(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks, uint32_t Timeout);
uint8_t BSP_SD_WriteBlocks(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks, uint32_t Timeout);
uint8_t BSP_SD_ReadBlocks_DMA(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks);
uint8_t BSP_SD_WriteBlocks_DMA(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks);
uint8_t BSP_SD_Erase(uint32_t StartAddr, uint32_t EndAddr);
uint8_t BSP_SD_GetCardState(void);
void    BSP_SD_GetCardInfo(BSP_SD_CardInfo *CardInfo);
uint8_t BSP_SD_IsDetected(void);

/* These __weak functions can be surcharged by application code in case the current settings
   (eg. interrupt priority, callbacks implementation) need to be changed for specific application needs */
void    BSP_SD_MspInit(SD_HandleTypeDef *hsd, void *Params);
void    BSP_SD_MspDeInit(SD_HandleTypeDef *hsd, void *Params);
void    BSP_SD_AbortCallback(void);
void    BSP_SD_WriteCpltCallback(void);
void    BSP_SD_ReadCpltCallback(void);

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

#endif /* __STM32L4R9I_EVAL_SD_H */


