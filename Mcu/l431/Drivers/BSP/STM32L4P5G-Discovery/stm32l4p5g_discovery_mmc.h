/**
  ******************************************************************************
  * @file    stm32l4p5g_discovery_mmc.h
  * @author  MCD Application Team
  * @brief   This file contains the common defines and functions prototypes for
  *          the stm32l4p5g_discovery_mmc.c driver.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef STM32L4P5G_DISCOVERY_MMC_H
#define STM32L4P5G_DISCOVERY_MMC_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "stm32l4p5g_discovery.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L4P5G_DISCOVERY
  * @{
  */

/** @addtogroup STM32L4P5G_DISCOVERY_MMC
  * @{
  */

/** @defgroup STM32L4P5G_DISCOVERY_MMC_Exported_Types Exported Types
  * @{
  */

/**
  * @brief SD Card information structure
  */
#define BSP_MMC_CardInfo HAL_MMC_CardInfoTypeDef
/**
  * @}
  */

/**
  * @brief SD Card CSD information structure
  */
#define BSP_MMC_CardCSD HAL_MMC_CardCSDTypeDef
/**
  * @}
  */

/**
  * @brief SD Card CID information structure
  */
#define BSP_MMC_CardCID HAL_MMC_CardCIDTypeDef
/**
  * @}
  */

/** @defgroup STM32L4P5G_DISCOVERY_MMC_Exported_Constants Exported Constants
  * @{
  */
/**
  * @brief  SD status structure definition
  */
#define MMC_OK                        ((uint8_t)0x00)
#define MMC_ERROR                     ((uint8_t)0x01)
#define MMC_ERROR_MMC_NOT_PRESENT     ((uint8_t)0x02)

/**
  * @brief  MMC transfer state definition
  */
#define MMC_TRANSFER_OK                ((uint8_t)0x00)
#define MMC_TRANSFER_BUSY              ((uint8_t)0x01)


#define MMC_PRESENT               ((uint8_t)0x01)
#define MMC_NOT_PRESENT           ((uint8_t)0x00)

#define MMC_DATATIMEOUT           ((uint32_t)0xFFFFFFFFU)


/**
  * @}
  */


/** @addtogroup STM32L4P5G_DISCOVERY_MMC_Exported_Functions
  * @{
  */
uint8_t BSP_MMC_Init(void);
uint8_t BSP_MMC_DeInit(void);
uint8_t BSP_MMC_ConfigWideBusOperation(uint32_t WideMode);
uint8_t BSP_MMC_ConfigSpeedBusOperation(uint32_t SpeedMode);

uint8_t BSP_MMC_ReadBlocks(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks, uint32_t Timeout);
uint8_t BSP_MMC_WriteBlocks(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks, uint32_t Timeout);
uint8_t BSP_MMC_ReadBlocks_DMA(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks);
uint8_t BSP_MMC_WriteBlocks_DMA(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks);
uint8_t BSP_MMC_ReadBlocks_IT(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks);
uint8_t BSP_MMC_WriteBlocks_IT(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks);
uint8_t BSP_MMC_ReadBlocks_DMAMultiBuffer(uint32_t *pData1, uint32_t *pData2, uint32_t ReadAddr, uint32_t NumOfBlocks);
uint8_t BSP_MMC_WriteBlocks_DMAMultiBuffer(uint32_t *pData1, uint32_t *pData2, uint32_t WriteAddr, uint32_t NumOfBlocks);
uint8_t BSP_MMC_Erase(uint32_t StartAddr, uint32_t EndAddr);
uint8_t BSP_MMC_GetCardState(void);
uint8_t BSP_MMC_GetCardInfo(BSP_MMC_CardInfo *CardInfo);
uint8_t BSP_MMC_GetCardCID(BSP_MMC_CardCID *CardCID);
uint8_t BSP_MMC_GetCardCSD(BSP_MMC_CardCSD *CardCSD);
void    BSP_MMC_IRQHandler(void);

/* These functions can be modified in case the current settings (e.g. DMA stream)
   need to be changed for specific application needs */
void    BSP_MMC_MspInit(MMC_HandleTypeDef *hmmc, void *Params);
void    BSP_MMC_MspDeInit(MMC_HandleTypeDef *hmmc, void *Params);
void    BSP_MMC_AbortCallback(void);
void    BSP_MMC_WriteCpltCallback(void);
void    BSP_MMC_ReadCpltCallback(void);


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

#endif /* __STM32L4P5G_DISCOVERY_MMC_H */


