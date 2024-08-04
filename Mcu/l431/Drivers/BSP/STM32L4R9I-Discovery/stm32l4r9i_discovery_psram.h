/**
  ******************************************************************************
  * @file    stm32l4r9i_discovery_psram.h
  * @author  MCD Application Team
  * @brief   This file contains the common defines and functions prototypes for
  *          the stm32l4r9i_discovery_psram.c driver.
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
#ifndef __STM32L4R9I_DISCOVERY_PSRAM_H
#define __STM32L4R9I_DISCOVERY_PSRAM_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L4R9I_DISCOVERY
  * @{
  */

/** @addtogroup STM32L4R9I_DISCOVERY_PSRAM
  * @{
  */

/** @defgroup STM32L4R9I_DISCOVERY_PSRAM_Exported_Types Exported Constants
  * @{
  */

/**
  * @brief  PSRAM status structure definition
  */
#define   PSRAM_OK         ((uint8_t)0x00)
#define   PSRAM_ERROR      ((uint8_t)0x01)

#define PSRAM_DEVICE_ADDR  ((uint32_t)0x60000000)
#define PSRAM_DEVICE_SIZE  ((uint32_t)0x400000)  /* PSRAM device size in Bytes */

/* #define SRAM_MEMORY_WIDTH    FMC_NORSRAM_MEM_BUS_WIDTH_8*/
#define PSRAM_MEMORY_WIDTH    FMC_NORSRAM_MEM_BUS_WIDTH_16

#define PSRAM_BURSTACCESS     FMC_BURST_ACCESS_MODE_DISABLE
/* #define PSRAM_BURSTACCESS     FMC_BURST_ACCESS_MODE_ENABLE*/

#define PSRAM_WRITEBURST      FMC_WRITE_BURST_DISABLE
/* #define PSRAM_WRITEBURST     FMC_WRITE_BURST_ENABLE */

#define CONTINUOUSCLOCK_FEATURE    FMC_CONTINUOUS_CLOCK_SYNC_ONLY
/* #define CONTINUOUSCLOCK_FEATURE     FMC_CONTINUOUS_CLOCK_SYNC_ASYNC */

/* DMA definitions for SRAM DMA transfer */
#define __PSRAM_DMAx_CLK_ENABLE            __HAL_RCC_DMA2_CLK_ENABLE
#define __PSRAM_DMAx_CLK_DISABLE           __HAL_RCC_DMA2_CLK_DISABLE
#define PSRAM_DMAx_INSTANCE                DMA2_Channel3
#define PSRAM_DMAx_IRQn                    DMA2_Channel3_IRQn
#define BSP_PSRAM_DMA_IRQHandler           DMA2_Channel3_IRQHandler


/** @defgroup STM32L4R9I_DISCOVERY_PSRAM_Exported_Variables Exported Variables
  * @{
  */
extern SRAM_HandleTypeDef psramHandle;

/**
  * @}
  */

/** @defgroup STM32L4R9I_DISCOVERY_PSRAM_Exported_Functions Exported Functions
  * @{
  */
uint8_t BSP_PSRAM_Init(void);
uint8_t BSP_PSRAM_DeInit(void);
uint8_t BSP_PSRAM_ReadData(uint32_t uwStartAddress, uint16_t *pData, uint32_t uwDataSize);
uint8_t BSP_PSRAM_ReadData_DMA(uint32_t uwStartAddress, uint16_t *pData, uint32_t uwDataSize);
uint8_t BSP_PSRAM_WriteData(uint32_t uwStartAddress, uint16_t *pData, uint32_t uwDataSize);
uint8_t BSP_PSRAM_WriteData_DMA(uint32_t uwStartAddress, uint16_t *pData, uint32_t uwDataSize);

/* These __weak functions can be surcharged by application code in case the current settings
   (eg. interrupt priority) need to be changed for specific application needs */
void BSP_PSRAM_MspInit(SRAM_HandleTypeDef *hsram, void *Params);
void BSP_PSRAM_MspDeInit(SRAM_HandleTypeDef *hsram, void *Params);

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

#endif /* __STM32L4R9I_DISCOVERY_PSRAM_H */


