/**
  ******************************************************************************
  * @file    stm32l4p5g_discovery_psram.h
  * @author  MCD Application Team
  * @brief   This file contains the common defines and functions prototypes for
  *          the stm32l4p5g_discovery_psram.c driver.
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
#ifndef __STM32L4P5G_DISCOVERY_PSRAM_H
#define __STM32L4P5G_DISCOVERY_PSRAM_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L4P5G_DISCOVERY
  * @{
  */

/** @addtogroup STM32L4P5G_DISCOVERY_PSRAM
  * @{
  */

/** @defgroup STM32L4P5G_DISCOVERY_PSRAM_Exported_Types Exported Constants
  * @{
  */

/**
  * @brief  PSRAM status structure definition
  */
#define   PSRAM_OK                   ((uint8_t)0x00)
#define   PSRAM_ERROR                ((uint8_t)0x01)

/* Read Operations */
#define READ_CMD_SRAM                           0x00
#define READ_LINEAR_BURST_CMD                   0x20
#define READ_HYBRID_BURST_CMD                   0x3F
/* Write Operations */
#define WRITE_CMD_SRAM                          0x80
#define WRITE_LINEAR_BURST_CMD                  0xA0
#define WRITE_HYBRID_BURST_CMD                  0xBF
/* Reset Operations */
#define RESET_CMD_SRAM                          0xFF
/* Register Operations */
#define READ_REG_CMD_SRAM                       0x40
#define WRITE_REG_CMD_SRAM                      0xC0
/* Default dummy clocks cycles */
#define DUMMY_CLOCK_CYCLES_SRAM_READ               5
#define DUMMY_CLOCK_CYCLES_SRAM_WRITE              4


#define PSRAM_DEVICE_ADDR                 0x90000000


/* DMA definitions for SRAM DMA transfer */
#define __PSRAM_DMAx_CLK_ENABLE            __HAL_RCC_DMA1_CLK_ENABLE
#define __PSRAM_DMAx_CLK_DISABLE           __HAL_RCC_DMA1_CLK_DISABLE
#define PSRAM_DMAx_INSTANCE                DMA1_Channel5
#define PSRAM_DMAx_IRQn                    DMA1_Channel5_IRQn
#define BSP_PSRAM_DMA_IRQHandler           DMA1_Channel5_IRQHandler
#define BSP_PSRAM_OSPI_IRQHandler          OCTOSPI1_IRQHandler


/** @defgroup STM32L4P5G_DISCOVERY_PSRAM_Exported_Variables Exported Variables
  * @{
  */

/**
  * @}
  */

/** @defgroup STM32L4P5G_DISCOVERY_PSRAM_Exported_Functions Exported Functions
  * @{
  */
uint8_t BSP_PSRAM_Init(void);
uint8_t BSP_PSRAM_DeInit(void);
uint8_t BSP_PSRAM_EnableMemoryMappedMode(void);
uint8_t BSP_PSRAM_ReadData(uint32_t uwStartAddress, uint8_t *pData, uint32_t uwDataSize);
uint8_t BSP_PSRAM_ReadData_IT(uint32_t uwStartAddress, uint8_t *pData, uint32_t uwDataSize);
uint8_t BSP_PSRAM_ReadData_DMA(uint32_t uwStartAddress, uint8_t *pData, uint32_t uwDataSize);
uint8_t BSP_PSRAM_WriteData(uint32_t uwStartAddress, uint8_t *pData, uint32_t uwDataSize);
uint8_t BSP_PSRAM_WriteData_IT(uint32_t uwStartAddress, uint8_t *pData, uint32_t uwDataSize);
uint8_t BSP_PSRAM_WriteData_DMA(uint32_t uwStartAddress, uint8_t *pData, uint32_t uwDataSize);


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

#endif /* __STM32L4P5G_DISCOVERY_PSRAM_H */


