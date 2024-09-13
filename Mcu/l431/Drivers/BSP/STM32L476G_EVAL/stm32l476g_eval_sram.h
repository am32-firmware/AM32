/**
  ******************************************************************************
  * @file    stm32l476g_eval_sram.h
  * @author  MCD Application Team
  * @brief   This file contains the common defines and functions prototypes for
  *          the stm32l476g_eval_sram.c driver.
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
#ifndef __STM32L476G_EVAL_SRAM_H
#define __STM32L476G_EVAL_SRAM_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L476G_EVAL
  * @{
  */

/** @addtogroup STM32L476G_EVAL_SRAM
  * @{
  */

/* Exported constants --------------------------------------------------------*/ 

/** @defgroup STM32L476G_EVAL_SRAM_Exported_Constants Exported Constants
  * @{
  */

/** 
  * @brief  SRAM status structure definition  
  */     
#define   SRAM_OK         0x00
#define   SRAM_ERROR      0x01

#define SRAM_DEVICE_ADDR  ((uint32_t)0x60000000)
#define SRAM_DEVICE_SIZE  ((uint32_t)0x200000)  /* SRAM device size in MBytes */  
  
/* #define SRAM_MEMORY_WIDTH    FMC_NORSRAM_MEM_BUS_WIDTH_8  */
#define SRAM_MEMORY_WIDTH    FMC_NORSRAM_MEM_BUS_WIDTH_16

#define SRAM_BURSTACCESS    FMC_BURST_ACCESS_MODE_DISABLE  
/* #define SRAM_BURSTACCESS    FMC_BURST_ACCESS_MODE_ENABLE*/
  
#define SRAM_WRITEBURST    FMC_WRITE_BURST_DISABLE  
/* #define SRAM_WRITEBURST   FMC_WRITE_BURST_ENABLE */
 
/* DMA definitions for SRAM DMA transfer */
#define SRAM_DMAx_CLK_ENABLE              __HAL_RCC_DMA1_CLK_ENABLE
#define SRAM_DMAx_CHANNEL                 DMA1_Channel1
#define SRAM_DMAx_IRQn                    DMA1_Channel1_IRQn
#define SRAM_DMAx_IRQHandler              DMA1_Channel1_IRQHandler  

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/

/** @defgroup STM32L476G_EVAL_SRAM_Exported_Functions Exported Functions
  * @{
  */    
uint8_t BSP_SRAM_Init(void);
uint8_t BSP_SRAM_ReadData(uint32_t uwStartAddress, uint16_t *pData, uint32_t uwDataSize);
uint8_t BSP_SRAM_ReadData_DMA(uint32_t uwStartAddress, uint16_t *pData, uint32_t uwDataSize);
uint8_t BSP_SRAM_WriteData(uint32_t uwStartAddress, uint16_t *pData, uint32_t uwDataSize);
uint8_t BSP_SRAM_WriteData_DMA(uint32_t uwStartAddress, uint16_t *pData, uint32_t uwDataSize);
void BSP_SRAM_DMA_IRQHandler(void);

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

#endif /* __STM32L476G_EVAL_SRAM_H */


