/**
  ******************************************************************************
  * @file    stm32l4r9i_eval_nor.h
  * @author  MCD Application Team
  * @brief   This file contains the common defines and functions prototypes for
  *          the stm32l4r9i_eval_nor.c driver.
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

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L4R9I_EVAL
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32L4R9I_EVAL_NOR_H
#define __STM32L4R9I_EVAL_NOR_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/** @addtogroup STM32L4R9I_EVAL_NOR
  * @{
  */


/* Exported constants --------------------------------------------------------*/
/** @defgroup STM32L4R9I_EVAL_NOR_Exported_Constants Exported Constants
  * @{
  */

/**
  * @brief  NOR status structure definition
  */
#define NOR_STATUS_OK           0x00
#define NOR_STATUS_ERROR        0x01

#define NOR_DEVICE_ADDR         ((uint32_t)0x68000000)

#define NOR_MEMORY_WIDTH        FMC_NORSRAM_MEM_BUS_WIDTH_16

#define NOR_BURSTACCESS         FMC_BURST_ACCESS_MODE_DISABLE

#define NOR_WRITEBURST          FMC_WRITE_BURST_DISABLE

/* NOR operations Timeout definitions */
#define BLOCKERASE_TIMEOUT      2000U  /* NOR block erase timeout : 2 s */
#define CHIPERASE_TIMEOUT       400000U  /* NOR chip erase timeout : 400 s */
#define PROGRAM_TIMEOUT         13U  /* NOR program timeout : 64 bytes * 200 us = 12.8 ms */

/* NOR Ready/Busy signal GPIO definitions */
#define NOR_READY_BUSY_PIN      GPIO_PIN_6
#define NOR_READY_BUSY_GPIO     GPIOD
#define NOR_READY_STATE         GPIO_PIN_SET
#define NOR_BUSY_STATE          GPIO_PIN_RESET

/**
  * @}
  */


/* Exported functions --------------------------------------------------------*/

/** @addtogroup STM32L4R9I_EVAL_NOR_Exported_Functions
  * @{
  */
uint8_t BSP_NOR_Init(void);
uint8_t BSP_NOR_ReadData(uint32_t uwStartAddress, uint16_t *pData, uint32_t uwDataSize);
uint8_t BSP_NOR_WriteData(uint32_t uwStartAddress, uint16_t *pData, uint32_t uwDataSize);
uint8_t BSP_NOR_ProgramData(uint32_t uwStartAddress, uint16_t *pData, uint32_t uwDataSize);
uint8_t BSP_NOR_Erase_Block(uint32_t BlockAddress);
uint8_t BSP_NOR_Erase_Chip(void);
uint8_t BSP_NOR_Read_ID(NOR_IDTypeDef *pNOR_ID);
void    BSP_NOR_ReturnToReadMode(void);

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __STM32L4R9I_EVAL_NOR_H */
/**
  * @}
  */

/**
  * @}
  */


