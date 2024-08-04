/**
  ******************************************************************************
  * @file    stm32l4r9i_eval_ospi_nor.h
  * @author  MCD Application Team
  * @brief   This file contains the common defines and functions prototypes for
  *          the stm32l4r9i_eval_ospi_nor.c driver.
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
#ifndef __STM32L4R9I_EVAL_OSPI_NOR_H
#define __STM32L4R9I_EVAL_OSPI_NOR_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "../Components/mx25lm51245g/mx25lm51245g.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L4R9I_EVAL
  * @{
  */

/** @addtogroup STM32L4R9I_EVAL_OSPI_NOR
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup STM32L4R9I_EVAL_OSPI_NOR_Exported_Constants Exported Constants
  * @{
  */
/* OSPI Error codes */
#define OSPI_NOR_OK            ((uint8_t)0x00)
#define OSPI_NOR_ERROR         ((uint8_t)0x01)
#define OSPI_NOR_BUSY          ((uint8_t)0x02)
#define OSPI_NOR_NOT_SUPPORTED ((uint8_t)0x04)
#define OSPI_NOR_SUSPENDED     ((uint8_t)0x08)

/**
  * @}
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup STM32L4R9I_EVAL_OSPI_NOR_Exported_Types Exported Types
  * @{
  */
/* OSPI Info */
typedef struct {
  uint32_t FlashSize;          /*!< Size of the flash */
  uint32_t EraseSectorSize;    /*!< Size of sectors for the erase operation */
  uint32_t EraseSectorsNumber; /*!< Number of sectors for the erase operation */
  uint32_t ProgPageSize;       /*!< Size of pages for the program operation */
  uint32_t ProgPagesNumber;    /*!< Number of pages for the program operation */
} OSPI_NOR_Info;

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup STM32L4R9I_EVAL_OSPI_NOR_Exported_Functions Exported Functions
  * @{
  */
uint8_t BSP_OSPI_NOR_Init                  (void);
uint8_t BSP_OSPI_NOR_DeInit                (void);
uint8_t BSP_OSPI_NOR_Read                  (uint8_t* pData, uint32_t ReadAddr, uint32_t Size);
uint8_t BSP_OSPI_NOR_Write                 (uint8_t* pData, uint32_t WriteAddr, uint32_t Size);
uint8_t BSP_OSPI_NOR_Erase_Block           (uint32_t BlockAddress);
uint8_t BSP_OSPI_NOR_Erase_Sector          (uint32_t Sector);
uint8_t BSP_OSPI_NOR_Erase_Chip            (void);
uint8_t BSP_OSPI_NOR_GetStatus             (void);
uint8_t BSP_OSPI_NOR_GetInfo               (OSPI_NOR_Info* pInfo);
uint8_t BSP_OSPI_NOR_EnableMemoryMappedMode(void);
uint8_t BSP_OSPI_NOR_SuspendErase          (void);
uint8_t BSP_OSPI_NOR_ResumeErase           (void);
uint8_t BSP_OSPI_NOR_EnterDeepPowerDown    (void);
uint8_t BSP_OSPI_NOR_LeaveDeepPowerDown    (void);

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

#endif /* __STM32L4R9I_EVAL_OSPI_NOR_H */


