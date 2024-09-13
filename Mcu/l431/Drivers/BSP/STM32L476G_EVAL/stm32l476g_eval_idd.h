 /**
  ******************************************************************************
  * @file    stm32l476g_eval_idd.h
  * @author  MCD Application Team
  * @brief   Header file for stm32l476g_eval_idd.c module.
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
#ifndef __STM32L476G_EVAL_IDD_H
#define __STM32L476G_EVAL_IDD_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l476g_eval.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L476G_EVAL
  * @{
  */

/** @addtogroup STM32L476G_EVAL_IDD
  * @{
  */

/** @defgroup STM32L476G_EVAL_IDD_Exported_Types  Exported Types
  * @{
  */

/** @defgroup IDD_Config  IDD Config
  * @{
  */
typedef enum 
{
  BSP_IDD_OK = 0,
  BSP_IDD_TIMEOUT = 1,
  BSP_IDD_ERROR = 0xFF
} 
IDD_StatusTypeDef;

typedef enum
{
  BSP_IDD_MODE_CALIBRATION = 0, 
  BSP_IDD_MODE_RUN,
  BSP_IDD_MODE_SLEEP,
  BSP_IDD_MODE_STOP,
  BSP_IDD_MODE_STANDBY,
  
}IDD_ModeTypdef;

/**
  * @}
  */

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/ 
/** @defgroup STM32L476G_EVAL_IDD_Exported_Constants Exported Constants
  * @{
  */
/**
  * @brief  Shunt values on evaluation board in ohms
  */
#define EVAL_IDD_SHUNT0_VALUE                  ((uint16_t) 1000)     /*!< value in ohm */
#define EVAL_IDD_SHUNT1_VALUE                  ((uint16_t) 1)        /*!< value in ohm */

/**
  * @brief  IDD Ampli Gain on evaluation board 
  */
#define EVAL_IDD_AMPLI_GAIN                    ((uint16_t) 50)      /*!< value is gain */

/**
  * @brief  IDD Full Scale Measure 
  */
#define EVAL_IDD_FULL_SCALE                    ((uint16_t) 4096)     /*!< value is adc max value */

/**
  * @brief  IDD Vdd Min on eval 
  */
#define EVAL_IDD_VDD                           ((uint16_t) 3300)     /*!< value in millivolt */

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup STM32L476G_EVAL_QSPI_Exported_Functions Exported Functions
  * @{
  */
uint8_t   BSP_IDD_Init(void);
void      BSP_IDD_EnterStopMode1(void);
void      BSP_IDD_EnterStopMode2(void);
void      BSP_IDD_ExitStopMode(void);
void      BSP_IDD_EnterSleepMode(void);
void      BSP_IDD_ExitSleepMode(void);
void      BSP_IDD_EnterRunMode(void);
void      BSP_IDD_EnterStandbyMode(void);
uint32_t  BSP_IDD_GetValue(IDD_ModeTypdef LowPowerMode, uint32_t Calibration, uint32_t *IddValue);

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

#endif /* __STM32L476G_EVAL_IDD_H */


