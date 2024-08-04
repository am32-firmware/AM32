/**
  ******************************************************************************
  * @file    stm32l4xx_nucleo_32.h
  * @author  MCD Application Team
  * @brief   This file contains definitions for:
  *          - LED available on STM32L4xx-Nucleo_32 Kit from STMicroelectronics
  *          - 7 segment display from Gravitech
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
#ifndef __STM32L4XX_NUCLEO_32_H
#define __STM32L4XX_NUCLEO_32_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
   

/** @addtogroup BSP
  * @{
  */

/** @defgroup STM32L4XX_NUCLEO_32 NUCLEO 32
  * @brief This section contains the exported types, constants and functions
  *        required to use the Nucleo 32 board.
  * @{
  */

/** @defgroup STM32L4XX_NUCLEO_32_Exported_Types Exported Types
  * @{
  */ 
   
typedef enum 
{
  LED3 = 0,
  LED_GREEN = LED3
} Led_TypeDef;

/**
  * @}
  */ 

/** @defgroup STM32L4XX_NUCLEO_32_Exported_Constants Exported Constants 
  * @brief Define for STM32L4XX_NUCLEO_32 board  
  * @{
  */ 

#if !defined (USE_STM32L4XX_NUCLEO_32)
 #define USE_STM32L4XX_NUCLEO_32
#endif

/** @defgroup STM32L4XX_NUCLEO_LED LED Constants
  * @{
  */

#define LEDn                               1

#define LED3_PIN                           GPIO_PIN_3
#define LED3_GPIO_PORT                     GPIOB
#define LED3_GPIO_CLK_ENABLE()             __HAL_RCC_GPIOB_CLK_ENABLE()  
#define LED3_GPIO_CLK_DISABLE()            __HAL_RCC_GPIOB_CLK_DISABLE()  

#define LEDx_GPIO_CLK_ENABLE(__INDEX__)    do {LED3_GPIO_CLK_ENABLE(); } while(0)
#define LEDx_GPIO_CLK_DISABLE(__INDEX__)   LED3_GPIO_CLK_DISABLE())

/**
  * @}
  */ 
  
/** @defgroup STM32L4XX_NUCLEO_32_BUS BUS Constants
  * @{
  */ 

#if defined(HAL_I2C_MODULE_ENABLED)
/*##################### I2C1 ###################################*/
/* User can use this section to tailor I2Cx instance used and associated resources */
/* Definition for I2C1 Pins */
#define BSP_I2C1                        I2C1
#define BSP_I2C1_CLK_ENABLE()           __HAL_RCC_I2C1_CLK_ENABLE()
#define BSP_I2C1_CLK_DISABLE()          __HAL_RCC_I2C1_CLK_DISABLE()
#define BSP_I2C1_FORCE_RESET()          __HAL_RCC_I2C1_FORCE_RESET()
#define BSP_I2C1_RELEASE_RESET()        __HAL_RCC_I2C1_RELEASE_RESET()  

#define BSP_I2C1_SCL_PIN                GPIO_PIN_6    /* PB.6 add wire between D5 and A5 */
#define BSP_I2C1_SDA_PIN                GPIO_PIN_7    /* PB.7 add wire between D4 and A4 */

#define BSP_I2C1_GPIO_PORT              GPIOB      /* GPIOB */
#define BSP_I2C1_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()
#define BSP_I2C1_GPIO_CLK_DISABLE()     __HAL_RCC_GPIOB_CLK_DISABLE() 
#define BSP_I2C1_SCL_SDA_AF             GPIO_AF4_I2C1
  
/* Maximum Timeout values for flags waiting loops. These timeouts are not based
   on accurate values, they just guarantee that the application will not remain
   stuck if the I2C communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...). */   
#define BSP_I2C1_TIMEOUT_MAX            1000

/* I2C TIMING is calculated in case of the I2C Clock source is the SYSCLK = 80 MHz */
/* Set 0x40E03E53 value to reach 100 KHz speed (Rise time = 640ns, Fall time = 20ns) */
#define I2C1_TIMING                     0x40E03E53

#endif /* HAL_I2C_MODULE_ENABLED */

/**
  * @}
  */

/**
  * @}
  */ 
  
/** @defgroup STM32L4XX_NUCLEO_32_Exported_Functions Exported Functions
  * @{
  */
  
uint32_t         BSP_GetVersion(void);
void             BSP_LED_Init(Led_TypeDef Led);
void             BSP_LED_On(Led_TypeDef Led);
void             BSP_LED_Off(Led_TypeDef Led);
void             BSP_LED_Toggle(Led_TypeDef Led);

/**
  * @}
  */

/**
  * @}
  */
  
/** @defgroup STM32L4XX_NUCLEO_32_GRAVITECH_4DIGITS GRAVITECH 4 DIGITS
  * @brief This section contains the exported functions
  *        required to use Gravitech shield 7 Segment Display
  * @{
  */ 

/** @defgroup STM32_GRAVITECH_4DIGITS_Exported_Constants Exported Constants
  * @{
  */

#define DIGIT4_SEG7_RESET 10000
/**
  * @}
  */

/** @defgroup STM32_GRAVITECH_4DIGITS_Exported_Functions Exported Functions
  * @{
  */   
  
HAL_StatusTypeDef BSP_DIGIT4_SEG7_Init(void);
HAL_StatusTypeDef BSP_DIGIT4_SEG7_Display(uint32_t Value);

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

#endif /* __STM32L4XX_NUCLEO_32_H */



