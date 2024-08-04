/**
  ******************************************************************************
  * @file    stm32l4xx_nucleo_144.h
  * @author  MCD Application Team
  * @brief   This file contains definitions for:
  *          - LEDs and push-button available on STM32L4XX-Nucleo-144 Kit
  *            from STMicroelectronics
  *          - LCD, joystick and microSD available on Adafruit 1.8" TFT LCD
  *            shield (reference ID 802)
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
#ifndef __STM32L4XX_NUCLEO_144_H
#define __STM32L4XX_NUCLEO_144_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L4XX_NUCLEO_144
  * @{
  */

/** @defgroup STM32L4XX_NUCLEO_144_Exported_Types Exported Types
  * @{
  */
typedef enum
{
  LED1 = 0,
  LED_GREEN = LED1,
  LED2 = 1,
  LED_BLUE = LED2,
  LED3 = 2,
  LED_RED = LED3
}
Led_TypeDef;

typedef enum
{
  BUTTON_USER = 0,
  /* Alias */
  BUTTON_KEY = BUTTON_USER
} Button_TypeDef;

typedef enum
{
  BUTTON_MODE_GPIO = 0,
  BUTTON_MODE_EXTI = 1
} ButtonMode_TypeDef;

typedef enum
{
  JOY_NONE  = 0,
  JOY_SEL   = 1,
  JOY_DOWN  = 2,
  JOY_LEFT  = 3,
  JOY_RIGHT = 4,
  JOY_UP    = 5
} JOYState_TypeDef;

/**
  * @}
  */

/** @defgroup STM32L4XX_NUCLEO_144_Exported_Constants Exported Constants
  * @{
  */

/**
  * @brief Define for STM32L4XX_NUCLEO_144 board
  */
#if !defined (USE_STM32L4XX_NUCLEO_144)
#define USE_STM32L4XX_NUCLEO_144
#endif

/** @defgroup STM32L4XX_NUCLEO_144_LED LED
  * @{
  */
#define LEDn                                    3

#define LED1_PIN                                GPIO_PIN_7
#define LED1_GPIO_PORT                          GPIOC
#define LED1_GPIO_CLK_ENABLE()                  __HAL_RCC_GPIOC_CLK_ENABLE()
#define LED1_GPIO_CLK_DISABLE()                 __HAL_RCC_GPIOC_CLK_DISABLE()

#define LED2_PIN                                GPIO_PIN_7
#define LED2_GPIO_PORT                          GPIOB
#define LED2_GPIO_CLK_ENABLE()                  __HAL_RCC_GPIOB_CLK_ENABLE()
#define LED2_GPIO_CLK_DISABLE()                 __HAL_RCC_GPIOB_CLK_DISABLE()

#define LED3_PIN                                GPIO_PIN_14
#define LED3_GPIO_PORT                          GPIOB
#define LED3_GPIO_CLK_ENABLE()                  __HAL_RCC_GPIOB_CLK_ENABLE()
#define LED3_GPIO_CLK_DISABLE()                 __HAL_RCC_GPIOB_CLK_DISABLE()

#define LEDx_GPIO_CLK_ENABLE(__INDEX__)   do { if((__INDEX__) == 0) {__HAL_RCC_GPIOC_CLK_ENABLE();} else\
                                                                    {__HAL_RCC_GPIOB_CLK_ENABLE();   }} while(0)
#define LEDx_GPIO_CLK_DISABLE(__INDEX__)  do { if((__INDEX__) == 0) {__HAL_RCC_GPIOC_CLK_DISABLE();} else\
                                                                    {__HAL_RCC_GPIOB_CLK_DISABLE();   }} while(0)

/**
  * @}
  */

/** @defgroup STM32L4XX_NUCLEO_144_BUTTON BUTTON
  * @{
  */
#define BUTTONn                                 1

/**
 * @brief Key push-button
 */
#define USER_BUTTON_PIN                       GPIO_PIN_13
#define USER_BUTTON_GPIO_PORT                 GPIOC
#define USER_BUTTON_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOC_CLK_ENABLE()
#define USER_BUTTON_GPIO_CLK_DISABLE()        __HAL_RCC_GPIOC_CLK_DISABLE()
#define USER_BUTTON_EXTI_LINE                 GPIO_PIN_13
#define USER_BUTTON_EXTI_IRQn                 EXTI15_10_IRQn

#define BUTTONx_GPIO_CLK_ENABLE(__INDEX__)    USER_BUTTON_GPIO_CLK_ENABLE()
#define BUTTONx_GPIO_CLK_DISABLE(__INDEX__)   USER_BUTTON_GPIO_CLK_DISABLE()

/* Aliases */
#define KEY_BUTTON_PIN                       USER_BUTTON_PIN
#define KEY_BUTTON_GPIO_PORT                 USER_BUTTON_GPIO_PORT
#define KEY_BUTTON_GPIO_CLK_ENABLE()         USER_BUTTON_GPIO_CLK_ENABLE()
#define KEY_BUTTON_GPIO_CLK_DISABLE()        USER_BUTTON_GPIO_CLK_DISABLE()
#define KEY_BUTTON_EXTI_LINE                 USER_BUTTON_EXTI_LINE
#define KEY_BUTTON_EXTI_IRQn                 USER_BUTTON_EXTI_IRQn

/**
  * @}
  */


/** @defgroup STM32L4XX_NUCLEO_144_PIN PIN
  * @{
  */
#define OTG_FS1_OVER_CURRENT_PIN                  GPIO_PIN_5
#define OTG_FS1_OVER_CURRENT_PORT                 GPIOG
#define OTG_FS1_OVER_CURRENT_PORT_CLK_ENABLE()    __HAL_RCC_GPIOG_CLK_ENABLE()

#define OTG_FS1_POWER_SWITCH_PIN                  GPIO_PIN_6
#define OTG_FS1_POWER_SWITCH_PORT                 GPIOG
#define OTG_FS1_POWER_SWITCH_PORT_CLK_ENABLE()    __HAL_RCC_GPIOG_CLK_ENABLE()

/**
  * @}
  */

#ifdef USE_STM32L4XX_NUCLEO_144_SMPS
/** @defgroup STM32L4XX_NUCLEO_144_SMPS SMPS
  * @{
  */
#define SMPS_OK     0
#define SMPS_KO     1

/**
  * @}
  */
#endif /* USE_STM32L4XX_NUCLEO_144_SMPS */
  

/** @defgroup STM32L4XX_NUCLEO_144_BUS BUS
  * @{
  */
/*############################### SPI_A #######################################*/
#ifdef HAL_SPI_MODULE_ENABLED

#define NUCLEO_SPIx                                     SPI1
#define NUCLEO_SPIx_CLK_ENABLE()                        __HAL_RCC_SPI1_CLK_ENABLE()

#define NUCLEO_SPIx_SCK_AF                              GPIO_AF5_SPI1
#define NUCLEO_SPIx_SCK_GPIO_PORT                       GPIOA
#define NUCLEO_SPIx_SCK_PIN                             GPIO_PIN_5
#define NUCLEO_SPIx_SCK_GPIO_CLK_ENABLE()               __HAL_RCC_GPIOA_CLK_ENABLE()
#define NUCLEO_SPIx_SCK_GPIO_CLK_DISABLE()              __HAL_RCC_GPIOA_CLK_DISABLE()

#define NUCLEO_SPIx_MISO_MOSI_AF                        GPIO_AF5_SPI1
#define NUCLEO_SPIx_MISO_MOSI_GPIO_PORT                 GPIOA
#define NUCLEO_SPIx_MISO_MOSI_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOA_CLK_ENABLE()
#define NUCLEO_SPIx_MISO_MOSI_GPIO_CLK_DISABLE()        __HAL_RCC_GPIOA_CLK_DISABLE()
#define NUCLEO_SPIx_MISO_PIN                            GPIO_PIN_6
#define NUCLEO_SPIx_MOSI_PIN                            GPIO_PIN_7
/* Maximum Timeout values for flags waiting loops. These timeouts are not based
   on accurate values, they just guarantee that the application will not remain
   stuck if the SPI communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...). */
#define NUCLEO_SPIx_TIMEOUT_MAX                   1000

#define NUCLEO_SPIx_CS_GPIO_PORT                        GPIOD
#define NUCLEO_SPIx_CS_PIN                              GPIO_PIN_14
#define NUCLEO_SPIx_CS_GPIO_CLK_ENABLE()                __HAL_RCC_GPIOD_CLK_ENABLE()
#define NUCLEO_SPIx_CS_GPIO_CLK_DISABLE()               __HAL_RCC_GPIOD_CLK_DISABLE()

#define SPIx__CS_LOW()       HAL_GPIO_WritePin(NUCLEO_SPIx_CS_GPIO_PORT, NUCLEO_SPIx_CS_PIN, GPIO_PIN_RESET)
#define SPIx__CS_HIGH()      HAL_GPIO_WritePin(NUCLEO_SPIx_CS_GPIO_PORT, NUCLEO_SPIx_CS_PIN, GPIO_PIN_SET)

/**
  * @brief  SD Control Lines management
  */
#define SD_CS_LOW()       HAL_GPIO_WritePin(SD_CS_GPIO_PORT, SD_CS_PIN, GPIO_PIN_RESET)
#define SD_CS_HIGH()      HAL_GPIO_WritePin(SD_CS_GPIO_PORT, SD_CS_PIN, GPIO_PIN_SET)

/**
  * @brief  LCD Control Lines management
  */
#define LCD_CS_LOW()      HAL_GPIO_WritePin(LCD_CS_GPIO_PORT, LCD_CS_PIN, GPIO_PIN_RESET)
#define LCD_CS_HIGH()     HAL_GPIO_WritePin(LCD_CS_GPIO_PORT, LCD_CS_PIN, GPIO_PIN_SET)
#define LCD_DC_LOW()      HAL_GPIO_WritePin(LCD_DC_GPIO_PORT, LCD_DC_PIN, GPIO_PIN_RESET)
#define LCD_DC_HIGH()     HAL_GPIO_WritePin(LCD_DC_GPIO_PORT, LCD_DC_PIN, GPIO_PIN_SET)

/**
  * @brief  SD Control Interface pins (shield D4)
  */
#define SD_CS_PIN                                 GPIO_PIN_14
#define SD_CS_GPIO_PORT                           GPIOF
#define SD_CS_GPIO_CLK_ENABLE()                 __HAL_RCC_GPIOF_CLK_ENABLE()
#define SD_CS_GPIO_CLK_DISABLE()                __HAL_RCC_GPIOF_CLK_DISABLE()

/**
  * @brief  LCD Control Interface pins (shield D10)
  */
#define LCD_CS_PIN                                 GPIO_PIN_14
#define LCD_CS_GPIO_PORT                           GPIOD
#define LCD_CS_GPIO_CLK_ENABLE()                 __HAL_RCC_GPIOD_CLK_ENABLE()
#define LCD_CS_GPIO_CLK_DISABLE()                __HAL_RCC_GPIOD_CLK_DISABLE()

/**
  * @brief  LCD Data/Command Interface pins (shield D8)
  */
#define LCD_DC_PIN                                 GPIO_PIN_12
#define LCD_DC_GPIO_PORT                           GPIOF
#define LCD_DC_GPIO_CLK_ENABLE()                 __HAL_RCC_GPIOF_CLK_ENABLE()
#define LCD_DC_GPIO_CLK_DISABLE()                __HAL_RCC_GPIOF_CLK_DISABLE()

#endif /* HAL_SPI_MODULE_ENABLED */

/*################################ ADC1 ######################################*/
/**
  * @brief  ADC Interface pins
  *         used to detect motion of Joystick available on Adafruit 1.8" TFT shield
  */

#ifdef HAL_ADC_MODULE_ENABLED

#define NUCLEO_ADCx                               ADC1

#define NUCLEO_ADCx_CHANNEL                       ADC_CHANNEL_2
#define NUCLEO_ADCx_SAMPLETIME                    ADC_SAMPLETIME_24CYCLES_5
#define NUCLEO_ADCx_CLK_ENABLE()                  __HAL_RCC_ADC_CLK_ENABLE()
#define NUCLEO_ADCx_CLK_DISABLE()                 __HAL_RCC_ADC_CLK_DISABLE()

#define NUCLEO_ADCx_GPIO_PORT                       GPIOC
#define NUCLEO_ADCx_GPIO_PIN                        GPIO_PIN_1
#define NUCLEO_ADCx_GPIO_CLK_ENABLE()             __HAL_RCC_GPIOC_CLK_ENABLE()
#define NUCLEO_ADCx_GPIO_CLK_DISABLE()            __HAL_RCC_GPIOC_CLK_DISABLE()

#endif /* HAL_ADC_MODULE_ENABLED */

/*##################### SMPS###################################*/
#ifdef USE_STM32L4XX_NUCLEO_144_SMPS

#ifdef USE_ADP5301ACBZ          /* ADP5301ACBZ */
#define PORT_SMPS               GPIOG
#define PIN_SMPS_ENABLE         GPIO_PIN_11
#define PIN_SMPS_POWERGOOD      GPIO_PIN_12
#define PIN_SMPS_SWITCH_ENABLE  GPIO_PIN_13

#define PWR_GPIO_SMPS           PWR_GPIO_G
#define PWR_GPIO_ENABLE         PWR_GPIO_BIT_11
#define PWR_GPIO_SWITCH_ENABLE  PWR_GPIO_BIT_13

#define PWR_AND_CLK_SMPS()   do { __HAL_RCC_PWR_CLK_ENABLE(); \
                                  HAL_PWREx_EnableVddIO2(); \
                                  __HAL_RCC_GPIOG_CLK_ENABLE(); } while(0)

#endif                          /* ADP5301ACBZ */

#endif /* USE_STM32L4XX_NUCLEO_144_SMPS */

/**
  * @}
  */

/**
  * @}
  */

/** @addtogroup STM32L4XX_NUCLEO_144_Exported_Functions
  * @{
  */
uint32_t         BSP_GetVersion(void);

/** @addtogroup STM32L4XX_NUCLEO_144_LED_Functions
  * @{
  */
void             BSP_LED_Init(Led_TypeDef Led);
void             BSP_LED_DeInit(Led_TypeDef Led);
void             BSP_LED_On(Led_TypeDef Led);
void             BSP_LED_Off(Led_TypeDef Led);
void             BSP_LED_Toggle(Led_TypeDef Led);
/**
  * @}
  */

/** @addtogroup STM32L4XX_NUCLEO_144_BUTTON_Functions
  * @{
  */
void             BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef ButtonMode);
void             BSP_PB_DeInit(Button_TypeDef Button);
uint32_t         BSP_PB_GetState(Button_TypeDef Button);
#if defined(HAL_ADC_MODULE_ENABLED)
uint8_t          BSP_JOY_Init(void);
void             BSP_JOY_DeInit(void);
JOYState_TypeDef BSP_JOY_GetState(void);
#endif /* HAL_ADC_MODULE_ENABLED */
/**
  * @}
  */

#ifdef USE_STM32L4XX_NUCLEO_144_SMPS
/** @addtogroup STM32L4XX_NUCLEO_144_SMPS_Functions
  * @{
  */
uint32_t         BSP_SMPS_Init(uint32_t VoltageRange);
uint32_t         BSP_SMPS_DeInit(void);
uint32_t         BSP_SMPS_Enable(uint32_t Delay, uint32_t Power_Good_Check);
uint32_t         BSP_SMPS_Disable(void);
uint32_t         BSP_SMPS_Supply_Enable(uint32_t Delay, uint32_t Power_Good_Check);
uint32_t         BSP_SMPS_Supply_Disable(void);
/**
  * @}
  */
#endif /* USE_STM32L4XX_NUCLEO_144_SMPS */

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

#endif /* __STM32L4XX_NUCLEO_144_H */


