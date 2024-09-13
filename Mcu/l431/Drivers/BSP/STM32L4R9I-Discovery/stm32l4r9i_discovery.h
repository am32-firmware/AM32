/**
  ******************************************************************************
  * @file    stm32l4r9i_discovery.h
  * @author  MCD Application Team
  * @brief   This file contains definitions for STM32L4R9I_DISCOVERY's LEDs,
  *          push-buttons hardware resources (MB1311).
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
#ifndef __STM32L4R9I_DISCOVERY_H
#define __STM32L4R9I_DISCOVERY_H

#ifdef __cplusplus
 extern "C" {
#endif

/**
  * @brief  Define for STM32L4R9I_DISCOVERY board
  */



/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#if defined(BSP_USE_CMSIS_OS)
#include "cmsis_os.h"
#endif

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L4R9I_DISCOVERY
  * @{
  */

/** @addtogroup STM32L4R9I_DISCOVERY_Common
  * @{
  */

/** @defgroup STM32L4R9I_DISCOVERY_Exported_Types Exported Types
  * @{
  */

/**
 * @brief LED Types Definition
 */
typedef enum
{
  LED1 = 0,
  LED2 = 1,
  LED_ORANGE = LED1,
  LED_GREEN  = LED2
}Led_TypeDef;


/**
 * @brief JOYSTICK Types Definition
 */
typedef enum
{
  JOY_SEL   = 0,
  JOY_LEFT  = 1,
  JOY_RIGHT = 2,
  JOY_DOWN  = 3,
  JOY_UP    = 4,
  JOY_NONE  = 5
}JOYState_TypeDef;

typedef enum
{
  JOY_MODE_GPIO = 0,
  JOY_MODE_EXTI = 1
}JOYMode_TypeDef;

/**
 * @brief COM Type Definition
 */
typedef enum
{
  COM1 = 0
}COM_TypeDef;

/**
  * @}
  */

/** @defgroup STM32L4R9I_DISCOVERY_Exported_Constants Exported Constants
  * @{
  */

/** @defgroup STM32L4R9I_DISCOVERY_LED LED Constants
  * @{
  */
/* LED number */
#define LEDn                              2

/* LED1 is accessed through the MFX */
#define LED1_PIN                          IO_PIN_0

#define LED2_PIN                          GPIO_PIN_4
#define LED2_GPIO_PORT                    GPIOH
#define LED2_GPIO_CLK_ENABLE()            __HAL_RCC_GPIOH_CLK_ENABLE()
#define LED2_GPIO_CLK_DISABLE()           __HAL_RCC_GPIOH_CLK_DISABLE()

/**
  * @}
  */

/** @defgroup STM32L4R9I_DISCOVERY_BUTTON  BUTTON Constants
  * @{
  */
#define JOYn                              5

/**
* @brief Joystick Right push-button
*/
#if defined(USE_STM32L4R9I_DISCO_REVA)
/* Joystick right and up pins inverted */
#define RIGHT_JOY_PIN                     IO_PIN_1
#else
#define RIGHT_JOY_PIN                     IO_PIN_3
#endif
#define RIGHT_JOY_EXTI_IRQn               EXTI1_IRQn

/**
* @brief Joystick Left push-button
*/
#define LEFT_JOY_PIN                      IO_PIN_4
#define LEFT_JOY_EXTI_IRQn                EXTI1_IRQn

/**
* @brief Joystick Up push-button
*/
#if defined(USE_STM32L4R9I_DISCO_REVA)
/* Joystick right and up pins inverted */
#define UP_JOY_PIN                        IO_PIN_3
#else
#define UP_JOY_PIN                        IO_PIN_1
#endif
#define UP_JOY_EXTI_IRQn                  EXTI1_IRQn

/**
 * @brief Joystick Down push-button
 */
#define DOWN_JOY_PIN                      IO_PIN_2
#define DOWN_JOY_EXTI_IRQn                EXTI1_IRQn

/**
 * @brief Joystick Sel push-button
 */
#define SEL_JOY_PIN                       GPIO_PIN_13   /* PC.13 */
#define SEL_JOY_GPIO_PORT                 GPIOC
#define SEL_JOY_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOC_CLK_ENABLE()
#define SEL_JOY_GPIO_CLK_DISABLE()        __HAL_RCC_GPIOC_CLK_DISABLE()
#define SEL_JOY_EXTI_IRQn                 EXTI15_10_IRQn

/**
  * @}
  */

/** @defgroup STM32L4R9I_DISCOVERY_MFX_PIN  MFX Pins Constants
  * @{
  */

/**
  * @brief Pins definition connected to MFX
  */
#define DSI_RST_PIN                    IO_PIN_10
#define EXT_RST_PIN                    IO_PIN_11
#define OTG_FS_POWER_SWITCH_PIN        IO_PIN_13
#define OTG_FS_OVER_CURRENT_PIN        IO_PIN_14
#define SD_DETECT_PIN                  IO_PIN_5

/**
  * @}
  */

/** @defgroup STM32L4R9I_DISCOVERY_COM COM Constants
  * @{
  */
#define COMn                                    1

/**
 * @brief Definition for COM port1, connected to USART2 (ST-Link USB Virtual Com Port)
 */
#define DISCOVERY_COM1                          USART2
#define DISCOVERY_COM1_CLK_ENABLE()             __HAL_RCC_USART2_CLK_ENABLE()
#define DISCOVERY_COM1_CLK_DISABLE()            __HAL_RCC_USART2_CLK_DISABLE()

#define DISCOVERY_COM1_TX_PIN                   GPIO_PIN_2
#define DISCOVERY_COM1_TX_GPIO_PORT             GPIOA
#define DISCOVERY_COM1_TX_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOA_CLK_ENABLE()
#define DISCOVERY_COM1_TX_GPIO_CLK_DISABLE()    __HAL_RCC_GPIOA_CLK_DISABLE()
#define DISCOVERY_COM1_TX_AF                    GPIO_AF7_USART2

#define DISCOVERY_COM1_RX_PIN                   GPIO_PIN_3
#define DISCOVERY_COM1_RX_GPIO_PORT             GPIOA
#define DISCOVERY_COM1_RX_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOA_CLK_ENABLE()
#define DISCOVERY_COM1_RX_GPIO_CLK_DISABLE()    __HAL_RCC_GPIOA_CLK_DISABLE()
#define DISCOVERY_COM1_RX_AF                    GPIO_AF7_USART2

#define DISCOVERY_COM1_IRQn                     USART2_IRQn


#define DISCOVERY_COMx_CLK_ENABLE(__INDEX__)            do { if((__INDEX__) == COM1) {DISCOVERY_COM1_CLK_ENABLE();}} while(0)
#define DISCOVERY_COMx_CLK_DISABLE(__INDEX__)           do { if((__INDEX__) == COM1) {DISCOVERY_COM1_CLK_DISABLE();}} while(0)

#define DISCOVERY_COMx_TX_GPIO_CLK_ENABLE(__INDEX__)    do { if((__INDEX__) == COM1) {DISCOVERY_COM1_TX_GPIO_CLK_ENABLE();}} while(0)
#define DISCOVERY_COMx_TX_GPIO_CLK_DISABLE(__INDEX__)   do { if((__INDEX__) == COM1) {DISCOVERY_COM1_TX_GPIO_CLK_DISABLE();}} while(0)

#define DISCOVERY_COMx_RX_GPIO_CLK_ENABLE(__INDEX__)    do { if((__INDEX__) == COM1) {DISCOVERY_COM1_RX_GPIO_CLK_ENABLE();}} while(0)
#define DISCOVERY_COMx_RX_GPIO_CLK_DISABLE(__INDEX__)   do { if((__INDEX__) == COM1) {DISCOVERY_COM1_RX_GPIO_CLK_DISABLE();}} while(0)

/**
  * @}
  */

/** @defgroup STM32L4R9I_DISCOVERY_BUS  BUS Constants
  * @{
  */

#if defined(HAL_I2C_MODULE_ENABLED)
/*##################### I2C1 ###################################*/

/* I2C TIMING Register define when I2C clock source is SYSCLK */
/* I2C TIMING is calculated in case of the I2C Clock source is the SYSCLK = 120 MHz */
/* Set 0xC080242F value to reach 100 KHz speed (Rise time = 640ns, Fall time = 20ns) */
#ifndef DISCOVERY_I2C1_TIMING
 #define DISCOVERY_I2C1_TIMING                  0xC080242F
#endif /* DISCOVERY_I2C1_TIMING */

/* Maximum Timeout values for flags waiting loops. These timeouts are not based
   on accurate values, they just guarantee that the application will not remain
   stuck if the I2C communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...). */
#define DISCOVERY_I2C1_TIMEOUT_MAX              3000

#define IDD_I2C_ADDRESS                  ((uint16_t) 0x84)
#define IO_I2C_ADDRESS                   ((uint16_t) 0x84)
#define AUDIO_I2C_ADDRESS                ((uint16_t) 0x94)
#define TS_I2C_ADDRESS                   ((uint16_t) 0x70)
#define CAMERA_I2C_ADDRESS               ((uint16_t) 0x60)

#endif /* HAL_I2C_MODULE_ENABLED */

/*##################### Audio Codec ##########################*/
/**
  * @brief  Audio codec chip reset definition
  */
/* Audio codec power on/off macro definition */
#define CODEC_AUDIO_POWER_OFF()      BSP_IO_WritePin(AUDIO_RESET_PIN, GPIO_PIN_RESET)
#define CODEC_AUDIO_POWER_ON()       BSP_IO_WritePin(AUDIO_RESET_PIN, GPIO_PIN_SET)

/* Audio Reset Pin definition */
#define AUDIO_RESET_PIN              IO_PIN_15

/*##################### MFX ##########################*/
/**
  * @brief  MFX interface pins
  */
#define MFX_INT_GPIO_PORT                 GPIOI
#define MFX_INT_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOI_CLK_ENABLE()
#define MFX_INT_GPIO_CLK_DISABLE()        __HAL_RCC_GPIOI_CLK_DISABLE()
#define MFX_INT_PIN                       GPIO_PIN_1                  /* PI.01 */
#define MFX_INT_EXTI_IRQn                 EXTI1_IRQn

#define MFX_WAKEUP_GPIO_PORT              GPIOB
#define MFX_WAKEUP_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()
#define MFX_WAKEUP_GPIO_CLK_DISABLE()     __HAL_RCC_GPIOB_CLK_DISABLE()
#define MFX_WAKEUP_PIN                    GPIO_PIN_2                  /* PB.02 */

/* Legacy  */
#define IDD_INT_GPIO_PORT                 MFX_INT_GPIO_PORT
#define IDD_INT_GPIO_CLK_ENABLE()         MFX_INT_GPIO_CLK_ENABLE()
#define IDD_INT_GPIO_CLK_DISABLE()        MFX_INT_GPIO_CLK_DISABLE()
#define IDD_INT_PIN                       MFX_INT_PIN
#define IDD_INT_EXTI_IRQn                 MFX_INT_EXTI_IRQn
#define IDD_WAKEUP_GPIO_PORT              MFX_WAKEUP_GPIO_PORT
#define IDD_WAKEUP_GPIO_CLK_ENABLE()      MFX_WAKEUP_GPIO_CLK_ENABLE()
#define IDD_WAKEUP_GPIO_CLK_DISABLE()     MFX_WAKEUP_GPIO_CLK_DISABLE()
#define IDD_WAKEUP_PIN                    MFX_WAKEUP_PIN

/**
  * @brief  Idd current measurement interface pins on MFX
  */
#define IDD_AMP_CONTROL_PIN               AGPIO_PIN_1

/**
  * @brief LCD pins
  */
/* TE pin */
#define LCD_TE_PIN                       GPIO_PIN_11
#define LCD_TE_CTRL_GPIO_PORT            GPIOF
#define LCD_TE_CTRL_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOF_CLK_ENABLE()
#define LCD_TE_CTRL_GPIO_CLK_DISABLE()   __HAL_RCC_GPIOF_CLK_DISABLE()

/* Backlight control pin */
#define LCD_BL_CTRL_PIN                  GPIO_PIN_1
#define LCD_BL_CTRL_GPIO_PORT            GPIOB
#define LCD_BL_CTRL_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOB_CLK_ENABLE()
#define LCD_BL_CTRL_GPIO_CLK_DISABLE()   __HAL_RCC_GPIOB_CLK_DISABLE()

/* TS INT pin */
#define TS_INT_PIN                       IO_PIN_9
#define TS_INT_EXTI_IRQn                 EXTI1_IRQn

/**
  * @}
  */

/**
  * @}
  */

/** @defgroup STM32L4R9I_DISCOVERY_Exported_Functions Exported Functions
  * @{
  */
uint32_t                BSP_GetVersion(void);
void                    BSP_LED_Init(Led_TypeDef Led);
void                    BSP_LED_DeInit(Led_TypeDef Led);
void                    BSP_LED_On(Led_TypeDef Led);
void                    BSP_LED_Off(Led_TypeDef Led);
void                    BSP_LED_Toggle(Led_TypeDef Led);
uint8_t                 BSP_JOY_Init(JOYMode_TypeDef Joy_Mode);
void                    BSP_JOY_DeInit(void);
JOYState_TypeDef        BSP_JOY_GetState(void);
#if defined(HAL_UART_MODULE_ENABLED)
void                    BSP_COM_Init(COM_TypeDef COM, UART_HandleTypeDef *husart);
void                    BSP_COM_DeInit(COM_TypeDef COM, UART_HandleTypeDef *huart);
#endif /* HAL_UART_MODULE_ENABLED */

/* These __weak functions can be surcharged by application code for specific application needs */
void                    BSP_ErrorHandler(void);

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

#endif /* __STM32L4R9I_DISCOVERY_H */


