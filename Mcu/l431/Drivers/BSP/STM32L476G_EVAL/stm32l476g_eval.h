/**
  ******************************************************************************
  * @file    stm32l476g_eval.h
  * @author  MCD Application Team
  * @brief   This file contains definitions for STM32L476G_EVAL's LEDs,
  *          push-buttons and COM ports hardware resources.
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
#ifndef __STM32L476G_EVAL_H
#define __STM32L476G_EVAL_H

#ifdef __cplusplus
 extern "C" {
#endif

/**
  * @brief  Define for STM32L476G_EVAL board
  */
#if !defined(USE_STM32L476G_EVAL_REVA) && !defined(USE_STM32L476G_EVAL_REVB)
#define USE_STM32L476G_EVAL_REVB   /* Support from board revision B onwards */
#endif


/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L476G_EVAL
  * @{
  */


/** @addtogroup STM32L476G_EVAL_Common
  * @{
  */

/** @defgroup STM32L476G_EVAL_Exported_Types Exported Types
  * @{
  */

/**
 * @brief LED Types Definition
 */
typedef enum
{
  LED1 = 0,
  LED_GREEN  = LED1,

#if defined(USE_STM32L476G_EVAL_REVB)
  LED3 = 2,
  LED_RED    = LED3,
#endif /* USE_STM32L476G_EVAL_REVB */

#if defined(USE_IOEXPANDER)  /* LED2, LED3 and LED4 over IOExpander on RevA, LED2 and LED4 on RevB */

  LED2 = 1,
  LED_ORANGE = LED2,

#if defined(USE_STM32L476G_EVAL_REVA)
  LED3 = 2,
  LED_RED    = LED3,
#endif /* USE_STM32L476G_EVAL_REVA */

  LED4 = 3,
  LED_BLUE   = LED4
#endif /* USE_IOEXPANDER */

}Led_TypeDef;

/**
 * @brief BUTTON Types Definition
 */
typedef enum
{
  BUTTON_TAMPER = 0,
  BUTTON_SEL   = 1,
  BUTTON_LEFT  = 2,
  BUTTON_RIGHT = 3,
  BUTTON_DOWN  = 4,
  BUTTON_UP    = 5

}Button_TypeDef;

typedef enum
{
  BUTTON_MODE_GPIO = 0,
  BUTTON_MODE_EXTI = 1

}ButtonMode_TypeDef;

#if defined(USE_IOEXPANDER)
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
#endif /* USE_IOEXPANDER */

/**
 * @brief COM Types Definition
 */
typedef enum
{
  COM1 = 0

}COM_TypeDef;

/**
  * @}
  */

/** @defgroup STM32L476G_EVAL_Exported_Constants Exported Constants
  * @{
  */

/** @defgroup STM32L476G_EVAL_LED LED Constants
  * @{
  */
#define LEDn                             4

#define LED1_PIN                         GPIO_PIN_2
#define LED1_GPIO_PORT                   GPIOB
#define LED1_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOB_CLK_ENABLE()
#define LED1_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOB_CLK_DISABLE()

#if defined(USE_STM32L476G_EVAL_REVB)
#define LED3_PIN                         GPIO_PIN_1
#define LED3_GPIO_PORT                   GPIOC
#define LED3_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOC_CLK_ENABLE()
#define LED3_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOC_CLK_DISABLE()
#endif /* USE_STM32L476G_EVAL_REVB */

#if defined(USE_IOEXPANDER)
/* LED2 is accessed through the IO Expander */
#define LED2_PIN                         IO1_PIN_0

#if defined(USE_STM32L476G_EVAL_REVA)
/* LED3 is accessed through the IO Expander */
#define LED3_PIN                         IO1_PIN_3
#endif

/* LED4 is accessed through the IO Expander */
#define LED4_PIN                         IO1_PIN_2
#endif /* USE_IOEXPANDER */

#if defined(USE_STM32L476G_EVAL_REVB)
#define LEDx_GPIO_CLK_ENABLE(__LED__)    do { if((__LED__) == LED1) { LED1_GPIO_CLK_ENABLE(); } else \
                                              if((__LED__) == LED3) { LED3_GPIO_CLK_ENABLE(); } } while(0)

#define LEDx_GPIO_CLK_DISABLE(__LED__)   do { if((__LED__) == LED1) { LED1_GPIO_CLK_DISABLE(); } else \
                                              if((__LED__) == LED3) { LED3_GPIO_CLK_DISABLE(); } } while(0)

#elif defined(USE_STM32L476G_EVAL_REVA)
#define LEDx_GPIO_CLK_ENABLE(__LED__)    do { if((__LED__) == LED1) { LED1_GPIO_CLK_ENABLE(); } } while(0)

#define LEDx_GPIO_CLK_DISABLE(__LED__)   do { if((__LED__) == LED1) { LED1_GPIO_CLK_DISABLE(); } } while(0)

#endif
/**
  * @}
  */

/** @defgroup STM32L476G_EVAL_BUTTON  BUTTON Constants
  * @{
  */
#if defined(USE_IOEXPANDER)
#define JOYn                             5
#define BUTTONn                          1 + JOYn
#else
#define BUTTONn                          1
#endif

/**
 * @brief Tamper/wake-up push-button
 */
#define TAMPER_BUTTON_PIN                GPIO_PIN_13
#define TAMPER_BUTTON_GPIO_PORT          GPIOC
#define TAMPER_BUTTON_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOC_CLK_ENABLE()
#define TAMPER_BUTTON_GPIO_CLK_DISABLE() __HAL_RCC_GPIOC_CLK_DISABLE()
#define TAMPER_BUTTON_EXTI_IRQn          EXTI15_10_IRQn

#define BUTTONx_GPIO_CLK_ENABLE(__BUTTON__)     do { if((__BUTTON__) == BUTTON_TAMPER) { TAMPER_BUTTON_GPIO_CLK_ENABLE(); } } while(0)

#define BUTTONx_GPIO_CLK_DISABLE(__BUTTON__)    do { if((__BUTTON__) == BUTTON_TAMPER) { TAMPER_BUTTON_GPIO_CLK_DISABLE(); } } while(0)


#if defined(USE_IOEXPANDER)
/**********************************************************/
/* The joystick buttons are accessed through the IO Expander */
/**********************************************************/
#define JOYSTICK_PIN                     GPIO_PIN_15

/**
 * @brief Joystick Right push-button
 */
#define RIGHT_JOY_PIN                    IO2_PIN_3
#define RIGHT_JOY_EXTI_IRQn              EXTI15_10_IRQn

/**
 * @brief Joystick Left push-button
 */
#define LEFT_JOY_PIN                     IO2_PIN_2
#define LEFT_JOY_EXTI_IRQn               EXTI15_10_IRQn

/**
 * @brief Joystick Up push-button
 */
#define UP_JOY_PIN                       IO2_PIN_4
#define UP_JOY_EXTI_IRQn                 EXTI15_10_IRQn

/**
 * @brief Joystick Down push-button
 */
#define DOWN_JOY_PIN                     IO2_PIN_1
#define DOWN_JOY_EXTI_IRQn               EXTI15_10_IRQn

/**
 * @brief Joystick Sel push-button
 */
#define SEL_JOY_PIN                      IO2_PIN_0
#define SEL_JOY_EXTI_IRQn                EXTI15_10_IRQn


#define JOY_ALL_PINS                     (RIGHT_JOY_PIN | LEFT_JOY_PIN | UP_JOY_PIN | DOWN_JOY_PIN | SEL_JOY_PIN)
#endif /* USE_IOEXPANDER */

/**
  * @}
  */

/** @defgroup STM32L476G_EVAL_COM  COM Constants
  * @{
  */
#define COMn                                    1

/**
 * @brief Definition for COM port1, connected to USART1
 */
#define EVAL_COM1                               USART1
#define EVAL_COM1_CLK_ENABLE()                  __HAL_RCC_USART1_CLK_ENABLE()
#define EVAL_COM1_CLK_DISABLE()                 __HAL_RCC_USART1_CLK_DISABLE()

#define EVAL_COM1_TX_PIN                        GPIO_PIN_6
#define EVAL_COM1_TX_GPIO_PORT                  GPIOB
#define EVAL_COM1_TX_GPIO_CLK_ENABLE()          __HAL_RCC_GPIOB_CLK_ENABLE()
#define EVAL_COM1_TX_GPIO_CLK_DISABLE()         __HAL_RCC_GPIOB_CLK_DISABLE()
#define EVAL_COM1_TX_AF                         GPIO_AF7_USART1

#define EVAL_COM1_RX_PIN                        GPIO_PIN_7
#define EVAL_COM1_RX_GPIO_PORT                  GPIOB
#define EVAL_COM1_RX_GPIO_CLK_ENABLE()          __HAL_RCC_GPIOB_CLK_ENABLE()
#define EVAL_COM1_RX_GPIO_CLK_DISABLE()         __HAL_RCC_GPIOB_CLK_DISABLE()
#define EVAL_COM1_RX_AF                         GPIO_AF7_USART1

#define EVAL_COM1_IRQn                          USART1_IRQn

#define COMx_CLK_ENABLE(__INDEX__)              do { if((__INDEX__) == COM1) { EVAL_COM1_CLK_ENABLE(); } } while(0)
#define COMx_CLK_DISABLE(__INDEX__)             do { if((__INDEX__) == COM1) { EVAL_COM1_CLK_DISABLE(); } } while(0)

#define COMx_TX_GPIO_CLK_ENABLE(__INDEX__)      do { if((__INDEX__) == COM1) { EVAL_COM1_TX_GPIO_CLK_ENABLE(); } } while(0)
#define COMx_TX_GPIO_CLK_DISABLE(__INDEX__)     do { if((__INDEX__) == COM1) { EVAL_COM1_TX_GPIO_CLK_DISABLE(); } } while(0)

#define COMx_RX_GPIO_CLK_ENABLE(__INDEX__)      do { if((__INDEX__) == COM1) { EVAL_COM1_RX_GPIO_CLK_ENABLE(); } } while(0)
#define COMx_RX_GPIO_CLK_DISABLE(__INDEX__)     do { if((__INDEX__) == COM1) { EVAL_COM1_RX_GPIO_CLK_DISABLE(); } } while(0)
/**
  * @}
  */

/** @defgroup STM32L476G_EVAL_BUS  BUS Constants
  * @{
  */
/* Exported constant IO ------------------------------------------------------*/
/*##################### I2Cx ###################################*/
/* User can use this section to tailor I2Cx instance used and associated
   resources */
/* Definition for I2Cx Pins */
#define EVAL_I2Cx_SCL_PIN                       GPIO_PIN_14
#define EVAL_I2Cx_SCL_GPIO_PORT                 GPIOG
#define EVAL_I2Cx_SDA_PIN                       GPIO_PIN_13
#define EVAL_I2Cx_SDA_GPIO_PORT                 GPIOG
#define EVAL_I2Cx_SCL_SDA_AF                    GPIO_AF4_I2C1

/* Definition for I2Cx clock resources */
#define EVAL_I2Cx                               I2C1
#define EVAL_I2Cx_CLK_ENABLE()                  __HAL_RCC_I2C1_CLK_ENABLE()
#define EVAL_I2Cx_SDA_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOG_CLK_ENABLE()
#define EVAL_I2Cx_SCL_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOG_CLK_ENABLE()

#define EVAL_I2Cx_FORCE_RESET()                 __HAL_RCC_I2C1_FORCE_RESET()
#define EVAL_I2Cx_RELEASE_RESET()               __HAL_RCC_I2C1_RELEASE_RESET()

/* Definition for I2Cx's NVIC */
#define EVAL_I2Cx_EV_IRQn                       I2C1_EV_IRQn
#define EVAL_I2Cx_EV_IRQHandler                 I2C1_EV_IRQHandler
#define EVAL_I2Cx_ER_IRQn                       I2C1_ER_IRQn
#define EVAL_I2Cx_ER_IRQHandler                 I2C1_ER_IRQHandler

/* I2C TIMING Register define when I2C clock source is SYSCLK */
/* I2C TIMING is calculated in case of the I2C Clock source is the SYSCLK = 80 MHz */
/* Set 0x90112626 value to reach 100 KHz speed (Rise time = 640ns, Fall time = 20ns) */
#ifndef EVAL_I2Cx_TIMING
 #define EVAL_I2Cx_TIMING                       0x90112626
#endif /* EVAL_I2Cx_TIMING */


/* Exported constantIO -------------------------------------------------------*/
/* I2C clock speed configuration (in Hz)
   WARNING:
   Make sure that this define is not already declared in other files (ie.
   stm324xg_eval.h file). It can be used in parallel by other modules. */
#ifndef BSP_I2C_SPEED
 #define BSP_I2C_SPEED                            100000
#endif /* BSP_I2C_SPEED */


#define IO1_I2C_ADDRESS                       0x82
#define IO2_I2C_ADDRESS                       0x84
#define AUDIO_I2C_ADDRESS                     0x34

/* Maximum Timeout values for flags waiting loops. These timeouts are not based
   on accurate values, they just guarantee that the application will not remain
   stuck if the I2C communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...). */
#define EVAL_I2Cx_TIMEOUT_MAX                   3000

/**
  * @}
  */

/**
  * @}
  */

/** @defgroup STM32L476G_EVAL_Exported_Functions Exported Functions
  * @{
  */
uint32_t                BSP_GetVersion(void);
void                    BSP_LED_Init(Led_TypeDef Led);
void                    BSP_LED_On(Led_TypeDef Led);
void                    BSP_LED_Off(Led_TypeDef Led);
void                    BSP_LED_Toggle(Led_TypeDef Led);
void                    BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef Button_Mode);
uint32_t                BSP_PB_GetState(Button_TypeDef Button);
#ifdef HAL_UART_MODULE_ENABLED
void                    BSP_COM_Init(COM_TypeDef COM, UART_HandleTypeDef* huart);
#endif /* HAL_UART_MODULE_ENABLED */
#if defined(USE_IOEXPANDER)
uint8_t                 BSP_JOY_Init(JOYMode_TypeDef Joy_Mode);
JOYState_TypeDef        BSP_JOY_GetState(void);
#endif /* USE_IOEXPANDER */

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

#endif /* __STM32L476G_EVAL_H */


