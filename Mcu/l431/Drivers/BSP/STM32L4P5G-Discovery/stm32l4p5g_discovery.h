/**
  ******************************************************************************
  * @file    stm32l4p5g_discovery.h
  * @author  MCD Application Team
  * @brief   This file contains definitions for STM32L4P5G_DISCOVERY's LEDs,
  *          push-buttons hardware resources (MB1311).
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
#ifndef __STM32L4P5G_DISCOVERY_H
#define __STM32L4P5G_DISCOVERY_H

#ifdef __cplusplus
 extern "C" {
#endif

/**
  * @brief  Define for STM32L4P5G_DISCOVERY board
  */



/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#if defined(BSP_USE_CMSIS_OS)
#include "cmsis_os.h"
#endif

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L4P5G_DISCOVERY
  * @{
  */

/** @addtogroup STM32L4P5G_DISCOVERY_Common
  * @{
  */

/** @defgroup STM32L4P5G_DISCOVERY_Exported_Types Exported Types
  * @{
  */

/**
 * @brief LED Types Definition
 */
typedef enum
{
  LED1 = 0,
  LED2 = 1,
  LED_GREEN = LED1,
  LED_BLUE  = LED2
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

/** @defgroup STM32L4P5G_DISCOVERY_Exported_Constants Exported Constants
  * @{
  */

/** @defgroup STM32L4P5G_DISCOVERY_LED LED Constants
  * @{
  */
/* LED number */
#define LEDn                              2

#define LED1_PIN                          GPIO_PIN_13
#define LED1_GPIO_PORT                    GPIOG
#define LED1_GPIO_CLK_ENABLE()            do { __HAL_RCC_PWR_CLK_ENABLE(); HAL_PWREx_EnableVddIO2(); __HAL_RCC_GPIOG_CLK_ENABLE(); } while(0)
#define LED1_GPIO_CLK_DISABLE()           __HAL_RCC_GPIOG_CLK_DISABLE()


#define LED2_PIN                          GPIO_PIN_14
#define LED2_GPIO_PORT                    GPIOG
#define LED2_GPIO_CLK_ENABLE()            do { __HAL_RCC_PWR_CLK_ENABLE(); HAL_PWREx_EnableVddIO2(); __HAL_RCC_GPIOG_CLK_ENABLE(); } while(0)
#define LED2_GPIO_CLK_DISABLE()           __HAL_RCC_GPIOG_CLK_DISABLE()


#define LEDx_GPIO_CLK_ENABLE(__LED__)     do { if((__LED__) == LED1) { LED1_GPIO_CLK_ENABLE(); } else \
                                               if((__LED__) == LED2) { LED2_GPIO_CLK_ENABLE(); } } while(0)

#define LEDx_GPIO_CLK_DISABLE(__LED__)    do { if((__LED__) == LED1) { LED1_GPIO_CLK_DISABLE(); } else \
                                               if((__LED__) == LED2) { LED2_GPIO_CLK_DISABLE(); } } while(0)

/**
  * @}
  */

/** @defgroup STM32L4P5G_DISCOVERY_BUTTON  BUTTON Constants
  * @{
  */
#define JOYn                              5

/**
* @brief Joystick Right push-button
*/
#define RIGHT_JOY_PIN                      GPIO_PIN_10   /* PI.10 */
#define RIGHT_JOY_GPIO_PORT                GPIOI
#define RIGHT_JOY_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOI_CLK_ENABLE()
#define RIGHT_JOY_GPIO_CLK_DISABLE()       __HAL_RCC_GPIOI_CLK_DISABLE()
#define RIGHT_JOY_EXTI_IRQn                EXTI15_10_IRQn

/**
* @brief Joystick Left push-button
*/
#define LEFT_JOY_PIN                       GPIO_PIN_11   /* PI.11 */
#define LEFT_JOY_GPIO_PORT                 GPIOI
#define LEFT_JOY_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOI_CLK_ENABLE()
#define LEFT_JOY_GPIO_CLK_DISABLE()        __HAL_RCC_GPIOI_CLK_DISABLE()
#define LEFT_JOY_EXTI_IRQn                 EXTI15_10_IRQn

/**
* @brief Joystick Up push-button
*/
#define UP_JOY_PIN                         GPIO_PIN_8   /* PI.8 */
#define UP_JOY_GPIO_PORT                   GPIOI
#define UP_JOY_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOI_CLK_ENABLE()
#define UP_JOY_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOI_CLK_DISABLE()
#define UP_JOY_EXTI_IRQn                   EXTI9_5_IRQn

/**
 * @brief Joystick Down push-button
 */
#define DOWN_JOY_PIN                       GPIO_PIN_9   /* PI.9 */
#define DOWN_JOY_GPIO_PORT                 GPIOI
#define DOWN_JOY_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOI_CLK_ENABLE()
#define DOWN_JOY_GPIO_CLK_DISABLE()        __HAL_RCC_GPIOI_CLK_DISABLE()
#define DOWN_JOY_EXTI_IRQn                 EXTI9_5_IRQn

/**
 * @brief Joystick Sel push-button
 */
#define SEL_JOY_PIN                        GPIO_PIN_13   /* PC.13 */
#define SEL_JOY_GPIO_PORT                  GPIOC
#define SEL_JOY_GPIO_CLK_ENABLE()          __HAL_RCC_GPIOC_CLK_ENABLE()
#define SEL_JOY_GPIO_CLK_DISABLE()         __HAL_RCC_GPIOC_CLK_DISABLE()
#define SEL_JOY_EXTI_IRQn                  EXTI15_10_IRQn

#define JOYx_GPIO_CLK_ENABLE(__JOY__)     do { if((__JOY__) == JOY_SEL)   { SEL_JOY_GPIO_CLK_ENABLE();   } else \
                                               if((__JOY__) == JOY_DOWN)  { DOWN_JOY_GPIO_CLK_ENABLE();  } else \
                                               if((__JOY__) == JOY_LEFT)  { LEFT_JOY_GPIO_CLK_ENABLE();  } else \
                                               if((__JOY__) == JOY_RIGHT) { RIGHT_JOY_GPIO_CLK_ENABLE(); } else \
                                               if((__JOY__) == JOY_UP)    { UP_JOY_GPIO_CLK_ENABLE(); }  } while(0)

#define JOYx_GPIO_CLK_DISABLE(__JOY__)    do { if((__JOY__) == JOY_SEL)   { SEL_JOY_GPIO_CLK_DISABLE();   } else \
                                               if((__JOY__) == JOY_DOWN)  { DOWN_JOY_GPIO_CLK_DISABLE();  } else \
                                               if((__JOY__) == JOY_LEFT)  { LEFT_JOY_GPIO_CLK_DISABLE();  } else \
                                               if((__JOY__) == JOY_RIGHT) { RIGHT_JOY_GPIO_CLK_DISABLE(); } else \
                                               if((__JOY__) == JOY_UP)    { UP_JOY_GPIO_CLK_DISABLE(); }  } while(0)

#define JOY_ALL_PINS                      (RIGHT_JOY_PIN | LEFT_JOY_PIN | UP_JOY_PIN | DOWN_JOY_PIN | SEL_JOY_PIN)

/**
  * @}
  */

/** @defgroup STM32L4P5G_DISCOVERY_MFX_PIN  MFX Pins Constants
  * @{
  */

/**
  * @brief Pins definition connected to MFX
  */

#if defined(USE_IOEXPANDER)
#define OTG_FS_POWER_SWITCH_PIN        IO_PIN_0
#define OTG_FS_OVER_CURRENT_PIN        IO_PIN_1
#endif /* USE_IOEXPANDER */

/**
  * @}
  */

/** @defgroup STM32L4P5G_DISCOVERY_COM COM Constants
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

/** @defgroup STM32L4P5G_DISCOVERY_BUS  BUS Constants
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
#define TS_I2C_ADDRESS                   ((uint16_t) 0x10)


/* I2C users */
#define BSP_I2C_NO_USER     0x00000000U
#define BSP_I2C_TS_USER     0x00000002U
#define BSP_I2C_MFX_USER    0x00000004U
#define BSP_I2C_ALL_USERS   0x0000000FU


#endif /* HAL_I2C_MODULE_ENABLED */


/*##################### MFX ##########################*/
/**
  * @brief  MFX interface pins
  */
#define MFX_INT_GPIO_PORT                 GPIOA
#define MFX_INT_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOA_CLK_ENABLE()
#define MFX_INT_GPIO_CLK_DISABLE()        __HAL_RCC_GPIOA_CLK_DISABLE()
#define MFX_INT_PIN                       GPIO_PIN_0                  /* PA.00 */
#define MFX_INT_EXTI_IRQn                 EXTI0_IRQn

#define MFX_WAKEUP_GPIO_PORT              GPIOF
#define MFX_WAKEUP_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOF_CLK_ENABLE()
#define MFX_WAKEUP_GPIO_CLK_DISABLE()     __HAL_RCC_GPIOF_CLK_DISABLE()
#define MFX_WAKEUP_PIN                    GPIO_PIN_5                  /* PF.05 */


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
  * @}
  */

/** @defgroup STM32L4P5G_DISCOVERY_SMPS SMPS
  * @{
  */
#define SMPS_OK     0
#define SMPS_KO     1


#define PORT_SMPS               GPIOH
#define PIN_SMPS_ENABLE         GPIO_PIN_6   /* PH.06 */
#define PIN_SMPS_POWERGOOD      GPIO_PIN_7   /* PH.07 */
#define PIN_SMPS_SWITCH_ENABLE  GPIO_PIN_15  /* PH.15 */

#define PIN_SMPS_V1             GPIO_PIN_2   /* PH.02 */
#define PIN_SMPS_V2             GPIO_PIN_4   /* PH.04 */
#define PIN_SMPS_V3             GPIO_PIN_13  /* PH.13 */

#define SMPS_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOH_CLK_ENABLE()
#define SMPS_GPIO_CLK_DISABLE() __HAL_RCC_GPIOH_CLK_DISABLE()

#define PWR_AND_CLK_SMPS()   do { __HAL_RCC_PWR_CLK_ENABLE(); \
                                  SMPS_GPIO_CLK_ENABLE(); } while(0)
/**
  * @}
  */


/**
  * @}
  */

/** @defgroup STM32L4P5G_DISCOVERY_Exported_Functions Exported Functions
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

void                    I2C1_DeInit(uint32_t user);

/**
  * @}
  */

/** @addtogroup STM32L4XX_NUCLEO_SMPS_Functions
  * @{
  */
uint32_t         BSP_SMPS_Init(void);
uint32_t         BSP_SMPS_DeInit(void);
uint32_t         BSP_SMPS_Enable(uint32_t Delay, uint32_t Power_Good_Check);
uint32_t         BSP_SMPS_Disable(void);
uint32_t         BSP_SMPS_Supply_Enable(uint32_t Delay, uint32_t Power_Good_Check);
uint32_t         BSP_SMPS_Supply_Disable(void);
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

#endif /* __STM32L4P5G_DISCOVERY_H */


