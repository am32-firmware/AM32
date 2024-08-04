/**
  ******************************************************************************
  * @file    stm32l476g_discovery.h
  * @author  MCD Application Team
  * @brief   This file contains definitions for STM32L476G_DISCOVERY's LEDs,
  *          push-buttons hardware resources (MB1184).
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
#ifndef __STM32L476G_DISCOVERY_H
#define __STM32L476G_DISCOVERY_H

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  Define for STM32L476G_DISCOVERY board
  */
#if !defined (USE_STM32L476G_DISCO_REVC) && !defined (USE_STM32L476G_DISCO_REVB) && !defined (USE_STM32L476G_DISCO_REVA)
#define USE_STM32L476G_DISCO_REVC
#endif


/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L476G_DISCOVERY
  * @{
  */

/** @addtogroup STM32L476G_DISCOVERY_Common
  * @{
  */

/** @defgroup STM32L476G_DISCOVERY_Exported_Types Exported Types
  * @{
  */

/**
 * @brief LED Types Definition
 */
#if defined (USE_STM32L476G_DISCO_REVC) || defined (USE_STM32L476G_DISCO_REVB)
typedef enum
{
  LED4 = 0,
  LED5 = 1,
  LED_RED    = LED4,
  LED_GREEN  = LED5
}
Led_TypeDef;
#elif defined (USE_STM32L476G_DISCO_REVA)
typedef enum
{
  LED3 = 0,
  LED4 = 1,
  LED_RED    = LED3,
  LED_GREEN  = LED4
} Led_TypeDef;
#endif

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
} JOYState_TypeDef;

typedef enum
{
  JOY_MODE_GPIO = 0,
  JOY_MODE_EXTI = 1
} JOYMode_TypeDef;

typedef enum
{
  SUPPLY_MODE_ERROR = 0,
  SUPPLY_MODE_EXTERNAL = 1,
  SUPPLY_MODE_BATTERY = 2
} SupplyMode_TypeDef;

/**
  * @}
  */

/** @defgroup STM32L476G_DISCOVERY_Exported_Constants Exported Constants
  * @{
  */

/** @defgroup STM32L476G_DISCOVERY_BATTERY BATTERY Detection Constants
  * @{
  */
#define BATTERY_DETECTION_PIN                 GPIO_PIN_3
#define BATTERY_DETECTION_GPIO_PORT           GPIOB
#define BATTERY_DETECTION_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOB_CLK_ENABLE()
#define BATTERY_DETECTION_GPIO_CLK_DISABLE()  __HAL_RCC_GPIOB_CLK_DISABLE()
/**
  * @}
  */

/** @defgroup STM32L476G_DISCOVERY_LED LED Constants
  * @{
  */
#define LEDn                              2

#if defined (USE_STM32L476G_DISCO_REVC) || defined (USE_STM32L476G_DISCO_REVB)
#define LED4_PIN                          GPIO_PIN_2
#define LED4_GPIO_PORT                    GPIOB
#define LED4_GPIO_CLK_ENABLE()            __HAL_RCC_GPIOB_CLK_ENABLE()
#define LED4_GPIO_CLK_DISABLE()           __HAL_RCC_GPIOB_CLK_DISABLE()

#define LED5_PIN                          GPIO_PIN_8
#define LED5_GPIO_PORT                    GPIOE
#define LED5_GPIO_CLK_ENABLE()            __HAL_RCC_GPIOE_CLK_ENABLE()
#define LED5_GPIO_CLK_DISABLE()           __HAL_RCC_GPIOE_CLK_DISABLE()

#define LEDx_GPIO_CLK_ENABLE(__LED__)     do { if((__LED__) == LED4) { LED4_GPIO_CLK_ENABLE(); } else \
                                               if((__LED__) == LED5) { LED5_GPIO_CLK_ENABLE(); } } while(0)

#define LEDx_GPIO_CLK_DISABLE(__LED__)    do { if((__LED__) == LED4) { LED4_GPIO_CLK_DISABLE(); } else \
                                               if((__LED__) == LED5) { LED5_GPIO_CLK_DISABLE(); } } while(0)

#elif defined (USE_STM32L476G_DISCO_REVA)
#define LED3_PIN                          GPIO_PIN_2
#define LED3_GPIO_PORT                    GPIOB
#define LED3_GPIO_CLK_ENABLE()            __HAL_RCC_GPIOB_CLK_ENABLE()
#define LED3_GPIO_CLK_DISABLE()           __HAL_RCC_GPIOB_CLK_DISABLE()

#define LED4_PIN                          GPIO_PIN_8
#define LED4_GPIO_PORT                    GPIOE
#define LED4_GPIO_CLK_ENABLE()            __HAL_RCC_GPIOE_CLK_ENABLE()
#define LED4_GPIO_CLK_DISABLE()           __HAL_RCC_GPIOE_CLK_DISABLE()

#define LEDx_GPIO_CLK_ENABLE(__LED__)     do { if((__LED__) == LED3) { LED3_GPIO_CLK_ENABLE(); } else \
                                               if((__LED__) == LED4) { LED4_GPIO_CLK_ENABLE(); } } while(0)

#define LEDx_GPIO_CLK_DISABLE(__LED__)    do { if((__LED__) == LED3) { LED3_GPIO_CLK_DISABLE(); } else \
                                               if((__LED__) == LED4) { LED4_GPIO_CLK_DISABLE(); } } while(0)

#endif
/**
  * @}
  */

/** @defgroup STM32L476G_DISCOVERY_BUTTON  BUTTON Constants
  * @{
  */
#define JOYn                              5

/**
* @brief Joystick Right push-button
*/
#define RIGHT_JOY_PIN                     GPIO_PIN_2  /* PA.02 */
#define RIGHT_JOY_GPIO_PORT               GPIOA
#define RIGHT_JOY_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOA_CLK_ENABLE()
#define RIGHT_JOY_GPIO_CLK_DISABLE()      __HAL_RCC_GPIOA_CLK_DISABLE()
#define RIGHT_JOY_EXTI_IRQn               EXTI2_IRQn

/**
* @brief Joystick Left push-button
*/
#define LEFT_JOY_PIN                      GPIO_PIN_1  /* PA.01 */
#define LEFT_JOY_GPIO_PORT                GPIOA
#define LEFT_JOY_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOA_CLK_ENABLE()
#define LEFT_JOY_GPIO_CLK_DISABLE()       __HAL_RCC_GPIOA_CLK_DISABLE()
#define LEFT_JOY_EXTI_IRQn                EXTI1_IRQn

/**
* @brief Joystick Up push-button
*/
#define UP_JOY_PIN                        GPIO_PIN_3  /* PA.03 */
#define UP_JOY_GPIO_PORT                  GPIOA
#define UP_JOY_GPIO_CLK_ENABLE()          __HAL_RCC_GPIOA_CLK_ENABLE()
#define UP_JOY_GPIO_CLK_DISABLE()         __HAL_RCC_GPIOA_CLK_DISABLE()
#define UP_JOY_EXTI_IRQn                  EXTI3_IRQn

/**
 * @brief Joystick Down push-button
 */
#define DOWN_JOY_PIN                      GPIO_PIN_5   /* PA.05 */
#define DOWN_JOY_GPIO_PORT                GPIOA
#define DOWN_JOY_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOA_CLK_ENABLE()
#define DOWN_JOY_GPIO_CLK_DISABLE()       __HAL_RCC_GPIOA_CLK_DISABLE()
#define DOWN_JOY_EXTI_IRQn                EXTI9_5_IRQn

/**
 * @brief Joystick Sel push-button
 */
#define SEL_JOY_PIN                       GPIO_PIN_0   /* PA.00 */
#define SEL_JOY_GPIO_PORT                 GPIOA
#define SEL_JOY_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOA_CLK_ENABLE()
#define SEL_JOY_GPIO_CLK_DISABLE()        __HAL_RCC_GPIOA_CLK_DISABLE()
#define SEL_JOY_EXTI_IRQn                 EXTI0_IRQn

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

/** @defgroup STM32L476G_DISCOVERY_BUS  BUS Constants
  * @{
  */
#if defined(HAL_SPI_MODULE_ENABLED)
/*##################### SPI2 ###################################*/
#define DISCOVERY_SPIx                          SPI2
#define DISCOVERY_SPIx_CLOCK_ENABLE()           __HAL_RCC_SPI2_CLK_ENABLE()
#define DISCOVERY_SPIx_CLOCK_DISABLE()          __HAL_RCC_SPI2_CLK_DISABLE()
#define DISCOVERY_SPIx_GPIO_PORT                GPIOD                      /* GPIOD */
#define DISCOVERY_SPIx_AF                       GPIO_AF5_SPI2
#define DISCOVERY_SPIx_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOD_CLK_ENABLE()
#define DISCOVERY_SPIx_GPIO_CLK_DISABLE()       __HAL_RCC_GPIOD_CLK_DISABLE()
#define DISCOVERY_SPIx_GPIO_FORCE_RESET()       __HAL_RCC_SPI2_FORCE_RESET()
#define DISCOVERY_SPIx_GPIO_RELEASE_RESET()     __HAL_RCC_SPI2_RELEASE_RESET()
#define DISCOVERY_SPIx_SCK_PIN                  GPIO_PIN_1                 /* PD.01*/
#define DISCOVERY_SPIx_MISO_PIN                 GPIO_PIN_3                 /* PD.03 */
#define DISCOVERY_SPIx_MOSI_PIN                 GPIO_PIN_4                 /* PD.04 */

/* Maximum Timeout values for flags waiting loops. These timeouts are not based
   on accurate values, they just guarantee that the application will not remain
   stuck if the SPI communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...). */
#define SPIx_TIMEOUT_MAX                        ((uint32_t)0x1000)
/* Read/Write command */
#define READWRITE_CMD                           ((uint8_t)0x80)
/* Multiple byte read/write command */
#define MULTIPLEBYTE_CMD                        ((uint8_t)0x40)
/* Dummy Byte Send by the SPI Master device in order to generate the Clock to the Slave device */
#define DUMMY_BYTE                              ((uint8_t)0x00)

#endif /* HAL_SPI_MODULE_ENABLED */

#if defined(HAL_I2C_MODULE_ENABLED)
/*##################### I2C1 ###################################*/
/* User can use this section to tailor I2C1 instance used and associated
   resources */
/* Definition for I2C1 Pins */
#define DISCOVERY_I2C1_SCL_GPIO_PORT            GPIOB
#define DISCOVERY_I2C1_SDA_GPIO_PORT            GPIOB
#if defined (USE_STM32L476G_DISCO_REVC) || defined (USE_STM32L476G_DISCO_REVB)
#define DISCOVERY_I2C1_SCL_PIN                  GPIO_PIN_6
#define DISCOVERY_I2C1_SDA_PIN                  GPIO_PIN_7
#elif defined (USE_STM32L476G_DISCO_REVA)
#define DISCOVERY_I2C1_SCL_PIN                  GPIO_PIN_8
#define DISCOVERY_I2C1_SDA_PIN                  GPIO_PIN_9
#endif
#define DISCOVERY_I2C1_SCL_SDA_AF               GPIO_AF4_I2C1

/* Definition for I2C1 clock resources */
#define DISCOVERY_I2C1                          I2C1
#define DISCOVERY_I2C1_CLK_ENABLE()             __HAL_RCC_I2C1_CLK_ENABLE()
#define DISCOVERY_I2C1_CLK_DISABLE()            __HAL_RCC_I2C1_CLK_DISABLE()
#define DISCOVERY_I2C1_SDA_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOB_CLK_ENABLE()
#define DISCOVERY_I2C1_SCL_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOB_CLK_ENABLE()
#define DISCOVERY_I2C1_SDA_GPIO_CLK_DISABLE()   __HAL_RCC_GPIOB_CLK_DISABLE()
#define DISCOVERY_I2C1_SCL_GPIO_CLK_DISABLE()   __HAL_RCC_GPIOB_CLK_DISABLE()
#define DISCOVERY_I2C1_FORCE_RESET()            __HAL_RCC_I2C1_FORCE_RESET()
#define DISCOVERY_I2C1_RELEASE_RESET()          __HAL_RCC_I2C1_RELEASE_RESET()

/* Definition for I2C1's NVIC */
#define DISCOVERY_I2C1_EV_IRQn                  I2C1_EV_IRQn
#define DISCOVERY_I2C1_EV_IRQHandler            I2C1_EV_IRQHandler
#define DISCOVERY_I2C1_ER_IRQn                  I2C1_ER_IRQn
#define DISCOVERY_I2C1_ER_IRQHandler            I2C1_ER_IRQHandler

/* I2C TIMING Register define when I2C clock source is SYSCLK */
/* I2C TIMING is calculated in case of the I2C Clock source is the SYSCLK = 80 MHz */
/* Set 0x90112626 value to reach 100 KHz speed (Rise time = 640ns, Fall time = 20ns) */
#ifndef DISCOVERY_I2C1_TIMING
#define DISCOVERY_I2C1_TIMING                  0x90112626
#endif /* DISCOVERY_I2C1_TIMING */

/* I2C clock speed configuration (in Hz)
   WARNING:
   Make sure that this define is not already declared in other files (ie.
   stm324xg_discovery.h file). It can be used in parallel by other modules. */
#ifndef BSP_I2C_SPEED
#define BSP_I2C_SPEED                              100000
#endif /* BSP_I2C_SPEED */


/* Audio codec I2C address */
#define AUDIO_I2C_ADDRESS                       ((uint16_t) 0x94)

/* Maximum Timeout values for flags waiting loops. These timeouts are not based
   on accurate values, they just guarantee that the application will not remain
   stuck if the I2C communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...). */
#define DISCOVERY_I2C1_TIMEOUT_MAX              3000


/*##################### I2C2 ###################################*/
/* User can use this section to tailor I2C2 instance used and associated
   resources */
/* Definition for I2C2 Pins */
#define DISCOVERY_I2C2_SCL_PIN                  GPIO_PIN_10
#define DISCOVERY_I2C2_SCL_GPIO_PORT            GPIOB
#define DISCOVERY_I2C2_SDA_PIN                  GPIO_PIN_11
#define DISCOVERY_I2C2_SDA_GPIO_PORT            GPIOB
#define DISCOVERY_I2C2_SCL_SDA_AF               GPIO_AF4_I2C2
/* Definition for I2C2 clock resources */
#define DISCOVERY_I2C2                          I2C2
#define DISCOVERY_I2C2_CLK_ENABLE()             __HAL_RCC_I2C2_CLK_ENABLE()
#define DISCOVERY_I2C2_CLK_DISABLE()            __HAL_RCC_I2C2_CLK_DISABLE()
#define DISCOVERY_I2C2_SDA_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOB_CLK_ENABLE()
#define DISCOVERY_I2C2_SCL_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOB_CLK_ENABLE()
#define DISCOVERY_I2C2_SDA_GPIO_CLK_DISABLE()   __HAL_RCC_GPIOB_CLK_DISABLE()
#define DISCOVERY_I2C2_SCL_GPIO_CLK_DISABLE()   __HAL_RCC_GPIOB_CLK_DISABLE()
#define DISCOVERY_I2C2_FORCE_RESET()            __HAL_RCC_I2C2_FORCE_RESET()
#define DISCOVERY_I2C2_RELEASE_RESET()          __HAL_RCC_I2C2_RELEASE_RESET()

/* Definition for I2C2's NVIC */
#define DISCOVERY_I2C2_EV_IRQn                  I2C2_EV_IRQn
#define DISCOVERY_I2C2_ER_IRQn                  I2C2_ER_IRQn

/* I2C TIMING Register define when I2C clock source is SYSCLK */
/* I2C TIMING is calculated in case of the I2C Clock source is the SYSCLK = 80 MHz */
/* Set 0x90112626 value to reach 100 KHz speed (Rise time = 640ns, Fall time = 20ns) */
#ifndef DISCOVERY_I2C2_TIMING
#define DISCOVERY_I2C2_TIMING                  0x90112626
#endif /* DISCOVERY_I2C2_TIMING */

/* I2C clock speed configuration (in Hz)
   WARNING:
   Make sure that this define is not already declared in other files (ie.
   stm324xg_discovery.h file). It can be used in parallel by other modules. */
#ifndef BSP_I2C_SPEED
#define BSP_I2C_SPEED                              100000
#endif /* BSP_I2C_SPEED */

#define IDD_I2C_ADDRESS                         ((uint16_t) 0x84)

/* Maximum Timeout values for flags waiting loops. These timeouts are not based
   on accurate values, they just guarantee that the application will not remain
   stuck if the I2C communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...). */
#define DISCOVERY_I2C2_TIMEOUT_MAX              3000
#endif /* HAL_I2C_MODULE_ENABLED */

/*##################### Accelerometer ##########################*/
/**
  * @brief  Accelerometer Chip Select macro definition
  */
#define ACCELERO_CS_LOW()                       HAL_GPIO_WritePin(ACCELERO_CS_GPIO_PORT, ACCELERO_CS_PIN, GPIO_PIN_RESET)
#define ACCELERO_CS_HIGH()                      HAL_GPIO_WritePin(ACCELERO_CS_GPIO_PORT, ACCELERO_CS_PIN, GPIO_PIN_SET)

/**
  * @brief  Accelerometer SPI Interface pins
  */
#define ACCELERO_CS_GPIO_PORT                   GPIOE                       /* GPIOE */
#define ACCELERO_CS_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOE_CLK_ENABLE()
#define ACCELERO_CS_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOE_CLK_DISABLE()
#define ACCELERO_CS_PIN                         GPIO_PIN_0                  /* PE.00 */

/**
  * @brief  Accelerometer Interrupt pins
  */
#define ACCELERO_XLINT_GPIO_PORT                  GPIOE                       /* GPIOE */
#define ACCELERO_XLINT_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOE_CLK_ENABLE()
#define ACCELERO_XLINT_GPIO_CLK_DISABLE()        __HAL_RCC_GPIOE_CLK_DISABLE()
#define ACCELERO_XLINT_PIN                       GPIO_PIN_1                  /* PE.01 */
#define ACCELERO_XLINT_EXTI_IRQn                 EXTI1_IRQn

/*##################### Magnetometer ##########################*/
/**
  * @brief  Magnetometer Chip Select macro definition
  */
#define MAGNETO_CS_LOW()                        HAL_GPIO_WritePin(MAGNETO_CS_GPIO_PORT, MAGNETO_CS_PIN, GPIO_PIN_RESET)
#define MAGNETO_CS_HIGH()                       HAL_GPIO_WritePin(MAGNETO_CS_GPIO_PORT, MAGNETO_CS_PIN, GPIO_PIN_SET)

/**
  * @brief  Magnetometer SPI Interface pins
  */
#define MAGNETO_CS_GPIO_PORT                    GPIOC                       /* GPIOC */
#define MAGNETO_CS_GPIO_CLK_ENABLE()            __HAL_RCC_GPIOC_CLK_ENABLE()
#define MAGNETO_CS_GPIO_CLK_DISABLE()           __HAL_RCC_GPIOC_CLK_DISABLE()
#define MAGNETO_CS_PIN                          GPIO_PIN_0                  /* PC.00 */


/**
  * @brief  Magnetometer Interrupt pins
  */
#define MAGNETO_INT_GPIO_PORT                   GPIOC                       /* GPIOC */
#define MAGNETO_INT_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOC_CLK_ENABLE()
#define MAGNETO_INT_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOC_CLK_DISABLE()
#define MAGNETO_INT1_PIN                        GPIO_PIN_1                  /* PC.01 */
#define MAGNETO_INT1_EXTI_IRQn                  EXTI1_IRQn

#define MAGNETO_DRDY_GPIO_PORT                   GPIOC                       /* GPIOC */
#define MAGNETO_DRDY_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOC_CLK_ENABLE()
#define MAGNETO_DRDY_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOC_CLK_DISABLE()
#define MAGNETO_DRDY_PIN                        GPIO_PIN_2                  /* PC.01 */


/*##################### Audio Codec ##########################*/
/**
  * @brief  Audio codec chip reset definition
  */
/* Audio codec power on/off macro definition */
#define CODEC_AUDIO_POWER_OFF()      HAL_GPIO_WritePin(AUDIO_RESET_GPIO, AUDIO_RESET_PIN, GPIO_PIN_RESET)
#define CODEC_AUDIO_POWER_ON()       HAL_GPIO_WritePin(AUDIO_RESET_GPIO, AUDIO_RESET_PIN, GPIO_PIN_SET)

/* Audio Reset Pin definition */
#define AUDIO_RESET_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOE_CLK_ENABLE()
#define AUDIO_RESET_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOE_CLK_DISABLE()
#define AUDIO_RESET_PIN                         GPIO_PIN_3
#define AUDIO_RESET_GPIO                        GPIOE

/*##################### Gyroscope ##########################*/
/**
  * @brief  Gyroscope Chip Select macro definition
  */
#define GYRO_CS_LOW()                           HAL_GPIO_WritePin(GYRO_CS_GPIO_PORT, GYRO_CS_PIN, GPIO_PIN_RESET)
#define GYRO_CS_HIGH()                          HAL_GPIO_WritePin(GYRO_CS_GPIO_PORT, GYRO_CS_PIN, GPIO_PIN_SET)

/**
  * @brief  Gyroscope SPI Interface pins
  */
#define GYRO_CS_GPIO_PORT                       GPIOD                       /* GPIOD */
#define GYRO_CS_GPIO_CLK_ENABLE()               __HAL_RCC_GPIOD_CLK_ENABLE()
#define GYRO_CS_GPIO_CLK_DISABLE()              __HAL_RCC_GPIOD_CLK_DISABLE()
#define GYRO_CS_PIN                             GPIO_PIN_7                  /* PD.07 */

/**
  * @brief  Gyroscope Interrupt pins
  */
#if defined (USE_STM32L476G_DISCO_REVC) || defined (USE_STM32L476G_DISCO_REVB)
#define GYRO_INT1_GPIO_PORT                     GPIOD                       /* GPIOD */
#define GYRO_INT1_GPIO_CLK_ENABLE()             __HAL_RCC_GPIOD_CLK_ENABLE()
#define GYRO_INT1_GPIO_CLK_DISABLE()            __HAL_RCC_GPIOD_CLK_DISABLE()
#define GYRO_INT1_PIN                           GPIO_PIN_2                  /* PD.02 */
#define GYRO_INT1_EXTI_IRQn                     EXTI2_IRQn
#define GYRO_INT2_GPIO_PORT                     GPIOB                       /* GPIOB */
#define GYRO_INT2_GPIO_CLK_ENABLE()             __HAL_RCC_GPIOB_CLK_ENABLE()
#define GYRO_INT2_GPIO_CLK_DISABLE()            __HAL_RCC_GPIOB_CLK_DISABLE()
#define GYRO_INT2_PIN                           GPIO_PIN_8                  /* PB.08 */
#define GYRO_INT2_EXTI_IRQn                     EXTI9_5_IRQn
#elif defined (USE_STM32L476G_DISCO_REVA)
#define GYRO_INT1_GPIO_PORT                     GPIOB                       /* GPIOB */
#define GYRO_INT1_GPIO_CLK_ENABLE()             __HAL_RCC_GPIOB_CLK_ENABLE()
#define GYRO_INT1_GPIO_CLK_DISABLE()            __HAL_RCC_GPIOB_CLK_DISABLE()
#define GYRO_INT1_PIN                           GPIO_PIN_6                  /* PB.06 */
#define GYRO_INT1_EXTI_IRQn                     EXTI9_5_IRQn
#define GYRO_INT2_GPIO_PORT                     GPIOB                       /* GPIOB */
#define GYRO_INT2_GPIO_CLK_ENABLE()             __HAL_RCC_GPIOB_CLK_ENABLE()
#define GYRO_INT2_GPIO_CLK_DISABLE()            __HAL_RCC_GPIOB_CLK_DISABLE()
#define GYRO_INT2_PIN                           GPIO_PIN_7                  /* PB.07 */
#define GYRO_INT2_EXTI_IRQn                     EXTI9_5_IRQn
#endif

/*##################### Idd ##########################*/
/**
  * @brief  Idd current measurement interface pins
  */
#define IDD_INT_GPIO_PORT                       GPIOC                       /* GPIOC */
#define IDD_INT_GPIO_CLK_ENABLE()               __HAL_RCC_GPIOC_CLK_ENABLE()
#define IDD_INT_GPIO_CLK_DISABLE()              __HAL_RCC_GPIOC_CLK_DISABLE()
#define IDD_INT_PIN                             GPIO_PIN_13                  /* PC.13 */
#define IDD_INT_EXTI_IRQn                       EXTI15_10_IRQn

#define IDD_WAKEUP_GPIO_PORT                    GPIOA                       /* GPIOA */
#define IDD_WAKEUP_GPIO_CLK_ENABLE()            __HAL_RCC_GPIOA_CLK_ENABLE()
#define IDD_WAKEUP_GPIO_CLK_DISABLE()           __HAL_RCC_GPIOA_CLK_DISABLE()
#define IDD_WAKEUP_PIN                          GPIO_PIN_4                  /* PA.04 */

/**
  * @}
  */

/**
  * @}
  */


/** @defgroup STM32L476G_DISCOVERY_Exported_Functions Exported Functions
  * @{
  */
uint32_t                BSP_GetVersion(void);
SupplyMode_TypeDef      BSP_SupplyModeDetection(void);
void                    BSP_LED_Init(Led_TypeDef Led);
void                    BSP_LED_DeInit(Led_TypeDef Led);
void                    BSP_LED_On(Led_TypeDef Led);
void                    BSP_LED_Off(Led_TypeDef Led);
void                    BSP_LED_Toggle(Led_TypeDef Led);
uint8_t                 BSP_JOY_Init(JOYMode_TypeDef Joy_Mode);
void                    BSP_JOY_DeInit(void);
JOYState_TypeDef        BSP_JOY_GetState(void);

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

#endif /* __STM32L476G_DISCOVERY_H */


