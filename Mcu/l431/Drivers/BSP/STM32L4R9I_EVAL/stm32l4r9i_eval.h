/**
  ******************************************************************************
  * @file    stm32l4r9i_eval.h
  * @author  MCD Application Team
  * @brief   This file contains definitions for STM32L4R9I_EVAL's LEDs,
  *          push-buttons and COM ports hardware resources.
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
#ifndef __STM32L4R9I_EVAL_H
#define __STM32L4R9I_EVAL_H

#ifdef __cplusplus
 extern "C" {
#endif

/**
  * @brief  Define for STM32L4R9I_EVAL board
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "stm32l4r9i_eval_io.h"
#if defined(BSP_USE_CMSIS_OS)
#include "cmsis_os.h"
#endif

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L4R9I_EVAL
  * @{
  */


/** @addtogroup STM32L4R9I_EVAL_Common
  * @{
  */

/** @defgroup STM32L4R9I_EVAL_Exported_Types Exported Types
  * @{
  */

/**
 * @brief LED Types Definition
 */
typedef enum
{
  LED1 = 0,
  LED_GREEN  = LED1,

  LED2 = 1,
  LED_ORANGE = LED2,

  LED3 = 2,
  LED_RED    = LED3,

  LED4 = 3,
  LED_BLUE   = LED4

}Led_TypeDef;

/**
 * @brief BUTTON Types Definition
 */
typedef enum
{
  BUTTON_WAKEUP = 0,
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
 * @brief COM Types Definition
 */
typedef enum
{
  COM1 = 0

}COM_TypeDef;

/**
  * @}
  */

/** @defgroup STM32L4R9I_EVAL_Exported_Constants Exported Constants
  * @{
  */

/** @defgroup STM32L4R9I_EVAL_LED LED Constants
  * @{
  */
#define LEDn                             4

#define LED1_PIN                         GPIO_PIN_15
#define LED1_GPIO_PORT                   GPIOB
#define LED1_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOB_CLK_ENABLE()
#define LED1_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOB_CLK_DISABLE()

#define LED2_PIN                         GPIO_PIN_13
#define LED2_GPIO_PORT                   GPIOH
#define LED2_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOH_CLK_ENABLE()
#define LED2_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOH_CLK_DISABLE()

#define LED3_PIN                         GPIO_PIN_14
#define LED3_GPIO_PORT                   GPIOH
#define LED3_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOH_CLK_ENABLE()
#define LED3_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOH_CLK_DISABLE()

#define LED4_PIN                         GPIO_PIN_15
#define LED4_GPIO_PORT                   GPIOH
#define LED4_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOH_CLK_ENABLE()
#define LED4_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOH_CLK_DISABLE()

#define LEDx_GPIO_CLK_ENABLE(__LED__)    do { if((__LED__) == LED1) { __HAL_RCC_GPIOB_CLK_ENABLE(); } else \
                                              __HAL_RCC_GPIOH_CLK_ENABLE(); } while(0)

#define LEDx_GPIO_CLK_DISABLE(__LED__)   do { if((__LED__) == LED1) { __HAL_RCC_GPIOB_CLK_DISABLE(); } else \
                                              __HAL_RCC_GPIOH_CLK_DISABLE(); } while(0)
/**
  * @}
  */

/** @defgroup STM32L4R9I_EVAL_BUTTON  BUTTON Constants
  * @{
  */
#define JOYn                             5
#define BUTTONn                          1 + JOYn

/**
 * @brief Wakeup/Tamper push-button
 */
#define WAKEUP_BUTTON_PIN                GPIO_PIN_13
#define WAKEUP_BUTTON_GPIO_PORT          GPIOC
#define WAKEUP_BUTTON_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOC_CLK_ENABLE()
#define WAKEUP_BUTTON_GPIO_CLK_DISABLE() __HAL_RCC_GPIOC_CLK_DISABLE()
#define WAKEUP_BUTTON_EXTI_IRQn          EXTI15_10_IRQn

#define BUTTONx_GPIO_CLK_ENABLE(__BUTTON__)     do { if((__BUTTON__) == BUTTON_WAKEUP) { WAKEUP_BUTTON_GPIO_CLK_ENABLE(); } } while(0)

#define BUTTONx_GPIO_CLK_DISABLE(__BUTTON__)    do { if((__BUTTON__) == BUTTON_WAKEUP) { WAKEUP_BUTTON_GPIO_CLK_DISABLE(); } } while(0)


/**********************************************************/
/* The joystick buttons are accessed thru the IO Expander */
/**********************************************************/

/**
 * @brief Joystick Sel push-button
 */
#define SEL_JOY_PIN                      IO_PIN_0
#define SEL_JOY_EXTI_IRQn                EXTI0_IRQn

/**
 * @brief Joystick Down push-button
 */
#if defined(USE_STM32L4R9I_EVAL_REVA)
/* Hide Joystick 180° rotation misplacement */
#define DOWN_JOY_PIN                     IO_PIN_4
#else
#define DOWN_JOY_PIN                     IO_PIN_1
#endif

#define DOWN_JOY_EXTI_IRQn               EXTI0_IRQn

/**
 * @brief Joystick Left push-button
 */
#if defined(USE_STM32L4R9I_EVAL_REVA)
/* Hide Joystick 180° rotation misplacement */
#define LEFT_JOY_PIN                     IO_PIN_3
#else
#define LEFT_JOY_PIN                     IO_PIN_2
#endif

#define LEFT_JOY_EXTI_IRQn               EXTI0_IRQn

/**
 * @brief Joystick Right push-button
 */
#if defined(USE_STM32L4R9I_EVAL_REVA)
/* Hide Joystick 180° rotation misplacement */
#define RIGHT_JOY_PIN                    IO_PIN_2
#else
#define RIGHT_JOY_PIN                    IO_PIN_3
#endif

#define RIGHT_JOY_EXTI_IRQn              EXTI0_IRQn

/**
 * @brief Joystick Up push-button
 */
#if defined(USE_STM32L4R9I_EVAL_REVA)
/* Hide Joystick 180° rotation misplacement */
#define UP_JOY_PIN                       IO_PIN_1
#else
#define UP_JOY_PIN                       IO_PIN_4
#endif

#define UP_JOY_EXTI_IRQn                 EXTI0_IRQn


#define JOY_ALL_PINS                     (RIGHT_JOY_PIN | LEFT_JOY_PIN | UP_JOY_PIN | DOWN_JOY_PIN | SEL_JOY_PIN)
#define JOY_NONE_PIN                     JOY_ALL_PINS

/**
  * @}
  */

/** @defgroup STM32L4R9I_EVAL_COM  COM Constants
  * @{
  */
#define COMn                                    1

/**
 * @brief Definition for COM port1, connected to LPUART1
 */
#define EVAL_COM1                               LPUART1
#define EVAL_COM1_CLK_ENABLE()                  __HAL_RCC_LPUART1_CLK_ENABLE()
#define EVAL_COM1_CLK_DISABLE()                 __HAL_RCC_LPUART1_CLK_DISABLE()

#define EVAL_COM1_TX_PIN                        GPIO_PIN_7
#define EVAL_COM1_TX_GPIO_PORT                  GPIOG
#define EVAL_COM1_TX_GPIO_CLK_ENABLE()          __HAL_RCC_GPIOG_CLK_ENABLE()
#define EVAL_COM1_TX_GPIO_CLK_DISABLE()         __HAL_RCC_GPIOG_CLK_DISABLE()
#define EVAL_COM1_TX_AF                         GPIO_AF8_LPUART1

#define EVAL_COM1_RX_PIN                        GPIO_PIN_8
#define EVAL_COM1_RX_GPIO_PORT                  GPIOG
#define EVAL_COM1_RX_GPIO_CLK_ENABLE()          __HAL_RCC_GPIOG_CLK_ENABLE()
#define EVAL_COM1_RX_GPIO_CLK_DISABLE()         __HAL_RCC_GPIOG_CLK_DISABLE()
#define EVAL_COM1_RX_AF                         GPIO_AF8_LPUART1

#define EVAL_COM1_IRQn                          LPUART1_IRQn

#define COMx_CLK_ENABLE(__INDEX__)              do { if((__INDEX__) == COM1) { EVAL_COM1_CLK_ENABLE(); } } while(0)
#define COMx_CLK_DISABLE(__INDEX__)             do { if((__INDEX__) == COM1) { EVAL_COM1_CLK_DISABLE(); } } while(0)

#define COMx_TX_GPIO_CLK_ENABLE(__INDEX__)      do { if((__INDEX__) == COM1) { EVAL_COM1_TX_GPIO_CLK_ENABLE(); } } while(0)
#define COMx_TX_GPIO_CLK_DISABLE(__INDEX__)     do { if((__INDEX__) == COM1) { EVAL_COM1_TX_GPIO_CLK_DISABLE(); } } while(0)

#define COMx_RX_GPIO_CLK_ENABLE(__INDEX__)      do { if((__INDEX__) == COM1) { EVAL_COM1_RX_GPIO_CLK_ENABLE(); } } while(0)
#define COMx_RX_GPIO_CLK_DISABLE(__INDEX__)     do { if((__INDEX__) == COM1) { EVAL_COM1_RX_GPIO_CLK_DISABLE(); } } while(0)

/**
  * @}
  */

#ifdef HAL_ADC_MODULE_ENABLED

/** @defgroup STM32L4R9I_EVAL_POTENTIOMETER  POTENTIOMETER Constants
  * @{
  */
/**
 * @brief  Potentiometer Status values
 */
#define BSP_POTENTIOMETER_OK       0x00
#define BSP_POTENTIOMETER_ERROR    0xFF

/**
 * @brief Definition for Potentiometer, connected to ADC1
 */
#define ADCx                            ADC1
#define ADCx_CLK_ENABLE()               __HAL_RCC_ADC_CLK_ENABLE()
#define ADCx_CHANNEL_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOA_CLK_ENABLE()

#define ADCx_FORCE_RESET()              __HAL_RCC_ADC_FORCE_RESET()
#define ADCx_RELEASE_RESET()            __HAL_RCC_ADC_RELEASE_RESET()

/* Definition for ADCx Channel Pin */
#define ADCx_CHANNEL_PIN                GPIO_PIN_0
#define ADCx_CHANNEL_GPIO_PORT          GPIOA

/* Definition for ADCx's Channel */
#define ADCx_CHANNEL                    ADC_CHANNEL_5
#define SAMPLINGTIME                    ADC_SAMPLETIME_3CYCLES
#define ADCx_POLL_TIMEOUT               10

#endif /* HAL_ADC_MODULE_ENABLED */

/**
 * @brief Definition for Potentiometer connected to OPAMP
 */
/* Definition for OPAMPx Pins */
#define OPAMPx_INP_PIN                  GPIO_PIN_0
#define OPAMPx_INP_GPIO_PORT            GPIOA
#define OPAMPx_INM_PIN                  GPIO_PIN_1
#define OPAMPx_INM_GPIO_PORT            GPIOA
#define OPAMPx_VOUT_PIN                 GPIO_PIN_3
#define OPAMPx_VOUT_GPIO_PORT           GPIOA

#define OPAMPx                          OPAMP
#define OPAMPx_CLK_ENABLE()             __HAL_RCC_OPAMP_CLK_ENABLE()
#define OPAMPx_CLK_DISABLE()            __HAL_RCC_OPAMP_CLK_DISABLE()
#define OPAMPx_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOA_CLK_ENABLE()
#define OPAMPx_GPIO_CLK_DISABLE()       __HAL_RCC_GPIOA_CLK_DISABLE()

#define OPAMPx_FORCE_RESET()            __HAL_RCC_OPAMP_FORCE_RESET()
#define OPAMPx_RELEASE_RESET()          __HAL_RCC_OPAMP_RELEASE_RESET()

/**
  * @brief Eval Pins definition connected to MFX
  */

#define DSI_RST_PIN                    IO_PIN_9
#define EXT_RST_PIN                    IO_PIN_8
#define LCD_DISP_PIN                   IO_PIN_11
#define LCD_RST_PIN                    IO_PIN_12
#define OTG_FS_POWER_SWITCH_PIN        IO_PIN_6
#define OTG_FS_OVER_CURRENT_PIN        IO_PIN_7
#define SD_DETECT_PIN                  IO_PIN_5
#define SWIRE_PIN                      IO_PIN_13

/**
  * @}
  */

/** @defgroup STM32L4R9I_EVAL_BUS  BUS Constants
  * @{
  */
/* Exported constant IO ------------------------------------------------------*/
/*##################### I2Cx ###################################*/
/* User can use this section to tailor I2Cx instance used and associated
   resources */
/* Definition for I2Cx Pins */
#define EVAL_I2Cx_SCL_PIN                       GPIO_PIN_4
#define EVAL_I2Cx_SCL_GPIO_PORT                 GPIOH
#define EVAL_I2Cx_SDA_PIN                       GPIO_PIN_5
#define EVAL_I2Cx_SDA_GPIO_PORT                 GPIOH
#define EVAL_I2Cx_SCL_SDA_AF                    GPIO_AF4_I2C2

/* Definition for I2Cx clock resources */
#define EVAL_I2Cx                               I2C2
#define EVAL_I2Cx_CLK_ENABLE()                  __HAL_RCC_I2C2_CLK_ENABLE()
#define EVAL_I2Cx_CLK_DISABLE()                 __HAL_RCC_I2C2_CLK_DISABLE()
#define EVAL_I2Cx_SDA_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOH_CLK_ENABLE()
#define EVAL_I2Cx_SDA_GPIO_CLK_DISABLE()        __HAL_RCC_GPIOH_CLK_DISABLE()
#define EVAL_I2Cx_SCL_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOH_CLK_ENABLE()
#define EVAL_I2Cx_SCL_GPIO_CLK_DISABLE()        __HAL_RCC_GPIOH_CLK_DISABLE()

#define EVAL_I2Cx_FORCE_RESET()                 __HAL_RCC_I2C2_FORCE_RESET()
#define EVAL_I2Cx_RELEASE_RESET()               __HAL_RCC_I2C2_RELEASE_RESET()

/* Definition for I2Cx's NVIC */
#define EVAL_I2Cx_EV_IRQn                       I2C2_EV_IRQn
#define EVAL_I2Cx_EV_IRQHandler                 I2C2_EV_IRQHandler
#define EVAL_I2Cx_ER_IRQn                       I2C2_ER_IRQn
#define EVAL_I2Cx_ER_IRQHandler                 I2C2_ER_IRQHandler

/* I2C clock setting */
#define EVAL_RCC_PERIPHCLK_I2C                  RCC_PERIPHCLK_I2C2
#define EVAL_RCC_CLKSOURCE_I2C                  RCC_I2C2CLKSOURCE_SYSCLK

/* I2C TIMING Register define when I2C clock source is SYSCLK */
/* I2C TIMING is calculated in case of the I2C Clock source is the SYSCLK = 120 MHz */
/* Set 0xC080242F value to reach 100 KHz speed (Rise time = 640ns, Fall time = 20ns) */
#ifndef EVAL_I2Cx_TIMING
 #define EVAL_I2Cx_TIMING                       0xC080242F
#endif /* EVAL_I2Cx_TIMING */


/* Definition for TS (Capacitive Touch Panel) I2Cx resources */
#define EVAL_TS_I2Cx                            I2C2
#define EVAL_TS_I2Cx_CLK_ENABLE()               __HAL_RCC_I2C2_CLK_ENABLE()
#define EVAL_TS_DMAx_CLK_ENABLE()               __HAL_RCC_DMA1_CLK_ENABLE()
#define EVAL_TS_I2Cx_SDA_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOH_CLK_ENABLE()
#define EVAL_TS_I2Cx_SDA_GPIO_CLK_DISABLE()     __HAL_RCC_GPIOH_CLK_DISABLE()
#define EVAL_TS_I2Cx_SCL_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOH_CLK_ENABLE()
#define EVAL_TS_I2Cx_SCL_GPIO_CLK_DISABLE()     __HAL_RCC_GPIOH_CLK_DISABLE()

#define EVAL_TS_I2Cx_FORCE_RESET()              __HAL_RCC_I2C2_FORCE_RESET()
#define EVAL_TS_I2Cx_RELEASE_RESET()            __HAL_RCC_I2C2_RELEASE_RESET()

/* Definition for I2Cx Pins */
#define EVAL_TS_I2Cx_SCL_PIN                    GPIO_PIN_4
#define EVAL_TS_I2Cx_SCL_GPIO_PORT              GPIOH
#define EVAL_TS_I2Cx_SDA_PIN                    GPIO_PIN_5
#define EVAL_TS_I2Cx_SDA_GPIO_PORT              GPIOH
#define EVAL_TS_I2Cx_SCL_SDA_AF                 GPIO_AF4_I2C2

/* I2C interrupt requests */
#define EVAL_TS_I2Cx_EV_IRQn                    I2C2_EV_IRQn
#define EVAL_TS_I2Cx_ER_IRQn                    I2C2_ER_IRQn

/* I2C clock setting */
#define EVAL_TS_RCC_PERIPHCLK_I2C               RCC_PERIPHCLK_I2C2
#define EVAL_TS_RCC_CLKSOURCE_I2C               RCC_I2C2CLKSOURCE_SYSCLK


/* Exported constantIO -------------------------------------------------------*/

#define EEPROM_I2C_ADDRESS                      0xA0

/* I2C clock speed configuration (in Hz)
   WARNING:
   Make sure that this define is not already declared in other files (ie.
   stm324xg_eval.h file). It can be used in parallel by other modules. */
#ifndef BSP_I2C_SPEED
 #define BSP_I2C_SPEED                          100000
#endif /* BSP_I2C_SPEED */


#define IO_I2C_ADDRESS                          0x84
#define IDD_I2C_ADDRESS                         IO_I2C_ADDRESS
#define AUDIO_I2C_ADDRESS                       0x34
#define LCD_RGB_I2C_ADDRESS                     0x70
#define TS_RGB_I2C_ADDRESS                      0x70
#define TS_DSI_I2C_ADDRESS                      0x70

/* Maximum Timeout values for flags waiting loops. These timeouts are not based
   on accurate values, they just guarantee that the application will not remain
   stuck if the I2C communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...). */
#define EVAL_I2Cx_TIMEOUT_MAX                   3000


/*##################### Audio Codec ##########################*/
/* Audio INT Pin definition */
#define AUDIO_INT_GPIO_PORT               GPIOI                      /* GPIOI */
#define AUDIO_INT_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOI_CLK_ENABLE()
#define AUDIO_INT_GPIO_CLK_DISABLE()      __HAL_RCC_GPIOI_CLK_DISABLE()
#define AUDIO_INT_PIN                     GPIO_PIN_6                 /* PI.06 */


/*##################### MFX #################################*/
/**
  * @brief  MFX interface pins
  */
#define MFX_INT_GPIO_PORT                 GPIOA                       /* GPIOA */
#define MFX_INT_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOA_CLK_ENABLE()
#define MFX_INT_GPIO_CLK_DISABLE()        __HAL_RCC_GPIOA_CLK_DISABLE()
#define MFX_INT_PIN                       GPIO_PIN_0                  /* PA.00 */
#define MFX_INT_EXTI_IRQn                 EXTI0_IRQn

#define MFX_WAKEUP_GPIO_PORT              GPIOG                       /* GPIOG */
#define MFX_WAKEUP_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOG_CLK_ENABLE()
#define MFX_WAKEUP_GPIO_CLK_DISABLE()     __HAL_RCC_GPIOG_CLK_DISABLE()
#define MFX_WAKEUP_PIN                    GPIO_PIN_9                  /* PG.09 */

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

/*##################### LCD ##########################*/

/** MB1314 DSI **/

/* DSI TE pin */
#define LCD_DSI_TE_PIN                       GPIO_PIN_6
#define LCD_DSI_TE_CTRL_GPIO_PORT            GPIOG
#define LCD_DSI_TE_CTRL_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOG_CLK_ENABLE()
#define LCD_DSI_TE_CTRL_GPIO_CLK_DISABLE()   __HAL_RCC_GPIOG_CLK_DISABLE()

/* Backlight control pin */
#define LCD_DSI_BL_CTRL_PIN                  GPIO_PIN_14
#define LCD_DSI_BL_CTRL_GPIO_PORT            GPIOB
#define LCD_DSI_BL_CTRL_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOB_CLK_ENABLE()
#define LCD_DSI_BL_CTRL_GPIO_CLK_DISABLE()   __HAL_RCC_GPIOB_CLK_DISABLE()

/* DSI TS INT pin */
#define TS_DSI_INT_PIN                       GPIO_PIN_2
#define TS_DSI_INT_GPIO_PORT                 GPIOC
#define TS_DSI_INT_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOC_CLK_ENABLE()
#define TS_DSI_INT_GPIO_CLK_DISABLE()        __HAL_RCC_GPIOC_CLK_DISABLE()
#define TS_DSI_INT_EXTI_IRQn                 EXTI2_IRQn


/** MB1315 RGB **/

/* Display enable pin */
#define LCD_RGB_DISP_PIN                     IO_PIN_11   /* MFX_GPIO11 */

/* Display enable pin */
#define LCD_RGB_RST_PIN                      IO_PIN_12   /* MFX_GPIO12 */

/* Backlight control pin */
#define LCD_RGB_BL_CTRL_PIN                  GPIO_PIN_5
#define LCD_RGB_BL_CTRL_GPIO_PORT            GPIOA
#define LCD_RGB_BL_CTRL_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOA_CLK_ENABLE()
#define LCD_RGB_BL_CTRL_GPIO_CLK_DISABLE()   __HAL_RCC_GPIOA_CLK_DISABLE()

/* RGB TS INT pin */
#define TS_RGB_INT_PIN                       GPIO_PIN_2
#define TS_RGB_INT_GPIO_PORT                 GPIOC
#define TS_RGB_INT_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOC_CLK_ENABLE()
#define TS_RGB_INT_GPIO_CLK_DISABLE()        __HAL_RCC_GPIOC_CLK_DISABLE()
#define TS_RGB_INT_EXTI_IRQn                 EXTI2_IRQn

/**
  * @}
  */

/**
  * @}
  */

/** @defgroup STM32L4R9I_EVAL_Exported_Functions Exported Functions
  * @{
  */
uint32_t         BSP_GetVersion(void);
void             BSP_LED_Init(Led_TypeDef Led);
void             BSP_LED_DeInit(Led_TypeDef Led);
void             BSP_LED_On(Led_TypeDef Led);
void             BSP_LED_Off(Led_TypeDef Led);
void             BSP_LED_Toggle(Led_TypeDef Led);
void             BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef ButtonMode);
void             BSP_PB_DeInit(Button_TypeDef Button);
uint32_t         BSP_PB_GetState(Button_TypeDef Button);
#if defined(HAL_UART_MODULE_ENABLED)
void             BSP_COM_Init(COM_TypeDef COM, UART_HandleTypeDef* huart);
void             BSP_COM_DeInit(COM_TypeDef COM, UART_HandleTypeDef *huart);
#endif /* HAL_UART_MODULE_ENABLED */
#ifdef HAL_ADC_MODULE_ENABLED
uint32_t         BSP_POTENTIOMETER_Init(void);
uint32_t         BSP_POTENTIOMETER_DeInit(void);
uint32_t         BSP_POTENTIOMETER_GetLevel(void);
#endif /* HAL_ADC_MODULE_ENABLED */
uint8_t          BSP_JOY_Init(JOYMode_TypeDef JoyMode);
void             BSP_JOY_DeInit(void);
JOYState_TypeDef BSP_JOY_GetState(void);

/* These __weak functions can be surcharged by application code for specific application needs */
void             BSP_ErrorHandler(void);

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

#endif /* __STM32L4R9I_EVAL_H */


