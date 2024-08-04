/**
  ******************************************************************************
  * @file    stm32l475e_iot01.h
  * @author  MCD Application Team
  * @brief   STM32L475E IOT01 board support package
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
#ifndef __STM32L475E_IOT01_H
#define __STM32L475E_IOT01_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L475E_IOT01
  * @{
  */
      
/** @addtogroup STM32L475E_IOT01_LOW_LEVEL
  * @{
  */ 

/** @defgroup STM32L475E_IOT01_LOW_LEVEL_Exported_Types LOW LEVEL Exported Types
  * @{
  */
typedef enum 
{
LED2 = 0,
LED_GREEN = LED2,
}Led_TypeDef;


typedef enum 
{  
  BUTTON_USER = 0
}Button_TypeDef;

typedef enum 
{  
  BUTTON_MODE_GPIO = 0,
  BUTTON_MODE_EXTI = 1
}ButtonMode_TypeDef;

typedef enum 
{
  COM1 = 0,
  COM2 = 0,
}COM_TypeDef;
/**
  * @}
  */ 

/** @defgroup STM32L475E_IOT01_LOW_LEVEL_Exported_Constants LOW LEVEL Exported Constants
  * @{
  */ 

/** 
  * @brief  Define for STM32L475E_IOT01 board
  */ 
#if !defined (USE_STM32L475E_IOT01)
 #define USE_STM32L475E_IOT01
#endif

#define LEDn                             ((uint8_t)1)

#define LED2_PIN                         GPIO_PIN_14
#define LED2_GPIO_PORT                   GPIOB
#define LED2_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOB_CLK_ENABLE()
#define LED2_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOB_CLK_DISABLE()



#define LEDx_GPIO_CLK_ENABLE(__INDEX__)  do{if((__INDEX__) == 0) LED2_GPIO_CLK_ENABLE();}while(0)

#define LEDx_GPIO_CLK_DISABLE(__INDEX__)  do{if((__INDEX__) == 0) LED2_GPIO_CLK_DISABLE();}while(0)

/* Only one User/Wakeup button */
#define BUTTONn                             ((uint8_t)1)

/**
  * @brief Wakeup push-button
  */
#define USER_BUTTON_PIN                   GPIO_PIN_13
#define USER_BUTTON_GPIO_PORT             GPIOC
#define USER_BUTTON_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOC_CLK_ENABLE()
#define USER_BUTTON_GPIO_CLK_DISABLE()    __HAL_RCC_GPIOC_CLK_DISABLE()
#define USER_BUTTON_EXTI_IRQn             EXTI15_10_IRQn

/**
  * @brief  NFC Gpio PINs
  */
#define NFC_GPIO_GPO_PIN                         GPIO_PIN_4
#define NFC_GPIO_GPO_PIN_PORT                    GPIOE
#define NFC_GPIO_RFDISABLE_PIN                   GPIO_PIN_2
#define NFC_GPIO_RFDISABLE_PIN_PORT              GPIOE  
#define NFC_GPIO_CLK_ENABLE()                    __HAL_RCC_GPIOE_CLK_ENABLE();
#define NFC_GPIO_CLK_DISABLE()                   __HAL_RCC_GPIOE_CLK_DISABLE();


#define COMn                              ((uint8_t)1)

/**
 * @brief Definition for COM port1, connected to USART1
 */ 
#define DISCOVERY_COM1                          USART1
#define DISCOVERY_COM1_CLK_ENABLE()             __HAL_RCC_USART1_CLK_ENABLE()
#define DISCOVERY_COM1_CLK_DISABLE()            __HAL_RCC_USART1_CLK_DISABLE()

#define DISCOVERY_COM1_TX_PIN                   GPIO_PIN_6
#define DISCOVERY_COM1_TX_GPIO_PORT             GPIOB
#define DISCOVERY_COM1_TX_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOB_CLK_ENABLE()   
#define DISCOVERY_COM1_TX_GPIO_CLK_DISABLE()    __HAL_RCC_GPIOB_CLK_DISABLE()  
#define DISCOVERY_COM1_TX_AF                    GPIO_AF7_USART1

#define DISCOVERY_COM1_RX_PIN                   GPIO_PIN_7
#define DISCOVERY_COM1_RX_GPIO_PORT             GPIOB
#define DISCOVERY_COM1_RX_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOB_CLK_ENABLE()   
#define DISCOVERY_COM1_RX_GPIO_CLK_DISABLE()    __HAL_RCC_GPIOB_CLK_DISABLE()  
#define DISCOVERY_COM1_RX_AF                    GPIO_AF7_USART1

#define DISCOVERY_COM1_IRQn                     USART1_IRQn
                                              

#define DISCOVERY_COMx_CLK_ENABLE(__INDEX__)            do { if((__INDEX__) == COM1) {DISCOVERY_COM1_CLK_ENABLE();}} while(0)
#define DISCOVERY_COMx_CLK_DISABLE(__INDEX__)           do { if((__INDEX__) == COM1) {DISCOVERY_COM1_CLK_DISABLE();}} while(0)

#define DISCOVERY_COMx_TX_GPIO_CLK_ENABLE(__INDEX__)    do { if((__INDEX__) == COM1) {DISCOVERY_COM1_TX_GPIO_CLK_ENABLE();}} while(0)
#define DISCOVERY_COMx_TX_GPIO_CLK_DISABLE(__INDEX__)   do { if((__INDEX__) == COM1) {DISCOVERY_COM1_TX_GPIO_CLK_DISABLE();}} while(0)

#define DISCOVERY_COMx_RX_GPIO_CLK_ENABLE(__INDEX__)    do { if((__INDEX__) == COM1) {DISCOVERY_COM1_RX_GPIO_CLK_ENABLE();}} while(0)
#define DISCOVERY_COMx_RX_GPIO_CLK_DISABLE(__INDEX__)   do { if((__INDEX__) == COM1) {DISCOVERY_COM1_RX_GPIO_CLK_DISABLE();}} while(0)


/* User can use this section to tailor I2Cx instance used and associated resources */
/* Definition for I2Cx resources */
#define DISCOVERY_I2Cx                             I2C2
#define DISCOVERY_I2Cx_CLK_ENABLE()                __HAL_RCC_I2C2_CLK_ENABLE()
#define DISCOVERY_I2Cx_CLK_DISABLE()               __HAL_RCC_I2C2_CLK_DISABLE()   
#define DISCOVERY_DMAx_CLK_ENABLE()                __HAL_RCC_DMA1_CLK_ENABLE()
#define DISCOVERY_I2Cx_SCL_SDA_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOB_CLK_ENABLE()
#define DISCOVERY_I2Cx_SCL_SDA_GPIO_CLK_DISABLE()  __HAL_RCC_GPIOB_CLK_DISABLE()
   
#define DISCOVERY_I2Cx_FORCE_RESET()               __HAL_RCC_I2C2_FORCE_RESET()
#define DISCOVERY_I2Cx_RELEASE_RESET()             __HAL_RCC_I2C2_RELEASE_RESET()

/* Definition for I2Cx Pins */
#define DISCOVERY_I2Cx_SCL_PIN                     GPIO_PIN_10
#define DISCOVERY_I2Cx_SDA_PIN                     GPIO_PIN_11                                               
#define DISCOVERY_I2Cx_SCL_SDA_GPIO_PORT           GPIOB
#define DISCOVERY_I2Cx_SCL_SDA_AF                  GPIO_AF4_I2C2

/* I2C interrupt requests */
#define DISCOVERY_I2Cx_EV_IRQn                     I2C2_EV_IRQn
#define DISCOVERY_I2Cx_ER_IRQn                     I2C2_ER_IRQn

/* I2C clock speed configuration (in Hz)
  WARNING:
   Make sure that this define is not already declared in other files.
   It can be used in parallel by other modules. */
#ifndef DISCOVERY_I2C_SPEED
 #define DISCOVERY_I2C_SPEED                             100000
#endif /* DISCOVERY_I2C_SPEED */

#ifndef DISCOVERY_I2Cx_TIMING  
#define DISCOVERY_I2Cx_TIMING                     ((uint32_t)0x00702681)
#endif /* DISCOVERY_I2Cx_TIMING */ 


/* I2C Sensors address */
/* LPS22HB (Pressure) I2C Address */
#define LPS22HB_I2C_ADDRESS  (uint8_t)0xBA
/* HTS221 (Humidity) I2C Address */
#define HTS221_I2C_ADDRESS   (uint8_t)0xBE

#ifdef USE_LPS22HB_TEMP
/* LPS22HB Sensor hardware I2C address */ 
#define TSENSOR_I2C_ADDRESS     LPS22HB_I2C_ADDRESS
#else /* USE_HTS221_TEMP */
/* HTS221 Sensor hardware I2C address */ 
#define TSENSOR_I2C_ADDRESS     HTS221_I2C_ADDRESS
#endif

/* NFC I2C address and specific config parameters */
#define M24SR_I2C_ADDR             (uint8_t)  0xAC /*!< M24SR address */
#define NFC_I2C_STATUS_SUCCESS     (uint16_t) 0x0000
#define NFC_I2C_ERROR_TIMEOUT      (uint16_t) 0x0011
#define NFC_I2C_TIMEOUT_STD        (uint32_t) 8 /* I2C Time out (ms), used to call Transmit/Receive HAL functions */
#define NFC_I2C_TIMEOUT_MAX        (uint32_t) 200 /* I2C Time out (ms), this is the maximum time needed by M24SR to complete any command */
#define NFC_I2C_TRIALS             (uint32_t) 1 /* In case M24SR will reply ACK failed allow to perform retry before returning error (HAL option not used) */


/**
  * @}
  */

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
                                               
/** @defgroup STM32L475E_IOT01_LOW_LEVEL_Exported_Functions LOW LEVEL Exported Functions
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
void             BSP_COM_Init(COM_TypeDef COM, UART_HandleTypeDef *husart);
void             BSP_COM_DeInit(COM_TypeDef COM, UART_HandleTypeDef *huart);
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

#endif /* __STM32L475E_IOT01_H */


