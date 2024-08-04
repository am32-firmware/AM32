/**
  ******************************************************************************
  * @file    stm32l476g_discovery.c
  * @author  MCD Application Team
  * @brief   This file provides a set of firmware functions to manage Leds,
  *          push-button and joystick of STM32L476G-Discovery board (MB1184)
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

/* Includes ------------------------------------------------------------------*/
#include "stm32l476g_discovery.h"

/** @addtogroup BSP
  * @{
  */

/** @defgroup STM32L476G_DISCOVERY STM32L476G-DISCOVERY
  * @{
  */

/** @defgroup STM32L476G_DISCOVERY_Common STM32L476G-DISCOVERY Common
  * @{
  */

/** @defgroup STM32L476G_DISCOVERY_Private_TypesDefinitions Private Types Definitions
  * @brief This file provides firmware functions to manage Leds, push-buttons,
  *        COM ports, SD card on SPI and temperature sensor (TS751) available on
  *        STM32L476G-DISCOVERY discoveryuation board from STMicroelectronics.
  * @{
  */

/**
  * @}
  */

/** @defgroup STM32L476G_DISCOVERY_Private_Defines Private Defines
  * @{
  */

/**
 * @brief STM32L476G DISCOVERY BSP Driver version number
   */
#define __STM32L476G_DISCOVERY_BSP_VERSION_MAIN   (0x02) /*!< [31:24] main version */
#define __STM32L476G_DISCOVERY_BSP_VERSION_SUB1   (0x00) /*!< [23:16] sub1 version */
#define __STM32L476G_DISCOVERY_BSP_VERSION_SUB2   (0x07) /*!< [15:8]  sub2 version */
#define __STM32L476G_DISCOVERY_BSP_VERSION_RC     (0x00) /*!< [7:0]  release candidate */
#define __STM32L476G_DISCOVERY_BSP_VERSION            ((__STM32L476G_DISCOVERY_BSP_VERSION_MAIN << 24)\
                                                      |(__STM32L476G_DISCOVERY_BSP_VERSION_SUB1 << 16)\
                                                      |(__STM32L476G_DISCOVERY_BSP_VERSION_SUB2 << 8 )\
                                                      |(__STM32L476G_DISCOVERY_BSP_VERSION_RC))
/**
  * @}
  */


/** @defgroup STM32L476G_DISCOVERY_Private_Macros Private Macros
  * @{
  */

/**
  * @}
  */


/** @defgroup STM32L476G_DISCOVERY_Exported_Variables Exported Variables
  * @{
  */

/**
 * @brief LED variables
 */
#if defined (USE_STM32L476G_DISCO_REVC) || defined (USE_STM32L476G_DISCO_REVB)
GPIO_TypeDef   *LED_PORT[LEDn] =  {LED4_GPIO_PORT,
                                   LED5_GPIO_PORT
                                  };

const uint16_t  LED_PIN[LEDn] =   {LED4_PIN,
                                   LED5_PIN
                                  };
#elif defined (USE_STM32L476G_DISCO_REVA)
GPIO_TypeDef   *LED_PORT[LEDn] =  {LED3_GPIO_PORT,
                                   LED4_GPIO_PORT
                                  };

const uint16_t  LED_PIN[LEDn] =   {LED3_PIN,
                                   LED4_PIN
                                  };
#endif


/**
 * @brief JOYSTICK variables
 */
GPIO_TypeDef *JOY_PORT[JOYn] =  {SEL_JOY_GPIO_PORT,
                                 DOWN_JOY_GPIO_PORT,
                                 LEFT_JOY_GPIO_PORT,
                                 RIGHT_JOY_GPIO_PORT,
                                 UP_JOY_GPIO_PORT
                                };

const uint16_t JOY_PIN[JOYn] =   {SEL_JOY_PIN,
                                  LEFT_JOY_PIN,
                                  RIGHT_JOY_PIN,
                                  DOWN_JOY_PIN,
                                  UP_JOY_PIN
                                 };

const uint8_t JOY_IRQn[JOYn] =   {SEL_JOY_EXTI_IRQn,
                                  LEFT_JOY_EXTI_IRQn,
                                  RIGHT_JOY_EXTI_IRQn,
                                  DOWN_JOY_EXTI_IRQn,
                                  UP_JOY_EXTI_IRQn
                                 };

/**
 * @brief BUS variables
 */
#if defined(HAL_I2C_MODULE_ENABLED)
uint32_t I2c1Timeout = DISCOVERY_I2C2_TIMEOUT_MAX;  /*<! Value of Timeout when I2C1 communication fails */
uint32_t I2c2Timeout = DISCOVERY_I2C2_TIMEOUT_MAX;  /*<! Value of Timeout when I2C2 communication fails */
static I2C_HandleTypeDef I2c1Handle;
static I2C_HandleTypeDef I2c2Handle;
#endif /* HAL_I2C_MODULE_ENABLED */

#if defined(HAL_SPI_MODULE_ENABLED)

/* LL definition */
#define __SPI_DIRECTION_2LINES(__HANDLE__)   do{\
                                             CLEAR_BIT((__HANDLE__)->Instance->CR1, SPI_CR1_RXONLY | SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE);\
                                             }while(0);

#define __SPI_DIRECTION_2LINES_RXONLY(__HANDLE__)   do{\
                                                   CLEAR_BIT((__HANDLE__)->Instance->CR1, SPI_CR1_RXONLY | SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE);\
                                                   SET_BIT((__HANDLE__)->Instance->CR1, SPI_CR1_RXONLY);\
                                                   }while(0);

#define __SPI_DIRECTION_1LINE_TX(__HANDLE__) do{\
                                             CLEAR_BIT((__HANDLE__)->Instance->CR1, SPI_CR1_RXONLY | SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE);\
                                             SET_BIT((__HANDLE__)->Instance->CR1, SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE);\
                                             }while(0);

#define __SPI_DIRECTION_1LINE_RX(__HANDLE__) do {\
                                             CLEAR_BIT((__HANDLE__)->Instance->CR1, SPI_CR1_RXONLY | SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE);\
                                             SET_BIT((__HANDLE__)->Instance->CR1, SPI_CR1_BIDIMODE);\
                                             } while(0);


uint32_t SpixTimeout = SPIx_TIMEOUT_MAX;            /*<! Value of Timeout when SPI communication fails */
static SPI_HandleTypeDef SpiHandle;
#endif /* HAL_SPI_MODULE_ENABLED */

/**
  * @}
  */

/** @defgroup STM32L476G_DISCOVERY_Private_FunctionPrototypes Private Functions
  * @{
  */
/**************************** Bus functions ************************************/
/* I2C2 bus function */
#if defined(HAL_I2C_MODULE_ENABLED)
static void               I2C2_Init(void);
static void               I2C2_MspInit(I2C_HandleTypeDef *hi2c);
static void               I2C2_DeInit(void);
static void               I2C2_MspDeInit(I2C_HandleTypeDef *hi2c);
static void               I2C2_WriteData(uint16_t Addr, uint16_t Reg, uint16_t RegSize, uint8_t Value);
static HAL_StatusTypeDef  I2C2_WriteBuffer(uint16_t Addr, uint16_t Reg, uint16_t RegSize, uint8_t *pBuffer, uint16_t Length);
static uint8_t            I2C2_ReadData(uint16_t Addr, uint16_t Reg, uint16_t RegSize);
static HAL_StatusTypeDef  I2C2_ReadBuffer(uint16_t Addr, uint16_t Reg, uint16_t RegSize, uint8_t *pBuffer, uint16_t Length);
static void               I2C2_Error(void);

static void               I2C1_Init(void);
static void               I2C1_MspInit(I2C_HandleTypeDef *hi2c);
static void               I2C1_DeInit(void);
static void               I2C1_MspDeInit(I2C_HandleTypeDef *hi2c);
static HAL_StatusTypeDef  I2C1_WriteBuffer(uint16_t Addr, uint16_t Reg, uint16_t RegSize, uint8_t *pBuffer, uint16_t Length);
static HAL_StatusTypeDef  I2C1_ReadBuffer(uint16_t Addr, uint16_t Reg, uint16_t RegSize, uint8_t *pBuffer, uint16_t Length);
static void               I2C1_Error(void);
#endif/* HAL_I2C_MODULE_ENABLED */

/* SPIx bus function */
#if defined(HAL_SPI_MODULE_ENABLED)
static void               SPIx_Init(void);
static void               SPIx_MspInit(SPI_HandleTypeDef *hspi);
static void               SPIx_DeInit(void);
static void               SPIx_MspDeInit(void);
static uint8_t            SPIx_WriteRead(uint8_t Byte);
static void               SPIx_Write(uint8_t byte);
static uint8_t            SPIx_Read(void);
#endif

/**************************** Link functions ***********************************/
#if defined(HAL_I2C_MODULE_ENABLED)
/* Link functions for EEPROM peripheral over I2C */
void                      EEPROM_I2C_IO_Init(void);
HAL_StatusTypeDef         EEPROM_I2C_IO_WriteData(uint16_t DevAddress, uint16_t MemAddress, uint8_t *pBuffer, uint32_t BufferSize);
HAL_StatusTypeDef         EEPROM_I2C_IO_ReadData(uint16_t DevAddress, uint16_t MemAddress, uint8_t *pBuffer, uint32_t BufferSize);
HAL_StatusTypeDef         EEPROM_I2C_IO_IsDeviceReady(uint16_t DevAddress, uint32_t Trials);

/* Link functions for Audio Codec peripheral */
void                      AUDIO_IO_Init(void);
void                      AUDIO_IO_DeInit(void);
void                      AUDIO_IO_Write(uint8_t Addr, uint8_t Reg, uint8_t Value);
uint8_t                   AUDIO_IO_Read(uint8_t Addr, uint8_t Reg);
void                      AUDIO_IO_Delay(uint32_t delay);
#endif/* HAL_I2C_MODULE_ENABLED */

#if defined(HAL_SPI_MODULE_ENABLED)
/* Link function for COMPASS / ACCELERO peripheral */
void                      ACCELERO_IO_Init(void);
void                      ACCELERO_IO_DeInit(void);
void                      ACCELERO_IO_ITConfig(void);
void                      ACCELERO_IO_Write(uint8_t RegisterAddr, uint8_t Value);
uint8_t                   ACCELERO_IO_Read(uint8_t RegisterAddr);

void                      MAGNETO_IO_Init(void);
void                      MAGNETO_IO_DeInit(void);
void                      MAGNETO_IO_ITConfig(void);
void                      MAGNETO_IO_Write(uint8_t RegisterAddr, uint8_t Value);
uint8_t                   MAGNETO_IO_Read(uint8_t RegisterAddr);


/* Link functions for GYRO peripheral */
void                      GYRO_IO_Init(void);
void                      GYRO_IO_DeInit(void);
void                      GYRO_IO_Write(uint8_t *pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite);
void                      GYRO_IO_Read(uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead);

#endif

#if defined(HAL_I2C_MODULE_ENABLED)
/*  Link functions IOExpander */
void                      IOE_Init(void);
void                      IOE_ITConfig(void);
void                      IOE_Delay(uint32_t Delay);
void                      IOE_Write(uint8_t Addr, uint8_t Reg, uint8_t Value);
uint8_t                   IOE_Read(uint8_t Addr, uint8_t Reg);
uint16_t                  IOE_ReadMultiple(uint8_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length);

/* Link functions for IDD measurement */
void                      MFX_IO_Init(void);
void                      MFX_IO_DeInit(void);
void                      MFX_IO_ITConfig(void);
void                      MFX_IO_EnableWakeupPin(void);
void                      MFX_IO_Wakeup(void);
void                      MFX_IO_Delay(uint32_t delay);
void                      MFX_IO_Write(uint16_t addr, uint8_t reg, uint8_t value);
uint8_t                   MFX_IO_Read(uint16_t addr, uint8_t reg);
void                      MFX_IO_WriteMultiple(uint16_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length);
uint16_t                  MFX_IO_ReadMultiple(uint16_t addr, uint8_t reg, uint8_t *buffer, uint16_t length);
#endif/* HAL_I2C_MODULE_ENABLED */
/**
  * @}
  */

/** @defgroup STM32L476G_DISCOVERY_Exported_Functions Exported Functions
  * @{
  */

/**
  * @brief  This method returns the STM32L476 DISCOVERY BSP Driver revision
  * @retval version : 0xXYZR (8bits for each decimal, R for RC)
  */
uint32_t BSP_GetVersion(void)
{
  return __STM32L476G_DISCOVERY_BSP_VERSION;
}

/**
  * @brief  This method returns the STM32L476 DISCOVERY supply mode
  * @retval Code of current supply mode
  *          This code can be one of following:
  *     @arg SUPPLY_MODE_EXTERNAL
  *     @arg SUPPLY_MODE_BATTERY
  */
SupplyMode_TypeDef BSP_SupplyModeDetection(void)
{
  SupplyMode_TypeDef supplymode = SUPPLY_MODE_ERROR;
  GPIO_InitTypeDef GPIO_InitStruct;

  BATTERY_DETECTION_GPIO_CLK_ENABLE();

  /* COMP GPIO pin configuration */
  GPIO_InitStruct.Pin = BATTERY_DETECTION_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(BATTERY_DETECTION_GPIO_PORT, &GPIO_InitStruct);

  HAL_Delay(400);
  if (HAL_GPIO_ReadPin(BATTERY_DETECTION_GPIO_PORT, GPIO_InitStruct.Pin) != GPIO_PIN_RESET)
  {
    supplymode = SUPPLY_MODE_EXTERNAL;
  }
  else
  {
    supplymode = SUPPLY_MODE_BATTERY;
  }

  HAL_GPIO_DeInit(BATTERY_DETECTION_GPIO_PORT, GPIO_InitStruct.Pin);

  return supplymode;
}

#if defined (USE_STM32L476G_DISCO_REVC) || defined (USE_STM32L476G_DISCO_REVB)
/**
  * @brief  Configures LED GPIOs.
  * @param  Led: Specifies the Led to be configured.
  *   This parameter can be one of following parameters:
  *     @arg LED4
  *     @arg LED5
  * @retval None
  */
#elif defined (USE_STM32L476G_DISCO_REVA)
/**
  * @brief  Configures LED GPIOs.
  * @param  Led: Specifies the Led to be configured.
  *   This parameter can be one of following parameters:
  *     @arg LED3
  *     @arg LED4
  * @retval None
  */
#endif
void BSP_LED_Init(Led_TypeDef Led)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  /* Enable the GPIO_LED clock */
  LEDx_GPIO_CLK_ENABLE(Led);

  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.Pin = LED_PIN[Led];
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

  HAL_GPIO_Init(LED_PORT[Led], &GPIO_InitStructure);

  HAL_GPIO_WritePin(LED_PORT[Led], GPIO_InitStructure.Pin, GPIO_PIN_RESET);
}

#if defined (USE_STM32L476G_DISCO_REVC) || defined (USE_STM32L476G_DISCO_REVB)
/**
  * @brief  Unconfigures LED GPIOs.
  * @param  Led: Specifies the Led to be unconfigured.
  *   This parameter can be one of following parameters:
  *     @arg LED4
  *     @arg LED5
  * @retval None
  */
#elif defined (USE_STM32L476G_DISCO_REVA)
/**
  * @brief  Unconfigures LED GPIOs.
  * @param  Led: Specifies the Led to be unconfigured.
  *   This parameter can be one of following parameters:
  *     @arg LED3
  *     @arg LED4
  * @retval None
  */
#endif
void BSP_LED_DeInit(Led_TypeDef Led)
{
  /* Enable the GPIO_LED clock */
  LEDx_GPIO_CLK_ENABLE(Led);

  HAL_GPIO_DeInit(LED_PORT[Led], LED_PIN[Led]);
}

#if defined (USE_STM32L476G_DISCO_REVC) || defined (USE_STM32L476G_DISCO_REVB)
/**
  * @brief  Turns selected LED On.
  * @param  Led: Specifies the Led to be set on.
  *   This parameter can be one of following parameters:
  *     @arg LED4
  *     @arg LED5
  * @retval None
  */
#elif defined (USE_STM32L476G_DISCO_REVA)
/**
  * @brief  Turns selected LED On.
  * @param  Led: Specifies the Led to be set on.
  *   This parameter can be one of following parameters:
  *     @arg LED3
  *     @arg LED4
  * @retval None
  */
#endif
void BSP_LED_On(Led_TypeDef Led)
{
  HAL_GPIO_WritePin(LED_PORT[Led], LED_PIN[Led], GPIO_PIN_SET);
}

#if defined (USE_STM32L476G_DISCO_REVC) || defined (USE_STM32L476G_DISCO_REVB)
/**
  * @brief  Turns selected LED Off.
  * @param  Led: Specifies the Led to be set off.
  *   This parameter can be one of following parameters:
  *     @arg LED4
  *     @arg LED5
  * @retval None
  */
#elif defined (USE_STM32L476G_DISCO_REVA)
/**
  * @brief  Turns selected LED Off.
  * @param  Led: Specifies the Led to be set off.
  *   This parameter can be one of following parameters:
  *     @arg LED3
  *     @arg LED4
  * @retval None
  */
#endif
void BSP_LED_Off(Led_TypeDef Led)
{
  HAL_GPIO_WritePin(LED_PORT[Led], LED_PIN[Led], GPIO_PIN_RESET);
}

#if defined (USE_STM32L476G_DISCO_REVC) || defined (USE_STM32L476G_DISCO_REVB)
/**
  * @brief  Toggles the selected LED.
  * @param  Led: Specifies the Led to be toggled.
  *   This parameter can be one of following parameters:
  *     @arg LED4
  *     @arg LED5
  * @retval None
  */
#elif defined (USE_STM32L476G_DISCO_REVA)
/**
  * @brief  Toggles the selected LED.
  * @param  Led: Specifies the Led to be toggled.
  *   This parameter can be one of following parameters:
  *     @arg LED3
  *     @arg LED4
  * @retval None
  */
#endif
void BSP_LED_Toggle(Led_TypeDef Led)
{
  HAL_GPIO_TogglePin(LED_PORT[Led], LED_PIN[Led]);
}

/**
  * @brief  Configures all buttons of the joystick in GPIO or EXTI modes.
  * @param  Joy_Mode: Joystick mode.
  *    This parameter can be one of the following values:
  *     @arg  JOY_MODE_GPIO: Joystick pins will be used as simple IOs
  *     @arg  JOY_MODE_EXTI: Joystick pins will be connected to EXTI line
  *                                 with interrupt generation capability
  * @retval HAL_OK: if all initializations are OK. Other value if error.
  */
uint8_t BSP_JOY_Init(JOYMode_TypeDef Joy_Mode)
{
  JOYState_TypeDef joykey;
  GPIO_InitTypeDef GPIO_InitStruct;

  /* Initialized the Joystick. */
  for (joykey = JOY_SEL; joykey < (JOY_SEL + JOYn) ; joykey++)
  {
    /* Enable the JOY clock */
    JOYx_GPIO_CLK_ENABLE(joykey);

    GPIO_InitStruct.Pin = JOY_PIN[joykey];
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    if (Joy_Mode == JOY_MODE_GPIO)
    {
      /* Configure Joy pin as input */
      GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
      HAL_GPIO_Init(JOY_PORT[joykey], &GPIO_InitStruct);
    }
    else if (Joy_Mode == JOY_MODE_EXTI)
    {
      /* Configure Joy pin as input with External interrupt */
      GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
      HAL_GPIO_Init(JOY_PORT[joykey], &GPIO_InitStruct);

      /* Enable and set Joy EXTI Interrupt to the lowest priority */
      HAL_NVIC_SetPriority((IRQn_Type)(JOY_IRQn[joykey]), 0x0F, 0x00);
      HAL_NVIC_EnableIRQ((IRQn_Type)(JOY_IRQn[joykey]));
    }
  }

  return HAL_OK;
}

/**
  * @brief  Unonfigures all GPIOs used as buttons of the joystick.
  * @retval None.
  */
void BSP_JOY_DeInit(void)
{
  JOYState_TypeDef joykey;

  /* Initialized the Joystick. */
  for (joykey = JOY_SEL; joykey < (JOY_SEL + JOYn) ; joykey++)
  {
    /* Enable the JOY clock */
    JOYx_GPIO_CLK_ENABLE(joykey);

    HAL_GPIO_DeInit(JOY_PORT[joykey], JOY_PIN[joykey]);
  }
}

/**
* @brief  Returns the current joystick status.
* @retval Code of the joystick key pressed
*          This code can be one of the following values:
*            @arg  JOY_NONE
*            @arg  JOY_SEL
*            @arg  JOY_DOWN
*            @arg  JOY_LEFT
*            @arg  JOY_RIGHT
*            @arg  JOY_UP
*/
JOYState_TypeDef BSP_JOY_GetState(void)
{
  JOYState_TypeDef joykey;

  for (joykey = JOY_SEL; joykey < (JOY_SEL + JOYn) ; joykey++)
  {
    if (HAL_GPIO_ReadPin(JOY_PORT[joykey], JOY_PIN[joykey]) == GPIO_PIN_SET)
    {
      /* Return Code Joystick key pressed */
      return joykey;
    }
  }

  /* No Joystick key pressed */
  return JOY_NONE;
}

/**
  * @}
  */

/** @defgroup STM32L476G_DISCOVERY_BusOperations_Functions Bus Operations Functions
  * @{
  */

/*******************************************************************************
                            BUS OPERATIONS
*******************************************************************************/
#if defined(HAL_SPI_MODULE_ENABLED)
/******************************* SPI Routines**********************************/
/**
  * @brief SPIx Bus initialization
  * @retval None
  */
static void SPIx_Init(void)
{
  if (HAL_SPI_GetState(&SpiHandle) == HAL_SPI_STATE_RESET)
  {
    /* SPI Config */
    SpiHandle.Instance = DISCOVERY_SPIx;
    /* SPI baudrate is set to 10 MHz (PCLK1/SPI_BaudRatePrescaler = 80/8 = 10 MHz)
      to verify these constraints:
      lsm303c SPI interface max baudrate is 10MHz for write/read
      PCLK1 max frequency is set to 80 MHz
      */
    SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    SpiHandle.Init.Direction = SPI_DIRECTION_2LINES;
    SpiHandle.Init.CLKPhase = SPI_PHASE_1EDGE;
    SpiHandle.Init.CLKPolarity = SPI_POLARITY_LOW;
    SpiHandle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    SpiHandle.Init.CRCPolynomial = 7;
    SpiHandle.Init.DataSize = SPI_DATASIZE_8BIT;
    SpiHandle.Init.FirstBit = SPI_FIRSTBIT_MSB;
    SpiHandle.Init.NSS = SPI_NSS_SOFT;
    SpiHandle.Init.TIMode = SPI_TIMODE_DISABLE;
    SpiHandle.Init.Mode = SPI_MODE_MASTER;

    SPIx_MspInit(&SpiHandle);
    HAL_SPI_Init(&SpiHandle);
  }
}

/**
  * @brief SPI MSP Init
  * @param hspi: SPI handle
  * @retval None
  */
static void SPIx_MspInit(SPI_HandleTypeDef *hspi)
{
  GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable SPIx clock  */
  DISCOVERY_SPIx_CLOCK_ENABLE();

  /* enable SPIx gpio clock */
  DISCOVERY_SPIx_GPIO_CLK_ENABLE();

  /* configure SPIx SCK, MOSI and MISO */
  GPIO_InitStructure.Pin = (DISCOVERY_SPIx_SCK_PIN | DISCOVERY_SPIx_MOSI_PIN | DISCOVERY_SPIx_MISO_PIN);
  GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Pull  = GPIO_NOPULL; // GPIO_PULLDOWN;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStructure.Alternate = DISCOVERY_SPIx_AF;
  HAL_GPIO_Init(DISCOVERY_SPIx_GPIO_PORT, &GPIO_InitStructure);
}

/**
  * @brief SPIx Bus Deinitialization
  * @retval None
  */
void SPIx_DeInit(void)
{
  if (HAL_SPI_GetState(&SpiHandle) != HAL_SPI_STATE_RESET)
  {
    /* SPI Deinit */
    HAL_SPI_DeInit(&SpiHandle);
    SPIx_MspDeInit();
  }
}

/**
  * @brief SPI MSP DeInit
  * @retval None
  */
static void SPIx_MspDeInit(void)
{
  /* enable SPIx gpio clock */
  DISCOVERY_SPIx_GPIO_CLK_ENABLE();

  /* Unconfigure SPIx SCK, MOSI and MISO */
  HAL_GPIO_DeInit(DISCOVERY_SPIx_GPIO_PORT, (DISCOVERY_SPIx_SCK_PIN | DISCOVERY_SPIx_MOSI_PIN | DISCOVERY_SPIx_MISO_PIN));

  DISCOVERY_SPIx_GPIO_FORCE_RESET();
  DISCOVERY_SPIx_GPIO_RELEASE_RESET();

  /* Disable SPIx clock  */
  DISCOVERY_SPIx_CLOCK_DISABLE();
}

/**
  * @brief  Sends a Byte through the SPI interface and return the Byte received
  *         from the SPI bus.
  * @param  Byte : Byte send.
  * @retval none.
  */
static uint8_t SPIx_WriteRead(uint8_t Byte)
{
  uint8_t receivedbyte;

  /* Enable the SPI */
  __HAL_SPI_ENABLE(&SpiHandle);
  /* check TXE flag */
  while ((SpiHandle.Instance->SR & SPI_FLAG_TXE) != SPI_FLAG_TXE);

  /* Write the data */
  *((__IO uint8_t *)&SpiHandle.Instance->DR) = Byte;

  while ((SpiHandle.Instance->SR & SPI_FLAG_RXNE) != SPI_FLAG_RXNE);
  receivedbyte = *((__IO uint8_t *)&SpiHandle.Instance->DR);

  /* Wait BSY flag */
  while ((SpiHandle.Instance->SR & SPI_FLAG_FTLVL) != SPI_FTLVL_EMPTY);
  while ((SpiHandle.Instance->SR & SPI_FLAG_BSY) == SPI_FLAG_BSY);

  /* disable the SPI */
  __HAL_SPI_DISABLE(&SpiHandle);

  return receivedbyte;
}

/**
  * @brief  Sends a Byte through the SPI interface.
  * @param  Byte : Byte to send.
  * @retval none.
  */
static void SPIx_Write(uint8_t Byte)
{
  /* Enable the SPI */
  __HAL_SPI_ENABLE(&SpiHandle);
  /* check TXE flag */
  while ((SpiHandle.Instance->SR & SPI_FLAG_TXE) != SPI_FLAG_TXE);

  /* Write the data */
  *((__IO uint8_t *)&SpiHandle.Instance->DR) = Byte;

  /* Wait BSY flag */
  while ((SpiHandle.Instance->SR & SPI_FLAG_BSY) == SPI_FLAG_BSY);

  /* disable the SPI */
  __HAL_SPI_DISABLE(&SpiHandle);
}

#if defined(__ICCARM__)
#pragma optimize=none
#endif
/**
  * @brief  Receives a Byte from the SPI bus.
  * @retval The received byte value
  */
static uint8_t SPIx_Read(void)
{
  uint8_t receivedbyte;

  __HAL_SPI_ENABLE(&SpiHandle);
  __DSB();
  __DSB();
  __DSB();
  __DSB();
  __DSB();
  __DSB();
  __DSB();
  __DSB();
  __HAL_SPI_DISABLE(&SpiHandle);

  while ((SpiHandle.Instance->SR & SPI_FLAG_RXNE) != SPI_FLAG_RXNE);
  /* read the received data */
  receivedbyte = *(__IO uint8_t *)&SpiHandle.Instance->DR;

  /* Wait for the BSY flag reset */
  while ((SpiHandle.Instance->SR & SPI_FLAG_BSY) == SPI_FLAG_BSY);


  return receivedbyte;
}
#endif /* HAL_SPI_MODULE_ENABLED */


#if defined(HAL_I2C_MODULE_ENABLED)
/******************************* I2C Routines**********************************/
/**
  * @brief Discovery I2C1 Bus initialization
  * @retval None
  */
static void I2C1_Init(void)
{
  if (HAL_I2C_GetState(&I2c1Handle) == HAL_I2C_STATE_RESET)
  {
    I2c1Handle.Instance              = DISCOVERY_I2C1;
    I2c1Handle.Init.Timing           = DISCOVERY_I2C1_TIMING;
    I2c1Handle.Init.OwnAddress1      = 0;
    I2c1Handle.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    I2c1Handle.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    I2c1Handle.Init.OwnAddress2      = 0;
    I2c1Handle.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    I2c1Handle.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;

    /* Init the I2C */
    I2C1_MspInit(&I2c1Handle);
    HAL_I2C_Init(&I2c1Handle);
  }
}

/**
  * @brief Discovery I2C1 MSP Initialization
  * @param hi2c: I2C handle
  * @retval None
  */
static void I2C1_MspInit(I2C_HandleTypeDef *hi2c)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_PeriphCLKInitTypeDef  RCC_PeriphCLKInitStruct;

  /* IOSV bit MUST be set to access GPIO port G[2:15] */
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();

  if (hi2c->Instance == DISCOVERY_I2C1)
  {
    /*##-1- Configure the Discovery I2C clock source. The clock is derived from the SYSCLK #*/
    RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
    RCC_PeriphCLKInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
    HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);

    /*##-2- Configure the GPIOs ################################################*/
    /* Enable GPIO clock */
    DISCOVERY_I2C1_SDA_GPIO_CLK_ENABLE();
    DISCOVERY_I2C1_SCL_GPIO_CLK_ENABLE();

    /* Configure I2C Rx/Tx as alternate function  */
    GPIO_InitStructure.Pin       = DISCOVERY_I2C1_SCL_PIN | DISCOVERY_I2C1_SDA_PIN;
    GPIO_InitStructure.Mode      = GPIO_MODE_AF_OD;
    GPIO_InitStructure.Pull      = GPIO_PULLUP;
    GPIO_InitStructure.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStructure.Alternate = DISCOVERY_I2C1_SCL_SDA_AF;
    HAL_GPIO_Init(DISCOVERY_I2C1_SCL_GPIO_PORT, &GPIO_InitStructure);

    /*##-3- Configure the Discovery I2C1 peripheral #######################################*/
    /* Enable Discovery I2C1 clock */
    DISCOVERY_I2C1_CLK_ENABLE();

    /* Force and release the I2C Peripheral Clock Reset */
    DISCOVERY_I2C1_FORCE_RESET();
    DISCOVERY_I2C1_RELEASE_RESET();

    /* Enable and set Discovery I2C1 Interrupt to the highest priority */
    HAL_NVIC_SetPriority(DISCOVERY_I2C1_EV_IRQn, 0x00, 0);
    HAL_NVIC_EnableIRQ(DISCOVERY_I2C1_EV_IRQn);

    /* Enable and set Discovery I2C1 Interrupt to the highest priority */
    HAL_NVIC_SetPriority(DISCOVERY_I2C1_ER_IRQn, 0x00, 0);
    HAL_NVIC_EnableIRQ(DISCOVERY_I2C1_ER_IRQn);
  }
}

/**
  * @brief Discovery I2C1 Bus Deitialization
  * @retval None
  */
static void I2C1_DeInit(void)
{
  if (HAL_I2C_GetState(&I2c1Handle) != HAL_I2C_STATE_RESET)
  {
    /* Deinit the I2C */
    HAL_I2C_DeInit(&I2c1Handle);
    I2C1_MspDeInit(&I2c1Handle);
  }
}

/**
  * @brief Discovery I2C1 MSP Deinitialization
  * @param hi2c: I2C handle
  * @retval None
  */
static void I2C1_MspDeInit(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == DISCOVERY_I2C1)
  {
    /*##-1- Unconfigure the GPIOs ################################################*/
    /* Enable GPIO clock */
    DISCOVERY_I2C1_SDA_GPIO_CLK_ENABLE();
    DISCOVERY_I2C1_SCL_GPIO_CLK_ENABLE();

    /* Deinit Rx/Tx pins */
    HAL_GPIO_DeInit(DISCOVERY_I2C1_SCL_GPIO_PORT, (DISCOVERY_I2C1_SCL_PIN | DISCOVERY_I2C1_SDA_PIN));

    /*##-2- Unconfigure the Discovery I2C1 peripheral ############################*/
    /* Force & Release the I2C Peripheral Clock Reset */
    DISCOVERY_I2C1_FORCE_RESET();
    DISCOVERY_I2C1_RELEASE_RESET();

    /* Disable Discovery I2C1 clock */
    DISCOVERY_I2C1_CLK_DISABLE();

    /* Disable Discovery I2C1 interrupts */
    HAL_NVIC_DisableIRQ(DISCOVERY_I2C1_EV_IRQn);
    HAL_NVIC_DisableIRQ(DISCOVERY_I2C1_ER_IRQn);

    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWREx_DisableVddIO2();
  }
}

/**
  * @brief  Write a value in a register of the device through BUS.
  * @param  Addr: Device address on BUS Bus.
  * @param  Reg: The target register address to write
  * @param  RegSize: The target register size (can be 8BIT or 16BIT)
  * @param  pBuffer: The target register value to be written
  * @param  Length: buffer size to be written
  * @retval None
  */
static HAL_StatusTypeDef I2C1_WriteBuffer(uint16_t Addr, uint16_t Reg, uint16_t RegSize, uint8_t *pBuffer, uint16_t Length)
{
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_I2C_Mem_Write(&I2c1Handle, Addr, (uint16_t)Reg, RegSize, pBuffer, Length, I2c1Timeout);

  /* Check the communication status */
  if (status != HAL_OK)
  {
    /* Re-Initiaize the BUS */
    I2C1_Error();
  }
  return status;
}

/**
  * @brief  Reads multiple data on the BUS.
  * @param  Addr: I2C Address
  * @param  Reg: Reg Address
  * @param  RegSize : The target register size (can be 8BIT or 16BIT)
  * @param  pBuffer: pointer to read data buffer
  * @param  Length: length of the data
  * @retval 0 if no problems to read multiple data
  */
static HAL_StatusTypeDef I2C1_ReadBuffer(uint16_t Addr, uint16_t Reg, uint16_t RegSize, uint8_t *pBuffer, uint16_t Length)
{
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_I2C_Mem_Read(&I2c1Handle, Addr, (uint16_t)Reg, RegSize, pBuffer, Length, I2c1Timeout);

  /* Check the communication status */
  if (status != HAL_OK)
  {
    /* Re-Initiaize the BUS */
    I2C1_Error();
  }
  return status;
}

/**
  * @brief Discovery I2C1 error treatment function
  * @retval None
  */
static void I2C1_Error(void)
{
  /* De-initialize the I2C communication BUS */
  HAL_I2C_DeInit(&I2c1Handle);

  /* Re- Initiaize the I2C communication BUS */
  I2C1_Init();
}

/**
  * @brief Discovery I2C2 Bus initialization
  * @retval None
  */
static void I2C2_Init(void)
{
  if (HAL_I2C_GetState(&I2c2Handle) == HAL_I2C_STATE_RESET)
  {
    I2c2Handle.Instance              = DISCOVERY_I2C2;
    I2c2Handle.Init.Timing           = DISCOVERY_I2C2_TIMING;
    I2c2Handle.Init.OwnAddress1      = 0;
    I2c2Handle.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    I2c2Handle.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    I2c2Handle.Init.OwnAddress2      = 0;
    I2c2Handle.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    I2c2Handle.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;

    /* Init the I2C */
    I2C2_MspInit(&I2c2Handle);
    HAL_I2C_Init(&I2c2Handle);
  }
}

/**
  * @brief Discovery I2C2 MSP Initialization
  * @param hi2c: I2C2 handle
  * @retval None
  */
static void I2C2_MspInit(I2C_HandleTypeDef *hi2c)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_PeriphCLKInitTypeDef  RCC_PeriphCLKInitStruct;

  if (hi2c->Instance == DISCOVERY_I2C2)
  {
    /*##-1- Configure the Discovery I2C2 clock source. The clock is derived from the SYSCLK #*/
    RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C2;
    RCC_PeriphCLKInitStruct.I2c2ClockSelection = RCC_I2C2CLKSOURCE_SYSCLK;
    HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);

    /*##-2- Configure the GPIOs ################################################*/
    /* Enable GPIO clock */
    DISCOVERY_I2C2_SDA_GPIO_CLK_ENABLE();
    DISCOVERY_I2C2_SCL_GPIO_CLK_ENABLE();

    /* Configure I2C Rx/Tx as alternate function  */
    GPIO_InitStructure.Pin       = DISCOVERY_I2C2_SCL_PIN | DISCOVERY_I2C2_SDA_PIN;
    GPIO_InitStructure.Mode      = GPIO_MODE_AF_OD;
    GPIO_InitStructure.Pull      = GPIO_PULLUP;
    GPIO_InitStructure.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStructure.Alternate = DISCOVERY_I2C2_SCL_SDA_AF;
    HAL_GPIO_Init(DISCOVERY_I2C2_SCL_GPIO_PORT, &GPIO_InitStructure);

    /*##-3- Configure the Discovery I2C2 peripheral #############################*/
    /* Enable Discovery_I2C2 clock */
    DISCOVERY_I2C2_CLK_ENABLE();

    /* Force and release the I2C Peripheral Clock Reset */
    DISCOVERY_I2C2_FORCE_RESET();
    DISCOVERY_I2C2_RELEASE_RESET();

    /* Enable and set Discovery I2C2 Interrupt to the highest priority */
    HAL_NVIC_SetPriority(DISCOVERY_I2C2_EV_IRQn, 0x00, 0);
    HAL_NVIC_EnableIRQ(DISCOVERY_I2C2_EV_IRQn);

    /* Enable and set Discovery I2C2 Interrupt to the highest priority */
    HAL_NVIC_SetPriority(DISCOVERY_I2C2_ER_IRQn, 0x00, 0);
    HAL_NVIC_EnableIRQ(DISCOVERY_I2C2_ER_IRQn);
  }
}

/**
  * @brief Discovery I2C2 Bus Deinitialization
  * @retval None
  */
static void I2C2_DeInit(void)
{
  if (HAL_I2C_GetState(&I2c2Handle) != HAL_I2C_STATE_RESET)
  {
    /* DeInit the I2C */
    HAL_I2C_DeInit(&I2c2Handle);
    I2C2_MspDeInit(&I2c2Handle);
  }
}

/**
  * @brief Discovery I2C2 MSP DeInitialization
  * @param hi2c: I2C2 handle
  * @retval None
  */
static void I2C2_MspDeInit(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == DISCOVERY_I2C2)
  {
    /*##-1- Unconfigure the GPIOs ################################################*/
    /* Enable GPIO clock */
    DISCOVERY_I2C2_SDA_GPIO_CLK_ENABLE();
    DISCOVERY_I2C2_SCL_GPIO_CLK_ENABLE();

    /* Configure I2C Rx/Tx as alternate function  */
    HAL_GPIO_DeInit(DISCOVERY_I2C2_SCL_GPIO_PORT, (DISCOVERY_I2C2_SCL_PIN | DISCOVERY_I2C2_SDA_PIN));

    /*##-2- Unconfigure the Discovery I2C2 peripheral ############################*/
    /* Force and release I2C Peripheral */
    DISCOVERY_I2C2_FORCE_RESET();
    DISCOVERY_I2C2_RELEASE_RESET();

    /* Disable Discovery I2C2 clock */
    DISCOVERY_I2C2_CLK_DISABLE();

    /* Disable Discovery I2C2 interrupts */
    HAL_NVIC_DisableIRQ(DISCOVERY_I2C2_EV_IRQn);
    HAL_NVIC_DisableIRQ(DISCOVERY_I2C2_ER_IRQn);
  }
}

/**
  * @brief  Write a value in a register of the device through BUS.
  * @param  Addr: Device address on BUS Bus.
  * @param  Reg: The target register address to write
  * @param  RegSize: The target register size (can be 8BIT or 16BIT)
  * @param  Value: The target register value to be written
  * @retval None
  */
static void I2C2_WriteData(uint16_t Addr, uint16_t Reg, uint16_t RegSize, uint8_t Value)
{
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_I2C_Mem_Write(&I2c2Handle, Addr, (uint16_t)Reg, RegSize, &Value, 1, I2c2Timeout);

  /* Check the communication status */
  if (status != HAL_OK)
  {
    /* Re-Initiaize the BUS */
    I2C2_Error();
  }
}

/**
  * @brief  Write a value in a register of the device through BUS.
  * @param  Addr: Device address on BUS Bus.
  * @param  Reg: The target register address to write
  * @param  RegSize: The target register size (can be 8BIT or 16BIT)
  * @param  pBuffer: The target register value to be written
  * @param  Length: buffer size to be written
  * @retval None
  */
static HAL_StatusTypeDef I2C2_WriteBuffer(uint16_t Addr, uint16_t Reg, uint16_t RegSize, uint8_t *pBuffer, uint16_t Length)
{
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_I2C_Mem_Write(&I2c2Handle, Addr, (uint16_t)Reg, RegSize, pBuffer, Length, I2c2Timeout);

  /* Check the communication status */
  if (status != HAL_OK)
  {
    /* Re-Initiaize the BUS */
    I2C2_Error();
  }

  return status;
}

/**
  * @brief  Read a register of the device through BUS
  * @param  Addr: Device address on BUS
  * @param  Reg: The target register address to read
  * @param  RegSize: The target register size (can be 8BIT or 16BIT)
  * @retval read register value
  */
static uint8_t I2C2_ReadData(uint16_t Addr, uint16_t Reg, uint16_t RegSize)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint8_t value = 0x0;

  status = HAL_I2C_Mem_Read(&I2c2Handle, Addr, Reg, RegSize, &value, 1, I2c2Timeout);

  /* Check the communication status */
  if (status != HAL_OK)
  {
    /* Re-Initiaize the BUS */
    I2C2_Error();
  }

  return value;
}

/**
  * @brief  Reads multiple data on the BUS.
  * @param  Addr: I2C Address
  * @param  Reg: Reg Address
  * @param  RegSize : The target register size (can be 8BIT or 16BIT)
  * @param  pBuffer: pointer to read data buffer
  * @param  Length: length of the data
  * @retval 0 if no problems to read multiple data
  */
static HAL_StatusTypeDef I2C2_ReadBuffer(uint16_t Addr, uint16_t Reg, uint16_t RegSize, uint8_t *pBuffer, uint16_t Length)
{
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_I2C_Mem_Read(&I2c2Handle, Addr, (uint16_t)Reg, RegSize, pBuffer, Length, I2c2Timeout);

  /* Check the communication status */
  if (status != HAL_OK)
  {
    /* Re-Initiaize the BUS */
    I2C2_Error();
  }

  return status;
}

/**
  * @brief Discovery I2C2 error treatment function
  * @retval None
  */
static void I2C2_Error(void)
{
  /* De-initialize the I2C communication BUS */
  HAL_I2C_DeInit(&I2c2Handle);

  /* Re- Initiaize the I2C communication BUS */
  I2C2_Init();
}
#endif /*HAL_I2C_MODULE_ENABLED*/


/*******************************************************************************
                            LINK OPERATIONS
*******************************************************************************/
#if defined(HAL_SPI_MODULE_ENABLED)
/*********************** LINK ACCELEROMETER ***********************************/
/**
  * @brief  Configures COMPASS/ACCELEROMETER io interface.
  * @retval None
  */
void ACCELERO_IO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable CS GPIO clock and  Configure GPIO PIN for Gyroscope Chip select */
  ACCELERO_CS_GPIO_CLK_ENABLE();
  GPIO_InitStructure.Pin = ACCELERO_CS_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull  = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(ACCELERO_CS_GPIO_PORT, &GPIO_InitStructure);

  /* Deselect : Chip Select high */
  ACCELERO_CS_HIGH();

  SPIx_Init();
}

/**
  * @brief  De-Configures COMPASS/ACCELEROMETER io interface.
  * @retval None
  */
void ACCELERO_IO_DeInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable CS GPIO clock and  Configure GPIO PIN for Gyroscope Chip select */
  ACCELERO_CS_GPIO_CLK_ENABLE();
  GPIO_InitStructure.Pin = ACCELERO_CS_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull  = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(ACCELERO_CS_GPIO_PORT, &GPIO_InitStructure);

  /* Deselect : Chip Select high */
  ACCELERO_CS_HIGH();

  /* Uninitialize SPI bus */
  SPIx_DeInit();
}

/**
  * @brief  Configures COMPASS / ACCELERO click IT
  * @retval None
  */
void ACCELERO_IO_ITConfig(void)
{
}

/**
  * @brief  Writes one byte to the COMPASS / ACCELEROMETER.
  * @param  RegisterAddr specifies the COMPASS / ACCELEROMETER register to be written.
  * @param  Value : Data to be written
  * @retval   None
 */
void ACCELERO_IO_Write(uint8_t RegisterAddr, uint8_t Value)
{
  ACCELERO_CS_LOW();
  __SPI_DIRECTION_1LINE_TX(&SpiHandle);
  /* call SPI Read data bus function */
  SPIx_Write(RegisterAddr);
  SPIx_Write(Value);
  ACCELERO_CS_HIGH();
}

/**
  * @brief  Reads a block of data from the COMPASS / ACCELEROMETER.
  * @param  RegisterAddr : specifies the COMPASS / ACCELEROMETER internal address register to read from
  * @retval ACCELEROMETER register value
  */
uint8_t ACCELERO_IO_Read(uint8_t RegisterAddr)
{
  RegisterAddr = RegisterAddr | ((uint8_t)0x80);
  ACCELERO_CS_LOW();
  __SPI_DIRECTION_1LINE_TX(&SpiHandle);
  SPIx_Write(RegisterAddr);
  __SPI_DIRECTION_1LINE_RX(&SpiHandle);
  uint8_t val = SPIx_Read();
  ACCELERO_CS_HIGH();
  return val;
}

/********************************* LINK MAGNETO *******************************/
/**
  * @brief  Configures COMPASS/MAGNETO SPI interface.
  * @retval None
  */
void MAGNETO_IO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable CS GPIO clock and  Configure GPIO PIN for Gyroscope Chip select */
  MAGNETO_CS_GPIO_CLK_ENABLE();
  GPIO_InitStructure.Pin = MAGNETO_CS_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull  = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(MAGNETO_CS_GPIO_PORT, &GPIO_InitStructure);

  /* Deselect : Chip Select high */
  MAGNETO_CS_HIGH();

  SPIx_Init();
}

/**
  * @brief  de-Configures COMPASS/MAGNETO SPI interface.
  * @retval None
  */
void MAGNETO_IO_DeInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable CS GPIO clock and  Configure GPIO PIN for Gyroscope Chip select */
  MAGNETO_CS_GPIO_CLK_ENABLE();
  GPIO_InitStructure.Pin = MAGNETO_CS_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull  = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(MAGNETO_CS_GPIO_PORT, &GPIO_InitStructure);

  /* Deselect : Chip Select high */
  MAGNETO_CS_HIGH();

  HAL_GPIO_DeInit(MAGNETO_CS_GPIO_PORT, MAGNETO_INT1_PIN | MAGNETO_DRDY_PIN);


  /* Uninitialize SPI bus */
  SPIx_DeInit();
}

/**
  * @brief  Writes one byte to the COMPASS/MAGNETO.
  * @param  RegisterAddr specifies the COMPASS/MAGNETO register to be written.
  * @param  Value : Data to be written
  * @retval   None
 */
void MAGNETO_IO_Write(uint8_t RegisterAddr, uint8_t Value)
{
  MAGNETO_CS_LOW();
  __SPI_DIRECTION_1LINE_TX(&SpiHandle);
  /* call SPI Read data bus function */
  SPIx_Write(RegisterAddr);
  SPIx_Write(Value);
  MAGNETO_CS_HIGH();
}

/**
  * @brief  Reads a block of data from the COMPASS/MAGNETO.
  * @param  RegisterAddr : specifies the COMPASS/MAGNETO internal address register to read from
  * @retval ACCELEROMETER register value
  */
uint8_t MAGNETO_IO_Read(uint8_t RegisterAddr)
{
  MAGNETO_CS_LOW();
  __SPI_DIRECTION_1LINE_TX(&SpiHandle);
  SPIx_Write(RegisterAddr | 0x80);
  __SPI_DIRECTION_1LINE_RX(&SpiHandle);
  uint8_t val = SPIx_Read();
  MAGNETO_CS_HIGH();
  return val;
}

/********************************* LINK GYRO *****************************/
/**
  * @brief  Configures the GYRO SPI interface.
  * @retval None
  */
void GYRO_IO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;


  /* Case GYRO not used in the demonstration software except being set in
     low power mode.
     To avoid access conflicts with accelerometer and magnetometer,
     initialize  XL_CS and MAG_CS pins then deselect these I/O */
  ACCELERO_CS_GPIO_CLK_ENABLE();
  GPIO_InitStructure.Pin = ACCELERO_CS_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull  = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(ACCELERO_CS_GPIO_PORT, &GPIO_InitStructure);

  /* Deselect : Chip Select high */
  ACCELERO_CS_HIGH();

  /* Enable CS GPIO clock and  Configure GPIO PIN for Gyroscope Chip select */
  MAGNETO_CS_GPIO_CLK_ENABLE();
  GPIO_InitStructure.Pin = MAGNETO_CS_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull  = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(MAGNETO_CS_GPIO_PORT, &GPIO_InitStructure);

  /* Deselect : Chip Select high */
  MAGNETO_CS_HIGH();


  /* Configure the Gyroscope Control pins ---------------------------------*/
  /* Enable CS GPIO clock and  Configure GPIO PIN for Gyroscope Chip select */
  GYRO_CS_GPIO_CLK_ENABLE();
  GPIO_InitStructure.Pin = GYRO_CS_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull  = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GYRO_CS_GPIO_PORT, &GPIO_InitStructure);

  /* Deselect : Chip Select high */
  GYRO_CS_HIGH();

  /* Enable INT1, INT2 GPIO clock and Configure GPIO PINs to detect Interrupts */
  GYRO_INT1_GPIO_CLK_ENABLE();
  GPIO_InitStructure.Pin = GYRO_INT1_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GYRO_INT1_GPIO_PORT, &GPIO_InitStructure);

  GYRO_INT2_GPIO_CLK_ENABLE();
  GPIO_InitStructure.Pin = GYRO_INT2_PIN;
  HAL_GPIO_Init(GYRO_INT2_GPIO_PORT, &GPIO_InitStructure);

  SPIx_Init();

}


/**
  * @brief  de-Configures GYRO SPI interface.
  * @retval None
  */
void GYRO_IO_DeInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  /* Enable CS GPIO clock */
  GYRO_CS_GPIO_CLK_ENABLE();

  GPIO_InitStructure.Pin = GYRO_CS_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull  = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GYRO_CS_GPIO_PORT, &GPIO_InitStructure);

  /* Deselect : Chip Select high */
  GYRO_CS_HIGH();

  GYRO_INT1_GPIO_CLK_ENABLE();
  GYRO_INT2_GPIO_CLK_ENABLE();

  /* Uninitialize the INT1/INT2 Pins */
  HAL_GPIO_DeInit(GYRO_INT1_GPIO_PORT, GYRO_INT1_PIN);
  HAL_GPIO_DeInit(GYRO_INT2_GPIO_PORT, GYRO_INT2_PIN);

  /* Uninitialize SPI bus */
  SPIx_DeInit();
}

/**
  * @brief  Writes one byte to the GYRO.
  * @param  pBuffer : pointer to the buffer  containing the data to be written to the GYRO.
  * @param  WriteAddr : GYRO's internal address to write to.
  * @param  NumByteToWrite: Number of bytes to write.
  * @retval None
  */
void GYRO_IO_Write(uint8_t *pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite)
{
  /* Configure the MS bit:
       - When 0, the address will remain unchanged in multiple read/write commands.
       - When 1, the address will be auto incremented in multiple read/write commands.
  */
  if (NumByteToWrite > 0x01)
  {
    WriteAddr |= (uint8_t)MULTIPLEBYTE_CMD;
  }
  /* Set chip select Low at the start of the transmission */
  GYRO_CS_LOW();
  __SPI_DIRECTION_2LINES(&SpiHandle);

  /* Send the Address of the indexed register */
  SPIx_WriteRead(WriteAddr);

  /* Send the data that will be written into the device (MSB First) */
  while (NumByteToWrite >= 0x01)
  {
    SPIx_WriteRead(*pBuffer);
    NumByteToWrite--;
    pBuffer++;
  }

  /* Set chip select High at the end of the transmission */
  GYRO_CS_HIGH();
}

/**
  * @brief  Reads a block of data from the GYROSCOPE.
  * @param  pBuffer : pointer to the buffer that receives the data read from the GYROSCOPE.
  * @param  ReadAddr : GYROSCOPE's internal address to read from.
  * @param  NumByteToRead : number of bytes to read from the GYROSCOPE.
  * @retval None
  */
void GYRO_IO_Read(uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{
  if (NumByteToRead > 0x01)
  {
    ReadAddr |= (uint8_t)(READWRITE_CMD | MULTIPLEBYTE_CMD);
  }
  else
  {
    ReadAddr |= (uint8_t)READWRITE_CMD;
  }
  /* Set chip select Low at the start of the transmission */
  GYRO_CS_LOW();
  __SPI_DIRECTION_2LINES(&SpiHandle);
  /* Send the Address of the indexed register */
  SPIx_WriteRead(ReadAddr);

  /* Receive the data that will be read from the device (MSB First) */
  while (NumByteToRead > 0x00)
  {
    /* Send dummy byte (0x00) to generate the SPI clock to GYROSCOPE (Slave device) */
    *pBuffer = SPIx_WriteRead(0x00);
    NumByteToRead--;
    pBuffer++;
  }

  /* Set chip select High at the end of the transmission */
  GYRO_CS_HIGH();
}
#endif /* HAL_SPI_MODULE_ENABLED */

#if defined(HAL_I2C_MODULE_ENABLED)
/********************************* LINK MFX ***********************************/
/**
  * @brief  Initializes MFX low level.
  * @retval None
  */
void MFX_IO_Init(void)
{
  /* I2C2 init */
  I2C2_Init();
}
/**
  * @brief  Deinitializes MFX low level.
  * @retval None
  */
void MFX_IO_DeInit(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  /* Enable wakeup gpio clock */
  IDD_WAKEUP_GPIO_CLK_ENABLE();

  /* MFX wakeup pin configuration */
  GPIO_InitStruct.Pin   = IDD_WAKEUP_PIN;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
  HAL_GPIO_Init(IDD_WAKEUP_GPIO_PORT, &GPIO_InitStruct);

  /* DeInit interrupt pin : disable IRQ before to avoid spurious interrupt */
  HAL_NVIC_DisableIRQ((IRQn_Type)(IDD_INT_EXTI_IRQn));
  IDD_INT_GPIO_CLK_ENABLE();
  HAL_GPIO_DeInit(IDD_INT_GPIO_PORT, IDD_INT_PIN);

  /* I2C2 Deinit */
  I2C2_DeInit();
}

/**
  * @brief  Configures MFX low level interrupt.
  * @retval None
  */
void MFX_IO_ITConfig(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  /* Enable the GPIO clock */
  IDD_INT_GPIO_CLK_ENABLE();

  /* MFX_OUT_IRQ (normally used for EXTI_WKUP) */
  GPIO_InitStruct.Pin   = IDD_INT_PIN;
  GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Mode  = GPIO_MODE_IT_RISING;
  HAL_GPIO_Init(IDD_INT_GPIO_PORT, &GPIO_InitStruct);

  /* Enable and set GPIO EXTI Interrupt to the lowest priority */
  HAL_NVIC_SetPriority((IRQn_Type)(IDD_INT_EXTI_IRQn), 0x0F, 0x0F);
  HAL_NVIC_EnableIRQ((IRQn_Type)(IDD_INT_EXTI_IRQn));
}

/**
  * @brief  Configures MFX wke up  pin.
  * @retval None
  */
void MFX_IO_EnableWakeupPin(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  /* Enable wakeup gpio clock */
  IDD_WAKEUP_GPIO_CLK_ENABLE();

  /* MFX wakeup pin configuration */
  GPIO_InitStruct.Pin   = IDD_WAKEUP_PIN;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  HAL_GPIO_Init(IDD_WAKEUP_GPIO_PORT, &GPIO_InitStruct);
}

/**
  * @brief  Wakeup MFX.
  * @retval None
  */
void MFX_IO_Wakeup(void)
{
  /* Set Wakeup pin to high to wakeup Idd measurement component from standby mode */
  HAL_GPIO_WritePin(IDD_WAKEUP_GPIO_PORT, IDD_WAKEUP_PIN, GPIO_PIN_SET);

  /* Wait */
  HAL_Delay(1);

  /* Set gpio pin basck to low */
  HAL_GPIO_WritePin(IDD_WAKEUP_GPIO_PORT, IDD_WAKEUP_PIN, GPIO_PIN_RESET);
}

/**
  * @brief  MFX writes single data.
  * @param  Addr: I2C address
  * @param  Reg: Register address
  * @param  Value: Data to be written
  * @retval None
  */
void MFX_IO_Write(uint16_t Addr, uint8_t Reg, uint8_t Value)
{
  I2C2_WriteData(Addr, Reg, I2C_MEMADD_SIZE_8BIT, Value);
}

/**
  * @brief  MFX reads single data.
  * @param  Addr: I2C address
  * @param  Reg: Register address
  * @retval Read data
  */
uint8_t MFX_IO_Read(uint16_t Addr, uint8_t Reg)
{
  return I2C2_ReadData(Addr, Reg, I2C_MEMADD_SIZE_8BIT);
}

/**
  * @brief  MFX reads multiple data.
  * @param  Addr: I2C address
  * @param  Reg: Register address
  * @param  Buffer: Pointer to data buffer
  * @param  Length: Length of the data
  * @retval Number of read data
  */
uint16_t MFX_IO_ReadMultiple(uint16_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length)
{
  return I2C2_ReadBuffer(Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, Buffer, Length);
}

/**
  * @brief  MFX writes multiple data.
  * @param  Addr: I2C address
  * @param  Reg: Register address
  * @param  Buffer: Pointer to data buffer
  * @param  Length: Length of the data
  * @retval None
  */
void MFX_IO_WriteMultiple(uint16_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length)
{
  I2C2_WriteBuffer(Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, Buffer, Length);
}

/**
  * @brief  MFX delay
  * @param  Delay: Delay in ms
  * @retval None
  */
void MFX_IO_Delay(uint32_t Delay)
{
  HAL_Delay(Delay);
}


/********************************* LINK AUDIO *********************************/
/**
  * @brief  Initializes Audio low level.
  * @retval None
  */
void AUDIO_IO_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  /* Enable Reset GPIO Clock */
  AUDIO_RESET_GPIO_CLK_ENABLE();

  /* Audio reset pin configuration */
  GPIO_InitStruct.Pin = AUDIO_RESET_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  HAL_GPIO_Init(AUDIO_RESET_GPIO, &GPIO_InitStruct);

  /* I2C bus init */
  I2C1_Init();

  /* Power Down the codec */
  CODEC_AUDIO_POWER_OFF();

  /* wait for a delay to insure registers erasing */
  HAL_Delay(5);

  /* Power on the codec */
  CODEC_AUDIO_POWER_ON();

  /* wait for a delay to insure registers erasing */
  HAL_Delay(5);
}

/**
  * @brief  Deinitializes Audio low level.
  * @retval None
  */
void AUDIO_IO_DeInit(void)                       /* TO DO */
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  /***********************************************************************/
  /* In case of battery-supplied powered, there is no audio codec-based
     features available. Set audio codec I/O default setting */
  /***********************************************************************/
  __HAL_RCC_GPIOE_CLK_ENABLE();
  GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP  ;
  GPIO_InitStruct.Pin       = (GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6);
  GPIO_InitStruct.Pull      = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET);

  /* I2C bus Deinit */
  I2C1_DeInit();
}

/**
  * @brief  Writes a single data.
  * @param  Addr: I2C address
  * @param  Reg: Reg address
  * @param  Value: Data to be written
  * @retval None
  */
void AUDIO_IO_Write(uint8_t Addr, uint8_t Reg, uint8_t Value)
{
  I2C1_WriteBuffer(Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, &Value, 1);
}

/**
  * @brief  Reads a single data.
  * @param  Addr: I2C address
  * @param  Reg: Reg address
  * @retval Data to be read
  */
uint8_t AUDIO_IO_Read(uint8_t Addr, uint8_t Reg)
{
  uint8_t Read_Value = 0;

  I2C1_ReadBuffer((uint16_t) Addr, (uint16_t) Reg, I2C_MEMADD_SIZE_8BIT, &Read_Value, 1);

  return Read_Value;
}

/**
  * @brief  AUDIO Codec delay
  * @param  Delay: Delay in ms
  * @retval None
  */
void AUDIO_IO_Delay(uint32_t Delay)
{
  HAL_Delay(Delay);
}
#endif /* HAL_I2C_MODULE_ENABLED */

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


