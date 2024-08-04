/**
  ******************************************************************************
  * @file    stm32l4p5g_discovery.c
  * @author  MCD Application Team
  * @brief   This file provides a set of firmware functions to manage Leds,
  *          push-button and joystick of STM32L4P5G_DISCOVERY board (MB1311)
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

/* Includes ------------------------------------------------------------------*/
#include "stm32l4p5g_discovery.h"
#include "stm32l4p5g_discovery_io.h"

/** @addtogroup BSP
  * @{
  */

/** @defgroup STM32L4P5G_DISCOVERY STM32L4P5G_DISCOVERY
  * @{
  */

/** @defgroup STM32L4P5G_DISCOVERY_Common STM32L4P5G_DISCOVERY Common
  * @{
  */

/** @defgroup STM32L4P5G_DISCOVERY_Private_Defines Private Defines
  * @{
  */

/**
 * @brief STM32L4P5G_DISCOVERY BSP Driver version number
   */
#define __STM32L4P5G_DISCOVERY_BSP_VERSION_MAIN   (0x01) /*!< [31:24] main version */
#define __STM32L4P5G_DISCOVERY_BSP_VERSION_SUB1   (0x01) /*!< [23:16] sub1 version */
#define __STM32L4P5G_DISCOVERY_BSP_VERSION_SUB2   (0x02) /*!< [15:8]  sub2 version */
#define __STM32L4P5G_DISCOVERY_BSP_VERSION_RC     (0x00) /*!< [7:0]  release candidate */
#define __STM32L4P5G_DISCOVERY_BSP_VERSION            ((__STM32L4P5G_DISCOVERY_BSP_VERSION_MAIN << 24)\
                                                      |(__STM32L4P5G_DISCOVERY_BSP_VERSION_SUB1 << 16)\
                                                      |(__STM32L4P5G_DISCOVERY_BSP_VERSION_SUB2 << 8 )\
                                                      |(__STM32L4P5G_DISCOVERY_BSP_VERSION_RC))


/**
  * @}
  */


/** @defgroup STM32L4P5G_DISCOVERY_Exported_Variables Exported Variables
  * @{
  */

/**
 * @brief LED variables
 */
GPIO_TypeDef   *LED_PORT[LEDn] = {LED1_GPIO_PORT,
                                  LED2_GPIO_PORT};

const uint16_t  LED_PIN[LEDn] = {LED1_PIN,
                                 LED2_PIN};

/**
 * @brief JOYSTICK variables
 */
GPIO_TypeDef *JOY_PORT[JOYn] =  {SEL_JOY_GPIO_PORT,
                                 LEFT_JOY_GPIO_PORT,
                                 RIGHT_JOY_GPIO_PORT,
                                 DOWN_JOY_GPIO_PORT,
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
 * @brief COM port variables
 */
#if defined(HAL_UART_MODULE_ENABLED)
USART_TypeDef* COM_USART[COMn] = {DISCOVERY_COM1};
GPIO_TypeDef* COM_TX_PORT[COMn] = {DISCOVERY_COM1_TX_GPIO_PORT};
GPIO_TypeDef* COM_RX_PORT[COMn] = {DISCOVERY_COM1_RX_GPIO_PORT};
const uint16_t COM_TX_PIN[COMn] = {DISCOVERY_COM1_TX_PIN};
const uint16_t COM_RX_PIN[COMn] = {DISCOVERY_COM1_RX_PIN};
const uint16_t COM_TX_AF[COMn] = {DISCOVERY_COM1_TX_AF};
const uint16_t COM_RX_AF[COMn] = {DISCOVERY_COM1_RX_AF};
#endif /* HAL_UART_MODULE_ENABLED */

/**
 * @brief BUS variables
 */
#if defined(HAL_I2C_MODULE_ENABLED)
uint32_t I2c1Timeout = DISCOVERY_I2C1_TIMEOUT_MAX;  /*<! Value of Timeout when I2C1 communication fails */
static   I2C_HandleTypeDef I2c1Handle = {0};
static uint32_t v_bspI2cUsers = BSP_I2C_NO_USER;
#if defined(BSP_USE_CMSIS_OS_NOT_USED)
static osSemaphoreId BspI2cSemaphore;
#endif
#endif /* HAL_I2C_MODULE_ENABLED */


/**
  * @}
  */

/** @defgroup STM32L4P5G_DISCOVERY_Private_FunctionPrototypes Private Functions Prototypes
  * @{
  */
/**************************** Bus functions ************************************/
/* I2C1 bus function */
#if defined(HAL_I2C_MODULE_ENABLED)
static void               I2C1_Init(uint32_t user);
static void               I2C1_MspInit(I2C_HandleTypeDef *hi2c);
static void               I2C1_MspDeInit(I2C_HandleTypeDef *hi2c);
static void               I2C1_WriteData(uint16_t Addr, uint16_t Reg, uint16_t RegSize, uint8_t Value);
static HAL_StatusTypeDef  I2C1_WriteBuffer(uint16_t Addr, uint16_t Reg, uint16_t RegSize, uint8_t *pBuffer, uint16_t Length);
static uint8_t            I2C1_ReadData(uint16_t Addr, uint16_t Reg, uint16_t RegSize);
static HAL_StatusTypeDef  I2C1_ReadBuffer(uint16_t Addr, uint16_t Reg, uint16_t RegSize, uint8_t *pBuffer, uint16_t Length);
static void               I2C1_Error(void);
static HAL_StatusTypeDef  I2C1_ReadMultiple(uint8_t Addr, uint16_t Reg, uint16_t MemAddSize, uint8_t *Buffer, uint16_t Length);
static HAL_StatusTypeDef  I2C1_WriteMultiple(uint8_t Addr, uint16_t Reg, uint16_t MemAddSize, uint8_t *Buffer, uint16_t Length);
static uint8_t            I2C1_isDeviceReady(uint16_t Addr, uint32_t Trials);
#endif/* HAL_I2C_MODULE_ENABLED */

/**************************** Link functions ***********************************/
#if defined(HAL_I2C_MODULE_ENABLED)

/* TouchScreen (TS) IO functions */
void            TS_IO_Init(void);
void            TS_IO_Write(uint8_t Addr, uint8_t Reg, uint8_t Value);
uint8_t         TS_IO_Read(uint8_t Addr, uint8_t Reg);
uint16_t        TS_IO_ReadMultiple(uint8_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length);
void            TS_IO_WriteMultiple(uint8_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length);
void            TS_IO_Delay(uint32_t Delay);

/* Link functions for IDD measurement */
void            MFX_IO_Init(void);
void            MFX_IO_DeInit(void);
void            MFX_IO_ITConfig (void);
void            MFX_IO_EnableWakeupPin(void);
void            MFX_IO_Wakeup(void);
void            MFX_IO_Delay(uint32_t delay);
void            MFX_IO_Write(uint16_t addr, uint8_t reg, uint8_t value);
uint8_t         MFX_IO_Read(uint16_t addr, uint8_t reg);
void            MFX_IO_WriteMultiple(uint16_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length);
uint16_t        MFX_IO_ReadMultiple(uint16_t addr, uint8_t reg, uint8_t *buffer, uint16_t length);

#endif/* HAL_I2C_MODULE_ENABLED */
/**
  * @}
  */

/** @defgroup STM32L4P5G_DISCOVERY_Exported_Functions Exported Functions
  * @{
  */

/**
  * @brief  Error Handler
  * @note   Defined as a weak function to be overwritten by the application.
  * @retval None
  */
__weak void BSP_ErrorHandler(void)
{
  while(1);
}

/**
  * @brief  This method returns the STM32L4P5G DISCOVERY BSP Driver revision
  * @retval version : 0xXYZR (8bits for each decimal, R for RC)
  */
uint32_t BSP_GetVersion(void)
{
  return __STM32L4P5G_DISCOVERY_BSP_VERSION;
}

/**
  * @brief  Configures LED GPIOs.
  * @param  Led: Specifies the Led to be configured.
  *   This parameter can be one of following parameters:
  *     @arg LED1
  *     @arg LED2
  * @retval None
  */
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

  HAL_GPIO_WritePin(LED_PORT[Led], GPIO_InitStructure.Pin, GPIO_PIN_SET);
}


/**
  * @brief  Unconfigures LED GPIOs.
  * @param  Led: Specifies the Led to be unconfigured.
  *   This parameter can be one of following parameters:
  *     @arg LED1
  *     @arg LED2
  * @retval None
  */
void BSP_LED_DeInit(Led_TypeDef Led)
{
  /* Enable the GPIO_LED clock */
  LEDx_GPIO_CLK_ENABLE(Led);

  HAL_GPIO_DeInit(LED_PORT[Led], LED_PIN[Led]);
}


/**
  * @brief  Turns selected LED On.
  * @param  Led: Specifies the Led to be set on.
  *   This parameter can be one of following parameters:
  *     @arg LED1
  *     @arg LED2
  * @retval None
  */
void BSP_LED_On(Led_TypeDef Led)
{
  HAL_GPIO_WritePin(LED_PORT[Led], LED_PIN[Led], GPIO_PIN_RESET);
}

/**
  * @brief  Turns selected LED Off.
  * @param  Led: Specifies the Led to be set off.
  *   This parameter can be one of following parameters:
  *     @arg LED1
  *     @arg LED2
  * @retval None
  */
void BSP_LED_Off(Led_TypeDef Led)
{
  HAL_GPIO_WritePin(LED_PORT[Led], LED_PIN[Led], GPIO_PIN_SET);
}

/**
  * @brief  Toggles the selected LED.
  * @param  Led: Specifies the Led to be toggled.
  *   This parameter can be one of following parameters:
  *     @arg LED1
  *     @arg LED2
  * @retval None
  */
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
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

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
  * @brief  Unconfigures all GPIOs used as buttons of the joystick.
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
    if (HAL_GPIO_ReadPin(JOY_PORT[joykey], JOY_PIN[joykey]) == GPIO_PIN_RESET)
    {
      /* Return Code Joystick key pressed */
      return joykey;
    }
  }

  /* No Joystick key pressed */
  return JOY_NONE;
}

#if defined(HAL_UART_MODULE_ENABLED)
/**
  * @brief  Configures COM port.
  * @param  COM: COM port to be configured.
  *          This parameter can be one of the following values:
  *            @arg  COM1
  * @param  huart: Pointer to a UART_HandleTypeDef structure that contains the
  *                configuration information for the specified USART peripheral.
  */
void BSP_COM_Init(COM_TypeDef COM, UART_HandleTypeDef *huart)
{
  GPIO_InitTypeDef gpio_init_structure;

  /* Enable GPIO clock */
  DISCOVERY_COMx_TX_GPIO_CLK_ENABLE(COM);
  DISCOVERY_COMx_RX_GPIO_CLK_ENABLE(COM);

  /* Enable USART clock */
  DISCOVERY_COMx_CLK_ENABLE(COM);

  /* Configure USART Tx as alternate function */
  gpio_init_structure.Pin = COM_TX_PIN[COM];
  gpio_init_structure.Mode = GPIO_MODE_AF_PP;
  gpio_init_structure.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio_init_structure.Pull = GPIO_NOPULL;
  gpio_init_structure.Alternate = COM_TX_AF[COM];
  HAL_GPIO_Init(COM_TX_PORT[COM], &gpio_init_structure);

  /* Configure USART Rx as alternate function */
  gpio_init_structure.Pin = COM_RX_PIN[COM];
  gpio_init_structure.Mode = GPIO_MODE_AF_PP;
  gpio_init_structure.Alternate = COM_RX_AF[COM];
  HAL_GPIO_Init(COM_RX_PORT[COM], &gpio_init_structure);

  /* USART configuration */
  huart->Instance = COM_USART[COM];
  HAL_UART_Init(huart);
}

/**
  * @brief  DeInit COM port.
  * @param  COM: COM port to be configured.
  *          This parameter can be one of the following values:
  *            @arg  COM1
  * @param  huart: Pointer to a UART_HandleTypeDef structure that contains the
  *                configuration information for the specified USART peripheral.
  */
void BSP_COM_DeInit(COM_TypeDef COM, UART_HandleTypeDef *huart)
{
  /* USART deinitialization */
  huart->Instance = COM_USART[COM];
  HAL_UART_DeInit(huart);

  /* Disable USART clock */
  DISCOVERY_COMx_CLK_DISABLE(COM);

  /* USART TX/RX pins deinitializations */
  HAL_GPIO_DeInit(COM_TX_PORT[COM], COM_TX_PIN[COM]);
  HAL_GPIO_DeInit(COM_RX_PORT[COM], COM_RX_PIN[COM]);

  /* Disable GPIOs clock is left for application */
}
#endif /* HAL_UART_MODULE_ENABLED */

/**
  * @}
  */

/** @defgroup STM32L4XX_NUCLEO_SMPS_Functions SMPS Functions
  * @{
  */

/******************************************************************************
                            SMPS OPERATIONS
*******************************************************************************/

/**
  * @brief  DeInitialize the external SMPS component
  * @note   Low power consumption GPIO settings
  * @retval SMPS status
  */
uint32_t BSP_SMPS_DeInit(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  PWR_AND_CLK_SMPS();

  /* --------------------------------------------------------------------------------------  */
  /* Added for Deinit if No PIN_SMPS_ENABLE & PIN_SMPS_SWITCH_ENABLE are not disabled before */

  /* Disable SMPS SWITCH */
  HAL_GPIO_WritePin(PORT_SMPS, PIN_SMPS_SWITCH_ENABLE, GPIO_PIN_RESET);

  HAL_Delay(1);

  /* Disable SMPS */
  HAL_GPIO_WritePin(PORT_SMPS, PIN_SMPS_ENABLE, GPIO_PIN_RESET);

  /* --------------------------------------------------------------------------------------  */
  /* Set all GPIO in output push/pull pulldown state to reduce power consumption  */
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Pull =  GPIO_PULLDOWN;

  /* Consider all SMPS pins */
  GPIO_InitStruct.Pin = PIN_SMPS_ENABLE | PIN_SMPS_SWITCH_ENABLE | PIN_SMPS_POWERGOOD | PIN_SMPS_V1 | PIN_SMPS_V2 | PIN_SMPS_V3;
  HAL_GPIO_Init(PORT_SMPS, &GPIO_InitStruct);

  return SMPS_OK;
}

/**
  * @brief  Initialize the external SMPS component
  * @note   Voltage scale must be set to range 2 before calling this function
  * @retval SMPS status
  */
uint32_t BSP_SMPS_Init(void)
{
  PWR_AND_CLK_SMPS();

  GPIO_InitTypeDef GPIO_InitStruct;

  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Pull = GPIO_PULLUP;

  GPIO_InitStruct.Pin = PIN_SMPS_POWERGOOD;
  HAL_GPIO_Init(PORT_SMPS, &GPIO_InitStruct);

  /* ------------------------------------------------------------------------ */

  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Pull = GPIO_NOPULL;

  GPIO_InitStruct.Pin = PIN_SMPS_ENABLE | PIN_SMPS_SWITCH_ENABLE | PIN_SMPS_V1 | PIN_SMPS_V2 | PIN_SMPS_V3;
  HAL_GPIO_Init(PORT_SMPS, &GPIO_InitStruct);

  /* --------- SMPS VOLTAGE RANGE SELECTION ----------------------------------*/
  /* ######################################################################## */

  /* - > Applied to ST1PS02DQTR */
  /* 1.25V                  */
  /* D0/D1/D2 = H/H/L      */

     HAL_GPIO_WritePin(PORT_SMPS,  PIN_SMPS_V3 , GPIO_PIN_RESET);
     HAL_GPIO_WritePin(PORT_SMPS, PIN_SMPS_V1 | PIN_SMPS_V2 , GPIO_PIN_SET);


  /* ######################################################################## */
  return SMPS_OK;
}

/**
  * @brief  Enable the external SMPS component
  * @param  Delay: delay in ms after enable
  * @param  Power_Good_Check: Enable Power good check
  * @note   Power_Good_Check is not used with all external
  *         SMPS components
  * @retval SMPS status
  *           @arg SMPS_OK: SMPS ENABLE OK
  *           @arg SMPS_KO: POWER GOOD CHECK FAILS
  */
uint32_t BSP_SMPS_Enable(uint32_t Delay, uint32_t Power_Good_Check)
{
  PWR_AND_CLK_SMPS();

  HAL_GPIO_WritePin(PORT_SMPS, PIN_SMPS_ENABLE, GPIO_PIN_SET);

  /* Delay upon request */
  if (Delay != 0)
  {
    HAL_Delay(Delay);
  }

  /* CHECK POWER GOOD or NOT */
  if (Power_Good_Check != 0)
  {
    if (GPIO_PIN_RESET == (HAL_GPIO_ReadPin(PORT_SMPS, PIN_SMPS_POWERGOOD)))
    {
      /* POWER GOOD KO */
      return SMPS_KO;
    }
  }

  /* SMPS ENABLED */
  return SMPS_OK;
}

/**
  * @brief  Disable the external SMPS component
  * @note   SMPS SWITCH should be disabled first !
  * @retval SMPS status
  *           @arg SMPS_OK: SMPS DISABLE OK - DONE
  *           @arg SMPS_KO: POWER GOOD CHECK FAILS
  *
  */
uint32_t BSP_SMPS_Disable(void)
{

  PWR_AND_CLK_SMPS();

  /* Check if SMPS SWITCH is disabled */
  if (HAL_GPIO_ReadPin(PORT_SMPS, PIN_SMPS_SWITCH_ENABLE) != GPIO_PIN_RESET)
  {
    /* ERROR AS SWITCH SHOULD BE DISABLED */
    return SMPS_KO;
  }

  /* Disable SMPS */
  HAL_GPIO_WritePin(PORT_SMPS, PIN_SMPS_ENABLE, GPIO_PIN_RESET);

  /* SMPS DISABLED */
  return SMPS_OK;
}

/**
  * @brief  Enable the external SMPS SWITCH component
  * @param  Delay: delay in ms before SMPS SWITCH ENABLE
  * @param  Power_Good_Check: Enable Power good check
  * @note   Power_Good_Check is not used with all boards
  * @retval SMPS status
  *           @arg SMPS_OK: SMPS ENABLE OK
  *           @arg SMPS_KO: POWER GOOD CHECK FAILS
  */
uint32_t BSP_SMPS_Supply_Enable(uint32_t Delay, uint32_t Power_Good_Check)
{
  PWR_AND_CLK_SMPS();

  if (Delay != 0)
  {
    HAL_Delay(Delay);
  }
  /* CHECK POWER GOOD or NOT */
  if (Power_Good_Check != 0)
  {
    if (GPIO_PIN_RESET == (HAL_GPIO_ReadPin(PORT_SMPS, PIN_SMPS_POWERGOOD)))
    {
      /* POWER GOOD KO */
      return SMPS_KO;
    }
  }

  /* SMPS SWITCH ENABLE */
  HAL_GPIO_WritePin(PORT_SMPS, PIN_SMPS_SWITCH_ENABLE, GPIO_PIN_SET);


  return SMPS_OK;
}

/**
  * @brief  Disable the external SMPS SWITCH component
  * @retval SMPS status
  *           @arg SMPS_OK: SMPS SWITCH DISABLE OK
  */
uint32_t BSP_SMPS_Supply_Disable(void)
{
  PWR_AND_CLK_SMPS();

  /* SMPS SWITCH DISABLE */
  HAL_GPIO_WritePin(PORT_SMPS, PIN_SMPS_SWITCH_ENABLE, GPIO_PIN_RESET);

  return SMPS_OK;
}


/**
  * @}
  */

/** @defgroup STM32L4P5G_DISCOVERY_Private_Functions Private Functions
  * @{
  */

/*******************************************************************************
                            BUS OPERATIONS
*******************************************************************************/

#if defined(HAL_I2C_MODULE_ENABLED)
/******************************* I2C Routines**********************************/
/**
  * @brief Discovery I2C1 Bus initialization
  * @retval None
  */
static void I2C1_Init(uint32_t user)
{
  if(HAL_I2C_GetState(&I2c1Handle) == HAL_I2C_STATE_RESET)
  {
#if defined(BSP_USE_CMSIS_OS_NOT_USED)
    /* Create semaphore to prevent multiple I2C access */
    osSemaphoreDef(BSP_I2C_SEM);
    BspI2cSemaphore = osSemaphoreCreate(osSemaphore(BSP_I2C_SEM), 1);
#endif

    I2c1Handle.Instance              = I2C1;
    I2c1Handle.Init.Timing           = DISCOVERY_I2C1_TIMING;
    I2c1Handle.Init.OwnAddress1      = 0x70;
    I2c1Handle.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    I2c1Handle.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    I2c1Handle.Init.OwnAddress2      = 0xFF;
    I2c1Handle.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    I2c1Handle.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;

    /* Init the I2C */
    I2C1_MspInit(&I2c1Handle);
    HAL_I2C_Init(&I2c1Handle);
  }
  /* Update BSP I2C users list */
  v_bspI2cUsers |= user;
}

/**
  * @brief Discovery I2C1 MSP Initialization
  * @param hi2c: I2C1 handle
  * @retval None
  */
static void I2C1_MspInit(I2C_HandleTypeDef *hi2c)
{
  GPIO_InitTypeDef          GPIO_InitStructure;
  RCC_PeriphCLKInitTypeDef  RCC_PeriphCLKInitStruct;

  if (hi2c->Instance == I2C1)
  {
    /*##-1- Configure the Discovery I2C1 clock source. The clock is derived from the SYSCLK #*/
    RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
    RCC_PeriphCLKInitStruct.I2c1ClockSelection   = RCC_I2C1CLKSOURCE_SYSCLK;
    HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);

    /*##-2- Configure the GPIOs ################################################*/
    /* Enable GPIO clock */
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Configure I2C pins as alternate function  */
    GPIO_InitStructure.Pin       = GPIO_PIN_6;
    GPIO_InitStructure.Mode      = GPIO_MODE_AF_OD;
    GPIO_InitStructure.Pull      = GPIO_PULLUP;
    GPIO_InitStructure.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStructure.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure); /* I2C1_SCL on PB6 */
    GPIO_InitStructure.Pin       = GPIO_PIN_7;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure); /* I2C1_SDA on PB7 */

    /*##-3- Configure the Discovery I2C1 peripheral #############################*/
    /* Enable I2C1 clock */
    __HAL_RCC_I2C1_CLK_ENABLE();

    /* Force and release the I2C1 Peripheral */
    __HAL_RCC_I2C1_FORCE_RESET();
    __HAL_RCC_I2C1_RELEASE_RESET();

    /* Enable and set I2C1 event interrupt to the highest priority */
    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 0x00, 0);
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);

    /* Enable and set Discovery I2C1 error interrupt to the highest priority */
    HAL_NVIC_SetPriority(I2C1_ER_IRQn, 0x00, 0);
    HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
  }
}

/**
  * @brief Discovery I2C1 Bus Deinitialization
  * @retval None
  */
void I2C1_DeInit(uint32_t user)
{
  /* Update BSP I2C users list */
  v_bspI2cUsers &= ~(user);

  if((HAL_I2C_GetState(&I2c1Handle) != HAL_I2C_STATE_RESET) && (v_bspI2cUsers == BSP_I2C_NO_USER))
  {
    /* DeInit the I2C */
    HAL_I2C_DeInit(&I2c1Handle);
    I2C1_MspDeInit(&I2c1Handle);

#if defined(BSP_USE_CMSIS_OS_NOT_USED)
    /* Delete semaphore to prevent multiple I2C access */
    osSemaphoreDelete(BspI2cSemaphore);
#endif
  }
}

/**
  * @brief Discovery I2C1 MSP DeInitialization
  * @param hi2c: I2C1 handle
  * @retval None
  */
static void I2C1_MspDeInit(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C1)
  {
    /*##-1- Unconfigure the GPIOs ################################################*/
    /* Reset configuration of I2C GPIOs as alternate function  */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6);  /* I2C1_SCL */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_7); /* I2C1_SDA */

    /*##-2- Unconfigure the Discovery I2C1 peripheral ############################*/
    /* Force and release I2C1 Peripheral */
    __HAL_RCC_I2C1_FORCE_RESET();
    __HAL_RCC_I2C1_RELEASE_RESET();

    /* Disable I2C1 clock */
    __HAL_RCC_I2C1_CLK_DISABLE();

    /* Disable I2C1 interrupts */
    HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);
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
static void I2C1_WriteData(uint16_t Addr, uint16_t Reg, uint16_t RegSize, uint8_t Value)
{
  HAL_StatusTypeDef status = HAL_OK;

#if defined(BSP_USE_CMSIS_OS_NOT_USED)
  /* Get semaphore to prevent multiple I2C access */
  osSemaphoreWait(BspI2cSemaphore, osWaitForever);
#endif

  status = HAL_I2C_Mem_Write(&I2c1Handle, Addr, (uint16_t)Reg, RegSize, &Value, 1, I2c1Timeout);

#if defined(BSP_USE_CMSIS_OS_NOT_USED)
  /* Release semaphore to prevent multiple I2C access */
  osSemaphoreRelease(BspI2cSemaphore);
#endif

  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Re-Initiaize the BUS */
    I2C1_Error();
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

#if defined(BSP_USE_CMSIS_OS_NOT_USED)
  /* Get semaphore to prevent multiple I2C access */
  osSemaphoreWait(BspI2cSemaphore, osWaitForever);
#endif

  status = HAL_I2C_Mem_Write(&I2c1Handle, Addr, (uint16_t)Reg, RegSize, pBuffer, Length, I2c1Timeout);

#if defined(BSP_USE_CMSIS_OS_NOT_USED)
  /* Release semaphore to prevent multiple I2C access */
  osSemaphoreRelease(BspI2cSemaphore);
#endif

  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Re-Initiaize the BUS */
    I2C1_Error();
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
static uint8_t I2C1_ReadData(uint16_t Addr, uint16_t Reg, uint16_t RegSize)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint8_t value = 0x0;

#if defined(BSP_USE_CMSIS_OS_NOT_USED)
  /* Get semaphore to prevent multiple I2C access */
  osSemaphoreWait(BspI2cSemaphore, osWaitForever);
#endif

  status = HAL_I2C_Mem_Read(&I2c1Handle, Addr, Reg, RegSize, &value, 1, I2c1Timeout);

#if defined(BSP_USE_CMSIS_OS_NOT_USED)
  /* Release semaphore to prevent multiple I2C access */
  osSemaphoreRelease(BspI2cSemaphore);
#endif

  /* Check the communication status */
  if(status != HAL_OK)
  {
    if (Addr == TS_I2C_ADDRESS)
    {
      /* No error reported for Touchscreen address
         in case LCD is not mounted on board */
      return 0x0;
    }
    else
    {
      /* Re-Initiaize the BUS */
      I2C1_Error();
    }
  }

  return value;
}

/**
* @brief  Checks if target device is ready for communication.
* @note   This function is used with Memory devices
* @param  Addr: Target device address
* @param  Trials: Number of trials
* @retval HAL status
*/
static uint8_t I2C1_isDeviceReady(uint16_t Addr, uint32_t Trials)
{
  HAL_StatusTypeDef status = HAL_OK;

#if defined(BSP_USE_CMSIS_OS_NOT_USED)
  /* Get semaphore to prevent multiple I2C access */
  osSemaphoreWait(BspI2cSemaphore, osWaitForever);
#endif

  status = HAL_I2C_IsDeviceReady(&I2c1Handle, Addr, Trials, 50);

#if defined(BSP_USE_CMSIS_OS_NOT_USED)
  /* Release semaphore to prevent multiple I2C access */
  osSemaphoreRelease(BspI2cSemaphore);
#endif

  /* Check the communication status */
  if(status != HAL_OK)
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

#if defined(BSP_USE_CMSIS_OS_NOT_USED)
  /* Get semaphore to prevent multiple I2C access */
  osSemaphoreWait(BspI2cSemaphore, osWaitForever);
#endif

  status = HAL_I2C_Mem_Read(&I2c1Handle, Addr, (uint16_t)Reg, RegSize, pBuffer, Length, I2c1Timeout);

#if defined(BSP_USE_CMSIS_OS_NOT_USED)
  /* Release semaphore to prevent multiple I2C access */
  osSemaphoreRelease(BspI2cSemaphore);
#endif

  /* Check the communication status */
  if(status != HAL_OK)
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
  uint32_t tmpI2cUsers;

  BSP_ErrorHandler();

  /* De-initialize the I2C communication BUS */
  tmpI2cUsers = v_bspI2cUsers;
  I2C1_DeInit(BSP_I2C_ALL_USERS);

  /* Re- Initiaize the I2C communication BUS */
  I2C1_Init(tmpI2cUsers);
}

/**
  * @brief  Reads multiple data.
  * @param  Addr: I2C address
  * @param  Reg: Reg address
  * @param  MemAddress: Memory address
  * @param  Buffer: Pointer to data buffer
  * @param  Length: Length of the data
  * @retval HAL status
  */
static HAL_StatusTypeDef I2C1_ReadMultiple(uint8_t  Addr,
                                           uint16_t Reg,
                                           uint16_t MemAddress,
                                           uint8_t *Buffer,
                                           uint16_t Length)
{
  HAL_StatusTypeDef status = HAL_OK;

#if defined(BSP_USE_CMSIS_OS_NOT_USED)
  /* Get semaphore to prevent multiple I2C access */
  osSemaphoreWait(BspI2cSemaphore, osWaitForever);
#endif

  status = HAL_I2C_Mem_Read(&I2c1Handle, Addr, (uint16_t)Reg, MemAddress, Buffer, Length, I2c1Timeout);

#if defined(BSP_USE_CMSIS_OS_NOT_USED)
  /* Release semaphore to prevent multiple I2C access */
  osSemaphoreRelease(BspI2cSemaphore);
#endif

  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* I2C error occurred */
    I2C1_Error();
  }
  return status;
}

/**
  * @brief  Writes a value in a register of the device through BUS.
  * @param  Addr: Device address on BUS Bus.
  * @param  Reg: The target register address to write
  * @param  MemAddress: Memory address
  * @param  Buffer: The target register value to be written
  * @param  Length: buffer size to be written
  * @retval HAL status
  */
static HAL_StatusTypeDef I2C1_WriteMultiple(uint8_t  Addr,
                                            uint16_t Reg,
                                            uint16_t MemAddress,
                                            uint8_t *Buffer,
                                            uint16_t Length)
{
  HAL_StatusTypeDef status = HAL_OK;

#if defined(BSP_USE_CMSIS_OS_NOT_USED)
  /* Get semaphore to prevent multiple I2C access */
  osSemaphoreWait(BspI2cSemaphore, osWaitForever);
#endif

  status = HAL_I2C_Mem_Write(&I2c1Handle, Addr, (uint16_t)Reg, MemAddress, Buffer, Length, I2c1Timeout);

#if defined(BSP_USE_CMSIS_OS_NOT_USED)
  /* Release semaphore to prevent multiple I2C access */
  osSemaphoreRelease(BspI2cSemaphore);
#endif

  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Re-Initialize the I2C Bus */
    I2C1_Error();
  }
  return status;
}



#endif /*HAL_I2C_MODULE_ENABLED*/

/*******************************************************************************
                            LINK OPERATIONS
*******************************************************************************/

#if defined(HAL_I2C_MODULE_ENABLED)
/********************************* LINK MFX ***********************************/
/**
  * @brief  Initializes MFX low level.
  * @retval None
  */
void MFX_IO_Init(void)
{
  /* I2C1 init */
  I2C1_Init(BSP_I2C_MFX_USER);

  /* Wait for device ready */
  if(I2C1_isDeviceReady(IO_I2C_ADDRESS, 4) != HAL_OK)
  {
     BSP_ErrorHandler();
  }
}
/**
  * @brief  Deinitializes MFX low level.
  * @retval None
  */
void MFX_IO_DeInit(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  /* Enable wakeup gpio clock */
  MFX_WAKEUP_GPIO_CLK_ENABLE();

  /* MFX wakeup pin configuration */
  GPIO_InitStruct.Pin   = MFX_WAKEUP_PIN;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
  HAL_GPIO_Init(MFX_WAKEUP_GPIO_PORT, &GPIO_InitStruct);

  /* DeInit interrupt pin : disable IRQ before to avoid spurious interrupt */
  HAL_NVIC_DisableIRQ((IRQn_Type)(MFX_INT_EXTI_IRQn));
  MFX_INT_GPIO_CLK_ENABLE();
  HAL_GPIO_DeInit(MFX_INT_GPIO_PORT, MFX_INT_PIN);

  /* I2C1 Deinit */
  I2C1_DeInit(BSP_I2C_MFX_USER);
}

/**
  * @brief  Configures MFX low level interrupt.
  * @retval None
  */
void MFX_IO_ITConfig(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  /* Enable the GPIO clock */
  MFX_INT_GPIO_CLK_ENABLE();

  /* MFX_OUT_IRQ (normally used for EXTI_WKUP) */
  GPIO_InitStruct.Pin   = MFX_INT_PIN;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Mode  = GPIO_MODE_IT_RISING;
  HAL_GPIO_Init(MFX_INT_GPIO_PORT, &GPIO_InitStruct);

  /* Enable and set GPIO EXTI Interrupt to the lowest priority */
  HAL_NVIC_SetPriority((IRQn_Type)(MFX_INT_EXTI_IRQn), 0x0F, 0x0F);
  HAL_NVIC_EnableIRQ((IRQn_Type)(MFX_INT_EXTI_IRQn));
}

/**
  * @brief  Configures MFX wke up  pin.
  * @retval None
  */
void MFX_IO_EnableWakeupPin(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  /* Enable wakeup gpio clock */
  MFX_WAKEUP_GPIO_CLK_ENABLE();

  /* MFX wakeup pin configuration */
  GPIO_InitStruct.Pin   = MFX_WAKEUP_PIN;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  HAL_GPIO_Init(MFX_WAKEUP_GPIO_PORT, &GPIO_InitStruct);
}

/**
  * @brief  Wakeup MFX.
  * @retval None
  */
void MFX_IO_Wakeup(void)
{
  /* Set Wakeup pin to high to wakeup Idd measurement component from standby mode */
  HAL_GPIO_WritePin(MFX_WAKEUP_GPIO_PORT, MFX_WAKEUP_PIN, GPIO_PIN_SET);

  /* Wait */
  HAL_Delay(1);

  /* Set gpio pin basck to low */
  HAL_GPIO_WritePin(MFX_WAKEUP_GPIO_PORT, MFX_WAKEUP_PIN, GPIO_PIN_RESET);
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
  I2C1_WriteData(Addr, Reg, I2C_MEMADD_SIZE_8BIT, Value);
}

/**
  * @brief  MFX reads single data.
  * @param  Addr: I2C address
  * @param  Reg: Register address
  * @retval Read data
  */
uint8_t MFX_IO_Read(uint16_t Addr, uint8_t Reg)
{
  return I2C1_ReadData(Addr, Reg, I2C_MEMADD_SIZE_8BIT);
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
 return I2C1_ReadBuffer(Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, Buffer, Length);
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
  I2C1_WriteBuffer(Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, Buffer, Length);
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



/************************** LINK TS (TouchScreen) *****************************/
/**
  * @brief  Initializes Touchscreen low level.
  * @retval None
  */
void TS_IO_Init(void)
{
  I2C1_Init(BSP_I2C_TS_USER);
}

/**
  * @brief  Writes a single data.
  * @param  Addr: I2C address
  * @param  Reg: Reg address
  * @param  Value: Data to be written
  * @retval None
  */
void TS_IO_Write(uint8_t Addr, uint8_t Reg, uint8_t Value)
{
  I2C1_WriteMultiple(Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT,(uint8_t*)&Value, 1);
}

/**
  * @brief  Reads a single data.
  * @param  Addr: I2C address
  * @param  Reg: Reg address
  * @retval Data to be read
  */
uint8_t TS_IO_Read(uint8_t Addr, uint8_t Reg)
{
  return I2C1_ReadData(Addr, Reg, I2C_MEMADD_SIZE_8BIT);
}

/**
  * @brief  Reads multiple data with I2C communication
  *         channel from TouchScreen.
  * @param  Addr: I2C address
  * @param  Reg: Register address
  * @param  Buffer: Pointer to data buffer
  * @param  Length: Length of the data
  * @retval Number of read data
  */
uint16_t TS_IO_ReadMultiple(uint8_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length)
{
 return I2C1_ReadMultiple(Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, Buffer, Length);
}

/**
  * @brief  Writes multiple data with I2C communication
  *         channel from MCU to TouchScreen.
  * @param  Addr: I2C address
  * @param  Reg: Register address
  * @param  Buffer: Pointer to data buffer
  * @param  Length: Length of the data
  * @retval None
  */
void TS_IO_WriteMultiple(uint8_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length)
{
  I2C1_WriteMultiple(Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, Buffer, Length);
}

/**
  * @brief  Delay function used in TouchScreen low level driver.
  * @param  Delay: Delay in ms
  * @retval None
  */
void TS_IO_Delay(uint32_t Delay)
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


