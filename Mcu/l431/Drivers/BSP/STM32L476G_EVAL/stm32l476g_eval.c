/**
  ******************************************************************************
  * @file    stm32l476g_eval.c
  * @author  MCD Application Team
  * @brief   This file provides a set of firmware functions to manage Leds, 
  *          push-button and COM ports for STM32L476G_EVAL
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
#include "stm32l476g_eval.h"
#if defined(USE_IOEXPANDER)
#include "stm32l476g_eval_io.h"
#endif /* USE_IOEXPANDER */

/** @addtogroup BSP
  * @{
  */ 

/** @defgroup STM32L476G_EVAL STM32L476G_EVAL
  * @{
  */ 

/** @defgroup STM32L476G_EVAL_Common STM32L476G_EVAL Common
  * @{
  */   
    
/** @defgroup STM32L476G_EVAL_Private_TypesDefinitions Private Types Definitions
  * @brief This file provides firmware functions to manage Leds, push-buttons, 
  *        COM ports, SD card on SPI and temperature sensor (TS751) available on 
  *        STM32L476G-EVAL evaluation board from STMicroelectronics.
  * @{
  */

typedef struct
{
  __IO uint16_t LCD_REG_R; /* Read Register */
  __IO uint16_t LCD_RAM_R; /* Read RAM */
  __IO uint16_t LCD_REG_W; /* Write Register */
  __IO uint16_t LCD_RAM_W; /* Write RAM */
} TFT_LCD_TypeDef;

/**
  * @}
  */

/** @defgroup STM32L476G_EVAL_Private_Defines Private Defines
  * @{
  */ 
/* LINK LCD */
#define START_BYTE         0x70
#define SET_INDEX          0x00
#define READ_STATUS        0x01
#define LCD_WRITE_REG      0x02
#define LCD_READ_REG       0x03

/**
 * @brief STM32L476G EVAL BSP Driver version number
   */
#define __STM32L476G_EVAL_BSP_VERSION_MAIN   (0x02) /*!< [31:24] main version */
#define __STM32L476G_EVAL_BSP_VERSION_SUB1   (0x01) /*!< [23:16] sub1 version */
#define __STM32L476G_EVAL_BSP_VERSION_SUB2   (0x07) /*!< [15:8]  sub2 version */
#define __STM32L476G_EVAL_BSP_VERSION_RC     (0x00) /*!< [7:0]  release candidate */
#define __STM32L476G_EVAL_BSP_VERSION            ((__STM32L476G_EVAL_BSP_VERSION_MAIN << 24)\
                                                 |(__STM32L476G_EVAL_BSP_VERSION_SUB1 << 16)\
                                                 |(__STM32L476G_EVAL_BSP_VERSION_SUB2 << 8 )\
                                                 |(__STM32L476G_EVAL_BSP_VERSION_RC))

/* Note: LCD /CS is NE3 - Bank 3 of NOR/SRAM Bank 1~4 */
#define TFT_LCD_BASE  FMC_BANK1_3
#define TFT_LCD       ((TFT_LCD_TypeDef *) TFT_LCD_BASE)

/**
  * @}
  */


/** @defgroup STM32L476G_EVAL_Exported_Variables Exported Variables
  * @{
  */

/**
 * @brief LED variables
 */                                 
#if defined(USE_STM32L476G_EVAL_REVA)
 /* 0xFFFF to mark as invalid port (LED2, LED3 and LED4 accessed through IOExpander) */
GPIO_TypeDef*   LED_PORT[LEDn] =                {LED1_GPIO_PORT,
                                                 (GPIO_TypeDef *)0xFFFF,
                                                 (GPIO_TypeDef *)0xFFFF,
                                                 (GPIO_TypeDef *)0xFFFF};
#elif defined(USE_STM32L476G_EVAL_REVB)
 /* 0xFFFF to mark as invalid port (LED2 and LED4 accessed through IOExpander) */
GPIO_TypeDef*   LED_PORT[LEDn] =                {LED1_GPIO_PORT,
                                                 (GPIO_TypeDef *)0xFFFF,
                                                 LED3_GPIO_PORT,
                                                 (GPIO_TypeDef *)0xFFFF};
#endif

#if defined(USE_IOEXPANDER)
const uint16_t  LED_PIN[LEDn] =                 {LED1_PIN,
                                                 LED2_PIN,
                                                 LED3_PIN,
                                                 LED4_PIN};
#else
#if defined(USE_STM32L476G_EVAL_REVA)
const uint16_t  LED_PIN[LEDn] =                 {LED1_PIN,
                                                 (uint16_t)0x0,
                                                 (uint16_t)0x0,
                                                 (uint16_t)0x0};
#elif defined(USE_STM32L476G_EVAL_REVB)
const uint16_t  LED_PIN[LEDn] =                 {LED1_PIN,
                                                 (uint16_t)0x0,
                                                 LED3_PIN,
                                                 (uint16_t)0x0};
#endif
#endif /* USE_IOEXPANDER */

/**
 * @brief BUTTON variables
 */
#if defined(USE_IOEXPANDER)
GPIO_TypeDef*   BUTTON_PORT[BUTTONn] =          {TAMPER_BUTTON_GPIO_PORT,
                                                 (GPIO_TypeDef *)0xFFFF,
                                                 (GPIO_TypeDef *)0xFFFF,
                                                 (GPIO_TypeDef *)0xFFFF,
                                                 (GPIO_TypeDef *)0xFFFF,
                                                 (GPIO_TypeDef *)0xFFFF}; 
/* 0xFFFF to mark as invalid port (Joystick buttons accessed through IOExpander) */                                                 
#else
GPIO_TypeDef*   BUTTON_PORT[BUTTONn] =          {TAMPER_BUTTON_GPIO_PORT}; 
#endif /* USE_IOEXPANDER */

#if defined(USE_IOEXPANDER)
const uint16_t  BUTTON_PIN[BUTTONn] =           {TAMPER_BUTTON_PIN,
                                                 SEL_JOY_PIN, 
                                                 LEFT_JOY_PIN, 
                                                 RIGHT_JOY_PIN, 
                                                 DOWN_JOY_PIN, 
                                                 UP_JOY_PIN}; 
                                
const uint8_t   BUTTON_IRQn[BUTTONn] =          {TAMPER_BUTTON_EXTI_IRQn,
                                                 SEL_JOY_EXTI_IRQn,
                                                 LEFT_JOY_EXTI_IRQn,
                                                 RIGHT_JOY_EXTI_IRQn,
                                                 DOWN_JOY_EXTI_IRQn,
                                                 UP_JOY_EXTI_IRQn};
#else
const uint16_t  BUTTON_PIN[BUTTONn] =           {TAMPER_BUTTON_PIN}; 
                                
const uint8_t   BUTTON_IRQn[BUTTONn] =          {TAMPER_BUTTON_EXTI_IRQn};
#endif /* USE_IOEXPANDER */

#if defined(USE_IOEXPANDER)
/**
 * @brief JOYSTICK variables
 */

const uint16_t JOY_PIN[JOYn] = {SEL_JOY_PIN, 
                                LEFT_JOY_PIN, 
                                RIGHT_JOY_PIN, 
                                DOWN_JOY_PIN, 
                                UP_JOY_PIN}; 

const uint8_t JOY_IRQn[JOYn] = {SEL_JOY_EXTI_IRQn,  
                                LEFT_JOY_EXTI_IRQn, 
                                RIGHT_JOY_EXTI_IRQn,
                                DOWN_JOY_EXTI_IRQn, 
                                UP_JOY_EXTI_IRQn};  
#endif /* USE_IOEXPANDER */

/**
 * @brief COM variables
 */
USART_TypeDef*  COM_USART[COMn]   =             {EVAL_COM1}; 

GPIO_TypeDef*   COM_TX_PORT[COMn] =             {EVAL_COM1_TX_GPIO_PORT};
 
GPIO_TypeDef*   COM_RX_PORT[COMn] =             {EVAL_COM1_RX_GPIO_PORT};

const uint16_t  COM_TX_PIN[COMn]  =             {EVAL_COM1_TX_PIN};

const uint16_t  COM_RX_PIN[COMn]  =             {EVAL_COM1_RX_PIN};
 
const uint16_t COM_TX_AF[COMn]    =             {EVAL_COM1_TX_AF};

const uint16_t COM_RX_AF[COMn]    =             {EVAL_COM1_RX_AF};

/**
 * @brief BUS variables
 */

#ifdef HAL_I2C_MODULE_ENABLED
uint32_t heval_I2cxTimeout = EVAL_I2Cx_TIMEOUT_MAX;   /*<! Value of Timeout when I2C communication fails */
I2C_HandleTypeDef heval_I2c;
#endif /* HAL_I2C_MODULE_ENABLED */

/**
  * @}
  */ 

/** @defgroup STM32L476G_EVAL_Private_FunctionPrototypes Private Functions
  * @{
  */ 
/* FMC bus function */
#ifdef HAL_SRAM_MODULE_ENABLED
static void               FMC_BANK3_WriteData(uint16_t Data);
static void               FMC_BANK3_WriteReg(uint8_t Reg);
static uint16_t           FMC_BANK3_ReadData(uint8_t Reg);
static void               FMC_BANK3_Init(void);
static void               FMC_BANK3_MspInit(void);

/* LCD IO functions */
void                      LCD_IO_Init(void);
void                      LCD_IO_WriteMultipleData(uint8_t *pData, uint32_t Size);
void                      LCD_IO_WriteReg(uint8_t Reg);
uint16_t                  LCD_IO_ReadData(uint8_t Reg);
void                      LCD_Delay (uint32_t delay);
#endif /* HAL_SRAM_MODULE_ENABLED */

/* I2Cx bus function */
#ifdef HAL_I2C_MODULE_ENABLED
/* Link function I2C for EEPROM, Audio Codec peripheral and IO expanders*/
static void               I2Cx_Init(void);
static void               I2Cx_MspInit(I2C_HandleTypeDef *hi2c);  
static void               I2Cx_DeInit(void);
static void               I2Cx_MspDeInit(I2C_HandleTypeDef *hi2c);  
static HAL_StatusTypeDef  I2Cx_WriteBuffer(uint16_t Addr, uint16_t Reg, uint16_t RegSize, uint8_t *pBuffer, uint16_t Length);
static HAL_StatusTypeDef  I2Cx_ReadBuffer(uint16_t Addr, uint16_t Reg, uint16_t RegSize, uint8_t *pBuffer, uint16_t Length);
static HAL_StatusTypeDef  I2Cx_IsDeviceReady(uint16_t DevAddress, uint32_t Trials);
static void               I2Cx_Error (void);

#if defined(USE_IOEXPANDER)
/* Link function I2C for IO expanders */
static void               I2Cx_WriteData(uint16_t Addr, uint16_t Reg, uint16_t RegSize, uint8_t Value);
static uint8_t            I2Cx_ReadData(uint16_t Addr, uint16_t Reg, uint16_t RegSize);

/* IOExpander IO functions */
void                      IOE_Init(void);
void                      IOE_ITConfig(void);
void                      IOE_Delay(uint32_t Delay);
void                      IOE_Write(uint8_t Addr, uint8_t Reg, uint8_t Value);
uint8_t                   IOE_Read(uint8_t Addr, uint8_t Reg);
uint16_t                  IOE_ReadMultiple(uint8_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length);
#endif /* USE_IOEXPANDER */

/* Link function for EEPROM peripheral over I2C */
void                      EEPROM_I2C_IO_Init(void);
HAL_StatusTypeDef         EEPROM_I2C_IO_WriteData(uint16_t DevAddress, uint16_t MemAddress, uint8_t* pBuffer, uint32_t BufferSize);
HAL_StatusTypeDef         EEPROM_I2C_IO_ReadData(uint16_t DevAddress, uint16_t MemAddress, uint8_t* pBuffer, uint32_t BufferSize);
HAL_StatusTypeDef         EEPROM_I2C_IO_IsDeviceReady(uint16_t DevAddress, uint32_t Trials);

/* Link function for Audio Codec peripheral */
void                      AUDIO_IO_Init(void);
void                      AUDIO_IO_DeInit(void);
void                      AUDIO_IO_Write(uint8_t Addr, uint16_t Reg, uint16_t Value);
uint16_t                  AUDIO_IO_Read(uint8_t Addr, uint16_t Reg);
void                      AUDIO_IO_Delay(uint32_t delay);
#endif/* HAL_I2C_MODULE_ENABLED */

/**
  * @}
  */ 

/** @defgroup STM32L476G_EVAL_Exported_Functions Exported Functions
  * @{
  */ 

/**
  * @brief  This method returns the STM32L476G EVAL BSP Driver revision
  * @retval version : 0xXYZR (8bits for each decimal, R for RC)
  */
uint32_t BSP_GetVersion(void)
{
  return __STM32L476G_EVAL_BSP_VERSION;
}

/**
  * @brief  Configures LED GPIO.
  * @param  Led: Specifies the Led to be configured. 
  *   This parameter can be one of following parameters:
  *     @arg LED1
  *     @arg LED2
  *     @arg LED3
  *     @arg LED4
  * @retval None
  */
void BSP_LED_Init(Led_TypeDef Led)
{
  GPIO_InitTypeDef  gpioinitstruct = {0};
  
#if defined(USE_STM32L476G_EVAL_REVA)
  if (Led == LED1)
#elif defined(USE_STM32L476G_EVAL_REVB)
  if ((Led == LED1) || (Led == LED3))
#endif
  {
    /* Enable the GPIO_LED clock */
    LEDx_GPIO_CLK_ENABLE(Led);

    /* Configure the GPIO_LED pin */
    gpioinitstruct.Pin    = LED_PIN[Led];
    gpioinitstruct.Mode   = GPIO_MODE_OUTPUT_PP;
    gpioinitstruct.Pull   = GPIO_PULLUP;
    gpioinitstruct.Speed  = GPIO_SPEED_FREQ_VERY_HIGH;

    HAL_GPIO_Init(LED_PORT[Led], &gpioinitstruct);

    /* By default, turn off LED */
    HAL_GPIO_WritePin(LED_PORT[Led], LED_PIN[Led], GPIO_PIN_SET);
  }
#if defined(USE_IOEXPANDER)
  else
  {
    /* Initialize the IO functionalities */
    BSP_IO_Init();
    
    BSP_IO_ConfigPin(LED_PIN[Led],IO_MODE_OUTPUT);
    
    /* By default, turn off LED */
    BSP_IO_WritePin(LED_PIN[Led], GPIO_PIN_SET);
  }
#endif /* USE_IOEXPANDER */     
}

/**
  * @brief  Turns selected LED On.
  * @param  Led: Specifies the Led to be set on. 
  *   This parameter can be one of following parameters:
  *     @arg LED1
  *     @arg LED2
  *     @arg LED3
  *     @arg LED4
  * @retval None
  */
void BSP_LED_On(Led_TypeDef Led)
{
#if defined(USE_STM32L476G_EVAL_REVA)
  if (Led == LED1)
#elif defined(USE_STM32L476G_EVAL_REVB)
  if ((Led == LED1) || (Led == LED3))
#endif
  {
    HAL_GPIO_WritePin(LED_PORT[Led], LED_PIN[Led], GPIO_PIN_RESET);
  }
#if defined(USE_IOEXPANDER)
  else
  {
    BSP_IO_WritePin(LED_PIN[Led], GPIO_PIN_RESET);
  }
#endif /* USE_IOEXPANDER */
}

/**
  * @brief  Turns selected LED Off.
  * @param  Led: Specifies the Led to be set off. 
  *   This parameter can be one of following parameters:
  *     @arg LED1
  *     @arg LED2
  *     @arg LED3
  *     @arg LED4
  * @retval None
  */
void BSP_LED_Off(Led_TypeDef Led)
{
#if defined(USE_STM32L476G_EVAL_REVA)
  if (Led == LED1)
#elif defined(USE_STM32L476G_EVAL_REVB)
  if ((Led == LED1) || (Led == LED3))
#endif
  {
    HAL_GPIO_WritePin(LED_PORT[Led], LED_PIN[Led], GPIO_PIN_SET);
  }
#if defined(USE_IOEXPANDER)
  else
  {
    BSP_IO_WritePin(LED_PIN[Led], GPIO_PIN_SET);
  }
#endif /* USE_IOEXPANDER */
}

/**
  * @brief  Toggles the selected LED.
  * @param  Led: Specifies the Led to be toggled. 
  *   This parameter can be one of following parameters:
  *     @arg LED1
  *     @arg LED2
  *     @arg LED3
  *     @arg LED4
  * @retval None
  */
void BSP_LED_Toggle(Led_TypeDef Led)
{
#if defined(USE_STM32L476G_EVAL_REVA)
  if (Led == LED1)
#elif defined(USE_STM32L476G_EVAL_REVB)
  if ((Led == LED1) || (Led == LED3))
#endif
  {
    HAL_GPIO_TogglePin(LED_PORT[Led], LED_PIN[Led]);
  }
#if defined(USE_IOEXPANDER)
  else
  {
    BSP_IO_TogglePin(LED_PIN[Led]);
  }
#endif /* USE_IOEXPANDER */
}

/**
  * @brief  Configures push button GPIO and EXTI Line.
  * @param  Button: Button to be configured.
  *   This parameter can be one of the following values: 
  *     @arg BUTTON_TAMPER: Wakeup/Tamper Push Button 
  * @param  Button_Mode: Button mode requested.
  *   This parameter can be one of the following values:   
  *     @arg BUTTON_MODE_GPIO: Button will be used as simple IO 
  *     @arg BUTTON_MODE_EXTI: Button will be connected to EXTI line
  *                            with interrupt generation capability
  * @retval None
  */
void BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef Button_Mode)
{
  GPIO_InitTypeDef gpioinitstruct = {0};

  if (Button == BUTTON_TAMPER)
  {
    /* Enable the corresponding Push Button clock */
    BUTTONx_GPIO_CLK_ENABLE(Button);
	
    /* Configure Push Button pin as input */
    gpioinitstruct.Pin = BUTTON_PIN[Button];
    gpioinitstruct.Pull = GPIO_PULLDOWN;
    gpioinitstruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    if (Button_Mode == BUTTON_MODE_GPIO)
    {
      /* Configure Button pin as input */
      gpioinitstruct.Mode = GPIO_MODE_INPUT;
      HAL_GPIO_Init(BUTTON_PORT[Button], &gpioinitstruct);
    }
    else if (Button_Mode == BUTTON_MODE_EXTI)
    {
      /* Configure Tamper/Wake-up Push Button pin as input with External interrupt, falling edge */
      gpioinitstruct.Mode = GPIO_MODE_IT_FALLING;
  
      HAL_GPIO_Init(BUTTON_PORT[Button], &gpioinitstruct);

      /* Enable and set Button EXTI Interrupt to the lowest priority */
      HAL_NVIC_SetPriority((IRQn_Type)(BUTTON_IRQn[Button]), 0x0F, 0);
      HAL_NVIC_EnableIRQ((IRQn_Type)(BUTTON_IRQn[Button]));
    }
  }
}

/**
  * @brief  Returns the selected button state.
  * @param  Button: Button to be checked.
  *   This parameter can be one of the following values:
  *     @arg BUTTON_TAMPER: Wakeup/Tamper Push Button 
  * @retval Button state
  */
uint32_t BSP_PB_GetState(Button_TypeDef Button)
{
  return HAL_GPIO_ReadPin(BUTTON_PORT[Button], BUTTON_PIN[Button]);
}

#if defined(USE_IOEXPANDER)
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
  BSP_IO_Init();
  
  /* Configure joystick pins in IT mode */
  if(Joy_Mode == JOY_MODE_EXTI)
  {
    BSP_IO_ConfigPin(JOY_ALL_PINS, IO_MODE_IT_FALLING_EDGE);
    
    /* Configure IO Expander interrupt */
    IOE_ITConfig();
  }
  else
  {
    /* GPIO mode configuration */
    BSP_IO_ConfigPin(JOY_ALL_PINS,IO_MODE_INPUT);
  }
  
  return HAL_OK;
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
  uint32_t tmp = 0;   
  
  /* Read the status  of all the joystick pins */
  tmp = (uint32_t)BSP_IO_ReadPin(JOY_ALL_PINS);
  
  /* Check the pressed joystick button */  
  if(!(tmp & SEL_JOY_PIN))
  {
    return(JOYState_TypeDef) JOY_SEL;
  }
  else if(!(tmp & DOWN_JOY_PIN))
  {
    return(JOYState_TypeDef) JOY_DOWN;
  } 
  else if(!(tmp & LEFT_JOY_PIN))
  {
    return(JOYState_TypeDef) JOY_LEFT;
  }
  else if(!(tmp & RIGHT_JOY_PIN))
  {
    return(JOYState_TypeDef) JOY_RIGHT;
  }
  else if(!(tmp & UP_JOY_PIN))
  {
    return(JOYState_TypeDef) JOY_UP;
  }
  else
  { 
    return(JOYState_TypeDef) JOY_NONE;
  }
}
#endif /* USE_IOEXPANDER */

#ifdef HAL_UART_MODULE_ENABLED
/**
  * @brief  Configures COM port.
  * @param  COM: Specifies the COM port to be configured.
  *   This parameter can be one of following parameters:    
  *     @arg COM1
  * @param  huart: pointer to a UART_HandleTypeDef structure that
  *   contains the configuration information for the specified UART peripheral.
  * @retval None
  */
void BSP_COM_Init(COM_TypeDef COM, UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef gpioinitstruct = {0};

  /* Enable GPIO clock */
  COMx_TX_GPIO_CLK_ENABLE(COM);
  COMx_RX_GPIO_CLK_ENABLE(COM);

  /* Enable USART clock */
  COMx_CLK_ENABLE(COM);

  /* Configure USART Tx as alternate function push-pull */
  gpioinitstruct.Pin = COM_TX_PIN[COM];
  gpioinitstruct.Mode = GPIO_MODE_AF_PP;
  gpioinitstruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpioinitstruct.Pull = GPIO_PULLUP;
  gpioinitstruct.Alternate = COM_TX_AF[COM];
  HAL_GPIO_Init(COM_TX_PORT[COM], &gpioinitstruct);
    
  /* Configure USART Rx as alternate function push-pull */
  gpioinitstruct.Pin = COM_RX_PIN[COM];
  gpioinitstruct.Alternate = COM_RX_AF[COM];
  HAL_GPIO_Init(COM_RX_PORT[COM], &gpioinitstruct);

  /* USART configuration */
  huart->Instance = COM_USART[COM];
  HAL_UART_Init(huart);
}
#endif /* HAL_UART_MODULE_ENABLED */

/**
  * @}
  */ 

/** @defgroup STM32L476G_EVAL_BusOperations_Functions Bus Operations Functions
  * @{
  */ 

/*******************************************************************************
                            BUS OPERATIONS
*******************************************************************************/
#ifdef HAL_SRAM_MODULE_ENABLED
/********************************* LINK LCD ***********************************/

/**
  * @brief  Initializes LCD low level.
  * @retval None
  */
void LCD_IO_Init(void) 
{
  BSP_IO_Init();

  BSP_IO_ConfigPin(IO2_PIN_9, IO_MODE_OUTPUT);
  
  /* By default, turn on Backlight */
  BSP_IO_WritePin(IO2_PIN_9, GPIO_PIN_RESET);

  FMC_BANK3_Init();
}

/**
  * @brief  Writes data on LCD data register.
  * @param  pData: Data to be written
  * @param  Size: number of data to write
  * @retval None
  */
void LCD_IO_WriteMultipleData(uint8_t *pData, uint32_t Size) 
{
  uint32_t counter = 0;
  uint16_t regvalue;
  
  regvalue = *pData | (*(pData+1) << 8);

  for (counter = Size; counter != 0; counter--)
  {
    /* Write 16-bit Reg */
    FMC_BANK3_WriteData(regvalue);
    counter--;
    pData += 2;
    regvalue = *pData | (*(pData+1) << 8);
  }
}

/**
  * @brief  Writes register on LCD register.
  * @param  Reg: Register to be written
  * @retval None
  */
void LCD_IO_WriteReg(uint8_t Reg) 
{
  /* Write 16-bit Index, then Write Reg */
  FMC_BANK3_WriteReg(Reg);
}

/**
  * @brief  Reads data from LCD data register.
  * @retval Read data.
  */
uint16_t LCD_IO_ReadData(uint8_t Reg) 
{
  return FMC_BANK3_ReadData(Reg);
}

/**
  * @brief  Wait for loop in ms.
  * @param  Delay in ms.
  * @retval None
  */
void LCD_Delay (uint32_t Delay)
{
  HAL_Delay(Delay);
}

/*************************** FMC Routines ************************************/
/**
  * @brief  Initializes FMC_BANK3 MSP.
  * @retval None
  */
static void FMC_BANK3_MspInit(void)
{
  GPIO_InitTypeDef gpioinitstruct = {0};

  /* Enable FMC clock */
  __HAL_RCC_FMC_CLK_ENABLE();

  /* Enable GPIOs clock */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  
  /* Common GPIO configuration */
  gpioinitstruct.Mode      = GPIO_MODE_AF_PP;
  gpioinitstruct.Pull      = GPIO_PULLDOWN;
  gpioinitstruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  gpioinitstruct.Alternate = GPIO_AF12_FMC;
  
  /*## Data Bus #######*/
  /* GPIOD configuration */
  gpioinitstruct.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_8 | GPIO_PIN_9 |
                         GPIO_PIN_10 | GPIO_PIN_14 | GPIO_PIN_15;
  HAL_GPIO_Init(GPIOD, &gpioinitstruct);

  /* GPIOE configuration */  
  gpioinitstruct.Pin   = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 |
                         GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 |
                         GPIO_PIN_14 | GPIO_PIN_15;
  HAL_GPIO_Init(GPIOE, &gpioinitstruct);
  
  /*## Address Bus #######*/
  /* GPIOF configuration */  
  gpioinitstruct.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
                         GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_12 | GPIO_PIN_13 |
                         GPIO_PIN_14 | GPIO_PIN_15;
  HAL_GPIO_Init(GPIOF, &gpioinitstruct);

  /* GPIOG configuration */  
  gpioinitstruct.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 |
                         GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
  HAL_GPIO_Init(GPIOG, &gpioinitstruct);
  
  /* GPIOD configuration */
  gpioinitstruct.Pin   = GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13;
  HAL_GPIO_Init(GPIOD, &gpioinitstruct);

  /* GPIOE configuration */
  gpioinitstruct.Pin   = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 |
                         GPIO_PIN_6 | GPIO_PIN_2;
  HAL_GPIO_Init(GPIOE, &gpioinitstruct);

  /*## NOE and NWE configuration #######*/ 
  gpioinitstruct.Pin = GPIO_PIN_4 |GPIO_PIN_5;
  HAL_GPIO_Init(GPIOD, &gpioinitstruct);
  
  /*## NE3 configuration #######*/
  gpioinitstruct.Pin = GPIO_PIN_10;
  HAL_GPIO_Init(GPIOG, &gpioinitstruct);
  
  /*## LCD_DO, NBL1 configuration #######*/
  gpioinitstruct.Pin = GPIO_PIN_6;
  HAL_GPIO_Init(GPIOA, &gpioinitstruct);
}

/**
  * @brief  Initializes LCD IO.
  * @retval None
  */
static void FMC_BANK3_Init(void) 
{  
  SRAM_HandleTypeDef hsram;
  FMC_NORSRAM_TimingTypeDef  sramtiming = {0};
  FMC_NORSRAM_TimingTypeDef  extendedtiming = {0};
  
  /*** LCD/CS is CE3 - Bank 3 of NOR/SRAM Bank 1~4 ***/  
  hsram.Instance  = FMC_NORSRAM_DEVICE;
  hsram.Extended  = FMC_NORSRAM_EXTENDED_DEVICE;

  /* Timings for READ accesses */
  sramtiming.AddressSetupTime      = 1;
  sramtiming.AddressHoldTime       = 1;
  sramtiming.DataSetupTime         = 32;
  sramtiming.BusTurnAroundDuration = 15;
  sramtiming.CLKDivision           = 16;
  sramtiming.DataLatency           = 17;
  sramtiming.AccessMode            = FMC_ACCESS_MODE_D;
  
  /* Extended timings for WRITE accesses */
  extendedtiming.AddressSetupTime      = 1;
  extendedtiming.AddressHoldTime       = 1;
  extendedtiming.DataSetupTime         = 5;
  extendedtiming.BusTurnAroundDuration = 2;
  extendedtiming.CLKDivision           = sramtiming.CLKDivision;
  extendedtiming.DataLatency           = sramtiming.DataLatency;
  extendedtiming.AccessMode            = sramtiming.AccessMode;

  hsram.Init.NSBank             = FMC_NORSRAM_BANK3;
  hsram.Init.DataAddressMux     = FMC_DATA_ADDRESS_MUX_DISABLE;
  hsram.Init.MemoryType         = FMC_MEMORY_TYPE_SRAM;
  hsram.Init.MemoryDataWidth    = FMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram.Init.BurstAccessMode    = FMC_BURST_ACCESS_MODE_DISABLE;
  hsram.Init.WaitSignalPolarity = FMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram.Init.WaitSignalActive   = FMC_WAIT_TIMING_BEFORE_WS;
  hsram.Init.WriteOperation     = FMC_WRITE_OPERATION_ENABLE;
  hsram.Init.WaitSignal         = FMC_WAIT_SIGNAL_DISABLE;
  hsram.Init.ExtendedMode       = FMC_EXTENDED_MODE_ENABLE;
  hsram.Init.AsynchronousWait   = FMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram.Init.WriteBurst         = FMC_WRITE_BURST_DISABLE;
  hsram.Init.ContinuousClock    = FMC_CONTINUOUS_CLOCK_SYNC_ASYNC;
  hsram.Init.PageSize           = FMC_PAGE_SIZE_NONE;

  /* Initialize the SRAM controller */
  FMC_BANK3_MspInit();
  HAL_SRAM_Init(&hsram, &sramtiming, &extendedtiming);
}

/**
  * @brief  Writes register value.
  * @param  Data: 
  * @retval None
  */
static void FMC_BANK3_WriteData(uint16_t Data) 
{
  /* Write 16-bit Data */
  TFT_LCD->LCD_RAM_W = Data;
}

/**
  * @brief  Writes register address.
  * @param  Reg: 
  * @retval None
  */
static void FMC_BANK3_WriteReg(uint8_t Reg) 
{
  /* Write 16-bit Index, then Write Reg */
  TFT_LCD->LCD_REG_W = Reg;
}

/**
  * @brief  Reads register value.
  * @retval Read value
  */
static uint16_t FMC_BANK3_ReadData(uint8_t Reg) 
{
  /* Read 16-bit Reg */
  return (TFT_LCD->LCD_RAM_R);
}
#endif /* HAL_SRAM_MODULE_ENABLED */

#ifdef HAL_I2C_MODULE_ENABLED
/******************************* I2C Routines**********************************/

/**
  * @brief Eval I2Cx MSP Initialization
  * @param hi2c: I2C handle
  * @retval None
  */
static void I2Cx_MspInit(I2C_HandleTypeDef *hi2c)
{
  GPIO_InitTypeDef  gpioinitstruct = {0};
  RCC_PeriphCLKInitTypeDef  RCC_PeriphCLKInitStruct;

  /* IOSV bit MUST be set to access GPIO port G[2:15] */
  __HAL_RCC_PWR_CLK_ENABLE();
  SET_BIT(PWR->CR2, PWR_CR2_IOSV); 
  
  if (hi2c->Instance == EVAL_I2Cx)
  {
    /*##-1- Configure the Eval I2C clock source. The clock is derived from the SYSCLK #*/
    RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
    RCC_PeriphCLKInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
    HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);

    /*##-2- Configure the GPIOs ################################################*/  

    /* Enable GPIO clock */
    EVAL_I2Cx_SDA_GPIO_CLK_ENABLE();
    EVAL_I2Cx_SCL_GPIO_CLK_ENABLE();
      
    /* Configure I2C Rx/Tx as alternate function  */
    gpioinitstruct.Pin       = EVAL_I2Cx_SCL_PIN | EVAL_I2Cx_SDA_PIN;
    gpioinitstruct.Mode      = GPIO_MODE_AF_OD;
    gpioinitstruct.Pull      = GPIO_PULLUP;
    gpioinitstruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpioinitstruct.Alternate = EVAL_I2Cx_SCL_SDA_AF;
    HAL_GPIO_Init(EVAL_I2Cx_SCL_GPIO_PORT, &gpioinitstruct);
      
    /*##-3- Configure the Eval I2Cx peripheral #######################################*/ 
    /* Enable Eval_I2Cx clock */
    EVAL_I2Cx_CLK_ENABLE();
    
    /* Force the I2C Peripheral Clock Reset */  
    EVAL_I2Cx_FORCE_RESET();
      
    /* Release the I2C Peripheral Clock Reset */  
    EVAL_I2Cx_RELEASE_RESET(); 
    
    /* Enable and set Eval I2Cx Interrupt to the highest priority */
    HAL_NVIC_SetPriority(EVAL_I2Cx_EV_IRQn, 0x00, 0);
    HAL_NVIC_EnableIRQ(EVAL_I2Cx_EV_IRQn);
    
    /* Enable and set Eval I2Cx Interrupt to the highest priority */
    HAL_NVIC_SetPriority(EVAL_I2Cx_ER_IRQn, 0x00, 0);
    HAL_NVIC_EnableIRQ(EVAL_I2Cx_ER_IRQn); 
  }
}

/**
  * @brief Eval I2Cx Bus initialization
  * @retval None
  */
static void I2Cx_Init(void)
{
  if(HAL_I2C_GetState(&heval_I2c) == HAL_I2C_STATE_RESET)
  {
    heval_I2c.Instance              = EVAL_I2Cx;
    heval_I2c.Init.Timing           = EVAL_I2Cx_TIMING;
    heval_I2c.Init.OwnAddress1      = 0;
    heval_I2c.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    heval_I2c.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    heval_I2c.Init.OwnAddress2      = 0;
    heval_I2c.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    heval_I2c.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;  

    HAL_I2C_DeInit(&heval_I2c);
    /* Init the I2C */
    I2Cx_MspInit(&heval_I2c);
    HAL_I2C_Init(&heval_I2c);
  }
}

/**
  * @brief Eval I2Cx MSP Deinitialization
  * @param hi2c: I2C handle
  * @retval None
  */
static void I2Cx_MspDeInit(I2C_HandleTypeDef *hi2c)
{
}

/**
  * @brief Eval I2Cx Bus deinitialization
  * @retval None
  */
static void I2Cx_DeInit(void)
{
  if(HAL_I2C_GetState(&heval_I2c) != HAL_I2C_STATE_RESET)
  {
    /* Deinit the I2C */
    HAL_I2C_DeInit(&heval_I2c);
    I2Cx_MspDeInit(&heval_I2c);
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
static HAL_StatusTypeDef I2Cx_WriteBuffer(uint16_t Addr, uint16_t Reg, uint16_t RegSize, uint8_t *pBuffer, uint16_t Length)
{
  HAL_StatusTypeDef status = HAL_OK;
  
  status = HAL_I2C_Mem_Write(&heval_I2c, Addr, (uint16_t)Reg, RegSize, pBuffer, Length, heval_I2cxTimeout); 

/* Check the communication status */
  if(status != HAL_OK)
  {
    /* Re-Initiaize the BUS */
    I2Cx_Error();
  }        
  return status;
}

#if defined(USE_IOEXPANDER)
/**
  * @brief  Write a value in a register of the device through BUS.
  * @param  Addr: Device address on BUS Bus.  
  * @param  Reg: The target register address to write
  * @param  RegSize: The target register size (can be 8BIT or 16BIT)
  * @param  Value: The target register value to be written 
  * @retval None 
  */
static void I2Cx_WriteData(uint16_t Addr, uint16_t Reg, uint16_t RegSize, uint8_t Value)
  {
  HAL_StatusTypeDef status = HAL_OK;
  
  status = HAL_I2C_Mem_Write(&heval_I2c, Addr, (uint16_t)Reg, RegSize, &Value, 1, heval_I2cxTimeout); 
  
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Re-Initiaize the BUS */
    I2Cx_Error();
  }        
}

/**
  * @brief  Read a register of the device through BUS
  * @param  Addr: Device address on BUS
  * @param  Reg: The target register address to read
  * @param  RegSize: The target register size (can be 8BIT or 16BIT)
  * @retval read register value
  */
static uint8_t I2Cx_ReadData(uint16_t Addr, uint16_t Reg, uint16_t RegSize)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint8_t value = 0x0;
  
  status = HAL_I2C_Mem_Read(&heval_I2c, Addr, Reg, RegSize, &value, 1, heval_I2cxTimeout);
    
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    I2Cx_Error();
  
  }
  return value;
}
#endif /* USE_IOEXPANDER */

/**
  * @brief  Reads multiple data on the BUS.
  * @param  Addr: I2C Address
  * @param  Reg: Reg Address 
  * @param  RegSize : The target register size (can be 8BIT or 16BIT)
  * @param  pBuffer: pointer to read data buffer
  * @param  Length: length of the data
  * @retval 0 if no problems to read multiple data
  */
static HAL_StatusTypeDef I2Cx_ReadBuffer(uint16_t Addr, uint16_t Reg, uint16_t RegSize, uint8_t *pBuffer, uint16_t Length)
{
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_I2C_Mem_Read(&heval_I2c, Addr, (uint16_t)Reg, RegSize, pBuffer, Length, heval_I2cxTimeout);
  
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Re-Initiaize the BUS */
    I2Cx_Error();
  }        
  return status;
}

/**
* @brief  Checks if target device is ready for communication. 
* @note   This function is used with Memory devices
* @param  DevAddress: Target device address
* @param  Trials: Number of trials
* @retval HAL status
*/
static HAL_StatusTypeDef I2Cx_IsDeviceReady(uint16_t DevAddress, uint32_t Trials)
{ 
  return (HAL_I2C_IsDeviceReady(&heval_I2c, DevAddress, Trials, heval_I2cxTimeout));
}


/**
  * @brief Eval I2Cx error treatment function
  * @retval None
  */
static void I2Cx_Error (void)
{
  /* De-initialize the I2C communication BUS */
  HAL_I2C_DeInit(&heval_I2c);
  
  /* Re- Initiaize the I2C communication BUS */
  I2Cx_Init();
}

/*******************************************************************************
                            LINK OPERATIONS
*******************************************************************************/
#if defined(USE_IOEXPANDER)
/***************************** LINK IOE ***************************************/

/**
  * @brief  Initializes IOE low level.
  * @retval None
  */
void IOE_Init(void) 
{
  I2Cx_Init();
}

/**
  * @brief  Configures IOE low level Interrupt.
  * @retval None
  */
void IOE_ITConfig(void)
{
  static uint8_t IOE_IT_Enabled = 0;  
  GPIO_InitTypeDef gpioinitstruct = {0};

  if(IOE_IT_Enabled == 0)
  {
    IOE_IT_Enabled = 1;
    
    /* Configure IOExpander Interrupt GPIO */
    gpioinitstruct.Pin   = GPIO_PIN_15;
    gpioinitstruct.Pull  = GPIO_NOPULL;
    gpioinitstruct.Speed = GPIO_SPEED_FREQ_HIGH;
    gpioinitstruct.Mode  = GPIO_MODE_IT_FALLING;
    HAL_GPIO_Init(GPIOG, &gpioinitstruct);
    
    /* All joystick buttons share the same IRQ Handler */
    HAL_NVIC_SetPriority(RIGHT_JOY_EXTI_IRQn, 0x0F, 0);
    HAL_NVIC_EnableIRQ(RIGHT_JOY_EXTI_IRQn);
  }
}

/**
  * @brief  IOE writes single data.
  * @param  Addr: I2C address
  * @param  Reg: Reg address 
  * @param  Value: Data to be written
  * @retval None
  */
void IOE_Write(uint8_t Addr, uint8_t Reg, uint8_t Value)
{
  I2Cx_WriteData(Addr, Reg, I2C_MEMADD_SIZE_8BIT, Value);
}

/**
  * @brief  IOE reads single data.
  * @param  Addr: I2C address
  * @param  Reg: Reg address 
  * @retval Read data
  */
uint8_t IOE_Read(uint8_t Addr, uint8_t Reg)
{
  return I2Cx_ReadData(Addr, Reg, I2C_MEMADD_SIZE_8BIT);
}

/**
  * @brief  IOE reads multiple data.
  * @param  Addr: I2C address
  * @param  Reg: Reg address 
  * @param  Buffer: Pointer to data buffer
  * @param  Length: Length of the data
  */
uint16_t IOE_ReadMultiple(uint8_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length)
{
  return I2Cx_ReadBuffer(Addr, Reg, I2C_MEMADD_SIZE_8BIT, Buffer, Length);
}


/** 
  * @brief  IOE Write multiple data.
  * @param  Addr: I2C address
  * @param  Reg: Reg address 
  * @param  Buffer: Pointer to data buffer
  * @param  Length: Length of the data
  */
void IOE_WriteMultiple(uint8_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length)
{
  I2Cx_WriteBuffer(Addr, Reg, I2C_MEMADD_SIZE_8BIT, Buffer, Length);
}

/**
  * @brief  IOE delay.
  * @param  Delay: Delay in ms
  * @retval None
  */
void IOE_Delay(uint32_t Delay)
{
  HAL_Delay(Delay);
}
#endif /* USE_IOEXPANDER */

/********************************* LINK AUDIO *********************************/

/**
  * @brief  Initializes Audio low level.
  * @retval None
  */
void AUDIO_IO_Init(void) 
{
  I2Cx_Init();
}

/**
  * @brief  Deinitializes Audio low level.
  * @retval None
  */
void AUDIO_IO_DeInit(void) 
{
  I2Cx_DeInit();
}

/**
  * @brief  Writes a single data.
  * @param  Addr: I2C address
  * @param  Reg: Reg address 
  * @param  Value: Data to be written
  * @retval None
  */
void AUDIO_IO_Write(uint8_t Addr, uint16_t Reg, uint16_t Value)
{
  uint16_t tmp = Value;
  
  Value = ((uint16_t)(tmp >> 8) & 0x00FF);
  Value |= ((uint16_t)(tmp << 8)& 0xFF00);
  
  I2Cx_WriteBuffer(Addr, Reg, I2C_MEMADD_SIZE_16BIT,(uint8_t*)&Value, 2);
}

/**
  * @brief  Reads a single data.
  * @param  Addr: I2C address
  * @param  Reg: Reg address 
  * @retval Data to be read
  */
uint16_t AUDIO_IO_Read(uint8_t Addr, uint16_t Reg)
{
  uint16_t Read_Value = 0, tmp = 0;
  
  I2Cx_ReadBuffer(Addr, Reg, I2C_MEMADD_SIZE_16BIT, (uint8_t*)&Read_Value, 2); 
  
  tmp = ((uint16_t)(Read_Value >> 8) & 0x00FF);
  tmp |= ((uint16_t)(Read_Value << 8)& 0xFF00);
  
  Read_Value = tmp;
  
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

/********************************* LINK I2C EEPROM *****************************/
/**
  * @brief  Initializes peripherals used by the I2C EEPROM driver.
  * @retval None
  */
void EEPROM_I2C_IO_Init(void)
{
  I2Cx_Init();
}

/**
  * @brief  Write data to I2C EEPROM driver
  * @param  DevAddress: Target device address
  * @param  MemAddress: Internal memory address
  * @param  pBuffer: Pointer to data buffer
  * @param  BufferSize: Amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef EEPROM_I2C_IO_WriteData(uint16_t DevAddress, uint16_t MemAddress, uint8_t* pBuffer, uint32_t BufferSize)
{
  return (I2Cx_WriteBuffer(DevAddress, MemAddress, I2C_MEMADD_SIZE_16BIT, pBuffer, BufferSize));
}

/**
  * @brief  Read data from I2C EEPROM driver
  * @param  DevAddress: Target device address
  * @param  MemAddress: Internal memory address
  * @param  pBuffer: Pointer to data buffer
  * @param  BufferSize: Amount of data to be read
  * @retval HAL status
  */
HAL_StatusTypeDef EEPROM_I2C_IO_ReadData(uint16_t DevAddress, uint16_t MemAddress, uint8_t* pBuffer, uint32_t BufferSize)
{
  return (I2Cx_ReadBuffer(DevAddress, MemAddress, I2C_MEMADD_SIZE_16BIT, pBuffer, BufferSize));
}

/**
* @brief  Checks if target device is ready for communication. 
* @note   This function is used with Memory devices
* @param  DevAddress: Target device address
* @param  Trials: Number of trials
* @retval HAL status
*/
HAL_StatusTypeDef EEPROM_I2C_IO_IsDeviceReady(uint16_t DevAddress, uint32_t Trials)
{ 
  return (I2Cx_IsDeviceReady(DevAddress, Trials));
}

#endif /*HAL_I2C_MODULE_ENABLED*/


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



