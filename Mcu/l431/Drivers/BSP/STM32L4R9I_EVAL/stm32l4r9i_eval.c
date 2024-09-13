/**
  ******************************************************************************
  * @file    stm32l4r9i_eval.c
  * @author  MCD Application Team
  * @brief   This file provides a set of firmware functions to manage Leds,
  *          push-button and COM ports for STM32L4R9I_EVAL
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

/* Includes ------------------------------------------------------------------*/
#include "stm32l4r9i_eval.h"
#include "stm32l4r9i_eval_io.h"

/** @addtogroup BSP
  * @{
  */

/** @defgroup STM32L4R9I_EVAL STM32L4R9I_EVAL
  * @{
  */

/** @defgroup STM32L4R9I_EVAL_Common STM32L4R9I_EVAL Common
  * @{
  */

/** @defgroup STM32L4R9I_EVAL_Private_TypesDefinitions Private Types Definitions
  * @brief This file provides firmware functions to manage Leds, push-buttons,
  *        Joystick, and COM ports available on STM32L4R9I-EVAL evaluation board
  *        from STMicroelectronics.
  * @{
  */

/**
  * @}
  */

/** @defgroup STM32L4R9I_EVAL_Exported_Defines Exported Defines
  * @{
  */

/**
 * @brief STM32L4R9I EVAL BSP Driver version number
   */
#define __STM32L4R9I_EVAL_BSP_VERSION_MAIN   (0x01) /*!< [31:24] main version */
#define __STM32L4R9I_EVAL_BSP_VERSION_SUB1   (0x01) /*!< [23:16] sub1 version */
#define __STM32L4R9I_EVAL_BSP_VERSION_SUB2   (0x02) /*!< [15:8]  sub2 version */
#define __STM32L4R9I_EVAL_BSP_VERSION_RC     (0x00) /*!< [7:0]  release candidate */
#define __STM32L4R9I_EVAL_BSP_VERSION            ((__STM32L4R9I_EVAL_BSP_VERSION_MAIN << 24)\
                                                 |(__STM32L4R9I_EVAL_BSP_VERSION_SUB1 << 16)\
                                                 |(__STM32L4R9I_EVAL_BSP_VERSION_SUB2 << 8 )\
                                                 |(__STM32L4R9I_EVAL_BSP_VERSION_RC))

#ifdef HAL_I2C_MODULE_ENABLED
/**
  * @brief BSP I2C users
  */
#define BSP_I2C_NO_USER     0x00000000U
#define BSP_I2C_AUDIO_USER  0x00000001U
#define BSP_I2C_TS_USER     0x00000002U
#define BSP_I2C_MFX_USER    0x00000004U
#define BSP_I2C_EEPROM_USER 0x00000008U
#define BSP_I2C_ALL_USERS   0x0000000FU
#endif

/**
  * @}
  */


/** @defgroup STM32L4R9I_EVAL_Exported_Variables Exported Variables
  * @{
  */

/**
 * @brief LED variables
 */
GPIO_TypeDef*   LED_PORT[LEDn] =                {LED1_GPIO_PORT,
                                                 LED2_GPIO_PORT,
                                                 LED3_GPIO_PORT,
                                                 LED4_GPIO_PORT};

const uint16_t  LED_PIN[LEDn] =                 {LED1_PIN,
                                                 LED2_PIN,
                                                 LED3_PIN,
                                                 LED4_PIN};

/**
 * @brief BUTTON variables
 */
GPIO_TypeDef*   BUTTON_PORT[BUTTONn] =          {WAKEUP_BUTTON_GPIO_PORT,
                                                 (GPIO_TypeDef *)0xFFFF,
                                                 (GPIO_TypeDef *)0xFFFF,
                                                 (GPIO_TypeDef *)0xFFFF,
                                                 (GPIO_TypeDef *)0xFFFF,
                                                 (GPIO_TypeDef *)0xFFFF};
/* 0xFFFF to mark as invalid port (Joystick buttons accessed through IOExpander) */

const uint16_t  BUTTON_PIN[BUTTONn] =           {WAKEUP_BUTTON_PIN,
                                                 SEL_JOY_PIN,
                                                 LEFT_JOY_PIN,
                                                 RIGHT_JOY_PIN,
                                                 DOWN_JOY_PIN,
                                                 UP_JOY_PIN};

const uint8_t   BUTTON_IRQn[BUTTONn] =          {WAKEUP_BUTTON_EXTI_IRQn,
                                                 SEL_JOY_EXTI_IRQn,
                                                 LEFT_JOY_EXTI_IRQn,
                                                 RIGHT_JOY_EXTI_IRQn,
                                                 DOWN_JOY_EXTI_IRQn,
                                                 UP_JOY_EXTI_IRQn};

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
uint32_t hi2c_evalTimeout = EVAL_I2Cx_TIMEOUT_MAX;   /*<! Value of Timeout when I2C communication fails */
I2C_HandleTypeDef hi2c_eval;
static uint32_t v_bspI2cUsers = BSP_I2C_NO_USER;
#if defined(BSP_USE_CMSIS_OS)
static osSemaphoreId BspI2cSemaphore;
#endif
#endif /* HAL_I2C_MODULE_ENABLED */

#ifdef HAL_ADC_MODULE_ENABLED
static ADC_HandleTypeDef hEvalADC = {0};
#endif

/**
  * @}
  */

/** @defgroup STM32L4R9I_EVAL_Private_FunctionPrototypes Private Functions
  * @{
  */
/* I2Cx bus function */
#ifdef HAL_I2C_MODULE_ENABLED
/* Link function I2C for EEPROM, Audio Codec peripheral and IO expanders*/
static void               I2Cx_Init(uint32_t user);
static void               I2Cx_MspInit(I2C_HandleTypeDef *hi2c);
static void               I2Cx_DeInit(uint32_t user);
static void               I2Cx_MspDeInit(I2C_HandleTypeDef *hi2c);
static HAL_StatusTypeDef  I2Cx_WriteBuffer(uint16_t Addr, uint16_t Reg, uint16_t RegSize, uint8_t *pBuffer, uint16_t Length);
static HAL_StatusTypeDef  I2Cx_ReadBuffer(uint16_t Addr, uint16_t Reg, uint16_t RegSize, uint8_t *pBuffer, uint16_t Length);
static HAL_StatusTypeDef  I2Cx_IsDeviceReady(uint16_t DevAddress, uint32_t Trials);
static void               I2Cx_Error (void);

/* Link function I2C for IO expanders */
static void               I2Cx_WriteData(uint16_t Addr, uint16_t Reg, uint16_t RegSize, uint8_t Value);
static void               I2Cx_WriteMultiple(uint16_t Addr, uint16_t Reg, uint16_t MemAddress, uint8_t *Buffer, uint16_t Length);
static uint8_t            I2Cx_ReadData(uint16_t Addr, uint16_t Reg, uint16_t RegSize);
static uint8_t            I2Cx_ReadMultiple(uint16_t Addr, uint16_t Reg, uint16_t MemAddress, uint8_t *Buffer, uint16_t Length);

/* Link function for IOExpander IO functions */
void                      MFX_IO_Init(void);
void                      MFX_IO_DeInit(void);
void                      MFX_IO_Wakeup(void);
void                      MFX_IO_EnableWakeupPin(void);
void                      MFX_IO_ITConfig(void);
void                      MFX_IO_Delay(uint32_t Delay);
void                      MFX_IO_Write(uint16_t Addr, uint8_t Reg, uint8_t Value);
uint8_t                   MFX_IO_Read(uint16_t Addr, uint8_t Reg);
uint16_t                  MFX_IO_ReadMultiple(uint16_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length);

/* Link function for TOUCHSCREEN IO functions */
void                      TS_IO_Init(void);
void                      TS_IO_Write(uint8_t Addr, uint8_t Reg, uint8_t Value);
uint8_t                   TS_IO_Read(uint8_t Addr, uint8_t Reg);
uint16_t                  TS_IO_ReadMultiple(uint8_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length);
void                      TS_IO_Delay(uint32_t Delay);

/* Link function for EEPROM peripheral over I2C */
void                      EEPROM_IO_Init(void);
HAL_StatusTypeDef         EEPROM_IO_WriteData(uint16_t DevAddress, uint16_t MemAddress, uint8_t* pBuffer, uint32_t BufferSize);
HAL_StatusTypeDef         EEPROM_IO_ReadData(uint16_t DevAddress, uint16_t MemAddress, uint8_t* pBuffer, uint32_t BufferSize);
HAL_StatusTypeDef         EEPROM_IO_IsDeviceReady(uint16_t DevAddress, uint32_t Trials);

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

/** @defgroup STM32L4R9I_EVAL_Exported_Functions Exported Functions
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
  * @brief  This method returns the STM32L4R9I EVAL BSP Driver revision
  * @retval version : 0xXYZR (8bits for each decimal, R for RC)
  */
uint32_t BSP_GetVersion(void)
{
  return __STM32L4R9I_EVAL_BSP_VERSION;
}

/**
  * @brief  Initialize LED GPIO.
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

/**
  * @brief  DeInitialize LED GPIO.
  * @param  Led: Specifies the Led to be configured.
  *   This parameter can be one of following parameters:
  *     @arg LED1
  *     @arg LED2
  *     @arg LED3
  *     @arg LED4
  * @retval None
  */
void BSP_LED_DeInit(Led_TypeDef Led)
{
  /* Enable the GPIO_LED clock */
  LEDx_GPIO_CLK_ENABLE(Led);

  HAL_GPIO_DeInit(LED_PORT[Led], LED_PIN[Led]);
}

/**
  * @brief  Turn selected LED On.
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
  HAL_GPIO_WritePin(LED_PORT[Led], LED_PIN[Led], GPIO_PIN_RESET);
}

/**
  * @brief  Turn selected LED Off.
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
  HAL_GPIO_WritePin(LED_PORT[Led], LED_PIN[Led], GPIO_PIN_SET);
}

/**
  * @brief  Toggle the selected LED.
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
  HAL_GPIO_TogglePin(LED_PORT[Led], LED_PIN[Led]);
}

/**
  * @brief  Initialize push button GPIO and EXTI Line.
  * @param  Button: Button to be configured.
  *   This parameter can be one of the following values:
  *     @arg BUTTON_WAKEUP: Wakeup/Tamper Push Button
  * @param  ButtonMode: Button mode requested.
  *   This parameter can be one of the following values:
  *     @arg BUTTON_MODE_GPIO: Button will be used as simple IO
  *     @arg BUTTON_MODE_EXTI: Button will be connected to EXTI line
  *                            with interrupt generation capability
  * @retval None
  */
void BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef ButtonMode)
{
  GPIO_InitTypeDef gpioinitstruct = {0};

  if (Button == BUTTON_WAKEUP)
  {
    /* Enable the corresponding Push Button clock */
    BUTTONx_GPIO_CLK_ENABLE(Button);

    /* Configure Push Button pin as input */
    gpioinitstruct.Pin = BUTTON_PIN[Button];
    gpioinitstruct.Pull = GPIO_PULLDOWN;
    gpioinitstruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    if (ButtonMode == BUTTON_MODE_GPIO)
    {
      /* Configure Button pin as input */
      gpioinitstruct.Mode = GPIO_MODE_INPUT;
      HAL_GPIO_Init(BUTTON_PORT[Button], &gpioinitstruct);
    }
    else
    {
      /* Configure Tamper/Wake-up Push Button pin as input with External interrupt, rising edge */
      gpioinitstruct.Mode = GPIO_MODE_IT_RISING;

      HAL_GPIO_Init(BUTTON_PORT[Button], &gpioinitstruct);

      /* Enable and set Button EXTI Interrupt to the lowest priority */
      HAL_NVIC_SetPriority((IRQn_Type)(BUTTON_IRQn[Button]), 0x0F, 0);
      HAL_NVIC_EnableIRQ((IRQn_Type)(BUTTON_IRQn[Button]));
    }
  }
}

/**
  * @brief  DeInitialize push button.
  * @param  Button: Button to be deinitialized.
  *   This parameter can be one of the following values:
  *     @arg BUTTON_WAKEUP: Wakeup/Tamper Push Button
  * @retval None
  */
void BSP_PB_DeInit(Button_TypeDef Button)
{
  if (Button == BUTTON_WAKEUP)
  {
    /* Enable the corresponding Push Button clock */
    BUTTONx_GPIO_CLK_ENABLE(Button);

    HAL_GPIO_DeInit(BUTTON_PORT[Button], BUTTON_PIN[Button]);

    /* Make sure Button EXTI Interrupt is disabled */
    HAL_NVIC_DisableIRQ((IRQn_Type)(BUTTON_IRQn[Button]));
  }
}

/**
  * @brief  Return the selected button state.
  * @param  Button: Button to be checked.
  *   This parameter can be one of the following values:
  *     @arg BUTTON_WAKEUP: Wakeup/Tamper Push Button
  * @retval Button state
  */
uint32_t BSP_PB_GetState(Button_TypeDef Button)
{
  return HAL_GPIO_ReadPin(BUTTON_PORT[Button], BUTTON_PIN[Button]);
}

/**
  * @brief  Initialize all buttons of the joystick in GPIO or EXTI modes.
  * @param  JoyMode: Joystick mode.
  *    This parameter can be one of the following values:
  *     @arg  JOY_MODE_GPIO: Joystick pins will be used as simple IOs
  *     @arg  JOY_MODE_EXTI: Joystick pins will be connected to EXTI line
  *                                 with interrupt generation capability
  * @retval HAL_OK: if all initializations are OK. Other value if error.
  */
uint8_t BSP_JOY_Init(JOYMode_TypeDef JoyMode)
{
  if (BSP_IO_Init() == IO_ERROR)
  {
    BSP_ErrorHandler();
  }

  /* Configure joystick pins in IT mode */
  if(JoyMode == JOY_MODE_EXTI)
  {
    BSP_IO_ConfigPin(JOY_ALL_PINS, IO_MODE_IT_LOW_LEVEL_PU);

    /* Configure IO Expander interrupt */
    MFX_IO_ITConfig();
  }
  else
  {
    /* GPIO mode configuration */
    BSP_IO_ConfigPin(JOY_ALL_PINS,IO_MODE_INPUT_PU);
  }

  return HAL_OK;
}

/**
  * @brief  DeInitialize all buttons of the joystick.
  * @retval None
  */
void BSP_JOY_DeInit(void)
{
  BSP_IO_ConfigPin(JOY_ALL_PINS, IO_MODE_OFF);
}

/**
  * @brief  Return the current joystick status.
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
  tmp = BSP_IO_ReadPin(JOY_ALL_PINS);

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

#ifdef HAL_UART_MODULE_ENABLED
/**
  * @brief  Initialize COM port.
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

  /* IOSV bit MUST be set to access GPIO port G[2:15] */
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();

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

/**
  * @brief  DeInitialize COM port.
  * @param  COM: Specifies the COM port to be deinitialized.
  *   This parameter can be one of following parameters:
  *     @arg COM1
  * @param  huart: pointer to a UART_HandleTypeDef structure that
  *   contains the configuration information for the specified UART peripheral.
  * @retval None
  */
void BSP_COM_DeInit(COM_TypeDef COM, UART_HandleTypeDef* huart)
{
  /* USART configuration */
  huart->Instance = COM_USART[COM];
  HAL_UART_DeInit(huart);
}
#endif /* HAL_UART_MODULE_ENABLED */


#ifdef HAL_ADC_MODULE_ENABLED
/**
  * @brief  Init the potentioneter RV2 measurement thanks to ADC.
  * Limitations : the potentiometer are exclusive with MFX, Audio codex,
  * OctoSPIP1, Debugging connector and MC operation. And they are mutually exclusive.
  * Please deinit all these components before usage.
  * @retval BSP_POTENTIOMETER_OK if no error
  */
uint32_t BSP_POTENTIOMETER_Init(void)
{
  GPIO_InitTypeDef          GPIO_InitStruct;
  ADC_ChannelConfTypeDef    sConfig;

  /* ADC clock enable */
  ADCx_CLK_ENABLE();
  /* ADC an GPIO Periph clock enable */
  ADCx_CHANNEL_GPIO_CLK_ENABLE();
  /* ADC Periph interface clock configuration */
  __HAL_RCC_ADC_CONFIG(RCC_ADCCLKSOURCE_SYSCLK);

  /* ADC Channel GPIO pin configuration */
  GPIO_InitStruct.Pin = ADCx_CHANNEL_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;

  HAL_GPIO_Init(ADCx_CHANNEL_GPIO_PORT, &GPIO_InitStruct);

  /*##-1- Configure the ADC peripheral #######################################*/
  hEvalADC.Instance          = ADCx;

  if (HAL_ADC_DeInit(&hEvalADC)) return BSP_POTENTIOMETER_ERROR;

  hEvalADC.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV2;      /* Synchronous clock mode, input ADC clock not divided */
  hEvalADC.Init.Resolution            = ADC_RESOLUTION_12B;            /* 12-bit resolution for converted data */
  hEvalADC.Init.DataAlign             = ADC_DATAALIGN_RIGHT;           /* Right-alignment for converted data */
  hEvalADC.Init.ScanConvMode          = DISABLE;                       /* Sequencer disabled (ADC conversion on only 1 channel: channel set on rank 1) */
  hEvalADC.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;           /* EOC flag picked-up to indicate conversion end */
  hEvalADC.Init.LowPowerAutoWait      = DISABLE;                       /* Auto-delayed conversion feature disabled */
  hEvalADC.Init.ContinuousConvMode    = DISABLE;                       /* Continuous mode disabled to have only 1 conversion at each conversion trig */
  hEvalADC.Init.NbrOfConversion       = 1;                             /* Parameter discarded because sequencer is disabled */
  hEvalADC.Init.DiscontinuousConvMode = DISABLE;                       /* Parameter discarded because sequencer is disabled */
  hEvalADC.Init.NbrOfDiscConversion   = 1;                             /* Parameter discarded because sequencer is disabled */
  hEvalADC.Init.ExternalTrigConv      = ADC_SOFTWARE_START;            /* Software start to trig the 1st conversion manually, without external event */
  hEvalADC.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE; /* Parameter discarded because software trigger chosen */
  hEvalADC.Init.DMAContinuousRequests = DISABLE;                       /* DMA one-shot mode selected (not applied to this example) */
  hEvalADC.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;      /* DR register is overwritten with the last conversion result in case of overrun */
  hEvalADC.Init.OversamplingMode      = DISABLE;                       /* No oversampling */

  if (HAL_ADC_Init(&hEvalADC) != HAL_OK) return BSP_POTENTIOMETER_ERROR;

  /*##-2- Configure ADC regular channel ######################################*/
  sConfig.Channel      = ADCx_CHANNEL;                /* Sampled channel number */
  sConfig.Rank         = ADC_REGULAR_RANK_1;          /* Rank of sampled channel number ADCx_CHANNEL */
  sConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;    /* Sampling time (number of clock cycles unit) */
  sConfig.SingleDiff   = ADC_SINGLE_ENDED;            /* Single-ended input channel */
  sConfig.OffsetNumber = ADC_OFFSET_NONE;             /* No offset subtraction */
  sConfig.Offset = 0;                                 /* Parameter discarded because offset correction is disabled */


  if (HAL_ADC_ConfigChannel(&hEvalADC, &sConfig) != HAL_OK) return BSP_POTENTIOMETER_ERROR;

  /* Run the ADC calibration in single-ended mode */
  if (HAL_ADCEx_Calibration_Start(&hEvalADC, ADC_SINGLE_ENDED)) return BSP_POTENTIOMETER_ERROR;


  return BSP_POTENTIOMETER_OK;
}

/**
  * @brief  DeInitialize the potentioneter RV2 measurement thanks to ADC.
  * Limitations : the potentiometer are exclusive with MFX, Audio codex,
  * OctoSPIP1, Debugging connector and MC operation. And they are mutually exclusive.
  * Please deinit all these components before usage.
  * @retval BSP_POTENTIOMETER_OK if no error
  */
uint32_t BSP_POTENTIOMETER_DeInit(void)
{
  if (HAL_ADC_DeInit(&hEvalADC) != HAL_OK) return BSP_POTENTIOMETER_ERROR;

  return BSP_POTENTIOMETER_OK;
}

/**
  * @brief  Get the potentioneter RV2 measurement thanks to ADC.
  * Limitations : the potentiometer are exclusive with MFX, Audio codex,
  * OctoSPIP1, Debugging connector and MC operation. And they are mutually exclusive.
  * Please deinit all these components before usage.
  * @retval value if no error else 0xFFFFFFFF
  */
uint32_t BSP_POTENTIOMETER_GetLevel(void)
{
  /*##-3- Start the conversion process #######################################*/
  if (HAL_ADC_Start(&hEvalADC) != HAL_OK) return BSP_POTENTIOMETER_ERROR;

  if(HAL_ADC_PollForConversion(&hEvalADC, ADCx_POLL_TIMEOUT)== HAL_OK)
  {
    return (HAL_ADC_GetValue(&hEvalADC));
  }
  else
  {
    return  0xFFFFFFFF;
  }
}
#endif /* HAL_ADC_MODULE_ENABLED */

/**
  * @}
  */

/** @defgroup STM32L4R9I_EVAL_BusOperations_Functions Bus Operations Functions
  * @{
  */

/*******************************************************************************
                            BUS OPERATIONS
*******************************************************************************/
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
  HAL_PWREx_EnableVddIO2();

  if (hi2c->Instance == EVAL_I2Cx)
  {
    /*##-1- Configure the Eval I2C clock source. The clock is derived from the SYSCLK #*/
    if(EVAL_RCC_PERIPHCLK_I2C == RCC_PERIPHCLK_I2C2)
    {
      RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C2;
      RCC_PeriphCLKInitStruct.I2c2ClockSelection   = EVAL_RCC_CLKSOURCE_I2C;
    }
    else
    {
      RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
      RCC_PeriphCLKInitStruct.I2c1ClockSelection   = EVAL_RCC_CLKSOURCE_I2C;
    }
    HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);

    /*##-2- Configure the GPIOs ################################################*/

    /* Enable GPIO clock */
    EVAL_I2Cx_SDA_GPIO_CLK_ENABLE();
    EVAL_I2Cx_SCL_GPIO_CLK_ENABLE();

    /* Configure I2C Rx/Tx as alternate function  */
    gpioinitstruct.Pin       = EVAL_I2Cx_SCL_PIN | EVAL_I2Cx_SDA_PIN;
    gpioinitstruct.Mode      = GPIO_MODE_AF_OD;
    gpioinitstruct.Pull      = GPIO_PULLUP;
    gpioinitstruct.Speed     = GPIO_SPEED_FREQ_MEDIUM; // GPIO_SPEED_FREQ_VERY_HIGH;
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
static void I2Cx_Init(uint32_t user)
{
  if(HAL_I2C_GetState(&hi2c_eval) == HAL_I2C_STATE_RESET)
  {
#if defined(BSP_USE_CMSIS_OS)
    /* Create semaphore to prevent multiple I2C access */
    osSemaphoreDef(BSP_I2C_SEM);
    BspI2cSemaphore = osSemaphoreCreate(osSemaphore(BSP_I2C_SEM), 1);
#endif

    hi2c_eval.Instance              = EVAL_I2Cx;
    hi2c_eval.Init.Timing           = EVAL_I2Cx_TIMING;
    hi2c_eval.Init.OwnAddress1      = 0;
    hi2c_eval.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    hi2c_eval.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    hi2c_eval.Init.OwnAddress2      = 0;
    hi2c_eval.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    hi2c_eval.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;

    HAL_I2C_DeInit(&hi2c_eval);
    /* Init the I2C */
    I2Cx_MspInit(&hi2c_eval);
    HAL_I2C_Init(&hi2c_eval);
  }
  /* Update BSP I2C users list */
  v_bspI2cUsers |= user;
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
static void I2Cx_DeInit(uint32_t user)
{
  /* Update BSP I2C users list */
  v_bspI2cUsers &= ~(user);

  if((HAL_I2C_GetState(&hi2c_eval) != HAL_I2C_STATE_RESET) && (v_bspI2cUsers == BSP_I2C_NO_USER))
  {
    /* Deinit the I2C */
    HAL_I2C_DeInit(&hi2c_eval);
    I2Cx_MspDeInit(&hi2c_eval);

#if defined(BSP_USE_CMSIS_OS)
    /* Delete semaphore to prevent multiple I2C access */
    osSemaphoreDelete(BspI2cSemaphore);
#endif
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

#if defined(BSP_USE_CMSIS_OS)
  /* Get semaphore to prevent multiple I2C access */
  osSemaphoreWait(BspI2cSemaphore, osWaitForever);
#endif

  status = HAL_I2C_Mem_Write(&hi2c_eval, Addr, (uint16_t)Reg, RegSize, pBuffer, Length, hi2c_evalTimeout);

#if defined(BSP_USE_CMSIS_OS)
  /* Release semaphore to prevent multiple I2C access */
  osSemaphoreRelease(BspI2cSemaphore);
#endif

/* Check the communication status */
  if(status != HAL_OK)
  {
    /* Re-Initiaize the BUS */
    I2Cx_Error();
  }
  return status;
}

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

#if defined(BSP_USE_CMSIS_OS)
  /* Get semaphore to prevent multiple I2C access */
  osSemaphoreWait(BspI2cSemaphore, osWaitForever);
#endif

  status = HAL_I2C_Mem_Write(&hi2c_eval, Addr, (uint16_t)Reg, RegSize, &Value, 1, hi2c_evalTimeout);

#if defined(BSP_USE_CMSIS_OS)
  /* Release semaphore to prevent multiple I2C access */
  osSemaphoreRelease(BspI2cSemaphore);
#endif

  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Re-Initiaize the BUS */
    I2Cx_Error();
  }
}

/**
  * @brief  Write multiple data value in a register of the device through BUS.
  * @param  Addr: Device address on BUS Bus.
  * @param  Reg: The target register address to write
  * @param  MemAddress: Memory address
  * @param  Buffer: The target register value to be written
  * @param  Length: buffer size to be written
  * @retval None
  */
static void I2Cx_WriteMultiple(uint16_t Addr, uint16_t Reg, uint16_t MemAddress, uint8_t *Buffer, uint16_t Length)
  {
  HAL_StatusTypeDef status = HAL_OK;

#if defined(BSP_USE_CMSIS_OS)
  /* Get semaphore to prevent multiple I2C access */
  osSemaphoreWait(BspI2cSemaphore, osWaitForever);
#endif

  status = HAL_I2C_Mem_Write(&hi2c_eval, Addr, (uint16_t)Reg, MemAddress, Buffer, Length, hi2c_evalTimeout);

#if defined(BSP_USE_CMSIS_OS)
  /* Release semaphore to prevent multiple I2C access */
  osSemaphoreRelease(BspI2cSemaphore);
#endif

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

#if defined(BSP_USE_CMSIS_OS)
  /* Get semaphore to prevent multiple I2C access */
  osSemaphoreWait(BspI2cSemaphore, osWaitForever);
#endif

  status = HAL_I2C_Mem_Read(&hi2c_eval, Addr, Reg, RegSize, &value, 1, hi2c_evalTimeout);

#if defined(BSP_USE_CMSIS_OS)
  /* Release semaphore to prevent multiple I2C access */
  osSemaphoreRelease(BspI2cSemaphore);
#endif

  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    I2Cx_Error();

  }
  return value;
}

/**
  * @brief  Read multiple data from a register of the device through BUS
  * @param  Addr: Device address on BUS
  * @param  Reg: The target register address to read
  * @param  MemAddress: Memory address
  * @param  Buffer: Pointer to data buffer
  * @param  Length: Length of the data
  * @retval Number of read data
  */
static uint8_t I2Cx_ReadMultiple(uint16_t Addr, uint16_t Reg, uint16_t MemAddress, uint8_t *Buffer, uint16_t Length)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint8_t value = 0x0;

#if defined(BSP_USE_CMSIS_OS)
  /* Get semaphore to prevent multiple I2C access */
  osSemaphoreWait(BspI2cSemaphore, osWaitForever);
#endif

  status = HAL_I2C_Mem_Read(&hi2c_eval, Addr, Reg, MemAddress, Buffer, Length, hi2c_evalTimeout);

#if defined(BSP_USE_CMSIS_OS)
  /* Release semaphore to prevent multiple I2C access */
  osSemaphoreRelease(BspI2cSemaphore);
#endif

  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    I2Cx_Error();

  }
  return value;
}

/**
  * @brief  Read multiple data on the BUS.
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

#if defined(BSP_USE_CMSIS_OS)
  /* Get semaphore to prevent multiple I2C access */
  osSemaphoreWait(BspI2cSemaphore, osWaitForever);
#endif

  status = HAL_I2C_Mem_Read(&hi2c_eval, Addr, (uint16_t)Reg, RegSize, pBuffer, Length, hi2c_evalTimeout);

#if defined(BSP_USE_CMSIS_OS)
  /* Release semaphore to prevent multiple I2C access */
  osSemaphoreRelease(BspI2cSemaphore);
#endif

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
  HAL_StatusTypeDef status = HAL_OK;

#if defined(BSP_USE_CMSIS_OS)
  /* Get semaphore to prevent multiple I2C access */
  osSemaphoreWait(BspI2cSemaphore, osWaitForever);
#endif

  status = HAL_I2C_IsDeviceReady(&hi2c_eval, DevAddress, Trials, hi2c_evalTimeout);

#if defined(BSP_USE_CMSIS_OS)
  /* Release semaphore to prevent multiple I2C access */
  osSemaphoreRelease(BspI2cSemaphore);
#endif

  return status;
}


/**
  * @brief Eval I2Cx error treatment function
  * @retval None
  */
static void I2Cx_Error (void)
{
  uint32_t tmpI2cUsers;

  BSP_ErrorHandler();

  /* De-initialize the I2C communication BUS */
  tmpI2cUsers = v_bspI2cUsers;
  I2Cx_DeInit(BSP_I2C_ALL_USERS);

  /* Re- Initiaize the I2C communication BUS */
  I2Cx_Init(tmpI2cUsers);
}

/*******************************************************************************
                            LINK OPERATIONS
*******************************************************************************/

/***************************** LINK MFX_IO ************************************/

/**
  * @brief  Initialize MFX_IO low level.
  * @retval None
  */
void MFX_IO_Init(void)
{
  I2Cx_Init(BSP_I2C_MFX_USER);
}

/**
  * @brief  Configure MFX_IO low level Interrupt.
  * @retval None
  */
void MFX_IO_ITConfig(void)
{
  GPIO_InitTypeDef gpioinitstruct = {0};

  /* Enable the GPIO EXTI clock */
  MFX_INT_GPIO_CLK_ENABLE();

  /* Configure MFX Interrupt GPIO */
  gpioinitstruct.Pin   = MFX_INT_PIN;
  gpioinitstruct.Pull  = GPIO_NOPULL;
  gpioinitstruct.Speed = GPIO_SPEED_FREQ_LOW;
  gpioinitstruct.Mode  = GPIO_MODE_IT_RISING;
  HAL_GPIO_Init(MFX_INT_GPIO_PORT, &gpioinitstruct);

  /* Enable and set GPIO EXTI Interrupt to the lowest priority */
  HAL_NVIC_SetPriority((IRQn_Type)(MFX_INT_EXTI_IRQn), 0x0F, 0x0F);
  HAL_NVIC_EnableIRQ(MFX_INT_EXTI_IRQn);
}

/**
  * @brief  MFX_IO writes single data.
  * @param  Addr: I2C address
  * @param  Reg: Reg address
  * @param  Value: Data to be written
  * @retval None
  */
void MFX_IO_Write(uint16_t Addr, uint8_t Reg, uint8_t Value)
{
  I2Cx_WriteData(Addr, Reg, I2C_MEMADD_SIZE_8BIT, Value);
}

/**
  * @brief  MFX_IO reads single data.
  * @param  Addr: I2C address
  * @param  Reg: Reg address
  * @retval Read data
  */
uint8_t MFX_IO_Read(uint16_t Addr, uint8_t Reg)
{
  return I2Cx_ReadData(Addr, Reg, I2C_MEMADD_SIZE_8BIT);
}

/**
  * @brief  MFX_IO reads multiple data.
  * @param  Addr: I2C address
  * @param  Reg: Reg address
  * @param  Buffer: Pointer to data buffer
  * @param  Length: Length of the data
  */
uint16_t MFX_IO_ReadMultiple(uint16_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length)
{
  return I2Cx_ReadBuffer(Addr, Reg, I2C_MEMADD_SIZE_8BIT, Buffer, Length);
}

/**
  * @brief  MFX_IO delay.
  * @param  Delay: Delay in ms
  * @retval None
  */
void MFX_IO_Delay(uint32_t Delay)
{
  HAL_Delay(Delay);
}

/**
  * @brief  MFX_IO wakeup.
  * @retval None
  */
void MFX_IO_Wakeup(void)
{
  /* Set Wakeup pin to high to wakeup MFX component from standby mode */
  HAL_GPIO_WritePin(MFX_WAKEUP_GPIO_PORT, MFX_WAKEUP_PIN, GPIO_PIN_SET);

  /* Wait */
  HAL_Delay(1);

  /* Set gpio pin basck to low */
  HAL_GPIO_WritePin(MFX_WAKEUP_GPIO_PORT, MFX_WAKEUP_PIN, GPIO_PIN_RESET);
}

/**
  * @brief  MFX_IO enable wakeup pin.
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
  * @brief  DeInitializes MFX_IO low level.
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

  I2Cx_DeInit(BSP_I2C_MFX_USER);
}

/********************************* LINK TOUCHSCREEN *********************************/

/**
  * @brief  Initializes Touchscreen low level.
  * @retval None
  */
void TS_IO_Init(void)
{
  I2Cx_Init(BSP_I2C_TS_USER);
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
  I2Cx_WriteMultiple(Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT,(uint8_t*)&Value, 1);
}

/**
  * @brief  Reads a single data.
  * @param  Addr: I2C address
  * @param  Reg: Reg address
  * @retval Data to be read
  */
uint8_t TS_IO_Read(uint8_t Addr, uint8_t Reg)
{
  uint8_t read_value = 0;

  I2Cx_ReadMultiple(Addr, Reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&read_value, 1);

  return read_value;
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
 return I2Cx_ReadMultiple(Addr, Reg, I2C_MEMADD_SIZE_8BIT, Buffer, Length);
}

/**
  * @brief  TS delay
  * @param  Delay: Delay in ms
  * @retval None
  */
void TS_IO_Delay(uint32_t Delay)
{
  HAL_Delay(Delay);
}


/********************************* LINK AUDIO *********************************/

/**
  * @brief  Initialize Audio low level.
  * @retval None
  */
void AUDIO_IO_Init(void)
{
  I2Cx_Init(BSP_I2C_AUDIO_USER);
}

/**
  * @brief  Deinitialize Audio low level.
  * @retval None
  */
void AUDIO_IO_DeInit(void)
{
  I2Cx_DeInit(BSP_I2C_AUDIO_USER);
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
  * @brief  Initialize peripherals used by the I2C EEPROM driver.
  * @retval None
  */
void EEPROM_IO_Init(void)
{
  I2Cx_Init(BSP_I2C_EEPROM_USER);
}

/**
  * @brief  Write data to I2C EEPROM driver
  * @param  DevAddress: Target device address
  * @param  MemAddress: Internal memory address
  * @param  pBuffer: Pointer to data buffer
  * @param  BufferSize: Amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef EEPROM_IO_WriteData(uint16_t DevAddress, uint16_t MemAddress, uint8_t* pBuffer, uint32_t BufferSize)
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
HAL_StatusTypeDef EEPROM_IO_ReadData(uint16_t DevAddress, uint16_t MemAddress, uint8_t* pBuffer, uint32_t BufferSize)
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
HAL_StatusTypeDef EEPROM_IO_IsDeviceReady(uint16_t DevAddress, uint32_t Trials)
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



