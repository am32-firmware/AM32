
/**
  ******************************************************************************
  * @file    stm32l4xx_nucleo_144.c
  * @author  MCD Application Team
  * @brief   This file provides set of firmware functions to manage:
  *          - LEDs and push-button available on STM32L4XX-Nucleo-144 Kit
  *            from STMicroelectronics
  *          - LCD, joystick and microSD available on Adafruit 1.8" TFT LCD
  *            shield (reference ID 802)
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
#include "stm32l4xx_nucleo_144.h"


/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L4XX_NUCLEO_144
  * @brief This file provides set of firmware functions to manage Leds and push-button
  *        available on STM32L4xx-Nucleo Kit from STMicroelectronics.
  * @{
  */

/** @defgroup STM32L4XX_NUCLEO_144_Private_Constants Private Constants
  * @{
  */

/**
  * @brief STM32L4xx NUCLEO BSP Driver version number
  */
#define __STM32L4xx_NUCLEO_144_BSP_VERSION_MAIN   (0x01) /*!< [31:24] main version */
#define __STM32L4xx_NUCLEO_144_BSP_VERSION_SUB1   (0x00) /*!< [23:16] sub1 version */
#define __STM32L4xx_NUCLEO_144_BSP_VERSION_SUB2   (0x09) /*!< [15:8]  sub2 version */
#define __STM32L4xx_NUCLEO_144_BSP_VERSION_RC     (0x00) /*!< [7:0]  release candidate */
#define __STM32L4xx_NUCLEO_144_BSP_VERSION       ((__STM32L4xx_NUCLEO_144_BSP_VERSION_MAIN << 24)\
                                                 |(__STM32L4xx_NUCLEO_144_BSP_VERSION_SUB1 << 16)\
                                                 |(__STM32L4xx_NUCLEO_144_BSP_VERSION_SUB2 << 8 )\
                                                 |(__STM32L4xx_NUCLEO_144_BSP_VERSION_RC))

/**
  * @brief LINK SD Card
  */
#define SD_DUMMY_BYTE            0xFF
#define SD_NO_RESPONSE_EXPECTED  0x80


#ifdef USE_STM32L4XX_NUCLEO_144_SMPS

/**
  * @brief SMPS
  */


#ifdef USE_ADP5301ACBZ          /* ADP5301ACBZ */

/* ######################################################################## */
/* #define PORT_SMPS               GPIOG              */
/* #define PIN_SMPS_ENABLE         GPIO_PIN_11        */
/* #define PIN_SMPS_POWERGOOD      GPIO_PIN_12        */
/* #define PIN_SMPS_SWITCH_ENABLE  GPIO_PIN_13        */

/* IN CASE OF SMPS VOLTAGE RANGE SELECTION            */
/* #define PIN_SMPS_V1             GPIO_PIN_10        */
/* ######################################################################## */

#endif                          /* ADP5301ACBZ */

/**
  * @}
  */

#endif /* USE_STM32L4XX_NUCLEO_144_SMPS */


/** @defgroup STM32L4XX_NUCLEO_144_Private_Variables Private Variables
  * @{
  */
GPIO_TypeDef *GPIO_PORT[LEDn] = {LED1_GPIO_PORT, LED2_GPIO_PORT, LED3_GPIO_PORT};

const uint16_t GPIO_PIN[LEDn] = {LED1_PIN, LED2_PIN, LED3_PIN};

GPIO_TypeDef *BUTTON_PORT[BUTTONn] = {USER_BUTTON_GPIO_PORT};
const uint16_t BUTTON_PIN[BUTTONn] = {USER_BUTTON_PIN};
const uint8_t BUTTON_IRQn[BUTTONn] = {USER_BUTTON_EXTI_IRQn};

/**
 * @brief BUS variables
 */

#ifdef HAL_SPI_MODULE_ENABLED
uint32_t SpixTimeout = NUCLEO_SPIx_TIMEOUT_MAX; /*<! Value of Timeout when SPI communication fails */
static SPI_HandleTypeDef hnucleo_Spi;
#endif /* HAL_SPI_MODULE_ENABLED */

#ifdef HAL_ADC_MODULE_ENABLED
static ADC_HandleTypeDef hnucleo_Adc;
/* ADC channel configuration structure declaration */
static ADC_ChannelConfTypeDef sConfig;
#endif /* HAL_ADC_MODULE_ENABLED */
/**
  * @}
  */

/* Private Function Prototypes */
#ifdef HAL_SPI_MODULE_ENABLED
static void       SPIx_Init(void);
static void       SPIx_MspInit(SPI_HandleTypeDef *hspi);
static void       SPIx_Write(uint8_t Value);
static void       SPIx_WriteReadData(const uint8_t *DataIn, uint8_t *DataOut, uint16_t DataLength);
static void       SPIx_Error(void);

/* SD IO functions */
void              SD_IO_Init(void);
void              SD_IO_CSState(uint8_t state);
void              SD_IO_WriteReadData(const uint8_t *DataIn, uint8_t *DataOut, uint16_t DataLength);
uint8_t           SD_IO_WriteByte(uint8_t Data);

/* LCD IO functions */
void              LCD_IO_Init(void);
void              LCD_IO_WriteData(uint8_t Data);
void              LCD_IO_WriteMultipleData(uint8_t *pData, uint32_t Size);
void              LCD_IO_WriteReg(uint8_t LCDReg);
void              LCD_Delay(uint32_t delay);
#endif /* HAL_SPI_MODULE_ENABLED */

#ifdef HAL_ADC_MODULE_ENABLED
static void       ADCx_Init(void);
static void       ADCx_DeInit(void);
static void       ADCx_MspInit(ADC_HandleTypeDef *hadc);
static void       ADCx_MspDeInit(ADC_HandleTypeDef *hadc);
#endif /* HAL_ADC_MODULE_ENABLED */

/** @defgroup STM32L4XX_NUCLEO_144_Exported_Functions Exported Functions
  * @{
  */

/**
  * @brief  This method returns the STM32L4xx NUCLEO BSP Driver revision
  * @retval version : 0xXYZR (8bits for each decimal, R for RC)
  */
uint32_t BSP_GetVersion(void)
{
  return __STM32L4xx_NUCLEO_144_BSP_VERSION;
}

/** @defgroup STM32L4XX_NUCLEO_144_LED_Functions LED Functions
  * @{
  */

/**
  * @brief  Configure LED GPIO.
  * @param  Led: Specifies the Led to be configured.
  *   This parameter can be one of following parameters:
  *     @arg  LED1
  *     @arg  LED2
  *     @arg  LED3
  * @retval None
  */
void BSP_LED_Init(Led_TypeDef Led)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  /* Enable the GPIO_LED Clock */
  LEDx_GPIO_CLK_ENABLE(Led);

  /* Configure the GPIO_LED pin */
  GPIO_InitStruct.Pin = GPIO_PIN[Led];
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

  HAL_GPIO_Init(GPIO_PORT[Led], &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_RESET);
}

/**
  * @brief  DeInitialize LEDs.
  * @param  Led: LED to be de-init.
  *   This parameter can be one of the following values:
  *     @arg  LED1
  *     @arg  LED2
  *     @arg  LED3
  * @note BSP_LED_DeInit() does not disable the GPIO clock
  * @retval None
  */
void BSP_LED_DeInit(Led_TypeDef Led)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  /* Turn off LED */
  HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_RESET);
  /* DeInit the GPIO_LED pin */
  GPIO_InitStruct.Pin = GPIO_PIN[Led];
  HAL_GPIO_DeInit(GPIO_PORT[Led], GPIO_InitStruct.Pin);
}

/**
  * @brief  Turn selected LED On.
  * @param  Led: Specifies the Led to be set on.
  *   This parameter can be one of following parameters:
  *     @arg  LED1
  *     @arg  LED2
  *     @arg  LED3
  * @retval None
  */
void BSP_LED_On(Led_TypeDef Led)
{
  HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_SET);
}

/**
  * @brief  Turn selected LED Off.
  * @param  Led: Specifies the Led to be set off.
  *   This parameter can be one of following parameters:
  *     @arg  LED1
  *     @arg  LED2
  *     @arg  LED3
  * @retval None
  */
void BSP_LED_Off(Led_TypeDef Led)
{
  HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_RESET);
}

/**
  * @brief  Toggle the selected LED.
  * @param  Led: Specifies the Led to be toggled.
  *   This parameter can be one of following parameters:
  *     @arg  LED1
  *     @arg  LED2
  *     @arg  LED3
  * @retval None
  */
void BSP_LED_Toggle(Led_TypeDef Led)
{
  HAL_GPIO_TogglePin(GPIO_PORT[Led], GPIO_PIN[Led]);
}

/**
  * @}
  */

/** @defgroup STM32L4XX_NUCLEO_144_BUTTON_Functions BUTTON Functions
  * @{
  */

/**
  * @brief  Configure Button GPIO and EXTI Line.
  * @param  Button: Specifies the Button to be configured.
  *   This parameter should be: BUTTON_USER
  * @param  ButtonMode: Specifies Button mode.
  *   This parameter can be one of following parameters:
  *     @arg BUTTON_MODE_GPIO: Button will be used as simple IO
  *     @arg BUTTON_MODE_EXTI: Button will be connected to EXTI line with interrupt
  *                            generation capability
  * @retval None
  */
void BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef ButtonMode)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* Enable the BUTTON Clock */
  BUTTONx_GPIO_CLK_ENABLE(Button);

  if (ButtonMode == BUTTON_MODE_GPIO)
  {
    /* Configure Button pin as input */
    GPIO_InitStruct.Pin = BUTTON_PIN[Button];
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(BUTTON_PORT[Button], &GPIO_InitStruct);
  }
  else if (ButtonMode == BUTTON_MODE_EXTI)
  {
    /* Configure Button pin as input with External interrupt */
    GPIO_InitStruct.Pin = BUTTON_PIN[Button];
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    HAL_GPIO_Init(BUTTON_PORT[Button], &GPIO_InitStruct);

    /* Enable and set Button EXTI Interrupt to the lowest priority */
    HAL_NVIC_SetPriority((IRQn_Type)(BUTTON_IRQn[Button]), 0x0F, 0x00);
    HAL_NVIC_EnableIRQ((IRQn_Type)(BUTTON_IRQn[Button]));
  }
}

/**
  * @brief  DeInitialize Push Button.
  * @param  Button: Button to be configured
  *   This parameter should be: BUTTON_USER
  * @note BSP_PB_DeInit() does not disable the GPIO clock
  * @retval None
  */
void BSP_PB_DeInit(Button_TypeDef Button)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  GPIO_InitStruct.Pin = BUTTON_PIN[Button];
  HAL_NVIC_DisableIRQ((IRQn_Type)(BUTTON_IRQn[Button]));
  HAL_GPIO_DeInit(BUTTON_PORT[Button], GPIO_InitStruct.Pin);
}

/**
  * @brief  Return the selected Button state.
  * @param  Button: Specifies the Button to be checked.
  *   This parameter should be: BUTTON_USER
  * @retval The Button GPIO pin value.
  */
uint32_t BSP_PB_GetState(Button_TypeDef Button)
{
  return HAL_GPIO_ReadPin(BUTTON_PORT[Button], BUTTON_PIN[Button]);
}

#ifdef HAL_ADC_MODULE_ENABLED
/**
  * @brief  Configure joystick available on adafruit 1.8" TFT shield
  *         managed through ADC to detect motion.
  * @retval Joystickstatus (0=> success, 1=> fail)
  */
uint8_t BSP_JOY_Init(void)
{
  ADCx_Init();

  /* Select the ADC Channel to be converted */
  sConfig.Channel      = NUCLEO_ADCx_CHANNEL;
  sConfig.Rank         = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = NUCLEO_ADCx_SAMPLETIME;
  sConfig.SingleDiff   = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset       = 0;

  /* Return Joystick initialization status */
  return (uint8_t)HAL_ADC_ConfigChannel(&hnucleo_Adc, &sConfig);
}

/**
  * @brief  DeInitialize joystick GPIOs.
  * @retval None.
  */
void BSP_JOY_DeInit(void)
{
  ADCx_DeInit();
}

/**
  * @brief  Return the Joystick key pressed.
  * @note   To know which Joystick key is pressed we need to detect the voltage
  *         level on each key output
  *           - None  : 3.3 V / 4095
  *           - SEL   : 1.055 V / 1308
  *           - DOWN  : 0.71 V / 88
  *           - LEFT  : 3.0 V / 3720
  *           - RIGHT : 0.595 V / 737
  *           - UP    : 1.65 V / 2046
  * @retval JOYState_TypeDef: Code of the Joystick key pressed.
  */
JOYState_TypeDef BSP_JOY_GetState(void)
{
  JOYState_TypeDef state;
  uint16_t  keyconvertedvalue = 0;

  /* Start the conversion process */
  HAL_ADC_Start(&hnucleo_Adc);

  /* Wait for the end of conversion */
  if (HAL_ADC_PollForConversion(&hnucleo_Adc, 10) != HAL_TIMEOUT)
  {
    /* Get the converted value of regular channel */
    keyconvertedvalue = HAL_ADC_GetValue(&hnucleo_Adc);
  }

  if ((keyconvertedvalue > 2010) && (keyconvertedvalue < 2090))
  {
    state = JOY_UP;
  }
  else if ((keyconvertedvalue > 680) && (keyconvertedvalue < 780))
  {
    state = JOY_RIGHT;
  }
  else if ((keyconvertedvalue > 1270) && (keyconvertedvalue < 1350))
  {
    state = JOY_SEL;
  }
  else if ((keyconvertedvalue > 50) && (keyconvertedvalue < 130))
  {
    state = JOY_DOWN;
  }
  else if ((keyconvertedvalue > 3680) && (keyconvertedvalue < 3760))
  {
    state = JOY_LEFT;
  }
  else
  {
    state = JOY_NONE;
  }

  /* Loop while a key is pressed */
  if (state != JOY_NONE)
  {
    keyconvertedvalue = HAL_ADC_GetValue(&hnucleo_Adc);
  }
  /* Return the code of the Joystick key pressed */
  return state;
}
#endif /* HAL_ADC_MODULE_ENABLED */

/**
  * @}
  */

#ifdef USE_STM32L4XX_NUCLEO_144_SMPS
#ifdef USE_ADP5301ACBZ          /* ADP5301ACBZ */

/** @defgroup STM32L4XX_NUCLEO_144_SMPS_Functions SMPS Functions
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

  /* To be updated */
  /* Set all GPIO in analog state to reduce power consumption, */
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;

  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Pull = GPIO_NOPULL;

  GPIO_InitStruct.Pin = PIN_SMPS_SWITCH_ENABLE;

  /* SWITCH */
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = PIN_SMPS_ENABLE | PIN_SMPS_SWITCH_ENABLE;
  /* --------- SMPS VOLTAGE RANGE SELECTION ----------------------------------*/
  /* ######################################################################## */
  /* GPIO_InitStruct.Pin = PIN_SMPS_ENABLE | PIN_SMPS_SWITCH_ENABLE | PIN_SMPS_V1; */

  /* ENABLE = OFF */
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  return SMPS_OK;
}

/**
  * @brief  Initialize the external SMPS component
  * @param  VoltageRange: Select operating SMPS supply
  *           @arg DCDC_AND_BOARD_DEPENDENT
  * @note   VoltageRange is not used with all boards
  *           VoltageRange is not used with MB11312A/S
  *           i.e. SMPS only PWR_REGULATOR_VOLTAGE_SCALE2.
  * @retval SMPS status
  */
uint32_t BSP_SMPS_Init(uint32_t VoltageRange)
{
  PWR_AND_CLK_SMPS();

  GPIO_InitTypeDef GPIO_InitStruct;

  /* Upon wake UP (standby case)                               */
  /* IF PIN_SMPS_ENABLE was pulled up                          */
  /* Then maintain PIN_SMPS_ENABLE = high                      */
  /* Needed to keep ENABLE HIGH                                */
  if (READ_BIT(PWR->PUCRG, PWR_GPIO_ENABLE))
  {
    HAL_GPIO_WritePin(PORT_SMPS, PIN_SMPS_ENABLE, GPIO_PIN_SET);
  }
  else
  {
    HAL_PWREx_EnableGPIOPullDown(PWR_GPIO_SMPS, PWR_GPIO_SWITCH_ENABLE);
    HAL_PWREx_EnablePullUpPullDownConfig();
    /* Level shifter consumes because of missing pull up/down, so pull it up (only one autorized PA13) */
    HAL_PWREx_EnableGPIOPullUp(PWR_GPIO_A, GPIO_PIN_13); /* SWD/TMS */
    HAL_PWREx_EnableGPIOPullUp(PWR_GPIO_B, GPIO_PIN_3); /* SWO */
  }
  /* ------------------------------------------------------------------------ */
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Pull = GPIO_PULLUP;

  GPIO_InitStruct.Pin = PIN_SMPS_POWERGOOD;

  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* ------------------------------------------------------------------------ */

  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Pull = GPIO_NOPULL;

  GPIO_InitStruct.Pin = PIN_SMPS_ENABLE | PIN_SMPS_SWITCH_ENABLE;
  /* --------- ADD SMPS VOLTAGE RANGE SELECTION -----------------------------*/
  /* - > Applicable to ST1PS02D1QTR on MB1312A/S */
  /* GPIO_InitStruct.Pin = PIN_SMPS_ENABLE | PIN_SMPS_SWITCH_ENABLE | PIN_SMPS_V1; */

  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* --------- SMPS VOLTAGE RANGE SELECTION ----------------------------------*/
  /* ######################################################################## */
  /* - > Not applicable to ADP5301A on MB1312A/S */
  /* ######################################################################## */
  /* - > Applicable to ST1PS02D1QTR on MB1312A/S */
  /* Control to be added */

  /* ST1PS02D1QTR on MB1312 */
  /* if (VoltageRange == ST1PS02D1QTR_VOUT_1_25) */
  /* HAL_GPIO_WritePin(PORT_SMPS, PIN_SMPS_V1, GPIO_PIN_SET); */
  /* 1.25V                  */
  /* D0/D1/D2 = H/L/L       */
  /* else */

  /* */
  /* ST1PS02D1QTR on MB1312 */
  /* ST1PS02D1QTR_VOUT_1_05 */
  /* 1.05V                  */
  /* D0/D1/D2 = L/L/L       */
  /* HAL_GPIO_WritePin(PORT_SMPS, PIN_SMPS_V1, GPIO_PIN_RESET); */
  /* ######################################################################## */
  return SMPS_OK;
}

/**
  * @brief  Enable the external SMPS component
  * @param  Delay: delay in ms after enable
  * @param  Power_Good_Check: Enable Power good check
  * @note   Power_Good_Check
  *  is not used with all boards
  *           VoltageRange is not used with MB11312A/S
  *           i.e. SMPS only PWR_REGULATOR_VOLTAGE_SCALE2 by board.
  * @retval SMPS status
  *           @arg SMPS_OK: SMPS ENABLE OK
  *           @arg SMPS_KO: POWER GOOD CHECK FAILS
  *
  */
uint32_t BSP_SMPS_Enable(uint32_t Delay, uint32_t Power_Good_Check)
{
  PWR_AND_CLK_SMPS();

  HAL_GPIO_WritePin(PORT_SMPS, PIN_SMPS_ENABLE, GPIO_PIN_SET);

  HAL_PWREx_EnableGPIOPullUp(PWR_GPIO_SMPS, PWR_GPIO_BIT_11);
  HAL_PWREx_EnablePullUpPullDownConfig();

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

  /* Check if SWITCH is DISABLE */
  if (HAL_GPIO_ReadPin(PORT_SMPS, PIN_SMPS_SWITCH_ENABLE) != GPIO_PIN_RESET)
  {
    /* ERROR AS SWITCH SHOULD BE DISABLED */
    return SMPS_KO;
  }

  /* Disable SMPS */
  HAL_GPIO_WritePin(PORT_SMPS, PIN_SMPS_ENABLE, GPIO_PIN_RESET);

  HAL_PWREx_EnableGPIOPullDown(PWR_GPIO_SMPS, PWR_GPIO_ENABLE);
  HAL_PWREx_EnablePullUpPullDownConfig();

  /* SMPS DISABLED */
  return SMPS_OK;
}

/**
  * @brief  Enable the external SMPS SWITCH component
  * @param  Delay: delay in ms before SMPS SWITCH ENABLE
  * @param  Power_Good_Check: Enable Power good check
  * @note   Power_Good_Check
  *  is not used with all boards
  *           VoltageRange is not used with MB11312A/S
  *           i.e. SMPS only PWR_REGULATOR_VOLTAGE_SCALE2 by board.
  * @retval SMPS status
  *           @arg SMPS_OK: SMPS ENABLE OK
  *           @arg SMPS_KO: POWER GOOD CHECK FAILS
  *
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

#if defined (FLASH_CFGR_LVEN)
  /* Configure Flash for lower voltage with SMPS                   */
  /* NUCLEO BOARD ALLOWS 1.2V supply voltage                       */
  /* shortest timing works in R2 when SMPS is used                 */
  HAL_FLASHEx_ConfigLVEPin(FLASH_LVE_PIN_FORCED);
#endif  
  return SMPS_OK;
}

/**
  * @brief  Disable the external SMPS component
  * @retval SMPS status
  *           @arg SMPS_OK: SMPS SWITCH DISABLE OK
  *
  */
uint32_t BSP_SMPS_Supply_Disable(void)
{
  PWR_AND_CLK_SMPS();
#if defined (FLASH_CFGR_LVEN)
  /* Configure Flash back to normal settings        */
  /* NUCLEO BOARD ALLOWS 1.2V supply voltage        */
  /* Now exiting from SMPS mode                     */
  /* hence longest timing shall be used             */

  HAL_FLASHEx_ConfigLVEPin(FLASH_LVE_PIN_CTRL);
#endif   
  /* SMPS SWITCH DISABLED */
  HAL_GPIO_WritePin(PORT_SMPS, PIN_SMPS_SWITCH_ENABLE, GPIO_PIN_RESET);

  return SMPS_OK;
}

#endif                          /* ADP5301ACBZ */

#endif /* USE_STM32L4XX_NUCLEO_144_SMPS */

/**
  * @}
  */

/**
  * @}
  */

/** @defgroup STM32L4XX_NUCLEO_144_Private_Functions Private Functions
  * @{
  */

/******************************************************************************
                            BUS OPERATIONS
*******************************************************************************/

/******************************* SPI ********************************/
#ifdef HAL_SPI_MODULE_ENABLED

/**
  * @brief  Initialize SPI MSP.
  * @retval None
  */
static void SPIx_MspInit(SPI_HandleTypeDef *hspi)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  /*** Configure the GPIOs ***/
  /* Enable GPIO clock */
  NUCLEO_SPIx_SCK_GPIO_CLK_ENABLE();
  NUCLEO_SPIx_MISO_MOSI_GPIO_CLK_ENABLE();

  /* Configure SPI SCK */
  GPIO_InitStruct.Pin = NUCLEO_SPIx_SCK_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = NUCLEO_SPIx_SCK_AF;
  HAL_GPIO_Init(NUCLEO_SPIx_SCK_GPIO_PORT, &GPIO_InitStruct);

  /* Configure SPI MISO and MOSI */
  GPIO_InitStruct.Pin = NUCLEO_SPIx_MOSI_PIN;
  GPIO_InitStruct.Alternate = NUCLEO_SPIx_MISO_MOSI_AF;
  GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
  HAL_GPIO_Init(NUCLEO_SPIx_MISO_MOSI_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = NUCLEO_SPIx_MISO_PIN;
  GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
  HAL_GPIO_Init(NUCLEO_SPIx_MISO_MOSI_GPIO_PORT, &GPIO_InitStruct);

  /*** Configure the SPI peripheral ***/
  /* Enable SPI clock */
  NUCLEO_SPIx_CLK_ENABLE();
}


/**
  * @brief  Initialize SPI HAL.
  * @retval None
  */
static void SPIx_Init(void)
{
  if (HAL_SPI_GetState(&hnucleo_Spi) == HAL_SPI_STATE_RESET)
  {
    /* SPI Config */
    hnucleo_Spi.Instance = NUCLEO_SPIx;
    /* SPI baudrate is set to 9 MHz maximum (APB1/SPI_BaudRatePrescaler = 72/8 = 9 MHz)
     to verify these constraints:
        - ST7735 LCD SPI interface max baudrate is 15MHz for write and 6.66MHz for read
          Since the provided driver doesn't use read capability from LCD, only constraint
          on write baudrate is considered.
        - SD card SPI interface max baudrate is 25MHz for write/read
        - PCLK2 max frequency is 72 MHz
     */
    hnucleo_Spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    hnucleo_Spi.Init.Direction = SPI_DIRECTION_2LINES;
    hnucleo_Spi.Init.CLKPhase = SPI_PHASE_2EDGE;
    hnucleo_Spi.Init.CLKPolarity = SPI_POLARITY_HIGH;
    hnucleo_Spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hnucleo_Spi.Init.CRCPolynomial = 7;
    hnucleo_Spi.Init.DataSize = SPI_DATASIZE_8BIT;
    hnucleo_Spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hnucleo_Spi.Init.NSS = SPI_NSS_SOFT;
    hnucleo_Spi.Init.TIMode = SPI_TIMODE_DISABLE;
    hnucleo_Spi.Init.Mode = SPI_MODE_MASTER;

    SPIx_MspInit(&hnucleo_Spi);
    HAL_SPI_Init(&hnucleo_Spi);
  }
}

/**
  * @brief  SPI Write byte(s) to device
  * @param  DataIn: Pointer to data buffer to write
  * @param  DataOut: Pointer to data buffer for read data
  * @param  DataLength: number of bytes to write
  * @retval None
  */
static void SPIx_WriteReadData(const uint8_t *DataIn, uint8_t *DataOut, uint16_t DataLength)
{
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_SPI_TransmitReceive(&hnucleo_Spi, (uint8_t *) DataIn, DataOut, DataLength, SpixTimeout);

  /* Check the communication status */
  if (status != HAL_OK)
  {
    /* Execute user timeout callback */
    SPIx_Error();
  }
}

/**
  * @brief  SPI Write a byte to device.
  * @param  Value: value to be written
  * @retval None
  */
static void SPIx_Write(uint8_t Value)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint8_t data;

  status = HAL_SPI_TransmitReceive(&hnucleo_Spi, (uint8_t *) &Value, &data, 1, SpixTimeout);

  /* Check the communication status */
  if (status != HAL_OK)
  {
    /* Execute user timeout callback */
    SPIx_Error();
  }
}

/**
  * @brief  SPI error treatment function
  * @retval None
  */
static void SPIx_Error(void)
{
  /* De-initialize the SPI communication BUS */
  HAL_SPI_DeInit(&hnucleo_Spi);

  /* Re-Initiaize the SPI communication BUS */
  SPIx_Init();
}

/******************************************************************************
                            LINK OPERATIONS
*******************************************************************************/

/********************************* LINK SD ************************************/
/**
  * @brief  Initialize the SD Card and put it into StandBy State (Ready for
  *         data transfer).
  * @retval None
  */
void SD_IO_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  uint8_t counter;

  /* SD_CS_GPIO Periph clock enable */
  SD_CS_GPIO_CLK_ENABLE();

  /* Configure SD_CS_PIN pin: SD Card CS pin */
  GPIO_InitStruct.Pin = SD_CS_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SD_CS_GPIO_PORT, &GPIO_InitStruct);

  /*------------Put SD in SPI mode--------------*/
  /* SD SPI Config */
  SPIx_Init();

  /* SD chip select high */
  SD_CS_HIGH();

  /* Send dummy byte 0xFF, 10 times with CS high */
  /* Rise CS and MOSI for 80 clocks cycles */
  for (counter = 0; counter <= 9; counter++)
  {
    /* Send dummy byte 0xFF */
    SD_IO_WriteByte(SD_DUMMY_BYTE);
  }
}

/**
  * @brief  Set the SD_CS pin.
  * @param  val pin value.
  * @retval None
  */
void SD_IO_CSState(uint8_t val)
{
  if (val == 1)
  {
    SD_CS_HIGH();
  }
  else
  {
    SD_CS_LOW();
  }
}

/**
  * @brief  Write bytes on the SD.
  * @param  DataIn     Input Data buffer pointer.
  * @param  DataOut    Output Data buffer pointer.
  * @param  DataLength Number of byte to send.
  * @retval None
  */
void SD_IO_WriteReadData(const uint8_t *DataIn, uint8_t *DataOut, uint16_t DataLength)
{
//  /* SD chip select low */
//  SD_CS_LOW();
  /* Send the byte */
  SPIx_WriteReadData(DataIn, DataOut, DataLength);
}

/**
  * @brief  Write a byte on the SD.
  * @param  Data byte to send.
  * @retval None
  */
uint8_t SD_IO_WriteByte(uint8_t Data)
{
  uint8_t tmp;
  /* Send the byte */
  SPIx_WriteReadData(&Data, &tmp, 1);
  return tmp;
}

/********************************* LINK LCD ***********************************/
/**
  * @brief  Initialize the LCD
  * @retval None
  */
void LCD_IO_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  /* LCD_CS_GPIO and LCD_DC_GPIO Periph clock enable */
  LCD_CS_GPIO_CLK_ENABLE();
  LCD_DC_GPIO_CLK_ENABLE();

  /* Configure LCD_CS_PIN pin: LCD Card CS pin */
  GPIO_InitStruct.Pin = LCD_CS_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LCD_CS_GPIO_PORT, &GPIO_InitStruct);

  /* Configure LCD_DC_PIN pin: LCD Card DC pin */
  GPIO_InitStruct.Pin = LCD_DC_PIN;
  HAL_GPIO_Init(LCD_DC_GPIO_PORT, &GPIO_InitStruct);

  /* LCD chip select high */
  LCD_CS_HIGH();

  /* LCD SPI Config */
  SPIx_Init();
}

/**
  * @brief  Write command to select the LCD register.
  * @param  LCDReg Address of the selected register.
  * @retval None
  */
void LCD_IO_WriteReg(uint8_t LCDReg)
{
  /* Reset LCD control line CS */
  LCD_CS_LOW();

  /* Set LCD data/command line DC to Low */
  LCD_DC_LOW();

  /* Send Command */
  SPIx_Write(LCDReg);

  /* Deselect : Chip Select high */
  LCD_CS_HIGH();
}

/**
  * @brief  Write data to select the LCD register.
  *         This function must be used after st7735_WriteReg() function
  * @param  Data data to write to the selected register.
  * @retval None
  */
void LCD_IO_WriteData(uint8_t Data)
{
  /* Reset LCD control line CS */
  LCD_CS_LOW();

  /* Set LCD data/command line DC to High */
  LCD_DC_HIGH();

  /* Send Data */
  SPIx_Write(Data);

  /* Deselect : Chip Select high */
  LCD_CS_HIGH();
}

/**
* @brief  Write register value.
* @param  pData Pointer on the register value
* @param  Size Size of byte to transmit to the register
* @retval None
*/
void LCD_IO_WriteMultipleData(uint8_t *pData, uint32_t Size)
{
  uint32_t counter = 0;
  __IO uint32_t data = 0;

  /* Reset LCD control line CS */
  LCD_CS_LOW();

  /* Set LCD data/command line DC to High */
  LCD_DC_HIGH();

  if (Size == 1)
  {
    /* Only 1 byte to be sent to LCD - general interface can be used */
    /* Send Data */
    SPIx_Write(*pData);
  }
  else
  {
    /* Several data should be sent in a raw */
    /* Direct SPI accesses for optimization */
    for (counter = Size; counter != 0; counter--)
    {
      while (((hnucleo_Spi.Instance->SR) & SPI_FLAG_TXE) != SPI_FLAG_TXE)
      {
      }
      /* Need to invert bytes for LCD*/
      *((__IO uint8_t *)&hnucleo_Spi.Instance->DR) = *(pData + 1);

      while (((hnucleo_Spi.Instance->SR) & SPI_FLAG_TXE) != SPI_FLAG_TXE)
      {
      }
      *((__IO uint8_t *)&hnucleo_Spi.Instance->DR) = *pData;
      counter--;
      pData += 2;
    }

    /* Wait until the bus is ready before releasing Chip select */
    while (((hnucleo_Spi.Instance->SR) & SPI_FLAG_BSY) != RESET)
    {
    }
  }

  /* Empty the Rx fifo */
  data = *(&hnucleo_Spi.Instance->DR);
  UNUSED(data);

  /* Deselect : Chip Select high */
  LCD_CS_HIGH();
}

/**
  * @brief  Wait for loop in ms.
  * @param  Delay in ms.
  * @retval None
  */
void LCD_Delay(uint32_t Delay)
{
  HAL_Delay(Delay);
}
#endif /* HAL_SPI_MODULE_ENABLED */

#ifdef HAL_ADC_MODULE_ENABLED
/******************************* LINK JOYSTICK ********************************/
/**
  * @brief  Initialize ADC MSP.
  * @param  hadc ADC handle
  * @retval None
  */
static void ADCx_MspInit(ADC_HandleTypeDef *hadc)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  /*** Configure the GPIOs ***/
  /* Enable GPIO clock */
  NUCLEO_ADCx_GPIO_CLK_ENABLE();

  /* Configure the selected ADC Channel as analog input */
  GPIO_InitStruct.Pin = NUCLEO_ADCx_GPIO_PIN ;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(NUCLEO_ADCx_GPIO_PORT, &GPIO_InitStruct);

  /*** Configure the ADC peripheral ***/
  /* Enable ADC clock */
  NUCLEO_ADCx_CLK_ENABLE();
}

/**
  * @brief  DeInitialize ADC MSP.
  * @param  hadc ADC handle
  * @note   ADCx_MspDeInit does not disable the GPIO clock
  * @retval None
  */
static void ADCx_MspDeInit(ADC_HandleTypeDef *hadc)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  /*** DeInit the ADC peripheral ***/
  /* Disable ADC clock */
  NUCLEO_ADCx_CLK_DISABLE();

  /* Configure the selected ADC Channel as analog input */
  GPIO_InitStruct.Pin = NUCLEO_ADCx_GPIO_PIN ;
  HAL_GPIO_DeInit(NUCLEO_ADCx_GPIO_PORT, GPIO_InitStruct.Pin);

  /* Disable GPIO clock has to be done by the application*/
  /* NUCLEO_ADCx_GPIO_CLK_DISABLE(); */
}

/**
  * @brief  Initialize ADC HAL.
  * @retval None
  */
static void ADCx_Init(void)
{
  /* Set ADC instance */
  hnucleo_Adc.Instance                   = NUCLEO_ADCx;

  if (HAL_ADC_GetState(&hnucleo_Adc) == HAL_ADC_STATE_RESET)
  {
    /* ADC Config */
    hnucleo_Adc.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;      /* ADC clock  */
    hnucleo_Adc.Init.Resolution            = ADC_RESOLUTION_12B;
    hnucleo_Adc.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hnucleo_Adc.Init.ScanConvMode          = ADC_SCAN_DISABLE;              /* Sequencer disabled (ADC conversion on only 1 channel: channel set on rank 1) */
    hnucleo_Adc.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    hnucleo_Adc.Init.LowPowerAutoWait      = DISABLE;
    hnucleo_Adc.Init.ContinuousConvMode    = DISABLE;                       /* Continuous mode disabled to have only 1 conversion at each conversion trig */
    hnucleo_Adc.Init.NbrOfConversion       = 1;                             /* Parameter discarded because sequencer is disabled */
    hnucleo_Adc.Init.DiscontinuousConvMode = DISABLE;                       /* Parameter discarded because sequencer is disabled */
    hnucleo_Adc.Init.NbrOfDiscConversion   = 1;                             /* Parameter discarded because sequencer is disabled */
    hnucleo_Adc.Init.ExternalTrigConv      = ADC_SOFTWARE_START;            /* Software start to trig the 1st conversion manually, without external event */
    hnucleo_Adc.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE; /* Parameter discarded because trig by software start */
    hnucleo_Adc.Init.DMAContinuousRequests = DISABLE;
    hnucleo_Adc.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;

    /* Initialize MSP related to ADC */
    ADCx_MspInit(&hnucleo_Adc);

    /* Initialize ADC */
    HAL_ADC_Init(&hnucleo_Adc);

    /* Run ADC calibration */
    HAL_ADCEx_Calibration_Start(&hnucleo_Adc, ADC_SINGLE_ENDED);
  }
}

/**
  * @brief  DeInitialize ADC HAL.
  * @retval None
  */
static void ADCx_DeInit(void)
{
  hnucleo_Adc.Instance   = NUCLEO_ADCx;

  HAL_ADC_DeInit(&hnucleo_Adc);
  ADCx_MspDeInit(&hnucleo_Adc);
}

#endif /* HAL_ADC_MODULE_ENABLED */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */


