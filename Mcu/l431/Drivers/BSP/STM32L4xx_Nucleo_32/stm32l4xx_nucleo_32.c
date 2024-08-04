/**
  ******************************************************************************
  * @file    stm32l4xx_nucleo_32.c
  * @author  MCD Application Team
  * @brief   This file provides set of firmware functions to manage:
  *          - LED available on STM32L4XX-Nucleo Kit 
  *            from STMicroelectronics.
  *          - Gravitech 7segment shield available separately.
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
#include "stm32l4xx_nucleo_32.h"
    
/** @addtogroup BSP
  * @{
  */ 
  
/** @addtogroup STM32L4XX_NUCLEO_32 NUCLEO 32
  * @{
  */
  
/** @defgroup STM32L4XX_NUCLEO_32_Private_Constants Private Constants
  * @{
  */ 
  
/**
  * @brief STM32L4XX NUCLEO BSP Driver version number
  */
#define __STM32L4XX_NUCLEO_32_BSP_VERSION_MAIN   (0x01) /*!< [31:24] main version */
#define __STM32L4XX_NUCLEO_32_BSP_VERSION_SUB1   (0x00) /*!< [23:16] sub1 version */
#define __STM32L4XX_NUCLEO_32_BSP_VERSION_SUB2   (0x05) /*!< [15:8]  sub2 version */
#define __STM32L4XX_NUCLEO_32_BSP_VERSION_RC     (0x00) /*!< [7:0]  release candidate */
#define __STM32L4XX_NUCLEO_32_BSP_VERSION        ((__STM32L4XX_NUCLEO_32_BSP_VERSION_MAIN << 24)\
                                                 |(__STM32L4XX_NUCLEO_32_BSP_VERSION_SUB1 << 16)\
                                                 |(__STM32L4XX_NUCLEO_32_BSP_VERSION_SUB2 << 8 )\
                                                 |(__STM32L4XX_NUCLEO_32_BSP_VERSION_RC))

/** @defgroup STM32L4XX_NUCLEO_32_Private_Variables Private Variables
  * @{
  */ 
GPIO_TypeDef* LED_PORT[LEDn] = {LED3_GPIO_PORT};
const uint16_t LED_PIN[LEDn] = {LED3_PIN};
                                

/**
 * @brief BUS variables
 */
#if defined(HAL_I2C_MODULE_ENABLED)
uint32_t I2c1Timeout = BSP_I2C1_TIMEOUT_MAX;    /*<! Value of Timeout when I2C1 communication fails */
I2C_HandleTypeDef nucleo32_I2c1;
#endif /* HAL_I2C_MODULE_ENABLED */

/**
  * @}
  */ 

/** @defgroup STM32L4XX_NUCLEO_32_Private_Function_Prototypes Private Function Prototypes
  * @{
  */ 

#if defined(HAL_I2C_MODULE_ENABLED)
/* I2C1 bus function */
/* Link function for I2C peripherals */
void               I2C1_Init(void);
void               I2C1_Error (void);
void               I2C1_MspInit(I2C_HandleTypeDef *hi2c);  
void               I2C1_Write(uint8_t Addr, uint8_t Reg, uint8_t Value);
uint8_t            I2C1_Read(uint8_t Addr, uint8_t Reg);
HAL_StatusTypeDef  I2C1_WriteBuffer(uint16_t Addr, uint8_t Reg, uint16_t RegSize, uint8_t *pBuffer, uint16_t Length);
HAL_StatusTypeDef  I2C1_ReadBuffer(uint16_t Addr, uint8_t Reg, uint16_t RegSize, uint8_t *pBuffer, uint16_t Length);
HAL_StatusTypeDef  I2C1_IsDeviceReady(uint16_t DevAddress, uint32_t Trials);
#endif /* HAL_I2C_MODULE_ENABLED */

/**
  * @}
  */ 

/** @addtogroup STM32L4XX_NUCLEO_32_Exported_Functions
  * @{
  */

/**
  * @brief  This method returns the STM32L4XX NUCLEO BSP Driver revision.
  * @retval version : 0xXYZR (8bits for each decimal, R for RC)
  */
uint32_t BSP_GetVersion(void)
{
  return __STM32L4XX_NUCLEO_32_BSP_VERSION;
}

/**
  * @brief  Configures LED GPIO.
  * @param  Led: Specifies the Led to be configured. 
  *   This parameter can be one of following parameters:
  *            @arg  LED3
  * @retval None
  */
void BSP_LED_Init(Led_TypeDef Led)
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  /* Enable the GPIO_LED Clock */
  LEDx_GPIO_CLK_ENABLE(Led);

  /* Configure the GPIO_LED pin */
  GPIO_InitStruct.Pin = LED_PIN[Led];
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  
  HAL_GPIO_Init(LED_PORT[Led], &GPIO_InitStruct);
  HAL_GPIO_WritePin(LED_PORT[Led], LED_PIN[Led], GPIO_PIN_RESET); 
}

/**
  * @brief  Turns selected LED On.
  * @param  Led: Specifies the Led to be set on. 
  *   This parameter can be one of following parameters:
  *            @arg  LED3
  * @retval None
  */
void BSP_LED_On(Led_TypeDef Led)
{
  HAL_GPIO_WritePin(LED_PORT[Led], LED_PIN[Led], GPIO_PIN_SET); 
}

/**
  * @brief  Turns selected LED Off. 
  * @param  Led: Specifies the Led to be set off. 
  *   This parameter can be one of following parameters:
  *            @arg  LED3
  * @retval None
  */
void BSP_LED_Off(Led_TypeDef Led)
{
  HAL_GPIO_WritePin(LED_PORT[Led], LED_PIN[Led], GPIO_PIN_RESET); 
}

/**
  * @brief  Toggles the selected LED.
  * @param  Led: Specifies the Led to be toggled. 
  *   This parameter can be one of following parameters:
  *            @arg  LED3
  * @retval None
  */
void BSP_LED_Toggle(Led_TypeDef Led)
{
  HAL_GPIO_TogglePin(LED_PORT[Led], LED_PIN[Led]);
}

/**
  * @}
  */

/** @defgroup STM32L4XX_NUCLEO_32_Private_Functions Private Functions
  * @{
  */ 

/******************************************************************************
                            BUS OPERATIONS
*******************************************************************************/
#if defined(HAL_I2C_MODULE_ENABLED)
/******************************* I2C Routines *********************************/

/**
  * @brief I2C Bus initialization
  * @retval None
  */
void I2C1_Init(void)
{
  if(HAL_I2C_GetState(&nucleo32_I2c1) == HAL_I2C_STATE_RESET)
  {
    nucleo32_I2c1.Instance              = BSP_I2C1;
    nucleo32_I2c1.Init.Timing           = I2C1_TIMING;
    nucleo32_I2c1.Init.OwnAddress1      = 0;
    nucleo32_I2c1.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    nucleo32_I2c1.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    nucleo32_I2c1.Init.OwnAddress2      = 0;
    nucleo32_I2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    nucleo32_I2c1.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    nucleo32_I2c1.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;  

    /* Init the I2C */
    I2C1_MspInit(&nucleo32_I2c1);
    HAL_I2C_Init(&nucleo32_I2c1);
  }
}

/**
  * @brief  Writes a single data.
  * @param  Addr: I2C address
  * @param  Reg: Register address 
  * @param  Value: Data to be written
  * @retval None
  */
void I2C1_Write(uint8_t Addr, uint8_t Reg, uint8_t Value)
{
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_I2C_Mem_Write(&nucleo32_I2c1, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, &Value, 1, I2c1Timeout); 

  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    I2C1_Error();
  }
}

/**
  * @brief  Reads a single data.
  * @param  Addr: I2C address
  * @param  Reg: Register address 
  * @retval Read data
  */
uint8_t I2C1_Read(uint8_t Addr, uint8_t Reg)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint8_t Value = 0;
  
  status = HAL_I2C_Mem_Read(&nucleo32_I2c1, Addr, Reg, I2C_MEMADD_SIZE_8BIT, &Value, 1, I2c1Timeout);
  
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    I2C1_Error();
  }
  return Value;   
}

/**
  * @brief  Reads multiple data on the BUS.
  * @param  Addr  : I2C Address
  * @param  Reg   : Reg Address 
  * @param  RegSize : The target register size (can be 8BIT or 16BIT)
  * @param  pBuffer : pointer to read data buffer
  * @param  Length : length of the data
  * @retval 0 if no problems to read multiple data
  */
HAL_StatusTypeDef I2C1_ReadBuffer(uint16_t Addr, uint8_t Reg, uint16_t RegSize, uint8_t *pBuffer, uint16_t Length)
{
  HAL_StatusTypeDef status = HAL_OK;
  
  status = HAL_I2C_Mem_Read(&nucleo32_I2c1, Addr, Reg, RegSize, pBuffer, Length, I2c1Timeout);
  
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Re-Initiaize the BUS */
    I2C1_Error();
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
HAL_StatusTypeDef I2C1_IsDeviceReady(uint16_t DevAddress, uint32_t Trials)
{ 
  return (HAL_I2C_IsDeviceReady(&nucleo32_I2c1, DevAddress, Trials, I2c1Timeout));
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
HAL_StatusTypeDef I2C1_WriteBuffer(uint16_t Addr, uint8_t Reg, uint16_t RegSize, uint8_t *pBuffer, uint16_t Length)
{
  HAL_StatusTypeDef status = HAL_OK;
  
  status = HAL_I2C_Mem_Write(&nucleo32_I2c1, Addr, Reg, RegSize, pBuffer, Length, I2c1Timeout); 
  
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Re-Initiaize the BUS */
    I2C1_Error();
  }        
  return status;
}

/**
  * @brief  Manages error callback to re-initialize I2C.
  * @retval None
  */
void I2C1_Error(void)
{
  /* De-initialize the I2C communication BUS */
  HAL_I2C_DeInit(&nucleo32_I2c1);
  
  /* Re-Initiaize the I2C communication BUS */
  I2C1_Init();
}

/**
  * @brief I2C MSP Initialization
  * @param hi2c: I2C handle
  * @retval None
  */
void I2C1_MspInit(I2C_HandleTypeDef *hi2c)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  RCC_PeriphCLKInitTypeDef RCC_PeriphCLKInitStruct;

  /*##-1- Set source clock to SYSCLK for I2C1 ################################################*/  
  RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  RCC_PeriphCLKInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
  HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);

  /*##-2- Configure the GPIOs ################################################*/  
  
  /* Enable GPIO clock */
  BSP_I2C1_GPIO_CLK_ENABLE();

  /* Configure I2C SCL & SDA as alternate function  */
  GPIO_InitStruct.Pin       = (BSP_I2C1_SCL_PIN| BSP_I2C1_SDA_PIN);
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = BSP_I2C1_SCL_SDA_AF;
  HAL_GPIO_Init(BSP_I2C1_GPIO_PORT, &GPIO_InitStruct);

  /*##-3- Configure the Eval I2C peripheral #######################################*/ 
  /* Enable I2C clock */
  BSP_I2C1_CLK_ENABLE();

  /* Force the I2C peripheral clock reset */
  BSP_I2C1_FORCE_RESET();

  /* Release the I2C peripheral clock reset */
  BSP_I2C1_RELEASE_RESET();
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

/** @addtogroup STM32L4XX_NUCLEO_32_GRAVITECH_4DIGITS
  * @{
  */
  
/** @addtogroup STM32_GRAVITECH_4DIGITS_Exported_Functions
  * @{
  */

#if defined(HAL_I2C_MODULE_ENABLED)

/**
  * @brief  BSP init of the 4 digits 7 Segment Display shield for Arduino Nano - Gravitech.
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef BSP_DIGIT4_SEG7_Init(void)
{
  uint8_t control[1] = {0x47};

  /* Init I2C */
  I2C1_Init();
  
  /* Configure the SAA1064 component */
  return I2C1_WriteBuffer(0x70, 0, 1, control, sizeof(control));
}

/**
  * @brief  BSP of the 4 digits 7 Segment Display shield for Arduino Nano - Gravitech.
            Display the value if value belong to [0-9999]
  * @param  Value A number between 0 and 9999 will be displayed on the screen.
  *         DIGIT4_SEG7_RESET will reset the screen (any value above 9999 will reset the screen also)
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef BSP_DIGIT4_SEG7_Display(uint32_t Value)
{
  const uint8_t lookup[10] = {0x3F,0x06,0x5B,0x4F,0x66,
                              0x6D,0x7D,0x07,0x7F,0x6F};
                              
  uint32_t thousands, hundreds, tens, base;
  HAL_StatusTypeDef status = HAL_ERROR;
  uint8_t d1d2d3d4[4];
  
  if (Value < 10000)
  {
    thousands = Value / 1000;
    hundreds = (Value - (thousands * 1000)) / 100;
    tens = (Value - ((thousands * 1000) + (hundreds * 100))) / 10;
    base = Value - ((thousands * 1000) + (hundreds * 100) + (tens * 10));

    d1d2d3d4[3] = lookup[thousands];
    d1d2d3d4[2] = lookup[hundreds];
    d1d2d3d4[1] = lookup[tens];
    d1d2d3d4[0] = lookup[base];
    
  }
  else
  {
    d1d2d3d4[3] = 0;
    d1d2d3d4[2] = 0;
    d1d2d3d4[1] = 0;
    d1d2d3d4[0] = 0;
  }
  
  /* Send the four digits to the SAA1064 component */
  status = I2C1_WriteBuffer(0x70, 1, 1, d1d2d3d4, sizeof(d1d2d3d4));

  return status;
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


