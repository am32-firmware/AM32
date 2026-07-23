/**
  *
  * @file    g32f031_ddl_gpio.c
  * @brief   GPIO DDL module driver.
  *
  * @attention
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *
  * 1. Redistributions of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of the copyright holder nor the names of its contributors
  *    may be used to endorse or promote products derived from this software without
  *    specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
  * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
  * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
  * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
  * OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  * The original code has been modified by Geehy Semiconductor.
  *
  * Copyright (c) 2017 STMicroelectronics.
 *  Copyright (C) 2026 Geehy Semiconductor
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  *
  */
#if defined(USE_FULL_DDL_DRIVER)

/* Includes ------------------------------------------------------------------*/
#include "g32f031_ddl_gpio.h"
#include "g32f031_ddl_rcc.h"
#include "g32f031_ddl_bus.h"

#ifdef  USE_FULL_ASSERT
#include "g32_assert.h"
#else
#define ASSERT_PARAM(_PARAM_) ((void)0U)
#endif

/** @addtogroup G32F031_DDL_Driver
  * @{
  */

#if defined (GPIOA) || defined (GPIOB)

/** @addtogroup GPIO_DDL GPIO
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/** @addtogroup GPIO_DDL_Private_Macros GPIO Private Macros
  * @{
  */
#define IS_DDL_GPIO_PIN(__VALUE__)          (((0x00000000U) < (__VALUE__)) && ((__VALUE__) <= (DDL_GPIO_PIN_ALL)))

#define IS_DDL_GPIO_MODE(__VALUE__)         (((__VALUE__) == DDL_GPIO_MODE_ANALOG)     ||\
                                            ((__VALUE__) == DDL_GPIO_MODE_INPUT)    ||\
                                            ((__VALUE__) == DDL_GPIO_MODE_OUTPUT) ||\
                                            ((__VALUE__) == DDL_GPIO_MODE_ALTERNATE))

#define IS_DDL_GPIO_OUTPUT_TYPE(__VALUE__)  (((__VALUE__) == DDL_GPIO_OUTPUT_PUSHPULL)  ||\
                                            ((__VALUE__) == DDL_GPIO_OUTPUT_OPENDRAIN))

#define IS_DDL_GPIO_DRIVE(__VALUE__)        (((__VALUE__) == DDL_GPIO_DRIVE_LOW)       ||\
                                            ((__VALUE__) == DDL_GPIO_DRIVE_HIGH))

#define IS_DDL_GPIO_PULL(__VALUE__)         (((__VALUE__) == DDL_GPIO_PULL_NO)   ||\
                                            ((__VALUE__) == DDL_GPIO_PULL_UP)   ||\
                                            ((__VALUE__) == DDL_GPIO_PULL_DOWN))

#define IS_DDL_GPIO_ALTERNATE(__VALUE__)    (((__VALUE__) == DDL_GPIO_AF_0  )   ||\
                                            ((__VALUE__) == DDL_GPIO_AF_1  )   ||\
                                            ((__VALUE__) == DDL_GPIO_AF_2  )   ||\
                                            ((__VALUE__) == DDL_GPIO_AF_3  )   ||\
                                            ((__VALUE__) == DDL_GPIO_AF_4  )   ||\
                                            ((__VALUE__) == DDL_GPIO_AF_5  )   ||\
                                            ((__VALUE__) == DDL_GPIO_AF_6  )   ||\
                                            ((__VALUE__) == DDL_GPIO_AF_7  )   ||\
                                            ((__VALUE__) == DDL_GPIO_AF_8  )   ||\
                                            ((__VALUE__) == DDL_GPIO_AF_9  )   ||\
                                            ((__VALUE__) == DDL_GPIO_AF_10 )   ||\
                                            ((__VALUE__) == DDL_GPIO_AF_11 )   ||\
                                            ((__VALUE__) == DDL_GPIO_AF_12 )   ||\
                                            ((__VALUE__) == DDL_GPIO_AF_13 )   ||\
                                            ((__VALUE__) == DDL_GPIO_AF_14 )   ||\
                                            ((__VALUE__) == DDL_GPIO_AF_15 ))
/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/** @addtogroup GPIO_DDL_Exported_Functions GPIO Exported Functions
  * @{
  */

/** @addtogroup GPIO_DDL_EF_Init
  * @{
  */

/**
  * @brief  De-initialize GPIO registers (Registers restored to their default values).
  * @param  GPIOx GPIO Port
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: GPIO registers are de-initialized
  *          - ERROR:   Wrong GPIO Port
  */
ErrorStatus DDL_GPIO_DeInit(GPIO_TypeDef *GPIOx)
{
  ErrorStatus status = SUCCESS;

  /* Check the parameters */
  ASSERT_PARAM(IS_GPIO_ALL_INSTANCE(GPIOx));

  DDL_RCC_Unlock();
  /* Force and Release reset on clock of GPIOx Port */
  if (GPIOx == GPIOA)
  {
    DDL_AHB_GRP1_ForceReset(DDL_AHB_GRP1_PERIPH_GPIOA);
    DDL_AHB_GRP1_ReleaseReset(DDL_AHB_GRP1_PERIPH_GPIOA);
  }
  else if (GPIOx == GPIOB)
  {
    DDL_AHB_GRP1_ForceReset(DDL_AHB_GRP1_PERIPH_GPIOB);
    DDL_AHB_GRP1_ReleaseReset(DDL_AHB_GRP1_PERIPH_GPIOB);
  }
  else
  {
    status = ERROR;
  }

  DDL_RCC_Lock();
  return (status);
}

/**
  * @brief  Initialize GPIO registers according to the specified parameters in GPIO_InitStruct.
  * @param  GPIOx GPIO Port
  * @param  GPIO_InitStruct pointer to a @ref DDL_GPIO_InitTypeDef structure
  *         that contains the configuration information for the specified GPIO peripheral.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: GPIO registers are initialized according to GPIO_InitStruct content
  *          - ERROR:   Not applicable
  */
ErrorStatus DDL_GPIO_Init(GPIO_TypeDef *GPIOx, DDL_GPIO_InitTypeDef *GPIO_InitStruct)
{
  uint32_t pinpos     = 0x00000000U;
  uint32_t currentpin = 0x00000000U;

  /* Check the parameters */
  ASSERT_PARAM(IS_GPIO_ALL_INSTANCE(GPIOx));
  ASSERT_PARAM(IS_DDL_GPIO_PIN(GPIO_InitStruct->Pin));
  ASSERT_PARAM(IS_DDL_GPIO_MODE(GPIO_InitStruct->Mode));
  ASSERT_PARAM(IS_DDL_GPIO_PULL(GPIO_InitStruct->Pull));

  /* ------------------------- Configure the port pins ---------------- */
  /* Initialize  pinpos on first pin set */
  pinpos = POSITION_VAL(GPIO_InitStruct->Pin);

  /* Configure the port pins */
  while (((GPIO_InitStruct->Pin) >> pinpos) != 0x00000000U)
  {
    /* Get current io position */
    currentpin = (GPIO_InitStruct->Pin) & (0x00000001U << pinpos);

    if (currentpin)
    {

      if ((GPIO_InitStruct->Mode == DDL_GPIO_MODE_OUTPUT) || (GPIO_InitStruct->Mode == DDL_GPIO_MODE_ALTERNATE))
      {
        /* Check Drive mode parameters */
        ASSERT_PARAM(IS_DDL_GPIO_DRIVE(GPIO_InitStruct->Drive));

        /* Drive mode configuration */
        DDL_GPIO_SetPinOutputDriveType(GPIOx, currentpin, GPIO_InitStruct->Drive);

        /* Check Output mode parameters */
        ASSERT_PARAM(IS_DDL_GPIO_OUTPUT_TYPE(GPIO_InitStruct->OutputType));

        /* Output mode configuration*/
        DDL_GPIO_SetPinOutputType(GPIOx, currentpin, GPIO_InitStruct->OutputType);
      }

      if (GPIO_InitStruct->Mode == DDL_GPIO_MODE_INPUT)
      {
        /* Enable Input mode */
        DDL_GPIO_SetPinInputMode(GPIOx, currentpin, GPIO_InitStruct->InputEnable);
      }

      /* Pull-up Pull down resistor configuration*/
      DDL_GPIO_SetPinPull(GPIOx, currentpin, GPIO_InitStruct->Pull);

      if (GPIO_InitStruct->Mode == DDL_GPIO_MODE_ALTERNATE)
      {
        /* Check Alternate parameter */
        ASSERT_PARAM(IS_DDL_GPIO_ALTERNATE(GPIO_InitStruct->Alternate));

        /* Alternate mode configuration */
        if (POSITION_VAL(currentpin) < 0x00000008U)
        {
          DDL_GPIO_SetAFPin_0_7(GPIOx, currentpin, GPIO_InitStruct->Alternate);
        }
        else
        {
          DDL_GPIO_SetAFPin_8_15(GPIOx, currentpin, GPIO_InitStruct->Alternate);
        }
      }

      /* Pin Mode configuration */
      DDL_GPIO_SetPinMode(GPIOx, currentpin, GPIO_InitStruct->Mode);
    }
    pinpos++;
  }

  return (SUCCESS);
}

/**
  * @brief Set each @ref DDL_GPIO_InitTypeDef field to default value.
  * @param GPIO_InitStruct pointer to a @ref DDL_GPIO_InitTypeDef structure
  *                          whose fields will be set to default values.
  * @retval None
  */

void DDL_GPIO_StructInit(DDL_GPIO_InitTypeDef *GPIO_InitStruct)
{
  /* Reset GPIO init structure parameters values */
  GPIO_InitStruct->Pin         = DDL_GPIO_PIN_ALL;
  GPIO_InitStruct->Mode        = DDL_GPIO_MODE_ANALOG;
  GPIO_InitStruct->Drive       = DDL_GPIO_DRIVE_LOW;
  GPIO_InitStruct->OutputType  = DDL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct->InputEnable = DDL_GPIO_INPUT_DISABLE;
  GPIO_InitStruct->Pull        = DDL_GPIO_PULL_NO;
  GPIO_InitStruct->Alternate   = DDL_GPIO_AF_0;
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#endif /* GPIOA || GPIOB */

/**
  * @}
  */

#endif /* USE_FULL_DDL_DRIVER */

