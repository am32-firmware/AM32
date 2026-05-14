/**
  *
  * @file    g32f031_ddl_gpio.h
  * @brief   Header file of GPIO DDL module.
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
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef G32F031_DDL_GPIO_H
#define G32F031_DDL_GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "g32f0xx.h"

/** @addtogroup G32F031_DDL_Driver
  * @{
  */

#if defined (GPIOA) || defined (GPIOB)

/** @defgroup GPIO_DDL GPIO
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
#if defined(USE_FULL_DDL_DRIVER)
/** @defgroup GPIO_DDL_ES_INIT GPIO Exported Init structures
  * @{
  */

/**
  * @brief GPIO Init Structure definition
  */
typedef struct
{
  uint32_t Pin;          /*!< Specifies the GPIO pins to be configured.
                              This parameter can be any value of @ref GPIO_DDL_EC_PIN */

  uint32_t Mode;         /*!< Specifies the operating mode for the selected pins.
                              This parameter can be a value of @ref GPIO_DDL_EC_MODE.

                              GPIO HW configuration can be modified afterwards using unitary function @ref DDL_GPIO_SetPinMode().*/

  uint32_t Drive;        /*!< Specifies the drive for the selected pins.
                              This parameter can be a value of @ref GPIO_DDL_EC_DRIVE.

                              GPIO HW configuration can be modified afterwards using unitary function @ref DDL_GPIO_SetPinOutputDriveType().*/

  uint32_t OutputType;   /*!< Specifies the operating output type for the selected pins.
                              This parameter can be a value of @ref GPIO_DDL_EC_OUTPUT.

                              GPIO HW configuration can be modified afterwards using unitary function @ref DDL_GPIO_SetPinOutputType().*/

  uint32_t InputEnable;  /*!< Specifies enable/disable the input type for the selected pins.
                              This parameter can be a value of @ref GPIO_DDL_EC_IER.

                              GPIO HW configuration can be modified afterwards using unitary function @ref DDL_GPIO_SetPinInputMode().*/

  uint32_t Pull;         /*!< Specifies the operating Pull-up/Pull down for the selected pins.
                              This parameter can be a value of @ref GPIO_DDL_EC_PULL.

                              GPIO HW configuration can be modified afterwards using unitary function @ref DDL_GPIO_SetPinPull().*/

  uint32_t Alternate;    /*!< Specifies the Peripheral to be connected to the selected pins.
                              This parameter can be a value of @ref GPIO_DDL_EC_AF.

                              GPIO HW configuration can be modified afterwards using unitary function @ref DDL_GPIO_SetAFPin_0_7() and DDL_GPIO_SetAFPin_8_15().*/
} DDL_GPIO_InitTypeDef;

///**
//  * @brief  GPIO Bit SET and Bit RESET enumeration
//  */
//typedef enum
//{
//  GPIO_PIN_RESET = 0U,
//  GPIO_PIN_SET
//}DDL_GPIO_PinState;

/**
  * @}
  */
#endif /* USE_FULL_DDL_DRIVER */

/* Exported constants --------------------------------------------------------*/
/** @defgroup GPIO_DDL_Exported_Constants GPIO Exported Constants
  * @{
  */

/** @defgroup GPIO_DDL_EC_PIN PIN
  * @{
  */
#define DDL_GPIO_PIN_0                      GPIO_BSRR_BS_0 /*!< Select pin 0 */
#define DDL_GPIO_PIN_1                      GPIO_BSRR_BS_1 /*!< Select pin 1 */
#define DDL_GPIO_PIN_2                      GPIO_BSRR_BS_2 /*!< Select pin 2 */
#define DDL_GPIO_PIN_3                      GPIO_BSRR_BS_3 /*!< Select pin 3 */
#define DDL_GPIO_PIN_4                      GPIO_BSRR_BS_4 /*!< Select pin 4 */
#define DDL_GPIO_PIN_5                      GPIO_BSRR_BS_5 /*!< Select pin 5 */
#define DDL_GPIO_PIN_6                      GPIO_BSRR_BS_6 /*!< Select pin 6 */
#define DDL_GPIO_PIN_7                      GPIO_BSRR_BS_7 /*!< Select pin 7 */
#define DDL_GPIO_PIN_8                      GPIO_BSRR_BS_8 /*!< Select pin 8 */
#define DDL_GPIO_PIN_9                      GPIO_BSRR_BS_9 /*!< Select pin 9 */
#define DDL_GPIO_PIN_10                     GPIO_BSRR_BS_10 /*!< Select pin 10 */
#define DDL_GPIO_PIN_11                     GPIO_BSRR_BS_11 /*!< Select pin 11 */
#define DDL_GPIO_PIN_12                     GPIO_BSRR_BS_12 /*!< Select pin 12 */
#define DDL_GPIO_PIN_13                     GPIO_BSRR_BS_13 /*!< Select pin 13 */
#define DDL_GPIO_PIN_14                     GPIO_BSRR_BS_14 /*!< Select pin 14 */
#define DDL_GPIO_PIN_15                     GPIO_BSRR_BS_15 /*!< Select pin 15 */
#define DDL_GPIO_PIN_ALL                    (GPIO_BSRR_BS_0 | GPIO_BSRR_BS_1  | GPIO_BSRR_BS_2  | \
                                            GPIO_BSRR_BS_3  | GPIO_BSRR_BS_4  | GPIO_BSRR_BS_5  | \
                                            GPIO_BSRR_BS_6  | GPIO_BSRR_BS_7  | GPIO_BSRR_BS_8  | \
                                            GPIO_BSRR_BS_9  | GPIO_BSRR_BS_10 | GPIO_BSRR_BS_11 | \
                                            GPIO_BSRR_BS_12 | GPIO_BSRR_BS_13 | GPIO_BSRR_BS_14 | \
                                            GPIO_BSRR_BS_15) /*!< Select all pins */
/**
  * @}
  */

/** @defgroup GPIO_DDL_EC_MODE Mode
  * @{
  */
#define DDL_GPIO_MODE_ANALOG                (0x00000000U)     /*!< Select analog mode */
#define DDL_GPIO_MODE_INPUT                 GPIO_MDR_MD0_0    /*!< Select input mode */
#define DDL_GPIO_MODE_OUTPUT                GPIO_MDR_MD0_1    /*!< Select output mode */
#define DDL_GPIO_MODE_ALTERNATE             GPIO_MDR_MD0      /*!< Select alternate function mode */
/**
  * @}
  */

/** @defgroup GPIO_DDL_EC_IER Input enable
  * @{
  */
#define DDL_GPIO_INPUT_DISABLE              (0x00000000U)     /*!< Select input disable mode */
#define DDL_GPIO_INPUT_ENABLE               GPIO_INENR_INEN0      /*!< Select input enable mode */
/**
  * @}
  */

/** @defgroup GPIO_DDL_EC_PULL Pull Up Pull Down
  * @{
  */
#define DDL_GPIO_PULL_NO                    (0x00000000U)                           /*!< Select I/O no pull */
#define DDL_GPIO_PULL_UP                    GPIO_PUPDR_PUEN0                      /*!< Select I/O pull up */
#define DDL_GPIO_PULL_DOWN                  (GPIO_PUPDR_PUEN0 | GPIO_PUPDR_PUS0) /*!< Select I/O pull down */
/**
  * @}
  */

/** @defgroup GPIO_DDL_EC_OUTPUT Output Type
  * @{
  */
#define DDL_GPIO_OUTPUT_PUSHPULL            (0x00000000U) /*!< Select push-pull as output type */
#define DDL_GPIO_OUTPUT_OPENDRAIN           GPIO_OTR_OT0  /*!< Select open-drain as output type */
/**
  * @}
  */

/** @defgroup GPIO_DDL_EC_DRIVE Mode
  * @{
  */
#define DDL_GPIO_DRIVE_LOW                  (0x00000000U)      /*!< Select Low output driver type */
#define DDL_GPIO_DRIVE_HIGH                 GPIO_DSR_DS0       /*!< Select high output driver type */
/**
  * @}
  */

/** @defgroup GPIO_DDL_EC_AF Alternate Function
  * @{
  */
#define DDL_GPIO_AF_0                       (0x0000000U) /*!< Select alternate function 0 */
#define DDL_GPIO_AF_1                       (0x0000001U) /*!< Select alternate function 1 */
#define DDL_GPIO_AF_2                       (0x0000002U) /*!< Select alternate function 2 */
#define DDL_GPIO_AF_3                       (0x0000003U) /*!< Select alternate function 3 */
#define DDL_GPIO_AF_4                       (0x0000004U) /*!< Select alternate function 4 */
#define DDL_GPIO_AF_5                       (0x0000005U) /*!< Select alternate function 5 */
#define DDL_GPIO_AF_6                       (0x0000006U) /*!< Select alternate function 6 */
#define DDL_GPIO_AF_7                       (0x0000007U) /*!< Select alternate function 7 */
#define DDL_GPIO_AF_8                       (0x0000008U) /*!< Select alternate function 8 */
#define DDL_GPIO_AF_9                       (0x0000009U) /*!< Select alternate function 9 */
#define DDL_GPIO_AF_10                      (0x000000AU) /*!< Select alternate function 10 */
#define DDL_GPIO_AF_11                      (0x000000BU) /*!< Select alternate function 11 */
#define DDL_GPIO_AF_12                      (0x000000CU) /*!< Select alternate function 12 */
#define DDL_GPIO_AF_13                      (0x000000DU) /*!< Select alternate function 13 */
#define DDL_GPIO_AF_14                      (0x000000EU) /*!< Select alternate function 14 */
#define DDL_GPIO_AF_15                      (0x000000FU) /*!< Select alternate function 15 */
/**
  * @}
  */

/** @defgroup GPIO_DDL_EC_AF3 Alternate Function 3
  * @{
  */
#define DDL_GPIO_AF3_ATMR_CH0               (0x00000000U)                                       /*!< Select alternate function to ATMR Channel 0 */
#define DDL_GPIO_AF3_ATMR_CH1               (GPIO_AF3RMP_AFRMP10_0)                           /*!< Select alternate function to ATMR Channel 1 */
#define DDL_GPIO_AF3_ATMR_CH2               (GPIO_AF3RMP_AFRMP10_1)                           /*!< Select alternate function to ATMR Channel 2 */
#define DDL_GPIO_AF3_ATMR_CH0N              (GPIO_AF3RMP_AFRMP10_1 | GPIO_AF3RMP_AFRMP10_0) /*!< Select alternate function to ATMR Complementary Channel 0 */
#define DDL_GPIO_AF3_ATMR_CH1N              (GPIO_AF3RMP_AFRMP10_2)                           /*!< Select alternate function to ATMR Complementary Channel 1 */
#define DDL_GPIO_AF3_ATMR_CH2N              (GPIO_AF3RMP_AFRMP10_2 | GPIO_AF3RMP_AFRMP10_0) /*!< Select alternate function to ATMR Complementary Channel 2 */
/**
  * @}
  */

/** @defgroup GPIO_DDL_EC_LOCK Mode
  * @{
  */
#define DDL_GPIO_LOCK_DISABLE               (0xA5A55A5AU)    /*!< Select Lock disable mode */
#define DDL_GPIO_LOCK_ENABLE                (0x55555555U)    /*!< Select Lock enable mode */
/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup GPIO_DDL_Exported_Macros GPIO Exported Macros
  * @{
  */

/** @defgroup GPIO_DDL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in GPIO register
  * @param  __INSTANCE__ GPIO Instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define DDL_GPIO_WriteReg(__INSTANCE__, __REG__, __VALUE__) WRITE_REG(__INSTANCE__->__REG__, (__VALUE__))

/**
  * @brief  Read a value in GPIO register
  * @param  __INSTANCE__ GPIO Instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define DDL_GPIO_ReadReg(__INSTANCE__, __REG__) READ_REG(__INSTANCE__->__REG__)
/**
  * @}
  */

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup GPIO_DDL_Exported_Functions GPIO Exported Functions
  * @{
  */

/** @defgroup GPIO_DDL_EF_Port_Configuration Port Configuration
  * @{
  */

/**
  * @brief  Configure gpio mode for a dedicated pin on dedicated port.
  * @note   I/O mode can be Input mode, General purpose output, Alternate function mode or Analog.
  * @note   Warning: only one pin can be passed as parameter.
  * @param  GPIOx GPIO Port
  * @param  Pin This parameter can be one of the following values:
  *         @arg @ref DDL_GPIO_PIN_0
  *         @arg @ref DDL_GPIO_PIN_1
  *         @arg @ref DDL_GPIO_PIN_2
  *         @arg @ref DDL_GPIO_PIN_3
  *         @arg @ref DDL_GPIO_PIN_4
  *         @arg @ref DDL_GPIO_PIN_5
  *         @arg @ref DDL_GPIO_PIN_6
  *         @arg @ref DDL_GPIO_PIN_7
  *         @arg @ref DDL_GPIO_PIN_8
  *         @arg @ref DDL_GPIO_PIN_9
  *         @arg @ref DDL_GPIO_PIN_10
  *         @arg @ref DDL_GPIO_PIN_11
  *         @arg @ref DDL_GPIO_PIN_12
  *         @arg @ref DDL_GPIO_PIN_13
  *         @arg @ref DDL_GPIO_PIN_14
  *         @arg @ref DDL_GPIO_PIN_15
  * @param  Mode This parameter can be one of the following values:
  *         @arg @ref DDL_GPIO_MODE_ANALOG
  *         @arg @ref DDL_GPIO_MODE_INPUT
  *         @arg @ref DDL_GPIO_MODE_OUTPUT
  *         @arg @ref DDL_GPIO_MODE_ALTERNATE
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_SetPinMode(GPIO_TypeDef *GPIOx, uint32_t Pin, uint32_t Mode)
{
  MODIFY_REG(GPIOx->MDR, (GPIO_MDR_MD0 << (POSITION_VAL(Pin) * 2U)), (Mode << (POSITION_VAL(Pin) * 2U)));
}

/**
  * @brief  Return gpio mode for a dedicated pin on dedicated port.
  * @note   I/O mode can be Input mode, General purpose output, Alternate function mode or Analog.
  * @note   Warning: only one pin can be passed as parameter.
  * @param  GPIOx GPIO Port
  * @param  Pin This parameter can be one of the following values:
  *         @arg @ref DDL_GPIO_PIN_0
  *         @arg @ref DDL_GPIO_PIN_1
  *         @arg @ref DDL_GPIO_PIN_2
  *         @arg @ref DDL_GPIO_PIN_3
  *         @arg @ref DDL_GPIO_PIN_4
  *         @arg @ref DDL_GPIO_PIN_5
  *         @arg @ref DDL_GPIO_PIN_6
  *         @arg @ref DDL_GPIO_PIN_7
  *         @arg @ref DDL_GPIO_PIN_8
  *         @arg @ref DDL_GPIO_PIN_9
  *         @arg @ref DDL_GPIO_PIN_10
  *         @arg @ref DDL_GPIO_PIN_11
  *         @arg @ref DDL_GPIO_PIN_12
  *         @arg @ref DDL_GPIO_PIN_13
  *         @arg @ref DDL_GPIO_PIN_14
  *         @arg @ref DDL_GPIO_PIN_15
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_GPIO_MODE_ANALOG
  *         @arg @ref DDL_GPIO_MODE_INPUT
  *         @arg @ref DDL_GPIO_MODE_OUTPUT
  *         @arg @ref DDL_GPIO_MODE_ALTERNATE
  */
__STATIC_INLINE uint32_t DDL_GPIO_GetPinMode(GPIO_TypeDef *GPIOx, uint32_t Pin)
{
  return (uint32_t)(READ_BIT(GPIOx->MDR,
                             (GPIO_MDR_MD0 << (POSITION_VAL(Pin) * 2U))) >> (POSITION_VAL(Pin) * 2U));
}

/**
  * @brief  Configure gpio input mode for several pins on dedicated port.
  * @note   Input type as to be set when gpio pin is in input or
  *         alternate modes. Possible type are enable or disable.
  * @param  GPIOx GPIO Port
  * @param  PinMask This parameter can be a combination of the following values:
  *         @arg @ref DDL_GPIO_PIN_0
  *         @arg @ref DDL_GPIO_PIN_1
  *         @arg @ref DDL_GPIO_PIN_2
  *         @arg @ref DDL_GPIO_PIN_3
  *         @arg @ref DDL_GPIO_PIN_4
  *         @arg @ref DDL_GPIO_PIN_5
  *         @arg @ref DDL_GPIO_PIN_6
  *         @arg @ref DDL_GPIO_PIN_7
  *         @arg @ref DDL_GPIO_PIN_8
  *         @arg @ref DDL_GPIO_PIN_9
  *         @arg @ref DDL_GPIO_PIN_10
  *         @arg @ref DDL_GPIO_PIN_11
  *         @arg @ref DDL_GPIO_PIN_12
  *         @arg @ref DDL_GPIO_PIN_13
  *         @arg @ref DDL_GPIO_PIN_14
  *         @arg @ref DDL_GPIO_PIN_15
  *         @arg @ref DDL_GPIO_PIN_ALL
  * @param  Input This parameter can be one of the following values:
  *         @arg @ref DDL_GPIO_INPUT_DISABLE
  *         @arg @ref DDL_GPIO_INPUT_ENABLE
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_SetPinInputMode(GPIO_TypeDef *GPIOx, uint32_t PinMask, uint32_t InputMode)
{
  MODIFY_REG(GPIOx->INENR, PinMask, (PinMask * InputMode));
}

/**
  * @brief  Return gpio input moed for several pins on dedicated port.
  * @note   Input type as to be set when gpio pin is in input or
  *         alternate modes. Possible type are enable or disable.
  * @note   Warning: only one pin can be passed as parameter.
  * @param  GPIOx GPIO Port
  * @param  Pin This parameter can be one of the following values:
  *         @arg @ref DDL_GPIO_PIN_0
  *         @arg @ref DDL_GPIO_PIN_1
  *         @arg @ref DDL_GPIO_PIN_2
  *         @arg @ref DDL_GPIO_PIN_3
  *         @arg @ref DDL_GPIO_PIN_4
  *         @arg @ref DDL_GPIO_PIN_5
  *         @arg @ref DDL_GPIO_PIN_6
  *         @arg @ref DDL_GPIO_PIN_7
  *         @arg @ref DDL_GPIO_PIN_8
  *         @arg @ref DDL_GPIO_PIN_9
  *         @arg @ref DDL_GPIO_PIN_10
  *         @arg @ref DDL_GPIO_PIN_11
  *         @arg @ref DDL_GPIO_PIN_12
  *         @arg @ref DDL_GPIO_PIN_13
  *         @arg @ref DDL_GPIO_PIN_14
  *         @arg @ref DDL_GPIO_PIN_15
  *         @arg @ref DDL_GPIO_PIN_ALL
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_GPIO_INPUT_DISABLE
  *         @arg @ref DDL_GPIO_INPUT_ENABLE
  */
__STATIC_INLINE uint32_t DDL_GPIO_GetPinInputMode(GPIO_TypeDef *GPIOx, uint32_t Pin)
{
  return (uint32_t)(READ_BIT(GPIOx->INENR, Pin) >> POSITION_VAL(Pin));
}

/**
  * @brief  Configure gpio output type for several pins on dedicated port.
  * @note   Output type as to be set when gpio pin is in output or
  *         alternate modes. Possible type are Push-pull or Open-drain.
  * @param  GPIOx GPIO Port
  * @param  PinMask This parameter can be a combination of the following values:
  *         @arg @ref DDL_GPIO_PIN_0
  *         @arg @ref DDL_GPIO_PIN_1
  *         @arg @ref DDL_GPIO_PIN_2
  *         @arg @ref DDL_GPIO_PIN_3
  *         @arg @ref DDL_GPIO_PIN_4
  *         @arg @ref DDL_GPIO_PIN_5
  *         @arg @ref DDL_GPIO_PIN_6
  *         @arg @ref DDL_GPIO_PIN_7
  *         @arg @ref DDL_GPIO_PIN_8
  *         @arg @ref DDL_GPIO_PIN_9
  *         @arg @ref DDL_GPIO_PIN_10
  *         @arg @ref DDL_GPIO_PIN_11
  *         @arg @ref DDL_GPIO_PIN_12
  *         @arg @ref DDL_GPIO_PIN_13
  *         @arg @ref DDL_GPIO_PIN_14
  *         @arg @ref DDL_GPIO_PIN_15
  *         @arg @ref DDL_GPIO_PIN_ALL
  * @param  OutputType This parameter can be one of the following values:
  *         @arg @ref DDL_GPIO_OUTPUT_PUSHPULL
  *         @arg @ref DDL_GPIO_OUTPUT_OPENDRAIN
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_SetPinOutputType(GPIO_TypeDef *GPIOx, uint32_t PinMask, uint32_t OutputType)
{
  MODIFY_REG(GPIOx->OTR, PinMask, (PinMask * OutputType));
}

/**
  * @brief  Return gpio output type for several pins on dedicated port.
  * @note   Output type as to be set when gpio pin is in output or
  *         alternate modes. Possible type are Push-pull or Open-drain.
  * @note   Warning: only one pin can be passed as parameter.
  * @param  GPIOx GPIO Port
  * @param  Pin This parameter can be one of the following values:
  *         @arg @ref DDL_GPIO_PIN_0
  *         @arg @ref DDL_GPIO_PIN_1
  *         @arg @ref DDL_GPIO_PIN_2
  *         @arg @ref DDL_GPIO_PIN_3
  *         @arg @ref DDL_GPIO_PIN_4
  *         @arg @ref DDL_GPIO_PIN_5
  *         @arg @ref DDL_GPIO_PIN_6
  *         @arg @ref DDL_GPIO_PIN_7
  *         @arg @ref DDL_GPIO_PIN_8
  *         @arg @ref DDL_GPIO_PIN_9
  *         @arg @ref DDL_GPIO_PIN_10
  *         @arg @ref DDL_GPIO_PIN_11
  *         @arg @ref DDL_GPIO_PIN_12
  *         @arg @ref DDL_GPIO_PIN_13
  *         @arg @ref DDL_GPIO_PIN_14
  *         @arg @ref DDL_GPIO_PIN_15
  *         @arg @ref DDL_GPIO_PIN_ALL
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_GPIO_OUTPUT_PUSHPULL
  *         @arg @ref DDL_GPIO_OUTPUT_OPENDRAIN
  */
__STATIC_INLINE uint32_t DDL_GPIO_GetPinOutputType(GPIO_TypeDef *GPIOx, uint32_t Pin)
{
  return (uint32_t)(READ_BIT(GPIOx->OTR, Pin) >> POSITION_VAL(Pin));
}

/**
  * @brief  Configure gpio pull-up or pull-down for a dedicated pin on a dedicated port.
  * @note   Warning: only one pin can be passed as parameter.
  * @param  GPIOx GPIO Port
  * @param  Pin This parameter can be one of the following values:
  *         @arg @ref DDL_GPIO_PIN_0
  *         @arg @ref DDL_GPIO_PIN_1
  *         @arg @ref DDL_GPIO_PIN_2
  *         @arg @ref DDL_GPIO_PIN_3
  *         @arg @ref DDL_GPIO_PIN_4
  *         @arg @ref DDL_GPIO_PIN_5
  *         @arg @ref DDL_GPIO_PIN_6
  *         @arg @ref DDL_GPIO_PIN_7
  *         @arg @ref DDL_GPIO_PIN_8
  *         @arg @ref DDL_GPIO_PIN_9
  *         @arg @ref DDL_GPIO_PIN_10
  *         @arg @ref DDL_GPIO_PIN_11
  *         @arg @ref DDL_GPIO_PIN_12
  *         @arg @ref DDL_GPIO_PIN_13
  *         @arg @ref DDL_GPIO_PIN_14
  *         @arg @ref DDL_GPIO_PIN_15
  * @param  Pull This parameter can be one of the following values:
  *         @arg @ref DDL_GPIO_PULL_NO
  *         @arg @ref DDL_GPIO_PULL_UP
  *         @arg @ref DDL_GPIO_PULL_DOWN
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_SetPinPull(GPIO_TypeDef *GPIOx, uint32_t Pin, uint32_t Pull)
{
  MODIFY_REG(GPIOx->PUPDR, ((GPIO_PUPDR_PUEN0 | GPIO_PUPDR_PUS0) << (POSITION_VAL(Pin) * 2U)), (Pull << (POSITION_VAL(Pin) * 2U)));
}

/**
  * @brief  Return gpio pull-up or pull-down for a dedicated pin on a dedicated port
  * @note   Warning: only one pin can be passed as parameter.
  * @param  GPIOx GPIO Port
  * @param  Pin This parameter can be one of the following values:
  *         @arg @ref DDL_GPIO_PIN_0
  *         @arg @ref DDL_GPIO_PIN_1
  *         @arg @ref DDL_GPIO_PIN_2
  *         @arg @ref DDL_GPIO_PIN_3
  *         @arg @ref DDL_GPIO_PIN_4
  *         @arg @ref DDL_GPIO_PIN_5
  *         @arg @ref DDL_GPIO_PIN_6
  *         @arg @ref DDL_GPIO_PIN_7
  *         @arg @ref DDL_GPIO_PIN_8
  *         @arg @ref DDL_GPIO_PIN_9
  *         @arg @ref DDL_GPIO_PIN_10
  *         @arg @ref DDL_GPIO_PIN_11
  *         @arg @ref DDL_GPIO_PIN_12
  *         @arg @ref DDL_GPIO_PIN_13
  *         @arg @ref DDL_GPIO_PIN_14
  *         @arg @ref DDL_GPIO_PIN_15
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_GPIO_PULL_NO
  *         @arg @ref DDL_GPIO_PULL_UP
  *         @arg @ref DDL_GPIO_PULL_DOWN
  */
__STATIC_INLINE uint32_t DDL_GPIO_GetPinPull(GPIO_TypeDef *GPIOx, uint32_t Pin)
{
  return (uint32_t)(READ_BIT(GPIOx->PUPDR,
                             ((GPIO_PUPDR_PUEN0 | GPIO_PUPDR_PUS0) << (POSITION_VAL(Pin) * 2U))) >> (POSITION_VAL(Pin) * 2U));
}

/**
  * @brief  Configure gpio output driver type for several pins on dedicated port.
  * @note   Output driver type as to be set when gpio pin is in output or
  *         alternate modes. Possible type are low or high.
  * @param  GPIOx GPIO Port
  * @param  PinMask This parameter can be a combination of the following values:
  *         @arg @ref DDL_GPIO_PIN_0
  *         @arg @ref DDL_GPIO_PIN_1
  *         @arg @ref DDL_GPIO_PIN_2
  *         @arg @ref DDL_GPIO_PIN_3
  *         @arg @ref DDL_GPIO_PIN_4
  *         @arg @ref DDL_GPIO_PIN_5
  *         @arg @ref DDL_GPIO_PIN_6
  *         @arg @ref DDL_GPIO_PIN_7
  *         @arg @ref DDL_GPIO_PIN_8
  *         @arg @ref DDL_GPIO_PIN_9
  *         @arg @ref DDL_GPIO_PIN_10
  *         @arg @ref DDL_GPIO_PIN_11
  *         @arg @ref DDL_GPIO_PIN_12
  *         @arg @ref DDL_GPIO_PIN_13
  *         @arg @ref DDL_GPIO_PIN_14
  *         @arg @ref DDL_GPIO_PIN_15
  *         @arg @ref DDL_GPIO_PIN_ALL
  * @param  OutputDriverType This parameter can be one of the following values:
  *         @arg @ref DDL_GPIO_DRIVE_LOW
  *         @arg @ref DDL_GPIO_DRIVE_HIGH
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_SetPinOutputDriveType(GPIO_TypeDef *GPIOx, uint32_t PinMask, uint32_t OutputDriverType)
{
  MODIFY_REG(GPIOx->DSR, PinMask, (PinMask * OutputDriverType));
}

/**
  * @brief  Return gpio output driver type for several pins on dedicated port.
  * @note   Output driver type as to be set when gpio pin is in output or
  *         alternate modes. Possible type are low or high.
  * @note   Warning: only one pin can be passed as parameter.
  * @param  GPIOx GPIO Port
  * @param  Pin This parameter can be one of the following values:
  *         @arg @ref DDL_GPIO_PIN_0
  *         @arg @ref DDL_GPIO_PIN_1
  *         @arg @ref DDL_GPIO_PIN_2
  *         @arg @ref DDL_GPIO_PIN_3
  *         @arg @ref DDL_GPIO_PIN_4
  *         @arg @ref DDL_GPIO_PIN_5
  *         @arg @ref DDL_GPIO_PIN_6
  *         @arg @ref DDL_GPIO_PIN_7
  *         @arg @ref DDL_GPIO_PIN_8
  *         @arg @ref DDL_GPIO_PIN_9
  *         @arg @ref DDL_GPIO_PIN_10
  *         @arg @ref DDL_GPIO_PIN_11
  *         @arg @ref DDL_GPIO_PIN_12
  *         @arg @ref DDL_GPIO_PIN_13
  *         @arg @ref DDL_GPIO_PIN_14
  *         @arg @ref DDL_GPIO_PIN_15
  *         @arg @ref DDL_GPIO_PIN_ALL
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_GPIO_DRIVE_LOW
  *         @arg @ref DDL_GPIO_DRIVE_HIGH
  */
__STATIC_INLINE uint32_t DDL_GPIO_GetPinOutputDriveType(GPIO_TypeDef *GPIOx, uint32_t Pin)
{
  return (uint32_t)(READ_BIT(GPIOx->DSR, Pin) >> POSITION_VAL(Pin));
}

/**
  * @brief  Configure gpio alternate function of a dedicated pin from 0 to 7 for a dedicated port.
  * @note   Possible values are from AF0 to AF7 depending on target.
  * @note   Warning: only one pin can be passed as parameter.
  * @param  GPIOx GPIO Port
  * @param  Pin This parameter can be one of the following values:
  *         @arg @ref DDL_GPIO_PIN_0
  *         @arg @ref DDL_GPIO_PIN_1
  *         @arg @ref DDL_GPIO_PIN_2
  *         @arg @ref DDL_GPIO_PIN_3
  *         @arg @ref DDL_GPIO_PIN_4
  *         @arg @ref DDL_GPIO_PIN_5
  *         @arg @ref DDL_GPIO_PIN_6
  *         @arg @ref DDL_GPIO_PIN_7
  * @param  Alternate This parameter can be one of the following values:
  *         @arg @ref DDL_GPIO_AF_0
  *         @arg @ref DDL_GPIO_AF_1
  *         @arg @ref DDL_GPIO_AF_2
  *         @arg @ref DDL_GPIO_AF_3
  *         @arg @ref DDL_GPIO_AF_4
  *         @arg @ref DDL_GPIO_AF_5
  *         @arg @ref DDL_GPIO_AF_6
  *         @arg @ref DDL_GPIO_AF_7
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_SetAFPin_0_7(GPIO_TypeDef *GPIOx, uint32_t Pin, uint32_t Alternate)
{
  MODIFY_REG(GPIOx->AFSELR0, (GPIO_AFSELR0_AFSEL0 << (POSITION_VAL(Pin) * 4U)),
             (Alternate << (POSITION_VAL(Pin) * 4U)));
}

/**
  * @brief  Return gpio alternate function of a dedicated pin from 0 to 7 for a dedicated port.
  * @param  GPIOx GPIO Port
  * @param  Pin This parameter can be one of the following values:
  *         @arg @ref DDL_GPIO_PIN_0
  *         @arg @ref DDL_GPIO_PIN_1
  *         @arg @ref DDL_GPIO_PIN_2
  *         @arg @ref DDL_GPIO_PIN_3
  *         @arg @ref DDL_GPIO_PIN_4
  *         @arg @ref DDL_GPIO_PIN_5
  *         @arg @ref DDL_GPIO_PIN_6
  *         @arg @ref DDL_GPIO_PIN_7
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_GPIO_AF_0
  *         @arg @ref DDL_GPIO_AF_1
  *         @arg @ref DDL_GPIO_AF_2
  *         @arg @ref DDL_GPIO_AF_3
  *         @arg @ref DDL_GPIO_AF_4
  *         @arg @ref DDL_GPIO_AF_5
  *         @arg @ref DDL_GPIO_AF_6
  *         @arg @ref DDL_GPIO_AF_7
  */
__STATIC_INLINE uint32_t DDL_GPIO_GetAFPin_0_7(GPIO_TypeDef *GPIOx, uint32_t Pin)
{
  return (uint32_t)(READ_BIT(GPIOx->AFSELR0,
                             (GPIO_AFSELR0_AFSEL0 << (POSITION_VAL(Pin) * 4U))) >> (POSITION_VAL(Pin) * 4U));
}

/**
  * @brief  Configure gpio alternate function of a dedicated pin from 8 to 15 for a dedicated port.
  * @note   Possible values are from AF0 to AF7 depending on target.
  * @note   Warning: only one pin can be passed as parameter.
  * @param  GPIOx GPIO Port
  * @param  Pin This parameter can be one of the following values:
  *         @arg @ref DDL_GPIO_PIN_8
  *         @arg @ref DDL_GPIO_PIN_9
  *         @arg @ref DDL_GPIO_PIN_10
  *         @arg @ref DDL_GPIO_PIN_11
  *         @arg @ref DDL_GPIO_PIN_12
  *         @arg @ref DDL_GPIO_PIN_13
  *         @arg @ref DDL_GPIO_PIN_14
  *         @arg @ref DDL_GPIO_PIN_15
  * @param  Alternate This parameter can be one of the following values:
  *         @arg @ref DDL_GPIO_AF_0
  *         @arg @ref DDL_GPIO_AF_1
  *         @arg @ref DDL_GPIO_AF_2
  *         @arg @ref DDL_GPIO_AF_3
  *         @arg @ref DDL_GPIO_AF_4
  *         @arg @ref DDL_GPIO_AF_5
  *         @arg @ref DDL_GPIO_AF_6
  *         @arg @ref DDL_GPIO_AF_7
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_SetAFPin_8_15(GPIO_TypeDef *GPIOx, uint32_t Pin, uint32_t Alternate)
{
  uint32_t temp = POSITION_VAL(Pin >> 8U);
  MODIFY_REG(GPIOx->AFSELR1, (GPIO_AFSELR1_AFSEL8 << temp * 4U),
             Alternate << (temp * 4U));
}

/**
  * @brief  Return gpio alternate function of a dedicated pin from 8 to 15 for a dedicated port.
  * @note   Possible values are from AF0 to AF7 depending on target.
  * @param  GPIOx GPIO Port
  * @param  Pin This parameter can be one of the following values:
  *         @arg @ref DDL_GPIO_PIN_8
  *         @arg @ref DDL_GPIO_PIN_9
  *         @arg @ref DDL_GPIO_PIN_10
  *         @arg @ref DDL_GPIO_PIN_11
  *         @arg @ref DDL_GPIO_PIN_12
  *         @arg @ref DDL_GPIO_PIN_13
  *         @arg @ref DDL_GPIO_PIN_14
  *         @arg @ref DDL_GPIO_PIN_15
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_GPIO_AF_0
  *         @arg @ref DDL_GPIO_AF_1
  *         @arg @ref DDL_GPIO_AF_2
  *         @arg @ref DDL_GPIO_AF_3
  *         @arg @ref DDL_GPIO_AF_4
  *         @arg @ref DDL_GPIO_AF_5
  *         @arg @ref DDL_GPIO_AF_6
  *         @arg @ref DDL_GPIO_AF_7
  */
__STATIC_INLINE uint32_t DDL_GPIO_GetAFPin_8_15(GPIO_TypeDef *GPIOx, uint32_t Pin)
{
  return (uint32_t)(READ_BIT(GPIOx->AFSELR1,
                             (GPIO_AFSELR1_AFSEL8 << (POSITION_VAL(Pin >> 8U) * 4U))) >> (POSITION_VAL(Pin >> 8U) * 4U));
}

/**
  * @brief  Configure gpio alternate function 3 of a dedicated pin from 10 to 15 for a dedicated port.
  * @note   Only for GPIOA.
  * @note   Warning: only one pin can be passed as parameter.
  * @param  GPIOx GPIO Port
  * @param  Pin This parameter can be one of the following values:
  *         @arg @ref DDL_GPIO_PIN_10
  *         @arg @ref DDL_GPIO_PIN_11
  *         @arg @ref DDL_GPIO_PIN_12
  *         @arg @ref DDL_GPIO_PIN_13
  *         @arg @ref DDL_GPIO_PIN_14
  *         @arg @ref DDL_GPIO_PIN_15
  * @param  Alternate This parameter can be one of the following values:
  *         @arg @ref DDL_GPIO_AF3_ATMR_CH0
  *         @arg @ref DDL_GPIO_AF3_ATMR_CH1
  *         @arg @ref DDL_GPIO_AF3_ATMR_CH2
  *         @arg @ref DDL_GPIO_AF3_ATMR_CH0N
  *         @arg @ref DDL_GPIO_AF3_ATMR_CH1N
  *         @arg @ref DDL_GPIO_AF3_ATMR_CH2N
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_SetAF3Pin_10_15(GPIO_TypeDef *GPIOx, uint32_t Pin, uint32_t Alternate)
{
  MODIFY_REG(GPIOx->AF3RMP, (GPIO_AF3RMP_AFRMP10 << ((POSITION_VAL(Pin) - 10U) * 4U)),
             (Alternate << ((POSITION_VAL(Pin) - 10U) * 4U)));
}

/**
  * @brief  Return gpio alternate function of a dedicated pin from 8 to 15 for a dedicated port.
  * @note   Only for GPIOA.
  * @param  GPIOx GPIO Port
  * @param  Pin This parameter can be one of the following values:
  *         @arg @ref DDL_GPIO_PIN_10
  *         @arg @ref DDL_GPIO_PIN_11
  *         @arg @ref DDL_GPIO_PIN_12
  *         @arg @ref DDL_GPIO_PIN_13
  *         @arg @ref DDL_GPIO_PIN_14
  *         @arg @ref DDL_GPIO_PIN_15
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_GPIO_AF3_ATMR_CH0
  *         @arg @ref DDL_GPIO_AF3_ATMR_CH1
  *         @arg @ref DDL_GPIO_AF3_ATMR_CH2
  *         @arg @ref DDL_GPIO_AF3_ATMR_CH0N
  *         @arg @ref DDL_GPIO_AF3_ATMR_CH1N
  *         @arg @ref DDL_GPIO_AF3_ATMR_CH2N
  */
__STATIC_INLINE uint32_t DDL_GPIO_GetAF3Pin_10_15(GPIO_TypeDef *GPIOx, uint32_t Pin)
{
  return (uint32_t)(READ_BIT(GPIOx->AF3RMP,
                             (GPIO_AF3RMP_AFRMP10 << ((POSITION_VAL(Pin) - 10U) << 2U))) >> ((POSITION_VAL(Pin) - 10U) << 2U));
}

/**
  * @brief  Lock key configuration of several port.
  * @param  GPIOx GPIO Port
  * @note   When the lock sequence has been applied on a port bit, the
  *         value of this port bit can no longer be modified until the
  *         next reset.
  * @note   Each lock bit freezes a specific configuration register
  *         (control and alternate function registers).
  * @param  Filter This parameter can be one of the following values:
  *         @arg @ref DDL_GPIO_LOCK_DISABLE
  *         @arg @ref DDL_GPIO_LOCK_ENABLE
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_LockKey(GPIO_TypeDef *GPIOx, uint32_t LockMode)
{
  WRITE_REG(GPIOx->LOCK, LockMode);
}

/**
  * @brief  Return Lock key Register.
  * @param  GPIOx GPIO Port
  * @retval Register value.
  */
__STATIC_INLINE uint32_t DDL_GPIO_IsPinLocked(GPIO_TypeDef *GPIOx)
{
  return (READ_REG(GPIOx->LOCK));
}

/**
  * @}
  */

/** @defgroup GPIO_DDL_EF_Data_Access Data Access
  * @{
  */

/**
  * @brief  Return full input data register value for a dedicated port.
  * @param  GPIOx GPIO Port
  * @retval Input data register value of port
  */
__STATIC_INLINE uint32_t DDL_GPIO_ReadInputPort(GPIO_TypeDef *GPIOx)
{
  return (uint32_t)(READ_REG(GPIOx->INDR));
}

/**
  * @brief  Return if input data level for several pins of dedicated port is high or low.
  * @param  GPIOx GPIO Port
  * @param  PinMask This parameter can be a combination of the following values:
  *         @arg @ref DDL_GPIO_PIN_0
  *         @arg @ref DDL_GPIO_PIN_1
  *         @arg @ref DDL_GPIO_PIN_2
  *         @arg @ref DDL_GPIO_PIN_3
  *         @arg @ref DDL_GPIO_PIN_4
  *         @arg @ref DDL_GPIO_PIN_5
  *         @arg @ref DDL_GPIO_PIN_6
  *         @arg @ref DDL_GPIO_PIN_7
  *         @arg @ref DDL_GPIO_PIN_8
  *         @arg @ref DDL_GPIO_PIN_9
  *         @arg @ref DDL_GPIO_PIN_10
  *         @arg @ref DDL_GPIO_PIN_11
  *         @arg @ref DDL_GPIO_PIN_12
  *         @arg @ref DDL_GPIO_PIN_13
  *         @arg @ref DDL_GPIO_PIN_14
  *         @arg @ref DDL_GPIO_PIN_15
  *         @arg @ref DDL_GPIO_PIN_ALL
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_GPIO_IsInputPinSet(GPIO_TypeDef *GPIOx, uint32_t PinMask)
{
  return (READ_BIT(GPIOx->INDR, PinMask) == (PinMask));
}

/**
  * @brief  Write output data register for the port.
  * @param  GPIOx GPIO Port
  * @param  PortValue Level value for each pin of the port
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_WriteOutputPort(GPIO_TypeDef *GPIOx, uint32_t PortValue)
{
  WRITE_REG(GPIOx->OUTDR, PortValue);
}

/**
  * @brief  Return full output data register value for a dedicated port.
  * @param  GPIOx GPIO Port
  * @retval Output data register value of port
  */
__STATIC_INLINE uint32_t DDL_GPIO_ReadOutputPort(GPIO_TypeDef *GPIOx)
{
  return (uint32_t)(READ_REG(GPIOx->OUTDR));
}

/**
  * @brief  Return if input data level for several pins of dedicated port is high or low.
  * @param  GPIOx GPIO Port
  * @param  PinMask This parameter can be a combination of the following values:
  *         @arg @ref DDL_GPIO_PIN_0
  *         @arg @ref DDL_GPIO_PIN_1
  *         @arg @ref DDL_GPIO_PIN_2
  *         @arg @ref DDL_GPIO_PIN_3
  *         @arg @ref DDL_GPIO_PIN_4
  *         @arg @ref DDL_GPIO_PIN_5
  *         @arg @ref DDL_GPIO_PIN_6
  *         @arg @ref DDL_GPIO_PIN_7
  *         @arg @ref DDL_GPIO_PIN_8
  *         @arg @ref DDL_GPIO_PIN_9
  *         @arg @ref DDL_GPIO_PIN_10
  *         @arg @ref DDL_GPIO_PIN_11
  *         @arg @ref DDL_GPIO_PIN_12
  *         @arg @ref DDL_GPIO_PIN_13
  *         @arg @ref DDL_GPIO_PIN_14
  *         @arg @ref DDL_GPIO_PIN_15
  *         @arg @ref DDL_GPIO_PIN_ALL
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_GPIO_IsOutputPinSet(GPIO_TypeDef *GPIOx, uint32_t PinMask)
{
  return (READ_BIT(GPIOx->OUTDR, PinMask) == (PinMask));
}

/**
  * @brief  Set several pins to high level on dedicated gpio port.
  * @param  GPIOx GPIO Port
  * @param  PinMask This parameter can be a combination of the following values:
  *         @arg @ref DDL_GPIO_PIN_0
  *         @arg @ref DDL_GPIO_PIN_1
  *         @arg @ref DDL_GPIO_PIN_2
  *         @arg @ref DDL_GPIO_PIN_3
  *         @arg @ref DDL_GPIO_PIN_4
  *         @arg @ref DDL_GPIO_PIN_5
  *         @arg @ref DDL_GPIO_PIN_6
  *         @arg @ref DDL_GPIO_PIN_7
  *         @arg @ref DDL_GPIO_PIN_8
  *         @arg @ref DDL_GPIO_PIN_9
  *         @arg @ref DDL_GPIO_PIN_10
  *         @arg @ref DDL_GPIO_PIN_11
  *         @arg @ref DDL_GPIO_PIN_12
  *         @arg @ref DDL_GPIO_PIN_13
  *         @arg @ref DDL_GPIO_PIN_14
  *         @arg @ref DDL_GPIO_PIN_15
  *         @arg @ref DDL_GPIO_PIN_ALL
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_SetOutputPin(GPIO_TypeDef *GPIOx, uint32_t PinMask)
{
  WRITE_REG(GPIOx->BSRR, PinMask);
}

/**
  * @brief  Set several pins to low level on dedicated gpio port.
  * @param  GPIOx GPIO Port
  * @param  PinMask This parameter can be a combination of the following values:
  *         @arg @ref DDL_GPIO_PIN_0
  *         @arg @ref DDL_GPIO_PIN_1
  *         @arg @ref DDL_GPIO_PIN_2
  *         @arg @ref DDL_GPIO_PIN_3
  *         @arg @ref DDL_GPIO_PIN_4
  *         @arg @ref DDL_GPIO_PIN_5
  *         @arg @ref DDL_GPIO_PIN_6
  *         @arg @ref DDL_GPIO_PIN_7
  *         @arg @ref DDL_GPIO_PIN_8
  *         @arg @ref DDL_GPIO_PIN_9
  *         @arg @ref DDL_GPIO_PIN_10
  *         @arg @ref DDL_GPIO_PIN_11
  *         @arg @ref DDL_GPIO_PIN_12
  *         @arg @ref DDL_GPIO_PIN_13
  *         @arg @ref DDL_GPIO_PIN_14
  *         @arg @ref DDL_GPIO_PIN_15
  *         @arg @ref DDL_GPIO_PIN_ALL
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_ResetOutputPin(GPIO_TypeDef *GPIOx, uint32_t PinMask)
{
  WRITE_REG(GPIOx->BSRR, (PinMask << 16));
}

/**
  * @brief  Toggle data value for several pin of dedicated port.
  * @param  GPIOx GPIO Port
  * @param  PinMask This parameter can be a combination of the following values:
  *         @arg @ref DDL_GPIO_PIN_0
  *         @arg @ref DDL_GPIO_PIN_1
  *         @arg @ref DDL_GPIO_PIN_2
  *         @arg @ref DDL_GPIO_PIN_3
  *         @arg @ref DDL_GPIO_PIN_4
  *         @arg @ref DDL_GPIO_PIN_5
  *         @arg @ref DDL_GPIO_PIN_6
  *         @arg @ref DDL_GPIO_PIN_7
  *         @arg @ref DDL_GPIO_PIN_8
  *         @arg @ref DDL_GPIO_PIN_9
  *         @arg @ref DDL_GPIO_PIN_10
  *         @arg @ref DDL_GPIO_PIN_11
  *         @arg @ref DDL_GPIO_PIN_12
  *         @arg @ref DDL_GPIO_PIN_13
  *         @arg @ref DDL_GPIO_PIN_14
  *         @arg @ref DDL_GPIO_PIN_15
  *         @arg @ref DDL_GPIO_PIN_ALL
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_TogglePin(GPIO_TypeDef *GPIOx, uint32_t PinMask)
{
  uint32_t odr = READ_REG(GPIOx->OUTDR);
  WRITE_REG(GPIOx->BSRR, ((odr & PinMask) << 16u) | (~odr & PinMask));
}

/**
  * @}
  */

#if defined(USE_FULL_DDL_DRIVER)
/** @defgroup GPIO_DDL_EF_Init Initialization and de-initialization functions
  * @{
  */

ErrorStatus DDL_GPIO_DeInit(GPIO_TypeDef *GPIOx);
ErrorStatus DDL_GPIO_Init(GPIO_TypeDef *GPIOx, DDL_GPIO_InitTypeDef *GPIO_InitStruct);
void        DDL_GPIO_StructInit(DDL_GPIO_InitTypeDef *GPIO_InitStruct);

/**
  * @}
  */
#endif /* USE_FULL_DDL_DRIVER */

/**
  * @}
  */

/**
  * @}
  */

#endif /* defined (GPIOA) || defined (GPIOB) */
/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* G32F031_DDL_GPIO_H */

