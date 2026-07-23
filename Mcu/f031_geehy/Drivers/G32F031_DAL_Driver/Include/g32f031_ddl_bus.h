/**
  *
  * @file    g32f031_ddl_bus.h
  * @brief   Header file of BUS DDL module.

  @verbatim
                      ##### RCC Limitations #####
  ==============================================================================
    [..]
      A delay between an RCC peripheral clock enable and the effective peripheral
      enabling should be taken into account in order to manage the peripheral read/write
      from/to registers.
      (+) This delay depends on the peripheral mapping.
        (++) AHB & APB peripherals, 1 dummy read is necessary

    [..]
      Workarounds:
      (#) For AHB & APB peripherals, a dummy read to the peripheral register has been
          inserted in each DDL_{BUS}_GRP{x}_EnableClock() function.

  @endverbatim
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
  * Copyright (c) 2016 STMicroelectronics.
  * Copyright (C) 2025 Geehy Semiconductor.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef G32F031_DDL_BUS_H
#define G32F031_DDL_BUS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "g32f0xx.h"

/** @addtogroup G32F031_DDL_Driver
  * @{
  */

#if defined(RCC)

/** @defgroup BUS_DDL BUS
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/** @defgroup BUS_DDL_Exported_Constants BUS Exported Constants
  * @{
  */

/** @defgroup BUS_DDL_EC_AHB_GRP1_PERIPH  AHB GRP1 PERIPH
  * @{
  */
#define DDL_AHB_GRP1_PERIPH_ALL           0xFFFFFFFFU
#define DDL_AHB_GRP1_PERIPH_GPIOA         RCC_AHBCLKEN_GPIOAEN
#define DDL_AHB_GRP1_PERIPH_GPIOB         RCC_AHBCLKEN_GPIOBEN
#define DDL_AHB_GRP1_PERIPH_DMA           RCC_AHBCLKEN_DMAEN
#define DDL_AHB_GRP1_PERIPH_CRC           RCC_AHBCLKEN_CRCEN
#define DDL_AHB_GRP1_PERIPH_DIV           RCC_AHBCLKEN_DIVEN
/**
  * @}
  */


/** @defgroup BUS_DDL_EC_APB_GRP1_PERIPH  APB GRP1 PERIPH
  * @{
  */
#define DDL_APB_GRP1_PERIPH_ALL           0xFFFFFFFFU
#define DDL_APB_GRP1_PERIPH_EINT          RCC_APBCLKEN_EINTEN
#define DDL_APB_GRP1_PERIPH_ATMR          RCC_APBCLKEN_ATMREN
#define DDL_APB_GRP1_PERIPH_GTMR          RCC_APBCLKEN_GTMREN
#define DDL_APB_GRP1_PERIPH_BTMR0         RCC_APBCLKEN_BTMR0EN
#define DDL_APB_GRP1_PERIPH_BTMR1         RCC_APBCLKEN_BTMR1EN
#define DDL_APB_GRP1_PERIPH_IWDT          RCC_APBCLKEN_IWDTEN
#define DDL_APB_GRP1_PERIPH_WWDT          RCC_APBCLKEN_WWDTEN
#define DDL_APB_GRP1_PERIPH_SPI           RCC_APBCLKEN_SPIEN
#define DDL_APB_GRP1_PERIPH_USART         RCC_APBCLKEN_USARTEN
#define DDL_APB_GRP1_PERIPH_UART          RCC_APBCLKEN_UARTEN
#define DDL_APB_GRP1_PERIPH_I2C           RCC_APBCLKEN_I2CEN
#define DDL_APB_GRP1_PERIPH_ADC           RCC_APBCLKEN_ADCEN
#define DDL_APB_GRP1_PERIPH_OPA           RCC_APBCLKEN_OPAEN
#define DDL_APB_GRP1_PERIPH_COMP0         RCC_APBCLKEN_COMP0EN
#define DDL_APB_GRP1_PERIPH_COMP1         RCC_APBCLKEN_COMP1EN

/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/** @defgroup BUS_DDL_Exported_Functions BUS Exported Functions
  * @{
  */

/** @defgroup BUS_DDL_EF_AHB AHB
  * @{
  */

/**
  * @brief  Enable AHB peripherals clock.
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref DDL_AHB_GRP1_PERIPH_GPIOA
  *         @arg @ref DDL_AHB_GRP1_PERIPH_GPIOB
  *         @arg @ref DDL_AHB_GRP1_PERIPH_DMA
  *         @arg @ref DDL_AHB_GRP1_PERIPH_CRC
  *         @arg @ref DDL_AHB_GRP1_PERIPH_DIV
  * @retval None
*/
__STATIC_INLINE void DDL_AHB_GRP1_EnableClock(uint32_t Periphs)
{
  __IO uint32_t tmpreg;
  SET_BIT(RCC->AHBCLKEN, Periphs);
  /* Delay after an RCC peripheral clock enabling */
  tmpreg = READ_BIT(RCC->AHBCLKEN, Periphs);
  (void)tmpreg;
}

/**
  * @brief  Check if AHB peripheral clock is enabled or not
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref DDL_AHB_GRP1_PERIPH_GPIOA
  *         @arg @ref DDL_AHB_GRP1_PERIPH_GPIOB
  *         @arg @ref DDL_AHB_GRP1_PERIPH_DMA
  *         @arg @ref DDL_AHB_GRP1_PERIPH_CRC
  *         @arg @ref DDL_AHB_GRP1_PERIPH_DIV
  * @retval State of Periphs (1 or 0).
*/
__STATIC_INLINE uint32_t DDL_AHB_GRP1_IsEnabledClock(uint32_t Periphs)
{
  return (READ_BIT(RCC->AHBCLKEN, Periphs) == Periphs);
}

/**
  * @brief  Disable AHB peripherals clock.
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref DDL_AHB_GRP1_PERIPH_GPIOA
  *         @arg @ref DDL_AHB_GRP1_PERIPH_GPIOB
  *         @arg @ref DDL_AHB_GRP1_PERIPH_DMA
  *         @arg @ref DDL_AHB_GRP1_PERIPH_CRC
  *         @arg @ref DDL_AHB_GRP1_PERIPH_DIV
  * @retval None
*/
__STATIC_INLINE void DDL_AHB_GRP1_DisableClock(uint32_t Periphs)
{
  CLEAR_BIT(RCC->AHBCLKEN, Periphs);
}

/**
  * @brief  Force AHB peripherals reset.
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref DDL_AHB_GRP1_PERIPH_GPIOA
  *         @arg @ref DDL_AHB_GRP1_PERIPH_GPIOB
  *         @arg @ref DDL_AHB_GRP1_PERIPH_DMA
  *         @arg @ref DDL_AHB_GRP1_PERIPH_CRC
  *         @arg @ref DDL_AHB_GRP1_PERIPH_DIV
  * @retval None
*/
__STATIC_INLINE void DDL_AHB_GRP1_ForceReset(uint32_t Periphs)
{
  SET_BIT(RCC->AHBRST, Periphs);
}

/**
  * @brief  Release AHB peripherals reset.
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref DDL_AHB_GRP1_PERIPH_GPIOA
  *         @arg @ref DDL_AHB_GRP1_PERIPH_GPIOB
  *         @arg @ref DDL_AHB_GRP1_PERIPH_DMA
  *         @arg @ref DDL_AHB_GRP1_PERIPH_CRC
  *         @arg @ref DDL_AHB_GRP1_PERIPH_DIV
  * @retval None
*/
__STATIC_INLINE void DDL_AHB_GRP1_ReleaseReset(uint32_t Periphs)
{
  CLEAR_BIT(RCC->AHBRST, Periphs);
}

/**
  * @}
  */


/** @defgroup BUS_DDL_EF_APB APB
  * @{
  */

/**
  * @brief  Enable APB1 peripherals clock.
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref DDL_APB_GRP1_PERIPH_EINT
  *         @arg @ref DDL_APB_GRP1_PERIPH_ATMR
  *         @arg @ref DDL_APB_GRP1_PERIPH_GTMR
  *         @arg @ref DDL_APB_GRP1_PERIPH_BTMR0
  *         @arg @ref DDL_APB_GRP1_PERIPH_BTMR1
  *         @arg @ref DDL_APB_GRP1_PERIPH_IWDT
  *         @arg @ref DDL_APB_GRP1_PERIPH_WWDT
  *         @arg @ref DDL_APB_GRP1_PERIPH_SPI
  *         @arg @ref DDL_APB_GRP1_PERIPH_USART
  *         @arg @ref DDL_APB_GRP1_PERIPH_UART
  *         @arg @ref DDL_APB_GRP1_PERIPH_I2C
  *         @arg @ref DDL_APB_GRP1_PERIPH_ADC
  *         @arg @ref DDL_APB_GRP1_PERIPH_OPA
  *         @arg @ref DDL_APB_GRP1_PERIPH_COMP0
  *         @arg @ref DDL_APB_GRP1_PERIPH_COMP1
  * @retval None
*/
__STATIC_INLINE void DDL_APB_GRP1_EnableClock(uint32_t Periphs)
{
  __IO uint32_t tmpreg;
  SET_BIT(RCC->APBCLKEN, Periphs);
  /* Delay after an RCC peripheral clock enabling */
  tmpreg = READ_BIT(RCC->APBCLKEN, Periphs);
  (void)tmpreg;
}

/**
  * @brief  Check if APB1 peripheral clock is enabled or not
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref DDL_APB_GRP1_PERIPH_EINT
  *         @arg @ref DDL_APB_GRP1_PERIPH_ATMR
  *         @arg @ref DDL_APB_GRP1_PERIPH_GTMR
  *         @arg @ref DDL_APB_GRP1_PERIPH_BTMR0
  *         @arg @ref DDL_APB_GRP1_PERIPH_BTMR1
  *         @arg @ref DDL_APB_GRP1_PERIPH_IWDT
  *         @arg @ref DDL_APB_GRP1_PERIPH_WWDT
  *         @arg @ref DDL_APB_GRP1_PERIPH_SPI
  *         @arg @ref DDL_APB_GRP1_PERIPH_USART
  *         @arg @ref DDL_APB_GRP1_PERIPH_UART
  *         @arg @ref DDL_APB_GRP1_PERIPH_I2C
  *         @arg @ref DDL_APB_GRP1_PERIPH_ADC
  *         @arg @ref DDL_APB_GRP1_PERIPH_OPA
  *         @arg @ref DDL_APB_GRP1_PERIPH_COMP0
  *         @arg @ref DDL_APB_GRP1_PERIPH_COMP1
  * @retval State of Periphs (1 or 0).
*/
__STATIC_INLINE uint32_t DDL_APB_GRP1_IsEnabledClock(uint32_t Periphs)
{
  return (READ_BIT(RCC->APBCLKEN, Periphs) == Periphs);
}

/**
  * @brief  Disable APB1 peripherals clock.
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref DDL_APB_GRP1_PERIPH_EINT
  *         @arg @ref DDL_APB_GRP1_PERIPH_ATMR
  *         @arg @ref DDL_APB_GRP1_PERIPH_GTMR
  *         @arg @ref DDL_APB_GRP1_PERIPH_BTMR0
  *         @arg @ref DDL_APB_GRP1_PERIPH_BTMR1
  *         @arg @ref DDL_APB_GRP1_PERIPH_IWDT
  *         @arg @ref DDL_APB_GRP1_PERIPH_WWDT
  *         @arg @ref DDL_APB_GRP1_PERIPH_SPI
  *         @arg @ref DDL_APB_GRP1_PERIPH_USART
  *         @arg @ref DDL_APB_GRP1_PERIPH_UART
  *         @arg @ref DDL_APB_GRP1_PERIPH_I2C
  *         @arg @ref DDL_APB_GRP1_PERIPH_ADC
  *         @arg @ref DDL_APB_GRP1_PERIPH_OPA
  *         @arg @ref DDL_APB_GRP1_PERIPH_COMP0
  *         @arg @ref DDL_APB_GRP1_PERIPH_COMP1
  * @retval None
*/
__STATIC_INLINE void DDL_APB_GRP1_DisableClock(uint32_t Periphs)
{
  CLEAR_BIT(RCC->APBCLKEN, Periphs);
}

/**
  * @brief  Force APB1 peripherals reset.
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref DDL_APB_GRP1_PERIPH_EINT
  *         @arg @ref DDL_APB_GRP1_PERIPH_ATMR
  *         @arg @ref DDL_APB_GRP1_PERIPH_GTMR
  *         @arg @ref DDL_APB_GRP1_PERIPH_BTMR0
  *         @arg @ref DDL_APB_GRP1_PERIPH_BTMR1
  *         @arg @ref DDL_APB_GRP1_PERIPH_WWDT
  *         @arg @ref DDL_APB_GRP1_PERIPH_SPI
  *         @arg @ref DDL_APB_GRP1_PERIPH_USART
  *         @arg @ref DDL_APB_GRP1_PERIPH_UART
  *         @arg @ref DDL_APB_GRP1_PERIPH_I2C
  *         @arg @ref DDL_APB_GRP1_PERIPH_ADC
  *         @arg @ref DDL_APB_GRP1_PERIPH_OPA
  *         @arg @ref DDL_APB_GRP1_PERIPH_COMP0
  *         @arg @ref DDL_APB_GRP1_PERIPH_COMP1
  * @retval None
*/
__STATIC_INLINE void DDL_APB_GRP1_ForceReset(uint32_t Periphs)
{
  SET_BIT(RCC->APBRST, Periphs);
}

/**
  * @brief  Release APB1 peripherals reset.
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref DDL_APB_GRP1_PERIPH_EINT
  *         @arg @ref DDL_APB_GRP1_PERIPH_ATMR
  *         @arg @ref DDL_APB_GRP1_PERIPH_GTMR
  *         @arg @ref DDL_APB_GRP1_PERIPH_BTMR0
  *         @arg @ref DDL_APB_GRP1_PERIPH_BTMR1
  *         @arg @ref DDL_APB_GRP1_PERIPH_WWDT
  *         @arg @ref DDL_APB_GRP1_PERIPH_SPI
  *         @arg @ref DDL_APB_GRP1_PERIPH_USART
  *         @arg @ref DDL_APB_GRP1_PERIPH_UART
  *         @arg @ref DDL_APB_GRP1_PERIPH_I2C
  *         @arg @ref DDL_APB_GRP1_PERIPH_ADC
  *         @arg @ref DDL_APB_GRP1_PERIPH_OPA
  *         @arg @ref DDL_APB_GRP1_PERIPH_COMP0
  *         @arg @ref DDL_APB_GRP1_PERIPH_COMP1
  * @retval None
*/
__STATIC_INLINE void DDL_APB_GRP1_ReleaseReset(uint32_t Periphs)
{
  CLEAR_BIT(RCC->APBRST, Periphs);
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

#endif /* defined(RCC) */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* G32F031_DDL_BUS_H */

