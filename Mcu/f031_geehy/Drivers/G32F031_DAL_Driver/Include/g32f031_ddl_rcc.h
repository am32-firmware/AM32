/**
  *
  * @file    g32f031_ddl_rcc.h
  * @brief   Header file of RCC DDL module.
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
  * This software is licensed under terms that can be found in the LICENSE file in
  * the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef G32F031_DDL_RCC_H
#define G32F031_DDL_RCC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "g32f0xx.h"

/** @addtogroup G32F031_DDL_Driver
  * @{
  */

#if defined(RCC)

/** @defgroup RCC_DDL RCC
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
#if defined(USE_FULL_DDL_DRIVER)
/** @defgroup RCC_DDL_Exported_Types RCC Exported Types
  * @{
  */

/** @defgroup DDL_ES_CLOCK_FREQ Clocks Frequency Structure
  * @{
  */

/**
  * @brief  RCC Clocks Frequency Structure
  */
typedef struct
{
  uint32_t SYSCLK_Frequency;        /*!< SYSCLK clock frequency */
  uint32_t HCLK_Frequency;          /*!< HCLK clock frequency */
  uint32_t PCLK_Frequency;          /*!< PCLK clock frequency */
} DDL_RCC_ClocksTypeDef;

/**
  * @}
  */

/**
  * @}
  */
#endif /* USE_FULL_DDL_DRIVER */

/* Exported constants --------------------------------------------------------*/
/** @defgroup RCC_DDL_Exported_Constants RCC Exported Constants
  * @{
  */

/** @defgroup RCC_DDL_EC_OSC_VALUES Oscillator Values adaptation
  * @brief    Defines used to adapt values of different oscillators
  * @note     These values could be modified in the user environment according to
  *           HW set-up.
  * @{
  */

#if !defined  (HSI_VALUE)
#define HSI_VALUE    64000000U  /*!< Value of the HSI oscillator in Hz */
#endif /* HSI_VALUE */

#if !defined  (LSI_VALUE)
#define LSI_VALUE    32768U     /*!< Value of the LSI oscillator in Hz */
#endif /* LSI_VALUE */
/**
  * @}
  */

/** @defgroup RCC_DDL_EC_IT_FLAG Interrupt Flags Defines
  * @{
  */
#define DDL_RCC_INT_LSIRDYFLG                RCC_ISR_LSIRDYFLG        /*!< LSI Ready Interrupt flag */
#define DDL_RCC_INT_HSIRDYFLG                RCC_ISR_HSIRDYFLG        /*!< HSI Ready Interrupt flag */
/**
  * @}
  */

/** @defgroup RCC_DDL_EC_IT_EN Interrupt Defines
  * @{
  */
#define DDL_RCC_INT_LSIRDYEN                 RCC_IER_LSIRDYIE       /*!< LSI Ready Interrupt Enable */
#define DDL_RCC_INT_HSIRDYEN                 RCC_IER_HSIRDYIE       /*!< HSI Ready Interrupt Enable */
/**
  * @}
  */

/** @defgroup RCC_DDL_EC_FLT_SEL NRST Filter Select
  * @{
  */
#define DDL_RCC_RSTCSR_DFLT_NONE            0x00000000U                                                 /*!< Digital filter none select */
#define DDL_RCC_RSTCSR_DFLT_1MS             RCC_RSTCSR_NRSTFLTSEL_0                                   /*!< Digital filter 1 ms */
#define DDL_RCC_RSTCSR_DFLT_5MS             RCC_RSTCSR_NRSTFLTSEL_1                                   /*!< Digital filter 5 ms */
#define DDL_RCC_RSTCSR_DFLT_10MS            (RCC_RSTCSR_NRSTFLTSEL_0 | RCC_RSTCSR_NRSTFLTSEL_1)     /*!< Digital filter 10 ms */
/**
  * @}
  */

/** @defgroup RCC_DDL_EC_RESET_FLAG Reset Flags Defines
  * @{
  */
#define DDL_RCC_RSTCSR_OPRSTFLG              RCC_RSTCSR_OPTRSTFLG       /*!< Opload reset flag */
#define DDL_RCC_RSTCSR_PINRSTFLG             RCC_RSTCSR_NRSTRSTFLG     /*!< PIN reset flag */
#define DDL_RCC_RSTCSR_PVDRSTFLG             RCC_RSTCSR_PVDRSTFLG      /*!< PVD reset flag */
#define DDL_RCC_RSTCSR_SFTRSTFLG             RCC_RSTCSR_SWRSTFLG      /*!< Software Reset flag */
#define DDL_RCC_RSTCSR_IWDTRSTFLG            RCC_RSTCSR_IWDTRSTFLG     /*!< Independent Watchdog reset flag */
#define DDL_RCC_RSTCSR_WWDTRSTFLG            RCC_RSTCSR_WWDTRSTFLG     /*!< Window watchdog reset flag */
#define DDL_RCC_RSTCSR_LOCKUPRSTFLG          RCC_RSTCSR_LOCKUPRSTFLG   /*!< Lockup reset flag */
#define DDL_RCC_RSTCSR_PORRSTFLG             RCC_RSTCSR_PORRSTFLG      /*!< Power reset flag */
/**
  * @}
  */

/** @defgroup RCC_DDL_EC_RESET_EN Reset flag enable
  * @{
  */
#define DDL_RCC_RSTCSR_PVDRSTEN              RCC_RSTCSR_PVDRSTEN       /*!< PVD reset enable */
#define DDL_RCC_RSTCSR_LOCKUPRSTEN           RCC_RSTCSR_LOCKUPRSTEN    /*!< Lockup reset enable */
/**
  * @}
  */

/** @defgroup RCC_DDL_EC_SYS_CLKSOURCE  System clock switch
  * @{
  */
#define DDL_RCC_SYS_CLKSOURCE_HSI            RCC_CFG_SW_HSI         /*!< HSI selection as system clock */
#define DDL_RCC_SYS_CLKSOURCE_LSI            RCC_CFG_SW_LSI         /*!< LSI selection as system clock */
/**
  * @}
  */

/** @defgroup RCC_DDL_EC_SYS_CLKSOURCE_STATUS  System clock switch status
  * @{
  */
#define DDL_RCC_SYS_CLKSOURCE_STATUS_HSI    RCC_CFG_SWSTS_HSI   /*!< HSI used as system clock */
#define DDL_RCC_SYS_CLKSOURCE_STATUS_LSI    RCC_CFG_SWSTS_LSI   /*!< LSI used as system clock */
/**
  * @}
  */

/** @defgroup RCC_DDL_EC_SYSCLK_DIV  AHB prescaler
  * @{
  */
#define DDL_RCC_AHB_DIV_1                    RCC_CFG_AHBCLKDIV_DIV1      /*!< SYSCLK not divided */
#define DDL_RCC_AHB_DIV_2                    RCC_CFG_AHBCLKDIV_DIV2      /*!< SYSCLK divided by 2 */
#define DDL_RCC_AHB_DIV_4                    RCC_CFG_AHBCLKDIV_DIV4      /*!< SYSCLK divided by 4 */
#define DDL_RCC_AHB_DIV_8                    RCC_CFG_AHBCLKDIV_DIV8      /*!< SYSCLK divided by 8 */
#define DDL_RCC_AHB_DIV_16                   RCC_CFG_AHBCLKDIV_DIV16     /*!< SYSCLK divided by 16 */
#define DDL_RCC_AHB_DIV_32                   RCC_CFG_AHBCLKDIV_DIV32     /*!< SYSCLK divided by 64 */
#define DDL_RCC_AHB_DIV_64                   RCC_CFG_AHBCLKDIV_DIV64     /*!< SYSCLK divided by 128 */
#define DDL_RCC_AHB_DIV_128                  RCC_CFG_AHBCLKDIV_DIV128    /*!< SYSCLK divided by 256 */
/**
  * @}
  */

/** @defgroup RCC_DDL_EC_APB_DIV  APB low-speed prescaler (APB)
  * @{
  */
#define DDL_RCC_APB_DIV_1                    RCC_CFG_APBCLKDIV_DIV1      /*!< HCLK not divided */
#define DDL_RCC_APB_DIV_2                    RCC_CFG_APBCLKDIV_DIV2      /*!< HCLK divided by 2 */
#define DDL_RCC_APB_DIV_4                    RCC_CFG_APBCLKDIV_DIV4      /*!< HCLK divided by 4 */
#define DDL_RCC_APB_DIV_8                    RCC_CFG_APBCLKDIV_DIV8      /*!< HCLK divided by 8 */
/**
  * @}
  */

/** @defgroup RCC_DDL_EC_HSI_DIV  HSI DIV
  * @{
  */
#define DDL_RCC_HSI_DIV_1                    RCC_CFG_HSIDIV_DIV1      /*!< HSI not divided */
#define DDL_RCC_HSI_DIV_2                    RCC_CFG_HSIDIV_DIV2      /*!< HSI divided by 2 */
#define DDL_RCC_HSI_DIV_4                    RCC_CFG_HSIDIV_DIV4      /*!< HSI divided by 4 */
#define DDL_RCC_HSI_DIV_8                    RCC_CFG_HSIDIV_DIV8      /*!< HSI divided by 8 */
/**
  * @}
  */

/** @defgroup RCC_DDL_EC_AHB_RESET AHB reset
  * @{
  */
#define DDL_RCC_AHB_RESET_GPIOA              RCC_AHBRST_GPIOARST
#define DDL_RCC_AHB_RESET_GPIOB              RCC_AHBRST_GPIOBRST
#define DDL_RCC_AHB_RESET_DMA                RCC_AHBRST_DMARST
#define DDL_RCC_AHB_RESET_CRC                RCC_AHBRST_CRCRST
#define DDL_RCC_AHB_RESET_DIV                RCC_AHBRST_DIVRST
/**
  * @}
  */

/** @defgroup RCC_DDL_EC_APB_RESET APB reset
  * @{
  */
#define DDL_RCC_APB_RESET_EINT               RCC_APBRST_EINTRST
#define DDL_RCC_APB_RESET_ATMR               RCC_APBRST_ATMRRST
#define DDL_RCC_APB_RESET_GTMR               RCC_APBRST_GTMRRST
#define DDL_RCC_APB_RESET_BTMR0              RCC_APBRST_BTMR0RST
#define DDL_RCC_APB_RESET_BTMR1              RCC_APBRST_BTMR1RST
#define DDL_RCC_APB_RESET_WWDT               RCC_APBRST_WWDTRST
#define DDL_RCC_APB_RESET_SPI                RCC_APBRST_SPIRST
#define DDL_RCC_APB_RESET_USART              RCC_APBRST_USARTRST
#define DDL_RCC_APB_RESET_UART               RCC_APBRST_UARTRST
#define DDL_RCC_APB_RESET_I2C                RCC_APBRST_I2CRST
#define DDL_RCC_APB_RESET_ADC                RCC_APBRST_ADCRST
#define DDL_RCC_APB_RESET_OPA                RCC_APBRST_OPARST
#define DDL_RCC_APB_RESET_COMP0              RCC_APBRST_COMP0RST
#define DDL_RCC_APB_RESET_COMP1              RCC_APBRST_COMP1RST
/**
  * @}
  */

/** @defgroup RCC_DDL_EC_AON_RESET AON reset
  * @{
  */
#define DDL_RCC_AON_RESET_LPTMR              RCC_AONCSR_LPTMRRST
/**
  * @}
  */

/** @defgroup RCC_DDL_EC_AON_ENABLE AON enable
  * @{
  */
#define DDL_RCC_AON_ENABLE_LPTMR             RCC_AONCSR_LPTMREN
/**
  * @}
  */

/** @defgroup RCC_DDL_EC_AHB_Peripheral AHB peripheral
  * @{
  */
#define DDL_RCC_AHB_PERIPHERAL_GPIOA         RCC_AHBCLKEN_GPIOAEN
#define DDL_RCC_AHB_PERIPHERAL_GPIOB         RCC_AHBCLKEN_GPIOBEN
#define DDL_RCC_AHB_PERIPHERAL_DMA           RCC_AHBCLKEN_DMAEN
#define DDL_RCC_AHB_PERIPHERAL_CRC           RCC_AHBCLKEN_CRCEN
#define DDL_RCC_AHB_PERIPHERAL_DIV           RCC_AHBCLKEN_DIVEN
/**
  * @}
  */

/** @defgroup RCC_DDL_EC_APB_PERIPHERAL APB peripheral
  * @{
  */
#define DDL_RCC_APB_PERIPHERAL_EINT          RCC_APBCLKEN_EINTEN
#define DDL_RCC_APB_PERIPHERAL_ATMR          RCC_APBCLKEN_ATMREN
#define DDL_RCC_APB_PERIPHERAL_GTMR          RCC_APBCLKEN_GTMREN
#define DDL_RCC_APB_PERIPHERAL_BTMR0         RCC_APBCLKEN_BTMR0EN
#define DDL_RCC_APB_PERIPHERAL_BTMR1         RCC_APBCLKEN_BTMR1EN
#define DDL_RCC_APB_PERIPHERAL_IWDT          RCC_APBCLKEN_IWDTEN
#define DDL_RCC_APB_PERIPHERAL_WWDT          RCC_APBCLKEN_WWDTEN
#define DDL_RCC_APB_PERIPHERAL_SPI           RCC_APBCLKEN_SPIEN
#define DDL_RCC_APB_PERIPHERAL_USART         RCC_APBCLKEN_USARTEN
#define DDL_RCC_APB_PERIPHERAL_UART          RCC_APBCLKEN_UARTEN
#define DDL_RCC_APB_PERIPHERAL_I2C           RCC_APBCLKEN_I2CEN
#define DDL_RCC_APB_PERIPHERAL_ADC           RCC_APBCLKEN_ADCEN
#define DDL_RCC_APB_PERIPHERAL_OPA           RCC_APBCLKEN_OPAEN
#define DDL_RCC_APB_PERIPHERAL_COMP0         RCC_APBCLKEN_COMP0EN
#define DDL_RCC_APB_PERIPHERAL_COMP1         RCC_APBCLKEN_COMP1EN
/**
  * @}
  */

/** @defgroup RCC_DDL_EC_AON_PERIPHERAL AON peripheral
  * @{
  */
#define DDL_RCC_AON_PERIPHERAL_LPTMR         RCC_AONCSR_LPTMREN
/**
  * @}
  */

/** @defgroup RCC_DDL_EC_MCOSOURCE MCO source selection
  * @{
  */
#define DDL_RCC_MCOSOURCE_SYSCLK             RCC_CFG_CLKOUTSEL_SYSCLK /*!< SYSCLK selection as MCO source */
#define DDL_RCC_MCOSOURCE_HSI                RCC_CFG_CLKOUTSEL_HSI    /*!< HSI selection as MCO source */
#define DDL_RCC_MCOSOURCE_LSI                RCC_CFG_CLKOUTSEL_LSI    /*!< LSI selection as MCO source */
#define DDL_RCC_MCOSOURCE_HCLK               RCC_CFG_CLKOUTSEL_HCLK   /*!< HCLK selection as MCO source */
/**
  * @}
  */

/** @defgroup RCC_DDL_EC_MCO_DIV  MCO prescaler
  * @{
  */
#define DDL_RCC_MCO_DIV_1                    RCC_CFG_CLKOUTDIV_DIV1  /*!< MCO not divided */
#define DDL_RCC_MCO_DIV_2                    RCC_CFG_CLKOUTDIV_DIV2  /*!< MCO divided by 2 */
#define DDL_RCC_MCO_DIV_4                    RCC_CFG_CLKOUTDIV_DIV4  /*!< MCO divided by 4 */
#define DDL_RCC_MCO_DIV_8                    RCC_CFG_CLKOUTDIV_DIV8  /*!< MCO divided by 8 */
#define DDL_RCC_MCO_DIV_16                   RCC_CFG_CLKOUTDIV_DIV16 /*!< MCO divided by 16 */
#define DDL_RCC_MCO_DIV_32                   RCC_CFG_CLKOUTDIV_DIV32 /*!< MCO divided by 32 */
#define DDL_RCC_MCO_DIV_64                   RCC_CFG_CLKOUTDIV_DIV64 /*!< MCO divided by 64 */
#define DDL_RCC_MCO_DIV_128                  RCC_CFG_CLKOUTDIV_DIV128 /*!< MCO divided by 128 */
/**
  * @}
  */

/** @defgroup RCC_DDL_EC_ADCCLK_Division  ADCCLK Division
  * @{
  */
#define DDL_RCC_ADCCLK_DIVISION_2            0x00000000U              /*!< ADCCLK Division with 2 */
#define DDL_RCC_ADCCLK_DIVISION_4            RCC_ADCCR_ADCCLKDIV_0   /*!< ADCCLK Division with 4 */
#define DDL_RCC_ADCCLK_DIVISION_8            RCC_ADCCR_ADCCLKDIV_1   /*!< ADCCLK Division with 8 */
#define DDL_RCC_ADCCLK_DIVISION_16           RCC_ADCCR_ADCCLKDIV     /*!< ADCCLK Division with 16 */
/**
  * @}
  */

#if defined(USE_FULL_DDL_DRIVER)
/** @defgroup RCC_DDL_EC_PERIPH_FREQUENCY Peripheral clock frequency
  * @{
  */
#define DDL_RCC_PERIPH_FREQUENCY_NO          0x00000000U                 /*!< No clock enabled for the peripheral            */
#define DDL_RCC_PERIPH_FREQUENCY_NA          0xFFFFFFFFU                 /*!< Frequency cannot be provided as external clock */
/**
  * @}
  */
#endif /* USE_FULL_DDL_DRIVER */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup RCC_DDL_Exported_Macros RCC Exported Macros
  * @{
  */

/** @defgroup RCC_DDL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in RCC register
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define DDL_RCC_WriteReg(__REG__, __VALUE__) WRITE_REG(RCC->__REG__, (__VALUE__))

/**
  * @brief  Read a value in RCC register
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define DDL_RCC_ReadReg(__REG__) READ_REG(RCC->__REG__)
/**
  * @}
  */

/** @defgroup RCC_DDL_EM_CALC_FREQ Calculate frequencies
  * @{
  */

/**
  * @brief  Helper macro to calculate the SYSCLK frequency
  * @param  __HSICLKFREQ__ HSI frequency
  * @param  __HSIPRESCALER__ This parameter can be one of the following values:
  *         @arg @ref DDL_RCC_HSI_DIV_1
  *         @arg @ref DDL_RCC_HSI_DIV_2
  *         @arg @ref DDL_RCC_HSI_DIV_4
  *         @arg @ref DDL_RCC_HSI_DIV_8
  * @retval HCLK clock frequency (in Hz)
  */
#define __DDL_RCC_CALC_SYSCLK_FREQ(__HSICLKFREQ__, __HSIPRESCALER__) ((__HSICLKFREQ__) >> HSIPrescTable[((__HSIPRESCALER__) & RCC_CFG_HSIDIV) >>  RCC_CFG_HSIDIV_Pos])

/**
  * @brief  Helper macro to calculate the HCLK frequency
  * @param  __SYSCLKFREQ__ SYSCLK frequency (based on HSI/lSI)
  * @param  __AHBPRESCALER__ This parameter can be one of the following values:
  *         @arg @ref DDL_RCC_AHB_DIV_1
  *         @arg @ref DDL_RCC_AHB_DIV_2
  *         @arg @ref DDL_RCC_AHB_DIV_4
  *         @arg @ref DDL_RCC_AHB_DIV_8
  *         @arg @ref DDL_RCC_AHB_DIV_16
  *         @arg @ref DDL_RCC_AHB_DIV_32
  *         @arg @ref DDL_RCC_AHB_DIV_64
  *         @arg @ref DDL_RCC_AHB_DIV_128
  * @retval HCLK clock frequency (in Hz)
  */
#define __DDL_RCC_CALC_HCLK_FREQ(__SYSCLKFREQ__, __AHBPRESCALER__) ((__SYSCLKFREQ__) >> AHBPrescTable[((__AHBPRESCALER__) & RCC_CFG_AHBCLKDIV) >>  RCC_CFG_AHBCLKDIV_Pos])

/**
  * @brief  Helper macro to calculate the PCLK frequency (ABP)
  * @param  __HCLKFREQ__ HCLK frequency
  * @param  __APBPRESCALER__ This parameter can be one of the following values:
  *         @arg @ref DDL_RCC_APB_DIV_1
  *         @arg @ref DDL_RCC_APB_DIV_2
  *         @arg @ref DDL_RCC_APB_DIV_4
  *         @arg @ref DDL_RCC_APB_DIV_8
  * @retval PCLK1 clock frequency (in Hz)
  */
#define __DDL_RCC_CALC_PCLK_FREQ(__HCLKFREQ__, __APBPRESCALER__) ((__HCLKFREQ__) >> APBPrescTable[(__APBPRESCALER__) >>  RCC_CFG_APBCLKDIV_Pos])

/**
  * @}
  */

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup RCC_DDL_Exported_Functions RCC Exported Functions
  * @{
  */

/** @defgroup RCC_DDL_EF_Lock Lock
  * @{
  */

/**
  * @brief  Lock the RCC register.
  * @retval None
  */
__STATIC_INLINE void DDL_RCC_Lock(void)
{
  SET_BIT(RCC->KEY, RCC_KEY_LOCKKEY);
}

/**
  * @brief  Unlock the RCC register.
  * @retval None
  */
__STATIC_INLINE void DDL_RCC_Unlock(void)
{
  WRITE_REG(RCC->KEY, (0xFFFFU & RCC_KEY_VALUE));
}

/**
  * @brief Check if the RCC KEY status is locked.
  * @retval None
  */
__STATIC_INLINE uint32_t DDL_RCC_IsActiveFlag_LOCK(void)
{
  return (READ_BIT(RCC->KEY, RCC_KEY_KEYST) != (RCC_KEY_KEYST));
}

/**
  * @}
  */

/** @defgroup RCC_DDL_EF_HSI HSI
  * @{
  */

/**
  * @brief  Enable HSI oscillator
  * @retval None
  */
__STATIC_INLINE void DDL_RCC_HSI_Enable(void)
{
  SET_BIT(RCC->CR, RCC_CR_HSIEN);
}

/**
  * @brief  Disable HSI oscillator
  * @retval None
  */
__STATIC_INLINE void DDL_RCC_HSI_Disable(void)
{
  CLEAR_BIT(RCC->CR, RCC_CR_HSIEN);
}

/**
  * @brief  Check if HSI clock is ready
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RCC_HSI_IsReady(void)
{
  return (uint32_t)(READ_BIT(RCC->CR, RCC_CR_HSIRDY) == (RCC_CR_HSIRDY));
}

/**
  * @}
  */

/** @defgroup RCC_DDL_EF_LSI LSI
  * @{
  */

/**
  * @brief  Enable LSI oscillator
  * @retval None
  */
__STATIC_INLINE void DDL_RCC_LSI_Enable(void)
{
  SET_BIT(RCC->AONCSR, RCC_AONCSR_LSIEN);
}

/**
  * @brief  Disable LSI oscillator
  * @retval None
  */
__STATIC_INLINE void DDL_RCC_LSI_Disable(void)
{
  CLEAR_BIT(RCC->AONCSR, RCC_AONCSR_LSIEN);
}

/**
  * @brief  Check if LSI clock is ready
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RCC_LSI_IsReady(void)
{
  return (uint32_t)(READ_BIT(RCC->AONCSR, RCC_AONCSR_LSIRDY) == (RCC_AONCSR_LSIRDY));
}

/**
  * @}
  */


/** @defgroup RCC_DDL_EF_System_Clock System Clock
  * @{
  */
/**
  * @brief  Configure System clock source
  * @param Source this parameter can be one of the following values:
  *        @arg @ref DDL_RCC_SYS_CLKSOURCE_HSI
  *        @arg @ref DDL_RCC_SYS_CLKSOURCE_LSI
  * @retval None
  */
__STATIC_INLINE void DDL_RCC_SetSysClkSource(uint32_t Source)
{
  MODIFY_REG(RCC->CFG, RCC_CFG_SWSEL, Source);
}

/**
  * @brief  Get System clock source
  * @retval return values can be one of the following values:
  *        @arg @ref DDL_RCC_SYS_CLKSOURCE_STATUS_HSI
  *        @arg @ref DDL_RCC_SYS_CLKSOURCE_STATUS_LSI
  */
__STATIC_INLINE uint32_t DDL_RCC_GetSysClkSource(void)
{
  return (uint32_t)(READ_BIT(RCC->CFG, RCC_CFG_SWSTS) >> RCC_CFG_SWSTS_Pos);
}

/**
  * @}
  */

/** @defgroup RCC_DDL_EF_Prescaler Prescaler
  * @{
  */

/**
  * @brief  Set HIS prescaler
  * @param  Prescaler This parameter can be one of the following values:
  *         @arg @ref DDL_RCC_HSI_DIV_1
  *         @arg @ref DDL_RCC_HSI_DIV_2
  *         @arg @ref DDL_RCC_HSI_DIV_4
  *         @arg @ref DDL_RCC_HSI_DIV_8
  * @retval None
  */
__STATIC_INLINE void DDL_RCC_SetHSIPrescaler(uint32_t Prescaler)
{
  MODIFY_REG(RCC->CFG, RCC_CFG_HSIDIV, Prescaler);
}

/**
  * @brief  Get HSI prescaler
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_RCC_HSI_DIV_1
  *         @arg @ref DDL_RCC_HSI_DIV_2
  *         @arg @ref DDL_RCC_HSI_DIV_4
  *         @arg @ref DDL_RCC_HSI_DIV_8
  */
__STATIC_INLINE uint32_t DDL_RCC_GetHSIPrescaler(void)
{
  return (uint32_t)(READ_BIT(RCC->CFG, RCC_CFG_HSIDIV));
}

/**
  * @brief  Set AHB prescaler
  * @param  Prescaler This parameter can be one of the following values:
  *         @arg @ref DDL_RCC_AHB_DIV_1
  *         @arg @ref DDL_RCC_AHB_DIV_2
  *         @arg @ref DDL_RCC_AHB_DIV_4
  *         @arg @ref DDL_RCC_AHB_DIV_8
  *         @arg @ref DDL_RCC_AHB_DIV_16
  *         @arg @ref DDL_RCC_AHB_DIV_32
  *         @arg @ref DDL_RCC_AHB_DIV_64
  *         @arg @ref DDL_RCC_AHB_DIV_128
  * @retval None
  */
__STATIC_INLINE void DDL_RCC_SetAHBPrescaler(uint32_t Prescaler)
{
  MODIFY_REG(RCC->CFG, RCC_CFG_AHBCLKDIV, Prescaler);
}

/**
  * @brief  Get AHB prescaler
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_RCC_AHB_DIV_1
  *         @arg @ref DDL_RCC_AHB_DIV_2
  *         @arg @ref DDL_RCC_AHB_DIV_4
  *         @arg @ref DDL_RCC_AHB_DIV_8
  *         @arg @ref DDL_RCC_AHB_DIV_16
  *         @arg @ref DDL_RCC_AHB_DIV_32
  *         @arg @ref DDL_RCC_AHB_DIV_64
  *         @arg @ref DDL_RCC_AHB_DIV_128
  */
__STATIC_INLINE uint32_t DDL_RCC_GetAHBPrescaler(void)
{
  return (uint32_t)(READ_BIT(RCC->CFG, RCC_CFG_AHBCLKDIV));
}

/**
  * @brief  Set APB prescaler
  * @param  Prescaler This parameter can be one of the following values:
  *         @arg @ref DDL_RCC_APB_DIV_1
  *         @arg @ref DDL_RCC_APB_DIV_2
  *         @arg @ref DDL_RCC_APB_DIV_4
  *         @arg @ref DDL_RCC_APB_DIV_8
  * @retval None
  */
__STATIC_INLINE void DDL_RCC_SetAPBPrescaler(uint32_t Prescaler)
{
  MODIFY_REG(RCC->CFG, RCC_CFG_APBCLKDIV, Prescaler);
}

/**
  * @brief  Get APB prescaler
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_RCC_APB_DIV_1
  *         @arg @ref DDL_RCC_APB_DIV_2
  *         @arg @ref DDL_RCC_APB_DIV_4
  *         @arg @ref DDL_RCC_APB_DIV_8
  */
__STATIC_INLINE uint32_t DDL_RCC_GetAPBPrescaler(void)
{
  return (uint32_t)(READ_BIT(RCC->CFG, RCC_CFG_APBCLKDIV));
}

/**
  * @}
  */

/** @defgroup RCC_DDL_EF_MCO MCO
  * @{
  */
/**
  * @brief  Enable clock output
  * @retval None
  */
__STATIC_INLINE void DDL_RCC_EnableMCO(void)
{
  SET_BIT(RCC->CFG, RCC_CFG_CLKOUTEN);
}

/**
  * @brief  Disable clock output
  * @retval None
  */
__STATIC_INLINE void DDL_RCC_DisableMCO(void)
{
  CLEAR_BIT(RCC->CFG, RCC_CFG_CLKOUTEN);
}

/**
  * @brief  Configure MCOx
  * @param  Source This parameter can be one of the following values:
  *         @arg @ref DDL_RCC_MCOSOURCE_SYSCLK
  *         @arg @ref DDL_RCC_MCOSOURCE_HSI
  *         @arg @ref DDL_RCC_MCOSOURCE_LSI
  *         @arg @ref DDL_RCC_MCOSOURCE_HCLK
  * @param  Prescaler This parameter can be one of the following values:
  *         @arg @ref DDL_RCC_MCO_DIV_1
  *         @arg @ref DDL_RCC_MCO_DIV_2
  *         @arg @ref DDL_RCC_MCO_DIV_4
  *         @arg @ref DDL_RCC_MCO_DIV_8
  *         @arg @ref DDL_RCC_MCO_DIV_16
  *         @arg @ref DDL_RCC_MCO_DIV_32
  *         @arg @ref DDL_RCC_MCO_DIV_64
  *         @arg @ref DDL_RCC_MCO_DIV_128
  * @retval None
  */
__STATIC_INLINE void DDL_RCC_ConfigMCO(uint32_t Source, uint32_t Prescaler)
{
  MODIFY_REG(RCC->CFG, (RCC_CFG_CLKOUTSEL | RCC_CFG_CLKOUTDIV), (Source | Prescaler));
}
/**
  * @}
  */

/** @defgroup RCC_DDL_EF_Peripheral_Reset Reset Peripheral Module
  * @{
  */

/**
  * @brief  Force reset the AON peripheral module.
  * @param  Peripheral This parameter can be one of the following values:
  *         @arg @ref DDL_RCC_AON_RESET_LPTMR
  * @retval None
  */
__STATIC_INLINE void DDL_RCC_ForceResetAONPeripheral(uint32_t Peripheral)
{
  SET_BIT(RCC->AONCSR, Peripheral);
}

/**
  * @brief  Release Reset the AON peripheral module.
  * @param  Peripheral This parameter can be one of the following values:
  *         @arg @ref DDL_RCC_AON_RESET_LPTMR
  * @retval None
  */
__STATIC_INLINE void DDL_RCC_ReleaseResetAONPeripheral(uint32_t Peripheral)
{
  CLEAR_BIT(RCC->AONCSR, Peripheral);
}

/**
  * @}
  */

/** @defgroup RCC_DDL_EF_Peripheral_Clock Enable Peripheral Clock
  * @{
  */

/**
  * @brief  Enable the AON peripheral Clock.
  * @param  Peripheral This parameter can be one of the following values:
  *         @arg @ref DDL_RCC_AON_ENABLE_LPTMR
  * @retval None
  */
__STATIC_INLINE void DDL_RCC_EnableAONPeripheralClock(uint32_t Peripheral)
{
  SET_BIT(RCC->AONCSR, Peripheral);
}

/**
  * @brief  Disable the AON peripheral Clock.
  * @param  Peripheral This parameter can be one of the following values:
  *         @arg @ref DDL_RCC_AON_ENABLE_LPTMR
  * @retval None
  */
__STATIC_INLINE void DDL_RCC_DisableAONPeripheralClock(uint32_t Peripheral)
{
  CLEAR_BIT(RCC->AONCSR, Peripheral);
}

/**
  * @brief  Check if AON peripheral clock is enabled or not
  * @param  Peripheral This parameter can be a combination of the following values:
  *         @arg @ref DDL_RCC_AON_ENABLE_LPTMR
  * @retval State of Peripheral clock (1 or 0).
*/
__STATIC_INLINE uint32_t DDL_RCC_AONP_IsEnabledClock(uint32_t Peripheral)
{
  return (READ_BIT(RCC->AONCSR, Peripheral) == Peripheral);
}

/**
  * @}
  */

/** @defgroup RCC_DDL_EF_NRSTFLT_Select NRST Filter select
  * @{
  */

/**
  * @brief  Set NRST Filter select
  * @param  FilterSelect This parameter can be one of the following values:
  *         @arg @ref DDL_RCC_RSTCSR_DFLT_NONE
  *         @arg @ref DDL_RCC_RSTCSR_DFLT_1MS
  *         @arg @ref DDL_RCC_RSTCSR_DFLT_5MS
  *         @arg @ref DDL_RCC_RSTCSR_DFLT_10MS
  * @retval None
  */
__STATIC_INLINE void DDL_RCC_SetNRSTFilter(uint32_t Filter)
{
  MODIFY_REG(RCC->RSTCSR, RCC_RSTCSR_NRSTFLTSEL, Filter);
}

/**
  * @brief  Get NRST Filter select
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_RCC_RSTCSR_DFLT_NONE
  *         @arg @ref DDL_RCC_RSTCSR_DFLT_1MS
  *         @arg @ref DDL_RCC_RSTCSR_DFLT_5MS
  *         @arg @ref DDL_RCC_RSTCSR_DFLT_10MS
  */
__STATIC_INLINE uint32_t DDL_RCC_GetNRSTFilter(void)
{
  return (uint32_t)(READ_BIT(RCC->RSTCSR, RCC_RSTCSR_NRSTFLTSEL));
}

/**
  * @}
  */

/** @defgroup RCC_DDL_EF_IT_SOURCE_Control IT SOURCE Control
  * @{
  */
/**
  * @brief  Enable LSI ready interrupt
  * @retval None
  */
__STATIC_INLINE void DDL_RCC_EnableIT_LSIRDY(void)
{
  SET_BIT(RCC->IER, DDL_RCC_INT_LSIRDYEN);
}

/**
  * @brief  Disable LSI ready interrupt
  * @retval None
  */
__STATIC_INLINE void DDL_RCC_DisableIT_LSIRDY(void)
{
  CLEAR_BIT(RCC->IER, DDL_RCC_INT_LSIRDYEN);
}

/**
  * @brief  Enable HSI ready interrupt
  * @retval None
  */
__STATIC_INLINE void DDL_RCC_EnableIT_HSIRDY(void)
{
  SET_BIT(RCC->IER, DDL_RCC_INT_HSIRDYEN);
}

/**
  * @brief  Disable HSI ready interrupt
  * @retval None
  */
__STATIC_INLINE void DDL_RCC_DisableIT_HSIRDY(void)
{
  CLEAR_BIT(RCC->IER, DDL_RCC_INT_HSIRDYEN);
}

/**
  * @brief  Checks if LSI ready interrupt source is enabled or disabled.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RCC_IsEnabledIT_LSIRDY(void)
{
  return (READ_BIT(RCC->IER, DDL_RCC_INT_LSIRDYEN) == (DDL_RCC_INT_LSIRDYEN));
}

/**
  * @brief  Checks if HSI ready interrupt source is enabled or disabled.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RCC_IsEnabledIT_HSIRDY(void)
{
  return (READ_BIT(RCC->IER, DDL_RCC_INT_HSIRDYEN) == (DDL_RCC_INT_HSIRDYEN));
}


/** @defgroup RCC_DDL_EF_IT_FLAG_Control IT FLAG Control
  * @{
  */
/**
  * @brief  Clear LSI ready interrupt flag
  * @retval None
  */
__STATIC_INLINE void DDL_RCC_ClearFlag_LSIRDY(void)
{
  CLEAR_BIT(RCC->ISR, DDL_RCC_INT_LSIRDYFLG);
}

/**
  * @brief  Clear HSI ready interrupt flag
  * @retval None
  */
__STATIC_INLINE void DDL_RCC_ClearFlag_HSIRDY(void)
{
  CLEAR_BIT(RCC->ISR, DDL_RCC_INT_HSIRDYFLG);
}

/**
  * @brief  Check if LSI ready interrupt occurred or not
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RCC_IsActiveFlag_LSIRDY(void)
{
  return (READ_BIT(RCC->ISR, DDL_RCC_INT_LSIRDYFLG) == (DDL_RCC_INT_LSIRDYFLG));
}

/**
  * @brief  Check if HSI ready interrupt occurred or not
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RCC_IsActiveFlag_HSIRDY(void)
{
  return (READ_BIT(RCC->ISR, DDL_RCC_INT_HSIRDYFLG) == (DDL_RCC_INT_HSIRDYFLG));
}

/**
  * @}
  */


/**
  * @}
  */

/** @defgroup RCC_DDL_EF_RESET_SOURCE_Control Reset SOURCE Control
  * @{
  */
/**
  * @brief  Enable PVD reset
  * @retval None
  */
__STATIC_INLINE void DDL_RCC_Enable_PVDRST(void)
{
  SET_BIT(RCC->RSTCSR, RCC_RSTCSR_PVDRSTEN);
}

/**
  * @brief  Disable PVD reset
  * @retval None
  */
__STATIC_INLINE void DDL_RCC_Disable_PVDRST(void)
{
  CLEAR_BIT(RCC->RSTCSR, RCC_RSTCSR_PVDRSTEN);
}

/**
  * @brief  Checks if PVD reset source is enabled or disabled.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RCC_IsEnabled_PVDRST(void)
{
  return (READ_BIT(RCC->RSTCSR, RCC_RSTCSR_PVDRSTEN) == (RCC_RSTCSR_PVDRSTEN));
}

/**
  * @brief  Enable LOCKUP reset
  * @retval None
  */
__STATIC_INLINE void DDL_RCC_Enable_LOCKUPRST(void)
{
  SET_BIT(RCC->RSTCSR, RCC_RSTCSR_LOCKUPRSTEN);
}

/**
  * @brief  Disable LOCKUP reset
  * @retval None
  */
__STATIC_INLINE void DDL_RCC_Disable_LOCKUPRST(void)
{
  CLEAR_BIT(RCC->RSTCSR, RCC_RSTCSR_LOCKUPRSTEN);
}

/**
  * @brief  Checks if LOCKUP reset source is enabled or disabled.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RCC_IsEnabled_LOCKUPRST(void)
{
  return (READ_BIT(RCC->RSTCSR, RCC_RSTCSR_LOCKUPRSTEN) == (RCC_RSTCSR_LOCKUPRSTEN));
}

/**
  * @}
  */


/** @defgroup RCC_DDL_EF_RESET_FLAG_Control Reset FLAG Control
  * @{
  */

/**
  * @brief  Clear opload reset flag
  * @retval None
  */
__STATIC_INLINE void DDL_RCC_ClearFlag_OPRST(void)
{
  CLEAR_BIT(RCC->RSTCSR, DDL_RCC_RSTCSR_OPRSTFLG);
}

/**
  * @brief  Clear pin reset flag
  * @retval None
  */
__STATIC_INLINE void DDL_RCC_ClearFlag_PINRST(void)
{
  CLEAR_BIT(RCC->RSTCSR, DDL_RCC_RSTCSR_PINRSTFLG);
}

/**
  * @brief  Clear porrst reset flag
  * @retval None
  */
__STATIC_INLINE void DDL_RCC_ClearFlag_PORRST(void)
{
  CLEAR_BIT(RCC->RSTCSR, DDL_RCC_RSTCSR_PORRSTFLG);
}

/**
  * @brief  Clear software reset flag
  * @retval None
  */
__STATIC_INLINE void DDL_RCC_ClearFlag_SFTRST(void)
{
  CLEAR_BIT(RCC->RSTCSR, DDL_RCC_RSTCSR_SFTRSTFLG);
}

/**
  * @brief  Clear iwdt reset flag
  * @retval None
  */
__STATIC_INLINE void DDL_RCC_ClearFlag_IWDTRST(void)
{
  CLEAR_BIT(RCC->RSTCSR, DDL_RCC_RSTCSR_IWDTRSTFLG);
}

/**
  * @brief  Clear wwdt reset flag
  * @retval None
  */
__STATIC_INLINE void DDL_RCC_ClearFlag_WWDTRST(void)
{
  CLEAR_BIT(RCC->RSTCSR, DDL_RCC_RSTCSR_WWDTRSTFLG);
}

/**
  * @brief  Clear pvd reset flag
  * @retval None
  */
__STATIC_INLINE void DDL_RCC_ClearFlag_PVDRST(void)
{
  CLEAR_BIT(RCC->RSTCSR, DDL_RCC_RSTCSR_PVDRSTFLG);
}

/**
  * @brief  Clear lockup reset flag
  * @retval None
  */
__STATIC_INLINE void DDL_RCC_ClearFlag_LOCKUPRST(void)
{
  CLEAR_BIT(RCC->RSTCSR, DDL_RCC_RSTCSR_LOCKUPRSTFLG);
}


/**
  * @brief  Check if opload reset flag is set or not.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RCC_IsActiveFlag_OPRST(void)
{
  return (READ_BIT(RCC->RSTCSR, DDL_RCC_RSTCSR_OPRSTFLG) == (DDL_RCC_RSTCSR_OPRSTFLG));
}

/**
  * @brief  Check if pin reset flag is set or not.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RCC_IsActiveFlag_PINRST(void)
{
  return (READ_BIT(RCC->RSTCSR, DDL_RCC_RSTCSR_PINRSTFLG) == (DDL_RCC_RSTCSR_PINRSTFLG));
}

/**
  * @brief  Check if Porrst reset flag is set or not.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RCC_IsActiveFlag_PORRST(void)
{
  return (READ_BIT(RCC->RSTCSR, DDL_RCC_RSTCSR_PORRSTFLG) == (DDL_RCC_RSTCSR_PORRSTFLG));
}

/**
  * @brief  Check if Software reset flag is set or not.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RCC_IsActiveFlag_SFTRST(void)
{
  return (READ_BIT(RCC->RSTCSR, DDL_RCC_RSTCSR_SFTRSTFLG) == (DDL_RCC_RSTCSR_SFTRSTFLG));
}

/**
  * @brief  Check if Independent Watchdog reset flag is set or not.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RCC_IsActiveFlag_IWDTRST(void)
{
  return (READ_BIT(RCC->RSTCSR, DDL_RCC_RSTCSR_IWDTRSTFLG) == (DDL_RCC_RSTCSR_IWDTRSTFLG));
}

/**
  * @brief  Check if Window Watchdog reset flag is set or not.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RCC_IsActiveFlag_WWDTRST(void)
{
  return (READ_BIT(RCC->RSTCSR, DDL_RCC_RSTCSR_WWDTRSTFLG) == (DDL_RCC_RSTCSR_WWDTRSTFLG));
}

/**
  * @brief  Check if PVD reset flag is set or not.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RCC_IsActiveFlag_PVDRST(void)
{
  return (READ_BIT(RCC->RSTCSR, DDL_RCC_RSTCSR_PVDRSTFLG) == (DDL_RCC_RSTCSR_PVDRSTFLG));
}

/**
  * @brief  Check if LOCKUP reset flag is set or not.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RCC_IsActiveFlag_LOCKUPRST(void)
{
  return (READ_BIT(RCC->RSTCSR, DDL_RCC_RSTCSR_LOCKUPRSTFLG) == (DDL_RCC_RSTCSR_LOCKUPRSTFLG));
}

/**
  * @}
  */

/** @defgroup RCC_DDL_EF_ADCCLK_DIV ADC module clock division
  * @{
  */

/**
  * @brief  Set ADC module clock division
  * @param  Div This parameter can be one of the following values:
  *         @arg @ref DDL_RCC_ADCCLK_DIVISION_2
  *         @arg @ref DDL_RCC_ADCCLK_DIVISION_4
  *         @arg @ref DDL_RCC_ADCCLK_DIVISION_8
  *         @arg @ref DDL_RCC_ADCCLK_DIVISION_16
  * @retval None
  */
__STATIC_INLINE void DDL_RCC_SetADCClkDiv(uint32_t Div)
{
  MODIFY_REG(RCC->ADCCR, RCC_ADCCR_ADCCLKDIV, Div);
}

/**
  * @brief  Get ADC module clock division
  * @retval Can be one of the following values:
  *         @arg @ref DDL_RCC_ADCCLK_DIVISION_2
  *         @arg @ref DDL_RCC_ADCCLK_DIVISION_4
  *         @arg @ref DDL_RCC_ADCCLK_DIVISION_8
  *         @arg @ref DDL_RCC_ADCCLK_DIVISION_16
  */
__STATIC_INLINE uint32_t DDL_RCC_GetADCClkDiv(void)
{
  return (READ_BIT(RCC->ADCCR, RCC_ADCCR_ADCCLKDIV));
}

/**
  * @}
  */

#if defined(USE_FULL_DDL_DRIVER)
/** @defgroup RCC_DDL_EF_Init De-initialization function
  * @{
  */
ErrorStatus DDL_RCC_DeInit(void);
/**
  * @}
  */

/** @defgroup RCC_DDL_EF_Update_CLOCKs_Freq Update system and peripherals clocks frequency functions
  * @{
  */
void DDL_RCC_GetSysctrlClocksFreq(DDL_RCC_ClocksTypeDef *RCC_Clocks);
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

 #endif  /* defined(RCC) */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* G32F031_DDL_RCC_H */
