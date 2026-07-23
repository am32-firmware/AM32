/**
  *
  * @file    g32f031_ddl_spi.c
  * @brief   SPI DDL module driver.
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
  *
  */
#if defined(USE_FULL_DDL_DRIVER)

/* Includes ------------------------------------------------------------------*/
#include "g32f031_ddl_spi.h"
#include "g32f031_ddl_bus.h"
#include "g32f031_ddl_rcc.h"

#ifdef  USE_FULL_ASSERT
#include "g32_assert.h"
#else
#define ASSERT_PARAM(_PARAM_) ((void)0U)
#endif /* USE_FULL_ASSERT */

/** @addtogroup G32F031_DDL_Driver
  * @{
  */

#if defined (SPI)

/** @addtogroup SPI_DDL SPI
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/** @defgroup SPI_DDL_Private_Constants SPI Private Constants
  * @{
  */
/* SPI registers Masks */
#define SPI_CR1_CLEAR_MASK                  (SPI_CR1_CPHA    | SPI_CR1_CPOL     | SPI_CR1_MSTR | \
                                            SPI_CR1_BR       | SPI_CR1_SPIEN      | SPI_CR1_LSBFIRST | \
                                            SPI_CR1_SSI      | SPI_CR1_SSM      | SPI_CR1_DFF | \
                                            SPI_CR1_BIDIOEN   | SPI_CR1_BIDIMEN )
/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/** @defgroup SPI_DDL_Private_Macros SPI Private Macros
  * @{
  */
#define IS_DDL_SPI_TRANSFER_DIRECTION(__VALUE__) (((__VALUE__) == DDL_SPI_FULL_DUPLEX)      \
                                                 || ((__VALUE__) == DDL_SPI_HALF_DUPLEX_RX) \
                                                 || ((__VALUE__) == DDL_SPI_HALF_DUPLEX_TX))

#define IS_DDL_SPI_MODE(__VALUE__) (((__VALUE__) == DDL_SPI_MODE_MASTER) \
                                   || ((__VALUE__) == DDL_SPI_MODE_SLAVE))

#define IS_DDL_SPI_DATAWIDTH(__VALUE__) (((__VALUE__) == DDL_SPI_DATAWIDTH_8BIT)  \
                                        || ((__VALUE__) == DDL_SPI_DATAWIDTH_16BIT))

#define IS_DDL_SPI_POLARITY(__VALUE__) (((__VALUE__) == DDL_SPI_POLARITY_LOW) \
                                       || ((__VALUE__) == DDL_SPI_POLARITY_HIGH))

#define IS_DDL_SPI_PHASE(__VALUE__) (((__VALUE__) == DDL_SPI_PHASE_1EDGE) \
                                    || ((__VALUE__) == DDL_SPI_PHASE_2EDGE))

#define IS_DDL_SPI_NSS(__VALUE__) (((__VALUE__) == DDL_SPI_NSS_SOFT)         \
                                  || ((__VALUE__) == DDL_SPI_NSS_HARD_INPUT))

#define IS_DDL_SPI_BAUDRATE(__VALUE__) (((__VALUE__) == DDL_SPI_BAUDRATEPRESCALER_DIV2)     \
                                       || ((__VALUE__) == DDL_SPI_BAUDRATEPRESCALER_DIV4)   \
                                       || ((__VALUE__) == DDL_SPI_BAUDRATEPRESCALER_DIV8)   \
                                       || ((__VALUE__) == DDL_SPI_BAUDRATEPRESCALER_DIV16)  \
                                       || ((__VALUE__) == DDL_SPI_BAUDRATEPRESCALER_DIV32)  \
                                       || ((__VALUE__) == DDL_SPI_BAUDRATEPRESCALER_DIV64)  \
                                       || ((__VALUE__) == DDL_SPI_BAUDRATEPRESCALER_DIV128) \
                                       || ((__VALUE__) == DDL_SPI_BAUDRATEPRESCALER_DIV256))

#define IS_DDL_SPI_BITORDER(__VALUE__) (((__VALUE__) == DDL_SPI_LSB_FIRST) \
                                       || ((__VALUE__) == DDL_SPI_MSB_FIRST))

/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/** @addtogroup SPI_DDL_Exported_Functions SPI Exported Functions
  * @{
  */

/** @addtogroup SPI_DDL_EF_Init
  * @{
  */

/**
  * @brief  De-initialize the SPI registers to their default reset values.
  * @param  SPIx SPI Instance
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: SPI registers are de-initialized
  *          - ERROR: SPI registers are not de-initialized
  */
ErrorStatus DDL_SPI_DeInit(SPI_TypeDef *SPIx)
{
  (void)SPIx;

  ErrorStatus status = ERROR;

  /* Check the parameters */
  ASSERT_PARAM(IS_SPI_ALL_INSTANCE(SPIx));

  DDL_RCC_Unlock();
  /* Force reset of SPI clock */
  DDL_APB_GRP1_ForceReset(DDL_APB_GRP1_PERIPH_SPI);

  /* Release reset of SPI clock */
  DDL_APB_GRP1_ReleaseReset(DDL_APB_GRP1_PERIPH_SPI);

  DDL_RCC_Lock();
  status = SUCCESS;

  return status;
}

/**
  * @brief  Initialize the SPI registers according to the specified parameters in SPI_InitStruct.
  * @param  SPIx SPI Instance
  * @param  SPI_InitStruct pointer to a @ref DDL_SPI_InitTypeDef structure
  * @retval An ErrorStatus enumeration value. (Return always SUCCESS)
  */
ErrorStatus DDL_SPI_Init(SPI_TypeDef *SPIx, DDL_SPI_InitTypeDef *SPI_InitStruct)
{
  ErrorStatus status = ERROR;

  /* Check the SPI Instance SPIx*/
  ASSERT_PARAM(IS_SPI_ALL_INSTANCE(SPIx));

  /* Check the SPI parameters from SPI_InitStruct*/
  ASSERT_PARAM(IS_DDL_SPI_TRANSFER_DIRECTION(SPI_InitStruct->TransferDirection));
  ASSERT_PARAM(IS_DDL_SPI_MODE(SPI_InitStruct->Mode));
  ASSERT_PARAM(IS_DDL_SPI_DATAWIDTH(SPI_InitStruct->DataWidth));
  ASSERT_PARAM(IS_DDL_SPI_POLARITY(SPI_InitStruct->ClockPolarity));
  ASSERT_PARAM(IS_DDL_SPI_PHASE(SPI_InitStruct->ClockPhase));
  ASSERT_PARAM(IS_DDL_SPI_NSS(SPI_InitStruct->NSS));
  ASSERT_PARAM(IS_DDL_SPI_BAUDRATE(SPI_InitStruct->BaudRate));
  ASSERT_PARAM(IS_DDL_SPI_BITORDER(SPI_InitStruct->BitOrder));

  if (DDL_SPI_IsEnabled(SPIx) == 0)
  {
    /*---------------------------- SPIx CR1 Configuration ------------------------
     * Configure SPIx CR1 with parameters:
     * - Transfer direction:  BMEN, BMOEN, LSBFIRST
     * - Mode:                MSTR, SSI, SSM
     * - Data width:          DFLSEL
     * - Clock polarity:      CPOL
     * - Clock phase:         CPHA
     * - NSS management:      SSM
     * - Baud rate prescaler: BR
     */

    MODIFY_REG(SPIx->CR1,
               SPI_CR1_CLEAR_MASK,
               SPI_InitStruct->TransferDirection | SPI_InitStruct->Mode | SPI_InitStruct->DataWidth |
               SPI_InitStruct->ClockPolarity | SPI_InitStruct->ClockPhase |
               SPI_InitStruct->NSS | SPI_InitStruct->BaudRate |
               SPI_InitStruct->BitOrder);
  }

  return status;
}

/**
  * @brief  Set each @ref DDL_SPI_InitTypeDef field to default value.
  * @param  SPI_InitStruct pointer to a @ref DDL_SPI_InitTypeDef structure
  *         whose fields will be set to default values.
  * @retval None
  */
void DDL_SPI_StructInit(DDL_SPI_InitTypeDef *SPI_InitStruct)
{
  /* Set SPI_InitStruct fields to default values */
  SPI_InitStruct->TransferDirection = DDL_SPI_FULL_DUPLEX;
  SPI_InitStruct->Mode              = DDL_SPI_MODE_SLAVE;
  SPI_InitStruct->DataWidth         = DDL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct->ClockPolarity     = DDL_SPI_POLARITY_LOW;
  SPI_InitStruct->ClockPhase        = DDL_SPI_PHASE_1EDGE;
  SPI_InitStruct->NSS               = DDL_SPI_NSS_HARD_INPUT;
  SPI_InitStruct->BaudRate          = DDL_SPI_BAUDRATEPRESCALER_DIV2;
  SPI_InitStruct->BitOrder          = DDL_SPI_MSB_FIRST;
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

#endif /* SPI */

/**
  * @}
  */

#endif /* USE_FULL_DDL_DRIVER */

