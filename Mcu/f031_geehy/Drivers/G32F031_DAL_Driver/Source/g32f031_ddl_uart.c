/**
  *
  * @file    g32f031_ddl_uart.c
  * @brief   UART DDL module driver.
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
#include "g32f031_ddl_uart.h"
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

#if defined (UART)

/** @addtogroup UART_DDL UART
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/** @addtogroup UART_DDL_Private_Macros UART Private Macros
  * @{
  */

/* __BAUDRATE__ The maximum Baud Rate is derived from the maximum clock available
 *              divided by the smallest oversampling used on the UART (i.e. 8)    */
#define IS_DDL_UART_BAUDRATE(__BAUDRATE__) ((__BAUDRATE__) <= 12500000U)

/* __VALUE__ In case of oversampling by 16 and 8, BRR content must be greater than or equal to 16d. */
#define IS_DDL_UART_BR_MIN(__VALUE__) ((__VALUE__) >= 16U)

#define IS_DDL_UART_DIRECTION(__VALUE__) (((__VALUE__) == DDL_UART_DIRECTION_NONE) \
                                          || ((__VALUE__) == DDL_UART_DIRECTION_RX) \
                                          || ((__VALUE__) == DDL_UART_DIRECTION_TX) \
                                          || ((__VALUE__) == DDL_UART_DIRECTION_TX_RX))

#define IS_DDL_UART_PARITY(__VALUE__) (((__VALUE__) == DDL_UART_PARITY_NONE) \
                                       || ((__VALUE__) == DDL_UART_PARITY_EVEN) \
                                       || ((__VALUE__) == DDL_UART_PARITY_ODD))

#define IS_DDL_UART_DATAWIDTH(__VALUE__) (((__VALUE__) == DDL_UART_DATAWIDTH_8B) \
                                          || ((__VALUE__) == DDL_UART_DATAWIDTH_9B) \
                                          || ((__VALUE__) == DDL_UART_DATAWIDTH_7B))

#define IS_DDL_UART_STOPBITS(__VALUE__) (((__VALUE__) == DDL_UART_STOPBITS_1) \
                                         || ((__VALUE__) == DDL_UART_STOPBITS_2))

/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/** @addtogroup UART_DDL_Exported_Functions UART Exported Functions
  * @{
  */

/** @addtogroup UART_DDL_EF_Init
  * @{
  */

/**
  * @brief  De-initialize UART registers (Registers restored to their default values).
  * @param  UARTx UART Instance
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: UART registers are de-initialized
  *          - ERROR: UART registers are not de-initialized
  */
ErrorStatus DDL_UART_DeInit(USART_TypeDef *UARTx)
{
  ErrorStatus status = SUCCESS;

  /* Check the parameters */
  ASSERT_PARAM(IS_USART_INSTANCE(UARTx));

  DDL_RCC_Unlock();

  if (UARTx == UART)
  {
    /* Force reset of UART clock */
    DDL_APB_GRP1_ForceReset(DDL_APB_GRP1_PERIPH_UART);

    /* Release reset of UART clock */
    DDL_APB_GRP1_ReleaseReset(DDL_APB_GRP1_PERIPH_UART);
  }
  else
  {
    status = ERROR;
  }

  DDL_RCC_Lock();
  return (status);
}

/**
  * @brief  Initialize UART registers according to the specified
  *         parameters in UART_InitStruct.
  * @note   As some bits in UART configuration registers can only be written when the UART is disabled (UART_CTRL1_UEN bit =0),
  *         UART IP should be in disabled state prior calling this function. Otherwise, ERROR result will be returned.
  * @note   Baud rate value stored in UART_InitStruct BaudRate field, should be valid (different from 0).
  * @param  UARTx UART Instance
  * @param  UART_InitStruct pointer to a DDL_UART_InitTypeDef structure
  *         that contains the configuration information for the specified UART peripheral.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: UART registers are initialized according to UART_InitStruct content
  *          - ERROR: Problem occurred during UART Registers initialization
  */
ErrorStatus DDL_UART_Init(USART_TypeDef *UARTx, DDL_UART_InitTypeDef *UART_InitStruct)
{
  ErrorStatus status = ERROR;
  uint32_t periphclk = DDL_RCC_PERIPH_FREQUENCY_NO;
  DDL_RCC_ClocksTypeDef rcc_clocks;

  /* Check the parameters */
  ASSERT_PARAM(IS_USART_INSTANCE(UARTx));
  ASSERT_PARAM(IS_DDL_UART_BAUDRATE(UART_InitStruct->BaudRate));
  ASSERT_PARAM(IS_DDL_UART_DATAWIDTH(UART_InitStruct->DataWidth));
  ASSERT_PARAM(IS_DDL_UART_STOPBITS(UART_InitStruct->StopBits));
  ASSERT_PARAM(IS_DDL_UART_PARITY(UART_InitStruct->Parity));
  ASSERT_PARAM(IS_DDL_UART_DIRECTION(UART_InitStruct->TransferDirection));

  /* UART needs to be in disabled state, in order to be able to configure some bits in
     CTRLx registers */
  if (DDL_UART_IsEnabled(UARTx) == 0U)
  {
    /*---------------------------- UART CR1 Configuration -----------------------
     * Configure UARTx CR1 (UART Word Length, Parity, Mode and Oversampling bits) with parameters:
     * - DataWidth:          UART_CR1_M bits according to UART_InitStruct->DataWidth value
     * - Parity:             UART_CR1_PCEN, UART_CR1_PSEL bits according to UART_InitStruct->Parity value
     * - TransferDirection:  UART_CR1_TEN, UART_CR1_REN bits according to UART_InitStruct->TransferDirection value
     * - Oversampling:       UART_CR1_OVER8 bit according to UART_InitStruct->OverSampling value.
     */
    MODIFY_REG(UARTx->CR1,
               (UART_CR1_M | UART_CR1_PCEN | UART_CR1_PSEL |
                UART_CR1_TEN | UART_CR1_REN),
               (UART_InitStruct->DataWidth | UART_InitStruct->Parity |
                UART_InitStruct->TransferDirection));

    /*---------------------------- UART CR2 Configuration -----------------------
     * Configure UARTx CR2 (Stop bits) with parameters:
     * - Stop Bits:          UART_CR2_STOP bits according to UART_InitStruct->StopBits value.
     * - CLKEN, CPOL, CPHA and LBCL bits are to be configured using DDL_UART_ClockInit().
     */
    DDL_UART_SetStopBitsLength(UARTx, UART_InitStruct->StopBits);

    /*---------------------------- UART BRR Configuration -----------------------
     * Retrieve Clock frequency used for UART Peripheral
     */
    DDL_RCC_GetSysctrlClocksFreq(&rcc_clocks);

    periphclk = rcc_clocks.PCLK_Frequency;

    /* Configure the UART Baud Rate :
       - valid baud rate value (different from 0) is required
       - Peripheral clock as returned by RCM service, should be valid (different from 0).
    */
    if ((periphclk != DDL_RCC_PERIPH_FREQUENCY_NO)
        && (UART_InitStruct->BaudRate != 0U))
    {
      status = SUCCESS;
      DDL_UART_SetBaudRate(UARTx,
                           periphclk,
                           UART_InitStruct->BaudRate);

      /* Check BR is greater than or equal to 16d */
      ASSERT_PARAM(IS_DDL_UART_BR_MIN(UARTx->BRR));
    }
  }

  return (status);
}

/**
  * @brief Set each @ref DDL_UART_InitTypeDef field to default value.
  * @param UART_InitStruct Pointer to a @ref DDL_UART_InitTypeDef structure
  *                         whose fields will be set to default values.
  * @retval None
  */

void DDL_UART_StructInit(DDL_UART_InitTypeDef *UART_InitStruct)
{
  /* Set UART_InitStruct fields to default values */
  UART_InitStruct->BaudRate            = 9600U;
  UART_InitStruct->DataWidth           = DDL_UART_DATAWIDTH_8B;
  UART_InitStruct->StopBits            = DDL_UART_STOPBITS_1;
  UART_InitStruct->Parity              = DDL_UART_PARITY_NONE ;
  UART_InitStruct->TransferDirection   = DDL_UART_DIRECTION_TX_RX;
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

#endif /* UART */

/**
  * @}
  */

#endif /* USE_FULL_DDL_DRIVER */


