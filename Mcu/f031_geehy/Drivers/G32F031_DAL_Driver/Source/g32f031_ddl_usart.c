/**
  *
  * @file    g32f031_ddl_usart.c
  * @brief   USART DDL module driver.
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
#include "g32f031_ddl_usart.h"
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

#if defined (USART)

/** @addtogroup USART_DDL USART
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/** @addtogroup USART_DDL_Private_Macros USART Private Macros
  * @{
  */

/* __BAUDRATE__ The maximum Baud Rate is derived from the maximum clock available
 *              divided by the smallest oversampling used on the USART (i.e. 8)    */
#define IS_DDL_USART_BAUDRATE(__BAUDRATE__) ((__BAUDRATE__) <= 12500000U)

/* __VALUE__ In case of oversampling by 16 and 8, BRR content must be greater than or equal to 16d. */
#define IS_DDL_USART_BR_MIN(__VALUE__) ((__VALUE__) >= 16U)

#define IS_DDL_USART_DIRECTION(__VALUE__) (((__VALUE__) == DDL_USART_DIRECTION_NONE) \
                                          || ((__VALUE__) == DDL_USART_DIRECTION_RX) \
                                          || ((__VALUE__) == DDL_USART_DIRECTION_TX) \
                                          || ((__VALUE__) == DDL_USART_DIRECTION_TX_RX))

#define IS_DDL_USART_PARITY(__VALUE__) (((__VALUE__) == DDL_USART_PARITY_NONE) \
                                       || ((__VALUE__) == DDL_USART_PARITY_EVEN) \
                                       || ((__VALUE__) == DDL_USART_PARITY_ODD))

#define IS_DDL_USART_DATAWIDTH(__VALUE__) (((__VALUE__) == DDL_USART_DATAWIDTH_8B) \
                                          || ((__VALUE__) == DDL_USART_DATAWIDTH_9B) \
                                          || ((__VALUE__) == DDL_USART_DATAWIDTH_7B))

#define IS_DDL_USART_OVERSAMPLING(__VALUE__) (((__VALUE__) == DDL_USART_OVERSAMPLING_16) \
                                             || ((__VALUE__) == DDL_USART_OVERSAMPLING_8))

#define IS_DDL_USART_LASTBITCLKOUTPUT(__VALUE__) (((__VALUE__) == DDL_USART_LASTCLKPULSE_NO_OUTPUT) \
                                                 || ((__VALUE__) == DDL_USART_LASTCLKPULSE_OUTPUT))

#define IS_DDL_USART_CLOCKPHASE(__VALUE__) (((__VALUE__) == DDL_USART_PHASE_1EDGE) \
                                           || ((__VALUE__) == DDL_USART_PHASE_2EDGE))

#define IS_DDL_USART_CLOCKPOLARITY(__VALUE__) (((__VALUE__) == DDL_USART_POLARITY_LOW) \
                                              || ((__VALUE__) == DDL_USART_POLARITY_HIGH))

#define IS_DDL_USART_CLOCKOUTPUT(__VALUE__) (((__VALUE__) == DDL_USART_CLOCK_DISABLE) \
                                            || ((__VALUE__) == DDL_USART_CLOCK_ENABLE))

#define IS_DDL_USART_STOPBITS(__VALUE__) (((__VALUE__) == DDL_USART_STOPBITS_1) \
                                         || ((__VALUE__) == DDL_USART_STOPBITS_2))

#define IS_DDL_USART_HWCONTROL(__VALUE__) (((__VALUE__) == DDL_USART_HWCONTROL_NONE) \
                                          || ((__VALUE__) == DDL_USART_HWCONTROL_RTS) \
                                          || ((__VALUE__) == DDL_USART_HWCONTROL_CTS) \
                                          || ((__VALUE__) == DDL_USART_HWCONTROL_RTS_CTS))

/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/** @addtogroup USART_DDL_Exported_Functions USART Exported Functions
  * @{
  */

/** @addtogroup USART_DDL_EF_Init
  * @{
  */

/**
  * @brief  De-initialize USART registers (Registers restored to their default values).
  * @param  USARTx USART Instance
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: USART registers are de-initialized
  *          - ERROR: USART registers are not de-initialized
  */
ErrorStatus DDL_USART_DeInit(USART_TypeDef *USARTx)
{
  ErrorStatus status = SUCCESS;

  /* Check the parameters */
  ASSERT_PARAM(IS_USART_INSTANCE(USARTx));

  DDL_RCC_Unlock();

  if (USARTx == USART)
  {
    /* Force reset of USART clock */
    DDL_APB_GRP1_ForceReset(DDL_APB_GRP1_PERIPH_USART);

    /* Release reset of USART clock */
    DDL_APB_GRP1_ReleaseReset(DDL_APB_GRP1_PERIPH_USART);
  }
  else if (USARTx == UART)
  {
    /* Force reset of USART clock */
    DDL_APB_GRP1_ForceReset(DDL_APB_GRP1_PERIPH_UART);

    /* Release reset of USART clock */
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
  * @brief  Initialize USART registers according to the specified
  *         parameters in USART_InitStruct.
  * @note   As some bits in USART configuration registers can only be written when the USART is disabled (USART_CTRL1_UEN bit =0),
  *         USART IP should be in disabled state prior calling this function. Otherwise, ERROR result will be returned.
  * @note   Baud rate value stored in USART_InitStruct BaudRate field, should be valid (different from 0).
  * @param  USARTx USART Instance
  * @param  USART_InitStruct pointer to a DDL_USART_InitTypeDef structure
  *         that contains the configuration information for the specified USART peripheral.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: USART registers are initialized according to USART_InitStruct content
  *          - ERROR: Problem occurred during USART Registers initialization
  */
ErrorStatus DDL_USART_Init(USART_TypeDef *USARTx, DDL_USART_InitTypeDef *USART_InitStruct)
{
  ErrorStatus status = ERROR;
  uint32_t periphclk = DDL_RCC_PERIPH_FREQUENCY_NO;
  DDL_RCC_ClocksTypeDef rcc_clocks;

  /* Check the parameters */
  ASSERT_PARAM(IS_USART_INSTANCE(USARTx));
  ASSERT_PARAM(IS_DDL_USART_BAUDRATE(USART_InitStruct->BaudRate));
  ASSERT_PARAM(IS_DDL_USART_DATAWIDTH(USART_InitStruct->DataWidth));
  ASSERT_PARAM(IS_DDL_USART_STOPBITS(USART_InitStruct->StopBits));
  ASSERT_PARAM(IS_DDL_USART_PARITY(USART_InitStruct->Parity));
  ASSERT_PARAM(IS_DDL_USART_DIRECTION(USART_InitStruct->TransferDirection));
  ASSERT_PARAM(IS_DDL_USART_HWCONTROL(USART_InitStruct->HardwareFlowControl));
  ASSERT_PARAM(IS_DDL_USART_OVERSAMPLING(USART_InitStruct->OverSampling));

  /* USART needs to be in disabled state, in order to be able to configure some bits in
     CTRLx registers */
  if (DDL_USART_IsEnabled(USARTx) == 0U)
  {
    /*---------------------------- USART CR1 Configuration -----------------------
     * Configure USARTx CR1 (USART Word Length, Parity, Mode and Oversampling bits) with parameters:
     * - DataWidth:          USART_CR1_M bits according to USART_InitStruct->DataWidth value
     * - Parity:             USART_CR1_PCEN, USART_CR1_PS bits according to USART_InitStruct->Parity value
     * - TransferDirection:  USART_CR1_TEN, USART_CR1_REN bits according to USART_InitStruct->TransferDirection value
     * - Oversampling:       USART_CR1_OSMCFG bit according to USART_InitStruct->OverSampling value.
     */
    MODIFY_REG(USARTx->CR1,
               (USART_CR1_M | USART_CR1_PCEN | USART_CR1_PS |
                USART_CR1_TEN | USART_CR1_REN | USART_CR1_OSMCFG),
               (USART_InitStruct->DataWidth | USART_InitStruct->Parity |
                USART_InitStruct->TransferDirection | USART_InitStruct->OverSampling));

    /*---------------------------- USART CR2 Configuration -----------------------
     * Configure USARTx CR2 (Stop bits) with parameters:
     * - Stop Bits:          USART_CR2_STOP bits according to USART_InitStruct->StopBits value.
     * - CLKEN, CPOL, CPHA and LBCL bits are to be configured using DDL_USART_ClockInit().
     */
    DDL_USART_SetStopBitsLength(USARTx, USART_InitStruct->StopBits);

    /*---------------------------- USART CTRL3 Configuration -----------------------
     * Configure USARTx CTRL3 (Hardware Flow Control) with parameters:
     * - HardwareFlowControl: USART_CR3_RTSEN, USART_CR3_CTSEN bits according to USART_InitStruct->HardwareFlowControl value.
     */
    DDL_USART_SetHWFlowCtrl(USARTx, USART_InitStruct->HardwareFlowControl);

    /*---------------------------- USART BRR Configuration -----------------------
     * Retrieve Clock frequency used for USART Peripheral
     */
    DDL_RCC_GetSysctrlClocksFreq(&rcc_clocks);

    periphclk = rcc_clocks.PCLK_Frequency;

    /* Configure the USART Baud Rate :
       - valid baud rate value (different from 0) is required
       - Peripheral clock as returned by RCM service, should be valid (different from 0).
    */
    if ((periphclk != DDL_RCC_PERIPH_FREQUENCY_NO)
        && (USART_InitStruct->BaudRate != 0U))
    {
      status = SUCCESS;
      DDL_USART_SetBaudRate(USARTx,
                           periphclk,
                           USART_InitStruct->OverSampling,
                           USART_InitStruct->BaudRate);

      /* Check BR is greater than or equal to 16d */
      ASSERT_PARAM(IS_DDL_USART_BR_MIN(USARTx->BRR));
    }
  }

  return (status);
}

/**
  * @brief Set each @ref DDL_USART_InitTypeDef field to default value.
  * @param USART_InitStruct Pointer to a @ref DDL_USART_InitTypeDef structure
  *                         whose fields will be set to default values.
  * @retval None
  */

void DDL_USART_StructInit(DDL_USART_InitTypeDef *USART_InitStruct)
{
  /* Set USART_InitStruct fields to default values */
  USART_InitStruct->BaudRate            = 9600U;
  USART_InitStruct->DataWidth           = DDL_USART_DATAWIDTH_8B;
  USART_InitStruct->StopBits            = DDL_USART_STOPBITS_1;
  USART_InitStruct->Parity              = DDL_USART_PARITY_NONE ;
  USART_InitStruct->TransferDirection   = DDL_USART_DIRECTION_TX_RX;
  USART_InitStruct->HardwareFlowControl = DDL_USART_HWCONTROL_NONE;
  USART_InitStruct->OverSampling        = DDL_USART_OVERSAMPLING_16;
}

/**
  * @brief  Initialize USART Clock related settings according to the
  *         specified parameters in the USART_ClockInitStruct.
  * @note   As some bits in USART configuration registers can only be written when the USART is disabled (USART_CTRL1_UEN bit =0),
  *         USART IP should be in disabled state prior calling this function. Otherwise, ERROR result will be returned.
  * @param  USARTx USART Instance
  * @param  USART_ClockInitStruct Pointer to a @ref DDL_USART_ClockInitTypeDef structure
  *         that contains the Clock configuration information for the specified USART peripheral.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: USART registers related to Clock settings are initialized according to USART_ClockInitStruct content
  *          - ERROR: Problem occurred during USART Registers initialization
  */
ErrorStatus DDL_USART_ClockInit(USART_TypeDef *USARTx, DDL_USART_ClockInitTypeDef *USART_ClockInitStruct)
{
  ErrorStatus status = SUCCESS;

  /* Check USART Instance and Clock signal output parameters */
  ASSERT_PARAM(IS_USART_INSTANCE(USARTx));
  ASSERT_PARAM(IS_DDL_USART_CLOCKOUTPUT(USART_ClockInitStruct->ClockOutput));

  /* USART needs to be in disabled state, in order to be able to configure some bits in
     CTRLx registers */
  if (DDL_USART_IsEnabled(USARTx) == 0U)
  {
    /*---------------------------- USART CR2 Configuration -----------------------*/
    /* If Clock signal has to be output */
    if (USART_ClockInitStruct->ClockOutput == DDL_USART_CLOCK_DISABLE)
    {
      /* Deactivate Clock signal delivery :
       * - Disable Clock Output:        USART_CR2_CLKEN cleared
       */
      DDL_USART_DisableSCLKOutput(USARTx);
    }
    else
    {
      /* Ensure USART instance is USART capable */
      ASSERT_PARAM(IS_USART_INSTANCE(USARTx));

      /* Check clock related parameters */
      ASSERT_PARAM(IS_DDL_USART_CLOCKPOLARITY(USART_ClockInitStruct->ClockPolarity));
      ASSERT_PARAM(IS_DDL_USART_CLOCKPHASE(USART_ClockInitStruct->ClockPhase));
      ASSERT_PARAM(IS_DDL_USART_LASTBITCLKOUTPUT(USART_ClockInitStruct->LastBitClockPulse));

      /*---------------------------- USART CR2 Configuration -----------------------
       * Configure USARTx CR2 (Clock signal related bits) with parameters:
       * - Enable Clock Output:         USART_CR2_CLKEN set
       * - Clock Polarity:              USART_CR2_CPOL bit according to USART_ClockInitStruct->ClockPolarity value
       * - Clock Phase:                 USART_CR2_CPHA bit according to USART_ClockInitStruct->ClockPhase value
       * - Last Bit Clock Pulse Output: USART_CR2_LBCL bit according to USART_ClockInitStruct->LastBitClockPulse value.
       */
      MODIFY_REG(USARTx->CR2,
                 USART_CR2_CLKEN | USART_CR2_CPHA | USART_CR2_CPOL | USART_CR2_LBCL,
                 USART_ClockInitStruct->ClockPolarity | USART_ClockInitStruct->ClockPhase |
                 USART_ClockInitStruct->LastBitClockPulse);
    }
  }
  /* Else (USART not in Disabled state => return ERROR */
  else
  {
    status = ERROR;
  }

  return (status);
}

/**
  * @brief Set each field of a @ref DDL_USART_ClockInitTypeDef type structure to default value.
  * @param USART_ClockInitStruct Pointer to a @ref DDL_USART_ClockInitTypeDef structure
  *                              whose fields will be set to default values.
  * @retval None
  */
void DDL_USART_ClockStructInit(DDL_USART_ClockInitTypeDef *USART_ClockInitStruct)
{
  /* Set DDL_USART_ClockInitStruct fields with default values */
  USART_ClockInitStruct->ClockOutput       = DDL_USART_CLOCK_DISABLE;
  USART_ClockInitStruct->ClockPolarity     = DDL_USART_POLARITY_LOW;            /* Not relevant when ClockOutput = DDL_USART_CLOCK_DISABLE */
  USART_ClockInitStruct->ClockPhase        = DDL_USART_PHASE_1EDGE;             /* Not relevant when ClockOutput = DDL_USART_CLOCK_DISABLE */
  USART_ClockInitStruct->LastBitClockPulse = DDL_USART_LASTCLKPULSE_NO_OUTPUT;  /* Not relevant when ClockOutput = DDL_USART_CLOCK_DISABLE */
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

#endif /* USART */

/**
  * @}
  */

#endif /* USE_FULL_DDL_DRIVER */


