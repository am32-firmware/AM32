/**
  *
  * @file    g32f031_ddl_uart.h
  * @brief   Header file of UART DDL module.
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
  * Copyright (C) 2026 Geehy Semiconductor.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef G32F031_DDL_UART_H
#define G32F031_DDL_UART_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "g32f0xx.h"

/** @addtogroup G32F031_DDL_Driver
  * @{
  */

#if defined (UART)

/** @defgroup UART_DDL UART
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
#if defined(USE_FULL_DDL_DRIVER)
/** @defgroup UART_DDL_ES_INIT UART Exported Init structures
  * @{
  */

/**
  * @brief DDL UART Init Structure definition
  */
typedef struct
{
  uint32_t BaudRate;                  /*!< This field defines expected Usart communication baud rate.

                                           This feature can be modified afterwards using unitary function @ref DDL_UART_SetBaudRate().*/

  uint32_t DataWidth;                 /*!< Specifies the number of data bits transmitted or received in a frame.
                                           This parameter can be a value of @ref UART_DDL_EC_DATAWIDTH.

                                           This feature can be modified afterwards using unitary function @ref DDL_UART_SetDataWidth().*/

  uint32_t StopBits;                  /*!< Specifies the number of stop bits transmitted.
                                           This parameter can be a value of @ref UART_DDL_EC_STOPBITS.

                                           This feature can be modified afterwards using unitary function @ref DDL_UART_SetStopBitsLength().*/

  uint32_t Parity;                    /*!< Specifies the parity mode.
                                           This parameter can be a value of @ref UART_DDL_EC_PARITY.

                                           This feature can be modified afterwards using unitary function @ref DDL_UART_SetParity().*/

  uint32_t TransferDirection;         /*!< Specifies whether the Receive and/or Transmit mode is enabled or disabled.
                                           This parameter can be a value of @ref UART_DDL_EC_DIRECTION.

                                           This feature can be modified afterwards using unitary function @ref DDL_UART_SetTransferDirection().*/

} DDL_UART_InitTypeDef;

/**
  * @}
  */
#endif /* USE_FULL_DDL_DRIVER */

/* Exported constants --------------------------------------------------------*/
/** @defgroup UART_DDL_Exported_Constants UART Exported Constants
  * @{
  */

/** @defgroup UART_DDL_EC_GET_FLAG Get Flags Defines
  * @brief    Flags defines which can be used with DDL_UART_ReadReg function
  * @{
  */
#define DDL_UART_SR_PEFLGFLG                       UART_SR_PEFLG                      /*!< Parity error flag */
#define DDL_UART_SR_FEFLGFLG                       UART_SR_FEFLG                      /*!< Framing error flag */
#define DDL_UART_SR_NEFLGFLG                       UART_SR_NEFLG                      /*!< Noise detected flag */
#define DDL_UART_SR_OVREFLG                     UART_SR_OREFLG                     /*!< Overrun error flag */
#define DDL_UART_SR_IDLEFLGFLG                     UART_SR_IDLEFLG                    /*!< Idle line detected flag */
#define DDL_UART_SR_RXBNEFLG                    UART_SR_RXNEFLG                    /*!< Read data register not empty flag */
#define DDL_UART_SR_TXCFLG                      UART_SR_TCFLG                      /*!< Transmission complete flag */
#define DDL_UART_SR_TXBEFLG                     UART_SR_TXEFLG                     /*!< Transmit data register empty flag */
#define DDL_UART_SR_LBDFLGFLG                      UART_SR_LBDFLG                     /*!< LIN break detection flag */
#define DDL_UART_SR_RXTOFLGFLG                    UART_SR_RXTOFLG                   /*!< Receive overtime flag */
#define DDL_UART_SR_ABCFLGFLG                     UART_SR_ABCFLG                    /*!< Automatic baud rate complete Flag */
#define DDL_UART_SR_ABERRFLGFLG                    UART_SR_ABERRFLG                   /*!< Automatic baud rate error Flag */
/**
  * @}
  */

/** @defgroup UART_DDL_EC_IT IT Defines
  * @brief    IT defines which can be used with DDL_UART_ReadReg and  DDL_UART_WriteReg functions
  * @{
  */
#define DDL_UART_CTRL1_IDLEIEN                  UART_CR1_IDLEIEN                 /*!< IDLE interrupt enable */
#define DDL_UART_CTRL1_RXBNEIEN                 UART_CR1_RXNEIEN                 /*!< Read data register not empty interrupt enable */
#define DDL_UART_CTRL1_TXCIEN                   UART_CR1_TCIEN                   /*!< Transmission complete interrupt enable */
#define DDL_UART_CTRL1_TXBEIEN                  UART_CR1_TXEIEN                  /*!< Transmit data register empty interrupt enable */
#define DDL_UART_CTRL1_PEIEN                    UART_CR1_PEIEN                   /*!< Parity error */
#define DDL_UART_CTRL1_RXTOIEN                  UART_CR1_RXTOIEN                 /*!< Receive overtime interrupt enable */
#define DDL_UART_CTRL2_LBDIEN                   UART_CR2_LBDIEN                  /*!< LIN break detection interrupt enable */
#define DDL_UART_CTRL2_ABCIEN                   UART_CR2_ABCIEN                  /*!< Automatic baud rate complete interrupt enable */
#define DDL_UART_CTRL2_ADEIEN                   UART_CR2_ABEIEN                  /*!< Automatic baud rate error interrupt enable */
#define DDL_UART_CTRL3_ERRIEN                   UART_CR3_EIEN                    /*!< Error interrupt enable */
/**
  * @}
  */

/** @defgroup UART_DDL_EC_DIRECTION Communication Direction
  * @{
  */
#define DDL_UART_DIRECTION_NONE                 0x00000000U                     /*!< Transmitter and Receiver are disabled */
#define DDL_UART_DIRECTION_RX                   UART_CR1_REN                     /*!< Transmitter is disabled and Receiver is enabled */
#define DDL_UART_DIRECTION_TX                   UART_CR1_TEN                     /*!< Transmitter is enabled and Receiver is disabled */
#define DDL_UART_DIRECTION_TX_RX                (UART_CR1_TEN |UART_CR1_REN)      /*!< Transmitter and Receiver are enabled */
/**
  * @}
  */

/** @defgroup UART_DDL_EC_PARITY Parity Control
  * @{
  */
#define DDL_UART_PARITY_NONE                    0x00000000U                     /*!< Parity control disabled */
#define DDL_UART_PARITY_EVEN                    UART_CR1_PCEN                    /*!< Parity control enabled and Even Parity is selected */
#define DDL_UART_PARITY_ODD                     (UART_CR1_PCEN | UART_CR1_PSEL)    /*!< Parity control enabled and Odd Parity is selected */
/**
  * @}
  */

/** @defgroup UART_DDL_EC_DATAWIDTH Datawidth
  * @{
  */
#define DDL_UART_DATAWIDTH_8B                   0x00000000U               /*!< 8 bits word length : Start bit, 8 data bits, n stop bits */
#define DDL_UART_DATAWIDTH_9B                   UART_CR1_M_0             /*!< 9 bits word length : Start bit, 9 data bits, n stop bits */
#define DDL_UART_DATAWIDTH_7B                   UART_CR1_M_1             /*!< 7 bits word length : Start bit, 7 data bits, n stop bits */
/**
  * @}
  */

/** @defgroup UART_LL_EC_TXRX TX RX Pins Swap
  * @{
  */
#define DDL_UART_TXRX_STANDARD                  0x00000000U           /*!< TX/RX pins are used as defined in standard pinout */
#define DDL_UART_TXRX_SWAPPED                   UART_CR1_SWAPEN        /*!< TX and RX pins functions are swapped.             */
/**
  * @}
  */

/** @defgroup UART_DDL_EC_STOPBITS Stop Bits
  * @{
  */
#define DDL_UART_STOPBITS_1                     0x00000000U           /*!< 1 stop bit */
#define DDL_UART_STOPBITS_2                     UART_CR2_STOP         /*!< 2 stop bits */
/**
  * @}
  */

/** @defgroup UART_DDL_EC_LINBREAK_DETECT LIN Break Detection Length
  * @{
  */
#define DDL_UART_LINBREAK_DETECT_10B            0x00000000U           /*!< 10-bit break detection method selected */
#define DDL_UART_LINBREAK_DETECT_11B            UART_CR2_LBDL        /*!< 11-bit break detection method selected */
/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup UART_DDL_Exported_Macros UART Exported Macros
  * @{
  */

/** @defgroup UART_DDL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in UART register
  * @param  __INSTANCE__ UART Instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define DDL_UART_WriteReg(__INSTANCE__, __REG__, __VALUE__) WRITE_REG(__INSTANCE__->__REG__, (__VALUE__))

/**
  * @brief  Read a value in UART register
  * @param  __INSTANCE__ UART Instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define DDL_UART_ReadReg(__INSTANCE__, __REG__) READ_REG(__INSTANCE__->__REG__)
/**
  * @}
  */

/** @defgroup UART_DDL_EM_Exported_Macros_Helper Exported_Macros_Helper
  * @{
  */

/**
  * @brief  Compute UARTDIV value according to Peripheral Clock and
  *         expected Baud Rate in 16 bits sampling mode (32 bits value of UARTDIV is returned)
  * @param  __PERIPHCLK__ Peripheral Clock frequency used for UART instance
  * @param  __BAUDRATE__ Baud rate value to achieve
  * @retval UARTDIV value to be used for BRR register filling in OverSampling_16 case
  */
#define __DDL_UART_DIV_SAMPLING16_100(__PERIPHCLK__, __BAUDRATE__)     ((uint32_t)((((uint64_t)(__PERIPHCLK__))*25)/(4*((uint64_t)(__BAUDRATE__)))))
#define __DDL_UART_DIVMANT_SAMPLING16(__PERIPHCLK__, __BAUDRATE__)     (__DDL_UART_DIV_SAMPLING16_100((__PERIPHCLK__), (__BAUDRATE__))/100)
#define __DDL_UART_DIVFRAQ_SAMPLING16(__PERIPHCLK__, __BAUDRATE__)     ((((__DDL_UART_DIV_SAMPLING16_100((__PERIPHCLK__), (__BAUDRATE__)) - (__DDL_UART_DIVMANT_SAMPLING16((__PERIPHCLK__), (__BAUDRATE__)) * 100)) * 16)\
                                                                         + 50) / 100)
/* UART BRR = mantissa + overflow + fraction
            = (UART DIVMANT << 4) + (UART DIVFRAQ & 0xF0) + (UART DIVFRAQ & 0x0F) */
#define __DDL_UART_DIV_SAMPLING16(__PERIPHCLK__, __BAUDRATE__)            (((__DDL_UART_DIVMANT_SAMPLING16((__PERIPHCLK__), (__BAUDRATE__)) << 4) + \
                                                                            (__DDL_UART_DIVFRAQ_SAMPLING16((__PERIPHCLK__), (__BAUDRATE__)) & 0xF0)) + \
                                                                           (__DDL_UART_DIVFRAQ_SAMPLING16((__PERIPHCLK__), (__BAUDRATE__)) & 0x0F))

/**
  * @}
  */

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/

/** @defgroup UART_DDL_Exported_Functions UART Exported Functions
  * @{
  */

/** @defgroup UART_DDL_EF_Configuration Configuration functions
  * @{
  */

/**
  * @brief  UART Enable
  * @param  UARTx UART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_UART_Enable(USART_TypeDef *UARTx)
{
  SET_BIT(UARTx->CR1, UART_CR1_UEN);
}

/**
  * @brief  UART Disable (all UART prescalers and outputs are disabled)
  * @note   When UART is disabled, UART prescalers and outputs are stopped immediately,
  *         and current operations are discarded. The configuration of the UART is kept, but all the status
  *         flags, in the UARTx_SR are set to their default values.
  * @param  UARTx UART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_UART_Disable(USART_TypeDef *UARTx)
{
  CLEAR_BIT(UARTx->CR1, UART_CR1_UEN);
}

/**
  * @brief  Indicate if UART is enabled
  * @param  UARTx UART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_UART_IsEnabled(USART_TypeDef *UARTx)
{
  return (READ_BIT(UARTx->CR1, UART_CR1_UEN) == (UART_CR1_UEN));
}

/**
  * @brief  Receiver Enable (Receiver is enabled and begins searching for a start bit)
  * @param  UARTx UART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_UART_EnableDirectionRx(USART_TypeDef *UARTx)
{
  SET_BIT(UARTx->CR1, UART_CR1_REN);
}

/**
  * @brief  Receiver Disable
  * @param  UARTx UART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_UART_DisableDirectionRx(USART_TypeDef *UARTx)
{
  CLEAR_BIT(UARTx->CR1, UART_CR1_REN);
}

/**
  * @brief  Transmitter Enable
  * @param  UARTx UART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_UART_EnableDirectionTx(USART_TypeDef *UARTx)
{
  SET_BIT(UARTx->CR1, UART_CR1_TEN);
}

/**
  * @brief  Transmitter Disable
  * @param  UARTx UART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_UART_DisableDirectionTx(USART_TypeDef *UARTx)
{
  CLEAR_BIT(UARTx->CR1, UART_CR1_TEN);
}

/**
  * @brief  Configure simultaneously enabled/disabled states
  *         of Transmitter and Receiver
  * @param  UARTx UART Instance
  * @param  TransferDirection This parameter can be one of the following values:
  *         @arg @ref DDL_UART_DIRECTION_NONE
  *         @arg @ref DDL_UART_DIRECTION_RX
  *         @arg @ref DDL_UART_DIRECTION_TX
  *         @arg @ref DDL_UART_DIRECTION_TX_RX
  * @retval None
  */
__STATIC_INLINE void DDL_UART_SetTransferDirection(USART_TypeDef *UARTx, uint32_t TransferDirection)
{
  MODIFY_REG(UARTx->CR1, UART_CR1_REN | UART_CR1_TEN, TransferDirection);
}

/**
  * @brief  Return enabled/disabled states of Transmitter and Receiver
  * @param  UARTx UART Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_UART_DIRECTION_NONE
  *         @arg @ref DDL_UART_DIRECTION_RX
  *         @arg @ref DDL_UART_DIRECTION_TX
  *         @arg @ref DDL_UART_DIRECTION_TX_RX
  */
__STATIC_INLINE uint32_t DDL_UART_GetTransferDirection(USART_TypeDef *UARTx)
{
  return (uint32_t)(READ_BIT(UARTx->CR1, UART_CR1_REN | UART_CR1_TEN));
}

/**
  * @brief  Configure Parity (enabled/disabled and parity mode if enabled).
  * @note   This function selects if hardware parity control (generation and detection) is enabled or disabled.
  *         When the parity control is enabled (Odd or Even), computed parity bit is inserted at the MSB position
  *         (9th or 8th bit depending on data width) and parity is checked on the received data.
  * @param  UARTx UART Instance
  * @param  Parity This parameter can be one of the following values:
  *         @arg @ref DDL_UART_PARITY_NONE
  *         @arg @ref DDL_UART_PARITY_EVEN
  *         @arg @ref DDL_UART_PARITY_ODD
  * @retval None
  */
__STATIC_INLINE void DDL_UART_SetParity(USART_TypeDef *UARTx, uint32_t Parity)
{
  MODIFY_REG(UARTx->CR1, UART_CR1_PSEL | UART_CR1_PCEN, Parity);
}

/**
  * @brief  Return Parity configuration (enabled/disabled and parity mode if enabled)
  * @param  UARTx UART Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_UART_PARITY_NONE
  *         @arg @ref DDL_UART_PARITY_EVEN
  *         @arg @ref DDL_UART_PARITY_ODD
  */
__STATIC_INLINE uint32_t DDL_UART_GetParity(USART_TypeDef *UARTx)
{
  return (uint32_t)(READ_BIT(UARTx->CR1, UART_CR1_PSEL | UART_CR1_PCEN));
}

/**
  * @brief  Set Word length (i.e. nb of data bits, excluding start and stop bits)
  * @param  UARTx UART Instance
  * @param  DataWidth This parameter can be one of the following values:
  *         @arg @ref DDL_UART_DATAWIDTH_8B
  *         @arg @ref DDL_UART_DATAWIDTH_9B
  *         @arg @ref DDL_UART_DATAWIDTH_7B
  * @retval None
  */
__STATIC_INLINE void DDL_UART_SetDataWidth(USART_TypeDef *UARTx, uint32_t DataWidth)
{
  MODIFY_REG(UARTx->CR1, UART_CR1_M, DataWidth);
}

/**
  * @brief  Return Word length (i.e. nb of data bits, excluding start and stop bits)
  * @param  UARTx UART Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_UART_DATAWIDTH_8B
  *         @arg @ref DDL_UART_DATAWIDTH_9B
  *         @arg @ref DDL_UART_DATAWIDTH_7B
  */
__STATIC_INLINE uint32_t DDL_UART_GetDataWidth(USART_TypeDef *UARTx)
{
  return (uint32_t)(READ_BIT(UARTx->CR1, UART_CR1_M));
}

/**
  * @brief  Receiver Timeout Enable
  * @param  UARTx UART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_UART_EnableRxTimeout(USART_TypeDef *UARTx)
{
  SET_BIT(UARTx->CR1, UART_CR1_RXTODEN);
}

/**
  * @brief  Receiver Timeout Disable
  * @param  UARTx UART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_UART_DisableRxTimeout(USART_TypeDef *UARTx)
{
  CLEAR_BIT(UARTx->CR1, UART_CR1_RXTODEN);
}

/**
  * @brief  Receiver Timeout Disable
  * @param  UARTx UART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_UART_IsEnabledRxTimeout(USART_TypeDef *UARTx)
{
  return (uint32_t)(READ_BIT(UARTx->CR1, UART_CR1_RXTODEN) == UART_CR1_RXTODEN ? 1UL : 0UL);
}

/**
  * @brief  Configure TX/RX pins swapping setting.
  * @param  UARTx UART Instance
  * @param  SwapConfig This parameter can be one of the following values:
  *         @arg @ref DDL_UART_TXRX_STANDARD
  *         @arg @ref DDL_UART_TXRX_SWAPPED
  * @retval None
  */
__STATIC_INLINE void DDL_UART_SetTXRXSwap(USART_TypeDef *UARTx, uint32_t SwapConfig)
{
  MODIFY_REG(UARTx->CR1, UART_CR1_SWAPEN, SwapConfig);
}

/**
  * @brief  Retrieve TX/RX pins swapping configuration.
  * @param  UARTx UART Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_UART_TXRX_STANDARD
  *         @arg @ref DDL_UART_TXRX_SWAPPED
  */
__STATIC_INLINE uint32_t DDL_UART_GetTXRXSwap(USART_TypeDef *UARTx)
{
  return (uint32_t)(READ_BIT(UARTx->CR1, UART_CR1_SWAPEN));
}

/**
  * @brief  Set the length of the stop bits
  * @param  UARTx UART Instance
  * @param  StopBits This parameter can be one of the following values:
  *         @arg @ref DDL_UART_STOPBITS_1
  *         @arg @ref DDL_UART_STOPBITS_2
  * @retval None
  */
__STATIC_INLINE void DDL_UART_SetStopBitsLength(USART_TypeDef *UARTx, uint32_t StopBits)
{
  MODIFY_REG(UARTx->CR2, UART_CR2_STOP, StopBits);
}

/**
  * @brief  Retrieve the length of the stop bits
  * @param  UARTx UART Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_UART_STOPBITS_1
  *         @arg @ref DDL_UART_STOPBITS_2
  */
__STATIC_INLINE uint32_t DDL_UART_GetStopBitsLength(USART_TypeDef *UARTx)
{
  return (uint32_t)(READ_BIT(UARTx->CR2, UART_CR2_STOP));
}

/**
  * @brief  Configure Character frame format (Datawidth, Parity control, Stop Bits)
  * @note   Call of this function is equivalent to following function call sequence :
  *         - Data Width configuration using @ref DDL_UART_SetDataWidth() function
  *         - Parity Control and mode configuration using @ref DDL_UART_SetParity() function
  *         - Stop bits configuration using @ref DDL_UART_SetStopBitsLength() function
  * @param  UARTx UART Instance
  * @param  DataWidth This parameter can be one of the following values:
  *         @arg @ref DDL_UART_DATAWIDTH_8B
  *         @arg @ref DDL_UART_DATAWIDTH_9B
  *         @arg @ref DDL_UART_DATAWIDTH_7B
  * @param  Parity This parameter can be one of the following values:
  *         @arg @ref DDL_UART_PARITY_NONE
  *         @arg @ref DDL_UART_PARITY_EVEN
  *         @arg @ref DDL_UART_PARITY_ODD
  * @param  StopBits This parameter can be one of the following values:
  *         @arg @ref DDL_UART_STOPBITS_1
  *         @arg @ref DDL_UART_STOPBITS_2
  * @retval None
  */
__STATIC_INLINE void DDL_UART_ConfigCharacter(USART_TypeDef *UARTx, uint32_t DataWidth, uint32_t Parity,
                                              uint32_t StopBits)
{
  MODIFY_REG(UARTx->CR1, UART_CR1_PSEL | UART_CR1_PCEN | UART_CR1_M, Parity | DataWidth);
  MODIFY_REG(UARTx->CR2, UART_CR2_STOP, StopBits);
}

/**
  * @brief  Set Receiver Timeout Value Setup.
  * @param  UARTx UART Instance
  * @param  Timeout 24 bit of the UART deassertion.
  * @retval None
  */
__STATIC_INLINE void DDL_UART_SetRxTimeout(USART_TypeDef *UARTx, uint32_t Timeout)
{
  MODIFY_REG(UARTx->RXTOR, UART_RXTOR_RXTO, (Timeout & UART_RXTOR_RXTO));
}

/**
  * @brief  Return 24 bit of the UART deassertion time as set in time of RXTOR.
  * @note   only 24bits (b24-b0) of returned value are relevant
  * @param  UARTx UART Instance
  * @retval Value between Min_Data=0x00 and Max_Data=0x00FFFFFF
  */
__STATIC_INLINE uint32_t DDL_UART_GetRxTimeout(USART_TypeDef *UARTx)
{
  return (uint32_t)(READ_BIT(UARTx->RXTOR, UART_RXTOR_RXTO));
}

/**
  * @brief  Enable One bit sampling method.
  * @param  UARTx UART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_UART_EnableOneBitSamp(USART_TypeDef *UARTx)
{
  SET_BIT(UARTx->CR3, UART_CR3_ONEBIT);
}

/**
  * @brief  Disable One bit sampling method
  * @param  UARTx UART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_UART_DisableOneBitSamp(USART_TypeDef *UARTx)
{
  CLEAR_BIT(UARTx->CR3, UART_CR3_ONEBIT);
}

/**
  * @brief  Indicate if One bit sampling method is enabled
  * @param  UARTx UART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_UART_IsEnabledOneBitSamp(USART_TypeDef *UARTx)
{
  return ((READ_BIT(UARTx->CR3, UART_CR3_ONEBIT) == (UART_CR3_ONEBIT)) ? 1UL : 0UL);
}

/**
  * @brief  Configure UART BRR register for achieving expected Baud Rate value.
  * @note   Compute and set UARTDIV value in BRR Register (full BRR content)
  *         according to used Peripheral Clock, Oversampling mode, and expected Baud Rate values
  * @note   Peripheral clock and Baud rate values provided as function parameters should be valid
  *         (Baud rate value != 0)
  * @param  UARTx UART Instance
  * @param  PeriphClk Peripheral Clock
  * @param  BaudRate Baud Rate
  * @retval None
  */
__STATIC_INLINE void DDL_UART_SetBaudRate(USART_TypeDef *UARTx, uint32_t PeriphClk, uint32_t BaudRate)
{
  UARTx->BRR = (uint16_t)(__DDL_UART_DIV_SAMPLING16(PeriphClk, BaudRate));
}

/**
  * @brief  Return current Baud Rate value, according to UARTDIV present in BRR register
  *         (full BRR content), and to used Peripheral Clock and Oversampling mode values
  * @note   In case of non-initialized or invalid value stored in BRR register, value 0 will be returned.
  * @param  UARTx UART Instance
  * @param  PeriphClk Peripheral Clock
  * @retval Baud Rate
  */
__STATIC_INLINE uint32_t DDL_UART_GetBaudRate(USART_TypeDef *UARTx, uint32_t PeriphClk)
{
  uint32_t uartdiv = 0x0U;
  uint32_t brrresult = 0x0U;

  uartdiv = UARTx->BRR;

  if ((uartdiv & 0xFFFFU) != 0U)
  {
    brrresult = PeriphClk / uartdiv;
  }

  return (brrresult);
}

/**
  * @}
  */

/** @defgroup UART_DDL_EF_Configuration_HalfDuplex Configuration functions related to Half Duplex feature
  * @{
  */

/**
  * @brief  Enable Single Wire Half-Duplex mode
  * @note   Macro @ref IS_UART_HALFDUPLEX_INSTANCE(UARTx) can be used to check whether or not
  *         Half-Duplex mode is supported by the UARTx instance.
  * @param  UARTx UART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_UART_EnableHalfDuplex(USART_TypeDef *UARTx)
{
  SET_BIT(UARTx->CR3, UART_CR3_HDSEL);
}

/**
  * @brief  Disable Single Wire Half-Duplex mode
  * @note   Macro @ref IS_UART_HALFDUPLEX_INSTANCE(UARTx) can be used to check whether or not
  *         Half-Duplex mode is supported by the UARTx instance.
  * @param  UARTx UART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_UART_DisableHalfDuplex(USART_TypeDef *UARTx)
{
  CLEAR_BIT(UARTx->CR3, UART_CR3_HDSEL);
}

/**
  * @brief  Indicate if Single Wire Half-Duplex mode is enabled
  * @note   Macro @ref IS_UART_HALFDUPLEX_INSTANCE(UARTx) can be used to check whether or not
  *         Half-Duplex mode is supported by the UARTx instance.
  * @param  UARTx UART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_UART_IsEnabledHalfDuplex(USART_TypeDef *UARTx)
{
  return (READ_BIT(UARTx->CR3, UART_CR3_HDSEL) == (UART_CR3_HDSEL));
}

/**
  * @}
  */

/** @defgroup UART_DDL_EF_Configuration_LIN Configuration functions related to LIN feature
  * @{
  */

/**
  * @brief  Set LIN Break Detection Length
  * @note   Macro @ref IS_UART_LIN_INSTANCE(UARTx) can be used to check whether or not
  *         LIN feature is supported by the UARTx instance.
  * @param  UARTx UART Instance
  * @param  LINBDLength This parameter can be one of the following values:
  *         @arg @ref DDL_UART_LINBREAK_DETECT_10B
  *         @arg @ref DDL_UART_LINBREAK_DETECT_11B
  * @retval None
  */
__STATIC_INLINE void DDL_UART_SetLINBrkDetectionLen(USART_TypeDef *UARTx, uint32_t LINBDLength)
{
  MODIFY_REG(UARTx->CR2, UART_CR2_LBDL, LINBDLength);
}

/**
  * @brief  Return LIN Break Detection Length
  * @note   Macro @ref IS_UART_LIN_INSTANCE(UARTx) can be used to check whether or not
  *         LIN feature is supported by the UARTx instance.
  * @param  UARTx UART Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_UART_LINBREAK_DETECT_10B
  *         @arg @ref DDL_UART_LINBREAK_DETECT_11B
  */
__STATIC_INLINE uint32_t DDL_UART_GetLINBrkDetectionLen(USART_TypeDef *UARTx)
{
  return (uint32_t)(READ_BIT(UARTx->CR2, UART_CR2_LBDL));
}

/**
  * @brief  Enable LIN mode
  * @note   Macro @ref IS_UART_LIN_INSTANCE(UARTx) can be used to check whether or not
  *         LIN feature is supported by the UARTx instance.
  * @param  UARTx UART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_UART_EnableLIN(USART_TypeDef *UARTx)
{
  SET_BIT(UARTx->CR2, UART_CR2_LINEN);
}

/**
  * @brief  Disable LIN mode
  * @note   Macro @ref IS_UART_LIN_INSTANCE(UARTx) can be used to check whether or not
  *         LIN feature is supported by the UARTx instance.
  * @param  UARTx UART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_UART_DisableLIN(USART_TypeDef *UARTx)
{
  CLEAR_BIT(UARTx->CR2, UART_CR2_LINEN);
}

/**
  * @brief  Indicate if LIN mode is enabled
  * @note   Macro @ref IS_UART_LIN_INSTANCE(UARTx) can be used to check whether or not
  *         LIN feature is supported by the UARTx instance.
  * @param  UARTx UART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_UART_IsEnabledLIN(USART_TypeDef *UARTx)
{
  return (READ_BIT(UARTx->CR2, UART_CR2_LINEN) == (UART_CR2_LINEN));
}

/**
  * @}
  */

/** @defgroup UART_DDL_EF_AutomaticBaudRate Automatic Baud Rate
  * @{
  */

/**
  * @brief  Enable Auto Baud-Rate Detection.
  * @param  UARTx UART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_UART_EnableAutoBaudRate(USART_TypeDef *UARTx)
{
  SET_BIT(UARTx->CR2, UART_CR2_ABEN);
}

/**
  * @brief  Disable Auto Baud-Rate Detection
  * @param  UARTx UART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_UART_DisableAutoBaudRate(USART_TypeDef *UARTx)
{
  CLEAR_BIT(UARTx->CR2, UART_CR2_ABEN);
}

/**
  * @brief  Indicate if Auto Baud-Rate Detection mechanism is enabled
  * @param  UARTx UART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_UART_IsEnabledAutoBaud(USART_TypeDef *UARTx)
{
  return ((READ_BIT(UARTx->CR2, UART_CR2_ABEN) == (UART_CR2_ABEN)) ? 1UL : 0UL);
}

/**
  * @}
  */

/** @defgroup UART_DDL_EF_AdvancedConfiguration Advanced Configurations services
  * @{
  */

/**
  * @brief  Perform basic configuration of UART for enabling use in LIN Mode
  * @note   In LIN mode, the following bits must be kept cleared:
  *           - STOP and CLKEN bits in the UART_CTRL2 register,
  *           - HDSEL bit in the UART_CTRL3 register.
  *         This function also set the UART/UART in LIN mode.
  * @note   Macro @ref IS_UART_LIN_INSTANCE(UARTx) can be used to check whether or not
  *         LIN feature is supported by the UARTx instance.
  * @note   Call of this function is equivalent to following function call sequence :
  *         - Clear CLKEN in CTRL2 using @ref DDL_UART_DisableSCLKOutput() function
  *         - Clear STOP in CTRL2 using @ref DDL_UART_SetStopBitsLength() function
  *         - Clear HDSEL in CTRL3 using @ref DDL_UART_DisableHalfDuplex() function
  *         - Set LINEN in CTRL2 using @ref DDL_UART_EnableLIN() function
  * @note   Other remaining configurations items related to LIN Mode
  *         (as Baud Rate, Word length, LIN Break Detection Length, ...) should be set using
  *         dedicated functions
  * @param  UARTx UART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_UART_ConfigLINMode(USART_TypeDef *UARTx)
{
  /* In LIN mode, the following bits must be kept cleared:
  - STOP and CLKEN bits in the UART_CTRL2 register,
  - HDSEL bits in the UART_CTRL3 register.*/
  CLEAR_BIT(UARTx->CR2, (UART_CR2_STOP));
  CLEAR_BIT(UARTx->CR3, (UART_CR3_HDSEL));
  /* Set the UART/UART in LIN mode */
  SET_BIT(UARTx->CR2, UART_CR2_LINEN);
}

/**
  * @brief  Perform basic configuration of UART for enabling use in Half Duplex Mode
  * @note   In Half Duplex mode, the following bits must be kept cleared:
  *           - LINEN bit in the UART_CTRL2 register,
  *           - CLKEN bit in the UART_CTRL2 register,
  *         This function also sets the UART/UART in Half Duplex mode.
  * @note   Macro @ref IS_UART_HALFDUPLEX_INSTANCE(UARTx) can be used to check whether or not
  *         Half-Duplex mode is supported by the UARTx instance.
  * @note   Call of this function is equivalent to following function call sequence :
  *         - Clear LINEN in CTRL2 using @ref DDL_UART_DisableLIN() function
  *         - Clear CLKEN in CTRL2 using @ref DDL_UART_DisableSCLKOutput() function
  *         - Set HDSEL in CTRL3 using @ref DDL_UART_EnableHalfDuplex() function
  * @note   Other remaining configurations items related to Half Duplex Mode
  *         (as Baud Rate, Word length, Parity, ...) should be set using
  *         dedicated functions
  * @param  UARTx UART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_UART_ConfigHalfDuplexMode(USART_TypeDef *UARTx)
{
  /* In Half Duplex mode, the following bits must be kept cleared:
  - LINEN and CLKEN bits in the UART_CTRL2 register */
  CLEAR_BIT(UARTx->CR2, (UART_CR2_LINEN));
  /* set the UART/UART in Half Duplex mode */
  SET_BIT(UARTx->CR3, UART_CR3_HDSEL);
}

/**
  * @brief  Perform basic configuration of UART for enabling use in Multi processor Mode
  *         (several UARTs connected in a network, one of the UARTs can be the master,
  *         its TX output connected to the RX inputs of the other slaves UARTs).
  * @note   In MultiProcessor mode, the following bits must be kept cleared:
  *           - LINEN bit in the UART_CTRL2 register,
  *           - CLKEN bit in the UART_CTRL2 register,
  *           - HDSEL bit in the UART_CTRL3 register.
  * @note   Call of this function is equivalent to following function call sequence :
  *         - Clear LINEN in CTRL2 using @ref DDL_UART_DisableLIN() function
  *         - Clear CLKEN in CTRL2 using @ref DDL_UART_DisableSCLKOutput() function
  *         - Clear HDSEL in CTRL3 using @ref DDL_UART_DisableHalfDuplex() function
  * @note   Other remaining configurations items related to Multi processor Mode
  *         (as Baud Rate, Wake Up Method, Node address, ...) should be set using
  *         dedicated functions
  * @param  UARTx UART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_UART_ConfigMultiProcessMode(USART_TypeDef *UARTx)
{
  /* In Multi Processor mode, the following bits must be kept cleared:
  - LINEN and CLKEN bits in the UART_CTRL2 register,
  - IREN, SCEN and HDSEL bits in the UART_CTRL3 register.*/
  CLEAR_BIT(UARTx->CR2, (UART_CR2_LINEN));
  CLEAR_BIT(UARTx->CR3, (UART_CR3_HDSEL));
}

/**
  * @}
  */

/** @defgroup UART_DDL_EF_FLAG_Management FLAG_Management
  * @{
  */

/**
  * @brief  Check if the UART Parity Error Flag is set or not
  * @param  UARTx UART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_UART_IsActiveFlag_PE(USART_TypeDef *UARTx)
{
  return (READ_BIT(UARTx->SR, UART_SR_PEFLG) == (UART_SR_PEFLG));
}

/**
  * @brief  Check if the UART Framing Error Flag is set or not
  * @param  UARTx UART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_UART_IsActiveFlag_FE(USART_TypeDef *UARTx)
{
  return (READ_BIT(UARTx->SR, UART_SR_FEFLG) == (UART_SR_FEFLG));
}

/**
  * @brief  Check if the UART Noise error detected Flag is set or not
  * @param  UARTx UART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_UART_IsActiveFlag_NE(USART_TypeDef *UARTx)
{
  return (READ_BIT(UARTx->SR, UART_SR_NEFLG) == (UART_SR_NEFLG));
}

/**
  * @brief  Check if the UART OverRun Error Flag is set or not
  * @param  UARTx UART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_UART_IsActiveFlag_ORE(USART_TypeDef *UARTx)
{
  return (READ_BIT(UARTx->SR, UART_SR_OREFLG) == (UART_SR_OREFLG));
}

/**
  * @brief  Check if the UART IDLE line detected Flag is set or not
  * @param  UARTx UART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_UART_IsActiveFlag_IDLE(USART_TypeDef *UARTx)
{
  return (READ_BIT(UARTx->SR, UART_SR_IDLEFLG) == (UART_SR_IDLEFLG));
}

/**
  * @brief  Check if the UART Read Data Register Not Empty Flag is set or not
  * @param  UARTx UART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_UART_IsActiveFlag_RXNE(USART_TypeDef *UARTx)
{
  return (READ_BIT(UARTx->SR, UART_SR_RXNEFLG) == (UART_SR_RXNEFLG));
}

/**
  * @brief  Check if the UART Transmission Complete Flag is set or not
  * @param  UARTx UART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_UART_IsActiveFlag_TC(USART_TypeDef *UARTx)
{
  return (READ_BIT(UARTx->SR, UART_SR_TCFLG) == (UART_SR_TCFLG));
}

/**
  * @brief  Check if the UART Transmit Data Register Empty Flag is set or not
  * @param  UARTx UART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_UART_IsActiveFlag_TXE(USART_TypeDef *UARTx)
{
  return (READ_BIT(UARTx->SR, UART_SR_TXEFLG) == (UART_SR_TXEFLG));
}

/**
  * @brief  Check if the UART LIN Break Detection Flag is set or not
  * @note   Macro @ref IS_UART_LIN_INSTANCE(UARTx) can be used to check whether or not
  *         LIN feature is supported by the UARTx instance.
  * @param  UARTx UART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_UART_IsActiveFlag_LBD(USART_TypeDef *UARTx)
{
  return (READ_BIT(UARTx->SR, UART_SR_LBDFLG) == (UART_SR_LBDFLG));
}

/**
  * @brief  Check if the UART Receive overtime flag is set or not
  * @param  UARTx UART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_UART_IsActiveFlag_RTO(USART_TypeDef *UARTx)
{
  return (READ_BIT(UARTx->SR, UART_SR_RXTOFLG) == (UART_SR_RXTOFLG));
}

/**
  * @brief  Check if the UART Automatic baud rate complete flag is set or not
  * @param  UARTx UART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_UART_IsActiveFlag_ABR(USART_TypeDef *UARTx)
{
  return (READ_BIT(UARTx->SR, UART_SR_ABCFLG) == (UART_SR_ABCFLG));
}

/**
  * @brief  Check if the UART Automatic baud rate error flag is set or not
  * @param  UARTx UART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_UART_IsActiveFlag_ABRE(USART_TypeDef *UARTx)
{
  return (READ_BIT(UARTx->SR, UART_SR_ABERRFLG) == (UART_SR_ABERRFLG));
}

/**
  * @brief  Check if the UART Send Break Flag is set or not
  * @param  UARTx UART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_UART_IsActiveFlag_SBK(USART_TypeDef *UARTx)
{
  return (READ_BIT(UARTx->CR1, UART_CR1_SBK) == (UART_CR1_SBK));
}

/**
  * @brief  Clear Parity Error Flag
  * @note   Clearing this flag is done by a read access to the UARTx_SR
  *         register followed by a read access to the UARTx_DR register.
  * @note   Please also consider that when clearing this flag, other flags as
  *         NE, FE, ORE, IDLE would also be cleared.
  * @param  UARTx UART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_UART_ClearFlag_PE(USART_TypeDef *UARTx)
{
  __IO uint32_t tmpreg;
  tmpreg = UARTx->SR;
  (void) tmpreg;
  tmpreg = UARTx->DR;
  (void) tmpreg;
}

/**
  * @brief  Clear Framing Error Flag
  * @note   Clearing this flag is done by a read access to the UARTx_SR
  *         register followed by a read access to the UARTx_DR register.
  * @note   Please also consider that when clearing this flag, other flags as
  *         PE, NE, ORE, IDLE would also be cleared.
  * @param  UARTx UART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_UART_ClearFlag_FE(USART_TypeDef *UARTx)
{
  __IO uint32_t tmpreg;
  tmpreg = UARTx->SR;
  (void) tmpreg;
  tmpreg = UARTx->DR;
  (void) tmpreg;
}

/**
  * @brief  Clear Noise detected Flag
  * @note   Clearing this flag is done by a read access to the UARTx_SR
  *         register followed by a read access to the UARTx_DR register.
  * @note   Please also consider that when clearing this flag, other flags as
  *         PE, FE, ORE, IDLE would also be cleared.
  * @param  UARTx UART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_UART_ClearFlag_NE(USART_TypeDef *UARTx)
{
  __IO uint32_t tmpreg;
  tmpreg = UARTx->SR;
  (void) tmpreg;
  tmpreg = UARTx->DR;
  (void) tmpreg;
}

/**
  * @brief  Clear OverRun Error Flag
  * @note   Clearing this flag is done by a read access to the UARTx_SR
  *         register followed by a read access to the UARTx_DR register.
  * @note   Please also consider that when clearing this flag, other flags as
  *         PE, NE, FE, IDLE would also be cleared.
  * @param  UARTx UART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_UART_ClearFlag_ORE(USART_TypeDef *UARTx)
{
  __IO uint32_t tmpreg;
  tmpreg = UARTx->SR;
  (void) tmpreg;
  tmpreg = UARTx->DR;
  (void) tmpreg;
}

/**
  * @brief  Clear IDLE line detected Flag
  * @note   Clearing this flag is done by a read access to the UARTx_SR
  *         register followed by a read access to the UARTx_DR register.
  * @note   Please also consider that when clearing this flag, other flags as
  *         PE, NE, FE, ORE would also be cleared.
  * @param  UARTx UART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_UART_ClearFlag_IDLE(USART_TypeDef *UARTx)
{
  __IO uint32_t tmpreg;
  tmpreg = UARTx->SR;
  (void) tmpreg;
  tmpreg = UARTx->DR;
  (void) tmpreg;
}

/**
  * @brief  Clear Transmission Complete Flag
  * @param  UARTx UART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_UART_ClearFlag_TC(USART_TypeDef *UARTx)
{
  CLEAR_BIT(UARTx->SR, UART_SR_TCFLG);
}

/**
  * @brief  Clear RX Not Empty Flag
  * @param  UARTx UART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_UART_ClearFlag_RXNE(USART_TypeDef *UARTx)
{
  CLEAR_BIT(UARTx->SR, UART_SR_RXNEFLG);
}

/**
  * @brief  Clear LIN Break Detection Flag
  * @note   Macro @ref IS_UART_LIN_INSTANCE(UARTx) can be used to check whether or not
  *         LIN feature is supported by the UARTx instance.
  * @param  UARTx UART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_UART_ClearFlag_LBD(USART_TypeDef *UARTx)
{
  CLEAR_BIT(UARTx->SR, UART_SR_LBDFLG);
}

/**
  * @brief  Clear RX Time out Flag
  * @param  UARTx UART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_UART_ClearFlag_RTO(USART_TypeDef *UARTx)
{
  CLEAR_BIT(UARTx->SR, UART_SR_RXTOFLG);
}

/**
  * @brief  Clear  Automatic baud rate complete Flag
  * @param  UARTx UART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_UART_ClearFlag_ABR(USART_TypeDef *UARTx)
{
  CLEAR_BIT(UARTx->SR, UART_SR_ABCFLG);
}

/**
  * @brief  Clear  Automatic baud rate Flag
  * @param  UARTx UART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_UART_ClearFlag_ABRE(USART_TypeDef *UARTx)
{
  CLEAR_BIT(UARTx->SR, UART_SR_ABERRFLG);
}

/**
  * @}
  */

/** @defgroup UART_DDL_EF_IT_Management IT_Management
  * @{
  */

/**
  * @brief  Enable IDLE Interrupt
  * @param  UARTx UART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_UART_EnableIT_IDLE(USART_TypeDef *UARTx)
{
  SET_BIT(UARTx->CR1, UART_CR1_IDLEIEN);
}

/**
  * @brief  Enable RX Not Empty Interrupt
  * @param  UARTx UART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_UART_EnableIT_RXNE(USART_TypeDef *UARTx)
{
  SET_BIT(UARTx->CR1, UART_CR1_RXNEIEN);
}

/**
  * @brief  Enable Transmission Complete Interrupt
  * @param  UARTx UART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_UART_EnableIT_TC(USART_TypeDef *UARTx)
{
  SET_BIT(UARTx->CR1, UART_CR1_TCIEN);
}

/**
  * @brief  Enable TX Empty Interrupt
  * @param  UARTx UART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_UART_EnableIT_TXE(USART_TypeDef *UARTx)
{
  SET_BIT(UARTx->CR1, UART_CR1_TXEIEN);
}

/**
  * @brief  Enable Parity Error Interrupt
  * @param  UARTx UART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_UART_EnableIT_PE(USART_TypeDef *UARTx)
{
  SET_BIT(UARTx->CR1, UART_CR1_PEIEN);
}

/**
  * @brief  Receiver Timeout Interrupt Enable
  * @param  UARTx UART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_UART_EnableIT_RTO(USART_TypeDef *UARTx)
{
  SET_BIT(UARTx->CR1, UART_CR1_RXTOIEN);
}

/**
  * @brief  Enable LIN Break Detection Interrupt
  * @note   Macro @ref IS_UART_LIN_INSTANCE(UARTx) can be used to check whether or not
  *         LIN feature is supported by the UARTx instance.
  * @param  UARTx UART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_UART_EnableIT_LBD(USART_TypeDef *UARTx)
{
  SET_BIT(UARTx->CR2, UART_CR2_LBDIEN);
}

/**
  * @brief  Enable Automatic baud rate complete Interrupt
  * @param  UARTx UART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_UART_EnableIT_ABR(USART_TypeDef *UARTx)
{
  SET_BIT(UARTx->CR2, UART_CR2_ABCIEN);
}

/**
  * @brief  Enable Automatic baud rate detection error Interrupt
  * @param  UARTx UART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_UART_EnableIT_ABRE(USART_TypeDef *UARTx)
{
  SET_BIT(UARTx->CR2, UART_CR2_ABEIEN);
}

/**
  * @brief  Enable Error Interrupt
  * @note   When set, Error Interrupt Enable Bit is enabling interrupt generation in case of a framing
  *         error, overrun error or noise flag (FE=1 or ORE=1 or NF=1 in the UARTx_SR register).
  *           0: Interrupt is inhibited
  *           1: An interrupt is generated when FE=1 or ORE=1 or NF=1 in the UARTx_SR register.
  * @param  UARTx UART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_UART_EnableIT_ERROR(USART_TypeDef *UARTx)
{
  SET_BIT(UARTx->CR3, UART_CR3_EIEN);
}

/**
  * @brief  Disable IDLE Interrupt
  * @param  UARTx UART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_UART_DisableIT_IDLE(USART_TypeDef *UARTx)
{
  CLEAR_BIT(UARTx->CR1, UART_CR1_IDLEIEN);
}

/**
  * @brief  Disable RX Not Empty Interrupt
  * @param  UARTx UART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_UART_DisableIT_RXNE(USART_TypeDef *UARTx)
{
  CLEAR_BIT(UARTx->CR1, UART_CR1_RXNEIEN);
}

/**
  * @brief  Disable Transmission Complete Interrupt
  * @param  UARTx UART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_UART_DisableIT_TC(USART_TypeDef *UARTx)
{
  CLEAR_BIT(UARTx->CR1, UART_CR1_TCIEN);
}

/**
  * @brief  Disable TX Empty Interrupt
  * @param  UARTx UART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_UART_DisableIT_TXE(USART_TypeDef *UARTx)
{
  CLEAR_BIT(UARTx->CR1, UART_CR1_TXEIEN);
}

/**
  * @brief  Disable Parity Error Interrupt
  * @param  UARTx UART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_UART_DisableIT_PE(USART_TypeDef *UARTx)
{
  CLEAR_BIT(UARTx->CR1, UART_CR1_PEIEN);
}

/**
  * @brief  Receiver Timeout Interrupt Disable
  * @param  UARTx UART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_UART_DisableIT_RTO(USART_TypeDef *UARTx)
{
  CLEAR_BIT(UARTx->CR1, UART_CR1_RXTOIEN);
}

/**
  * @brief  Disable LIN Break Detection Interrupt
  * @note   Macro @ref IS_UART_LIN_INSTANCE(UARTx) can be used to check whether or not
  *         LIN feature is supported by the UARTx instance.
  * @param  UARTx UART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_UART_DisableIT_LBD(USART_TypeDef *UARTx)
{
  CLEAR_BIT(UARTx->CR2, UART_CR2_LBDIEN);
}

/**
  * @brief  Disable Automatic bps complete Interrupt
  * @param  UARTx UART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_UART_DisableIT_ABR(USART_TypeDef *UARTx)
{
  CLEAR_BIT(UARTx->CR2, UART_CR2_ABCIEN);
}

/**
  * @brief  Disable Automatic bps detection error Interrupt
  * @param  UARTx UART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_UART_DisableIT_ABRE(USART_TypeDef *UARTx)
{
  CLEAR_BIT(UARTx->CR2, UART_CR2_ABEIEN);
}

/**
  * @brief  Disable Error Interrupt
  * @note   When set, Error Interrupt Enable Bit is enabling interrupt generation in case of a framing
  *         error, overrun error or noise flag (FE=1 or ORE=1 or NF=1 in the UARTx_SR register).
  *           0: Interrupt is inhibited
  *           1: An interrupt is generated when FE=1 or ORE=1 or NF=1 in the UARTx_SR register.
  * @param  UARTx UART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_UART_DisableIT_ERROR(USART_TypeDef *UARTx)
{
  CLEAR_BIT(UARTx->CR3, UART_CR3_EIEN);
}

/**
  * @brief  Check if the UART IDLE Interrupt  source is enabled or disabled.
  * @param  UARTx UART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_UART_IsEnabledIT_IDLE(USART_TypeDef *UARTx)
{
  return (READ_BIT(UARTx->CR1, UART_CR1_IDLEIEN) == (UART_CR1_IDLEIEN));
}

/**
  * @brief  Check if the UART RX Not Empty Interrupt is enabled or disabled.
  * @param  UARTx UART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_UART_IsEnabledIT_RXNE(USART_TypeDef *UARTx)
{
  return (READ_BIT(UARTx->CR1, UART_CR1_RXNEIEN) == (UART_CR1_RXNEIEN));
}

/**
  * @brief  Check if the UART Transmission Complete Interrupt is enabled or disabled.
  * @param  UARTx UART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_UART_IsEnabledIT_TC(USART_TypeDef *UARTx)
{
  return (READ_BIT(UARTx->CR1, UART_CR1_TCIEN) == (UART_CR1_TCIEN));
}

/**
  * @brief  Check if the UART TX Empty Interrupt is enabled or disabled.
  * @param  UARTx UART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_UART_IsEnabledIT_TXE(USART_TypeDef *UARTx)
{
  return (READ_BIT(UARTx->CR1, UART_CR1_TXEIEN) == (UART_CR1_TXEIEN));
}

/**
  * @brief  Check if the UART Parity Error Interrupt is enabled or disabled.
  * @param  UARTx UART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_UART_IsEnabledIT_PE(USART_TypeDef *UARTx)
{
  return (READ_BIT(UARTx->CR1, UART_CR1_PEIEN) == (UART_CR1_PEIEN));
}

/**
  * @brief  Check if the UART LIN Break Detection Interrupt is enabled or disabled.
  * @note   Macro @ref IS_UART_LIN_INSTANCE(UARTx) can be used to check whether or not
  *         LIN feature is supported by the UARTx instance.
  * @param  UARTx UART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_UART_IsEnabledIT_LBD(USART_TypeDef *UARTx)
{
  return (READ_BIT(UARTx->CR2, UART_CR2_LBDIEN) == (UART_CR2_LBDIEN));
}

/**
  * @brief  Check if the UART Automatic baud rate complete Interrupt is enabled or disabled.
  * @param  UARTx UART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_UART_IsEnabledIT_ABR(USART_TypeDef *UARTx)
{
  return (READ_BIT(UARTx->CR2, UART_CR2_ABCIEN) == (UART_CR2_ABCIEN));
}

/**
  * @brief  Check if the UART Automatic baud rate error Interrupt is enabled or disabled.
  * @param  UARTx UART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_UART_IsEnabledIT_ABRE(USART_TypeDef *UARTx)
{
  return (READ_BIT(UARTx->CR2, UART_CR2_ABEIEN) == (UART_CR2_ABEIEN));
}

/**
  * @}
  */

/** @defgroup UART_DDL_EF_DMA_Management DMA_Management
  * @{
  */

/**
  * @brief  Enable DMA Mode for reception
  * @param  UARTx UART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_UART_EnableDMAReq_RX(USART_TypeDef *UARTx)
{
  SET_BIT(UARTx->CR3, UART_CR3_DMAR);
}

/**
  * @brief  Disable DMA Mode for reception
  * @param  UARTx UART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_UART_DisableDMAReq_RX(USART_TypeDef *UARTx)
{
  CLEAR_BIT(UARTx->CR3, UART_CR3_DMAR);
}

/**
  * @brief  Check if DMA Mode is enabled for reception
  * @param  UARTx UART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_UART_IsEnabledDMAReq_RX(USART_TypeDef *UARTx)
{
  return (READ_BIT(UARTx->CR3, UART_CR3_DMAR) == (UART_CR3_DMAR));
}

/**
  * @brief  Enable DMA Mode for transmission
  * @param  UARTx UART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_UART_EnableDMAReq_TX(USART_TypeDef *UARTx)
{
  SET_BIT(UARTx->CR3, UART_CR3_DMAT);
}

/**
  * @brief  Disable DMA Mode for transmission
  * @param  UARTx UART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_UART_DisableDMAReq_TX(USART_TypeDef *UARTx)
{
  CLEAR_BIT(UARTx->CR3, UART_CR3_DMAT);
}

/**
  * @brief  Check if DMA Mode is enabled for transmission
  * @param  UARTx UART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_UART_IsEnabledDMAReq_TX(USART_TypeDef *UARTx)
{
  return (READ_BIT(UARTx->CR3, UART_CR3_DMAT) == (UART_CR3_DMAT));
}

/**
  * @brief  Get the data register address used for DMA transfer
  * @note   Address of Data Register is valid for both Transmit and Receive transfers.
  * @param  UARTx UART Instance
  * @retval Address of data register
  */
__STATIC_INLINE uint32_t DDL_UART_DMA_GetRegAddr(USART_TypeDef *UARTx)
{
  /* return address of DR register */
  return ((uint32_t) &(UARTx->DR));
}

/**
  * @}
  */

/** @defgroup UART_DDL_EF_Data_Management Data_Management
  * @{
  */

/**
  * @brief  Read Receiver Data register (Receive Data value, 8 bits)
  * @param  UARTx UART Instance
  * @retval Value between Min_Data=0x00 and Max_Data=0xFF
  */
__STATIC_INLINE uint8_t DDL_UART_ReceiveData8(USART_TypeDef *UARTx)
{
  return (uint8_t)(READ_BIT(UARTx->DR, UART_DR_DATA));
}

/**
  * @brief  Read Receiver Data register (Receive Data value, 9 bits)
  * @param  UARTx UART Instance
  * @retval Value between Min_Data=0x00 and Max_Data=0x1FF
  */
__STATIC_INLINE uint16_t DDL_UART_ReceiveData9(USART_TypeDef *UARTx)
{
  return (uint16_t)(READ_BIT(UARTx->DR, UART_DR_DATA));
}

/**
  * @brief  Write in Transmitter Data Register (Transmit Data value, 8 bits)
  * @param  UARTx UART Instance
  * @param  Value between Min_Data=0x00 and Max_Data=0xFF
  * @retval None
  */
__STATIC_INLINE void DDL_UART_TransmitData8(USART_TypeDef *UARTx, uint8_t Value)
{
  UARTx->DR = Value;
}

/**
  * @brief  Write in Transmitter Data Register (Transmit Data value, 9 bits)
  * @param  UARTx UART Instance
  * @param  Value between Min_Data=0x00 and Max_Data=0x1FF
  * @retval None
  */
__STATIC_INLINE void DDL_UART_TransmitData9(USART_TypeDef *UARTx, uint16_t Value)
{
  UARTx->DR = Value & 0x1FFU;
}

/**
  * @}
  */

/** @defgroup UART_DDL_EF_Execution Execution
  * @{
  */

/**
  * @brief  Request Break sending
  * @param  UARTx UART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_UART_RequestBreakSending(USART_TypeDef *UARTx)
{
  SET_BIT(UARTx->CR1, UART_CR1_SBK);
}

/**
  * @}
  */

#if defined(USE_FULL_DDL_DRIVER)
/** @defgroup UART_DDL_EF_Init Initialization and de-initialization functions
  * @{
  */
ErrorStatus DDL_UART_DeInit(USART_TypeDef *UARTx);
ErrorStatus DDL_UART_Init(USART_TypeDef *UARTx, DDL_UART_InitTypeDef *UART_InitStruct);
void        DDL_UART_StructInit(DDL_UART_InitTypeDef *UART_InitStruct);
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

#endif /* UART */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* G32F031_DDL_UART_H */

