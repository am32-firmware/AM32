/**
  *
  * @file    g32f031_ddl_usart.h
  * @brief   Header file of USART DDL module.
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
#ifndef G32F031_DDL_USART_H
#define G32F031_DDL_USART_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "g32f0xx.h"

/** @addtogroup G32F031_DDL_Driver
  * @{
  */

#if defined (USART)

/** @defgroup USART_DDL USART
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
#if defined(USE_FULL_DDL_DRIVER)
/** @defgroup USART_DDL_ES_INIT USART Exported Init structures
  * @{
  */

/**
  * @brief DDL USART Init Structure definition
  */
typedef struct
{
  uint32_t BaudRate;                  /*!< This field defines expected Usart communication baud rate.

                                           This feature can be modified afterwards using unitary function @ref DDL_USART_SetBaudRate().*/

  uint32_t DataWidth;                 /*!< Specifies the number of data bits transmitted or received in a frame.
                                           This parameter can be a value of @ref USART_DDL_EC_DATAWIDTH.

                                           This feature can be modified afterwards using unitary function @ref DDL_USART_SetDataWidth().*/

  uint32_t StopBits;                  /*!< Specifies the number of stop bits transmitted.
                                           This parameter can be a value of @ref USART_DDL_EC_STOPBITS.

                                           This feature can be modified afterwards using unitary function @ref DDL_USART_SetStopBitsLength().*/

  uint32_t Parity;                    /*!< Specifies the parity mode.
                                           This parameter can be a value of @ref USART_DDL_EC_PARITY.

                                           This feature can be modified afterwards using unitary function @ref DDL_USART_SetParity().*/

  uint32_t TransferDirection;         /*!< Specifies whether the Receive and/or Transmit mode is enabled or disabled.
                                           This parameter can be a value of @ref USART_DDL_EC_DIRECTION.

                                           This feature can be modified afterwards using unitary function @ref DDL_USART_SetTransferDirection().*/

  uint32_t HardwareFlowControl;       /*!< Specifies whether the hardware flow control mode is enabled or disabled.
                                           This parameter can be a value of @ref USART_DDL_EC_HWCONTROL.

                                           This feature can be modified afterwards using unitary function @ref DDL_USART_SetHWFlowCtrl().*/

  uint32_t OverSampling;              /*!< Specifies whether USART oversampling mode is 16 or 8.
                                           This parameter can be a value of @ref USART_DDL_EC_OVERSAMPLING.

                                           This feature can be modified afterwards using unitary function @ref DDL_USART_SetOverSampling().*/

} DDL_USART_InitTypeDef;

/**
  * @brief DDL USART Clock Init Structure definition
  */
typedef struct
{
  uint32_t ClockOutput;               /*!< Specifies whether the USART clock is enabled or disabled.
                                           This parameter can be a value of @ref USART_DDL_EC_CLOCK.

                                           USART HW configuration can be modified afterwards using unitary functions
                                           @ref DDL_USART_EnableSCLKOutput() or @ref DDL_USART_DisableSCLKOutput().
                                           For more details, refer to description of this function. */

  uint32_t ClockPolarity;             /*!< Specifies the steady state of the serial clock.
                                           This parameter can be a value of @ref USART_DDL_EC_POLARITY.

                                           USART HW configuration can be modified afterwards using unitary functions @ref DDL_USART_SetClockPolarity().
                                           For more details, refer to description of this function. */

  uint32_t ClockPhase;                /*!< Specifies the clock transition on which the bit capture is made.
                                           This parameter can be a value of @ref USART_DDL_EC_PHASE.

                                           USART HW configuration can be modified afterwards using unitary functions @ref DDL_USART_SetClockPhase().
                                           For more details, refer to description of this function. */

  uint32_t LastBitClockPulse;         /*!< Specifies whether the clock pulse corresponding to the last transmitted
                                           data bit (MSB) has to be output on the SCLK pin in synchronous mode.
                                           This parameter can be a value of @ref USART_DDL_EC_LASTCLKPULSE.

                                           USART HW configuration can be modified afterwards using unitary functions @ref DDL_USART_SetLastClkPulseOutput().
                                           For more details, refer to description of this function. */

} DDL_USART_ClockInitTypeDef;

/**
  * @}
  */
#endif /* USE_FULL_DDL_DRIVER */

/* Exported constants --------------------------------------------------------*/
/** @defgroup USART_DDL_Exported_Constants USART Exported Constants
  * @{
  */

/** @defgroup USART_DDL_EC_GET_FLAG Get Flags Defines
  * @brief    Flags defines which can be used with DDL_USART_ReadReg function
  * @{
  */
#define DDL_USART_SR_PEFLGFLG                          USART_SR_PEFLG                   /*!< Parity error flag */
#define DDL_USART_SR_FEFLGFLG                          USART_SR_FEFLG                   /*!< Framing error flag */
#define DDL_USART_SR_NEFLGFLG                          USART_SR_NEFLG                   /*!< Noise detected flag */
#define DDL_USART_SR_OVREFLG                        USART_SR_OREFLG                  /*!< Overrun error flag */
#define DDL_USART_SR_IDLEFLGFLG                        USART_SR_IDLEFLG                 /*!< Idle line detected flag */
#define DDL_USART_SR_RXBNEFLG                       USART_SR_RXNEFLG                 /*!< Read data register not empty flag */
#define DDL_USART_SR_TXCFLG                         USART_SR_TCFLG                   /*!< Transmission complete flag */
#define DDL_USART_SR_TXBEFLG                        USART_SR_TXEFLG                  /*!< Transmit data register empty flag */
#define DDL_USART_SR_LBDFLGFLG                         USART_SR_LBDFLG                  /*!< LIN break detection flag */
#define DDL_USART_SR_CTSFLGFLG                         USART_SR_CTSFLG                  /*!< CTS flag */
#define DDL_USART_SR_RXTOFLGFLG                       USART_SR_RXTOFLG                /*!< Receive overtime flag */
#define DDL_USART_SR_ABCFLGFLG                        USART_SR_ABCFLG                 /*!< Automatic baud rate complete Flag */
#define DDL_USART_SR_ABERRFLGFLG                       USART_SR_ABERRFLG                /*!< Automatic baud rate error Flag */
#define DDL_USART_SR_CMFLGFLG                         USART_SR_CMFLG                  /*!< Character matching flag */
/**
  * @}
  */

/** @defgroup USART_DDL_EC_IT IT Defines
  * @brief    IT defines which can be used with DDL_USART_ReadReg and  DDL_USART_WriteReg functions
  * @{
  */
#define DDL_USART_CTRL1_IDLEIEN                     USART_CR1_IDLEIEN              /*!< IDLE interrupt enable */
#define DDL_USART_CTRL1_RXBNEIEN                    USART_CR1_RXNEIEN              /*!< Read data register not empty interrupt enable */
#define DDL_USART_CTRL1_TXCIEN                      USART_CR1_TCIEN                /*!< Transmission complete interrupt enable */
#define DDL_USART_CTRL1_TXBEIEN                     USART_CR1_TXEIEN               /*!< Transmit data register empty interrupt enable */
#define DDL_USART_CTRL1_PEIEN                       USART_CR1_PEIEN                /*!< Parity error */
#define DDL_USART_CTRL1_RXTOIEN                     USART_CR1_RXTOIEN              /*!< Receive overtime interrupt enable */
#define DDL_USART_CTRL1_CMIEN                       USART_CR1_CMIEN                /*!< Character matching interrupt enable */
#define DDL_USART_CTRL2_LBDIEN                      USART_CR2_LBDIEN               /*!< LIN break detection interrupt enable */
#define DDL_USART_CTRL2_ABCIEN                      USART_CR2_ABCIEN               /*!< Automatic baud rate complete interrupt enable */
#define DDL_USART_CTRL2_ABEIEN                      USART_CR2_ABEIEN               /*!< Automatic baud rate error interrupt enable */
#define DDL_USART_CTRL3_ERRIEN                      USART_CR3_EIEN                 /*!< Error interrupt enable */
#define DDL_USART_CTRL3_CTSIEN                      USART_CR3_CTSIEN               /*!< CTS interrupt enable */
/**
  * @}
  */

/** @defgroup USART_DDL_EC_DIRECTION Communication Direction
  * @{
  */
#define DDL_USART_DIRECTION_NONE                 0x00000000U                        /*!< Transmitter and Receiver are disabled */
#define DDL_USART_DIRECTION_RX                   USART_CR1_REN                       /*!< Transmitter is disabled and Receiver is enabled */
#define DDL_USART_DIRECTION_TX                   USART_CR1_TEN                       /*!< Transmitter is enabled and Receiver is disabled */
#define DDL_USART_DIRECTION_TX_RX                (USART_CR1_TEN |USART_CR1_REN)       /*!< Transmitter and Receiver are enabled */
/**
  * @}
  */

/** @defgroup USART_DDL_EC_PARITY Parity Control
  * @{
  */
#define DDL_USART_PARITY_NONE                    0x00000000U                          /*!< Parity control disabled */
#define DDL_USART_PARITY_EVEN                    USART_CR1_PCEN                        /*!< Parity control enabled and Even Parity is selected */
#define DDL_USART_PARITY_ODD                     (USART_CR1_PCEN | USART_CR1_PS)       /*!< Parity control enabled and Odd Parity is selected */
/**
  * @}
  */

/** @defgroup USART_DDL_EC_WAKEUP Wakeup
  * @{
  */
#define DDL_USART_WAKEUP_IDLELINE                0x00000000U           /*!<  USART wake up from Mute mode on Idle Line */
#define DDL_USART_WAKEUP_ADDRESSMARK             USART_CR1_WKSEL        /*!<  USART wake up from Mute mode on Address Mark */
/**
  * @}
  */

/** @defgroup USART_DDL_EC_DATAWIDTH Datawidth
  * @{
  */
#define DDL_USART_DATAWIDTH_8B                   0x00000000U               /*!< 8 bits word length : Start bit, 8 data bits, n stop bits */
#define DDL_USART_DATAWIDTH_9B                   USART_CR1_M_0             /*!< 9 bits word length : Start bit, 9 data bits, n stop bits */
#define DDL_USART_DATAWIDTH_7B                   USART_CR1_M_1             /*!< 7 bits word length : Start bit, 7 data bits, n stop bits */
/**
  * @}
  */

/** @defgroup USART_DDL_EC_OVERSAMPLING Oversampling
  * @{
  */
#define DDL_USART_OVERSAMPLING_16                0x00000000U            /*!< Oversampling by 16 */
#define DDL_USART_OVERSAMPLING_8                 USART_CR1_OSMCFG        /*!< Oversampling by 8 */
/**
  * @}
  */

/** @defgroup USART_LL_EC_TXRX TX RX Pins Swap
  * @{
  */
#define DDL_USART_TXRX_STANDARD                  0x00000000U           /*!< TX/RX pins are used as defined in standard pinout */
#define DDL_USART_TXRX_SWAPPED                   (USART_CR1_SWAPEN)      /*!< TX and RX pins functions are swapped.             */
/**
  * @}
  */

#if defined(USE_FULL_DDL_DRIVER)
/** @defgroup USART_DDL_EC_CLOCK Clock Signal
  * @{
  */
#define DDL_USART_CLOCK_DISABLE                  0x00000000U            /*!< Clock signal not provided */
#define DDL_USART_CLOCK_ENABLE                   USART_CR2_CLKEN        /*!< Clock signal provided */
/**
  * @}
  */
#endif /*USE_FULL_DDL_DRIVER*/

/** @defgroup USART_DDL_EC_LASTCLKPULSE Last Clock Pulse
  * @{
  */
#define DDL_USART_LASTCLKPULSE_NO_OUTPUT         0x00000000U           /*!< The clock pulse of the last data bit is not output to the SCLK pin */
#define DDL_USART_LASTCLKPULSE_OUTPUT            USART_CR2_LBCL        /*!< The clock pulse of the last data bit is output to the SCLK pin */
/**
  * @}
  */

/** @defgroup USART_DDL_EC_BITORDER Bit Order
  * @{
  */
#define DDL_USART_BITORDER_LSBFIRST              0x00000000U           /*!< data is transmitted/received with data bit 0 first, following the start bit */
#define DDL_USART_BITORDER_MSBFIRST              USART_CR2_MSBFIRST    /*!< data is transmitted/received with the MSB first, following the start bit */
/**
  * @}
  */

/** @defgroup USART_DDL_EC_PHASE Clock Phase
  * @{
  */
#define DDL_USART_PHASE_1EDGE                    0x00000000U           /*!< The first clock transition is the first data capture edge */
#define DDL_USART_PHASE_2EDGE                    USART_CR2_CPHA        /*!< The second clock transition is the first data capture edge */
/**
  * @}
  */

/** @defgroup USART_DDL_EC_POLARITY Clock Polarity
  * @{
  */
#define DDL_USART_POLARITY_LOW                   0x00000000U           /*!< Steady low value on SCLK pin outside transmission window*/
#define DDL_USART_POLARITY_HIGH                  USART_CR2_CPOL        /*!< Steady high value on SCLK pin outside transmission window */
/**
  * @}
  */

/** @defgroup USART_DDL_EC_STOPBITS Stop Bits
  * @{
  */
#define DDL_USART_STOPBITS_1                     0x00000000U                              /*!< 1 stop bit */
#define DDL_USART_STOPBITS_2                     USART_CR2_STOP                           /*!< 2 stop bits */
/**
  * @}
  */

/** @defgroup USART_DDL_EC_ADDRESS_DETECT Address detection select
  * @{
  */
#define DDL_USART_ADDRESS_DETECT_4B                    0x00000000U           /*!< 4 bit address detection */
#define DDL_USART_ADDRESS_DETECT_7B                    USART_CR2_ADDRM7       /*!< 7 bit address detection */
/**
  * @}
  */

/** @defgroup USART_DDL_EC_HWCONTROL Hardware Control
  * @{
  */
#define DDL_USART_HWCONTROL_NONE                 0x00000000U                          /*!< CTS and RTS hardware flow control disabled */
#define DDL_USART_HWCONTROL_RTS                  USART_CR3_RTSEN                       /*!< RTS output enabled, data is only requested when there is space in the receive buffer */
#define DDL_USART_HWCONTROL_CTS                  USART_CR3_CTSEN                       /*!< CTS mode enabled, data is only transmitted when the nCTS input is asserted (tied to 0) */
#define DDL_USART_HWCONTROL_RTS_CTS              (USART_CR3_RTSEN | USART_CR3_CTSEN)    /*!< CTS and RTS hardware flow control enabled */
/**
  * @}
  */

/** @defgroup USART_DDL_EC_LINBREAK_DETECT LIN Break Detection Length
  * @{
  */
#define DDL_USART_LINBREAK_DETECT_10B            0x00000000U           /*!< 10-bit break detection method selected */
#define DDL_USART_LINBREAK_DETECT_11B            USART_CR2_LBDL        /*!< 11-bit break detection method selected */
/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup USART_DDL_Exported_Macros USART Exported Macros
  * @{
  */

/** @defgroup USART_DDL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in USART register
  * @param  __INSTANCE__ USART Instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define DDL_USART_WriteReg(__INSTANCE__, __REG__, __VALUE__) WRITE_REG(__INSTANCE__->__REG__, (__VALUE__))

/**
  * @brief  Read a value in USART register
  * @param  __INSTANCE__ USART Instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define DDL_USART_ReadReg(__INSTANCE__, __REG__) READ_REG(__INSTANCE__->__REG__)
/**
  * @}
  */

/** @defgroup USART_DDL_EM_Exported_Macros_Helper Exported_Macros_Helper
  * @{
  */

/**
  * @brief  Compute USARTDIV value according to Peripheral Clock and
  *         expected Baud Rate in 8 bits sampling mode (32 bits value of USARTDIV is returned)
  * @param  __PERIPHCLK__ Peripheral Clock frequency used for USART instance
  * @param  __BAUDRATE__ Baud rate value to achieve
  * @retval USARTDIV value to be used for BRR register filling in OverSampling_8 case
  */
#define __DDL_USART_DIV_SAMPLING8_100(__PERIPHCLK__, __BAUDRATE__)      ((uint32_t)((((uint64_t)(__PERIPHCLK__))*25)/(2*((uint64_t)(__BAUDRATE__)))))
#define __DDL_USART_DIVMANT_SAMPLING8(__PERIPHCLK__, __BAUDRATE__)      (__DDL_USART_DIV_SAMPLING8_100((__PERIPHCLK__), (__BAUDRATE__))/100)
#define __DDL_USART_DIVFRAQ_SAMPLING8(__PERIPHCLK__, __BAUDRATE__)      ((((__DDL_USART_DIV_SAMPLING8_100((__PERIPHCLK__), (__BAUDRATE__)) - (__DDL_USART_DIVMANT_SAMPLING8((__PERIPHCLK__), (__BAUDRATE__)) * 100)) * 8)\
                                                                         + 50) / 100)
/* USART BRR = mantissa + overflow + fraction
            = (USART DIVMANT << 4) + ((USART DIVFRAQ & 0xF8) << 1) + (USART DIVFRAQ & 0x07) */
#define __DDL_USART_DIV_SAMPLING8(__PERIPHCLK__, __BAUDRATE__)             (((__DDL_USART_DIVMANT_SAMPLING8((__PERIPHCLK__), (__BAUDRATE__)) << 4) + \
                                                                            ((__DDL_USART_DIVFRAQ_SAMPLING8((__PERIPHCLK__), (__BAUDRATE__)) & 0xF8) << 1)) + \
                                                                           (__DDL_USART_DIVFRAQ_SAMPLING8((__PERIPHCLK__), (__BAUDRATE__)) & 0x07))

/**
  * @brief  Compute USARTDIV value according to Peripheral Clock and
  *         expected Baud Rate in 16 bits sampling mode (32 bits value of USARTDIV is returned)
  * @param  __PERIPHCLK__ Peripheral Clock frequency used for USART instance
  * @param  __BAUDRATE__ Baud rate value to achieve
  * @retval USARTDIV value to be used for BRR register filling in OverSampling_16 case
  */
#define __DDL_USART_DIV_SAMPLING16_100(__PERIPHCLK__, __BAUDRATE__)     ((uint32_t)((((uint64_t)(__PERIPHCLK__))*25)/(4*((uint64_t)(__BAUDRATE__)))))
#define __DDL_USART_DIVMANT_SAMPLING16(__PERIPHCLK__, __BAUDRATE__)     (__DDL_USART_DIV_SAMPLING16_100((__PERIPHCLK__), (__BAUDRATE__))/100)
#define __DDL_USART_DIVFRAQ_SAMPLING16(__PERIPHCLK__, __BAUDRATE__)     ((((__DDL_USART_DIV_SAMPLING16_100((__PERIPHCLK__), (__BAUDRATE__)) - (__DDL_USART_DIVMANT_SAMPLING16((__PERIPHCLK__), (__BAUDRATE__)) * 100)) * 16)\
                                                                         + 50) / 100)
/* USART BRR = mantissa + overflow + fraction
            = (USART DIVMANT << 4) + (USART DIVFRAQ & 0xF0) + (USART DIVFRAQ & 0x0F) */
#define __DDL_USART_DIV_SAMPLING16(__PERIPHCLK__, __BAUDRATE__)            (((__DDL_USART_DIVMANT_SAMPLING16((__PERIPHCLK__), (__BAUDRATE__)) << 4) + \
                                                                            (__DDL_USART_DIVFRAQ_SAMPLING16((__PERIPHCLK__), (__BAUDRATE__)) & 0xF0)) + \
                                                                           (__DDL_USART_DIVFRAQ_SAMPLING16((__PERIPHCLK__), (__BAUDRATE__)) & 0x0F))

/**
  * @}
  */

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/

/** @defgroup USART_DDL_Exported_Functions USART Exported Functions
  * @{
  */

/** @defgroup USART_DDL_EF_Configuration Configuration functions
  * @{
  */

/**
  * @brief  USART Enable
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_Enable(USART_TypeDef *USARTx)
{
  SET_BIT(USARTx->CR1, USART_CR1_UEN);
}

/**
  * @brief  USART Disable (all USART prescalers and outputs are disabled)
  * @note   When USART is disabled, USART prescalers and outputs are stopped immediately,
  *         and current operations are discarded. The configuration of the USART is kept, but all the status
  *         flags, in the USARTx_SR are set to their default values.
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_Disable(USART_TypeDef *USARTx)
{
  CLEAR_BIT(USARTx->CR1, USART_CR1_UEN);
}

/**
  * @brief  Indicate if USART is enabled
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsEnabled(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->CR1, USART_CR1_UEN) == (USART_CR1_UEN));
}

/**
  * @brief  Receiver Enable (Receiver is enabled and begins searching for a start bit)
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_EnableDirectionRx(USART_TypeDef *USARTx)
{
  SET_BIT(USARTx->CR1, USART_CR1_REN);
}

/**
  * @brief  Receiver Disable
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_DisableDirectionRx(USART_TypeDef *USARTx)
{
  CLEAR_BIT(USARTx->CR1, USART_CR1_REN);
}

/**
  * @brief  Transmitter Enable
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_EnableDirectionTx(USART_TypeDef *USARTx)
{
  SET_BIT(USARTx->CR1, USART_CR1_TEN);
}

/**
  * @brief  Transmitter Disable
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_DisableDirectionTx(USART_TypeDef *USARTx)
{
  CLEAR_BIT(USARTx->CR1, USART_CR1_TEN);
}

/**
  * @brief  Configure simultaneously enabled/disabled states
  *         of Transmitter and Receiver
  * @param  USARTx USART Instance
  * @param  TransferDirection This parameter can be one of the following values:
  *         @arg @ref DDL_USART_DIRECTION_NONE
  *         @arg @ref DDL_USART_DIRECTION_RX
  *         @arg @ref DDL_USART_DIRECTION_TX
  *         @arg @ref DDL_USART_DIRECTION_TX_RX
  * @retval None
  */
__STATIC_INLINE void DDL_USART_SetTransferDirection(USART_TypeDef *USARTx, uint32_t TransferDirection)
{
  MODIFY_REG(USARTx->CR1, USART_CR1_REN | USART_CR1_TEN, TransferDirection);
}

/**
  * @brief  Return enabled/disabled states of Transmitter and Receiver
  * @param  USARTx USART Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_USART_DIRECTION_NONE
  *         @arg @ref DDL_USART_DIRECTION_RX
  *         @arg @ref DDL_USART_DIRECTION_TX
  *         @arg @ref DDL_USART_DIRECTION_TX_RX
  */
__STATIC_INLINE uint32_t DDL_USART_GetTransferDirection(USART_TypeDef *USARTx)
{
  return (uint32_t)(READ_BIT(USARTx->CR1, USART_CR1_REN | USART_CR1_TEN));
}

/**
  * @brief  Configure Parity (enabled/disabled and parity mode if enabled).
  * @note   This function selects if hardware parity control (generation and detection) is enabled or disabled.
  *         When the parity control is enabled (Odd or Even), computed parity bit is inserted at the MSB position
  *         (9th or 8th bit depending on data width) and parity is checked on the received data.
  * @param  USARTx USART Instance
  * @param  Parity This parameter can be one of the following values:
  *         @arg @ref DDL_USART_PARITY_NONE
  *         @arg @ref DDL_USART_PARITY_EVEN
  *         @arg @ref DDL_USART_PARITY_ODD
  * @retval None
  */
__STATIC_INLINE void DDL_USART_SetParity(USART_TypeDef *USARTx, uint32_t Parity)
{
  MODIFY_REG(USARTx->CR1, USART_CR1_PS | USART_CR1_PCEN, Parity);
}

/**
  * @brief  Return Parity configuration (enabled/disabled and parity mode if enabled)
  * @param  USARTx USART Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_USART_PARITY_NONE
  *         @arg @ref DDL_USART_PARITY_EVEN
  *         @arg @ref DDL_USART_PARITY_ODD
  */
__STATIC_INLINE uint32_t DDL_USART_GetParity(USART_TypeDef *USARTx)
{
  return (uint32_t)(READ_BIT(USARTx->CR1, USART_CR1_PS | USART_CR1_PCEN));
}

/**
  * @brief  Set Receiver Wake Up method from Mute mode.
  * @param  USARTx USART Instance
  * @param  Method This parameter can be one of the following values:
  *         @arg @ref DDL_USART_WAKEUP_IDLELINE
  *         @arg @ref DDL_USART_WAKEUP_ADDRESSMARK
  * @retval None
  */
__STATIC_INLINE void DDL_USART_SetWakeUpMethod(USART_TypeDef *USARTx, uint32_t Method)
{
  MODIFY_REG(USARTx->CR1, USART_CR1_WKSEL, Method);
}

/**
  * @brief  Return Receiver Wake Up method from Mute mode
  * @param  USARTx USART Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_USART_WAKEUP_IDLELINE
  *         @arg @ref DDL_USART_WAKEUP_ADDRESSMARK
  */
__STATIC_INLINE uint32_t DDL_USART_GetWakeUpMethod(USART_TypeDef *USARTx)
{
  return (uint32_t)(READ_BIT(USARTx->CR1, USART_CR1_WKSEL));
}

/**
  * @brief  Set Word length (i.e. nb of data bits, excluding start and stop bits)
  * @param  USARTx USART Instance
  * @param  DataWidth This parameter can be one of the following values:
  *         @arg @ref DDL_USART_DATAWIDTH_8B
  *         @arg @ref DDL_USART_DATAWIDTH_9B
  *         @arg @ref DDL_USART_DATAWIDTH_7B
  * @retval None
  */
__STATIC_INLINE void DDL_USART_SetDataWidth(USART_TypeDef *USARTx, uint32_t DataWidth)
{
  MODIFY_REG(USARTx->CR1, USART_CR1_M, DataWidth);
}

/**
  * @brief  Return Word length (i.e. nb of data bits, excluding start and stop bits)
  * @param  USARTx USART Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_USART_DATAWIDTH_8B
  *         @arg @ref DDL_USART_DATAWIDTH_9B
  *         @arg @ref DDL_USART_DATAWIDTH_7B
  */
__STATIC_INLINE uint32_t DDL_USART_GetDataWidth(USART_TypeDef *USARTx)
{
  return (uint32_t)(READ_BIT(USARTx->CR1, USART_CR1_M));
}

/**
  * @brief  Set Oversampling to 8-bit or 16-bit mode
  * @param  USARTx USART Instance
  * @param  OverSampling This parameter can be one of the following values:
  *         @arg @ref DDL_USART_OVERSAMPLING_16
  *         @arg @ref DDL_USART_OVERSAMPLING_8
  * @retval None
  */
__STATIC_INLINE void DDL_USART_SetOverSampling(USART_TypeDef *USARTx, uint32_t OverSampling)
{
  MODIFY_REG(USARTx->CR1, USART_CR1_OSMCFG, OverSampling);
}

/**
  * @brief  Return Oversampling mode
  * @param  USARTx USART Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_USART_OVERSAMPLING_16
  *         @arg @ref DDL_USART_OVERSAMPLING_8
  */
__STATIC_INLINE uint32_t DDL_USART_GetOverSampling(USART_TypeDef *USARTx)
{
  return (uint32_t)(READ_BIT(USARTx->CR1, USART_CR1_OSMCFG));
}

/**
  * @brief  Receiver Timeout Enable
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_EnableRxTimeout(USART_TypeDef *USARTx)
{
  SET_BIT(USARTx->CR1, USART_CR1_RXTODEN);
}

/**
  * @brief  Receiver Timeout Disable
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_DisableRxTimeout(USART_TypeDef *USARTx)
{
  CLEAR_BIT(USARTx->CR1, USART_CR1_RXTODEN);
}

/**
  * @brief  Receiver Timeout Disable
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsEnabledRxTimeout(USART_TypeDef *USARTx)
{
  return (uint32_t)(READ_BIT(USARTx->CR1, USART_CR1_RXTODEN) == USART_CR1_RXTODEN ? 1UL : 0UL);
}

/**
  * @brief  Configure TX/RX pins swapping setting.
  * @param  USARTx USART Instance
  * @param  SwapConfig This parameter can be one of the following values:
  *         @arg @ref DDL_USART_TXRX_STANDARD
  *         @arg @ref DDL_USART_TXRX_SWAPPED
  * @retval None
  */
__STATIC_INLINE void DDL_USART_SetTXRXSwap(USART_TypeDef *USARTx, uint32_t SwapConfig)
{
  MODIFY_REG(USARTx->CR1, USART_CR1_SWAPEN, SwapConfig);
}

/**
  * @brief  Retrieve TX/RX pins swapping configuration.
  * @param  USARTx USART Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_USART_TXRX_STANDARD
  *         @arg @ref DDL_USART_TXRX_SWAPPED
  */
__STATIC_INLINE uint32_t DDL_USART_GetTXRXSwap(USART_TypeDef *USARTx)
{
  return (uint32_t)(READ_BIT(USARTx->CR1, USART_CR1_SWAPEN));
}

/**
  * @brief  Set Address of the USART node.
  * @note   This is used in multiprocessor communication during Mute mode or Stop mode,
  *         for wake up with address mark detection.
  * @note   4bits address node is used when 4-bit Address Detection is selected in ADDM7.
  *         (b7-b4 should be set to 0)
  *         8bits address node is used when 7-bit Address Detection is selected in ADDM7.
  *         (This is used in multiprocessor communication during Mute mode or Stop mode,
  *         for wake up with 7-bit address mark detection.
  *         The MSB of the character sent by the transmitter should be equal to 1.
  *         It may also be used for character detection during normal reception,
  *         Mute mode inactive (for example, end of block detection in ModBus protocol).
  *         In this case, the whole received character (8-bit) is compared to the ADD[7:0]
  *         value and CMF flag is set on match)
  * @param  USARTx USART Instance
  * @param  AddressLen This parameter can be one of the following values:
  *         @arg @ref DDL_USART_ADDRESS_DETECT_4B
  *         @arg @ref DDL_USART_ADDRESS_DETECT_7B
  * @param  NodeAddress 4 or 7 bit Address of the USART node.
  * @retval None
  */
__STATIC_INLINE void DDL_USART_ConfigNodeAddress(USART_TypeDef *USARTx, uint32_t AddressLen, uint32_t NodeAddress)
{
  MODIFY_REG(USARTx->CR2, USART_CR2_ADDR | USART_CR2_ADDRM7,
             (uint32_t)(AddressLen | (NodeAddress << USART_CR2_ADDR_Pos)));
}

/**
  * @brief  Return 8 bit Address of the USART node as set in ADD field of CR2.
  * @param  USARTx USART Instance
  * @retval Address of the USART node (Value between Min_Data=0 and Max_Data=255)
  */
__STATIC_INLINE uint32_t DDL_USART_GetNodeAddress(USART_TypeDef *USARTx)
{
  return (uint32_t)(READ_BIT(USARTx->CR2, USART_CR2_ADDR) >> USART_CR2_ADDR_Pos);
}

/**
  * @brief  Return Length of Node Address used in Address Detection mode (7-bit or 4-bit)
  * @param  USARTx USART Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_USART_ADDRESS_DETECT_4B
  *         @arg @ref DDL_USART_ADDRESS_DETECT_7B
  */
__STATIC_INLINE uint32_t DDL_USART_GetNodeAddressLen(USART_TypeDef *USARTx)
{
  return (uint32_t)(READ_BIT(USARTx->CR2, USART_CR2_ADDRM7));
}

/**
  * @brief  Configure if Clock pulse of the last data bit is output to the SCLK pin or not
  * @note   Macro @ref IS_USART_INSTANCE(USARTx) can be used to check whether or not
  *         Synchronous mode is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @param  LastBitClockPulse This parameter can be one of the following values:
  *         @arg @ref DDL_USART_LASTCLKPULSE_NO_OUTPUT
  *         @arg @ref DDL_USART_LASTCLKPULSE_OUTPUT
  * @retval None
  */
__STATIC_INLINE void DDL_USART_SetLastClkPulseOutput(USART_TypeDef *USARTx, uint32_t LastBitClockPulse)
{
  MODIFY_REG(USARTx->CR2, USART_CR2_LBCL, LastBitClockPulse);
}

/**
  * @brief  Retrieve Clock pulse of the last data bit output configuration
  *         (Last bit Clock pulse output to the SCLK pin or not)
  * @note   Macro @ref IS_USART_INSTANCE(USARTx) can be used to check whether or not
  *         Synchronous mode is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_USART_LASTCLKPULSE_NO_OUTPUT
  *         @arg @ref DDL_USART_LASTCLKPULSE_OUTPUT
  */
__STATIC_INLINE uint32_t DDL_USART_GetLastClkPulseOutput(USART_TypeDef *USARTx)
{
  return (uint32_t)(READ_BIT(USARTx->CR2, USART_CR2_LBCL));
}

/**
  * @brief  Select the phase of the clock output on the SCLK pin in synchronous mode
  * @note   Macro @ref IS_USART_INSTANCE(USARTx) can be used to check whether or not
  *         Synchronous mode is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @param  ClockPhase This parameter can be one of the following values:
  *         @arg @ref DDL_USART_PHASE_1EDGE
  *         @arg @ref DDL_USART_PHASE_2EDGE
  * @retval None
  */
__STATIC_INLINE void DDL_USART_SetClockPhase(USART_TypeDef *USARTx, uint32_t ClockPhase)
{
  MODIFY_REG(USARTx->CR2, USART_CR2_CPHA, ClockPhase);
}

/**
  * @brief  Return phase of the clock output on the SCLK pin in synchronous mode
  * @note   Macro @ref IS_USART_INSTANCE(USARTx) can be used to check whether or not
  *         Synchronous mode is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_USART_PHASE_1EDGE
  *         @arg @ref DDL_USART_PHASE_2EDGE
  */
__STATIC_INLINE uint32_t DDL_USART_GetClockPhase(USART_TypeDef *USARTx)
{
  return (uint32_t)(READ_BIT(USARTx->CR2, USART_CR2_CPHA));
}

/**
  * @brief  Select the polarity of the clock output on the SCLK pin in synchronous mode
  * @note   Macro @ref IS_USART_INSTANCE(USARTx) can be used to check whether or not
  *         Synchronous mode is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @param  ClockPolarity This parameter can be one of the following values:
  *         @arg @ref DDL_USART_POLARITY_LOW
  *         @arg @ref DDL_USART_POLARITY_HIGH
  * @retval None
  */
__STATIC_INLINE void DDL_USART_SetClockPolarity(USART_TypeDef *USARTx, uint32_t ClockPolarity)
{
  MODIFY_REG(USARTx->CR2, USART_CR2_CPOL, ClockPolarity);
}

/**
  * @brief  Return polarity of the clock output on the SCLK pin in synchronous mode
  * @note   Macro @ref IS_USART_INSTANCE(USARTx) can be used to check whether or not
  *         Synchronous mode is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_USART_POLARITY_LOW
  *         @arg @ref DDL_USART_POLARITY_HIGH
  */
__STATIC_INLINE uint32_t DDL_USART_GetClockPolarity(USART_TypeDef *USARTx)
{
  return (uint32_t)(READ_BIT(USARTx->CR2, USART_CR2_CPOL));
}

/**
  * @brief  Configure Clock signal format (Phase Polarity and choice about output of last bit clock pulse)
  * @note   Macro @ref IS_USART_INSTANCE(USARTx) can be used to check whether or not
  *         Synchronous mode is supported by the USARTx instance.
  * @note   Call of this function is equivalent to following function call sequence :
  *         - Clock Phase configuration using @ref DDL_USART_SetClockPhase() function
  *         - Clock Polarity configuration using @ref DDL_USART_SetClockPolarity() function
  *         - Output of Last bit Clock pulse configuration using @ref DDL_USART_SetLastClkPulseOutput() function
  * @param  USARTx USART Instance
  * @param  Phase This parameter can be one of the following values:
  *         @arg @ref DDL_USART_PHASE_1EDGE
  *         @arg @ref DDL_USART_PHASE_2EDGE
  * @param  Polarity This parameter can be one of the following values:
  *         @arg @ref DDL_USART_POLARITY_LOW
  *         @arg @ref DDL_USART_POLARITY_HIGH
  * @param  LBCPOutput This parameter can be one of the following values:
  *         @arg @ref DDL_USART_LASTCLKPULSE_NO_OUTPUT
  *         @arg @ref DDL_USART_LASTCLKPULSE_OUTPUT
  * @retval None
  */
__STATIC_INLINE void DDL_USART_ConfigClock(USART_TypeDef *USARTx, uint32_t Phase, uint32_t Polarity, uint32_t LBCPOutput)
{
  MODIFY_REG(USARTx->CR2, USART_CR2_CPHA | USART_CR2_CPOL | USART_CR2_LBCL, Phase | Polarity | LBCPOutput);
}

/**
  * @brief  Enable Clock output on SCLK pin
  * @note   Macro @ref IS_USART_INSTANCE(USARTx) can be used to check whether or not
  *         Synchronous mode is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_EnableSCLKOutput(USART_TypeDef *USARTx)
{
  SET_BIT(USARTx->CR2, USART_CR2_CLKEN);
}

/**
  * @brief  Disable Clock output on SCLK pin
  * @note   Macro @ref IS_USART_INSTANCE(USARTx) can be used to check whether or not
  *         Synchronous mode is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_DisableSCLKOutput(USART_TypeDef *USARTx)
{
  CLEAR_BIT(USARTx->CR2, USART_CR2_CLKEN);
}

/**
  * @brief  Indicate if Clock output on SCLK pin is enabled
  * @note   Macro @ref IS_USART_INSTANCE(USARTx) can be used to check whether or not
  *         Synchronous mode is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsEnabledSCLKOutput(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->CR2, USART_CR2_CLKEN) == (USART_CR2_CLKEN));
}

/**
  * @brief  Set the length of the stop bits
  * @param  USARTx USART Instance
  * @param  StopBits This parameter can be one of the following values:
  *         @arg @ref DDL_USART_STOPBITS_1
  *         @arg @ref DDL_USART_STOPBITS_2
  * @retval None
  */
__STATIC_INLINE void DDL_USART_SetStopBitsLength(USART_TypeDef *USARTx, uint32_t StopBits)
{
  MODIFY_REG(USARTx->CR2, USART_CR2_STOP, StopBits);
}

/**
  * @brief  Retrieve the length of the stop bits
  * @param  USARTx USART Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_USART_STOPBITS_1
  *         @arg @ref DDL_USART_STOPBITS_2
  */
__STATIC_INLINE uint32_t DDL_USART_GetStopBitsLength(USART_TypeDef *USARTx)
{
  return (uint32_t)(READ_BIT(USARTx->CR2, USART_CR2_STOP));
}

/**
  * @brief  Configure transfer bit order (either Less or Most Significant Bit First).
  * @param  USARTx USART Instance
  * @param  BitOrder This parameter can be one of the following values:
  *         @arg @ref DDL_USART_BITORDER_LSBFIRST
  *         @arg @ref DDL_USART_BITORDER_MSBFIRST
  * @retval None
  */
__STATIC_INLINE void DDL_USART_SetTransferBitOrder(USART_TypeDef *USARTx, uint32_t BitOrder)
{
  MODIFY_REG(USARTx->CR2, USART_CR2_MSBFIRST, BitOrder);
}

/**
  * @brief  Disable Most significant bit first
  * @param  USARTx USART Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_USART_BITORDER_LSBFIRST
  *         @arg @ref DDL_USART_BITORDER_MSBFIRST
  */
__STATIC_INLINE uint32_t DDL_USART_GetTransferBitOrder(USART_TypeDef *USARTx)
{
  return (uint32_t)(READ_BIT(USARTx->CR2, USART_CR2_MSBFIRST));
}

/**
  * @brief  Configure Character frame format (Datawidth, Parity control, Stop Bits)
  * @note   Call of this function is equivalent to following function call sequence :
  *         - Data Width configuration using @ref DDL_USART_SetDataWidth() function
  *         - Parity Control and mode configuration using @ref DDL_USART_SetParity() function
  *         - Stop bits configuration using @ref DDL_USART_SetStopBitsLength() function
  * @param  USARTx USART Instance
  * @param  DataWidth This parameter can be one of the following values:
  *         @arg @ref DDL_USART_DATAWIDTH_8B
  *         @arg @ref DDL_USART_DATAWIDTH_9B
  *         @arg @ref DDL_USART_DATAWIDTH_7B
  * @param  Parity This parameter can be one of the following values:
  *         @arg @ref DDL_USART_PARITY_NONE
  *         @arg @ref DDL_USART_PARITY_EVEN
  *         @arg @ref DDL_USART_PARITY_ODD
  * @param  StopBits This parameter can be one of the following values:
  *         @arg @ref DDL_USART_STOPBITS_1
  *         @arg @ref DDL_USART_STOPBITS_2
  * @retval None
  */
__STATIC_INLINE void DDL_USART_ConfigCharacter(USART_TypeDef *USARTx, uint32_t DataWidth, uint32_t Parity,
                                              uint32_t StopBits)
{
  MODIFY_REG(USARTx->CR1, USART_CR1_PS | USART_CR1_PCEN | USART_CR1_M, Parity | DataWidth);
  MODIFY_REG(USARTx->CR2, USART_CR2_STOP, StopBits);
}

/**
  * @brief  Set Receiver Timeout Value Setup.
  * @param  USARTx USART Instance
  * @param  Timeout 24 bit of the USART deassertion.
  * @retval None
  */
__STATIC_INLINE void DDL_USART_SetRxTimeout(USART_TypeDef *USARTx, uint32_t Timeout)
{
  MODIFY_REG(USARTx->RXTOR, USART_RXTOR_RXTO, (Timeout & USART_RXTOR_RXTO));
}

/**
  * @brief  Return 24 bit of the USART deassertion time as set in time of RXTOR.
  * @note   only 24bits (b24-b0) of returned value are relevant
  * @param  USARTx USART Instance
  * @retval Value between Min_Data=0x00 and Max_Data=0x00FFFFFF
  */
__STATIC_INLINE uint32_t DDL_USART_GetRxTimeout(USART_TypeDef *USARTx)
{
  return (uint32_t)(READ_BIT(USARTx->RXTOR, USART_RXTOR_RXTO));
}

/**
  * @brief  Enable RTS HW Flow Control
  * @note   Macro @ref IS_USART_HWFLOW_INSTANCE(USARTx) can be used to check whether or not
  *         Hardware Flow control feature is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_EnableRTSHWFlowCtrl(USART_TypeDef *USARTx)
{
  SET_BIT(USARTx->CR3, USART_CR3_RTSEN);
}

/**
  * @brief  Disable RTS HW Flow Control
  * @note   Macro @ref IS_USART_HWFLOW_INSTANCE(USARTx) can be used to check whether or not
  *         Hardware Flow control feature is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_DisableRTSHWFlowCtrl(USART_TypeDef *USARTx)
{
  CLEAR_BIT(USARTx->CR3, USART_CR3_RTSEN);
}

/**
  * @brief  Enable CTS HW Flow Control
  * @note   Macro @ref IS_USART_HWFLOW_INSTANCE(USARTx) can be used to check whether or not
  *         Hardware Flow control feature is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_EnableCTSHWFlowCtrl(USART_TypeDef *USARTx)
{
  SET_BIT(USARTx->CR3, USART_CR3_CTSEN);
}

/**
  * @brief  Disable CTS HW Flow Control
  * @note   Macro @ref IS_USART_HWFLOW_INSTANCE(USARTx) can be used to check whether or not
  *         Hardware Flow control feature is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_DisableCTSHWFlowCtrl(USART_TypeDef *USARTx)
{
  CLEAR_BIT(USARTx->CR3, USART_CR3_CTSEN);
}

/**
  * @brief  Configure HW Flow Control mode (both CTS and RTS)
  * @note   Macro @ref IS_USART_HWFLOW_INSTANCE(USARTx) can be used to check whether or not
  *         Hardware Flow control feature is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @param  HardwareFlowControl This parameter can be one of the following values:
  *         @arg @ref DDL_USART_HWCONTROL_NONE
  *         @arg @ref DDL_USART_HWCONTROL_RTS
  *         @arg @ref DDL_USART_HWCONTROL_CTS
  *         @arg @ref DDL_USART_HWCONTROL_RTS_CTS
  * @retval None
  */
__STATIC_INLINE void DDL_USART_SetHWFlowCtrl(USART_TypeDef *USARTx, uint32_t HardwareFlowControl)
{
  MODIFY_REG(USARTx->CR3, USART_CR3_RTSEN | USART_CR3_CTSEN, HardwareFlowControl);
}

/**
  * @brief  Return HW Flow Control configuration (both CTS and RTS)
  * @note   Macro @ref IS_USART_HWFLOW_INSTANCE(USARTx) can be used to check whether or not
  *         Hardware Flow control feature is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_USART_HWCONTROL_NONE
  *         @arg @ref DDL_USART_HWCONTROL_RTS
  *         @arg @ref DDL_USART_HWCONTROL_CTS
  *         @arg @ref DDL_USART_HWCONTROL_RTS_CTS
  */
__STATIC_INLINE uint32_t DDL_USART_GetHWFlowCtrl(USART_TypeDef *USARTx)
{
  return (uint32_t)(READ_BIT(USARTx->CR3, USART_CR3_RTSEN | USART_CR3_CTSEN));
}

/**
  * @brief  Enable One bit sampling method.
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_EnableOneBitSamp(USART_TypeDef *USARTx)
{
  SET_BIT(USARTx->CR3, USART_CR3_ONEBIT);
}

/**
  * @brief  Disable One bit sampling method
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_DisableOneBitSamp(USART_TypeDef *USARTx)
{
  CLEAR_BIT(USARTx->CR3, USART_CR3_ONEBIT);
}

/**
  * @brief  Indicate if One bit sampling method is enabled
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsEnabledOneBitSamp(USART_TypeDef *USARTx)
{
  return ((READ_BIT(USARTx->CR3, USART_CR3_ONEBIT) == (USART_CR3_ONEBIT)) ? 1UL : 0UL);
}

/**
  * @brief  Configure USART BRR register for achieving expected Baud Rate value.
  * @note   Compute and set USARTDIV value in BRR Register (full BRR content)
  *         according to used Peripheral Clock, Oversampling mode, and expected Baud Rate values
  * @note   Peripheral clock and Baud rate values provided as function parameters should be valid
  *         (Baud rate value != 0)
  * @param  USARTx USART Instance
  * @param  PeriphClk Peripheral Clock
  * @param  OverSampling This parameter can be one of the following values:
  *         @arg @ref DDL_USART_OVERSAMPLING_16
  *         @arg @ref DDL_USART_OVERSAMPLING_8
  * @param  BaudRate Baud Rate
  * @retval None
  */
__STATIC_INLINE void DDL_USART_SetBaudRate(USART_TypeDef *USARTx, uint32_t PeriphClk, uint32_t OverSampling,
                                          uint32_t BaudRate)
{
  if (OverSampling == DDL_USART_OVERSAMPLING_8)
  {
    USARTx->BRR = (uint16_t)(__DDL_USART_DIV_SAMPLING8(PeriphClk, BaudRate));
  }
  else
  {
    USARTx->BRR = (uint16_t)(__DDL_USART_DIV_SAMPLING16(PeriphClk, BaudRate));
  }
}

/**
  * @brief  Return current Baud Rate value, according to USARTDIV present in BRR register
  *         (full BRR content), and to used Peripheral Clock and Oversampling mode values
  * @note   In case of non-initialized or invalid value stored in BRR register, value 0 will be returned.
  * @param  USARTx USART Instance
  * @param  PeriphClk Peripheral Clock
  * @param  OverSampling This parameter can be one of the following values:
  *         @arg @ref DDL_USART_OVERSAMPLING_16
  *         @arg @ref DDL_USART_OVERSAMPLING_8
  * @retval Baud Rate
  */
__STATIC_INLINE uint32_t DDL_USART_GetBaudRate(USART_TypeDef *USARTx, uint32_t PeriphClk, uint32_t OverSampling)
{
  uint32_t usartdiv = 0x0U;
  uint32_t brrresult = 0x0U;

  usartdiv = USARTx->BRR;

  if (OverSampling == DDL_USART_OVERSAMPLING_8)
  {
    if ((usartdiv & 0xFFF7U) != 0U)
    {
      usartdiv = (uint16_t)((usartdiv & 0xFFF0U) | ((usartdiv & 0x0007U) << 1U)) ;
      brrresult = (PeriphClk * 2U) / usartdiv;
    }
  }
  else
  {
    if ((usartdiv & 0xFFFFU) != 0U)
    {
      brrresult = PeriphClk / usartdiv;
    }
  }
  return (brrresult);
}

/**
  * @}
  */

/** @defgroup USART_DDL_EF_Configuration_HalfDuplex Configuration functions related to Half Duplex feature
  * @{
  */

/**
  * @brief  Enable Single Wire Half-Duplex mode
  * @note   Macro @ref IS_USART_HALFDUPLEX_INSTANCE(USARTx) can be used to check whether or not
  *         Half-Duplex mode is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_EnableHalfDuplex(USART_TypeDef *USARTx)
{
  SET_BIT(USARTx->CR3, USART_CR3_HDSEL);
}

/**
  * @brief  Disable Single Wire Half-Duplex mode
  * @note   Macro @ref IS_USART_HALFDUPLEX_INSTANCE(USARTx) can be used to check whether or not
  *         Half-Duplex mode is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_DisableHalfDuplex(USART_TypeDef *USARTx)
{
  CLEAR_BIT(USARTx->CR3, USART_CR3_HDSEL);
}

/**
  * @brief  Indicate if Single Wire Half-Duplex mode is enabled
  * @note   Macro @ref IS_USART_HALFDUPLEX_INSTANCE(USARTx) can be used to check whether or not
  *         Half-Duplex mode is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsEnabledHalfDuplex(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->CR3, USART_CR3_HDSEL) == (USART_CR3_HDSEL));
}

/**
  * @}
  */

/** @defgroup USART_DDL_EF_Configuration_LIN Configuration functions related to LIN feature
  * @{
  */

/**
  * @brief  Set LIN Break Detection Length
  * @note   Macro @ref IS_USART_LIN_INSTANCE(USARTx) can be used to check whether or not
  *         LIN feature is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @param  LINBDLength This parameter can be one of the following values:
  *         @arg @ref DDL_USART_LINBREAK_DETECT_10B
  *         @arg @ref DDL_USART_LINBREAK_DETECT_11B
  * @retval None
  */
__STATIC_INLINE void DDL_USART_SetLINBrkDetectionLen(USART_TypeDef *USARTx, uint32_t LINBDLength)
{
  MODIFY_REG(USARTx->CR2, USART_CR2_LBDL, LINBDLength);
}

/**
  * @brief  Return LIN Break Detection Length
  * @note   Macro @ref IS_USART_LIN_INSTANCE(USARTx) can be used to check whether or not
  *         LIN feature is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_USART_LINBREAK_DETECT_10B
  *         @arg @ref DDL_USART_LINBREAK_DETECT_11B
  */
__STATIC_INLINE uint32_t DDL_USART_GetLINBrkDetectionLen(USART_TypeDef *USARTx)
{
  return (uint32_t)(READ_BIT(USARTx->CR2, USART_CR2_LBDL));
}

/**
  * @brief  Enable LIN mode
  * @note   Macro @ref IS_USART_LIN_INSTANCE(USARTx) can be used to check whether or not
  *         LIN feature is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_EnableLIN(USART_TypeDef *USARTx)
{
  SET_BIT(USARTx->CR2, USART_CR2_LINEN);
}

/**
  * @brief  Disable LIN mode
  * @note   Macro @ref IS_USART_LIN_INSTANCE(USARTx) can be used to check whether or not
  *         LIN feature is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_DisableLIN(USART_TypeDef *USARTx)
{
  CLEAR_BIT(USARTx->CR2, USART_CR2_LINEN);
}

/**
  * @brief  Indicate if LIN mode is enabled
  * @note   Macro @ref IS_USART_LIN_INSTANCE(USARTx) can be used to check whether or not
  *         LIN feature is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsEnabledLIN(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->CR2, USART_CR2_LINEN) == (USART_CR2_LINEN));
}

/**
  * @}
  */

/** @defgroup USART_DDL_EF_AutomaticBaudRate Automatic Baud Rate
  * @{
  */

/**
  * @brief  Enable Auto Baud-Rate Detection.
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_EnableAutoBaudRate(USART_TypeDef *USARTx)
{
  SET_BIT(USARTx->CR2, USART_CR2_ABEN);
}

/**
  * @brief  Disable Auto Baud-Rate Detection
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_DisableAutoBaudRate(USART_TypeDef *USARTx)
{
  CLEAR_BIT(USARTx->CR2, USART_CR2_ABEN);
}

/**
  * @brief  Indicate if Auto Baud-Rate Detection mechanism is enabled
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsEnabledAutoBaud(USART_TypeDef *USARTx)
{
  return ((READ_BIT(USARTx->CR2, USART_CR2_ABEN) == (USART_CR2_ABEN)) ? 1UL : 0UL);
}

/**
  * @}
  */

/** @defgroup USART_LL_EF_Configuration_DE Configuration functions related to Driver Enable feature
  * @{
  */

/**
  * @brief  Set Driver Enable deassertion time.
  * @param  USARTx USART Instance
  * @param  time This value can between 0x0 and 0x1F.
  * @retval None
  */
__STATIC_INLINE void DDL_USART_SetDEDeassertionTime(USART_TypeDef *USARTx, uint32_t time)
{
  MODIFY_REG(USARTx->CR1, USART_CR1_DEDT, ((time << USART_CR1_DEDT_Pos) & USART_CR1_DEDT));
}

/**
  * @brief  Return 5 bit of the USART deassertion time as set in DEDT of CTRL1.
  * @param  USARTx USART Instance
  * @retval The return value can between 0x0 and 0x1F
  */
__STATIC_INLINE uint32_t DDL_USART_GetDEDeassertionTime(USART_TypeDef *USARTx)
{
  return (uint32_t)(READ_BIT(USARTx->CR1, USART_CR1_DEDT) >> USART_CR1_DEDT_Pos);
}

/**
  * @brief  Set Driver Enable assertion time.
  * @param  USARTx USART Instance
  * @param  time This value can between 0x0 and 0x1F.
  * @retval None
  */
__STATIC_INLINE void DDL_USART_SetDEAssertionTime(USART_TypeDef *USARTx, uint32_t time)
{
  MODIFY_REG(USARTx->CR1, USART_CR1_DEAT, ((time << USART_CR1_DEAT_Pos) & USART_CR1_DEAT));
}

/**
  * @brief  Return 5 bit of the USART deassertion time as set in DEAT of CTRL1.
  * @param  USARTx USART Instance
  * @retval The return value can between 0x0 and 0x1F
  */
__STATIC_INLINE uint32_t DDL_USART_GetDEAssertionTime(USART_TypeDef *USARTx)
{
  return (uint32_t)(READ_BIT(USARTx->CR1, USART_CR1_DEAT) >> USART_CR1_DEAT_Pos);
}

/**
  * @brief  Enable Driver Enable mode
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_EnableDEMode(USART_TypeDef *USARTx)
{
  SET_BIT(USARTx->CR3, USART_CR3_DEMEN);
}

/**
  * @brief  Disable Driver Enable mode
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_DisableDEMode(USART_TypeDef *USARTx)
{
  CLEAR_BIT(USARTx->CR3, USART_CR3_DEMEN);
}

/**
  * @brief  Indicate if Driver Enable (DE) Mode is enabled
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsEnabledDEMode(USART_TypeDef *USARTx)
{
  return ((READ_BIT(USARTx->CR3, USART_CR3_DEMEN) == (USART_CR3_DEMEN)) ? 1UL : 0UL);
}

/**
  * @}
  */

/** @defgroup USART_DDL_EF_AdvancedConfiguration Advanced Configurations services
  * @{
  */

/**
  * @brief  Perform basic configuration of USART for enabling use in Asynchronous Mode (USART)
  * @note   In USART mode, the following bits must be kept cleared:
  *           - LINEN bit in the USART_CTRL2 register,
  *           - CLKEN bit in the USART_CTRL2 register,
  *           - HDSEL bit in the USART_CTRL3 register.
  * @note   Call of this function is equivalent to following function call sequence :
  *         - Clear LINEN in CTRL2 using @ref DDL_USART_DisableLIN() function
  *         - Clear CLKEN in CTRL2 using @ref DDL_USART_DisableSCLKOutput() function
  *         - Clear HDSEL in CTRL3 using @ref DDL_USART_DisableHalfDuplex() function
  * @note   Other remaining configurations items related to Asynchronous Mode
  *         (as Baud Rate, Word length, Parity, ...) should be set using
  *         dedicated functions
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_ConfigAsyncMode(USART_TypeDef *USARTx)
{
  /* In Asynchronous mode, the following bits must be kept cleared:
  - LINEN, CLKEN bits in the USART_CTRL2 register,
  - HDSEL bits in the USART_CTRL3 register.*/
  CLEAR_BIT(USARTx->CR2, (USART_CR2_LINEN | USART_CR2_CLKEN));
  CLEAR_BIT(USARTx->CR3, (USART_CR3_HDSEL));
}

/**
  * @brief  Perform basic configuration of USART for enabling use in Synchronous Mode
  * @note   In Synchronous mode, the following bits must be kept cleared:
  *           - LINEN bit in the USART_CTRL2 register,
  *           - HDSEL bit in the USART_CTRL3 register.
  *         This function also sets the USART in Synchronous mode.
  * @note   Macro @ref IS_USART_INSTANCE(USARTx) can be used to check whether or not
  *         Synchronous mode is supported by the USARTx instance.
  * @note   Call of this function is equivalent to following function call sequence :
  *         - Clear LINEN in CTRL2 using @ref DDL_USART_DisableLIN() function
  *         - Clear HDSEL in CTRL3 using @ref DDL_USART_DisableHalfDuplex() function
  *         - Set CLKEN in CTRL2 using @ref DDL_USART_EnableSCLKOutput() function
  * @note   Other remaining configurations items related to Synchronous Mode
  *         (as Baud Rate, Word length, Parity, Clock Polarity, ...) should be set using
  *         dedicated functions
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_ConfigSyncMode(USART_TypeDef *USARTx)
{
  /* In Synchronous mode, the following bits must be kept cleared:
  - LINEN bit in the USART_CTRL2 register,
  - HDSEL bits in the USART_CTRL3 register.*/
  CLEAR_BIT(USARTx->CR2, (USART_CR2_LINEN));
  CLEAR_BIT(USARTx->CR3, (USART_CR3_HDSEL));
  /* set the USART/USART in Synchronous mode */
  SET_BIT(USARTx->CR2, USART_CR2_CLKEN);
}

/**
  * @brief  Perform basic configuration of USART for enabling use in LIN Mode
  * @note   In LIN mode, the following bits must be kept cleared:
  *           - STOP and CLKEN bits in the USART_CTRL2 register,
  *           - HDSEL bit in the USART_CTRL3 register.
  *         This function also set the USART/USART in LIN mode.
  * @note   Macro @ref IS_USART_LIN_INSTANCE(USARTx) can be used to check whether or not
  *         LIN feature is supported by the USARTx instance.
  * @note   Call of this function is equivalent to following function call sequence :
  *         - Clear CLKEN in CTRL2 using @ref DDL_USART_DisableSCLKOutput() function
  *         - Clear STOP in CTRL2 using @ref DDL_USART_SetStopBitsLength() function
  *         - Clear HDSEL in CTRL3 using @ref DDL_USART_DisableHalfDuplex() function
  *         - Set LINEN in CTRL2 using @ref DDL_USART_EnableLIN() function
  * @note   Other remaining configurations items related to LIN Mode
  *         (as Baud Rate, Word length, LIN Break Detection Length, ...) should be set using
  *         dedicated functions
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_ConfigLINMode(USART_TypeDef *USARTx)
{
  /* In LIN mode, the following bits must be kept cleared:
  - STOP and CLKEN bits in the USART_CTRL2 register,
  - HDSEL bits in the USART_CTRL3 register.*/
  CLEAR_BIT(USARTx->CR2, (USART_CR2_CLKEN | USART_CR2_STOP));
  CLEAR_BIT(USARTx->CR3, (USART_CR3_HDSEL));
  /* Set the USART/USART in LIN mode */
  SET_BIT(USARTx->CR2, USART_CR2_LINEN);
}

/**
  * @brief  Perform basic configuration of USART for enabling use in Half Duplex Mode
  * @note   In Half Duplex mode, the following bits must be kept cleared:
  *           - LINEN bit in the USART_CTRL2 register,
  *           - CLKEN bit in the USART_CTRL2 register,
  *         This function also sets the USART/USART in Half Duplex mode.
  * @note   Macro @ref IS_USART_HALFDUPLEX_INSTANCE(USARTx) can be used to check whether or not
  *         Half-Duplex mode is supported by the USARTx instance.
  * @note   Call of this function is equivalent to following function call sequence :
  *         - Clear LINEN in CTRL2 using @ref DDL_USART_DisableLIN() function
  *         - Clear CLKEN in CTRL2 using @ref DDL_USART_DisableSCLKOutput() function
  *         - Set HDSEL in CTRL3 using @ref DDL_USART_EnableHalfDuplex() function
  * @note   Other remaining configurations items related to Half Duplex Mode
  *         (as Baud Rate, Word length, Parity, ...) should be set using
  *         dedicated functions
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_ConfigHalfDuplexMode(USART_TypeDef *USARTx)
{
  /* In Half Duplex mode, the following bits must be kept cleared:
  - LINEN and CLKEN bits in the USART_CTRL2 register */
  CLEAR_BIT(USARTx->CR2, (USART_CR2_LINEN | USART_CR2_CLKEN));
  /* set the USART/USART in Half Duplex mode */
  SET_BIT(USARTx->CR3, USART_CR3_HDSEL);
}

/**
  * @brief  Perform basic configuration of USART for enabling use in Multi processor Mode
  *         (several USARTs connected in a network, one of the USARTs can be the master,
  *         its TX output connected to the RX inputs of the other slaves USARTs).
  * @note   In MultiProcessor mode, the following bits must be kept cleared:
  *           - LINEN bit in the USART_CTRL2 register,
  *           - CLKEN bit in the USART_CTRL2 register,
  *           - HDSEL bit in the USART_CTRL3 register.
  * @note   Call of this function is equivalent to following function call sequence :
  *         - Clear LINEN in CTRL2 using @ref DDL_USART_DisableLIN() function
  *         - Clear CLKEN in CTRL2 using @ref DDL_USART_DisableSCLKOutput() function
  *         - Clear HDSEL in CTRL3 using @ref DDL_USART_DisableHalfDuplex() function
  * @note   Other remaining configurations items related to Multi processor Mode
  *         (as Baud Rate, Wake Up Method, Node address, ...) should be set using
  *         dedicated functions
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_ConfigMultiProcessMode(USART_TypeDef *USARTx)
{
  /* In Multi Processor mode, the following bits must be kept cleared:
  - LINEN and CLKEN bits in the USART_CTRL2 register,
  - IREN, SCEN and HDSEL bits in the USART_CTRL3 register.*/
  CLEAR_BIT(USARTx->CR2, (USART_CR2_LINEN | USART_CR2_CLKEN));
  CLEAR_BIT(USARTx->CR3, (USART_CR3_HDSEL));
}

/**
  * @}
  */

/** @defgroup USART_DDL_EF_FLAG_Management FLAG_Management
  * @{
  */

/**
  * @brief  Check if the USART Parity Error Flag is set or not
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsActiveFlag_PE(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->SR, USART_SR_PEFLG) == (USART_SR_PEFLG));
}

/**
  * @brief  Check if the USART Framing Error Flag is set or not
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsActiveFlag_FE(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->SR, USART_SR_FEFLG) == (USART_SR_FEFLG));
}

/**
  * @brief  Check if the USART Noise error detected Flag is set or not
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsActiveFlag_NE(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->SR, USART_SR_NEFLG) == (USART_SR_NEFLG));
}

/**
  * @brief  Check if the USART OverRun Error Flag is set or not
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsActiveFlag_ORE(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->SR, USART_SR_OREFLG) == (USART_SR_OREFLG));
}

/**
  * @brief  Check if the USART IDLE line detected Flag is set or not
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsActiveFlag_IDLE(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->SR, USART_SR_IDLEFLG) == (USART_SR_IDLEFLG));
}

/**
  * @brief  Check if the USART Read Data Register Not Empty Flag is set or not
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsActiveFlag_RXNE(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->SR, USART_SR_RXNEFLG) == (USART_SR_RXNEFLG));
}

/**
  * @brief  Check if the USART Transmission Complete Flag is set or not
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsActiveFlag_TC(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->SR, USART_SR_TCFLG) == (USART_SR_TCFLG));
}

/**
  * @brief  Check if the USART Transmit Data Register Empty Flag is set or not
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsActiveFlag_TXE(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->SR, USART_SR_TXEFLG) == (USART_SR_TXEFLG));
}

/**
  * @brief  Check if the USART LIN Break Detection Flag is set or not
  * @note   Macro @ref IS_USART_LIN_INSTANCE(USARTx) can be used to check whether or not
  *         LIN feature is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsActiveFlag_LBD(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->SR, USART_SR_LBDFLG) == (USART_SR_LBDFLG));
}

/**
  * @brief  Check if the USART CTS Flag is set or not
  * @note   Macro @ref IS_USART_HWFLOW_INSTANCE(USARTx) can be used to check whether or not
  *         Hardware Flow control feature is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsActiveFlag_nCTS(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->SR, USART_SR_CTSFLG) == (USART_SR_CTSFLG));
}

/**
  * @brief  Check if the USART Receive overtime flag is set or not
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsActiveFlag_RTO(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->SR, USART_SR_RXTOFLG) == (USART_SR_RXTOFLG));
}

/**
  * @brief  Check if the USART Automatic baud rate complete flag is set or not
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsActiveFlag_ABR(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->SR, USART_SR_ABCFLG) == (USART_SR_ABCFLG));
}

/**
  * @brief  Check if the USART Automatic baud rate error flag is set or not
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsActiveFlag_ABRE(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->SR, USART_SR_ABERRFLG) == (USART_SR_ABERRFLG));
}

/**
  * @brief  Check if the USART Character matching flag is set or not
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsActiveFlag_CM(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->SR, USART_SR_CMFLG) == (USART_SR_CMFLG));
}

/**
  * @brief  Check if the USART Send Break Flag is set or not
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsActiveFlag_SBK(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->CR1, USART_CR1_SBK) == (USART_CR1_SBK));
}

/**
  * @brief  Check if the USART Receive Wake Up from mute mode Flag is set or not
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsActiveFlag_RWU(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->CR1, USART_CR1_RWU) == (USART_CR1_RWU));
}

/**
  * @brief  Clear Parity Error Flag
  * @note   Clearing this flag is done by a read access to the USARTx_SR
  *         register followed by a read access to the USARTx_DR register.
  * @note   Please also consider that when clearing this flag, other flags as
  *         NE, FE, ORE, IDLE would also be cleared.
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_ClearFlag_PE(USART_TypeDef *USARTx)
{
  __IO uint32_t tmpreg;
  tmpreg = USARTx->SR;
  (void) tmpreg;
  tmpreg = USARTx->DR;
  (void) tmpreg;
}

/**
  * @brief  Clear Framing Error Flag
  * @note   Clearing this flag is done by a read access to the USARTx_SR
  *         register followed by a read access to the USARTx_DR register.
  * @note   Please also consider that when clearing this flag, other flags as
  *         PE, NE, ORE, IDLE would also be cleared.
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_ClearFlag_FE(USART_TypeDef *USARTx)
{
  __IO uint32_t tmpreg;
  tmpreg = USARTx->SR;
  (void) tmpreg;
  tmpreg = USARTx->DR;
  (void) tmpreg;
}

/**
  * @brief  Clear Noise detected Flag
  * @note   Clearing this flag is done by a read access to the USARTx_SR
  *         register followed by a read access to the USARTx_DR register.
  * @note   Please also consider that when clearing this flag, other flags as
  *         PE, FE, ORE, IDLE would also be cleared.
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_ClearFlag_NE(USART_TypeDef *USARTx)
{
  __IO uint32_t tmpreg;
  tmpreg = USARTx->SR;
  (void) tmpreg;
  tmpreg = USARTx->DR;
  (void) tmpreg;
}

/**
  * @brief  Clear OverRun Error Flag
  * @note   Clearing this flag is done by a read access to the USARTx_SR
  *         register followed by a read access to the USARTx_DR register.
  * @note   Please also consider that when clearing this flag, other flags as
  *         PE, NE, FE, IDLE would also be cleared.
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_ClearFlag_ORE(USART_TypeDef *USARTx)
{
  __IO uint32_t tmpreg;
  tmpreg = USARTx->SR;
  (void) tmpreg;
  tmpreg = USARTx->DR;
  (void) tmpreg;
}

/**
  * @brief  Clear IDLE line detected Flag
  * @note   Clearing this flag is done by a read access to the USARTx_SR
  *         register followed by a read access to the USARTx_DR register.
  * @note   Please also consider that when clearing this flag, other flags as
  *         PE, NE, FE, ORE would also be cleared.
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_ClearFlag_IDLE(USART_TypeDef *USARTx)
{
  __IO uint32_t tmpreg;
  tmpreg = USARTx->SR;
  (void) tmpreg;
  tmpreg = USARTx->DR;
  (void) tmpreg;
}

/**
  * @brief  Clear Transmission Complete Flag
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_ClearFlag_TC(USART_TypeDef *USARTx)
{
  CLEAR_BIT(USARTx->SR, USART_SR_TCFLG);
}

/**
  * @brief  Clear RX Not Empty Flag
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_ClearFlag_RXNE(USART_TypeDef *USARTx)
{
  CLEAR_BIT(USARTx->SR, USART_SR_RXNEFLG);
}

/**
  * @brief  Clear LIN Break Detection Flag
  * @note   Macro @ref IS_USART_LIN_INSTANCE(USARTx) can be used to check whether or not
  *         LIN feature is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_ClearFlag_LBD(USART_TypeDef *USARTx)
{
  CLEAR_BIT(USARTx->SR, USART_SR_LBDFLG);
}

/**
  * @brief  Clear CTS Interrupt Flag
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_ClearFlag_nCTS(USART_TypeDef *USARTx)
{
  CLEAR_BIT(USARTx->SR, USART_SR_CTSFLG);
}

/**
  * @brief  Clear RX Time out Flag
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_ClearFlag_RTO(USART_TypeDef *USARTx)
{
  CLEAR_BIT(USARTx->SR, USART_SR_RXTOFLG);
}

/**
  * @brief  Clear  Automatic baud rate complete Flag
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_ClearFlag_ABR(USART_TypeDef *USARTx)
{
  CLEAR_BIT(USARTx->SR, USART_SR_ABCFLG);
}

/**
  * @brief  Clear  Automatic baud rate Flag
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_ClearFlag_ABRE(USART_TypeDef *USARTx)
{
  CLEAR_BIT(USARTx->SR, USART_SR_ABERRFLG);
}

/**
  * @brief  Clear character matching Flag
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_ClearFlag_CM(USART_TypeDef *USARTx)
{
  CLEAR_BIT(USARTx->SR, USART_SR_CMFLG);
}

/**
  * @}
  */

/** @defgroup USART_DDL_EF_IT_Management IT_Management
  * @{
  */

/**
  * @brief  Enable IDLE Interrupt
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_EnableIT_IDLE(USART_TypeDef *USARTx)
{
  SET_BIT(USARTx->CR1, USART_CR1_IDLEIEN);
}

/**
  * @brief  Enable RX Not Empty Interrupt
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_EnableIT_RXNE(USART_TypeDef *USARTx)
{
  SET_BIT(USARTx->CR1, USART_CR1_RXNEIEN);
}

/**
  * @brief  Enable Transmission Complete Interrupt
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_EnableIT_TC(USART_TypeDef *USARTx)
{
  SET_BIT(USARTx->CR1, USART_CR1_TCIEN);
}

/**
  * @brief  Enable TX Empty Interrupt
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_EnableIT_TXE(USART_TypeDef *USARTx)
{
  SET_BIT(USARTx->CR1, USART_CR1_TXEIEN);
}

/**
  * @brief  Enable Parity Error Interrupt
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_EnableIT_PE(USART_TypeDef *USARTx)
{
  SET_BIT(USARTx->CR1, USART_CR1_PEIEN);
}

/**
  * @brief  Enable character matching Interrupt
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_EnableIT_CM(USART_TypeDef *USARTx)
{
  SET_BIT(USARTx->CR1, USART_CR1_CMIEN);
}

/**
  * @brief  Receiver Timeout Interrupt Enable
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_EnableIT_RTO(USART_TypeDef *USARTx)
{
  SET_BIT(USARTx->CR1, USART_CR1_RXTOIEN);
}

/**
  * @brief  Enable LIN Break Detection Interrupt
  * @note   Macro @ref IS_USART_LIN_INSTANCE(USARTx) can be used to check whether or not
  *         LIN feature is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_EnableIT_LBD(USART_TypeDef *USARTx)
{
  SET_BIT(USARTx->CR2, USART_CR2_LBDIEN);
}

/**
  * @brief  Enable Automatic baud rate complete Interrupt
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_EnableIT_ABR(USART_TypeDef *USARTx)
{
  SET_BIT(USARTx->CR2, USART_CR2_ABCIEN);
}

/**
  * @brief  Enable Automatic baud rate detection error Interrupt
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_EnableIT_ABRE(USART_TypeDef *USARTx)
{
  SET_BIT(USARTx->CR2, USART_CR2_ABEIEN);
}

/**
  * @brief  Enable Error Interrupt
  * @note   When set, Error Interrupt Enable Bit is enabling interrupt generation in case of a framing
  *         error, overrun error or noise flag (FE=1 or ORE=1 or NF=1 in the USARTx_SR register).
  *           0: Interrupt is inhibited
  *           1: An interrupt is generated when FE=1 or ORE=1 or NF=1 in the USARTx_SR register.
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_EnableIT_ERROR(USART_TypeDef *USARTx)
{
  SET_BIT(USARTx->CR3, USART_CR3_EIEN);
}

/**
  * @brief  Enable CTS Interrupt
  * @note   Macro @ref IS_USART_HWFLOW_INSTANCE(USARTx) can be used to check whether or not
  *         Hardware Flow control feature is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_EnableIT_CTS(USART_TypeDef *USARTx)
{
  SET_BIT(USARTx->CR3, USART_CR3_CTSIEN);
}

/**
  * @brief  Disable IDLE Interrupt
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_DisableIT_IDLE(USART_TypeDef *USARTx)
{
  CLEAR_BIT(USARTx->CR1, USART_CR1_IDLEIEN);
}

/**
  * @brief  Disable RX Not Empty Interrupt
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_DisableIT_RXNE(USART_TypeDef *USARTx)
{
  CLEAR_BIT(USARTx->CR1, USART_CR1_RXNEIEN);
}

/**
  * @brief  Disable Transmission Complete Interrupt
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_DisableIT_TC(USART_TypeDef *USARTx)
{
  CLEAR_BIT(USARTx->CR1, USART_CR1_TCIEN);
}

/**
  * @brief  Disable TX Empty Interrupt
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_DisableIT_TXE(USART_TypeDef *USARTx)
{
  CLEAR_BIT(USARTx->CR1, USART_CR1_TXEIEN);
}

/**
  * @brief  Disable Parity Error Interrupt
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_DisableIT_PE(USART_TypeDef *USARTx)
{
  CLEAR_BIT(USARTx->CR1, USART_CR1_PEIEN);
}

/**
  * @brief  Disable character matching Interrupt
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_DisableIT_CM(USART_TypeDef *USARTx)
{
  CLEAR_BIT(USARTx->CR1, USART_CR1_CMIEN);
}

/**
  * @brief  Receiver Timeout Interrupt Disable
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_DisableIT_RTO(USART_TypeDef *USARTx)
{
  CLEAR_BIT(USARTx->CR1, USART_CR1_RXTOIEN);
}

/**
  * @brief  Disable LIN Break Detection Interrupt
  * @note   Macro @ref IS_USART_LIN_INSTANCE(USARTx) can be used to check whether or not
  *         LIN feature is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_DisableIT_LBD(USART_TypeDef *USARTx)
{
  CLEAR_BIT(USARTx->CR2, USART_CR2_LBDIEN);
}

/**
  * @brief  Disable Automatic bps complete Interrupt
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_DisableIT_ABR(USART_TypeDef *USARTx)
{
  CLEAR_BIT(USARTx->CR2, USART_CR2_ABCIEN);
}

/**
  * @brief  Disable Automatic bps detection error Interrupt
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_DisableIT_ABRE(USART_TypeDef *USARTx)
{
  CLEAR_BIT(USARTx->CR2, USART_CR2_ABEIEN);
}

/**
  * @brief  Disable Error Interrupt
  * @note   When set, Error Interrupt Enable Bit is enabling interrupt generation in case of a framing
  *         error, overrun error or noise flag (FE=1 or ORE=1 or NF=1 in the USARTx_SR register).
  *           0: Interrupt is inhibited
  *           1: An interrupt is generated when FE=1 or ORE=1 or NF=1 in the USARTx_SR register.
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_DisableIT_ERROR(USART_TypeDef *USARTx)
{
  CLEAR_BIT(USARTx->CR3, USART_CR3_EIEN);
}

/**
  * @brief  Disable CTS Interrupt
  * @note   Macro @ref IS_USART_HWFLOW_INSTANCE(USARTx) can be used to check whether or not
  *         Hardware Flow control feature is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_DisableIT_CTS(USART_TypeDef *USARTx)
{
  CLEAR_BIT(USARTx->CR3, USART_CR3_CTSIEN);
}

/**
  * @brief  Check if the USART IDLE Interrupt  source is enabled or disabled.
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsEnabledIT_IDLE(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->CR1, USART_CR1_IDLEIEN) == (USART_CR1_IDLEIEN));
}

/**
  * @brief  Check if the USART RX Not Empty Interrupt is enabled or disabled.
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsEnabledIT_RXNE(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->CR1, USART_CR1_RXNEIEN) == (USART_CR1_RXNEIEN));
}

/**
  * @brief  Check if the USART Transmission Complete Interrupt is enabled or disabled.
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsEnabledIT_TC(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->CR1, USART_CR1_TCIEN) == (USART_CR1_TCIEN));
}

/**
  * @brief  Check if the USART TX Empty Interrupt is enabled or disabled.
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsEnabledIT_TXE(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->CR1, USART_CR1_TXEIEN) == (USART_CR1_TXEIEN));
}

/**
  * @brief  Check if the USART Parity Error Interrupt is enabled or disabled.
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsEnabledIT_PE(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->CR1, USART_CR1_PEIEN) == (USART_CR1_PEIEN));
}

/**
  * @brief  Check if the USART character matching Interrupt is enabled or disabled.
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsEnabledIT_CM(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->CR1, USART_CR1_CMIEN) == (USART_CR1_CMIEN));
}

/**
  * @brief  Check if the USART LIN Break Detection Interrupt is enabled or disabled.
  * @note   Macro @ref IS_USART_LIN_INSTANCE(USARTx) can be used to check whether or not
  *         LIN feature is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsEnabledIT_LBD(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->CR2, USART_CR2_LBDIEN) == (USART_CR2_LBDIEN));
}

/**
  * @brief  Check if the USART Automatic baud rate complete Interrupt is enabled or disabled.
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsEnabledIT_ABR(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->CR2, USART_CR2_ABCIEN) == (USART_CR2_ABCIEN));
}

/**
  * @brief  Check if the USART Automatic baud rate error Interrupt is enabled or disabled.
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsEnabledIT_ABRE(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->CR2, USART_CR2_ABEIEN) == (USART_CR2_ABEIEN));
}

/**
  * @brief  Check if the USART CTS Interrupt is enabled or disabled.
  * @note   Macro @ref IS_USART_HWFLOW_INSTANCE(USARTx) can be used to check whether or not
  *         Hardware Flow control feature is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsEnabledIT_CTS(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->CR3, USART_CR3_CTSIEN) == (USART_CR3_CTSIEN));
}

/**
  * @}
  */

/** @defgroup USART_DDL_EF_DMA_Management DMA_Management
  * @{
  */

/**
  * @brief  Enable DMA Mode for reception
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_EnableDMAReq_RX(USART_TypeDef *USARTx)
{
  SET_BIT(USARTx->CR3, USART_CR3_DMAR);
}

/**
  * @brief  Disable DMA Mode for reception
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_DisableDMAReq_RX(USART_TypeDef *USARTx)
{
  CLEAR_BIT(USARTx->CR3, USART_CR3_DMAR);
}

/**
  * @brief  Check if DMA Mode is enabled for reception
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsEnabledDMAReq_RX(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->CR3, USART_CR3_DMAR) == (USART_CR3_DMAR));
}

/**
  * @brief  Enable DMA Mode for transmission
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_EnableDMAReq_TX(USART_TypeDef *USARTx)
{
  SET_BIT(USARTx->CR3, USART_CR3_DMAT);
}

/**
  * @brief  Disable DMA Mode for transmission
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_DisableDMAReq_TX(USART_TypeDef *USARTx)
{
  CLEAR_BIT(USARTx->CR3, USART_CR3_DMAT);
}

/**
  * @brief  Check if DMA Mode is enabled for transmission
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsEnabledDMAReq_TX(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->CR3, USART_CR3_DMAT) == (USART_CR3_DMAT));
}

/**
  * @brief  Get the data register address used for DMA transfer
  * @note   Address of Data Register is valid for both Transmit and Receive transfers.
  * @param  USARTx USART Instance
  * @retval Address of data register
  */
__STATIC_INLINE uint32_t DDL_USART_DMA_GetRegAddr(USART_TypeDef *USARTx)
{
  /* return address of DR register */
  return ((uint32_t) &(USARTx->DR));
}

/**
  * @}
  */

/** @defgroup USART_DDL_EF_Data_Management Data_Management
  * @{
  */

/**
  * @brief  Read Receiver Data register (Receive Data value, 8 bits)
  * @param  USARTx USART Instance
  * @retval Value between Min_Data=0x00 and Max_Data=0xFF
  */
__STATIC_INLINE uint8_t DDL_USART_ReceiveData8(USART_TypeDef *USARTx)
{
  return (uint8_t)(READ_BIT(USARTx->DR, USART_DR_DATA));
}

/**
  * @brief  Read Receiver Data register (Receive Data value, 9 bits)
  * @param  USARTx USART Instance
  * @retval Value between Min_Data=0x00 and Max_Data=0x1FF
  */
__STATIC_INLINE uint16_t DDL_USART_ReceiveData9(USART_TypeDef *USARTx)
{
  return (uint16_t)(READ_BIT(USARTx->DR, USART_DR_DATA));
}

/**
  * @brief  Write in Transmitter Data Register (Transmit Data value, 8 bits)
  * @param  USARTx USART Instance
  * @param  Value between Min_Data=0x00 and Max_Data=0xFF
  * @retval None
  */
__STATIC_INLINE void DDL_USART_TransmitData8(USART_TypeDef *USARTx, uint8_t Value)
{
  USARTx->DR = Value;
}

/**
  * @brief  Write in Transmitter Data Register (Transmit Data value, 9 bits)
  * @param  USARTx USART Instance
  * @param  Value between Min_Data=0x00 and Max_Data=0x1FF
  * @retval None
  */
__STATIC_INLINE void DDL_USART_TransmitData9(USART_TypeDef *USARTx, uint16_t Value)
{
  USARTx->DR = Value & 0x1FFU;
}

/**
  * @}
  */

/** @defgroup USART_DDL_EF_Execution Execution
  * @{
  */

/**
  * @brief  Request Break sending
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_RequestBreakSending(USART_TypeDef *USARTx)
{
  SET_BIT(USARTx->CR1, USART_CR1_SBK);
}

/**
  * @brief  Put USART in Mute mode
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_RequestEnterMuteMode(USART_TypeDef *USARTx)
{
  SET_BIT(USARTx->CR1, USART_CR1_RWU);
}

/**
  * @brief  Put USART in Active mode
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_RequestExitMuteMode(USART_TypeDef *USARTx)
{
  CLEAR_BIT(USARTx->CR1, USART_CR1_RWU);
}

/**
  * @}
  */

#if defined(USE_FULL_DDL_DRIVER)
/** @defgroup USART_DDL_EF_Init Initialization and de-initialization functions
  * @{
  */
ErrorStatus DDL_USART_DeInit(USART_TypeDef *USARTx);
ErrorStatus DDL_USART_Init(USART_TypeDef *USARTx, DDL_USART_InitTypeDef *USART_InitStruct);
void        DDL_USART_StructInit(DDL_USART_InitTypeDef *USART_InitStruct);
ErrorStatus DDL_USART_ClockInit(USART_TypeDef *USARTx, DDL_USART_ClockInitTypeDef *USART_ClockInitStruct);
void        DDL_USART_ClockStructInit(DDL_USART_ClockInitTypeDef *USART_ClockInitStruct);
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

#endif /* USART */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* G32F031_DDL_USART_H */

