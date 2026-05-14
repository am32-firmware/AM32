/**
  *
  * @file    g32f031_ddl_spi.h
  * @brief   Header file of SPI DDL module.
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
#ifndef G32F031_DDL_SPI_H
#define G32F031_DDL_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "g32f0xx.h"

/** @addtogroup G32F031_DDL_Driver
  * @{
  */
#if defined (SPI)

/** @defgroup SPI_DDL SPI
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
#if defined(USE_FULL_DDL_DRIVER)
/** @defgroup SPI_DDL_ES_INIT SPI Exported Init structure
  * @{
  */

/**
  * @brief  SPI Init structures definition
  */
typedef struct
{
  uint32_t TransferDirection;       /*!< Specifies the SPI unidirectional or bidirectional data mode.
                                         This parameter can be a value of @ref SPI_DDL_EC_TRANSFER_MODE.

                                         This feature can be modified afterwards using unitary function @ref DDL_SPI_SetTransferDirection().*/

  uint32_t Mode;                    /*!< Specifies the SPI mode (Master/Slave).
                                         This parameter can be a value of @ref SPI_DDL_EC_MODE.

                                         This feature can be modified afterwards using unitary function @ref DDL_SPI_SetMode().*/

  uint32_t DataWidth;               /*!< Specifies the SPI data width.
                                         This parameter can be a value of @ref SPI_DDL_EC_DATAWIDTH.

                                         This feature can be modified afterwards using unitary function @ref DDL_SPI_SetDataWidth().*/

  uint32_t ClockPolarity;           /*!< Specifies the serial clock steady state.
                                         This parameter can be a value of @ref SPI_DDL_EC_POLARITY.

                                         This feature can be modified afterwards using unitary function @ref DDL_SPI_SetClockPolarity().*/

  uint32_t ClockPhase;              /*!< Specifies the clock active edge for the bit capture.
                                         This parameter can be a value of @ref SPI_DDL_EC_PHASE.

                                         This feature can be modified afterwards using unitary function @ref DDL_SPI_SetClockPhase().*/

  uint32_t NSS;                     /*!< Specifies whether the NSS signal is managed by hardware (NSS pin) or by software using the SSI bit.
                                         This parameter can be a value of @ref SPI_DDL_EC_NSS_MODE.

                                         This feature can be modified afterwards using unitary function @ref DDL_SPI_SetNSSMode().*/

  uint32_t BaudRate;                /*!< Specifies the BaudRate prescaler value which will be used to configure the transmit and receive SCK clock.
                                         This parameter can be a value of @ref SPI_DDL_EC_BAUDRATEPRESCALER.
                                         @note The communication clock is derived from the master clock. The slave clock does not need to be set.

                                         This feature can be modified afterwards using unitary function @ref DDL_SPI_SetBaudRatePrescaler().*/

  uint32_t BitOrder;                /*!< Specifies whether data transfers start from MSB or LSB bit.
                                         This parameter can be a value of @ref SPI_DDL_EC_BIT_ORDER.

                                         This feature can be modified afterwards using unitary function @ref DDL_SPI_SetTransferBitOrder().*/
} DDL_SPI_InitTypeDef;

/**
  * @}
  */
#endif /* USE_FULL_DDL_DRIVER */

/* Exported constants --------------------------------------------------------*/
/** @defgroup SPI_DDL_Exported_Constants SPI Exported Constants
  * @{
  */

/** @defgroup SPI_DDL_EC_GET_FLAG Get Flags Defines
  * @brief    Flags defines which can be used with DDL_SPI_ReadReg function
  * @{
  */
#define DDL_SPI_SR_RXNE                SPI_SR_RXFIFONEFLG         /*!< Rx buffer not empty flag         */
#define DDL_SPI_SR_TXE                 SPI_SR_TXFIFOEFLG          /*!< Tx buffer empty flag             */
#define DDL_SPI_SR_BSY                 SPI_SR_BUSYFLG             /*!< Busy flag                        */
#define DDL_SPI_SR_OVRFLG                 SPI_SR_OVRFLG              /*!< Overrun flag                     */
/**
  * @}
  */

/** @defgroup SPI_DDL_EC_IT IT Defines
  * @brief    IT defines which can be used with DDL_SPI_ReadReg and  DDL_SPI_WriteReg functions
  * @{
  */
#define DDL_SPI_CR2_RXNEIE             SPI_CR2_RXFIFONEIEN     /*!< Rx buffer not empty interrupt enable */
#define DDL_SPI_CR2_TXEIE              SPI_CR2_TXFIFOEIEN      /*!< Tx buffer empty interrupt enable     */
#define DDL_SPI_CR2_OVRIE              SPI_CR2_OVRIEN          /*!< overflow interrupt enable               */
/**
  * @}
  */

/** @defgroup SPI_DDL_EC_MODE Operation Mode
  * @{
  */
#define DDL_SPI_MODE_MASTER                (SPI_CR1_MSTR | SPI_CR1_SSI)    /*!< Master configuration  */
#define DDL_SPI_MODE_SLAVE                  0x00000000U                            /*!< Slave configuration   */
/**
  * @}
  */

/** @defgroup SPI_DDL_EC_PHASE Clock Phase
  * @{
  */
#define DDL_SPI_PHASE_1EDGE                 0x00000000U               /*!< First clock transition is the first data capture edge  */
#define DDL_SPI_PHASE_2EDGE                 (SPI_CR1_CPHA)          /*!< Second clock transition is the first data capture edge */
/**
  * @}
  */

/** @defgroup SPI_DDL_EC_POLARITY Clock Polarity
  * @{
  */
#define DDL_SPI_POLARITY_LOW                0x00000000U               /*!< Clock to 0 when idle */
#define DDL_SPI_POLARITY_HIGH               (SPI_CR1_CPOL)          /*!< Clock to 1 when idle */
/**
  * @}
  */

/** @defgroup SPI_DDL_EC_BAUDRATEPRESCALER Baud Rate Prescaler
  * @{
  */
#define DDL_SPI_BAUDRATEPRESCALER_DIV2      0x00000000U                                            /*!< BaudRate control equal to fPCLK/2   */
#define DDL_SPI_BAUDRATEPRESCALER_DIV4      (SPI_CR1_BR_0)                                       /*!< BaudRate control equal to fPCLK/4   */
#define DDL_SPI_BAUDRATEPRESCALER_DIV8      (SPI_CR1_BR_1)                                       /*!< BaudRate control equal to fPCLK/8   */
#define DDL_SPI_BAUDRATEPRESCALER_DIV16     (SPI_CR1_BR_1 | SPI_CR1_BR_0)                      /*!< BaudRate control equal to fPCLK/16  */
#define DDL_SPI_BAUDRATEPRESCALER_DIV32     (SPI_CR1_BR_2)                                       /*!< BaudRate control equal to fPCLK/32  */
#define DDL_SPI_BAUDRATEPRESCALER_DIV64     (SPI_CR1_BR_2 | SPI_CR1_BR_0)                      /*!< BaudRate control equal to fPCLK/64  */
#define DDL_SPI_BAUDRATEPRESCALER_DIV128    (SPI_CR1_BR_2 | SPI_CR1_BR_1)                      /*!< BaudRate control equal to fPCLK/128 */
#define DDL_SPI_BAUDRATEPRESCALER_DIV256    (SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0)     /*!< BaudRate control equal to fPCLK/256 */
/**
  * @}
  */

/** @defgroup SPI_DDL_EC_BIT_ORDER Transmission Bit Order
  * @{
  */
#define DDL_SPI_LSB_FIRST                   (SPI_CR1_LSBFIRST)       /*!< Data is transmitted/received with the LSB first */
#define DDL_SPI_MSB_FIRST                   0x00000000U               /*!< Data is transmitted/received with the MSB first */
/**
  * @}
  */

/** @defgroup SPI_DDL_EC_TRANSFER_MODE Transfer Mode
  * @{
  */
#define DDL_SPI_FULL_DUPLEX                 0x00000000U                              /*!< Full-Duplex mode. Rx and Tx transfer on 2 lines */
#define DDL_SPI_SIMPLEX_RX                  (SPI_CR1_RXONLY)                       /*!< Simplex Rx mode.  Rx transfer only on 1 line    */
#define DDL_SPI_HALF_DUPLEX_RX              (SPI_CR1_BIDIMEN)                     /*!< Half-Duplex Rx mode. Rx transfer on 1 line      */
#define DDL_SPI_HALF_DUPLEX_TX              (SPI_CR1_BIDIMEN | SPI_CR1_BIDIOEN)  /*!< Half-Duplex Tx mode. Tx transfer on 1 line      */
/**
  * @}
  */

/** @defgroup SPI_DDL_EC_NSS_MODE Slave Select Pin Mode
  * @{
  */
#define DDL_SPI_NSS_SOFT                    (SPI_CR1_SSM)                     /*!< NSS managed internally. NSS pin not used and free              */
#define DDL_SPI_NSS_HARD_INPUT              0x00000000U                          /*!< NSS pin used in Input. Only used in Master mode                */
/**
  * @}
  */

/** @defgroup SPI_DDL_EC_DATAWIDTH Datawidth
  * @{
  */
#define DDL_SPI_DATAWIDTH_8BIT              0x00000000U                       /*!< Data length for SPI transfer:  8 bits */
#define DDL_SPI_DATAWIDTH_16BIT             (SPI_CR1_DFF)                /*!< Data length for SPI transfer:  16 bits */
/**
  * @}
  */

/** @defgroup SPI_DDL_EC_RXFIFO_LEVEL Receive FIFO Level
  * @{
  */
#define DDL_SPI_RECEIVE_FIFO_LEVEL_EMPTY              0x00000000U                           /*!< Receive FIFO empty */
#define DDL_SPI_RECEIVE_FIFO_LEVEL_1_4                (SPI_SR_RXFIFOVAL_0)                    /*!< Receive FIFO 1/4 full */
#define DDL_SPI_RECEIVE_FIFO_LEVEL_1_2                (SPI_SR_RXFIFOVAL_1)                    /*!< Receive FIFO 1/2 full */
#define DDL_SPI_RECEIVE_FIFO_LEVEL_FULL               (SPI_SR_RXFIFOVAL_1 | SPI_SR_RXFIFOVAL_0) /*!< Receive FIFO full */
/**
  * @}
  */

/** @defgroup SPI_DDL_EC_TXFIFO_LEVEL Transmit FIFO Level
  * @{
  */
#define DDL_SPI_TRANSMIT_FIFO_LEVEL_EMPTY              0x00000000U                           /*!< Transmit FIFO empty */
#define DDL_SPI_TRANSMIT_FIFO_LEVEL_1_4                (SPI_SR_TXFIFOVAL_0)                    /*!< Transmit FIFO 1/4 full */
#define DDL_SPI_TRANSMIT_FIFO_LEVEL_1_2                (SPI_SR_TXFIFOVAL_1)                    /*!< Transmit FIFO 1/2 full */
#define DDL_SPI_TRANSMIT_FIFO_LEVEL_FULL               (SPI_SR_TXFIFOVAL_1 | SPI_SR_TXFIFOVAL_0) /*!< Transmit FIFO full */
/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup SPI_DDL_Exported_Macros SPI Exported Macros
  * @{
  */

/** @defgroup SPI_DDL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in SPI register
  * @param  __INSTANCE__ SPI Instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define DDL_SPI_WriteReg(__INSTANCE__, __REG__, __VALUE__) WRITE_REG(__INSTANCE__->__REG__, (__VALUE__))

/**
  * @brief  Read a value in SPI register
  * @param  __INSTANCE__ SPI Instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define DDL_SPI_ReadReg(__INSTANCE__, __REG__) READ_REG(__INSTANCE__->__REG__)
/**
  * @}
  */

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup SPI_DDL_Exported_Functions SPI Exported Functions
  * @{
  */

/** @defgroup SPI_DDL_EF_Configuration Configuration
  * @{
  */

/**
  * @brief  Enable SPI peripheral
  * @param  SPIx SPI Instance
  * @retval None
  */
__STATIC_INLINE void DDL_SPI_Enable(SPI_TypeDef *SPIx)
{
  SET_BIT(SPIx->CR1, SPI_CR1_SPIEN);
}

/**
  * @brief  Disable SPI peripheral
  * @param  SPIx SPI Instance
  * @retval None
  */
__STATIC_INLINE void DDL_SPI_Disable(SPI_TypeDef *SPIx)
{
  CLEAR_BIT(SPIx->CR1, SPI_CR1_SPIEN);
}

/**
  * @brief  Check if SPI peripheral is enabled
  * @param  SPIx SPI Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_SPI_IsEnabled(SPI_TypeDef *SPIx)
{
  return ((READ_BIT(SPIx->CR1, SPI_CR1_SPIEN) == (SPI_CR1_SPIEN)) ? 1UL : 0UL);
}

/**
  * @brief  Set SPI operation mode to Master or Slave
  * @param  SPIx SPI Instance
  * @param  Mode This parameter can be one of the following values:
  *         @arg @ref DDL_SPI_MODE_MASTER
  *         @arg @ref DDL_SPI_MODE_SLAVE
  * @retval None
  */
__STATIC_INLINE void DDL_SPI_SetMode(SPI_TypeDef *SPIx, uint32_t Mode)
{
  MODIFY_REG(SPIx->CR1, SPI_CR1_MSTR | SPI_CR1_SSI, Mode);
}

/**
  * @brief  Get SPI operation mode (Master or Slave)
  * @param  SPIx SPI Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_SPI_MODE_MASTER
  *         @arg @ref DDL_SPI_MODE_SLAVE
  */
__STATIC_INLINE uint32_t DDL_SPI_GetMode(SPI_TypeDef *SPIx)
{
  return (uint32_t)(READ_BIT(SPIx->CR1, SPI_CR1_MSTR | SPI_CR1_SSI));
}

/**
  * @brief  Set clock phase
  * @param  SPIx SPI Instance
  * @param  ClockPhase This parameter can be one of the following values:
  *         @arg @ref DDL_SPI_PHASE_1EDGE
  *         @arg @ref DDL_SPI_PHASE_2EDGE
  * @retval None
  */
__STATIC_INLINE void DDL_SPI_SetClockPhase(SPI_TypeDef *SPIx, uint32_t ClockPhase)
{
  MODIFY_REG(SPIx->CR1, SPI_CR1_CPHA, ClockPhase);
}

/**
  * @brief  Get clock phase
  * @param  SPIx SPI Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_SPI_PHASE_1EDGE
  *         @arg @ref DDL_SPI_PHASE_2EDGE
  */
__STATIC_INLINE uint32_t DDL_SPI_GetClockPhase(SPI_TypeDef *SPIx)
{
  return (uint32_t)(READ_BIT(SPIx->CR1, SPI_CR1_CPHA));
}

/**
  * @brief  Set clock polarity
  * @param  SPIx SPI Instance
  * @param  ClockPolarity This parameter can be one of the following values:
  *         @arg @ref DDL_SPI_POLARITY_LOW
  *         @arg @ref DDL_SPI_POLARITY_HIGH
  * @retval None
  */
__STATIC_INLINE void DDL_SPI_SetClockPolarity(SPI_TypeDef *SPIx, uint32_t ClockPolarity)
{
  MODIFY_REG(SPIx->CR1, SPI_CR1_CPOL, ClockPolarity);
}

/**
  * @brief  Get clock polarity
  * @param  SPIx SPI Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_SPI_POLARITY_LOW
  *         @arg @ref DDL_SPI_POLARITY_HIGH
  */
__STATIC_INLINE uint32_t DDL_SPI_GetClockPolarity(SPI_TypeDef *SPIx)
{
  return (uint32_t)(READ_BIT(SPIx->CR1, SPI_CR1_CPOL));
}

/**
  * @brief  Set baud rate prescaler
  * @param  SPIx SPI Instance
  * @param  BaudRate This parameter can be one of the following values:
  *         @arg @ref DDL_SPI_BAUDRATEPRESCALER_DIV2
  *         @arg @ref DDL_SPI_BAUDRATEPRESCALER_DIV4
  *         @arg @ref DDL_SPI_BAUDRATEPRESCALER_DIV8
  *         @arg @ref DDL_SPI_BAUDRATEPRESCALER_DIV16
  *         @arg @ref DDL_SPI_BAUDRATEPRESCALER_DIV32
  *         @arg @ref DDL_SPI_BAUDRATEPRESCALER_DIV64
  *         @arg @ref DDL_SPI_BAUDRATEPRESCALER_DIV128
  *         @arg @ref DDL_SPI_BAUDRATEPRESCALER_DIV256
  * @retval None
  */
__STATIC_INLINE void DDL_SPI_SetBaudRatePrescaler(SPI_TypeDef *SPIx, uint32_t BaudRate)
{
  MODIFY_REG(SPIx->CR1, SPI_CR1_BR, BaudRate);
}

/**
  * @brief  Get baud rate prescaler
  * @param  SPIx SPI Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_SPI_BAUDRATEPRESCALER_DIV2
  *         @arg @ref DDL_SPI_BAUDRATEPRESCALER_DIV4
  *         @arg @ref DDL_SPI_BAUDRATEPRESCALER_DIV8
  *         @arg @ref DDL_SPI_BAUDRATEPRESCALER_DIV16
  *         @arg @ref DDL_SPI_BAUDRATEPRESCALER_DIV32
  *         @arg @ref DDL_SPI_BAUDRATEPRESCALER_DIV64
  *         @arg @ref DDL_SPI_BAUDRATEPRESCALER_DIV128
  *         @arg @ref DDL_SPI_BAUDRATEPRESCALER_DIV256
  */
__STATIC_INLINE uint32_t DDL_SPI_GetBaudRatePrescaler(SPI_TypeDef *SPIx)
{
  return (uint32_t)(READ_BIT(SPIx->CR1, SPI_CR1_BR));
}

/**
  * @brief  Set transfer bit order
  * @param  SPIx SPI Instance
  * @param  BitOrder This parameter can be one of the following values:
  *         @arg @ref DDL_SPI_LSB_FIRST
  *         @arg @ref DDL_SPI_MSB_FIRST
  * @retval None
  */
__STATIC_INLINE void DDL_SPI_SetTransferBitOrder(SPI_TypeDef *SPIx, uint32_t BitOrder)
{
  MODIFY_REG(SPIx->CR1, SPI_CR1_LSBFIRST, BitOrder);
}

/**
  * @brief  Get transfer bit order
  * @param  SPIx SPI Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_SPI_LSB_FIRST
  *         @arg @ref DDL_SPI_MSB_FIRST
  */
__STATIC_INLINE uint32_t DDL_SPI_GetTransferBitOrder(SPI_TypeDef *SPIx)
{
  return (uint32_t)(READ_BIT(SPIx->CR1, SPI_CR1_LSBFIRST));
}

/**
  * @brief  Set transfer direction mode
  * @param  SPIx SPI Instance
  * @param  TransferDirection This parameter can be one of the following values:
  *         @arg @ref DDL_SPI_FULL_DUPLEX
  *         @arg @ref DDL_SPI_SIMPLEX_RX
  *         @arg @ref DDL_SPI_HALF_DUPLEX_RX
  *         @arg @ref DDL_SPI_HALF_DUPLEX_TX
  * @retval None
  */
__STATIC_INLINE void DDL_SPI_SetTransferDirection(SPI_TypeDef *SPIx, uint32_t TransferDirection)
{
  MODIFY_REG(SPIx->CR1, SPI_CR1_RXONLY | SPI_CR1_BIDIOEN | SPI_CR1_BIDIMEN, TransferDirection);
}

/**
  * @brief  Get transfer direction mode
  * @param  SPIx SPI Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_SPI_FULL_DUPLEX
  *         @arg @ref DDL_SPI_SIMPLEX_RX
  *         @arg @ref DDL_SPI_HALF_DUPLEX_RX
  *         @arg @ref DDL_SPI_HALF_DUPLEX_TX
  */
__STATIC_INLINE uint32_t DDL_SPI_GetTransferDirection(SPI_TypeDef *SPIx)
{
  return (uint32_t)(READ_BIT(SPIx->CR1, SPI_CR1_RXONLY | SPI_CR1_BIDIOEN | SPI_CR1_BIDIMEN));
}

/**
  * @brief  Set frame data width
  * @param  SPIx SPI Instance
  * @param  DataWidth This parameter can be one of the following values:
  *         @arg @ref DDL_SPI_DATAWIDTH_8BIT
  *         @arg @ref DDL_SPI_DATAWIDTH_16BIT
  * @retval None
  */
__STATIC_INLINE void DDL_SPI_SetDataWidth(SPI_TypeDef *SPIx, uint32_t DataWidth)
{
  MODIFY_REG(SPIx->CR1, SPI_CR1_DFF, DataWidth);
}

/**
  * @brief  Get frame data width
  * @param  SPIx SPI Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_SPI_DATAWIDTH_8BIT
  *         @arg @ref DDL_SPI_DATAWIDTH_16BIT
  */
__STATIC_INLINE uint32_t DDL_SPI_GetDataWidth(SPI_TypeDef *SPIx)
{
  return (uint32_t)(READ_BIT(SPIx->CR1, SPI_CR1_DFF));
}

/**
  * @}
  */

/** @defgroup SPI_DDL_EF_NSS_Management Slave Select Pin Management
  * @{
  */

/**
  * @brief  Set NSS mode
  * @param  SPIx SPI Instance
  * @param  NSS This parameter can be one of the following values:
  *         @arg @ref DDL_SPI_NSS_SOFT
  *         @arg @ref DDL_SPI_NSS_HARD_INPUT
  * @retval None
  */
__STATIC_INLINE void DDL_SPI_SetNSSMode(SPI_TypeDef *SPIx, uint32_t NSS)
{
  MODIFY_REG(SPIx->CR1, SPI_CR1_SSM,  NSS);
}

/**
  * @brief  Get NSS mode
  * @param  SPIx SPI Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_SPI_NSS_SOFT
  *         @arg @ref DDL_SPI_NSS_HARD_OUTPUT
  */
__STATIC_INLINE uint32_t DDL_SPI_GetNSSMode(SPI_TypeDef *SPIx)
{
  return (uint32_t)(READ_BIT(SPIx->CR1, SPI_CR1_SSM));
}

/**
  * @}
  */

/** @defgroup SPI_DDL_EF_FLAG_Management FLAG Management
  * @{
  */

/**
  * @brief  Check if Rx buffer is not empty
  * @param  SPIx SPI Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_SPI_IsActiveFlag_RXNE(SPI_TypeDef *SPIx)
{
  return ((READ_BIT(SPIx->SR, SPI_SR_RXFIFONEFLG) == (SPI_SR_RXFIFONEFLG)) ? 1UL : 0UL);
}

/**
  * @brief  Check if Tx buffer is empty
  * @param  SPIx SPI Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_SPI_IsActiveFlag_TXE(SPI_TypeDef *SPIx)
{
  return ((READ_BIT(SPIx->SR, SPI_SR_TXFIFOEFLG) == (SPI_SR_TXFIFOEFLG)) ? 1UL : 0UL);
}

/**
  * @brief  Get overrun error flag
  * @param  SPIx SPI Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_SPI_IsActiveFlag_OVR(SPI_TypeDef *SPIx)
{
  return ((READ_BIT(SPIx->SR, SPI_SR_OVRFLG) == (SPI_SR_OVRFLG)) ? 1UL : 0UL);
}

/**
  * @brief  Get busy flag
  * @param  SPIx SPI Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_SPI_IsActiveFlag_BUSY(SPI_TypeDef *SPIx)
{
  return ((READ_BIT(SPIx->SR, SPI_SR_BUSYFLG) == (SPI_SR_BUSYFLG)) ? 1UL : 0UL);
}


/**
 * @brief Retrieves the current receive FIFO level of a SPI instance.
 * @param  SPIx  Pointer to the SPI instance.
 *
 * @retval The current receive FIFO level as a uint32_t value.
 *         @arg @ref DDL_SPI_RECEIVE_FIFO_LEVEL_EMPTY
 *         @arg @ref DDL_SPI_RECEIVE_FIFO_LEVEL_1_4
 *         @arg @ref DDL_SPI_RECEIVE_FIFO_LEVEL_1_2
 *         @arg @ref DDL_SPI_RECEIVE_FIFO_LEVEL_FULL
 */
__STATIC_INLINE uint32_t DDL_SPI_GetReceiveFIFOLevel(SPI_TypeDef* SPIx)
{
  return (uint32_t)(READ_BIT(SPIx->SR, SPI_SR_RXFIFOVAL));
}

/**
 * @brief Retrieves the current transmit FIFO level of a SPI instance.
 * @param  SPIx  Pointer to the SPI instance.
 *
 * @retval The current transmit FIFO level as a uint32_t value.
 *         @arg @ref DDL_SPI_TRANSMIT_FIFO_LEVEL_EMPTY
 *         @arg @ref DDL_SPI_TRANSMIT_FIFO_LEVEL_1_4
 *         @arg @ref DDL_SPI_TRANSMIT_FIFO_LEVEL_1_2
 *         @arg @ref DDL_SPI_TRANSMIT_FIFO_LEVEL_FULL
 */
__STATIC_INLINE uint32_t DDL_SPI_GetTransmitFIFOLevel(SPI_TypeDef* SPIx)
{
  return (uint32_t)(READ_BIT(SPIx->SR, SPI_SR_TXFIFOVAL));
}

/**
  * @brief  Clear overrun error flag
  * @param  SPIx SPI Instance
  * @retval None
  */
__STATIC_INLINE void DDL_SPI_ClearFlag_OVR(SPI_TypeDef *SPIx)
{
  __IO uint32_t tmpreg;
  tmpreg = SPIx->DR;
  (void) tmpreg;
  tmpreg = SPIx->SR;
  (void) tmpreg;
}

/**
  * @}
  */

/** @defgroup SPI_DDL_EF_IT_Management Interrupt Management
  * @{
  */


/**
 * @brief Enable the SPI overflow interrupt.
 * @param SPIx Pointer to the SPI peripheral instance.
 *
 * @retval None
 */
__STATIC_INLINE void DDL_SPI_EnableIT_OVR(SPI_TypeDef *SPIx)
{
  SET_BIT(SPIx->CR2, SPI_CR2_OVRIEN);
}

/**
  * @brief  Enable Rx buffer not empty interrupt
  * @param  SPIx SPI Instance
  * @retval None
  */
__STATIC_INLINE void DDL_SPI_EnableIT_RXNE(SPI_TypeDef *SPIx)
{
  SET_BIT(SPIx->CR2, SPI_CR2_RXFIFONEIEN);
}

/**
  * @brief  Enable Tx buffer empty interrupt
  * @param  SPIx SPI Instance
  * @retval None
  */
__STATIC_INLINE void DDL_SPI_EnableIT_TXE(SPI_TypeDef *SPIx)
{
  SET_BIT(SPIx->CR2, SPI_CR2_TXFIFOEIEN);
}

/**
  * @brief  Disable overflow interrupt
  * @param  SPIx SPI Instance
  * @retval None
  */
__STATIC_INLINE void DDL_SPI_DisableIT_OVR(SPI_TypeDef *SPIx)
{
  CLEAR_BIT(SPIx->CR2, SPI_CR2_OVRIEN);
}

/**
  * @brief  Disable Rx buffer not empty interrupt
  * @param  SPIx SPI Instance
  * @retval None
  */
__STATIC_INLINE void DDL_SPI_DisableIT_RXNE(SPI_TypeDef *SPIx)
{
  CLEAR_BIT(SPIx->CR2, SPI_CR2_RXFIFONEIEN);
}

/**
  * @brief  Disable Tx buffer empty interrupt
  * @param  SPIx SPI Instance
  * @retval None
  */
__STATIC_INLINE void DDL_SPI_DisableIT_TXE(SPI_TypeDef *SPIx)
{
  CLEAR_BIT(SPIx->CR2, SPI_CR2_TXFIFOEIEN);
}

/**
  * @brief  Check if overflow interrupt is enabled
  * @param  SPIx SPI Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_SPI_IsEnabledIT_OVR(SPI_TypeDef *SPIx)
{
  return ((READ_BIT(SPIx->CR2, SPI_CR2_OVRIEN) == (SPI_CR2_OVRIEN)) ? 1UL : 0UL);
}

/**
  * @brief  Check if Rx buffer not empty interrupt is enabled
  * @param  SPIx SPI Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_SPI_IsEnabledIT_RXNE(SPI_TypeDef *SPIx)
{
  return ((READ_BIT(SPIx->CR2, SPI_CR2_RXFIFONEIEN) == (SPI_CR2_RXFIFONEIEN)) ? 1UL : 0UL);
}

/**
  * @brief  Check if Tx buffer empty interrupt
  * @param  SPIx SPI Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_SPI_IsEnabledIT_TXE(SPI_TypeDef *SPIx)
{
  return ((READ_BIT(SPIx->CR2, SPI_CR2_TXFIFOEIEN) == (SPI_CR2_TXFIFOEIEN)) ? 1UL : 0UL);
}

/**
  * @}
  */

/** @defgroup SPI_DDL_EF_DMA_Management DMA Management
  * @{
  */

/**
  * @brief  Enable DMA Rx
  * @param  SPIx SPI Instance
  * @retval None
  */
__STATIC_INLINE void DDL_SPI_EnableDMAReq_RX(SPI_TypeDef *SPIx)
{
  SET_BIT(SPIx->CR2, SPI_CR2_RXDMAEN);
}

/**
  * @brief  Disable DMA Rx
  * @param  SPIx SPI Instance
  * @retval None
  */
__STATIC_INLINE void DDL_SPI_DisableDMAReq_RX(SPI_TypeDef *SPIx)
{
  CLEAR_BIT(SPIx->CR2, SPI_CR2_RXDMAEN);
}

/**
  * @brief  Check if DMA Rx is enabled
  * @param  SPIx SPI Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_SPI_IsEnabledDMAReq_RX(SPI_TypeDef *SPIx)
{
  return ((READ_BIT(SPIx->CR2, SPI_CR2_RXDMAEN) == (SPI_CR2_RXDMAEN)) ? 1UL : 0UL);
}

/**
  * @brief  Enable DMA Tx
  * @param  SPIx SPI Instance
  * @retval None
  */
__STATIC_INLINE void DDL_SPI_EnableDMAReq_TX(SPI_TypeDef *SPIx)
{
  SET_BIT(SPIx->CR2, SPI_CR2_TXDMAEN);
}

/**
  * @brief  Disable DMA Tx
  * @param  SPIx SPI Instance
  * @retval None
  */
__STATIC_INLINE void DDL_SPI_DisableDMAReq_TX(SPI_TypeDef *SPIx)
{
  CLEAR_BIT(SPIx->CR2, SPI_CR2_TXDMAEN);
}

/**
  * @brief  Check if DMA Tx is enabled
  * @param  SPIx SPI Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_SPI_IsEnabledDMAReq_TX(SPI_TypeDef *SPIx)
{
  return ((READ_BIT(SPIx->CR2, SPI_CR2_TXDMAEN) == (SPI_CR2_TXDMAEN)) ? 1UL : 0UL);
}

/**
  * @brief  Get the data register address used for DMA transfer
  * @param  SPIx SPI Instance
  * @retval Address of data register
  */
__STATIC_INLINE uint32_t DDL_SPI_DMA_GetRegAddr(SPI_TypeDef *SPIx)
{
  return (uint32_t) &(SPIx->DR);
}

/**
  * @}
  */

/** @defgroup SPI_DDL_EF_DATA_Management DATA Management
  * @{
  */

/**
  * @brief  Read 8-Bits in the data register
  * @param  SPIx SPI Instance
  * @retval RxData Value between Min_Data=0x00 and Max_Data=0xFF
  */
__STATIC_INLINE uint8_t DDL_SPI_ReceiveData8(SPI_TypeDef *SPIx)
{
  return (uint8_t)(READ_REG(SPIx->DR));
}

/**
  * @brief  Read 16-Bits in the data register
  * @param  SPIx SPI Instance
  * @retval RxData Value between Min_Data=0x00 and Max_Data=0xFFFF
  */
__STATIC_INLINE uint16_t DDL_SPI_ReceiveData16(SPI_TypeDef *SPIx)
{
  return (uint16_t)(READ_REG(SPIx->DR));
}

/**
  * @brief  Write 8-Bits in the data register
  * @param  SPIx SPI Instance
  * @param  TxData Value between Min_Data=0x00 and Max_Data=0xFF
  * @retval None
  */
__STATIC_INLINE void DDL_SPI_TransmitData8(SPI_TypeDef *SPIx, uint8_t TxData)
{
  SPIx->DR = TxData;
}

/**
  * @brief  Write 16-Bits in the data register
  * @param  SPIx SPI Instance
  * @param  TxData Value between Min_Data=0x00 and Max_Data=0xFFFF
  * @retval None
  */
__STATIC_INLINE void DDL_SPI_TransmitData16(SPI_TypeDef *SPIx, uint16_t TxData)
{
  SPIx->DR = TxData;
}

/**
  * @}
  */
#if defined(USE_FULL_DDL_DRIVER)
/** @defgroup SPI_DDL_EF_Init Initialization and de-initialization functions
  * @{
  */

ErrorStatus DDL_SPI_DeInit(SPI_TypeDef *SPIx);
ErrorStatus DDL_SPI_Init(SPI_TypeDef *SPIx, DDL_SPI_InitTypeDef *SPI_InitStruct);
void        DDL_SPI_StructInit(DDL_SPI_InitTypeDef *SPI_InitStruct);

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

#endif /* SPI */
/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* G32F031_DDL_SPI_H */

