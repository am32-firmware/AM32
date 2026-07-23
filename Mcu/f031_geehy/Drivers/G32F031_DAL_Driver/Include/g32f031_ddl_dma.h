/**
  *
  * @file    g32f031_ddl_dma.h
  * @brief   Header file of DMA DDL module.
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
  * Copyright (C) 2026 Geehy Semiconductor.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file in
  * the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  *
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef G32F031_DDL_DMA_H
#define G32F031_DDL_DMA_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "g32f0xx.h"

/** @addtogroup G32F031_DDL_Driver
  * @{
  */
#if defined (DMA)

/** @defgroup DMA_DDL DMA
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/** @defgroup DMA_DDL_Private_Variables DMA Private Variables
  * @{
  */
/* Array used to get the DMA channel register offset versus channel index DDL_DMA_CHANNEL_x */
static const uint8_t CHANNEL_OFFSET_TAB[] =
{
  (uint8_t)(DMA_Channel0_BASE - DMA_BASE),
  (uint8_t)(DMA_Channel1_BASE - DMA_BASE),
};
/**
  * @}
  */

/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
#if defined(USE_FULL_DDL_DRIVER)
/** @defgroup DMA_DDL_ES_INIT DMA Exported Init structure
  * @{
  */
typedef struct
{
  uint32_t PeriphOrM2MSrcAddress;  /*!< Specifies the peripheral base address for DMA transfer
                                        or as Source base address in case of memory to memory transfer direction.

                                        This parameter must be a value between Min_Data = 0 and Max_Data = 0xFFFFFFFF. */

  uint32_t MemoryOrM2MDstAddress;  /*!< Specifies the memory base address for DMA transfer
                                        or as Destination base address in case of memory to memory transfer direction.

                                        This parameter must be a value between Min_Data = 0 and Max_Data = 0xFFFFFFFF. */

  uint32_t Direction;              /*!< Specifies if the data will be transferred from memory to peripheral,
                                        from memory to memory or from peripheral to memory.
                                        This parameter can be a value of @ref DMA_DDL_EC_DIRECTION

                                        This feature can be modified afterwards using unitary function @ref DDL_DMA_SetDataTransferDirection(). */

  uint32_t Mode;                   /*!< Specifies the normal or circular operation mode.
                                        This parameter can be a value of @ref DMA_DDL_EC_MODE
                                        @note The circular buffer mode cannot be used if the memory to memory
                                              data transfer direction is configured on the selected Channel

                                        This feature can be modified afterwards using unitary function @ref DDL_DMA_SetMode(). */

  uint32_t PeriphOrM2MSrcIncMode;  /*!< Specifies whether the Peripheral address or Source address in case of memory to memory transfer direction
                                        is incremented or not.
                                        This parameter can be a value of @ref DMA_DDL_EC_PERIPH

                                        This feature can be modified afterwards using unitary function @ref DDL_DMA_SetPeriphIncMode(). */

  uint32_t MemoryOrM2MDstIncMode;  /*!< Specifies whether the Memory address or Destination address in case of memory to memory transfer direction
                                        is incremented or not.
                                        This parameter can be a value of @ref DMA_DDL_EC_MEMORY

                                        This feature can be modified afterwards using unitary function @ref DDL_DMA_SetMemoryIncMode(). */

  uint32_t PeriphOrM2MSrcDataSize; /*!< Specifies the Peripheral data size alignment or Source data size alignment (byte, half word, word)
                                        in case of memory to memory transfer direction.
                                        This parameter can be a value of @ref DMA_DDL_EC_PDATAALIGN

                                        This feature can be modified afterwards using unitary function @ref DDL_DMA_SetPeriphSize(). */

  uint32_t MemoryOrM2MDstDataSize; /*!< Specifies the Memory data size alignment or Destination data size alignment (byte, half word, word)
                                        in case of memory to memory transfer direction.
                                        This parameter can be a value of @ref DMA_DDL_EC_MDATAALIGN

                                        This feature can be modified afterwards using unitary function @ref DDL_DMA_SetMemorySize(). */

  uint32_t NbData;                 /*!< Specifies the number of data to transfer, in data unit.
                                        The data unit is equal to the source buffer configuration set in PeripheralSize
                                        or MemorySize parameters depending in the transfer direction.
                                        This parameter must be a value between Min_Data = 0 and Max_Data = 0x0000FFFF

                                        This feature can be modified afterwards using unitary function @ref DDL_DMA_SetDataLength(). */

  uint32_t Priority;               /*!< Specifies the channel priority level.
                                        This parameter can be a value of @ref DMA_DDL_EC_PRIORITY

                                        This feature can be modified afterwards using unitary function @ref DDL_DMA_SetChannelPriorityLevel(). */

  uint32_t FIFOMode;               /*!< Specifies if the FIFO mode or Direct mode will be used for the specified channel.
                                        This parameter can be a value of @ref DMA_DDL_FIFOMODE
                                        @note The Direct mode (FIFO mode disabled) cannot be used if the
                                        memory-to-memory data transfer is configured on the selected channel

                                        This feature can be modified afterwards using unitary functions @ref DDL_DMA_EnableFifoMode() or @ref DDL_DMA_EnableFifoMode() . */

  uint32_t FIFOThreshold;          /*!< Specifies the FIFO threshold level.
                                        This parameter can be a value of @ref DMA_DDL_EC_FIFOTHRESHOLD

                                        This feature can be modified afterwards using unitary function @ref DDL_DMA_SetFIFOThreshold(). */

  uint32_t MemBurst;               /*!< Specifies the Burst transfer configuration for the memory transfers.
                                        It specifies the amount of data to be transferred in a single non interruptible
                                        transaction.
                                        This parameter can be a value of @ref DMA_DDL_EC_MBURST
                                        @note The burst mode is possible only if the address Increment mode is enabled.

                                        This feature can be modified afterwards using unitary function @ref DDL_DMA_SetMemoryBurstxfer(). */

  uint32_t PeriphBurst;            /*!< Specifies the Burst transfer configuration for the peripheral transfers.
                                        It specifies the amount of data to be transferred in a single non interruptible
                                        transaction.
                                        This parameter can be a value of @ref DMA_DDL_EC_PBURST
                                        @note The burst mode is possible only if the address Increment mode is enabled.

                                        This feature can be modified afterwards using unitary function @ref DDL_DMA_SetPeriphBurstxfer(). */

  uint32_t Peripheral;             /*!< Specifies the peripheral corresponding to the channel.
                                        It specifies the amount of data to be transferred in a single non interruptible
                                        transaction.
                                        This parameter can be a value of @ref DMA_DDL_EC_PERIPHERAL
                                        @note The burst mode is possible only if the address Increment mode is enabled.

                                        This feature can be modified afterwards using unitary function @ref DDL_DMA_SetPeripheralSelection(). */

} DDL_DMA_InitTypeDef;
/**
  * @}
  */
#endif /*USE_FULL_DDL_DRIVER*/
/* Exported constants --------------------------------------------------------*/
/** @defgroup DMA_DDL_Exported_Constants DMA Exported Constants
  * @{
  */

/** @defgroup DMA_DDL_EC_CHANNEL CHANNEL
  * @{
  */
#define DDL_DMA_CHANNEL_0                  0x00000000U
#define DDL_DMA_CHANNEL_1                  0x00000001U
#define DDL_DMA_CHANNEL_ALL                0xFFFF0000U
/**
  * @}
  */

/** @defgroup DMA_DDL_EC_DIRECTION DIRECTION
  * @{
  */
#define DDL_DMA_DIRECTION_PERIPH_TO_MEMORY 0x00000000U                /*!< Peripheral to memory direction */
#define DDL_DMA_DIRECTION_MEMORY_TO_PERIPH DMA_SCFGx_DIRCFG_0         /*!< Memory to peripheral direction */
#define DDL_DMA_DIRECTION_MEMORY_TO_MEMORY DMA_SCFGx_DIRCFG_1         /*!< Memory to memory direction     */
/**
  * @}
  */

/** @defgroup DMA_DDL_EC_MODE MODE
  * @{
  */
#define DDL_DMA_MODE_NORMAL                0x00000000U                /*!< Normal Mode                  */
#define DDL_DMA_MODE_CIRCULAR              DMA_SCFGx_CIRCMEN          /*!< Circular Mode                */
/**
  * @}
  */

/** @defgroup DMA_DDL_EC_DOUBLEBUFFER_MODE DOUBLEBUFFER MODE
  * @{
  */
#define DDL_DMA_DOUBLEBUFFER_MODE_DISABLE  0x00000000U                /*!< Disable double buffering mode */
#define DDL_DMA_DOUBLEBUFFER_MODE_ENABLE   DMA_SCFGx_DBM              /*!< Enable double buffering mode  */
/**
  * @}
  */

/** @defgroup DMA_DDL_EC_PERIPH PERIPH
  * @{
  */
#define DDL_DMA_PERIPH_NOINCREMENT         0x00000000U                /*!< Peripheral increment mode Disable */
#define DDL_DMA_PERIPH_INCREMENT           DMA_SCFGx_PINCM            /*!< Peripheral increment mode Enable  */
/**
  * @}
  */

/** @defgroup DMA_DDL_EC_MEMORY MEMORY
  * @{
  */
#define DDL_DMA_MEMORY_NOINCREMENT         0x00000000U                /*!< Memory increment mode Disable */
#define DDL_DMA_MEMORY_INCREMENT           DMA_SCFGx_MINCM            /*!< Memory increment mode Enable  */
/**
  * @}
  */

/** @defgroup DMA_DDL_EC_PDATAALIGN PDATAALIGN
  * @{
  */
#define DDL_DMA_PDATAALIGN_BYTE            0x00000000U                /*!< Peripheral data alignment : Byte     */
#define DDL_DMA_PDATAALIGN_HALFWORD        DMA_SCFGx_PSIZECFG_0     /*!< Peripheral data alignment : HalfWord */
#define DDL_DMA_PDATAALIGN_WORD            DMA_SCFGx_PSIZECFG_1     /*!< Peripheral data alignment : Word     */
/**
  * @}
  */

/** @defgroup DMA_DDL_EC_MDATAALIGN MDATAALIGN
  * @{
  */
#define DDL_DMA_MDATAALIGN_BYTE            0x00000000U                /*!< Memory data alignment : Byte     */
#define DDL_DMA_MDATAALIGN_HALFWORD        DMA_SCFGx_MSIZECFG_0     /*!< Memory data alignment : HalfWord */
#define DDL_DMA_MDATAALIGN_WORD            DMA_SCFGx_MSIZECFG_1     /*!< Memory data alignment : Word     */
/**
  * @}
  */

/** @defgroup DMA_DDL_EC_OFFSETSIZE OFFSETSIZE
  * @{
  */
#define DDL_DMA_OFFSETSIZE_PSIZE           0x00000000U                /*!< Peripheral increment offset size is linked to the PSIZE */
#define DDL_DMA_OFFSETSIZE_FIXEDTO4        DMA_SCFGx_PERISIZE        /*!< Peripheral increment offset size is fixed to 4 (32-bit alignment) */
/**
  * @}
  */

/** @defgroup DMA_DDL_EC_PRIORITY PRIORITY
  * @{
  */
#define DDL_DMA_PRIORITY_LOW               0x00000000U                /*!< Priority level : Low */
#define DDL_DMA_PRIORITY_HIGH              DMA_SCFGx_PRILCFG          /*!< Priority level : High */
/**
  * @}
  */

/** @defgroup DMA_DDL_EC_PERIPHERAL PERIPHERAL
  * @{
  */
#define DDL_DMA_PERIPHERAL_0               0x00000000U                                              /* Select Peripheral0 of DMA Instance */
#define DDL_DMA_PERIPHERAL_1               (DMA_SCFGx_PSEL_0)                                       /* Select Peripheral1 of DMA Instance */
#define DDL_DMA_PERIPHERAL_2               (DMA_SCFGx_PSEL_1)                                       /* Select Peripheral2 of DMA Instance */
#define DDL_DMA_PERIPHERAL_3               (DMA_SCFGx_PSEL_1 | DMA_SCFGx_PSEL_0)                    /* Select Peripheral3 of DMA Instance */
#define DDL_DMA_PERIPHERAL_4               (DMA_SCFGx_PSEL_2)                                       /* Select Peripheral4 of DMA Instance */
#define DDL_DMA_PERIPHERAL_5               (DMA_SCFGx_PSEL_2 | DMA_SCFGx_PSEL_0)                    /* Select Peripheral5 of DMA Instance */
#define DDL_DMA_PERIPHERAL_6               (DMA_SCFGx_PSEL_2 | DMA_SCFGx_PSEL_1)                    /* Select Peripheral6 of DMA Instance */
#define DDL_DMA_PERIPHERAL_7               (DMA_SCFGx_PSEL_2 | DMA_SCFGx_PSEL_1 | DMA_SCFGx_PSEL_0) /* Select Peripheral7 of DMA Instance */
#define DDL_DMA_PERIPHERAL_8               (DMA_SCFGx_PSEL_3)                                       /* Select Peripheral8 of DMA Instance */
#define DDL_DMA_PERIPHERAL_9               (DMA_SCFGx_PSEL_3 | DMA_SCFGx_PSEL_0)                    /* Select Peripheral9 of DMA Instance */
#define DDL_DMA_PERIPHERAL_10              (DMA_SCFGx_PSEL_3 | DMA_SCFGx_PSEL_1)                    /* Select Peripheral10 of DMA Instance */
#define DDL_DMA_PERIPHERAL_11              (DMA_SCFGx_PSEL_3 | DMA_SCFGx_PSEL_1 | DMA_SCFGx_PSEL_0) /* Select Peripheral11 of DMA Instance */
#define DDL_DMA_PERIPHERAL_12              (DMA_SCFGx_PSEL_3 | DMA_SCFGx_PSEL_2)                    /* Select Peripheral12 of DMA Instance */
#define DDL_DMA_PERIPHERAL_13              (DMA_SCFGx_PSEL_3 | DMA_SCFGx_PSEL_2 | DMA_SCFGx_PSEL_0) /* Select Peripheral13 of DMA Instance */
#define DDL_DMA_PERIPHERAL_14              (DMA_SCFGx_PSEL_3 | DMA_SCFGx_PSEL_2 | DMA_SCFGx_PSEL_1) /* Select Peripheral14 of DMA Instance */
#define DDL_DMA_PERIPHERAL_15              (DMA_SCFGx_PSEL)                                         /* Select Peripheral15 of DMA Instance */
/**
  * @}
  */

/** @defgroup DMA_DDL_EC_MBURST MBURST
  * @{
  */
#define DDL_DMA_MBURST_SINGLE              0x00000000U                             /*!< Memory burst single transfer configuration */
#define DDL_DMA_MBURST_INC4                DMA_SCFGx_MBCFG_0                       /*!< Memory burst of 4 beats transfer configuration */
#define DDL_DMA_MBURST_INC8                DMA_SCFGx_MBCFG_1                       /*!< Memory burst of 8 beats transfer configuration */
#define DDL_DMA_MBURST_INC16               (DMA_SCFGx_MBCFG_0 | DMA_SCFGx_MBCFG_1) /*!< Memory burst of 16 beats transfer configuration */
/**
  * @}
  */

/** @defgroup DMA_DDL_EC_PBURST PBURST
  * @{
  */
#define DDL_DMA_PBURST_SINGLE              0x00000000U                             /*!< Peripheral burst single transfer configuration */
#define DDL_DMA_PBURST_INC4                DMA_SCFGx_PBCFG_0                       /*!< Peripheral burst of 4 beats transfer configuration */
#define DDL_DMA_PBURST_INC8                DMA_SCFGx_PBCFG_1                       /*!< Peripheral burst of 8 beats transfer configuration */
#define DDL_DMA_PBURST_INC16               (DMA_SCFGx_PBCFG_0 | DMA_SCFGx_PBCFG_1) /*!< Peripheral burst of 16 beats transfer configuration */
/**
  * @}
  */

/** @defgroup DMA_DDL_FIFOMODE DMA_DDL_FIFOMODE
  * @{
  */
#define DDL_DMA_FIFOMODE_DISABLE           0x00000000U                             /*!< FIFO mode disable (direct mode is enabled) */
#define DDL_DMA_FIFOMODE_ENABLE            DMA_FIFOCRx_DMDEN                        /*!< FIFO mode enable  */
/**
  * @}
  */

/** @defgroup DMA_DDL_EC_FIFOSTATUS_0 FIFOSTATUS 0
  * @{
  */
#define DDL_DMA_FIFOSTATUS_0_25            0x00000000U                             /*!< 0 < fifo_level < 1/4    */
#define DDL_DMA_FIFOSTATUS_25_50           DMA_FIFOCRx_FSTS_0                       /*!< 1/4 < fifo_level < 1/2  */
#define DDL_DMA_FIFOSTATUS_50_75           DMA_FIFOCRx_FSTS_1                       /*!< 1/2 < fifo_level < 3/4  */
#define DDL_DMA_FIFOSTATUS_75_100          (DMA_FIFOCRx_FSTS_1 | DMA_FIFOCRx_FSTS_0) /*!< 3/4 < fifo_level < full */
#define DDL_DMA_FIFOSTATUS_EMPTY           DMA_FIFOCRx_FSTS_2                       /*!< FIFO is empty           */
#define DDL_DMA_FIFOSTATUS_FULL            (DMA_FIFOCRx_FSTS_2 | DMA_FIFOCRx_FSTS_0) /*!< FIFO is full            */
/**
  * @}
  */

/** @defgroup DMA_DDL_EC_FIFOTHRESHOLD FIFOTHRESHOLD
  * @{
  */
#define DDL_DMA_FIFOTHRESHOLD_1_4          0x00000000U                             /*!< FIFO threshold 1 quart full configuration  */
#define DDL_DMA_FIFOTHRESHOLD_1_2          DMA_FIFOCRx_FTHSEL_0                     /*!< FIFO threshold half full configuration     */
#define DDL_DMA_FIFOTHRESHOLD_3_4          DMA_FIFOCRx_FTHSEL_1                     /*!< FIFO threshold 3 quarts full configuration */
#define DDL_DMA_FIFOTHRESHOLD_FULL         DMA_FIFOCRx_FTHSEL                       /*!< FIFO threshold full configuration          */
/**
  * @}
  */

/** @defgroup DMA_DDL_EC_CURRENTTARGETMEM CURRENTTARGETMEM
  * @{
  */
#define DDL_DMA_CURRENTTARGETMEM0          0x00000000U                             /*!< Set CurrentTarget Memory to Memory 0  */
#define DDL_DMA_CURRENTTARGETMEM1          DMA_SCFGx_CTARG                         /*!< Set CurrentTarget Memory to Memory 1  */
/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup DMA_DDL_Exported_Macros DMA Exported Macros
  * @{
  */

/** @defgroup DMA_DDL_EM_WRITE_READ Common Write and read registers macros
  * @{
  */
/**
  * @brief  Write a value in DMA register
  * @param  __INSTANCE__ DMA Instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define DDL_DMA_WriteReg(__INSTANCE__, __REG__, __VALUE__) WRITE_REG(__INSTANCE__->__REG__, (__VALUE__))

/**
  * @brief  Read a value in DMA register
  * @param  __INSTANCE__ DMA Instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define DDL_DMA_ReadReg(__INSTANCE__, __REG__) READ_REG(__INSTANCE__->__REG__)

/**
  * @brief  Get channel address in DMA register
  * @param  __INSTANCE__ DMA Instance
  * @param  __CHANNEL__ Channel num
  * @retval Current channel register address
  */
#define __DDL_DMA_GET_CHANNEL_INSTANCE(__INSTANCE__, __CHANNEL__) (uint32_t)((uint32_t)__INSTANCE__ + CHANNEL_OFFSET_TAB[__CHANNEL__])

/**
  * @}
  */

/**
  * @}
  */


/* Exported functions --------------------------------------------------------*/
 /** @defgroup DMA_DDL_Exported_Functions DMA Exported Functions
  * @{
  */

/** @defgroup DMA_DDL_EF_Configuration Configuration
  * @{
  */
/**
  * @brief Enable DMA channel.
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_EnableChannel(DMA_TypeDef *DMAx, uint32_t Channel)
{
  SET_BIT(((DMA_Channel_TypeDef *)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->SCFG, DMA_SCFGx_CHEN);
}

/**
  * @brief Disable DMA channel.
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_DisableChannel(DMA_TypeDef *DMAx, uint32_t Channel)
{
  CLEAR_BIT(((DMA_Channel_TypeDef *)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->SCFG, DMA_SCFGx_CHEN);
}

/**
  * @brief Check if DMA channel is enabled or disabled.
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DMA_IsEnabledChannel(DMA_TypeDef *DMAx, uint32_t Channel)
{
  return (READ_BIT(((DMA_Channel_TypeDef*)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->SCFG, DMA_SCFGx_CHEN) == (DMA_SCFGx_CHEN));
}

/**
  * @brief  Configure all parameters linked to DMA transfer.
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @param  Configuration This parameter must be a combination of all the following values:
  *         @arg @ref DDL_DMA_DIRECTION_PERIPH_TO_MEMORY or @ref DDL_DMA_DIRECTION_MEMORY_TO_PERIPH or @ref DDL_DMA_DIRECTION_MEMORY_TO_MEMORY
  *         @arg @ref DDL_DMA_MODE_NORMAL or @ref DDL_DMA_MODE_CIRCULAR
  *         @arg @ref DDL_DMA_PERIPH_INCREMENT or @ref DDL_DMA_PERIPH_NOINCREMENT
  *         @arg @ref DDL_DMA_MEMORY_INCREMENT or @ref DDL_DMA_MEMORY_NOINCREMENT
  *         @arg @ref DDL_DMA_PDATAALIGN_BYTE or @ref DDL_DMA_PDATAALIGN_HALFWORD or @ref DDL_DMA_PDATAALIGN_WORD
  *         @arg @ref DDL_DMA_MDATAALIGN_BYTE or @ref DDL_DMA_MDATAALIGN_HALFWORD or @ref DDL_DMA_MDATAALIGN_WORD
  *         @arg @ref DDL_DMA_PRIORITY_LOW or @ref DDL_DMA_PRIORITY_HIGH
  *@retval None
  */
__STATIC_INLINE void DDL_DMA_ConfigTransfer(DMA_TypeDef *DMAx, uint32_t Channel, uint32_t Configuration)
{
  MODIFY_REG(((DMA_Channel_TypeDef*)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->SCFG,
             DMA_SCFGx_DIRCFG | DMA_SCFGx_CIRCMEN | DMA_SCFGx_PINCM | DMA_SCFGx_MINCM | DMA_SCFGx_PSIZECFG | DMA_SCFGx_MSIZECFG | DMA_SCFGx_PRILCFG,
             Configuration);
}

/**
  * @brief Set Data transfer direction (read from peripheral or from memory).
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @param  Direction This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_DIRECTION_PERIPH_TO_MEMORY
  *         @arg @ref DDL_DMA_DIRECTION_MEMORY_TO_PERIPH
  *         @arg @ref DDL_DMA_DIRECTION_MEMORY_TO_MEMORY
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_SetDataTransferDirection(DMA_TypeDef *DMAx, uint32_t Channel, uint32_t  Direction)
{
  MODIFY_REG(((DMA_Channel_TypeDef*)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->SCFG, DMA_SCFGx_DIRCFG, Direction);
}

/**
  * @brief Get Data transfer direction (read from peripheral or from memory).
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_DMA_DIRECTION_PERIPH_TO_MEMORY
  *         @arg @ref DDL_DMA_DIRECTION_MEMORY_TO_PERIPH
  *         @arg @ref DDL_DMA_DIRECTION_MEMORY_TO_MEMORY
  */
__STATIC_INLINE uint32_t DDL_DMA_GetDataTransferDirection(DMA_TypeDef *DMAx, uint32_t Channel)
{
  return (READ_BIT(((DMA_Channel_TypeDef*)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->SCFG, DMA_SCFGx_DIRCFG));
}

/**
  * @brief Set DMA mode normal, circular or peripheral flow control.
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @param  Mode This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_MODE_NORMAL
  *         @arg @ref DDL_DMA_MODE_CIRCULAR
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_SetMode(DMA_TypeDef *DMAx, uint32_t Channel, uint32_t Mode)
{
  MODIFY_REG(((DMA_Channel_TypeDef*)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->SCFG, DMA_SCFGx_CIRCMEN, Mode);
}

/**
  * @brief Get DMA mode normal, circular or peripheral flow control.
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_DMA_MODE_NORMAL
  *         @arg @ref DDL_DMA_MODE_CIRCULAR
  */
__STATIC_INLINE uint32_t DDL_DMA_GetMode(DMA_TypeDef *DMAx, uint32_t Channel)
{
  return (READ_BIT(((DMA_Channel_TypeDef *)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->SCFG, DMA_SCFGx_CIRCMEN));
}

/**
  * @brief  Set Peripheral increment mode.
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @param  IncrementMode This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_PERIPH_NOINCREMENT
  *         @arg @ref DDL_DMA_PERIPH_INCREMENT
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_SetPeriphIncMode(DMA_TypeDef *DMAx, uint32_t Channel, uint32_t IncrementMode)
{
  MODIFY_REG(((DMA_Channel_TypeDef*)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->SCFG, DMA_SCFGx_PINCM, IncrementMode);
}

/**
  * @brief Get Peripheral increment mode.
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_DMA_PERIPH_NOINCREMENT
  *         @arg @ref DDL_DMA_PERIPH_INCREMENT
  */
__STATIC_INLINE uint32_t DDL_DMA_GetPeriphIncMode(DMA_TypeDef *DMAx, uint32_t Channel)
{
  return (READ_BIT(((DMA_Channel_TypeDef*)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->SCFG, DMA_SCFGx_PINCM));
}

/**
  * @brief Set Memory increment mode.
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @param  IncrementMode This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_MEMORY_NOINCREMENT
  *         @arg @ref DDL_DMA_MEMORY_INCREMENT
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_SetMemoryIncMode(DMA_TypeDef *DMAx, uint32_t Channel, uint32_t IncrementMode)
{
  MODIFY_REG(((DMA_Channel_TypeDef*)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->SCFG, DMA_SCFGx_MINCM, IncrementMode);
}

/**
  * @brief Get Memory increment mode.
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_DMA_MEMORY_NOINCREMENT
  *         @arg @ref DDL_DMA_MEMORY_INCREMENT
  */
__STATIC_INLINE uint32_t DDL_DMA_GetMemoryIncMode(DMA_TypeDef *DMAx, uint32_t Channel)
{
  return (READ_BIT(((DMA_Channel_TypeDef*)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->SCFG, DMA_SCFGx_MINCM));
}

/**
  * @brief Set Peripheral size.
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @param  Size This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_PDATAALIGN_BYTE
  *         @arg @ref DDL_DMA_PDATAALIGN_HALFWORD
  *         @arg @ref DDL_DMA_PDATAALIGN_WORD
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_SetPeriphSize(DMA_TypeDef *DMAx, uint32_t Channel, uint32_t  Size)
{
  MODIFY_REG(((DMA_Channel_TypeDef*)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->SCFG, DMA_SCFGx_PSIZECFG, Size);
}

/**
  * @brief Get Peripheral size.
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_DMA_PDATAALIGN_BYTE
  *         @arg @ref DDL_DMA_PDATAALIGN_HALFWORD
  *         @arg @ref DDL_DMA_PDATAALIGN_WORD
  */
__STATIC_INLINE uint32_t DDL_DMA_GetPeriphSize(DMA_TypeDef *DMAx, uint32_t Channel)
{
  return (READ_BIT(((DMA_Channel_TypeDef*)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->SCFG, DMA_SCFGx_PSIZECFG));
}

/**
  * @brief Set Memory size.
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @param  Size This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_MDATAALIGN_BYTE
  *         @arg @ref DDL_DMA_MDATAALIGN_HALFWORD
  *         @arg @ref DDL_DMA_MDATAALIGN_WORD
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_SetMemorySize(DMA_TypeDef *DMAx, uint32_t Channel, uint32_t  Size)
{
  MODIFY_REG(((DMA_Channel_TypeDef*)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->SCFG, DMA_SCFGx_MSIZECFG, Size);
}

/**
  * @brief Get Memory size.
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_DMA_MDATAALIGN_BYTE
  *         @arg @ref DDL_DMA_MDATAALIGN_HALFWORD
  *         @arg @ref DDL_DMA_MDATAALIGN_WORD
  */
__STATIC_INLINE uint32_t DDL_DMA_GetMemorySize(DMA_TypeDef *DMAx, uint32_t Channel)
{
  return (READ_BIT(((DMA_Channel_TypeDef*)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->SCFG, DMA_SCFGx_MSIZECFG));
}

/**
  * @brief Set Peripheral increment offset size.
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @param  OffsetSize This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_OFFSETSIZE_PSIZE
  *         @arg @ref DDL_DMA_OFFSETSIZE_FIXEDTO4
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_SetIncOffsetSize(DMA_TypeDef *DMAx, uint32_t Channel, uint32_t OffsetSize)
{
  MODIFY_REG(((DMA_Channel_TypeDef*)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->SCFG, DMA_SCFGx_PERISIZE, OffsetSize);
}

/**
  * @brief Get Peripheral increment offset size.
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_DMA_OFFSETSIZE_PSIZE
  *         @arg @ref DDL_DMA_OFFSETSIZE_FIXEDTO4
  */
__STATIC_INLINE uint32_t DDL_DMA_GetIncOffsetSize(DMA_TypeDef *DMAx, uint32_t Channel)
{
  return (READ_BIT(((DMA_Channel_TypeDef*)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->SCFG, DMA_SCFGx_PERISIZE));
}

/**
  * @brief Set Channel priority level.
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @param  Priority This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_PRIORITY_LOW
  *         @arg @ref DDL_DMA_PRIORITY_HIGH
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_SetChannelPriorityLevel(DMA_TypeDef *DMAx, uint32_t Channel, uint32_t  Priority)
{
  MODIFY_REG(((DMA_Channel_TypeDef*)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->SCFG, DMA_SCFGx_PRILCFG, Priority);
}

/**
  * @brief Get Channel priority level.
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_DMA_PRIORITY_LOW
  *         @arg @ref DDL_DMA_PRIORITY_HIGH
  */
__STATIC_INLINE uint32_t DDL_DMA_GetChannelPriorityLevel(DMA_TypeDef *DMAx, uint32_t Channel)
{
  return (READ_BIT(((DMA_Channel_TypeDef*)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->SCFG, DMA_SCFGx_PRILCFG));
}

/**
  * @brief Set Number of data to transfer.
  * @note   This action has no effect if
  *         channel is enabled.
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @param  NbData Between 0 to 0xFFFF
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_SetDataLength(DMA_TypeDef * DMAx, uint32_t Channel, uint32_t NbData)
{
  MODIFY_REG(((DMA_Channel_TypeDef *)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->NDATA, DMA_NDATAx_NDATA, NbData);
}

/**
  * @brief Get Number of data to transfer.
  * @note   Once the channel is enabled, the return value indicate the
  *         remaining bytes to be transmitted.
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @retval Between 0 to 0xFFFF
  */
__STATIC_INLINE uint32_t DDL_DMA_GetDataLength(DMA_TypeDef * DMAx, uint32_t Channel)
{
  return (READ_BIT(((DMA_Channel_TypeDef *)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->NDATA, DMA_NDATAx_NDATA));
}

/**
  * @brief Select Peripheral number associated to the Channel.
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_PERIPHERAL_0
  *         @arg @ref DDL_DMA_PERIPHERAL_1
  *         @arg @ref DDL_DMA_PERIPHERAL_2
  *         @arg @ref DDL_DMA_PERIPHERAL_3
  *         @arg @ref DDL_DMA_PERIPHERAL_4
  *         @arg @ref DDL_DMA_PERIPHERAL_5
  *         @arg @ref DDL_DMA_PERIPHERAL_6
  *         @arg @ref DDL_DMA_PERIPHERAL_7
  *         @arg @ref DDL_DMA_PERIPHERAL_8
  *         @arg @ref DDL_DMA_PERIPHERAL_9
  *         @arg @ref DDL_DMA_PERIPHERAL_10
  *         @arg @ref DDL_DMA_PERIPHERAL_11
  *         @arg @ref DDL_DMA_PERIPHERAL_12
  *         @arg @ref DDL_DMA_PERIPHERAL_13
  *         @arg @ref DDL_DMA_PERIPHERAL_14
  *         @arg @ref DDL_DMA_PERIPHERAL_15
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_SetPeripheralSelection(DMA_TypeDef *DMAx, uint32_t Channel, uint32_t Peripheral)
{
  MODIFY_REG(((DMA_Channel_TypeDef*)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->SCFG, DMA_SCFGx_PSEL, Peripheral);
}

/**
  * @brief Get the Peripheral number associated to the Channel.
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_DMA_PERIPHERAL_0
  *         @arg @ref DDL_DMA_PERIPHERAL_1
  *         @arg @ref DDL_DMA_PERIPHERAL_2
  *         @arg @ref DDL_DMA_PERIPHERAL_3
  *         @arg @ref DDL_DMA_PERIPHERAL_4
  *         @arg @ref DDL_DMA_PERIPHERAL_5
  *         @arg @ref DDL_DMA_PERIPHERAL_6
  *         @arg @ref DDL_DMA_PERIPHERAL_7
  *         @arg @ref DDL_DMA_PERIPHERAL_8
  *         @arg @ref DDL_DMA_PERIPHERAL_9
  *         @arg @ref DDL_DMA_PERIPHERAL_10
  *         @arg @ref DDL_DMA_PERIPHERAL_11
  *         @arg @ref DDL_DMA_PERIPHERAL_12
  *         @arg @ref DDL_DMA_PERIPHERAL_13
  *         @arg @ref DDL_DMA_PERIPHERAL_14
  *         @arg @ref DDL_DMA_PERIPHERAL_15
  */
__STATIC_INLINE uint32_t DDL_DMA_GetPeripheralSelection(DMA_TypeDef *DMAx, uint32_t Channel)
{
  return (READ_BIT(((DMA_Channel_TypeDef*)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->SCFG, DMA_SCFGx_PSEL));
}

/**
  * @brief Set Memory burst transfer configuration.
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @param  Mburst This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_MBURST_SINGLE
  *         @arg @ref DDL_DMA_MBURST_INC4
  *         @arg @ref DDL_DMA_MBURST_INC8
  *         @arg @ref DDL_DMA_MBURST_INC16
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_SetMemoryBurstxfer(DMA_TypeDef *DMAx, uint32_t Channel, uint32_t Mburst)
{
  MODIFY_REG(((DMA_Channel_TypeDef*)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->SCFG, DMA_SCFGx_MBCFG, Mburst);
}

/**
  * @brief Get Memory burst transfer configuration.
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_DMA_MBURST_SINGLE
  *         @arg @ref DDL_DMA_MBURST_INC4
  *         @arg @ref DDL_DMA_MBURST_INC8
  *         @arg @ref DDL_DMA_MBURST_INC16
  */
__STATIC_INLINE uint32_t DDL_DMA_GetMemoryBurstxfer(DMA_TypeDef *DMAx, uint32_t Channel)
{
  return (READ_BIT(((DMA_Channel_TypeDef*)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->SCFG, DMA_SCFGx_MBCFG));
}

/**
  * @brief Set  Peripheral burst transfer configuration.
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @param  Pburst This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_PBURST_SINGLE
  *         @arg @ref DDL_DMA_PBURST_INC4
  *         @arg @ref DDL_DMA_PBURST_INC8
  *         @arg @ref DDL_DMA_PBURST_INC16
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_SetPeriphBurstxfer(DMA_TypeDef *DMAx, uint32_t Channel, uint32_t Pburst)
{
  MODIFY_REG(((DMA_Channel_TypeDef*)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->SCFG, DMA_SCFGx_PBCFG, Pburst);
}

/**
  * @brief Get Peripheral burst transfer configuration.
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_DMA_PBURST_SINGLE
  *         @arg @ref DDL_DMA_PBURST_INC4
  *         @arg @ref DDL_DMA_PBURST_INC8
  *         @arg @ref DDL_DMA_PBURST_INC16
  */
__STATIC_INLINE uint32_t DDL_DMA_GetPeriphBurstxfer(DMA_TypeDef *DMAx, uint32_t Channel)
{
  return (READ_BIT(((DMA_Channel_TypeDef*)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->SCFG, DMA_SCFGx_PBCFG));
}

/**
  * @brief Set Current target (only in double buffer mode) to Memory 1 or Memory 0.
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @param CurrentMemory This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CURRENTTARGETMEM0
  *         @arg @ref DDL_DMA_CURRENTTARGETMEM1
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_SetCurrentTargetMem(DMA_TypeDef *DMAx, uint32_t Channel, uint32_t CurrentMemory)
{
   MODIFY_REG(((DMA_Channel_TypeDef*)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->SCFG, DMA_SCFGx_CTARG, CurrentMemory);
}

/**
  * @brief Set Current target (only in double buffer mode) to Memory 1 or Memory 0.
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_DMA_CURRENTTARGETMEM0
  *         @arg @ref DDL_DMA_CURRENTTARGETMEM1
  */
__STATIC_INLINE uint32_t DDL_DMA_GetCurrentTargetMem(DMA_TypeDef *DMAx, uint32_t Channel)
{
  return (READ_BIT(((DMA_Channel_TypeDef*)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->SCFG, DMA_SCFGx_CTARG));
}

/**
  * @brief Enable the double buffer mode.
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_EnableDoubleBufferMode(DMA_TypeDef *DMAx, uint32_t Channel)
{
  SET_BIT(((DMA_Channel_TypeDef *)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->SCFG, DMA_SCFGx_DBM);
}

/**
  * @brief Disable the double buffer mode.
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_DisableDoubleBufferMode(DMA_TypeDef *DMAx, uint32_t Channel)
{
  CLEAR_BIT(((DMA_Channel_TypeDef *)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->SCFG, DMA_SCFGx_DBM);
}

/**
  * @brief Get FIFO status.
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_DMA_FIFOSTATUS_0_25
  *         @arg @ref DDL_DMA_FIFOSTATUS_25_50
  *         @arg @ref DDL_DMA_FIFOSTATUS_50_75
  *         @arg @ref DDL_DMA_FIFOSTATUS_75_100
  *         @arg @ref DDL_DMA_FIFOSTATUS_EMPTY
  *         @arg @ref DDL_DMA_FIFOSTATUS_FULL
  */
__STATIC_INLINE uint32_t DDL_DMA_GetFIFOStatus(DMA_TypeDef *DMAx, uint32_t Channel)
{
  return (READ_BIT(((DMA_Channel_TypeDef*)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->FIFOCR, DMA_FIFOCRx_FSTS));
}

/**
  * @brief Disable Fifo mode.
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_DisableFifoMode(DMA_TypeDef *DMAx, uint32_t Channel)
{
  CLEAR_BIT(((DMA_Channel_TypeDef *)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->FIFOCR, DMA_FIFOCRx_DMDEN);
}

/**
  * @brief Enable Fifo mode.
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_EnableFifoMode(DMA_TypeDef *DMAx, uint32_t Channel)
{
  SET_BIT(((DMA_Channel_TypeDef *)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->FIFOCR, DMA_FIFOCRx_DMDEN);
}

/**
  * @brief Select FIFO threshold.
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @param  Threshold This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_FIFOTHRESHOLD_1_4
  *         @arg @ref DDL_DMA_FIFOTHRESHOLD_1_2
  *         @arg @ref DDL_DMA_FIFOTHRESHOLD_3_4
  *         @arg @ref DDL_DMA_FIFOTHRESHOLD_FULL
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_SetFIFOThreshold(DMA_TypeDef *DMAx, uint32_t Channel, uint32_t Threshold)
{
  MODIFY_REG(((DMA_Channel_TypeDef*)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->FIFOCR, DMA_FIFOCRx_FTHSEL, Threshold);
}

/**
  * @brief Get FIFO threshold.
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_DMA_FIFOTHRESHOLD_1_4
  *         @arg @ref DDL_DMA_FIFOTHRESHOLD_1_2
  *         @arg @ref DDL_DMA_FIFOTHRESHOLD_3_4
  *         @arg @ref DDL_DMA_FIFOTHRESHOLD_FULL
  */
__STATIC_INLINE uint32_t DDL_DMA_GetFIFOThreshold(DMA_TypeDef *DMAx, uint32_t Channel)
{
  return (READ_BIT(((DMA_Channel_TypeDef*)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->FIFOCR, DMA_FIFOCRx_FTHSEL));
}

/**
  * @brief Configure the FIFO .
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @param  FifoMode This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_FIFOMODE_ENABLE
  *         @arg @ref DDL_DMA_FIFOMODE_DISABLE
  * @param  FifoThreshold This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_FIFOTHRESHOLD_1_4
  *         @arg @ref DDL_DMA_FIFOTHRESHOLD_1_2
  *         @arg @ref DDL_DMA_FIFOTHRESHOLD_3_4
  *         @arg @ref DDL_DMA_FIFOTHRESHOLD_FULL
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_ConfigFifo(DMA_TypeDef *DMAx, uint32_t Channel, uint32_t FifoMode, uint32_t FifoThreshold)
{
  MODIFY_REG(((DMA_Channel_TypeDef*)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->FIFOCR, DMA_FIFOCRx_FTHSEL|DMA_FIFOCRx_DMDEN, FifoMode|FifoThreshold);
}

/**
  * @brief Configure the Source and Destination addresses.
  * @note   This API must not be called when the DMA channel is enabled.
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @param  SrcAddress Between 0 to 0xFFFFFFFF
  * @param  DstAddress Between 0 to 0xFFFFFFFF
  * @param  Direction This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_DIRECTION_PERIPH_TO_MEMORY
  *         @arg @ref DDL_DMA_DIRECTION_MEMORY_TO_PERIPH
  *         @arg @ref DDL_DMA_DIRECTION_MEMORY_TO_MEMORY
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_ConfigAddresses(DMA_TypeDef* DMAx, uint32_t Channel, uint32_t SrcAddress, uint32_t DstAddress, uint32_t Direction)
{
  /* Direction Memory to Periph */
  if (Direction == DDL_DMA_DIRECTION_MEMORY_TO_PERIPH)
  {
    WRITE_REG(((DMA_Channel_TypeDef*)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->M0ADDR, SrcAddress);
    WRITE_REG(((DMA_Channel_TypeDef*)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->PADDR, DstAddress);
  }
  /* Direction Periph to Memory and Memory to Memory */
  else
  {
    WRITE_REG(((DMA_Channel_TypeDef*)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->PADDR, SrcAddress);
    WRITE_REG(((DMA_Channel_TypeDef*)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->M0ADDR, DstAddress);
  }
}

/**
  * @brief  Set the Memory address.
  * @note   Interface used for direction DDL_DMA_DIRECTION_PERIPH_TO_MEMORY or DDL_DMA_DIRECTION_MEMORY_TO_PERIPH only.
  * @note   This API must not be called when the DMA channel is enabled.
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @param  MemoryAddress Between 0 to 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_SetMemoryAddress(DMA_TypeDef* DMAx, uint32_t Channel, uint32_t MemoryAddress)
{
  WRITE_REG(((DMA_Channel_TypeDef*)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->M0ADDR, MemoryAddress);
}

/**
  * @brief  Get the Memory address.
  * @note   Interface used for direction DDL_DMA_DIRECTION_PERIPH_TO_MEMORY or DDL_DMA_DIRECTION_MEMORY_TO_PERIPH only.
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @retval Between 0 to 0xFFFFFFFF
  */
__STATIC_INLINE uint32_t DDL_DMA_GetMemoryAddress(DMA_TypeDef* DMAx, uint32_t Channel)
{
  return (READ_REG(((DMA_Channel_TypeDef*)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->M0ADDR));
}

/**
  * @brief  Set the Peripheral address.
  * @note   Interface used for direction DDL_DMA_DIRECTION_PERIPH_TO_MEMORY or DDL_DMA_DIRECTION_MEMORY_TO_PERIPH only.
  * @note   This API must not be called when the DMA channel is enabled.
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @param  PeriphAddress Between 0 to 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_SetPeriphAddress(DMA_TypeDef* DMAx, uint32_t Channel, uint32_t PeriphAddress)
{
  WRITE_REG(((DMA_Channel_TypeDef*)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->PADDR, PeriphAddress);
}

/**
  * @brief  Get the Peripheral address.
  * @note   Interface used for direction DDL_DMA_DIRECTION_PERIPH_TO_MEMORY or DDL_DMA_DIRECTION_MEMORY_TO_PERIPH only.
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @retval Between 0 to 0xFFFFFFFF
  */
__STATIC_INLINE uint32_t DDL_DMA_GetPeriphAddress(DMA_TypeDef* DMAx, uint32_t Channel)
{
  return (READ_REG(((DMA_Channel_TypeDef *)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->PADDR));
}

/**
  * @brief  Set the Memory to Memory Source address.
  * @note   Interface used for direction DDL_DMA_DIRECTION_MEMORY_TO_MEMORY only.
  * @note   This API must not be called when the DMA channel is enabled.
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @param  MemoryAddress Between 0 to 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_SetM2MSrcAddress(DMA_TypeDef* DMAx, uint32_t Channel, uint32_t MemoryAddress)
{
  WRITE_REG(((DMA_Channel_TypeDef*)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->PADDR, MemoryAddress);
}

/**
  * @brief  Get the Memory to Memory Source address.
  * @note   Interface used for direction DDL_DMA_DIRECTION_MEMORY_TO_MEMORY only.
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @retval Between 0 to 0xFFFFFFFF
  */
__STATIC_INLINE uint32_t DDL_DMA_GetM2MSrcAddress(DMA_TypeDef* DMAx, uint32_t Channel)
  {
   return (READ_REG(((DMA_Channel_TypeDef *)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->PADDR));
  }

/**
  * @brief  Set the Memory to Memory Destination address.
  * @note   Interface used for direction DDL_DMA_DIRECTION_MEMORY_TO_MEMORY only.
  * @note   This API must not be called when the DMA channel is enabled.
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @param  MemoryAddress Between 0 to 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_SetM2MDstAddress(DMA_TypeDef* DMAx, uint32_t Channel, uint32_t MemoryAddress)
  {
    WRITE_REG(((DMA_Channel_TypeDef*)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->M0ADDR, MemoryAddress);
  }

/**
  * @brief  Get the Memory to Memory Destination address.
  * @note   Interface used for direction DDL_DMA_DIRECTION_MEMORY_TO_MEMORY only.
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @retval Between 0 to 0xFFFFFFFF
  */
__STATIC_INLINE uint32_t DDL_DMA_GetM2MDstAddress(DMA_TypeDef* DMAx, uint32_t Channel)
{
 return (READ_REG(((DMA_Channel_TypeDef*)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->M0ADDR));
}

/**
  * @brief Set Memory 1 address (used in case of Double buffer mode).
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @param  Address Between 0 to 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_SetMemory1Address(DMA_TypeDef *DMAx, uint32_t Channel, uint32_t Address)
{
  MODIFY_REG(((DMA_Channel_TypeDef*)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->M1ADDR, DMA_M1ADDRx_M1ADDR, Address);
}

/**
  * @brief Get Memory 1 address (used in case of Double buffer mode).
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @retval Between 0 to 0xFFFFFFFF
  */
__STATIC_INLINE uint32_t DDL_DMA_GetMemory1Address(DMA_TypeDef *DMAx, uint32_t Channel)
{
  return (((DMA_Channel_TypeDef*)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->M1ADDR);
}

/**
  * @}
  */

/** @defgroup DMA_DDL_EF_FLAG_Management FLAG_Management
  * @{
  */

/**
  * @brief Get Channel 0 half transfer flag.
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DMA_IsActiveFlag_HT0(DMA_TypeDef *DMAx)
{
  return (READ_BIT(DMAx->ISR ,DMA_ISR_HTXIFLG0)==(DMA_ISR_HTXIFLG0));
}

/**
  * @brief Get Channel 1 half transfer flag.
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DMA_IsActiveFlag_HT1(DMA_TypeDef *DMAx)
{
  return (READ_BIT(DMAx->ISR ,DMA_ISR_HTXIFLG1)==(DMA_ISR_HTXIFLG1));
}

/**
  * @brief Get Channel 0 transfer complete flag.
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DMA_IsActiveFlag_TC0(DMA_TypeDef *DMAx)
{
  return (READ_BIT(DMAx->ISR ,DMA_ISR_TXCIFLG0)==(DMA_ISR_TXCIFLG0));
}

/**
  * @brief Get Channel 1 transfer complete flag.
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DMA_IsActiveFlag_TC1(DMA_TypeDef *DMAx)
{
  return (READ_BIT(DMAx->ISR ,DMA_ISR_TXCIFLG1)==(DMA_ISR_TXCIFLG1));
}

/**
  * @brief Get Channel 0 transfer error flag.
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DMA_IsActiveFlag_TE0(DMA_TypeDef *DMAx)
{
  return (READ_BIT(DMAx->ISR ,DMA_ISR_TXEIFLG0)==(DMA_ISR_TXEIFLG0));
}

/**
  * @brief Get Channel 1 transfer error flag.
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DMA_IsActiveFlag_TE1(DMA_TypeDef *DMAx)
{
  return (READ_BIT(DMAx->ISR ,DMA_ISR_TXEIFLG1)==(DMA_ISR_TXEIFLG1));
}


/**
  * @brief Get Channel 0 direct mode error flag.
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DMA_IsActiveFlag_DME0(DMA_TypeDef *DMAx)
{
  return (READ_BIT(DMAx->ISR ,DMA_ISR_DMEIFLG0)==(DMA_ISR_DMEIFLG0));
}

/**
  * @brief Get Channel 1 direct mode error flag.
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DMA_IsActiveFlag_DME1(DMA_TypeDef *DMAx)
{
  return (READ_BIT(DMAx->ISR ,DMA_ISR_DMEIFLG1)==(DMA_ISR_DMEIFLG1));
}


/**
  * @brief Get Channel 0 FIFO error flag.
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DMA_IsActiveFlag_FE0(DMA_TypeDef *DMAx)
{
  return (READ_BIT(DMAx->ISR ,DMA_ISR_FEIFLG0)==(DMA_ISR_FEIFLG0));
}

/**
  * @brief Get Channel 1 FIFO error flag.
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DMA_IsActiveFlag_FE1(DMA_TypeDef *DMAx)
{
  return (READ_BIT(DMAx->ISR ,DMA_ISR_FEIFLG1)==(DMA_ISR_FEIFLG1));
}

/**
  * @brief Clear Channel 0 half transfer flag.
  * @param  DMAx DMAx Instance
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_ClearFlag_HT0(DMA_TypeDef *DMAx)
{
  WRITE_REG(DMAx->IFCLR , DMA_IFCLR_CHTXIFLG0);
}

/**
  * @brief Clear Channel 1 half transfer flag.
  * @param  DMAx DMAx Instance
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_ClearFlag_HT1(DMA_TypeDef *DMAx)
{
  WRITE_REG(DMAx->IFCLR , DMA_IFCLR_CHTXIFLG1);
}


/**
  * @brief Clear Channel 0 transfer complete flag.
  * @param  DMAx DMAx Instance
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_ClearFlag_TC0(DMA_TypeDef *DMAx)
{
  WRITE_REG(DMAx->IFCLR , DMA_IFCLR_CTXCIFLG0);
}

/**
  * @brief Clear Channel 1 transfer complete flag.
  * @param  DMAx DMAx Instance
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_ClearFlag_TC1(DMA_TypeDef *DMAx)
{
  WRITE_REG(DMAx->IFCLR , DMA_IFCLR_CTXCIFLG1);
}


/**
  * @brief Clear Channel 0 transfer error flag.
  * @param  DMAx DMAx Instance
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_ClearFlag_TE0(DMA_TypeDef *DMAx)
{
  WRITE_REG(DMAx->IFCLR , DMA_IFCLR_CTXEIFLG0);
}

/**
  * @brief Clear Channel 1 transfer error flag.
  * @param  DMAx DMAx Instance
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_ClearFlag_TE1(DMA_TypeDef *DMAx)
{
  WRITE_REG(DMAx->IFCLR , DMA_IFCLR_CTXEIFLG1);
}


/**
  * @brief Clear Channel 0 direct mode error flag.
  * @param  DMAx DMAx Instance
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_ClearFlag_DME0(DMA_TypeDef *DMAx)
{
  WRITE_REG(DMAx->IFCLR , DMA_IFCLR_CDMEIFLG0);
}

/**
  * @brief Clear Channel 1 direct mode error flag.
  * @param  DMAx DMAx Instance
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_ClearFlag_DME1(DMA_TypeDef *DMAx)
{
  WRITE_REG(DMAx->IFCLR , DMA_IFCLR_CDMEIFLG1);
}


/**
  * @brief Clear Channel 0 FIFO error flag.
  * @param  DMAx DMAx Instance
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_ClearFlag_FE0(DMA_TypeDef *DMAx)
{
  WRITE_REG(DMAx->IFCLR , DMA_IFCLR_CFEIFLG0);
}

/**
  * @brief Clear Channel 1 FIFO error flag.
  * @param  DMAx DMAx Instance
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_ClearFlag_FE1(DMA_TypeDef *DMAx)
{
  WRITE_REG(DMAx->IFCLR , DMA_IFCLR_CFEIFLG1);
}

/**
  * @}
  */

/** @defgroup DMA_DDL_EF_IT_Management IT_Management
  * @{
  */

/**
  * @brief Enable Half transfer interrupt.
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_EnableIT_HT(DMA_TypeDef *DMAx, uint32_t Channel)
{
  SET_BIT(((DMA_Channel_TypeDef *)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->SCFG, DMA_SCFGx_HTXIEN);
}

/**
  * @brief Enable Transfer error interrupt.
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_EnableIT_TE(DMA_TypeDef *DMAx, uint32_t Channel)
{
  SET_BIT(((DMA_Channel_TypeDef *)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->SCFG, DMA_SCFGx_TXEIEN);
}

/**
  * @brief Enable Transfer complete interrupt.
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_EnableIT_TC(DMA_TypeDef *DMAx, uint32_t Channel)
{
  SET_BIT(((DMA_Channel_TypeDef *)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->SCFG, DMA_SCFGx_TXCIEN);
}

/**
  * @brief Enable Direct mode error interrupt.
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_EnableIT_DME(DMA_TypeDef *DMAx, uint32_t Channel)
{
  SET_BIT(((DMA_Channel_TypeDef *)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->SCFG, DMA_SCFGx_DMEIEN);
}

/**
  * @brief Enable FIFO error interrupt.
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_EnableIT_FE(DMA_TypeDef *DMAx, uint32_t Channel)
{
  SET_BIT(((DMA_Channel_TypeDef *)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->FIFOCR, DMA_FIFOCRx_FEIEN);
}

/**
  * @brief Disable Half transfer interrupt.
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_DisableIT_HT(DMA_TypeDef *DMAx, uint32_t Channel)
{
  CLEAR_BIT(((DMA_Channel_TypeDef *)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->SCFG, DMA_SCFGx_HTXIEN);
}

/**
  * @brief Disable Transfer error interrupt.
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_DisableIT_TE(DMA_TypeDef *DMAx, uint32_t Channel)
{
  CLEAR_BIT(((DMA_Channel_TypeDef *)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->SCFG, DMA_SCFGx_TXEIEN);
}

/**
  * @brief Disable Transfer complete interrupt.
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_DisableIT_TC(DMA_TypeDef *DMAx, uint32_t Channel)
{
  CLEAR_BIT(((DMA_Channel_TypeDef *)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->SCFG, DMA_SCFGx_TXCIEN);
}

/**
  * @brief Disable Direct mode error interrupt.
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_DisableIT_DME(DMA_TypeDef *DMAx, uint32_t Channel)
{
  CLEAR_BIT(((DMA_Channel_TypeDef *)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->SCFG, DMA_SCFGx_DMEIEN);
}

/**
  * @brief Disable FIFO error interrupt.
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_DisableIT_FE(DMA_TypeDef *DMAx, uint32_t Channel)
{
  CLEAR_BIT(((DMA_Channel_TypeDef *)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->FIFOCR, DMA_FIFOCRx_FEIEN);
}

/**
  * @brief Check if Half transfer interrupt is enabled.
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DMA_IsEnabledIT_HT(DMA_TypeDef *DMAx, uint32_t Channel)
{
  return (READ_BIT(((DMA_Channel_TypeDef*)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->SCFG, DMA_SCFGx_HTXIEN) == DMA_SCFGx_HTXIEN);
}

/**
  * @brief Check if Transfer error nterrup is enabled.
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DMA_IsEnabledIT_TE(DMA_TypeDef *DMAx, uint32_t Channel)
{
  return (READ_BIT(((DMA_Channel_TypeDef*)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->SCFG, DMA_SCFGx_TXEIEN) == DMA_SCFGx_TXEIEN);
}

/**
  * @brief Check if Transfer complete interrupt is enabled.
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DMA_IsEnabledIT_TC(DMA_TypeDef *DMAx, uint32_t Channel)
{
  return (READ_BIT(((DMA_Channel_TypeDef*)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->SCFG, DMA_SCFGx_TXCIEN) == DMA_SCFGx_TXCIEN);
}

/**
  * @brief Check if Direct mode error interrupt is enabled.
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DMA_IsEnabledIT_DME(DMA_TypeDef *DMAx, uint32_t Channel)
{
  return (READ_BIT(((DMA_Channel_TypeDef*)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->SCFG, DMA_SCFGx_DMEIEN) == DMA_SCFGx_DMEIEN);
}

/**
  * @brief Check if FIFO error interrupt is enabled.
  * @param  DMAx DMAx Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DMA_IsEnabledIT_FE(DMA_TypeDef *DMAx, uint32_t Channel)
{
  return (READ_BIT(((DMA_Channel_TypeDef*)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel])))->FIFOCR, DMA_FIFOCRx_FEIEN) == DMA_FIFOCRx_FEIEN);
}

/**
  * @}
  */

#if defined(USE_FULL_DDL_DRIVER)
/** @defgroup DMA_DDL_EF_Init Initialization and de-initialization functions
  * @{
  */

uint32_t DDL_DMA_Init(DMA_TypeDef *DMAx, uint32_t Channel, DDL_DMA_InitTypeDef *DMA_InitStruct);
uint32_t DDL_DMA_DeInit(DMA_TypeDef *DMAx, uint32_t Channel);
void DDL_DMA_StructInit(DDL_DMA_InitTypeDef *DMA_InitStruct);

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

#endif /* DMA */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* G32F031_DDL_DMA_H */

