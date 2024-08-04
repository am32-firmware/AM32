/**
  ******************************************************************************
  * @file    stm32l4p5g_discovery_mmc.c
  * @author  MCD Application Team
  * @brief   This file includes the EMMC driver mounted on STM32L4P5G-DISCOVERY
  *          board.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* File Info : -----------------------------------------------------------------
                                   User NOTES
1. How To use this driver:
--------------------------
   - This driver is used to drive the EMMC mounted on STM32L4P5G-DISCOVERY
     board.
   - This driver does not need a specific component driver for the EMMC device
     to be included with.

2. Driver description:
---------------------
  + Initialization steps:
     o Initialize the external EMMC memory using the BSP_MMC_Init() function. This
       function includes the MSP layer hardware resources initialization and the
       SDIO interface configuration to interface with the external EMMC. It
       also includes the EMMC initialization sequence.
     o The function BSP_MMC_GetCardInfo() is used to get the MMC information
       which is stored in the structure "HAL_MMC_CardInfoTypedef".

  + Micro MMC card operations
     o The micro MMC card can be accessed with read/write block(s) operations once
       it is ready for access. The access can be performed whether using the polling
       mode by calling the functions BSP_MMC_ReadBlocks()/BSP_MMC_WriteBlocks(), or by DMA
       transfer using the functions BSP_MMC_ReadBlocks_DMA()/BSP_MMC_WriteBlocks_DMA()
     o The DMA transfer complete is used with interrupt mode. Once the MMC transfer
       is complete, the MMC interrupt is handled using the function BSP_MMC_IRQHandler(),
       the DMA Tx/Rx transfer complete are handled using the functions
       MMC_DMA_Tx_IRQHandler()/MMC_DMA_Rx_IRQHandler() that should be defined by user.
       The corresponding user callbacks are implemented by the user at application level.
     o The MMC erase block(s) is performed using the function BSP_MMC_Erase() with specifying
       the number of blocks to erase.
     o The MMC runtime status is returned when calling the function BSP_MMC_GetStatus().

------------------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "stm32l4p5g_discovery_mmc.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L4P5G_DISCOVERY
  * @{
  */

/** @defgroup STM32L4P5G_DISCOVERY_MMC STM32L4P5G_DISCOVERY_MMC
  * @{
  */

/** @defgroup STM32L4P5G_DISCOVERY_MMC_Exported_Variables Exported Variables
  * @{
  */
MMC_HandleTypeDef hmmc_card;
BSP_MMC_CardCID   mmc_cid;
BSP_MMC_CardCSD   mmc_csd;
/**
  * @}
  */



/** @defgroup STM32L4P5G_DISCOVERY_MMC_Exported_Functions  Exported Functions
  * @{
  */

/**
  * @brief  Initializes the MMC card device.
  * @retval MMC status
  */
uint8_t BSP_MMC_Init(void)
{
  uint8_t mmc_state = MMC_OK;

  /* uMMC device interface configuration */
  hmmc_card.Instance = SDMMC1;

  /* if CLKDIV = 0 then SDMMC Clock frequency = SDMMC Kernel Clock
     else SDMMC Clock frequency = SDMMC Kernel Clock / [2 * CLKDIV].
  */
  hmmc_card.Init.ClockDiv            = SDMMC_TRANSFER_CLK_DIV;
  hmmc_card.Init.ClockPowerSave      = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hmmc_card.Init.ClockEdge           = SDMMC_CLOCK_EDGE_RISING;
  hmmc_card.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_ENABLE;
  hmmc_card.Init.BusWide             = SDMMC_BUS_WIDE_1B;

  /* Msp MMC initialization */
  BSP_MMC_MspInit(&hmmc_card, NULL);

  /* HAL MMC initialization */
  if(HAL_MMC_Init(&hmmc_card) != HAL_OK)
  {
    mmc_state = MMC_ERROR;
  }

  return  mmc_state;
}



/**
  * @brief  DeInitializes the MMC card device.
  * @retval MMC status
  */
uint8_t BSP_MMC_DeInit(void)
{
  uint8_t mmc_state = MMC_OK;

  hmmc_card.Instance = SDMMC1;

  /* HAL MMC deinitialization */
  if(HAL_MMC_DeInit(&hmmc_card) != HAL_OK)
  {
    mmc_state = MMC_ERROR;
  }

  /* Msp MMC deinitialization */
  hmmc_card.Instance = SDMMC1;
  BSP_MMC_MspDeInit(&hmmc_card, NULL);

  return  mmc_state;
}


/**
  * @brief  Enables wide bus operation.
  * @param  WideMode: Specifies the MMC card wide bus mode
  *          This parameter can be one of the following values:
  *            @arg SDMMC_BUS_WIDE_8B: 8-bit data transfer
  *            @arg SDMMC_BUS_WIDE_4B: 4-bit data transfer
  *            @arg SDMMC_BUS_WIDE_1B: 1-bit data transfer
  * @retval MMC status
  */
uint8_t BSP_MMC_ConfigWideBusOperation(uint32_t WideMode)
{
  if( HAL_MMC_ConfigWideBusOperation(&hmmc_card, WideMode) == HAL_OK)
  {
    return MMC_OK;
  }
  else
  {
    return MMC_ERROR;
  }
}

/**
  * @brief  Configure the speed bus mode.
  * @param  SpeedMode: Specifies the MMC card speed bus mode
  *          This parameter can be one of the following values:
  *            @arg SDMMC_SPEED_MODE_AUTO: Max speed mode supported by the card
  *            @arg SDMMC_SPEED_MODE_DEFAULT: Default Speed (MMC @ 26MHz)
  *            @arg SDMMC_SPEED_MODE_HIGH: High Speed (MMC @ 52 MHz)
  *            @arg SDMMC_SPEED_MODE_DDR: High Speed DDR (MMC DDR @ 52 MHz)
  * @retval MMC status
  */
uint8_t BSP_MMC_ConfigSpeedBusOperation(uint32_t SpeedMode)
{
  if( HAL_MMC_ConfigSpeedBusOperation(&hmmc_card, SpeedMode) == HAL_OK)
  {
    return MMC_OK;
  }
  else
  {
    return MMC_ERROR;
  }
}



/**
  * @brief  Reads block(s) from a specified address in an MMC card, in polling mode.
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  ReadAddr: Address from where data is to be read
  * @param  NumOfBlocks: Number of MMC blocks to read
  * @param  Timeout: Timeout for read operation
  * @retval MMC status
  */
uint8_t BSP_MMC_ReadBlocks(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks, uint32_t Timeout)
{

  if( HAL_MMC_ReadBlocks(&hmmc_card, (uint8_t *)pData, ReadAddr, NumOfBlocks, Timeout) == HAL_OK)
  {
    return MMC_OK;
  }
  else
  {
    return MMC_ERROR;
  }

}

/**
  * @brief  Writes block(s) to a specified address in an MMC card, in polling mode.
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  WriteAddr: Address where data is to be written
  * @param  NumOfBlocks: Number of MMC blocks to write
  * @param  Timeout: Timeout for write operation
  * @retval MMC status
  */
uint8_t BSP_MMC_WriteBlocks(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks, uint32_t Timeout)
{

  if( HAL_MMC_WriteBlocks(&hmmc_card, (uint8_t *)pData, WriteAddr, NumOfBlocks, Timeout) == HAL_OK)
  {
    return MMC_OK;
  }
  else
  {
    return MMC_ERROR;
  }
}

/**
  * @brief  Reads block(s) from a specified address in an MMC card, in DMA mode.
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  ReadAddr: Address from where data is to be read
  * @param  NumOfBlocks: Number of MMC blocks to read
  * @retval MMC status
  */
uint8_t BSP_MMC_ReadBlocks_DMA(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks)
{

  if( HAL_MMC_ReadBlocks_DMA(&hmmc_card, (uint8_t *)pData, ReadAddr, NumOfBlocks) == HAL_OK)
  {
    return MMC_OK;
  }
  else
  {
    return MMC_ERROR;
  }
}

/**
  * @brief  Writes block(s) to a specified address in an MMC card, in DMA mode.
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  WriteAddr: Address where data is to be written
  * @param  NumOfBlocks: Number of MMC blocks to write
  * @retval MMC status
  */
uint8_t BSP_MMC_WriteBlocks_DMA(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks)
{

  if( HAL_MMC_WriteBlocks_DMA(&hmmc_card, (uint8_t *)pData, WriteAddr, NumOfBlocks) == HAL_OK)
  {
    return MMC_OK;
  }
  else
  {
    return MMC_ERROR;
  }
}


/**
  * @brief  Reads block(s) from a specified address in an MMC card, in IT mode.
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  ReadAddr: Address from where data is to be read
  * @param  NumOfBlocks: Number of MMC blocks to read
  * @retval MMC status
  */
uint8_t BSP_MMC_ReadBlocks_IT(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks)
{

  if( HAL_MMC_ReadBlocks_IT(&hmmc_card, (uint8_t *)pData, ReadAddr, NumOfBlocks) == HAL_OK)
  {
    return MMC_OK;
  }
  else
  {
    return MMC_ERROR;
  }
}

/**
  * @brief  Writes block(s) to a specified address in an MMC card, in IT mode.
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  WriteAddr: Address where data is to be written
  * @param  NumOfBlocks: Number of MMC blocks to write
  * @retval MMC status
  */
uint8_t BSP_MMC_WriteBlocks_IT(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks)
{

  if( HAL_MMC_WriteBlocks_IT(&hmmc_card, (uint8_t *)pData, WriteAddr, NumOfBlocks) == HAL_OK)
  {
    return MMC_OK;
  }
  else
  {
    return MMC_ERROR;
  }
}


/**
  * @brief  Reads block(s) from a specified address in a card. The received Data will be stored in Data1 and Data2.
  * @param  pData1: Pointer to the Data1 that will contain/receive the transferred data
  * @param  pData2: Pointer to the Data2 that will contain/receive the transferred data
  * @param  ReadAddr: Address from where data is to be read
  * @param  NumOfBlocks: Size of Data1 in Blocks. Data1 and Data2 must have the same size.
  * @retval MMC status
  */
uint8_t BSP_MMC_ReadBlocks_DMAMultiBuffer(uint32_t *pData1, uint32_t *pData2, uint32_t ReadAddr, uint32_t NumOfBlocks)
{

  if( HAL_MMCEx_ConfigDMAMultiBuffer(&hmmc_card, pData1, pData2, NumOfBlocks) != HAL_OK)
  {
    return MMC_ERROR;
  }
  else
  {
    if( HAL_MMCEx_ReadBlocksDMAMultiBuffer(&hmmc_card, ReadAddr, 2 * NumOfBlocks) == HAL_OK)
    {
      return MMC_OK;
    }
    else
    {
      return MMC_ERROR;
    }
  }
}

/**
  * @brief  Writes block(s) from a specified address in a card. The written Data are stored in Data1 and Data2.
  * @param  pData1: Pointer to the Data1 that contains the written data
  * @param  pData2: Pointer to the Data2 that contains written data
  * @param  WriteAddr: Address from where data is to be written
  * @param  NumOfBlocks: Size of Data1 in Blocks. Data1 and Data2 must have the same size.
  * @retval MMC status
  */
uint8_t BSP_MMC_WriteBlocks_DMAMultiBuffer(uint32_t *pData1, uint32_t *pData2, uint32_t WriteAddr, uint32_t NumOfBlocks)
{

  if( HAL_MMCEx_ConfigDMAMultiBuffer(&hmmc_card, pData1, pData2, NumOfBlocks) != HAL_OK)
  {
    return MMC_ERROR;
  }
  else
  {
    if( HAL_MMCEx_WriteBlocksDMAMultiBuffer(&hmmc_card, WriteAddr, 2 * NumOfBlocks) == HAL_OK)
    {
      return MMC_OK;
    }
    else
    {
      return MMC_ERROR;
    }
  }
}





/**
  * @brief  Erases the specified memory area of the given MMC card.
  * @param  StartAddr: Start byte address
  * @param  EndAddr: End byte address
  * @retval MMC status
  */
uint8_t BSP_MMC_Erase(uint32_t StartAddr, uint32_t EndAddr)
{

  if( HAL_MMC_Erase(&hmmc_card, StartAddr, EndAddr) == HAL_OK)
  {
    return MMC_OK;
  }
  else
  {
    return MMC_ERROR;
  }
}

/**
  * @brief  Initializes the MMC MSP.
  * @param  hmmc: MMC handle
  * @param  Params User parameters
  * @retval None
  */
__weak void BSP_MMC_MspInit(MMC_HandleTypeDef *hmmc, void *Params)
{
  /* __weak function can be modified by the application */

  GPIO_InitTypeDef gpio_init_structure;

  /* Enable SDIO clock */
  __HAL_RCC_SDMMC1_CLK_ENABLE();

  /* Enable GPIOs clock */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* Enable DMA2 clocks */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* Common GPIO configuration */
  gpio_init_structure.Mode      = GPIO_MODE_AF_PP;
  gpio_init_structure.Pull      = GPIO_PULLUP;
  gpio_init_structure.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio_init_structure.Alternate = GPIO_AF12_SDMMC1;

  /* SDMMC GPIO CLKIN PB8, D0 PC8, D1 PC9, D2 PC10, D3 PC11, CK PC12, CMD PD2 */

  /* GPIOC configuration, D0 PC8, D1 PC9, D2 PC10, D3 PC11, D6 PC6, D7 PC7, CK PC12*/
  gpio_init_structure.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
  HAL_GPIO_Init(GPIOC, &gpio_init_structure);

  /* GPIOD configuration, CMD PD2 */
  gpio_init_structure.Pin = GPIO_PIN_2;
  HAL_GPIO_Init(GPIOD, &gpio_init_structure);

  /* GPIOB configuration, D4 PB8, D5 PB9 */
  gpio_init_structure.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  HAL_GPIO_Init(GPIOB, &gpio_init_structure);


  /* NVIC configuration for SDMMC interrupts */
  HAL_NVIC_SetPriority(SDMMC1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(SDMMC1_IRQn);

}

/**
  * @brief  DeInitializes the MMC MSP.
  * @param  hmmc: MMC handle
  * @param  Params User parameters
  * @retval None
  */
__weak void BSP_MMC_MspDeInit(MMC_HandleTypeDef *hmmc, void *Params)
{
    /* Disable NVIC for SDIO interrupts */
    HAL_NVIC_DisableIRQ(SDMMC1_IRQn);

    /* DeInit GPIO pins can be done in the application
       (by surcharging this __weak function) */

    /* Force the SDMMC Periheral Clock Reset */
    __HAL_RCC_SDMMC1_FORCE_RESET();

    /* Release the SDMMC Periheral Clock Reset */
    __HAL_RCC_SDMMC1_RELEASE_RESET();

    /* Disable SDMMC1 clock */
    __HAL_RCC_SDMMC1_CLK_DISABLE();

    /* GPIO pins clock and DMA clocks can be shut down in the application
       by surcharging this __weak function */
}

/**
  * @brief  Handles MMC card interrupt request.
  * @retval None
  */
void BSP_MMC_IRQHandler(void)
{
  HAL_MMC_IRQHandler(&hmmc_card);
}



/**
  * @brief  Gets the current MMC card data status.
  * @retval Data transfer state.
  *          This value can be one of the following values:
  *            @arg  MMC_TRANSFER_OK: No data transfer is acting
  *            @arg  MMC_TRANSFER_BUSY: Data transfer is acting
  */
uint8_t BSP_MMC_GetCardState(void)
{
  return((HAL_MMC_GetCardState(&hmmc_card) == HAL_MMC_CARD_TRANSFER ) ? MMC_TRANSFER_OK : MMC_TRANSFER_BUSY);
}


/**
  * @brief  Get MMC information about specific MMC card.
  * @param  CardInfo: Pointer to HAL_MMC_CardInfoTypedef structure
  * @retval None
  */
uint8_t BSP_MMC_GetCardInfo(BSP_MMC_CardInfo *CardInfo)
{
  if( HAL_MMC_GetCardInfo(&hmmc_card, CardInfo) == HAL_OK)
  {
    return MMC_OK;
  }
  else
  {
    return MMC_ERROR;
  }
}

/**
  * @brief  Get MMC information of the card which are stored on the CID register.
  * @param  CardCID: Pointer to a BSP_MMC_CardCID structure that
  *         contains all CID register parameters
  * @retval None
  */
uint8_t BSP_MMC_GetCardCID(BSP_MMC_CardCID *CardCID)
{
  if( HAL_MMC_GetCardCID(&hmmc_card, CardCID) == HAL_OK)
  {
    return MMC_OK;
  }
  else
  {
    return MMC_ERROR;
  }
}

/**
  * @brief  Get MMC information of the card which are stored on the CSD register.
  * @param  CardCSD: Pointer to a BSP_MMC_CardCSD structure that
  *         contains all CSD register parameters
  * @retval None
  */
uint8_t BSP_MMC_GetCardCSD(BSP_MMC_CardCSD *CardCSD)
{
  if( HAL_MMC_GetCardCSD(&hmmc_card, CardCSD) == HAL_OK)
  {
    return MMC_OK;
  }
  else
  {
    return MMC_ERROR;
  }
}

/**
  * @brief MMC Abort callbacks
  * @param hmmc: MMC handle
  * @retval None
  */
void HAL_MMC_AbortCallback(MMC_HandleTypeDef *hmmc)
{
  BSP_MMC_AbortCallback();
}


/**
  * @brief Tx Transfer completed callbacks
  * @param hmmc: MMC handle
  * @retval None
  */
void HAL_MMC_TxCpltCallback(MMC_HandleTypeDef *hmmc)
{
  BSP_MMC_WriteCpltCallback();
}

/**
  * @brief Rx Transfer completed callbacks
  * @param hmmc: MMC handle
  * @retval None
  */
void HAL_MMC_RxCpltCallback(MMC_HandleTypeDef *hmmc)
{
  BSP_MMC_ReadCpltCallback();
}

/**
  * @brief BSP MMC Abort callbacks
  * @retval None
  */
__weak void BSP_MMC_AbortCallback(void)
{

}

/**
  * @brief BSP Tx Transfer completed callbacks
  * @retval None
  */
__weak void BSP_MMC_WriteCpltCallback(void)
{

}

/**
  * @brief BSP Rx Transfer completed callbacks
  * @retval None
  */
__weak void BSP_MMC_ReadCpltCallback(void)
{

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

/**
  * @}
  */


