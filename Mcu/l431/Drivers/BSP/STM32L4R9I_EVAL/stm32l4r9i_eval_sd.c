/**
  ******************************************************************************
  * @file    stm32l4r9i_eval_sd.c
  * @author  MCD Application Team
  * @brief   This file includes the uSD card driver.
  @verbatim
  ==============================================================================
                     ##### How to use this driver #####
  ==============================================================================
  (#) This driver is used to drive the micro SD external card mounted on STM32L4R9I_EVAL
     evaluation board.

  (#) This driver does not need a specific component driver for the micro SD device
     to be included with.

  (#) Initialization steps:
       (++) Initialize the micro SD card using the BSP_SD_Init() function. This
            function includes the MSP layer hardware resources initialization (BSP_SD_MspInit())
            and the SDMMC1 interface configuration to interface with the external micro SD. It
            also includes the micro SD initialization sequence.
       (++) To check the SD card presence you can use the function BSP_SD_IsDetected() which
            returns the detection status.
       (++) If SD presence detection interrupt mode is desired, you must configure the
            SD detection interrupt mode by calling the functions BSP_SD_ITConfig().
            The interrupt is generated as an external interrupt whenever the
            micro SD card is plugged/unplugged in/from the evaluation board. The SD detection
            is managed by MFX, so the SD detection interrupt has to be treated by MFX_IRQOUT
            gpio pin IRQ handler.
       (++) The function BSP_SD_GetCardInfo() is used to get the micro SD card information
            which is stored in the structure "HAL_SD_CardInfoTypedef".

  (#) Micro SD card operations
       (++) The micro SD card can be accessed with read/write block(s) operations once
            it is reay for access. The access can be performed whether using the polling
            mode by calling the functions BSP_SD_ReadBlocks()/BSP_SD_WriteBlocks(),
            or by DMA transfer using the functions BSP_SD_ReadBlocks_DMA()/BSP_SD_WriteBlocks_DMA().
       (++) The DMA transfer complete is used with interrupt mode. Once the SD transfer
            is complete, the DMA Tx/Rx transfer complete are handled using the
            BSP_SD_WriteCpltCallback()/BSP_SD_ReadCpltCallback() user callback functions implemented
            by the user at application level.
       (++) The SD erase block(s) is performed using the function BSP_SD_Erase() with specifying
            the number of blocks to erase.
       (++) The SD runtime status is returned when calling the function BSP_SD_GetStatus().
   [..]
  @endverbatim
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32l4r9i_eval_io.h"
#include "stm32l4r9i_eval_sd.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L4R9I_EVAL
  * @{
  */

/** @defgroup STM32L4R9I_EVAL_SD STM32L4R9I_EVAL SD
  * @{
  */

/* Private variables ---------------------------------------------------------*/

/** @defgroup STM32L4R9I_EVAL_SD_Private_Variables Private Variables
  * @{
  */
SD_HandleTypeDef hsd_eval;
static uint8_t UseExtiModeDetection = 0;
/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/

/* Exported functions ---------------------------------------------------------*/

/** @addtogroup STM32L4R9I_EVAL_SD_Exported_Functions
  * @{
  */

/**
  * @brief  Initializes the SD card device.
  * @retval SD status
  */
uint8_t BSP_SD_Init(void)
{
  uint8_t sd_state = MSD_OK;

  /* uSD device interface configuration */
  hsd_eval.Instance = SDMMC1;
  hsd_eval.Init.ClockEdge           = SDMMC_CLOCK_EDGE_RISING;
  hsd_eval.Init.ClockPowerSave      = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd_eval.Init.BusWide             = SDMMC_BUS_WIDE_4B;
  hsd_eval.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_ENABLE;
  hsd_eval.Init.ClockDiv            = 1;
  hsd_eval.Init.Transceiver         = SDMMC_TRANSCEIVER_DISABLE;

  /* Initialize IO functionalities (MFX) used by SD detect pin */
  BSP_IO_Init();

  /* Check if the SD card is plugged in the slot */
  BSP_IO_ConfigPin(SD_DETECT_PIN, IO_MODE_INPUT_PU);

  /* Check if the SD card is plugged in the slot */
  if(BSP_SD_IsDetected() != SD_PRESENT)
  {
    return MSD_ERROR_SD_NOT_PRESENT;
  }

  /* Msp SD initialization */
  BSP_SD_MspInit(&hsd_eval, NULL);

  /* HAL SD initialization */
  if(HAL_SD_Init(&hsd_eval) != HAL_OK)
  {
    sd_state = MSD_ERROR;
  }

  return  sd_state;
}

/**
  * @brief  DeInitializes the SD card device.
  * @retval SD status
  */
uint8_t BSP_SD_DeInit(void)
{
  uint8_t sd_state = MSD_OK;

  hsd_eval.Instance = SDMMC1;

  /* Set back Mfx pin to INPUT mode in case it was in exti */
  UseExtiModeDetection = 0;

  /* HAL SD deinitialization */
  if(HAL_SD_DeInit(&hsd_eval) != HAL_OK)
  {
    sd_state = MSD_ERROR;
  }

  /* Msp SD deinitialization */
  BSP_SD_MspDeInit(&hsd_eval, NULL);

  return  sd_state;
}

/**
  * @brief  Configures Interrupt mode for SD detection pin.
  * @retval IO_OK: if all initializations are OK. Other value if error.
  */
uint8_t BSP_SD_ITConfig(void)
{
  /* Configure Interrupt mode for SD detection pin */
  /* Note: disabling exti mode can be done calling BSP_SD_DeInit() */
  UseExtiModeDetection = 1;

  /* Check SD card detect pin */
  if(BSP_IO_ReadPin(SD_DETECT_PIN) == IO_PIN_RESET)
  {
    /* Card present */
    return BSP_IO_ConfigPin(SD_DETECT_PIN, IO_MODE_IT_RISING_EDGE_PU);
  }
  else
  {
    /* Card not present */
    return BSP_IO_ConfigPin(SD_DETECT_PIN, IO_MODE_IT_FALLING_EDGE_PU);
  }
}

/**
 * @brief  Detects if SD card is correctly plugged in the memory slot or not.
 * @retval Returns if SD is detected or not
 */
uint8_t BSP_SD_IsDetected(void)
{
  __IO uint8_t status = SD_PRESENT;

  /* Check SD card detect pin */
  if (BSP_IO_ReadPin(SD_DETECT_PIN) == IO_PIN_RESET)
  {
    /* Card present */
    if (UseExtiModeDetection)
    {
      BSP_IO_ConfigPin(SD_DETECT_PIN, IO_MODE_IT_RISING_EDGE_PU);
    }
  }
  else
  {
    /* Card not present */
    status = SD_NOT_PRESENT;
    if (UseExtiModeDetection)
    {
      BSP_IO_ConfigPin(SD_DETECT_PIN, IO_MODE_IT_FALLING_EDGE_PU);
    }
  }

  return status;
}

/**
  * @brief  Reads block(s) from a specified address in an SD card, in polling mode.
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  ReadAddr: Address from where data is to be read
  * @param  NumOfBlocks: Number of SD blocks to read
  * @param  Timeout: Timeout for read operation
  * @retval SD status
  */
uint8_t BSP_SD_ReadBlocks(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks, uint32_t Timeout)
{
  HAL_StatusTypeDef  sd_state = HAL_OK;

  sd_state =  HAL_SD_ReadBlocks(&hsd_eval, (uint8_t *)pData, ReadAddr, NumOfBlocks, Timeout);

  if  (sd_state == HAL_OK)
  {
    return MSD_OK;
  }
  else
  {
    return MSD_ERROR;
  }
}

/**
  * @brief  Writes block(s) to a specified address in an SD card, in polling mode.
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  WriteAddr: Address from where data is to be written
  * @param  NumOfBlocks: Number of SD blocks to write
  * @param  Timeout: Timeout for write operation
  * @retval SD status
  */
uint8_t BSP_SD_WriteBlocks(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks, uint32_t Timeout)
{
  HAL_StatusTypeDef  sd_state = HAL_OK;

  sd_state = HAL_SD_WriteBlocks(&hsd_eval, (uint8_t *)pData, WriteAddr, NumOfBlocks, Timeout);

  if( sd_state == HAL_OK)
  {
    return MSD_OK;
  }
  else
  {
    return MSD_ERROR;
  }
}

/**
  * @brief  Reads block(s) from a specified address in an SD card, in DMA mode.
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  ReadAddr: Address from where data is to be read
  * @param  NumOfBlocks: Number of SD blocks to read
  * @retval SD status
  */
uint8_t BSP_SD_ReadBlocks_DMA(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks)
{
  HAL_StatusTypeDef  sd_state = HAL_OK;

  /* Read block(s) in DMA transfer mode */
  sd_state = HAL_SD_ReadBlocks_DMA(&hsd_eval, (uint8_t *)pData, ReadAddr, NumOfBlocks);

  if( sd_state == HAL_OK)
  {
    return MSD_OK;
  }
  else
  {
    return MSD_ERROR;
  }
}

/**
  * @brief  Writes block(s) to a specified address in an SD card, in DMA mode.
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  WriteAddr: Address from where data is to be written
  * @param  NumOfBlocks: Number of SD blocks to write
  * @retval SD status
  */
uint8_t BSP_SD_WriteBlocks_DMA(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks)
{
  HAL_StatusTypeDef  sd_state = HAL_OK;

  /* Write block(s) in DMA transfer mode */
  sd_state = HAL_SD_WriteBlocks_DMA(&hsd_eval, (uint8_t *)pData, WriteAddr, NumOfBlocks);

  if( sd_state == HAL_OK)
  {
    return MSD_OK;
  }
  else
  {
    return MSD_ERROR;
  }
}

/**
  * @brief  Erases the specified memory area of the given SD card.
  * @param  StartAddr: Start byte address
  * @param  EndAddr: End byte address
  * @retval SD status
  */
uint8_t BSP_SD_Erase(uint32_t StartAddr, uint32_t EndAddr)
{
  HAL_StatusTypeDef  sd_state = HAL_OK;

  sd_state = HAL_SD_Erase(&hsd_eval, StartAddr, EndAddr);

  if( sd_state == HAL_OK)
  {
    return MSD_OK;
  }
  else
  {
    return MSD_ERROR;
  }
}

/**
  * @brief  Gets the current SD card data status.
  * @retval Data transfer state.
  *          This value can be one of the following values:
  *            @arg  SD_TRANSFER_OK: No data transfer is acting
  *            @arg  SD_TRANSFER_BUSY: Data transfer is acting
  */
uint8_t BSP_SD_GetCardState(void)
{
  return((HAL_SD_GetCardState(&hsd_eval) == HAL_SD_CARD_TRANSFER ) ? SD_TRANSFER_OK : SD_TRANSFER_BUSY);
}

/**
  * @brief  Get SD information about specific SD card.
  * @param  CardInfo: Pointer to HAL_SD_CardInfoTypedef structure
  * @retval None
  */
void BSP_SD_GetCardInfo(BSP_SD_CardInfo *CardInfo)
{
  /* Get SD card Information */
  HAL_SD_GetCardInfo(&hsd_eval, CardInfo);
}

/**
  * @brief  Initializes the SD MSP.
  * @retval None
  */
__weak void BSP_SD_MspInit(SD_HandleTypeDef *hsd, void *Params)
{
  GPIO_InitTypeDef gpioinitstruct = {0};
  RCC_OscInitTypeDef        RCC_OscInitStruct;
  RCC_PeriphCLKInitTypeDef  RCC_PeriphClkInit;

  /* Check whether HSI48 is enabled or not */
  HAL_RCC_GetOscConfig(&RCC_OscInitStruct);
  if(RCC_OscInitStruct.HSI48State != RCC_HSI48_ON)
  {
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
    RCC_OscInitStruct.HSI48State     = RCC_HSI48_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_NONE;
    if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
      while(1) {}
    }
  }

  /* Configure the Eval SDMMC1 clock source. The clock is derived from the HSI48 */
  RCC_PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SDMMC1;
  RCC_PeriphClkInit.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_HSI48;
  if(HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphClkInit) != HAL_OK)
  {
    while(1) {}
  }

  /* Enable SDMMC1 clock */
  __HAL_RCC_SDMMC1_CLK_ENABLE();

  /* Enable GPIOs clock */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* Common GPIO configuration */
  gpioinitstruct.Mode      = GPIO_MODE_AF_PP;
  gpioinitstruct.Pull      = GPIO_NOPULL;
  gpioinitstruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  gpioinitstruct.Alternate = GPIO_AF12_SDMMC1;

  /* GPIOC configuration */
  gpioinitstruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
  HAL_GPIO_Init(GPIOC, &gpioinitstruct);

  /* GPIOD configuration */
  gpioinitstruct.Pin = GPIO_PIN_2;
  HAL_GPIO_Init(GPIOD, &gpioinitstruct);

  /* NVIC configuration for SDMMC1 interrupts */
  HAL_NVIC_SetPriority(SDMMCx_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(SDMMCx_IRQn);
}

/**
  * @brief  De-Initializes the SD MSP.
  * @retval None
  */
__weak void BSP_SD_MspDeInit(SD_HandleTypeDef *hsd, void *Params)
{
  GPIO_InitTypeDef gpioinitstruct = {0};

  /* NVIC configuration for SDMMC1 interrupts */
  HAL_NVIC_DisableIRQ(SDMMCx_IRQn);

  /* Disable SDMMC1 clock */
  __HAL_RCC_SDMMC1_CLK_DISABLE();

  /* Enable GPIOs clock */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* Common GPIO configuration */
  gpioinitstruct.Mode      = GPIO_MODE_ANALOG;
  gpioinitstruct.Pull      = GPIO_NOPULL;
  gpioinitstruct.Speed     = GPIO_SPEED_FREQ_LOW;
  gpioinitstruct.Alternate = 0;

  /* GPIOC configuration */
  gpioinitstruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
  HAL_GPIO_Init(GPIOC, &gpioinitstruct);

  /* GPIOD configuration */
  gpioinitstruct.Pin = GPIO_PIN_2;
  HAL_GPIO_Init(GPIOD, &gpioinitstruct);
}

/**
  * @brief BSP SD Abort callback
  * @retval None
  */
__weak void BSP_SD_AbortCallback(void)
{

}

/**
  * @brief BSP Tx Transfer completed callback
  * @retval None
  */
__weak void BSP_SD_WriteCpltCallback(void)
{

}

/**
  * @brief BSP Rx Transfer completed callback
  * @retval None
  */
__weak void BSP_SD_ReadCpltCallback(void)
{

}

/**
  * @brief SD Abort callback
  * @param hsd: SD handle
  * @retval None
  */
void HAL_SD_AbortCallback(SD_HandleTypeDef *hsd)
{
  BSP_SD_AbortCallback();
}

/**
  * @brief Tx Transfer completed callback
  * @param hsd: SD handle
  * @retval None
  */
void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd)
{
  BSP_SD_WriteCpltCallback();
}

/**
  * @brief Rx Transfer completed callback
  * @param hsd: SD handle
  * @retval None
  */
void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsd)
{
  BSP_SD_ReadCpltCallback();
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


