/**
  ******************************************************************************
  * @file    stm32l4r9i_discovery_psram.c
  * @author  MCD Application Team
  * @brief   This file includes the PSRAM driver for the IS61WV51216BLL-10MLI memory
  *          device mounted on STM32L4R9I_DISCOVERY boards.
  @verbatim
  How To use this driver:
  -----------------------
   - This driver is used to drive the IS66WVC2M16ECLL-7010BLI external memory mounted
     on STM32L4R9I discovery board.
   - This driver does not need a specific component driver for the PSRAM device
     to be included with.

  Driver description:
  ------------------
  + Initialization steps:
     o Initialize the PSRAM external memory using the BSP_PSRAM_Init() function. This
       function includes the MSP layer hardware resources initialization and the
       FMC controller configuration to interface with the external PSRAM memory.

  + PSRAM read/write operations
     o PSRAM external memory can be accessed with read/write operations once it is
       initialized.
       Read/write operation can be performed with AHB access using the functions
       BSP_PSRAM_ReadData()/BSP_PSRAM_WriteData(), or by DMA transfer using the functions
       BSP_PSRAM_ReadData_DMA()/BSP_PSRAM_WriteData_DMA().
     o The AHB access is performed with 16-bit width transaction, the DMA transfer
       configuration is fixed at single (no burst) halfword transfer.
     o User can implement his own functions for read/write access with his desired
       configurations.
     o If interrupt mode is used for DMA transfer, the function BSP_PSRAM_DMA_IRQHandler()
       is called in IRQ handler file, to serve the generated interrupt once the DMA
       transfer is complete.
  @endverbatim
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32l4r9i_discovery_psram.h"
#include "stm32l4r9i_discovery_io.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L4R9I_DISCOVERY
  * @{
  */

/** @defgroup STM32L4R9I_DISCOVERY_PSRAM STM32L4R9I_DISCOVERY PSRAM
  * @{
  */

/** @defgroup STM32L4R9I_DISCOVERY_PSRAM_Private_Variables Exported Variables
  * @{
  */
SRAM_HandleTypeDef psramHandle = {0};

/* LCD/PSRAM initialization status sharing the same power source */
extern uint32_t bsp_lcd_initialized;
extern uint32_t bsp_psram_initialized;

/**
  * @}
  */

/** @defgroup STM32L4R9I_DISCOVERY_PSRAM_Private_Function_Prototypes Private Function Prototypes
  * @{
  */
static void PSRAM_PowerOn(void);
static void PSRAM_PowerOff(void);
/**
  * @}
  */

/** @defgroup STM32L4R9I_DISCOVERY_PSRAM_Private_Functions Private Functions
  * @{
  */

/**
  * @brief  Initializes the PSRAM device.
  * @retval PSRAM status
  */
uint8_t BSP_PSRAM_Init(void)
{
  static uint8_t psram_status = PSRAM_OK;

  if (bsp_psram_initialized == 0)
  {
    static FMC_NORSRAM_TimingTypeDef Timing;

    /* Power on PSRAM */
    PSRAM_PowerOn();

    /* PSRAM device configuration */
    /* Timing configuration derived from system clock (up to 120Mhz)
    for 60Mhz as PSRAM clock frequency */
    Timing.AddressSetupTime      = 4;
    Timing.AddressHoldTime       = 2;
    Timing.DataSetupTime         = 6;
    Timing.BusTurnAroundDuration = 1;
    Timing.CLKDivision           = 2;
    Timing.DataLatency           = 2;
    Timing.AccessMode        = FMC_ACCESS_MODE_A;

    psramHandle.Init.NSBank             = FMC_NORSRAM_BANK1;
    psramHandle.Init.DataAddressMux     = FMC_DATA_ADDRESS_MUX_DISABLE;
    psramHandle.Init.MemoryType         = FMC_MEMORY_TYPE_PSRAM;
    psramHandle.Init.MemoryDataWidth    = FMC_NORSRAM_MEM_BUS_WIDTH_16;
    psramHandle.Init.BurstAccessMode    = FMC_BURST_ACCESS_MODE_DISABLE;
    psramHandle.Init.WaitSignalPolarity = FMC_WAIT_SIGNAL_POLARITY_HIGH;
    psramHandle.Init.WaitSignalActive   = FMC_WAIT_TIMING_BEFORE_WS;
    psramHandle.Init.WriteOperation     = FMC_WRITE_OPERATION_ENABLE;
    psramHandle.Init.WaitSignal         = FMC_WAIT_SIGNAL_DISABLE;
    psramHandle.Init.ExtendedMode       = FMC_EXTENDED_MODE_DISABLE;
    psramHandle.Init.AsynchronousWait   = FMC_ASYNCHRONOUS_WAIT_DISABLE;
    psramHandle.Init.WriteBurst         = FMC_WRITE_BURST_DISABLE;
    psramHandle.Init.ContinuousClock    = FMC_CONTINUOUS_CLOCK_SYNC_ASYNC;
    psramHandle.Init.WriteFifo          = FMC_WRITE_FIFO_DISABLE;
    psramHandle.Init.NBLSetupTime       = 0;
    psramHandle.Init.PageSize           = FMC_PAGE_SIZE_NONE;

    psramHandle.Instance  = FMC_NORSRAM_DEVICE;
    psramHandle.Extended  = FMC_NORSRAM_EXTENDED_DEVICE;

    /* PSRAM controller initialization */
    BSP_PSRAM_MspInit(&psramHandle, NULL); /* __weak function can be rewritten by the application */
    if(HAL_SRAM_Init(&psramHandle, &Timing, &Timing) != HAL_OK)
    {
      psram_status = PSRAM_ERROR;
    }
    else
    {
      psram_status = PSRAM_OK;
    }

    bsp_psram_initialized = 1;
  }

  return psram_status;
}

/**
  * @brief  DeInitializes the PSRAM device.
  * @retval PSRAM status
  */
uint8_t BSP_PSRAM_DeInit(void)
{
  static uint8_t psram_status = PSRAM_OK;

  if (bsp_psram_initialized == 1)
  {
    /* PSRAM device de-initialization */
    psramHandle.Instance = FMC_NORSRAM_DEVICE;
    psramHandle.Extended = FMC_NORSRAM_EXTENDED_DEVICE;

    if(HAL_SRAM_DeInit(&psramHandle) != HAL_OK)
    {
      psram_status = PSRAM_ERROR;
    }
    else
    {
      psram_status = PSRAM_OK;
    }

    /* PSRAM controller de-initialization */
    BSP_PSRAM_MspDeInit(&psramHandle, NULL);

    /* Power off PSRAM */
    PSRAM_PowerOff();

    bsp_psram_initialized = 0;
  }

  return psram_status;
}

/**
  * @brief  Reads an amount of data from the PSRAM device in polling mode.
  * @param  uwStartAddress: Read start address
  * @param  pData: Pointer to data to be read
  * @param  uwDataSize: Size of read data from the memory
  * @retval PSRAM status
  */
uint8_t BSP_PSRAM_ReadData(uint32_t uwStartAddress, uint16_t *pData, uint32_t uwDataSize)
{
  if(HAL_SRAM_Read_16b(&psramHandle, (uint32_t *)uwStartAddress, pData, uwDataSize) != HAL_OK)
  {
    return PSRAM_ERROR;
  }
  else
  {
    return PSRAM_OK;
  }
}

/**
  * @brief  Reads an amount of data from the PSRAM device in DMA mode.
  * @param  uwStartAddress: Read start address
  * @param  pData: Pointer to data to be read
  * @param  uwDataSize: Size of read data from the memory
  * @retval PSRAM status
  */
uint8_t BSP_PSRAM_ReadData_DMA(uint32_t uwStartAddress, uint16_t *pData, uint32_t uwDataSize)
{
  if(HAL_SRAM_Read_DMA(&psramHandle, (uint32_t *)uwStartAddress, (uint32_t *)pData, uwDataSize) != HAL_OK)
  {
    return PSRAM_ERROR;
  }
  else
  {
    return PSRAM_OK;
  }
}

/**
  * @brief  Writes an amount of data from the PSRAM device in polling mode.
  * @param  uwStartAddress: Write start address
  * @param  pData: Pointer to data to be written
  * @param  uwDataSize: Size of written data from the memory
  * @retval PSRAM status
  */
uint8_t BSP_PSRAM_WriteData(uint32_t uwStartAddress, uint16_t *pData, uint32_t uwDataSize)
{
  if(HAL_SRAM_Write_16b(&psramHandle, (uint32_t *)uwStartAddress, pData, uwDataSize) != HAL_OK)
  {
    return PSRAM_ERROR;
  }
  else
  {
    return PSRAM_OK;
  }
}

/**
  * @brief  Writes an amount of data from the PSRAM device in DMA mode.
  * @param  uwStartAddress: Write start address
  * @param  pData: Pointer to data to be written
  * @param  uwDataSize: Size of written data from the memory
  * @retval PSRAM status
  */
uint8_t BSP_PSRAM_WriteData_DMA(uint32_t uwStartAddress, uint16_t *pData, uint32_t uwDataSize)
{
  if(HAL_SRAM_Write_DMA(&psramHandle, (uint32_t *)uwStartAddress, (uint32_t *)pData, uwDataSize) != HAL_OK)
  {
    return PSRAM_ERROR;
  }
  else
  {
    return PSRAM_OK;
  }
}

/**
  * @brief  Initializes PSRAM MSP.
  * @param  hsram: PSRAM handle
  * @param  Params
  * @retval None
  */
__weak void BSP_PSRAM_MspInit(SRAM_HandleTypeDef  *hsram, void *Params)
{
  static DMA_HandleTypeDef dma_handle;
  GPIO_InitTypeDef GPIO_Init_Structure;

  /* Enable DMAx clock */
  __PSRAM_DMAx_CLK_ENABLE();
  __HAL_RCC_DMAMUX1_CLK_ENABLE();

  /* Enable FMC clock */
  __HAL_RCC_FMC_CLK_ENABLE();

  /* Enable GPIOs clock */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  /* IOSV bit MUST be set to access GPIO port G[2:15] */
  __HAL_RCC_PWR_CLK_ENABLE();
  SET_BIT(PWR->CR2, PWR_CR2_IOSV);

  /* Common GPIO configuration */
  GPIO_Init_Structure.Mode      = GPIO_MODE_AF_PP;
  GPIO_Init_Structure.Pull      = GPIO_NOPULL;
  GPIO_Init_Structure.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_Init_Structure.Alternate = GPIO_AF12_FMC;

  /*## Data Bus #######*/
  /* GPIOD configuration */
  GPIO_Init_Structure.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_8 | GPIO_PIN_9 |
                              GPIO_PIN_10 | GPIO_PIN_14 | GPIO_PIN_15;
  HAL_GPIO_Init(GPIOD, &GPIO_Init_Structure);

  /* GPIOE configuration */
  GPIO_Init_Structure.Pin   = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 |
                              GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 |
                              GPIO_PIN_14 | GPIO_PIN_15;
  HAL_GPIO_Init(GPIOE, &GPIO_Init_Structure);


  /*## Address Bus #######*/
  /* GPIOF configuration */
  GPIO_Init_Structure.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
                              GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_12 | GPIO_PIN_13 |
                              GPIO_PIN_14 | GPIO_PIN_15;
  HAL_GPIO_Init(GPIOF, &GPIO_Init_Structure);


  /* GPIOG configuration */
  GPIO_Init_Structure.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 |
                              GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
  HAL_GPIO_Init(GPIOG, &GPIO_Init_Structure);

  /* GPIOD configuration */
  GPIO_Init_Structure.Pin   = GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13;
  HAL_GPIO_Init(GPIOD, &GPIO_Init_Structure);

  /* GPIOE configuration */
  GPIO_Init_Structure.Pin   = GPIO_PIN_3 | GPIO_PIN_4;
  HAL_GPIO_Init(GPIOE, &GPIO_Init_Structure);
  GPIO_Init_Structure.Pull = GPIO_PULLUP;


  /*## NOE and NWE configuration #######*/
  GPIO_Init_Structure.Pin = GPIO_PIN_4 |GPIO_PIN_5;
  HAL_GPIO_Init(GPIOD, &GPIO_Init_Structure);

  /* Chip select configuration */
  /*## NE1 configuration #######*/
  GPIO_Init_Structure.Pin = GPIO_PIN_7;
  HAL_GPIO_Init(GPIOD, &GPIO_Init_Structure);

  /*## NE2, NE3, NE4 configuration #######*/
  GPIO_Init_Structure.Pin = GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_12;
  HAL_GPIO_Init(GPIOG, &GPIO_Init_Structure);

  /*## NBL0, NBL1 configuration #######*/
  GPIO_Init_Structure.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  GPIO_Init_Structure.Pull      = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_Init_Structure);


  GPIO_Init_Structure.Pull      = GPIO_PULLDOWN;
  /*## CLK and NWAIT configuration #######*/
  GPIO_Init_Structure.Pin = GPIO_PIN_3 | GPIO_PIN_6;
  HAL_GPIO_Init(GPIOD, &GPIO_Init_Structure);

  /*## ADVn configuration #######*/
  GPIO_Init_Structure.Pin = GPIO_PIN_7;
  HAL_GPIO_Init(GPIOB, &GPIO_Init_Structure);

  /* Configure common DMA parameters */
  dma_handle.Init.Direction           = DMA_MEMORY_TO_MEMORY;
  dma_handle.Init.PeriphInc           = DMA_PINC_ENABLE;
  dma_handle.Init.MemInc              = DMA_MINC_ENABLE;
  dma_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  dma_handle.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
  dma_handle.Init.Mode                = DMA_NORMAL;
  dma_handle.Init.Priority            = DMA_PRIORITY_HIGH;

  dma_handle.Instance = PSRAM_DMAx_INSTANCE;

  /* Deinitialize the Channel for new transfer */
  HAL_DMA_DeInit(&dma_handle);

  /* Configure the DMA Channel */
  HAL_DMA_Init(&dma_handle);

  /* Associate the DMA handle to the FMC SRAM one */
  hsram->hdma = &dma_handle;

  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);

  /* NVIC configuration for DMA transfer complete interrupt */
  HAL_NVIC_SetPriority(PSRAM_DMAx_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(PSRAM_DMAx_IRQn);
}


/**
  * @brief  DeInitializes SRAM MSP.
  * @param  hsram: SRAM handle
  * @param  Params
  * @retval None
  */
__weak void BSP_PSRAM_MspDeInit(SRAM_HandleTypeDef  *hsram, void *Params)
{
  static DMA_HandleTypeDef dma_handle;

  /* Disable NVIC configuration for DMA interrupt */
  HAL_NVIC_DisableIRQ(PSRAM_DMAx_IRQn);

  /* Deinitialize the stream for new transfer */
  dma_handle.Instance = PSRAM_DMAx_INSTANCE;
  HAL_DMA_DeInit(&dma_handle);

  /* Deinitialize GPIOs */
  HAL_GPIO_DeInit(GPIOB, GPIO_PIN_7);
  HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0  | GPIO_PIN_1  | GPIO_PIN_3  | GPIO_PIN_4  |
                         GPIO_PIN_5  | GPIO_PIN_6  | GPIO_PIN_7  | GPIO_PIN_8  |
                         GPIO_PIN_9  | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 |
                         GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
  HAL_GPIO_DeInit(GPIOE, GPIO_PIN_0  | GPIO_PIN_1  | GPIO_PIN_3  | GPIO_PIN_4  |
                         GPIO_PIN_7  | GPIO_PIN_8  | GPIO_PIN_9  | GPIO_PIN_10 |
                         GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 |
                         GPIO_PIN_15);
  HAL_GPIO_DeInit(GPIOF, GPIO_PIN_0  | GPIO_PIN_1  | GPIO_PIN_2  | GPIO_PIN_3  |
                         GPIO_PIN_4  | GPIO_PIN_5  | GPIO_PIN_12 | GPIO_PIN_13 |
                         GPIO_PIN_14 | GPIO_PIN_15);
  HAL_GPIO_DeInit(GPIOG, GPIO_PIN_0  | GPIO_PIN_1  | GPIO_PIN_2  | GPIO_PIN_3  |
                         GPIO_PIN_4  | GPIO_PIN_5  | GPIO_PIN_9  | GPIO_PIN_10 |
                         GPIO_PIN_12);

  /* GPIO pins clock, FMC clock and DMA clock can be shut down in the applications
     by surcharging this __weak function */
}

/**
  * @brief  PSRAM power on
  *         Power on PSRAM.
  */
static void PSRAM_PowerOn(void)
{
  /* Configure DSI_RESET and DSI_POWER_ON only if lcd is not currently used */
  if(bsp_lcd_initialized == 0)
  {
    BSP_IO_Init();

#if defined(USE_STM32L4R9I_DISCO_REVA) || defined(USE_STM32L4R9I_DISCO_REVB)
    /* Set DSI_POWER_ON to input floating to avoid I2C issue during input PD configuration */
    BSP_IO_ConfigPin(IO_PIN_8, IO_MODE_INPUT);

    /* Configure the GPIO connected to DSI_RESET signal */
    BSP_IO_ConfigPin(IO_PIN_10, IO_MODE_OUTPUT);

    /* Activate DSI_RESET (active low) */
    BSP_IO_WritePin(IO_PIN_10, GPIO_PIN_RESET);

    /* Configure the GPIO connected to DSI_POWER_ON signal as input pull down */
    /* to activate 3V3_LCD. VDD_LCD is also activated if VDD = 3,3V */
    BSP_IO_ConfigPin(IO_PIN_8, IO_MODE_INPUT_PD);

    /* Wait at least 1ms before enabling 1V8_LCD */
    HAL_Delay(1);

    /* Configure the GPIO connected to DSI_POWER_ON signal as output low */
    /* to activate 1V8_LCD. VDD_LCD is also activated if VDD = 1,8V */
    BSP_IO_WritePin(IO_PIN_8, GPIO_PIN_RESET);
    BSP_IO_ConfigPin(IO_PIN_8, IO_MODE_OUTPUT);
#else /* USE_STM32L4R9I_DISCO_REVA || USE_STM32L4R9I_DISCO_REVB */
    /* Configure the GPIO connected to DSI_3V3_POWERON signal as output low */
    /* to activate 3V3_LCD. VDD_LCD is also activated if VDD = 3,3V */
    BSP_IO_WritePin(IO_PIN_8, GPIO_PIN_RESET);
    BSP_IO_ConfigPin(IO_PIN_8, IO_MODE_OUTPUT);

    /* Wait at least 1ms before enabling 1V8_LCD */
    HAL_Delay(1);

    /* Configure the GPIO connected to DSI_1V8_POWERON signal as output low */
    /* to activate 1V8_LCD. VDD_LCD is also activated if VDD = 1,8V */
    BSP_IO_WritePin(AGPIO_PIN_2, GPIO_PIN_RESET);
    BSP_IO_ConfigPin(AGPIO_PIN_2, IO_MODE_OUTPUT);
#endif /* USE_STM32L4R9I_DISCO_REVA || USE_STM32L4R9I_DISCO_REVB */

    /* Wait at least 15 ms (minimum reset low width is 10ms and add margin for 1V8_LCD ramp-up) */
    HAL_Delay(15);
  }
}

/**
  * @brief  PSRAM power off
  *         Power off PSRAM.
  */
static void PSRAM_PowerOff(void)
{
  /* Set DSI_POWER_ON to analog mode only if lcd is not currently used */
  if(bsp_lcd_initialized == 0)
  {
#if defined(USE_STM32L4R9I_DISCO_REVA) || defined(USE_STM32L4R9I_DISCO_REVB)
    BSP_IO_ConfigPin(IO_PIN_8, IO_MODE_ANALOG);
#else /* USE_STM32L4R9I_DISCO_REVA || USE_STM32L4R9I_DISCO_REVB */
    /* Disable first DSI_1V8_PWRON then DSI_3V3_PWRON */
    BSP_IO_ConfigPin(AGPIO_PIN_2, IO_MODE_ANALOG);
    BSP_IO_ConfigPin(IO_PIN_8, IO_MODE_ANALOG);
#endif /* USE_STM32L4R9I_DISCO_REVA || USE_STM32L4R9I_DISCO_REVB */
  }
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


