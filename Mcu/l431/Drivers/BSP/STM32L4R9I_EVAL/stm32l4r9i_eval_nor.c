/**
  ******************************************************************************
  * @file    stm32l4r9i_eval_nor.c
  * @author  MCD Application Team
  * @brief   This file includes a standard driver for the M29W128GL70ZA6E NOR
  *          memories mounted on STM32L4R9I-EVAL board.
  @verbatim
  ==============================================================================
                     ##### How to use this driver #####
  ==============================================================================
  [..]
   (#) This driver is used to drive the M29W128GL70ZA6E NOR flash external
       memory mounted on STM32L4R9I-EVAL evaluation board.

   (#) This driver does not need a specific component driver for the NOR device
       to be included with.

   (#) Initialization steps:
       (++) Initialize the NOR external memory using the BSP_NOR_Init() function. This
            function includes the MSP layer hardware resources initialization and the
            FMC controller configuration to interface with the external NOR memory.

   (#) NOR flash operations
       (++) NOR external memory can be accessed with read/write operations once it is
            initialized.
            Read/write operation can be performed with AHB access using the functions
            BSP_NOR_ReadData()/BSP_NOR_WriteData(). The BSP_NOR_WriteData() performs write operation
            of an amount of data by unit (halfword). You can also perform a program data
            operation of an amount of data using the function BSP_NOR_ProgramData().
       (++) The function BSP_NOR_Read_ID() returns the chip IDs stored in the structure
            "NOR_IDTypeDef". (see the NOR IDs in the memory data sheet)
       (++) Perform erase block operation using the function BSP_NOR_Erase_Block() and by
            specifying the block address. You can perform an erase operation of the whole
            chip by calling the function BSP_NOR_Erase_Chip().
       (++) After other operations, the function BSP_NOR_ReturnToReadMode() allows the NOR
            flash to return to read mode to perform read operations on it.
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
#include "stm32l4r9i_eval_nor.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L4R9I_EVAL
  * @{
  */

/** @defgroup STM32L4R9I_EVAL_NOR STM32L4R9I_EVAL NOR
  * @{
  */

/* Private constants ---------------------------------------------------------*/

/** @defgroup STM32L4R9I_EVAL_NOR_Private_Constants Private Constants
  * @{
  */
/* Timings for NOR M29W128GL70ZA6E */
#define NOR_ADDR_SETUP_TIME  9 /* 70ns with a clock at 120 MHz (period of 8.33 ns) */
#define NOR_DATA_SETUP_TIME  5 /* 45ns with a clock at 120 MHz (period of 8.33 ns) */
#define NOR_DATA_HOLD_TIME   0
#define NOR_TURN_AROUND_TIME 4 /* 30ns with a clock at 120 MHz (perido of 8.33 ns) */

#define NOR_MAX_BUFFER_SIZE  32 /* 16-bits bus so 32 words */

/**
  * @}
  */

/* Private variables ---------------------------------------------------------*/

/** @defgroup STM32L4R9I_EVAL_NOR_Private_Variables Private Variables
  * @{
  */
static NOR_HandleTypeDef norHandle;
static FMC_NORSRAM_TimingTypeDef Timing;

/**
  * @}
  */

/* Private functions ---------------------------------------------------------*/

/** @defgroup STM32L4R9I_EVAL_NOR_Private_Functions Private Functions
  * @{
  */
static void NOR_MspInit(void);

/**
  * @}
  */

/** @defgroup STM32L4R9I_EVAL_NOR_Exported_Functions Exported Functions
  * @{
  */

/**
  * @brief  Initializes the NOR device.
  * @retval NOR memory status
  */
uint8_t BSP_NOR_Init(void)
{
  norHandle.Instance  = FMC_NORSRAM_DEVICE;
  norHandle.Extended  = FMC_NORSRAM_EXTENDED_DEVICE;

  /* NOR device configuration */
  Timing.AddressSetupTime      = NOR_ADDR_SETUP_TIME;
  Timing.AddressHoldTime       = 1;                    /* Min value, Don't care on NOR Access mode B */
  Timing.DataSetupTime         = NOR_DATA_SETUP_TIME;
  Timing.DataHoldTime          = NOR_DATA_HOLD_TIME;
  Timing.CLKDivision           = 2;                    /* Min value, Don't care on NOR Access mode B */
  Timing.DataLatency           = 2;                    /* Min value, Don't care on NOR Access mode B */
  Timing.BusTurnAroundDuration = NOR_TURN_AROUND_TIME;
  Timing.AccessMode            = FMC_ACCESS_MODE_B;

  norHandle.Init.NSBank             = FMC_NORSRAM_BANK3;
  norHandle.Init.DataAddressMux     = FMC_DATA_ADDRESS_MUX_DISABLE;
  norHandle.Init.MemoryType         = FMC_MEMORY_TYPE_NOR;
  norHandle.Init.MemoryDataWidth    = FMC_NORSRAM_MEM_BUS_WIDTH_16;
  norHandle.Init.BurstAccessMode    = FMC_BURST_ACCESS_MODE_DISABLE;
  norHandle.Init.WaitSignal         = FMC_WAIT_SIGNAL_DISABLE;
  norHandle.Init.WaitSignalActive   = FMC_WAIT_TIMING_BEFORE_WS;
  norHandle.Init.WaitSignalPolarity = FMC_WAIT_SIGNAL_POLARITY_LOW;
  norHandle.Init.WriteOperation     = FMC_WRITE_OPERATION_ENABLE;
  norHandle.Init.ExtendedMode       = FMC_EXTENDED_MODE_DISABLE;
  norHandle.Init.AsynchronousWait   = FMC_ASYNCHRONOUS_WAIT_ENABLE;
  norHandle.Init.WriteBurst         = FMC_BURST_ACCESS_MODE_DISABLE;
  norHandle.Init.ContinuousClock    = FMC_CONTINUOUS_CLOCK_SYNC_ONLY;
  norHandle.Init.WriteFifo          = FMC_WRITE_FIFO_DISABLE;
  norHandle.Init.NBLSetupTime       = 0;
  norHandle.Init.PageSize           = FMC_PAGE_SIZE_NONE;

  /* NOR controller initialization */
  NOR_MspInit();

  if(HAL_NOR_Init(&norHandle, &Timing, &Timing) != HAL_OK)
  {
    return NOR_STATUS_ERROR;
  }
  else
  {
    return NOR_STATUS_OK;
  }
}

/**
  * @brief  Reads an amount of data from the NOR device.
  * @param  uwStartAddress: Read start address
  * @param  pData: Pointer to data to be read
  * @param  uwDataSize: Size of data to read
  * @retval NOR memory status
  */
uint8_t BSP_NOR_ReadData(uint32_t uwStartAddress, uint16_t* pData, uint32_t uwDataSize)
{
  if(HAL_NOR_ReadBuffer(&norHandle, NOR_DEVICE_ADDR + uwStartAddress, pData, uwDataSize) != HAL_OK)
  {
    return NOR_STATUS_ERROR;
  }
  else
  {
    return NOR_STATUS_OK;
  }
}

/**
  * @brief  Returns the NOR memory to read mode.
  * @retval None
  */
void BSP_NOR_ReturnToReadMode(void)
{
   HAL_NOR_ReturnToReadMode(&norHandle);
}

/**
  * @brief  Writes an amount of data to the NOR device.
  * @param  uwStartAddress: Write start address
  * @param  pData: Pointer to data to be written
  * @param  uwDataSize: Size of data to write
  * @retval NOR memory status
  */
uint8_t BSP_NOR_WriteData(uint32_t uwStartAddress, uint16_t* pData, uint32_t uwDataSize)
{
  uint32_t index = uwDataSize;

  while(index > 0)
  {
    /* Write data to NOR */
    HAL_NOR_Program(&norHandle, (uint32_t *)(NOR_DEVICE_ADDR + uwStartAddress), pData);

    /* Read NOR device status */
    if(HAL_NOR_GetStatus(&norHandle, NOR_DEVICE_ADDR, PROGRAM_TIMEOUT) != HAL_NOR_STATUS_SUCCESS)
    {
      return NOR_STATUS_ERROR;
    }

    /* Update the counters */
    index--;
    uwStartAddress += 2;
    pData++;
  }

  return NOR_STATUS_OK;
}

/**
  * @brief  Programs an amount of data to the NOR device.
  * @param  uwStartAddress: Write start address
  * @param  pData: Pointer to data to be written
  * @param  uwDataSize: Size of data to write
  * @retval NOR memory status
  */
uint8_t BSP_NOR_ProgramData(uint32_t uwStartAddress, uint16_t* pData, uint32_t uwDataSize)
{
  uint32_t buffer_size, data_nb;
  uint16_t *src_addr, *dest_addr;
  src_addr = pData;
  dest_addr = (uint16_t *)(NOR_DEVICE_ADDR + uwStartAddress);
  data_nb = uwDataSize;

  do
  {
    if (uwDataSize < NOR_MAX_BUFFER_SIZE)
    {
      buffer_size = data_nb;
    }
    else
    {
      buffer_size = NOR_MAX_BUFFER_SIZE;
    }

    /* Send NOR program buffer operation */
    HAL_NOR_ProgramBuffer(&norHandle, (uint32_t)dest_addr, src_addr, buffer_size);

    /* Return the NOR memory status */
    if(HAL_NOR_GetStatus(&norHandle, NOR_DEVICE_ADDR, PROGRAM_TIMEOUT) != HAL_NOR_STATUS_SUCCESS)
    {
      return NOR_STATUS_ERROR;
    }

    data_nb -= buffer_size;
    src_addr += buffer_size;
    dest_addr += buffer_size;
  } while(data_nb > 0);

  return NOR_STATUS_OK;
}

/**
  * @brief  Erases the specified block of the NOR device.
  * @param  BlockAddress: Block address to erase
  * @retval NOR memory status
  */
uint8_t BSP_NOR_Erase_Block(uint32_t BlockAddress)
{
  /* Send NOR erase block operation */
  HAL_NOR_Erase_Block(&norHandle, BlockAddress, NOR_DEVICE_ADDR);

  /* Return the NOR memory status */
  if(HAL_NOR_GetStatus(&norHandle, NOR_DEVICE_ADDR, BLOCKERASE_TIMEOUT) != HAL_NOR_STATUS_SUCCESS)
  {
    return NOR_STATUS_ERROR;
  }
  else
  {
    return NOR_STATUS_OK;
  }
}

/**
  * @brief  Erases the entire NOR chip.
  * @retval NOR memory status
  */
uint8_t BSP_NOR_Erase_Chip(void)
{
  /* Send NOR Erase chip operation */
  HAL_NOR_Erase_Chip(&norHandle, NOR_DEVICE_ADDR);

  /* Return the NOR memory status */
  if(HAL_NOR_GetStatus(&norHandle, NOR_DEVICE_ADDR, CHIPERASE_TIMEOUT) != HAL_NOR_STATUS_SUCCESS)
  {
    return NOR_STATUS_ERROR;
  }
  else
  {
    return NOR_STATUS_OK;
  }
}

/**
  * @brief  Reads NOR flash IDs.
  * @param  pNOR_ID : Pointer to NOR ID structure
  * @retval NOR memory status
  */
uint8_t BSP_NOR_Read_ID(NOR_IDTypeDef *pNOR_ID)
{
  if(HAL_NOR_Read_ID(&norHandle, pNOR_ID) != HAL_OK)
  {
    return NOR_STATUS_ERROR;
  }
  else
  {
    return NOR_STATUS_OK;
  }
}

/**
  * @}
  */

/** @addtogroup STM32L4R9I_EVAL_NOR_Private_Functions
  * @{
  */

/**
  * @brief  Initializes the NOR MSP.
  * @retval None
  */
static void NOR_MspInit(void)
{
  GPIO_InitTypeDef gpioinitstruct = {0};

  /* Enable FMC clock */
  __HAL_RCC_FMC_CLK_ENABLE();

  /* Enable GPIOs clock */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();

  /* Common GPIO configuration */
  gpioinitstruct.Mode      = GPIO_MODE_AF_PP;
  gpioinitstruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  gpioinitstruct.Alternate = GPIO_AF12_FMC;

  /*## NE configuration #######*/
  /* NE1 : SRAM */
  gpioinitstruct.Pull = GPIO_PULLUP;
  gpioinitstruct.Pin = GPIO_PIN_7;
  HAL_GPIO_Init(GPIOD, &gpioinitstruct);
  /* NE3 : NOR */
  /* NE4 : TFT LCD */
  gpioinitstruct.Pin = GPIO_PIN_10 | GPIO_PIN_12;
  HAL_GPIO_Init(GPIOG, &gpioinitstruct);

  /*## NOE and NWE configuration #######*/
  gpioinitstruct.Pin  = GPIO_PIN_4 | GPIO_PIN_5;
  HAL_GPIO_Init(GPIOD, &gpioinitstruct);

  /*## Ready/Busy configuration #######*/
  gpioinitstruct.Pin = NOR_READY_BUSY_PIN;
  HAL_GPIO_Init(NOR_READY_BUSY_GPIO, &gpioinitstruct);

  /*## Address Bus #######*/
  /* GPIOD configuration */
  gpioinitstruct.Pull = GPIO_NOPULL;
  gpioinitstruct.Pin  = GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13;
  HAL_GPIO_Init(GPIOD, &gpioinitstruct);

  /* GPIOE configuration */
  gpioinitstruct.Pin = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 |
                       GPIO_PIN_5 | GPIO_PIN_6;
  HAL_GPIO_Init(GPIOE, &gpioinitstruct);

  /* GPIOF configuration */
  gpioinitstruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
                       GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_12 | GPIO_PIN_13 |
                       GPIO_PIN_14 | GPIO_PIN_15;
  HAL_GPIO_Init(GPIOF, &gpioinitstruct);

  /* GPIOG configuration */
  gpioinitstruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 |
                       GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
  HAL_GPIO_Init(GPIOG, &gpioinitstruct);

  /*## Data Bus #######*/
  /* GPIOD configuration */
  gpioinitstruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_8 | GPIO_PIN_9 |
                       GPIO_PIN_10 | GPIO_PIN_14 | GPIO_PIN_15;
  HAL_GPIO_Init(GPIOD, &gpioinitstruct);

  /* GPIOE configuration */
  gpioinitstruct.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 |
                       GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 |
                       GPIO_PIN_14 | GPIO_PIN_15;
  HAL_GPIO_Init(GPIOE, &gpioinitstruct);
}

/**
  * @brief  NOR BSP Wait for Ready/Busy signal.
  * @param  hnor: Pointer to NOR handle
  * @param  Timeout: Timeout duration
  * @retval None
  */
void HAL_NOR_MspWait(NOR_HandleTypeDef *hnor, uint32_t Timeout)
{
#if 0
  uint32_t timeout = Timeout;

  /* Polling on Ready/Busy signal */
  while((HAL_GPIO_ReadPin(NOR_READY_BUSY_GPIO, NOR_READY_BUSY_PIN) != NOR_BUSY_STATE) && (timeout > 0))
  {
    timeout--;
  }

  timeout = Timeout;

  /* Polling on Ready/Busy signal */
  while((HAL_GPIO_ReadPin(NOR_READY_BUSY_GPIO, NOR_READY_BUSY_PIN) != NOR_READY_STATE) && (timeout > 0))
  {
    timeout--;
  }
#endif
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



