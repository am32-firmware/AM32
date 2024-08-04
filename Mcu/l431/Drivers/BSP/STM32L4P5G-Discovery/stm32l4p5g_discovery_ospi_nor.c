/**
  ******************************************************************************
  * @file    stm32l4p5g_discovery_ospi_nor.c
  * @author  MCD Application Team
  * @brief   This file includes a standard driver for the MX25LM51245G OSPI
  *          memory mounted on STM32L4P5G_DISCOVERY board.
  @verbatim
  ==============================================================================
                     ##### How to use this driver #####
  ==============================================================================
  [..]
   (#) This driver is used to drive the MX25LM51245G OSPI external
       memory mounted on STM32L4P5G_DISCOVERY evaluation board.

   (#) This driver need a specific component driver (MX25LM51245G) to be included with.

   (#) Initialization steps:
       (++) Initialize the OPSI external memory using the BSP_OSPI_NOR_Init() function. This
            function includes the MSP layer hardware resources initialization and the
            OSPI interface with the external memory.

   (#) OSPI memory operations
       (++) OSPI memory can be accessed with read/write operations once it is
            initialized.
            Read/write operation can be performed with AHB access using the functions
            BSP_OSPI_NOR_Read()/BSP_OSPI_NOR_Write().
       (++) The function BSP_OSPI_NOR_GetInfo() returns the configuration of the OSPI memory.
            (see the OSPI memory data sheet)
       (++) Perform erase block operation using the function BSP_OSPI_NOR_Erase_Block() and by
            specifying the block address. You can perform an erase operation of the whole
            chip by calling the function BSP_OSPI_NOR_Erase_Chip().
       (++) The function BSP_OSPI_NOR_GetStatus() returns the current status of the OSPI memory.
            (see the OSPI memory data sheet)
  @endverbatim
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

/* Includes ------------------------------------------------------------------*/
#include "stm32l4p5g_discovery_ospi_nor.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L4P5G_DISCOVERY
  * @{
  */

/** @defgroup STM32L4P5G_DISCOVERY_OSPI_NOR STM32L4P5G_DISCOVERY OSPI NOR
  * @{
  */

/* Private constants --------------------------------------------------------*/
/** @defgroup STM32L4P5G_DISCOVERY_OSPI_NOR_Private_Constants Private Constants
  * @{
  */
#define OSPI_NOR_OCTAL_DISABLE      0x0
#define OSPI_NOR_OCTAL_ENABLE       0x1

#define DUMMY_CYCLES_READ_OCTAL     MX25LM51245G_DUMMY_CYCLES_READ_OCTAL_66M
#define DUMMY_CYCLES_CR_CFG         MX25LM51245G_CR2_DC_66M
/**
  * @}
  */




/* Private variables ---------------------------------------------------------*/

/** @defgroup STM32L4P5G_DISCOVERY_OSPI_NOR_Private_Variables Private Variables
  * @{
  */
OSPI_HandleTypeDef OSPINORHandle;



/**
  * @}
  */


/* Private functions ---------------------------------------------------------*/

/** @defgroup STM32L4P5G_DISCOVERY_OSPI_NOR_Private_Functions Private Functions
  * @{
  */
static void    OSPI_NOR_MspInit            (void);
static void    OSPI_NOR_MspDeInit          (void);
static uint8_t OSPI_NOR_ResetMemory        (OSPI_HandleTypeDef *hospi);
static uint8_t OSPI_NOR_WriteEnable        (OSPI_HandleTypeDef *hospi);
static uint8_t OSPI_NOR_AutoPollingMemReady(OSPI_HandleTypeDef *hospi, uint32_t Timeout);
static uint8_t OSPI_NOR_OctalMode          (OSPI_HandleTypeDef *hospi, uint8_t Operation);
/**
  * @}
  */

/* Exported functions ---------------------------------------------------------*/

/** @addtogroup STM32L4P5G_DISCOVERY_OSPI_NOR_Exported_Functions
  * @{
  */

/**
  * @brief  Initializes the OSPI interface.
  * @retval OSPI memory status
  */
uint8_t BSP_OSPI_NOR_Init(void)
{
  OSPIM_CfgTypeDef OSPIM_Cfg_Struct;

  OSPINORHandle.Instance = OCTOSPI2;

  /* Call the DeInit function to reset the driver */
  if (HAL_OSPI_DeInit(&OSPINORHandle) != HAL_OK)
  {
    return OSPI_NOR_ERROR;
  }

  /* System level initialization */
  OSPI_NOR_MspInit();

  /* OctoSPI initialization */
  OSPINORHandle.Init.FifoThreshold         = 4;
  OSPINORHandle.Init.DualQuad              = HAL_OSPI_DUALQUAD_DISABLE;
  OSPINORHandle.Init.MemoryType            = HAL_OSPI_MEMTYPE_MACRONIX;
  OSPINORHandle.Init.DeviceSize            = OSPI_FLASH_SIZE;
  OSPINORHandle.Init.ChipSelectHighTime    = 2;
  OSPINORHandle.Init.FreeRunningClock      = HAL_OSPI_FREERUNCLK_DISABLE;
  OSPINORHandle.Init.ClockMode             = HAL_OSPI_CLOCK_MODE_0;
  OSPINORHandle.Init.ClockPrescaler        = 2; /* OctoSPI clock = 110MHz / ClockPrescaler = 55MHz */
  OSPINORHandle.Init.SampleShifting        = HAL_OSPI_SAMPLE_SHIFTING_NONE;
  OSPINORHandle.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_ENABLE;
  OSPINORHandle.Init.ChipSelectBoundary    = 0;
  OSPINORHandle.Init.DelayBlockBypass      = HAL_OSPI_DELAY_BLOCK_USED;
  OSPINORHandle.Init.MaxTran               = 0;

  if (HAL_OSPI_Init(&OSPINORHandle) != HAL_OK)
  {
    return OSPI_NOR_ERROR;
  }

  /* OctoSPI IO Manager configuration */
  OSPIM_Cfg_Struct.ClkPort     = 2;
  OSPIM_Cfg_Struct.DQSPort     = 2;
  OSPIM_Cfg_Struct.IOHighPort  = HAL_OSPIM_IOPORT_2_HIGH;
  OSPIM_Cfg_Struct.IOLowPort   = HAL_OSPIM_IOPORT_2_LOW;
  OSPIM_Cfg_Struct.NCSPort     = 2;
  OSPIM_Cfg_Struct.Req2AckTime = 1;

  if (HAL_OSPIM_Config(&OSPINORHandle, &OSPIM_Cfg_Struct, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return OSPI_NOR_ERROR;
  }

   /* Reset Memory ------------------------------------- */
   if (OSPI_NOR_ResetMemory(&OSPINORHandle) != HAL_OK)
   {
     return OSPI_NOR_ERROR;
   }

   /* Insert delay */
   HAL_Delay(1);

   /* Configure the memory in octal mode ------------------------------------- */
  if (OSPI_NOR_OctalMode(&OSPINORHandle, OSPI_NOR_OCTAL_ENABLE)!= HAL_OK)
  {
     return OSPI_NOR_ERROR;
  }



  return OSPI_NOR_OK;
}

/**
  * @brief  De-Initializes the OSPI interface.
  * @retval OSPI memory status
  */
uint8_t BSP_OSPI_NOR_DeInit(void)
{
  OSPINORHandle.Instance = OCTOSPI2;

  /* Abort any activity on OctoSPI (eg Memory-mapped mode) */
  if (HAL_OSPI_GetState(&OSPINORHandle) != HAL_OSPI_STATE_READY)
  {
    if (HAL_OSPI_Abort(&OSPINORHandle) != HAL_OK)
    {
      return OSPI_NOR_ERROR;
    }
  }

  /* OctoSPI octal mode disable */
  if (OSPI_NOR_OctalMode(&OSPINORHandle, OSPI_NOR_OCTAL_DISABLE) != OSPI_NOR_OK)
  {
    return OSPI_NOR_ERROR;
  }

  /* Call the DeInit function to reset the driver */
  if (HAL_OSPI_DeInit(&OSPINORHandle) != HAL_OK)
  {
    return OSPI_NOR_ERROR;
  }

  /* System level De-initialization */
  OSPI_NOR_MspDeInit();

  return OSPI_NOR_OK;
}

/**
  * @brief  Reads an amount of data from the OSPI memory.
  * @param  pData: Pointer to data to be read
  * @param  ReadAddr: Read start address
  * @param  Size: Size of data to read
  * @retval OSPI memory status
  */
uint8_t BSP_OSPI_NOR_Read(uint8_t* pData, uint32_t ReadAddr, uint32_t Size)
{
  OSPI_RegularCmdTypeDef sCommand;

  sCommand.OperationType         = HAL_OSPI_OPTYPE_COMMON_CFG;

  sCommand.FlashId               = HAL_OSPI_FLASH_ID_1;
  sCommand.InstructionMode       = HAL_OSPI_INSTRUCTION_8_LINES;
  sCommand.InstructionSize       = HAL_OSPI_INSTRUCTION_16_BITS;
  sCommand.InstructionDtrMode    = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  sCommand.AddressDtrMode        = HAL_OSPI_ADDRESS_DTR_DISABLE;
  sCommand.AlternateBytes        = 0x00;
  sCommand.AlternateBytesMode    = HAL_OSPI_ALTERNATE_BYTES_NONE;
  sCommand.AlternateBytesSize    = HAL_OSPI_ALTERNATE_BYTES_8_BITS;
  sCommand.AlternateBytesDtrMode = HAL_OSPI_ALTERNATE_BYTES_DTR_DISABLE;
  sCommand.DataDtrMode           = HAL_OSPI_DATA_DTR_DISABLE;
  sCommand.DummyCycles           = DUMMY_CLOCK_CYCLES_READ;
  sCommand.DQSMode               = HAL_OSPI_DQS_DISABLE;
  sCommand.SIOOMode              = HAL_OSPI_SIOO_INST_EVERY_CMD;

  sCommand.Instruction = OCTAL_IO_READ_CMD;
  sCommand.AddressMode = HAL_OSPI_ADDRESS_8_LINES;
  sCommand.AddressSize = HAL_OSPI_ADDRESS_32_BITS;

  sCommand.DataMode    = HAL_OSPI_DATA_8_LINES;
  sCommand.Address     = ReadAddr;
  sCommand.NbData      = Size;

  if (HAL_OSPI_Command(&OSPINORHandle, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return OSPI_NOR_ERROR;
  }

  if (HAL_OSPI_Receive(&OSPINORHandle, pData, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return OSPI_NOR_ERROR;
  }

  return OSPI_NOR_OK;
}

/**
  * @brief  Writes an amount of data to the OSPI memory.
  * @param  pData: Pointer to data to be written
  * @param  WriteAddr: Write start address
  * @param  Size: Size of data to write
  * @retval OSPI memory status
  */
uint8_t BSP_OSPI_NOR_Write(uint8_t* pData, uint32_t WriteAddr, uint32_t Size)
{
  OSPI_RegularCmdTypeDef sCommand;
  uint32_t end_addr, current_size, current_addr;

  /* Calculation of the size between the write address and the end of the page */
  current_size = MX25LM51245G_PAGE_SIZE - (WriteAddr % MX25LM51245G_PAGE_SIZE);

  /* Check if the size of the data is less than the remaining place in the page */
  if (current_size > Size)
  {
    current_size = Size;
  }

  /* Initialize the address variables */
  current_addr = WriteAddr;
  end_addr = WriteAddr + Size;

  sCommand.OperationType         = HAL_OSPI_OPTYPE_COMMON_CFG;

  sCommand.FlashId               = HAL_OSPI_FLASH_ID_1;
  sCommand.Instruction           = OCTAL_PAGE_PROG_CMD;
  sCommand.InstructionMode       = HAL_OSPI_INSTRUCTION_8_LINES;
  sCommand.InstructionSize       = HAL_OSPI_INSTRUCTION_16_BITS;
  sCommand.InstructionDtrMode    = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  sCommand.AddressDtrMode        = HAL_OSPI_ADDRESS_DTR_DISABLE;
  sCommand.AlternateBytes        = 0x00;
  sCommand.AlternateBytesMode    = HAL_OSPI_ALTERNATE_BYTES_NONE;
  sCommand.AlternateBytesSize    = HAL_OSPI_ALTERNATE_BYTES_8_BITS;
  sCommand.AlternateBytesDtrMode = HAL_OSPI_ALTERNATE_BYTES_DTR_DISABLE;
  sCommand.DataDtrMode           = HAL_OSPI_DATA_DTR_DISABLE;
  sCommand.DummyCycles           = 0;
  sCommand.DQSMode               = HAL_OSPI_DQS_DISABLE;
  sCommand.SIOOMode              = HAL_OSPI_SIOO_INST_EVERY_CMD;

  sCommand.Instruction = OCTAL_PAGE_PROG_CMD;
  sCommand.AddressMode = HAL_OSPI_ADDRESS_8_LINES;
  sCommand.AddressSize = HAL_OSPI_ADDRESS_32_BITS;

  sCommand.DataMode    = HAL_OSPI_DATA_8_LINES;


  /* Perform the write page by page */
  do
  {
    sCommand.Address = current_addr;
    sCommand.NbData  = current_size;

    /* Enable write operations */
    if (OSPI_NOR_WriteEnable(&OSPINORHandle) != OSPI_NOR_OK)
    {
      return OSPI_NOR_ERROR;
    }

    /* Configure the command */
    if (HAL_OSPI_Command(&OSPINORHandle, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
      return OSPI_NOR_ERROR;
    }

    /* Transmission of the data */
    if (HAL_OSPI_Transmit(&OSPINORHandle, pData, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
      return OSPI_NOR_ERROR;
    }

    /* Configure automatic polling mode to wait for end of program */
    if (OSPI_NOR_AutoPollingMemReady(&OSPINORHandle, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != OSPI_NOR_OK)
    {
      return OSPI_NOR_ERROR;
    }

    /* Update the address and size variables for next page programming */
    current_addr += current_size;
    pData += current_size;
    current_size = ((current_addr + MX25LM51245G_PAGE_SIZE) > end_addr) ? (end_addr - current_addr) : MX25LM51245G_PAGE_SIZE;
  } while (current_addr < end_addr);


  return OSPI_NOR_OK;
}

/**
  * @brief  Erases the specified block of the OSPI memory.
  * @param  BlockAddress: Block address to erase
  * @retval OSPI memory status
  */
uint8_t BSP_OSPI_NOR_Erase_Block(uint32_t BlockAddress)
{
  OSPI_RegularCmdTypeDef sCommand;

  sCommand.OperationType         = HAL_OSPI_OPTYPE_COMMON_CFG;
  sCommand.FlashId               = HAL_OSPI_FLASH_ID_1;
  sCommand.InstructionMode       = HAL_OSPI_INSTRUCTION_8_LINES;
  sCommand.InstructionSize       = HAL_OSPI_INSTRUCTION_16_BITS;
  sCommand.InstructionDtrMode    = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  sCommand.AddressDtrMode        = HAL_OSPI_ADDRESS_DTR_DISABLE;
  sCommand.AlternateBytes        = 0x00;
  sCommand.AlternateBytesMode    = HAL_OSPI_ALTERNATE_BYTES_NONE;
  sCommand.AlternateBytesSize    = HAL_OSPI_ALTERNATE_BYTES_8_BITS;
  sCommand.AlternateBytesDtrMode = HAL_OSPI_ALTERNATE_BYTES_DTR_DISABLE;
  sCommand.DataDtrMode           = HAL_OSPI_DATA_DTR_DISABLE;
  sCommand.DummyCycles           = 0;
  sCommand.DQSMode               = HAL_OSPI_DQS_DISABLE;
  sCommand.SIOOMode              = HAL_OSPI_SIOO_INST_EVERY_CMD;

  sCommand.InstructionDtrMode    = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  sCommand.AddressDtrMode        = HAL_OSPI_ADDRESS_DTR_DISABLE;



  sCommand.Instruction = OCTAL_BLOCK_ERASE_CMD;

  sCommand.AddressMode = HAL_OSPI_ADDRESS_8_LINES;
  sCommand.AddressSize = HAL_OSPI_ADDRESS_32_BITS;
  sCommand.Address     = BlockAddress;
  sCommand.DataMode    = HAL_OSPI_DATA_NONE;

 /* Enable write operations */
  if (OSPI_NOR_WriteEnable(&OSPINORHandle) != OSPI_NOR_OK)
  {
    return OSPI_NOR_ERROR;
  }

  /* Send the command */
  if (HAL_OSPI_Command(&OSPINORHandle, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return OSPI_NOR_ERROR;
  }

  /* Configure automatic polling mode to wait for end of erase */
  if (OSPI_NOR_AutoPollingMemReady(&OSPINORHandle, MX25LM51245G_BLOCK_ERASE_MAX_TIME) != OSPI_NOR_OK)
  {
    return OSPI_NOR_ERROR;
  }

  return OSPI_NOR_OK;
}

/**
  * @brief  Erases the specified sector of the OSPI memory.
  * @param  Sector: Sector address to erase (0 to 255)
  * @retval OSPI memory status
  * @note This function is non blocking meaning that sector erase
  *       operation is started but not completed when the function
  *       returns. Application has to call BSP_QSPI_GetStatus()
  *       to know when the device is available again (i.e. erase operation
  *       completed).
  */
uint8_t BSP_OSPI_NOR_Erase_Sector(uint32_t Sector)
{
  OSPI_RegularCmdTypeDef sCommand;

  sCommand.OperationType         = HAL_OSPI_OPTYPE_COMMON_CFG;
  sCommand.FlashId               = HAL_OSPI_FLASH_ID_1;
  sCommand.InstructionMode       = HAL_OSPI_INSTRUCTION_8_LINES;
  sCommand.InstructionSize       = HAL_OSPI_INSTRUCTION_16_BITS;
  sCommand.InstructionDtrMode    = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  sCommand.AddressDtrMode        = HAL_OSPI_ADDRESS_DTR_DISABLE;
  sCommand.AlternateBytes        = 0x00;
  sCommand.AlternateBytesMode    = HAL_OSPI_ALTERNATE_BYTES_NONE;
  sCommand.AlternateBytesSize    = HAL_OSPI_ALTERNATE_BYTES_8_BITS;
  sCommand.AlternateBytesDtrMode = HAL_OSPI_ALTERNATE_BYTES_DTR_DISABLE;
  sCommand.DataDtrMode           = HAL_OSPI_DATA_DTR_DISABLE;
  sCommand.DummyCycles           = 0;
  sCommand.DQSMode               = HAL_OSPI_DQS_DISABLE;
  sCommand.SIOOMode              = HAL_OSPI_SIOO_INST_EVERY_CMD;

  sCommand.InstructionDtrMode    = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  sCommand.AddressDtrMode        = HAL_OSPI_ADDRESS_DTR_DISABLE;


  if (Sector >= (uint32_t)(MX25LM51245G_FLASH_SIZE/MX25LM51245G_SECTOR_SIZE))
  {
    return OSPI_NOR_ERROR;
  }

  sCommand.Instruction = OCTAL_SECTOR_ERASE_CMD;

  sCommand.AddressMode = HAL_OSPI_ADDRESS_8_LINES;
  sCommand.AddressSize = HAL_OSPI_ADDRESS_32_BITS;
  sCommand.Address     = (Sector * MX25LM51245G_SECTOR_SIZE);
  sCommand.DataMode    = HAL_OSPI_DATA_NONE;

 /* Enable write operations */
  if (OSPI_NOR_WriteEnable(&OSPINORHandle) != OSPI_NOR_OK)
  {
    return OSPI_NOR_ERROR;
  }

  /* Send the command */
  if (HAL_OSPI_Command(&OSPINORHandle, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return OSPI_NOR_ERROR;
  }

  if (OSPI_NOR_AutoPollingMemReady(&OSPINORHandle, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != OSPI_NOR_OK)
  {
    return OSPI_NOR_ERROR;
  }

  return OSPI_NOR_OK;
}

/**
  * @brief  Erases the entire OSPI memory.
  * @retval OSPI memory status
  */
uint8_t BSP_OSPI_NOR_Erase_Chip(void)
{
  OSPI_RegularCmdTypeDef sCommand;


  sCommand.OperationType         = HAL_OSPI_OPTYPE_COMMON_CFG;
  sCommand.FlashId               = HAL_OSPI_FLASH_ID_1;
  sCommand.InstructionMode       = HAL_OSPI_INSTRUCTION_8_LINES;
  sCommand.InstructionSize       = HAL_OSPI_INSTRUCTION_16_BITS;
  sCommand.InstructionDtrMode    = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  sCommand.AddressDtrMode        = HAL_OSPI_ADDRESS_DTR_DISABLE;
  sCommand.AlternateBytes        = 0x00;
  sCommand.AlternateBytesMode    = HAL_OSPI_ALTERNATE_BYTES_NONE;
  sCommand.AlternateBytesSize    = HAL_OSPI_ALTERNATE_BYTES_8_BITS;
  sCommand.AlternateBytesDtrMode = HAL_OSPI_ALTERNATE_BYTES_DTR_DISABLE;
  sCommand.DataDtrMode           = HAL_OSPI_DATA_DTR_DISABLE;
  sCommand.DummyCycles           = 0;
  sCommand.DQSMode               = HAL_OSPI_DQS_DISABLE;
  sCommand.SIOOMode              = HAL_OSPI_SIOO_INST_EVERY_CMD;

  sCommand.Instruction = OCTAL_CHIP_ERASE_CMD;

   sCommand.AddressMode = HAL_OSPI_ADDRESS_NONE ;
   sCommand.AddressSize = HAL_OSPI_ADDRESS_24_BITS;
   sCommand.Address     = 0;
   sCommand.DataMode    = HAL_OSPI_DATA_NONE;


  /* Enable write operations */
  if (OSPI_NOR_WriteEnable(&OSPINORHandle) != OSPI_NOR_OK)
  {
    return OSPI_NOR_ERROR;
  }

  /* Send the command */
  if (HAL_OSPI_Command(&OSPINORHandle, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return OSPI_NOR_ERROR;
  }

  /* Configure automatic polling mode to wait for end of erase */
  if (OSPI_NOR_AutoPollingMemReady(&OSPINORHandle, MX25LM51245G_CHIP_ERASE_MAX_TIME) != OSPI_NOR_OK)
  {
    return OSPI_NOR_ERROR;
  }

  return OSPI_NOR_OK;
}

/**
  * @brief  Reads current status of the OSPI memory.
  * @retval OSPI memory status
  */
uint8_t BSP_OSPI_NOR_GetStatus(void)
{
  OSPI_RegularCmdTypeDef sCommand;
  uint8_t reg[2];

  /* Initialize the read security register command */
  sCommand.OperationType         = HAL_OSPI_OPTYPE_COMMON_CFG;
  sCommand.FlashId               = HAL_OSPI_FLASH_ID_1;
  sCommand.Instruction           = OCTAL_READ_SECURITY_REG_CMD;
  sCommand.InstructionMode       = HAL_OSPI_INSTRUCTION_8_LINES;
  sCommand.InstructionSize       = HAL_OSPI_INSTRUCTION_16_BITS;
  sCommand.Address               = 0;
  sCommand.AddressMode           = HAL_OSPI_ADDRESS_8_LINES;
  sCommand.AddressSize           = HAL_OSPI_ADDRESS_32_BITS;
  sCommand.AlternateBytesMode    = HAL_OSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode              = HAL_OSPI_DATA_8_LINES;
  sCommand.NbData                = 2;
  sCommand.DummyCycles           = DUMMY_CYCLES_READ_OCTAL;
  sCommand.SIOOMode              = HAL_OSPI_SIOO_INST_EVERY_CMD;


  sCommand.InstructionDtrMode    = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  sCommand.AddressDtrMode        = HAL_OSPI_ADDRESS_DTR_DISABLE;
  sCommand.DataDtrMode           = HAL_OSPI_DATA_DTR_DISABLE;
  sCommand.DQSMode               = HAL_OSPI_DQS_DISABLE;


  /* Configure the command */
  if (HAL_OSPI_Command(&OSPINORHandle, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return OSPI_NOR_ERROR;
  }

  /* Reception of the data */
  if (HAL_OSPI_Receive(&OSPINORHandle, reg, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return OSPI_NOR_ERROR;
  }

  /* Check the value of the register */
  if ((reg[0] & (MX25LM51245G_SECR_P_FAIL | MX25LM51245G_SECR_E_FAIL)) != 0)
  {
    return OSPI_NOR_ERROR;
  }
  else if ((reg[0] & (MX25LM51245G_SECR_PSB | MX25LM51245G_SECR_ESB)) != 0)
  {
    return OSPI_NOR_SUSPENDED;
  }

  /* Initialize the read status register command */
  sCommand.Instruction = OCTAL_READ_STATUS_REG_CMD;

  /* Configure the command */
  if (HAL_OSPI_Command(&OSPINORHandle, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return OSPI_NOR_ERROR;
  }

  /* Reception of the data */
  if (HAL_OSPI_Receive(&OSPINORHandle, reg, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return OSPI_NOR_ERROR;
  }

  /* Check the value of the register */
  if ((reg[0] & MX25LM51245G_SR_WIP) != 0)
  {
    return OSPI_NOR_BUSY;
  }
  else
  {
    return OSPI_NOR_OK;
  }
}

/**
  * @brief  Return the configuration of the OSPI memory.
  * @param  pInfo: pointer on the configuration structure
  * @retval OSPI memory status
  */
uint8_t BSP_OSPI_NOR_GetInfo(OSPI_NOR_Info* pInfo)
{
  /* Configure the structure with the memory configuration */
  pInfo->FlashSize          = MX25LM51245G_FLASH_SIZE;
  pInfo->EraseSectorSize    = MX25LM51245G_BLOCK_SIZE;
  pInfo->EraseSectorsNumber = (MX25LM51245G_FLASH_SIZE/MX25LM51245G_BLOCK_SIZE);
  pInfo->ProgPageSize       = MX25LM51245G_PAGE_SIZE;
  pInfo->ProgPagesNumber    = (MX25LM51245G_FLASH_SIZE/MX25LM51245G_PAGE_SIZE);

  return OSPI_NOR_OK;
}

/**
  * @brief  Configure the OSPI in memory-mapped mode
  * @retval OSPI memory status
  */
uint8_t BSP_OSPI_NOR_EnableMemoryMappedMode(void)
{
  OSPI_RegularCmdTypeDef   sCommand;
  OSPI_MemoryMappedTypeDef sMemMappedCfg;

  /* Enable write operations */
  if (OSPI_NOR_WriteEnable(&OSPINORHandle) != OSPI_NOR_OK)
  {
    return OSPI_NOR_ERROR;
  }

  sCommand.OperationType         = HAL_OSPI_OPTYPE_WRITE_CFG;
  sCommand.FlashId               = HAL_OSPI_FLASH_ID_1;
  sCommand.InstructionMode       = HAL_OSPI_INSTRUCTION_8_LINES;
  sCommand.InstructionSize       = HAL_OSPI_INSTRUCTION_16_BITS;
  sCommand.InstructionDtrMode    = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  sCommand.AddressDtrMode        = HAL_OSPI_ADDRESS_DTR_DISABLE;
  sCommand.AlternateBytes        = 0x00;
  sCommand.AlternateBytesMode    = HAL_OSPI_ALTERNATE_BYTES_NONE;
  sCommand.AlternateBytesSize    = HAL_OSPI_ALTERNATE_BYTES_8_BITS;
  sCommand.AlternateBytesDtrMode = HAL_OSPI_ALTERNATE_BYTES_DTR_DISABLE;
  sCommand.DataDtrMode           = HAL_OSPI_DATA_DTR_DISABLE;
  sCommand.DummyCycles           = 0;
  sCommand.DQSMode               = HAL_OSPI_DQS_DISABLE;
  sCommand.SIOOMode              = HAL_OSPI_SIOO_INST_EVERY_CMD;

  sCommand.InstructionDtrMode    = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  sCommand.AddressDtrMode        = HAL_OSPI_ADDRESS_DTR_DISABLE;



  sCommand.Instruction = OCTAL_PAGE_PROG_CMD;

  sCommand.AddressMode = HAL_OSPI_ADDRESS_8_LINES;
  sCommand.AddressSize = HAL_OSPI_ADDRESS_32_BITS;
  sCommand.Address     = 0;
  sCommand.DataMode    = HAL_OSPI_DATA_8_LINES;
  sCommand.NbData      = 1;
  sCommand.DQSMode     = HAL_OSPI_DQS_ENABLE;

  if (HAL_OSPI_Command(&OSPINORHandle, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return OSPI_NOR_ERROR;
  }


  sCommand.Instruction           = OCTAL_IO_READ_CMD;
  sCommand.OperationType         = HAL_OSPI_OPTYPE_READ_CFG;
  sCommand.DummyCycles           = DUMMY_CLOCK_CYCLES_READ;
  sCommand.DQSMode               = HAL_OSPI_DQS_DISABLE;



  if (HAL_OSPI_Command(&OSPINORHandle, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return OSPI_NOR_ERROR;
  }

   /* Configure the memory mapped mode */
  sMemMappedCfg.TimeOutActivation = HAL_OSPI_TIMEOUT_COUNTER_ENABLE;
  sMemMappedCfg.TimeOutPeriod     = 0x20;

  if (HAL_OSPI_MemoryMapped(&OSPINORHandle, &sMemMappedCfg) != HAL_OK)
  {
    return OSPI_NOR_ERROR;
  }

  return OSPI_NOR_OK;
}

/**
  * @brief  This function suspends an ongoing erase command.
  * @retval OSPI memory status
  */
uint8_t BSP_OSPI_NOR_SuspendErase(void)
{
  OSPI_RegularCmdTypeDef sCommand;

  /* Check whether the device is busy (erase operation is
  in progress).
  */
  if (BSP_OSPI_NOR_GetStatus() == OSPI_NOR_BUSY)
  {
    /* Initialize the suspend command */
    sCommand.OperationType         = HAL_OSPI_OPTYPE_COMMON_CFG;
    sCommand.FlashId               = HAL_OSPI_FLASH_ID_1;
    sCommand.Instruction           = OCTAL_PROG_ERASE_SUSPEND_CMD;
    sCommand.InstructionMode       = HAL_OSPI_INSTRUCTION_8_LINES;
    sCommand.InstructionSize       = HAL_OSPI_INSTRUCTION_16_BITS;
    sCommand.AddressMode           = HAL_OSPI_ADDRESS_NONE;
    sCommand.AlternateBytesMode    = HAL_OSPI_ALTERNATE_BYTES_NONE;
    sCommand.DataMode              = HAL_OSPI_DATA_NONE;
    sCommand.DummyCycles           = 0;
    sCommand.DQSMode               = HAL_OSPI_DQS_DISABLE;
    sCommand.SIOOMode              = HAL_OSPI_SIOO_INST_EVERY_CMD;

    sCommand.InstructionDtrMode    = HAL_OSPI_INSTRUCTION_DTR_DISABLE;


  /* Send the command */
    if (HAL_OSPI_Command(&OSPINORHandle, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
      return OSPI_NOR_ERROR;
    }

    if (BSP_OSPI_NOR_GetStatus() == OSPI_NOR_SUSPENDED)
    {
      return OSPI_NOR_OK;
    }

    return OSPI_NOR_ERROR;
  }

  return OSPI_NOR_OK;
}

/**
  * @brief  This function resumes a paused erase command.
  * @retval OSPI memory status
  */
uint8_t BSP_OSPI_NOR_ResumeErase(void)
{
  OSPI_RegularCmdTypeDef sCommand;

  /* Check whether the device is in suspended state */
  if (BSP_OSPI_NOR_GetStatus() == OSPI_NOR_SUSPENDED)
  {
    /* Initialize the resume command */
    sCommand.OperationType         = HAL_OSPI_OPTYPE_COMMON_CFG;
    sCommand.FlashId               = HAL_OSPI_FLASH_ID_1;
    sCommand.Instruction           = OCTAL_PROG_ERASE_RESUME_CMD;
    sCommand.InstructionMode       = HAL_OSPI_INSTRUCTION_8_LINES;
    sCommand.InstructionSize       = HAL_OSPI_INSTRUCTION_16_BITS;
    sCommand.AddressMode           = HAL_OSPI_ADDRESS_NONE;
    sCommand.AlternateBytesMode    = HAL_OSPI_ALTERNATE_BYTES_NONE;
    sCommand.DataMode              = HAL_OSPI_DATA_NONE;
    sCommand.DummyCycles           = 0;
    sCommand.DQSMode               = HAL_OSPI_DQS_DISABLE;
    sCommand.SIOOMode              = HAL_OSPI_SIOO_INST_EVERY_CMD;


    sCommand.InstructionDtrMode    = HAL_OSPI_INSTRUCTION_DTR_DISABLE;


    /* Send the command */
    if (HAL_OSPI_Command(&OSPINORHandle, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
      return OSPI_NOR_ERROR;
    }

    /*
    When this command is executed, the status register write in progress bit is set to 1, and
    the flag status register program erase controller bit is set to 0. This command is ignored
    if the device is not in a suspended state.
    */

    if (BSP_OSPI_NOR_GetStatus() == OSPI_NOR_BUSY)
    {
      return OSPI_NOR_OK;
    }

    return OSPI_NOR_ERROR;
  }

  return OSPI_NOR_OK;
}

/**
  * @brief  This function enter the OSPI memory in deep power down mode.
  * @retval OSPI memory status
  */
uint8_t BSP_OSPI_NOR_EnterDeepPowerDown(void)
{
  OSPI_RegularCmdTypeDef sCommand;

  sCommand.OperationType         = HAL_OSPI_OPTYPE_COMMON_CFG;
  sCommand.FlashId               = HAL_OSPI_FLASH_ID_1;
  sCommand.InstructionMode       = HAL_OSPI_INSTRUCTION_8_LINES;
  sCommand.InstructionSize       = HAL_OSPI_INSTRUCTION_16_BITS;
  sCommand.InstructionDtrMode    = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  sCommand.AddressDtrMode        = HAL_OSPI_ADDRESS_DTR_DISABLE;
  sCommand.AlternateBytes        = 0x00;
  sCommand.AlternateBytesMode    = HAL_OSPI_ALTERNATE_BYTES_NONE;
  sCommand.AlternateBytesSize    = HAL_OSPI_ALTERNATE_BYTES_8_BITS;
  sCommand.AlternateBytesDtrMode = HAL_OSPI_ALTERNATE_BYTES_DTR_DISABLE;
  sCommand.DataDtrMode           = HAL_OSPI_DATA_DTR_DISABLE;
  sCommand.DummyCycles           = 0;
  sCommand.DQSMode               = HAL_OSPI_DQS_DISABLE;
  sCommand.SIOOMode              = HAL_OSPI_SIOO_INST_EVERY_CMD;


  /* Enable write operations ------------------------------------------ */
  sCommand.Instruction = OCTAL_ENTER_DEEP_POWER_DOWN_CMD;
  sCommand.AddressMode = HAL_OSPI_ADDRESS_NONE;
  sCommand.DataMode    = HAL_OSPI_DATA_NONE;



  /* Send the command */
  if (HAL_OSPI_Command(&OSPINORHandle, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return OSPI_NOR_ERROR;
  }

  /* ---          Memory takes 10us max to enter deep power down          --- */

  return OSPI_NOR_OK;
}

/**
  * @brief  This function leave the OSPI memory from deep power down mode.
  * @retval OSPI memory status
  */
uint8_t BSP_OSPI_NOR_LeaveDeepPowerDown(void)
{
  OSPI_RegularCmdTypeDef sCommand;

  sCommand.OperationType         = HAL_OSPI_OPTYPE_COMMON_CFG;
  sCommand.FlashId               = HAL_OSPI_FLASH_ID_1;
  sCommand.InstructionMode       = HAL_OSPI_INSTRUCTION_8_LINES;
  sCommand.InstructionSize       = HAL_OSPI_INSTRUCTION_16_BITS;
  sCommand.InstructionDtrMode    = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  sCommand.AddressDtrMode        = HAL_OSPI_ADDRESS_DTR_DISABLE;
  sCommand.AlternateBytes        = 0x00;
  sCommand.AlternateBytesMode    = HAL_OSPI_ALTERNATE_BYTES_NONE;
  sCommand.AlternateBytesSize    = HAL_OSPI_ALTERNATE_BYTES_8_BITS;
  sCommand.AlternateBytesDtrMode = HAL_OSPI_ALTERNATE_BYTES_DTR_DISABLE;
  sCommand.DataDtrMode           = HAL_OSPI_DATA_DTR_DISABLE;
  sCommand.DummyCycles           = 0;
  sCommand.DQSMode               = HAL_OSPI_DQS_DISABLE;
  sCommand.SIOOMode              = HAL_OSPI_SIOO_INST_EVERY_CMD;

            /* Enable write operations ------------------------------------------ */
  sCommand.Instruction = OCTAL_NO_OPERATION_CMD;
  sCommand.AddressMode = HAL_OSPI_ADDRESS_NONE;
  sCommand.DataMode    = HAL_OSPI_DATA_NONE;


  /* Send the command */
  if (HAL_OSPI_Command(&OSPINORHandle, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return OSPI_NOR_ERROR;
  }

  /* --- A NOP command is sent to the memory, as the nCS should be low for at least 20 ns --- */
  /* ---                  Memory takes 30us min to leave deep power down                  --- */

  return OSPI_NOR_OK;
}
/**
  * @}
  */

/** @addtogroup STM32L4P5G_DISCOVERY_OSPI_NOR_Private_Functions
  * @{
  */

/**
  * @brief  Initializes the OSPI MSP.
  * @retval None
  */
static void OSPI_NOR_MspInit(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* Enable the OctoSPI memory interface clock */
  __HAL_RCC_OSPIM_CLK_ENABLE();
  __HAL_RCC_OSPI2_CLK_ENABLE();

  /* Reset the OctoSPI memory interface */
  __HAL_RCC_OSPI2_FORCE_RESET();
  __HAL_RCC_OSPI2_RELEASE_RESET();

  /* Enable GPIO clocks */
  /* IOSV bit MUST be set to access GPIO port G[2:15] */
  __HAL_RCC_PWR_CLK_ENABLE();
  SET_BIT(PWR->CR2, PWR_CR2_IOSV);

  /* Enable DQS, CLK, NCLK, D0, D1, D2, D3 clock */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  /* Enable NCS, D4, D5, D6, D7 clock */
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /* OctoSPI CS GPIO pin configuration  */
  GPIO_InitStruct.Pin       = GPIO_PIN_12;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_OCTOSPIM_P2;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* OctoSPI DQS GPIO pins configuration  */
  GPIO_InitStruct.Pin       = GPIO_PIN_12;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /* OctoSPI CLK, NLK, D0, D1, D2, D3 GPIO pins configuration  */
  GPIO_InitStruct.Pin       = (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /* OctoSPI D4, D5, D6, D7 GPIO pins configuration  */
  GPIO_InitStruct.Pin       = (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_9 | GPIO_PIN_10);
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
}

/**
  * @brief  De-Initializes the OSPI MSP.
  * @retval None
  */
static void OSPI_NOR_MspDeInit(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* OctoSPI CLK, D0-D3 DQS GPIO pins de-configuration  */
  HAL_GPIO_DeInit(GPIOF, (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_12));
  /* OctoSPI CS, D4-D7 GPIO pins de-configuration  */
  HAL_GPIO_DeInit(GPIOG, (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_12));

  /* Set CS in pull up mode (optimum default setting) */
  GPIO_InitStruct.Mode      = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pin       = GPIO_PIN_12;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* Set DQS in no pull, low state (optimum default setting) */
  GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Pin       = GPIO_PIN_12;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_RESET);

  /* Reset the OctoSPI memory interface */
  __HAL_RCC_OSPI2_FORCE_RESET();
  __HAL_RCC_OSPI2_RELEASE_RESET();

  /* Disable the OctoSPI memory interface clock */
  __HAL_RCC_OSPI2_CLK_DISABLE();
  __HAL_RCC_OSPIM_CLK_DISABLE();
}

/**
  * @brief  This function reset the OSPI memory.
  * @param  hospi: OSPI handle
  * @retval None
  */
static uint8_t OSPI_NOR_ResetMemory(OSPI_HandleTypeDef *hospi)
{
  OSPI_RegularCmdTypeDef  sCommand;


  /* Initialize the reset enable command */
  sCommand.OperationType         = HAL_OSPI_OPTYPE_COMMON_CFG;
  sCommand.FlashId               = HAL_OSPI_FLASH_ID_1;
  sCommand.InstructionMode       = HAL_OSPI_INSTRUCTION_1_LINE;
  sCommand.InstructionSize       = HAL_OSPI_INSTRUCTION_8_BITS;
  sCommand.InstructionDtrMode    = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  sCommand.AddressMode           = HAL_OSPI_ADDRESS_NONE;
  sCommand.AlternateBytesMode    = HAL_OSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataDtrMode           = HAL_OSPI_DATA_DTR_DISABLE;
  sCommand.DummyCycles           = 0;
  sCommand.DQSMode               = HAL_OSPI_DQS_DISABLE;
  sCommand.SIOOMode              = HAL_OSPI_SIOO_INST_EVERY_CMD;



  sCommand.DataMode = HAL_OSPI_DATA_NONE;

  sCommand.Instruction = RESET_ENABLE_CMD;
  /* Send the command */
  if (HAL_OSPI_Command(hospi, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return OSPI_NOR_ERROR;
  }


  /* Send the reset memory command */
  sCommand.Instruction = RESET_MEMORY_CMD;
  if (HAL_OSPI_Command(hospi, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
     return OSPI_NOR_ERROR;
  }


  HAL_Delay(40);

   return OSPI_NOR_OK;

}

/**
  * @brief  This function send a Write Enable and wait it is effective.
  * @param  hospi: OSPI handle
  * @retval None
  */
static uint8_t OSPI_NOR_WriteEnable(OSPI_HandleTypeDef *hospi)
{
  OSPI_RegularCmdTypeDef  sCommand;
  uint8_t reg[2];

  /* Enable write operations ------------------------------------------ */
  sCommand.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
  sCommand.FlashId            = HAL_OSPI_FLASH_ID_1;
  sCommand.Instruction        = OCTAL_WRITE_ENABLE_CMD;
  sCommand.InstructionMode    = HAL_OSPI_INSTRUCTION_8_LINES;
  sCommand.InstructionSize    = HAL_OSPI_INSTRUCTION_16_BITS;
  sCommand.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  sCommand.AddressMode        = HAL_OSPI_ADDRESS_NONE;
  sCommand.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode           = HAL_OSPI_DATA_NONE;
  sCommand.DummyCycles        = 0;
  sCommand.DQSMode            = HAL_OSPI_DQS_DISABLE;
  sCommand.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;

  if (HAL_OSPI_Command(hospi, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return OSPI_NOR_ERROR;
  }

  /* Configure automatic polling mode to wait for write enabling ---- */
  sCommand.Instruction    = OCTAL_READ_STATUS_REG_CMD;
  sCommand.Address        = 0x0;
  sCommand.AddressMode    = HAL_OSPI_ADDRESS_8_LINES;
  sCommand.AddressSize    = HAL_OSPI_ADDRESS_32_BITS;
  sCommand.AddressDtrMode = HAL_OSPI_ADDRESS_DTR_DISABLE;
  sCommand.DataMode       = HAL_OSPI_DATA_8_LINES;
  sCommand.DataDtrMode    = HAL_OSPI_DATA_DTR_DISABLE;
  sCommand.NbData         = 1;
  sCommand.DummyCycles    = DUMMY_CLOCK_CYCLES_READ_REG;

  do
  {
    if (HAL_OSPI_Command(hospi, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
      return OSPI_NOR_ERROR;
    }

    if (HAL_OSPI_Receive(hospi, reg, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
      return OSPI_NOR_ERROR;
    }
  } while((reg[0] & WRITE_ENABLE_MASK_VALUE) != WRITE_ENABLE_MATCH_VALUE);

  return OSPI_NOR_OK;
}

/**
  * @brief  This function read the SR of the memory and wait the EOP.
  * @param  hospi: OSPI handle
  * @param  Timeout: Timeout for auto-polling
  * @retval None
  */
static uint8_t OSPI_NOR_AutoPollingMemReady(OSPI_HandleTypeDef *hospi, uint32_t Timeout)
{
  OSPI_RegularCmdTypeDef  sCommand;
  uint8_t reg[2];

  /* Configure automatic polling mode to wait for memory ready ------ */
  sCommand.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
  sCommand.FlashId            = HAL_OSPI_FLASH_ID_1;
  sCommand.Instruction        = OCTAL_READ_STATUS_REG_CMD;
  sCommand.InstructionMode    = HAL_OSPI_INSTRUCTION_8_LINES;
  sCommand.InstructionSize    = HAL_OSPI_INSTRUCTION_16_BITS;
  sCommand.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  sCommand.Address            = 0x0;
  sCommand.AddressMode        = HAL_OSPI_ADDRESS_8_LINES;
  sCommand.AddressSize        = HAL_OSPI_ADDRESS_32_BITS;
  sCommand.AddressDtrMode     = HAL_OSPI_ADDRESS_DTR_DISABLE;
  sCommand.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode           = HAL_OSPI_DATA_8_LINES;
  sCommand.DataDtrMode        = HAL_OSPI_DATA_DTR_DISABLE;
  sCommand.NbData             = 1;
  sCommand.DummyCycles        = DUMMY_CLOCK_CYCLES_READ_REG;
  sCommand.DQSMode            = HAL_OSPI_DQS_DISABLE;
  sCommand.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;

  do
  {
    if (HAL_OSPI_Command(hospi, &sCommand, Timeout) != HAL_OK)
    {
      return OSPI_NOR_ERROR;
    }

    if (HAL_OSPI_Receive(hospi, reg, Timeout) != HAL_OK)
    {
      return OSPI_NOR_ERROR;
    }
  } while((reg[0] & MEMORY_READY_MASK_VALUE) != MEMORY_READY_MATCH_VALUE);


  return OSPI_NOR_OK;
}

/**
  * @brief  This function enables/disables the octal mode of the memory.
  * @param  hospi: OSPI handle
  * @param  Operation: Octal operation mode enable/disable
  * @retval None
  */
static uint8_t OSPI_NOR_OctalMode(OSPI_HandleTypeDef *hospi, uint8_t Operation)
{
  OSPI_RegularCmdTypeDef  sCommand;
  OSPI_AutoPollingTypeDef sConfig;
  uint8_t reg;
  uint8_t reg_array[2];

  sCommand.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
  sCommand.FlashId            = HAL_OSPI_FLASH_ID_1;
  sCommand.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  sCommand.AddressSize        = HAL_OSPI_ADDRESS_32_BITS;
  sCommand.AddressDtrMode     = HAL_OSPI_ADDRESS_DTR_DISABLE;
  sCommand.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataDtrMode        = HAL_OSPI_DATA_DTR_DISABLE;
  sCommand.DQSMode            = HAL_OSPI_DQS_DISABLE;
  sCommand.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;

  sConfig.MatchMode           = HAL_OSPI_MATCH_MODE_AND;
  sConfig.Interval            = AUTO_POLLING_INTERVAL;
  sConfig.AutomaticStop       = HAL_OSPI_AUTOMATIC_STOP_ENABLE;


  /* Activate/deactivate the Octal mode */
  if (Operation == OSPI_NOR_OCTAL_ENABLE)
  {

    /* Enable write operations ---------------------------------------- */
    sCommand.Instruction        = WRITE_ENABLE_CMD;
    sCommand.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;
    sCommand.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
    sCommand.AddressMode        = HAL_OSPI_ADDRESS_NONE;
    sCommand.DataMode           = HAL_OSPI_DATA_NONE;
    sCommand.DummyCycles        = 0;


    if (HAL_OSPI_Command(hospi, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
      return OSPI_NOR_ERROR;
    }

    /* Configure automatic polling mode to wait for write enabling ---- */
    sCommand.Instruction = READ_STATUS_REG_CMD;
    sCommand.DataMode    = HAL_OSPI_DATA_1_LINE;
    sCommand.DataDtrMode = HAL_OSPI_DATA_DTR_DISABLE;
    sCommand.NbData      = 1;

    if (HAL_OSPI_Command(hospi, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
      return OSPI_NOR_ERROR;
    }

    sConfig.Match         = WRITE_ENABLE_MATCH_VALUE;
    sConfig.Mask          = WRITE_ENABLE_MASK_VALUE;


    if (HAL_OSPI_AutoPolling(hospi, &sConfig, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
      return OSPI_NOR_ERROR;
    }

    /* Write Configuration register 2 (with new dummy cycles) --------- */
    sCommand.Instruction    = WRITE_CFG_REG_2_CMD;
    sCommand.Address        = CONFIG_REG2_ADDR3;
    sCommand.AddressMode    = HAL_OSPI_ADDRESS_1_LINE;
    sCommand.AddressSize    = HAL_OSPI_ADDRESS_32_BITS;
    sCommand.AddressDtrMode = HAL_OSPI_ADDRESS_DTR_DISABLE;

    if (HAL_OSPI_Command(hospi, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
      return OSPI_NOR_ERROR;
    }

    reg = CR2_DUMMY_CYCLES_66MHZ;

    if (HAL_OSPI_Transmit(hospi, &reg, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
      return OSPI_NOR_ERROR;
    }

    /* Enable write operations ---------------------------------------- */
    sCommand.Instruction = WRITE_ENABLE_CMD;
    sCommand.AddressMode = HAL_OSPI_ADDRESS_NONE;
    sCommand.DataMode    = HAL_OSPI_DATA_NONE;

    if (HAL_OSPI_Command(hospi, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
      return OSPI_NOR_ERROR;
    }

    /* Configure automatic polling mode to wait for write enabling ---- */
    sCommand.Instruction = READ_STATUS_REG_CMD;
    sCommand.DataMode    = HAL_OSPI_DATA_1_LINE;

    if (HAL_OSPI_Command(hospi, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
      return OSPI_NOR_ERROR;
    }

    sConfig.Match = WRITE_ENABLE_MATCH_VALUE;
    sConfig.Mask  = WRITE_ENABLE_MASK_VALUE;

    if (HAL_OSPI_AutoPolling(hospi, &sConfig, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
      return OSPI_NOR_ERROR;
    }

    /* Write Configuration register 2 (with octal mode) --------------- */
    sCommand.Instruction = WRITE_CFG_REG_2_CMD;
    sCommand.Address     = CONFIG_REG2_ADDR1;
    sCommand.AddressMode = HAL_OSPI_ADDRESS_1_LINE;

    if (HAL_OSPI_Command(hospi, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
      return OSPI_NOR_ERROR;
    }

    reg = CR2_STR_OPI_ENABLE;

    if (HAL_OSPI_Transmit(hospi, &reg, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
      return OSPI_NOR_ERROR;
    }

    /* Wait that the configuration is effective and check that memory is ready */
    HAL_Delay(MEMORY_REG_WRITE_DELAY);

    /* Wait that the memory is ready ---------------------------------- */
    if (OSPI_NOR_AutoPollingMemReady(hospi, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != OSPI_NOR_OK)
    {
      return OSPI_NOR_ERROR;
    }

       /* Check the configuration has been correctly done */
    sCommand.Instruction     = OCTAL_READ_CFG_REG_2_CMD;
    sCommand.InstructionMode = HAL_OSPI_INSTRUCTION_8_LINES;
    sCommand.InstructionSize = HAL_OSPI_INSTRUCTION_16_BITS;
    sCommand.AddressMode     = HAL_OSPI_ADDRESS_8_LINES;
    sCommand.DataMode        = HAL_OSPI_DATA_8_LINES;
    sCommand.DummyCycles     = DUMMY_CYCLES_READ_OCTAL;
    sCommand.NbData          = 2;


    if (HAL_OSPI_Command(&OSPINORHandle, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
      return OSPI_NOR_ERROR;
    }

    if (HAL_OSPI_Receive(&OSPINORHandle, reg_array, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
      return OSPI_NOR_ERROR;
    }


    if (reg_array[0] != MX25LM51245G_CR2_SOPI)
    {
      return OSPI_NOR_ERROR;
    }


  }
  else
  {
    /* Enable write operations */
    if (OSPI_NOR_WriteEnable(&OSPINORHandle) != OSPI_NOR_OK)
    {
      return OSPI_NOR_ERROR;
    }

    /* Write Configuration register 2 (with Octal I/O SPI protocol) */
    sCommand.Instruction     = OCTAL_WRITE_CFG_REG_2_CMD;
    sCommand.InstructionMode = HAL_OSPI_INSTRUCTION_8_LINES;
    sCommand.InstructionSize = HAL_OSPI_INSTRUCTION_16_BITS;
    sCommand.Address         = MX25LM51245G_CR2_REG1_ADDR;
    sCommand.AddressMode     = HAL_OSPI_ADDRESS_8_LINES;
    sCommand.DataMode        = HAL_OSPI_DATA_8_LINES;
    sCommand.NbData          = 2;
    sCommand.DummyCycles     = 0;
    reg_array[0] = 0;
    reg_array[1] = 0;


    if (HAL_OSPI_Command(&OSPINORHandle, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
      return OSPI_NOR_ERROR;
    }

    if (HAL_OSPI_Transmit(&OSPINORHandle, reg_array, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
      return OSPI_NOR_ERROR;
    }

    /* Wait that the configuration is effective and check that memory is ready */
    HAL_Delay(MX25LM51245G_WRITE_REG_MAX_TIME);

    sCommand.Instruction     = READ_STATUS_REG_CMD;
    sCommand.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
    sCommand.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
    sCommand.AddressMode     = HAL_OSPI_ADDRESS_NONE;
    sCommand.DataMode        = HAL_OSPI_DATA_1_LINE;
    sCommand.NbData          = 1;
    sCommand.DummyCycles     = 0;

    sConfig.Match = 0;
    sConfig.Mask  = MX25LM51245G_SR_WIP;

    if (HAL_OSPI_Command(&OSPINORHandle, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
      return OSPI_NOR_ERROR;
    }

    if (HAL_OSPI_AutoPolling(&OSPINORHandle, &sConfig, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
      return OSPI_NOR_ERROR;
    }

    /* Check the configuration has been correctly done */
    sCommand.Instruction = READ_CFG_REG_2_CMD;
    sCommand.Address     = MX25LM51245G_CR2_REG1_ADDR;
    sCommand.AddressMode = HAL_OSPI_ADDRESS_1_LINE;

    if (HAL_OSPI_Command(&OSPINORHandle, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
      return OSPI_NOR_ERROR;
    }

    if (HAL_OSPI_Receive(&OSPINORHandle, reg_array, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
      return OSPI_NOR_ERROR;
    }

    if (reg_array[0] != 0)
    {
      return OSPI_NOR_ERROR;
    }

  }

  return OSPI_NOR_OK;
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



