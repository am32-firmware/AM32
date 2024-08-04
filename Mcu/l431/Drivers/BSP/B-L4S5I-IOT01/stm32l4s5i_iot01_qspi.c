/**
  ******************************************************************************
  * @file    stm32l4s5i_iot01_qspi.c
  * @author  MCD Application Team
  * @brief   This file includes a standard driver for the MX25R6435F QSPI 
  *          memory mounted on STM32L4S5I IOT01 board.
  @verbatim
  ==============================================================================
                     ##### How to use this driver #####
  ==============================================================================  
  [..] 
   (#) This driver is used to drive the MX25R6435F QSPI external 
       memory mounted on STM32L4S5I IOT01 board.
       
   (#) This driver need a specific component driver (MX25R6435F) to be included with.

   (#) Initialization steps:
       (++) Initialize the QPSI external memory using the BSP_QSPI_Init() function. This 
            function includes the MSP layer hardware resources initialization and the
            QSPI interface with the external memory. The BSP_QSPI_DeInit() can be used 
            to deactivate the QSPI interface.
  
   (#) QSPI memory operations
       (++) QSPI memory can be accessed with read/write operations once it is
            initialized.
            Read/write operation can be performed with AHB access using the functions
            BSP_QSPI_Read()/BSP_QSPI_Write().
       (++) The function to the QSPI memory in memory-mapped mode is possible after 
            the call of the function BSP_QSPI_EnableMemoryMappedMode().
       (++) The function BSP_QSPI_GetInfo() returns the configuration of the QSPI memory. 
            (see the QSPI memory data sheet)
       (++) Perform erase block operation using the function BSP_QSPI_Erase_Block() and by
            specifying the block address. You can perform an erase operation of the whole 
            chip by calling the function BSP_QSPI_Erase_Chip(). 
       (++) The function BSP_QSPI_GetStatus() returns the current status of the QSPI memory. 
            (see the QSPI memory data sheet)
       (++) Perform erase sector operation using the function BSP_QSPI_Erase_Sector()
            which is not blocking. So the function BSP_QSPI_GetStatus() should be used
            to check if the memory is busy, and the functions BSP_QSPI_SuspendErase()/
            BSP_QSPI_ResumeErase() can be used to perform other operations during the 
            sector erase.
       (++) Deep power down of the QSPI memory is managed with the call of the functions
            BSP_QSPI_EnterDeepPowerDown()/BSP_QSPI_LeaveDeepPowerDown()
  @endverbatim
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32l4s5i_iot01_qspi.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L4S5I_IOT01
  * @{
  */

/** @defgroup STM32L4S5I_IOT01_QSPI QSPI
  * @{
  */

/* Private constants --------------------------------------------------------*/ 
/** @defgroup STM32L4S5I_IOT01_QSPI_Private_Constants QSPI Private Constants
  * @{
  */
#define QSPI_QUAD_DISABLE       0x0
#define QSPI_QUAD_ENABLE        0x1

#define QSPI_HIGH_PERF_DISABLE  0x0
#define QSPI_HIGH_PERF_ENABLE   0x1
/**
  * @}
  */
/* Private variables ---------------------------------------------------------*/

/** @defgroup STM32L4S5I_IOT01_QSPI_Private_Variables QSPI Private Variables
  * @{
  */
OSPI_HandleTypeDef OSPIHandle;

/**
  * @}
  */


/* Private functions ---------------------------------------------------------*/

/** @defgroup STM32L4S5I_IOT01_QSPI_Private_Functions QSPI Private Functions
  * @{
  */
static uint8_t QSPI_ResetMemory        (OSPI_HandleTypeDef *hospi);
static uint8_t QSPI_WriteEnable        (OSPI_HandleTypeDef *hospi);
static uint8_t QSPI_AutoPollingMemReady(OSPI_HandleTypeDef *hospi, uint32_t Timeout);
static uint8_t QSPI_QuadMode           (OSPI_HandleTypeDef *hospi, uint8_t Operation);
static uint8_t QSPI_HighPerfMode       (OSPI_HandleTypeDef *hospi, uint8_t Operation);

/**
  * @}
  */

/* Exported functions ---------------------------------------------------------*/

/** @addtogroup STM32L4S5I_IOT01_QSPI_Exported_Functions
  * @{
  */

/**
  * @brief  Initializes the QSPI interface.
  * @retval QSPI memory status
  */
uint8_t BSP_QSPI_Init(void)
{ 
  OSPIHandle.Instance = OCTOSPI1;

  /* Call the DeInit function to reset the driver */
  if (HAL_OSPI_DeInit(&OSPIHandle) != HAL_OK)
  {
    return QSPI_ERROR;
  }
        
  /* System level initialization */
  BSP_QSPI_MspInit();
  
  /* QSPI initialization */
  OSPIHandle.Init.FifoThreshold         = 4;
  OSPIHandle.Init.DualQuad              = HAL_OSPI_DUALQUAD_DISABLE;
  OSPIHandle.Init.MemoryType            = HAL_OSPI_MEMTYPE_MACRONIX;
  OSPIHandle.Init.DeviceSize            = POSITION_VAL(MX25R6435F_FLASH_SIZE);
  OSPIHandle.Init.ChipSelectHighTime    = 1;
  OSPIHandle.Init.FreeRunningClock      = HAL_OSPI_FREERUNCLK_DISABLE;
  OSPIHandle.Init.ClockMode             = HAL_OSPI_CLOCK_MODE_0;
  OSPIHandle.Init.ClockPrescaler        = 4; /* QSPI clock = 110MHz / ClockPrescaler = 27.5 MHz */
  OSPIHandle.Init.SampleShifting        = HAL_OSPI_SAMPLE_SHIFTING_NONE;
  OSPIHandle.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_ENABLE;
  OSPIHandle.Init.ChipSelectBoundary    = 0;
  OSPIHandle.Init.DelayBlockBypass      = HAL_OSPI_DELAY_BLOCK_USED;

  if (HAL_OSPI_Init(&OSPIHandle) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* QSPI memory reset */
  if (QSPI_ResetMemory(&OSPIHandle) != QSPI_OK)
  {
    return QSPI_NOT_SUPPORTED;
  }
 
  /* QSPI quad enable */
  if (QSPI_QuadMode(&OSPIHandle, QSPI_QUAD_ENABLE) != QSPI_OK)
  {
    return QSPI_ERROR;
  }
 
  /* High performance mode enable */
  if (QSPI_HighPerfMode(&OSPIHandle, QSPI_HIGH_PERF_ENABLE) != QSPI_OK)
  {
    return QSPI_ERROR;
  }
  
  /* Re-configure the clock for the high performance mode */
  OSPIHandle.Init.ClockPrescaler = 2; /* QSPI clock = 110MHz / ClockPrescaler = 55 MHz */

  if (HAL_OSPI_Init(&OSPIHandle) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  return QSPI_OK;
}

/**
  * @brief  De-Initializes the QSPI interface.
  * @retval QSPI memory status
  */
uint8_t BSP_QSPI_DeInit(void)
{ 
  OSPIHandle.Instance = OCTOSPI1;
  
  /* Call the DeInit function to reset the driver */
  if (HAL_OSPI_DeInit(&OSPIHandle) != HAL_OK)
  {
    return QSPI_ERROR;
  }
  
  /* System level De-initialization */
  BSP_QSPI_MspDeInit();
  
  return QSPI_OK;
}

/**
  * @brief  Reads an amount of data from the QSPI memory.
  * @param  pData    : Pointer to data to be read
  * @param  ReadAddr : Read start address
  * @param  Size     : Size of data to read    
  * @retval QSPI memory status
  */
uint8_t BSP_QSPI_Read(uint8_t* pData, uint32_t ReadAddr, uint32_t Size)
{
  OSPI_RegularCmdTypeDef sCommand;

  /* Initialize the read command */
  sCommand.OperationType         = HAL_OSPI_OPTYPE_COMMON_CFG;
  sCommand.FlashId               = HAL_OSPI_FLASH_ID_1;
  sCommand.Instruction           = QUAD_INOUT_READ_CMD;
  sCommand.InstructionMode       = HAL_OSPI_INSTRUCTION_1_LINE;
  sCommand.InstructionSize       = HAL_OSPI_INSTRUCTION_8_BITS;
  sCommand.InstructionDtrMode    = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  sCommand.Address               = ReadAddr;
  sCommand.AddressMode           = HAL_OSPI_ADDRESS_4_LINES;
  sCommand.AddressSize           = HAL_OSPI_ADDRESS_24_BITS;
  sCommand.AddressDtrMode        = HAL_OSPI_ADDRESS_DTR_DISABLE;
  sCommand.AlternateBytes        = MX25R6435F_ALT_BYTES_NO_PE_MODE;
  sCommand.AlternateBytesMode    = HAL_OSPI_ALTERNATE_BYTES_4_LINES;
  sCommand.AlternateBytesSize    = HAL_OSPI_ALTERNATE_BYTES_8_BITS;
  sCommand.AlternateBytesDtrMode = HAL_OSPI_ALTERNATE_BYTES_DTR_DISABLE;
  sCommand.DataMode              = HAL_OSPI_DATA_4_LINES;
  sCommand.NbData                = Size;
  sCommand.DataDtrMode           = HAL_OSPI_DATA_DTR_DISABLE;
  sCommand.DummyCycles           = MX25R6435F_DUMMY_CYCLES_READ_QUAD;
  sCommand.DQSMode               = HAL_OSPI_DQS_DISABLE;
  sCommand.SIOOMode              = HAL_OSPI_SIOO_INST_EVERY_CMD;
  
  /* Configure the command */
  if (HAL_OSPI_Command(&OSPIHandle, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }
  
  /* Reception of the data */
  if (HAL_OSPI_Receive(&OSPIHandle, pData, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  return QSPI_OK;
}

/**
  * @brief  Writes an amount of data to the QSPI memory.
  * @param  pData     : Pointer to data to be written
  * @param  WriteAddr : Write start address
  * @param  Size      : Size of data to write    
  * @retval QSPI memory status
  */
uint8_t BSP_QSPI_Write(uint8_t* pData, uint32_t WriteAddr, uint32_t Size)
{
  OSPI_RegularCmdTypeDef sCommand;
  uint32_t end_addr, current_size, current_addr;

  /* Calculation of the size between the write address and the end of the page */
  current_size = MX25R6435F_PAGE_SIZE - (WriteAddr % MX25R6435F_PAGE_SIZE);

  /* Check if the size of the data is less than the remaining place in the page */
  if (current_size > Size)
  {
    current_size = Size;
  }

  /* Initialize the address variables */
  current_addr = WriteAddr;
  end_addr = WriteAddr + Size;

  /* Initialize the program command */
  sCommand.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
  sCommand.FlashId            = HAL_OSPI_FLASH_ID_1;
  sCommand.Instruction        = QUAD_PAGE_PROG_CMD;
  sCommand.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;
  sCommand.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
  sCommand.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  sCommand.AddressMode        = HAL_OSPI_ADDRESS_4_LINES;
  sCommand.AddressSize        = HAL_OSPI_ADDRESS_24_BITS;
  sCommand.AddressDtrMode     = HAL_OSPI_ADDRESS_DTR_DISABLE;
  sCommand.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode           = HAL_OSPI_DATA_4_LINES;
  sCommand.DataDtrMode        = HAL_OSPI_DATA_DTR_DISABLE;
  sCommand.DummyCycles        = 0;
  sCommand.DQSMode            = HAL_OSPI_DQS_DISABLE;
  sCommand.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;
  
  /* Perform the write page by page */
  do
  {
    sCommand.Address = current_addr;
    sCommand.NbData  = current_size;

    /* Enable write operations */
    if (QSPI_WriteEnable(&OSPIHandle) != QSPI_OK)
    {
      return QSPI_ERROR;
    }
    
    /* Configure the command */
    if (HAL_OSPI_Command(&OSPIHandle, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
      return QSPI_ERROR;
    }
    
    /* Transmission of the data */
    if (HAL_OSPI_Transmit(&OSPIHandle, pData, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
      return QSPI_ERROR;
    }
    
    /* Configure automatic polling mode to wait for end of program */  
    if (QSPI_AutoPollingMemReady(&OSPIHandle, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != QSPI_OK)
    {
      return QSPI_ERROR;
    }
    
    /* Update the address and size variables for next page programming */
    current_addr += current_size;
    pData += current_size;
    current_size = ((current_addr + MX25R6435F_PAGE_SIZE) > end_addr) ? (end_addr - current_addr) : MX25R6435F_PAGE_SIZE;
  } while (current_addr < end_addr);
  
  return QSPI_OK;
}

/**
  * @brief  Erases the specified block of the QSPI memory. 
  * @param  BlockAddress : Block address to erase  
  * @retval QSPI memory status
  */
uint8_t BSP_QSPI_Erase_Block(uint32_t BlockAddress)
{
  OSPI_RegularCmdTypeDef sCommand;

  /* Initialize the erase command */
  sCommand.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
  sCommand.FlashId            = HAL_OSPI_FLASH_ID_1;
  sCommand.Instruction        = BLOCK_ERASE_CMD;
  sCommand.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;
  sCommand.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
  sCommand.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  sCommand.Address            = BlockAddress;
  sCommand.AddressMode        = HAL_OSPI_ADDRESS_1_LINE;
  sCommand.AddressSize        = HAL_OSPI_ADDRESS_24_BITS;
  sCommand.AddressDtrMode     = HAL_OSPI_ADDRESS_DTR_DISABLE;
  sCommand.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode           = HAL_OSPI_DATA_NONE;
  sCommand.DummyCycles        = 0;
  sCommand.DQSMode            = HAL_OSPI_DQS_DISABLE;
  sCommand.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;

  /* Enable write operations */
  if (QSPI_WriteEnable(&OSPIHandle) != QSPI_OK)
  {
    return QSPI_ERROR;
  }

  /* Send the command */
  if (HAL_OSPI_Command(&OSPIHandle, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }
  
  /* Configure automatic polling mode to wait for end of erase */  
  if (QSPI_AutoPollingMemReady(&OSPIHandle, MX25R6435F_BLOCK_ERASE_MAX_TIME) != QSPI_OK)
  {
    return QSPI_ERROR;
  }

  return QSPI_OK;
}

/**
  * @brief  Erases the specified sector of the QSPI memory. 
  * @param  Sector : Sector address to erase (0 to 2047) 
  * @retval QSPI memory status
  * @note This function is non blocking meaning that sector erase
  *       operation is started but not completed when the function 
  *       returns. Application has to call BSP_QSPI_GetStatus()
  *       to know when the device is available again (i.e. erase operation
  *       completed).
  */
uint8_t BSP_QSPI_Erase_Sector(uint32_t Sector)
{
  OSPI_RegularCmdTypeDef sCommand;
  
  if (Sector >= (uint32_t)(MX25R6435F_FLASH_SIZE/MX25R6435F_SECTOR_SIZE))
  {
    return QSPI_ERROR;
  }
  
  /* Initialize the erase command */
  sCommand.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
  sCommand.FlashId            = HAL_OSPI_FLASH_ID_1;
  sCommand.Instruction        = SECTOR_ERASE_CMD;
  sCommand.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;
  sCommand.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
  sCommand.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  sCommand.Address            = (Sector * MX25R6435F_SECTOR_SIZE);
  sCommand.AddressMode        = HAL_OSPI_ADDRESS_1_LINE;
  sCommand.AddressSize        = HAL_OSPI_ADDRESS_24_BITS;
  sCommand.AddressDtrMode     = HAL_OSPI_ADDRESS_DTR_DISABLE;
  sCommand.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode           = HAL_OSPI_DATA_NONE;
  sCommand.DummyCycles        = 0;
  sCommand.DQSMode            = HAL_OSPI_DQS_DISABLE;
  sCommand.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;
  
  /* Enable write operations */
  if (QSPI_WriteEnable(&OSPIHandle) != QSPI_OK)
  {
    return QSPI_ERROR;
  }
  
  /* Send the command */
  if (HAL_OSPI_Command(&OSPIHandle, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }
  
  return QSPI_OK;
}

/**
  * @brief  Erases the entire QSPI memory.
  * @retval QSPI memory status
  */
uint8_t BSP_QSPI_Erase_Chip(void)
{
  OSPI_RegularCmdTypeDef sCommand;

  /* Initialize the erase command */
  sCommand.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
  sCommand.FlashId            = HAL_OSPI_FLASH_ID_1;
  sCommand.Instruction        = CHIP_ERASE_CMD;
  sCommand.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;
  sCommand.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
  sCommand.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  sCommand.AddressMode        = HAL_OSPI_ADDRESS_NONE;
  sCommand.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode           = HAL_OSPI_DATA_NONE;
  sCommand.DummyCycles        = 0;
  sCommand.DQSMode            = HAL_OSPI_DQS_DISABLE;
  sCommand.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;

  /* Enable write operations */
  if (QSPI_WriteEnable(&OSPIHandle) != QSPI_OK)
  {
    return QSPI_ERROR;
  }

  /* Send the command */
  if (HAL_OSPI_Command(&OSPIHandle, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }
  
  /* Configure automatic polling mode to wait for end of erase */  
  if (QSPI_AutoPollingMemReady(&OSPIHandle, MX25R6435F_CHIP_ERASE_MAX_TIME) != QSPI_OK)
  {
    return QSPI_ERROR;
  }

  return QSPI_OK;
}

/**
  * @brief  Reads current status of the QSPI memory.
  * @retval QSPI memory status
  */
uint8_t BSP_QSPI_GetStatus(void)
{
  OSPI_RegularCmdTypeDef sCommand;
  uint8_t reg;

  /* Initialize the read security register command */
  sCommand.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
  sCommand.FlashId            = HAL_OSPI_FLASH_ID_1;
  sCommand.Instruction        = READ_SEC_REG_CMD;
  sCommand.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;
  sCommand.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
  sCommand.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  sCommand.AddressMode        = HAL_OSPI_ADDRESS_NONE;
  sCommand.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode           = HAL_OSPI_DATA_1_LINE;
  sCommand.NbData             = 1;
  sCommand.DataDtrMode        = HAL_OSPI_DATA_DTR_DISABLE;
  sCommand.DummyCycles        = 0;
  sCommand.DQSMode            = HAL_OSPI_DQS_DISABLE;
  sCommand.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_OSPI_Command(&OSPIHandle, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Reception of the data */
  if (HAL_OSPI_Receive(&OSPIHandle, &reg, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }
  
  /* Check the value of the register */
  if ((reg & (MX25R6435F_SECR_P_FAIL | MX25R6435F_SECR_E_FAIL)) != 0)
  {
    return QSPI_ERROR;
  }
  else if ((reg & (MX25R6435F_SECR_PSB | MX25R6435F_SECR_ESB)) != 0)
  {
    return QSPI_SUSPENDED;
  }

  /* Initialize the read status register command */
  sCommand.Instruction = READ_STATUS_REG_CMD;

  /* Configure the command */
  if (HAL_OSPI_Command(&OSPIHandle, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Reception of the data */
  if (HAL_OSPI_Receive(&OSPIHandle, &reg, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Check the value of the register */
  if ((reg & MX25R6435F_SR_WIP) != 0)
  {
    return QSPI_BUSY;
  }
  else
  {
    return QSPI_OK;
  }
}

/**
  * @brief  Return the configuration of the QSPI memory.
  * @param  pInfo : pointer on the configuration structure  
  * @retval QSPI memory status
  */
uint8_t BSP_QSPI_GetInfo(QSPI_Info* pInfo)
{
  /* Configure the structure with the memory configuration */
  pInfo->FlashSize          = MX25R6435F_FLASH_SIZE;
  pInfo->EraseSectorSize    = MX25R6435F_SECTOR_SIZE;
  pInfo->EraseSectorsNumber = (MX25R6435F_FLASH_SIZE/MX25R6435F_SECTOR_SIZE);
  pInfo->ProgPageSize       = MX25R6435F_PAGE_SIZE;
  pInfo->ProgPagesNumber    = (MX25R6435F_FLASH_SIZE/MX25R6435F_PAGE_SIZE);
  
  return QSPI_OK;
}

/**
  * @brief  Configure the QSPI in memory-mapped mode
  * @retval QSPI memory status
  */
uint8_t BSP_QSPI_EnableMemoryMappedMode(void)
{
  OSPI_RegularCmdTypeDef      sCommand;
  OSPI_MemoryMappedTypeDef sMemMappedCfg;

  /* Configure the command for the read instruction */
  sCommand.OperationType         = HAL_OSPI_OPTYPE_READ_CFG;
  sCommand.FlashId               = HAL_OSPI_FLASH_ID_1;
  sCommand.Instruction           = QUAD_INOUT_READ_CMD;
  sCommand.InstructionMode       = HAL_OSPI_INSTRUCTION_1_LINE;
  sCommand.InstructionSize       = HAL_OSPI_INSTRUCTION_8_BITS;
  sCommand.InstructionDtrMode    = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  sCommand.AddressMode           = HAL_OSPI_ADDRESS_4_LINES;
  sCommand.AddressSize           = HAL_OSPI_ADDRESS_24_BITS;
  sCommand.AddressDtrMode        = HAL_OSPI_ADDRESS_DTR_DISABLE;
  sCommand.AlternateBytes        = MX25R6435F_ALT_BYTES_NO_PE_MODE;
  sCommand.AlternateBytesMode    = HAL_OSPI_ALTERNATE_BYTES_4_LINES;
  sCommand.AlternateBytesSize    = HAL_OSPI_ALTERNATE_BYTES_8_BITS;
  sCommand.AlternateBytesDtrMode = HAL_OSPI_ALTERNATE_BYTES_DTR_DISABLE;
  sCommand.DataMode              = HAL_OSPI_DATA_4_LINES;
  sCommand.DataDtrMode           = HAL_OSPI_DATA_DTR_DISABLE;
  sCommand.DummyCycles           = MX25R6435F_DUMMY_CYCLES_READ_QUAD;
  sCommand.DQSMode               = HAL_OSPI_DQS_DISABLE;
  sCommand.SIOOMode              = HAL_OSPI_SIOO_INST_EVERY_CMD;
  
  /* Configure the command */
  if (HAL_OSPI_Command(&OSPIHandle, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Configure the command for the program instruction */
  sCommand.OperationType      = HAL_OSPI_OPTYPE_WRITE_CFG;
  sCommand.Instruction        = QUAD_PAGE_PROG_CMD;
  sCommand.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
  sCommand.DummyCycles        = 0;
  
  /* Configure the command */
  if (HAL_OSPI_Command(&OSPIHandle, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Configure the memory mapped mode */
  sMemMappedCfg.TimeOutActivation = HAL_OSPI_TIMEOUT_COUNTER_DISABLE;
  
  if (HAL_OSPI_MemoryMapped(&OSPIHandle, &sMemMappedCfg) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  return QSPI_OK;
}

/**
  * @brief  This function suspends an ongoing erase command.
  * @retval QSPI memory status
  */
uint8_t BSP_QSPI_SuspendErase(void)
{
  OSPI_RegularCmdTypeDef sCommand;
  
  /* Check whether the device is busy (erase operation is 
  in progress).
  */
  if (BSP_QSPI_GetStatus() == QSPI_BUSY)
  {
    /* Initialize the suspend command */
    sCommand.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
    sCommand.FlashId            = HAL_OSPI_FLASH_ID_1;
    sCommand.Instruction        = PROG_ERASE_SUSPEND_CMD;
    sCommand.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;
    sCommand.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
    sCommand.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
    sCommand.AddressMode        = HAL_OSPI_ADDRESS_NONE;
    sCommand.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
    sCommand.DataMode           = HAL_OSPI_DATA_NONE;
    sCommand.DummyCycles        = 0;
    sCommand.DQSMode            = HAL_OSPI_DQS_DISABLE;
    sCommand.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;
    
    /* Send the command */
    if (HAL_OSPI_Command(&OSPIHandle, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
      return QSPI_ERROR;
    }
    
    if (BSP_QSPI_GetStatus() == QSPI_SUSPENDED)
    {
      return QSPI_OK;
    }
    
    return QSPI_ERROR;
  }
  
  return QSPI_OK;
}

/**
  * @brief  This function resumes a paused erase command.
  * @retval QSPI memory status
  */
uint8_t BSP_QSPI_ResumeErase(void)
{
  OSPI_RegularCmdTypeDef sCommand;
  
  /* Check whether the device is in suspended state */
  if (BSP_QSPI_GetStatus() == QSPI_SUSPENDED)
  {
    /* Initialize the resume command */
    sCommand.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
    sCommand.FlashId            = HAL_OSPI_FLASH_ID_1;
    sCommand.Instruction        = PROG_ERASE_RESUME_CMD;
    sCommand.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;
    sCommand.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
    sCommand.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
    sCommand.AddressMode        = HAL_OSPI_ADDRESS_NONE;
    sCommand.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
    sCommand.DataMode           = HAL_OSPI_DATA_NONE;
    sCommand.DummyCycles        = 0;
    sCommand.DQSMode            = HAL_OSPI_DQS_DISABLE;
    sCommand.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;
    
    /* Send the command */
    if (HAL_OSPI_Command(&OSPIHandle, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
      return QSPI_ERROR;
    }
    
    /*
    When this command is executed, the status register write in progress bit is set to 1, and
    the flag status register program erase controller bit is set to 0. This command is ignored
    if the device is not in a suspended state.
    */
    
    if (BSP_QSPI_GetStatus() == QSPI_BUSY)
    {
      return QSPI_OK;
    }
    
    return QSPI_ERROR;
  }

  return QSPI_OK;
}

/**
  * @brief  This function enter the QSPI memory in deep power down mode.
  * @retval QSPI memory status
  */
uint8_t BSP_QSPI_EnterDeepPowerDown(void)
{
  OSPI_RegularCmdTypeDef sCommand;
  
  /* Initialize the deep power down command */
  sCommand.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
  sCommand.FlashId            = HAL_OSPI_FLASH_ID_1;
  sCommand.Instruction        = DEEP_POWER_DOWN_CMD;
  sCommand.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;
  sCommand.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
  sCommand.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  sCommand.AddressMode        = HAL_OSPI_ADDRESS_NONE;
  sCommand.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode           = HAL_OSPI_DATA_NONE;
  sCommand.DummyCycles        = 0;
  sCommand.DQSMode            = HAL_OSPI_DQS_DISABLE;
  sCommand.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;
    
  /* Send the command */
  if (HAL_OSPI_Command(&OSPIHandle, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }
  
  /* ---          Memory takes 10us max to enter deep power down          --- */
  /* --- At least 30us should be respected before leaving deep power down --- */
  
  return QSPI_OK;
}

/**
  * @brief  This function leave the QSPI memory from deep power down mode.
  * @retval QSPI memory status
  */
uint8_t BSP_QSPI_LeaveDeepPowerDown(void)
{
  OSPI_RegularCmdTypeDef sCommand;
  
  /* Initialize the erase command */
  sCommand.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
  sCommand.FlashId            = HAL_OSPI_FLASH_ID_1;
  sCommand.Instruction        = NO_OPERATION_CMD;
  sCommand.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;
  sCommand.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
  sCommand.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  sCommand.AddressMode        = HAL_OSPI_ADDRESS_NONE;
  sCommand.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode           = HAL_OSPI_DATA_NONE;
  sCommand.DummyCycles        = 0;
  sCommand.DQSMode            = HAL_OSPI_DQS_DISABLE;
  sCommand.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;
    
  /* Send the command */
  if (HAL_OSPI_Command(&OSPIHandle, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }
  
  /* --- A NOP command is sent to the memory, as the nCS should be low for at least 20 ns --- */
  /* ---                  Memory takes 35us min to leave deep power down                  --- */
  
  return QSPI_OK;
}

/**
  * @brief  Initializes the QSPI MSP.
  * @retval None
  */
__weak void BSP_QSPI_MspInit(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* Enable the QuadSPI memory interface clock */
  __HAL_RCC_OSPI1_CLK_ENABLE();

  /* Reset the QuadSPI memory interface */
  __HAL_RCC_OSPI1_FORCE_RESET();
  __HAL_RCC_OSPI1_RELEASE_RESET();

  /* Enable GPIO clocks */
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /* QSPI CLK, CS, D0, D1, D2 and D3 GPIO pins configuration  */
  GPIO_InitStruct.Pin       = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 |\
                              GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OCTOSPIM_P1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}

/**
  * @brief  De-Initializes the QSPI MSP.
  * @retval None
  */
__weak void BSP_QSPI_MspDeInit(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* QSPI CLK, CS, D0-D3 GPIO pins de-configuration  */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  GPIO_InitStruct.Pin       = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 |\
                              GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;

  HAL_GPIO_DeInit(GPIOE, GPIO_InitStruct.Pin);
    
  /* Reset the QuadSPI memory interface */
  __HAL_RCC_OSPI1_FORCE_RESET();
  __HAL_RCC_OSPI1_RELEASE_RESET();

  /* Disable the QuadSPI memory interface clock */
  __HAL_RCC_OSPI1_CLK_DISABLE();
}

/**
  * @}
  */

/** @addtogroup STM32L4S5I_IOT01_QSPI_Private_Functions 
  * @{
  */

/**
  * @brief  This function reset the QSPI memory.
  * @param  hospi : QSPI handle
  * @retval None
  */
static uint8_t QSPI_ResetMemory(OSPI_HandleTypeDef *hospi)
{
  OSPI_RegularCmdTypeDef sCommand;

  /* Initialize the reset enable command */
  sCommand.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
  sCommand.FlashId            = HAL_OSPI_FLASH_ID_1;
  sCommand.Instruction        = RESET_ENABLE_CMD;
  sCommand.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;
  sCommand.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
  sCommand.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  sCommand.AddressMode        = HAL_OSPI_ADDRESS_NONE;
  sCommand.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode           = HAL_OSPI_DATA_NONE;
  sCommand.DummyCycles        = 0;
  sCommand.DQSMode            = HAL_OSPI_DQS_DISABLE;
  sCommand.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;

  /* Send the command */
  if (HAL_OSPI_Command(hospi, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Send the reset memory command */
  sCommand.Instruction = RESET_MEMORY_CMD;
  if (HAL_OSPI_Command(hospi, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Configure automatic polling mode to wait the memory is ready */  
  if (QSPI_AutoPollingMemReady(hospi, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != QSPI_OK)
  {
    return QSPI_ERROR;
  }

  return QSPI_OK;
}

/**
  * @brief  This function send a Write Enable and wait it is effective.
  * @param  hospi : QSPI handle
  * @retval None
  */
static uint8_t QSPI_WriteEnable(OSPI_HandleTypeDef *hospi)
{
  OSPI_RegularCmdTypeDef sCommand;
  OSPI_AutoPollingTypeDef sConfig;

  /* Enable write operations */
  sCommand.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
  sCommand.FlashId            = HAL_OSPI_FLASH_ID_1;
  sCommand.Instruction        = WRITE_ENABLE_CMD;
  sCommand.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;
  sCommand.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
  sCommand.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  sCommand.AddressMode        = HAL_OSPI_ADDRESS_NONE;
  sCommand.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode           = HAL_OSPI_DATA_NONE;
  sCommand.DummyCycles        = 0;
  sCommand.DQSMode            = HAL_OSPI_DQS_DISABLE;
  sCommand.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;

  if (HAL_OSPI_Command(hospi, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }
  
  /* Configure automatic polling mode to wait for write enabling */  
  sConfig.Match         = MX25R6435F_SR_WEL;
  sConfig.Mask          = MX25R6435F_SR_WEL;
  sConfig.MatchMode     = HAL_OSPI_MATCH_MODE_AND;
  sConfig.Interval      = 0x10;
  sConfig.AutomaticStop = HAL_OSPI_AUTOMATIC_STOP_ENABLE;

  sCommand.Instruction  = READ_STATUS_REG_CMD;
  sCommand.DataMode     = HAL_OSPI_DATA_1_LINE;
  sCommand.NbData       = 1;
  sCommand.DataDtrMode  = HAL_OSPI_DATA_DTR_DISABLE;

  if (HAL_OSPI_Command(hospi, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  if (HAL_OSPI_AutoPolling(hospi, &sConfig, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  return QSPI_OK;
}

/**
  * @brief  This function read the SR of the memory and wait the EOP.
  * @param  hospi   : QSPI handle
  * @param  Timeout : Timeout for auto-polling
  * @retval None
  */
static uint8_t QSPI_AutoPollingMemReady(OSPI_HandleTypeDef *hospi, uint32_t Timeout)
{
  OSPI_RegularCmdTypeDef sCommand;
  OSPI_AutoPollingTypeDef sConfig;

  /* Configure automatic polling mode to wait for memory ready */  
  sCommand.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
  sCommand.FlashId            = HAL_OSPI_FLASH_ID_1;
  sCommand.Instruction        = READ_STATUS_REG_CMD;
  sCommand.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;
  sCommand.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
  sCommand.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  sCommand.AddressMode        = HAL_OSPI_ADDRESS_NONE;
  sCommand.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode           = HAL_OSPI_DATA_1_LINE;
  sCommand.NbData             = 1;
  sCommand.DataDtrMode        = HAL_OSPI_DATA_DTR_DISABLE;
  sCommand.DummyCycles        = 0;
  sCommand.DQSMode            = HAL_OSPI_DQS_DISABLE;
  sCommand.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;

  sConfig.Match         = 0;
  sConfig.Mask          = MX25R6435F_SR_WIP;
  sConfig.MatchMode     = HAL_OSPI_MATCH_MODE_AND;
  sConfig.Interval      = 0x10;
  sConfig.AutomaticStop = HAL_OSPI_AUTOMATIC_STOP_ENABLE;

  if (HAL_OSPI_Command(hospi, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  if (HAL_OSPI_AutoPolling(hospi, &sConfig, Timeout) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  return QSPI_OK;
}

/**
  * @brief  This function enables/disables the Quad mode of the memory.
  * @param  hospi     : QSPI handle
  * @param  Operation : QSPI_QUAD_ENABLE or QSPI_QUAD_DISABLE mode  
  * @retval None
  */
static uint8_t QSPI_QuadMode(OSPI_HandleTypeDef *hospi, uint8_t Operation)
{
  OSPI_RegularCmdTypeDef sCommand;
  uint8_t reg;

  /* Read status register */
  sCommand.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
  sCommand.FlashId            = HAL_OSPI_FLASH_ID_1;
  sCommand.Instruction        = READ_STATUS_REG_CMD;
  sCommand.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;
  sCommand.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
  sCommand.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  sCommand.AddressMode        = HAL_OSPI_ADDRESS_NONE;
  sCommand.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode           = HAL_OSPI_DATA_1_LINE;
  sCommand.DataDtrMode        = HAL_OSPI_DATA_DTR_DISABLE;
  sCommand.DummyCycles        = 0;
  sCommand.NbData             = 1;
  sCommand.DQSMode            = HAL_OSPI_DQS_DISABLE;
  sCommand.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;

  if (HAL_OSPI_Command(hospi, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  if (HAL_OSPI_Receive(hospi, &reg, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Enable write operations */
  if (QSPI_WriteEnable(hospi) != QSPI_OK)
  {
    return QSPI_ERROR;
  }
  
  /* Activate/deactivate the Quad mode */
  if (Operation == QSPI_QUAD_ENABLE)
  {
    SET_BIT(reg, MX25R6435F_SR_QE);
  }
  else
  {
    CLEAR_BIT(reg, MX25R6435F_SR_QE);
  }

  sCommand.Instruction = WRITE_STATUS_CFG_REG_CMD;

  if (HAL_OSPI_Command(hospi, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  if (HAL_OSPI_Transmit(hospi, &reg, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Wait that memory is ready */  
  if (QSPI_AutoPollingMemReady(hospi, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != QSPI_OK)
  {
    return QSPI_ERROR;
  }
  
  /* Check the configuration has been correctly done */
  sCommand.Instruction = READ_STATUS_REG_CMD;

  if (HAL_OSPI_Command(hospi, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  if (HAL_OSPI_Receive(hospi, &reg, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }
  
  if ((((reg & MX25R6435F_SR_QE) == 0) && (Operation == QSPI_QUAD_ENABLE)) ||
      (((reg & MX25R6435F_SR_QE) != 0) && (Operation == QSPI_QUAD_DISABLE)))
  {
    return QSPI_ERROR;
  }

  return QSPI_OK;
}

/**
  * @brief  This function enables/disables the high performance mode of the memory.
  * @param  hospi     : QSPI handle
  * @param  Operation : QSPI_HIGH_PERF_ENABLE or QSPI_HIGH_PERF_DISABLE high performance mode    
  * @retval None
  */
static uint8_t QSPI_HighPerfMode(OSPI_HandleTypeDef *hospi, uint8_t Operation)
{
  OSPI_RegularCmdTypeDef sCommand;
  uint8_t reg[3];

  /* Read status register */
  sCommand.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
  sCommand.FlashId            = HAL_OSPI_FLASH_ID_1;
  sCommand.Instruction        = READ_STATUS_REG_CMD;
  sCommand.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;
  sCommand.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
  sCommand.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  sCommand.AddressMode        = HAL_OSPI_ADDRESS_NONE;
  sCommand.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode           = HAL_OSPI_DATA_1_LINE;
  sCommand.DataDtrMode        = HAL_OSPI_DATA_DTR_DISABLE;
  sCommand.DummyCycles        = 0;
  sCommand.NbData             = 1;
  sCommand.DQSMode            = HAL_OSPI_DQS_DISABLE;
  sCommand.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;

  if (HAL_OSPI_Command(hospi, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  if (HAL_OSPI_Receive(hospi, &(reg[0]), HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Read configuration registers */
  sCommand.Instruction = READ_CFG_REG_CMD;
  sCommand.NbData      = 2;

  if (HAL_OSPI_Command(hospi, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  if (HAL_OSPI_Receive(hospi, &(reg[1]), HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Enable write operations */
  if (QSPI_WriteEnable(hospi) != QSPI_OK)
  {
    return QSPI_ERROR;
  }
  
  /* Activate/deactivate the Quad mode */
  if (Operation == QSPI_HIGH_PERF_ENABLE)
  {
    SET_BIT(reg[2], MX25R6435F_CR2_LH_SWITCH);
  }
  else
  {
    CLEAR_BIT(reg[2], MX25R6435F_CR2_LH_SWITCH);
  }

  sCommand.Instruction = WRITE_STATUS_CFG_REG_CMD;
  sCommand.NbData      = 3;

  if (HAL_OSPI_Command(hospi, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  if (HAL_OSPI_Transmit(hospi, &(reg[0]), HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Wait that memory is ready */  
  if (QSPI_AutoPollingMemReady(hospi, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != QSPI_OK)
  {
    return QSPI_ERROR;
  }
  
  /* Check the configuration has been correctly done */
  sCommand.Instruction = READ_CFG_REG_CMD;
  sCommand.NbData      = 2;

  if (HAL_OSPI_Command(hospi, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  if (HAL_OSPI_Receive(hospi, &(reg[0]), HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }
  
  if ((((reg[1] & MX25R6435F_CR2_LH_SWITCH) == 0) && (Operation == QSPI_HIGH_PERF_ENABLE)) ||
      (((reg[1] & MX25R6435F_CR2_LH_SWITCH) != 0) && (Operation == QSPI_HIGH_PERF_DISABLE)))
  {
    return QSPI_ERROR;
  }

  return QSPI_OK;
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



