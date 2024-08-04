/**
  ******************************************************************************
  * @file    stm32l4p5g_discovery_psram.c
  * @author  MCD Application Team
  * @brief   This file includes the PSRAM driver for the Octal PSRAM APS6408L-3OB-BA
  *          memory device mounted on STM32L4P5G_DISCOVERY boards.
  @verbatim
  How To use this driver:
  -----------------------
   - This driver is used to drive the APS6408L-3OB-BA external memory mounted
     on STM32L4P5G discovery board.
   - This driver does not need a specific component driver for the PSRAM device
     to be included with.

  Driver description:
  ------------------
  + Initialization steps:
     o Initialize the PSRAM external memory using the BSP_PSRAM_Init() function. This
       function includes the MSP layer hardware resources initialization and the
       OCTOSPI controller configuration to interface with the external PSRAM memory.

  + PSRAM read/write operations
     o PSRAM external memory can be accessed with read/write operations once it is
       initialized.
       Read/write operation can be performed with AHB access using the functions
       BSP_PSRAM_ReadData()/BSP_PSRAM_WriteData() in polling mode,
       BSP_PSRAM_ReadData_IT()/BSP_PSRAM_WriteData_IT() in interrupt mode,
       or by DMA transfer using the functions
       BSP_PSRAM_ReadData_DMA()/BSP_PSRAM_WriteData_DMA().
     o The AHB access is performed with 18-bit width transaction, the DMA transfer
       configuration is fixed at single (no burst) byte transfer.
     o User can implement his own functions for read/write access with his desired
       configurations.
     o Ior transfer in DMA mode, the function BSP_PSRAM_DMA_IRQHandler()
       is called in IRQ handler file, to serve the generated interrupt once the DMA
       transfer is complete.
     o Ior transfer in interrupt mode, the function BSP_PSRAM_OSPI_IRQHandler()
       is called in IRQ handler file, to serve the generated interrupt once the
       transfer is complete.
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
#include "stm32l4p5g_discovery_psram.h"
#include "stm32l4p5g_discovery_io.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L4P5G_DISCOVERY
  * @{
  */

/** @defgroup STM32L4P5G_DISCOVERY_PSRAM STM32L4P5G_DISCOVERY PSRAM
  * @{
  */

/** @defgroup STM32L4P5G_DISCOVERY_PSRAM_Private_Defines Private Defines
  * @{
  */

#define OSPI_P1_CS_PIN                   GPIO_PIN_11
#define OSPI_P1_CLK_PIN                  GPIO_PIN_10
#define OSPI_P1_NCLK_PIN                 GPIO_PIN_9
#define OSPI_P1_D0_PIN                   GPIO_PIN_12
#define OSPI_P1_D1_PIN                   GPIO_PIN_13
#define OSPI_P1_D2_PIN                   GPIO_PIN_7
#define OSPI_P1_D3_PIN                   GPIO_PIN_6
#define OSPI_P1_D4_PIN                   GPIO_PIN_4
#define OSPI_P1_D5_PIN                   GPIO_PIN_5
#define OSPI_P1_D6_PIN                   GPIO_PIN_3
#define OSPI_P1_D7_PIN                   GPIO_PIN_7
#define OSPI_P1_DQS_PIN                  GPIO_PIN_6

#define OSPI_P2_CS_PIN                   GPIO_PIN_12
#define OSPI_P2_CLK_PIN                  GPIO_PIN_4
#define OSPI_P2_NCLK_PIN                 GPIO_PIN_5
#define OSPI_P2_D0_PIN                   GPIO_PIN_0
#define OSPI_P2_D1_PIN                   GPIO_PIN_1
#define OSPI_P2_D2_PIN                   GPIO_PIN_2
#define OSPI_P2_D3_PIN                   GPIO_PIN_3
#define OSPI_P2_D4_PIN                   GPIO_PIN_0
#define OSPI_P2_D5_PIN                   GPIO_PIN_1
#define OSPI_P2_D6_PIN                   GPIO_PIN_9
#define OSPI_P2_D7_PIN                   GPIO_PIN_10
#define OSPI_P2_DQS_PIN                  GPIO_PIN_12

#define OSPI_P1_CS_GPIO_PORT             GPIOE
#define OSPI_P1_CLK_GPIO_PORT            GPIOE
#define OSPI_P1_NCLK_GPIO_PORT           GPIOE
#define OSPI_P1_D0_GPIO_PORT             GPIOE
#define OSPI_P1_D1_GPIO_PORT             GPIOE
#define OSPI_P1_D2_GPIO_PORT             GPIOA
#define OSPI_P1_D3_GPIO_PORT             GPIOA
#define OSPI_P1_D4_GPIO_PORT             GPIOD
#define OSPI_P1_D5_GPIO_PORT             GPIOD
#define OSPI_P1_D6_GPIO_PORT             GPIOC
#define OSPI_P1_D7_GPIO_PORT             GPIOD
#define OSPI_P1_DQS_GPIO_PORT            GPIOG

#define OSPI_P2_CS_GPIO_PORT             GPIOG
#define OSPI_P2_CLK_GPIO_PORT            GPIOF
#define OSPI_P2_NCLK_GPIO_PORT           GPIOF
#define OSPI_P2_D0_GPIO_PORT             GPIOF
#define OSPI_P2_D1_GPIO_PORT             GPIOF
#define OSPI_P2_D2_GPIO_PORT             GPIOF
#define OSPI_P2_D3_GPIO_PORT             GPIOF
#define OSPI_P2_D4_GPIO_PORT             GPIOG
#define OSPI_P2_D5_GPIO_PORT             GPIOG
#define OSPI_P2_D6_GPIO_PORT             GPIOG
#define OSPI_P2_D7_GPIO_PORT             GPIOG
#define OSPI_P2_DQS_GPIO_PORT            GPIOF

/**
  * @}
  */

/** @defgroup STM32L4P5G_DISCOVERY_PSRAM_Private_Macros Private Macros
  * @{
  */
#define OSPI_P1_CS_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOE_CLK_ENABLE()
#define OSPI_P1_CLK_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOE_CLK_ENABLE()
#define OSPI_P1_NCLK_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOE_CLK_ENABLE()
#define OSPI_P1_D0_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOE_CLK_ENABLE()
#define OSPI_P1_D1_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOE_CLK_ENABLE()
#define OSPI_P1_D2_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOA_CLK_ENABLE()
#define OSPI_P1_D3_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOA_CLK_ENABLE()
#define OSPI_P1_D4_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOD_CLK_ENABLE()
#define OSPI_P1_D5_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOD_CLK_ENABLE()
#define OSPI_P1_D6_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOC_CLK_ENABLE()
#define OSPI_P1_D7_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOD_CLK_ENABLE()
#define OSPI_P1_DQS_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOG_CLK_ENABLE()

#define OSPI_P2_CS_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOG_CLK_ENABLE()
#define OSPI_P2_CLK_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOF_CLK_ENABLE()
#define OSPI_P2_NCLK_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOF_CLK_ENABLE()
#define OSPI_P2_D0_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOF_CLK_ENABLE()
#define OSPI_P2_D1_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOF_CLK_ENABLE()
#define OSPI_P2_D2_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOF_CLK_ENABLE()
#define OSPI_P2_D3_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOF_CLK_ENABLE()
#define OSPI_P2_D4_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOG_CLK_ENABLE()
#define OSPI_P2_D5_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOG_CLK_ENABLE()
#define OSPI_P2_D6_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOG_CLK_ENABLE()
#define OSPI_P2_D7_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOG_CLK_ENABLE()
#define OSPI_P2_DQS_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOF_CLK_ENABLE()

#define OSPI_P1_CS_GPIO_CLK_DISABLE()    __HAL_RCC_GPIOE_CLK_DISABLE()
#define OSPI_P1_CLK_GPIO_CLK_DISABLE()   __HAL_RCC_GPIOE_CLK_DISABLE()
#define OSPI_P1_NCLK_GPIO_CLK_DISABLE()  __HAL_RCC_GPIOE_CLK_DISABLE()
#define OSPI_P1_D0_GPIO_CLK_DISABLE()    __HAL_RCC_GPIOE_CLK_DISABLE()
#define OSPI_P1_D1_GPIO_CLK_DISABLE()    __HAL_RCC_GPIOE_CLK_DISABLE()
#define OSPI_P1_D2_GPIO_CLK_DISABLE()    __HAL_RCC_GPIOA_CLK_DISABLE()
#define OSPI_P1_D3_GPIO_CLK_DISABLE()    __HAL_RCC_GPIOA_CLK_DISABLE()
#define OSPI_P1_D4_GPIO_CLK_DISABLE()    __HAL_RCC_GPIOD_CLK_DISABLE()
#define OSPI_P1_D5_GPIO_CLK_DISABLE()    __HAL_RCC_GPIOD_CLK_DISABLE()
#define OSPI_P1_D6_GPIO_CLK_DISABLE()    __HAL_RCC_GPIOC_CLK_DISABLE()
#define OSPI_P1_D7_GPIO_CLK_DISABLE()    __HAL_RCC_GPIOD_CLK_DISABLE()
#define OSPI_P1_DQS_GPIO_CLK_DISABLE()   __HAL_RCC_GPIOG_CLK_DISABLE()

#define OSPI_P2_CS_GPIO_CLK_DISABLE()    __HAL_RCC_GPIOG_CLK_DISABLE()
#define OSPI_P2_CLK_GPIO_CLK_DISABLE()   __HAL_RCC_GPIOF_CLK_DISABLE()
#define OSPI_P2_NCLK_GPIO_CLK_DISABLE()  __HAL_RCC_GPIOF_CLK_DISABLE()
#define OSPI_P2_D0_GPIO_CLK_DISABLE()    __HAL_RCC_GPIOF_CLK_DISABLE()
#define OSPI_P2_D1_GPIO_CLK_DISABLE()    __HAL_RCC_GPIOF_CLK_DISABLE()
#define OSPI_P2_D2_GPIO_CLK_DISABLE()    __HAL_RCC_GPIOF_CLK_DISABLE()
#define OSPI_P2_D3_GPIO_CLK_DISABLE()    __HAL_RCC_GPIOF_CLK_DISABLE()
#define OSPI_P2_D4_GPIO_CLK_DISABLE()    __HAL_RCC_GPIOG_CLK_DISABLE()
#define OSPI_P2_D5_GPIO_CLK_DISABLE()    __HAL_RCC_GPIOG_CLK_DISABLE()
#define OSPI_P2_D6_GPIO_CLK_DISABLE()    __HAL_RCC_GPIOG_CLK_DISABLE()
#define OSPI_P2_D7_GPIO_CLK_DISABLE()    __HAL_RCC_GPIOG_CLK_DISABLE()
#define OSPI_P2_DQS_GPIO_CLK_DISABLE()   __HAL_RCC_GPIOF_CLK_DISABLE()

/**
  * @}
  */

/** @defgroup STM32L4P5G_DISCOVERY_PSRAM_Private_Variables Private Variables
  * @{
  */
OSPI_HandleTypeDef OSPIPSRAMHandle;
uint32_t bsp_psram_initialized = 0;

/**
  * @}
  */

/** @defgroup STM32L4P5G_DISCOVERY_PSRAM_Private_Function_Prototypes Private Function Prototypes
  * @{
  */
static void PSRAM_MspInit(void);
static void PSRAM_MspDeInit(void);
/**
  * @}
  */

/** @defgroup STM32L4P5G_DISCOVERY_PSRAM_Private_Functions Private Functions
  * @{
  */


/**
  * @brief  Initializes the PSRAM device.
  * @retval PSRAM status
  */
uint8_t BSP_PSRAM_Init(void)
{
  OSPIM_CfgTypeDef OSPIM_Cfg_Struct;
  OSPI_RegularCmdTypeDef  sCommand = {0};
  uint8_t reg[2];

  if (bsp_psram_initialized == 0)
  {

    PSRAM_MspInit();

    OSPIM_Cfg_Struct.ClkPort    = 1;
    OSPIM_Cfg_Struct.DQSPort    = 1;
    OSPIM_Cfg_Struct.IOHighPort = HAL_OSPIM_IOPORT_1_HIGH;
    OSPIM_Cfg_Struct.IOLowPort  = HAL_OSPIM_IOPORT_1_LOW;
    OSPIM_Cfg_Struct.NCSPort    = 1;

    OSPIM_Cfg_Struct.Req2AckTime = 1;

    OSPIPSRAMHandle.Instance = OCTOSPI1;
    HAL_OSPI_DeInit(&OSPIPSRAMHandle);

    OSPIPSRAMHandle.Init.FifoThreshold         = 2;
    OSPIPSRAMHandle.Init.DualQuad              = HAL_OSPI_DUALQUAD_DISABLE;
    OSPIPSRAMHandle.Init.MemoryType            = HAL_OSPI_MEMTYPE_APMEMORY;
    OSPIPSRAMHandle.Init.DeviceSize            = 23; /* 64 MBits */
    OSPIPSRAMHandle.Init.ChipSelectHighTime    = 1;
    OSPIPSRAMHandle.Init.FreeRunningClock      = HAL_OSPI_FREERUNCLK_DISABLE;
    OSPIPSRAMHandle.Init.ClockMode             = HAL_OSPI_CLOCK_MODE_0;
    OSPIPSRAMHandle.Init.ClockPrescaler        = 0x03;
    OSPIPSRAMHandle.Init.SampleShifting        = HAL_OSPI_SAMPLE_SHIFTING_NONE;
    OSPIPSRAMHandle.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_ENABLE;
    OSPIPSRAMHandle.Init.ChipSelectBoundary    = 4;
    OSPIPSRAMHandle.Init.DelayBlockBypass      = HAL_OSPI_DELAY_BLOCK_USED;
    OSPIPSRAMHandle.Init.MaxTran               = 0;

    if (HAL_OSPIM_Config(&OSPIPSRAMHandle, &OSPIM_Cfg_Struct, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
      return PSRAM_ERROR;
    }

    if (HAL_OSPI_Init(&OSPIPSRAMHandle) != HAL_OK) {
      return PSRAM_ERROR;
    }

    sCommand.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
    sCommand.FlashId            = HAL_OSPI_FLASH_ID_1;
    sCommand.InstructionMode    = HAL_OSPI_INSTRUCTION_8_LINES;
    sCommand.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
    sCommand.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
    sCommand.AddressMode        = HAL_OSPI_ADDRESS_8_LINES;
    sCommand.AddressSize        = HAL_OSPI_ADDRESS_32_BITS;
    sCommand.AddressDtrMode     = HAL_OSPI_ADDRESS_DTR_ENABLE;
    sCommand.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
    sCommand.DataMode           = HAL_OSPI_DATA_8_LINES;
    sCommand.DataDtrMode        = HAL_OSPI_DATA_DTR_ENABLE;
    sCommand.DQSMode            = HAL_OSPI_DQS_ENABLE;
    sCommand.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;
    sCommand.Instruction        = READ_REG_CMD_SRAM;
    sCommand.Address            = 0;
    sCommand.NbData             = 2;
    sCommand.DummyCycles        = 5;

    if (HAL_OSPI_Command(&OSPIPSRAMHandle, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
      return PSRAM_ERROR;
    }

    if (HAL_OSPI_Receive(&OSPIPSRAMHandle, reg, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
      return PSRAM_ERROR;
    }

    sCommand.Instruction = WRITE_REG_CMD_SRAM;
    sCommand.DummyCycles = 0;
    MODIFY_REG(reg[0], 0x03, 0x00);

    if (HAL_OSPI_Command(&OSPIPSRAMHandle, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
      return PSRAM_ERROR;
    }

    if (HAL_OSPI_Transmit(&OSPIPSRAMHandle, reg, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
      return PSRAM_ERROR;
    }

    bsp_psram_initialized = 1;
  }

  return PSRAM_OK;
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
    if(HAL_OSPI_DeInit(&OSPIPSRAMHandle) != HAL_OK)
    {
      psram_status = PSRAM_ERROR;
    }
    else
    {
      psram_status = PSRAM_OK;
    }

    /* PSRAM controller de-initialization */
    PSRAM_MspDeInit();

    bsp_psram_initialized = 0;
  }

  return psram_status;
}


/**
  * @brief  Configure the OSPI in memory-mapped mode.
  * @retval PSRAM status
  */
uint8_t BSP_PSRAM_EnableMemoryMappedMode(void)
{
  OSPI_RegularCmdTypeDef  sCommand = {0};
  OSPI_MemoryMappedTypeDef sMemMappedCfg = {0};

  sCommand.OperationType      = HAL_OSPI_OPTYPE_WRITE_CFG;
  sCommand.FlashId            = HAL_OSPI_FLASH_ID_1;
  sCommand.InstructionMode    = HAL_OSPI_INSTRUCTION_8_LINES;
  sCommand.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
  sCommand.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  sCommand.AddressMode        = HAL_OSPI_ADDRESS_8_LINES;
  sCommand.AddressSize        = HAL_OSPI_ADDRESS_32_BITS;
  sCommand.AddressDtrMode     = HAL_OSPI_ADDRESS_DTR_ENABLE;
  sCommand.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode           = HAL_OSPI_DATA_8_LINES;
  sCommand.DataDtrMode        = HAL_OSPI_DATA_DTR_ENABLE;
  sCommand.DQSMode            = HAL_OSPI_DQS_ENABLE;
  sCommand.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;

  sCommand.Address            = 0;
  sCommand.NbData             = 10;

  sCommand.Instruction = WRITE_REG_CMD_SRAM;
  sCommand.DummyCycles = DUMMY_CLOCK_CYCLES_SRAM_WRITE;


    if (HAL_OSPI_Command(&OSPIPSRAMHandle, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
      return PSRAM_ERROR;
    }

    sCommand.OperationType = HAL_OSPI_OPTYPE_READ_CFG;
    sCommand.Instruction   = READ_CMD_SRAM;
    sCommand.DummyCycles   = DUMMY_CLOCK_CYCLES_SRAM_READ;

    if (HAL_OSPI_Command(&OSPIPSRAMHandle, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
      return PSRAM_ERROR;
    }

    sMemMappedCfg.TimeOutActivation = HAL_OSPI_TIMEOUT_COUNTER_DISABLE;
    if (HAL_OSPI_MemoryMapped(&OSPIPSRAMHandle, &sMemMappedCfg) != HAL_OK) {
      return PSRAM_ERROR;
    }

  return PSRAM_OK;
}


/**
  * @brief  Reads an amount of data from the PSRAM device in polling mode.
  * @param  uwStartAddress: Read start address
  * @param  pData: Pointer to data to be read
  * @param  uwDataSize: Size of read data from the memory
  * @retval PSRAM status
  */
uint8_t BSP_PSRAM_ReadData(uint32_t uwStartAddress, uint8_t *pData, uint32_t uwDataSize)
{
  OSPI_RegularCmdTypeDef  sCommand = {0};

  sCommand.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
  sCommand.FlashId            = HAL_OSPI_FLASH_ID_1;
  sCommand.InstructionMode    = HAL_OSPI_INSTRUCTION_8_LINES;
  sCommand.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
  sCommand.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  sCommand.AddressMode        = HAL_OSPI_ADDRESS_8_LINES;
  sCommand.AddressSize        = HAL_OSPI_ADDRESS_32_BITS;
  sCommand.AddressDtrMode     = HAL_OSPI_ADDRESS_DTR_ENABLE;
  sCommand.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode           = HAL_OSPI_DATA_8_LINES;
  sCommand.DataDtrMode        = HAL_OSPI_DATA_DTR_ENABLE;
  sCommand.DQSMode            = HAL_OSPI_DQS_ENABLE;
  sCommand.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;


  /* 1- Reading Sequence ------------------------------------------------ */
  sCommand.Instruction        = READ_CMD_SRAM;
  sCommand.Address            = uwStartAddress;
  sCommand.NbData             = uwDataSize;
  sCommand.DummyCycles        = DUMMY_CLOCK_CYCLES_SRAM_READ;

  if (HAL_OSPI_Command(&OSPIPSRAMHandle, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return PSRAM_ERROR;
  }

  if (HAL_OSPI_Receive(&OSPIPSRAMHandle, pData, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return PSRAM_ERROR;
  }

  return PSRAM_OK;
}


/**
  * @brief  Reads an amount of data from the PSRAM device in IT mode.
  * @param  uwStartAddress: Read start address
  * @param  pData: Pointer to data to be read
  * @param  uwDataSize: Size of read data from the memory
  * @retval PSRAM status
  */
uint8_t BSP_PSRAM_ReadData_IT(uint32_t uwStartAddress, uint8_t *pData, uint32_t uwDataSize)
{
  OSPI_RegularCmdTypeDef  sCommand = {0};

  sCommand.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
  sCommand.FlashId            = HAL_OSPI_FLASH_ID_1;
  sCommand.InstructionMode    = HAL_OSPI_INSTRUCTION_8_LINES;
  sCommand.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
  sCommand.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  sCommand.AddressMode        = HAL_OSPI_ADDRESS_8_LINES;
  sCommand.AddressSize        = HAL_OSPI_ADDRESS_32_BITS;
  sCommand.AddressDtrMode     = HAL_OSPI_ADDRESS_DTR_ENABLE;
  sCommand.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode           = HAL_OSPI_DATA_8_LINES;
  sCommand.DataDtrMode        = HAL_OSPI_DATA_DTR_ENABLE;
  sCommand.DQSMode            = HAL_OSPI_DQS_ENABLE;
  sCommand.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;


  /* 1- Reading Sequence ------------------------------------------------ */
  sCommand.Instruction        = READ_CMD_SRAM;
  sCommand.Address            = uwStartAddress;
  sCommand.NbData             = uwDataSize;
  sCommand.DummyCycles        = DUMMY_CLOCK_CYCLES_SRAM_READ;

  if (HAL_OSPI_Command(&OSPIPSRAMHandle, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return PSRAM_ERROR;
  }

  if (HAL_OSPI_Receive_IT(&OSPIPSRAMHandle, pData) != HAL_OK)
  {
    return PSRAM_ERROR;
  }

  return PSRAM_OK;

}


/**
  * @brief  Reads an amount of data from the PSRAM device in DMA mode.
  * @param  uwStartAddress: Read start address
  * @param  pData: Pointer to data to be read
  * @param  uwDataSize: Size of read data from the memory
  * @retval PSRAM status
  */
uint8_t BSP_PSRAM_ReadData_DMA(uint32_t uwStartAddress, uint8_t *pData, uint32_t uwDataSize)
{
  OSPI_RegularCmdTypeDef  sCommand = {0};

  sCommand.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
  sCommand.FlashId            = HAL_OSPI_FLASH_ID_1;
  sCommand.InstructionMode    = HAL_OSPI_INSTRUCTION_8_LINES;
  sCommand.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
  sCommand.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  sCommand.AddressMode        = HAL_OSPI_ADDRESS_8_LINES;
  sCommand.AddressSize        = HAL_OSPI_ADDRESS_32_BITS;
  sCommand.AddressDtrMode     = HAL_OSPI_ADDRESS_DTR_ENABLE;
  sCommand.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode           = HAL_OSPI_DATA_8_LINES;
  sCommand.DataDtrMode        = HAL_OSPI_DATA_DTR_ENABLE;
  sCommand.DQSMode            = HAL_OSPI_DQS_ENABLE;
  sCommand.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;


  /* 1- Reading Sequence ------------------------------------------------ */
  sCommand.Instruction        = READ_CMD_SRAM;
  sCommand.Address            = uwStartAddress;
  sCommand.NbData             = uwDataSize;
  sCommand.DummyCycles        = DUMMY_CLOCK_CYCLES_SRAM_READ;

  if (HAL_OSPI_Command(&OSPIPSRAMHandle, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return PSRAM_ERROR;
  }

  if (HAL_OSPI_Receive_DMA(&OSPIPSRAMHandle, pData) != HAL_OK)
  {
    return PSRAM_ERROR;
  }

  return PSRAM_OK;

}

/**
  * @brief  Writes an amount of data from the PSRAM device in polling mode.
  * @param  uwStartAddress: Write start address
  * @param  pData: Pointer to data to be written
  * @param  uwDataSize: Size of written data from the memory
  * @retval PSRAM status
  */
uint8_t BSP_PSRAM_WriteData(uint32_t uwStartAddress, uint8_t *pData, uint32_t uwDataSize)
{
  OSPI_RegularCmdTypeDef  sCommand = {0};

  sCommand.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
  sCommand.FlashId            = HAL_OSPI_FLASH_ID_1;
  sCommand.InstructionMode    = HAL_OSPI_INSTRUCTION_8_LINES;
  sCommand.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
  sCommand.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  sCommand.AddressMode        = HAL_OSPI_ADDRESS_8_LINES;
  sCommand.AddressSize        = HAL_OSPI_ADDRESS_32_BITS;
  sCommand.AddressDtrMode     = HAL_OSPI_ADDRESS_DTR_ENABLE;
  sCommand.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode           = HAL_OSPI_DATA_8_LINES;
  sCommand.DataDtrMode        = HAL_OSPI_DATA_DTR_ENABLE;
  sCommand.DQSMode            = HAL_OSPI_DQS_ENABLE;
  sCommand.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;

  /* 1- Writing Sequence ------------------------------------------------ */
  sCommand.Instruction        = WRITE_CMD_SRAM;
  sCommand.Address            = uwStartAddress;
  sCommand.NbData             = uwDataSize;
  sCommand.DummyCycles        = DUMMY_CLOCK_CYCLES_SRAM_WRITE;

  if (HAL_OSPI_Command(&OSPIPSRAMHandle, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return PSRAM_ERROR;
  }

  if (HAL_OSPI_Transmit(&OSPIPSRAMHandle, pData, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return PSRAM_ERROR;
  }

  return PSRAM_OK;
}


/**
  * @brief  Writes an amount of data from the PSRAM device in IT mode.
  * @param  uwStartAddress: Write start address
  * @param  pData: Pointer to data to be written
  * @param  uwDataSize: Size of written data from the memory
  * @retval PSRAM status
  */
uint8_t BSP_PSRAM_WriteData_IT(uint32_t uwStartAddress, uint8_t *pData, uint32_t uwDataSize)
{
  OSPI_RegularCmdTypeDef  sCommand = {0};

  sCommand.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
  sCommand.FlashId            = HAL_OSPI_FLASH_ID_1;
  sCommand.InstructionMode    = HAL_OSPI_INSTRUCTION_8_LINES;
  sCommand.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
  sCommand.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  sCommand.AddressMode        = HAL_OSPI_ADDRESS_8_LINES;
  sCommand.AddressSize        = HAL_OSPI_ADDRESS_32_BITS;
  sCommand.AddressDtrMode     = HAL_OSPI_ADDRESS_DTR_ENABLE;
  sCommand.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode           = HAL_OSPI_DATA_8_LINES;
  sCommand.DataDtrMode        = HAL_OSPI_DATA_DTR_ENABLE;
  sCommand.DQSMode            = HAL_OSPI_DQS_ENABLE;
  sCommand.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;

  /* 1- Writing Sequence ------------------------------------------------ */
  sCommand.Instruction        = WRITE_CMD_SRAM;
  sCommand.Address            = uwStartAddress;
  sCommand.NbData             = uwDataSize;
  sCommand.DummyCycles        = DUMMY_CLOCK_CYCLES_SRAM_WRITE;

  if (HAL_OSPI_Command(&OSPIPSRAMHandle, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return PSRAM_ERROR;
  }

  if (HAL_OSPI_Transmit_IT(&OSPIPSRAMHandle, pData) != HAL_OK)
  {
    return PSRAM_ERROR;
  }

  return PSRAM_OK;
}

/**
  * @brief  Writes an amount of data from the PSRAM device in DMA mode.
  * @param  uwStartAddress: Write start address
  * @param  pData: Pointer to data to be written
  * @param  uwDataSize: Size of written data from the memory
  * @retval PSRAM status
  */
uint8_t BSP_PSRAM_WriteData_DMA(uint32_t uwStartAddress, uint8_t *pData, uint32_t uwDataSize)
{
  OSPI_RegularCmdTypeDef  sCommand = {0};

  sCommand.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
  sCommand.FlashId            = HAL_OSPI_FLASH_ID_1;
  sCommand.InstructionMode    = HAL_OSPI_INSTRUCTION_8_LINES;
  sCommand.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
  sCommand.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  sCommand.AddressMode        = HAL_OSPI_ADDRESS_8_LINES;
  sCommand.AddressSize        = HAL_OSPI_ADDRESS_32_BITS;
  sCommand.AddressDtrMode     = HAL_OSPI_ADDRESS_DTR_ENABLE;
  sCommand.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode           = HAL_OSPI_DATA_8_LINES;
  sCommand.DataDtrMode        = HAL_OSPI_DATA_DTR_ENABLE;
  sCommand.DQSMode            = HAL_OSPI_DQS_ENABLE;
  sCommand.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;

  /* 1- Writing Sequence ------------------------------------------------ */
  sCommand.Instruction        = WRITE_CMD_SRAM;
  sCommand.Address            = uwStartAddress;
  sCommand.NbData             = uwDataSize;
  sCommand.DummyCycles        = DUMMY_CLOCK_CYCLES_SRAM_WRITE;

  if (HAL_OSPI_Command(&OSPIPSRAMHandle, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return PSRAM_ERROR;
  }

  if (HAL_OSPI_Transmit_DMA(&OSPIPSRAMHandle, pData) != HAL_OK)
  {
    return PSRAM_ERROR;
  }

  return PSRAM_OK;
}

/**
  * @brief  Initializes PSRAM MSP.
  * @param  hsram: PSRAM handle
  * @param  Params
  * @retval None
  */
static void PSRAM_MspInit(void)
{
  static DMA_HandleTypeDef dma_handle;
  GPIO_InitTypeDef GPIO_InitStruct;


  /* Enable DMAx clock */
  __PSRAM_DMAx_CLK_ENABLE();
  __HAL_RCC_DMAMUX1_CLK_ENABLE();

  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;

  __HAL_RCC_OSPIM_CLK_ENABLE();

  /* IOSV bit MUST be set to access GPIO port G[2:15] */
  __HAL_RCC_PWR_CLK_ENABLE();
  SET_BIT(PWR->CR2, PWR_CR2_IOSV);

    /* Enable the GPIO clocks */
    OSPI_P1_CS_GPIO_CLK_ENABLE();
    OSPI_P1_CLK_GPIO_CLK_ENABLE();
    OSPI_P1_NCLK_GPIO_CLK_ENABLE();
    OSPI_P1_D0_GPIO_CLK_ENABLE();
    OSPI_P1_D1_GPIO_CLK_ENABLE();
    OSPI_P1_D2_GPIO_CLK_ENABLE();
    OSPI_P1_D3_GPIO_CLK_ENABLE();
    OSPI_P1_D4_GPIO_CLK_ENABLE();
    OSPI_P1_D5_GPIO_CLK_ENABLE();
    OSPI_P1_D6_GPIO_CLK_ENABLE();
    OSPI_P1_D7_GPIO_CLK_ENABLE();
    OSPI_P1_DQS_GPIO_CLK_ENABLE();

    GPIO_InitStruct.Alternate = GPIO_AF10_OCTOSPIM_P1;

    /* Configure the OSPI_P1_CS pin */
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Pin  = OSPI_P1_CS_PIN;
    HAL_GPIO_Init(OSPI_P1_CS_GPIO_PORT, &GPIO_InitStruct);

    /* Configure the OSPI_P1_CLK pin */
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Pin  = OSPI_P1_CLK_PIN;
    HAL_GPIO_Init(OSPI_P1_CLK_GPIO_PORT, &GPIO_InitStruct);

    /* Configure the OSPI_P1_NCLK pin */
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Pin  = OSPI_P1_NCLK_PIN;
    HAL_GPIO_Init(OSPI_P1_NCLK_GPIO_PORT, &GPIO_InitStruct);

    /* Configure the OSPI_P1_D0 pin */
    GPIO_InitStruct.Pin  = OSPI_P1_D0_PIN;
    HAL_GPIO_Init(OSPI_P1_D0_GPIO_PORT, &GPIO_InitStruct);

    /* Configure the OSPI_P1_D1 pin */
    GPIO_InitStruct.Pin  = OSPI_P1_D1_PIN;
    HAL_GPIO_Init(OSPI_P1_D1_GPIO_PORT, &GPIO_InitStruct);

    /* Configure the OSPI_P1_D2 pin */
    GPIO_InitStruct.Pin  = OSPI_P1_D2_PIN;
    HAL_GPIO_Init(OSPI_P1_D2_GPIO_PORT, &GPIO_InitStruct);

    /* Configure the OSPI_P1_D3 pin */
    GPIO_InitStruct.Pin  = OSPI_P1_D3_PIN;
    HAL_GPIO_Init(OSPI_P1_D3_GPIO_PORT, &GPIO_InitStruct);

    /* Configure the OSPI_P1_D6 pin */
    GPIO_InitStruct.Pin  = OSPI_P1_D6_PIN;
    HAL_GPIO_Init(OSPI_P1_D6_GPIO_PORT, &GPIO_InitStruct);

    /* Configure the OSPI_P1_D7 pin */
    GPIO_InitStruct.Pin  = OSPI_P1_D7_PIN;
    HAL_GPIO_Init(OSPI_P1_D7_GPIO_PORT, &GPIO_InitStruct);

    /* Configure the OSPI_P1_D4 pin */
    GPIO_InitStruct.Pin  = OSPI_P1_D4_PIN;
    HAL_GPIO_Init(OSPI_P1_D4_GPIO_PORT, &GPIO_InitStruct);

    /* Configure the OSPI_P1_D5 pin */
    GPIO_InitStruct.Pin  = OSPI_P1_D5_PIN;
    HAL_GPIO_Init(OSPI_P1_D5_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Alternate = GPIO_AF3_OCTOSPIM_P1;

    /* Configure the OSPI_P1_DQS pin */
    GPIO_InitStruct.Pin  = OSPI_P1_DQS_PIN;
    HAL_GPIO_Init(OSPI_P1_DQS_GPIO_PORT, &GPIO_InitStruct);

  /* Enable the OctoSPI memory interface clock */
  __HAL_RCC_OSPI1_CLK_ENABLE();

  /* Reset the OctoSPI memory interface */
  __HAL_RCC_OSPI1_FORCE_RESET();
  __HAL_RCC_OSPI1_RELEASE_RESET();

  /* Enable and set OctoSPI interrupt to the lowest priority */
  HAL_NVIC_SetPriority(OCTOSPI1_IRQn, 0x0F, 0);
  HAL_NVIC_EnableIRQ(OCTOSPI1_IRQn);


  /* Configure common DMA parameters */
  dma_handle.Init.PeriphInc           = DMA_PINC_DISABLE;
  dma_handle.Init.MemInc              = DMA_MINC_ENABLE;
  dma_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  dma_handle.Init.MemDataAlignment    = DMA_PDATAALIGN_BYTE;
  dma_handle.Init.Mode                = DMA_NORMAL;
  dma_handle.Init.Priority            = DMA_PRIORITY_HIGH;
  /* Configure the OctoSPI DMA */
  dma_handle.Init.Request = DMA_REQUEST_OCTOSPI1;

  dma_handle.Instance = PSRAM_DMAx_INSTANCE;

  /* Deinitialize the Channel for new transfer */
  HAL_DMA_DeInit(&dma_handle);

    /* Initialize the DMA channel */
  __HAL_LINKDMA(&OSPIPSRAMHandle, hdma, dma_handle);

  /* Configure the DMA Channel */
  HAL_DMA_Init(&dma_handle);


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
static void PSRAM_MspDeInit(void)
{
  static DMA_HandleTypeDef dma_handle;

  /* Disable NVIC configuration for DMA interrupt */
  HAL_NVIC_DisableIRQ(PSRAM_DMAx_IRQn);

  /* Deinitialize the stream for new transfer */
  dma_handle.Instance = PSRAM_DMAx_INSTANCE;
  HAL_DMA_DeInit(&dma_handle);

  /* Deinitialize GPIOs */
  HAL_GPIO_DeInit(OSPI_P1_CS_GPIO_PORT, OSPI_P1_CS_PIN);
  HAL_GPIO_DeInit(OSPI_P1_CLK_GPIO_PORT, OSPI_P1_CLK_PIN);
  HAL_GPIO_DeInit(OSPI_P1_NCLK_GPIO_PORT, OSPI_P1_NCLK_PIN);
  HAL_GPIO_DeInit(OSPI_P1_D0_GPIO_PORT, OSPI_P1_D0_PIN);
  HAL_GPIO_DeInit(OSPI_P1_D1_GPIO_PORT, OSPI_P1_D1_PIN);
  HAL_GPIO_DeInit(OSPI_P1_D2_GPIO_PORT, OSPI_P1_D2_PIN);
  HAL_GPIO_DeInit(OSPI_P1_D3_GPIO_PORT, OSPI_P1_D3_PIN);
  HAL_GPIO_DeInit(OSPI_P1_D4_GPIO_PORT, OSPI_P1_D4_PIN);
  HAL_GPIO_DeInit(OSPI_P1_D5_GPIO_PORT, OSPI_P1_D5_PIN);
  HAL_GPIO_DeInit(OSPI_P1_D6_GPIO_PORT, OSPI_P1_D6_PIN);
  HAL_GPIO_DeInit(OSPI_P1_D7_GPIO_PORT, OSPI_P1_D7_PIN);
  HAL_GPIO_DeInit(OSPI_P1_DQS_GPIO_PORT, OSPI_P1_DQS_PIN);

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


