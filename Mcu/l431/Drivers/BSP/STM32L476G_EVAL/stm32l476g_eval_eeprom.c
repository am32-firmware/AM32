/**
  ******************************************************************************
  * @file    stm32l476g_eval_eeprom.c
  * @author  MCD Application Team
  * @brief   This file provides a set of functions needed to manage a M24LR64 
  *          I2C EEPROM memory, or a M95040-R SPI EEPROM memory.
  *          
  *          =================================================================== 
  *          Notes:
  *           - This driver is intended for STM32L4xx families devices only.
  *           - The I2C EEPROM memory (M24LR64) is available on separate daughter 
  *             board ANT7-M24LR-A, which is provided with the STM32L476G_EVAL 
  *             EVAL board.
  *          ===================================================================
  *              
  *          It implements a high level communication layer for read and write 
  *          from/to this memory. The needed STM32L4xx hardware resources (I2C,
  *          SPI and GPIO) are defined in stm32l476g_eval.h file,
  *          and the initialization is performed depending of EEPROMs
  *          in EEPROM_I2C_IO_Init() or EEPROM_SPI_IO_Init() functions
  *          declared in stm32l476g_eval.c file.
  *          You can easily tailor this driver to any other development board, 
  *          by just adapting the defines for hardware resources and 
  *          EEPROM_I2C_IO_Init() or EEPROM_SPI_IO_Init() functions. 
  *        
  *          @note In this driver, basic read and write functions
  *          (BSP_EEPROM_ReadBuffer() and BSP_EEPROM_WriteBuffer())
  *          use Polling mode to perform the data transfer to/from EEPROM memories.
  *     +-----------------------------------------------------------------+
  *     |               Pin assignment for M24LR64 EEPROM                 |
  *     +---------------------------------------+-----------+-------------+
  *     |  STM32L4xx I2C Pins                   |   EEPROM  |   Pin       |
  *     +---------------------------------------+-----------+-------------+
  *     | EEPROM_I2C_SDA_PIN/ SDA               |   SDA     |    1        |
  *     | EEPROM_I2C_SCL_PIN/ SCL               |   SCL     |    2        |
  *     | .                                     |   VDD     |    3 (1.8V) |
  *     | .                                     |   GND     |    4 (0 V)  |
  *     +---------------------------------------+-----------+-------------+
  *
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
#include "stm32l476g_eval_eeprom.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L476G_EVAL
  * @{
  */

/** @defgroup STM32L476G_EVAL_EEPROM STM32L476G_EVAL EEPROM
  * @brief      This file includes the I2C EEPROM driver
  *             of STM32L476G_EVAL-EVAL board.
  * @{
  */

/* Private variables ---------------------------------------------------------*/

/** @defgroup STM32L476G_EVAL_EEPROM_Private_Variables EEPROM Private Variables
  * @{
  */
__IO uint16_t  EEPROMAddress = 0;
__IO uint16_t  EEPROMPageSize = 0;
__IO uint16_t  EEPROMAddress2 = 0;
__IO uint16_t  EEPROMPageSize2 = 0;
__IO uint16_t  EEPROMDataRead = 0;
__IO uint8_t   EEPROMDataWrite = 0;

static EEPROM_DrvTypeDef *EEPROM_SelectedDevice = 0;
/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/

/** @defgroup STM32L476G_EVAL_EEPROM_Private_Functions EEPROM Private Functions
  * @{
  */
static uint32_t EEPROM_I2C_Init(void);
static uint32_t EEPROM_I2C_ReadBuffer(uint8_t* pBuffer, uint16_t ReadAddr, uint32_t* NumByteToRead);
static uint32_t EEPROM_I2C_WritePage(uint8_t* pBuffer, uint16_t WriteAddr, uint32_t* NumByteToWrite);
static uint32_t EEPROM_I2C_WaitEepromStandbyState(void);
/**
  * @}
  */
  
/** @defgroup STM32L476G_EVAL_EEPROM_Private_Types EEPROM Private Types
  * @{
  */
/* EEPROM I2C driver typedef */
EEPROM_DrvTypeDef EEPROM_I2C_Drv =
{
  EEPROM_I2C_Init,
  EEPROM_I2C_ReadBuffer,
  EEPROM_I2C_WritePage
};

/**
  * @}
  */

/* Exported functions ---------------------------------------------------------*/

/** @addtogroup STM32L476G_EVAL_EEPROM_Exported_Functions
  * @{
  */

/**
  * @brief  Initializes peripherals used by the EEPROM device selected.
  * @retval EEPROM_OK (0) if operation is correctly performed, else return value 
  *         different from EEPROM_OK (0)
  */
uint32_t BSP_EEPROM_Init(void)
{ 
  if(EEPROM_SelectedDevice->Init != 0)
  {
    return (EEPROM_SelectedDevice->Init());
  }
  else
  {
    return EEPROM_FAIL;
  }
}

/**
  * @brief  Select the EEPROM device to communicate.
  * @param  DeviceID: Specifies the EEPROM device to be selected. 
  *   This parameter can be one of following parameters:
  *     @arg BSP_EEPROM_M24LR64
  *     @arg BSP_EEPROM_M24LR64_A01
  *     @arg BSP_EEPROM_M24128
  * 
  * @retval EEPROM_OK (0) if operation is correctly performed, else return value 
  *         different from EEPROM_OK (0)
  */
void BSP_EEPROM_SelectDevice(uint8_t DeviceID)
{
  switch(DeviceID)
  {
  case BSP_EEPROM_M24LR64 :
    EEPROM_SelectedDevice = &EEPROM_I2C_Drv;
    break;
    
  default:
    break;
  }
}

/**
  * @brief  Reads a block of data from the EEPROM device selected.
  * @param  pBuffer : pointer to the buffer that receives the data read from 
  *         the EEPROM.
  * @param  ReadAddr : EEPROM's internal address to start reading from.
  * @param  NumByteToRead : pointer to the variable holding number of bytes to 
  *         be read from the EEPROM.
  * 
  *        @note The variable pointed by NumByteToRead is reset to 0 when all the 
  *              data are read from the EEPROM. Application should monitor this 
  *              variable in order know when the transfer is complete.
  * 
  * @retval EEPROM_OK (0) if operation is correctly performed, else return value 
  *         different from EEPROM_OK (0) or the timeout user callback.
  */
uint32_t BSP_EEPROM_ReadBuffer(uint8_t* pBuffer, uint16_t ReadAddr, uint32_t* NumByteToRead)
{
  if(EEPROM_SelectedDevice->ReadBuffer != 0)
  {
    return (EEPROM_SelectedDevice->ReadBuffer(pBuffer, ReadAddr, NumByteToRead));
  }
  else
  {
    return EEPROM_FAIL;
  }
}

/**
  * @brief  Writes buffer of data to the EEPROM device selected.
  * @param  pBuffer : pointer to the buffer  containing the data to be written 
  *         to the EEPROM.
  * @param  WriteAddr : EEPROM's internal address to write to.
  * @param  NumByteToWrite : number of bytes to write to the EEPROM.
  * @retval EEPROM_OK (0) if operation is correctly performed, else return value 
  *         different from EEPROM_OK (0) or the timeout user callback.
  */
uint32_t BSP_EEPROM_WriteBuffer(uint8_t* pBuffer, uint16_t WriteAddr, uint32_t NumByteToWrite)
{
  uint16_t numofpage = 0, numofsingle = 0, count = 0;
  uint16_t addr = 0;
  uint32_t dataindex = 0;
  uint32_t status = EEPROM_OK;

  addr = WriteAddr % EEPROMPageSize;
  count = EEPROMPageSize - addr;
  numofpage =  NumByteToWrite / EEPROMPageSize;
  numofsingle = NumByteToWrite % EEPROMPageSize;
 
  if(EEPROM_SelectedDevice->WritePage == 0)
  {
    return EEPROM_FAIL;
  }
  
  /*!< If WriteAddr is EEPROM_PAGESIZE aligned  */
  if(addr == 0) 
  {
    /*!< If NumByteToWrite < EEPROM_PAGESIZE */
    if(numofpage == 0) 
    {
      /* Store the number of data to be written */
      dataindex = numofsingle;
      /* Start writing data */
      status = EEPROM_SelectedDevice->WritePage(pBuffer, WriteAddr, (uint32_t*)(&dataindex));
      if (status != EEPROM_OK)
      {
        return status;
      }
    }
    /*!< If NumByteToWrite > EEPROM_PAGESIZE */
    else  
    {
      while(numofpage--)
      {
        /* Store the number of data to be written */
        dataindex = EEPROMPageSize;        
        status = EEPROM_SelectedDevice->WritePage(pBuffer, WriteAddr, (uint32_t*)(&dataindex));
        if (status != EEPROM_OK)
        {
          return status;
        }
        
        WriteAddr +=  EEPROMPageSize;
        pBuffer += EEPROMPageSize;
      }

      if(numofsingle!=0)
      {
        /* Store the number of data to be written */
        dataindex = numofsingle;          
        status = EEPROM_SelectedDevice->WritePage(pBuffer, WriteAddr, (uint32_t*)(&dataindex));
        if (status != EEPROM_OK)
        {
          return status;
        }
          }
        }
    }
  /*!< If WriteAddr is not EEPROM_PAGESIZE aligned  */
  else 
  {
    /*!< If NumByteToWrite < EEPROM_PAGESIZE */
    if(numofpage== 0) 
    {
      /*!< If the number of data to be written is more than the remaining space 
      in the current page: */
      if (NumByteToWrite > count)
      {
        /* Store the number of data to be written */
        dataindex = count;        
        /*!< Write the data contained in same page */
        status = EEPROM_SelectedDevice->WritePage(pBuffer, WriteAddr, (uint32_t*)(&dataindex));
        if (status != EEPROM_OK)
        {
          return status;
        }
        
        /* Store the number of data to be written */
        dataindex = (NumByteToWrite - count);          
        /*!< Write the remaining data in the following page */
        status = EEPROM_SelectedDevice->WritePage((uint8_t*)(pBuffer + count), (WriteAddr + count), (uint32_t*)(&dataindex));
        if (status != EEPROM_OK)
        {
          return status;
        }
      }      
      else      
      {
        /* Store the number of data to be written */
        dataindex = numofsingle;         
        status = EEPROM_SelectedDevice->WritePage(pBuffer, WriteAddr, (uint32_t*)(&dataindex));
        if (status != EEPROM_OK)
        {
          return status;
        }
        }
      }     
    /*!< If NumByteToWrite > EEPROM_PAGESIZE */
    else
    {
      NumByteToWrite -= count;
      numofpage =  NumByteToWrite / EEPROMPageSize;
      numofsingle = NumByteToWrite % EEPROMPageSize;
      
      if(count != 0)
      {  
        /* Store the number of data to be written */
        dataindex = count;         
        status = EEPROM_SelectedDevice->WritePage(pBuffer, WriteAddr, (uint32_t*)(&dataindex));
        if (status != EEPROM_OK)
        {
          return status;
        }
        WriteAddr += count;
        pBuffer += count;
      } 
      
      while(numofpage--)
      {
        /* Store the number of data to be written */
        dataindex = EEPROMPageSize;          
        status = EEPROM_SelectedDevice->WritePage(pBuffer, WriteAddr, (uint32_t*)(&dataindex));
        if (status != EEPROM_OK)
        {
          return status;
        }
        WriteAddr +=  EEPROMPageSize;
        pBuffer += EEPROMPageSize;  
      }
      if(numofsingle != 0)
      {
        /* Store the number of data to be written */
        dataindex = numofsingle;           
        status = EEPROM_SelectedDevice->WritePage(pBuffer, WriteAddr, (uint32_t*)(&dataindex));
        if (status != EEPROM_OK)
        {
          return status;
        }
      }
    }
  }  
  
  /* If all operations OK, return EEPROM_OK (0) */
  return EEPROM_OK;
}

/**
  * @brief  Basic management of the timeout situation.
  * @retval None.
  */
__weak void BSP_EEPROM_TIMEOUT_UserCallback(void)
{
}
/**
  * @}
  */

/** @addtogroup STM32L476G_EVAL_EEPROM_Private_Functions
  * @{
  */

/**
  * @brief  Initializes peripherals used by the I2C EEPROM driver.
  * @note There are 2 different versions of M24LR64 (A01 & A02).
  *             Then try to connect on 1st one (EEPROM_I2C_ADDRESS_A01) 
  *             and if problem, check the 2nd one (EEPROM_I2C_ADDRESS_A02)
  * @retval EEPROM_OK (0) if operation is correctly performed, else return value 
  *         different from EEPROM_OK (0)
  */
static uint32_t EEPROM_I2C_Init(void)
{
  EEPROM_I2C_IO_Init();

  /*Select the EEPROM address for M24LR64 A02 and check if OK*/
  EEPROMAddress = EEPROM_ADDRESS_M24LR64_A01;
  EEPROMPageSize = EEPROM_PAGESIZE_M24LR64;
  if (EEPROM_I2C_IO_IsDeviceReady(EEPROMAddress, EEPROM_MAX_TRIALS) != HAL_OK) 
  {
    /*Select the EEPROM address for M24LR64 A01 and check if OK*/
    EEPROMAddress = EEPROM_ADDRESS_M24LR64_A02;
    EEPROMPageSize = EEPROM_PAGESIZE_M24LR64;
    if (EEPROM_I2C_IO_IsDeviceReady(EEPROMAddress, EEPROM_MAX_TRIALS) != HAL_OK) 
    {
        return EEPROM_FAIL;
    }
  }

  return EEPROM_OK;
}

/**
  * @brief  Reads a block of data from the I2C EEPROM.
  * @param  pBuffer : pointer to the buffer that receives the data read from 
  *         the EEPROM.
  * @param  ReadAddr : EEPROM's internal address to start reading from.
  * @param  NumByteToRead : pointer to the variable holding number of bytes to 
  *         be read from the EEPROM.
  * 
  * @retval EEPROM_OK (0) if operation is correctly performed, else return value 
  *         different from EEPROM_OK (0) or the timeout user callback.
  */
static uint32_t EEPROM_I2C_ReadBuffer(uint8_t* pBuffer, uint16_t ReadAddr, uint32_t* NumByteToRead)
{  
  uint32_t buffersize = *NumByteToRead;
  
  if (EEPROM_I2C_IO_ReadData(EEPROMAddress, ReadAddr, pBuffer, buffersize) != HAL_OK)
  {
    return EEPROM_FAIL;
  }
  
  /* If all operations OK, return EEPROM_OK (0) */
  return EEPROM_OK;
}

/**
  * @brief  Writes more than one byte to the EEPROM with a single WRITE cycle.
  *
  * @note   The number of bytes (combined to write start address) must not 
  *         cross the EEPROM page boundary. This function can only write into
  *         the boundaries of an EEPROM page.
  *         This function doesn't check on boundaries condition (in this driver 
  *         the function BSP_EEPROM_WriteBuffer() which calls EEPROM_WritePage() is 
  *         responsible of checking on Page boundaries).
  * 
  * @param  pBuffer : pointer to the buffer containing the data to be written to 
  *         the EEPROM.
  * @param  WriteAddr : EEPROM's internal address to write to.
  * @param  NumByteToWrite : pointer to the variable holding number of bytes to 
  *         be written into the EEPROM. 
  * 
  *        @note The variable pointed by NumByteToWrite is reset to 0 when all the 
  *              data are written to the EEPROM. Application should monitor this 
  *              variable in order know when the transfer is complete.
  * 
  * 
  * @retval EEPROM_OK (0) if operation is correctly performed, else return value 
  *         different from EEPROM_OK (0) or the timeout user callback.
  */
static uint32_t EEPROM_I2C_WritePage(uint8_t* pBuffer, uint16_t WriteAddr, uint32_t* NumByteToWrite)
{ 
  uint32_t buffersize = *NumByteToWrite;

  if (EEPROM_I2C_IO_WriteData(EEPROMAddress, WriteAddr, pBuffer, buffersize) != HAL_OK)
  {
    return EEPROM_FAIL;
  }
  
  /* Wait for EEPROM Standby state */
  if (EEPROM_I2C_WaitEepromStandbyState() != EEPROM_OK) 
  {
    return EEPROM_FAIL;
  }
  
  return EEPROM_OK;
}

/**
  * @brief  Wait for EEPROM I2C Standby state.
  * 
  * @note  This function allows to wait and check that EEPROM has finished the 
  *        last operation. It is mostly used after Write operation: after receiving
  *        the buffer to be written, the EEPROM may need additional time to actually
  *        perform the write operation. During this time, it doesn't answer to
  *        I2C packets addressed to it. Once the write operation is complete
  *        the EEPROM responds to its address.
  * 
  * @retval EEPROM_OK (0) if operation is correctly performed, else return value 
  *         different from EEPROM_OK (0) or the timeout user callback.
  */
static uint32_t EEPROM_I2C_WaitEepromStandbyState(void)  
{
    /* Check if the maximum allowed number of trials has bee reached */
  if (EEPROM_I2C_IO_IsDeviceReady(EEPROMAddress, EEPROM_MAX_TRIALS) != HAL_OK)
  {
    /* If the maximum number of trials has been reached, exit the function */
    BSP_EEPROM_TIMEOUT_UserCallback();
    return EEPROM_TIMEOUT;
  }
  return EEPROM_OK;
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



