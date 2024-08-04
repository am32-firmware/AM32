/**
  ******************************************************************************
  * @file    st25dv.c 
  * @author  MMY Application Team
  * @brief   This file provides set of driver functions to manage communication 
  *          between BSP and ST25DV chip.
  ******************************************************************************
 * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "st25dv.h"


/** @addtogroup BSP
  * @{
  */

/** @defgroup ST25DV ST25DV driver
  * @brief    This module implements the functions to drive the ST25DV NFC dynamic tag.
  * @details  As recommended by the STM32 Cube methodology, this driver provides a standard structure to expose the NFC tag standard API.\n
  *           It also provides an extended API through its extended driver structure.\n
  *           To be usable on any MCU, this driver calls several IOBus functions.
  *           The IOBus functions are implemented outside this driver, and are in charge of accessing the MCU peripherals used for the communication with the tag.
  * @{
  */

/* External variables --------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/

  
/* Private macros ------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static int32_t ReadRegWrap(void *Handle, uint16_t Reg, uint8_t *pData, uint16_t Length);
static int32_t WriteRegWrap(void *Handle, uint16_t Reg, const uint8_t *pData, uint16_t Length);
static int32_t ST25DV_Init( ST25DV_Object_t* );
static int32_t ST25DV_ReadID(ST25DV_Object_t* pObj,  uint8_t * const pICRef );
static int32_t ST25DV_IsDeviceReady(ST25DV_Object_t* pObj,  const uint32_t Trials );
static int32_t ST25DV_GetGPOStatus(ST25DV_Object_t* pObj,  uint16_t * const pGPOStatus );
static int32_t ST25DV_ConfigureGPO(ST25DV_Object_t* pObj,  const uint16_t ITConf );
static int32_t ST25DV_ReadData(ST25DV_Object_t* pObj,  uint8_t * const pData, const uint16_t TarAddr, const uint16_t NbByte );
static int32_t ST25DV_WriteData(ST25DV_Object_t* pObj,  const uint8_t * const pData, const uint16_t TarAddr, const uint16_t NbByte );

/* Global variables ---------------------------------------------------------*/
/**
  * @brief    Standard NFC tag driver API for the ST25DV.
  * @details  Provides a generic way to access the ST25DV implementation of the NFC tag standard driver functions. 
  */
ST25DV_Drv_t St25Dv_Drv =
{
  ST25DV_Init,
  ST25DV_ReadID,
  ST25DV_IsDeviceReady,
  ST25DV_GetGPOStatus,
  ST25DV_ConfigureGPO,
  ST25DV_ReadData,
  ST25DV_WriteData,
};


/* Public functions ---------------------------------------------------------*/

/**
 * @brief  Register Component Bus IO operations
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t ST25DV_RegisterBusIO (ST25DV_Object_t* pObj, ST25DV_IO_t *pIO)
{
  int32_t ret = ST25DV_OK;

  if (pObj == NULL)
  {
    ret = ST25DV_ERROR;
  }
  else
  {
    pObj->IO.Init        = pIO->Init;
    pObj->IO.DeInit      = pIO->DeInit;
    pObj->IO.Write       = pIO->Write;
    pObj->IO.Read        = pIO->Read;
    pObj->IO.IsReady     = pIO->IsReady;
    pObj->IO.GetTick     = pIO->GetTick;

    pObj->Ctx.ReadReg  = ReadRegWrap;
    pObj->Ctx.WriteReg = WriteRegWrap;
    pObj->Ctx.handle   = pObj;

    if (pObj->IO.Init == NULL)
    {
      ret = ST25DV_ERROR;
    }
    else
    {
      if (pObj->IO.Init() != 0)
      {
        ret = ST25DV_ERROR;
      }
    }
  }

  return ret;
}

/**
  * @brief  ST25DV nfctag Initialization.
  * @param  pObj the device pObj
  * @retval Component error status.
  */
static int32_t ST25DV_Init( ST25DV_Object_t *pObj )
{
  int32_t status = ST25DV_OK;
  if (pObj->IsInitialized == 0U)
  {
    uint8_t nfctag_id;
    ST25DV_ReadID(pObj,&nfctag_id);
    if( (nfctag_id != I_AM_ST25DV04) && (nfctag_id != I_AM_ST25DV64) )
    {
      status = ST25DV_ERROR;
    } else {
      pObj->IsInitialized = 1U;
    }
  }

  return status;
}


/**
  * @brief  Reads the ST25DV ID.
  * @param  pObj the device pObj
  * @param  pICRef Pointeron a uint8_t used to return the ST25DV ID.
  * @retval Component error status.
  */
static int32_t ST25DV_ReadID(ST25DV_Object_t* pObj, uint8_t * const pICRef )
{
  /* Read ICRef on device */
  return st25dv_get_icref(&(pObj->Ctx), pICRef);
}

/**
  * @brief    Checks the ST25DV availability.
  * @param pObj the device pObj
  * @details  The ST25DV I2C is NACKed when a RF communication is on-going.
  *           This function determines if the ST25DV is ready to answer an I2C request. 
  * @param    Trials Max number of tentative.
  * @retval   Component error status.
  */
static int32_t ST25DV_IsDeviceReady(ST25DV_Object_t* pObj,  const uint32_t Trials )
{
  /* Test communication with device */
  return pObj->IO.IsReady(ST25DV_ADDR_DATA_I2C, Trials );
}

/**
  * @brief  Reads the ST25DV GPO configuration.
  * @param  pObj the device pObj
  * @param  pGPOStatus  Pointer on a uint16_t used to return the current GPO consiguration, as:
  *                     - RFUSERSTATE = 0x01
  *                     - RFBUSY = 0x02
  *                     - RFINTERRUPT = 0x04
  *                     - FIELDFALLING = 0x08
  *                     - FIELDRISING = 0x10
  *                     - RFPUTMSG = 0x20
  *                     - RFGETMSG = 0x40
  *                     - RFWRITE = 0x80
  *
  * @retval   Component error status.
  */
static int32_t ST25DV_GetGPOStatus(ST25DV_Object_t* pObj,  uint16_t * const pGPOStatus )
{
  uint8_t reg_value;
  int32_t status;
  
  /* Read value of GPO register */
  status = st25dv_get_gpo_all(&(pObj->Ctx),  &reg_value);
  if( status == ST25DV_OK )
  {
    /* Extract GPO configuration */
    *pGPOStatus = (uint16_t)reg_value;
  }
  return status;
}

/**
  * @brief    Configures the ST25DV GPO.
  * @details  Needs the I2C Password presentation to be effective.
  * @param    pObj the device pObj
  * @param    ITConf Provides the GPO configuration to apply:
  *           - RFUSERSTATE = 0x01
  *           - RFBUSY = 0x02
  *           - RFINTERRUPT = 0x04
  *           - FIELDFALLING = 0x08
  *           - FIELDRISING = 0x10
  *           - RFPUTMSG = 0x20
  *           - RFGETMSG = 0x40
  *           - RFWRITE = 0x80
  *
  * @retval   Component error status.
  */
static int32_t ST25DV_ConfigureGPO(ST25DV_Object_t* pObj,  const uint16_t ITConf )
{
  /* Write GPO configuration to register */
  return st25dv_set_gpo_all( &(pObj->Ctx),  (uint8_t *)&ITConf);
}

/**
  * @brief  Reads N bytes of Data, starting from the specified I2C address.
  * @param  pObj the device pObj
  * @param  pData   Pointer used to return the read data.
  * @param  TarAddr I2C data memory address to read.
  * @param  NbByte  Number of bytes to be read.
  * @retval Component error status.
  */
static int32_t ST25DV_ReadData(ST25DV_Object_t* pObj,  uint8_t * const pData, const uint16_t TarAddr, const uint16_t NbByte )
{
  /* Read Data in user memory */
  return pObj->IO.Read(ST25DV_ADDR_DATA_I2C, TarAddr, pData, NbByte );
}

/**
  * @brief  Writes N bytes of Data starting from the specified I2C Address.
  * @param  pObj the device pObj
  * @param  pData   Pointer on the data to be written.
  * @param  TarAddr I2C data memory address to be written.
  * @param  NbByte  Number of bytes to be written.
  * @retval Component error status.
  */
static int32_t ST25DV_WriteData(ST25DV_Object_t* pObj,  const uint8_t * const pData, const uint16_t TarAddr, const uint16_t NbByte )
{ 
  int32_t ret;
  uint16_t split_data_nb;
  const uint8_t *pdata_index = (const uint8_t *)pData;
  uint16_t bytes_to_write = NbByte;
  uint16_t mem_addr = TarAddr;
  
  /* ST25DV can write a maximum of 256 bytes in EEPROM per i2c communication */
  do
  {
    /* Split write if data to write is superior of max write bytes for ST25DV */
    if( bytes_to_write > ST25DV_MAX_WRITE_BYTE )
    {
      /* DataSize higher than max page write, copy data by page */
      split_data_nb = (uint16_t)ST25DV_MAX_WRITE_BYTE;
    }
    else
    {
      /* DataSize lower or equal to max page write, copy only last bytes */
      split_data_nb = bytes_to_write;
    }
    /* Write split_data_nb bytes in memory */
    ret = pObj->IO.Write( ST25DV_ADDR_DATA_I2C, mem_addr, pdata_index, split_data_nb);

    if( ret == ST25DV_OK )
    {
      int32_t pollstatus;
      /* Poll until EEPROM is available */
      uint32_t tickstart = pObj->IO.GetTick();
      /* Wait until ST25DV is ready or timeout occurs */
      do
      {
        pollstatus = pObj->IO.IsReady( ST25DV_ADDR_DATA_I2C, 1 );
      } while( ( (uint32_t)((int32_t)pObj->IO.GetTick() - (int32_t)tickstart) < ST25DV_WRITE_TIMEOUT) && (pollstatus != ST25DV_OK) );
      
      if( pollstatus != ST25DV_OK )
      {
        ret = ST25DV_TIMEOUT;
      }
    }

    /* update index, dest address, size for next write */
    pdata_index += split_data_nb;
    mem_addr += split_data_nb;
    bytes_to_write -= split_data_nb;
  }
  while( ( bytes_to_write > 0 ) && ( ret == ST25DV_OK ) );
  
  return ret;
}

/**
  * @brief  Reads the ST25DV IC Revision.
  * @param  pObj the device pObj
  * @param  pICRev Pointer on the uint8_t used to return the ST25DV IC Revision number.
  * @retval Component error status.
  */
int32_t ST25DV_ReadICRev(ST25DV_Object_t* pObj,  uint8_t * const pICRev )
{
  /* Read ICRev on device */
  return st25dv_get_icrev(&(pObj->Ctx), pICRev);
}


/**
  * @brief  Reads the ST25DV ITtime duration for the GPO pulses.
  * @param  pObj the device pObj
  * @param  pITtime Pointer used to return the coefficient for the GPO Pulse duration (Pulse duration = 302,06 us - ITtime * 512 / fc).
  * @retval Component error status.
  */
int32_t ST25DV_ReadITPulse(ST25DV_Object_t* pObj, ST25DV_PULSE_DURATION * const pITtime )
{
  uint8_t reg_value;
  int32_t status;
  
  /* Read ITtime register value */
  status = st25dv_get_ittime_delay( &(pObj->Ctx), &reg_value);
  if( status == ST25DV_OK )
  {  
    /* Extract delay coefficient value */
    *pITtime = (ST25DV_PULSE_DURATION)reg_value;
  }
  return status;
}

/**
  * @brief    Configures the ST25DV ITtime duration for the GPO pulse.
  * @details  Needs the I2C Password presentation to be effective.
  * @param    pObj the device pObj
  * @param    ITtime Coefficient for the Pulse duration to be written (Pulse duration = 302,06 us - ITtime * 512 / fc)
  * @retval   Component error status.
  */
int32_t ST25DV_WriteITPulse( ST25DV_Object_t* pObj, const ST25DV_PULSE_DURATION ITtime )
{
  uint8_t reg_value;
  
  /* prepare data to write */
  reg_value = (uint8_t)ITtime;
  
  /* Write value for ITtime register */
  return st25dv_set_ittime_delay( &(pObj->Ctx), &reg_value );
}


/**
  * @brief  Reads N bytes from Registers, starting at the specified I2C address.
  * @param  pObj the device pObj
  * @param  pData   Pointer used to return the read data.
  * @param  TarAddr I2C memory address to be read.
  * @param  NbByte  Number of bytes to be read.
  * @retval Component error status.
  */
int32_t ST25DV_ReadRegister(ST25DV_Object_t* pObj,  uint8_t * const pData, const uint16_t TarAddr, const uint16_t NbByte )
{  
  /* Read Data in system memory */
  return pObj->IO.Read( ST25DV_ADDR_SYST_I2C, TarAddr, pData,  NbByte );
}

/**
  * @brief    Writes N bytes to the specified register.
  * @param    pObj the device pObj
  * @details  Needs the I2C Password presentation to be effective.
  * @param    pData   Pointer on the data to be written.
  * @param    TarAddr I2C register address to written.
  * @param    NbByte  Number of bytes to be written.
  * @retval   Component error status.
*/
int32_t ST25DV_WriteRegister(ST25DV_Object_t* pObj,  const uint8_t * const pData, const uint16_t TarAddr, const uint16_t NbByte )
{ 
  int32_t ret;
  uint8_t split_data_nb;
  uint16_t bytes_to_write = NbByte;
  uint16_t mem_addr = TarAddr;
  const uint8_t *pdata_index = (const uint8_t *)pData;
  
  /* ST25DV can write a maximum of 256 bytes in EEPROM per i2c communication */
  do
  {
    /* Split write if data to write is superior of max write bytes for ST25DV */
    if( bytes_to_write > ST25DV_MAX_WRITE_BYTE )
    {
      /* DataSize higher than max page write, copy data by page */
      split_data_nb = (uint8_t)ST25DV_MAX_WRITE_BYTE;
    }
    else
    {
      /* DataSize lower or equal to max page write, copy only last bytes */
      split_data_nb = bytes_to_write;
    }
    /* Write split_data_nb bytes in register */
    ret = pObj->IO.Write( ST25DV_ADDR_SYST_I2C, mem_addr, pdata_index,  split_data_nb);
    if( ret == ST25DV_OK )
    {
      int32_t pollstatus;
      /* Poll until EEPROM is available */
      uint32_t tickstart = pObj->IO.GetTick();
      /* Wait until ST25DV is ready or timeout occurs */
      do
      {
        pollstatus = pObj->IO.IsReady( ST25DV_ADDR_DATA_I2C, 1 );
      } while( ( (uint32_t)((int32_t)pObj->IO.GetTick() - (int32_t)tickstart) < ST25DV_WRITE_TIMEOUT) && (pollstatus != ST25DV_OK) );
      
      if( pollstatus != ST25DV_OK )
      {
        ret = ST25DV_TIMEOUT;
      }
    }

    /* update index, dest address, size for next write */
    pdata_index += split_data_nb;
    mem_addr += split_data_nb;
    bytes_to_write -= split_data_nb;
  }
  while( ( bytes_to_write > 0 ) && ( ret == ST25DV_OK ) );
  
  return ret;
}

/**
  * @brief  Reads the ST25DV UID.
  * @param  pObj the device pObj
  * @param  pUid Pointer used to return the ST25DV UID value.
  * @retval Component error status.
  */
int32_t ST25DV_ReadUID(ST25DV_Object_t* pObj,  ST25DV_UID * const pUid )
{
  uint8_t reg_value[8];
  uint8_t i;
  int32_t status;
  
  /* Read value of UID registers */
  status = st25dv_get_uid( &(pObj->Ctx), reg_value);
  if( status == ST25DV_OK )
  {
    /* Store information in 2 WORD */
    pUid->MsbUid = 0;
  
    for( i = 0; i < 4; i++ )
    {
      pUid->MsbUid = (pUid->MsbUid << 8) | reg_value[7 - i];
    }
  
    pUid->LsbUid = 0;
  
    for( i = 0; i < 4; i++ )
    {
      pUid->LsbUid = (pUid->LsbUid << 8) | reg_value[3 - i];
    }
  }
  return status;
}

/**
  * @brief  Reads the ST25DV DSFID.
  * @param  pObj the device pObj
  * @param  pDsfid Pointer used to return the ST25DV DSFID value.
  * @retval Component error status.
  */
int32_t ST25DV_ReadDSFID(ST25DV_Object_t* pObj,  uint8_t * const pDsfid )
{
  /* Read DSFID register */
  return st25dv_get_dsfid(&(pObj->Ctx), pDsfid);
}

/**
  * @brief  Reads the ST25DV DSFID RF Lock state.
  * @param  pObj the device pObj
  * @param  pLockDsfid Pointer on a ST25DV_LOCK_STATUS used to return the DSFID lock state.
  * @retval Component error status.
  */
int32_t ST25DV_ReadDsfidRFProtection(ST25DV_Object_t* pObj,  ST25DV_LOCK_STATUS * const pLockDsfid )
{
  uint8_t reg_value;
  int32_t status;
  
  /* Read register */
  status = st25dv_get_lockdsfid(&(pObj->Ctx), &reg_value );
  if( status == ST25DV_OK )
  {
  
    /* Extract Lock Status */
    if( reg_value == 0 )
    {
      *pLockDsfid = ST25DV_UNLOCKED;
    }
    else
    {
      *pLockDsfid = ST25DV_LOCKED;
    }
  }
  return status;
}

/**
  * @brief  Reads the ST25DV AFI.
  * @param  pObj the device pObj
  * @param  pAfi Pointer used to return the ST25DV AFI value.
  * @retval Component error status.
  */
int32_t ST25DV_ReadAFI(ST25DV_Object_t* pObj,  uint8_t * const pAfi )
{
  /* Read AFI register */
  return st25dv_get_afi(&(pObj->Ctx), pAfi);
}

/**
  * @brief  Reads the AFI RF Lock state.
  * @param  pObj the device pObj
  * @param  pLockAfi Pointer on a ST25DV_LOCK_STATUS used to return the ASFID lock state.
  * @retval Component error status.
  */
int32_t ST25DV_ReadAfiRFProtection(ST25DV_Object_t* pObj,  ST25DV_LOCK_STATUS * const pLockAfi )
{
  uint8_t reg_value;
  int32_t status;
  
  /* Read register */
  status = st25dv_get_lockafi( &(pObj->Ctx), &reg_value);
  if( status == ST25DV_OK )
  {
 
    /* Extract Lock Status */
    if( reg_value == 0 )
    {
      *pLockAfi = ST25DV_UNLOCKED;
    }
    else
    {
      *pLockAfi = ST25DV_LOCKED;
    }
  }
  return status;
}

/**
  * @brief  Reads the I2C Protected Area state.
  * @param  pObj the device pObj
  * @param  pProtZone Pointer on a ST25DV_I2C_PROT_ZONE structure used to return the Protected Area state.
  * @retval Component error status.
  */
int32_t ST25DV_ReadI2CProtectZone(ST25DV_Object_t* pObj,  ST25DV_I2C_PROT_ZONE * const pProtZone )
{
  uint8_t reg_value;
  int32_t status;
  
  /* Read value of I2c Protected Zone register */
  status = st25dv_get_i2css_all( &(pObj->Ctx), &reg_value);
  if( status == ST25DV_OK )
  {
  
    /* Dispatch information to corresponding struct member */
    pProtZone->ProtectZone1 = (ST25DV_PROTECTION_CONF)( (reg_value & ST25DV_I2CSS_PZ1_MASK) >> ST25DV_I2CSS_PZ1_SHIFT );
    pProtZone->ProtectZone2 = (ST25DV_PROTECTION_CONF)( (reg_value & ST25DV_I2CSS_PZ2_MASK) >> ST25DV_I2CSS_PZ2_SHIFT );
    pProtZone->ProtectZone3 = (ST25DV_PROTECTION_CONF)( (reg_value & ST25DV_I2CSS_PZ3_MASK) >> ST25DV_I2CSS_PZ3_SHIFT );
    pProtZone->ProtectZone4 = (ST25DV_PROTECTION_CONF)( (reg_value & ST25DV_I2CSS_PZ4_MASK) >> ST25DV_I2CSS_PZ4_SHIFT );
  }
  return status;
}

/**
  * @brief    Sets the I2C write-protected state to an EEPROM Area.
  * @details  Needs the I2C Password presentation to be effective.
  * @param    pObj the device pObj
  * @param    Zone                ST25DV_PROTECTION_ZONE value coresponding to the area to protect.
  * @param    ReadWriteProtection ST25DV_PROTECTION_CONF value corresponding to the protection to be set.
  * @retval   Component error status.
*/
int32_t ST25DV_WriteI2CProtectZonex(ST25DV_Object_t* pObj,  const ST25DV_PROTECTION_ZONE Zone, const ST25DV_PROTECTION_CONF ReadWriteProtection )
{
  int32_t status;
  uint8_t reg_value = 0;
   
  /* Compute and update new i2c Zone Security Status */
  switch( Zone )
  {
    case ST25DV_PROT_ZONE1:
      /* Read protection is not allowed for Zone 1 */
      reg_value = (ReadWriteProtection & 0x01);
      status = st25dv_set_i2css_pz1( &(pObj->Ctx), &reg_value);
      break;
    case ST25DV_PROT_ZONE2:
      reg_value = ReadWriteProtection;
      status = st25dv_set_i2css_pz2( &(pObj->Ctx), &reg_value);
      break;
    case ST25DV_PROT_ZONE3:
      reg_value = ReadWriteProtection;
      status = st25dv_set_i2css_pz3( &(pObj->Ctx), &reg_value);
      break;
    case ST25DV_PROT_ZONE4:
      reg_value = ReadWriteProtection;
      status = st25dv_set_i2css_pz4( &(pObj->Ctx), &reg_value);
      break;
    
    default:
      return ST25DV_ERROR;
  }
  
  /* Write I2CZSS register */
  return status;
}

/**
  * @brief  Reads the CCile protection state.
  * @param  pObj the device pObj
  * @param  pLockCCFile Pointer on a ST25DV_LOCK_CCFILE value corresponding to the lock state of the CCFile.
  * @retval Component error status.
  */
int32_t ST25DV_ReadLockCCFile(ST25DV_Object_t* pObj,  ST25DV_LOCK_CCFILE * const pLockCCFile )
{
  uint8_t reg_value;
  int32_t status;
  
  /* Get actual LOCKCCFILE register value */
  status = st25dv_get_lockccfile_all( &(pObj->Ctx), &reg_value);
  if( status == ST25DV_OK )
  {
  
    /* Extract CCFile block information */
    if( reg_value & ST25DV_LOCKCCFILE_BLCK0_MASK )
    {
      pLockCCFile->LckBck0 = ST25DV_LOCKED;
    }
    else
    {
      pLockCCFile->LckBck0 = ST25DV_UNLOCKED;
    }
  
    if( reg_value & ST25DV_LOCKCCFILE_BLCK1_MASK )
    {
      pLockCCFile->LckBck1 = ST25DV_LOCKED;
    }
    else
    {
      pLockCCFile->LckBck1 = ST25DV_UNLOCKED;
    }
  }
  return status;
}

/**
  * @brief  Locks the CCile to prevent any RF write access.
  * @details  Needs the I2C Password presentation to be effective.
  * @param  pObj the device pObj
  * @param  NbBlockCCFile ST25DV_CCFILE_BLOCK value corresponding to the number of blocks to be locked.
  * @param  LockCCFile    ST25DV_LOCK_CCFILE value corresponding to the lock state to apply on the CCFile.
  * @retval Component error status.
  */
int32_t ST25DV_WriteLockCCFile(ST25DV_Object_t* pObj,  const ST25DV_CCFILE_BLOCK NbBlockCCFile, const ST25DV_LOCK_STATUS LockCCFile )
{
  uint8_t reg_value;
  
  /* Configure value to write on register */
  if( NbBlockCCFile == ST25DV_CCFILE_1BLCK )
  {
    if( LockCCFile == ST25DV_LOCKED )
    {
      reg_value = ST25DV_LOCKCCFILE_BLCK0_MASK;
    }
    else
    {
      reg_value = 0x00;
    }
  }
  else
  {
    if( LockCCFile == ST25DV_LOCKED )
    {
      reg_value = ST25DV_LOCKCCFILE_BLCK0_MASK | ST25DV_LOCKCCFILE_BLCK1_MASK;
    }
    else
    {
      reg_value = 0x00;
    }
  }
  
  /* Write LOCKCCFILE register */
  return st25dv_set_lockccfile_all( &(pObj->Ctx), &reg_value);
}

/**
  * @brief  Reads the Cfg registers protection.
  * @param  pObj the device pObj
  * @param  pLockCfg Pointer on a ST25DV_LOCK_STATUS value corresponding to the Cfg registers lock state.
  * @retval Component error status.
  */
int32_t ST25DV_ReadLockCFG(ST25DV_Object_t* pObj,  ST25DV_LOCK_STATUS * const pLockCfg )
{
  uint8_t reg_value;
  int32_t status;
  
  /* Get actual LOCKCCFILE register value */
  status = st25dv_get_lockcfg_b0(&(pObj->Ctx), &reg_value);
  if( status == ST25DV_OK )
  {

    /* Extract LOCKCFG block information */
    if( reg_value )
    {
      *pLockCfg = ST25DV_LOCKED;
    }
    else
    {
      *pLockCfg = ST25DV_UNLOCKED;
    }
  }
  return status;  
}

/**
  * @brief  Lock/Unlock the Cfg registers, to prevent any RF write access.
  * @details  Needs the I2C Password presentation to be effective.
  * @param  pObj the device pObj
  * @param  LockCfg ST25DV_LOCK_STATUS value corresponding to the lock state to be written.
  * @retval Component error status.
  */
int32_t ST25DV_WriteLockCFG(ST25DV_Object_t* pObj,  const ST25DV_LOCK_STATUS LockCfg )
{
  uint8_t reg_value;
  
  /* Configure value to write on register */
  reg_value = (uint8_t)LockCfg;
  
  /* Write LOCKCFG register */
  return st25dv_set_lockcfg_b0(&(pObj->Ctx), &reg_value );
}

/**
  * @brief  Presents I2C password, to authorize the I2C writes to protected areas.
  * @param  pObj the device pObj
  * @param  PassWord Password value on 32bits
  * @retval Component error status.
  */
int32_t ST25DV_PresentI2CPassword(ST25DV_Object_t* pObj,  const ST25DV_PASSWD PassWord )
{
  uint8_t ai2c_message[17] = {0};
  uint8_t i;
  
  /* Build I2C Message with Password + Validation code 0x09 + Password */
  ai2c_message[8] = 0x09;
  for( i = 0; i < 4; i++ )
  {
    ai2c_message[i] = ( PassWord.MsbPasswd >> ( (3 - i) * 8) ) & 0xFF;
    ai2c_message[i + 4] = ( PassWord.LsbPasswd >> ( (3 - i) * 8) ) & 0xFF;
    ai2c_message[i + 9] = ai2c_message[i];
    ai2c_message[i + 13] = ai2c_message[i + 4];
  };
  
  /* Present password to ST25DV */
  return ST25DV_WriteRegister(pObj, ai2c_message, ST25DV_I2CPASSWD_REG, 17 );
}

/**
  * @brief  Writes a new I2C password.
  * @details  Needs the I2C Password presentation to be effective.
  * @param  pObj the device pObj
  * @param  PassWord New I2C PassWord value on 32bits.
  * @retval Component error status.
  */
int32_t ST25DV_WriteI2CPassword( ST25DV_Object_t* pObj, const ST25DV_PASSWD PassWord )
{
  uint8_t ai2c_message[17] = {0};
  uint8_t i;
  
  /* Build I2C Message with Password + Validation code 0x07 + Password */
  ai2c_message[8] = 0x07;

  for( i = 0; i < 4; i++ )
  {
    ai2c_message[i] = ( PassWord.MsbPasswd >> ( (3 - i) * 8) ) & 0xFF;
    ai2c_message[i + 4] = ( PassWord.LsbPasswd >> ( (3 - i) * 8) ) & 0xFF;
    ai2c_message[i + 9] = ai2c_message[i];
    ai2c_message[i + 13] = ai2c_message[i + 4];
  };
  
  /* Write new password in I2CPASSWD register */
  return ST25DV_WriteRegister(pObj, ai2c_message, ST25DV_I2CPASSWD_REG, 17 );
}

/**
  * @brief  Reads the RF Zone Security Status (defining the allowed RF accesses).
  * @param  pObj the device pObj
  * @param  Zone        ST25DV_PROTECTION_ZONE value coresponding to the protected area.
  * @param  pRfprotZone Pointer on a ST25DV_RF_PROT_ZONE value corresponding to the area protection state.
  * @retval Component error status.
  */
int32_t ST25DV_ReadRFZxSS( ST25DV_Object_t* pObj, const ST25DV_PROTECTION_ZONE Zone, ST25DV_RF_PROT_ZONE * const pRfprotZone )
{
  uint8_t reg_value;
  int32_t status;
  
  /* Read actual value of Sector Security Status register */
  switch( Zone )
  {
    case ST25DV_PROT_ZONE1:
      status = st25dv_get_rfa1ss_all(&(pObj->Ctx), &reg_value);
      break;
    case ST25DV_PROT_ZONE2:
      status = st25dv_get_rfa2ss_all(&(pObj->Ctx), &reg_value);
      break;
    case ST25DV_PROT_ZONE3:
      status = st25dv_get_rfa3ss_all(&(pObj->Ctx), &reg_value);
      break;
    case ST25DV_PROT_ZONE4:
      status = st25dv_get_rfa4ss_all(&(pObj->Ctx), &reg_value);
      break;
    default:
      status = ST25DV_ERROR;
  }
  
  if( status == ST25DV_OK )
  {
    /* Extract Sector Security Status configuration */
    pRfprotZone->PasswdCtrl = (ST25DV_PASSWD_PROT_STATUS)((reg_value & ST25DV_RFA1SS_PWDCTRL_MASK) >> ST25DV_RFA1SS_PWDCTRL_SHIFT);
    pRfprotZone->RWprotection = (ST25DV_PROTECTION_CONF)((reg_value & ST25DV_RFA1SS_RWPROT_MASK) >> ST25DV_RFA1SS_RWPROT_SHIFT);
  }
  return status;
}

/**
  * @brief  Writes the RF Zone Security Status (defining the allowed RF accesses)
  * @details  Needs the I2C Password presentation to be effective.
  * @param  pObj the device pObj
  * @param  Zone        ST25DV_PROTECTION_ZONE value corresponding to the area on which to set the RF protection.
  * @param  RfProtZone  Pointer on a ST25DV_RF_PROT_ZONE value defininf the protection to be set on the area.
  * @retval Component error status.
  */
int32_t ST25DV_WriteRFZxSS( ST25DV_Object_t* pObj, const ST25DV_PROTECTION_ZONE Zone, const ST25DV_RF_PROT_ZONE RfProtZone )
{
  uint8_t reg_value;
  int32_t status;
  

  /* Update Sector Security Status */
  reg_value = (RfProtZone.RWprotection << ST25DV_RFA1SS_RWPROT_SHIFT) & ST25DV_RFA1SS_RWPROT_MASK;
  reg_value |= ((RfProtZone.PasswdCtrl << ST25DV_RFA1SS_PWDCTRL_SHIFT) & ST25DV_RFA1SS_PWDCTRL_MASK);
  
  /* Write Sector Security register */
  switch( Zone )
  {
    case ST25DV_PROT_ZONE1:
      status = st25dv_set_rfa1ss_all(&(pObj->Ctx), &reg_value);
      break;
    case ST25DV_PROT_ZONE2:
      status = st25dv_set_rfa2ss_all(&(pObj->Ctx), &reg_value);
      break;
    case ST25DV_PROT_ZONE3:
      status = st25dv_set_rfa3ss_all(&(pObj->Ctx), &reg_value);
      break;
    case ST25DV_PROT_ZONE4:
      status = st25dv_set_rfa4ss_all(&(pObj->Ctx), &reg_value);
      break;
    
    default:
      status = ST25DV_ERROR;
  }
    return status; 
}

/**
  * @brief  Reads the value of the an area end address.
  * @param  pObj the device pObj
  * @param  EndZone ST25DV_END_ZONE value corresponding to an area end address.
  * @param  pEndZ   Pointer used to return the end address of the area.
  * @retval Component error status.
  */
int32_t ST25DV_ReadEndZonex( ST25DV_Object_t* pObj, const ST25DV_END_ZONE EndZone, uint8_t * pEndZ )
{
  int32_t status;

  /* Read the corresponding End zone */ 
  switch( EndZone )
  {
    case ST25DV_ZONE_END1:
      status = st25dv_get_enda1(&(pObj->Ctx),pEndZ);
      break;
    case ST25DV_ZONE_END2:
      status = st25dv_get_enda2(&(pObj->Ctx),pEndZ);
      break;
    case ST25DV_ZONE_END3:
      status = st25dv_get_enda3(&(pObj->Ctx),pEndZ);
      break;
    
    default:
      status = ST25DV_ERROR;
  }
  
  return status;
}

/**
  * @brief    Sets the end address of an area.
  * @details  Needs the I2C Password presentation to be effective.
  * @note     The ST25DV answers a NACK when setting the EndZone2 & EndZone3 to same value than repectively EndZone1 & EndZone2.\n
  *           These NACKs are ok.
  * @param  pObj the device pObj
  * @param  EndZone ST25DV_END_ZONE value corresponding to an area.
  * @param  EndZ   End zone value to be written.
  * @retval Component error status.
  */
int32_t ST25DV_WriteEndZonex( ST25DV_Object_t* pObj, const ST25DV_END_ZONE EndZone, const uint8_t EndZ )
{
  int32_t status;
  
  /* Write the corresponding End zone value in register */  
  switch( EndZone )
  {
    case ST25DV_ZONE_END1:
      status = st25dv_set_enda1(&(pObj->Ctx),&EndZ);
      break;
    case ST25DV_ZONE_END2:
      status = st25dv_set_enda2(&(pObj->Ctx),&EndZ);
      break;
    case ST25DV_ZONE_END3:
      status = st25dv_set_enda3(&(pObj->Ctx),&EndZ);
      break;
    
    default:
      status = ST25DV_ERROR;
  }

  return status;
}

/**
  * @brief  Initializes the end address of the ST25DV areas with their default values (end of memory).
  * @details  Needs the I2C Password presentation to be effective..
  *           The ST25DV answers a NACK when setting the EndZone2 & EndZone3 to same value than repectively EndZone1 & EndZone2.
  *           These NACKs are ok.
  * @param  pObj the device pObj
  * @retval Component error status.
  */
int32_t ST25DV_InitEndZone( ST25DV_Object_t* pObj )
{
  uint8_t endval = 0xFF;
  uint32_t maxmemlength;
  ST25DV_MEM_SIZE memsize;
  int32_t ret;
  
  memsize.Mem_Size = 0;
  memsize.BlockSize = 0;

  /* Get EEPROM mem size */
  ST25DV_ReadMemSize(pObj, &memsize );
  maxmemlength = (memsize.Mem_Size + 1) * (memsize.BlockSize + 1);
  
  /* Compute Max value for endzone register */
  endval = (maxmemlength / 32) - 1;
  
  /* Write EndZone value to ST25DV registers */
  ret = ST25DV_WriteEndZonex(pObj, ST25DV_ZONE_END3, endval );
  if( (ret == ST25DV_OK) || (ret == ST25DV_NACK) )
  {
    ret = ST25DV_WriteEndZonex(pObj,  ST25DV_ZONE_END2, endval );
    if( (ret == ST25DV_OK) || (ret == ST25DV_NACK) )
    {
      ret = ST25DV_WriteEndZonex(pObj,  ST25DV_ZONE_END1, endval );
    }
  }
  return ret;
}

/**
  * @brief  Creates user areas with defined lengths.
  * @details  Needs the I2C Password presentation to be effective.
  * @param  pObj the device pObj
  * @param  Zone1Length Length of area1 in bytes (32 to 8192, 0x20 to 0x2000)
  * @param  Zone2Length Length of area2 in bytes (0 to 8128, 0x00 to 0x1FC0)
  * @param  Zone3Length Length of area3 in bytes (0 to 8064, 0x00 to 0x1F80)
  * @param  Zone4Length Length of area4 in bytes (0 to 8000, 0x00 to 0x1F40)
  * @retval Component error status.
  */
int32_t ST25DV_CreateUserZone( ST25DV_Object_t* pObj, uint16_t Zone1Length, uint16_t Zone2Length, uint16_t Zone3Length, uint16_t Zone4Length )
{
  uint8_t EndVal;
  ST25DV_MEM_SIZE memsize;
  uint16_t maxmemlength = 0;
  int32_t ret = ST25DV_OK;
  
  memsize.Mem_Size = 0;
  memsize.BlockSize = 0;

  ST25DV_ReadMemSize(pObj, &memsize );
  
  maxmemlength = (memsize.Mem_Size + 1) * (memsize.BlockSize + 1);
  
  /* Checks that values of different zones are in bounds */
  if( ( Zone1Length < 32 ) || ( Zone1Length > maxmemlength ) || ( Zone2Length > (maxmemlength - 32) ) 
      || ( Zone3Length > (maxmemlength - 64) ) || ( Zone4Length > (maxmemlength - 96) ) )
  {
    ret = ST25DV_ERROR;
  }

  /* Checks that the total is less than the authorised maximum */
  if( ( Zone1Length + Zone2Length + Zone3Length + Zone4Length ) > maxmemlength )
  {
    ret = ST25DV_ERROR;
  }
  
  if ( ret == ST25DV_OK)
  {
    /* if The value for each Length is not a multiple of 64 correct it. */
    if( (Zone1Length % 32) != 0 )
    {
      Zone1Length = Zone1Length - ( Zone1Length % 32 );
    }
  
    if( (Zone2Length % 32) != 0 )
    {
      Zone2Length = Zone2Length - ( Zone2Length % 32 );
    }
  
    if( (Zone3Length % 32) != 0 )
    {
      Zone3Length = Zone3Length - ( Zone3Length % 32 );
    }
  
    /* First right 0xFF in each Endx value */
    ret = ST25DV_InitEndZone( pObj);
    if( (ret == ST25DV_OK) || (ret == ST25DV_NACK) )
    {
      /* Then Write corresponding value for each zone */
      EndVal = (uint8_t)( (Zone1Length / 32 ) - 1 );
      ret = ST25DV_WriteEndZonex(pObj, ST25DV_ZONE_END1, EndVal );
      if( (ret == ST25DV_OK) || (ret == ST25DV_NACK) )
      {
  
        EndVal = (uint8_t)( ((Zone1Length + Zone2Length) / 32 ) - 1 );
        ret = ST25DV_WriteEndZonex(pObj, ST25DV_ZONE_END2, EndVal );
        if( (ret == ST25DV_OK) || (ret == ST25DV_NACK) )
        {
          EndVal = (uint8_t)( ((Zone1Length + Zone2Length + Zone3Length) / 32 ) - 1 );
          ret = ST25DV_WriteEndZonex(pObj, ST25DV_ZONE_END3, EndVal );
        }
      }
    }
  }
  return ret;
}

/**
  * @brief  Reads the ST25DV Memory Size.
  * @param  pObj the device pObj
  * @param  pSizeInfo Pointer on a ST25DV_MEM_SIZE structure used to return the Memory size information.
  * @retval Component error status.
  */
int32_t ST25DV_ReadMemSize( ST25DV_Object_t* pObj, ST25DV_MEM_SIZE * const pSizeInfo )
{
  uint8_t memsize_msb;
  uint8_t memsize_lsb;
  int32_t status;
  
  /* Read actual value of MEM_SIZE register */
  status = st25dv_get_mem_size_lsb(&(pObj->Ctx), &memsize_lsb);
  if( status == ST25DV_OK )
  {
    status = st25dv_get_mem_size_msb(&(pObj->Ctx), &memsize_msb);
    if( status == ST25DV_OK )
    {
      status = st25dv_get_blk_size(&(pObj->Ctx), &(pSizeInfo->BlockSize));
      if( status == ST25DV_OK )
      {
        /* Extract Memory information */
        pSizeInfo->Mem_Size = memsize_msb;
        pSizeInfo->Mem_Size = (pSizeInfo->Mem_Size << 8) |memsize_lsb;
      }
    }
  }
  return status;
}

/**
  * @brief  Reads the Energy harvesting mode.
  * @param  pObj the device pObj
  * @param  pEH_mode Pointer on a ST25DV_EH_MODE_STATUS value corresponding to the Energy Harvesting state.
  * @retval Component error status.
  */
int32_t ST25DV_ReadEHMode( ST25DV_Object_t* pObj, ST25DV_EH_MODE_STATUS * const pEH_mode )
{
  uint8_t reg_value;
  int32_t status;
  
  /* Read actual value of EH_MODE register */
  status = st25dv_get_eh_mode( &(pObj->Ctx), &reg_value);
  if( status == ST25DV_OK )
  {
    /* Extract EH_mode configuration */
    if( reg_value )
    {
      *pEH_mode = ST25DV_EH_ON_DEMAND;
    }
    else
    {
      *pEH_mode = ST25DV_EH_ACTIVE_AFTER_BOOT;
    }
  }
  
  return status;
}

/**
  * @brief  Sets the Energy harvesting mode.
  * @details  Needs the I2C Password presentation to be effective.
  * @param  pObj the device pObj
  * @param  EH_mode ST25DV_EH_MODE_STATUS value for the Energy harvesting mode to be set.
  * @retval Component error status.
  */
int32_t ST25DV_WriteEHMode( ST25DV_Object_t* pObj, const ST25DV_EH_MODE_STATUS EH_mode )
{
  uint8_t reg_value;
  
  /* Update EH_mode */
  reg_value = (uint8_t)EH_mode;
  
  /* Write EH_MODE register */
  return st25dv_set_eh_mode(&(pObj->Ctx), &reg_value);
}

/**
  * @brief  Reads the RF Management configuration.
  * @param  pObj the device pObj
  * @param  pRF_Mngt Pointer on a ST25DV_RF_MNGT structure used to return the RF Management configuration.
  * @retval Component error status.
  */
int32_t ST25DV_ReadRFMngt( ST25DV_Object_t* pObj, ST25DV_RF_MNGT * const pRF_Mngt )
{
  int32_t status;
  uint8_t reg_value = 0;
  
  /* Read actual value of RF_MNGT register */
  status = st25dv_get_rf_mngt_all(&(pObj->Ctx), &reg_value);

  if( status == ST25DV_OK )
  {
    /* Extract RF Disable information */
    if( (reg_value & ST25DV_RF_MNGT_RFDIS_MASK) == ST25DV_RF_MNGT_RFDIS_MASK )
    {
      pRF_Mngt->RfDisable = ST25DV_ENABLE;
    }
    else
    {
      pRF_Mngt->RfDisable = ST25DV_DISABLE;
    }
    
    /* Extract RF Sleep information */
    if( (reg_value & ST25DV_RF_MNGT_RFSLEEP_MASK) == ST25DV_RF_MNGT_RFSLEEP_MASK )
    {
      pRF_Mngt->RfSleep = ST25DV_ENABLE;
    }
    else
    {
      pRF_Mngt->RfSleep = ST25DV_DISABLE;
    }
  }
  
  return status;
}

/**
  * @brief  Sets the RF Management configuration.
  * @details  Needs the I2C Password presentation to be effective.
  * @param  pObj the device pObj
  * @param  Rfmngt Value of the RF Management configuration to be written.
  * @retval Component error status.
  */
int32_t ST25DV_WriteRFMngt( ST25DV_Object_t* pObj, const uint8_t Rfmngt )
{
  /* Write RF_MNGT register */
  return st25dv_set_rf_mngt_all(&(pObj->Ctx), &Rfmngt);
}

/**
  * @brief  Reads the RFDisable register information.
  * @param  pRFDisable Pointer on a ST25DV_EN_STATUS value corresponding to the RF Disable status.
  * @retval Component error status.
  */
int32_t ST25DV_GetRFDisable( ST25DV_Object_t* pObj, ST25DV_EN_STATUS * const pRFDisable )
{
  int32_t status;
  uint8_t reg_value = 0;
  
  /* Read actual value of RF_MNGT register */
  status = st25dv_get_rf_mngt_rfdis(&(pObj->Ctx), &reg_value);
  
  /* Extract RFDisable information */
  if( status == ST25DV_OK )
  {
    if( reg_value )
    {
      *pRFDisable = ST25DV_ENABLE;
    }
    else
    {
      *pRFDisable = ST25DV_DISABLE;
    }
  }
  
  return status;
}

/**
  * @brief  Sets the RF Disable configuration.
  * @details  Needs the I2C Password presentation to be effective.
  * @param  pObj the device pObj
  * @retval Component error status.
  */
int32_t ST25DV_SetRFDisable( ST25DV_Object_t* pObj )
{
  uint8_t reg_value = 1;
  
  /* Write RF_MNGT register */  
  return st25dv_set_rf_mngt_rfdis(&(pObj->Ctx), &reg_value);
}

/**
  * @brief  Resets the RF Disable configuration
  * @details  Needs the I2C Password presentation to be effective.
  * @param  pObj the device pObj
  * @retval Component error status.
  */
int32_t ST25DV_ResetRFDisable( ST25DV_Object_t* pObj )
{
  uint8_t reg_value = 0;
  
  /* Write RF_MNGT register */  
  return st25dv_set_rf_mngt_rfdis(&(pObj->Ctx), &reg_value);
}

/**
  * @brief  Reads the RFSleep register information.
  * @param  pObj the device pObj
  * @param  pRFSleep Pointer on a ST25DV_EN_STATUS value corresponding to the RF Sleep status.
  * @retval Component error status.
  */
int32_t ST25DV_GetRFSleep( ST25DV_Object_t* pObj, ST25DV_EN_STATUS * const pRFSleep )
{
  int32_t status;
  uint8_t reg_value = 0;
  
  
  /* Read actual value of RF_MNGT register */
  status = st25dv_get_rf_mngt_rfsleep(&(pObj->Ctx), &reg_value);
  
  /* Extract RFDisable information */
  if( status == ST25DV_OK )
  {
    if( reg_value )
    {
      *pRFSleep = ST25DV_ENABLE;
    }
    else
    {
      *pRFSleep = ST25DV_DISABLE;
    }
  }
 
  return status;
}

/**
  * @brief  Sets the RF Sleep configuration.
  * @details  Needs the I2C Password presentation to be effective.
  * @param  pObj the device pObj
  * @retval Component error status.
  */
int32_t ST25DV_SetRFSleep(ST25DV_Object_t* pObj )
{
  uint8_t reg_value = 1;
  
  /* Write RF_MNGT register */  
  return st25dv_set_rf_mngt_rfsleep(&(pObj->Ctx), &reg_value);
}

/**
  * @brief  Resets the RF Sleep configuration.
  * @details  Needs the I2C Password presentation to be effective.
  * @param  pObj the device pObj
  * @retval Component error status.
  */
int32_t ST25DV_ResetRFSleep( ST25DV_Object_t* pObj )
{
  uint8_t reg_value = 0;
  
  /* Write RF_MNGT register */  
  return st25dv_set_rf_mngt_rfsleep(&(pObj->Ctx), &reg_value);
}

/**
  * @brief  Reads the Mailbox mode.
  * @param  pObj the device pObj
  * @param  pMB_mode Pointer on a ST25DV_EH_MODE_STATUS value used to return the Mailbox mode.
  * @retval Component error status.
  */
int32_t ST25DV_ReadMBMode( ST25DV_Object_t* pObj, ST25DV_EN_STATUS * const pMB_mode )
{
  uint8_t reg_value;
  int32_t status;
  
  /* Read actual value of MB_MODE register */
  status = st25dv_get_mb_mode_rw(&(pObj->Ctx), &reg_value);
  if( status == ST25DV_OK )
  {
    /* Extract Mailbox mode status */
    if( reg_value  )
    {
      *pMB_mode = ST25DV_ENABLE;
    }
    else
    {
      *pMB_mode = ST25DV_DISABLE;
    }
  }
  return status;
}

/**
  * @brief  Sets the Mailbox mode.
  * @details  Needs the I2C Password presentation to be effective.
  * @param  pObj the device pObj
  * @param  MB_mode ST25DV_EN_STATUS value corresponding to the Mailbox mode to be set.
  * @retval Component error status.
  */
int32_t ST25DV_WriteMBMode( ST25DV_Object_t* pObj, const ST25DV_EN_STATUS MB_mode )
{
  uint8_t reg_value;
  int32_t status;
  /* Update Mailbox mode status */
  reg_value = (uint8_t)MB_mode;
  
  /* Write MB_MODE register */
  status = st25dv_set_mb_mode_rw(&(pObj->Ctx), &reg_value);

  return status;
}

/**
  * @brief  Reads the Mailbox watchdog duration coefficient.
  * @param  pObj the device pObj
  * @param  pWdgDelay Pointer on a uint8_t used to return the watchdog duration coefficient.
  * @retval Component error status.
  */
int32_t ST25DV_ReadMBWDG( ST25DV_Object_t* pObj, uint8_t * const pWdgDelay )
{  
  /* Read actual value of MB_WDG register */  
  return st25dv_get_mb_wdg_delay(&(pObj->Ctx), pWdgDelay);
}

/**
  * @brief  Writes the Mailbox watchdog coefficient delay
  * @details  Needs the I2C Password presentation to be effective.
  * @param  pObj the device pObj
  * @param  WdgDelay Watchdog duration coefficient to be written (Watch dog duration = MB_WDG*30 ms +/- 6%).
  * @retval Component error status.
  */
int32_t ST25DV_WriteMBWDG( ST25DV_Object_t* pObj, const uint8_t WdgDelay )
{
   /* Write MB_WDG register */
  return st25dv_set_mb_wdg_delay(&(pObj->Ctx), &WdgDelay);
}

/**
  * @brief  Reads N bytes of data from the Mailbox, starting at the specified byte offset.
  * @param  pObj the device pObj
  * @param  pData   Pointer on the buffer used to return the read data.
  * @param  Offset  Offset in the Mailbox memory, byte number to start the read.
  * @param  NbByte  Number of bytes to be read.
  * @retval Component error status.
  */
int32_t ST25DV_ReadMailboxData( ST25DV_Object_t* pObj, uint8_t * const pData, const uint16_t Offset, const uint16_t NbByte )
{
  int32_t status = ST25DV_ERROR;
  if( Offset <= ST25DV_MAX_MAILBOX_LENGTH )
  {
    status =  pObj->IO.Read( ST25DV_ADDR_DATA_I2C, ST25DV_MAILBOX_RAM_REG + Offset, pData,  NbByte );
  } 
  /* Read Data in user memory */
  return status;
}

/**
  * @brief  Writes N bytes of data in the Mailbox, starting from first Mailbox Address.
  * @param  pObj the device pObj
  * @param  pData   Pointer to the buffer containing the data to be written.
  * @param  NbByte  Number of bytes to be written.
  * @retval Component error status.
  */
int32_t ST25DV_WriteMailboxData( ST25DV_Object_t* pObj, const uint8_t * const pData, const uint16_t NbByte )
{ 
  int32_t status = ST25DV_ERROR;
  
  /* ST25DV can write a maximum of 256 bytes in Mailbox */
  if( NbByte <= ST25DV_MAX_MAILBOX_LENGTH )
  {
    /* Write NbByte data in memory */
    status =  pObj->IO.Write( ST25DV_ADDR_DATA_I2C, ST25DV_MAILBOX_RAM_REG, pData,  NbByte );
  }
  
  return status;
}

/**
  * @brief  Reads N bytes from the mailbox registers, starting at the specified I2C address.
  * @param  pObj the device pObj
  * @param  pData   Pointer on the buffer used to return the data.
  * @param  TarAddr I2C memory address to be read.
  * @param  NbByte  Number of bytes to be read.
  * @retval Component error status.
  */
int32_t ST25DV_ReadMailboxRegister( ST25DV_Object_t* pObj, uint8_t * const pData, const uint16_t TarAddr, const uint16_t NbByte )
{ 
  int32_t status = ST25DV_ERROR;
  if( (TarAddr >= ST25DV_GPO_DYN_REG) && (TarAddr <= ST25DV_MBLEN_DYN_REG) )
  {
    status =  pObj->IO.Read( ST25DV_ADDR_DATA_I2C, TarAddr,pData,  NbByte);
  }
  
  return status;
}

/**
  * @brief  Writes N bytes to the specified mailbox register.
  * @param  pObj the device pObj
  * @param  pData   Pointer on the data to be written.
  * @param  TarAddr I2C register address to be written.
  * @param  NbByte  Number of bytes to be written.
  * @retval Component error status.
  */
int32_t ST25DV_WriteMailboxRegister( ST25DV_Object_t* pObj, const uint8_t * const pData, const uint16_t TarAddr, const uint16_t NbByte )
{ 
  int32_t status = ST25DV_ERROR;
  if( (TarAddr >= ST25DV_GPO_DYN_REG) && (TarAddr <= ST25DV_MBLEN_DYN_REG) )
  {
    status =   pObj->IO.Write( ST25DV_ADDR_DATA_I2C, TarAddr,pData,  NbByte);
  }
  
  /* Write NbByte data in memory */
  return status;
}

/**
  * @brief  Reads the status of the security session open register.
  * @param  pObj the device pObj
  * @param  pSession Pointer on a ST25DV_I2CSSO_STATUS value used to return the session status.
  * @retval Component error status.
  */
int32_t ST25DV_ReadI2CSecuritySession_Dyn( ST25DV_Object_t* pObj, ST25DV_I2CSSO_STATUS * const pSession )
{
  uint8_t reg_value;
  int32_t status;

  /* Read actual value of I2C_SSO_DYN register */
  status = st25dv_get_i2c_sso_dyn_i2csso(&(pObj->Ctx), &reg_value);
  if( status == ST25DV_OK )
  {
  
    /* Extract Open session information */
    if( reg_value )
    {
      *pSession = ST25DV_SESSION_OPEN;
    }
    else
    {
      *pSession = ST25DV_SESSION_CLOSED;
    }
  }
  return status;
}

/**
  * @brief  Reads the IT status register from the ST25DV.
  * @param  pObj the device pObj
  * @param  pITStatus Pointer on uint8_t, used to return the IT status, such as:
  *                       - RFUSERSTATE = 0x01
  *                       - RFBUSY = 0x02
  *                       - RFINTERRUPT = 0x04
  *                       - FIELDFALLING = 0x08
  *                       - FIELDRISING = 0x10
  *                       - RFPUTMSG = 0x20
  *                       - RFGETMSG = 0x40
  *                       - RFWRITE = 0x80
  *
  * @retval Component error status.
  */
int32_t ST25DV_ReadITSTStatus_Dyn( ST25DV_Object_t* pObj, uint8_t * const pITStatus )
{
  /* Read value of ITStatus register */
  return st25dv_get_itsts_dyn_all(&(pObj->Ctx), pITStatus );
}

/**
  * @brief  Read value of dynamic GPO register configuration.
  * @param  pObj the device pObj
  * @param  pGPO ST25DV_GPO pointer of the dynamic GPO configuration to store.
  * @retval Component error status.
  */
int32_t ST25DV_ReadGPO_Dyn( ST25DV_Object_t* pObj, uint8_t *GPOConfig )
{
  /* Read actual value of ST25DV_GPO_DYN_REG register */
  return st25dv_get_gpo_dyn_all(&(pObj->Ctx), GPOConfig);
}

/**
  * @brief  Get dynamique GPO enable status
  * @param  pObj the device pObj
  * @param  pGPO_en ST25DV_EN_STATUS pointer of the GPO enable status to store
  * @retval Component error status.
  */
int32_t ST25DV_GetGPO_en_Dyn( ST25DV_Object_t* pObj, ST25DV_EN_STATUS * const pGPO_en )
{
  uint8_t reg_value;
  int32_t status;
  
  /* Read actual value of GPO_DYN register */
  status = st25dv_get_gpo_dyn_enable(&(pObj->Ctx), &reg_value);
  if( status == ST25DV_OK )
  {
    /* Extract GPO enable status information */
    if( reg_value )
    {
      *pGPO_en = ST25DV_ENABLE;
    }
    else
    {
      *pGPO_en = ST25DV_DISABLE;
    }
  }
  return status;
}

/**
  * @brief  Set dynamique GPO enable configuration.
  * @param  pObj the device pObj
  * @param  None No parameters.
  * @retval Component error status.
  */
int32_t ST25DV_SetGPO_en_Dyn( ST25DV_Object_t* pObj )
{
  uint8_t reg_value = 1;
  
  /* Write GPO_DYN Register */
  return st25dv_set_gpo_dyn_enable(&(pObj->Ctx), &reg_value);
}

/**
  * @brief  Reset dynamique GPO enable configuration.
  * @param  pObj the device pObj
  * @param  None No parameters.
  * @retval Component error status.
  */
int32_t ST25DV_ResetGPO_en_Dyn( ST25DV_Object_t* pObj )
{
  uint8_t reg_value = 0;
  
  /* Write GPO_DYN Register */
  return st25dv_set_gpo_dyn_enable(&(pObj->Ctx), &reg_value);
}

/**
  * @brief  Read value of dynamic EH Ctrl register configuration
  * @param  pObj the device pObj
  * @param  pEH_CTRL : ST25DV_EH_CTRL pointer of the dynamic EH Ctrl configuration to store
  * @retval Component error status.
  */
int32_t ST25DV_ReadEHCtrl_Dyn( ST25DV_Object_t* pObj, ST25DV_EH_CTRL * const pEH_CTRL )
{
  int32_t status;
  uint8_t reg_value = 0;
  
  /* Read actual value of ST25DV_EH_CTRL_DYN_REG register */
  status = st25dv_get_eh_ctrl_dyn_all(&(pObj->Ctx), &reg_value);
  
  if( status == ST25DV_OK )
  {
    /* Extract EH EN Mode configuration */
    if( (reg_value & ST25DV_EH_CTRL_DYN_EH_EN_MASK) == ST25DV_EH_CTRL_DYN_EH_EN_MASK )
    {
      pEH_CTRL->EH_EN_Mode = ST25DV_ENABLE;
    }
    else
    {
      pEH_CTRL->EH_EN_Mode = ST25DV_DISABLE;
    }
    
    /* Extract EH_ON configuration */
    if( (reg_value & ST25DV_EH_CTRL_DYN_EH_ON_MASK) == ST25DV_EH_CTRL_DYN_EH_ON_MASK )
    {
      pEH_CTRL->EH_on = ST25DV_ENABLE;
    }
    else
    {
      pEH_CTRL->EH_on = ST25DV_DISABLE;
    }
    
    /* Extract FIELD_ON configuration */
    if( (reg_value & ST25DV_EH_CTRL_DYN_FIELD_ON_MASK) == ST25DV_EH_CTRL_DYN_FIELD_ON_MASK )
    {
      pEH_CTRL->Field_on = ST25DV_ENABLE;
    }
    else
    {
      pEH_CTRL->Field_on = ST25DV_DISABLE;
    }
    
    /* Extract VCC_ON configuration */
    if( (reg_value & ST25DV_EH_CTRL_DYN_VCC_ON_MASK) == ST25DV_EH_CTRL_DYN_VCC_ON_MASK )
    {
      pEH_CTRL->VCC_on = ST25DV_ENABLE;
    }
    else
    {
      pEH_CTRL->VCC_on = ST25DV_DISABLE;
    }
  }
  
  return status;
}

/**
  * @brief  Reads the Energy Harvesting dynamic status.
  * @param  pObj the device pObj
  * @param  pEH_Val Pointer on a ST25DV_EN_STATUS value used to return the Energy Harvesting dynamic status.
  * @retval Component error status.
  */
int32_t ST25DV_GetEHENMode_Dyn( ST25DV_Object_t* pObj, ST25DV_EN_STATUS * const pEH_Val )
{
  uint8_t reg_value;
  int32_t status;
  
  /* Read actual value of EH_CTRL_DYN register */
  status = st25dv_get_eh_ctrl_dyn_eh_en(&(pObj->Ctx), &reg_value);
  if( status == ST25DV_OK )
  {
  
    /* Extract Energy Harvesting status information */
    if( reg_value )
    {
      *pEH_Val = ST25DV_ENABLE;
    }
    else
    {
      *pEH_Val = ST25DV_DISABLE;
    }
  }
  return status;
}

/**
  * @brief  Dynamically sets the Energy Harvesting mode.
  * @param  pObj the device pObj
  * @retval Component error status.
  */
int32_t ST25DV_SetEHENMode_Dyn( ST25DV_Object_t* pObj )
{
  uint8_t reg_value = 1;
  
  /* Write EH_CTRL_DYN Register */
  return st25dv_set_eh_ctrl_dyn_eh_en(&(pObj->Ctx), &reg_value);
}

/**
  * @brief  Dynamically unsets the Energy Harvesting mode.
  * @param  pObj the device pObj
  * @retval Component error status.
  */
int32_t ST25DV_ResetEHENMode_Dyn( ST25DV_Object_t* pObj )
{
  uint8_t reg_value = 0;
  
  /* Write EH_CTRL_DYN Register */
  return st25dv_set_eh_ctrl_dyn_eh_en(&(pObj->Ctx), &reg_value);
}

/**
  * @brief  Reads the EH_ON status from the EH_CTRL_DYN register.
  * @param  pObj the device pObj
  * @param  pEHON Pointer on a ST25DV_EN_STATUS value used to return the EHON status.
  * @retval Component error status.
  */
int32_t ST25DV_GetEHON_Dyn( ST25DV_Object_t* pObj, ST25DV_EN_STATUS * const pEHON )
{
  int32_t status;
  uint8_t reg_value = 0;
  
  /* Read actual value of EH_CTRL_DYN register */
  status = st25dv_get_eh_ctrl_dyn_eh_on(&(pObj->Ctx), &reg_value);
  
  /* Extract RF Field information */
  if( status == ST25DV_OK )
  {
    if( reg_value )
    {
      *pEHON = ST25DV_ENABLE;
    }
    else
    {
      *pEHON = ST25DV_DISABLE;
    }
  }
  
  return status;
}

/**
  * @brief  Checks if RF Field is present in front of the ST25DV.
  * @param  pObj the device pObj
  * @param  pRF_Field Pointer on a ST25DV_FIELD_STATUS value used to return the field presence.
  * @retval Component error status.
  */
int32_t ST25DV_GetRFField_Dyn( ST25DV_Object_t* pObj, ST25DV_FIELD_STATUS * const pRF_Field )
{
  int32_t status;
  uint8_t reg_value = 0;
  
  /* Read actual value of EH_CTRL_DYN register */
  status = st25dv_get_eh_ctrl_dyn_field_on(&(pObj->Ctx), &reg_value );
  
  /* Extract RF Field information */
  if( status == ST25DV_OK )
  {
    if( reg_value )
    {
      *pRF_Field = ST25DV_FIELD_ON;
    }
    else
    {
      *pRF_Field = ST25DV_FIELD_OFF;
    }
  }
  
  return status;
}

/**
  * @brief  Check if VCC is supplying the ST25DV.
  * @param  pObj the device pObj
  * @param  pVCC ST25DV_VCC_STATUS pointer of the VCC status to store
  * @retval Component error status.
  */
int32_t ST25DV_GetVCC_Dyn( ST25DV_Object_t* pObj, ST25DV_VCC_STATUS * const pVCC )
{
  int32_t status;
  uint8_t reg_value = 0;
  
  /* Read actual value of EH_CTRL_DYN register */
  status = st25dv_get_eh_ctrl_dyn_vcc_on(&(pObj->Ctx), &reg_value );
  
  /* Extract VCC information */
  if( status == ST25DV_OK )
  {
    if( reg_value )
    {
      *pVCC = ST25DV_VCC_ON;
    }
    else
    {
      *pVCC = ST25DV_VCC_OFF;
    }
  }
  
  return status;
}

/**
  * @brief  Read value of dynamic RF Management configuration
  * @param  pObj the device pObj
  * @param  pRF_Mngt : ST25DV_RF_MNGT pointer of the dynamic RF Management configuration to store
  * @retval Component error status.
  */
int32_t ST25DV_ReadRFMngt_Dyn( ST25DV_Object_t* pObj, ST25DV_RF_MNGT * const pRF_Mngt )
{
  int32_t status;
  uint8_t reg_value = 0;
  
  /* Read actual value of RF_MNGT_DYN register */
  status = st25dv_get_rf_mngt_dyn_all(&(pObj->Ctx), &reg_value);
  
  if( status == ST25DV_OK )
  {
    /* Extract RF Disable configuration */
    if( (reg_value & ST25DV_RF_MNGT_DYN_RFDIS_MASK) == ST25DV_RF_MNGT_DYN_RFDIS_MASK )
    {
      pRF_Mngt->RfDisable = ST25DV_ENABLE;
    }
    else
    {
      pRF_Mngt->RfDisable = ST25DV_DISABLE;
    }
    
    /* Extract RF Sleep configuration */
    if( (reg_value & ST25DV_RF_MNGT_DYN_RFSLEEP_MASK) == ST25DV_RF_MNGT_DYN_RFSLEEP_MASK )
    {
      pRF_Mngt->RfSleep = ST25DV_ENABLE;
    }
    else
    {
      pRF_Mngt->RfSleep = ST25DV_DISABLE;
    }
  }
  
  return status;
}

/**
  * @brief  Writes a value to the RF Management dynamic register.
  * @param  pObj the device pObj
  * @param  RF_Mngt Value to be written to the RF Management dynamic register.
  * @retval Component error status.
  */
int32_t ST25DV_WriteRFMngt_Dyn( ST25DV_Object_t* pObj, const uint8_t RF_Mngt )
{
  /* Write value to RF_MNGT_DYN register */
  return st25dv_set_rf_mngt_dyn_all(&(pObj->Ctx), &RF_Mngt);
}

/**
  * @brief  Reads the RFDisable dynamic register information.
  * @param  pObj the device pObj
  * @param  pRFDisable Pointer on a ST25DV_EN_STATUS value used to return the RF Disable state.
  * @retval Component error status.
  */
int32_t ST25DV_GetRFDisable_Dyn( ST25DV_Object_t* pObj, ST25DV_EN_STATUS * const pRFDisable )
{
  int32_t status;
  uint8_t reg_value = 0;
  
  /* Read actual value of RF_MNGT_DYN register */
  status = st25dv_get_rf_mngt_dyn_rfdis(&(pObj->Ctx), &reg_value);
  
  /* Extract RFDisable information */
  if( status == ST25DV_OK )
  {
    if( reg_value )
    {
      *pRFDisable = ST25DV_ENABLE;
    }
    else
    {
      *pRFDisable = ST25DV_DISABLE;
    }
  }
  
  return status;
}

/**
  * @brief  Sets the RF Disable dynamic configuration.
  * @param  pObj the device pObj
  * @retval Component error status.
  */
int32_t ST25DV_SetRFDisable_Dyn( ST25DV_Object_t* pObj )
{
  uint8_t reg_value = 1;
  
  return st25dv_set_rf_mngt_dyn_rfdis(&(pObj->Ctx), &reg_value);
}

/**
  * @brief  Unsets the RF Disable dynamic configuration.
  * @param  pObj the device pObj
  * @retval Component error status.
  */
int32_t ST25DV_ResetRFDisable_Dyn( ST25DV_Object_t* pObj )
{
  uint8_t reg_value = 0;
  
  return st25dv_set_rf_mngt_dyn_rfdis(&(pObj->Ctx), &reg_value);
}

/**
  * @brief  Reads the RFSleep dynamic register information.
  * @param  pObj the device pObj
  * @param  pRFSleep Pointer on a ST25DV_EN_STATUS values used to return the RF Sleep state.
  * @retval Component error status.
  */
int32_t ST25DV_GetRFSleep_Dyn( ST25DV_Object_t* pObj, ST25DV_EN_STATUS * const pRFSleep )
{
  int32_t status;
  uint8_t reg_value = 0;
  
  /* Read actual value of RF_MNGT_DYN register */
  status = st25dv_get_rf_mngt_dyn_rfsleep(&(pObj->Ctx), &reg_value);
  
  /* Extract RFSleep information */
  if( status == ST25DV_OK )
  {
    if( reg_value )
    {
      *pRFSleep = ST25DV_ENABLE;
    }
    else
    {
      *pRFSleep = ST25DV_DISABLE;
    }
  }
  
  return status;
}

/**
  * @brief  Sets the RF Sleep dynamic configuration.
  * @param  pObj the device pObj
  * @retval Component error status.
  */
int32_t ST25DV_SetRFSleep_Dyn( ST25DV_Object_t* pObj )
{
  uint8_t reg_value = 1;
  
  return st25dv_set_rf_mngt_dyn_rfsleep(&(pObj->Ctx), &reg_value);
}

/**
  * @brief  Unsets the RF Sleep dynamic configuration.
  * @param  pObj the device pObj
  * @retval Component error status.
  */
int32_t ST25DV_ResetRFSleep_Dyn( ST25DV_Object_t* pObj )
{
  uint8_t reg_value = 0;
  
  return st25dv_set_rf_mngt_dyn_rfsleep(&(pObj->Ctx), &reg_value);
}

/**
  * @brief  Reads the Mailbox ctrl dynamic register.
  * @param  pObj the device pObj
  * @param  pCtrlStatus Pointer on a ST25DV_MB_CTRL_DYN_STATUS structure used to return the dynamic Mailbox ctrl information.
  * @retval Component error status.
  */
int32_t ST25DV_ReadMBCtrl_Dyn( ST25DV_Object_t* pObj, ST25DV_MB_CTRL_DYN_STATUS * const pCtrlStatus )
{
  uint8_t reg_value;
  int32_t status;
  
  /* Read MB_CTRL_DYN register */
  status = st25dv_get_mb_ctrl_dyn_all(&(pObj->Ctx), &reg_value);
  if( status == ST25DV_OK )
  {
    /* Extract Mailbox ctrl information */
    pCtrlStatus->MbEnable = (reg_value & ST25DV_MB_CTRL_DYN_MBEN_MASK) >> ST25DV_MB_CTRL_DYN_MBEN_SHIFT;
    pCtrlStatus->HostPutMsg = (reg_value & ST25DV_MB_CTRL_DYN_HOSTPUTMSG_MASK) >> ST25DV_MB_CTRL_DYN_HOSTPUTMSG_SHIFT;
    pCtrlStatus->RfPutMsg = (reg_value & ST25DV_MB_CTRL_DYN_RFPUTMSG_MASK) >> ST25DV_MB_CTRL_DYN_RFPUTMSG_SHIFT;
    pCtrlStatus->HostMissMsg = (reg_value & ST25DV_MB_CTRL_DYN_HOSTMISSMSG_MASK) >> ST25DV_MB_CTRL_DYN_HOSTMISSMSG_SHIFT;
    pCtrlStatus->RFMissMsg = (reg_value & ST25DV_MB_CTRL_DYN_RFMISSMSG_MASK) >> ST25DV_MB_CTRL_DYN_RFMISSMSG_SHIFT;
    pCtrlStatus->CurrentMsg = (ST25DV_CURRENT_MSG)((reg_value & ST25DV_MB_CTRL_DYN_CURRENTMSG_MASK) >> ST25DV_MB_CTRL_DYN_CURRENTMSG_SHIFT);
  }
  return status;
}

/**
  * @brief  Reads the Mailbox Enable dynamic configuration.
  * @param  pObj the device pObj
  * @retval Component error status.
  */
int32_t ST25DV_GetMBEN_Dyn( ST25DV_Object_t* pObj, ST25DV_EN_STATUS * const pMBEN )
{
  uint8_t reg_value;
  int32_t status;
  
  /* Read MB_CTRL_DYN register */
  status = st25dv_get_mb_ctrl_dyn_mben( &(pObj->Ctx),&reg_value );
  if( status == ST25DV_OK )
  {
    if( reg_value )
    {
      *pMBEN = ST25DV_ENABLE;
    }
    else
    {
      *pMBEN = ST25DV_DISABLE;
    }
  }
  return status;
}

/**
  * @brief  Sets the Mailbox Enable dynamic configuration.
  * @param  pObj the device pObj
  * @retval Component error status.
  */
int32_t ST25DV_SetMBEN_Dyn( ST25DV_Object_t* pObj )
{
  uint8_t reg_value = 1;
    
  /* Write MB_CTRL_DYN register */
  return st25dv_set_mb_ctrl_dyn_mben( &(pObj->Ctx),&reg_value );
}

/**
  * @brief  Unsets the Mailbox Enable dynamic configuration.
  * @param  pObj the device pObj
  * @retval Component error status.
  */
int32_t ST25DV_ResetMBEN_Dyn( ST25DV_Object_t* pObj )
{
  uint8_t reg_value = 0;
    
  /* Write MB_CTRL_DYN register */
  return st25dv_set_mb_ctrl_dyn_mben( &(pObj->Ctx),&reg_value );
}

/**
  * @brief  Reads the Mailbox message length dynamic register.
  * @param  pMBLength Pointer on a uint8_t used to return the Mailbox message length.
  * @param  pObj the device pObj
  * @retval Component error status.
  */
int32_t ST25DV_ReadMBLength_Dyn( ST25DV_Object_t* pObj, uint8_t * const pMBLength )
{
  /* Read actual value of MBLEN_DYN register */
  return st25dv_get_mblen_dyn_mblen( &(pObj->Ctx),pMBLength );
}

static int32_t ReadRegWrap(void *handle, uint16_t Reg, uint8_t* pData, uint16_t len)
{
  int32_t status;
  ST25DV_Object_t *pObj = (ST25DV_Object_t *)handle;
  if(Reg & (ST25DV_IS_DYNAMIC_REGISTER))
  {
    status = pObj->IO.Read(ST25DV_ADDR_DATA_I2C, Reg, pData, len);
  } else {
    status = pObj->IO.Read(ST25DV_ADDR_SYST_I2C, Reg, pData, len);
  }
  return status;
}

static int32_t WriteRegWrap(void *handle, uint16_t Reg, const uint8_t* pData, uint16_t len)
{
  int32_t ret;
  ST25DV_Object_t *pObj = (ST25DV_Object_t *)handle;
  if(Reg & (ST25DV_IS_DYNAMIC_REGISTER))
  {
    ret = pObj->IO.Write(ST25DV_ADDR_DATA_I2C, Reg, pData, len);
  } else {
    ret = pObj->IO.Write(ST25DV_ADDR_SYST_I2C, Reg, pData, len);
    if( ret == ST25DV_OK )
    {
      int32_t pollstatus;
      /* Poll until EEPROM is available */
      uint32_t tickstart = pObj->IO.GetTick();
      /* Wait until ST25DV is ready or timeout occurs */
      do
      {
        pollstatus = pObj->IO.IsReady( ST25DV_ADDR_SYST_I2C, 1 );
      } while( ( (uint32_t)((int32_t)pObj->IO.GetTick() - (int32_t)tickstart) < ST25DV_WRITE_TIMEOUT) && (pollstatus != ST25DV_OK) );
    
      if( pollstatus != ST25DV_OK )
      {
        ret = ST25DV_TIMEOUT;
      }
    }
  }

  return ret;
}

/**
 * @}
 */

/**
 * @}
 */


/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
