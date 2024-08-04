/**
  ******************************************************************************
  * @file    stm32l475e_iot01_accelero.c
  * @author  MCD Application Team
  * @brief   This file provides a set of functions needed to manage the accelerometer sensor
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
#include "stm32l475e_iot01_accelero.h"
/** @addtogroup BSP
  * @{
  */ 

/** @addtogroup STM32L475E_IOT01
  * @{
  */

/** @defgroup STM32L475E_IOT01_ACCELERO ACCELERO
  * @{
  */

/** @defgroup STM32L475E_IOT01_ACCELERO_Private_Variables ACCELERO Private Variables 
  * @{
  */    
static ACCELERO_DrvTypeDef *AccelerometerDrv;  
/**
  * @}
  */

/** @defgroup STM32L475E_IOT01_ACCELERO_Private_Functions ACCELERO Private Functions
  * @{
  */ 
/**
  * @brief  Initialize the ACCELERO.
  * @retval ACCELERO_OK or ACCELERO_ERROR
  */
ACCELERO_StatusTypeDef BSP_ACCELERO_Init(void)
{  
  ACCELERO_StatusTypeDef ret = ACCELERO_OK;
  uint16_t ctrl = 0x0000;
  ACCELERO_InitTypeDef LSM6DSL_InitStructure;

  if(Lsm6dslAccDrv.ReadID() != LSM6DSL_ACC_GYRO_WHO_AM_I)
  {
    ret = ACCELERO_ERROR;
  }
  else
  {
    /* Initialize the ACCELERO accelerometer driver structure */
    AccelerometerDrv = &Lsm6dslAccDrv;
  
    /* MEMS configuration ------------------------------------------------------*/
    /* Fill the ACCELERO accelerometer structure */
    LSM6DSL_InitStructure.AccOutput_DataRate = LSM6DSL_ODR_52Hz;
    LSM6DSL_InitStructure.Axes_Enable = 0;
    LSM6DSL_InitStructure.AccFull_Scale = LSM6DSL_ACC_FULLSCALE_2G;
    LSM6DSL_InitStructure.BlockData_Update = LSM6DSL_BDU_BLOCK_UPDATE;
    LSM6DSL_InitStructure.High_Resolution = 0;
    LSM6DSL_InitStructure.Communication_Mode = 0;
        
    /* Configure MEMS: data rate, full scale  */
    ctrl =  (LSM6DSL_InitStructure.AccOutput_DataRate | LSM6DSL_InitStructure.AccFull_Scale);
    
    /* Configure MEMS: BDU and Auto-increment for multi read/write */
    ctrl |= ((LSM6DSL_InitStructure.BlockData_Update | LSM6DSL_ACC_GYRO_IF_INC_ENABLED) << 8);

    /* Configure the ACCELERO accelerometer main parameters */
    AccelerometerDrv->Init(ctrl);
  }  

  return ret;
}

/**
  * @brief  DeInitialize the ACCELERO.
  * @retval None.
  */
void BSP_ACCELERO_DeInit(void)
{
  /* DeInitialize the accelerometer IO interfaces */
  if(AccelerometerDrv != NULL)
  {
    if(AccelerometerDrv->DeInit != NULL)
    {
      AccelerometerDrv->DeInit();
    }
  }
}

/**
  * @brief  Set/Unset the ACCELERO in low power mode.
  * @param  status 0 means disable Low Power Mode, otherwise Low Power Mode is enabled
  * @retval None
  */
void BSP_ACCELERO_LowPower(uint16_t status)
{
  /* Set/Unset the ACCELERO in low power mode */
  if(AccelerometerDrv != NULL)
  {
    if(AccelerometerDrv->LowPower != NULL)
    {
      AccelerometerDrv->LowPower(status);
    }
  }
}

/**
  * @brief  Get XYZ acceleration values.
  * @param  pDataXYZ Pointer on 3 angular accelerations table with  
  *                  pDataXYZ[0] = X axis, pDataXYZ[1] = Y axis, pDataXYZ[2] = Z axis
  * @retval None
  */
void BSP_ACCELERO_AccGetXYZ(int16_t *pDataXYZ)
{
  if(AccelerometerDrv != NULL)
  {
    if(AccelerometerDrv->GetXYZ != NULL)
    {   
      AccelerometerDrv->GetXYZ(pDataXYZ);
    }
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


