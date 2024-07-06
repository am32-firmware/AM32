/**
  ******************************************************************************
  * @file    stm32l476g_discovery_compass.c
  * @author  MCD Application Team
  * @brief   This file provides a set of functions needed to manage the E-Compass
  *          (ACCELEROMETER + MAGNETOMETER) MEMS LSM303C available on STM32L476G-Discovery
  *          board.
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
#include "stm32l476g_discovery.h"
#include "stm32l476g_discovery_compass.h"
#include "../Components/lsm303c/lsm303c.h"
#include <math.h>

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L476G_DISCOVERY
  * @{
  */

/** @defgroup STM32L476G_DISCOVERY_COMPASS STM32L476G-DISCOVERY COMPASS
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/** @defgroup STM32L476G_DISCOVERY_COMPASS_Private_Types Private Types
  * @{
  */
/**
  * @}
  */

/* Private defines ------------------------------------------------------------*/
/** @defgroup STM32L476G_DISCOVERY_COMPASS_Private_Constants Private Constants
  * @{
  */
/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/** @defgroup STM32L476G_DISCOVERY_COMPASS_Private_Macros Private Macros
  * @{
  */
/**
  * @}
  */

/* Private variables ---------------------------------------------------------*/
/** @defgroup STM32L476G_DISCOVERY_COMPASS_Private_Variables Private Variables
  * @{
  */
static ACCELERO_DrvTypeDef *AccelerometerDrv;
static MAGNETO_DrvTypeDef  *MagnetoDrv;

/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/
/** @addtogroup STM32L476G_DISCOVERY_COMPASS_Private_FunctionPrototypes Private Functions
  * @{
  */
/**
  * @}
  */

/* Exported functions ---------------------------------------------------------*/
/** @addtogroup STM32L476G_DISCOVERY_COMPASS_Exported_Functions
  * @{
  */
extern void  ACCELERO_IO_DeInit(void);
extern void  MAGNETO_IO_DeInit(void);

/**
  * @brief  Initialize the COMPASS.
  * @retval COMPASS_OK or COMPASS_ERROR
  */
COMPASS_StatusTypeDef BSP_COMPASS_Init(void)
{
  COMPASS_StatusTypeDef ret = COMPASS_OK;
  uint16_t ctrl = 0x0000;
  ACCELERO_InitTypeDef LSM303C_InitStructure;
  ACCELERO_FilterConfigTypeDef LSM303C_FilterStructure;
  MAGNETO_InitTypeDef LSM303C_InitStructureMag;

  if (Lsm303cDrv_accelero.ReadID() != LMS303C_ACC_ID)
  {
    ret = COMPASS_ERROR;
  }
  else
  {
    /* Initialize the COMPASS accelerometer driver structure */
    AccelerometerDrv = &Lsm303cDrv_accelero;

    /* MEMS configuration ------------------------------------------------------*/
    /* Fill the COMPASS accelerometer structure */
    LSM303C_InitStructure.AccOutput_DataRate = LSM303C_ACC_ODR_50_HZ;
    LSM303C_InitStructure.Axes_Enable = LSM303C_ACC_AXES_ENABLE;
    LSM303C_InitStructure.AccFull_Scale = LSM303C_ACC_FULLSCALE_2G;
    LSM303C_InitStructure.BlockData_Update = LSM303C_ACC_BDU_CONTINUOUS;
    LSM303C_InitStructure.High_Resolution = LSM303C_ACC_HR_DISABLE;
    LSM303C_InitStructure.Communication_Mode = LSM303C_ACC_SPI_MODE;

    /* Configure MEMS: data rate, power mode, full scale and axes */
    ctrl = (LSM303C_InitStructure.High_Resolution | LSM303C_InitStructure.AccOutput_DataRate | \
            LSM303C_InitStructure.Axes_Enable | LSM303C_InitStructure.BlockData_Update);

    ctrl |= (LSM303C_InitStructure.AccFull_Scale | LSM303C_InitStructure.Communication_Mode) << 8;

    /* Configure the COMPASS accelerometer main parameters */
    AccelerometerDrv->Init(ctrl);

    /* Fill the COMPASS accelerometer HPF structure */
    LSM303C_FilterStructure.HighPassFilter_Mode_Selection = LSM303C_ACC_HPM_NORMAL_MODE;
    LSM303C_FilterStructure.HighPassFilter_CutOff_Frequency = LSM303C_ACC_DFC1_ODRDIV50;
    LSM303C_FilterStructure.HighPassFilter_Stat = LSM303C_ACC_HPI2S_INT1_DISABLE | LSM303C_ACC_HPI2S_INT2_DISABLE;

    /* Configure MEMS: mode, cutoff frequency, Filter status, Click, AOI1 and AOI2 */
    ctrl = (uint8_t)(LSM303C_FilterStructure.HighPassFilter_Mode_Selection | \
                     LSM303C_FilterStructure.HighPassFilter_CutOff_Frequency | \
                     LSM303C_FilterStructure.HighPassFilter_Stat);

    /* Configure the COMPASS accelerometer LPF main parameters */
    AccelerometerDrv->FilterConfig(ctrl);
  }

  if (Lsm303cDrv_magneto.ReadID() != LMS303C_MAG_ID)
  {
    ret = COMPASS_ERROR;
  }
  else
  {
    /* Initialize the COMPASS magnetometer driver structure */
    MagnetoDrv = &Lsm303cDrv_magneto;

    /* MEMS configuration ------------------------------------------------------*/
    /* Fill the COMPASS magnetometer structure */
    LSM303C_InitStructureMag.Register1 = LSM303C_MAG_TEMPSENSOR_DISABLE | LSM303C_MAG_OM_XY_ULTRAHIGH | LSM303C_MAG_ODR_40_HZ;
    LSM303C_InitStructureMag.Register2 = LSM303C_MAG_FS_16_GA | LSM303C_MAG_REBOOT_DEFAULT | LSM303C_MAG_SOFT_RESET_DEFAULT;
    LSM303C_InitStructureMag.Register3 = LSM303C_MAG_SPI_MODE | LSM303C_MAG_CONFIG_NORMAL_MODE | LSM303C_MAG_CONTINUOUS_MODE;
    LSM303C_InitStructureMag.Register4 = LSM303C_MAG_OM_Z_ULTRAHIGH | LSM303C_MAG_BLE_LSB;
    LSM303C_InitStructureMag.Register5 = LSM303C_MAG_BDU_CONTINUOUS;
    /* Configure the COMPASS magnetometer main parameters */
    MagnetoDrv->Init(LSM303C_InitStructureMag);
  }

  return ret;
}

/**
  * @brief  DeInitialize the COMPASS.
  * @retval None.
  */
void BSP_COMPASS_DeInit(void)
{
  /* DeInitialize the COMPASS accelerometer & magnetometer IO interfaces */
  ACCELERO_IO_DeInit();
  MAGNETO_IO_DeInit();
}

/**
  * @brief  Put the COMPASS in low power mode.
  * @retval None
  */
void BSP_COMPASS_LowPower(void)
{
  /* Put the COMPASS accelerometer in low power mode */
  if (AccelerometerDrv != NULL)
  {
    if (AccelerometerDrv->LowPower != NULL)
    {
      AccelerometerDrv->LowPower(LSM303C_ACC_ODR_OFF);
    }
  }
  /* Put the COMPASS magnetometer in low power mode */
  if (MagnetoDrv != NULL)
  {
    if (MagnetoDrv->LowPower != NULL)
    {
      MagnetoDrv->LowPower(LSM303C_MAG_POWERDOWN2_MODE);
    }
  }
}

/**
  * @brief  Get XYZ acceleration values.
  * @param  pDataXYZ Pointer on 3 angular accelerations table with
  *                  pDataXYZ[0] = X axis, pDataXYZ[1] = Y axis, pDataXYZ[2] = Z axis
  * @retval None
  */
void BSP_COMPASS_AccGetXYZ(int16_t *pDataXYZ)
{
  if (AccelerometerDrv != NULL)
  {
    if (AccelerometerDrv->GetXYZ != NULL)
    {
      AccelerometerDrv->GetXYZ(pDataXYZ);
    }
  }
}

/**
  * @brief  Get XYZ magnetometer values.
  * @param  pDataXYZ Pointer on 3 magnetometer values table with
  *                  pDataXYZ[0] = X axis, pDataXYZ[1] = Y axis, pDataXYZ[2] = Z axis
  * @retval None
  */
void BSP_COMPASS_MagGetXYZ(int16_t *pDataXYZ)
{
  if (MagnetoDrv != NULL)
  {
    if (MagnetoDrv->GetXYZ != NULL)
    {
      MagnetoDrv->GetXYZ(pDataXYZ);
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


