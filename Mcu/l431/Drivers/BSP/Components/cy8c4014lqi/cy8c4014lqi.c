/**
  ******************************************************************************
  * @file    cy8c4014lqi.c
  * @author  MCD Application Team
  * @brief   This file provides a set of functions needed to manage the CY8C4014LQI
  *          touch screen devices.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "cy8c4014lqi.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup Component
  * @{
  */

/** @addtogroup CY8C4014LQI
  * @{
  */

/* Private typedef -----------------------------------------------------------*/

/** @defgroup CY8C4014LQI_Private_Types_Definitions
  * @{
  */

/* cy8c4014lqi Handle definition. */
typedef struct
{
  uint8_t i2cInitialized;

  /* field holding the current number of simultaneous active touches */
  uint8_t currActiveTouchNb;

  /* field holding the touch index currently managed */
  uint8_t currActiveTouchIdx;

} cy8c4014lqi_handle_TypeDef;

/**
  * @}
  */

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile uint8_t is_touch = 0;


/** @defgroup CY8C4014LQI_Private_Variables
  * @{
  */

/* Touch screen driver structure initialization */
TS_DrvTypeDef cy8c4014lqi_ts_drv =
{
  cy8c4014lqi_Init,
  cy8c4014lqi_ReadID,
  cy8c4014lqi_Reset,
  cy8c4014lqi_TS_Start,
  cy8c4014lqi_TS_DetectTouch,
  cy8c4014lqi_TS_GetXY,
  cy8c4014lqi_TS_EnableIT,
  cy8c4014lqi_TS_ClearIT,
  cy8c4014lqi_TS_ITStatus,
  cy8c4014lqi_TS_DisableIT
};

/* Global cy8c4014lqi handle */
static cy8c4014lqi_handle_TypeDef cy8c4014lqi_handle = { CY8C4014LQI_I2C_NOT_INITIALIZED, 0U, 0U};

/**
  * @}
  */

/* Private functions prototypes-----------------------------------------------*/

/** @defgroup CY8C4014LQI_Private_Functions
  * @{
  */

static uint8_t  cy8c4014lqi_Get_I2C_InitializedStatus(void);
static void     cy8c4014lqi_I2C_InitializeIfRequired(void);

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/

/** @addtogroup CY8C4014LQI_Exported_Functions
  * @{
  */

/**
  * @brief  Initialize the cy8c4014lqi communication bus
  *         from MCU to CY8C4014LQI : ie I2C channel initialization (if required).
  * @param  DeviceAddr: Device address on communication Bus (I2C slave address of CY8C4014LQI).
  * @retval None
  */
void cy8c4014lqi_Init(uint16_t DeviceAddr)
{
  /* Initialize I2C link if needed */
  cy8c4014lqi_I2C_InitializeIfRequired();
}

/**
  * @brief  Software Reset the cy8c4014lqi.
  *         @note : Not applicable to CY8C4014LQI.
  * @param  DeviceAddr: Device address on communication Bus (I2C slave address of CY8C4014LQI).
  * @retval None
  */
void cy8c4014lqi_Reset(uint16_t DeviceAddr)
{
  /* Do nothing */
  /* No software reset sequence available in CY8C4014LQI IC */
}

/**
  * @brief  Read the cy8c4014lqi device ID, pre initialize I2C in case of need to be
  *         able to read the CY8C4014LQI device ID, and verify this is a CY8C4014LQI.
  * @param  DeviceAddr: I2C CY8C4014LQI Slave address.
  * @retval The Device ID (two bytes).
  */
uint16_t cy8c4014lqi_ReadID(uint16_t DeviceAddr)
{
  /* Initialize I2C link if needed */
  cy8c4014lqi_I2C_InitializeIfRequired();

  /* Return the device ID value */
  return(TS_IO_Read(DeviceAddr, CY8C4014LQI_ADEVICE_ID));
}

/**
  * @brief  Configures the touch Screen IC device to start detecting touches
  * @param  DeviceAddr: Device address on communication Bus (I2C slave address).
  * @retval None.
  */
void cy8c4014lqi_TS_Start(uint16_t DeviceAddr)
{
  /* Do nothing */
  /* No software available in CY8C4014LQI IC */
}

/**
  * @brief  Return if there is touches detected or not.
  *         Try to detect new touches and forget the old ones (reset internal global
  *         variables).
  * @param  DeviceAddr: Device address on communication Bus.
  * @retval : Number of active touches detected (can be 0, 1 or 2).
  */
uint8_t cy8c4014lqi_TS_DetectTouch(uint16_t DeviceAddr)
{
      return is_touch;
}

/**
  * @brief  Get the touch screen X and Y positions values
  *         Manage multi touch thanks to touch Index global
  *         variable 'cy8c4014lqi_handle.currActiveTouchIdx'.
  * @param  DeviceAddr: Device address on communication Bus.
  * @param  X: Pointer to X position value
  * @param  Y: Pointer to Y position value
  * @retval None.
  */
void cy8c4014lqi_TS_GetXY(uint16_t DeviceAddr, uint16_t *X, uint16_t *Y)
{

    *X = TS_IO_Read(CY8C4014LQI_ADDR, CY8C4014LQI_POS_X);
    *Y = TS_IO_Read(CY8C4014LQI_ADDR, CY8C4014LQI_POS_Y);
}

/**
  * @brief  Configure the CY8C4014LQI device to generate IT on given INT pin
  *         connected to MCU as EXTI.
  * @param  DeviceAddr: Device address on communication Bus (Slave I2C address of CY8C4014LQI).
  * @retval None
  */
void cy8c4014lqi_TS_EnableIT(uint16_t DeviceAddr)
{
  /* Do nothing */
  /* Action done by BSP software */
}

/**
  * @brief  Configure the CY8C4014LQI device to stop generating IT on the given INT pin
  *         connected to MCU as EXTI.
  * @param  DeviceAddr: Device address on communication Bus (Slave I2C address of CY8C4014LQI).
  * @retval None
  */
void cy8c4014lqi_TS_DisableIT(uint16_t DeviceAddr)
{
  /* Do nothing */
  /* Action done by BSP software */
}

/**
  * @brief  Get IT status from CY8C4014LQI interrupt status registers
  *         Should be called Following an EXTI coming to the MCU to know the detailed
  *         reason of the interrupt.
  *         @note : This feature is not applicable to CY8C4014LQI.
  * @param  DeviceAddr: Device address on communication Bus (I2C slave address of CY8C4014LQI).
  * @retval TS interrupts status : always return 0 here
  */
uint8_t cy8c4014lqi_TS_ITStatus(uint16_t DeviceAddr)
{
  /* Always return 0 as feature not applicable to CY8C4014LQI */
  return 0U;
}

/**
  * @brief  Clear IT status in CY8C4014LQI interrupt status clear registers
  *         Should be called Following an EXTI coming to the MCU.
  *         @note : This feature is not applicable to CY8C4014LQI.
  * @param  DeviceAddr: Device address on communication Bus (I2C slave address of CY8C4014LQI).
  * @retval None
  */
void cy8c4014lqi_TS_ClearIT(uint16_t DeviceAddr)
{
  /* Do nothing */
  /* Action done by BSP software */
}

/**
  * @brief  Configure gesture feature (enable/disable).
  * @param  DeviceAddr: Device address on communication Bus (I2C slave address of CY8C4014LQI).
  * @param  Activation : Enable or disable gesture feature. Possible values are
  *         CY8C4014LQI_GESTURE_DISABLE or CY8C4014LQI_GESTURE_ENABLE.
  * @retval None.
  */
void cy8c4014lqi_TS_GestureConfig(uint16_t DeviceAddr, uint32_t Activation)
{
  /* Do nothing */
  /* No software available in CY8C4014LQI IC */
}

/**
  * @brief  Get the last touch gesture identification (zoom, move up/down...).
  * @param  DeviceAddr: Device address on communication Bus (I2C slave address of CY8C4014LQI).
  * @param  pGestureId : Pointer to get last touch gesture Identification.
  * @retval None.
  */
void cy8c4014lqi_TS_GetGestureID(uint16_t DeviceAddr, uint32_t * pGestureId)
{
  /* Do nothing */
  /* No software available in CY8C4014LQI IC */
}

/**
  * @brief  Get the touch detailed informations on touch number 'touchIdx' (0..1)
  *         This touch detailed information contains :
  *         - weight that was applied to this touch
  *         - sub-area of the touch in the touch panel
  *         - event of linked to the touch (press down, lift up, ...)
  * @param  DeviceAddr: Device address on communication Bus (I2C slave address of CY8C4014LQI).
  * @param  touchIdx : Passed index of the touch (0..1) on which we want to get the
  *                    detailed information.
  * @param  pWeight : Pointer to to get the weight information of 'touchIdx'.
  * @param  pArea   : Pointer to to get the sub-area information of 'touchIdx'.
  * @param  pEvent  : Pointer to to get the event information of 'touchIdx'.
  * @note   Area and Weight features are not supported by CY8C4014LQI. Return always 0 value.
  * @retval None.
  */
void cy8c4014lqi_TS_GetTouchInfo(uint16_t   DeviceAddr,
                            uint32_t   touchIdx,
                            uint32_t * pWeight,
                            uint32_t * pArea,
                            uint32_t * pEvent)
{
  /* Do nothing */
  /* No software available in CY8C4014LQI IC */
}

/**
  * @}
  */

/* Private functions bodies---------------------------------------------------*/

/** @addtogroup CY8C4014LQI_Private_Functions
  * @{
  */

/**
  * @brief  Return the status of I2C was initialized or not.
  * @param  None.
  * @retval : I2C initialization status.
  */
static uint8_t cy8c4014lqi_Get_I2C_InitializedStatus(void)
{
  return(cy8c4014lqi_handle.i2cInitialized);
}

/**
  * @brief  I2C initialize if needed.
  * @param  None.
  * @retval : None.
  */
static void cy8c4014lqi_I2C_InitializeIfRequired(void)
{
  if(cy8c4014lqi_Get_I2C_InitializedStatus() == CY8C4014LQI_I2C_NOT_INITIALIZED)
  {
    /* Initialize TS IO BUS layer (I2C) */
    TS_IO_Init();

    /* Set state to initialized */
    cy8c4014lqi_handle.i2cInitialized = CY8C4014LQI_I2C_INITIALIZED;
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


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
