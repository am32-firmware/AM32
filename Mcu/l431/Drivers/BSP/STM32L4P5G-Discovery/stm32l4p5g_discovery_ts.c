/**
  ******************************************************************************
  * @file    stm32l4p5g_discovery_ts.c
  * @author  MCD Application Team
  * @brief   This file provides a set of functions needed to manage the Touch
  *          Screen on DSI LCD of STM32L4P5G_DISCOVERY discovery board.
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

/* File Info : -----------------------------------------------------------------
                                   User NOTES
1. How To use this driver:
--------------------------
   - This driver is used to drive the touch screen module of the STM32L4P5G_DISCOVERY
     discovery board on the DSI LCD mounted on MB1535 daughter board.
     The touch screen driver IC is a CY8C4014LQI type.

2. Driver description:
---------------------
  + Initialization steps:
     o Initialize the TS module using the BSP_TS_Init() function. This
       function includes the MSP layer hardware resources initialization and the
       communication layer configuration to start the TS use. The LCD size properties
       (x and y) are passed as parameters.
     o By default, the TS interrupt is enabled by calling the function BSP_TS_ITConfig().
       The TS interrupt mode is generated as an external interrupt whenever a touch is
       detected.

  + Touch screen use
     o The touch screen state is captured whenever the function BSP_TS_GetState() is
       used. This function returns information about the last LCD touch
       in the TS_StateTypeDef structure.
     o CY8C4014LQI doesn't support weight and area features (always 0 value returned).

  + Gesture detection
     o Gesture detection reports horizontal and vertical swipes
     o Initialization to record gesture starting point is achieved with BSP_TS_GestureInit()
     o BSP_TS_Get_GestureId() can be called anytime to report which gesture has been detected
       if any.

------------------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "stm32l4p5g_discovery_ts.h"
#include "math.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L4P5G_DISCOVERY
  * @{
  */

/** @addtogroup STM32L4P5G_DISCOVERY_TS STM32L4P5G_DISCOVERY TS
  * @{
  */

/** @defgroup STM32L4P5G_DISCOVERY_TS_Private_Variables Private Variables
  * @{
  */

static TS_DrvTypeDef *tsDriver;
static uint8_t        I2C_Address   = 0;
TS_StateTypeDef        TS_State={0};
extern volatile uint8_t is_touch;

/**
  * @}
  */

/** @addtogroup STM32L4P5G_DISCOVERY_TS_Exported_Functions Exported Functions
  * @{
  */

/**
  * @brief  Initialize and configures the touch screen functionalities and
  *         configures all necessary hardware resources (GPIOs, I2C, clocks..).
  * @param  ts_SizeX : Maximum X size of the TS area on LCD
  * @param  ts_SizeY : Maximum Y size of the TS area on LCD
  * @retval TS_OK if all initializations are OK. Other value if error.
  */
uint8_t BSP_TS_Init(uint16_t ts_SizeX, uint16_t ts_SizeY)
{
  uint8_t  status  = TS_OK;

  /* Initialize the communication channel to sensor (I2C) if necessary */
  /* that is initialization is done only once after a power up         */
  cy8c4014lqi_ts_drv.Init(TS_I2C_ADDRESS);


  /* Touch Screen reset */
  if (BSP_TS_ResetTouchData(&TS_State) !=  TS_OK)
  {
    status = TS_ERROR;
  }
  else
  {
    GPIO_InitTypeDef  GPIO_InitStruct;

    /* Enable the Touch Screen Interrupt GPIO clock */
    TS_INT_GPIO_CLK_ENABLE();

    /* TS_OUT_IRQ (normally used for EXTI_WKUP) */
    GPIO_InitStruct.Pin   = TS_INT_PIN;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Mode  = GPIO_MODE_IT_RISING_FALLING;
    HAL_GPIO_Init(TS_INT_GPIO_PORT, &GPIO_InitStruct);

    /* Ensure that CY8C4014LQI TouchScreen IC controller is ready */
    HAL_Delay(30);

    /* Scan CY8C4014LQI TouchScreen IC controller ID register by I2C Read */
    /* Verify this is a CY8C4014LQI, otherwise this is an error case      */
    if(cy8c4014lqi_ts_drv.ReadID(TS_I2C_ADDRESS) != 0xff)
    {
      /* Found CY8C4014LQI : Initialize the TS driver structure */
      tsDriver    = &cy8c4014lqi_ts_drv;
    }
    else
    {
      status = TS_DEVICE_NOT_FOUND;
    }
  }

  return status;
}

/**
  * @brief  Deinitialize the TouchScreen.
  * @retval TS state
  */
uint8_t BSP_TS_DeInit(void)
{
  /* Actually ts_driver does not provide a DeInit function */
  /* Just de-initialize the Touch Screen Interrupt GPIO */
  HAL_GPIO_DeInit(TS_INT_GPIO_PORT, TS_INT_PIN);

  return TS_OK;
}

/**
  * @brief  Return status and positions of the touch screen.
  * @param  TS_State: Pointer to touch screen current state structure
  * @retval TS_OK if OK. Other value if error.
  * @note   CY8C4014LQI doesn't support weight and area features (always 0 value returned).
  */
uint8_t BSP_TS_GetState(TS_StateTypeDef *TS_State)
{
  tsDriver->GetXY(I2C_Address, &TS_State->x, &TS_State->y);
  /* Update TouchDetected value */
  TS_State->TouchDetected = (HAL_GPIO_ReadPin(TS_INT_GPIO_PORT, TS_INT_PIN) == 0) ? 1 : 0;

  return TS_OK;
}

/**
  * @brief  Configure gesture feature.
  * @param  TS_State: Pointer to touch screen current state structure
  * @param  State: Enable/Disable gesture feature.
  * @retval TS_OK is OK.
  */
uint8_t BSP_TS_GestureConfig(TS_StateTypeDef *TS_State, FunctionalState State)
{
  uint8_t  ts_status = TS_OK;

  /* Return the TS_OK because feature not available on CY8C4014LQI */
  /* Achieved by software in BSP */
  return(ts_status);
}

/**
  * @brief  Initialize gesture feature in recording start position
  * @param  TS_State: Pointer to touch screen current state structure
  * @retval TS_OK if OK. Other value if error.
  */
uint8_t BSP_TS_GestureInit(TS_StateTypeDef *TS_State)
{
  uint8_t  ts_status = TS_OK;

    BSP_TS_GetState(TS_State);
    TS_State->x_start = TS_State->x;
    TS_State->y_start = TS_State->y;

  return(ts_status);
}

/**
  * @brief  Update gesture Id following a touch detected.
  * @param  TS_State: Pointer to touch screen current state structure
  * @retval TS_OK if OK. Other value if error.
  */
uint8_t BSP_TS_Get_GestureId(TS_StateTypeDef *TS_State)
{
  uint8_t  ts_status = TS_OK;
  int16_t diff_x;
  int16_t diff_y;
  uint16_t abs_diff_x;
  uint16_t abs_diff_y;
  uint32_t swipe = 0;

  BSP_TS_GetState(TS_State);

  /* Get gesture Id */
   diff_x = TS_State->x - TS_State->x_start;
   diff_y = TS_State->y - TS_State->y_start;
   if (diff_x >= 0)
   {
     abs_diff_x = diff_x;
   }
   else
   {
     abs_diff_x = -diff_x;
   }
   if (diff_y >= 0)
   {
     abs_diff_y = diff_y;
   }
   else
   {
     abs_diff_y = -diff_y;
   }

  swipe = (uint32_t)(sqrt((diff_x * diff_x) + (diff_y * diff_y)));

  if (swipe < 50)
  {
    TS_State->gestureId = GEST_ID_NO_GESTURE;
  }
  else
  {
    if ((abs_diff_y > abs_diff_x) && (diff_y > 0))
    {
      TS_State->gestureId = GEST_ID_MOVE_UP;
    }
    else if ((abs_diff_y > abs_diff_x) && (diff_y < 0))
    {
      TS_State->gestureId = GEST_ID_MOVE_DOWN;
    }
    else if ((abs_diff_y <= abs_diff_x) && (diff_x < 0))
    {
      TS_State->gestureId = GEST_ID_MOVE_LEFT;
    }
    else if ((abs_diff_y <= abs_diff_x) && (diff_x > 0))
    {
      TS_State->gestureId = GEST_ID_MOVE_RIGHT;
    }
    else
    {
      TS_State->gestureId = GEST_ID_NO_GESTURE;
    }
  }

  return(ts_status);
}

/**
  * @brief  Reset all touch data before a new acquisition
  *         of touch information.
  * @param  TS_State: Pointer to touch screen current state structure
  * @retval TS_OK if OK, TS_ERROR if problem found.
  */
uint8_t BSP_TS_ResetTouchData(TS_StateTypeDef *TS_State)
{
  uint8_t  ts_status = TS_ERROR;
  GPIO_InitTypeDef GPIO_InitStruct;

  if (TS_State != (TS_StateTypeDef *)NULL)
  {
    TS_State->gestureId             = GEST_ID_NO_GESTURE;
    TS_State->TouchDetected         = 0;
    TS_State->x                     = 0;
    TS_State->x_start               = 0;
    TS_State->y                     = 0;
    TS_State->y_start               = 0;

    /* Touch Screen RESET configuration */
    TS_RST_GPIO_CLK_ENABLE();

    GPIO_InitStruct.Pin   = TS_RST_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    HAL_GPIO_Init(TS_RST_GPIO_PORT, &GPIO_InitStruct);

    HAL_GPIO_WritePin(TS_RST_GPIO_PORT, TS_RST_PIN, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(TS_RST_GPIO_PORT, TS_RST_PIN, GPIO_PIN_SET);
    HAL_Delay(2);

    ts_status = TS_OK;
  } /* of if (TS_State != (TS_StateTypeDef *)NULL) */

  return (ts_status);
}

/**
  * @brief  Configure and enable the touch screen interrupts.
  * @retval TS_OK if OK. Other value if error.
  */
uint8_t BSP_TS_ITConfig(void)
{
  /* Enable and set GPIO EXTI Interrupt to the lowest priority */
  HAL_NVIC_SetPriority((IRQn_Type)(TS_INT_EXTI_IRQn), 0x0F, 0x00);
  HAL_NVIC_EnableIRQ((IRQn_Type)(TS_INT_EXTI_IRQn));

  return TS_OK;
}

/**
  * @brief  Deconfigure and disable the touch screen interrupts.
  * @retval TS_OK if OK. Other value if error.
  */
uint8_t BSP_TS_ITDisable(void)
{
  /* Disable GPIO EXTI Interrupt */
  HAL_NVIC_DisableIRQ((IRQn_Type)(TS_INT_EXTI_IRQn));

  return TS_OK;
}

/**
  * @brief  Clear all touch screen interrupts.
  */
void BSP_TS_ITClear(void)
{
  /* Empty function on component CY8C4014LQI */
}

/**
  * @brief  Get the touch screen interrupt status.
  * @retval TS_OK if OK. Other value if error.
  */
uint8_t BSP_TS_ITGetStatus(void)
{
  /* Return the TS_OK because feature not available on CY8C4014LQI */
  return TS_OK;
}

void BSP_TS_IRQHandler(void)
{
  uint8_t int_pin = HAL_GPIO_ReadPin(TS_INT_GPIO_PORT, TS_INT_PIN);

  if (int_pin == 0)
  {
    TS_State.TouchDetected = 1;
    /* The screen is touched */
    BSP_TS_PressCallback();
  }
  else
  {
    TS_State.TouchDetected = 0;
    /* The screen is released */
    BSP_TS_ReleaseCallback();
  }
}

/**
  * @brief  TouchScreen Press callback.
  */
__weak void BSP_TS_PressCallback(void)
{
  /* NOTE : This function should not be modified; when the callback is needed,
            the BSP_TS_PressCallback() can implemented in the user file
   */
}

/**
  * @brief  TouchScreen Release callback.
  */
__weak void BSP_TS_ReleaseCallback(void)
{
  /* NOTE : This function should not be modified; when the callback is needed,
            the BSP_TS_ReleaseCallback() can implemented in the user file
   */
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

/**
  * @}
  */


