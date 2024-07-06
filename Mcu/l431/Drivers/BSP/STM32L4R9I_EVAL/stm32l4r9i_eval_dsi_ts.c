/**
  ******************************************************************************
  * @file    stm32l4r9i_eval_dsi_ts.c
  * @author  MCD Application Team
  * @brief   This file provides a set of functions needed to manage the Touch
  *          Screen on DSI LCD of STM32L4R9I-EVAL evaluation board.
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

/* File Info : -----------------------------------------------------------------
                                   User NOTES
1. How To use this driver:
--------------------------
   - This driver is used to drive the touch screen module of the STM32L4R9I-EVAL
     evaluation board on the DSI LCD mounted on MB1314 daughter board.
     The touch screen driver IC is a FT3X67 type.

2. Driver description:
---------------------
  + Initialization steps:
     o Initialize the TS module using the BSP_TS_Init() function. This
       function includes the MSP layer hardware resources initialization and the
       communication layer configuration to start the TS use. The LCD size properties
       (x and y) are passed as parameters.
     o If TS interrupt mode is desired, you must configure the TS interrupt mode
       by calling the function BSP_TS_ITConfig(). The TS interrupt mode is generated
       as an external interrupt whenever a touch is detected.

  + Touch screen use
     o The touch screen state is captured whenever the function BSP_TS_GetState() is
       used. This function returns information about the last LCD touch occurred
       in the TS_StateTypeDef structure.
     o FT3X67 doesn't support weight and area features (always 0 value returned).

------------------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "stm32l4r9i_eval_ts.h"
#include "math.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L4R9I_EVAL
  * @{
  */

#if defined(USE_GVO_390x390)

/** @addtogroup STM32L4R9I_EVAL_DSI_TS STM32L4R9I_EVAL DSI TS
  * @{
  */

/** @defgroup STM32L4R9I_EVAL_DSI_TS_Private_Variables Private Variables
  * @{
  */

static TS_DrvTypeDef *tsDriver;
static uint8_t        I2C_Address   = 0;
static uint8_t        HwRotation    = 0;

/**
  * @}
  */

/** @addtogroup STM32L4R9I_EVAL_DSI_TS_Exported_Functions Exported Functions
  * @{
  */

/**
  * @brief  Initializes and configures the touch screen functionalities and
  *         configures all necessary hardware resources (GPIOs, I2C, clocks..).
  * @param  ts_SizeX : Maximum X size of the TS area on LCD
  * @param  ts_SizeY : Maximum Y size of the TS area on LCD
  * @retval TS_OK if all initializations are OK. Other value if error.
  */
uint8_t BSP_DSI_TS_Init(uint16_t ts_SizeX, uint16_t ts_SizeY)
{
  uint8_t  status  = TS_OK;

  /* Initialize the communication channel to sensor (I2C) if necessary */
  /* that is initialization is done only once after a power up         */
  ft3x67_ts_drv.Init(TS_DSI_I2C_ADDRESS);

  /* Scan FT3X67 TouchScreen IC controller ID register by I2C Read */
  /* Verify this is a FT3X67, otherwise this is an error case      */
  if(ft3x67_ts_drv.ReadID(TS_DSI_I2C_ADDRESS) == FT3X67_ID_VALUE)
  {
    /* Found FT3X67 : Initialize the TS driver structure */
    tsDriver    = &ft3x67_ts_drv;
    I2C_Address = TS_DSI_I2C_ADDRESS;

    /* Calibrate, Configure and Start the TouchScreen driver */
    tsDriver->Start(I2C_Address);

    /* Read firmware register to know if HW rotation is implemented */
    if(TS_IO_Read(I2C_Address, FT3X67_FIRMID_REG) != 0x01)
    {
      HwRotation = 1;
    }
  }
  else
  {
    status = TS_DEVICE_NOT_FOUND;
  }
  return status;
}

/**
  * @brief  DeInitializes the TouchScreen.
  * @retval TS state
  */
uint8_t BSP_DSI_TS_DeInit(void)
{
  /* Actually ts_driver does not provide a DeInit function */
  return TS_OK;
}

/**
  * @brief  Returns status and positions of the touch screen.
  * @param  TS_State: Pointer to touch screen current state structure
  * @retval TS_OK if OK. Other value if error.
  * @note   FT3X67 doesn't support weight and area features (always 0 value returned).
  */
uint8_t BSP_DSI_TS_GetState(TS_StateTypeDef *TS_State)
{
  static uint32_t xf[TS_MAX_NB_TOUCH] = {0, 0}; /* Final x value */
  static uint32_t yf[TS_MAX_NB_TOUCH] = {0, 0}; /* Final y value */
  uint8_t  ts_status = TS_OK;
  uint16_t x[TS_MAX_NB_TOUCH]; /* Initial x value */
  uint16_t y[TS_MAX_NB_TOUCH]; /* Initial y value */
  int16_t  xc, yc;   /* Coordinates transfered to center screen */
  int16_t  xr, yr;   /* Coordinates after rotation */
  uint32_t index;
  uint32_t weight = 0;
  uint32_t area   = 0;
  uint32_t event  = 0;

  /* Check and update the number of touches active detected */
  TS_State->touchDetected = tsDriver->DetectTouch(I2C_Address);
  if(TS_State->touchDetected)
  {
    for(index = 0; index < TS_State->touchDetected; index++)
    {
      /* Get each touch coordinates */
      tsDriver->GetXY(I2C_Address, &(x[index]), &(y[index]));

      if(HwRotation == 0)
      {
        /* First translate coordonates to center screen */
        xc = x[index] - 195;
        yc = y[index] - 195;

        /* Apply rotation of 45° */
        xr = (int16_t) (sqrt(2) * (xc - yc) / 2);
        yr = (int16_t) (sqrt(2) * (xc + yc) / 2);

        /* Revert the initial translation */
        xf[index] = xr + 195;
        yf[index] = yr + 195;

        TS_State->touchX[index] = xf[index];
        TS_State->touchY[index] = yf[index];
      }
      else
      {
        TS_State->touchX[index] = x[index];
        TS_State->touchY[index] = y[index];
      }

      /* Get touch info related to the current touch */
      ft3x67_TS_GetTouchInfo(I2C_Address, index, &weight, &area, &event);

      /* Update TS_State structure */
      TS_State->touchWeight[index] = weight;
      TS_State->touchArea[index]   = area;

      /* Remap touch event */
      switch(event)
      {
        case FT3X67_TOUCH_EVT_FLAG_PRESS_DOWN  :
          TS_State->touchEventId[index] = TOUCH_EVENT_PRESS_DOWN;
          break;
        case FT3X67_TOUCH_EVT_FLAG_LIFT_UP :
          TS_State->touchEventId[index] = TOUCH_EVENT_LIFT_UP;
          break;
        case FT3X67_TOUCH_EVT_FLAG_CONTACT :
          TS_State->touchEventId[index] = TOUCH_EVENT_CONTACT;
          break;
        case FT3X67_TOUCH_EVT_FLAG_NO_EVENT :
          TS_State->touchEventId[index] = TOUCH_EVENT_NO_EVT;
          break;
        default :
          ts_status = TS_ERROR;
          break;
      } /* of switch(event) */

    } /* of for(index=0; index < TS_State->touchDetected; index++) */
  } /* end of if(TS_State->touchDetected != 0) */

  return (ts_status);
}

/**
  * @brief  Configure gesture feature.
  * @param  State: Enable/Disable gesture feature.
  * @retval TS_OK if OK. Other value if error.
  */
uint8_t BSP_DSI_TS_GestureConfig(FunctionalState State)
{
  uint8_t  ts_status = TS_OK;
  uint32_t Activation;

  /* Configure gesture feature */
  Activation = (State == ENABLE) ? FT3X67_GESTURE_ENABLE : FT3X67_GESTURE_DISABLE;
  ft3x67_TS_GestureConfig(I2C_Address, Activation);

  return(ts_status);
}

/**
  * @brief  Update gesture Id following a touch detected.
  * @param  TS_State: Pointer to touch screen current state structure
  * @retval TS_OK if OK. Other value if error.
  */
uint8_t BSP_DSI_TS_Get_GestureId(TS_StateTypeDef *TS_State)
{
  uint32_t gestureId = 0;
  uint8_t  ts_status = TS_OK;

  /* Get gesture Id */
  ft3x67_TS_GetGestureID(I2C_Address, &gestureId);

  /* Remap gesture Id to a TS_GestureIdTypeDef value */
  switch(gestureId)
  {
    case FT3X67_GEST_ID_NO_GESTURE :
      TS_State->gestureId = GEST_ID_NO_GESTURE;
      break;
    case FT3X67_GEST_ID_MOVE_UP :
      TS_State->gestureId = GEST_ID_MOVE_UP;
      break;
    case FT3X67_GEST_ID_MOVE_RIGHT :
      TS_State->gestureId = GEST_ID_MOVE_RIGHT;
      break;
    case FT3X67_GEST_ID_MOVE_DOWN :
      TS_State->gestureId = GEST_ID_MOVE_DOWN;
      break;
    case FT3X67_GEST_ID_MOVE_LEFT :
      TS_State->gestureId = GEST_ID_MOVE_LEFT;
      break;
    case FT3X67_GEST_ID_DOUBLE_CLICK :
      TS_State->gestureId = GEST_ID_DOUBLE_CLICK;
      break;
    default :
      ts_status = TS_ERROR;
      break;
  } /* of switch(gestureId) */

  return(ts_status);
}

/**
  * @brief  Function used to reset all touch data before a new acquisition
  *         of touch information.
  * @param  TS_State: Pointer to touch screen current state structure
  * @retval TS_OK if OK, TS_ERROR if problem found.
  */
uint8_t BSP_DSI_TS_ResetTouchData(TS_StateTypeDef *TS_State)
{
  uint8_t  ts_status = TS_ERROR;
  uint32_t index;

  if (TS_State != (TS_StateTypeDef *)NULL)
  {
    TS_State->gestureId     = GEST_ID_NO_GESTURE;
    TS_State->touchDetected = 0;

    for(index = 0; index < TS_MAX_NB_TOUCH; index++)
    {
      TS_State->touchX[index]       = 0;
      TS_State->touchY[index]       = 0;
      TS_State->touchArea[index]    = 0;
      TS_State->touchEventId[index] = TOUCH_EVENT_NO_EVT;
      TS_State->touchWeight[index]  = 0;
    }

    ts_status = TS_OK;
  } /* of if (TS_State != (TS_StateTypeDef *)NULL) */

  return (ts_status);
}

/**
  * @brief  Configures and enables the touch screen interrupts.
  * @retval TS_OK if OK. Other value if error.
  */
uint8_t BSP_DSI_TS_ITConfig(void)
{
  GPIO_InitTypeDef gpio_init_structure;

  /* Configure Interrupt mode for TS detection pin */
  gpio_init_structure.Pin = TS_DSI_INT_PIN;
  gpio_init_structure.Pull = GPIO_PULLUP;
  gpio_init_structure.Speed = GPIO_SPEED_FAST;
  gpio_init_structure.Mode = GPIO_MODE_IT_FALLING;
  HAL_GPIO_Init(TS_DSI_INT_GPIO_PORT, &gpio_init_structure);

  /* Enable and set Touch screen EXTI Interrupt to the lowest priority */
  HAL_NVIC_SetPriority((IRQn_Type)(TS_DSI_INT_EXTI_IRQn), 0x0F, 0x00);
  HAL_NVIC_EnableIRQ((IRQn_Type)(TS_DSI_INT_EXTI_IRQn));

  /* Enable the TS ITs */
  tsDriver->EnableIT(I2C_Address);

  return TS_OK;
}

/**
  * @brief  Clears all touch screen interrupts.
  */
void BSP_DSI_TS_ITClear(void)
{
  /* Empty function on component FT3X67 */
}

/**
  * @brief  Gets the touch screen interrupt status.
  * @retval TS_OK if OK. Other value if error.
  */
uint8_t BSP_DSI_TS_ITGetStatus(void)
{
  /* Return the TS_OK because feature not available on FT3X67 */
  return TS_OK;
}

/**
  * @}
  */

/**
  * @}
  */

#endif /* USE_GVO_390x390 */

/**
  * @}
  */

/**
  * @}
  */


