/**
  ******************************************************************************
  * @file    stm32l496g_discovery_ts.c
  * @author  MCD Application Team
  * @brief   This file provides a set of functions needed to manage the Touch
  *          Screen on STM32L496G-DISCOVERY evaluation board.
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
   - This driver is used to drive the touch screen module of the STM32L496G-DISCOVERY
     evaluation board on the FRIDA LCD mounted on MB1261 daughter board.
     The touch screen driver IC is a FT6x36 type which share the same register naming
     with FT6206 type.

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
------------------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "stm32l496g_discovery.h"
#include "stm32l496g_discovery_ts.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L496G-DISCOVERY
  * @{
  */

/** @defgroup STM32L496G-DISCOVERY_TS STM32L496G-DISCOVERY TS
  * @{
  */

/** @defgroup STM32L496G-DISCOVERY_TS_Private_Types_Definitions TS Private Types Definitions
  * @{
  */
/**
  * @}
  */

/** @defgroup STM32L496G-DISCOVERY_TS_Private_Defines TS Private Types Defines
  * @{
  */
/**
  * @}
  */

/** @defgroup STM32L496G-DISCOVERY_TS_Private_Macros TS Private Macros
  * @{
  */
/**
  * @}
  */

/** @defgroup STM32L496G-DISCOVERY_TS_Imported_Variables TS Imported Variables
  * @{
  */
/**
  * @}
  */

/** @defgroup STM32L496G-DISCOVERY_TS_Private_Variables TS Private Variables
  * @{
  */
static TS_DrvTypeDef *tsDriver;
static uint8_t  I2C_Address = 0;
static uint8_t  tsOrientation = TS_SWAP_NONE;
static uint8_t  TS_orientation = TS_ORIENTATION_UNDEFINED;

/* Table for touchscreen event information display on LCD : table indexed on enum @ref TS_TouchEventTypeDef information */
char *ts_event_string_tab[TOUCH_EVENT_NB_MAX] = { "None",
                                                  "Press down",
                                                  "Lift up",
                                                  "Contact"
                                                };

/* Table for touchscreen gesture Id information display on LCD : table indexed on enum @ref TS_GestureIdTypeDef information */
char *ts_gesture_id_string_tab[GEST_ID_NB_MAX] = { "None",
                                                   "Move Up",
                                                   "Move Right",
                                                   "Move Down",
                                                   "Move Left",
                                                   "Zoom In",
                                                   "Zoom Out"
                                                 };
/**
  * @}
  */

/** @defgroup STM32L496G-DISCOVERY_TS_Private_Function_Prototypes TS Private Function Prototypes
  * @{
  */
/**
  * @}
  */

/** @defgroup STM32L496G-DISCOVERY_TS_Public_Functions TS Public Functions
  * @{
  */
/**
  * @brief  Initializes and configures the touch screen functionalities and
  *         configures all necessary hardware resources (GPIOs, I2C, clocks..).
  * @param  ts_SizeX : Maximum X size of the TS area on LCD
  * @param  ts_SizeY : Maximum Y size of the TS area on LCD
  * @retval TS_OK if all initializations are OK. Other value if error.
  */
uint8_t BSP_TS_Init(uint16_t ts_SizeX, uint16_t ts_SizeY)
{
  return (BSP_TS_InitEx(ts_SizeX, ts_SizeY, TS_ORIENTATION_LANDSCAPE));
}

/**
  * @brief  Initializes and configures the touch screen functionalities and
  *         configures all necessary hardware resources (GPIOs, I2C, clocks..)
  *         with a given orientation
  * @param  ts_SizeX : Maximum X size of the TS area on LCD
  * @param  ts_SizeY : Maximum Y size of the TS area on LCD
  * @param  orientation : TS_ORIENTATION_LANDSCAPE or TS_ORIENTATION_PORTRAIT
  * @retval TS_OK if all initializations are OK. Other value if error.
  */
uint8_t BSP_TS_InitEx(uint16_t ts_SizeX, uint16_t ts_SizeY, uint8_t  orientation)
{
  uint8_t ts_status = TS_OK;
  uint16_t read_id = 0;

  /* Note : I2C_Address is un-initialized here, but is not used at all in init function */
  /* but the prototype of Init() is like that in template and should be respected       */

  /* Initialize the communication channel to sensor (I2C) if necessary */
  /* that is initialization is done only once after a power up         */
  ft6x06_ts_drv.Init(I2C_Address);

  /* Scan FT6x36 TouchScreen IC controller ID register by I2C Read */
  /* Verify this is a FT6x36, otherwise this is an error case      */

  read_id = ft6x06_ts_drv.ReadID(TS_I2C_ADDRESS);

  if (read_id == FT6x36_ID_VALUE)
  {
    /* Found FT6x36 : Initialize the TS driver structure */
    tsDriver = &ft6x06_ts_drv;

    I2C_Address    = TS_I2C_ADDRESS;

    /* Get LCD chosen orientation */
    if (orientation == TS_ORIENTATION_PORTRAIT)
    {
      tsOrientation = TS_SWAP_X | TS_SWAP_Y;
      TS_orientation = TS_ORIENTATION_PORTRAIT;
    }
    else
    {
      tsOrientation = TS_SWAP_XY | TS_SWAP_Y;
      TS_orientation = TS_ORIENTATION_LANDSCAPE;
    }

    if (ts_status == TS_OK)
    {
      /* Software reset the TouchScreen */
      tsDriver->Reset(I2C_Address);

      /* Calibrate, Configure and Start the TouchScreen driver */
      tsDriver->Start(I2C_Address);

      return TS_OK;

    } /* of if(ts_status == TS_OK) */
  }

  return TS_DEVICE_NOT_FOUND;


}

/**
  * @brief  Configures and enables the touch screen interrupts.
  * @retval TS_OK if all initializations are OK. Other value if error.
  */
uint8_t BSP_TS_ITConfig(void)
{
  uint8_t ts_status = TS_OK;

  /* Msp Init of GPIO used for TS_INT pin coming from TouchScreen driver IC FT6x36 */
  /* When touchscreen is operated in interrupt mode */
  BSP_TS_INT_MspInit();

  /* Enable the TS in interrupt mode */
  /* In that case the INT output of FT6206 when new touch is available */
  /* is active on low level and directed on EXTI */
  tsDriver->EnableIT(I2C_Address);

  return (ts_status);
}

/**
  * @brief  deConfigures and enables the touch screen interrupts.
  * @retval TS_OK if all initializations are OK. Other value if error.
  */
uint8_t BSP_TS_ITDeConfig(void)
{
  uint8_t ts_status = TS_OK;

  /* Msp Init of GPIO used for TS_INT pin coming from TouchScreen driver IC FT6x36 */
  /* When touchscreen is operated in interrupt mode */
  BSP_TS_INT_MspDeInit();

  /* Enable the TS in interrupt mode */
  /* In that case the INT output of FT6206 when new touch is available */
  /* is active on low level and directed on EXTI */
  tsDriver->DisableIT(I2C_Address);

  return (ts_status);
}


/**
  * @brief  Returns status and positions of the touch screen.
  * @param  TS_State: Pointer to touch screen current state structure
  * @retval TS_OK if all initializations are OK. Other value if error.
  */
uint8_t BSP_TS_GetState(TS_StateTypeDef *TS_State)
{
  static uint32_t _x[TS_MAX_NB_TOUCH] = {0, 0};
  static uint32_t _y[TS_MAX_NB_TOUCH] = {0, 0};
  uint8_t ts_status = TS_OK;
  uint16_t tmp;
  uint16_t Raw_x[TS_MAX_NB_TOUCH];
  uint16_t Raw_y[TS_MAX_NB_TOUCH];
  uint16_t xDiff;
  uint16_t yDiff;
  uint32_t index;
#if (TS_MULTI_TOUCH_SUPPORTED == 1)
  uint32_t weight = 0;
  uint32_t area = 0;
  uint32_t event = 0;
#endif /* TS_MULTI_TOUCH_SUPPORTED == 1 */

  /* Check and update the number of touches active detected */
  TS_State->touchDetected = tsDriver->DetectTouch(I2C_Address);
  if (TS_State->touchDetected)
  {
    for (index = 0; index < TS_State->touchDetected; index++)
    {
      /* Get each touch coordinates */
      tsDriver->GetXY(I2C_Address, &(Raw_x[index]), &(Raw_y[index]));

      if (tsOrientation & TS_SWAP_XY)
      {
        tmp = Raw_x[index];
        Raw_x[index] = Raw_y[index];
        Raw_y[index] = tmp;
      }

      if (tsOrientation & TS_SWAP_X)
      {
        Raw_x[index] = FT_6206_MAX_WIDTH_HEIGHT - 1 - Raw_x[index];
      }

      if (tsOrientation & TS_SWAP_Y)
      {
        Raw_y[index] = FT_6206_MAX_WIDTH_HEIGHT - 1 - Raw_y[index];
      }

      xDiff = Raw_x[index] > _x[index] ? (Raw_x[index] - _x[index]) : (_x[index] - Raw_x[index]);
      yDiff = Raw_y[index] > _y[index] ? (Raw_y[index] - _y[index]) : (_y[index] - Raw_y[index]);

      if ((xDiff + yDiff) > 5)
      {
        _x[index] = Raw_x[index];
        _y[index] = Raw_y[index];
      }


      TS_State->touchX[index] = _x[index];
      TS_State->touchY[index] = _y[index];

#if (TS_MULTI_TOUCH_SUPPORTED == 1)

      /* Get touch info related to the current touch */
      ft6x06_TS_GetTouchInfo(I2C_Address, index, &weight, &area, &event);

      /* Update TS_State structure */
      TS_State->touchWeight[index] = weight;
      TS_State->touchArea[index]   = area;

      /* Remap touch event */
      switch (event)
      {
        case FT6206_TOUCH_EVT_FLAG_PRESS_DOWN  :
          TS_State->touchEventId[index] = TOUCH_EVENT_PRESS_DOWN;
          break;
        case FT6206_TOUCH_EVT_FLAG_LIFT_UP :
          TS_State->touchEventId[index] = TOUCH_EVENT_LIFT_UP;
          break;
        case FT6206_TOUCH_EVT_FLAG_CONTACT :
          TS_State->touchEventId[index] = TOUCH_EVENT_CONTACT;
          break;
        case FT6206_TOUCH_EVT_FLAG_NO_EVENT :
          TS_State->touchEventId[index] = TOUCH_EVENT_NO_EVT;
          break;
        default :
          ts_status = TS_ERROR;
          break;
      } /* of switch(event) */

#endif /* TS_MULTI_TOUCH_SUPPORTED == 1 */

    } /* of for(index=0; index < TS_State->touchDetected; index++) */

#if (TS_MULTI_TOUCH_SUPPORTED == 1)
    /* Get gesture Id */
    ts_status = BSP_TS_Get_GestureId(TS_State);
#endif /* TS_MULTI_TOUCH_SUPPORTED == 1 */

  } /* end of if(TS_State->touchDetected != 0) */

  return (ts_status);
}

#if (TS_MULTI_TOUCH_SUPPORTED == 1)
/**
  * @brief  Update gesture Id following a touch detected.
  * @param  TS_State: Pointer to touch screen current state structure
  * @retval TS_OK if all initializations are OK. Other value if error.
  */
uint8_t BSP_TS_Get_GestureId(TS_StateTypeDef *TS_State)
{
  uint32_t gestureId = 0;
  uint8_t  ts_status = TS_OK;

  /* Get gesture Id */
  ft6x06_TS_GetGestureID(I2C_Address, &gestureId);

  /* Remap gesture Id to a TS_GestureIdTypeDef value */
  switch (gestureId)
  {
    case FT6206_GEST_ID_NO_GESTURE :
      TS_State->gestureId = GEST_ID_NO_GESTURE;
      break;
    case FT6206_GEST_ID_MOVE_UP :
      TS_State->gestureId = GEST_ID_MOVE_UP;
      break;
    case FT6206_GEST_ID_MOVE_RIGHT :
      TS_State->gestureId = GEST_ID_MOVE_RIGHT;
      break;
    case FT6206_GEST_ID_MOVE_DOWN :
      TS_State->gestureId = GEST_ID_MOVE_DOWN;
      break;
    case FT6206_GEST_ID_MOVE_LEFT :
      TS_State->gestureId = GEST_ID_MOVE_LEFT;
      break;
    case FT6206_GEST_ID_ZOOM_IN :
      TS_State->gestureId = GEST_ID_ZOOM_IN;
      break;
    case FT6206_GEST_ID_ZOOM_OUT :
      TS_State->gestureId = GEST_ID_ZOOM_OUT;
      break;
    default :
      ts_status = TS_ERROR;
      break;
  } /* of switch(gestureId) */

  return (ts_status);
}
#endif /* TS_MULTI_TOUCH_SUPPORTED == 1 */


/** @defgroup STM32L496G-DISCOVERY_TS_Private_Functions TS Private Functions
  * @{
  */

#if (TS_MULTI_TOUCH_SUPPORTED == 1)
/**
  * @brief  Function used to reset all touch data before a new acquisition
  *         of touch information.
  * @param  TS_State: Pointer to touch screen current state structure
  * @retval TS_OK if OK, TE_ERROR if problem found.
  */
uint8_t BSP_TS_ResetTouchData(TS_StateTypeDef *TS_State)
{
  uint8_t ts_status = TS_ERROR;
  uint32_t index;

  if (TS_State != (TS_StateTypeDef *)NULL)
  {
    TS_State->gestureId = GEST_ID_NO_GESTURE;
    TS_State->touchDetected = 0;

    for (index = 0; index < TS_MAX_NB_TOUCH; index++)
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
#endif /* TS_MULTI_TOUCH_SUPPORTED == 1 */


uint8_t BSP_TS_GetOrientation(void)
{
  return TS_orientation;
}

/**
  * @brief  Initializes the TS_INT pin MSP.
  * @param  None
  * @retval None
  */
__weak void BSP_TS_INT_MspInit(void)
{
  GPIO_InitTypeDef  gpio_init_structure;

  __HAL_RCC_PWR_CLK_ENABLE();
  SET_BIT(PWR->CR2, PWR_CR2_IOSV);

  TS_INT_GPIO_CLK_ENABLE();

  /* GPIO configuration in input for TouchScreen interrupt signal on TS_INT pin */
  gpio_init_structure.Pin       = TS_INT_PIN;

  /* Configure Interrupt mode for TS_INT pin falling edge : when a new touch is available */
  /* TS_INT pin is active on low level on new touch available */
  gpio_init_structure.Pin = TS_INT_PIN;
  gpio_init_structure.Pull = GPIO_PULLUP;
  gpio_init_structure.Speed = GPIO_SPEED_FAST;
  gpio_init_structure.Mode = GPIO_MODE_IT_FALLING;
  HAL_GPIO_Init(TS_INT_GPIO_PORT, &gpio_init_structure);

  /* Enable and set the TS_INT EXTI Interrupt to an intermediate priority */
  HAL_NVIC_SetPriority((IRQn_Type)(TS_INT_EXTI_IRQn), 0x08, 0x00);
  HAL_NVIC_EnableIRQ((IRQn_Type)(TS_INT_EXTI_IRQn));
}

/**
  * @brief  Initializes the TS_INT pin MSP.
  * @param  None
  * @retval None
  */
__weak void BSP_TS_INT_MspDeInit(void)
{
  GPIO_InitTypeDef  gpio_init_structure;

  __HAL_RCC_PWR_CLK_ENABLE();
  SET_BIT(PWR->CR2, PWR_CR2_IOSV);

  TS_INT_GPIO_CLK_ENABLE();

  /* GPIO configuration in input for TouchScreen interrupt signal on TS_INT pin */
  gpio_init_structure.Pin       = TS_INT_PIN;

  gpio_init_structure.Pin = GPIO_PIN_All;
  gpio_init_structure.Mode = GPIO_MODE_ANALOG;
  gpio_init_structure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TS_INT_GPIO_PORT, &gpio_init_structure);

  HAL_NVIC_DisableIRQ((IRQn_Type)(TS_INT_EXTI_IRQn));
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


