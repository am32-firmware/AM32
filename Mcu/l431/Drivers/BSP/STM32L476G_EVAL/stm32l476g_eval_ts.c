/**
  ******************************************************************************
  * @file    stm32l476g_eval_ts.c
  * @author  MCD Application Team
  * @brief   This file provides a set of functions needed to manage the touch 
  *          screen on STM32L476G-EVAL evaluation board.
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
  
/* File Info : -----------------------------------------------------------------
                                   User NOTES
1. How To use this driver:
--------------------------
   - This driver is used to drive the touch screen module of the STM32L476G-EVAL 
     evaluation board on the HX8347G LCD mounted on MB989 rev B daughter board .
   - The STMPE811 IO expander device component driver must be included with this 
     driver in order to run the TS module commanded by the IO expander device 
     mounted on the evaluation board.

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
     o If TS interrupt mode is used, the function BSP_TS_ITGetStatus() is needed to get
       the interrupt status. To clear the IT pending bits, you should call the 
       function BSP_TS_ITClear().
     o The IT is handled using the corresponding external interrupt IRQ handler,
       the user IT callback treatment is implemented on the same external interrupt
       callback.
 
------------------------------------------------------------------------------*/   

/* Includes ------------------------------------------------------------------*/
#include "stm32l476g_eval_ts.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L476G_EVAL
  * @{
  */

/** @defgroup STM32L476G_EVAL_TS STM32L476G_EVAL TS
  * @{
  */

/* Private types ---------------------------------------------------------*/

/** @defgroup STM32L476G_EVAL_TS_Private_Types_Definitions Private Types
  * @{
  */
/**
  * @}
  */

/* Private constants ---------------------------------------------------------*/

/** @defgroup STM32L476G_EVAL_TS_Private_Constants Private Constants
  * @{
  */
/**
  * @}
  */

/** @defgroup STM32L476G_EVAL_TS_Private_Macros Private Macros
  * @{
  */
/**
  * @}
  */

/* Private variables ---------------------------------------------------------*/

/** @defgroup STM32L476G_EVAL_TS_Private_Variables Private Variables
  * @{
  */
static TS_DrvTypeDef *ts_driver;
static uint16_t ts_x_boundary, ts_y_boundary; 
static uint8_t  ts_orientation;
/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/

/** @defgroup STM32L476G_EVAL_TS_Private_Functions Private Functions
  * @{
  */
/**
  * @}
  */

/* Exported functions ---------------------------------------------------------*/

/** @addtogroup STM32L476G_EVAL_TS_Exported_Functions
  * @{
  */

/**
  * @brief  Initializes and configures the touch screen functionalities and 
  *         configures all necessary hardware resources (GPIOs, clocks..).
  * @param  xSize: Maximum X size of the TS area on LCD
  * @param  ySize: Maximum Y size of the TS area on LCD  
  * @retval TS_OK if all initializations are OK. Other value if error.
  */
uint8_t BSP_TS_Init(uint16_t xSize, uint16_t ySize)
{
  uint8_t ret = TS_ERROR;
  
  if(stmpe811_ts_drv.ReadID(TS_I2C_ADDRESS) == STMPE811_ID)
  {
    /* Initialize the TS driver structure */
    ts_driver = &stmpe811_ts_drv;
    
    /* Initialize x and y positions boundaries */
    ts_x_boundary  = xSize;
    ts_y_boundary  = ySize;
    ts_orientation = TS_SWAP_Y | TS_SWAP_XY;
    ret = TS_OK;
  }
  
  if(ret == TS_OK)
  {
    /* Initialize the LL TS Driver */
    ts_driver->Init(TS_I2C_ADDRESS);
    ts_driver->Start(TS_I2C_ADDRESS);
  }  
  
  return ret;
}

/**
  * @brief  Configures and enables the touch screen interrupts.
  * @retval TS_OK if all initializations are OK. Other value if error.
  */
uint8_t BSP_TS_ITConfig(void)
{ 
  /* Call component driver to enable TS ITs */
  ts_driver->EnableIT(TS_I2C_ADDRESS);
  
  return TS_OK;  
}

/**
  * @brief  Gets the touch screen interrupt status.
  * @retval TS_OK if all initializations are OK. Other value if error.
  */
uint8_t BSP_TS_ITGetStatus(void)
{
  /* Call component driver to enable TS ITs */
  return (ts_driver->GetITStatus(TS_I2C_ADDRESS));
}

/**
  * @brief  Returns status and positions of the touch screen.
  * @param  TS_State: Pointer to touch screen current state structure
  * @retval TS_OK if all initializations are OK. Other value if error.
  */
uint8_t BSP_TS_GetState(TS_StateTypeDef *TS_State)
{
  static uint32_t _x = 0, _y = 0;
  uint16_t xDiff, yDiff , x , y;
  uint16_t swap;
  
  TS_State->TouchDetected = ts_driver->DetectTouch(TS_I2C_ADDRESS);
  
  if(TS_State->TouchDetected)
  {
    ts_driver->GetXY(TS_I2C_ADDRESS, &x, &y); 
    
    if(ts_orientation & TS_SWAP_X)
    {
      x = 4096 - x;  
    }
    
    if(ts_orientation & TS_SWAP_Y)
    {
      y = 4096 - y;
    }
    
    if(ts_orientation & TS_SWAP_XY)
    {
      swap = y; 
      y = x;      
      x = swap;      
    }

    xDiff = x > _x? (x - _x): (_x - x);
    yDiff = y > _y? (y - _y): (_y - y); 
    
    if (xDiff + yDiff > 5)
    {
      _x = x;
      _y = y; 
    }
    
    TS_State->x = (ts_x_boundary * _x) >> 12;
    TS_State->y = (ts_y_boundary * _y) >> 12; 
  }  
  
  return TS_OK;
}

/**
  * @brief  Clears all touch screen interrupts.
  * @retval None
  */
void BSP_TS_ITClear(void)
{
  ts_driver->ClearIT(TS_I2C_ADDRESS); 
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


