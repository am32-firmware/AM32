/**
  ******************************************************************************
  * @file    stm32l4r9i_discovery_io.c
  * @author  MCD Application Team
  * @brief   This file provides a set of functions needed to manage the IO pins
  *          on STM32L4R9I_DISCOVERY discovery board.
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
   - This driver is used to drive the IO module of the STM32L4R9I_DISCOVERY
     board.
   - The MFXSTM32L152 IO expander device component driver must be included with this
     driver in order to run the IO functionalities commanded by the IO expander
     device mounted on the discovery board.

2. Driver description:
---------------------
  + Initialization steps:
     o Initialize the IO module using the BSP_IO_Init() function. This
       function includes the MSP layer hardware resources initialization and the
       communication layer configuration to start the IO functionalities use.

  + IO functionalities use
     o The IO pin mode is configured when calling the function BSP_IO_ConfigPin(), you
       must specify the desired IO mode by choosing the "IO_ModeTypedef" parameter
       predefined value.
     o If an IO pin is used in interrupt mode, the function BSP_IO_ITGetStatus() is
       needed to get the interrupt status. To clear the IT pending bits, you should
       call the function BSP_IO_ITClear() with specifying the IO pending bit to clear.
     o The IT is handled using the corresponding external interrupt IRQ handler,
       the user IT callback treatment is implemented on the same external interrupt
       callback.
     o To get/set an IO pin combination state you can use the functions
       BSP_IO_ReadPin()/BSP_IO_WritePin() or the function BSP_IO_TogglePin() to toggle the pin
       state.

------------------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "stm32l4r9i_discovery_io.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L4R9I_DISCOVERY
  * @{
  */

/** @defgroup STM32L4R9I_DISCOVERY_IO STM32L4R9I_DISCOVERY IO
  * @{
  */

/* Private variables ---------------------------------------------------------*/

/** @defgroup STM32L4R9I_DISCOVERY_IO_Private_Variables Private Variables
  * @{
  */
static IO_DrvTypeDef *io_driver;

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/

/** @addtogroup STM32L4R9I_DISCOVERY_IO_Exported_Functions
  * @{
  */

/**
  * @brief  Initialize and configure the IO functionalities and configures all
  *         necessary hardware resources (GPIOs, clocks..).
  * @note   BSP_IO_Init() is using HAL_Delay() function to ensure that stmpe811
  *         IO Expander is correctly reset. HAL_Delay() function provides accurate
  *         delay (in milliseconds) based on variable incremented in SysTick ISR.
  *         This implies that if BSP_IO_Init() is called from a peripheral ISR process,
  *         then the SysTick interrupt must have higher priority (numerically lower)
  *         than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
  * @retval IO_OK: if all initializations are OK. Other value if error.
  */
uint8_t BSP_IO_Init(void)
{
  uint8_t ret = IO_OK;
  uint8_t mfxstm32l152_id = 0;

  if (io_driver == NULL) /* Checks if MFX initialization has been already done */
  {
    mfxstm32l152_idd_drv.WakeUp(IO_I2C_ADDRESS);

    HAL_Delay(10);

    /* Read ID and verify the IO expander is ready */
    mfxstm32l152_id = mfxstm32l152_io_drv.ReadID(IO_I2C_ADDRESS);

    if((mfxstm32l152_id == MFXSTM32L152_ID_1) || (mfxstm32l152_id == MFXSTM32L152_ID_2))
    {
      /* Initialize the MFX */
      io_driver = &mfxstm32l152_io_drv;

      /* Initialize the MFX IO driver structure  */
      if(io_driver->Init != NULL)
      {
        io_driver->Init(IO_I2C_ADDRESS);
        io_driver->Start(IO_I2C_ADDRESS, IO_PIN_ALL);
      }
      else
      {
        ret = IO_ERROR;
      }
    }
    else
    {
      ret = IO_ERROR;
    }
  }
  else
  {
    /* MFX initialization already done : do nothing */
  }

  return ret;
}

/**
  * @brief  DeInitialize the IO to allow Mfx Initialization to be executed again
  * @note   BSP_IO_Init() has no effect if the io_driver is already initialized
  *         BSP_IO_DeInit() allows to erase the pointer such to allow init to be effective
  * @retval IO_OK
  */
uint8_t BSP_IO_DeInit(void)
{
  io_driver = NULL;
  return IO_OK;
}

/**
  * @brief  Get the selected pins IT status.
  * @param  IO_Pin: Selected pin(s) to check the status.
  *          This parameter can be any combination of the IO pins.
  * @retval Status of the checked IO pin(s).
  */
uint32_t BSP_IO_ITGetStatus(uint32_t IO_Pin)
{
  /* Return the IO Pin IT status */
  return (io_driver->ITStatus(IO_I2C_ADDRESS, IO_Pin));
}

/**
  * @brief  Clear the selected IO IT pending bit.
  * @param  IO_Pin: Selected pin(s) to clear the status.
  *          This parameter can be any combination of the IO pins.
  * @retval None
  */
void BSP_IO_ITClear(uint32_t IO_Pin)
{
  /* Clear the selected IO IT pending bits */
  io_driver->ClearIT(IO_I2C_ADDRESS, IO_Pin);
}

/**
  * @brief  Configure the IO pin(s) according to IO mode structure value.
  * @param  IO_Pin: Output IO pin(s) to be set or reset.
  *          This parameter can be any combination of the IO pin(s).
  * @param  IO_Mode: IO pin mode to configure
  *          This parameter can be one of the following values:
  *            @arg  IO_MODE_INPUT
  *            @arg  IO_MODE_OUTPUT
  *            @arg  IO_MODE_IT_RISING_EDGE
  *            @arg  IO_MODE_IT_FALLING_EDGE
  *            @arg  IO_MODE_IT_LOW_LEVEL
  *            @arg  IO_MODE_IT_HIGH_LEVEL
  * @retval IO_OK: if all initializations are OK. Other value if error.
  */
uint8_t BSP_IO_ConfigPin(uint32_t IO_Pin, IO_ModeTypedef IO_Mode)
{
  /* Configure the selected IO pin(s) mode */
  io_driver->Config(IO_I2C_ADDRESS, IO_Pin, IO_Mode);

  return IO_OK;
}

/**
  * @brief  Set the selected IO pin(s) state.
  * @param  IO_Pin: Selected IO pin(s) to write.
  *          This parameter can be any combination of the IO pin(s).
  * @param  PinState: New pin state to write
  * @retval None
  */
void BSP_IO_WritePin(uint32_t IO_Pin, uint8_t PinState)
{
  /* Set the IO pin(s) state */
  io_driver->WritePin(IO_I2C_ADDRESS, IO_Pin, PinState);
}

/**
  * @brief  Get the selected IO pin(s) current state.
  * @param  IO_Pin: Selected pin(s) to read.
  *          This parameter can be any combination of the IO pin(s).
  * @retval The current pins state
  */
uint32_t BSP_IO_ReadPin(uint32_t IO_Pin)
{
  return(io_driver->ReadPin(IO_I2C_ADDRESS, IO_Pin));
}

/**
  * @brief  Toggle the selected IO pin(s) state
  * @param  IO_Pin: Selected IO pin(s) to toggle.
  *          This parameter can be any combination of the IO pin(s).
  * @retval None
  */
void BSP_IO_TogglePin(uint32_t IO_Pin)
{
  /* Toggle the IO selected pin(s) state */
  if(io_driver->ReadPin(IO_I2C_ADDRESS, IO_Pin) != 0) /* Set */
  {
    io_driver->WritePin(IO_I2C_ADDRESS, IO_Pin, 0); /* Reset */
  }
  else
  {
    io_driver->WritePin(IO_I2C_ADDRESS, IO_Pin, 1);  /* Set */
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


