/**
  ******************************************************************************
  * @file    cy8c4014lqi.h
  * @author  MCD Application Team
  * @brief   This file contains all the functions prototypes for the
  *          cy8c4014lqi.c touch screen driver.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CY8C4014LQI_H
#define __CY8C4014LQI_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "../Common/ts.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup Component
  * @{
  */

/** @defgroup CY8C4014LQI
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/** @defgroup CY8C4014LQI_Exported_Constants
  * @{
  */
#define CY8C4014LQI_ADDR         (0x08<<1)

#define CY8C4014LQI_ADEVICE_ID    (0x00)
#define CY8C4014LQI_VERSION       (0x01)
#define CY8C4014LQI_SYSMODE  	  (0x03)
#define CY8C4014LQI_TOUCH_EVENT   (0x04)
#define CY8C4014LQI_POS_X         (0x06)
#define CY8C4014LQI_POS_Y         (0x07)
#define CY8C4014LQI_GESTURE       (0x08)
#define CY8C4014LQI_DISTENCE_X    (0x09)
#define CY8C4014LQI_DISTENCE_Y    (0x0A)
#define DATA_VALUE_FLAG           (0x0B)


/* Possible values of global variable 'TS_I2C_Initialized' */
#define CY8C4014LQI_I2C_NOT_INITIALIZED          0x00U
#define CY8C4014LQI_I2C_INITIALIZED              0x01U


/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/** @defgroup CY8C4014LQI_Exported_Functions
  * @{
  */

void     cy8c4014lqi_Init(uint16_t DeviceAddr);
void     cy8c4014lqi_Reset(uint16_t DeviceAddr);
uint16_t cy8c4014lqi_ReadID(uint16_t DeviceAddr);
void     cy8c4014lqi_TS_Start(uint16_t DeviceAddr);
uint8_t  cy8c4014lqi_TS_DetectTouch(uint16_t DeviceAddr);
void     cy8c4014lqi_TS_GetXY(uint16_t DeviceAddr, uint16_t *X, uint16_t *Y);
void     cy8c4014lqi_TS_EnableIT(uint16_t DeviceAddr);
void     cy8c4014lqi_TS_DisableIT(uint16_t DeviceAddr);
uint8_t  cy8c4014lqi_TS_ITStatus (uint16_t DeviceAddr);
void     cy8c4014lqi_TS_ClearIT (uint16_t DeviceAddr);
void     cy8c4014lqi_TS_GestureConfig(uint16_t DeviceAddr, uint32_t Activation);
void     cy8c4014lqi_TS_GetGestureID(uint16_t DeviceAddr, uint32_t * pGestureId);
void     cy8c4014lqi_TS_GetTouchInfo(uint16_t   DeviceAddr,
                                uint32_t   touchIdx,
                                uint32_t * pWeight,
                                uint32_t * pArea,
                                uint32_t * pEvent);

/**
  * @}
  */

/* Imported TS IO functions --------------------------------------------------------*/

/** @defgroup CY8C4014LQI_Imported_Functions
  * @{
  */

/* TouchScreen (TS) external IO functions */
extern void     TS_IO_Init(void);
extern void     TS_IO_Write(uint8_t Addr, uint8_t Reg, uint8_t Value);
extern uint8_t  TS_IO_Read(uint8_t Addr, uint8_t Reg);
extern uint16_t TS_IO_ReadMultiple(uint8_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length);
extern void     TS_IO_Delay(uint32_t Delay);

/**
  * @}
  */

/* Imported global variables --------------------------------------------------------*/

/** @defgroup CY8C4014LQI_Imported_Globals
  * @{
  */

/* Touch screen driver structure */
extern TS_DrvTypeDef cy8c4014lqi_ts_drv;

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

#ifdef __cplusplus
}
#endif
#endif /* __CY8C4014LQI_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
