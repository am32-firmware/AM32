/**
  ******************************************************************************
  * @file    stm32l4r9i_eval_dsi_lcd.h
  * @author  MCD Application Team
  * @brief   This file contains the common defines and functions prototypes for
  *          the stm32l4r9i_eval_dsi_lcd.c driver.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32L4R9I_EVAL_DSI_LCD_H
#define __STM32L4R9I_EVAL_DSI_LCD_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Include LCD component Driver */

#include "stm32l4r9i_eval.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L4R9I_EVAL
  * @{
  */

/** @defgroup STM32L4R9I_EVAL_LCD STM32L4R9I_EVAL LCD
  * @{
  */

/** @defgroup STM32L4R9I_EVAL_DSI_LCD STM32L4R9I_EVAL DSI LCD
  * @{
  */

/** @addtogroup STM32L4R9I_EVAL_DSI_LCD_Exported_Variables
  * @{
  */

/* DMA2D handle variable */
extern DMA2D_HandleTypeDef hdma2d_eval;

/**
  * @}
  */

/** @addtogroup STM32L4R9I_EVAL_DSI_LCD_Exported_Functions
  * @{
  */

uint8_t  BSP_DSI_LCD_Init(void);
uint8_t  BSP_DSI_LCD_DeInit(void);

uint32_t BSP_DSI_LCD_GetXSize(void);
uint32_t BSP_DSI_LCD_GetYSize(void);

uint8_t  BSP_DSI_LCD_SetTransparency(uint32_t LayerIndex, uint8_t Transparency);
uint8_t  BSP_DSI_LCD_SetColorKeying(uint32_t LayerIndex, uint32_t RGBValue);
uint8_t  BSP_DSI_LCD_ResetColorKeying(uint32_t LayerIndex);

uint8_t  BSP_DSI_LCD_SelectLayer(uint32_t LayerIndex);
uint8_t  BSP_DSI_LCD_SetLayerVisible(uint32_t LayerIndex, FunctionalState State);

void     BSP_DSI_LCD_SetTextColor(uint32_t Color);
uint32_t BSP_DSI_LCD_GetTextColor(void);
void     BSP_DSI_LCD_SetBackColor(uint32_t Color);
uint32_t BSP_DSI_LCD_GetBackColor(void);
void     BSP_DSI_LCD_SetFont(sFONT *fonts);
sFONT    *BSP_DSI_LCD_GetFont(void);

uint32_t BSP_DSI_LCD_ReadPixel(uint16_t Xpos, uint16_t Ypos);
void     BSP_DSI_LCD_DrawPixel(uint16_t Xpos, uint16_t Ypos, uint32_t pixel);
void     BSP_DSI_LCD_Clear(uint32_t Color);
void     BSP_DSI_LCD_ClearStringLine(uint32_t Line);
void     BSP_DSI_LCD_DisplayStringAtLine(uint16_t Line, uint8_t *ptr);
void     BSP_DSI_LCD_DisplayStringAt(uint16_t Xpos, uint16_t Ypos, uint8_t *Text, Text_AlignModeTypdef Mode);
void     BSP_DSI_LCD_DisplayChar(uint16_t Xpos, uint16_t Ypos, uint8_t Ascii);

void     BSP_DSI_LCD_DrawHLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length);
void     BSP_DSI_LCD_DrawVLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length);
void     BSP_DSI_LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void     BSP_DSI_LCD_DrawRect(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height);
void     BSP_DSI_LCD_DrawCircle(uint16_t Xpos, uint16_t Ypos, uint16_t Radius);
void     BSP_DSI_LCD_DrawPolygon(pPoint Points, uint16_t PointCount);
void     BSP_DSI_LCD_DrawEllipse(int Xpos, int Ypos, int XRadius, int YRadius);
void     BSP_DSI_LCD_DrawBitmap(uint32_t Xpos, uint32_t Ypos, uint8_t *pbmp);

void     BSP_DSI_LCD_FillRect(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height);
void     BSP_DSI_LCD_FillCircle(uint16_t Xpos, uint16_t Ypos, uint16_t Radius);
void     BSP_DSI_LCD_FillPolygon(pPoint Points, uint16_t PointCount);
void     BSP_DSI_LCD_FillEllipse(int Xpos, int Ypos, int XRadius, int YRadius);

void     BSP_DSI_LCD_DisplayOff(void);
void     BSP_DSI_LCD_DisplayOn(void);

void     BSP_DSI_LCD_Refresh(void);
uint8_t  BSP_DSI_LCD_IsFrameBufferAvailable(void);

void     BSP_DSI_LCD_SetBrightness(uint8_t BrightnessValue);

/* These __weak functions can be surcharged by application code for specific application needs */
void     BSP_DSI_LCD_MspInit(void);
void     BSP_DSI_LCD_MspDeInit(void);
void     BSP_DSI_LCD_DMA2D_IRQHandler(void);
void     BSP_DSI_LCD_DSI_IRQHandler(void);
void     BSP_DSI_LCD_LTDC_IRQHandler(void);
void     BSP_DSI_LCD_LTDC_ER_IRQHandler(void);

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

#ifdef __cplusplus
}
#endif

#endif /* __STM32L4R9I_EVAL_DSI_LCD_H */


