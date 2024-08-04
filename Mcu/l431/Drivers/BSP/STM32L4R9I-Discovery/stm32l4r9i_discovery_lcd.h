/**
  ******************************************************************************
  * @file    stm32l4r9i_discovery_lcd.h
  * @author  MCD Application Team
  * @brief   This file contains the common defines and functions prototypes for
  *          the stm32l4r9i_discovery_lcd.c driver.
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
#ifndef __STM32L4R9I_DISCOVERY_LCD_H
#define __STM32L4R9I_DISCOVERY_LCD_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32l4r9i_discovery.h"
#include "../../../Utilities/Fonts/fonts.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L4R9I_DISCOVERY
  * @{
  */

/** @addtogroup STM32L4R9I_DISCOVERY_LCD STM32L4R9I_DISCOVERY LCD
  * @{
  */

/** @defgroup STM32L4R9I_DISCOVERY_LCD_Exported_Constants LCD Exported Constants
  * @{
  */

/* LTDC layer configuration structure */
#define LCD_LayerCfgTypeDef               LTDC_LayerCfgTypeDef

/* Maximum number of LTDC layers */
#define LTDC_MAX_LAYER_NUMBER             ((uint32_t) 2)

/* LTDC Background layer index */
#define LTDC_ACTIVE_LAYER_BACKGROUND      LTDC_LAYER_1

/* LTDC Foreground layer index : Not used on STM32L4R9I_DISCOVERY, only one layer used */
#define LTDC_ACTIVE_LAYER_FOREGROUND      LTDC_LAYER_2

/* Number of LTDC layers */
#define LTDC_NB_OF_LAYERS                 ((uint32_t) 2)

/* LTDC Default used layer index */
#define LTDC_DEFAULT_ACTIVE_LAYER         LTDC_ACTIVE_LAYER_BACKGROUND

/* LCD status */
#define   LCD_OK         0x00
#define   LCD_ERROR      0x01
#define   LCD_TIMEOUT    0x02

/* LCD color definitions values in ARGB8888 format */
/* Blue value in ARGB8888 format */
#define LCD_COLOR_BLUE          ((uint32_t) 0xFF0000FF)
/* Green value in ARGB8888 format */
#define LCD_COLOR_GREEN         ((uint32_t) 0xFF00FF00)
/* Red value in ARGB8888 format */
#define LCD_COLOR_RED           ((uint32_t) 0xFFFF0000)
/* Cyan value in ARGB8888 format */
#define LCD_COLOR_CYAN          ((uint32_t) 0xFF00FFFF)
/* Magenta value in ARGB8888 format */
#define LCD_COLOR_MAGENTA       ((uint32_t) 0xFFFF00FF)
/* Yellow value in ARGB8888 format */
#define LCD_COLOR_YELLOW        ((uint32_t) 0xFFFFFF00)
/* Light Blue value in ARGB8888 format */
#define LCD_COLOR_LIGHTBLUE     ((uint32_t) 0xFF8080FF)
/* Light Green value in ARGB8888 format */
#define LCD_COLOR_LIGHTGREEN    ((uint32_t) 0xFF80FF80)
/* Light Red value in ARGB8888 format */
#define LCD_COLOR_LIGHTRED      ((uint32_t) 0xFFFF8080)
/* Light Cyan value in ARGB8888 format */
#define LCD_COLOR_LIGHTCYAN     ((uint32_t) 0xFF80FFFF)
/* Light Magenta value in ARGB8888 format */
#define LCD_COLOR_LIGHTMAGENTA  ((uint32_t) 0xFFFF80FF)
/* Light Yellow value in ARGB8888 format */
#define LCD_COLOR_LIGHTYELLOW   ((uint32_t) 0xFFFFFF80)
/* Dark Blue value in ARGB8888 format */
#define LCD_COLOR_DARKBLUE      ((uint32_t) 0xFF000080)
/* Light Dark Green value in ARGB8888 format */
#define LCD_COLOR_DARKGREEN     ((uint32_t) 0xFF008000)
/* Light Dark Red value in ARGB8888 format */
#define LCD_COLOR_DARKRED       ((uint32_t) 0xFF800000)
/* Dark Cyan value in ARGB8888 format */
#define LCD_COLOR_DARKCYAN      ((uint32_t) 0xFF008080)
/* Dark Magenta value in ARGB8888 format */
#define LCD_COLOR_DARKMAGENTA   ((uint32_t) 0xFF800080)
/* Dark Yellow value in ARGB8888 format */
#define LCD_COLOR_DARKYELLOW    ((uint32_t) 0xFF808000)
/* White value in ARGB8888 format */
#define LCD_COLOR_WHITE         ((uint32_t) 0xFFFFFFFF)
/* Light Gray value in ARGB8888 format */
#define LCD_COLOR_LIGHTGRAY     ((uint32_t) 0xFFD3D3D3)
/* Gray value in ARGB8888 format */
#define LCD_COLOR_GRAY          ((uint32_t) 0xFF808080)
/* Dark Gray value in ARGB8888 format */
#define LCD_COLOR_DARKGRAY      ((uint32_t) 0xFF404040)
/* Black value in ARGB8888 format */
#define LCD_COLOR_BLACK         ((uint32_t) 0xFF000000)
/* Brown value in ARGB8888 format */
#define LCD_COLOR_BROWN         ((uint32_t) 0xFFA52A2A)
/* Orange value in ARGB8888 format */
#define LCD_COLOR_ORANGE        ((uint32_t) 0xFFFFA500)
/* Transparent value in ARGB8888 format */
#define LCD_COLOR_TRANSPARENT   ((uint32_t) 0xFF000000)

/* LCD default font */
#define LCD_DEFAULT_FONT        Font24

/**
  * @}
  */

/** @defgroup STM32L4R9I_DISCOVERY_LCD_Exported_Types LCD Exported Types
  * @{
  */

/* LCD Drawing main properties */
typedef struct
{
  uint32_t TextColor; /*!< Specifies the color of text */
  uint32_t BackColor; /*!< Specifies the background color below the text */
  sFONT    *pFont;    /*!< Specifies the font used for the text */
} LCD_DrawPropTypeDef;

/* LCD Drawing point (pixel) geometric definition */
typedef struct
{
  int16_t X; /*!< geometric X position of drawing */
  int16_t Y; /*!< geometric Y position of drawing */
} Point;

/* Pointer on LCD Drawing point (pixel) geometric definition */
typedef Point * pPoint;

/* LCD drawing Line alignment mode definitions */
typedef enum
{
  CENTER_MODE             = 0x01,    /*!< Center mode */
  RIGHT_MODE              = 0x02,    /*!< Right mode  */
  LEFT_MODE               = 0x03     /*!< Left mode   */
} Text_AlignModeTypdef;

/* LCD_OrientationTypeDef : Possible values of Display Orientation */
typedef enum
{
  LCD_ORIENTATION_PORTRAIT  = 0x00, /*!< Portrait orientation choice of LCD screen  */
  LCD_ORIENTATION_LANDSCAPE = 0x01, /*!< Landscape orientation choice of LCD screen */
  LCD_ORIENTATION_INVALID   = 0x02  /*!< Invalid orientation choice of LCD screen   */
} LCD_OrientationTypeDef;

/**
  * @}
  */

/** @addtogroup STM32L4R9I_DISCOVERY_LCD_Exported_Functions
  * @{
  */

uint8_t  BSP_LCD_Init(void);
uint8_t  BSP_LCD_DeInit(void);

uint32_t BSP_LCD_GetXSize(void);
uint32_t BSP_LCD_GetYSize(void);

uint8_t  BSP_LCD_SetTransparency(uint32_t LayerIndex, uint8_t Transparency);
uint8_t  BSP_LCD_SetColorKeying(uint32_t LayerIndex, uint32_t RGBValue);
uint8_t  BSP_LCD_ResetColorKeying(uint32_t LayerIndex);

uint8_t  BSP_LCD_SelectLayer(uint32_t LayerIndex);
uint8_t  BSP_LCD_SetLayerVisible(uint32_t LayerIndex, FunctionalState State);

void     BSP_LCD_SetTextColor(uint32_t Color);
uint32_t BSP_LCD_GetTextColor(void);
void     BSP_LCD_SetBackColor(uint32_t Color);
uint32_t BSP_LCD_GetBackColor(void);
void     BSP_LCD_SetFont(sFONT *fonts);
sFONT    *BSP_LCD_GetFont(void);

uint32_t BSP_LCD_ReadPixel(uint16_t Xpos, uint16_t Ypos);
void     BSP_LCD_DrawPixel(uint16_t Xpos, uint16_t Ypos, uint32_t pixel);
void     BSP_LCD_Clear(uint32_t Color);
void     BSP_LCD_ClearStringLine(uint32_t Line);
void     BSP_LCD_DisplayStringAtLine(uint16_t Line, uint8_t *ptr);
void     BSP_LCD_DisplayStringAt(uint16_t Xpos, uint16_t Ypos, uint8_t *Text, Text_AlignModeTypdef Mode);
void     BSP_LCD_DisplayChar(uint16_t Xpos, uint16_t Ypos, uint8_t Ascii);

void     BSP_LCD_DrawHLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length);
void     BSP_LCD_DrawVLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length);
void     BSP_LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void     BSP_LCD_DrawRect(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height);
void     BSP_LCD_DrawCircle(uint16_t Xpos, uint16_t Ypos, uint16_t Radius);
void     BSP_LCD_DrawPolygon(pPoint Points, uint16_t PointCount);
void     BSP_LCD_DrawEllipse(int Xpos, int Ypos, int XRadius, int YRadius);
void     BSP_LCD_DrawBitmap(uint32_t Xpos, uint32_t Ypos, uint8_t *pbmp);

void     BSP_LCD_FillRect(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height);
void     BSP_LCD_FillCircle(uint16_t Xpos, uint16_t Ypos, uint16_t Radius);
void     BSP_LCD_FillPolygon(pPoint Points, uint16_t PointCount);
void     BSP_LCD_FillEllipse(int Xpos, int Ypos, int XRadius, int YRadius);

void     BSP_LCD_DisplayOff(void);
void     BSP_LCD_DisplayOn(void);

void     BSP_LCD_Refresh(void);
uint8_t  BSP_LCD_IsFrameBufferAvailable(void);

void     BSP_LCD_SetBrightness(uint8_t BrightnessValue);

/* These __weak functions can be surcharged by application code for specific application needs */
void     BSP_LCD_MspInit(void);
void     BSP_LCD_MspDeInit(void);

void     BSP_LCD_DMA2D_IRQHandler(void);
void     BSP_LCD_DSI_IRQHandler(void);
void     BSP_LCD_LTDC_IRQHandler(void);
void     BSP_LCD_LTDC_ER_IRQHandler(void);

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

#endif /* __STM32L4R9I_DISCOVERY_LCD_H */


