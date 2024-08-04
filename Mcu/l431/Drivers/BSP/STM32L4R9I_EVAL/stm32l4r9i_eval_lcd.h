/**
  ******************************************************************************
  * @file    stm32l4r9i_eval_lcd.h
  * @author  MCD Application Team
  * @brief   This file contains alias for compatibility with DSI and TFT LCD.
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
#ifndef __STM32L4R9I_EVAL_LCD_H
#define __STM32L4R9I_EVAL_LCD_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "../../../Utilities/Fonts/fonts.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L4R9I_EVAL
  * @{
  */

/** @addtogroup STM32L4R9I_EVAL_LCD STM32L4R9I_EVAL LCD
  * @{
  */

/** @defgroup STM32L4R9I_EVAL_LCD_Exported_Constants Exported Constants
  * @{
  */

/* LTDC layer configuration structure */
#define LCD_LayerCfgTypeDef               LTDC_LayerCfgTypeDef

/* Maximum number of LTDC layers */
#define LTDC_MAX_LAYER_NUMBER             ((uint32_t) 2)

/* LTDC Background layer index */
#define LTDC_ACTIVE_LAYER_BACKGROUND      LTDC_LAYER_1

/* LTDC Foreground layer index : Not used on STM32L4R9I_EVAL, only one layer used */
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

/** @defgroup STM32L4R9I_EVAL_LCD_Exported_Types Exported Types
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

/** @addtogroup STM32L4R9I_EVAL_DSI_LCD_Exported_Functions
  * @{
  */

#if defined(USE_GVO_390x390)

#include "stm32l4r9i_eval_dsi_lcd.h"

#define  BSP_LCD_DMA2D_IRQHandler     BSP_DSI_LCD_DMA2D_IRQHandler
#define  BSP_LCD_DSI_IRQHandler       BSP_DSI_LCD_DSI_IRQHandler
#define  BSP_LCD_LTDC_IRQHandler      BSP_DSI_LCD_LTDC_IRQHandler
#define  BSP_LCD_LTDC_ER_IRQHandler   BSP_DSI_LCD_LTDC_ER_IRQHandler

#define  BSP_LCD_Init                 BSP_DSI_LCD_Init
#define  BSP_LCD_DeInit               BSP_DSI_LCD_DeInit

#define  BSP_LCD_MspDeInit            BSP_DSI_LCD_MspDeInit
#define  BSP_LCD_MspInit              BSP_DSI_LCD_MspInit

#define  BSP_LCD_GetXSize             BSP_DSI_LCD_GetXSize
#define  BSP_LCD_GetYSize             BSP_DSI_LCD_GetYSize

#define  BSP_LCD_SetTransparency      BSP_DSI_LCD_SetTransparency
#define  BSP_LCD_SetColorKeying       BSP_DSI_LCD_SetColorKeying
#define  BSP_LCD_ResetColorKeying     BSP_DSI_LCD_ResetColorKeying

#define  BSP_LCD_SelectLayer          BSP_DSI_LCD_SelectLayer
#define  BSP_LCD_SetLayerVisible      BSP_DSI_LCD_SetLayerVisible

#define  BSP_LCD_SetTextColor         BSP_DSI_LCD_SetTextColor
#define  BSP_LCD_GetTextColor         BSP_DSI_LCD_GetTextColor
#define  BSP_LCD_SetBackColor         BSP_DSI_LCD_SetBackColor
#define  BSP_LCD_GetBackColor         BSP_DSI_LCD_GetBackColor
#define  BSP_LCD_SetFont              BSP_DSI_LCD_SetFont
#define  BSP_LCD_GetFont              BSP_DSI_LCD_GetFont

#define  BSP_LCD_ReadPixel            BSP_DSI_LCD_ReadPixel
#define  BSP_LCD_DrawPixel            BSP_DSI_LCD_DrawPixel
#define  BSP_LCD_Clear                BSP_DSI_LCD_Clear
#define  BSP_LCD_ClearStringLine      BSP_DSI_LCD_ClearStringLine
#define  BSP_LCD_DisplayStringAtLine  BSP_DSI_LCD_DisplayStringAtLine
#define  BSP_LCD_DisplayStringAt      BSP_DSI_LCD_DisplayStringAt
#define  BSP_LCD_DisplayChar          BSP_DSI_LCD_DisplayChar

#define  BSP_LCD_DrawHLine            BSP_DSI_LCD_DrawHLine
#define  BSP_LCD_DrawVLine            BSP_DSI_LCD_DrawVLine
#define  BSP_LCD_DrawLine             BSP_DSI_LCD_DrawLine
#define  BSP_LCD_DrawRect             BSP_DSI_LCD_DrawRect
#define  BSP_LCD_DrawCircle           BSP_DSI_LCD_DrawCircle
#define  BSP_LCD_DrawPolygon          BSP_DSI_LCD_DrawPolygon
#define  BSP_LCD_DrawEllipse          BSP_DSI_LCD_DrawEllipse
#define  BSP_LCD_DrawBitmap           BSP_DSI_LCD_DrawBitmap

#define  BSP_LCD_FillRect             BSP_DSI_LCD_FillRect
#define  BSP_LCD_FillCircle           BSP_DSI_LCD_FillCircle
#define  BSP_LCD_FillPolygon          BSP_DSI_LCD_FillPolygon
#define  BSP_LCD_FillEllipse          BSP_DSI_LCD_FillEllipse

#define  BSP_LCD_DisplayOff           BSP_DSI_LCD_DisplayOff
#define  BSP_LCD_DisplayOn            BSP_DSI_LCD_DisplayOn

#define  BSP_LCD_Refresh                 BSP_DSI_LCD_Refresh
#define  BSP_LCD_IsFrameBufferAvailable  BSP_DSI_LCD_IsFrameBufferAvailable

#define  BSP_LCD_SetBrightness        BSP_DSI_LCD_SetBrightness

#endif /* USE_GVO_390x390 */

#if defined(USE_ROCKTECH_480x272)

#include "stm32l4r9i_eval_rgb_lcd.h"

#define  BSP_LCD_DMA2D_IRQHandler     BSP_RGB_LCD_DMA2D_IRQHandler
#define  BSP_LCD_LTDC_IRQHandler      BSP_RGB_LCD_LTDC_IRQHandler
#define  BSP_LCD_LTDC_ER_IRQHandler   BSP_RGB_LCD_LTDC_ER_IRQHandler

#define  BSP_LCD_Init                 BSP_RGB_LCD_Init
#define  BSP_LCD_DeInit               BSP_RGB_LCD_DeInit

#define  BSP_LCD_MspDeInit            BSP_RGB_LCD_MspDeInit
#define  BSP_LCD_MspInit              BSP_RGB_LCD_MspInit

#define  BSP_LCD_GetXSize             BSP_RGB_LCD_GetXSize
#define  BSP_LCD_GetYSize             BSP_RGB_LCD_GetYSize

#define  BSP_LCD_SetTransparency      BSP_RGB_LCD_SetTransparency
#define  BSP_LCD_SetColorKeying       BSP_RGB_LCD_SetColorKeying
#define  BSP_LCD_ResetColorKeying     BSP_RGB_LCD_ResetColorKeying

#define  BSP_LCD_LayerDefaultInit     BSP_RGB_LCD_LayerRgb565Init
#define  BSP_LCD_SelectLayer          BSP_RGB_LCD_SelectLayer
#define  BSP_LCD_SetLayerVisible      BSP_RGB_LCD_SetLayerVisible

#define  BSP_LCD_SetTextColor         BSP_RGB_LCD_SetTextColor
#define  BSP_LCD_GetTextColor         BSP_RGB_LCD_GetTextColor
#define  BSP_LCD_SetBackColor         BSP_RGB_LCD_SetBackColor
#define  BSP_LCD_GetBackColor         BSP_RGB_LCD_GetBackColor
#define  BSP_LCD_SetFont              BSP_RGB_LCD_SetFont
#define  BSP_LCD_GetFont              BSP_RGB_LCD_GetFont

#define  BSP_LCD_ReadPixel            BSP_RGB_LCD_ReadPixel
#define  BSP_LCD_DrawPixel            BSP_RGB_LCD_DrawPixel
#define  BSP_LCD_Clear                BSP_RGB_LCD_Clear
#define  BSP_LCD_ClearStringLine      BSP_RGB_LCD_ClearStringLine
#define  BSP_LCD_DisplayStringAtLine  BSP_RGB_LCD_DisplayStringAtLine
#define  BSP_LCD_DisplayStringAt      BSP_RGB_LCD_DisplayStringAt
#define  BSP_LCD_DisplayChar          BSP_RGB_LCD_DisplayChar

#define  BSP_LCD_DrawHLine            BSP_RGB_LCD_DrawHLine
#define  BSP_LCD_DrawVLine            BSP_RGB_LCD_DrawVLine
#define  BSP_LCD_DrawLine             BSP_RGB_LCD_DrawLine
#define  BSP_LCD_DrawRect             BSP_RGB_LCD_DrawRect
#define  BSP_LCD_DrawCircle           BSP_RGB_LCD_DrawCircle
#define  BSP_LCD_DrawPolygon          BSP_RGB_LCD_DrawPolygon
#define  BSP_LCD_DrawEllipse          BSP_RGB_LCD_DrawEllipse
#define  BSP_LCD_DrawBitmap           BSP_RGB_LCD_DrawBitmap

#define  BSP_LCD_FillRect             BSP_RGB_LCD_FillRect
#define  BSP_LCD_FillCircle           BSP_RGB_LCD_FillCircle
#define  BSP_LCD_FillPolygon          BSP_RGB_LCD_FillPolygon
#define  BSP_LCD_FillEllipse          BSP_RGB_LCD_FillEllipse

#define  BSP_LCD_DisplayOff           BSP_RGB_LCD_DisplayOff
#define  BSP_LCD_DisplayOn            BSP_RGB_LCD_DisplayOn

#endif /* USE_ROCKTECH_480x272 */

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

#endif /* __STM32L4R9I_EVAL_LCD_H */


