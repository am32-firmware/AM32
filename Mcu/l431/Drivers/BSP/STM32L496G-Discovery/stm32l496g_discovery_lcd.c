/**
  ******************************************************************************
  * @file    stm32l496g_discovery_lcd.c
  * @author  MCD Application Team
  * @brief   This file includes the driver for Liquid Crystal Display (LCD) module
  *          mounted on STM32L496G-DISCOVERY board.
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
   - This driver is used to drive indirectly an LCD TFT.
   - This driver supports the LS016B8UY LCD.
   - The LS016B8UY and ST7789H2 components driver MUST be included with this driver.

2. Driver description:
---------------------
  + Initialization steps:
     o Initialize the LCD using the BSP_LCD_Init() function.

  + Display on LCD
     o Clear the hole LCD using BSP_LCD_Clear() function or only one specified string
       line using the BSP_LCD_ClearStringLine() function.
     o Display a character on the specified line and column using the BSP_LCD_DisplayChar()
       function or a complete string line using the BSP_LCD_DisplayStringAtLine() function.
     o Display a string line on the specified position (x,y in pixel) and align mode
       using the BSP_LCD_DisplayStringAtLine() function.
     o Draw and fill a basic shapes (dot, line, rectangle, circle, ellipse, .. bitmap)
       on LCD using the available set of functions.

------------------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "stm32l496g_discovery_lcd.h"
#include "../../../Utilities/Fonts/fonts.h"
#include "../../../Utilities/Fonts/font24.c"
#include "../../../Utilities/Fonts/font20.c"
#include "../../../Utilities/Fonts/font16.c"
#include "../../../Utilities/Fonts/font12.c"
#include "../../../Utilities/Fonts/font8.c"


static uint32_t bsp_lcd_initialized = 0;
static uint8_t LCD_orientation = LCD_ORIENTATION_UNDEFINED;

uint32_t dimming_on = 0;

typedef struct dimming_config_s
{
  uint8_t   ongoing;
  uint8_t   start;
  uint8_t   stop;
  uint8_t   step;
  uint8_t   delay;
} dimming_config_t;

#define __DIMMING_CYCLE_VALUE(value)  (uint32_t)((PERIOD_VALUE * value)/100)
static dimming_config_t dimming_config = { 0, 25, 5, 1, PULSE_DECREASE_DELAY };

#if defined(LPTIMER_DIMMING)
LPTIM_HandleTypeDef         LCD_LpTimHandle;
#endif
/* Timer handler declaration */
TIM_HandleTypeDef    LCD_TimHandle;
/* Timer Output Compare Configuration Structure declaration */
TIM_OC_InitTypeDef LCD_sConfig;

#if defined(LPTIMER_DIMMING)
extern uint32_t dimming_enable;
#endif


extern uint32_t ts_io_init;

/* Use by application to not use backlight */
FlagStatus WakeUpFromStandby;
FlagStatus WakeUpFromShutdown;


/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L496G_DISCOVERY
  * @{
  */

/** @defgroup STM32L496G_DISCOVERY_LCD STM32L496G-DISCOVERY LCD
  * @{
  */

/** @defgroup STM32L496G_DISCOVERY_LCD_Private_TypesDefinitions STM32L496G Discovery Lcd Private TypesDef
  * @{
  */
/**
  * @}
  */

/** @defgroup STM32L496G_DISCOVERY_LCD_Private_Defines STM32L496G Discovery Lcd Private Defines
  * @{
  */
#define POLY_X(Z)              ((int32_t)((Points + Z)->X))
#define POLY_Y(Z)              ((int32_t)((Points + Z)->Y))
/**
  * @}
  */

/** @defgroup STM32L496G_DISCOVERY_LCD_Private_Macros STM32L496G Discovery Lcd Private Macros
  * @{
  */
#define ABS(X)  ((X) > 0 ? (X) : -(X))
/**
  * @}
  */

/** @defgroup STM32L496G_DISCOVERY_LCD_Private_Variables STM32L496G Discovery Lcd Private Variables
  * @{
  */
LCD_DrawPropTypeDef DrawProp;
static LCD_DrvTypeDef  *LcdDrv;


/**
  * @}
  */

/** @defgroup STM32L496G_DISCOVERY_LCD_Private_FunctionPrototypes STM32L496G Discovery Lcd Private Prototypes
  * @{
  */
static void DrawChar(uint16_t Xpos, uint16_t Ypos, const uint8_t *c);
static void SetDisplayWindow(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height);
static void FillTriangle(uint16_t x1, uint16_t x2, uint16_t x3, uint16_t y1, uint16_t y2, uint16_t y3);
/**
  * @}
  */

/** @defgroup STM32L496G_DISCOVERY_LCD_Private_Functions STM32L496G Discovery Lcd Private Functions
  * @{
  */
/**
  * @brief  Initializes the LCD.
  * @param  None
  * @retval LCD state
  */
uint8_t BSP_LCD_Init(void)
{
  return (BSP_LCD_InitEx(LCD_ORIENTATION_LANDSCAPE));
}

/**
  * @brief  Initializes the LCD with a given orientation.
  * @param  orientation: LCD_ORIENTATION_PORTRAIT or LCD_ORIENTATION_LANDSCAPE
  * @retval LCD state
  */
uint8_t BSP_LCD_InitEx(uint32_t orientation)
{
  uint8_t ret = LCD_ERROR;
  uint32_t i = 0;

  if (bsp_lcd_initialized == 1)
  {
    ret = LCD_OK;
  }
  else
  {
    /* Initialize the IO functionalities */
    if (BSP_IO_Init() == IO_ERROR)
    {
      BSP_ErrorHandler();
    }

    /* Initialize LCD special pins GPIOs */
    BSP_LCD_MspInit();

    /* LCD Power On */
    HAL_GPIO_WritePin(LCD_PWR_CTRL_GPIO_PORT, LCD_PWR_CTRL_PIN, GPIO_PIN_RESET);

    /* Default value for draw propriety */
    DrawProp.BackColor = 0xFFFF;
    DrawProp.pFont     = &Font24;
    DrawProp.TextColor = 0x0000;

    if ((WakeUpFromStandby == RESET) && (WakeUpFromShutdown == RESET))
    {
      /* Backlight control signal assertion */
      HAL_GPIO_WritePin(LCD_BL_CTRL_GPIO_PORT, LCD_BL_CTRL_PIN, GPIO_PIN_SET);
    }

    /* Reset the LCD */
    BSP_LCD_Reset();

    if (ST7789H2_drv.ReadID() == ST7789H2_ID)
    {
      LcdDrv = &ST7789H2_drv;

      /* LCD Init */
      LcdDrv->Init();

      /* Fill LCD frame memory with white pixels (or black pixels if INIT_BLACK_LCD is enabled) */
      ST7789H2_WriteReg(ST7789H2_WRITE_RAM, (uint8_t *)NULL, 0);  /* RAM Write Data */
      for (i = 0; i < (ST7789H2_LCD_PIXEL_WIDTH * ST7789H2_LCD_PIXEL_HEIGHT); i++)
      {
#if defined(INIT_BLACK_LCD)
        LCD_IO_WriteData(0x0);
#else
        LCD_IO_WriteData(0xFFFF);
#endif
      }

      if (orientation == LCD_ORIENTATION_PORTRAIT)
      {
        ST7789H2_SetOrientation(ST7789H2_ORIENTATION_PORTRAIT);
        LCD_orientation = LCD_ORIENTATION_PORTRAIT;
      }
      else
      {
        LCD_orientation = LCD_ORIENTATION_LANDSCAPE;
      }
      /* Initialize the font */
      BSP_LCD_SetFont(&LCD_DEFAULT_FONT);

      bsp_lcd_initialized = 1;
      ret = LCD_OK;
    }
  }

  return ret;
}

/**
  * @brief  Reset the LCD.
  * @param  None
  * @retval LCD state
  */
void BSP_LCD_Reset(void)
{
  /* Apply hardware reset according to procedure indicated in FRD154BP2901 documentation */
  BSP_IO_WritePin(LCD_RST_PIN, GPIO_PIN_RESET);
  HAL_Delay(5);   /* Reset signal asserted during 5ms  */
  BSP_IO_WritePin(LCD_RST_PIN, GPIO_PIN_SET);
  HAL_Delay(10);  /* Reset signal released during 10ms */
  BSP_IO_WritePin(LCD_RST_PIN, GPIO_PIN_RESET);
  HAL_Delay(20);  /* Reset signal asserted during 20ms */
  BSP_IO_WritePin(LCD_RST_PIN, GPIO_PIN_SET);
  HAL_Delay(10);  /* Reset signal released during 10ms */
}

/**
  * @brief  DeInitializes the LCD.
  * @param  None
  * @retval LCD state
  */
uint8_t BSP_LCD_DeInit(void)
{
  BSP_LCD_MspDeInit();

  bsp_lcd_initialized = 0;
  ts_io_init = 0;

  return LCD_OK;
}

/**
  * @brief  Gets the LCD X size.
  * @param  None
  * @retval Used LCD X size
  */
uint32_t BSP_LCD_GetXSize(void)
{
  return (LcdDrv->GetLcdPixelWidth());
}

/**
  * @brief  Gets the LCD Y size.
  * @param  None
  * @retval Used LCD Y size
  */
uint32_t BSP_LCD_GetYSize(void)
{
  return (LcdDrv->GetLcdPixelHeight());
}

/**
  * @brief  Gets the LCD text color.
  * @param  None
  * @retval Used text color.
  */
uint16_t BSP_LCD_GetTextColor(void)
{
  return DrawProp.TextColor;
}

/**
  * @brief  Gets the LCD background color.
  * @param  None
  * @retval Used background color
  */
uint16_t BSP_LCD_GetBackColor(void)
{
  return DrawProp.BackColor;
}

/**
  * @brief  Sets the LCD text color.
  * @param  Color: Text color code RGB(5-6-5)
  * @retval None
  */
void BSP_LCD_SetTextColor(uint16_t Color)
{
  DrawProp.TextColor = Color;
}

/**
  * @brief  Sets the LCD background color.
  * @param  Color: Background color code RGB(5-6-5)
  * @retval None
  */
void BSP_LCD_SetBackColor(uint16_t Color)
{
  DrawProp.BackColor = Color;
}

/**
  * @brief  Sets the LCD text font.
  * @param  fonts: Font to be used
  * @retval None
  */
void BSP_LCD_SetFont(sFONT *fonts)
{
  DrawProp.pFont = fonts;
}

/**
  * @brief  Gets the LCD text font.
  * @param  None
  * @retval Used font
  */
sFONT *BSP_LCD_GetFont(void)
{
  return DrawProp.pFont;
}

/**
  * @brief  Clears the hole LCD.
  * @param  Color: Color of the background
  * @retval None
  */
void BSP_LCD_Clear(uint16_t Color)
{
  uint32_t counter = 0;
  uint32_t y_size = 0;
  uint32_t color_backup = DrawProp.TextColor;

  DrawProp.TextColor = Color;
  y_size =  BSP_LCD_GetYSize();

  for (counter = 0; counter < y_size; counter++)
  {
    BSP_LCD_DrawHLine(0, counter, BSP_LCD_GetXSize());
  }
  DrawProp.TextColor = color_backup;
  BSP_LCD_SetTextColor(DrawProp.TextColor);
}

/**
  * @brief  Clears the selected line.
  * @param  Line: Line to be cleared
  *          This parameter can be one of the following values:
  *            @arg  0..9: if the Current fonts is Font16x24
  *            @arg  0..19: if the Current fonts is Font12x12 or Font8x12
  *            @arg  0..29: if the Current fonts is Font8x8
  * @retval None
  */
void BSP_LCD_ClearStringLine(uint16_t Line)
{
  uint32_t color_backup = DrawProp.TextColor;

  DrawProp.TextColor = DrawProp.BackColor;;

  /* Draw a rectangle with background color */
  BSP_LCD_FillRect(0, (Line * DrawProp.pFont->Height), BSP_LCD_GetXSize(), DrawProp.pFont->Height);

  DrawProp.TextColor = color_backup;
  BSP_LCD_SetTextColor(DrawProp.TextColor);
}

/**
  * @brief  Displays one character.
  * @param  Xpos: Start column address
  * @param  Ypos: Line where to display the character shape.
  * @param  Ascii: Character ascii code
  *           This parameter must be a number between Min_Data = 0x20 and Max_Data = 0x7E
  * @retval None
  */
void BSP_LCD_DisplayChar(uint16_t Xpos, uint16_t Ypos, uint8_t Ascii)
{
  DrawChar(Xpos, Ypos, &DrawProp.pFont->table[(Ascii - ' ') *\
                                              DrawProp.pFont->Height * ((DrawProp.pFont->Width + 7) / 8)]);
}

/**
  * @brief  Displays characters on the LCD.
  * @param  Xpos: X position (in pixel)
  * @param  Ypos: Y position (in pixel)
  * @param  Text: Pointer to string to display on LCD
  * @param  Mode: Display mode
  *          This parameter can be one of the following values:
  *            @arg  CENTER_MODE
  *            @arg  RIGHT_MODE
  *            @arg  LEFT_MODE
  * @retval None
  */
void BSP_LCD_DisplayStringAt(uint16_t Xpos, uint16_t Ypos, uint8_t *Text, Line_ModeTypdef Mode)
{
  uint16_t refcolumn = 1, i = 0;
  uint32_t size = 0, xsize = 0;
  uint8_t  *ptr = Text;

  /* Get the text size */
  while (*ptr++)
  {
    size ++ ;
  }

  /* Characters number per line */
  xsize = (BSP_LCD_GetXSize() / DrawProp.pFont->Width);

  switch (Mode)
  {
    case CENTER_MODE:
    {
      refcolumn = Xpos + ((xsize - size) * DrawProp.pFont->Width) / 2;
      break;
    }
    case LEFT_MODE:
    {
      refcolumn = Xpos;
      break;
    }
    case RIGHT_MODE:
    {
      refcolumn =  - Xpos + ((xsize - size) * DrawProp.pFont->Width);
      break;
    }
    default:
    {
      refcolumn = Xpos;
      break;
    }
  }

  /* Check that the Start column is located in the screen */
  if ((refcolumn < 1) || (refcolumn >= 0x8000))
  {
    refcolumn = 1;
  }

  /* Send the string character by character on lCD */
  while ((*Text != 0) && (((BSP_LCD_GetXSize() - (i * DrawProp.pFont->Width)) & 0xFFFF) >= DrawProp.pFont->Width))
  {
    /* Display one character on LCD */
    BSP_LCD_DisplayChar(refcolumn, Ypos, *Text);
    /* Decrement the column position by 16 */
    refcolumn += DrawProp.pFont->Width;
    /* Point on the next character */
    Text++;
    i++;
  }
}

/**
  * @brief  Displays a character on the LCD.
  * @param  Line: Line where to display the character shape
  *          This parameter can be one of the following values:
  *            @arg  0..9: if the Current fonts is Font16x24
  *            @arg  0..19: if the Current fonts is Font12x12 or Font8x12
  *            @arg  0..29: if the Current fonts is Font8x8
  * @param  ptr: Pointer to string to display on LCD
  * @retval None
  */
void BSP_LCD_DisplayStringAtLine(uint16_t Line, uint8_t *ptr)
{
  BSP_LCD_DisplayStringAt(0, LINE(Line), ptr, LEFT_MODE);
}

/**
  * @brief  Reads an LCD pixel.
  * @param  Xpos: X position
  * @param  Ypos: Y position
  * @retval RGB pixel color
  */
uint16_t BSP_LCD_ReadPixel(uint16_t Xpos, uint16_t Ypos)
{
  uint16_t ret = 0;

  if (LcdDrv->ReadPixel != NULL)
  {
    ret = LcdDrv->ReadPixel(Xpos, Ypos);
  }

  return ret;
}

/**
  * @brief  Draws a pixel on LCD.
  * @param  Xpos: X position
  * @param  Ypos: Y position
  * @param  RGB_Code: Pixel color in RGB mode (5-6-5)
  * @retval None
  */
void BSP_LCD_DrawPixel(uint16_t Xpos, uint16_t Ypos, uint16_t RGB_Code)
{
  if (LcdDrv->WritePixel != NULL)
  {
    LcdDrv->WritePixel(Xpos, Ypos, RGB_Code);
  }
}

/**
  * @brief  Draws an horizontal line.
  * @param  Xpos: X position
  * @param  Ypos: Y position
  * @param  Length: Line length
  * @retval None
  */
void BSP_LCD_DrawHLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length)
{
  uint32_t index = 0;

  if (LcdDrv->DrawHLine != NULL)
  {
    LcdDrv->DrawHLine(DrawProp.TextColor, Xpos, Ypos, Length);
  }
  else
  {
    for (index = 0; index < Length; index++)
    {
      BSP_LCD_DrawPixel((Xpos + index), Ypos, DrawProp.TextColor);
    }
  }
}

/**
  * @brief  Draws a vertical line.
  * @param  Xpos: X position
  * @param  Ypos: Y position
  * @param  Length: Line length
  * @retval None
  */
void BSP_LCD_DrawVLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length)
{
  uint32_t index = 0;

  if (LcdDrv->DrawVLine != NULL)
  {
    LcdDrv->DrawVLine(DrawProp.TextColor, Xpos, Ypos, Length);
  }
  else
  {
    for (index = 0; index < Length; index++)
    {
      BSP_LCD_DrawPixel(Xpos, Ypos + index, DrawProp.TextColor);
    }
  }
}

/**
  * @brief  Draws an uni-line (between two points).
  * @param  x1: Point 1 X position
  * @param  y1: Point 1 Y position
  * @param  x2: Point 2 X position
  * @param  y2: Point 2 Y position
  * @retval None
  */
void BSP_LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
  int16_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0,
          yinc1 = 0, yinc2 = 0, den = 0, num = 0, numadd = 0, numpixels = 0,
          curpixel = 0;

  deltax = ABS(x2 - x1);        /* The difference between the x's */
  deltay = ABS(y2 - y1);        /* The difference between the y's */
  x = x1;                       /* Start x off at the first pixel */
  y = y1;                       /* Start y off at the first pixel */

  if (x2 >= x1)                 /* The x-values are increasing */
  {
    xinc1 = 1;
    xinc2 = 1;
  }
  else                          /* The x-values are decreasing */
  {
    xinc1 = -1;
    xinc2 = -1;
  }

  if (y2 >= y1)                 /* The y-values are increasing */
  {
    yinc1 = 1;
    yinc2 = 1;
  }
  else                          /* The y-values are decreasing */
  {
    yinc1 = -1;
    yinc2 = -1;
  }

  if (deltax >= deltay)         /* There is at least one x-value for every y-value */
  {
    xinc1 = 0;                  /* Don't change the x when numerator >= denominator */
    yinc2 = 0;                  /* Don't change the y for every iteration */
    den = deltax;
    num = deltax / 2;
    numadd = deltay;
    numpixels = deltax;         /* There are more x-values than y-values */
  }
  else                          /* There is at least one y-value for every x-value */
  {
    xinc2 = 0;                  /* Don't change the x for every iteration */
    yinc1 = 0;                  /* Don't change the y when numerator >= denominator */
    den = deltay;
    num = deltay / 2;
    numadd = deltax;
    numpixels = deltay;         /* There are more y-values than x-values */
  }

  for (curpixel = 0; curpixel <= numpixels; curpixel++)
  {
    BSP_LCD_DrawPixel(x, y, DrawProp.TextColor);  /* Draw the current pixel */
    num += numadd;                            /* Increase the numerator by the top of the fraction */
    if (num >= den)                           /* Check if numerator >= denominator */
    {
      num -= den;                             /* Calculate the new numerator value */
      x += xinc1;                             /* Change the x as appropriate */
      y += yinc1;                             /* Change the y as appropriate */
    }
    x += xinc2;                               /* Change the x as appropriate */
    y += yinc2;                               /* Change the y as appropriate */
  }
}

/**
  * @brief  Draws a rectangle.
  * @param  Xpos: X position
  * @param  Ypos: Y position
  * @param  Width: Rectangle width
  * @param  Height: Rectangle height
  * @retval None
  */
void BSP_LCD_DrawRect(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height)
{
  /* Draw horizontal lines */
  BSP_LCD_DrawHLine(Xpos, Ypos, Width);
  BSP_LCD_DrawHLine(Xpos, (Ypos + Height), Width);

  /* Draw vertical lines */
  BSP_LCD_DrawVLine(Xpos, Ypos, Height);
  BSP_LCD_DrawVLine((Xpos + Width), Ypos, Height);
}

/**
  * @brief  Draws a circle.
  * @param  Xpos: X position
  * @param  Ypos: Y position
  * @param  Radius: Circle radius
  * @retval None
  */
void BSP_LCD_DrawCircle(uint16_t Xpos, uint16_t Ypos, uint16_t Radius)
{
  int32_t  decision;       /* Decision Variable */
  uint32_t  current_x;   /* Current X Value */
  uint32_t  current_y;   /* Current Y Value */

  decision = 3 - (Radius << 1);
  current_x = 0;
  current_y = Radius;

  while (current_x <= current_y)
  {
    BSP_LCD_DrawPixel((Xpos + current_x), (Ypos - current_y), DrawProp.TextColor);

    BSP_LCD_DrawPixel((Xpos - current_x), (Ypos - current_y), DrawProp.TextColor);

    BSP_LCD_DrawPixel((Xpos + current_y), (Ypos - current_x), DrawProp.TextColor);

    BSP_LCD_DrawPixel((Xpos - current_y), (Ypos - current_x), DrawProp.TextColor);

    BSP_LCD_DrawPixel((Xpos + current_x), (Ypos + current_y), DrawProp.TextColor);

    BSP_LCD_DrawPixel((Xpos - current_x), (Ypos + current_y), DrawProp.TextColor);

    BSP_LCD_DrawPixel((Xpos + current_y), (Ypos + current_x), DrawProp.TextColor);

    BSP_LCD_DrawPixel((Xpos - current_y), (Ypos + current_x), DrawProp.TextColor);

    /* Initialize the font */
    BSP_LCD_SetFont(&LCD_DEFAULT_FONT);

    if (decision < 0)
    {
      decision += (current_x << 2) + 6;
    }
    else
    {
      decision += ((current_x - current_y) << 2) + 10;
      current_y--;
    }
    current_x++;
  }
}

/**
  * @brief  Draws an poly-line (between many points).
  * @param  Points: Pointer to the points array
  * @param  PointCount: Number of points
  * @retval None
  */
void BSP_LCD_DrawPolygon(pPoint Points, uint16_t PointCount)
{
  int16_t x = 0, y = 0;

  if (PointCount < 2)
  {
    return;
  }

  BSP_LCD_DrawLine(Points->X, Points->Y, (Points + PointCount - 1)->X, (Points + PointCount - 1)->Y);

  while (--PointCount)
  {
    x = Points->X;
    y = Points->Y;
    Points++;
    BSP_LCD_DrawLine(x, y, Points->X, Points->Y);
  }
}

/**
  * @brief  Draws an ellipse on LCD.
  * @param  Xpos: X position
  * @param  Ypos: Y position
  * @param  XRadius: Ellipse X radius
  * @param  YRadius: Ellipse Y radius
  * @retval None
  */
void BSP_LCD_DrawEllipse(int Xpos, int Ypos, int XRadius, int YRadius)
{
  int x = 0, y = -YRadius, err = 2 - 2 * XRadius, e2;
  float k = 0, rad1 = 0, rad2 = 0;

  rad1 = XRadius;
  rad2 = YRadius;

  k = (float)(rad2 / rad1);

  do
  {
    BSP_LCD_DrawPixel((Xpos - (uint16_t)(x / k)), (Ypos + y), DrawProp.TextColor);
    BSP_LCD_DrawPixel((Xpos + (uint16_t)(x / k)), (Ypos + y), DrawProp.TextColor);
    BSP_LCD_DrawPixel((Xpos + (uint16_t)(x / k)), (Ypos - y), DrawProp.TextColor);
    BSP_LCD_DrawPixel((Xpos - (uint16_t)(x / k)), (Ypos - y), DrawProp.TextColor);

    e2 = err;
    if (e2 <= x)
    {
      err += ++x * 2 + 1;
      if (-y == x && e2 <= y)
      {
        e2 = 0;
      }
    }
    if (e2 > y)
    {
      err += ++y * 2 + 1;
    }
  }
  while (y <= 0);
}

/**
  * @brief  Draws a bitmap picture (16 bpp).
  * @param  Xpos: Bmp X position in the LCD
  * @param  Ypos: Bmp Y position in the LCD
  * @param  pbmp: Pointer to Bmp picture address.
  * @retval None
  */
void BSP_LCD_DrawBitmap(uint16_t Xpos, uint16_t Ypos, uint8_t *pbmp)
{
  uint32_t height = 0;
  uint32_t width  = 0;


  /* Read bitmap width */
  width = *(uint16_t *)(pbmp + 18);
  width |= (*(uint16_t *)(pbmp + 20)) << 16;

  /* Read bitmap height */
  height = *(uint16_t *)(pbmp + 22);
  height |= (*(uint16_t *)(pbmp + 24)) << 16;

  SetDisplayWindow(Xpos, Ypos, width, height);

  if (LcdDrv->DrawBitmap != NULL)
  {
    LcdDrv->DrawBitmap(Xpos, Ypos, pbmp);
  }
  SetDisplayWindow(0, 0, BSP_LCD_GetXSize(), BSP_LCD_GetYSize());
}

/**
  * @brief  Draws RGB Image (16 bpp).
  * @param  Xpos:  X position in the LCD
  * @param  Ypos:  Y position in the LCD
  * @param  Xsize: X size in the LCD
  * @param  Ysize: Y size in the LCD
  * @param  pdata: Pointer to the RGB Image address.
  * @retval None
  */
void BSP_LCD_DrawRGBImage(uint16_t Xpos, uint16_t Ypos, uint16_t Xsize, uint16_t Ysize, uint8_t *pdata)
{

  SetDisplayWindow(Xpos, Ypos, Xsize, Ysize);

  if (LcdDrv->DrawRGBImage != NULL)
  {
    LcdDrv->DrawRGBImage(Xpos, Ypos, Xsize, Ysize, pdata);
  }
  SetDisplayWindow(0, 0, BSP_LCD_GetXSize(), BSP_LCD_GetYSize());
}

/**
  * @brief  Draws a full rectangle.
  * @param  Xpos: X position
  * @param  Ypos: Y position
  * @param  Width: Rectangle width
  * @param  Height: Rectangle height
  * @retval None
  */
void BSP_LCD_FillRect(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height)
{
  BSP_LCD_SetTextColor(DrawProp.TextColor);
  do
  {
    BSP_LCD_DrawHLine(Xpos, Ypos++, Width);
  }
  while (Height--);
}

/**
  * @brief  Draws a full circle.
  * @param  Xpos: X position
  * @param  Ypos: Y position
  * @param  Radius: Circle radius
  * @retval None
  */
void BSP_LCD_FillCircle(uint16_t Xpos, uint16_t Ypos, uint16_t Radius)
{
  int32_t  decision;        /* Decision Variable */
  uint32_t  current_x;    /* Current X Value */
  uint32_t  current_y;    /* Current Y Value */

  decision = 3 - (Radius << 1);

  current_x = 0;
  current_y = Radius;

  BSP_LCD_SetTextColor(DrawProp.TextColor);

  while (current_x <= current_y)
  {
    if (current_y > 0)
    {
      BSP_LCD_DrawHLine(Xpos - current_y, Ypos + current_x, 2 * current_y);
      BSP_LCD_DrawHLine(Xpos - current_y, Ypos - current_x, 2 * current_y);
    }

    if (current_x > 0)
    {
      BSP_LCD_DrawHLine(Xpos - current_x, Ypos - current_y, 2 * current_x);
      BSP_LCD_DrawHLine(Xpos - current_x, Ypos + current_y, 2 * current_x);
    }
    if (decision < 0)
    {
      decision += (current_x << 2) + 6;
    }
    else
    {
      decision += ((current_x - current_y) << 2) + 10;
      current_y--;
    }
    current_x++;
  }

  BSP_LCD_SetTextColor(DrawProp.TextColor);
  BSP_LCD_DrawCircle(Xpos, Ypos, Radius);
}

/**
  * @brief  Draws a full poly-line (between many points).
  * @param  Points: Pointer to the points array
  * @param  PointCount: Number of points
  * @retval None
  */
void BSP_LCD_FillPolygon(pPoint Points, uint16_t PointCount)
{
  int16_t X = 0, Y = 0, X2 = 0, Y2 = 0, X_center = 0, Y_center = 0, X_first = 0, Y_first = 0, pixelX = 0, pixelY = 0, counter = 0;
  uint16_t  IMAGE_LEFT = 0, IMAGE_RIGHT = 0, IMAGE_TOP = 0, IMAGE_BOTTOM = 0;

  IMAGE_LEFT = IMAGE_RIGHT = Points->X;
  IMAGE_TOP = IMAGE_BOTTOM = Points->Y;

  for (counter = 1; counter < PointCount; counter++)
  {
    pixelX = POLY_X(counter);
    if (pixelX < IMAGE_LEFT)
    {
      IMAGE_LEFT = pixelX;
    }
    if (pixelX > IMAGE_RIGHT)
    {
      IMAGE_RIGHT = pixelX;
    }

    pixelY = POLY_Y(counter);
    if (pixelY < IMAGE_TOP)
    {
      IMAGE_TOP = pixelY;
    }
    if (pixelY > IMAGE_BOTTOM)
    {
      IMAGE_BOTTOM = pixelY;
    }
  }

  if (PointCount < 2)
  {
    return;
  }

  X_center = (IMAGE_LEFT + IMAGE_RIGHT) / 2;
  Y_center = (IMAGE_BOTTOM + IMAGE_TOP) / 2;

  X_first = Points->X;
  Y_first = Points->Y;

  while (--PointCount)
  {
    X = Points->X;
    Y = Points->Y;
    Points++;
    X2 = Points->X;
    Y2 = Points->Y;

    FillTriangle(X, X2, X_center, Y, Y2, Y_center);
    FillTriangle(X, X_center, X2, Y, Y_center, Y2);
    FillTriangle(X_center, X2, X, Y_center, Y2, Y);
  }

  FillTriangle(X_first, X2, X_center, Y_first, Y2, Y_center);
  FillTriangle(X_first, X_center, X2, Y_first, Y_center, Y2);
  FillTriangle(X_center, X2, X_first, Y_center, Y2, Y_first);
}

/**
  * @brief  Draws a full ellipse.
  * @param  Xpos: X position
  * @param  Ypos: Y position
  * @param  XRadius: Ellipse X radius
  * @param  YRadius: Ellipse Y radius
  * @retval None
  */
void BSP_LCD_FillEllipse(int Xpos, int Ypos, int XRadius, int YRadius)
{
  int x = 0, y = -YRadius, err = 2 - 2 * XRadius, e2;
  float k = 0, rad1 = 0, rad2 = 0;

  rad1 = XRadius;
  rad2 = YRadius;

  k = (float)(rad2 / rad1);

  do
  {
    BSP_LCD_DrawHLine((Xpos - (uint16_t)(x / k)), (Ypos + y), (2 * (uint16_t)(x / k) + 1));
    BSP_LCD_DrawHLine((Xpos - (uint16_t)(x / k)), (Ypos - y), (2 * (uint16_t)(x / k) + 1));

    e2 = err;
    if (e2 <= x)
    {
      err += ++x * 2 + 1;
      if (-y == x && e2 <= y)
      {
        e2 = 0;
      }
    }
    if (e2 > y)
    {
      err += ++y * 2 + 1;
    }
  }
  while (y <= 0);
}

/**
  * @brief  Enables the display.
  * @param  None
  * @retval None
  */
void BSP_LCD_DisplayOn(void)
{
  LcdDrv->DisplayOn();
}

/**
  * @brief  Disables the display.
  * @param  None
  * @retval None
  */
void BSP_LCD_DisplayOff(void)
{
  LcdDrv->DisplayOff();
}


/**
  * @brief  LCD screen dimming enable
  * @note   Screen brightness is gradually decreased
  * @param  start   : value in percent to start from
  * @param  stop    : value in percent to stop to
  * @param  step    : step value in percent
  * @param  delay   : delay in milliseconds between each step
  * @retval None
  */
void BSP_LCD_ScreenDimmingConfig(const uint8_t start, const uint8_t stop, const uint8_t step, const uint8_t delay)
{
  if ((dimming_config.ongoing == 0)
      && (start <= 100) && (stop <= 100)
      && (step > 0) && (step < 100))
  {
    dimming_config.start = start;
    dimming_config.stop  = stop;
    dimming_config.step  = step;
    dimming_config.delay = delay;
  }
}


/**
  * @brief  LCD screen dimming enable
  * @note   Screen brightness is gradually decreased
  * @param  None
  * @retval None
  */
void BSP_LCD_ScreenDimmingOn(void)
{
  static uint32_t i = 0;

#if defined(LPTIMER_DIMMING)
  /* Set that dim feature is active */
  if (dimming_on == 0)
  {
    dimming_on = 1;
    i = PULSE_DECREASE_START;
    /* Always redo the full initialization as there is no apriori knowledge
      of IO or timer settings at this point (may have been modified by application) */

    __HAL_RCC_LSI_ENABLE();
    /* Select LSI as LPTIM1 clock source */
    __HAL_RCC_LPTIM1_CONFIG(RCC_LPTIM1CLKSOURCE_LSI);


    LCD_LpTimHandle.Instance = LPTIM1;
    __HAL_LPTIM_RESET_HANDLE_STATE(&LCD_LpTimHandle); /* to force MSP call */

    LCD_LpTimHandle.Init.Clock.Source    = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
    LCD_LpTimHandle.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV1;
    LCD_LpTimHandle.Init.CounterSource   = LPTIM_COUNTERSOURCE_INTERNAL;
    LCD_LpTimHandle.Init.Trigger.Source  = LPTIM_TRIGSOURCE_SOFTWARE;
    LCD_LpTimHandle.Init.OutputPolarity  = LPTIM_OUTPUTPOLARITY_HIGH;
    LCD_LpTimHandle.Init.UpdateMode      = LPTIM_UPDATE_IMMEDIATE;
    LCD_LpTimHandle.Init.Input1Source    = LPTIM_INPUT1SOURCE_COMP1;

    /* Initialize LPTIM peripheral according to the passed parameters */
    if (HAL_LPTIM_Init(&LCD_LpTimHandle) != HAL_OK)
    {
      /* Initialization Error */
      BSP_ErrorHandler();
    }

    if (HAL_LPTIM_PWM_Start(&LCD_LpTimHandle, PERIOD_VALUE, i) != HAL_OK)
    {
      BSP_ErrorHandler();
    }


  }
  else
  {
    i++;
  }

  /* Optional delay to slow down the dimming transition. */
  HAL_Delay(PULSE_DECREASE_DELAY);
  if (HAL_LPTIM_PWM_Start(&LCD_LpTimHandle, PERIOD_VALUE, i) != HAL_OK)
  {
    BSP_ErrorHandler();
  }
  if (i == PULSE_VALUE)
  {
    dimming_enable = 0;
  }


#else
  /* Counter Prescaler value */
  uint32_t uhPrescalerValue = 0;
  int32_t   step;

  /* Reject this while ongoing dimming */
  if (dimming_config.ongoing)
  {
    return;
  }

  /* Set that dim feature is active */
  dimming_on = 1;
  dimming_config.ongoing = 1;

  if (dimming_config.stop > dimming_config.start)
  {
    step      = dimming_config.step;
  }
  else
  {
    step      = -dimming_config.step;
  }

  /* Always redo the full initialization as there is no apriori knowledge
    of IO or timer settings at this point (may have been modified by application) */

  /* Compute the prescaler value to have TIM1 counter clock equal to 16000000 Hz */
  uhPrescalerValue = (uint32_t)(SystemCoreClock / 16000000) - 1;

  LCD_TimHandle.Instance = TIMx;

  __HAL_TIM_RESET_HANDLE_STATE(&LCD_TimHandle); /* to force MSP call */
  LCD_TimHandle.Init.Prescaler         = uhPrescalerValue;
  LCD_TimHandle.Init.Period            = PERIOD_VALUE;
  LCD_TimHandle.Init.ClockDivision     = 0;
  LCD_TimHandle.Init.CounterMode       = (step > 0 ? TIM_COUNTERMODE_DOWN : TIM_COUNTERMODE_UP);
  LCD_TimHandle.Init.RepetitionCounter = 0;
  if (HAL_TIM_PWM_Init(&LCD_TimHandle) != HAL_OK)
  {
    /* Initialization Error */
    BSP_ErrorHandler();
  }

  /* Common configuration for all channels */
  LCD_sConfig.OCMode       = TIM_OCMODE_PWM1;
  LCD_sConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;
  LCD_sConfig.OCFastMode   = TIM_OCFAST_DISABLE;
  LCD_sConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  LCD_sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;

  LCD_sConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;

  /* Set the pulse value for channel */
  LCD_sConfig.Pulse = __DIMMING_CYCLE_VALUE(dimming_config.start);
  if (HAL_TIM_PWM_ConfigChannel(&LCD_TimHandle, &LCD_sConfig, TIMx_CHANNEL) != HAL_OK)
  {
    /* Configuration Error */
    BSP_ErrorHandler();
  }

  /* Start Timer channel */
  if (HAL_TIM_PWM_Start(&LCD_TimHandle, TIMx_CHANNEL) != HAL_OK)
  {
    /* PWM Generation Error */
    BSP_ErrorHandler();
  }

  /* Set the pulse value for the timer channel */
  i = dimming_config.start;
  while (dimming_config.ongoing)
  {
    __HAL_TIM_SET_COMPARE(&LCD_TimHandle, TIMx_CHANNEL, __DIMMING_CYCLE_VALUE(i));

    /* Exit if stop is reached */
    if (((step > 0) && (i >= dimming_config.stop))
        || ((step < 0) && (i <= dimming_config.stop)))
    {
      dimming_config.ongoing = 0;
    }
    else
    {
      HAL_Delay(dimming_config.delay);
      i += step;
    }
  }
#endif
}

/**
  * @brief  LCD screen dimming disable
  * @note   Screen brightness is immediately set to its highest level
  * @param  None
  * @retval None
  */
void BSP_LCD_ScreenDimmingOff(void)
{
#if defined(LPTIMER_DIMMING)
  GPIO_InitTypeDef GPIO_InitStructure;
#endif

  if (dimming_on == 1)
  {
    /* Stop ongoing dimming */
    dimming_config.ongoing = 0;

#if defined(LPTIMER_DIMMING)

    /* Restore LCD BL GPIO setting */
    GPIO_InitStructure.Pin       = LCD_BL_CTRL_PIN;
    GPIO_InitStructure.Mode      = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull      = GPIO_NOPULL;
    GPIO_InitStructure.Alternate = 0;
    GPIO_InitStructure.Speed   = GPIO_SPEED_LOW;
    HAL_GPIO_Init(LCD_BL_CTRL_GPIO_PORT, &GPIO_InitStructure);

    /* Stop LP Timer channel */
    if (HAL_LPTIM_PWM_Stop(&LCD_LpTimHandle) != HAL_OK)
    {
      /* PWM Generation Error */
      BSP_ErrorHandler();
    }

    /* Disable timer clock for power consumption reasons */
    __HAL_RCC_LPTIM1_CLK_DISABLE();
#else
    /* Stop Timer channel */
    if (HAL_TIM_PWM_Stop(&LCD_TimHandle, TIMx_CHANNEL) != HAL_OK)
    {
      /* PWM Generation Error */
      BSP_ErrorHandler();
    }

    /* Disable timer clock for power consumption reasons */
    TIMx_CLK_DISABLE();
#endif

    dimming_on = 0;
  }
}



/**
  * @brief TIM MSP Initialization
  *        This function configures the hardware resources used for screen dimming:
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration
  * @param htim: TIM handle pointer
  * @retval None
  */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
  GPIO_InitTypeDef   GPIO_InitStruct;

  /* TIMx Peripheral clock enable */
  TIMx_CLK_ENABLE();

  /* Timer channel configuration */

  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

  GPIO_InitStruct.Alternate = TIMx_CHANNEl_AF;
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);
}


#if defined(LPTIMER_DIMMING)
/**
* @brief  LPTIM MSP Init
* @param  hlptim : LPTIM handle
* @retval None
*/
void HAL_LPTIM_MspInit(LPTIM_HandleTypeDef *hlptim)
{
  GPIO_InitTypeDef     GPIO_InitStruct;

  /* ## - 1 - Enable LPTIM clock ############################################ */
  __HAL_RCC_LPTIM1_CLK_ENABLE();

  /* ## - 2 - Force & Release the LPTIM Periheral Clock Reset ############### */
  /* Force the LPTIM Periheral Clock Reset */
  __HAL_RCC_LPTIM1_FORCE_RESET();

  /* Release the LPTIM Periheral Clock Reset */
  __HAL_RCC_LPTIM1_RELEASE_RESET();

  /* ## - 3 - Enable & Configure LPTIM Ultra Low Power Input ################# */
  /* Configure PG.15 (LPTIM1_OUT) in alternate function,
  Low speed push-pull mode and pull-up enabled. */

  /* Enable GPIO PORT(s)*/
  LCD_BL_CTRL_GPIO_CLK_ENABLE();

  /* Configure Backlight control pin GPIO  */
  GPIO_InitStruct.Pin = LCD_BL_CTRL_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF1_LPTIM1;
  HAL_GPIO_Init(LCD_BL_CTRL_GPIO_PORT, &GPIO_InitStruct);
}
#endif /* defined(LPTIMER_DIMMING) */

uint8_t BSP_LCD_GetOrientation(void)
{
  return LCD_orientation;
}

/**
  * @brief  Initializes the LCD GPIO special pins MSP.
  * @param  None
  * @retval None
  */
__weak void BSP_LCD_MspInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable GPIOs clock */
  LCD_TE_GPIO_CLK_ENABLE();
  LCD_BL_CTRL_GPIO_CLK_ENABLE();
  LCD_PWR_CTRL_GPIO_CLK_ENABLE();

  /* LCD_RESET GPIO configuration */
  if (BSP_IO_Init() == IO_ERROR)
  {
    BSP_ErrorHandler();
  }
  BSP_IO_ConfigPin(LCD_RST_PIN, IO_MODE_OUTPUT);

  /* LCD_BL_CTRL GPIO configuration */
  GPIO_InitStructure.Pin       = LCD_BL_CTRL_PIN;   /* LCD_BL_CTRL pin has to be manually controlled */
  GPIO_InitStructure.Mode      = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull      = GPIO_NOPULL;
  GPIO_InitStructure.Alternate = 0;
  GPIO_InitStructure.Speed   = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_BL_CTRL_GPIO_PORT, &GPIO_InitStructure);

  /* Power on the screen (also done in Touch Screen driver ... */
  GPIO_InitStructure.Pin = LCD_PWR_CTRL_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP /*GPIO_MODE_OUTPUT_PP*/;
  GPIO_InitStructure.Pull =   GPIO_NOPULL;
  GPIO_InitStructure.Alternate = 0;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_PWR_CTRL_GPIO_PORT, &GPIO_InitStructure);
}

/**
  * @brief  DeInitializes LCD GPIO special pins MSP.
  * @param  None
  * @retval None
  */
__weak void BSP_LCD_MspDeInit(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  /* LCD_BL_CTRL GPIO deactivation */
  GPIO_InitStructure.Pin = LCD_BL_CTRL_PIN;
  HAL_GPIO_DeInit(LCD_BL_CTRL_GPIO_PORT, GPIO_InitStructure.Pin);

  /* LCD_BL_CTRL GPIO deactivation */
  GPIO_InitStructure.Pin = LCD_PWR_CTRL_PIN;
  HAL_GPIO_DeInit(LCD_PWR_CTRL_GPIO_PORT, GPIO_InitStructure.Pin);

  /* GPIO pins clock can be shut down in the application
     by surcharging this __weak function */
}

/******************************************************************************
                            Static Functions
*******************************************************************************/

/**
  * @brief  Draws a character on LCD.
  * @param  Xpos: Line where to display the character shape
  * @param  Ypos: Start column address
  * @param  c: Pointer to the character data
  * @retval None
  */
static void DrawChar(uint16_t Xpos, uint16_t Ypos, const uint8_t *c)
{
  uint32_t i = 0, j = 0;
  uint16_t height, width;
  uint8_t offset;
  uint8_t *pchar;
  uint32_t line;

  height = DrawProp.pFont->Height;
  width  = DrawProp.pFont->Width;

  offset =  8 * ((width + 7) / 8) -  width ;

  for (i = 0; i < height; i++)
  {
    pchar = ((uint8_t *)c + (width + 7) / 8 * i);

    switch (((width + 7) / 8))
    {
      case 1:
        line =  pchar[0];
        break;

      case 2:
        line = (pchar[0] << 8) | pchar[1];
        break;

      case 3:
      default:
        line = (pchar[0] << 16) | (pchar[1] << 8) | pchar[2];
        break;
    }

    for (j = 0; j < width; j++)
    {
      if ((line & (1 << (width - j + offset - 1))) != 0)
      {
        BSP_LCD_DrawPixel((Xpos + j), Ypos, DrawProp.TextColor);
      }
      else
      {
        BSP_LCD_DrawPixel((Xpos + j), Ypos, DrawProp.BackColor);
      }
    }
    Ypos++;
  }
}

/**
  * @brief  Sets display window.
  * @param  LayerIndex: layer index
  * @param  Xpos: LCD X position
  * @param  Ypos: LCD Y position
  * @param  Width: LCD window width
  * @param  Height: LCD window height
  * @retval None
  */
static void SetDisplayWindow(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height)
{
  if (LcdDrv->SetDisplayWindow != NULL)
  {
    LcdDrv->SetDisplayWindow(Xpos, Ypos, Width, Height);
  }
}

/**
  * @brief  Fills a triangle (between 3 points).
  * @param  Points: Pointer to the points array
  * @param  x1: Point 1 X position
  * @param  y1: Point 1 Y position
  * @param  x2: Point 2 X position
  * @param  y2: Point 2 Y position
  * @param  x3: Point 3 X position
  * @param  y3: Point 3 Y position
  * @retval None
  */
static void FillTriangle(uint16_t x1, uint16_t x2, uint16_t x3, uint16_t y1, uint16_t y2, uint16_t y3)
{
  int16_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0,
          yinc1 = 0, yinc2 = 0, den = 0, num = 0, numadd = 0, numpixels = 0,
          curpixel = 0;

  deltax = ABS(x2 - x1);        /* The difference between the x's */
  deltay = ABS(y2 - y1);        /* The difference between the y's */
  x = x1;                       /* Start x off at the first pixel */
  y = y1;                       /* Start y off at the first pixel */

  if (x2 >= x1)                 /* The x-values are increasing */
  {
    xinc1 = 1;
    xinc2 = 1;
  }
  else                          /* The x-values are decreasing */
  {
    xinc1 = -1;
    xinc2 = -1;
  }

  if (y2 >= y1)                 /* The y-values are increasing */
  {
    yinc1 = 1;
    yinc2 = 1;
  }
  else                          /* The y-values are decreasing */
  {
    yinc1 = -1;
    yinc2 = -1;
  }

  if (deltax >= deltay)         /* There is at least one x-value for every y-value */
  {
    xinc1 = 0;                  /* Don't change the x when numerator >= denominator */
    yinc2 = 0;                  /* Don't change the y for every iteration */
    den = deltax;
    num = deltax / 2;
    numadd = deltay;
    numpixels = deltax;         /* There are more x-values than y-values */
  }
  else                          /* There is at least one y-value for every x-value */
  {
    xinc2 = 0;                  /* Don't change the x for every iteration */
    yinc1 = 0;                  /* Don't change the y when numerator >= denominator */
    den = deltay;
    num = deltay / 2;
    numadd = deltax;
    numpixels = deltay;         /* There are more y-values than x-values */
  }

  for (curpixel = 0; curpixel <= numpixels; curpixel++)
  {
    BSP_LCD_DrawLine(x, y, x3, y3);

    num += numadd;              /* Increase the numerator by the top of the fraction */
    if (num >= den)             /* Check if numerator >= denominator */
    {
      num -= den;               /* Calculate the new numerator value */
      x += xinc1;               /* Change the x as appropriate */
      y += yinc1;               /* Change the y as appropriate */
    }
    x += xinc2;                 /* Change the x as appropriate */
    y += yinc2;                 /* Change the y as appropriate */
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


