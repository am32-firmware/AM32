/**
  ******************************************************************************
  * @file    stm32l476g_eval_lcd.c
  * @author  MCD Application Team
  * @brief   This file includes the driver for Liquid Crystal Display (LCD) module
  *          mounted on STM32L476G-EVAL evaluation board.
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
   - This driver is used to drive indirectly an LCD TFT.
   - This driver supports the HX8347G and HX8347I LCD mounted on MB895 daughter board

2. Driver description:
---------------------
  + Initialization steps:
     o Initialize the LCD using the BSP_LCD_Init() function.

  + Display on LCD
     o Clear the hole LCD using the BSP_LCD_Clear() function or only one specified
       string line using the BSP_LCD_ClearStringLine() function.
     o Display a character on the specified line and column using the BSP_LCD_DisplayChar()
       function or a complete string line using the BSP_LCD_DisplayStringAtLine() function.
     o Display a string line on the specified position (x,y in pixel) and align mode
       using the BSP_LCD_DisplayStringAtLine() function.
     o Draw and fill a basic shapes (dot, line, rectangle, circle, ellipse, .. bitmap, raw picture)
       on LCD using a set of functions.

------------------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "stm32l476g_eval_lcd.h"
#include "../../../Utilities/Fonts/fonts.h"
#include "../../../Utilities/Fonts/font24.c"
#include "../../../Utilities/Fonts/font20.c"
#include "../../../Utilities/Fonts/font16.c"
#include "../../../Utilities/Fonts/font12.c"
#include "../../../Utilities/Fonts/font8.c"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L476G_EVAL
  * @{
  */

/** @defgroup STM32L476G_EVAL_LCD STM32L476G_EVAL LCD
  * @{
  */

/** @defgroup STM32L476G_EVAL_LCD_Private_Types_Definitions Private Types Definitions
  * @{
  */

/**
  * @}
  */

/** @defgroup STM32L476G_EVAL_LCD_Private_Constants Private Constants
  * @{
  */
#define POLY_X(Z)               ((int32_t)((pPoints + (Z))->X))
#define POLY_Y(Z)               ((int32_t)((pPoints + (Z))->Y))

#define MAX_HEIGHT_FONT         17
#define MAX_WIDTH_FONT          24
#define OFFSET_BITMAP           54
/**
  * @}
  */

/** @defgroup STM32L476G_EVAL_LCD_Private_Macros Private Macros
  * @{
  */
#define ABS(X)  ((X) > 0 ? (X) : -(X))

/**
  * @}
  */

/** @defgroup STM32L476G_EVAL_LCD_Private_Variables Private Variables
  * @{
  */
LCD_DrawPropTypeDef DrawProp;

static LCD_DrvTypeDef  *lcd_drv;

/* Max size of bitmap will based on a font24 (17x24) */
static uint8_t bitmap[MAX_HEIGHT_FONT*MAX_WIDTH_FONT*2+OFFSET_BITMAP] = {0};

/**
  * @}
  */

/** @defgroup STM32L476G_EVAL_LCD_Private_Functions Private Functions
  * @{
  */
static void LCD_DrawPixel(uint16_t Xpos, uint16_t Ypos, uint16_t RGBCode);
static void LCD_DrawChar(uint16_t Xpos, uint16_t Ypos, const uint8_t *pChar);
static void LCD_SetDisplayWindow(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height);

/**
  * @}
  */

/** @addtogroup STM32L476G_EVAL_LCD_Exported_Functions
  * @{
  */

/**
  * @brief  Initializes the LCD.
  * @retval LCD state (LCD_OK/LCD_ERROR)
  */
uint8_t BSP_LCD_Init(void)
{
  /* Default value for draw propriety */
  DrawProp.BackColor = 0xFFFF;
  DrawProp.pFont     = &Font24;
  DrawProp.TextColor = 0x0000;

  if(hx8347i_drv.ReadID() == HX8347I_ID)
  {
    lcd_drv = &hx8347i_drv;
  }
  else if(hx8347g_drv.ReadID() == HX8347G_ID)
  {
    lcd_drv = &hx8347g_drv;
  }
  else
  {
    /* Unknown LCD */
    return LCD_ERROR;
  }

  /* LCD Init */
  lcd_drv->Init();

  /* Initialize the font */
  BSP_LCD_SetFont(&LCD_DEFAULT_FONT);

  return LCD_OK;
}

/**
  * @brief  Gets the LCD X size.
  * @retval Used LCD X size
  */
uint32_t BSP_LCD_GetXSize(void)
{
  return(lcd_drv->GetLcdPixelWidth());
}

/**
  * @brief  Gets the LCD Y size.
  * @retval Used LCD Y size
  */
uint32_t BSP_LCD_GetYSize(void)
{
  return(lcd_drv->GetLcdPixelHeight());
}

/**
  * @brief  Gets the LCD text color.
  * @retval Used text color.
  */
uint16_t BSP_LCD_GetTextColor(void)
{
  return DrawProp.TextColor;
}

/**
  * @brief  Gets the LCD background color.
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
  * @param  pFonts: Font to be used
  * @retval None
  */
void BSP_LCD_SetFont(sFONT *pFonts)
{
  DrawProp.pFont = pFonts;
}

/**
  * @brief  Gets the LCD text font.
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

  uint32_t color_backup = DrawProp.TextColor;
  DrawProp.TextColor = Color;

  for(counter = 0; counter < BSP_LCD_GetYSize(); counter++)
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
  uint32_t colorbackup = DrawProp.TextColor;
  DrawProp.TextColor = DrawProp.BackColor;;

  /* Draw a rectangle with background color */
  BSP_LCD_FillRect(0, (Line * DrawProp.pFont->Height), BSP_LCD_GetXSize(), DrawProp.pFont->Height);

  DrawProp.TextColor = colorbackup;
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
  LCD_DrawChar(Ypos, Xpos, &DrawProp.pFont->table[(Ascii-' ') *\
    DrawProp.pFont->Height * ((DrawProp.pFont->Width + 7) / 8)]);
}

/**
  * @brief  Displays characters on the LCD.
  * @param  Xpos: X position (in pixel)
  * @param  Ypos: Y position (in pixel)
  * @param  pText: Pointer to string to display on LCD
  * @param  Mode: Display mode
  *          This parameter can be one of the following values:
  *            @arg  CENTER_MODE
  *            @arg  RIGHT_MODE
  *            @arg  LEFT_MODE
  * @retval None
  */
void BSP_LCD_DisplayStringAt(uint16_t Xpos, uint16_t Ypos, uint8_t *pText, Line_ModeTypdef Mode)
{
  uint16_t refcolumn = 1, counter = 0;
  uint32_t size = 0, ysize = 0;
  uint8_t  *ptr = pText;

  /* Get the text size */
  while (*ptr++) size ++ ;

  /* Characters number per line */
  ysize = (BSP_LCD_GetXSize()/DrawProp.pFont->Width);

  switch (Mode)
  {
  case CENTER_MODE:
    {
      refcolumn = Xpos + ((ysize - size)* DrawProp.pFont->Width) / 2;
      break;
    }
  case LEFT_MODE:
    {
      refcolumn = Xpos;
      break;
    }
  case RIGHT_MODE:
    {
      refcolumn = -Xpos + ((ysize - size)*DrawProp.pFont->Width);
      break;
    }
  default:
    {
      refcolumn = Xpos;
      break;
    }
  }

  /* Send the string character by character on lCD */
  while ((*pText != 0) && (((BSP_LCD_GetXSize() - (counter*DrawProp.pFont->Width)) & 0xFFFF) >= DrawProp.pFont->Width))
  {
    /* Display one character on LCD */
    BSP_LCD_DisplayChar(refcolumn, Ypos, *pText);
    /* Decrement the column position by 16 */
    refcolumn += DrawProp.pFont->Width;
    /* Point on the next character */
    pText++;
    counter++;
  }
}

/**
  * @brief  Displays a character on the LCD.
  * @param  Line: Line where to display the character shape
  *          This parameter can be one of the following values:
  *            @arg  0..9: if the Current fonts is Font16x24
  *            @arg  0..19: if the Current fonts is Font12x12 or Font8x12
  *            @arg  0..29: if the Current fonts is Font8x8
  * @param  pText: Pointer to string to display on LCD
  * @retval None
  */
void BSP_LCD_DisplayStringAtLine(uint16_t Line, uint8_t *pText)
{
  BSP_LCD_DisplayStringAt(0, LINE(Line),pText, LEFT_MODE);
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

  if(lcd_drv->ReadPixel != NULL)
  {
    ret = lcd_drv->ReadPixel(Xpos, Ypos);
  }

  return ret;
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

  if(lcd_drv->DrawHLine != NULL)
  {
    lcd_drv->DrawHLine(DrawProp.TextColor, Ypos, Xpos, Length);
  }
  else
  {
    for(index = 0; index < Length; index++)
    {
      LCD_DrawPixel((Ypos + index), Xpos, DrawProp.TextColor);
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

  if(lcd_drv->DrawVLine != NULL)
  {
    LCD_SetDisplayWindow(Ypos, Xpos, 1, Length);
    lcd_drv->DrawVLine(DrawProp.TextColor, Ypos, Xpos, Length);
    LCD_SetDisplayWindow(0, 0, BSP_LCD_GetXSize(), BSP_LCD_GetYSize());
  }
  else
  {
    for(index = 0; index < Length; index++)
    {
      LCD_DrawPixel(Ypos, Xpos + index, DrawProp.TextColor);
    }
  }
}

/**
  * @brief  Draws an uni-line (between two points).
  * @param  X1: Point 1 X position
  * @param  Y1: Point 1 Y position
  * @param  X2: Point 2 X position
  * @param  Y2: Point 2 Y position
  * @retval None
  */
void BSP_LCD_DrawLine(uint16_t X1, uint16_t Y1, uint16_t X2, uint16_t Y2)
{
  int16_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0,
  yinc1 = 0, yinc2 = 0, den = 0, num = 0, numadd = 0, numpixels = 0,
  curpixel = 0;

  deltax = ABS(Y2 - Y1);        /* The difference between the x's */
  deltay = ABS(X2 - X1);        /* The difference between the y's */
  x = Y1;                       /* Start x off at the first pixel */
  y = X1;                       /* Start y off at the first pixel */

  if (Y2 >= Y1)                 /* The x-values are increasing */
  {
    xinc1 = 1;
    xinc2 = 1;
  }
  else                          /* The x-values are decreasing */
  {
    xinc1 = -1;
    xinc2 = -1;
  }

  if (X2 >= X1)                 /* The y-values are increasing */
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
    LCD_DrawPixel(x, y, DrawProp.TextColor);  /* Draw the current pixel */
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
  BSP_LCD_DrawHLine(Xpos, (Ypos+ Height), Width);

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
  uint32_t  curx;   /* Current X Value */
  uint32_t  cury;   /* Current Y Value */

  decision = 3 - (Radius << 1);
  curx = 0;
  cury = Radius;

  while (curx <= cury)
  {
    LCD_DrawPixel((Ypos + curx), (Xpos - cury), DrawProp.TextColor);

    LCD_DrawPixel((Ypos - curx), (Xpos - cury), DrawProp.TextColor);

    LCD_DrawPixel((Ypos + cury), (Xpos - curx), DrawProp.TextColor);

    LCD_DrawPixel((Ypos - cury), (Xpos - curx), DrawProp.TextColor);

    LCD_DrawPixel((Ypos + curx), (Xpos + cury), DrawProp.TextColor);

    LCD_DrawPixel((Ypos - curx), (Xpos + cury), DrawProp.TextColor);

    LCD_DrawPixel((Ypos + cury), (Xpos + curx), DrawProp.TextColor);

    LCD_DrawPixel((Ypos - cury), (Xpos + curx), DrawProp.TextColor);

    /* Initialize the font */
    BSP_LCD_SetFont(&LCD_DEFAULT_FONT);

    if (decision < 0)
    {
      decision += (curx << 2) + 6;
    }
    else
    {
      decision += ((curx - cury) << 2) + 10;
      cury--;
    }
    curx++;
  }
}

/**
  * @brief  Draws an poly-line (between many points).
  * @param  pPoints: Pointer to the points array
  * @param  PointCount: Number of points
  * @retval None
  */
void BSP_LCD_DrawPolygon(pPoint pPoints, uint16_t PointCount)
{
  int16_t x = 0, y = 0;

  if(PointCount < 2)
  {
    return;
  }

  BSP_LCD_DrawLine(pPoints->X, pPoints->Y, (pPoints+PointCount-1)->X, (pPoints+PointCount-1)->Y);

  while(--PointCount)
  {
    x = pPoints->X;
    y = pPoints->Y;
    pPoints++;
    BSP_LCD_DrawLine(x, y, pPoints->X, pPoints->Y);
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
  int x = 0, y = -XRadius, err = 2-2*YRadius, e2;
  float k = 0, rad1 = 0, rad2 = 0;

  rad1 = YRadius;
  rad2 = XRadius;

  k = (float)(rad2/rad1);

  do {
    LCD_DrawPixel((Ypos-(uint16_t)(x/k)), (Xpos+y), DrawProp.TextColor);
    LCD_DrawPixel((Ypos+(uint16_t)(x/k)), (Xpos+y), DrawProp.TextColor);
    LCD_DrawPixel((Ypos+(uint16_t)(x/k)), (Xpos-y), DrawProp.TextColor);
    LCD_DrawPixel((Ypos-(uint16_t)(x/k)), (Xpos-y), DrawProp.TextColor);

    e2 = err;
    if (e2 <= x) {
      err += ++x*2+1;
      if (-y == x && e2 <= y) e2 = 0;
    }
    if (e2 > y) err += ++y*2+1;
  }
  while (y <= 0);
}

/**
  * @brief  Draws a bitmap picture loaded in the internal Flash (32 bpp).
  * @param  Xpos: Bmp X position in the LCD
  * @param  Ypos: Bmp Y position in the LCD
  * @param  pBmp: Pointer to Bmp picture address in the internal Flash
  * @retval None
  */
void BSP_LCD_DrawBitmap(uint16_t Xpos, uint16_t Ypos, uint8_t *pBmp)
{
  uint32_t height = 0, width  = 0;

  /* Read bitmap width */
  width = *(uint16_t *) (pBmp + 18);
  width |= (*(uint16_t *) (pBmp + 20)) << 16;

  /* Read bitmap height */
  height = *(uint16_t *) (pBmp + 22);
  height |= (*(uint16_t *) (pBmp + 24)) << 16;

  /* Remap Ypos, hx8347g and hx8347i work with inverted X in case of bitmap */
  /* X = 0, cursor is on Bottom corner */
  Ypos = BSP_LCD_GetYSize() - Ypos - height;

  LCD_SetDisplayWindow(Ypos, Xpos, width, height);

  if(lcd_drv->DrawBitmap != NULL)
  {
    lcd_drv->DrawBitmap(Ypos, Xpos, pBmp);
  }
  LCD_SetDisplayWindow(0, 0, BSP_LCD_GetXSize(), BSP_LCD_GetYSize());
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
  while(Height--);
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
  uint32_t  curx;    /* Current X Value */
  uint32_t  cury;    /* Current Y Value */

  decision = 3 - (Radius << 1);

  curx = 0;
  cury = Radius;

  BSP_LCD_SetTextColor(DrawProp.TextColor);

  while (curx <= cury)
  {
    if(cury > 0)
    {
      BSP_LCD_DrawVLine(Xpos + curx, Ypos - cury, 2*cury);
      BSP_LCD_DrawVLine(Xpos - curx, Ypos - cury, 2*cury);
    }

    if(curx > 0)
    {
      BSP_LCD_DrawVLine(Xpos - cury, Ypos - curx, 2*curx);
      BSP_LCD_DrawVLine(Xpos + cury, Ypos - curx, 2*curx);
    }
    if (decision < 0)
    {
      decision += (curx << 2) + 6;
    }
    else
    {
      decision += ((curx - cury) << 2) + 10;
      cury--;
    }
    curx++;
  }

  BSP_LCD_SetTextColor(DrawProp.TextColor);
  BSP_LCD_DrawCircle(Xpos, Ypos, Radius);
}

/**
  * @brief  Fill triangle.
  * @param  X1: specifies the point 1 x position.
  * @param  X2: specifies the point 2 x position.
  * @param  X3: specifies the point 3 x position.
  * @param  Y1: specifies the point 1 y position.
  * @param  Y2: specifies the point 2 y position.
  * @param  Y3: specifies the point 3 y position.
  * @retval None
  */
void BSP_LCD_FillTriangle(uint16_t X1, uint16_t X2, uint16_t X3, uint16_t Y1, uint16_t Y2, uint16_t Y3)
{
  int16_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0,
  yinc1 = 0, yinc2 = 0, den = 0, num = 0, numadd = 0, numpixels = 0,
  curpixel = 0;

  deltax = ABS(X2 - X1);        /* The difference between the x's */
  deltay = ABS(Y2 - Y1);        /* The difference between the y's */
  x = X1;                       /* Start x off at the first pixel */
  y = Y1;                       /* Start y off at the first pixel */

  if (X2 >= X1)                 /* The x-values are increasing */
  {
    xinc1 = 1;
    xinc2 = 1;
  }
  else                          /* The x-values are decreasing */
  {
    xinc1 = -1;
    xinc2 = -1;
  }

  if (Y2 >= Y1)                 /* The y-values are increasing */
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
    BSP_LCD_DrawLine(x, y, X3, Y3);

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
  * @brief  Displays a  full poly-line (between many points).
  * @param  pPoints: pointer to the points array.
  * @param  PointCount: Number of points.
  * @retval None
  */
void BSP_LCD_FillPolygon(pPoint pPoints, uint16_t PointCount)
{

  int16_t x = 0, y = 0, x2 = 0, y2 = 0, xcenter = 0, ycenter = 0, xfirst = 0, yfirst = 0, pixelx = 0, pixely = 0, counter = 0;
  uint16_t  imageleft = 0, imageright = 0, imagetop = 0, imagebottom = 0;

  imageleft = imageright = pPoints->X;
  imagetop= imagebottom = pPoints->Y;

  for(counter = 1; counter < PointCount; counter++)
  {
    pixelx = POLY_X(counter);
    if(pixelx < imageleft)
    {
      imageleft = pixelx;
    }
    if(pixelx > imageright)
    {
      imageright = pixelx;
    }

    pixely = POLY_Y(counter);
    if(pixely < imagetop)
    {
      imagetop = pixely;
    }
    if(pixely > imagebottom)
    {
      imagebottom = pixely;
    }
  }

  if(PointCount < 2)
  {
    return;
  }

  xcenter = (imageleft + imageright)/2;
  ycenter = (imagebottom + imagetop)/2;

  xfirst = pPoints->X;
  yfirst = pPoints->Y;

  while(--PointCount)
  {
    x = pPoints->X;
    y = pPoints->Y;
    pPoints++;
    x2 = pPoints->X;
    y2 = pPoints->Y;

    BSP_LCD_FillTriangle(x, x2, xcenter, y, y2, ycenter);
    BSP_LCD_FillTriangle(x, xcenter, x2, y, ycenter, y2);
    BSP_LCD_FillTriangle(xcenter, x2, x, ycenter, y2, y);
  }

  BSP_LCD_FillTriangle(xfirst, x2, xcenter, yfirst, y2, ycenter);
  BSP_LCD_FillTriangle(xfirst, xcenter, x2, yfirst, ycenter, y2);
  BSP_LCD_FillTriangle(xcenter, x2, xfirst, ycenter, y2, yfirst);
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
  int x = 0, y = -XRadius, err = 2-2*YRadius, e2;
  float k = 0, rad1 = 0, rad2 = 0;

  rad1 = YRadius;
  rad2 = XRadius;

  k = (float)(rad2/rad1);

  do
  {
    BSP_LCD_DrawVLine((Xpos+y), (Ypos-(uint16_t)(x/k)), (2*(uint16_t)(x/k) + 1));
    BSP_LCD_DrawVLine((Xpos-y), (Ypos-(uint16_t)(x/k)), (2*(uint16_t)(x/k) + 1));

    e2 = err;
    if (e2 <= x)
    {
      err += ++x*2+1;
      if (-y == x && e2 <= y) e2 = 0;
    }
    if (e2 > y) err += ++y*2+1;
  }
  while (y <= 0);
}

/**
  * @brief  Enables the display.
  * @retval None
  */
void BSP_LCD_DisplayOn(void)
{
  lcd_drv->DisplayOn();
}

/**
  * @brief  Disables the display.
  * @retval None
  */
void BSP_LCD_DisplayOff(void)
{
  lcd_drv->DisplayOff();
}

/******************************************************************************
                            Static Function
*******************************************************************************/
/**
  * @brief  Draws a pixel on LCD.
  * @param  Xpos: X position
  * @param  Ypos: Y position
  * @param  RGBCode: Pixel color in RGB mode (5-6-5)
  * @retval None
  */
static void LCD_DrawPixel(uint16_t Xpos, uint16_t Ypos, uint16_t RGBCode)
{
  if(lcd_drv->WritePixel != NULL)
  {
    lcd_drv->WritePixel(Xpos, Ypos, RGBCode);
  }
}

/**
  * @brief  Draws a character on LCD.
  * @param  Xpos: Line where to display the character shape
  * @param  Ypos: Start column address
  * @param  pChar: Pointer to the character data
  * @retval None
  */
static void LCD_DrawChar(uint16_t Xpos, uint16_t Ypos, const uint8_t *pChar)
{
  uint32_t counterh = 0, counterw = 0, index = 0;
  uint16_t height = 0, width = 0;
  uint8_t offset = 0;
  uint8_t *pchar = NULL;
  uint32_t line = 0;

  height = DrawProp.pFont->Height;
  width  = DrawProp.pFont->Width;

  /* Fill bitmap header*/
  *(uint16_t *) (bitmap + 2) = (uint16_t)(height*width*2+OFFSET_BITMAP);
  *(uint16_t *) (bitmap + 4) = (uint16_t)((height*width*2+OFFSET_BITMAP)>>16);
  *(uint16_t *) (bitmap + 10) = OFFSET_BITMAP;
  *(uint16_t *) (bitmap + 18) = (uint16_t)(width);
  *(uint16_t *) (bitmap + 20) = (uint16_t)((width)>>16);
  *(uint16_t *) (bitmap + 22) = (uint16_t)(height);
  *(uint16_t *) (bitmap + 24) = (uint16_t)((height)>>16);

  offset =  8 *((width + 7)/8) -  width ;

  for(counterh = 0; counterh < height; counterh++)
  {
    pchar = ((uint8_t *)pChar + (width + 7)/8 * counterh);

    if(((width + 7)/8) == 3)
    {
      line =  (pchar[0]<< 16) | (pchar[1]<< 8) | pchar[2];
    }

    if(((width + 7)/8) == 2)
    {
      line =  (pchar[0]<< 8) | pchar[1];
    }

    if(((width + 7)/8) == 1)
    {
      line =  pchar[0];
    }

    for (counterw = 0; counterw < width; counterw++)
    {
      /* Image in the bitmap is written from the bottom to the top */
      /* Need to invert image in the bitmap */
      index = (((height-counterh-1)*width)+(counterw))*2+OFFSET_BITMAP;
      if((line & (1 << (width- counterw + offset- 1))) != 0)
      {
        bitmap[index] = (uint8_t)DrawProp.TextColor;
        bitmap[index+1] = (uint8_t)(DrawProp.TextColor >> 8);
      }
      else
      {
        bitmap[index] = (uint8_t)DrawProp.BackColor;
        bitmap[index+1] = (uint8_t)(DrawProp.BackColor >> 8);
      }
    }
  }

  BSP_LCD_DrawBitmap(Ypos, Xpos, bitmap);
}

/**
  * @brief  Sets display window.
  * @param  Xpos: LCD X position
  * @param  Ypos: LCD Y position
  * @param  Width: LCD window width
  * @param  Height: LCD window height
  * @retval None
  */
static void LCD_SetDisplayWindow(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height)
{
  if(lcd_drv->SetDisplayWindow != NULL)
  {
    lcd_drv->SetDisplayWindow(Xpos, Ypos, Width, Height);
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


