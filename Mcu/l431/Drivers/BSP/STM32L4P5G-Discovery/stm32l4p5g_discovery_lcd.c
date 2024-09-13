/**
  ******************************************************************************
  * @file    stm32l4p5g_discovery_lcd.c
  * @author  MCD Application Team
  * @brief   This file includes the driver for Liquid Crystal Display (LCD) module
  *          mounted on STM32L4P5G_DISCOVERY board.
  @verbatim
  1. How To use this driver:
  --------------------------
     - This driver is used to drive directly an LCD TFT using the LTDC controller.
     - This driver uses timing and setting for LPM012M503A LCD.

  2. Driver description:
  ---------------------
    + Initialization steps:
       o Initialize the LCD using the BSP_LCD_Init() function.
       o Apply the Layer configuration using the BSP_LCD_LayerDefaultInit() function.
       o Select the LCD layer to be used using the BSP_LCD_SelectLayer() function.
       o Enable the LCD display using the BSP_LCD_DisplayOn() function.

    + Options
       o Configure and enable the color keying functionality using the
         BSP_LCD_SetColorKeying() function.
       o Modify in the fly the transparency and/or the frame buffer address
         using the following functions:
         - BSP_LCD_SetTransparency()
         - BSP_LCD_SetLayerAddress()

    + Display on LCD
       o Clear the hole LCD using BSP_LCD_Clear() function or only one specified string
         line using the BSP_LCD_ClearStringLine() function.
       o Display a character on the specified line and column using the BSP_LCD_DisplayChar()
         function or a complete string line using the BSP_LCD_DisplayStringAtLine() function.
       o Display a string line on the specified position (x,y in pixel) and align mode
         using the BSP_LCD_DisplayStringAtLine() function.
       o Draw and fill a basic shapes (dot, line, rectangle, circle, ellipse, .. bitmap)
         on LCD using the available set of functions.
  @endverbatim
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32l4p5g_discovery_lcd.h"
#include "stm32l4p5g_discovery_io.h"
#include "../../../Utilities/Fonts/fonts.h"
#include "../../../Utilities/Fonts/font24.c"
#include "../../../Utilities/Fonts/font20.c"
#include "../../../Utilities/Fonts/font16.c"
#include "../../../Utilities/Fonts/font12.c"
#include "../../../Utilities/Fonts/font8.c"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L4P5G_DISCOVERY
  * @{
  */

/** @defgroup STM32L4P5G_DISCOVERY_LCD STM32L4P5G_DISCOVERY LCD
  * @{
  */

/** @defgroup LPM012M503A_Exported_Constants
  * @{
  */

#define RTC_ASYNCH_PREDIV    0x7F /* LSE */
#define RTC_SYNCH_PREDIV     0xFF /* LSE */

/** @defgroup STM32L4P5G_DISCOVERY_RGB_LCD_Private_Defines Private Defines
  * @{
  */
#define POLY_X(Z)              ((int32_t)((Points + Z)->X))
#define POLY_Y(Z)              ((int32_t)((Points + Z)->Y))

/**
  * @}
  */

/** @defgroup STM32L4P5G_DISCOVERY_RGB_LCD_Private_Macros Private Macros
  * @{
  */
#define ABS(X)  ((X) > 0 ? (X) : -(X))
/**
  * @}
  */

/** @defgroup STM32L4P5G_DISCOVERY_RGB_LCD_Exported_Variables Exported Variables
  * @{
  */
/* DMA2D handle */
DMA2D_HandleTypeDef  hdma2d_disco;
/**
  * @}
  */

/** @defgroup STM32L4P5G_DISCOVERY_RGB_LCD_Private_Variables Private Variables
  * @{
  */


/* LTDC handle */
LTDC_HandleTypeDef  hltdc_disco;
RTC_HandleTypeDef RtcHandle_disco;

/* Default LCD configuration with LCD Layer 1 */
static uint32_t            ActiveLayer = LTDC_DEFAULT_ACTIVE_LAYER;
static LCD_DrawPropTypeDef DrawProp[LTDC_MAX_LAYER_NUMBER];
static uint8_t RTC_Config(void);
/**
  * @}
  */

/** @defgroup STM32L4P5G_DISCOVERY_RGB_LCD_Private_FunctionPrototypes Private Function Prototypes
  * @{
  */
static void DrawChar(uint16_t Xpos, uint16_t Ypos, const uint8_t *c);
static void FillTriangle(uint16_t x1, uint16_t x2, uint16_t x3, uint16_t y1, uint16_t y2, uint16_t y3);
static void LL_FillBuffer(uint32_t LayerIndex, void *pDst, uint32_t xSize, uint32_t ySize, uint32_t OffLine, uint32_t ColorIndex);
static void LL_ConvertLineToARGB8888(void * pSrc, void *pDst, uint32_t xSize, uint32_t ColorMode);
/**
  * @}
  */

/** @defgroup STM32L4P5G_DISCOVERY_RGB_LCD_Exported_Functions Exported Functions
  * @{
  */

/**
  * @brief  Initializes the LCD.
  * @retval LCD state
  */
uint8_t BSP_LCD_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIOs Configuration */
  /* Display */
  LCD_DISP_GPIO_CLK_ENABLE();
  GPIO_InitStruct.Pin = LCD_DISP_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(LCD_DISP_GPIO_PORT, &GPIO_InitStruct);

  /* Data Enable */
  LCD_DE_GPIO_CLK_ENABLE();
  GPIO_InitStruct.Pin = LCD_DE_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(LCD_DE_GPIO_PORT, &GPIO_InitStruct);

  /* Backlight */
  LCD_BL_CTRL_GPIO_CLK_ENABLE();
  GPIO_InitStruct.Pin = LCD_BL_CTRL_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(LCD_BL_CTRL_GPIO_PORT, &GPIO_InitStruct);

  /* RTC configuration */
  if (RTC_Config() != LCD_OK)
  {
    return LCD_ERROR;
  }

  /* Enable Display */
  BSP_LCD_DisplayOn();

  /* LCD clock configuration */
  if (BSP_LCD_ClockConfig() != LCD_OK)
  {
    return LCD_ERROR;
  }



  /* LTDC setting */

  /* Timing Configuration */
  hltdc_disco.Init.HorizontalSync = 1;                // HSYNC width = 2 cycles
  hltdc_disco.Init.VerticalSync = 0;                  // VSYNC width = 1 cycle
  hltdc_disco.Init.AccumulatedHBP = 1+ 2;             // HBP = 2 cycles
  hltdc_disco.Init.AccumulatedVBP = 0+ 1;             // VBP = 1 cycle
  hltdc_disco.Init.AccumulatedActiveW = 1+2 + 240;    // HA  = 240  cycles
  hltdc_disco.Init.AccumulatedActiveH = 0+1 + 240;    // VA  = 240  cycles
  hltdc_disco.Init.TotalWidth = 1+2+240 + 2;          // HFP = 2 cycles
  hltdc_disco.Init.TotalHeigh = 0+1+240 + 2 +1;       // VFP = 2 cycles + 1 Extra cycle for synchronous stop
  /* Initialize the LCD pixel width and pixel height */
  hltdc_disco.LayerCfg->ImageWidth = 240;
  hltdc_disco.LayerCfg->ImageHeight = 240;
  /* Background value */
  hltdc_disco.Init.Backcolor.Blue = 0;
  hltdc_disco.Init.Backcolor.Green = 0x30; // 0XC0 : Blue /
  hltdc_disco.Init.Backcolor.Red = 0;
  /* Polarity */
  hltdc_disco.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc_disco.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc_disco.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc_disco.Init.PCPolarity = LTDC_PCPOLARITY_IIPC;
  hltdc_disco.Instance = LTDC;


  if(HAL_LTDC_GetState(&hltdc_disco) == HAL_LTDC_STATE_RESET)
  {
    /* Initialize the LCD Msp: this __weak function can be rewritten by the application */
    BSP_LCD_MspInit(&hltdc_disco, NULL);
  }
  HAL_LTDC_Init(&hltdc_disco);



  return LCD_OK;
}

/**
  * @brief  DeInitializes the LCD.
  * @retval LCD state
  */
uint8_t BSP_LCD_DeInit(void)
{
  /* Initialize the hltdc_disco Instance parameter */
  hltdc_disco.Instance = LTDC;

 /* Disable LTDC block */
  __HAL_LTDC_DISABLE(&hltdc_disco);

  /* DeInit the LTDC */
  HAL_LTDC_DeInit(&hltdc_disco);

  /* DeInit the LTDC MSP : this __weak function can be rewritten by the application */
  BSP_LCD_MspDeInit(&hltdc_disco, NULL);

  return LCD_OK;
}

/**
  * @brief  Gets the LCD X size.
  * @retval Used LCD X size
  */
uint32_t BSP_LCD_GetXSize(void)
{
  return hltdc_disco.LayerCfg[ActiveLayer].ImageWidth;
}

/**
  * @brief  Gets the LCD Y size.
  * @retval Used LCD Y size
  */
uint32_t BSP_LCD_GetYSize(void)
{
  return hltdc_disco.LayerCfg[ActiveLayer].ImageHeight;
}

/**
  * @brief  Set the LCD X size.
  * @param  imageWidthPixels : image width in pixels unit
  * @retval None
  */
void BSP_LCD_SetXSize(uint32_t imageWidthPixels)
{
  hltdc_disco.LayerCfg[ActiveLayer].ImageWidth = imageWidthPixels;
}

/**
  * @brief  Set the LCD Y size.
  * @param  imageHeightPixels : image height in lines unit
  * @retval None
  */
void BSP_LCD_SetYSize(uint32_t imageHeightPixels)
{
  hltdc_disco.LayerCfg[ActiveLayer].ImageHeight = imageHeightPixels;
}

/**
  * @brief  Initializes the LCD layer in ARGB8888 format (32 bits per pixel).
  * @param  LayerIndex: Layer foreground or background
  * @param  FB_Address: Layer frame buffer
  * @retval None
  */
void BSP_LCD_LayerDefaultInit(uint32_t address)
{

  /* Configure the LTDC layer */
  LTDC_LayerCfgTypeDef  layer_cfg;

  layer_cfg.WindowX0 = 0;
  layer_cfg.WindowX1 = 240;
  layer_cfg.WindowY0 = 0;
  layer_cfg.WindowY1 = 240;
  layer_cfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB888;
  layer_cfg.FBStartAdress = address;
  layer_cfg.Alpha = 255;
  layer_cfg.Alpha0 = 0;
  layer_cfg.Backcolor.Blue = 0;
  layer_cfg.Backcolor.Green = 0;
  layer_cfg.Backcolor.Red = 0;
  layer_cfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  layer_cfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  layer_cfg.ImageWidth = 240;
  layer_cfg.ImageHeight = 240;
  HAL_LTDC_ConfigLayer(&hltdc_disco, &layer_cfg, 0);

}

/**
  * @brief  Initializes the LCD layer in RGB565 format (16 bits per pixel).
  * @param  LayerIndex: Layer foreground or background
  * @param  FB_Address: Layer frame buffer
  * @retval None
  */
void BSP_LCD_LayerRgb565Init(uint32_t address)
{
  /* Configure the LTDC layer */
  LTDC_LayerCfgTypeDef  layer_cfg;

  layer_cfg.WindowX0 = 0;
  layer_cfg.WindowX1 = 240;
  layer_cfg.WindowY0 = 0;
  layer_cfg.WindowY1 = 240;
  layer_cfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
  layer_cfg.FBStartAdress = address;
  layer_cfg.Alpha = 255;
  layer_cfg.Alpha0 = 0;
  layer_cfg.Backcolor.Blue = 0;
  layer_cfg.Backcolor.Green = 0;
  layer_cfg.Backcolor.Red = 0;
  layer_cfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  layer_cfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  layer_cfg.ImageWidth = 240;
  layer_cfg.ImageHeight = 240;
  HAL_LTDC_ConfigLayer(&hltdc_disco, &layer_cfg, 0);
}

/**
  * @brief  Selects the LCD Layer.
  * @param  LayerIndex: Layer foreground or background
  * @retval None
  */
void BSP_LCD_SelectLayer(uint32_t LayerIndex)
{
  ActiveLayer = LayerIndex;
}

/**
  * @brief  Sets an LCD Layer visible
  * @param  LayerIndex: Visible Layer
  * @param  State: New state of the specified layer
  *          This parameter can be one of the following values:
  *            @arg  ENABLE
  *            @arg  DISABLE
  * @retval None
  */
void BSP_LCD_SetLayerVisible(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

   /* Enable interrupt */
   HAL_LTDC_ProgramLineEvent(&hltdc_disco, 244);
  __HAL_LTDC_ENABLE_IT(&hltdc_disco, LTDC_IT_LI);

  /* Enable the LTDC */
  __HAL_LTDC_ENABLE(&hltdc_disco);

  /* Enable the Pads */

    /* Set pixel clock */
  LCD_PIXELS_CLOCK_GPIO_CLK_ENABLE();
  GPIO_InitStruct.Pin = LCD_PIXELS_CLOCK_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  GPIO_InitStruct.Alternate = GPIO_AF11_LTDC;
  HAL_GPIO_Init(LCD_PIXELS_CLOCK_GPIO_PORT, &GPIO_InitStruct);

  /* RGB pins */
  /* PE2..3 R6..7 */
  /* PE5..6 G6..7 */
  /* PE7..8 B6..7 */
  LCD_RGB_GPIO_CLK_ENABLE();
  GPIO_InitStruct.Pin = LCD_R6_PIN | LCD_R7_PIN | LCD_G6_PIN  | LCD_G7_PIN | LCD_B6_PIN  | LCD_B7_PIN  ;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  GPIO_InitStruct.Alternate = GPIO_AF11_LTDC;
  HAL_GPIO_Init(LCD_RGB_GPIO_PORT, &GPIO_InitStruct);


  /* Set HSYNC and VSYNC IOs */
  LCD_HSYNC_GPIO_CLK_ENABLE();
  GPIO_InitStruct.Pin = LCD_HSYNC_PIN ;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  GPIO_InitStruct.Alternate = GPIO_AF11_LTDC;
  HAL_GPIO_Init(LCD_HSYNC_GPIO_PORT, &GPIO_InitStruct);

  LCD_VSYNC_GPIO_CLK_ENABLE();
  GPIO_InitStruct.Pin = LCD_VSYNC_PIN ;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  GPIO_InitStruct.Alternate = GPIO_AF11_LTDC;
  HAL_GPIO_Init(LCD_VSYNC_GPIO_PORT, &GPIO_InitStruct);

}

/**
  * @brief  Sets an LCD Layer visible without reloading.
  * @param  LayerIndex: Visible Layer
  * @param  State: New state of the specified layer
  *          This parameter can be one of the following values:
  *            @arg  ENABLE
  *            @arg  DISABLE
  * @retval None
  */
void BSP_LCD_SetLayerVisible_NoReload(uint32_t LayerIndex, FunctionalState State)
{
  if(State == ENABLE)
  {
    __HAL_LTDC_LAYER_ENABLE(&hltdc_disco, LayerIndex);
  }
  else
  {
    __HAL_LTDC_LAYER_DISABLE(&hltdc_disco, LayerIndex);
  }
  /* Do not set the Reload  */
}

/**
  * @brief  Configures the transparency.
  * @param  LayerIndex: Layer foreground or background.
  * @param  Transparency: Transparency
  *           This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF
  * @retval None
  */
void BSP_LCD_SetTransparency(uint32_t LayerIndex, uint8_t Transparency)
{
  HAL_LTDC_SetAlpha(&hltdc_disco, Transparency, LayerIndex);
}

/**
  * @brief  Configures the transparency without reloading.
  * @param  LayerIndex: Layer foreground or background.
  * @param  Transparency: Transparency
  *           This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF
  * @retval None
  */
void BSP_LCD_SetTransparency_NoReload(uint32_t LayerIndex, uint8_t Transparency)
{
  HAL_LTDC_SetAlpha_NoReload(&hltdc_disco, Transparency, LayerIndex);
}

/**
  * @brief  Sets an LCD layer frame buffer address.
  * @param  LayerIndex: Layer foreground or background
  * @param  Address: New LCD frame buffer value
  * @retval None
  */
void BSP_LCD_SetLayerAddress(uint32_t LayerIndex, uint32_t Address)
{
  HAL_LTDC_SetAddress(&hltdc_disco, Address, LayerIndex);
}

/**
  * @brief  Sets an LCD layer frame buffer address without reloading.
  * @param  LayerIndex: Layer foreground or background
  * @param  Address: New LCD frame buffer value
  * @retval None
  */
void BSP_LCD_SetLayerAddress_NoReload(uint32_t LayerIndex, uint32_t Address)
{
  HAL_LTDC_SetAddress_NoReload(&hltdc_disco, Address, LayerIndex);
}

/**
  * @brief  Sets display window.
  * @param  LayerIndex: Layer index
  * @param  Xpos: LCD X position
  * @param  Ypos: LCD Y position
  * @param  Width: LCD window width
  * @param  Height: LCD window height
  * @retval None
  */
void BSP_LCD_SetLayerWindow(uint16_t LayerIndex, uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height)
{
  /* Reconfigure the layer size */
  HAL_LTDC_SetWindowSize(&hltdc_disco, Width, Height, LayerIndex);

  /* Reconfigure the layer position */
  HAL_LTDC_SetWindowPosition(&hltdc_disco, Xpos, Ypos, LayerIndex);
}

/**
  * @brief  Sets display window without reloading.
  * @param  LayerIndex: Layer index
  * @param  Xpos: LCD X position
  * @param  Ypos: LCD Y position
  * @param  Width: LCD window width
  * @param  Height: LCD window height
  * @retval None
  */
void BSP_LCD_SetLayerWindow_NoReload(uint16_t LayerIndex, uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height)
{
  /* Reconfigure the layer size */
  HAL_LTDC_SetWindowSize_NoReload(&hltdc_disco, Width, Height, LayerIndex);

  /* Reconfigure the layer position */
  HAL_LTDC_SetWindowPosition_NoReload(&hltdc_disco, Xpos, Ypos, LayerIndex);
}

/**
  * @brief  Configures and sets the color keying.
  * @param  LayerIndex: Layer foreground or background
  * @param  RGBValue: Color reference
  * @retval None
  */
void BSP_LCD_SetColorKeying(uint32_t LayerIndex, uint32_t RGBValue)
{
  /* Configure and Enable the color Keying for LCD Layer */
  HAL_LTDC_ConfigColorKeying(&hltdc_disco, RGBValue, LayerIndex);
  HAL_LTDC_EnableColorKeying(&hltdc_disco, LayerIndex);
}

/**
  * @brief  Configures and sets the color keying without reloading.
  * @param  LayerIndex: Layer foreground or background
  * @param  RGBValue: Color reference
  * @retval None
  */
void BSP_LCD_SetColorKeying_NoReload(uint32_t LayerIndex, uint32_t RGBValue)
{
  /* Configure and Enable the color Keying for LCD Layer */
  HAL_LTDC_ConfigColorKeying_NoReload(&hltdc_disco, RGBValue, LayerIndex);
  HAL_LTDC_EnableColorKeying_NoReload(&hltdc_disco, LayerIndex);
}

/**
  * @brief  Disables the color keying.
  * @param  LayerIndex: Layer foreground or background
  * @retval None
  */
void BSP_LCD_ResetColorKeying(uint32_t LayerIndex)
{
  /* Disable the color Keying for LCD Layer */
  HAL_LTDC_DisableColorKeying(&hltdc_disco, LayerIndex);
}

/**
  * @brief  Disables the color keying without reloading.
  * @param  LayerIndex: Layer foreground or background
  * @retval None
  */
void BSP_LCD_ResetColorKeying_NoReload(uint32_t LayerIndex)
{
  /* Disable the color Keying for LCD Layer */
  HAL_LTDC_DisableColorKeying_NoReload(&hltdc_disco, LayerIndex);
}

/**
  * @brief  Disables the color keying without reloading.
  * @param  ReloadType: can be one of the following values
  *         - LCD_RELOAD_IMMEDIATE
  *         - LCD_RELOAD_VERTICAL_BLANKING
  * @retval None
  */
void BSP_LCD_Reload(uint32_t ReloadType)
{
  HAL_LTDC_Reload (&hltdc_disco, ReloadType);
}

/**
  * @brief  Sets the LCD text color.
  * @param  Color: Text color code ARGB(8-8-8-8)
  * @retval None
  */
void BSP_LCD_SetTextColor(uint32_t Color)
{
  DrawProp[ActiveLayer].TextColor = Color;
}

/**
  * @brief  Gets the LCD text color.
  * @retval Used text color.
  */
uint32_t BSP_LCD_GetTextColor(void)
{
  return DrawProp[ActiveLayer].TextColor;
}

/**
  * @brief  Sets the LCD background color.
  * @param  Color: Layer background color code ARGB(8-8-8-8)
  * @retval None
  */
void BSP_LCD_SetBackColor(uint32_t Color)
{
  DrawProp[ActiveLayer].BackColor = Color;
}

/**
  * @brief  Gets the LCD background color.
  * @retval Used background colour
  */
uint32_t BSP_LCD_GetBackColor(void)
{
  return DrawProp[ActiveLayer].BackColor;
}

/**
  * @brief  Sets the LCD text font.
  * @param  fonts: Layer font to be used
  * @retval None
  */
void BSP_LCD_SetFont(sFONT *fonts)
{
  DrawProp[ActiveLayer].pFont = fonts;
}

/**
  * @brief  Gets the LCD text font.
  * @retval Used layer font
  */
sFONT *BSP_LCD_GetFont(void)
{
  return DrawProp[ActiveLayer].pFont;
}

/**
  * @brief  Reads an LCD pixel.
  * @param  Xpos: X position
  * @param  Ypos: Y position
  * @retval RGB pixel color
  */
uint32_t BSP_LCD_ReadPixel(uint16_t Xpos, uint16_t Ypos)
{
  uint32_t ret = 0;

  if(hltdc_disco.LayerCfg[ActiveLayer].PixelFormat == LTDC_PIXEL_FORMAT_ARGB8888)
  {
    /* Read data value from SDRAM memory */
    ret = *(__IO uint32_t*) (hltdc_disco.LayerCfg[ActiveLayer].FBStartAdress + (2*(Ypos*BSP_LCD_GetXSize() + Xpos)));
  }
  else if(hltdc_disco.LayerCfg[ActiveLayer].PixelFormat == LTDC_PIXEL_FORMAT_RGB888)
  {
    /* Read data value from SDRAM memory */
    ret = (*(__IO uint32_t*) (hltdc_disco.LayerCfg[ActiveLayer].FBStartAdress + (2*(Ypos*BSP_LCD_GetXSize() + Xpos))) & 0x00FFFFFF);
  }
  else if((hltdc_disco.LayerCfg[ActiveLayer].PixelFormat == LTDC_PIXEL_FORMAT_RGB565) || \
          (hltdc_disco.LayerCfg[ActiveLayer].PixelFormat == LTDC_PIXEL_FORMAT_ARGB4444) || \
          (hltdc_disco.LayerCfg[ActiveLayer].PixelFormat == LTDC_PIXEL_FORMAT_AL88))
  {
    /* Read data value from SDRAM memory */
    ret = *(__IO uint16_t*) (hltdc_disco.LayerCfg[ActiveLayer].FBStartAdress + (2*(Ypos*BSP_LCD_GetXSize() + Xpos)));
  }
  else
  {
    /* Read data value from SDRAM memory */
    ret = *(__IO uint8_t*) (hltdc_disco.LayerCfg[ActiveLayer].FBStartAdress + (2*(Ypos*BSP_LCD_GetXSize() + Xpos)));
  }

  return ret;
}

/**
  * @brief  Clears the whole LCD.
  * @param  Color: Color of the background
  * @retval None
  */
void BSP_LCD_Clear(uint32_t Color)
{
  /* Clear the LCD */
  LL_FillBuffer(ActiveLayer, (uint32_t *)(hltdc_disco.LayerCfg[ActiveLayer].FBStartAdress), BSP_LCD_GetXSize(), BSP_LCD_GetYSize(), 0, Color);
}

/**
  * @brief  Clears the selected line.
  * @param  Line: Line to be cleared
  * @retval None
  */
void BSP_LCD_ClearStringLine(uint32_t Line)
{
  uint32_t color_backup = DrawProp[ActiveLayer].TextColor;
  DrawProp[ActiveLayer].TextColor = DrawProp[ActiveLayer].BackColor;

  /* Draw rectangle with background color */
  BSP_LCD_FillRect(0, (Line * DrawProp[ActiveLayer].pFont->Height), BSP_LCD_GetXSize(), DrawProp[ActiveLayer].pFont->Height);

  DrawProp[ActiveLayer].TextColor = color_backup;
  BSP_LCD_SetTextColor(DrawProp[ActiveLayer].TextColor);
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
  DrawChar(Xpos, Ypos, &DrawProp[ActiveLayer].pFont->table[(Ascii-' ') *\
    DrawProp[ActiveLayer].pFont->Height * ((DrawProp[ActiveLayer].pFont->Width + 7) / 8)]);
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
void BSP_LCD_DisplayStringAt(uint16_t Xpos, uint16_t Ypos, uint8_t *Text, Text_AlignModeTypdef Mode)
{
  uint16_t ref_column = 1, i = 0;
  uint32_t size = 0, xsize = 0;
  uint8_t  *ptr = Text;

  /* Get the text size */
  while (*ptr++) size ++ ;

  /* Characters number per line */
  xsize = (BSP_LCD_GetXSize()/DrawProp[ActiveLayer].pFont->Width);

  switch (Mode)
  {
  case CENTER_MODE:
    {
      ref_column = Xpos + ((xsize - size)* DrawProp[ActiveLayer].pFont->Width) / 2;
      break;
    }
  case LEFT_MODE:
    {
      ref_column = Xpos;
      break;
    }
  case RIGHT_MODE:
    {
      ref_column = - Xpos + ((xsize - size)*DrawProp[ActiveLayer].pFont->Width);
      break;
    }
  default:
    {
      ref_column = Xpos;
      break;
    }
  }

  /* Check that the Start column is located in the screen */
  if ((ref_column < 1) || (ref_column >= 0x8000))
  {
    ref_column = 1;
  }

  /* Send the string character by character on LCD */
  while ((*Text != 0) && (((BSP_LCD_GetXSize() - (i*DrawProp[ActiveLayer].pFont->Width)) & 0xFFFF) >= DrawProp[ActiveLayer].pFont->Width))
  {
    /* Display one character on LCD */
    BSP_LCD_DisplayChar(ref_column, Ypos, *Text);
    /* Decrement the column position by 16 */
    ref_column += DrawProp[ActiveLayer].pFont->Width;
    /* Point on the next character */
    Text++;
    i++;
  }
}

/**
  * @brief  Displays a maximum of 60 characters on the LCD.
  * @param  Line: Line where to display the character shape
  * @param  ptr: Pointer to string to display on LCD
  * @retval None
  */
void BSP_LCD_DisplayStringAtLine(uint16_t Line, uint8_t *ptr)
{
  BSP_LCD_DisplayStringAt(0, LINE(Line), ptr, LEFT_MODE);
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
  uint32_t  Xaddress = 0;

  /* Get the line address */
  if(hltdc_disco.LayerCfg[ActiveLayer].PixelFormat == LTDC_PIXEL_FORMAT_RGB565)
  { /* RGB565 format */
    Xaddress = (hltdc_disco.LayerCfg[ActiveLayer].FBStartAdress) + 2*(BSP_LCD_GetXSize()*Ypos + Xpos);
  }
  else
  { /* RGB888 format */
    Xaddress = (hltdc_disco.LayerCfg[ActiveLayer].FBStartAdress) + 3*(BSP_LCD_GetXSize()*Ypos + Xpos);
  }

  /* Write line */
  LL_FillBuffer(ActiveLayer, (uint32_t *)Xaddress, Length, 1, 0, DrawProp[ActiveLayer].TextColor);
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
  uint32_t  Xaddress = 0;

  /* Get the line address */
  if(hltdc_disco.LayerCfg[ActiveLayer].PixelFormat == LTDC_PIXEL_FORMAT_RGB565)
  { /* RGB565 format */
    Xaddress = (hltdc_disco.LayerCfg[ActiveLayer].FBStartAdress) + 2*(BSP_LCD_GetXSize()*Ypos + Xpos);
  }
  else
  { /* RGB888 format */
    Xaddress = (hltdc_disco.LayerCfg[ActiveLayer].FBStartAdress) + 3*(BSP_LCD_GetXSize()*Ypos + Xpos);
  }

  /* Write line */
  LL_FillBuffer(ActiveLayer, (uint32_t *)Xaddress, 1, Length, (BSP_LCD_GetXSize() - 1), DrawProp[ActiveLayer].TextColor);
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
  yinc1 = 0, yinc2 = 0, den = 0, num = 0, num_add = 0, num_pixels = 0,
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
    num_add = deltay;
    num_pixels = deltax;         /* There are more x-values than y-values */
  }
  else                          /* There is at least one y-value for every x-value */
  {
    xinc2 = 0;                  /* Don't change the x for every iteration */
    yinc1 = 0;                  /* Don't change the y when numerator >= denominator */
    den = deltay;
    num = deltay / 2;
    num_add = deltax;
    num_pixels = deltay;         /* There are more y-values than x-values */
  }

  for (curpixel = 0; curpixel <= num_pixels; curpixel++)
  {
    BSP_LCD_DrawPixel(x, y, DrawProp[ActiveLayer].TextColor);   /* Draw the current pixel */
    num += num_add;                            /* Increase the numerator by the top of the fraction */
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
  int32_t   decision;    /* Decision Variable */
  uint32_t  current_x;   /* Current X Value */
  uint32_t  current_y;   /* Current Y Value */

  decision = 3 - (Radius << 1);
  current_x = 0;
  current_y = Radius;

  while (current_x <= current_y)
  {
    BSP_LCD_DrawPixel((Xpos + current_x), (Ypos - current_y), DrawProp[ActiveLayer].TextColor);

    BSP_LCD_DrawPixel((Xpos - current_x), (Ypos - current_y), DrawProp[ActiveLayer].TextColor);

    BSP_LCD_DrawPixel((Xpos + current_y), (Ypos - current_x), DrawProp[ActiveLayer].TextColor);

    BSP_LCD_DrawPixel((Xpos - current_y), (Ypos - current_x), DrawProp[ActiveLayer].TextColor);

    BSP_LCD_DrawPixel((Xpos + current_x), (Ypos + current_y), DrawProp[ActiveLayer].TextColor);

    BSP_LCD_DrawPixel((Xpos - current_x), (Ypos + current_y), DrawProp[ActiveLayer].TextColor);

    BSP_LCD_DrawPixel((Xpos + current_y), (Ypos + current_x), DrawProp[ActiveLayer].TextColor);

    BSP_LCD_DrawPixel((Xpos - current_y), (Ypos + current_x), DrawProp[ActiveLayer].TextColor);

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

  if(PointCount < 2)
  {
    return;
  }

  BSP_LCD_DrawLine(Points->X, Points->Y, (Points+PointCount-1)->X, (Points+PointCount-1)->Y);

  while(--PointCount)
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
  int x = 0, y = -YRadius, err = 2-2*XRadius, e2;
  float k = 0, rad1 = 0, rad2 = 0;

  rad1 = XRadius;
  rad2 = YRadius;

  k = (float)(rad2/rad1);

  do {
    BSP_LCD_DrawPixel((Xpos-(uint16_t)(x/k)), (Ypos+y), DrawProp[ActiveLayer].TextColor);
    BSP_LCD_DrawPixel((Xpos+(uint16_t)(x/k)), (Ypos+y), DrawProp[ActiveLayer].TextColor);
    BSP_LCD_DrawPixel((Xpos+(uint16_t)(x/k)), (Ypos-y), DrawProp[ActiveLayer].TextColor);
    BSP_LCD_DrawPixel((Xpos-(uint16_t)(x/k)), (Ypos-y), DrawProp[ActiveLayer].TextColor);

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
  * @brief  Draws a pixel on LCD.
  * @param  Xpos: X position
  * @param  Ypos: Y position
  * @param  RGB_Code: Pixel color in ARGB mode (8-8-8-8)
  * @retval None
  */
void BSP_LCD_DrawPixel(uint16_t Xpos, uint16_t Ypos, uint32_t RGB_Code)
{
  /* Write data value to all SDRAM memory */
  if(hltdc_disco.LayerCfg[ActiveLayer].PixelFormat == LTDC_PIXEL_FORMAT_RGB565)
  { /* RGB565 format */
    *(__IO uint16_t*) (hltdc_disco.LayerCfg[ActiveLayer].FBStartAdress + (2*(Ypos*BSP_LCD_GetXSize() + Xpos))) = (uint16_t)RGB_Code;
  }
  else
  { /* ARGB8888 format */
    *(__IO uint32_t*) (hltdc_disco.LayerCfg[ActiveLayer].FBStartAdress + (3*(Ypos*BSP_LCD_GetXSize() + Xpos))) = RGB_Code;
  }
}

/**
  * @brief  Draws a bitmap picture loaded in the internal Flash in ARGB888 format (32 bits per pixel).
  * @param  Xpos: Bmp X position in the LCD
  * @param  Ypos: Bmp Y position in the LCD
  * @param  pbmp: Pointer to Bmp picture address in the internal Flash
  * @retval None
  */
void BSP_LCD_DrawBitmap(uint32_t Xpos, uint32_t Ypos, uint8_t *pbmp)
{
  uint32_t index = 0, width = 0, height = 0, bit_pixel = 0;
  uint32_t address;
  uint32_t input_color_mode = 0;

  /* Get bitmap data address offset */
  index = *(__IO uint16_t *) (pbmp + 10);
  index |= (*(__IO uint16_t *) (pbmp + 12)) << 16;

  /* Read bitmap width */
  width = *(uint16_t *) (pbmp + 18);
  width |= (*(uint16_t *) (pbmp + 20)) << 16;

  /* Read bitmap height */
  height = *(uint16_t *) (pbmp + 22);
  height |= (*(uint16_t *) (pbmp + 24)) << 16;

  /* Read bit/pixel */
  bit_pixel = *(uint16_t *) (pbmp + 28);

  /* Set the address */
  address = hltdc_disco.LayerCfg[ActiveLayer].FBStartAdress + (((BSP_LCD_GetXSize()*Ypos) + Xpos)*(4));

  /* Get the layer pixel format */
  if ((bit_pixel/8) == 4)
  {
    input_color_mode = DMA2D_INPUT_ARGB8888;
  }
  else if ((bit_pixel/8) == 2)
  {
    input_color_mode = DMA2D_INPUT_RGB565;
  }
  else
  {
    input_color_mode = DMA2D_INPUT_RGB888;
  }

  /* Bypass the bitmap header */
  pbmp += (index + (width * (height - 1) * (bit_pixel/8)));

  /* Convert picture to ARGB8888 pixel format */
  for(index=0; index < height; index++)
  {
    /* Pixel format conversion */
    LL_ConvertLineToARGB8888((uint32_t *)pbmp, (uint32_t *)address, width, input_color_mode);

    /* Increment the source and destination buffers */
    address+=  (BSP_LCD_GetXSize()*4);
    pbmp -= width*(bit_pixel/8);
  }
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
  uint32_t  x_address = 0;

  /* Set the text color */
  BSP_LCD_SetTextColor(DrawProp[ActiveLayer].TextColor);

  /* Get the rectangle start address */
  if(hltdc_disco.LayerCfg[ActiveLayer].PixelFormat == LTDC_PIXEL_FORMAT_RGB565)
  { /* RGB565 format */
    x_address = (hltdc_disco.LayerCfg[ActiveLayer].FBStartAdress) + 2*(BSP_LCD_GetXSize()*Ypos + Xpos);
  }
  else
  { /* RGB888 format */
    x_address = (hltdc_disco.LayerCfg[ActiveLayer].FBStartAdress) + 3*(BSP_LCD_GetXSize()*Ypos + Xpos);
  }
  /* Fill the rectangle */
  LL_FillBuffer(ActiveLayer, (uint32_t *)x_address, Width, Height, (BSP_LCD_GetXSize() - Width), DrawProp[ActiveLayer].TextColor);
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
  int32_t  decision;     /* Decision Variable */
  uint32_t  current_x;   /* Current X Value */
  uint32_t  current_y;   /* Current Y Value */

  decision = 3 - (Radius << 1);

  current_x = 0;
  current_y = Radius;

  BSP_LCD_SetTextColor(DrawProp[ActiveLayer].TextColor);

  while (current_x <= current_y)
  {
    if(current_y > 0)
    {
      BSP_LCD_DrawHLine(Xpos - current_y, Ypos + current_x, 2*current_y);
      BSP_LCD_DrawHLine(Xpos - current_y, Ypos - current_x, 2*current_y);
    }

    if(current_x > 0)
    {
      BSP_LCD_DrawHLine(Xpos - current_x, Ypos - current_y, 2*current_x);
      BSP_LCD_DrawHLine(Xpos - current_x, Ypos + current_y, 2*current_x);
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

  BSP_LCD_SetTextColor(DrawProp[ActiveLayer].TextColor);
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
  uint16_t  image_left = 0, image_right = 0, image_top = 0, image_bottom = 0;

  image_left = image_right = Points->X;
  image_top= image_bottom = Points->Y;

  for(counter = 1; counter < PointCount; counter++)
  {
    pixelX = POLY_X(counter);
    if(pixelX < image_left)
    {
      image_left = pixelX;
    }
    if(pixelX > image_right)
    {
      image_right = pixelX;
    }

    pixelY = POLY_Y(counter);
    if(pixelY < image_top)
    {
      image_top = pixelY;
    }
    if(pixelY > image_bottom)
    {
      image_bottom = pixelY;
    }
  }

  if(PointCount < 2)
  {
    return;
  }

  X_center = (image_left + image_right)/2;
  Y_center = (image_bottom + image_top)/2;

  X_first = Points->X;
  Y_first = Points->Y;

  while(--PointCount)
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
  int x = 0, y = -YRadius, err = 2-2*XRadius, e2;
  float k = 0, rad1 = 0, rad2 = 0;

  rad1 = XRadius;
  rad2 = YRadius;

  k = (float)(rad2/rad1);

  do
  {
    BSP_LCD_DrawHLine((Xpos-(uint16_t)(x/k)), (Ypos+y), (2*(uint16_t)(x/k) + 1));
    BSP_LCD_DrawHLine((Xpos-(uint16_t)(x/k)), (Ypos-y), (2*(uint16_t)(x/k) + 1));

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
  /* Display On */

  /* Display Enable */
  HAL_GPIO_WritePin(LCD_DISP_GPIO_PORT, LCD_DISP_PIN, GPIO_PIN_SET);

  /* Data Enable */
  HAL_GPIO_WritePin(LCD_DE_GPIO_PORT, LCD_DE_PIN, GPIO_PIN_SET);

  /* Backlight Enable */
  HAL_GPIO_WritePin(LCD_BL_CTRL_GPIO_PORT, LCD_BL_CTRL_PIN, GPIO_PIN_SET);
}

/**
  * @brief  Disables the display.
  * @retval None
  */
void BSP_LCD_DisplayOff(void)
{
  /* Display Off */
  HAL_GPIO_WritePin(LCD_DISP_GPIO_PORT, LCD_DISP_PIN, GPIO_PIN_RESET);

  /* Data Off */
  HAL_GPIO_WritePin(LCD_DE_GPIO_PORT, LCD_DE_PIN, GPIO_PIN_RESET);

  /* Backlight Off */
  HAL_GPIO_WritePin(LCD_BL_CTRL_GPIO_PORT, LCD_BL_CTRL_PIN, GPIO_PIN_RESET);
}

/**
  * @brief  Initializes the LTDC MSP.
  * @param  hltdc: LTDC handle
  * @param  Params
  * @retval None
  */
__weak void BSP_LCD_MspInit(LTDC_HandleTypeDef *hltdc, void *Params)
{
    /* LTDC Peripheral clock enable */
  __HAL_RCC_LTDC_CLK_ENABLE();

  /* Configure the NVIC for LTDC */
  HAL_NVIC_SetPriority(LTDC_IRQn,0,0);

  /* Enable the LTDC global Interrupt */
  HAL_NVIC_EnableIRQ(LTDC_IRQn);

  /* DMA2D Peripheral clock enable */
  __HAL_RCC_DMA2D_CLK_ENABLE();


}

/**
  * @brief  DeInitializes BSP_LCD_ MSP.
  * @param  hltdc: LTDC handle
  * @param  Params
  * @retval None
  */
__weak void BSP_LCD_MspDeInit(LTDC_HandleTypeDef *hltdc, void *Params)
{
   /* Disable the LTDC global Interrupt */
  HAL_NVIC_DisableIRQ(LTDC_IRQn);

  /* LTDC Peripheral clock disable */
  __HAL_RCC_LTDC_CLK_DISABLE();
}


/**
  * @brief  Initializes the RTC MSP.
  * @param  hrtc: RTC handle
  * @param  Params
  * @retval None
  */
__weak uint8_t BSP_RTC_MspInit(RTC_HandleTypeDef *hrtc, void *Params)
{
   GPIO_InitTypeDef GPIO_InitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_PeriphCLKInitTypeDef  PeriphClkInitStruct;

  /* GPIO Configuration */
  /* PB2: RTC_OUT */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  GPIO_InitStruct.Alternate = GPIO_AF0_RTC_50Hz;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*##-1- Configure LSE as RTC clock source ###################################*/
  HAL_RCC_GetOscConfig(&RCC_OscInitStruct);

  RCC_OscInitStruct.OscillatorType =  RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.LSIState = RCC_LSI_OFF;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    return LCD_ERROR;
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    return LCD_ERROR;
  }

  /* Configures the External Low Speed oscillator (LSE) drive capability */
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /*##-2- Enable RTC peripheral Clocks #######################################*/

  /* Enable RTC Clock */
  __HAL_RCC_RTC_ENABLE();

  return LCD_OK;

}

/**
  * @brief  DeInitializes BSP_RTC_ MSP.
  * @param  hrtc: RTC handle
  * @param  Params
  * @retval None
  */
__weak void BSP_RTC_MspDeInit(RTC_HandleTypeDef *hrtc, void *Params)
{
  /*##-1- Reset peripherals ##################################################*/
  __HAL_RCC_RTC_DISABLE();
}


/**
  * @brief  Clock Config.
  * @param  hltdc: LTDC handle
  * @param  Params
  * @note   This API is called by BSP_LCD_Init()
  *         Being __weak it can be overwritten by the application
  * @retval None
  */
__weak uint8_t BSP_LCD_ClockConfig(void)
{
  RCC_PeriphCLKInitTypeDef  PeriphClkInit;

  /* LCD clock configuration */
  /* PLLSAI2_VCO Input = MSI_VALUE/PLLSAI2_M = 8 /2 = 4 Mhz */
  /* PLLSAI2_VCO Output = PLLSAI2_VCO Input * PLLSAI2N =  4 * 40 = 160 Mhz */
  /* PLLLCDCLK = PLLSAI2_VCO Output/PLLSAIR = 160/6 = 26.6 Mhz */
  /* LTDC clock frequency = PLLLCDCLK / LTDC_PLLSAI_DIVR_4 = 26.6/16 =1.67Mhz */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
  PeriphClkInit.PLLSAI2.PLLSAI2Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI2.PLLSAI2M = 2;
  PeriphClkInit.PLLSAI2.PLLSAI2N = 40;
  PeriphClkInit.PLLSAI2.PLLSAI2R = 6;
  PeriphClkInit.PLLSAI2.PLLSAI2ClockOut = RCC_PLLSAI2_LTDCCLK;
  PeriphClkInit.LtdcClockSelection = RCC_LTDCCLKSOURCE_PLLSAI2_DIV16;
  if(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    return LCD_ERROR;
  }
  else
  {
    return LCD_OK;
  }

}



__weak void BSP_LCD_LineEventCallback(LTDC_HandleTypeDef *hltdc)
{
  GPIO_InitTypeDef GPIO_InitStruct;

    /* Pixels CLK */
  LCD_PIXELS_CLOCK_GPIO_CLK_ENABLE();
  GPIO_InitStruct.Pin = LCD_PIXELS_CLOCK_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  GPIO_InitStruct.Alternate = GPIO_AF11_LTDC;
  HAL_GPIO_Init(LCD_PIXELS_CLOCK_GPIO_PORT, &GPIO_InitStruct);

  /* RGB */
  LCD_RGB_GPIO_CLK_ENABLE();
  GPIO_InitStruct.Pin = LCD_R6_PIN | LCD_R7_PIN | LCD_G6_PIN  | LCD_G7_PIN | LCD_B6_PIN  | LCD_B7_PIN  ;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  GPIO_InitStruct.Alternate = GPIO_AF11_LTDC;
  HAL_GPIO_Init(LCD_RGB_GPIO_PORT, &GPIO_InitStruct);


    /* Set HSYNC and VSYNC IOs */
  /* HSYNC */
  LCD_HSYNC_GPIO_CLK_ENABLE();
  GPIO_InitStruct.Pin = LCD_HSYNC_PIN ;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  GPIO_InitStruct.Alternate = GPIO_AF11_LTDC;
  HAL_GPIO_Init(LCD_HSYNC_GPIO_PORT, &GPIO_InitStruct);
  /* VSYNC */
  LCD_VSYNC_GPIO_CLK_ENABLE();
  GPIO_InitStruct.Pin = LCD_VSYNC_PIN ;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  GPIO_InitStruct.Alternate = GPIO_AF11_LTDC;
  HAL_GPIO_Init(LCD_VSYNC_GPIO_PORT, &GPIO_InitStruct);


  /* Disable the LTDC */
  __HAL_LTDC_DISABLE(hltdc);
}



/*******************************************************************************
                       LTDC and DMA2D BSP Routines
*******************************************************************************/
/**
  * @brief  Handles LTDC interrupt request.
  * @note : Can be surcharged by application code implementation of the function.
  */
__weak void BSP_LCD_LTDC_IRQHandler(void)
{
  HAL_LTDC_IRQHandler(&(hltdc_disco));
}

/**
  * @brief  This function handles LTDC Error interrupt Handler.
  * @note : Can be surcharged by application code implementation of the function.
  */

__weak void BSP_LCD_LTDC_ER_IRQHandler(void)
{
  HAL_LTDC_IRQHandler(&(hltdc_disco));
}

/**
  * @brief  Handles DMA2D interrupt request.
  * @note : Can be surcharged by application code implementation of the function.
  */
__weak void BSP_LCD_DMA2D_IRQHandler(void)
{
  HAL_DMA2D_IRQHandler(&hdma2d_disco);
}

/**
  * @}
  */

/** @defgroup STM32L4P5G_DISCOVERY_RGB_LCD_Private_Functions Private Functions
  * @{
  */

/*******************************************************************************
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
  uint8_t  offset;
  uint8_t  *pchar;
  uint32_t line;

  height = DrawProp[ActiveLayer].pFont->Height;
  width  = DrawProp[ActiveLayer].pFont->Width;

  offset =  8 *((width + 7)/8) -  width ;

  for(i = 0; i < height; i++)
  {
    pchar = ((uint8_t *)c + (width + 7)/8 * i);

    switch(((width + 7)/8))
    {

    case 1:
      line =  pchar[0];
      break;

    case 2:
      line =  (pchar[0]<< 8) | pchar[1];
      break;

    case 3:
    default:
      line =  (pchar[0]<< 16) | (pchar[1]<< 8) | pchar[2];
      break;
    }

    for (j = 0; j < width; j++)
    {
      if ((line & (1 << (width - j + offset - 1))) != 0)
      {
        BSP_LCD_DrawPixel((Xpos + j), Ypos, DrawProp[ActiveLayer].TextColor);
      }
      else
      {
        BSP_LCD_DrawPixel((Xpos + j), Ypos, DrawProp[ActiveLayer].BackColor);
      }
    }
    Ypos++;
  }
}

/**
  * @brief  Fills a triangle (between 3 points).
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
  yinc1 = 0, yinc2 = 0, den = 0, num = 0, num_add = 0, num_pixels = 0,
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
    num_add = deltay;
    num_pixels = deltax;         /* There are more x-values than y-values */
  }
  else                          /* There is at least one y-value for every x-value */
  {
    xinc2 = 0;                  /* Don't change the x for every iteration */
    yinc1 = 0;                  /* Don't change the y when numerator >= denominator */
    den = deltay;
    num = deltay / 2;
    num_add = deltax;
    num_pixels = deltay;         /* There are more y-values than x-values */
  }

  for (curpixel = 0; curpixel <= num_pixels; curpixel++)
  {
    BSP_LCD_DrawLine(x, y, x3, y3);

    num += num_add;              /* Increase the numerator by the top of the fraction */
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
  * @brief  Fills a buffer.
  * @param  LayerIndex: Layer index
  * @param  pDst: Pointer to destination buffer
  * @param  xSize: Buffer width
  * @param  ySize: Buffer height
  * @param  OffLine: Offset
  * @param  ColorIndex: Color index
  * @retval None
  */
static void LL_FillBuffer(uint32_t LayerIndex, void *pDst, uint32_t xSize, uint32_t ySize, uint32_t OffLine, uint32_t ColorIndex)
{
  /* Register to memory mode with ARGB8888 as color Mode */
  hdma2d_disco.Init.Mode         = DMA2D_R2M;
  if(hltdc_disco.LayerCfg[ActiveLayer].PixelFormat == LTDC_PIXEL_FORMAT_RGB565)
  { /* RGB565 format */
    hdma2d_disco.Init.ColorMode    = DMA2D_OUTPUT_RGB565;
  }
  else
  { /* ARGB8888 format */
    hdma2d_disco.Init.ColorMode    = DMA2D_OUTPUT_RGB888;
  }
  hdma2d_disco.Init.OutputOffset = OffLine;

  hdma2d_disco.Instance = DMA2D;

  /* DMA2D Initialization */
  if(HAL_DMA2D_Init(&hdma2d_disco) == HAL_OK)
  {
    if(HAL_DMA2D_ConfigLayer(&hdma2d_disco, LayerIndex) == HAL_OK)
    {
      if (HAL_DMA2D_Start(&hdma2d_disco, ColorIndex, (uint32_t)pDst, xSize, ySize) == HAL_OK)
      {
        /* Polling For DMA transfer */
        HAL_DMA2D_PollForTransfer(&hdma2d_disco, 10);
      }
    }
  }
}

/**
  * @brief  Converts a line to an ARGB8888 pixel format.
  * @param  pSrc: Pointer to source buffer
  * @param  pDst: Output color
  * @param  xSize: Buffer width
  * @param  ColorMode: Input color mode
  * @retval None
  */
static void LL_ConvertLineToARGB8888(void *pSrc, void *pDst, uint32_t xSize, uint32_t ColorMode)
{
  /* Configure the DMA2D Mode, Color Mode and output offset */
  hdma2d_disco.Init.Mode         = DMA2D_M2M_PFC;
  hdma2d_disco.Init.ColorMode    = DMA2D_OUTPUT_ARGB8888;
  hdma2d_disco.Init.OutputOffset = 0;

  /* Foreground Configuration */
  hdma2d_disco.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d_disco.LayerCfg[1].InputAlpha = 0xFF;
  hdma2d_disco.LayerCfg[1].InputColorMode = ColorMode;
  hdma2d_disco.LayerCfg[1].InputOffset = 0;

  hdma2d_disco.Instance = DMA2D;

  /* DMA2D Initialization */
  if(HAL_DMA2D_Init(&hdma2d_disco) == HAL_OK)
  {
    if(HAL_DMA2D_ConfigLayer(&hdma2d_disco, 1) == HAL_OK)
    {
      if (HAL_DMA2D_Start(&hdma2d_disco, (uint32_t)pSrc, (uint32_t)pDst, xSize, 1) == HAL_OK)
      {
        /* Polling For DMA transfer */
        HAL_DMA2D_PollForTransfer(&hdma2d_disco, 10);
      }
    }
  }
}

/**
  * @brief  Configure the RTC peripheral by selecting the clock source.
  * @param  None
  * @retval None
  */
uint8_t RTC_Config(void)
{
  /*##-1- Configure the RTC peripheral #######################################*/
  /* Configure RTC prescaler and RTC data registers */
  /* RTC configured as follows:
      - Hour Format    = Format 24
      - Asynch Prediv  = Value according to source clock
      - Synch Prediv   = Value according to source clock
      - OutPut         = Output Disable
      - OutPutRemap    = Enable to get CALIB on PB2
      - OutPutPolarity = High Polarity
      - OutPutType     = Open Drain */
  /* Enable the Calibration Output */

  RtcHandle_disco.Instance = RTC;
  RtcHandle_disco.Init.HourFormat = RTC_HOURFORMAT_24;
  RtcHandle_disco.Init.AsynchPrediv = RTC_ASYNCH_PREDIV; /* 0x7F */
  RtcHandle_disco.Init.SynchPrediv = RTC_SYNCH_PREDIV; /* (32768 kHz / 128) - 1 = 0xFF */
  RtcHandle_disco.Init.OutPut = RTC_OUTPUT_DISABLE; // As CALIB has to be output
  RtcHandle_disco.Init.OutPutRemap = RTC_OUTPUT_REMAP_POS1 ; // Remap set to PB2 (but need to use PC13 in final application)
  RtcHandle_disco.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  RtcHandle_disco.Init.OutPutType = RTC_OUTPUT_TYPE_PUSHPULL;

  if(HAL_RTC_GetState(&RtcHandle_disco) == HAL_RTC_STATE_RESET)
  {
    /* Initialize the RTC Msp: this __weak function can be rewritten by the application */
    if (BSP_RTC_MspInit(&RtcHandle_disco, NULL) != LCD_OK)
    {
      return LCD_ERROR;
    }
  }
  if(HAL_RTC_Init(&RtcHandle_disco) != HAL_OK)
  {
    /* Initialization Error */
    return LCD_ERROR;
  }

  /* Ultra low power calibration  */
  if (HAL_RTCEx_SetLowPowerCalib(&RtcHandle_disco, RTC_LPCAL_SET) != HAL_OK)
  {
    /* Initialization Error */
    return LCD_ERROR;
  }

  /* Enable the calibration output */
  /* 1Hz = Backlight OFF */
  //HAL_RTCEx_SetCalibrationOutPut(&RtcHandle, RTC_CALIBOUTPUT_1HZ);
  HAL_RTCEx_SetCalibrationOutPut(&RtcHandle_disco, RTC_CALIBOUTPUT_512HZ);

  return LCD_OK;

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



