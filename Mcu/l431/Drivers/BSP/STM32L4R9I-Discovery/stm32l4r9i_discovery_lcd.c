/**
  ******************************************************************************
  * @file    stm32l4r9i_discovery_lcd.c
  * @author  MCD Application Team
  * @brief   This file includes the driver for DSI Liquid Crystal Display (LCD)
  *          module mounted on STM32L4R9I_DISCOVERY board.
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

/* File Info: ------------------------------------------------------------------
                                   User NOTES
1. How To use this driver:
--------------------------
   - This driver is used to drive directly in command mode a LCD TFT using the
     DSI interface.
     The following IPs are implied : DSI Host IP block working
     in conjunction to the LTDC controller.
   - This driver is linked by construction to LCD.

2. Driver description:
----------------------
  + Initialization steps:
     o Initialize the LCD using the BSP_LCD_Init() function.
       Please note that LCD display will be turned on at the end of this function.
     o De-inietialize the LCD using BSP_LCD_DeInit() function.

  + Options
     o Modify in the fly the display properties using the following functions:
       - BSP_LCD_SetTransparency().
       - BSP_LCD_SetColorKeying().
       - BSP_LCD_ResetColorKeying().
       - BSP_LCD_SetFont().
       - BSP_LCD_SetBackColor().
       - BSP_LCD_SetTextColor().
       - BSP_LCD_SetBrightness().
     o You can set display on/off using following functions:
       - BSP_LCD_DisplayOn().
       - BSP_LCD_DisplayOff().

  + Display on LCD
     o First, check that frame buffer is available using BSP_LCD_IsFrameBufferAvailable().
     o When frame buffer is available, modify it using following functions:
       - BSP_LCD_Clear().
       - BSP_LCD_ClearStringLine().
       - BSP_LCD_DisplayChar().
       - BSP_LCD_DisplayStringAt().
       - BSP_LCD_DisplayStringAtLine().
       - BSP_LCD_DrawBitmap().
       - BSP_LCD_DrawCircle().
       - BSP_LCD_DrawEllipse().
       - ....

     o Call BSP_LCD_Refresh() to refresh LCD display.

------------------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "stm32l4r9i_discovery_lcd.h"
#include "stm32l4r9i_discovery_gfxmmu_lut.h"
#include "stm32l4r9i_discovery_io.h"
#include "../../../Utilities/Fonts/fonts.h"
#include "../../../Utilities/Fonts/font24.c"
#include "../../../Utilities/Fonts/font20.c"
#include "../../../Utilities/Fonts/font16.c"
#include "../../../Utilities/Fonts/font12.c"
#include "../../../Utilities/Fonts/font8.c"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L4R9I_DISCOVERY
  * @{
  */

/** @defgroup STM32L4R9I_DISCOVERY_LCD STM32L4R9I_DISCOVERY LCD
  * @{
  */

/** @defgroup STM32L4R9I_DISCOVERY_LCD_Private_Macros Private Macros
  * @{
  */
#define ABS(X)                 ((X) > 0 ? (X) : -(X))

#define POLY_X(Z)              ((int32_t)((Points + (Z))->X))
#define POLY_Y(Z)              ((int32_t)((Points + (Z))->Y))
/**
  * @}
  */

/** @defgroup STM32L4R9I_DISCOVERY_LCD_Exported_Variables Exported Variables
  * @{
  */
/* DMA2D handle */
DMA2D_HandleTypeDef  hdma2d_discovery;

/**
  * @}
  */

/** @defgroup STM32L4R9I_DISCOVERY_LCD_Private_Variables Private Variables
  * @{
  */
/* LCD/PSRAM initialization status sharing the same power source */
extern uint32_t bsp_lcd_initialized;
extern uint32_t bsp_psram_initialized;

/* Flag to indicate if HSE has to be disabled during de-initialization */
static uint32_t bsp_lcd_hse_to_disable = 0;
/* Default Active LTDC Layer in which drawing is made is LTDC Layer Background */
static uint32_t  ActiveLayer = LTDC_ACTIVE_LAYER_BACKGROUND;
/* Current Drawing Layer properties variable */
static LCD_DrawPropTypeDef DrawProp[LTDC_MAX_LAYER_NUMBER];

/* Physical frame buffer for background and foreground layers */
/* 390*390 pixels with 32bpp - 20% */
#if defined ( __ICCARM__ )  /* IAR Compiler */
  #pragma data_alignment = 16
uint32_t              PhysFrameBuffer[121680];
#elif defined (__GNUC__)    /* GNU Compiler */
uint32_t              PhysFrameBuffer[121680] __attribute__ ((aligned (16)));
#else                       /* ARM Compiler */
__align(16) uint32_t  PhysFrameBuffer[121680];
#endif

/* Global variable used to know if frame buffer is available (1) or not because refresh is on going (0) */
__IO uint32_t FrameBufferAvailable = 1;
/* LCD size */
uint32_t lcd_x_size = 390;
uint32_t lcd_y_size = 390;
/* GFXMMU, LTDC and DSI handles */
GFXMMU_HandleTypeDef hgfxmmu_discovery;
LTDC_HandleTypeDef   hltdc_discovery;
DSI_HandleTypeDef    hdsi_discovery;
/**
  * @}
  */

/** @defgroup STM32L4R9I_DISCOVERY_LCD_Private_FunctionPrototypes Private FunctionPrototypes
  * @{
  */
static void LCD_PowerOn(void);
static void LCD_PowerOff(void);
static void DrawChar(uint16_t Xpos, uint16_t Ypos, const uint8_t *c);
static void FillTriangle(uint16_t x1, uint16_t x2, uint16_t x3, uint16_t y1, uint16_t y2, uint16_t y3);
static void LL_FillBuffer(uint32_t LayerIndex, void *pDst, uint32_t xSize, uint32_t ySize, uint32_t OffLine, uint32_t ColorIndex);
static void LL_ConvertLineToARGB8888(void * pSrc, void *pDst, uint32_t xSize, uint32_t ColorMode);
/**
  * @}
  */

/** @defgroup STM32L4R9I_DISCOVERY_LCD_Exported_Functions Exported Functions
  * @{
  */

/**
  * @brief  Initialize the DSI LCD.
  * @note   The initialization is done as below:
  *     - GFXMMU initialization
  *     - DSI PLL initialization
  *     - DSI initialization
  *     - LTDC initialization
  *     - RM67162 LCD Display IC Driver initialization
  * @retval LCD state
  */
uint8_t BSP_LCD_Init(void)
{
  if(bsp_lcd_initialized == 0)
  {
    LTDC_LayerCfgTypeDef    LayerCfg;
    DSI_PLLInitTypeDef      dsiPllInit;
    DSI_PHY_TimerTypeDef    PhyTimings;
    DSI_HOST_TimeoutTypeDef HostTimeouts;
    DSI_LPCmdTypeDef        LPCmd;
    DSI_CmdCfgTypeDef       CmdCfg;

    /* Power on LCD */
    LCD_PowerOn();

    /* Call first MSP Initialize
     * This will set IP blocks LTDC, DSI and DMA2D
     * - out of reset
     * - clocked
     * - NVIC IRQ related to IP blocks enabled
    */
    BSP_LCD_MspInit();

    /************************/
    /* GFXMMU CONFIGURATION */
    /************************/
    hgfxmmu_discovery.Instance = GFXMMU;
    __HAL_GFXMMU_RESET_HANDLE_STATE(&hgfxmmu_discovery);
    hgfxmmu_discovery.Init.BlocksPerLine                     = GFXMMU_192BLOCKS;
    hgfxmmu_discovery.Init.DefaultValue                      = 0xFFFFFFFF;
    hgfxmmu_discovery.Init.Buffers.Buf0Address               = (uint32_t) PhysFrameBuffer;
    hgfxmmu_discovery.Init.Buffers.Buf1Address               = 0; /* NU */
    hgfxmmu_discovery.Init.Buffers.Buf2Address               = 0; /* NU */
    hgfxmmu_discovery.Init.Buffers.Buf3Address               = 0; /* NU */
    hgfxmmu_discovery.Init.Interrupts.Activation             = DISABLE;
    hgfxmmu_discovery.Init.Interrupts.UsedInterrupts         = GFXMMU_AHB_MASTER_ERROR_IT; /* NU */
    if(HAL_OK != HAL_GFXMMU_Init(&hgfxmmu_discovery))
    {
      return(LCD_ERROR);
    }

    /* Initialize LUT */
    if(HAL_OK != HAL_GFXMMU_ConfigLut(&hgfxmmu_discovery, 0, 390, (uint32_t) gfxmmu_lut_config_argb8888))
    {
      return(LCD_ERROR);
    }
    /* Disable non visible lines : from line 390 to 1023 */
    if(HAL_OK != HAL_GFXMMU_DisableLutLines(&hgfxmmu_discovery, 390, 634))
    {
      return(LCD_ERROR);
    }

    /**********************/
    /* LTDC CONFIGURATION */
    /**********************/

    /* LTDC initialization */
    hltdc_discovery.Instance = LTDC;
    __HAL_LTDC_RESET_HANDLE_STATE(&hltdc_discovery);
    hltdc_discovery.Init.HSPolarity         = LTDC_HSPOLARITY_AL;
    hltdc_discovery.Init.VSPolarity         = LTDC_VSPOLARITY_AL;
    hltdc_discovery.Init.DEPolarity         = LTDC_DEPOLARITY_AL;
    hltdc_discovery.Init.PCPolarity         = LTDC_PCPOLARITY_IPC;
    hltdc_discovery.Init.HorizontalSync     = 0;   /* HSYNC width - 1 */
    hltdc_discovery.Init.VerticalSync       = 0;   /* VSYNC width - 1 */
    hltdc_discovery.Init.AccumulatedHBP     = 1;   /* HSYNC width + HBP - 1 */
    hltdc_discovery.Init.AccumulatedVBP     = 1;   /* VSYNC width + VBP - 1 */
    hltdc_discovery.Init.AccumulatedActiveW = 391; /* HSYNC width + HBP + Active width - 1 */
    hltdc_discovery.Init.AccumulatedActiveH = 391; /* VSYNC width + VBP + Active height - 1 */
    hltdc_discovery.Init.TotalWidth         = 392; /* HSYNC width + HBP + Active width + HFP - 1 */
    hltdc_discovery.Init.TotalHeigh         = 392; /* VSYNC width + VBP + Active height + VFP - 1 */
    hltdc_discovery.Init.Backcolor.Red      = 255;
    hltdc_discovery.Init.Backcolor.Green    = 255;
    hltdc_discovery.Init.Backcolor.Blue     = 0;
    hltdc_discovery.Init.Backcolor.Reserved = 0xFF;
    if(HAL_LTDC_Init(&hltdc_discovery) != HAL_OK)
    {
      return(LCD_ERROR);
    }

    /* LTDC layers configuration */
    LayerCfg.WindowX0        = 0;
    LayerCfg.WindowX1        = 390;
    LayerCfg.WindowY0        = 0;
    LayerCfg.WindowY1        = 390;
    LayerCfg.PixelFormat     = LTDC_PIXEL_FORMAT_ARGB8888;
    LayerCfg.Alpha           = 0xFF; /* NU default value */
    LayerCfg.Alpha0          = 0; /* NU default value */
    LayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA; /* NU default value */
    LayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA; /* NU default value */
    LayerCfg.FBStartAdress   = GFXMMU_VIRTUAL_BUFFER0_BASE;
    LayerCfg.ImageWidth      = 768; /* virtual frame buffer contains 768 pixels per line for 32bpp */
    LayerCfg.ImageHeight     = 390;
    LayerCfg.Backcolor.Red   = 0; /* NU default value */
    LayerCfg.Backcolor.Green = 0; /* NU default value */
    LayerCfg.Backcolor.Blue  = 0; /* NU default value */
    LayerCfg.Backcolor.Reserved = 0xFF;
    if(HAL_LTDC_ConfigLayer(&hltdc_discovery, &LayerCfg, 0) != HAL_OK)
    {
      return(LCD_ERROR);
    }

    DrawProp[0].BackColor = LCD_COLOR_WHITE;
    DrawProp[0].pFont     = &Font24;
    DrawProp[0].TextColor = LCD_COLOR_BLACK;

    /*********************/
    /* DSI CONFIGURATION */
    /*********************/

    /* DSI initialization */
    hdsi_discovery.Instance = DSI;
    __HAL_DSI_RESET_HANDLE_STATE(&hdsi_discovery);
    hdsi_discovery.Init.AutomaticClockLaneControl = DSI_AUTO_CLK_LANE_CTRL_DISABLE;
    /* We have 1 data lane at 500Mbps => lane byte clock at 500/8 = 62,5 MHZ */
    /* We want TX escape clock at around 20MHz and under 20MHz so clock division is set to 4 */
    hdsi_discovery.Init.TXEscapeCkdiv             = 4;
    hdsi_discovery.Init.NumberOfLanes             = DSI_ONE_DATA_LANE;
    /* We have HSE value at 16 Mhz and we want data lane at 500Mbps */
    dsiPllInit.PLLNDIV = 125;
    dsiPllInit.PLLIDF  = DSI_PLL_IN_DIV4;
    dsiPllInit.PLLODF  = DSI_PLL_OUT_DIV1;
    if(HAL_DSI_Init(&hdsi_discovery, &dsiPllInit) != HAL_OK)
    {
      return(LCD_ERROR);
    }

    PhyTimings.ClockLaneHS2LPTime  = 33; /* Tclk-post + Tclk-trail + Ths-exit = [(60ns + 52xUI) + (60ns) + (300ns)]/16ns */
    PhyTimings.ClockLaneLP2HSTime  = 30; /* Tlpx + (Tclk-prepare + Tclk-zero) + Tclk-pre = [150ns + 300ns + 8xUI]/16ns */
    PhyTimings.DataLaneHS2LPTime   = 11; /* Ths-trail + Ths-exit = [(60ns + 4xUI) + 100ns]/16ns */
    PhyTimings.DataLaneLP2HSTime   = 21; /* Tlpx + (Ths-prepare + Ths-zero) + Ths-sync = [150ns + (145ns + 10xUI) + 8xUI]/16ns */
    PhyTimings.DataLaneMaxReadTime = 0;
    PhyTimings.StopWaitTime        = 7;
    if(HAL_DSI_ConfigPhyTimer(&hdsi_discovery, &PhyTimings) != HAL_OK)
    {
      return(LCD_ERROR);
    }

    HostTimeouts.TimeoutCkdiv                 = 1;
    HostTimeouts.HighSpeedTransmissionTimeout = 0;
    HostTimeouts.LowPowerReceptionTimeout     = 0;
    HostTimeouts.HighSpeedReadTimeout         = 0;
    HostTimeouts.LowPowerReadTimeout          = 0;
    HostTimeouts.HighSpeedWriteTimeout        = 0;
    HostTimeouts.HighSpeedWritePrespMode      = 0;
    HostTimeouts.LowPowerWriteTimeout         = 0;
    HostTimeouts.BTATimeout                   = 0;
    if(HAL_DSI_ConfigHostTimeouts(&hdsi_discovery, &HostTimeouts) != HAL_OK)
    {
      return(LCD_ERROR);
    }

    LPCmd.LPGenShortWriteNoP  = DSI_LP_GSW0P_ENABLE;
    LPCmd.LPGenShortWriteOneP = DSI_LP_GSW1P_ENABLE;
    LPCmd.LPGenShortWriteTwoP = DSI_LP_GSW2P_ENABLE;
    LPCmd.LPGenShortReadNoP   = DSI_LP_GSR0P_ENABLE;
    LPCmd.LPGenShortReadOneP  = DSI_LP_GSR1P_ENABLE;
    LPCmd.LPGenShortReadTwoP  = DSI_LP_GSR2P_ENABLE;
    LPCmd.LPGenLongWrite      = DSI_LP_GLW_DISABLE;
    LPCmd.LPDcsShortWriteNoP  = DSI_LP_DSW0P_ENABLE;
    LPCmd.LPDcsShortWriteOneP = DSI_LP_DSW1P_ENABLE;
    LPCmd.LPDcsShortReadNoP   = DSI_LP_DSR0P_ENABLE;
    LPCmd.LPDcsLongWrite      = DSI_LP_DLW_DISABLE;
    LPCmd.LPMaxReadPacket     = DSI_LP_MRDP_DISABLE;
    LPCmd.AcknowledgeRequest  = DSI_ACKNOWLEDGE_DISABLE;
    if(HAL_DSI_ConfigCommand(&hdsi_discovery, &LPCmd) != HAL_OK)
    {
      return(LCD_ERROR);
    }

    CmdCfg.VirtualChannelID      = 0;
    CmdCfg.ColorCoding           = DSI_RGB888;
    CmdCfg.CommandSize           = 390;
    CmdCfg.TearingEffectSource   = DSI_TE_DSILINK;
    CmdCfg.TearingEffectPolarity = DSI_TE_FALLING_EDGE;
    CmdCfg.HSPolarity            = DSI_HSYNC_ACTIVE_LOW;
    CmdCfg.VSPolarity            = DSI_VSYNC_ACTIVE_LOW;
    CmdCfg.DEPolarity            = DSI_DATA_ENABLE_ACTIVE_HIGH;
    CmdCfg.VSyncPol              = DSI_VSYNC_FALLING;
    CmdCfg.AutomaticRefresh      = DSI_AR_ENABLE;
    CmdCfg.TEAcknowledgeRequest  = DSI_TE_ACKNOWLEDGE_ENABLE;
    if(HAL_DSI_ConfigAdaptedCommandMode(&hdsi_discovery, &CmdCfg) != HAL_OK)
    {
      return(LCD_ERROR);
    }

    /* Disable the Tearing Effect interrupt activated by default on previous function */
    __HAL_DSI_DISABLE_IT(&hdsi_discovery, DSI_IT_TE);

    if(HAL_DSI_ConfigFlowControl(&hdsi_discovery, DSI_FLOW_CONTROL_BTA) != HAL_OK)
    {
      return(LCD_ERROR);
    }

    /* Enable DSI */
    __HAL_DSI_ENABLE(&hdsi_discovery);

    /*************************/
    /* LCD POWER ON SEQUENCE */
    /*************************/
    /* Step 1 */
    /* Go to command 2 */
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0xFE, 0x01);
    /* IC Frame rate control, set power, sw mapping, mux switch timing command */
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x06, 0x62);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x0E, 0x80);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x0F, 0x80);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x10, 0x71);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x13, 0x81);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x14, 0x81);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x15, 0x82);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x16, 0x82);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x18, 0x88);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x19, 0x55);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x1A, 0x10);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x1C, 0x99);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x1D, 0x03);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x1E, 0x03);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x1F, 0x03);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x20, 0x03);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x25, 0x03);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x26, 0x8D);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x2A, 0x03);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x2B, 0x8D);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x36, 0x00);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x37, 0x10);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x3A, 0x00);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x3B, 0x00);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x3D, 0x20);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x3F, 0x3A);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x40, 0x30);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x41, 0x1A);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x42, 0x33);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x43, 0x22);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x44, 0x11);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x45, 0x66);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x46, 0x55);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x47, 0x44);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x4C, 0x33);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x4D, 0x22);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x4E, 0x11);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x4F, 0x66);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x50, 0x55);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x51, 0x44);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x57, 0x33);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x6B, 0x1B);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x70, 0x55);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x74, 0x0C);

    /* Go to command 3 */
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0xFE, 0x02);
    /* Set the VGMP/VGSP coltage control */
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x9B, 0x40);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x9C, 0x00);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x9D, 0x20);

    /* Go to command 4 */
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0xFE, 0x03);
    /* Set the VGMP/VGSP coltage control */
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x9B, 0x40);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x9C, 0x00);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x9D, 0x20);


    /* Go to command 5 */
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0xFE, 0x04);
    /* VSR command */
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x5D, 0x10);
    /* VSR1 timing set */
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x00, 0x8D);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x01, 0x00);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x02, 0x01);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x03, 0x01);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x04, 0x10);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x05, 0x01);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x06, 0xA7);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x07, 0x20);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x08, 0x00);
    /* VSR2 timing set */
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x09, 0xC2);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x0A, 0x00);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x0B, 0x02);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x0C, 0x01);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x0D, 0x40);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x0E, 0x06);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x0F, 0x01);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x10, 0xA7);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x11, 0x00);
    /* VSR3 timing set */
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x12, 0xC2);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x13, 0x00);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x14, 0x02);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x15, 0x01);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x16, 0x40);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x17, 0x07);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x18, 0x01);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x19, 0xA7);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x1A, 0x00);
    /* VSR4 timing set */
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x1B, 0x82);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x1C, 0x00);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x1D, 0xFF);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x1E, 0x05);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x1F, 0x60);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x20, 0x02);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x21, 0x01);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x22, 0x7C);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x23, 0x00);
    /* VSR5 timing set */
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x24, 0xC2);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x25, 0x00);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x26, 0x04);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x27, 0x02);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x28, 0x70);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x29, 0x05);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x2A, 0x74);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x2B, 0x8D);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x2D, 0x00);
    /* VSR6 timing set */
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x2F, 0xC2);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x30, 0x00);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x31, 0x04);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x32, 0x02);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x33, 0x70);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x34, 0x07);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x35, 0x74);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x36, 0x8D);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x37, 0x00);
    /* VSR marping command */
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x5E, 0x20);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x5F, 0x31);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x60, 0x54);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x61, 0x76);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x62, 0x98);

    /* Go to command 6 */
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0xFE, 0x05);
    /* Set the ELVSS voltage */
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x05, 0x17);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x2A, 0x04);
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x91, 0x00);

    /* Go back in standard commands */
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0xFE, 0x00);

    /* Set tear off */
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, DSI_SET_TEAR_OFF, 0x0);

    /* Set DSI mode to internal timing added vs ORIGINAL for Command mode */
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0xC2, 0x0);

    /* Set memory address MODIFIED vs ORIGINAL */
    uint8_t InitParam1[4]= {0x00, 0x04, 0x01, 0x89}; // MODIF OFe: adjusted w/ real image
    HAL_DSI_LongWrite(&hdsi_discovery, 0, DSI_DCS_LONG_PKT_WRITE, 4, DSI_SET_COLUMN_ADDRESS, InitParam1);
    uint8_t InitParam2[4]= {0x00, 0x00, 0x01, 0x85};
    HAL_DSI_LongWrite(&hdsi_discovery, 0, DSI_DCS_LONG_PKT_WRITE, 4, DSI_SET_PAGE_ADDRESS, InitParam2);

    /* Sleep out */
    HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P0, DSI_EXIT_SLEEP_MODE, 0x0);

    HAL_Delay(120);

    /* Set display on */
    if(HAL_DSI_ShortWrite(&hdsi_discovery,
                          0,
                          DSI_DCS_SHORT_PKT_WRITE_P0,
                          DSI_SET_DISPLAY_ON,
                          0x0) != HAL_OK)
    {
      return(LCD_ERROR);
    }


    /* Enable DSI Wrapper */
    __HAL_DSI_WRAPPER_ENABLE(&hdsi_discovery);

    /* Initialize the font */
    BSP_LCD_SetFont(&LCD_DEFAULT_FONT);

    bsp_lcd_initialized = 1;
  }

  return(LCD_OK);
}

/**
  * @brief  De-initialize the DSI LCD.
  * @retval LCD state
  */
uint8_t BSP_LCD_DeInit(void)
{
  if(bsp_lcd_initialized == 1)
  {
    /* Disable DSI wrapper */
    __HAL_DSI_WRAPPER_DISABLE(&hdsi_discovery);

    /* Set display off */
    if(HAL_DSI_ShortWrite(&hdsi_discovery,
                          0,
                          DSI_DCS_SHORT_PKT_WRITE_P0,
                          DSI_SET_DISPLAY_OFF,
                          0x0) != HAL_OK)
    {
      return(LCD_ERROR);
    }

    /* Wait before entering in sleep mode */
    HAL_Delay(2000);

    /* Put LCD in sleep mode */
    if(HAL_DSI_ShortWrite(&hdsi_discovery,
                          0,
                          DSI_DCS_SHORT_PKT_WRITE_P0,
                          DSI_ENTER_SLEEP_MODE,
                          0x0) != HAL_OK)
    {
      return(LCD_ERROR);
    }

    HAL_Delay(120);

    /* Power off LCD */
    LCD_PowerOff();

    /* De-initialize DSI */
    if(HAL_DSI_DeInit(&hdsi_discovery) != HAL_OK)
    {
      return(LCD_ERROR);
    }

    /* De-initialize LTDC */
    if(HAL_LTDC_DeInit(&hltdc_discovery) != HAL_OK)
    {
      return(LCD_ERROR);
    }

    /* De-initialize GFXMMU */
    if(HAL_GFXMMU_DeInit(&hgfxmmu_discovery) != HAL_OK)
    {
      return(LCD_ERROR);
    }

    /* Call MSP de-initialize function */
    BSP_LCD_MspDeInit();

    bsp_lcd_initialized = 0;
  }

  return(LCD_OK);
}

/**
  * @brief  Gets the LCD X size.
  * @retval Used LCD X size
  */
uint32_t BSP_LCD_GetXSize(void)
{
  return (lcd_x_size);
}

/**
  * @brief  Gets the LCD Y size.
  * @retval Used LCD Y size
  */
uint32_t BSP_LCD_GetYSize(void)
{
  return (lcd_y_size);
}

/**
  * @brief  Selects the LCD Layer.
  * @param  LayerIndex: Layer foreground (1) or background (0)
  * @note : Only background layer can be used.
  * @retval LCD state
  */
uint8_t BSP_LCD_SelectLayer(uint32_t LayerIndex)
{
  uint8_t status = LCD_OK;

  if(LayerIndex == LTDC_ACTIVE_LAYER_BACKGROUND)
  {
    ActiveLayer = LayerIndex;
  }
  else
  {
    status = LCD_ERROR;
  }
  return(status);
}

/**
  * @brief  Sets an LCD Layer visible
  * @param  LayerIndex: Visible Layer
  * @param  State: New state of the specified layer
  *          This parameter can be one of the following values:
  *            @arg  ENABLE
  *            @arg  DISABLE
  * @note : Only background layer can be used.
  * @retval LCD state
  */
uint8_t BSP_LCD_SetLayerVisible(uint32_t LayerIndex, FunctionalState State)
{
  uint8_t status = LCD_OK;

  if(LayerIndex == LTDC_ACTIVE_LAYER_BACKGROUND)
  {
    if(State == ENABLE)
    {
      __HAL_LTDC_LAYER_ENABLE(&(hltdc_discovery), LayerIndex);
    }
    else
    {
      __HAL_LTDC_LAYER_DISABLE(&(hltdc_discovery), LayerIndex);
    }
    __HAL_LTDC_RELOAD_IMMEDIATE_CONFIG(&(hltdc_discovery));
  }
  else
  {
    status = LCD_ERROR;
  }
  return(status);
}

/**
  * @brief  Configures the transparency.
  * @param  LayerIndex: Layer foreground or background.
  * @param  Transparency: Transparency
  *           This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF
  * @note : Only background layer can be used.
  * @retval LCD state
  */
uint8_t BSP_LCD_SetTransparency(uint32_t LayerIndex, uint8_t Transparency)
{
  uint8_t status = LCD_OK;

  if(LayerIndex == LTDC_ACTIVE_LAYER_BACKGROUND)
  {
    HAL_LTDC_SetAlpha(&(hltdc_discovery), Transparency, LayerIndex);
  }
  else
  {
    status = LCD_ERROR;
  }
  return(status);
}

/**
  * @brief  Configures and sets the color keying.
  * @param  LayerIndex: Layer foreground (1) or background (0)
  * @param  RGBValue: Color reference
  * @note : Only background layer can be used.
  * @retval LCD state
  */
uint8_t BSP_LCD_SetColorKeying(uint32_t LayerIndex, uint32_t RGBValue)
{
  uint8_t status = LCD_OK;

  if(LayerIndex == LTDC_ACTIVE_LAYER_BACKGROUND)
  {
    /* Configure and Enable the color Keying for LCD Layer */
    HAL_LTDC_ConfigColorKeying(&(hltdc_discovery), RGBValue, LayerIndex);
    HAL_LTDC_EnableColorKeying(&(hltdc_discovery), LayerIndex);
  }
  else
  {
    status = LCD_ERROR;
  }
  return(status);
}

/**
  * @brief  Disables the color keying.
  * @param  LayerIndex: Layer foreground (1) or background (0)
  * @note : Only background layer can be used.
  * @retval LCD state
  */
uint8_t BSP_LCD_ResetColorKeying(uint32_t LayerIndex)
{
  uint8_t status = LCD_OK;

  if(LayerIndex == LTDC_ACTIVE_LAYER_BACKGROUND)
  {
    /* Disable the color Keying for LCD Layer */
    HAL_LTDC_DisableColorKeying(&(hltdc_discovery), LayerIndex);
  }
  else
  {
    status = LCD_ERROR;
  }
  return(status);
}

/**
  * @brief  Sets the LCD text color.
  * @param  Color: Text color code ARGB8888
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
  * @param  Color: Layer background color code ARGB8888
  */
void BSP_LCD_SetBackColor(uint32_t Color)
{
  DrawProp[ActiveLayer].BackColor = Color;
}

/**
  * @brief  Gets the LCD background color.
  * @retval Used background color
  */
uint32_t BSP_LCD_GetBackColor(void)
{
  return DrawProp[ActiveLayer].BackColor;
}

/**
  * @brief  Sets the LCD text font.
  * @param  fonts: Layer font to be used
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
  * @retval ARGB8888 pixel color
  */
uint32_t BSP_LCD_ReadPixel(uint16_t Xpos, uint16_t Ypos)
{
  uint32_t ret = 0;

  /* Read value of corresponding pixel */
  /* We have 768 pixels per line and 4 bytes per pixel so 3072 bytes per line */
  ret = *(__IO uint32_t*) (hltdc_discovery.LayerCfg[ActiveLayer].FBStartAdress + (4*(Ypos*768 + Xpos)));

  /* Return pixel value */
  return ret;
}

/**
  * @brief  Clears the whole currently active layer of LTDC.
  * @param  Color: Color of the background (in ARGB8888 format)
  */
void BSP_LCD_Clear(uint32_t Color)
{
  /* Clear the LCD */
  /* Offset line is 768 - 390 = 378 */
  LL_FillBuffer(ActiveLayer, (uint32_t *)(hltdc_discovery.LayerCfg[ActiveLayer].FBStartAdress), 390, 390, 378, Color);
}

/**
  * @brief  Clears the selected line in currently active layer.
  * @param  Line: Line to be cleared
  */
void BSP_LCD_ClearStringLine(uint32_t Line)
{
  uint32_t color_backup = DrawProp[ActiveLayer].TextColor;
  DrawProp[ActiveLayer].TextColor = DrawProp[ActiveLayer].BackColor;

  /* Draw rectangle with background color */
  BSP_LCD_FillRect(0, (Line * DrawProp[ActiveLayer].pFont->Height), 390, DrawProp[ActiveLayer].pFont->Height);

  DrawProp[ActiveLayer].TextColor = color_backup;
  BSP_LCD_SetTextColor(DrawProp[ActiveLayer].TextColor);
}

/**
  * @brief  Displays one character in currently active layer.
  * @param  Xpos: Start column address
  * @param  Ypos: Line where to display the character shape.
  * @param  Ascii: Character ascii code
  *           This parameter must be a number between Min_Data = 0x20 and Max_Data = 0x7E
  */
void BSP_LCD_DisplayChar(uint16_t Xpos, uint16_t Ypos, uint8_t Ascii)
{
  DrawChar(Xpos, Ypos, &DrawProp[ActiveLayer].pFont->table[(Ascii-' ') * DrawProp[ActiveLayer].pFont->Height * ((DrawProp[ActiveLayer].pFont->Width + 7) / 8)]);
}

/**
  * @brief  Displays characters in currently active layer.
  * @param  Xpos: X position (in pixel)
  * @param  Ypos: Y position (in pixel)
  * @param  Text: Pointer to string to display on LCD
  * @param  Mode: Display mode
  *          This parameter can be one of the following values:
  *            @arg  CENTER_MODE
  *            @arg  RIGHT_MODE
  *            @arg  LEFT_MODE
  */
void BSP_LCD_DisplayStringAt(uint16_t Xpos, uint16_t Ypos, uint8_t *Text, Text_AlignModeTypdef Mode)
{
  uint16_t refcolumn = 1, i = 0;
  uint32_t size = 0, xsize = 0;
  uint8_t  *ptr = Text;

  /* Get the text size */
  while (*ptr++) size ++ ;

  /* Characters number per line */
  xsize = (390/DrawProp[ActiveLayer].pFont->Width);

  switch (Mode)
  {
  case CENTER_MODE:
    {
      refcolumn = Xpos + ((xsize - size)* DrawProp[ActiveLayer].pFont->Width) / 2;
      break;
    }
  case LEFT_MODE:
    {
      refcolumn = Xpos;
      break;
    }
  case RIGHT_MODE:
    {
      refcolumn = - Xpos + ((xsize - size)*DrawProp[ActiveLayer].pFont->Width);
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

  /* Send the string character by character on LCD */
  while ((*Text != 0) && (((390 - (i*DrawProp[ActiveLayer].pFont->Width)) & 0xFFFF) >= DrawProp[ActiveLayer].pFont->Width))
  {
    /* Display one character on LCD */
    BSP_LCD_DisplayChar(refcolumn, Ypos, *Text);
    /* Decrement the column position by 16 */
    refcolumn += DrawProp[ActiveLayer].pFont->Width;

    /* Point on the next character */
    Text++;
    i++;
  }

}

/**
  * @brief  Displays string on specified line of the LCD.
  * @param  Line: Line where to display the character shape
  * @param  ptr: Pointer to string to display on LCD
  */
void BSP_LCD_DisplayStringAtLine(uint16_t Line, uint8_t *ptr)
{
  BSP_LCD_DisplayStringAt(0, Line * (((sFONT *)BSP_LCD_GetFont())->Height), ptr, CENTER_MODE);
}

/**
  * @brief  Draws an horizontal line in currently active layer.
  * @param  Xpos: X position
  * @param  Ypos: Y position
  * @param  Length: Line length
  */
void BSP_LCD_DrawHLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length)
{
  uint32_t  Xaddress = 0;

  /* Get the line address */
  Xaddress = (hltdc_discovery.LayerCfg[ActiveLayer].FBStartAdress) + 4*(768*Ypos + Xpos);

  /* Write line */
  LL_FillBuffer(ActiveLayer, (uint32_t *)Xaddress, Length, 1, (768 - Length), DrawProp[ActiveLayer].TextColor);
}

/**
  * @brief  Draws a vertical line in currently active layer.
  * @param  Xpos: X position
  * @param  Ypos: Y position
  * @param  Length: Line length
  */
void BSP_LCD_DrawVLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length)
{
  uint32_t  Xaddress = 0;

  /* Get the line address */
  Xaddress = (hltdc_discovery.LayerCfg[ActiveLayer].FBStartAdress) + 4*(768*Ypos + Xpos);

  /* Write line */
  LL_FillBuffer(ActiveLayer, (uint32_t *)Xaddress, 1, Length, (768 - 1), DrawProp[ActiveLayer].TextColor);
}

/**
  * @brief  Draws an uni-line (between two points) in currently active layer.
  * @param  x1: Point 1 X position
  * @param  y1: Point 1 Y position
  * @param  x2: Point 2 X position
  * @param  y2: Point 2 Y position
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
    BSP_LCD_DrawPixel(x, y, DrawProp[ActiveLayer].TextColor);   /* Draw the current pixel */
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
  * @brief  Draws a rectangle in currently active layer.
  * @param  Xpos: X position
  * @param  Ypos: Y position
  * @param  Width: Rectangle width
  * @param  Height: Rectangle height
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
  * @brief  Draws a circle in currently active layer.
  * @param  Xpos: X position
  * @param  Ypos: Y position
  * @param  Radius: Circle radius
  */
void BSP_LCD_DrawCircle(uint16_t Xpos, uint16_t Ypos, uint16_t Radius)
{
  int32_t   D;    /* Decision Variable */
  uint32_t  CurX; /* Current X Value */
  uint32_t  CurY; /* Current Y Value */

  D = 3 - (Radius << 1);
  CurX = 0;
  CurY = Radius;

  while (CurX <= CurY)
  {
    BSP_LCD_DrawPixel((Xpos + CurX), (Ypos - CurY), DrawProp[ActiveLayer].TextColor);

    BSP_LCD_DrawPixel((Xpos - CurX), (Ypos - CurY), DrawProp[ActiveLayer].TextColor);

    BSP_LCD_DrawPixel((Xpos + CurY), (Ypos - CurX), DrawProp[ActiveLayer].TextColor);

    BSP_LCD_DrawPixel((Xpos - CurY), (Ypos - CurX), DrawProp[ActiveLayer].TextColor);

    BSP_LCD_DrawPixel((Xpos + CurX), (Ypos + CurY), DrawProp[ActiveLayer].TextColor);

    BSP_LCD_DrawPixel((Xpos - CurX), (Ypos + CurY), DrawProp[ActiveLayer].TextColor);

    BSP_LCD_DrawPixel((Xpos + CurY), (Ypos + CurX), DrawProp[ActiveLayer].TextColor);

    BSP_LCD_DrawPixel((Xpos - CurY), (Ypos + CurX), DrawProp[ActiveLayer].TextColor);

    if (D < 0)
    {
      D += (CurX << 2) + 6;
    }
    else
    {
      D += ((CurX - CurY) << 2) + 10;
      CurY--;
    }
    CurX++;
  }
}

/**
  * @brief  Draws an poly-line (between many points) in currently active layer.
  * @param  Points: Pointer to the points array
  * @param  PointCount: Number of points
  */
void BSP_LCD_DrawPolygon(pPoint Points, uint16_t PointCount)
{
  int16_t X = 0, Y = 0;

  if(PointCount < 2)
  {
    return;
  }

  BSP_LCD_DrawLine(Points->X, Points->Y, (Points+PointCount-1)->X, (Points+PointCount-1)->Y);

  while(--PointCount)
  {
    X = Points->X;
    Y = Points->Y;
    Points++;
    BSP_LCD_DrawLine(X, Y, Points->X, Points->Y);
  }
}

/**
  * @brief  Draws an ellipse on LCD in currently active layer.
  * @param  Xpos: X position
  * @param  Ypos: Y position
  * @param  XRadius: Ellipse X radius
  * @param  YRadius: Ellipse Y radius
  */
void BSP_LCD_DrawEllipse(int Xpos, int Ypos, int XRadius, int YRadius)
{
  int x = 0, y = -YRadius, err = 2-2*XRadius, e2;
  float K = 0, rad1 = 0, rad2 = 0;

  rad1 = XRadius;
  rad2 = YRadius;

  K = (float)(rad2/rad1);

  do {
    BSP_LCD_DrawPixel((Xpos-(uint16_t)(x/K)), (Ypos+y), DrawProp[ActiveLayer].TextColor);
    BSP_LCD_DrawPixel((Xpos+(uint16_t)(x/K)), (Ypos+y), DrawProp[ActiveLayer].TextColor);
    BSP_LCD_DrawPixel((Xpos+(uint16_t)(x/K)), (Ypos-y), DrawProp[ActiveLayer].TextColor);
    BSP_LCD_DrawPixel((Xpos-(uint16_t)(x/K)), (Ypos-y), DrawProp[ActiveLayer].TextColor);

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
  * @brief  Draws a bitmap picture loaded in the internal Flash (32 bpp) in currently active layer.
  * @param  Xpos: Bmp X position in the LCD
  * @param  Ypos: Bmp Y position in the LCD
  * @param  pbmp: Pointer to Bmp picture address in the internal Flash
  */
void BSP_LCD_DrawBitmap(uint32_t Xpos, uint32_t Ypos, uint8_t *pbmp)
{
  uint32_t index = 0, width = 0, height = 0, bit_pixel = 0;
  uint32_t Address;
  uint32_t InputColorMode = 0;

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
  Address = hltdc_discovery.LayerCfg[ActiveLayer].FBStartAdress + (((768*Ypos) + Xpos)*(4));

  /* Get the layer pixel format */
  if ((bit_pixel/8) == 4)
  {
    InputColorMode = DMA2D_INPUT_ARGB8888;
  }
  else if ((bit_pixel/8) == 2)
  {
    InputColorMode = DMA2D_INPUT_RGB565;
  }
  else
  {
    InputColorMode = DMA2D_INPUT_RGB888;
  }

  /* Bypass the bitmap header */
  pbmp += (index + (width * (height - 1) * (bit_pixel/8)));

  /* Convert picture to ARGB8888 pixel format */
  for(index=0; index < height; index++)
  {
    /* Pixel format conversion */
    LL_ConvertLineToARGB8888((uint32_t *)pbmp, (uint32_t *)Address, width, InputColorMode);

    /* Increment the source and destination buffers */
    Address+=  (768*4);
    pbmp -= width*(bit_pixel/8);
  }
}

/**
  * @brief  Draws a full rectangle in currently active layer.
  * @param  Xpos: X position
  * @param  Ypos: Y position
  * @param  Width: Rectangle width
  * @param  Height: Rectangle height
  */
void BSP_LCD_FillRect(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height)
{
  uint32_t  Xaddress = 0;

  /* Set the text color */
  BSP_LCD_SetTextColor(DrawProp[ActiveLayer].TextColor);

  /* Get the rectangle start address */
  Xaddress = (hltdc_discovery.LayerCfg[ActiveLayer].FBStartAdress) + 4*(768*Ypos + Xpos);

  /* Fill the rectangle */
  LL_FillBuffer(ActiveLayer, (uint32_t *)Xaddress, Width, Height, (768 - Width), DrawProp[ActiveLayer].TextColor);
}

/**
  * @brief  Draws a full circle in currently active layer.
  * @param  Xpos: X position
  * @param  Ypos: Y position
  * @param  Radius: Circle radius
  */
void BSP_LCD_FillCircle(uint16_t Xpos, uint16_t Ypos, uint16_t Radius)
{
  int32_t  D;     /* Decision Variable */
  uint32_t  CurX; /* Current X Value */
  uint32_t  CurY; /* Current Y Value */

  D = 3 - (Radius << 1);

  CurX = 0;
  CurY = Radius;

  BSP_LCD_SetTextColor(DrawProp[ActiveLayer].TextColor);

  while (CurX <= CurY)
  {
    if(CurY > 0)
    {
      BSP_LCD_DrawHLine(Xpos - CurY, Ypos + CurX, 2*CurY);
      BSP_LCD_DrawHLine(Xpos - CurY, Ypos - CurX, 2*CurY);
    }

    if(CurX > 0)
    {
      BSP_LCD_DrawHLine(Xpos - CurX, Ypos - CurY, 2*CurX);
      BSP_LCD_DrawHLine(Xpos - CurX, Ypos + CurY, 2*CurX);
    }
    if (D < 0)
    {
      D += (CurX << 2) + 6;
    }
    else
    {
      D += ((CurX - CurY) << 2) + 10;
      CurY--;
    }
    CurX++;
  }

  BSP_LCD_SetTextColor(DrawProp[ActiveLayer].TextColor);
  BSP_LCD_DrawCircle(Xpos, Ypos, Radius);
}

/**
  * @brief  Draws a full poly-line (between many points) in currently active layer.
  * @param  Points: Pointer to the points array
  * @param  PointCount: Number of points
  */
void BSP_LCD_FillPolygon(pPoint Points, uint16_t PointCount)
{
  int16_t X = 0, Y = 0, X2 = 0, Y2 = 0, X_center = 0, Y_center = 0, X_first = 0, Y_first = 0, pixelX = 0, pixelY = 0, counter = 0;
  uint16_t  IMAGE_LEFT = 0, IMAGE_RIGHT = 0, IMAGE_TOP = 0, IMAGE_BOTTOM = 0;

  IMAGE_LEFT = IMAGE_RIGHT = Points->X;
  IMAGE_TOP= IMAGE_BOTTOM = Points->Y;

  for(counter = 1; counter < PointCount; counter++)
  {
    pixelX = POLY_X(counter);
    if(pixelX < IMAGE_LEFT)
    {
      IMAGE_LEFT = pixelX;
    }
    if(pixelX > IMAGE_RIGHT)
    {
      IMAGE_RIGHT = pixelX;
    }

    pixelY = POLY_Y(counter);
    if(pixelY < IMAGE_TOP)
    {
      IMAGE_TOP = pixelY;
    }
    if(pixelY > IMAGE_BOTTOM)
    {
      IMAGE_BOTTOM = pixelY;
    }
  }

  if(PointCount < 2)
  {
    return;
  }

  X_center = (IMAGE_LEFT + IMAGE_RIGHT)/2;
  Y_center = (IMAGE_BOTTOM + IMAGE_TOP)/2;

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
  * @brief  Draws a full ellipse in currently active layer.
  * @param  Xpos: X position
  * @param  Ypos: Y position
  * @param  XRadius: Ellipse X radius
  * @param  YRadius: Ellipse Y radius
  */
void BSP_LCD_FillEllipse(int Xpos, int Ypos, int XRadius, int YRadius)
{
  int x = 0, y = -YRadius, err = 2-2*XRadius, e2;
  float K = 0, rad1 = 0, rad2 = 0;

  rad1 = XRadius;
  rad2 = YRadius;

  K = (float)(rad2/rad1);

  do
  {
    BSP_LCD_DrawHLine((Xpos-(uint16_t)(x/K)), (Ypos+y), (2*(uint16_t)(x/K) + 1));
    BSP_LCD_DrawHLine((Xpos-(uint16_t)(x/K)), (Ypos-y), (2*(uint16_t)(x/K) + 1));

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
  * @brief  Draws a pixel on LCD.
  * @param  Xpos: X position
  * @param  Ypos: Y position
  * @param  RGB_Code: Pixel color in ARGB mode (8-8-8-8)
  */
void BSP_LCD_DrawPixel(uint16_t Xpos, uint16_t Ypos, uint32_t RGB_Code)
{
  /* Write pixel */
  *(__IO uint32_t*) (hltdc_discovery.LayerCfg[ActiveLayer].FBStartAdress + (4*(Ypos*768 + Xpos))) = RGB_Code;
}

/**
  * @brief  Switch back on the display if was switched off by previous call of BSP_LCD_DisplayOff().
  */
void BSP_LCD_DisplayOn(void)
{
  /* Send Display on DCS command to display */
  HAL_DSI_ShortWrite(&hdsi_discovery,
                     0,
                     DSI_DCS_SHORT_PKT_WRITE_P0,
                     DSI_SET_DISPLAY_ON,
                     0x0);
}

/**
  * @brief  Switch Off the display.
  */
void BSP_LCD_DisplayOff(void)
{
  /* Send Display off DCS Command to display */
  HAL_DSI_ShortWrite(&hdsi_discovery,
                     0,
                     DSI_DCS_SHORT_PKT_WRITE_P0,
                     DSI_SET_DISPLAY_OFF,
                     0x0);
}

/**
  * @brief  Refresh the display.
  */
void BSP_LCD_Refresh(void)
{
  /* Set frame buffer busy */
  FrameBufferAvailable = 0;

  /* Set tear on */
  HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, DSI_SET_TEAR_ON, 0x0);
}

/**
  * @brief  Check if frame buffer is available.
  * @retval LCD_OK if frame buffer is available else LCD_ERROR (frame buffer busy)
  */
uint8_t BSP_LCD_IsFrameBufferAvailable(void)
{
  uint8_t status;
  /* Check frame buffer status */
  status = (FrameBufferAvailable == 1) ? LCD_OK : LCD_ERROR;

  return(status);
}

/**
  * @brief  Set the brightness value
  * @param  BrightnessValue: [0% Min (black), 100% Max]
  */
void BSP_LCD_SetBrightness(uint8_t BrightnessValue)
{
  /* Send Display on DCS command to display */
  HAL_DSI_ShortWrite(&hdsi_discovery,
                     0,
                     DSI_DCS_SHORT_PKT_WRITE_P1,
                     0x51, (uint16_t)(BrightnessValue * 255)/100);
}

/*******************************************************************************
                       LTDC, DMA2D and DSI BSP Routines
*******************************************************************************/
/**
  * @brief  Handles DMA2D interrupt request.
  * @note   Application can surcharge if needed this function implementation.
  */
__weak void BSP_LCD_DMA2D_IRQHandler(void)
{
  HAL_DMA2D_IRQHandler(&hdma2d_discovery);
}

/**
  * @brief  Handles DSI interrupt request.
  * @note   Application can surcharge if needed this function implementation.
  */
__weak void BSP_LCD_DSI_IRQHandler(void)
{
  HAL_DSI_IRQHandler(&(hdsi_discovery));
}

/**
  * @brief  Handles LTDC interrupt request.
  * @note   Application can surcharge if needed this function implementation.
  */
__weak void BSP_LCD_LTDC_IRQHandler(void)
{
  HAL_LTDC_IRQHandler(&(hltdc_discovery));
}

/**
  * @brief  This function handles LTDC Error interrupt Handler.
  * @note   Application can surcharge if needed this function implementation.
  */

__weak void BSP_LCD_LTDC_ER_IRQHandler(void)
{
  HAL_LTDC_IRQHandler(&(hltdc_discovery));
}

/**
  * @brief  De-Initializes the BSP LCD Msp.
  * @note   Application can surcharge if needed this function implementation.
  */
__weak void BSP_LCD_MspDeInit(void)
{
  /* Disable IRQ of LTDC IP */
  HAL_NVIC_DisableIRQ(LTDC_IRQn);
  HAL_NVIC_DisableIRQ(LTDC_ER_IRQn);

  /* Disable IRQ of DMA2D IP */
  HAL_NVIC_DisableIRQ(DMA2D_IRQn);

  /* Disable IRQ of DSI IP */
  HAL_NVIC_DisableIRQ(DSI_IRQn);

  /* Disable HSE used for DSI PLL */
  if(bsp_lcd_hse_to_disable == 1)
  {
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_OFF;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
      while(1);
    }
    /* Workaround for long HSE startup time (deinit PH0) */
    HAL_GPIO_DeInit(GPIOH, GPIO_PIN_0);

    bsp_lcd_hse_to_disable = 0;
  }

  /* Force and let in reset state GFXMMU, LTDC, DMA2D and DSI Host + Wrapper IPs */
  __HAL_RCC_GFXMMU_FORCE_RESET();
  __HAL_RCC_LTDC_FORCE_RESET();
  __HAL_RCC_DMA2D_FORCE_RESET();
  __HAL_RCC_DSI_FORCE_RESET();

  /* Disable the GFXMMU, LTDC, DMA2D and DSI Host and Wrapper clocks */
  __HAL_RCC_GFXMMU_CLK_DISABLE();
  __HAL_RCC_LTDC_CLK_DISABLE();
  __HAL_RCC_DMA2D_CLK_DISABLE();
  __HAL_RCC_DSI_CLK_DISABLE();
}

/**
  * @brief  Initialize the BSP LCD Msp.
  * @note   Application can surcharge if needed this function implementation.
  */
__weak void BSP_LCD_MspInit(void)
{
  /* Enable the GFXMMU clock */
  __HAL_RCC_GFXMMU_CLK_ENABLE();

  /* Reset of GFXMMU IP */
  __HAL_RCC_GFXMMU_FORCE_RESET();
  __HAL_RCC_GFXMMU_RELEASE_RESET();

  /* Enable the LTDC clock */
  __HAL_RCC_LTDC_CLK_ENABLE();

  /* Reset of LTDC IP */
  __HAL_RCC_LTDC_FORCE_RESET();
  __HAL_RCC_LTDC_RELEASE_RESET();

  /* Enable the DMA2D clock */
  __HAL_RCC_DMA2D_CLK_ENABLE();

  /* Reset of DMA2D IP */
  __HAL_RCC_DMA2D_FORCE_RESET();
  __HAL_RCC_DMA2D_RELEASE_RESET();

  /* Enable DSI Host and wrapper clocks */
  __HAL_RCC_DSI_CLK_ENABLE();

  /* Reset the DSI Host and wrapper */
  __HAL_RCC_DSI_FORCE_RESET();
  __HAL_RCC_DSI_RELEASE_RESET();

  /* Configure the clock for the LTDC */
  /* We want DSI PHI at 500MHz */
  /* We have only one line => 500Mbps */
  /* With 24bits per pixel, equivalent PCLK is 500/24 = 20.8MHz */
  /* We will set PCLK at 15MHz */
  /* Following values are OK with MSI = 4MHz */
  /* (4*60)/(1*4*4) = 15MHz */
  RCC_PeriphCLKInitTypeDef  PeriphClkInit;
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
  PeriphClkInit.PLLSAI2.PLLSAI2Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI2.PLLSAI2M = 1;
  PeriphClkInit.PLLSAI2.PLLSAI2N = 60;
  PeriphClkInit.PLLSAI2.PLLSAI2R = RCC_PLLR_DIV4;
  PeriphClkInit.LtdcClockSelection = RCC_LTDCCLKSOURCE_PLLSAI2_DIV4;
  PeriphClkInit.PLLSAI2.PLLSAI2ClockOut = RCC_PLLSAI2_LTDCCLK;
  if(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    while(1);
  }

  /* Enable HSE used for DSI PLL */
  RCC_OscInitTypeDef RCC_OscInitStruct;
  HAL_RCC_GetOscConfig(&RCC_OscInitStruct);
  if(RCC_OscInitStruct.HSEState == RCC_HSE_OFF)
  {
    /* Workaround for long HSE startup time (set PH0 to output PP low) */
    GPIO_InitTypeDef  GPIO_InitStruct;
    __HAL_RCC_GPIOH_CLK_ENABLE();
    GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Pin       = GPIO_PIN_0;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_0, GPIO_PIN_RESET);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
      while(1);
    }
    bsp_lcd_hse_to_disable = 1;
  }

  /* NVIC configuration for LTDC interrupts that are now enabled */
  HAL_NVIC_SetPriority(LTDC_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(LTDC_IRQn);
  HAL_NVIC_SetPriority(LTDC_ER_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(LTDC_ER_IRQn);

  /* NVIC configuration for DMA2D interrupt that is now enabled */
  HAL_NVIC_SetPriority(DMA2D_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA2D_IRQn);

  /* NVIC configuration for DSI interrupt that is now enabled */
  HAL_NVIC_SetPriority(DSI_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DSI_IRQn);
}

/**
  * @}
  */

/** @defgroup STM32L4R9I_DISCOVERY_LCD_Private_Functions LCD Private Functions
  * @{
  */

/**
  * @brief  End of Refresh DSI callback.
  * @param  hdsi: pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @retval None
  */
void HAL_DSI_EndOfRefreshCallback(DSI_HandleTypeDef *hdsi)
{
  /* Clear pending tearing effect flag */
  __HAL_DSI_CLEAR_FLAG(hdsi, DSI_FLAG_TE);

  /* Set frame buffer available */
  FrameBufferAvailable = 1;
}

/**
  * @brief  LCD power on
  *         Power on LCD.
  */
static void LCD_PowerOn(void)
{
  /* Configure DSI_RESET and DSI_POWER_ON only if psram is not currently used */
  if(bsp_psram_initialized == 0)
  {
    BSP_IO_Init();

#if defined(USE_STM32L4R9I_DISCO_REVA) || defined(USE_STM32L4R9I_DISCO_REVB)
    /* Set DSI_POWER_ON to input floating to avoid I2C issue during input PD configuration */
    BSP_IO_ConfigPin(IO_PIN_8, IO_MODE_INPUT);

    /* Configure the GPIO connected to DSI_RESET signal */
    BSP_IO_ConfigPin(IO_PIN_10, IO_MODE_OUTPUT);

    /* Activate DSI_RESET (active low) */
    BSP_IO_WritePin(IO_PIN_10, GPIO_PIN_RESET);

    /* Configure the GPIO connected to DSI_POWER_ON signal as input pull down */
    /* to activate 3V3_LCD. VDD_LCD is also activated if VDD = 3,3V */
    BSP_IO_ConfigPin(IO_PIN_8, IO_MODE_INPUT_PD);

    /* Wait at least 1ms before enabling 1V8_LCD */
    HAL_Delay(1);

    /* Configure the GPIO connected to DSI_POWER_ON signal as output low */
    /* to activate 1V8_LCD. VDD_LCD is also activated if VDD = 1,8V */
    BSP_IO_WritePin(IO_PIN_8, GPIO_PIN_RESET);
    BSP_IO_ConfigPin(IO_PIN_8, IO_MODE_OUTPUT);
#else /* USE_STM32L4R9I_DISCO_REVA || USE_STM32L4R9I_DISCO_REVB */
    /* Configure the GPIO connected to DSI_3V3_POWERON signal as output low */
    /* to activate 3V3_LCD. VDD_LCD is also activated if VDD = 3,3V */
    BSP_IO_WritePin(IO_PIN_8, GPIO_PIN_RESET);
    BSP_IO_ConfigPin(IO_PIN_8, IO_MODE_OUTPUT);

    /* Wait at least 1ms before enabling 1V8_LCD */
    HAL_Delay(1);

    /* Configure the GPIO connected to DSI_1V8_POWERON signal as output low */
    /* to activate 1V8_LCD. VDD_LCD is also activated if VDD = 1,8V */
    BSP_IO_WritePin(AGPIO_PIN_2, GPIO_PIN_RESET);
    BSP_IO_ConfigPin(AGPIO_PIN_2, IO_MODE_OUTPUT);
#endif /* USE_STM32L4R9I_DISCO_REVA || USE_STM32L4R9I_DISCO_REVB */

    /* Wait at least 15 ms (minimum reset low width is 10ms and add margin for 1V8_LCD ramp-up) */
    HAL_Delay(15);
  }

#if defined(USE_STM32L4R9I_DISCO_REVA) || defined(USE_STM32L4R9I_DISCO_REVB)
  /* Deactivate DSI_RESET */
  BSP_IO_WritePin(IO_PIN_10, GPIO_PIN_SET);
#else /* USE_STM32L4R9I_DISCO_REVA || USE_STM32L4R9I_DISCO_REVB */
  /* Configure the GPIO connected to DSI_RESET signal */
  BSP_IO_ConfigPin(IO_PIN_10, IO_MODE_OUTPUT);

  /* Deactivate DSI_RESET */
  BSP_IO_WritePin(IO_PIN_10, GPIO_PIN_SET);
#endif /* USE_STM32L4R9I_DISCO_REVA || USE_STM32L4R9I_DISCO_REVB */

  /* Wait reset complete time (maximum time is 5ms when LCD in sleep mode and 120ms when LCD is not in sleep mode) */
  HAL_Delay(120);
}

/**
  * @brief  LCD power off
  *         Power off LCD.
  */
static void LCD_PowerOff(void)
{
  /* Activate DSI_RESET */
  BSP_IO_WritePin(IO_PIN_10, GPIO_PIN_RESET);

  /* Wait at least 5 ms */
  HAL_Delay(5);

  /* Set DSI_POWER_ON to analog mode only if psram is not currently used */
  if(bsp_psram_initialized == 0)
  {
#if defined(USE_STM32L4R9I_DISCO_REVA) || defined(USE_STM32L4R9I_DISCO_REVB)
    BSP_IO_ConfigPin(IO_PIN_8, IO_MODE_ANALOG);
#else /* USE_STM32L4R9I_DISCO_REVA || USE_STM32L4R9I_DISCO_REVB */
    /* Disable first DSI_1V8_PWRON then DSI_3V3_PWRON */
    BSP_IO_ConfigPin(AGPIO_PIN_2, IO_MODE_ANALOG);
    BSP_IO_ConfigPin(IO_PIN_8, IO_MODE_ANALOG);
#endif /* USE_STM32L4R9I_DISCO_REVA || USE_STM32L4R9I_DISCO_REVB */
  }
}

/**
  * @brief  Draws a character on LCD.
  * @param  Xpos: Line where to display the character shape
  * @param  Ypos: Start column address
  * @param  c: Pointer to the character data
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
  * @brief  Fills a buffer.
  * @param  LayerIndex: Layer index
  * @param  pDst: Pointer to destination buffer
  * @param  xSize: Buffer width
  * @param  ySize: Buffer height
  * @param  OffLine: Offset
  * @param  ColorIndex: Color index
  */
static void LL_FillBuffer(uint32_t LayerIndex, void *pDst, uint32_t xSize, uint32_t ySize, uint32_t OffLine, uint32_t ColorIndex)
{
  /* Register to memory mode with ARGB8888 as color Mode */
  hdma2d_discovery.Init.Mode          = DMA2D_R2M;
  hdma2d_discovery.Init.ColorMode     = DMA2D_OUTPUT_ARGB8888;
  hdma2d_discovery.Init.OutputOffset  = OffLine;
  hdma2d_discovery.Init.AlphaInverted = DMA2D_REGULAR_ALPHA;
  hdma2d_discovery.Init.RedBlueSwap   = DMA2D_RB_REGULAR;

  hdma2d_discovery.Instance = DMA2D;

  /* DMA2D Initialization */
  if(HAL_DMA2D_Init(&hdma2d_discovery) == HAL_OK)
  {
    if(HAL_DMA2D_ConfigLayer(&hdma2d_discovery, LayerIndex) == HAL_OK)
    {
      if (HAL_DMA2D_Start(&hdma2d_discovery, ColorIndex, (uint32_t)pDst, xSize, ySize) == HAL_OK)
      {
        /* Polling For DMA transfer */
        HAL_DMA2D_PollForTransfer(&hdma2d_discovery, 10);
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
  */
static void LL_ConvertLineToARGB8888(void *pSrc, void *pDst, uint32_t xSize, uint32_t ColorMode)
{
  /* Configure the DMA2D Mode, Color Mode and output offset */
  hdma2d_discovery.Init.Mode           = DMA2D_M2M_PFC;
  hdma2d_discovery.Init.ColorMode      = DMA2D_OUTPUT_ARGB8888;
  hdma2d_discovery.Init.OutputOffset   = 0;
  hdma2d_discovery.Init.LineOffsetMode = DMA2D_LOM_PIXELS;
  hdma2d_discovery.Init.BytesSwap      = DMA2D_BYTES_REGULAR;
  hdma2d_discovery.Init.AlphaInverted  = DMA2D_REGULAR_ALPHA;
  hdma2d_discovery.Init.RedBlueSwap    = DMA2D_RB_REGULAR;

  /* Foreground Configuration */
  hdma2d_discovery.LayerCfg[1].AlphaMode      = DMA2D_NO_MODIF_ALPHA;
  hdma2d_discovery.LayerCfg[1].InputAlpha     = 0xFF;
  hdma2d_discovery.LayerCfg[1].InputColorMode = ColorMode;
  hdma2d_discovery.LayerCfg[1].InputOffset    = 0;
  hdma2d_discovery.LayerCfg[1].AlphaInverted  = DMA2D_REGULAR_ALPHA;
  hdma2d_discovery.LayerCfg[1].RedBlueSwap    = DMA2D_RB_REGULAR;

  hdma2d_discovery.Instance = DMA2D;

  /* DMA2D Initialization */
  if(HAL_DMA2D_Init(&hdma2d_discovery) == HAL_OK)
  {
    if(HAL_DMA2D_ConfigLayer(&hdma2d_discovery, 1) == HAL_OK)
    {
      if (HAL_DMA2D_Start(&hdma2d_discovery, (uint32_t)pSrc, (uint32_t)pDst, xSize, 1) == HAL_OK)
      {
        /* Polling For DMA transfer */
        HAL_DMA2D_PollForTransfer(&hdma2d_discovery, 10);
      }
    }
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


