/**
  ******************************************************************************
  * @file    stm32l476g_eval_glass_lcd.c
  * @author  MCD Application Team
  * @brief   This file includes the LCD Glass driver for LCD XHO5002B Module of
  *          STM32L476G-EVAL board.
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

/* Includes ------------------------------------------------------------------*/
#include "stm32l476g_eval_glass_lcd.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L476G_EVAL
  * @{
  */

/** @defgroup STM32L476G_EVAL_GLASS_LCD STM32L476G_EVAL GLASS LCD
  * @brief This file includes the LCD Glass driver for LCD_XHO5002B Module of
  *        STM32L476G-EVAL board.
  * @{
  */


/* Private constants ---------------------------------------------------------*/

/** @defgroup STM32L476G_EVAL_GLASS_LCD_Private_Constants Private Constants
  * @{
  */
#define ASCII_CHAR_SPACE              0x20  /*   */
#define ASCII_CHAR_FORWARD_SLASH      0x2F  /* / */
#define ASCII_CHAR_0                  0x30  /* 0 */
#define ASCII_CHAR_COLON              0x3A  /* : */
#define ASCII_CHAR_AT_SYMBOL          0x40  /* @ */
#define ASCII_CHAR_A                  0x41  /* A */
#define ASCII_CHAR_LEFT_OPEN_BRACKET  0x5B  /* [ */
/**
  * @}
  */

/* Private variables ---------------------------------------------------------*/

/** @defgroup STM32L476G_EVAL_GLASS_LCD_Private_Variables Private Variables
  * @{
  */

/**
  @verbatim
================================================================================
                              GLASS LCD MAPPING
================================================================================
               A
       _  ----------
     P|_| |\   |H  /|
         F| G  |  I |B
          |  \ | /  |
          --M-- --N--
          |   /| \  |
         E|  L |  J |C
       _  | /  |K  \|
     Q|_| -----------
              D

A LCD character coding is based on the following matrix:
             COM0    COM1    COM4    COM5     COM6    COM7
  SEG(n)    { 0 ,     0 ,     I ,     B ,      C ,     J }
  SEG(n+1)  { D ,     K ,     A ,     H ,      M ,     N }
  SEG(n+2)  { Q ,     L ,     G ,     F ,      P ,     E }

The character A for example is:
-----------------------------------------------------------
             COM0    COM1     COM4    COM5    COM6     COM7
  SEG(n)    { 0 ,     0 ,      0 ,     1 ,     1 ,      0 }
  SEG(n+1)  { 0 ,     0 ,      1 ,     0 ,     1 ,      1 }
  SEG(n+2)  { 0 ,     0 ,      0 ,     1 ,     0 ,      1 }
   --------------------------------------------------------
           =  0       0        2       5       3        6 hex

   => 'A' = 0x002536

  @endverbatim
  */

LCD_HandleTypeDef LCDHandle;

/**
  * @brief LETTERS AND NUMBERS MAPPING DEFINITION
  */
__IO const uint32_t Mask[] =
  {
    0x00F00000, 0x000F0000, 0x0000F000, 0x00000F00, 0x000000F0, 0x0000000F
  };
const uint8_t Shift[6] =
  {
    20, 16, 12, 8, 4, 0
  };

uint32_t Digit[6];     /* Digit frame buffer */

/* Letters and number map of the custom LCD 8x40(STM8L152D-EVAL evaluation board) */
__IO const uint32_t LetterMap[26] =
  {
    /*  A            B          C           D           E            F   */
    0x00002536, 0x00202536, 0x00202404, 0x00222310, 0x00202426, 0x00002426,
    /*   G            H          I            J           K         L    */
    0x00202416, 0x00000536, 0x00222200, 0x00200114, 0x00001425, 0x00200404,
    /*   M          N           O            P          Q         R      */
    0x00005514, 0x00004515, 0x00202514, 0x00002526, 0x00002532, 0x00002527,
    /*   S           T           U           V           W           X   */
    0x00202432, 0x00022200, 0x00200514, 0x00041404, 0x00040515, 0x00045001,
    /*  Y          Z     */
    0x00025000, 0x00243000
  };

__IO const uint32_t NumberMap[10] =
  {
    /*   0           1           2          3           4   */
    0x00202514, 0x00000110, 0x00202126, 0x00202132, 0x00000532,
    /*    5         6            7          8           9   */
    0x00202432, 0x00202436, 0x00002110, 0x00202536, 0x00202532
  };

/* Constant table for cap characters 'A' --> 'Z' */
const uint16_t CapLetterMap[26]=
{
  /* A       B       C       D       E       F       G       H       I  */
  0xFE00, 0x6714, 0x1D00, 0x4714, 0x9D00, 0x9C00, 0x3F00, 0xFA00, 0x0014,
  /* J       K       L       M       N       O       P       Q       R  */
  0x5300, 0x9841, 0x1900, 0x5A48, 0x5A09, 0x5F00, 0xFC00, 0x5F01, 0xFC01,
  /* S       T       U       V       W       X       Y       Z  */
  0xAF00, 0x0414, 0x5b00, 0x18C0, 0x5A81, 0x00C9, 0x0058, 0x05C0
};

/**
  * @}
  */

/** @defgroup STM32L476G_EVAL_GLASS_LCD_Private_Functions Private Functions
  * @{
  */
static void Convert(uint8_t* c, Point_Typedef Point, DoublePoint_Typedef DoublePoint);
static void LCD_MspInit(LCD_HandleTypeDef *hlcd);

/**
  * @}
  */

/** @addtogroup STM32L476G_EVAL_GLASS_LCD_Exported_Functions
  * @{
  */

/**
  * @brief  Configures the LCD GLASS relative GPIO port IOs and LCD peripheral.
  * @retval None
  */
void BSP_LCD_GLASS_Init(void)
{
  LCDHandle.Instance              = LCD;
  LCDHandle.Init.Prescaler        = LCD_PRESCALER_4;
  LCDHandle.Init.Divider          = LCD_DIVIDER_16;
  LCDHandle.Init.Duty             = LCD_DUTY_1_8;
  LCDHandle.Init.Bias             = LCD_BIAS_1_4;
  LCDHandle.Init.VoltageSource    = LCD_VOLTAGESOURCE_INTERNAL;
  LCDHandle.Init.Contrast         = LCD_CONTRASTLEVEL_7;
  LCDHandle.Init.DeadTime         = LCD_DEADTIME_0;
  LCDHandle.Init.PulseOnDuration  = LCD_PULSEONDURATION_2;
  LCDHandle.Init.HighDrive        = LCD_HIGHDRIVE_DISABLE;
  LCDHandle.Init.BlinkMode        = LCD_BLINKMODE_OFF;
  LCDHandle.Init.BlinkFrequency   = LCD_BLINKFREQUENCY_DIV8;
  LCDHandle.Init.MuxSegment       = LCD_MUXSEGMENT_DISABLE;

  __HAL_LCD_RESET_HANDLE_STATE(&LCDHandle);

  /* Initialize the LCD */
  LCD_MspInit(&LCDHandle);
  HAL_LCD_Init(&LCDHandle);
}

/**
  * @brief  Configures the LCD Blink mode and Blink frequency.
  * @param  BlinkMode: specifies the LCD blink mode.
  *   This parameter can be one of the following values:
  *     @arg LCD_BLINKMODE_OFF:           Blink disabled
  *     @arg LCD_BLINKMODE_SEG0_COM0:     Blink enabled on SEG[0], COM[0] (1 pixel)
  *     @arg LCD_BLINKMODE_SEG0_ALLCOM:   Blink enabled on SEG[0], all COM (up to 8
  *                                       pixels according to the programmed duty)
  *     @arg LCD_BLINKMODE_ALLSEG_ALLCOM: Blink enabled on all SEG and all COM
  *                                       (all pixels)
  * @param  BlinkFrequency: specifies the LCD blink frequency.
  *     @arg LCD_BLINKFREQUENCY_DIV8:    The Blink frequency = fLcd/8
  *     @arg LCD_BLINKFREQUENCY_DIV16:   The Blink frequency = fLcd/16
  *     @arg LCD_BLINKFREQUENCY_DIV32:   The Blink frequency = fLcd/32
  *     @arg LCD_BLINKFREQUENCY_DIV64:   The Blink frequency = fLcd/64
  *     @arg LCD_BLINKFREQUENCY_DIV128:  The Blink frequency = fLcd/128
  *     @arg LCD_BLINKFREQUENCY_DIV256:  The Blink frequency = fLcd/256
  *     @arg LCD_BLINKFREQUENCY_DIV512:  The Blink frequency = fLcd/512
  *     @arg LCD_BLINKFREQUENCY_DIV1024: The Blink frequency = fLcd/1024
  * @retval None
  */
void BSP_LCD_GLASS_BlinkConfig(uint32_t BlinkMode, uint32_t BlinkFrequency)
{
  __HAL_LCD_BLINK_CONFIG(&LCDHandle, BlinkMode, BlinkFrequency);
}

/**
  * @brief  LCD contrast setting
  * @param  Contrast: specifies the LCD Contrast.
  *   This parameter can be one of the following values:
  *     @arg LCD_CONTRASTLEVEL_0: Maximum Voltage = 2.60V
  *     @arg LCD_CONTRASTLEVEL_1: Maximum Voltage = 2.73V
  *     @arg LCD_CONTRASTLEVEL_2: Maximum Voltage = 2.86V
  *     @arg LCD_CONTRASTLEVEL_3: Maximum Voltage = 2.99V
  *     @arg LCD_CONTRASTLEVEL_4: Maximum Voltage = 3.12V
  *     @arg LCD_CONTRASTLEVEL_5: Maximum Voltage = 3.26V
  *     @arg LCD_CONTRASTLEVEL_6: Maximum Voltage = 3.40V
  *     @arg LCD_CONTRASTLEVEL_7: Maximum Voltage = 3.55V
  * @retval None
  */
void BSP_LCD_GLASS_Contrast(uint32_t Contrast)
{
  __HAL_LCD_CONTRAST_CONFIG(&LCDHandle, Contrast);
}

/**
  * @brief This function writes a char in the LCD frame buffer.
  * @param ch: the character to display.
  * @param Point: a point to add in front of char
  *        This parameter can be: POINT_OFF or POINT_ON
  * @param Column: flag indicating if a column has to be add in front
  *        of displayed character.
  *        This parameter can be: DOUBLEPOINT_OFF or DOUBLEPOINT_ON.
  * @param Position: position in the LCD of the character to write [1:7]
  * @retval None
  * @note  Required preconditions: The LCD should be cleared before to start the
  *        write operation.
  */
void BSP_LCD_GLASS_WriteChar(uint8_t* ch, uint8_t Point, uint8_t Column, uint8_t Position)
{
  /* To convert displayed character in segment in array digit */
  Convert(ch, (Point_Typedef)Point, (DoublePoint_Typedef)Column);
  switch(Position)
  {
    /* Position 1 on LCD */
    case 1:
      /* Write Digit 0 on COM0 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER0, 0xFFFFFBFF, (Digit[0] << (uint32_t)0x09));
      /* Write Digit 1 on COM1 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER2, 0xFFFFF3FF, (Digit[1] << (uint32_t)0x09));
      /* Write Digit 2 on COM4 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER8, 0xFFFFF1FF, (Digit[2] << (uint32_t)0x09));
      /* Write Digit 3 on COM5 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER10, 0xFFFFF1FF, (Digit[3] << (uint32_t)0x09));
      /* Write Digit 4 on COM6 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER12, 0xFFFFF9FF, (Digit[4] << (uint32_t)0x09));
      /* Write Digit 5 on COM7 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER14, 0xFFFFF1FF, (Digit[5] << (uint32_t)0x09));
      break;

    /* Position 2 on LCD */
    case 2:
      /* Write Digit 0 on COM0 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER0, 0xFFFF9FFF, (Digit[0] << (uint32_t)0x0C));
      /* Write Digit 1 on COM1 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER2, 0xFFFF9FFF, (Digit[1] << (uint32_t)0x0C));
      /* Write Digit 2 on COM4 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER8, 0xFFFF8FFF, (Digit[2] << (uint32_t)0x0C));
      /* Write Digit 3 on COM5 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER10, 0xFFFF8FFF, (Digit[3] << (uint32_t)0x0C));
      /* Write Digit 4 on COM6 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER12, 0xFFFF8FFF, (Digit[4] << (uint32_t)0x0C));
      /* Write Digit 5 on COM7 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER14, 0xFFFF8FFF, (Digit[5] << (uint32_t)0x0C));
      break;

    /* Position 3 on LCD */
    case 3:
      /* Write Digit 0 on COM0 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER0, 0xFFFCFFFF, (Digit[0] << (uint32_t)0x0F));
      /* Write Digit 1 on COM1 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER2, 0xFFFCFFFF, (Digit[1] << (uint32_t)0x0F));
      /* Write Digit 2 on COM4 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER8, 0xFFFC7FFF, (Digit[2] << (uint32_t)0x0F));
      /* Write Digit 3 on COM5 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER10, 0xFFFC7FFF, (Digit[3] << (uint32_t)0x0F));
      /* Write Digit 4 on COM6 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER12, 0xFFFC7FFF, (Digit[4] << (uint32_t)0x0F));
      /* Write Digit 5 on COM7 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER14, 0xFFFC7FFF, (Digit[5] << (uint32_t)0x0F));
      break;

    /* Position 4 on LCD */
    case 4:
      /* Write Digit 0 on COM0 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER0, 0xFFE7FFFF, (Digit[0] << (uint32_t)0x12));
      /* Write Digit 1 on COM1 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER2, 0xFFE7FFFF, (Digit[1] << (uint32_t)0x12));
      /* Write Digit 2 on COM4 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER8, 0xFFE3FFFF, (Digit[2] << (uint32_t)0x12));
      /* Write Digit 3 on COM5 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER10, 0xFFE3FFFF, (Digit[3] << (uint32_t)0x12));
      /* Write Digit 4 on COM6 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER12, 0xFFE3FFFF, (Digit[4] << (uint32_t)0x12));
      /* Write Digit 5 on COM7 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER14, 0xFFE3FFFF, (Digit[5] << (uint32_t)0x12));
      break;

    /* Position 5 on LCD */
    case 5:
      /* Write Digit 0 on COM0 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER0, 0xFFFFFFF9, Digit[0]);
      /* Write Digit 1 on COM1 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER2, 0xFFFFFFF9, Digit[1]);
      /* Write Digit 2 on COM4 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER8, 0xFFFFFFF8, Digit[2]);
      /* Write Digit 3 on COM5 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER10, 0xFFFFFFF8, Digit[3]);
      /* Write Digit 4 on COM6 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER12, 0xFFFFFFF8, Digit[4]);
      /* Write Digit 5 on COM7 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER14, 0xFFFFFFF8, Digit[5]);
      break;

    /* Position 6 on LCD */
    case 6:
      /* Write Digit 0 on COM0 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER0, 0xFFFFFFCF, (Digit[0] << (uint32_t)0x03));
      /* Write Digit 1 on COM1 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER2, 0xFFFFFFCF, (Digit[1] << (uint32_t)0x03));
      /* Write Digit 2 on COM4 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER8, 0xFFFFFFC7, (Digit[2] << (uint32_t)0x03));
      /* Write Digit 3 on COM5 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER10, 0xFFFFFFC7, (Digit[3] << (uint32_t)0x03));
      /* Write Digit 4 on COM6 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER12, 0xFFFFFFC7, (Digit[4] << (uint32_t)0x03));
      /* Write Digit 5 on COM7 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER14, 0xFFFFFFC7, (Digit[5] << (uint32_t)0x03));
      break;

    /* Position 7 on LCD */
    case 7:
      /* Write Digit 0 on COM0 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER0, 0xFFFFFE7F, (Digit[0] << (uint32_t)0x06));
      /* Write Digit 1 on COM1 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER2, 0xFFFFFE7F, (Digit[1] << (uint32_t)0x06));
      /* Write Digit 2 on COM4 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER8, 0xFFFFFE3F, (Digit[2] << (uint32_t)0x06));
      /* Write Digit 3 on COM5 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER10, 0xFFFFFE3F, (Digit[3] << (uint32_t)0x06));
      /* Write Digit 4 on COM6 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER12, 0xFFFFFE3F, (Digit[4] << (uint32_t)0x06));
      /* Write Digit 5 on COM7 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER14, 0xFFFFFE3F, (Digit[5] << (uint32_t)0x06));
      break;

  default:
    break;
  }

  /* Update the LCD display */
  HAL_LCD_UpdateDisplayRequest(&LCDHandle);
}

/**
  * @brief Setting bar on LCD, writes bar value in LCD frame buffer
  * @param BarLevel: specifies the LCD GLASS Battery Level.
  *     This parameter can be one of the following values:
  *     @arg BATTERYLEVEL_OFF: LCD GLASS Battery Empty
  *     @arg BATTERYLEVEL_1_4: LCD GLASS Battery 1/4 Full
  *     @arg BATTERYLEVEL_1_2: LCD GLASS Battery 1/2 Full
  *     @arg BATTERYLEVEL_3_4: LCD GLASS Battery 3/4 Full
  *     @arg BATTERYLEVEL_FULL: LCD GLASS Battery Full
  * @retval None
  */
void BSP_LCD_GLASS_BarLevelConfig(uint8_t BarLevel)
{
  switch (BarLevel)
  {
  case BATTERYLEVEL_OFF:
    /* Set BATTERYLEVEL_3_4 off  */
    /* Set BATTERYLEVEL_1_4 off  */
    HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER0, 0xFFFFFFF6, 0);
    /* Set BATTERYLEVEL_1_2 off  */
    /* Set BATTERYLEVEL_FULL off  */
    HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER2, 0xFFFFFFF6, 0);
    break;

  /* BARLEVEL 1/4 */
  case BATTERYLEVEL_1_4:
    /* Set BATTERYLEVEL_1_4  on  */
    HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER0, 0xFFFFFFF6, (uint32_t)(0x00000001));
    HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER2, 0xFFFFFFF6, 0);
    break;

  /* BARLEVEL 1/2 */
  case BATTERYLEVEL_1_2:
    /* Set BatteryLevel_1_4 on  */
    HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER0, 0xFFFFFFF6, (uint32_t)(0x00000001));
    /* Set BatteryLevel_1_2 on  */
    HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER2, 0xFFFFFFF6, (uint32_t)(0x00000001));
    break;

  /* Battery Level 3/4 */
  case BATTERYLEVEL_3_4:
    /* Set BATTERYLEVEL_3_4 on  */
    /* Set BatteryLevel_1_4 on  */
    HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER0, 0xFFFFFFF6, (uint32_t)(0x00000001));
    /* Set BatteryLevel_1_2 on  */
    HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER2, 0xFFFFFFF6, (uint32_t)(0x00000009));
    break;

  /* BATTERYLEVEL_FULL */
  case BATTERYLEVEL_FULL:
    /* Set BATTERYLEVEL_3_4 on  */
    /* Set BatteryLevel_1_4 on  */
    HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER0, 0xFFFFFFF6, (uint32_t)(0x00000009));
    /* Set BATTERYLEVEL_FULL on  */
    /* Set BatteryLevel_1_2 on  */
    HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER2, 0xFFFFFFF6, (uint32_t)(0x00000009));
    break;

  default:
    break;
  }

  /* Update the LCD display */
  HAL_LCD_UpdateDisplayRequest(&LCDHandle);
}

/**
  * @brief This function Clear a char in the LCD RAM.
  * @param position: Position in the LCD of the character to write.
  *                  This parameter can be any value in range [1:7].
  * @retval None
  */
void BSP_LCD_GLASS_ClearChar(uint8_t position)
{
  switch (position)
  {
    /* Clear position 1 on LCD */
    case 1:
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER0, 0xFFFFFBFF, 0);
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER2, 0xFFFFF3FF, 0);
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER8, 0xFFFFF1FF, 0);
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER10, 0xFFFFF1FF, 0);
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER12, 0xFFFFF9FF, 0);
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER14, 0xFFFFF1FF, 0);
      break;

    /* Clear position 2 on LCD */
    case 2:
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER0, 0xFFFF9FFF, 0);
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER2, 0xFFFF9FFF, 0);
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER8, 0xFFFF8FFF, 0);
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER10, 0xFFFF8FFF, 0);
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER12, 0xFFFF8FFF, 0);
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER14, 0xFFFF8FFF, 0);
      break;

    /* Clear position 3 on LCD */
    case 3:
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER0, 0xFFFCFFFF, 0);
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER2, 0xFFFCFFFF, 0);
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER8, 0xFFFC7FFF, 0);
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER10, 0xFFFC7FFF, 0);
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER12, 0xFFFC7FFF, 0);
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER14, 0xFFFC7FFF, 0);
      break;

    /* Clear position 4 on LCD */
    case 4:
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER0, 0xFFE7FFFF, 0);
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER2, 0xFFE7FFFF, 0);
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER8, 0xFFE3FFFF, 0);
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER10, 0xFFE3FFFF, 0);
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER12, 0xFFE3FFFF, 0);
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER14, 0xFFE3FFFF, 0);
      break;

    /* Clear position 5 on LCD */
    case 5:
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER0, 0xFFFFFFF9, 0);
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER2, 0xFFFFFFF9, 0);
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER8, 0xFFFFFFF8, 0);
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER10, 0xFFFFFFF8, 0);
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER12, 0xFFFFFFF8, 0);
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER14, 0xFFFFFFF8, 0);
      break;

    /* Clear position 6 on LCD */
    case 6:
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER0, 0xFFFFFFCF, 0);
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER2, 0xFFFFFFCF, 0);
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER8, 0xFFFFFFC7, 0);
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER10, 0xFFFFFFC7, 0);
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER12, 0xFFFFFFC7, 0);
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER14, 0xFFFFFFC7, 0);
      break;

    /* Clear position 7 on LCD */
    case 7:
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER0, 0xFFFFFE7F, 0);
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER2, 0xFFFFFE7F, 0);
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER8, 0xFFFFFE3F, 0);
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER10, 0xFFFFFE3F, 0);
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER12, 0xFFFFFE3F, 0);
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER14, 0xFFFFFE3F, 0);
      break;

    default:
      break;
  }
  /* Update the LCD display */
  HAL_LCD_UpdateDisplayRequest(&LCDHandle);
}

/**
  * @brief  This function writes a char in the LCD RAM.
  * @param  Ch: The character to display.
  * @param  Point: A point to add in front of char.
  *          This parameter  can be one of the following values:
  *              @arg POINT_OFF: No point to add in front of char.
  *              @arg POINT_ON: Add a point in front of char.
  * @param  DoublePoint: Flag indicating if a apostrophe has to be add in front
  *                     of displayed character.
  *          This parameter  can be one of the following values:
  *              @arg DOUBLEPOINT_OFF: No colon to add in back of char.
  *              @arg DOUBLEPOINT_ON: Add an colon in back of char.
  * @param  Position: Position in the LCD of the character to write.
  *                   This parameter can be any value in range [1:7].
  * @retval None
  */
void BSP_LCD_GLASS_DisplayChar(uint8_t* Ch, Point_Typedef Point, DoublePoint_Typedef DoublePoint, uint8_t Position)
{
  /*!< LCD Write Char */
  BSP_LCD_GLASS_WriteChar(Ch, Point, DoublePoint, Position);

  /* Update the LCD display */
  HAL_LCD_UpdateDisplayRequest(&LCDHandle);
}

/**
  * @brief  This function writes a char in the LCD RAM.
  * @param  ptr: Pointer to string to display on the LCD Glass.
  * @retval None
  */
void BSP_LCD_GLASS_DisplayString(uint8_t* ptr)
{
  uint8_t position = 0x01;

  /*!< Send the string character by character on lCD */
  while ((*ptr != 0) && (position < 8))
  {
    /*!< Display one character on LCD */
    BSP_LCD_GLASS_WriteChar(ptr, 0, 0, position);
    /*!< Point on the next character */
    ptr++;
    /*!< Increment the character counter */
    position++;
  }
}

/**
  * @brief  This function Clears the LCD Glass Text Zone.
  * @retval None
  */
void BSP_LCD_GLASS_ClearTextZone(void)
{
  HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER0, 0xFFE49A49, 0);
  HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER2, 0xFFE49A49, 0);
  HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER8, 0xFFE00000, 0);
  HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER10, 0xFFE00000, 0);
  HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER12, 0xFFE00800, 0);
  HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER14, 0xFFE00000, 0);

  /*!< Request LCD RAM update */
  HAL_LCD_UpdateDisplayRequest(&LCDHandle);
}

/**
  * @brief This function writes a char in the LCD RAM.
  * @param ptr: Pointer to string to display on the LCD Glass.
  * @retval None
  * @note Required preconditions: Char is ASCCI value "Ored" with decimal point or Column flag
  */
void BSP_LCD_GLASS_DisplayStrDeci(uint16_t* ptr)
{
  uint8_t index = 1;
  uint8_t tmpchar;

  /* Send the string character by character on lCD */
  while((*ptr != 0) && (index < 8))
  {
    tmpchar = (*ptr) & 0x00FF;

    switch((*ptr) & 0xF000)
    {
    case DOT:
      /* Display one character on LCD with decimal point */
      BSP_LCD_GLASS_WriteChar(&tmpchar, POINT_ON, DOUBLEPOINT_OFF, index);
      break;
    case DOUBLE_DOT:
      /* Display one character on LCD with decimal point */
      BSP_LCD_GLASS_WriteChar(&tmpchar, POINT_OFF, DOUBLEPOINT_ON, index);
      break;
    default:
      BSP_LCD_GLASS_WriteChar(&tmpchar, POINT_OFF, DOUBLEPOINT_OFF, index);
      break;
    }/* Point on the next character */
    ptr++;

    /* Increment the character counter */
    index++;
  }
}

/**
  * @brief  This function Clear the whole LCD RAM.
  * @retval None
  */
void BSP_LCD_GLASS_Clear(void)
{
  HAL_LCD_Clear(&LCDHandle);
}

/**
  * @brief  Display a string in scrolling mode
  * @param  ptr: Pointer to string to display on the LCD Glass.
  * @param  nScroll: Specifies how many time the message will be scrolled
  * @param  ScrollSpeed : Specifies the speed of the scroll, low value gives
  *         higher speed
  * @retval None
  * @note    Required preconditions: The LCD should be cleared before to start the
  *         write operation.
  */
void BSP_LCD_GLASS_ScrollSentence(uint8_t* ptr, uint16_t nScroll, uint16_t ScrollSpeed)
{
  uint8_t repetition = 0, nbrchar = 0, sizestr = 0;
  uint8_t* ptr1;
  uint8_t str[8] = "";

  if(ptr == 0)
  {
    return;
  }

  /* To calculate end of string */
  for(ptr1 = ptr, sizestr = 0; *ptr1 != 0; sizestr++, ptr1++);

  ptr1 = ptr;

  BSP_LCD_GLASS_DisplayString(ptr);
  HAL_Delay(ScrollSpeed);

  /* To shift the string for scrolling display*/
  for (repetition = 0; repetition < nScroll; repetition++)
  {
    for(nbrchar = 0; nbrchar < sizestr; nbrchar++)
    {
      *(str) =* (ptr1+((nbrchar)%sizestr));
      *(str+1) =* (ptr1+((nbrchar+1)%sizestr));
      *(str+2) =* (ptr1+((nbrchar+2)%sizestr));
      *(str+3) =* (ptr1+((nbrchar+3)%sizestr));
      *(str+4) =* (ptr1+((nbrchar+4)%sizestr));
      *(str+5) =* (ptr1+((nbrchar+5)%sizestr));
      *(str+6) =* (ptr1+((nbrchar+6)%sizestr));
      *(str+7) =* (ptr1+((nbrchar+7)%sizestr));
      BSP_LCD_GLASS_ClearTextZone();
      BSP_LCD_GLASS_DisplayString(str);

      HAL_Delay(ScrollSpeed);
    }
  }
}

/**
  * @brief  Configure ST Logo display.
  * @param  NewState: enable or disable the logo display .
  * @retval None
  */
void BSP_LCD_GLASS_DisplayLogo(FunctionalState NewState)
{
  if (NewState != DISABLE)
  {
    /* Set logo on  */
    HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER0, 0xFFFFFFBF, 0x00000040);
  }
  else
  {
    /* Set logo of  */
    HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER0, 0xFFFFFFBF, 0);
  }

  /*!< Request LCD RAM update */
  HAL_LCD_UpdateDisplayRequest(&LCDHandle);
}

/**
  * @brief  Configure the LCD ArrowDirection.
  * @param  ArrowDirection: Specify the ArrowDirection to set.
  * @retval None
  */
void BSP_LCD_GLASS_ArrowConfig(ArrowDirection_TypeDef ArrowDirection)
{
  switch (ArrowDirection)
  {
    /* ARROWDIRECTION_UP */
    case ARROWDIRECTION_UP:
      /* Set ARROWDIRECTION_UP on  */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER0, 0xFFFFEFFF, 0x00001000);
      break;

    /* ARROWDIRECTION_LEFT */
    case ARROWDIRECTION_LEFT :
      /* Set ARROWDIRECTION_LEFT on  */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER2, 0xFFFFEFFF, 0x00001000);
      break;

    /* ARROWDIRECTION_DOWN */
    case ARROWDIRECTION_DOWN:
      /* Set ARROWDIRECTION_DOWN on  */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER2, 0xFFFF7FFF, 0x00008000);
      break;

    /* ARROWDIRECTION_RIGHT */
    case ARROWDIRECTION_RIGHT:
      /* Set ARROWDIRECTION_RIGHT on  */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER0, 0xFFFF7FFF, 0x00008000);
      break;

    case ARROWDIRECTION_OFF:
      /* Set ARROWDIRECTION_UP off  */
      /* Set ARROWDIRECTION_RIGHT off  */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER0, 0xFFFF6FFF, 0);
      /* Set ARROWDIRECTION_LEFT off  */
      /* Set ARROWDIRECTION_DOWN off  */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER2, 0xFFFF6FFF, 0);
      break;

    default:
      break;
  }

  /*!< Request LCD RAM update */
  HAL_LCD_UpdateDisplayRequest(&LCDHandle);
}

/**
  * @brief  Configure the LCD Temperature level display .
  * @param  Temperature: indicate the Temperature level to set.
  * @retval None
  */
void BSP_LCD_GLASS_TemperatureConfig(TemperatureLevel_TypeDef Temperature)
{
  switch (Temperature)
  {
    /* Temperature level 1*/
    case TEMPERATURELEVEL_1 :
      /* Clear Temperature level 5 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER0, 0xFFFFFDFF, 0);
      /* Clear Temperature level 4 and level 2 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, 0xFFFFF3FF, 0);
      /* Clear Temperature level 1 and level 3 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, 0xFFFFF3FF, 0);
      /* Set Temperature level 1*/
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER2, 0xFFFFFDFF, 0x00000200);
      break;

    /* Temperature level 2 */
    case TEMPERATURELEVEL_2:
      /* Clear Temperature level 4 and level 2 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, 0xFFFFF3FF, 0);
      /* Clear Temperature level 1 and level 3 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, 0xFFFFF3FF, 0);
      /* Set Temperature level 1 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER2, 0xFFFFFDFF, 0x00000200);
      /* Set Temperature level 2 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER0, 0xFFFFFDFF, 0x00000200);
      break;

      /* Temperature level 3 */
    case TEMPERATURELEVEL_3:
      /* Clear Temperature level 1 and level 3 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, 0xFFFFF3FF, 0);
      /* Set Temperature level 1 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER2, 0xFFFFFDFF, 0x00000200);
      /* Set Temperature level 2 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER0, 0xFFFFFDFF, 0x00000200);
      /* Set Temperature level 3 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, 0xFFFFF3FF, 0x00000400);
      break;

      /* Temperature level 4*/
    case TEMPERATURELEVEL_4:
      /* Set Temperature level 1  */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER2, 0xFFFFFDFF, 0x00000200);
      /* Set Temperature level 2  */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER0, 0xFFFFFDFF, 0x00000200);
      /* Set Temperature level 3 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, 0xFFFFF3FF, 0x00000400);
      /* Set Temperature level 4 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, 0xFFFFF3FF, 0x00000400);
      break;

      /* Temperature_2*/
    case TEMPERATURELEVEL_5:
      /* Set Temperature level 1 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER2, 0xFFFFFDFF, 0x00000200);
      /* Set Temperature level 2 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER0, 0xFFFFFDFF, 0x00000200);
      /* Set Temperature level 3 and level 5 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, 0xFFFFF3FF, 0x00000C00);
      /* Set Temperature level 4 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, 0xFFFFF3FF, 0x00000400);
      break;

      /* Temperature_6*/
    case  TEMPERATURELEVEL_6:
      /* Set Temperature level 1 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER2, 0xFFFFFDFF, 0x00000200);
      /* Set Temperature level 2 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER0, 0xFFFFFDFF, 0x00000200);
      /* Set Temperature level 3 and level 5 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, 0xFFFFF3FF, 0x00000C00);
      /* Set Temperature level 4 and level 6 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, 0xFFFFF3FF, 0x00000C00);
      break;

    case TEMPERATURELEVEL_OFF:
      /* Clear Temperature level 5 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER0, 0xFFFFFDFF, 0);
      /* Clear Temperature level 6 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER2, 0xFFFFFDFF, 0);
      /* Clear Temperature level 4 and level 2 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, 0xFFFFF3FF, 0);
      /* Clear Temperature level 1 and level 3 */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, 0xFFFFF3FF, 0);
      break;

    default:
      break;
  }

  /*!< Request LCD RAM update */
  HAL_LCD_UpdateDisplayRequest(&LCDHandle);
}

/**
  * @brief  Configure the LCD Value Unit.
  * @param  ValueUnit: indicate the Value Unit to set.
  * @retval None
  */
void BSP_LCD_GLASS_ValueUnitConfig(ValueUnit_TypeDef ValueUnit)
{
  switch (ValueUnit)
  {
    /* VALUEUNIT MILLIAMPERE*/
    case VALUEUNIT_MILLIAMPERE:
      /* Set VALUEUNIT_MILLIAMPERE on  */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER2, 0xFFFBFFFF, 0x00040000);
      break;

    /* VALUEUNIT MICROAMPERE)*/
    case VALUEUNIT_MICROAMPERE:
      /* Set VALUEUNIT_MICROAMPERE  */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER0, 0xFFFBFFFF, 0x00040000);
      break;

    /* VALUEUNIT NANOAMPERE*/
    case  VALUEUNIT_NANOAMPERE:
      /* Set VALUEUNIT_NANOAMPERE on  */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER2, 0xFFFFFFBF, 0x00000040);
      break;

    case  VALUEUNIT_OFF:
      /* Set VALUEUNIT_MILLIAMPERE and VALUEUNIT_NANOAMPERE OFF */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER2, 0xFFFBFFBF, 0);
      /* Set VALUEUNIT_MICROAMPERE off  */
      HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER0, 0xFFFBFFFF, 0);
      break;

    default:
      break;
  }

  /*!< Request LCD RAM update */
  HAL_LCD_UpdateDisplayRequest(&LCDHandle);
}

/**
  * @brief  Configure the LCD sign.
  * @param  Sign: indicate the sign to set.
  * @param  NewState: enable or disable the Digit.
  * @retval None
  */
void BSP_LCD_GLASS_SignCmd(Sign_TypeDef Sign, FunctionalState NewState)
{
  if (NewState != DISABLE)
  {
    switch (Sign)
    {
      /* Sign positive */
      case SIGN_POSITIVE:
        /* Set SIGN_POSITIVE on */
        HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER12, 0xFFFFF7FF, 0x00000800);
        break;

      /* Sign negative*/
      case SIGN_NEGATIVE:
        /* Set SIGN_NEGATIVE on */
        HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER0, 0xFFFFF7FF, 0x00000800);
        break;

      default:
        break;
    }
  }
  else
  {
    switch (Sign)
    {
      /* Sign positive */
      case SIGN_POSITIVE:
        /* Set SIGN_POSITIVE off */
        HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER12, 0xFFFFF7FF, 0);
        break;

      /* Sign negative */
      case SIGN_NEGATIVE:
        /* Set SIGN_NEGATIVE off */
        HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER0, 0xFFFFF7FF, 0);
        break;

      default:
        break;
    }
  }

  /*!< Request LCD RAM update */
  HAL_LCD_UpdateDisplayRequest(&LCDHandle);
}
/**
  * @brief  Set Matrix Pixel on.
  * @param  PixelRow: Specify Matrix Row.
  * @param  PixelColumn: Specify Matrix Column.
  * @retval None
  */
void BSP_LCD_GLASS_WriteMatrixPixel(PixelRow_TypeDef PixelRow, PixelColumn_TypeDef PixelColumn)
{
  switch (PixelRow)
  {
    case PIXELROW_1:
      switch (PixelColumn)
      {
        case PIXELCOLUMN_1:
          /* Position : Row = 1 , Column= 1 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, ~((uint32_t)0x00000200), (uint32_t)0x00000200);
          break;

        case PIXELCOLUMN_2:
          /* Position : Row = 1 , Column= 2 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, ~((uint32_t)0x00004000), (uint32_t)0x00004000);
          break;

        case PIXELCOLUMN_3:
          /* Position : Row = 1 , Column= 3 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, ~((uint32_t)0x00002000), (uint32_t)0x00002000);
          break;

        case PIXELCOLUMN_4:
          /* Position : Row = 1 , Column= 4 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, ~((uint32_t)0x00001000), (uint32_t)0x00001000);
          break;

        case PIXELCOLUMN_5:
          /* Position : Row = 1 , Column= 5 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, ~((uint32_t)0x00020000), (uint32_t)0x00020000);
          break;

        case PIXELCOLUMN_6:
          /* Position : Row = 1 , Column= 6 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, ~((uint32_t)0x00010000), (uint32_t)0x00010000);
          break;

        case PIXELCOLUMN_7:
          /* Position : Row = 1 , Column= 7 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, ~((uint32_t)0x00008000), (uint32_t)0x00008000);
          break;

        case PIXELCOLUMN_8:
          /* Position : Row = 1 , Column= 8 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, ~((uint32_t)0x00100000), (uint32_t)0x00100000);
          break;

        case PIXELCOLUMN_9:
          /* Position : Row = 1 , Column= 9 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, ~((uint32_t)0x00080000), (uint32_t)0x00080000);
          break;

        case PIXELCOLUMN_10:
          /* Position : Row = 1 , Column= 10 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, ~((uint32_t)0x00040000), (uint32_t)0x00040000);
          break;

        case PIXELCOLUMN_11:
          /* Position : Row = 1 , Column= 11 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, ~((uint32_t)0x00000004), (uint32_t)0x00000004);
          break;

        case PIXELCOLUMN_12:
          /* Position : Row = 1 , Column= 12 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, ~((uint32_t)0x00000002), (uint32_t)0x00000002);
          break;

        case PIXELCOLUMN_13:
          /* Position : Row = 1 , Column= 13 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, ~((uint32_t)0x00000001), (uint32_t)0x00000001);
          break;

        case PIXELCOLUMN_14:
          /* Position : Row = 1 , Column= 14 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, ~((uint32_t)0x00000020), (uint32_t)0x00000020);
          break;

        case PIXELCOLUMN_15:
          /* Position : Row = 1 , Column= 15 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, ~((uint32_t)0x00000010), (uint32_t)0x00000010);
          break;

        case PIXELCOLUMN_16:
          /* Position : Row = 1 , Column= 16 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, ~((uint32_t)0x00000008), (uint32_t)0x00000008);
          break;

        case PIXELCOLUMN_17:
          /* Position : Row = 1 , Column= 17 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, ~((uint32_t)0x00000100), (uint32_t)0x00000100);
          break;

        case PIXELCOLUMN_18:
          /* Position : Row = 1 , Column= 18 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, ~((uint32_t)0x00000080), (uint32_t)0x00000080);
          break;

        case PIXELCOLUMN_19:
          /* Position : Row = 1 , Column= 19 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, ~((uint32_t)0x00000040), (uint32_t)0x00000040);
          break;

        default:
          break;

      }
      break;

    case PIXELROW_2:
      switch (PixelColumn)
      {
        case PIXELCOLUMN_1:
          /* Position : Row = 2 , Column= 1 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, ~((uint32_t)0x00000200), (uint32_t)0x00000200);
          break;

        case PIXELCOLUMN_2:
          /* Position : Row = 2 , Column= 2 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, ~((uint32_t)0x00004000), (uint32_t)0x00004000);
          break;

        case PIXELCOLUMN_3:
          /* Position : Row = 2 , Column= 3 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, ~((uint32_t)0x00002000), (uint32_t)0x00002000);
          break;

        case PIXELCOLUMN_4:
          /* Position : Row = 2 , Column= 4 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, ~((uint32_t)0x00001000), (uint32_t)0x00001000);
          break;

        case PIXELCOLUMN_5:
          /* Position : Row = 2 , Column= 5 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, ~((uint32_t)0x00020000), (uint32_t)0x00020000);
          break;

        case PIXELCOLUMN_6:
          /* Position : Row = 2 , Column= 6 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, ~((uint32_t)0x00010000), (uint32_t)0x00010000);
          break;

        case PIXELCOLUMN_7:
          /* Position : Row = 2 , Column= 7 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, ~((uint32_t)0x00008000), (uint32_t)0x00008000);
          break;

        case PIXELCOLUMN_8:
          /* Position : Row = 2 , Column= 8 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, ~((uint32_t)0x00100000), (uint32_t)0x00100000);
          break;

        case PIXELCOLUMN_9:
          /* Position : Row = 2 , Column= 9 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, ~((uint32_t)0x00080000), (uint32_t)0x00080000);
          break;

        case PIXELCOLUMN_10:
          /* Position : Row = 2 , Column= 10 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, ~((uint32_t)0x00040000), (uint32_t)0x00040000);
          break;

        case PIXELCOLUMN_11:
          /* Position : Row = 2 , Column= 11 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, ~((uint32_t)0x00000004), (uint32_t)0x00000004);
          break;

        case PIXELCOLUMN_12:
          /* Position : Row = 2 , Column= 12 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, ~((uint32_t)0x00000002), (uint32_t)0x00000002);
          break;

        case PIXELCOLUMN_13:
          /* Position : Row = 2 , Column= 13 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, ~((uint32_t)0x00000001), (uint32_t)0x00000001);
          break;

        case PIXELCOLUMN_14:
          /* Position : Row = 2 , Column= 14 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, ~((uint32_t)0x00000020), (uint32_t)0x00000020);
          break;

        case PIXELCOLUMN_15:
          /* Position : Row = 2 , Column= 15 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, ~((uint32_t)0x00000010), (uint32_t)0x00000010);
          break;

        case PIXELCOLUMN_16:
          /* Position : Row = 2 , Column= 16 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, ~((uint32_t)0x00000008), (uint32_t)0x00000008);
          break;

        case PIXELCOLUMN_17:
          /* Position : Row = 2 , Column= 17 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, ~((uint32_t)0x00000100), (uint32_t)0x00000100);
          break;

        case PIXELCOLUMN_18:
          /* Position : Row = 2 , Column= 18 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, ~((uint32_t)0x00000080), (uint32_t)0x00000080);
          break;

        case PIXELCOLUMN_19:
          /* Position : Row = 2 , Column= 19 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, ~((uint32_t)0x00000040), (uint32_t)0x00000040);
          break;

        default:
          break;
      }
      break;

    case PIXELROW_3:
      switch (PixelColumn)
      {
        case PIXELCOLUMN_1:
          /* Position : Row = 3 , Column= 1 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, ~((uint32_t)0x00200000), (uint32_t)0x00200000);
          break;

        case PIXELCOLUMN_2:
          /* Position : Row = 3 , Column= 2 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, ~((uint32_t)0x00400000), (uint32_t)0x00400000);
          break;

        case PIXELCOLUMN_3:
          /* Position : Row = 3 , Column= 3 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, ~((uint32_t)0x00800000), (uint32_t)0x00800000);
          break;

        case PIXELCOLUMN_4:
          /* Position : Row = 3 , Column= 4 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, ~((uint32_t)0x01000000), (uint32_t)0x01000000);
          break;

        case PIXELCOLUMN_5:
          /* Position : Row = 3 , Column= 5 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, ~((uint32_t)0x02000000), (uint32_t)0x02000000);
          break;

        case PIXELCOLUMN_6:
          /* Position : Row = 3 , Column= 6 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, ~((uint32_t)0x04000000), (uint32_t)0x04000000);
          break;

        case PIXELCOLUMN_7:
          /* Position : Row = 3 , Column= 7 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, ~((uint32_t)0x08000000), (uint32_t)0x08000000);
          break;

        case PIXELCOLUMN_8:
          /* Position : Row = 3 , Column= 8 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, ~((uint32_t)0x10000000), (uint32_t)0x10000000);
          break;

        case PIXELCOLUMN_9:
          /* Position : Row = 3 , Column= 9*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, ~((uint32_t)0x20000000), (uint32_t)0x20000000);
          break;

        case PIXELCOLUMN_10:
          /* Position : Row = 3 , Column= 10*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, ~((uint32_t)0x40000000), (uint32_t)0x40000000);
          break;

        case PIXELCOLUMN_11:
          /* Position : Row = 3 , Column= 11*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, ~((uint32_t)0x80000000), (uint32_t)0x80000000);
          break;

        case PIXELCOLUMN_12:
          /* Position : Row = 3 , Column= 12*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER5, ~((uint32_t)0x00000001), (uint32_t)0x00000001);
          break;

        case PIXELCOLUMN_13:
          /* Position : Row = 3 , Column= 13*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER5, ~((uint32_t)0x00000002), (uint32_t)0x00000002);
          break;

        case PIXELCOLUMN_14:
          /* Position : Row = 3 , Column= 14*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER5, ~((uint32_t)0x00000004), (uint32_t)0x00000004);
          break;

        case PIXELCOLUMN_15:
          /* Position : Row = 3 , Column= 15*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER5, ~((uint32_t)0x00000008), (uint32_t)0x00000008);
          break;

        case PIXELCOLUMN_16:
          /* Position : Row = 3 , Column= 16*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER5, ~((uint32_t)0x00000010), (uint32_t)0x00000010);
          break;

        case PIXELCOLUMN_17:
          /* Position : Row = 3 , Column= 17*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER5, ~((uint32_t)0x00000020), (uint32_t)0x00000020);
          break;

        case PIXELCOLUMN_18:
          /* Position : Row = 3 , Column= 18*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER5, ~((uint32_t)0x00000040), (uint32_t)0x00000040);
          break;

        case PIXELCOLUMN_19:
          /* Position : Row = 3 , Column= 19 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER5, ~((uint32_t)0x00000080), (uint32_t)0x00000080);
          break;

        default:
          break;
      }
      break;

    case PIXELROW_4:
      switch (PixelColumn)
      {
        case PIXELCOLUMN_1:
          /* Position : Row = 4 , Column= 1 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, ~((uint32_t)0x00200000), (uint32_t)0x00200000);
          break;

        case PIXELCOLUMN_2:
          /* Position : Row = 4 , Column= 2 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, ~((uint32_t)0x00400000), (uint32_t)0x00400000);
          break;

        case PIXELCOLUMN_3:
          /* Position : Row = 4 , Column= 3 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, ~((uint32_t)0x00800000), (uint32_t)0x00800000);
          break;

        case PIXELCOLUMN_4:
          /* Position : Row = 4 , Column= 4 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, ~((uint32_t)0x01000000), (uint32_t)0x01000000);
          break;

        case PIXELCOLUMN_5:
          /* Position : Row = 4 , Column= 5 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, ~((uint32_t)0x02000000), (uint32_t)0x02000000);
          break;

        case PIXELCOLUMN_6:
          /* Position : Row = 4 , Column= 6 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, ~((uint32_t)0x04000000), (uint32_t)0x04000000);
          break;

        case PIXELCOLUMN_7:
          /* Position : Row = 4 , Column= 7 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, ~((uint32_t)0x08000000), (uint32_t)0x08000000);
          break;

        case PIXELCOLUMN_8:
          /* Position : Row = 4 , Column= 8 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, ~((uint32_t)0x10000000), (uint32_t)0x10000000);
          break;

        case PIXELCOLUMN_9:
          /* Position : Row = 4 , Column= 9*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, ~((uint32_t)0x20000000), (uint32_t)0x20000000);
          break;

        case PIXELCOLUMN_10:
          /* Position : Row = 4 , Column= 10*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, ~((uint32_t)0x40000000), (uint32_t)0x40000000);
          break;

        case PIXELCOLUMN_11:
          /* Position : Row = 4 , Column= 11*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, ~((uint32_t)0x80000000), (uint32_t)0x80000000);
          break;

        case PIXELCOLUMN_12:
          /* Position : Row = 4 , Column= 12*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER7, ~((uint32_t)0x00000001), (uint32_t)0x00000001);
          break;

        case PIXELCOLUMN_13:
          /* Position : Row = 4 , Column= 13*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER7, ~((uint32_t)0x00000002), (uint32_t)0x00000002);
          break;

        case PIXELCOLUMN_14:
          /* Position : Row = 4 , Column= 14*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER7, ~((uint32_t)0x00000004), (uint32_t)0x00000004);
          break;

        case PIXELCOLUMN_15:
          /* Position : Row = 4 , Column= 15*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER7, ~((uint32_t)0x00000008), (uint32_t)0x00000008);
          break;

        case PIXELCOLUMN_16:
          /* Position : Row = 4 , Column= 16*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER7, ~((uint32_t)0x00000010), (uint32_t)0x00000010);
          break;

        case PIXELCOLUMN_17:
          /* Position : Row = 4 , Column= 17*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER7, ~((uint32_t)0x00000020), (uint32_t)0x00000020);
          break;

        case PIXELCOLUMN_18:
          /* Position : Row = 4 , Column= 18*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER7, ~((uint32_t)0x00000040), (uint32_t)0x00000040);
          break;

        case PIXELCOLUMN_19:
          /* Position : Row = 4 , Column= 19 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER7, ~((uint32_t)0x00000080), (uint32_t)0x00000080);
          break;

        default:
          break;

      }
      break;

    case PIXELROW_5:
      switch (PixelColumn)
      {
        case PIXELCOLUMN_1:
          /* Position : Row = 5 , Column= 1 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER0, ~((uint32_t)0x00200000), (uint32_t)0x00200000);
          break;

        case PIXELCOLUMN_2:
          /* Position : Row = 5 , Column= 2 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER0, ~((uint32_t)0x00400000), (uint32_t)0x00400000);
          break;

        case PIXELCOLUMN_3:
          /* Position : Row = 5 , Column= 3 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER0, ~((uint32_t)0x00800000), (uint32_t)0x00800000);
          break;

        case PIXELCOLUMN_4:
          /* Position : Row = 5 , Column= 4 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER0, ~((uint32_t)0x01000000), (uint32_t)0x01000000);
          break;

        case PIXELCOLUMN_5:
          /* Position : Row = 5 , Column= 5 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER0, ~((uint32_t)0x02000000), (uint32_t)0x02000000);
          break;

        case PIXELCOLUMN_6:
          /* Position : Row = 5 , Column= 6 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER0, ~((uint32_t)0x04000000), (uint32_t)0x04000000);
          break;

        case PIXELCOLUMN_7:
          /* Position : Row = 5 , Column= 7 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER0, ~((uint32_t)0x08000000), (uint32_t)0x08000000);
          break;

        case PIXELCOLUMN_8:
          /* Position : Row = 5 , Column= 8 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER0, ~((uint32_t)0x10000000), (uint32_t)0x10000000);
          break;

        case PIXELCOLUMN_9:
          /* Position : Row = 5 , Column= 9*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER0, ~((uint32_t)0x20000000), (uint32_t)0x20000000);
          break;

        case PIXELCOLUMN_10:
          /* Position : Row = 5 , Column= 10*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER0, ~((uint32_t)0x40000000), (uint32_t)0x40000000);
          break;

        case PIXELCOLUMN_11:
          /* Position : Row = 5 , Column= 11*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER0, ~((uint32_t)0x80000000), (uint32_t)0x80000000);
          break;

        case PIXELCOLUMN_12:
          /* Position : Row = 5 , Column= 12*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER1, ~((uint32_t)0x00000001), (uint32_t)0x00000001);
          break;

        case PIXELCOLUMN_13:
          /* Position : Row = 5 , Column= 13*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER1, ~((uint32_t)0x00000002), (uint32_t)0x00000002);
          break;

        case PIXELCOLUMN_14:
          /* Position : Row = 5 , Column= 14*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER1, ~((uint32_t)0x00000004), (uint32_t)0x00000004);
          break;

        case PIXELCOLUMN_15:
          /* Position : Row = 5 , Column= 15*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER1, ~((uint32_t)0x00000008), (uint32_t)0x00000008);
          break;

        case PIXELCOLUMN_16:
          /* Position : Row = 5 , Column= 16*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER1, ~((uint32_t)0x00000010), (uint32_t)0x00000010);
          break;

        case PIXELCOLUMN_17:
          /* Position : Row = 5 , Column= 17*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER1, ~((uint32_t)0x00000020), (uint32_t)0x00000020);
          break;

        case PIXELCOLUMN_18:
          /* Position : Row = 5 , Column= 18*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER1, ~((uint32_t)0x00000040), (uint32_t)0x00000040);
          break;

        case PIXELCOLUMN_19:
          /* Position : Row = 5 , Column= 19 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER1, ~((uint32_t)0x00000080), (uint32_t)0x00000080);
          break;

        default:
          break;
      }
      break;

    case PIXELROW_6:
      switch (PixelColumn)
      {
        case PIXELCOLUMN_1:
          /* Position : Row = 6 , Column= 1 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER2, ~((uint32_t)0x00200000), (uint32_t)0x00200000);
          break;

        case PIXELCOLUMN_2:
          /* Position : Row = 6 , Column= 2 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER2, ~((uint32_t)0x00400000), (uint32_t)0x00400000);
          break;

        case PIXELCOLUMN_3:
          /* Position : Row = 6 , Column= 3 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER2, ~((uint32_t)0x00800000), (uint32_t)0x00800000);
          break;

        case PIXELCOLUMN_4:
          /* Position : Row = 6 , Column= 4 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER2, ~((uint32_t)0x01000000), (uint32_t)0x01000000);
          break;

        case PIXELCOLUMN_5:
          /* Position : Row = 6 , Column= 5 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER2, ~((uint32_t)0x02000000), (uint32_t)0x02000000);
          break;

        case PIXELCOLUMN_6:
          /* Position : Row = 6 , Column= 6 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER2, ~((uint32_t)0x04000000), (uint32_t)0x04000000);
          break;

        case PIXELCOLUMN_7:
          /* Position : Row = 6 , Column= 7 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER2, ~((uint32_t)0x08000000), (uint32_t)0x08000000);
          break;

        case PIXELCOLUMN_8:
          /* Position : Row = 6 , Column= 8 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER2, ~((uint32_t)0x10000000), (uint32_t)0x10000000);
          break;

        case PIXELCOLUMN_9:
          /* Position : Row = 6 , Column= 9*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER2, ~((uint32_t)0x20000000), (uint32_t)0x20000000);
          break;

        case PIXELCOLUMN_10:
          /* Position : Row = 6 , Column= 10*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER2, ~((uint32_t)0x40000000), (uint32_t)0x40000000);
          break;

        case PIXELCOLUMN_11:
          /* Position : Row = 6 , Column= 11*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER2, ~((uint32_t)0x80000000), (uint32_t)0x80000000);
          break;

        case PIXELCOLUMN_12:
          /* Position : Row = 6 , Column= 12*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER3, ~((uint32_t)0x00000001), (uint32_t)0x00000001);
          break;

        case PIXELCOLUMN_13:
          /* Position : Row = 6 , Column= 13*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER3, ~((uint32_t)0x00000002), (uint32_t)0x00000002);
          break;

        case PIXELCOLUMN_14:
          /* Position : Row = 6 , Column= 14*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER3, ~((uint32_t)0x00000004), (uint32_t)0x00000004);
          break;

        case PIXELCOLUMN_15:
          /* Position : Row = 6 , Column= 15*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER3, ~((uint32_t)0x00000008), (uint32_t)0x00000008);
          break;

        case PIXELCOLUMN_16:
          /* Position : Row = 6 , Column= 16*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER3, ~((uint32_t)0x00000010), (uint32_t)0x00000010);
          break;

        case PIXELCOLUMN_17:
          /* Position : Row = 6 , Column= 17*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER3, ~((uint32_t)0x00000020), (uint32_t)0x00000020);
          break;

        case PIXELCOLUMN_18:
          /* Position : Row = 6 , Column= 18*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER3, ~((uint32_t)0x00000040), (uint32_t)0x00000040);
          break;

        case PIXELCOLUMN_19:
          /* Position : Row = 6 , Column= 19 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER3, ~((uint32_t)0x00000080), (uint32_t)0x00000080);
          break;

        default:
          break;
      }
      break;

    case PIXELROW_7:
      switch (PixelColumn)
      {
        case PIXELCOLUMN_1:
          /* Position : Row = 7 , Column =1 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER14, ~((uint32_t)0x00200000), (uint32_t)0x00200000);
          break;

        case PIXELCOLUMN_2:
          /* Position : Row = 7 , Column =2 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER14, ~((uint32_t)0x00400000), (uint32_t)0x00400000);
          break;

        case PIXELCOLUMN_3:
          /* Position : Row = 7 , Column =3 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER14, ~((uint32_t)0x00800000), (uint32_t)0x00800000);
          break;

        case PIXELCOLUMN_4:
          /* Position : Row = 7 , Column =4 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER14, ~((uint32_t)0x01000000), (uint32_t)0x01000000);
          break;

        case PIXELCOLUMN_5:
          /* Position : Row = 7 , Column =5 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER14, ~((uint32_t)0x02000000), (uint32_t)0x02000000);
          break;

        case PIXELCOLUMN_6:
          /* Position : Row = 7 , Column =6 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER14, ~((uint32_t)0x04000000), (uint32_t)0x04000000);
          break;

        case PIXELCOLUMN_7:
          /* Position : Row = 7 , Column =7 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER14, ~((uint32_t)0x08000000), (uint32_t)0x08000000);
          break;

        case PIXELCOLUMN_8:
          /* Position : Row = 7 , Column =8 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER14, ~((uint32_t)0x10000000), (uint32_t)0x10000000);
          break;

        case PIXELCOLUMN_9:
          /* Position : Row = 7 , Column =9*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER14, ~((uint32_t)0x20000000), (uint32_t)0x20000000);
          break;

        case PIXELCOLUMN_10:
          /* Position : Row = 7 , Column =10*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER14, ~((uint32_t)0x40000000), (uint32_t)0x40000000);
          break;

        case PIXELCOLUMN_11:
          /* Position : Row = 7 , Column =11*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER14, ~((uint32_t)0x80000000), (uint32_t)0x80000000);
          break;

        case PIXELCOLUMN_12:
          /* Position : Row = 7 , Column =12*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER15, ~((uint32_t)0x00000001), (uint32_t)0x00000001);
          break;

        case PIXELCOLUMN_13:
          /* Position : Row = 7 , Column =13*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER15, ~((uint32_t)0x00000002), (uint32_t)0x00000002);
          break;

        case PIXELCOLUMN_14:
          /* Position : Row = 7 , Column =14*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER15, ~((uint32_t)0x00000004), (uint32_t)0x00000004);
          break;

        case PIXELCOLUMN_15:
          /* Position : Row = 7 , Column =15*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER15, ~((uint32_t)0x00000008), (uint32_t)0x00000008);
          break;

        case PIXELCOLUMN_16:
          /* Position : Row = 7 , Column =16*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER15, ~((uint32_t)0x00000010), (uint32_t)0x00000010);
          break;

        case PIXELCOLUMN_17:
          /* Position : Row = 7 , Column =17*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER15, ~((uint32_t)0x00000020), (uint32_t)0x00000020);
          break;

        case PIXELCOLUMN_18:
          /* Position : Row = 7 , Column =18*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER15, ~((uint32_t)0x00000040), (uint32_t)0x00000040);
          break;

        case PIXELCOLUMN_19:
          /* Position : Row = 7 , Column =19 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER15, ~((uint32_t)0x00000080), (uint32_t)0x00000080);
          break;

        default:
          break;
      }
      break;

    case PIXELROW_8:
      switch (PixelColumn)
      {
        case PIXELCOLUMN_1:
          /* Position : Row = 8 , Column =1 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER12, ~((uint32_t)0x00200000), (uint32_t)0x00200000);
          break;

        case PIXELCOLUMN_2:
          /* Position : Row = 8 , Column =2 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER12, ~((uint32_t)0x00400000), (uint32_t)0x00400000);
          break;

        case PIXELCOLUMN_3:
          /* Position : Row = 8 , Column =3 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER12, ~((uint32_t)0x00800000), (uint32_t)0x00800000);
          break;

        case PIXELCOLUMN_4:
          /* Position : Row = 8 , Column =4 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER12, ~((uint32_t)0x01000000), (uint32_t)0x01000000);
          break;

        case PIXELCOLUMN_5:
          /* Position : Row = 8 , Column =5 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER12, ~((uint32_t)0x02000000), (uint32_t)0x02000000);
          break;

        case PIXELCOLUMN_6:
          /* Position : Row = 8 , Column =6 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER12, ~((uint32_t)0x04000000), (uint32_t)0x04000000);
          break;

        case PIXELCOLUMN_7:
          /* Position : Row = 8 , Column =7 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER12, ~((uint32_t)0x08000000), (uint32_t)0x08000000);
          break;

        case PIXELCOLUMN_8:
          /* Position : Row = 8 , Column =8 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER12, ~((uint32_t)0x10000000), (uint32_t)0x10000000);
          break;

        case PIXELCOLUMN_9:
          /* Position : Row = 8 , Column =9*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER12, ~((uint32_t)0x20000000), (uint32_t)0x20000000);
          break;

        case PIXELCOLUMN_10:
          /* Position : Row = 8 , Column =10*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER12, ~((uint32_t)0x40000000), (uint32_t)0x40000000);
          break;

        case PIXELCOLUMN_11:
          /* Position : Row = 8 , Column =11*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER12, ~((uint32_t)0x80000000), (uint32_t)0x80000000);
          break;

        case PIXELCOLUMN_12:
          /* Position : Row = 8 , Column =12*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER13, ~((uint32_t)0x00000001), (uint32_t)0x00000001);
          break;

        case PIXELCOLUMN_13:
          /* Position : Row = 8 , Column =13*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER13, ~((uint32_t)0x00000002), (uint32_t)0x00000002);
          break;

        case PIXELCOLUMN_14:
          /* Position : Row = 8 , Column =14*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER13, ~((uint32_t)0x00000004), (uint32_t)0x00000004);
          break;

        case PIXELCOLUMN_15:
          /* Position : Row = 8 , Column =15*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER13, ~((uint32_t)0x00000008), (uint32_t)0x00000008);
          break;

        case PIXELCOLUMN_16:
          /* Position : Row = 8 , Column =16*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER13, ~((uint32_t)0x00000010), (uint32_t)0x00000010);
          break;

        case PIXELCOLUMN_17:
          /* Position : Row = 8 , Column =17*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER13, ~((uint32_t)0x00000020), (uint32_t)0x00000020);
          break;

        case PIXELCOLUMN_18:
          /* Position : Row = 8 , Column =18*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER13, ~((uint32_t)0x00000040), (uint32_t)0x00000040);
          break;

        case PIXELCOLUMN_19:
          /* Position : Row = 8 , Column =19 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER13, ~((uint32_t)0x00000080), (uint32_t)0x00000080);
          break;

        default:
          break;
      }
      break;

    case PIXELROW_9:
      switch (PixelColumn)
      {
        case PIXELCOLUMN_1:
          /* Position : Row = 9 , Column =1 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER10, ~((uint32_t)0x00200000), (uint32_t)0x00200000);
          break;

        case PIXELCOLUMN_2:
          /* Position : Row = 9 , Column =2 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER10, ~((uint32_t)0x00400000), (uint32_t)0x00400000);
          break;

        case PIXELCOLUMN_3:
          /* Position : Row = 9 , Column =3 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER10, ~((uint32_t)0x00800000), (uint32_t)0x00800000);
          break;

        case PIXELCOLUMN_4:
          /* Position : Row = 9 , Column =4 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER10, ~((uint32_t)0x01000000), (uint32_t)0x01000000);
          break;

        case PIXELCOLUMN_5:
          /* Position : Row = 9 , Column =5 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER10, ~((uint32_t)0x02000000), (uint32_t)0x02000000);
          break;

        case PIXELCOLUMN_6:
          /* Position : Row = 9 , Column =6 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER10, ~((uint32_t)0x04000000), (uint32_t)0x04000000);
          break;

        case PIXELCOLUMN_7:
          /* Position : Row = 9 , Column =7 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER10, ~((uint32_t)0x08000000), (uint32_t)0x08000000);
          break;

        case PIXELCOLUMN_8:
          /* Position : Row = 9 , Column =8 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER10, ~((uint32_t)0x10000000), (uint32_t)0x10000000);
          break;

        case PIXELCOLUMN_9:
          /* Position : Row = 9 , Column =9*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER10, ~((uint32_t)0x20000000), (uint32_t)0x20000000);
          break;

        case PIXELCOLUMN_10:
          /* Position : Row = 9 , Column =10*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER10, ~((uint32_t)0x40000000), (uint32_t)0x40000000);
          break;

        case PIXELCOLUMN_11:
          /* Position : Row = 9 , Column =11*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER10, ~((uint32_t)0x80000000), (uint32_t)0x80000000);
          break;

        case PIXELCOLUMN_12:
          /* Position : Row = 9 , Column =12*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER11, ~((uint32_t)0x00000001), (uint32_t)0x00000001);
          break;

        case PIXELCOLUMN_13:
          /* Position : Row = 9 , Column =13*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER11, ~((uint32_t)0x00000002), (uint32_t)0x00000002);
          break;

        case PIXELCOLUMN_14:
          /* Position : Row = 9 , Column =14*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER11, ~((uint32_t)0x00000004), (uint32_t)0x00000004);
          break;

        case PIXELCOLUMN_15:
          /* Position : Row = 9 , Column =15*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER11, ~((uint32_t)0x00000008), (uint32_t)0x00000008);
          break;

        case PIXELCOLUMN_16:
          /* Position : Row = 9 , Column =16*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER11, ~((uint32_t)0x00000010), (uint32_t)0x00000010);
          break;

        case PIXELCOLUMN_17:
          /* Position : Row = 9 , Column =17*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER11, ~((uint32_t)0x00000020), (uint32_t)0x00000020);
          break;

        case PIXELCOLUMN_18:
          /* Position : Row = 9 , Column =18*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER11, ~((uint32_t)0x00000040), (uint32_t)0x00000040);
          break;

        case PIXELCOLUMN_19:
          /* Position : Row = 9 , Column =19 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER11, ~((uint32_t)0x00000080), (uint32_t)0x00000080);
          break;

        default:
          break;
      }
      break;

    case PIXELROW_10:
      switch (PixelColumn)
      {
        case PIXELCOLUMN_1:
          /* Position : Row = 10 , Column= 1 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER8, ~((uint32_t)0x00200000), (uint32_t)0x00200000);
          break;

        case PIXELCOLUMN_2:
          /* Position : Row = 10 , Column= 2 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER8, ~((uint32_t)0x00400000), (uint32_t)0x00400000);
          break;

        case PIXELCOLUMN_3:
          /* Position : Row = 10 , Column= 3 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER8, ~((uint32_t)0x00800000), (uint32_t)0x00800000);
          break;

        case PIXELCOLUMN_4:
          /* Position : Row = 10 , Column= 4 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER8, ~((uint32_t)0x01000000), (uint32_t)0x01000000);
          break;

        case PIXELCOLUMN_5:
          /* Position : Row = 10 , Column= 5 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER8, ~((uint32_t)0x02000000), (uint32_t)0x02000000);
          break;

        case PIXELCOLUMN_6:
          /* Position : Row = 10 , Column= 6 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER8, ~((uint32_t)0x04000000), (uint32_t)0x04000000);
          break;

        case PIXELCOLUMN_7:
          /* Position : Row = 10 , Column= 7 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER8, ~((uint32_t)0x08000000), (uint32_t)0x08000000);
          break;

        case PIXELCOLUMN_8:
          /* Position : Row = 10 , Column= 8 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER8, ~((uint32_t)0x10000000), (uint32_t)0x10000000);
          break;

        case PIXELCOLUMN_9:
          /* Position : Row = 10 , Column= 9*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER8, ~((uint32_t)0x20000000), (uint32_t)0x20000000);
          break;

        case PIXELCOLUMN_10:
          /* Position : Row = 10 , Column= 10*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER8, ~((uint32_t)0x40000000), (uint32_t)0x40000000);
          break;

        case PIXELCOLUMN_11:
          /* Position : Row = 10 , Column= 11*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER8, ~((uint32_t)0x80000000), (uint32_t)0x80000000);
          break;

        case PIXELCOLUMN_12:
          /* Position : Row = 10 , Column= 12*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER9, ~((uint32_t)0x00000001), (uint32_t)0x00000001);
          break;

        case PIXELCOLUMN_13:
          /* Position : Row = 10 , Column= 13*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER9, ~((uint32_t)0x00000002), (uint32_t)0x00000002);
          break;

        case PIXELCOLUMN_14:
          /* Position : Row = 10 , Column= 14*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER9, ~((uint32_t)0x00000004), (uint32_t)0x00000004);
          break;

        case PIXELCOLUMN_15:
          /* Position : Row = 10 , Column= 15*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER9, ~((uint32_t)0x00000008), (uint32_t)0x00000008);
          break;

        case PIXELCOLUMN_16:
          /* Position : Row = 10 , Column= 16*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER9, ~((uint32_t)0x00000010), (uint32_t)0x00000010);
          break;

        case PIXELCOLUMN_17:
          /* Position : Row = 10 , Column= 17*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER9, ~((uint32_t)0x00000020), (uint32_t)0x00000020);
          break;

        case PIXELCOLUMN_18:
          /* Position : Row = 10 , Column= 18*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER9, ~((uint32_t)0x00000040), (uint32_t)0x00000040);
          break;

        case PIXELCOLUMN_19:
          /* Position : Row = 10 , Column= 19 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER9, ~((uint32_t)0x00000080), (uint32_t)0x00000080);
          break;

        default:
          break;
      }
    break;
  }

  /*!< Request LCD RAM update */
  HAL_LCD_UpdateDisplayRequest(&LCDHandle);
}

/**
  * @brief  Set Matrix Pixel off.
  * @param  PixelRow: Specify Matrix Row.
  * @param  PixelColumn: Specify Matrix Column.
  * @retval None
  */
void BSP_LCD_GLASS_ClearMatrixPixel(PixelRow_TypeDef PixelRow, PixelColumn_TypeDef PixelColumn)
{
  switch (PixelRow)
  {
    case PIXELROW_1:
      switch (PixelColumn)
      {
        case PIXELCOLUMN_1:
          /* Position : Row = 1 , Column= 1 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, (uint32_t)0xFFFFFDFF, 0);
          break;

        case PIXELCOLUMN_2:
          /* Position : Row = 1 , Column= 2 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, (uint32_t)0xFFFFBFFF, 0);
          break;

        case PIXELCOLUMN_3:
          /* Position : Row = 1 , Column= 3 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, (uint32_t)0xFFFFDFFF, 0);
          break;

        case PIXELCOLUMN_4:
          /* Position : Row = 1 , Column= 4 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, (uint32_t)0xFFFFEFFF, 0);
          break;

        case PIXELCOLUMN_5:
          /* Position : Row = 1 , Column= 5 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, (uint32_t)0xFFFDFFFF, 0);
          break;

        case PIXELCOLUMN_6:
          /* Position : Row = 1 , Column= 6 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, (uint32_t)0xFFFEFFFF, 0);
          break;

        case PIXELCOLUMN_7:
          /* Position : Row = 1 , Column= 7 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, (uint32_t)0xFFFF7FFF, 0);
          break;

        case PIXELCOLUMN_8:
          /* Position : Row = 1 , Column= 8 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, (uint32_t)0xFFEFFFFF, 0);
          break;

        case PIXELCOLUMN_9:
          /* Position : Row = 1 , Column= 9 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, (uint32_t)0xFFF7FFFF, 0);
          break;

        case PIXELCOLUMN_10:
          /* Position : Row = 1 , Column= 10 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, (uint32_t)0xFFFBFFFF, 0);
          break;

        case PIXELCOLUMN_11:
          /* Position : Row = 1 , Column= 11 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, (uint32_t)0xFFFFFFFB, 0);
          break;

        case PIXELCOLUMN_12:
          /* Position : Row = 1 , Column= 12 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, (uint32_t)0xFFFFFFFD, 0);
          break;

        case PIXELCOLUMN_13:
          /* Position : Row = 1 , Column= 13 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, (uint32_t)0xFFFFFFFE, 0);
          break;

        case PIXELCOLUMN_14:
          /* Position : Row = 1 , Column= 14 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, (uint32_t)0xFFFFFFDF, 0);
          break;

        case PIXELCOLUMN_15:
          /* Position : Row = 1 , Column= 15 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, (uint32_t)0xFFFFFFEF, 0);
          break;

        case PIXELCOLUMN_16:
          /* Position : Row = 1 , Column= 16 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, (uint32_t)0xFFFFFFF7, 0);
          break;

        case PIXELCOLUMN_17:
          /* Position : Row = 1 , Column= 17 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, (uint32_t)0xFFFFFEFF, 0);
          break;

        case PIXELCOLUMN_18:
          /* Position : Row = 1 , Column= 18 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, (uint32_t)0xFFFFFF7F, 0);
          break;

        case PIXELCOLUMN_19:
          /* Position : Row = 1 , Column= 19 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, (uint32_t)0xFFFFFFBF, 0);
          break;

        default:
          break;
      }
      break;

    case PIXELROW_2:

      switch (PixelColumn)
      {
        case PIXELCOLUMN_1:
          /* Position : Row = 2 , Column= 1 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, (uint32_t)0xFFFFFDFF, 0);
          break;

        case PIXELCOLUMN_2:
          /* Position : Row = 2 , Column= 2 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, (uint32_t)0xFFFFBFFF, 0);
          break;

        case PIXELCOLUMN_3:
          /* Position : Row = 2 , Column= 3 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, (uint32_t)0xFFFFDFFF, 0);
          break;

        case PIXELCOLUMN_4:
          /* Position : Row = 2 , Column= 4 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, (uint32_t)0xFFFFEFFF, 0);
          break;

        case PIXELCOLUMN_5:
          /* Position : Row = 2 , Column= 5 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, (uint32_t)0xFFFDFFFF, 0);
          break;

        case PIXELCOLUMN_6:
          /* Position : Row = 2 , Column= 6 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, (uint32_t)0xFFFEFFFF, 0);
          break;

        case PIXELCOLUMN_7:
          /* Position : Row = 2 , Column= 7 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, (uint32_t)0xFFFF7FFF, 0);
          break;

        case PIXELCOLUMN_8:
          /* Position : Row = 2 , Column= 8 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, (uint32_t)0xFFEFFFFF, 0);
          break;

        case PIXELCOLUMN_9:
          /* Position : Row = 2 , Column= 9 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, (uint32_t)0xFFF7FFFF, 0);
          break;

        case PIXELCOLUMN_10:
          /* Position : Row = 2 , Column= 10 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, (uint32_t)0xFFFBFFFF, 0);
          break;

        case PIXELCOLUMN_11:
          /* Position : Row = 2 , Column= 11 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, (uint32_t)0xFFFFFFFB, 0);
          break;

        case PIXELCOLUMN_12:
          /* Position : Row = 2 , Column= 12 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, (uint32_t)0xFFFFFFFD, 0);
          break;

        case PIXELCOLUMN_13:
          /* Position : Row = 2 , Column= 13 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, (uint32_t)0xFFFFFFFE, 0);
          break;

        case PIXELCOLUMN_14:
          /* Position : Row = 2 , Column= 14 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, (uint32_t)0xFFFFFFDF, 0);
          break;

        case PIXELCOLUMN_15:
          /* Position : Row = 2 , Column= 15 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, (uint32_t)0xFFFFFFEF, 0);
          break;

        case PIXELCOLUMN_16:
          /* Position : Row = 2 , Column= 16 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, (uint32_t)0xFFFFFFF7, 0);
          break;

        case PIXELCOLUMN_17:
          /* Position : Row = 2 , Column= 17 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, (uint32_t)0xFFFFFEFF, 0);
          break;

        case PIXELCOLUMN_18:
          /* Position : Row = 2 , Column= 18 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, (uint32_t)0xFFFFFF7F, 0);
          break;

        case PIXELCOLUMN_19:
          /* Position : Row = 2 , Column= 19 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, (uint32_t)0xFFFFFFBF, 0);
          break;

        default:
          break;
      }
      break;

    case PIXELROW_3:
      switch (PixelColumn)
      {
        case PIXELCOLUMN_1:
          /* Position : Row = 3 , Column= 1 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, (uint32_t)0xFFDFFFFF, 0);
          break;

        case PIXELCOLUMN_2:
          /* Position : Row = 3 , Column= 2 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, (uint32_t)0xFFBFFFFF, 0);
          break;

        case PIXELCOLUMN_3:
          /* Position : Row = 3 , Column= 3 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, (uint32_t)0xFF7FFFFF, 0);
          break;

        case PIXELCOLUMN_4:
          /* Position : Row = 3 , Column= 4 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, (uint32_t)0xFEFFFFFF, 0);
          break;

        case PIXELCOLUMN_5:
          /* Position : Row = 3 , Column= 5 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, (uint32_t)0xFDFFFFFF, 0);
          break;

        case PIXELCOLUMN_6:
          /* Position : Row = 3 , Column= 6 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, (uint32_t)0xFBFFFFFF, 0);
          break;

        case PIXELCOLUMN_7:
          /* Position : Row = 3 , Column= 7 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, (uint32_t)0xF7FFFFFF, 0);
          break;

        case PIXELCOLUMN_8:
          /* Position : Row = 3 , Column= 8 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, (uint32_t)0xEFFFFFFF, 0);
          break;

        case PIXELCOLUMN_9:
          /* Position : Row = 3 , Column= 9*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, (uint32_t)0xDFFFFFFF, 0);
          break;

        case PIXELCOLUMN_10:
          /* Position : Row = 3 , Column= 10*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, (uint32_t)0xBFFFFFFF, 0);
          break;

        case PIXELCOLUMN_11:
          /* Position : Row = 3 , Column= 11*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER4, (uint32_t)0x7FFFFFFF, 0);
          break;

        case PIXELCOLUMN_12:
          /* Position : Row = 3 , Column= 12*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER5, (uint32_t)0xFFFFFFFE, 0);
          break;

        case PIXELCOLUMN_13:
          /* Position : Row = 3 , Column= 13*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER5, (uint32_t)0xFFFFFFFD, 0);
          break;

        case PIXELCOLUMN_14:
          /* Position : Row = 3 , Column= 14*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER5, (uint32_t)0xFFFFFFFB, 0);
          break;

        case PIXELCOLUMN_15:
          /* Position : Row = 3 , Column= 15*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER5, (uint32_t)0xFFFFFFF7, 0);
          break;

        case PIXELCOLUMN_16:
          /* Position : Row = 3 , Column= 16*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER5, (uint32_t)0xFFFFFFEF, 0);
          break;

        case PIXELCOLUMN_17:
          /* Position : Row = 3 , Column= 17*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER5, (uint32_t)0xFFFFFFDF, 0);
          break;

        case PIXELCOLUMN_18:
          /* Position : Row = 3 , Column= 18*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER5, (uint32_t)0xFFFFFFBF, 0);
          break;

        case PIXELCOLUMN_19:
          /* Position : Row = 3 , Column= 19 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER5, (uint32_t)0xFFFFFF7F, 0);
          break;

        default:
          break;
      }
      break;

    case PIXELROW_4:
      switch (PixelColumn)
      {
        case PIXELCOLUMN_1:
          /* Position : Row = 4 , Column= 1 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, (uint32_t)0xFFDFFFFF, 0);
          break;

        case PIXELCOLUMN_2:
          /* Position : Row = 4 , Column= 2 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, (uint32_t)0xFFBFFFFF, 0);
          break;

        case PIXELCOLUMN_3:
          /* Position : Row = 4 , Column= 3 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, (uint32_t)0xFF7FFFFF, 0);
          break;

        case PIXELCOLUMN_4:
          /* Position : Row = 4 , Column= 4 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, (uint32_t)0xFEFFFFFF, 0);
          break;

        case PIXELCOLUMN_5:
          /* Position : Row = 4 , Column= 5 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, (uint32_t)0xFDFFFFFF, 0);
          break;

        case PIXELCOLUMN_6:
          /* Position : Row = 4 , Column= 6 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, (uint32_t)0xFBFFFFFF, 0);
          break;

        case PIXELCOLUMN_7:
          /* Position : Row = 4 , Column= 7 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, (uint32_t)0xF7FFFFFF, 0);
          break;

        case PIXELCOLUMN_8:
          /* Position : Row = 4 , Column= 8 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, (uint32_t)0xEFFFFFFF, 0);
          break;

        case PIXELCOLUMN_9:
          /* Position : Row = 4 , Column= 9*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, (uint32_t)0xDFFFFFFF, 0);
          break;

        case PIXELCOLUMN_10:
          /* Position : Row = 4 , Column= 10*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, (uint32_t)0xBFFFFFFF, 0);
          break;

        case PIXELCOLUMN_11:
          /* Position : Row = 4 , Column= 11*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER6, (uint32_t)0x7FFFFFFF, 0);
          break;

        case PIXELCOLUMN_12:
          /* Position : Row = 4 , Column= 12*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER7, (uint32_t)0xFFFFFFFE, 0);
          break;

        case PIXELCOLUMN_13:
          /* Position : Row = 4 , Column= 13*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER7, (uint32_t)0xFFFFFFFD, 0);
          break;

        case PIXELCOLUMN_14:
          /* Position : Row = 4 , Column= 14*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER7, (uint32_t)0xFFFFFFFB, 0);
          break;

        case PIXELCOLUMN_15:
          /* Position : Row = 4 , Column= 15*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER7, (uint32_t)0xFFFFFFF7, 0);
          break;

        case PIXELCOLUMN_16:
          /* Position : Row = 4 , Column= 16*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER7, (uint32_t)0xFFFFFFEF, 0);
          break;

        case PIXELCOLUMN_17:
          /* Position : Row = 4 , Column= 17*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER7, (uint32_t)0xFFFFFFDF, 0);
          break;

        case PIXELCOLUMN_18:
          /* Position : Row = 4 , Column= 18*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER7, (uint32_t)0xFFFFFFBF, 0);
          break;

        case PIXELCOLUMN_19:
          /* Position : Row = 4 , Column= 19 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER7, (uint32_t)0xFFFFFF7F, 0);
          break;

        default:
          break;
      }
      break;

    case PIXELROW_5:
      switch (PixelColumn)
      {
        case PIXELCOLUMN_1:
          /* Position : Row = 5 , Column= 1 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER0, (uint32_t)0xFFDFFFFF, 0);
          break;

        case PIXELCOLUMN_2:
          /* Position : Row = 5 , Column= 2 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER0, (uint32_t)0xFFBFFFFF, 0);
          break;

        case PIXELCOLUMN_3:
          /* Position : Row = 5 , Column= 3 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER0, (uint32_t)0xFF7FFFFF, 0);
          break;

        case PIXELCOLUMN_4:
          /* Position : Row = 5 , Column= 4 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER0, (uint32_t)0xFEFFFFFF, 0);
          break;

        case PIXELCOLUMN_5:
          /* Position : Row = 5 , Column= 5 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER0, (uint32_t)0xFDFFFFFF, 0);
          break;

        case PIXELCOLUMN_6:
          /* Position : Row = 5 , Column= 6 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER0, (uint32_t)0xFBFFFFFF, 0);
          break;

        case PIXELCOLUMN_7:
          /* Position : Row = 5 , Column= 7 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER0, (uint32_t)0xF7FFFFFF, 0);
          break;

        case PIXELCOLUMN_8:
          /* Position : Row = 5 , Column= 8 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER0, (uint32_t)0xEFFFFFFF, 0);
          break;

        case PIXELCOLUMN_9:
          /* Position : Row = 5 , Column= 9*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER0, (uint32_t)0xDFFFFFFF, 0);
          break;

        case PIXELCOLUMN_10:
          /* Position : Row = 5 , Column= 10*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER0, (uint32_t)0xBFFFFFFF, 0);
          break;

        case PIXELCOLUMN_11:
          /* Position : Row = 5 , Column= 11*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER0, (uint32_t)0x7FFFFFFF, 0);
          break;

        case PIXELCOLUMN_12:
          /* Position : Row = 5 , Column= 12*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER1, (uint32_t)0xFFFFFFFE, 0);
          break;

        case PIXELCOLUMN_13:
          /* Position : Row = 5 , Column= 13*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER1, (uint32_t)0xFFFFFFFD, 0);
          break;

        case PIXELCOLUMN_14:
          /* Position : Row = 5 , Column= 14*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER1, (uint32_t)0xFFFFFFFB, 0);
          break;

        case PIXELCOLUMN_15:
          /* Position : Row = 5 , Column= 15*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER1, (uint32_t)0xFFFFFFF7, 0);
          break;

        case PIXELCOLUMN_16:
          /* Position : Row = 5 , Column= 16*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER1, (uint32_t)0xFFFFFFEF, 0);
          break;

        case PIXELCOLUMN_17:
          /* Position : Row = 5 , Column= 17*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER1, (uint32_t)0xFFFFFFDF, 0);
          break;

        case PIXELCOLUMN_18:
          /* Position : Row = 5 , Column= 18*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER1, (uint32_t)0xFFFFFFBF, 0);
          break;

        case PIXELCOLUMN_19:
          /* Position : Row = 5 , Column= 19 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER1, (uint32_t)0xFFFFFF7F, 0);
          break;

        default:
          break;
      }
      break;

    case PIXELROW_6:
      switch (PixelColumn)
      {
        case PIXELCOLUMN_1:
          /* Position : Row = 6 , Column= 1 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER2, (uint32_t)0xFFDFFFFF, 0);
          break;

        case PIXELCOLUMN_2:
          /* Position : Row = 6 , Column= 2 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER2, (uint32_t)0xFFBFFFFF, 0);
          break;

        case PIXELCOLUMN_3:
          /* Position : Row = 6 , Column= 3 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER2, (uint32_t)0xFF7FFFFF, 0);
          break;

        case PIXELCOLUMN_4:
          /* Position : Row = 6 , Column= 4 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER2, (uint32_t)0xFEFFFFFF, 0);
          break;

        case PIXELCOLUMN_5:
          /* Position : Row = 6 , Column= 5 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER2, (uint32_t)0xFDFFFFFF, 0);
          break;

        case PIXELCOLUMN_6:
          /* Position : Row = 6 , Column= 6 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER2, (uint32_t)0xFBFFFFFF, 0);
          break;

        case PIXELCOLUMN_7:
          /* Position : Row = 6 , Column= 7 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER2, (uint32_t)0xF7FFFFFF, 0);
          break;

        case PIXELCOLUMN_8:
          /* Position : Row = 6 , Column= 8 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER2, (uint32_t)0xEFFFFFFF, 0);
          break;

        case PIXELCOLUMN_9:
          /* Position : Row = 6 , Column= 9*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER2, (uint32_t)0xDFFFFFFF, 0);
          break;

        case PIXELCOLUMN_10:
          /* Position : Row = 6 , Column= 10*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER2, (uint32_t)0xBFFFFFFF, 0);
          break;

        case PIXELCOLUMN_11:
          /* Position : Row = 6 , Column= 11*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER2, (uint32_t)0x7FFFFFFF, 0);
          break;

        case PIXELCOLUMN_12:
          /* Position : Row = 6 , Column= 12*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER3, (uint32_t)0xFFFFFFFE, 0);
          break;

        case PIXELCOLUMN_13:
          /* Position : Row = 6 , Column= 13*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER3, (uint32_t)0xFFFFFFFD, 0);
          break;

        case PIXELCOLUMN_14:
          /* Position : Row = 6 , Column= 14*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER3, (uint32_t)0xFFFFFFFB, 0);
          break;

        case PIXELCOLUMN_15:
          /* Position : Row = 6 , Column= 15*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER3, (uint32_t)0xFFFFFFF7, 0);
          break;

        case PIXELCOLUMN_16:
          /* Position : Row = 6 , Column= 16*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER3, (uint32_t)0xFFFFFFEF, 0);
          break;

        case PIXELCOLUMN_17:
          /* Position : Row = 6 , Column= 17*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER3, (uint32_t)0xFFFFFFDF, 0);
          break;

        case PIXELCOLUMN_18:
          /* Position : Row = 6 , Column= 18*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER3, (uint32_t)0xFFFFFFBF, 0);
          break;

        case PIXELCOLUMN_19:
          /* Position : Row = 6 , Column= 19 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER3, (uint32_t)0xFFFFFF7F, 0);
          break;

        default:
          break;
      }
      break;

    case PIXELROW_7:
      switch (PixelColumn)
      {
        case PIXELCOLUMN_1:
          /* Position : Row = 7 , Column =1 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER14, (uint32_t)0xFFDFFFFF, 0);
          break;

        case PIXELCOLUMN_2:
          /* Position : Row = 7 , Column =2 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER14, (uint32_t)0xFFBFFFFF, 0);
          break;

        case PIXELCOLUMN_3:
          /* Position : Row = 7 , Column =3 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER14, (uint32_t)0xFF7FFFFF, 0);
          break;

        case PIXELCOLUMN_4:
          /* Position : Row = 7 , Column =4 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER14, (uint32_t)0xFEFFFFFF, 0);
          break;

        case PIXELCOLUMN_5:
          /* Position : Row = 7 , Column =5 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER14, (uint32_t)0xFDFFFFFF, 0);
          break;

        case PIXELCOLUMN_6:
          /* Position : Row = 7 , Column =6 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER14, (uint32_t)0xFBFFFFFF, 0);
          break;

        case PIXELCOLUMN_7:
          /* Position : Row = 7 , Column =7 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER14, (uint32_t)0xF7FFFFFF, 0);
          break;

        case PIXELCOLUMN_8:
          /* Position : Row = 7 , Column =8 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER14, (uint32_t)0xEFFFFFFF, 0);
          break;

        case PIXELCOLUMN_9:
          /* Position : Row = 7 , Column =9*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER14, (uint32_t)0xDFFFFFFF, 0);
          break;

        case PIXELCOLUMN_10:
          /* Position : Row = 7 , Column =10*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER14, (uint32_t)0xBFFFFFFF, 0);
          break;

        case PIXELCOLUMN_11:
          /* Position : Row = 7 , Column =11*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER14, (uint32_t)0x7FFFFFFF, 0);
          break;

        case PIXELCOLUMN_12:
          /* Position : Row = 7 , Column =12*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER15, (uint32_t)0xFFFFFFFE, 0);
          break;

        case PIXELCOLUMN_13:
          /* Position : Row = 7 , Column =13*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER15, (uint32_t)0xFFFFFFFD, 0);
          break;

        case PIXELCOLUMN_14:
          /* Position : Row = 7 , Column =14*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER15, (uint32_t)0xFFFFFFFB, 0);
          break;

        case PIXELCOLUMN_15:
          /* Position : Row = 7 , Column =15*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER15, (uint32_t)0xFFFFFFF7, 0);
          break;

        case PIXELCOLUMN_16:
          /* Position : Row = 7 , Column =16*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER15, (uint32_t)0xFFFFFFEF, 0);
          break;

        case PIXELCOLUMN_17:
          /* Position : Row = 7 , Column =17*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER15, (uint32_t)0xFFFFFFDF, 0);
          break;

        case PIXELCOLUMN_18:
          /* Position : Row = 7 , Column =18*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER15, (uint32_t)0xFFFFFFBF, 0);
          break;

        case PIXELCOLUMN_19:
          /* Position : Row = 7 , Column =19 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER15, (uint32_t)0xFFFFFF7F, 0);
          break;

        default:
          break;
      }
      break;

    case PIXELROW_8:
      switch (PixelColumn)
        {
        case PIXELCOLUMN_1:
          /* Position : Row = 8 , Column =1 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER12, (uint32_t)0xFFDFFFFF, 0);
          break;

        case PIXELCOLUMN_2:
          /* Position : Row = 8 , Column =2 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER12, (uint32_t)0xFFBFFFFF, 0);
          break;

        case PIXELCOLUMN_3:
          /* Position : Row = 8 , Column =3 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER12, (uint32_t)0xFF7FFFFF, 0);
          break;

        case PIXELCOLUMN_4:
          /* Position : Row = 8 , Column =4 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER12, (uint32_t)0xFEFFFFFF, 0);
          break;

        case PIXELCOLUMN_5:
          /* Position : Row = 8 , Column =5 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER12, (uint32_t)0xFDFFFFFF, 0);
          break;

        case PIXELCOLUMN_6:
          /* Position : Row = 8 , Column =6 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER12, (uint32_t)0xFBFFFFFF, 0);
          break;

        case PIXELCOLUMN_7:
          /* Position : Row = 8 , Column =7 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER12, (uint32_t)0xF7FFFFFF, 0);
          break;

        case PIXELCOLUMN_8:
          /* Position : Row = 8 , Column =8 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER12, (uint32_t)0xEFFFFFFF, 0);
          break;

        case PIXELCOLUMN_9:
          /* Position : Row = 8 , Column =9*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER12, (uint32_t)0xDFFFFFFF, 0);
          break;

        case PIXELCOLUMN_10:
          /* Position : Row = 8 , Column =10*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER12, (uint32_t)0xBFFFFFFF, 0);
          break;

        case PIXELCOLUMN_11:
          /* Position : Row = 8 , Column =11*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER12, (uint32_t)0x7FFFFFFF, 0);
          break;

        case PIXELCOLUMN_12:
          /* Position : Row = 8 , Column =12*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER13, (uint32_t)0xFFFFFFFE, 0);
          break;

        case PIXELCOLUMN_13:
          /* Position : Row = 8 , Column =13*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER13, (uint32_t)0xFFFFFFFD, 0);
          break;

        case PIXELCOLUMN_14:
          /* Position : Row = 8 , Column =14*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER13, (uint32_t)0xFFFFFFFB, 0);
          break;

        case PIXELCOLUMN_15:
          /* Position : Row = 8 , Column =15*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER13, (uint32_t)0xFFFFFFF7, 0);
          break;

        case PIXELCOLUMN_16:
          /* Position : Row = 8 , Column =16*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER13, (uint32_t)0xFFFFFFEF, 0);
          break;

        case PIXELCOLUMN_17:
          /* Position : Row = 8 , Column =17*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER13, (uint32_t)0xFFFFFFDF, 0);
          break;

        case PIXELCOLUMN_18:
          /* Position : Row = 8 , Column =18*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER13, (uint32_t)0xFFFFFFBF, 0);
          break;

        case PIXELCOLUMN_19:
          /* Position : Row = 8 , Column =19 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER13, (uint32_t)0xFFFFFF7F, 0);
          break;

          default:
          break;
        }
      break;

    case PIXELROW_9:
      switch (PixelColumn)
        {
        case PIXELCOLUMN_1:
          /* Position : Row = 9 , Column =1 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER10, (uint32_t)0xFFDFFFFF, 0);
          break;

        case PIXELCOLUMN_2:
          /* Position : Row = 9 , Column =2 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER10, (uint32_t)0xFFBFFFFF, 0);
          break;

        case PIXELCOLUMN_3:
          /* Position : Row = 9 , Column =3 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER10, (uint32_t)0xFF7FFFFF, 0);
          break;

        case PIXELCOLUMN_4:
          /* Position : Row = 9 , Column =4 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER10, (uint32_t)0xFEFFFFFF, 0);
          break;

        case PIXELCOLUMN_5:
          /* Position : Row = 9 , Column =5 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER10, (uint32_t)0xFDFFFFFF, 0);
          break;

        case PIXELCOLUMN_6:
          /* Position : Row = 9 , Column =6 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER10, (uint32_t)0xFBFFFFFF, 0);
          break;

        case PIXELCOLUMN_7:
          /* Position : Row = 9 , Column =7 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER10, (uint32_t)0xF7FFFFFF, 0);
          break;

        case PIXELCOLUMN_8:
          /* Position : Row = 9 , Column =8 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER10, (uint32_t)0xEFFFFFFF, 0);
          break;

        case PIXELCOLUMN_9:
          /* Position : Row = 9 , Column =9*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER10, (uint32_t)0xDFFFFFFF, 0);
          break;

        case PIXELCOLUMN_10:
          /* Position : Row = 9 , Column =10*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER10, (uint32_t)0xBFFFFFFF, 0);
          break;

        case PIXELCOLUMN_11:
          /* Position : Row = 9 , Column =11*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER10, (uint32_t)0x7FFFFFFF, 0);
          break;

        case PIXELCOLUMN_12:
          /* Position : Row = 9 , Column =12*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER11, (uint32_t)0xFFFFFFFE, 0);
          break;

        case PIXELCOLUMN_13:
          /* Position : Row = 9 , Column =13*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER11, (uint32_t)0xFFFFFFFD, 0);
          break;

        case PIXELCOLUMN_14:
          /* Position : Row = 9 , Column =14*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER11, (uint32_t)0xFFFFFFFB, 0);
          break;

        case PIXELCOLUMN_15:
          /* Position : Row = 9 , Column =15*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER11, (uint32_t)0xFFFFFFF7, 0);
          break;

        case PIXELCOLUMN_16:
          /* Position : Row = 9 , Column =16*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER11, (uint32_t)0xFFFFFFEF, 0);
          break;

        case PIXELCOLUMN_17:
          /* Position : Row = 9 , Column =17*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER11, (uint32_t)0xFFFFFFDF, 0);
          break;

        case PIXELCOLUMN_18:
          /* Position : Row = 9 , Column =18*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER11, (uint32_t)0xFFFFFFBF, 0);
          break;

        case PIXELCOLUMN_19:
          /* Position : Row = 9 , Column =19 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER11, (uint32_t)0xFFFFFF7F, 0);
          break;

          default:
          break;
        }
      break;

    case PIXELROW_10:
      switch (PixelColumn)
        {
        case PIXELCOLUMN_1:
          /* Position : Row = 10 , Column= 1 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER8, (uint32_t)0xFFDFFFFF, 0);
          break;

        case PIXELCOLUMN_2:
          /* Position : Row = 10 , Column= 2 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER8, (uint32_t)0xFFBFFFFF, 0);
          break;

        case PIXELCOLUMN_3:
          /* Position : Row = 10 , Column= 3 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER8, (uint32_t)0xFF7FFFFF, 0);
          break;

        case PIXELCOLUMN_4:
          /* Position : Row = 10 , Column= 4 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER8, (uint32_t)0xFEFFFFFF, 0);
          break;

        case PIXELCOLUMN_5:
          /* Position : Row = 10 , Column= 5 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER8, (uint32_t)0xFDFFFFFF, 0);
          break;

        case PIXELCOLUMN_6:
          /* Position : Row = 10 , Column= 6 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER8, (uint32_t)0xFBFFFFFF, 0);
          break;

        case PIXELCOLUMN_7:
          /* Position : Row = 10 , Column= 7 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER8, (uint32_t)0xF7FFFFFF, 0);
          break;

        case PIXELCOLUMN_8:
          /* Position : Row = 10 , Column= 8 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER8, (uint32_t)0xEFFFFFFF, 0);
          break;

        case PIXELCOLUMN_9:
          /* Position : Row = 10 , Column= 9*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER8, (uint32_t)0xDFFFFFFF, 0);
          break;

        case PIXELCOLUMN_10:
          /* Position : Row = 10 , Column= 10*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER8, (uint32_t)0xBFFFFFFF, 0);
          break;

        case PIXELCOLUMN_11:
          /* Position : Row = 10 , Column= 11*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER8, (uint32_t)0x7FFFFFFF, 0);
          break;

        case PIXELCOLUMN_12:
          /* Position : Row = 10 , Column= 12*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER9, (uint32_t)0xFFFFFFFE, 0);
          break;

        case PIXELCOLUMN_13:
          /* Position : Row = 10 , Column= 13*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER9, (uint32_t)0xFFFFFFFD, 0);
          break;

        case PIXELCOLUMN_14:
          /* Position : Row = 10 , Column= 14*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER9, (uint32_t)0xFFFFFFFB, 0);
          break;

        case PIXELCOLUMN_15:
          /* Position : Row = 10 , Column= 15*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER9, (uint32_t)0xFFFFFFF7, 0);
          break;

        case PIXELCOLUMN_16:
          /* Position : Row = 10 , Column= 16*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER9, (uint32_t)0xFFFFFFEF, 0);
          break;

        case PIXELCOLUMN_17:
          /* Position : Row = 10 , Column= 17*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER9, (uint32_t)0xFFFFFFDF, 0);
          break;

        case PIXELCOLUMN_18:
          /* Position : Row = 10 , Column= 18*/
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER9, (uint32_t)0xFFFFFFBF, 0);
          break;

        case PIXELCOLUMN_19:
          /* Position : Row = 10 , Column= 19 */
          HAL_LCD_Write(&LCDHandle, LCD_RAM_REGISTER9, (uint32_t)0xFFFFFF7F, 0);
          break;

          default:
          break;
        }
      break;
  }

  /*!< Request LCD RAM update */
  HAL_LCD_UpdateDisplayRequest(&LCDHandle);
}

/**
  * @}
  */

/** @addtogroup STM32L476G_EVAL_LCD_Private_Functions
  * @{
  */

/**
  * @brief  LCD MSP Init.
  * @param  hlcd: LCD handle
  * @retval None
  */
static void LCD_MspInit(LCD_HandleTypeDef *hlcd)
{
  GPIO_InitTypeDef  gpioinitstruct = {0};
  RCC_OscInitTypeDef oscinitstruct = {0};

  /*##-1- Enable PWR  peripheral Clock #######################################*/
  __HAL_RCC_PWR_CLK_ENABLE();

  /*##-2- Allow Access and Reset the Backup Domaine ##########################*/
  /* Allow Access to Backup Domaine */
  HAL_PWR_EnableBkUpAccess();

  /* Reset the Backup Domaine */
  __HAL_RCC_BACKUPRESET_FORCE();
  __HAL_RCC_BACKUPRESET_RELEASE();

  /*##-3- Configure LSE as RTC clock source ###################################*/
  oscinitstruct.OscillatorType =  RCC_OSCILLATORTYPE_LSE;
  oscinitstruct.PLL.PLLState = RCC_PLL_NONE;
  oscinitstruct.LSEState = RCC_LSE_ON;
  if(HAL_RCC_OscConfig(&oscinitstruct) != HAL_OK)
  {
    while(1);
  }
 __HAL_RCC_RTC_CONFIG(RCC_RTCCLKSOURCE_LSE);


  /*##-4- Enable LCD GPIO Clocks #############################################*/
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*##-5- Configure peripheral GPIO ##########################################*/
  /* Configure Output for LCD */
  /* Port A */
  gpioinitstruct.Pin       = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_6 | \
                             GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | \
                             GPIO_PIN_15;
  gpioinitstruct.Mode      = GPIO_MODE_AF_PP;
  gpioinitstruct.Pull      = GPIO_NOPULL;
  gpioinitstruct.Speed     = GPIO_SPEED_FREQ_MEDIUM;
  gpioinitstruct.Alternate = GPIO_AF11_LCD;
  HAL_GPIO_Init(GPIOA, &gpioinitstruct);


  /* Configure Output for LCD */
  /* Port B */
  gpioinitstruct.Pin       = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_3 | GPIO_PIN_4 | \
                             GPIO_PIN_5 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | \
                             GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | \
                             GPIO_PIN_15;
  HAL_GPIO_Init(GPIOB, &gpioinitstruct);


  /* Configure Output for LCD */
  /* Port C*/
  gpioinitstruct.Pin       = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | \
                             GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | \
                             GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | \
                             GPIO_PIN_12;
  HAL_GPIO_Init(GPIOC, &gpioinitstruct);

  /* Configure Output for LCD */
  /* Port D*/
  gpioinitstruct.Pin       = GPIO_PIN_2 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | \
                             GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | \
                             GPIO_PIN_15;
  HAL_GPIO_Init(GPIOD, &gpioinitstruct);

  /* Configure Output for LCD */
  /* Port E*/
  gpioinitstruct.Pin       = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
  HAL_GPIO_Init(GPIOE, &gpioinitstruct);

  /*##-6- Enable LCD peripheral Clock ########################################*/
  __HAL_RCC_LCD_CLK_ENABLE();
}

/**
  * @brief  Converts an ascii char to the a LCD Digit (previous coding).
  * @param  c: a char to display.
  * @param  Point: a point to add in front of a char
  *         This parameter can be: POINT_OFF or POINT_ON
  * @param  DoublePoint: flag indicating if Double Point has to be added in front
  *         of the displayed character.
  *         This parameter can be: DOUBLEPOINT_ON or DOUBLEPOINT_OFF.
  * @retval None
  */
static void Convert(uint8_t* c, Point_Typedef Point, DoublePoint_Typedef DoublePoint)
{
  uint32_t ch = 0 , tmp = 0;
  uint8_t i = 0;

  /* The character c is a letter in upper case*/
  if ((*c < (uint8_t)ASCII_CHAR_LEFT_OPEN_BRACKET)&(*c > (uint8_t)ASCII_CHAR_AT_SYMBOL))
  {
    ch = LetterMap[*c-(uint8_t)ASCII_CHAR_A];
  }
  /* The character c is a number*/
  if ((*c < (uint8_t)ASCII_CHAR_COLON)&(*c > (uint8_t)ASCII_CHAR_FORWARD_SLASH))
  {
    ch = NumberMap[*c-(uint8_t)ASCII_CHAR_0];
  }
  /* The character c is a space character */
  if (*c == (uint8_t)ASCII_CHAR_SPACE)
  {
    ch = (uint32_t)0x00;
  }
  /* Set the Q pixel in the character that can be displayed if the point is on */
  if (Point == POINT_ON)
  {
    ch |= (uint32_t)0x00400000;
  }

  /* Set the P pixel in the character that can be displayed if the double point is on */
  if (DoublePoint == DOUBLEPOINT_ON)
  {
    ch |= (uint32_t)0x00000040;
  }

  for (i = 0;i < 6; i++)
  {
    tmp = ch & Mask[i];
    Digit[i] = (uint8_t)(tmp >> (uint8_t)Shift[i]);
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


