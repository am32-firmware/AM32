/**
  ******************************************************************************
  * @file    stm32l476g_discovery_glass_lcd.c
  * @author  MCD Application Team
  * @brief   This file provides a set of functions needed to manage the
  *          LCD Glass driver for the STM32L476G-Discovery board.
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
#include "stm32l476g_discovery_glass_lcd.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L476G_DISCOVERY
  * @{
  */

/** @defgroup STM32L476G_DISCOVERY_GLASS_LCD STM32L476G-DISCOVERY GLASS LCD
  * @brief This file includes the LCD Glass driver for LCD Module of
  *        STM32L476G-DISCOVERY board.
  * @{
  */

/* Private constants ---------------------------------------------------------*/

/** @defgroup STM32L476G_DISCOVERY_GLASS_LCD_Private_Constants Private Constants
  * @{
  */
#define ASCII_CHAR_0                  0x30  /* 0 */
#define ASCII_CHAR_AT_SYMBOL          0x40  /* @ */
#define ASCII_CHAR_LEFT_OPEN_BRACKET  0x5B  /* [ */
#define ASCII_CHAR_APOSTROPHE         0x60  /* ` */
#define ASCII_CHAR_LEFT_OPEN_BRACE    0x7B  /* ( */
/**
  * @}
  */

/* Private variables ---------------------------------------------------------*/

/** @defgroup STM32L476G_DISCOVERY_GLASS_LCD_Private_Variables Private Variables
  * @{
  */

/* this variable can be used for accelerate the scrolling exit when push user button */
__IO uint8_t bLCDGlass_KeyPressed = 0;

/**
  @verbatim
================================================================================
                              GLASS LCD MAPPING
================================================================================
LCD allows to display information on six 14-segment digits and 4 bars:

  1       2       3       4       5       6
-----   -----   -----   -----   -----   -----
|\|/| o |\|/| o |\|/| o |\|/| o |\|/|   |\|/|   BAR3
-- --   -- --   -- --   -- --   -- --   -- --   BAR2
|/|\| o |/|\| o |/|\| o |/|\| o |/|\|   |/|\|   BAR1
----- * ----- * ----- * ----- * -----   -----   BAR0

LCD segment mapping:
--------------------
  -----A-----        _
  |\   |   /|   COL |_|
  F H  J  K B
  |  \ | /  |        _
  --G-- --M--   COL |_|
  |  / | \  |
  E Q  P  N C
  |/   |   \|        _
  -----D-----   DP  |_|

 An LCD character coding is based on the following matrix:
COM           0   1   2     3
SEG(n)      { E , D , P ,   N   }
SEG(n+1)    { M , C , COL , DP  }
SEG(23-n-1) { B , A , K ,   J   }
SEG(23-n)   { G , F , Q ,   H   }
with n positive odd number.

 The character 'A' for example is:
  -------------------------------
LSB   { 1 , 0 , 0 , 0   }
      { 1 , 1 , 0 , 0   }
      { 1 , 1 , 0 , 0   }
MSB   { 1 , 1 , 0 , 0   }
      -------------------
  'A' =  F    E   0   0 hexa

  @endverbatim
*/

LCD_HandleTypeDef LCDHandle;

/* Constant table for cap characters 'A' --> 'Z' */
const uint16_t CapLetterMap[26] =
{
  /* A      B      C      D      E      F      G      H      I  */
  0xFE00, 0x6714, 0x1D00, 0x4714, 0x9D00, 0x9C00, 0x3F00, 0xFA00, 0x0014,
  /* J      K      L      M      N      O      P      Q      R  */
  0x5300, 0x9841, 0x1900, 0x5A48, 0x5A09, 0x5F00, 0xFC00, 0x5F01, 0xFC01,
  /* S      T      U      V      W      X      Y      Z  */
  0xAF00, 0x0414, 0x5b00, 0x18C0, 0x5A81, 0x00C9, 0x0058, 0x05C0
};

/* Constant table for number '0' --> '9' */
const uint16_t NumberMap[10] =
{
  /* 0      1      2      3      4      5      6      7      8      9  */
  0x5F00, 0x4200, 0xF500, 0x6700, 0xEa00, 0xAF00, 0xBF00, 0x04600, 0xFF00, 0xEF00
};

uint32_t Digit[4];     /* Digit frame buffer */

/* LCD BAR status: To save the bar setting after writing in LCD RAM memory */
uint8_t LCDBar = BATTERYLEVEL_FULL;

/**
  * @}
  */

/** @defgroup STM32L476G_DISCOVERY_LCD_Private_Functions Private Functions
  * @{
  */
static void Convert(uint8_t *Char, Point_Typedef Point, DoublePoint_Typedef Colon);
static void WriteChar(uint8_t *ch, Point_Typedef Point, DoublePoint_Typedef Colon, DigitPosition_Typedef Position);
static void LCD_MspInit(LCD_HandleTypeDef *hlcd);
static void LCD_MspDeInit(LCD_HandleTypeDef *hlcd);

/**
  * @}
  */

/** @addtogroup STM32L476G_DISCOVERY_LCD_Exported_Functions
  * @{
  */

/**
  * @brief  Initialize the LCD GLASS relative GPIO port IOs and LCD peripheral.
  * @retval None
  */
void BSP_LCD_GLASS_Init(void)
{
  LCDHandle.Instance              = LCD;
  LCDHandle.Init.Prescaler        = LCD_PRESCALER_1;
  LCDHandle.Init.Divider          = LCD_DIVIDER_31;
#if defined (USE_STM32L476G_DISCO_REVC) || defined (USE_STM32L476G_DISCO_REVB)
  LCDHandle.Init.Duty             = LCD_DUTY_1_4;
#elif defined (USE_STM32L476G_DISCO_REVA)
  LCDHandle.Init.Duty             = LCD_DUTY_1_8;
#endif
  LCDHandle.Init.Bias             = LCD_BIAS_1_3;
  LCDHandle.Init.VoltageSource    = LCD_VOLTAGESOURCE_INTERNAL;
  LCDHandle.Init.Contrast         = LCD_CONTRASTLEVEL_5;
  LCDHandle.Init.DeadTime         = LCD_DEADTIME_0;
  LCDHandle.Init.PulseOnDuration  = LCD_PULSEONDURATION_4;
  LCDHandle.Init.HighDrive        = LCD_HIGHDRIVE_DISABLE;
  LCDHandle.Init.BlinkMode        = LCD_BLINKMODE_OFF;
  LCDHandle.Init.BlinkFrequency   = LCD_BLINKFREQUENCY_DIV32;
  LCDHandle.Init.MuxSegment       = LCD_MUXSEGMENT_DISABLE;

  /* Initialize the LCD */
  LCD_MspInit(&LCDHandle);
  HAL_LCD_Init(&LCDHandle);

  BSP_LCD_GLASS_Clear();
}

/**
  * @brief  DeInitialize the LCD GLASS relative GPIO port IOs and LCD peripheral.
  * @retval None
  */
void BSP_LCD_GLASS_DeInit(void)
{
  /* De-Initialize the LCD */
  LCD_MspDeInit(&LCDHandle);
  HAL_LCD_DeInit(&LCDHandle);
}


/**
  * @brief  Configure the LCD Blink mode and Blink frequency.
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
  * @brief  Configure the LCD contrast.
  * @param  Contrast: specifies the LCD contrast value.
  *   This parameter can be one of the following values:
  *     @arg LCD_CONTRASTLEVEL_0: Maximum Voltage = 2.60V
  *     @arg LCD_CONTRASTLEVEL_1: Maximum Voltage = 2.73V
  *     @arg LCD_CONTRASTLEVEL_2: Maximum Voltage = 2.86V
  *     @arg LCD_CONTRASTLEVEL_3: Maximum Voltage = 2.99V
  *     @arg LCD_CONTRASTLEVEL_4: Maximum Voltage = 3.12V
  *     @arg LCD_CONTRASTLEVEL_5: Maximum Voltage = 3.25V
  *     @arg LCD_CONTRASTLEVEL_6: Maximum Voltage = 3.38V
  *     @arg LCD_CONTRASTLEVEL_7: Maximum Voltage = 3.51V
  * @retval None
  */
void BSP_LCD_GLASS_Contrast(uint32_t Contrast)
{
  __HAL_LCD_CONTRAST_CONFIG(&LCDHandle, Contrast);
}

/**
  * @brief Display one or several bar in LCD frame buffer.
  * @param BarId: specifies the LCD GLASS Bar to display
  *     This parameter can be one of the following values:
  *     @arg BAR0: LCD GLASS Bar 0
  *     @arg BAR0: LCD GLASS Bar 1
  *     @arg BAR0: LCD GLASS Bar 2
  *     @arg BAR0: LCD GLASS Bar 3
  * @retval None
  */
void BSP_LCD_GLASS_DisplayBar(uint32_t BarId)
{
  uint32_t position = 0;

  /* Check which bar is selected */
  while ((BarId) >> position)
  {
    /* Check if current bar is selected */
    switch (BarId & (1 << position))
    {
        /* Bar 0 */
      case LCD_BAR_0:
        /* Set BAR0 */
        HAL_LCD_Write(&LCDHandle, LCD_BAR0_2_COM, ~(LCD_BAR0_SEG), LCD_BAR0_SEG);
        break;

        /* Bar 1 */
      case LCD_BAR_1:
        /* Set BAR1 */
        HAL_LCD_Write(&LCDHandle, LCD_BAR1_3_COM, ~(LCD_BAR1_SEG), LCD_BAR1_SEG);
        break;

        /* Bar 2 */
      case LCD_BAR_2:
        /* Set BAR2 */
        HAL_LCD_Write(&LCDHandle, LCD_BAR0_2_COM, ~(LCD_BAR2_SEG), LCD_BAR2_SEG);
        break;

        /* Bar 3 */
      case LCD_BAR_3:
        /* Set BAR3 */
        HAL_LCD_Write(&LCDHandle, LCD_BAR1_3_COM, ~(LCD_BAR3_SEG), LCD_BAR3_SEG);
        break;

      default:
        break;
    }
    position++;
  }

  /* Update the LCD display */
  HAL_LCD_UpdateDisplayRequest(&LCDHandle);
}

/**
  * @brief Clear one or several bar in LCD frame buffer.
  * @param BarId: specifies the LCD GLASS Bar to display
  *     This parameter can be combination of one of the following values:
  *     @arg LCD_BAR_0: LCD GLASS Bar 0
  *     @arg LCD_BAR_1: LCD GLASS Bar 1
  *     @arg LCD_BAR_2: LCD GLASS Bar 2
  *     @arg LCD_BAR_3: LCD GLASS Bar 3
  * @retval None
  */
void BSP_LCD_GLASS_ClearBar(uint32_t BarId)
{
  uint32_t position = 0;

  /* Check which bar is selected */
  while ((BarId) >> position)
  {
    /* Check if current bar is selected */
    switch (BarId & (1 << position))
    {
        /* Bar 0 */
      case LCD_BAR_0:
        /* Set BAR0 */
        HAL_LCD_Write(&LCDHandle, LCD_BAR0_2_COM, ~(LCD_BAR0_SEG) , 0);
        break;

        /* Bar 1 */
      case LCD_BAR_1:
        /* Set BAR1 */
        HAL_LCD_Write(&LCDHandle, LCD_BAR1_3_COM, ~(LCD_BAR1_SEG), 0);
        break;

        /* Bar 2 */
      case LCD_BAR_2:
        /* Set BAR2 */
        HAL_LCD_Write(&LCDHandle, LCD_BAR0_2_COM, ~(LCD_BAR2_SEG), 0);
        break;

        /* Bar 3 */
      case LCD_BAR_3:
        /* Set BAR3 */
        HAL_LCD_Write(&LCDHandle, LCD_BAR1_3_COM, ~(LCD_BAR3_SEG), 0);
        break;

      default:
        break;
    }
    position++;
  }

  /* Update the LCD display */
  HAL_LCD_UpdateDisplayRequest(&LCDHandle);
}

/**
  * @brief Configure the bar level on LCD by writing bar value in LCD frame buffer.
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
      /* BATTERYLEVEL_OFF */
    case BATTERYLEVEL_OFF:
      /* Set BAR0 & BAR2 off */
      HAL_LCD_Write(&LCDHandle, LCD_BAR0_2_COM, ~(LCD_BAR0_SEG | LCD_BAR2_SEG), 0);
      /* Set BAR1 & BAR3 off */
      HAL_LCD_Write(&LCDHandle, LCD_BAR1_3_COM, ~(LCD_BAR1_SEG | LCD_BAR3_SEG), 0);
      LCDBar = BATTERYLEVEL_OFF;
      break;

      /* BARLEVEL 1/4 */
    case BATTERYLEVEL_1_4:
      /* Set BAR0 on & BAR2 off */
      HAL_LCD_Write(&LCDHandle, LCD_BAR0_2_COM, ~(LCD_BAR0_SEG | LCD_BAR2_SEG), LCD_BAR0_SEG);
      /* Set BAR1 & BAR3 off */
      HAL_LCD_Write(&LCDHandle, LCD_BAR1_3_COM, ~(LCD_BAR1_SEG | LCD_BAR3_SEG), 0);
      LCDBar = BATTERYLEVEL_1_4;
      break;

      /* BARLEVEL 1/2 */
    case BATTERYLEVEL_1_2:
      /* Set BAR0 on & BAR2 off */
      HAL_LCD_Write(&LCDHandle, LCD_BAR0_2_COM, ~(LCD_BAR0_SEG | LCD_BAR2_SEG), LCD_BAR0_SEG);
      /* Set BAR1 on & BAR3 off */
      HAL_LCD_Write(&LCDHandle, LCD_BAR1_3_COM, ~(LCD_BAR1_SEG | LCD_BAR3_SEG), LCD_BAR1_SEG);
      LCDBar = BATTERYLEVEL_1_2;
      break;

      /* Battery Level 3/4 */
    case BATTERYLEVEL_3_4:
      /* Set BAR0 & BAR2 on */
      HAL_LCD_Write(&LCDHandle, LCD_BAR0_2_COM, ~(LCD_BAR0_SEG | LCD_BAR2_SEG), (LCD_BAR0_SEG | LCD_BAR2_SEG));
      /* Set BAR1 on & BAR3 off */
      HAL_LCD_Write(&LCDHandle, LCD_BAR1_3_COM, ~(LCD_BAR1_SEG | LCD_BAR3_SEG), LCD_BAR1_SEG);
      LCDBar = BATTERYLEVEL_3_4;
      break;

      /* BATTERYLEVEL_FULL */
    case BATTERYLEVEL_FULL:
      /* Set BAR0 & BAR2 on */
      HAL_LCD_Write(&LCDHandle, LCD_BAR0_2_COM, ~(LCD_BAR0_SEG | LCD_BAR2_SEG), (LCD_BAR0_SEG | LCD_BAR2_SEG));
      /* Set BAR1 on & BAR3 on */
      HAL_LCD_Write(&LCDHandle, LCD_BAR1_3_COM, ~(LCD_BAR1_SEG | LCD_BAR3_SEG), (LCD_BAR1_SEG | LCD_BAR3_SEG));
      LCDBar = BATTERYLEVEL_FULL;
      break;

    default:
      break;
  }

  /* Update the LCD display */
  HAL_LCD_UpdateDisplayRequest(&LCDHandle);
}

/**
  * @brief  Write a character in the LCD RAM buffer.
  * @param  ch: The character to display.
  * @param  Point: A point to add in front of char.
  *          This parameter can be one of the following values:
  *              @arg POINT_OFF: No point to add in front of char.
  *              @arg POINT_ON: Add a point in front of char.
  * @param  Colon: Flag indicating if a colon character has to be added in front
  *                     of displayed character.
  *          This parameter can be one of the following values:
  *              @arg DOUBLEPOINT_OFF: No colon to add in back of char.
  *              @arg DOUBLEPOINT_ON: Add an colon in back of char.
  * @param  Position: Position in the LCD of the character to write.
  *                   This parameter can be any value in range [1:6].
  * @retval None
  * @note   Required preconditions: The LCD should be cleared before to start the
  *         write operation.
  */
void BSP_LCD_GLASS_DisplayChar(uint8_t *ch, Point_Typedef Point, DoublePoint_Typedef Colon, DigitPosition_Typedef Position)
{
  WriteChar(ch, Point, Colon, Position);

  /* Update the LCD display */
  HAL_LCD_UpdateDisplayRequest(&LCDHandle);
}

/**
  * @brief  Write a character string in the LCD RAM buffer.
  * @param  ptr: Pointer to string to display on the LCD Glass.
  * @retval None
  */
void BSP_LCD_GLASS_DisplayString(uint8_t *ptr)
{
  DigitPosition_Typedef position = LCD_DIGIT_POSITION_1;

  /* Send the string character by character on lCD */
  while ((*ptr != 0) && (position <= LCD_DIGIT_POSITION_6))
  {
    /* Write one character on LCD */
    WriteChar(ptr, POINT_OFF, DOUBLEPOINT_OFF, position);

    /* Point on the next character */
    ptr++;

    /* Increment the character counter */
    position++;
  }
  /* Update the LCD display */
  HAL_LCD_UpdateDisplayRequest(&LCDHandle);
}

/**
  * @brief  Write a character string with decimal point in the LCD RAM buffer.
  * @param  ptr: Pointer to string to display on the LCD Glass.
  * @retval None
  * @note Required preconditions: Char is ASCCI value "ORed" with decimal point or Colon flag
  */
void BSP_LCD_GLASS_DisplayStrDeci(uint16_t *ptr)
{
  DigitPosition_Typedef index = LCD_DIGIT_POSITION_1;
  uint8_t tmpchar = 0;

  /* Send the string character by character on lCD */
  while ((*ptr != 0) && (index <= LCD_DIGIT_POSITION_6))
  {
    tmpchar = (*ptr) & 0x00FF;

    switch ((*ptr) & 0xF000)
    {
      case DOT:
        /* Write one character on LCD with decimal point */
        WriteChar(&tmpchar, POINT_ON, DOUBLEPOINT_OFF, index);
        break;
      case DOUBLE_DOT:
        /* Write one character on LCD with decimal point */
        WriteChar(&tmpchar, POINT_OFF, DOUBLEPOINT_ON, index);
        break;
      default:
        WriteChar(&tmpchar, POINT_OFF, DOUBLEPOINT_OFF, index);
        break;
    }/* Point on the next character */
    ptr++;

    /* Increment the character counter */
    index++;
  }
  /* Update the LCD display */
  HAL_LCD_UpdateDisplayRequest(&LCDHandle);
}

/**
  * @brief  Clear the whole LCD RAM buffer.
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
  * @note   Required preconditions: The LCD should be cleared before to start the
  *         write operation.
  */
void BSP_LCD_GLASS_ScrollSentence(uint8_t *ptr, uint16_t nScroll, uint16_t ScrollSpeed)
{
  uint8_t repetition = 0, nbrchar = 0, sizestr = 0;
  uint8_t *ptr1;
  uint8_t str[6] = "";

  /* Reset interrupt variable in case key was press before entering function */
  bLCDGlass_KeyPressed = 0;

  if (ptr == 0)
  {
    return;
  }

  /* To calculate end of string */
  for (ptr1 = ptr, sizestr = 0; *ptr1 != 0; sizestr++, ptr1++);

  ptr1 = ptr;

  BSP_LCD_GLASS_DisplayString(str);
  HAL_Delay(ScrollSpeed);

  /* To shift the string for scrolling display*/
  for (repetition = 0; repetition < nScroll; repetition++)
  {
    for (nbrchar = 0; nbrchar < sizestr; nbrchar++)
    {
      *(str) = * (ptr1 + ((nbrchar + 1) % sizestr));
      *(str + 1) = * (ptr1 + ((nbrchar + 2) % sizestr));
      *(str + 2) = * (ptr1 + ((nbrchar + 3) % sizestr));
      *(str + 3) = * (ptr1 + ((nbrchar + 4) % sizestr));
      *(str + 4) = * (ptr1 + ((nbrchar + 5) % sizestr));
      *(str + 5) = * (ptr1 + ((nbrchar + 6) % sizestr));
      BSP_LCD_GLASS_Clear();
      BSP_LCD_GLASS_DisplayString(str);

      /* user button pressed stop the scrolling sentence */
      if (bLCDGlass_KeyPressed)
      {
        bLCDGlass_KeyPressed = 0;
        return;
      }
      HAL_Delay(ScrollSpeed);
    }
  }
}

/**
  * @}
  */

/** @addtogroup STM32L476G_DISCOVERY_LCD_Private_Functions
  * @{
  */

/**
  * @brief  Initialize the LCD MSP.
  * @param  hlcd: LCD handle
  * @retval None
  */
static void LCD_MspInit(LCD_HandleTypeDef *hlcd)
{
  GPIO_InitTypeDef  gpioinitstruct = {0};
  RCC_OscInitTypeDef oscinitstruct = {0};
  RCC_PeriphCLKInitTypeDef periphclkstruct = {0};

  /*##-1- Enable PWR  peripheral Clock #######################################*/
  __HAL_RCC_PWR_CLK_ENABLE();

  /*##-2- Configure LSE as RTC clock source ###################################*/
  oscinitstruct.OscillatorType  = RCC_OSCILLATORTYPE_LSE;
  oscinitstruct.PLL.PLLState    = RCC_PLL_NONE;
  oscinitstruct.LSEState        = RCC_LSE_ON;
  if (HAL_RCC_OscConfig(&oscinitstruct) != HAL_OK)
  {
    while (1);
  }

  /*##-3- Select LSE as RTC clock source.##########################*/
  /* Backup domain management is done in RCC function */
  periphclkstruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  periphclkstruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  HAL_RCCEx_PeriphCLKConfig(&periphclkstruct);

  /*##-4- Enable LCD GPIO Clocks #############################################*/
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();


  /*##-5- Configure peripheral GPIO ##########################################*/
  /* Configure Output for LCD */
  /* Port A */
  gpioinitstruct.Pin        = LCD_GPIO_BANKA_PINS;
  gpioinitstruct.Mode       = GPIO_MODE_AF_PP;
  gpioinitstruct.Pull       = GPIO_NOPULL;
  gpioinitstruct.Speed      = GPIO_SPEED_FREQ_VERY_HIGH;
  gpioinitstruct.Alternate  = GPIO_AF11_LCD;
  HAL_GPIO_Init(GPIOA, &gpioinitstruct);

  /* Port B */
  gpioinitstruct.Pin        = LCD_GPIO_BANKB_PINS;
  HAL_GPIO_Init(GPIOB, &gpioinitstruct);

  /* Port C*/
  gpioinitstruct.Pin        = LCD_GPIO_BANKC_PINS;
  HAL_GPIO_Init(GPIOC, &gpioinitstruct);

  /* Port D */
  gpioinitstruct.Pin        = LCD_GPIO_BANKD_PINS;
  HAL_GPIO_Init(GPIOD, &gpioinitstruct);

  /* Wait for the external capacitor Cext which is connected to the VLCD pin is charged
  (approximately 2ms for Cext=1uF) */
  HAL_Delay(2);

  /*##-6- Enable LCD peripheral Clock ########################################*/
  __HAL_RCC_LCD_CLK_ENABLE();
}

/**
  * @brief  DeInitialize the LCD MSP.
  * @param  hlcd: LCD handle
  * @retval None
  */
static void LCD_MspDeInit(LCD_HandleTypeDef *hlcd)
{
  uint32_t gpiopin = 0;

  /*##-1- Enable LCD GPIO Clocks #############################################*/
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*##-1- Configure peripheral GPIO ##########################################*/
  /* Configure Output for LCD */
  /* Port A */
  gpiopin = LCD_GPIO_BANKA_PINS;
  HAL_GPIO_DeInit(GPIOA, gpiopin);

  /* Port B */
  gpiopin = LCD_GPIO_BANKB_PINS;
  HAL_GPIO_DeInit(GPIOB, gpiopin);

  /* Port C*/
  gpiopin = LCD_GPIO_BANKC_PINS;
  HAL_GPIO_DeInit(GPIOC, gpiopin);

  /* Port D */
  gpiopin = LCD_GPIO_BANKD_PINS;
  HAL_GPIO_DeInit(GPIOD, gpiopin);

  /*##-5- Enable LCD peripheral Clock ########################################*/
  __HAL_RCC_LCD_CLK_DISABLE();
}

/**
  * @brief  Convert an ascii char to the a LCD digit.
  * @param  Char: a char to display.
  * @param  Point: a point to add in front of char
  *         This parameter can be: POINT_OFF or POINT_ON
  * @param  Colon : flag indicating if a colon character has to be added in front
  *         of displayed character.
  *         This parameter can be: DOUBLEPOINT_OFF or DOUBLEPOINT_ON.
  * @retval None
  */
static void Convert(uint8_t *Char, Point_Typedef Point, DoublePoint_Typedef Colon)
{
  uint16_t ch = 0 ;
  uint8_t loop = 0, index = 0;

  switch (*Char)
  {
    case ' ' :
      ch = 0x00;
      break;

    case '*':
      ch = C_STAR;
      break;

    case '(' :
      ch = C_OPENPARMAP;
      break;

    case ')' :
      ch = C_CLOSEPARMAP;
      break;

    case 'd' :
      ch = C_DMAP;
      break;

    case 'm' :
      ch = C_MMAP;
      break;

    case 'n' :
      ch = C_NMAP;
      break;

    case 'µ' :
      ch = C_UMAP;
      break;

    case '-' :
      ch = C_MINUS;
      break;

    case '+' :
      ch = C_PLUS;
      break;

    case '/' :
      ch = C_SLATCH;
      break;

    case '°' :
      ch = C_PERCENT_1;
      break;
    case '%' :
      ch = C_PERCENT_2;
      break;
    case 255 :
      ch = C_FULL;
      break ;

    case '0':
    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
    case '6':
    case '7':
    case '8':
    case '9':
      ch = NumberMap[*Char - ASCII_CHAR_0];
      break;

    default:
      /* The character Char is one letter in upper case*/
      if ((*Char < ASCII_CHAR_LEFT_OPEN_BRACKET) && (*Char > ASCII_CHAR_AT_SYMBOL))
      {
        ch = CapLetterMap[*Char - 'A'];
      }
      /* The character Char is one letter in lower case*/
      if ((*Char < ASCII_CHAR_LEFT_OPEN_BRACE) && (*Char > ASCII_CHAR_APOSTROPHE))
      {
        ch = CapLetterMap[*Char - 'a'];
      }
      break;
  }

  /* Set the digital point can be displayed if the point is on */
  if (Point == POINT_ON)
  {
    ch |= 0x0002;
  }

  /* Set the "COL" segment in the character that can be displayed if the colon is on */
  if (Colon == DOUBLEPOINT_ON)
  {
    ch |= 0x0020;
  }

  for (loop = 12, index = 0 ; index < 4; loop -= 4, index++)
  {
    Digit[index] = (ch >> loop) & 0x0f; /*To isolate the less significant digit */
  }
}

/**
  * @brief  Write a character in the LCD frame buffer.
  * @param  ch: the character to display.
  * @param  Point: a point to add in front of char
  *         This parameter can be: POINT_OFF or POINT_ON
  * @param  Colon: flag indicating if a colon character has to be added in front
  *         of displayed character.
  *         This parameter can be: DOUBLEPOINT_OFF or DOUBLEPOINT_ON.
  * @param  Position: position in the LCD of the character to write [1:6]
  * @retval None
  */
static void WriteChar(uint8_t *ch, Point_Typedef Point, DoublePoint_Typedef Colon, DigitPosition_Typedef Position)
{
  uint32_t data = 0x00;
  /* To convert displayed character in segment in array digit */
  Convert(ch, (Point_Typedef)Point, (DoublePoint_Typedef)Colon);

  switch (Position)
  {
      /* Position 1 on LCD (Digit1)*/
    case LCD_DIGIT_POSITION_1:
      data = ((Digit[0] & 0x1) << LCD_SEG0_SHIFT) | (((Digit[0] & 0x2) >> 1) << LCD_SEG1_SHIFT)
             | (((Digit[0] & 0x4) >> 2) << LCD_SEG22_SHIFT) | (((Digit[0] & 0x8) >> 3) << LCD_SEG23_SHIFT);
      HAL_LCD_Write(&LCDHandle, LCD_DIGIT1_COM0, LCD_DIGIT1_COM0_SEG_MASK, data); /* 1G 1B 1M 1E */

      data = ((Digit[1] & 0x1) << LCD_SEG0_SHIFT) | (((Digit[1] & 0x2) >> 1) << LCD_SEG1_SHIFT)
             | (((Digit[1] & 0x4) >> 2) << LCD_SEG22_SHIFT) | (((Digit[1] & 0x8) >> 3) << LCD_SEG23_SHIFT);
      HAL_LCD_Write(&LCDHandle, LCD_DIGIT1_COM1, LCD_DIGIT1_COM1_SEG_MASK, data) ; /* 1F 1A 1C 1D  */

      data = ((Digit[2] & 0x1) << LCD_SEG0_SHIFT) | (((Digit[2] & 0x2) >> 1) << LCD_SEG1_SHIFT)
             | (((Digit[2] & 0x4) >> 2) << LCD_SEG22_SHIFT) | (((Digit[2] & 0x8) >> 3) << LCD_SEG23_SHIFT);
      HAL_LCD_Write(&LCDHandle, LCD_DIGIT1_COM2, LCD_DIGIT1_COM2_SEG_MASK, data) ; /* 1Q 1K 1Col 1P  */

      data = ((Digit[3] & 0x1) << LCD_SEG0_SHIFT) | (((Digit[3] & 0x2) >> 1) << LCD_SEG1_SHIFT)
             | (((Digit[3] & 0x4) >> 2) << LCD_SEG22_SHIFT) | (((Digit[3] & 0x8) >> 3) << LCD_SEG23_SHIFT);
      HAL_LCD_Write(&LCDHandle, LCD_DIGIT1_COM3, LCD_DIGIT1_COM3_SEG_MASK, data) ; /* 1H 1J 1DP 1N  */
      break;

      /* Position 2 on LCD (Digit2)*/
    case LCD_DIGIT_POSITION_2:
      data = ((Digit[0] & 0x1) << LCD_SEG2_SHIFT) | (((Digit[0] & 0x2) >> 1) << LCD_SEG3_SHIFT)
             | (((Digit[0] & 0x4) >> 2) << LCD_SEG20_SHIFT) | (((Digit[0] & 0x8) >> 3) << LCD_SEG21_SHIFT);
      HAL_LCD_Write(&LCDHandle, LCD_DIGIT2_COM0, LCD_DIGIT2_COM0_SEG_MASK, data); /* 1G 1B 1M 1E */

      data = ((Digit[1] & 0x1) << LCD_SEG2_SHIFT) | (((Digit[1] & 0x2) >> 1) << LCD_SEG3_SHIFT)
             | (((Digit[1] & 0x4) >> 2) << LCD_SEG20_SHIFT) | (((Digit[1] & 0x8) >> 3) << LCD_SEG21_SHIFT);
      HAL_LCD_Write(&LCDHandle, LCD_DIGIT2_COM1, LCD_DIGIT2_COM1_SEG_MASK, data) ; /* 1F 1A 1C 1D  */

      data = ((Digit[2] & 0x1) << LCD_SEG2_SHIFT) | (((Digit[2] & 0x2) >> 1) << LCD_SEG3_SHIFT)
             | (((Digit[2] & 0x4) >> 2) << LCD_SEG20_SHIFT) | (((Digit[2] & 0x8) >> 3) << LCD_SEG21_SHIFT);
      HAL_LCD_Write(&LCDHandle, LCD_DIGIT2_COM2, LCD_DIGIT2_COM2_SEG_MASK, data) ; /* 1Q 1K 1Col 1P  */

      data = ((Digit[3] & 0x1) << LCD_SEG2_SHIFT) | (((Digit[3] & 0x2) >> 1) << LCD_SEG3_SHIFT)
             | (((Digit[3] & 0x4) >> 2) << LCD_SEG20_SHIFT) | (((Digit[3] & 0x8) >> 3) << LCD_SEG21_SHIFT);
      HAL_LCD_Write(&LCDHandle, LCD_DIGIT2_COM3, LCD_DIGIT2_COM3_SEG_MASK, data) ; /* 1H 1J 1DP 1N  */
      break;

      /* Position 3 on LCD (Digit3)*/
    case LCD_DIGIT_POSITION_3:
      data = ((Digit[0] & 0x1) << LCD_SEG4_SHIFT) | (((Digit[0] & 0x2) >> 1) << LCD_SEG5_SHIFT)
             | (((Digit[0] & 0x4) >> 2) << LCD_SEG18_SHIFT) | (((Digit[0] & 0x8) >> 3) << LCD_SEG19_SHIFT);
      HAL_LCD_Write(&LCDHandle, LCD_DIGIT3_COM0, LCD_DIGIT3_COM0_SEG_MASK, data); /* 1G 1B 1M 1E */

      data = ((Digit[1] & 0x1) << LCD_SEG4_SHIFT) | (((Digit[1] & 0x2) >> 1) << LCD_SEG5_SHIFT)
             | (((Digit[1] & 0x4) >> 2) << LCD_SEG18_SHIFT) | (((Digit[1] & 0x8) >> 3) << LCD_SEG19_SHIFT);
      HAL_LCD_Write(&LCDHandle, LCD_DIGIT3_COM1, LCD_DIGIT3_COM1_SEG_MASK, data) ; /* 1F 1A 1C 1D  */

      data = ((Digit[2] & 0x1) << LCD_SEG4_SHIFT) | (((Digit[2] & 0x2) >> 1) << LCD_SEG5_SHIFT)
             | (((Digit[2] & 0x4) >> 2) << LCD_SEG18_SHIFT) | (((Digit[2] & 0x8) >> 3) << LCD_SEG19_SHIFT);
      HAL_LCD_Write(&LCDHandle, LCD_DIGIT3_COM2, LCD_DIGIT3_COM2_SEG_MASK, data) ; /* 1Q 1K 1Col 1P  */

      data = ((Digit[3] & 0x1) << LCD_SEG4_SHIFT) | (((Digit[3] & 0x2) >> 1) << LCD_SEG5_SHIFT)
             | (((Digit[3] & 0x4) >> 2) << LCD_SEG18_SHIFT) | (((Digit[3] & 0x8) >> 3) << LCD_SEG19_SHIFT);
      HAL_LCD_Write(&LCDHandle, LCD_DIGIT3_COM3, LCD_DIGIT3_COM3_SEG_MASK, data) ; /* 1H 1J 1DP 1N  */
      break;

      /* Position 4 on LCD (Digit4)*/
    case LCD_DIGIT_POSITION_4:
      data = ((Digit[0] & 0x1) << LCD_SEG6_SHIFT) | (((Digit[0] & 0x8) >> 3) << LCD_SEG17_SHIFT);
      HAL_LCD_Write(&LCDHandle, LCD_DIGIT4_COM0, LCD_DIGIT4_COM0_SEG_MASK, data); /* 1G 1B 1M 1E */

      data = (((Digit[0] & 0x2) >> 1) << LCD_SEG7_SHIFT) | (((Digit[0] & 0x4) >> 2) << LCD_SEG16_SHIFT);
      HAL_LCD_Write(&LCDHandle, LCD_DIGIT4_COM0_1, LCD_DIGIT4_COM0_1_SEG_MASK, data); /* 1G 1B 1M 1E */

      data = ((Digit[1] & 0x1) << LCD_SEG6_SHIFT) | (((Digit[1] & 0x8) >> 3) << LCD_SEG17_SHIFT);
      HAL_LCD_Write(&LCDHandle, LCD_DIGIT4_COM1, LCD_DIGIT4_COM1_SEG_MASK, data) ; /* 1F 1A 1C 1D  */

      data = (((Digit[1] & 0x2) >> 1) << LCD_SEG7_SHIFT) | (((Digit[1] & 0x4) >> 2) << LCD_SEG16_SHIFT);
      HAL_LCD_Write(&LCDHandle, LCD_DIGIT4_COM1_1, LCD_DIGIT4_COM1_1_SEG_MASK, data) ; /* 1F 1A 1C 1D  */

      data = ((Digit[2] & 0x1) << LCD_SEG6_SHIFT) | (((Digit[2] & 0x8) >> 3) << LCD_SEG17_SHIFT);
      HAL_LCD_Write(&LCDHandle, LCD_DIGIT4_COM2, LCD_DIGIT4_COM2_SEG_MASK, data) ; /* 1Q 1K 1Col 1P  */

      data = (((Digit[2] & 0x2) >> 1) << LCD_SEG7_SHIFT) | (((Digit[2] & 0x4) >> 2) << LCD_SEG16_SHIFT);
      HAL_LCD_Write(&LCDHandle, LCD_DIGIT4_COM2_1, LCD_DIGIT4_COM2_1_SEG_MASK, data) ; /* 1Q 1K 1Col 1P  */

      data = ((Digit[3] & 0x1) << LCD_SEG6_SHIFT) | (((Digit[3] & 0x8) >> 3) << LCD_SEG17_SHIFT);
      HAL_LCD_Write(&LCDHandle, LCD_DIGIT4_COM3, LCD_DIGIT4_COM3_SEG_MASK, data) ; /* 1H 1J 1DP 1N  */

      data = (((Digit[3] & 0x2) >> 1) << LCD_SEG7_SHIFT) | (((Digit[3] & 0x4) >> 2) << LCD_SEG16_SHIFT);
      HAL_LCD_Write(&LCDHandle, LCD_DIGIT4_COM3_1, LCD_DIGIT4_COM3_1_SEG_MASK, data) ; /* 1H 1J 1DP 1N  */
      break;

      /* Position 5 on LCD (Digit5)*/
    case LCD_DIGIT_POSITION_5:
      data = (((Digit[0] & 0x2) >> 1) << LCD_SEG9_SHIFT) | (((Digit[0] & 0x4) >> 2) << LCD_SEG14_SHIFT);
      HAL_LCD_Write(&LCDHandle, LCD_DIGIT5_COM0, LCD_DIGIT5_COM0_SEG_MASK, data); /* 1G 1B 1M 1E */

      data = ((Digit[0] & 0x1) << LCD_SEG8_SHIFT) | (((Digit[0] & 0x8) >> 3) << LCD_SEG15_SHIFT);
      HAL_LCD_Write(&LCDHandle, LCD_DIGIT5_COM0_1, LCD_DIGIT5_COM0_1_SEG_MASK, data); /* 1G 1B 1M 1E */

      data = (((Digit[1] & 0x2) >> 1) << LCD_SEG9_SHIFT) | (((Digit[1] & 0x4) >> 2) << LCD_SEG14_SHIFT);
      HAL_LCD_Write(&LCDHandle, LCD_DIGIT5_COM1, LCD_DIGIT5_COM1_SEG_MASK, data) ; /* 1F 1A 1C 1D  */

      data = ((Digit[1] & 0x1) << LCD_SEG8_SHIFT) | (((Digit[1] & 0x8) >> 3) << LCD_SEG15_SHIFT);
      HAL_LCD_Write(&LCDHandle, LCD_DIGIT5_COM1_1, LCD_DIGIT5_COM1_1_SEG_MASK, data) ; /* 1F 1A 1C 1D  */

      data = (((Digit[2] & 0x2) >> 1) << LCD_SEG9_SHIFT) | (((Digit[2] & 0x4) >> 2) << LCD_SEG14_SHIFT);
      HAL_LCD_Write(&LCDHandle, LCD_DIGIT5_COM2, LCD_DIGIT5_COM2_SEG_MASK, data) ; /* 1Q 1K 1Col 1P  */

      data = ((Digit[2] & 0x1) << LCD_SEG8_SHIFT) | (((Digit[2] & 0x8) >> 3) << LCD_SEG15_SHIFT);
      HAL_LCD_Write(&LCDHandle, LCD_DIGIT5_COM2_1, LCD_DIGIT5_COM2_1_SEG_MASK, data) ; /* 1Q 1K 1Col 1P  */

      data = (((Digit[3] & 0x2) >> 1) << LCD_SEG9_SHIFT) | (((Digit[3] & 0x4) >> 2) << LCD_SEG14_SHIFT);
      HAL_LCD_Write(&LCDHandle, LCD_DIGIT5_COM3, LCD_DIGIT5_COM3_SEG_MASK, data) ; /* 1H 1J 1DP 1N  */

      data = ((Digit[3] & 0x1) << LCD_SEG8_SHIFT) | (((Digit[3] & 0x8) >> 3) << LCD_SEG15_SHIFT);
      HAL_LCD_Write(&LCDHandle, LCD_DIGIT5_COM3_1, LCD_DIGIT5_COM3_1_SEG_MASK, data) ; /* 1H 1J 1DP 1N  */
      break;

      /* Position 6 on LCD (Digit6)*/
    case LCD_DIGIT_POSITION_6:
      data = ((Digit[0] & 0x1) << LCD_SEG10_SHIFT) | (((Digit[0] & 0x2) >> 1) << LCD_SEG11_SHIFT)
             | (((Digit[0] & 0x4) >> 2) << LCD_SEG12_SHIFT) | (((Digit[0] & 0x8) >> 3) << LCD_SEG13_SHIFT);
      HAL_LCD_Write(&LCDHandle, LCD_DIGIT6_COM0, LCD_DIGIT6_COM0_SEG_MASK, data); /* 1G 1B 1M 1E */

      data = ((Digit[1] & 0x1) << LCD_SEG10_SHIFT) | (((Digit[1] & 0x2) >> 1) << LCD_SEG11_SHIFT)
             | (((Digit[1] & 0x4) >> 2) << LCD_SEG12_SHIFT) | (((Digit[1] & 0x8) >> 3) << LCD_SEG13_SHIFT);
      HAL_LCD_Write(&LCDHandle, LCD_DIGIT6_COM1, LCD_DIGIT6_COM1_SEG_MASK, data) ; /* 1F 1A 1C 1D  */

      data = ((Digit[2] & 0x1) << LCD_SEG10_SHIFT) | (((Digit[2] & 0x2) >> 1) << LCD_SEG11_SHIFT)
             | (((Digit[2] & 0x4) >> 2) << LCD_SEG12_SHIFT) | (((Digit[2] & 0x8) >> 3) << LCD_SEG13_SHIFT);
      HAL_LCD_Write(&LCDHandle, LCD_DIGIT6_COM2, LCD_DIGIT6_COM2_SEG_MASK, data) ; /* 1Q 1K 1Col 1P  */

      data = ((Digit[3] & 0x1) << LCD_SEG10_SHIFT) | (((Digit[3] & 0x2) >> 1) << LCD_SEG11_SHIFT)
             | (((Digit[3] & 0x4) >> 2) << LCD_SEG12_SHIFT) | (((Digit[3] & 0x8) >> 3) << LCD_SEG13_SHIFT);
      HAL_LCD_Write(&LCDHandle, LCD_DIGIT6_COM3, LCD_DIGIT6_COM3_SEG_MASK, data) ; /* 1H 1J 1DP 1N  */
      break;

    default:
      break;
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


