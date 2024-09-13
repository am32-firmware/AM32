/**
 ******************************************************************************
 * @file    stm32l476g_discovery_glass_lcd.h
 * @author  MCD Application Team
 * @brief   Header file for stm32l476g_discovery_glass_lcd.c module.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32L476G_DISCOVERY_GLASS_LCD_H
#define __STM32L476G_DISCOVERY_GLASS_LCD_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l476g_discovery.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L476G_DISCOVERY
  * @{
  */

/** @addtogroup STM32L476G_DISCOVERY_GLASS_LCD
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/** @defgroup STM32L476G_DISCOVERY_GLASS_LCD_Exported_Types Exported Types
  * @{
  */
/**
  * @brief LCD Glass digit position
  */
typedef enum
{
  LCD_DIGIT_POSITION_1 = 0,
  LCD_DIGIT_POSITION_2 = 1,
  LCD_DIGIT_POSITION_3 = 2,
  LCD_DIGIT_POSITION_4 = 3,
  LCD_DIGIT_POSITION_5 = 4,
  LCD_DIGIT_POSITION_6 = 5,
  LCD_DIGIT_MAX_NUMBER = 6,
}
DigitPosition_Typedef;
/**
  * @brief LCD Glass point
  * Warning: element values correspond to LCD Glass point.
  */

typedef enum
{
  POINT_OFF = 0,
  POINT_ON = 1
} Point_Typedef;

/**
  * @brief LCD Glass Double point
  * Warning: element values correspond to LCD Glass Double point.
  */
typedef enum
{
  DOUBLEPOINT_OFF = 0,
  DOUBLEPOINT_ON = 1
} DoublePoint_Typedef;

/**
  * @brief LCD Glass Battery Level
  * element values correspond to different LCD Glass battery levels
  */
typedef enum
{
  BATTERYLEVEL_OFF = 0,
  BATTERYLEVEL_1_4 = 1,
  BATTERYLEVEL_1_2 = 2,
  BATTERYLEVEL_3_4 = 3,
  BATTERYLEVEL_FULL = 4
} BatteryLevel_Typedef;

/**
  * @brief LCD Glass Bar Id
  */
typedef enum
{
  LCD_BAR_NONE  = 0,
  LCD_BAR_0     = (1 << 0),
  LCD_BAR_1     = (1 << 1),
  LCD_BAR_2     = (1 << 2),
  LCD_BAR_3     = (1 << 3)
} BarId_Typedef;
/**
  * @}
  */

/** @defgroup STM32L476G_DISCOVERY_GLASS_LCD_Exported_Constants Exported Constants
  * @{
*/
/**
  * @brief LCD digit definition
  */
#define COM_PER_DIGIT_NB          4/*!< Specifies number of COM to address a digit */
#define SEG_PER_DIGIT_NB          4/*!< Specifies number of SEG to address a digit */

#define LCD_MAP_CHAR_COM0_SEG_1ST_POS   (1 << LCD_MAP_CHAR_COM0_SEG_1ST_SHIFT)
#define LCD_MAP_CHAR_COM0_SEG_2ND_POS   (1 << LCD_MAP_CHAR_COM0_SEG_2ND_SHIFT)
#define LCD_MAP_CHAR_COM0_SEG_3RD_POS   (1 << LCD_MAP_CHAR_COM0_SEG_3RD_SHIFT)
#define LCD_MAP_CHAR_COM0_SEG_4TH_POS   (1 << LCD_MAP_CHAR_COM0_SEG_4TH_SHIFT)
#define LCD_MAP_CHAR_COM1_SEG_1ST_POS   (1 << LCD_MAP_CHAR_COM1_SEG_1ST_SHIFT)
#define LCD_MAP_CHAR_COM1_SEG_2ND_POS   (1 << LCD_MAP_CHAR_COM1_SEG_2ND_SHIFT)
#define LCD_MAP_CHAR_COM1_SEG_3RD_POS   (1 << LCD_MAP_CHAR_COM1_SEG_3RD_SHIFT)
#define LCD_MAP_CHAR_COM1_SEG_4TH_POS   (1 << LCD_MAP_CHAR_COM1_SEG_4TH_SHIFT)
#define LCD_MAP_CHAR_COM2_SEG_1ST_POS   (1 << LCD_MAP_CHAR_COM2_SEG_1ST_SHIFT)
#define LCD_MAP_CHAR_COM2_SEG_2ND_POS   (1 << LCD_MAP_CHAR_COM2_SEG_2ND_SHIFT)
#define LCD_MAP_CHAR_COM2_SEG_3RD_POS   (1 << LCD_MAP_CHAR_COM2_SEG_3RD_SHIFT)
#define LCD_MAP_CHAR_COM2_SEG_4TH_POS   (1 << LCD_MAP_CHAR_COM2_SEG_4TH_SHIFT)
#define LCD_MAP_CHAR_COM3_SEG_1ST_POS   (1 << LCD_MAP_CHAR_COM3_SEG_1ST_SHIFT)
#define LCD_MAP_CHAR_COM3_SEG_2ND_POS   (1 << LCD_MAP_CHAR_COM3_SEG_2ND_SHIFT)
#define LCD_MAP_CHAR_COM3_SEG_3RD_POS   (1 << LCD_MAP_CHAR_COM3_SEG_3RD_SHIFT)
#define LCD_MAP_CHAR_COM3_SEG_4TH_POS   (1 << LCD_MAP_CHAR_COM3_SEG_4TH_SHIFT)
#define LCD_MAP_CHAR_COM0_SEG_1ST_SHIFT 0x00000000
#define LCD_MAP_CHAR_COM0_SEG_2ND_SHIFT 0x00000001
#define LCD_MAP_CHAR_COM0_SEG_3RD_SHIFT 0x00000002
#define LCD_MAP_CHAR_COM0_SEG_4TH_SHIFT 0x00000003
#define LCD_MAP_CHAR_COM1_SEG_1ST_SHIFT 0x00000004
#define LCD_MAP_CHAR_COM1_SEG_2ND_SHIFT 0x00000005
#define LCD_MAP_CHAR_COM1_SEG_3RD_SHIFT 0x00000006
#define LCD_MAP_CHAR_COM1_SEG_4TH_SHIFT 0x00000007
#define LCD_MAP_CHAR_COM2_SEG_1ST_SHIFT 0x00000008
#define LCD_MAP_CHAR_COM2_SEG_2ND_SHIFT 0x00000009
#define LCD_MAP_CHAR_COM2_SEG_3RD_SHIFT 0x00000010
#define LCD_MAP_CHAR_COM2_SEG_4TH_SHIFT 0x00000011
#define LCD_MAP_CHAR_COM3_SEG_1ST_SHIFT 0x00000012
#define LCD_MAP_CHAR_COM3_SEG_2ND_SHIFT 0x00000013
#define LCD_MAP_CHAR_COM3_SEG_3RD_SHIFT 0x00000014
#define LCD_MAP_CHAR_COM3_SEG_4TH_SHIFT 0x00000015

/**
  * @brief LCD Digit defines
  */
#define LCD_DIGIT1_COM0               LCD_COM0
#define LCD_DIGIT1_COM0_SEG_MASK      ~(LCD_SEG0 | LCD_SEG1 | LCD_SEG22 | LCD_SEG23)
#define LCD_DIGIT1_COM1               LCD_COM1
#define LCD_DIGIT1_COM1_SEG_MASK      ~(LCD_SEG0 | LCD_SEG1 | LCD_SEG22 | LCD_SEG23)
#define LCD_DIGIT1_COM2               LCD_COM2
#define LCD_DIGIT1_COM2_SEG_MASK      ~(LCD_SEG0 | LCD_SEG1 | LCD_SEG22 | LCD_SEG23)
#define LCD_DIGIT1_COM3               LCD_COM3
#define LCD_DIGIT1_COM3_SEG_MASK      ~(LCD_SEG0 | LCD_SEG1 | LCD_SEG22 | LCD_SEG23)

#define LCD_DIGIT2_COM0               LCD_COM0
#define LCD_DIGIT2_COM0_SEG_MASK      ~(LCD_SEG2 | LCD_SEG3 | LCD_SEG20 | LCD_SEG21)
#define LCD_DIGIT2_COM1               LCD_COM1
#define LCD_DIGIT2_COM1_SEG_MASK      ~(LCD_SEG2 | LCD_SEG3 | LCD_SEG20 | LCD_SEG21)
#define LCD_DIGIT2_COM2               LCD_COM2
#define LCD_DIGIT2_COM2_SEG_MASK      ~(LCD_SEG2 | LCD_SEG3 | LCD_SEG20 | LCD_SEG21)
#define LCD_DIGIT2_COM3               LCD_COM3
#define LCD_DIGIT2_COM3_SEG_MASK      ~(LCD_SEG2 | LCD_SEG3 | LCD_SEG20 | LCD_SEG21)

#define LCD_DIGIT3_COM0               LCD_COM0
#define LCD_DIGIT3_COM0_SEG_MASK      ~(LCD_SEG4 | LCD_SEG5 | LCD_SEG18 | LCD_SEG19)
#define LCD_DIGIT3_COM1               LCD_COM1
#define LCD_DIGIT3_COM1_SEG_MASK      ~(LCD_SEG4 | LCD_SEG5 | LCD_SEG18 | LCD_SEG19)
#define LCD_DIGIT3_COM2               LCD_COM2
#define LCD_DIGIT3_COM2_SEG_MASK      ~(LCD_SEG4 | LCD_SEG5 | LCD_SEG18 | LCD_SEG19)
#define LCD_DIGIT3_COM3               LCD_COM3
#define LCD_DIGIT3_COM3_SEG_MASK      ~(LCD_SEG4 | LCD_SEG5 | LCD_SEG18 | LCD_SEG19)

#define LCD_DIGIT4_COM0               LCD_COM0
#define LCD_DIGIT4_COM0_SEG_MASK      ~(LCD_SEG6 | LCD_SEG17)
#define LCD_DIGIT4_COM0_1             LCD_COM0_1
#define LCD_DIGIT4_COM0_1_SEG_MASK    ~(LCD_SEG7 | LCD_SEG16)
#define LCD_DIGIT4_COM1               LCD_COM1
#define LCD_DIGIT4_COM1_SEG_MASK      ~(LCD_SEG6 |  LCD_SEG17)
#define LCD_DIGIT4_COM1_1             LCD_COM1_1
#define LCD_DIGIT4_COM1_1_SEG_MASK    ~(LCD_SEG7 | LCD_SEG16)
#define LCD_DIGIT4_COM2               LCD_COM2
#define LCD_DIGIT4_COM2_SEG_MASK      ~(LCD_SEG6 | LCD_SEG17)
#define LCD_DIGIT4_COM2_1             LCD_COM2_1
#define LCD_DIGIT4_COM2_1_SEG_MASK    ~(LCD_SEG7 | LCD_SEG16)
#define LCD_DIGIT4_COM3               LCD_COM3
#define LCD_DIGIT4_COM3_SEG_MASK      ~(LCD_SEG6 | LCD_SEG17)
#define LCD_DIGIT4_COM3_1             LCD_COM3_1
#define LCD_DIGIT4_COM3_1_SEG_MASK    ~(LCD_SEG7 | LCD_SEG16)

#define LCD_DIGIT5_COM0               LCD_COM0
#define LCD_DIGIT5_COM0_SEG_MASK      ~(LCD_SEG9 | LCD_SEG14)
#define LCD_DIGIT5_COM0_1             LCD_COM0_1
#define LCD_DIGIT5_COM0_1_SEG_MASK    ~(LCD_SEG8 | LCD_SEG15)
#define LCD_DIGIT5_COM1               LCD_COM1
#define LCD_DIGIT5_COM1_SEG_MASK      ~(LCD_SEG9 | LCD_SEG14)
#define LCD_DIGIT5_COM1_1             LCD_COM1_1
#define LCD_DIGIT5_COM1_1_SEG_MASK    ~(LCD_SEG8 | LCD_SEG15)
#define LCD_DIGIT5_COM2               LCD_COM2
#define LCD_DIGIT5_COM2_SEG_MASK      ~(LCD_SEG9 | LCD_SEG14)
#define LCD_DIGIT5_COM2_1             LCD_COM2_1
#define LCD_DIGIT5_COM2_1_SEG_MASK    ~(LCD_SEG8 | LCD_SEG15)
#define LCD_DIGIT5_COM3               LCD_COM3
#define LCD_DIGIT5_COM3_SEG_MASK      ~(LCD_SEG9 | LCD_SEG14)
#define LCD_DIGIT5_COM3_1             LCD_COM3_1
#define LCD_DIGIT5_COM3_1_SEG_MASK    ~(LCD_SEG8 | LCD_SEG15)

#define LCD_DIGIT6_COM0               LCD_COM0
#define LCD_DIGIT6_COM0_SEG_MASK      ~(LCD_SEG10 | LCD_SEG11 | LCD_SEG12 | LCD_SEG13)
#define LCD_DIGIT6_COM1               LCD_COM1
#define LCD_DIGIT6_COM1_SEG_MASK      ~(LCD_SEG10 | LCD_SEG11 | LCD_SEG12 | LCD_SEG13)
#define LCD_DIGIT6_COM2               LCD_COM2
#define LCD_DIGIT6_COM2_SEG_MASK      ~(LCD_SEG10 | LCD_SEG11 | LCD_SEG12 | LCD_SEG13)
#define LCD_DIGIT6_COM3               LCD_COM3
#define LCD_DIGIT6_COM3_SEG_MASK      ~(LCD_SEG10 | LCD_SEG11 | LCD_SEG12 | LCD_SEG13)

/**
  * @brief LCD Bar location
  */
#define LCD_BAR0_2_COM            LCD_COM3
#define LCD_BAR1_3_COM            LCD_COM2
#define LCD_BAR0_SEG              LCD_SEG11
#define LCD_BAR1_SEG              LCD_SEG11
#define LCD_BAR2_SEG              LCD_SEG9
#define LCD_BAR3_SEG              LCD_SEG9
#define LCD_BAR0_2_SEG_MASK       ~(LCD_BAR0_SEG | LCD_BAR2_SEG)
#define LCD_BAR1_3_SEG_MASK       ~(LCD_BAR1_SEG | LCD_BAR3_SEG)

/**
  * @brief LCD segments & coms redefinition.
  * LCD component segments & coms are not necessarily link to MCU segmnents & coms output.
  */
#if defined (USE_STM32L476G_DISCO_REVC) || defined (USE_STM32L476G_DISCO_REVB)
#define LCD_COM0          MCU_LCD_COM0
#define LCD_COM0_1        MCU_LCD_COM0_1
#define LCD_COM1          MCU_LCD_COM1
#define LCD_COM1_1        MCU_LCD_COM1_1
#define LCD_COM2          MCU_LCD_COM2
#define LCD_COM2_1        MCU_LCD_COM2_1
#define LCD_COM3          MCU_LCD_COM3
#define LCD_COM3_1        MCU_LCD_COM3_1
#elif defined (USE_STM32L476G_DISCO_REVA)
#define LCD_COM0          MCU_LCD_COM5
#define LCD_COM0_1        MCU_LCD_COM5_1
#define LCD_COM1          MCU_LCD_COM7
#define LCD_COM1_1        MCU_LCD_COM7_1
#define LCD_COM2          MCU_LCD_COM6
#define LCD_COM2_1        MCU_LCD_COM6_1
#define LCD_COM3          MCU_LCD_COM4
#define LCD_COM3_1        MCU_LCD_COM4_1
#endif
#define LCD_SEG0          MCU_LCD_SEG4
#define LCD_SEG1          MCU_LCD_SEG23
#define LCD_SEG2          MCU_LCD_SEG6
#define LCD_SEG3          MCU_LCD_SEG13
#define LCD_SEG4          MCU_LCD_SEG15
#define LCD_SEG5          MCU_LCD_SEG29
#define LCD_SEG6          MCU_LCD_SEG31
#define LCD_SEG7          MCU_LCD_SEG33
#define LCD_SEG8          MCU_LCD_SEG35
#define LCD_SEG9          MCU_LCD_SEG25
#define LCD_SEG10         MCU_LCD_SEG17
#define LCD_SEG11         MCU_LCD_SEG8
#define LCD_SEG12         MCU_LCD_SEG9
#define LCD_SEG13         MCU_LCD_SEG26
#define LCD_SEG14         MCU_LCD_SEG24
#define LCD_SEG15         MCU_LCD_SEG34
#define LCD_SEG16         MCU_LCD_SEG32
#define LCD_SEG17         MCU_LCD_SEG30
#define LCD_SEG18         MCU_LCD_SEG28
#define LCD_SEG19         MCU_LCD_SEG14
#define LCD_SEG20         MCU_LCD_SEG12
#define LCD_SEG21         MCU_LCD_SEG5
#define LCD_SEG22         MCU_LCD_SEG22
#define LCD_SEG23         MCU_LCD_SEG3
#define LCD_SEG0_SHIFT          MCU_LCD_SEG4_SHIFT
#define LCD_SEG1_SHIFT          MCU_LCD_SEG23_SHIFT
#define LCD_SEG2_SHIFT          MCU_LCD_SEG6_SHIFT
#define LCD_SEG3_SHIFT          MCU_LCD_SEG13_SHIFT
#define LCD_SEG4_SHIFT          MCU_LCD_SEG15_SHIFT
#define LCD_SEG5_SHIFT          MCU_LCD_SEG29_SHIFT
#define LCD_SEG6_SHIFT          MCU_LCD_SEG31_SHIFT
#define LCD_SEG7_SHIFT          MCU_LCD_SEG33_SHIFT
#define LCD_SEG8_SHIFT          MCU_LCD_SEG35_SHIFT
#define LCD_SEG9_SHIFT          MCU_LCD_SEG25_SHIFT
#define LCD_SEG10_SHIFT         MCU_LCD_SEG17_SHIFT
#define LCD_SEG11_SHIFT         MCU_LCD_SEG8_SHIFT
#define LCD_SEG12_SHIFT         MCU_LCD_SEG9_SHIFT
#define LCD_SEG13_SHIFT         MCU_LCD_SEG26_SHIFT
#define LCD_SEG14_SHIFT         MCU_LCD_SEG24_SHIFT
#define LCD_SEG15_SHIFT         MCU_LCD_SEG34_SHIFT
#define LCD_SEG16_SHIFT         MCU_LCD_SEG32_SHIFT
#define LCD_SEG17_SHIFT         MCU_LCD_SEG30_SHIFT
#define LCD_SEG18_SHIFT         MCU_LCD_SEG28_SHIFT
#define LCD_SEG19_SHIFT         MCU_LCD_SEG14_SHIFT
#define LCD_SEG20_SHIFT         MCU_LCD_SEG12_SHIFT
#define LCD_SEG21_SHIFT         MCU_LCD_SEG5_SHIFT
#define LCD_SEG22_SHIFT         MCU_LCD_SEG22_SHIFT
#define LCD_SEG23_SHIFT         MCU_LCD_SEG3_SHIFT

/**
  * @brief STM32 LCD segments & coms definitions.
  */
#define MCU_LCD_COM0          LCD_RAM_REGISTER0
#define MCU_LCD_COM0_1        LCD_RAM_REGISTER1
#define MCU_LCD_COM1          LCD_RAM_REGISTER2
#define MCU_LCD_COM1_1        LCD_RAM_REGISTER3
#define MCU_LCD_COM2          LCD_RAM_REGISTER4
#define MCU_LCD_COM2_1        LCD_RAM_REGISTER5
#define MCU_LCD_COM3          LCD_RAM_REGISTER6
#define MCU_LCD_COM3_1        LCD_RAM_REGISTER7
#define MCU_LCD_COM4          LCD_RAM_REGISTER8
#define MCU_LCD_COM4_1        LCD_RAM_REGISTER9
#define MCU_LCD_COM5          LCD_RAM_REGISTER10
#define MCU_LCD_COM5_1        LCD_RAM_REGISTER11
#define MCU_LCD_COM6          LCD_RAM_REGISTER12
#define MCU_LCD_COM6_1        LCD_RAM_REGISTER13
#define MCU_LCD_COM7          LCD_RAM_REGISTER14
#define MCU_LCD_COM7_1        LCD_RAM_REGISTER15
#define MCU_LCD_SEG0          (1U << MCU_LCD_SEG0_SHIFT)
#define MCU_LCD_SEG1          (1U << MCU_LCD_SEG1_SHIFT)
#define MCU_LCD_SEG2          (1U << MCU_LCD_SEG2_SHIFT)
#define MCU_LCD_SEG3          (1U << MCU_LCD_SEG3_SHIFT)
#define MCU_LCD_SEG4          (1U << MCU_LCD_SEG4_SHIFT)
#define MCU_LCD_SEG5          (1U << MCU_LCD_SEG5_SHIFT)
#define MCU_LCD_SEG6          (1U << MCU_LCD_SEG6_SHIFT)
#define MCU_LCD_SEG7          (1U << MCU_LCD_SEG7_SHIFT)
#define MCU_LCD_SEG8          (1U << MCU_LCD_SEG8_SHIFT)
#define MCU_LCD_SEG9          (1U << MCU_LCD_SEG9_SHIFT)
#define MCU_LCD_SEG10         (1U << MCU_LCD_SEG10_SHIFT)
#define MCU_LCD_SEG11         (1U << MCU_LCD_SEG11_SHIFT)
#define MCU_LCD_SEG12         (1U << MCU_LCD_SEG12_SHIFT)
#define MCU_LCD_SEG13         (1U << MCU_LCD_SEG13_SHIFT)
#define MCU_LCD_SEG14         (1U << MCU_LCD_SEG14_SHIFT)
#define MCU_LCD_SEG15         (1U << MCU_LCD_SEG15_SHIFT)
#define MCU_LCD_SEG16         (1U << MCU_LCD_SEG16_SHIFT)
#define MCU_LCD_SEG17         (1U << MCU_LCD_SEG17_SHIFT)
#define MCU_LCD_SEG18         (1U << MCU_LCD_SEG18_SHIFT)
#define MCU_LCD_SEG19         (1U << MCU_LCD_SEG19_SHIFT)
#define MCU_LCD_SEG20         (1U << MCU_LCD_SEG20_SHIFT)
#define MCU_LCD_SEG21         (1U << MCU_LCD_SEG21_SHIFT)
#define MCU_LCD_SEG22         (1U << MCU_LCD_SEG22_SHIFT)
#define MCU_LCD_SEG23         (1U << MCU_LCD_SEG23_SHIFT)
#define MCU_LCD_SEG24         (1U << MCU_LCD_SEG24_SHIFT)
#define MCU_LCD_SEG25         (1U << MCU_LCD_SEG25_SHIFT)
#define MCU_LCD_SEG26         (1U << MCU_LCD_SEG26_SHIFT)
#define MCU_LCD_SEG27         (1U << MCU_LCD_SEG27_SHIFT)
#define MCU_LCD_SEG28         (1U << MCU_LCD_SEG28_SHIFT)
#define MCU_LCD_SEG29         (1U << MCU_LCD_SEG29_SHIFT)
#define MCU_LCD_SEG30         (1U << MCU_LCD_SEG30_SHIFT)
#define MCU_LCD_SEG31         (1U << MCU_LCD_SEG31_SHIFT)
#define MCU_LCD_SEG32         (1U << MCU_LCD_SEG32_SHIFT)
#define MCU_LCD_SEG33         (1U << MCU_LCD_SEG33_SHIFT)
#define MCU_LCD_SEG34         (1U << MCU_LCD_SEG34_SHIFT)
#define MCU_LCD_SEG35         (1U << MCU_LCD_SEG35_SHIFT)
#define MCU_LCD_SEG36         (1U << MCU_LCD_SEG36_SHIFT)
#define MCU_LCD_SEG37         (1U << MCU_LCD_SEG37_SHIFT)
#define MCU_LCD_SEG38         (1U << MCU_LCD_SEG38_SHIFT)
#define MCU_LCD_SEG0_SHIFT    0
#define MCU_LCD_SEG1_SHIFT    1
#define MCU_LCD_SEG2_SHIFT    2
#define MCU_LCD_SEG3_SHIFT    3
#define MCU_LCD_SEG4_SHIFT    4
#define MCU_LCD_SEG5_SHIFT    5
#define MCU_LCD_SEG6_SHIFT    6
#define MCU_LCD_SEG7_SHIFT    7
#define MCU_LCD_SEG8_SHIFT    8
#define MCU_LCD_SEG9_SHIFT    9
#define MCU_LCD_SEG10_SHIFT   10
#define MCU_LCD_SEG11_SHIFT   11
#define MCU_LCD_SEG12_SHIFT   12
#define MCU_LCD_SEG13_SHIFT   13
#define MCU_LCD_SEG14_SHIFT   14
#define MCU_LCD_SEG15_SHIFT   15
#define MCU_LCD_SEG16_SHIFT   16
#define MCU_LCD_SEG17_SHIFT   17
#define MCU_LCD_SEG18_SHIFT   18
#define MCU_LCD_SEG19_SHIFT   19
#define MCU_LCD_SEG20_SHIFT   20
#define MCU_LCD_SEG21_SHIFT   21
#define MCU_LCD_SEG22_SHIFT   22
#define MCU_LCD_SEG23_SHIFT   23
#define MCU_LCD_SEG24_SHIFT   24
#define MCU_LCD_SEG25_SHIFT   25
#define MCU_LCD_SEG26_SHIFT   26
#define MCU_LCD_SEG27_SHIFT   27
#define MCU_LCD_SEG28_SHIFT   28
#define MCU_LCD_SEG29_SHIFT   29
#define MCU_LCD_SEG30_SHIFT   30
#define MCU_LCD_SEG31_SHIFT   31
#define MCU_LCD_SEG32_SHIFT   0
#define MCU_LCD_SEG33_SHIFT   1
#define MCU_LCD_SEG34_SHIFT   2
#define MCU_LCD_SEG35_SHIFT   3
#define MCU_LCD_SEG36_SHIFT   4
#define MCU_LCD_SEG37_SHIFT   5
#define MCU_LCD_SEG38_SHIFT   6
#define MCU_LCD_SEG39_SHIFT   7
#define MCU_LCD_SEG40_SHIFT   8
#define MCU_LCD_SEG41_SHIFT   9
#define MCU_LCD_SEG42_SHIFT   10
#define MCU_LCD_SEG43_SHIFT   11

/**
  * @brief LCD Pins definition.
  */
#if defined (USE_STM32L476G_DISCO_REVC) || defined (USE_STM32L476G_DISCO_REVB)
#define LCD_GPIO_BANKA_PINS  (GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 |    \
                              GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_15)
#define LCD_GPIO_BANKB_PINS  (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 |    \
                              GPIO_PIN_5 | GPIO_PIN_9 | GPIO_PIN_12 |   \
                              GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15)
#define LCD_GPIO_BANKC_PINS  (GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 |    \
                              GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8)
#define LCD_GPIO_BANKD_PINS  (GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 |   \
                              GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | \
                              GPIO_PIN_14 | GPIO_PIN_15)
#elif defined (USE_STM32L476G_DISCO_REVA)
#define LCD_GPIO_BANKA_PINS  (GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_15)
#define LCD_GPIO_BANKB_PINS  (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 |    \
                              GPIO_PIN_5 | GPIO_PIN_12 | GPIO_PIN_13 |  \
                              GPIO_PIN_14 | GPIO_PIN_15)
#define LCD_GPIO_BANKC_PINS  (GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 |    \
                              GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 |    \
                              GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12)
#define LCD_GPIO_BANKD_PINS  (GPIO_PIN_2 | GPIO_PIN_8 | GPIO_PIN_9 |    \
                              GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | \
                              GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15)
#endif

/* Define for scrolling sentences*/
#define SCROLL_SPEED_HIGH     150
#define SCROLL_SPEED_MEDIUM   300
#define SCROLL_SPEED_LOW      450

#define DOT                   ((uint16_t) 0x8000 ) /* for add decimal point in string */
#define DOUBLE_DOT            ((uint16_t) 0x4000) /* for add decimal point in string */

/* code for '(' character */
#define C_OPENPARMAP          ((uint16_t) 0x0028)

/* code for ')' character */
#define C_CLOSEPARMAP         ((uint16_t) 0x0011)

/* code for 'd' character */
#define C_DMAP                ((uint16_t) 0xf300)

/* code for 'm' character */
#define C_MMAP                ((uint16_t) 0xb210)

/* code for 'n' character */
#define C_NMAP                ((uint16_t) 0x2210)

/* code for 'µ' character */
#define C_UMAP                ((uint16_t) 0x6084)

/* constant code for '*' character */
#define C_STAR                ((uint16_t) 0xA0DD)

/* constant code for '-' character */
#define C_MINUS               ((uint16_t) 0xA000)

/* constant code for '+' character */
#define C_PLUS                ((uint16_t) 0xA014)

/* constant code for '/' */
#define C_SLATCH              ((uint16_t) 0x00c0)

/* constant code for ° */
#define C_PERCENT_1           ((uint16_t) 0xec00)

/* constant code for small o */
#define C_PERCENT_2           ((uint16_t) 0xb300)

#define C_FULL                ((uint16_t) 0xffdd)

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/

/** @defgroup STM32L476G_DISCOVERY_LCD_Exported_Functions Exported Functions
  * @{
  */
void BSP_LCD_GLASS_Init(void);
void BSP_LCD_GLASS_DeInit(void);
void BSP_LCD_GLASS_BlinkConfig(uint32_t BlinkMode, uint32_t BlinkFrequency);
void BSP_LCD_GLASS_Contrast(uint32_t Contrast);
void BSP_LCD_GLASS_DisplayChar(uint8_t *ch, Point_Typedef Point, DoublePoint_Typedef Column, DigitPosition_Typedef Position);
void BSP_LCD_GLASS_DisplayString(uint8_t *ptr);
void BSP_LCD_GLASS_DisplayStrDeci(uint16_t *ptr);
void BSP_LCD_GLASS_ScrollSentence(uint8_t *ptr, uint16_t nScroll, uint16_t ScrollSpeed);
void BSP_LCD_GLASS_DisplayBar(uint32_t BarId);
void BSP_LCD_GLASS_ClearBar(uint32_t BarId);
void BSP_LCD_GLASS_BarLevelConfig(uint8_t BarLevel);
void BSP_LCD_GLASS_Clear(void);
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

#endif /* __STM32L476G_DISCOVERY_GLASS_LCD_H */


