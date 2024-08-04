/**
  ******************************************************************************
  * @file    hx8347i.h
  * @author  MCD Application Team
  * @brief   This file contains all the functions prototypes for the hx8347i.c
  *          driver.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HX8347I_H
#define HX8347I_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "../Common/lcd.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup Components
  * @{
  */

/** @addtogroup hx8347i
  * @{
  */

/** @defgroup HX8347I_Exported_Types
  * @{
  */

/**
  * @}
  */

/** @defgroup HX8347I_Exported_Constants
  * @{
  */
/**
  * @brief  HX8347I ID
  */
#define  HX8347I_ID    0x0095

/**
  * @brief  HX8347I Size
  */
#define  HX8347I_LCD_PIXEL_WIDTH    ((uint16_t)320)
#define  HX8347I_LCD_PIXEL_HEIGHT   ((uint16_t)240)

/**
  * @brief  HX8347I Registers
  */
#define LCD_REG_0             0x00
#define LCD_REG_1             0x01
#define LCD_REG_2             0x02
#define LCD_REG_3             0x03
#define LCD_REG_4             0x04
#define LCD_REG_5             0x05
#define LCD_REG_6             0x06
#define LCD_REG_7             0x07
#define LCD_REG_8             0x08
#define LCD_REG_9             0x09
#define LCD_REG_10            0x0A
#define LCD_REG_11            0x0B
#define LCD_REG_12            0x0C
#define LCD_REG_13            0x0D
#define LCD_REG_14            0x0E
#define LCD_REG_15            0x0F
#define LCD_REG_16            0x10
#define LCD_REG_17            0x11
#define LCD_REG_18            0x12
#define LCD_REG_19            0x13
#define LCD_REG_20            0x14
#define LCD_REG_21            0x15
#define LCD_REG_22            0x16
#define LCD_REG_23            0x17
#define LCD_REG_24            0x18
#define LCD_REG_25            0x19
#define LCD_REG_26            0x1A
#define LCD_REG_27            0x1B
#define LCD_REG_28            0x1C
#define LCD_REG_29            0x1D
#define LCD_REG_30            0x1E
#define LCD_REG_31            0x1F
#define LCD_REG_32            0x20
#define LCD_REG_33            0x21
#define LCD_REG_34            0x22
#define LCD_REG_35            0x23
#define LCD_REG_36            0x24
#define LCD_REG_37            0x25
#define LCD_REG_38            0x26
#define LCD_REG_39            0x27
#define LCD_REG_40            0x28
#define LCD_REG_41            0x29
#define LCD_REG_42            0x2A
#define LCD_REG_43            0x2B
#define LCD_REG_44            0x2C
#define LCD_REG_45            0x2D
#define LCD_REG_46            0x2E
#define LCD_REG_47            0x2F
#define LCD_REG_48            0x30
#define LCD_REG_49            0x31
#define LCD_REG_50            0x32
#define LCD_REG_51            0x33
#define LCD_REG_52            0x34
#define LCD_REG_53            0x35
#define LCD_REG_54            0x36
#define LCD_REG_55            0x37
#define LCD_REG_56            0x38
#define LCD_REG_57            0x39
#define LCD_REG_58            0x3A
#define LCD_REG_59            0x3B
#define LCD_REG_60            0x3C
#define LCD_REG_61            0x3D
#define LCD_REG_62            0x3E
#define LCD_REG_63            0x3F
#define LCD_REG_64            0x40
#define LCD_REG_65            0x41
#define LCD_REG_66            0x42
#define LCD_REG_67            0x43
#define LCD_REG_68            0x44
#define LCD_REG_69            0x45
#define LCD_REG_70            0x46
#define LCD_REG_71            0x47
#define LCD_REG_72            0x48
#define LCD_REG_73            0x49
#define LCD_REG_74            0x4A
#define LCD_REG_75            0x4B
#define LCD_REG_76            0x4C
#define LCD_REG_77            0x4D
#define LCD_REG_78            0x4E
#define LCD_REG_79            0x4F
#define LCD_REG_80            0x50
#define LCD_REG_81            0x51
#define LCD_REG_82            0x52
#define LCD_REG_83            0x53
#define LCD_REG_84            0x54
#define LCD_REG_85            0x55
#define LCD_REG_86            0x56
#define LCD_REG_87            0x57
#define LCD_REG_88            0x58
#define LCD_REG_89            0x59
#define LCD_REG_90            0x5A
#define LCD_REG_91            0x5B
#define LCD_REG_92            0x5C
#define LCD_REG_93            0x5D
#define LCD_REG_94            0x5E
#define LCD_REG_95            0x5F
#define LCD_REG_96            0x60
#define LCD_REG_97            0x61
#define LCD_REG_98            0x62
#define LCD_REG_129           0x81
#define LCD_REG_130           0x82
#define LCD_REG_132           0x84
#define LCD_REG_133           0x85
#define LCD_REG_228           0xE4
#define LCD_REG_229           0xE5
#define LCD_REG_230           0xE6
#define LCD_REG_231           0xE7
#define LCD_REG_232           0xE8
#define LCD_REG_233           0xE9
#define LCD_REG_234           0xEA
#define LCD_REG_235           0xEB
#define LCD_REG_236           0xEC
#define LCD_REG_237           0xED
#define LCD_REG_243           0xF3
#define LCD_REG_244           0xF4
#define LCD_REG_255           0xFF

/**
  * @}
  */

/** @defgroup HX8347I_Exported_Functions
  * @{
  */
void     hx8347i_Init(void);
uint16_t hx8347i_ReadID(void);
void     hx8347i_WriteReg(uint8_t LCDReg, uint16_t LCDRegValue);
uint16_t hx8347i_ReadReg(uint8_t LCDReg);

void     hx8347i_DisplayOn(void);
void     hx8347i_DisplayOff(void);
void     hx8347i_SetCursor(uint16_t Xpos, uint16_t Ypos);
void     hx8347i_WritePixel(uint16_t Xpos, uint16_t Ypos, uint16_t RGBCode);
uint16_t hx8347i_ReadPixel(uint16_t Xpos, uint16_t Ypos);

void     hx8347i_DrawHLine(uint16_t RGBCode, uint16_t Xpos, uint16_t Ypos, uint16_t Length);
void     hx8347i_DrawVLine(uint16_t RGBCode, uint16_t Xpos, uint16_t Ypos, uint16_t Length);
void     hx8347i_DrawBitmap(uint16_t Xpos, uint16_t Ypos, uint8_t *pbmp);

void     hx8347i_SetDisplayWindow(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height);


uint16_t hx8347i_GetLcdPixelWidth(void);
uint16_t hx8347i_GetLcdPixelHeight(void);

/* LCD driver structure */
extern LCD_DrvTypeDef   hx8347i_drv;

/* LCD IO functions */
void     LCD_IO_Init(void);
void     LCD_IO_WriteMultipleData(uint8_t *pData, uint32_t Size);
void     LCD_IO_WriteReg(uint8_t Reg);
uint16_t LCD_IO_ReadData(uint16_t Reg);
void     LCD_Delay (uint32_t delay);
/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* HX8347I_H */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
