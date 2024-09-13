/**
  ******************************************************************************
  * @file    mx25r6435f.h
  * @author  MCD Application Team
  * @brief   This file contains all the description of the MX25R6435F QSPI memory.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
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
#ifndef __MX25R6435F_H
#define __MX25R6435F_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/

/** @addtogroup BSP
  * @{
  */ 

/** @addtogroup Components
  * @{
  */ 
  
/** @addtogroup mx25r6435f
  * @{
  */

/** @defgroup MX25R6435F_Exported_Types
  * @{
  */
   
/**
  * @}
  */ 

/** @defgroup MX25R6435F_Exported_Constants
  * @{
  */
   
/** 
  * @brief  MX25R6435F Configuration  
  */  
#define MX25R6435F_FLASH_SIZE                0x800000  /* 64 MBits => 8MBytes */
#define MX25R6435F_BLOCK_SIZE                0x10000   /* 128 blocks of 64KBytes */
#define MX25R6435F_SUBBLOCK_SIZE             0x8000    /* 256 blocks of 32KBytes */
#define MX25R6435F_SECTOR_SIZE               0x1000    /* 2048 sectors of 4kBytes */
#define MX25R6435F_PAGE_SIZE                 0x100     /* 32768 pages of 256 bytes */

#define MX25R6435F_DUMMY_CYCLES_READ         8
#define MX25R6435F_DUMMY_CYCLES_READ_DUAL    4
#define MX25R6435F_DUMMY_CYCLES_READ_QUAD    4
#define MX25R6435F_DUMMY_CYCLES_2READ        2
#define MX25R6435F_DUMMY_CYCLES_4READ        4

#define MX25R6435F_ALT_BYTES_PE_MODE         0xA5
#define MX25R6435F_ALT_BYTES_NO_PE_MODE      0xAA

#define MX25R6435F_CHIP_ERASE_MAX_TIME       240000
#define MX25R6435F_BLOCK_ERASE_MAX_TIME      3500
#define MX25R6435F_SUBBLOCK_ERASE_MAX_TIME   3000
#define MX25R6435F_SECTOR_ERASE_MAX_TIME     240

/** 
  * @brief  MX25R6435F Commands  
  */  
/* Read Operations */
#define READ_CMD                             0x03
#define FAST_READ_CMD                        0x0B
#define DUAL_OUT_READ_CMD                    0x3B
#define DUAL_INOUT_READ_CMD                  0xBB
#define QUAD_OUT_READ_CMD                    0x6B
#define QUAD_INOUT_READ_CMD                  0xEB

/* Program Operations */
#define PAGE_PROG_CMD                        0x02
#define QUAD_PAGE_PROG_CMD                   0x38

/* Erase Operations */
#define SECTOR_ERASE_CMD                     0x20
#define SUBBLOCK_ERASE_CMD                   0x52
#define BLOCK_ERASE_CMD                      0xD8
#define CHIP_ERASE_CMD                       0x60
#define CHIP_ERASE_CMD_2                     0xC7

#define PROG_ERASE_RESUME_CMD                0x7A
#define PROG_ERASE_RESUME_CMD_2              0x30
#define PROG_ERASE_SUSPEND_CMD               0x75
#define PROG_ERASE_SUSPEND_CMD_2             0xB0

/* Identification Operations */
#define READ_ID_CMD                          0x9F
#define READ_ELECTRONIC_ID_CMD               0xAB
#define READ_ELEC_MANUFACTURER_DEVICE_ID_CMD 0x90
#define READ_SERIAL_FLASH_DISCO_PARAM_CMD    0x5A

/* Write Operations */
#define WRITE_ENABLE_CMD                     0x06
#define WRITE_DISABLE_CMD                    0x04

/* Register Operations */
#define READ_STATUS_REG_CMD                  0x05
#define READ_CFG_REG_CMD                     0x15
#define WRITE_STATUS_CFG_REG_CMD             0x01

#define READ_SEC_REG_CMD                     0x2B
#define WRITE_SEC_REG_CMD                    0x2F

/* Power Down Operations */
#define DEEP_POWER_DOWN_CMD                  0xB9

/* Burst Operations */
#define SET_BURST_LENGTH_CMD                 0xC0

/* One-Time Programmable Operations */
#define ENTER_SECURED_OTP_CMD                0xB1
#define EXIT_SECURED_OTP_CMD                 0xC1

/* No Operation */
#define NO_OPERATION_CMD                     0x00

/* Reset Operations */
#define RESET_ENABLE_CMD                     0x66
#define RESET_MEMORY_CMD                     0x99
#define RELEASE_READ_ENHANCED_CMD            0xFF

/** 
  * @brief  MX25R6435F Registers  
  */ 
/* Status Register */
#define MX25R6435F_SR_WIP                    ((uint8_t)0x01)    /*!< Write in progress */
#define MX25R6435F_SR_WEL                    ((uint8_t)0x02)    /*!< Write enable latch */
#define MX25R6435F_SR_BP                     ((uint8_t)0x3C)    /*!< Block protect */
#define MX25R6435F_SR_QE                     ((uint8_t)0x40)    /*!< Quad enable */
#define MX25R6435F_SR_SRWD                   ((uint8_t)0x80)    /*!< Status register write disable */

/* Configuration Register 1 */
#define MX25R6435F_CR1_TB                    ((uint8_t)0x08)    /*!< Top / bottom */

/* Configuration Register 2 */
#define MX25R6435F_CR2_LH_SWITCH             ((uint8_t)0x02)    /*!< Low power / high performance switch */

/* Security Register */
#define MX25R6435F_SECR_SOI                  ((uint8_t)0x01)    /*!< Secured OTP indicator */
#define MX25R6435F_SECR_LDSO                 ((uint8_t)0x02)    /*!< Lock-down secured OTP */
#define MX25R6435F_SECR_PSB                  ((uint8_t)0x04)    /*!< Program suspend bit */
#define MX25R6435F_SECR_ESB                  ((uint8_t)0x08)    /*!< Erase suspend bit */
#define MX25R6435F_SECR_P_FAIL               ((uint8_t)0x20)    /*!< Program fail flag */
#define MX25R6435F_SECR_E_FAIL               ((uint8_t)0x40)    /*!< Erase fail flag */

/**
  * @}
  */
  
/** @defgroup MX25R6435F_Exported_Functions
  * @{
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

/**
  * @}
  */
  
#ifdef __cplusplus
}
#endif

#endif /* __MX25R6435F_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
