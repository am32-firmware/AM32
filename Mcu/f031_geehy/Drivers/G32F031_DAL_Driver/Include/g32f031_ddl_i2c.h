/**
  *
  * @file    g32f031_ddl_i2c.h
  * @brief   Header file of I2C DDL module.
  *
  * @attention
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *
  * 1. Redistributions of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of the copyright holder nor the names of its contributors
  *    may be used to endorse or promote products derived from this software without
  *    specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
  * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
  * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
  * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
  * OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  * The original code has been modified by Geehy Semiconductor.
  *
 *  Copyright (C) 2026 Geehy Semiconductor
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file in
  * the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  */

  /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef G32F031_DDL_I2C_H
#define G32F031_DDL_I2C_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "g32f0xx.h"

/** @addtogroup G32F031_DDL_Driver
  * @{
  */

#if defined(I2C)

/** @defgroup I2C_DDL I2C
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/** @defgroup I2C_DDL_Private_Constants I2C Private Constants
  * @{
  */

/* Defines used to perform compute and check in the macros */
#define DDL_I2C_MAX_SPEED_STANDARD           100000U
#define DDL_I2C_MAX_SPEED_FAST               400000U
#define DDL_I2C_MAX_SPEED_ULTRAFAST          1000000U
/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#if defined(USE_FULL_DDL_DRIVER)
/** @defgroup I2C_DDL_ES_INIT I2C Exported Init structure
  * @{
  */
typedef struct
{
  uint32_t PeripheralMode;      /*!< Specifies the peripheral mode.
                                     This parameter can be a value of @ref I2C_DDL_EC_PERIPHERAL_MODE

                                     This feature can be modified afterwards using unitary function @ref DDL_I2C_SetMode(). */

  uint32_t ClockSpeed;          /*!< Specifies the clock frequency.
                                     This parameter must be set to a value lower than 400000Hz (in Hz)

                                     This feature can be modified afterwards using unitary function @ref DDL_I2C_SetClockPeriod()
                                     or @ref DDL_I2C_SetDutyCycle() or @ref DDL_I2C_SetClockSpeedMode() or @ref DDL_I2C_ConfigSpeed(). */

  uint32_t DutyCycle;           /*!< Specifies the I2C fast mode duty cycle.
                                     This parameter can be a value of @ref I2C_DDL_EC_DUTYCYCLE

                                     This feature can be modified afterwards using unitary function @ref DDL_I2C_SetDutyCycle(). */

  uint32_t AnalogFilter;        /*!< Enables or disables analog noise filter.
                                     This parameter can be a value of @ref I2C_DDL_EC_ANALOGFILTER_SELECTION

                                     This feature can be modified afterwards using unitary functions @ref DDL_I2C_EnableAnalogFilter() or DDL_I2C_DisableAnalogFilter(). */

  uint32_t DigitalFilter;       /*!< Configures the digital noise filter.
                                     This parameter can be a number between Min_Data = 0x00 and Max_Data = 0x0F

                                     This feature can be modified afterwards using unitary function @ref DDL_I2C_SetDigitalFilter(). */

  uint32_t OwnAddress1;         /*!< Specifies the device own address 1.
                                     This parameter must be a value between Min_Data = 0x00 and Max_Data = 0x3FF

                                     This feature can be modified afterwards using unitary function @ref DDL_I2C_SetOwnAddress1(). */

  uint32_t TypeAcknowledge;     /*!< Specifies the ACKnowledge or Non ACKnowledge condition after the address receive match code or next received byte.
                                     This parameter can be a value of @ref I2C_DDL_EC_I2C_ACKNOWLEDGE

                                     This feature can be modified afterwards using unitary function @ref DDL_I2C_AcknowledgeNextData(). */

  uint32_t OwnAddrSize;         /*!< Specifies the device own address 1 size (7-bit or 10-bit).
                                     This parameter can be a value of @ref I2C_DDL_EC_OWNADDRESS1

                                     This feature can be modified afterwards using unitary function @ref DDL_I2C_SetOwnAddress1(). */
} DDL_I2C_InitTypeDef;
/**
  * @}
  */
#endif /*USE_FULL_DDL_DRIVER*/

/* Exported constants --------------------------------------------------------*/
/** @defgroup I2C_DDL_Exported_Constants I2C Exported Constants
  * @{
  */

#define DDL_I2C_SMBUS_TYPE_MASTER                (I2C_CR1_SMBTCFG)
#define DDL_I2C_SMBUS_TYPE_SLAVE                 (0x0UL)

/** @defgroup I2C_DDL_EC_OWNADDRESS1 Own Address 1 Length
  * @{
  */
#define DDL_I2C_OWNADDRESS1_7BIT                 (0x00004000UL)                                 /*!< Own address 1 is a 7-bit address.   */
#define DDL_I2C_OWNADDRESS1_10BIT                (uint32_t)(I2C_SADDR1_ADDRLEN | 0x00004000U)   /*!< Own address 1 is a 10-bit address.  */
/**
  * @}
  */

/** @defgroup I2C_DDL_EC_I2C_ACKNOWLEDGE Acknowledge Generation
  * @{
  */
#define DDL_I2C_ACK                               I2C_CR1_ACKEN      /*!< ACK is sent after current received byte. */
#define DDL_I2C_NACK                              0x00000000UL         /*!< NACK is sent after current received byte.*/
/**
  * @}
  */

/** @defgroup I2C_DDL_EC_ANALOGFILTER_SELECTION  Analog Filter Selection
  * @{
  */
#define DDL_I2C_ANALOGFILTER_ENABLE              (0x00000000UL)               /*!< Analog filter is enabled. */
#define DDL_I2C_ANALOGFILTER_DISABLE             (I2C_FILTER_ANFDISEN)          /*!< Analog filter is disabled.*/
/**
  * @}
  */

/** @defgroup I2C_DDL_EC_DUTYCYCLE Fast Mode Duty Cycle
  * @{
  */
#define DDL_I2C_DUTYCYCLE_2                      (0x00000000UL)               /*!< I2C fast mode Tlow/Thigh = 2        */
#define DDL_I2C_DUTYCYCLE_16_9                   (I2C_CLKCR_FDUTYCFG)       /*!< I2C fast mode Tlow/Thigh = 16/9     */
/**
  * @}
  */

/** @defgroup I2C_DDL_EC_CLOCK_SPEED_MODE Master Clock Speed Mode
  * @{
  */
#define DDL_I2C_CLOCK_SPEED_STANDARD_MODE        (0x00000000UL)               /*!< Master clock speed range is standard mode */
#define DDL_I2C_CLOCK_SPEED_FAST_MODE            (I2C_CLKCR_SPEEDCFG)       /*!< Master clock speed range is fast mode     */
/**
  * @}
  */

/** @defgroup I2C_DDL_EC_DIRECTION Read Write Direction
  * @{
  */
#define DDL_I2C_DIRECTION_WRITE                  (I2C_SR2_TRFLG)             /*!< Bus is in write transfer */
#define DDL_I2C_DIRECTION_READ                   (0x00000000UL)               /*!< Bus is in read transfer  */
/**
  * @}
  */

/** @defgroup I2C_DDL_EC_PERIPHERAL_MODE Peripheral Mode
  * @{
  */
#define DDL_I2C_MODE_I2C                     (0x00000000UL)                                                     /*!< I2C Master or Slave mode                                    */
#define DDL_I2C_MODE_SMBUS_HOST              (uint32_t)(I2C_CR1_SMBEN | I2C_CR1_SMBTCFG | I2C_CR1_ARPEN)  /*!< SMBus Host address acknowledge                              */
#define DDL_I2C_MODE_SMBUS_DEVICE            I2C_CR1_SMBEN                                                    /*!< SMBus Device default mode (Default address not acknowledge) */
#define DDL_I2C_MODE_SMBUS_DEVICE_ARP        (uint32_t)(I2C_CR1_SMBEN | I2C_CR1_ARPEN)                      /*!< SMBus Device Default address acknowledge                    */
/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup I2C_DDL_Exported_Macros I2C Exported Macros
  * @{
  */

/** @defgroup I2C_DDL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in I2C register
  * @param  __INSTANCE__ I2C Instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define DDL_I2C_WriteReg(__INSTANCE__, __REG__, __VALUE__) WRITE_REG(__INSTANCE__->__REG__, (__VALUE__))

/**
  * @brief  Read a value in I2C register
  * @param  __INSTANCE__ I2C Instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define DDL_I2C_ReadReg(__INSTANCE__, __REG__) READ_REG(__INSTANCE__->__REG__)
/**
  * @}
  */

/** @defgroup I2C_DDL_EM_Exported_Macros_Helper Exported_Macros_Helper
  * @{
  */

/**
  * @brief  Convert Peripheral Clock Frequency in Mhz.
  * @param  __PCLK__ This parameter must be a value of peripheral clock (in Hz).
  * @retval Value of peripheral clock (in Mhz)
  */
#define __DDL_I2C_FREQ_HZ_TO_MHZ(__PCLK__)                               (uint32_t)((__PCLK__)/1000000U)

/**
  * @brief  Convert Peripheral Clock Frequency in Hz.
  * @param  __PCLK__ This parameter must be a value of peripheral clock (in Mhz).
  * @retval Value of peripheral clock (in Hz)
  */
#define __DDL_I2C_FREQ_MHZ_TO_HZ(__PCLK__)                               (uint32_t)((__PCLK__)*1000000U)

/**
  * @brief  Compute I2C Clock rising time.
  * @param  __FREQRANGE__ This parameter must be a value of peripheral clock (in Mhz).
  * @param  __SPEED__ This parameter must be a value lower than 400kHz (in Hz).
  * @retval Value between Min_Data=0x02 and Max_Data=0x3F
  */
#define __DDL_I2C_RISE_TIME(__FREQRANGE__, __SPEED__)                    (uint32_t)(((__SPEED__) <= DDL_I2C_MAX_SPEED_STANDARD) ? ((__FREQRANGE__) + 1U) : ((((__FREQRANGE__) * 300U) / 1000U) + 1U))

/**
  * @brief  Compute Speed clock range to a Clock Control Register (I2C_CLKCR_CLKCFG) value.
  * @param  __PCLK__ This parameter must be a value of peripheral clock (in Hz).
  * @param  __SPEED__ This parameter must be a value lower than 400kHz (in Hz).
  * @param  __DUTYCYCLE__ This parameter can be one of the following values:
  *         @arg @ref DDL_I2C_DUTYCYCLE_2
  *         @arg @ref DDL_I2C_DUTYCYCLE_16_9
  * @retval Value between Min_Data=0x004 and Max_Data=0xFFF, except in FAST DUTY mode where Min_Data=0x001.
  */
#define __DDL_I2C_SPEED_TO_CCR(__PCLK__, __SPEED__, __DUTYCYCLE__)       (uint32_t)(((__SPEED__) <= DDL_I2C_MAX_SPEED_STANDARD)? \
                                                                                  (__DDL_I2C_SPEED_STANDARD_TO_CCR((__PCLK__), (__SPEED__))) : \
                                                                                  (__DDL_I2C_SPEED_FAST_TO_CCR((__PCLK__), (__SPEED__), (__DUTYCYCLE__))))

/**
  * @brief  Compute Speed Standard clock range to a Clock Control Register (I2C_CLKCR_CLKCFG) value.
  * @param  __PCLK__ This parameter must be a value of peripheral clock (in Hz).
  * @param  __SPEED__ This parameter must be a value lower than 100kHz (in Hz).
  * @retval Value between Min_Data=0x004 and Max_Data=0xFFF.
  */
#define __DDL_I2C_SPEED_STANDARD_TO_CCR(__PCLK__, __SPEED__)             (uint32_t)(((((__PCLK__)/((__SPEED__) << 1U)) & I2C_CLKCR_CLKCFG) < 4U)? 4U:((__PCLK__) / ((__SPEED__) << 1U)))

/**
  * @brief  Compute Speed Fast clock range to a Clock Control Register (I2C_CLKCR_CLKCFG) value.
  * @param  __PCLK__ This parameter must be a value of peripheral clock (in Hz).
  * @param  __SPEED__ This parameter must be a value between Min_Data=100Khz and Max_Data=1Mhz (in Hz).
  * @param  __DUTYCYCLE__ This parameter can be one of the following values:
  *         @arg @ref DDL_I2C_DUTYCYCLE_2
  *         @arg @ref DDL_I2C_DUTYCYCLE_16_9
  * @retval Value between Min_Data=0x001 and Max_Data=0xFFF
  */
#define __DDL_I2C_SPEED_FAST_TO_CCR(__PCLK__, __SPEED__, __DUTYCYCLE__)  (uint32_t)(((__DUTYCYCLE__) == DDL_I2C_DUTYCYCLE_2)? \
                                                                            (((((__PCLK__) / ((__SPEED__) * 3U)) & I2C_CLKCR_CLKCFG) == 0U)? 1U:((__PCLK__) / ((__SPEED__) * 3U))) : \
                                                                            (((((__PCLK__) / ((__SPEED__) * 25U)) & I2C_CLKCR_CLKCFG) == 0U)? 1U:((__PCLK__) / ((__SPEED__) * 25U))))

/**
  * @brief  Get the Least significant bits of a 10-Bits address.
  * @param  __ADDRESS__ This parameter must be a value of a 10-Bits slave address.
  * @retval Value between Min_Data=0x00 and Max_Data=0xFF
  */
#define __DDL_I2C_10BIT_ADDRESS(__ADDRESS__)                             ((uint8_t)((uint16_t)((__ADDRESS__) & (uint16_t)(0x00FF))))

/**
  * @brief  Convert a 10-Bits address to a 10-Bits header with Write direction.
  * @param  __ADDRESS__ This parameter must be a value of a 10-Bits slave address.
  * @retval Value between Min_Data=0xF0 and Max_Data=0xF6
  */
#define __DDL_I2C_10BIT_HEADER_WRITE(__ADDRESS__)                        ((uint8_t)((uint16_t)((uint16_t)(((uint16_t)((__ADDRESS__) & (uint16_t)(0x0300))) >> 7) | (uint16_t)(0xF0))))

/**
  * @brief  Convert a 10-Bits address to a 10-Bits header with Read direction.
  * @param  __ADDRESS__ This parameter must be a value of a 10-Bits slave address.
  * @retval Value between Min_Data=0xF1 and Max_Data=0xF7
  */
#define __DDL_I2C_10BIT_HEADER_READ(__ADDRESS__)                         ((uint8_t)((uint16_t)((uint16_t)(((uint16_t)((__ADDRESS__) & (uint16_t)(0x0300))) >> 7) | (uint16_t)(0xF1))))

/**
  * @}
  */

/**
  * @}
  */


/* Exported functions --------------------------------------------------------*/
/** @defgroup I2C_DDL_Exported_Functions I2C Exported Functions
  * @{
  */

/** @defgroup I2C_DDL_EF_Configuration Configuration
  * @{
  */

/**
  * @brief  Enable I2C
  * @param  I2Cx i2c instance
  * @retval None
  */
__STATIC_INLINE void DDL_I2C_Enable(I2C_TypeDef *I2Cx)
{
  SET_BIT(I2Cx->CR1, I2C_CR1_I2CEN);
}

/**
  * @brief  Disable I2C
  * @param  I2Cx i2c instance
  * @retval None
  */
__STATIC_INLINE void DDL_I2C_Disable(I2C_TypeDef *I2Cx)
{
  CLEAR_BIT(I2Cx->CR1, I2C_CR1_I2CEN);
}

/**
  * @brief  Get I2C enable status
  * @param  I2Cx i2c instance
  * @retval Status:
  *         @arg @ref 0: Disable
  *         @arg @ref 1: Enable
  */
__STATIC_INLINE uint32_t DDL_I2C_IsEnabled(I2C_TypeDef *I2Cx)
{
  return (READ_BIT(I2Cx->CR1, I2C_CR1_I2CEN) == (I2C_CR1_I2CEN));
}

/**
  * @brief  Enable SMBUS
  * @param  I2Cx i2c instance
  * @retval None
  */
__STATIC_INLINE void DDL_I2C_EnableSMBUS(I2C_TypeDef *I2Cx)
{
  SET_BIT(I2Cx->CR1, I2C_CR1_SMBEN);
}

/**
  * @brief  Disable SMBUS
  * @param  I2Cx i2c instance
  * @retval None
  */
__STATIC_INLINE void DDL_I2C_DisableSMBUS(I2C_TypeDef *I2Cx)
{
  CLEAR_BIT(I2Cx->CR1, I2C_CR1_SMBEN);
}

/**
  * @brief  Get SMBUS enable status
  * @param  I2Cx i2c instance
  * @retval Status:
  *         @arg @ref 0: Disable
  *         @arg @ref 1: Enable
  */
__STATIC_INLINE uint32_t DDL_I2C_IsEnabledSMBUS(I2C_TypeDef *I2Cx)
{
  return (uint32_t)(READ_BIT(I2Cx->CR1, I2C_CR1_SMBEN) == (I2C_CR1_SMBEN));
}

/**
  * @brief  Set SMBUS type
  * @param  I2Cx i2c instance
  * @param  type type
  *         @arg @ref DDL_I2C_SMBUS_TYPE_MASTER
  *         @arg @ref DDL_I2C_SMBUS_TYPE_SLAVE
  * @retval None
  */
__STATIC_INLINE void DDL_I2C_SetSMBUSType(I2C_TypeDef *I2Cx, uint32_t type)
{
  MODIFY_REG(I2Cx->CR1, I2C_CR1_SMBTCFG, type);
}

/**
  * @brief  Set SMBUS type
  * @param  I2Cx i2c instance
  * @retval type
  *         @arg @ref DDL_I2C_SMBUS_TYPE_MASTER
  *         @arg @ref DDL_I2C_SMBUS_TYPE_SLAVE
  */
__STATIC_INLINE uint32_t DDL_I2C_GetSMBUSType(I2C_TypeDef *I2Cx)
{
  return (uint32_t)READ_BIT(I2Cx->CR1, I2C_CR1_SMBTCFG);
}

/**
  * @brief  Enable ARP
  * @param  I2Cx i2c instance
  * @retval None
  */
__STATIC_INLINE void DDL_I2C_EnableARP(I2C_TypeDef *I2Cx)
{
  SET_BIT(I2Cx->CR1, I2C_CR1_ARPEN);
}

/**
  * @brief  Disable ARP
  * @param  I2Cx i2c instance
  * @retval None
  */
__STATIC_INLINE void DDL_I2C_DisableARP(I2C_TypeDef *I2Cx)
{
  CLEAR_BIT(I2Cx->CR1, I2C_CR1_ARPEN);
}

/**
  * @brief  Get ARP enable status
  * @param  I2Cx i2c instance
  * @retval Status:
  *         @arg @ref 0: Disable
  *         @arg @ref 1: Enable
  */
__STATIC_INLINE uint32_t DDL_I2C_IsEnabledARP(I2C_TypeDef *I2Cx)
{
  return (uint32_t)(READ_BIT(I2Cx->CR1, I2C_CR1_ARPEN) == (I2C_CR1_ARPEN));
}

/**
  * @brief  Enable SMBus Packet Error Calculation (PEC).
  * @note   Macro @ref IS_SMBUS_ALL_INSTANCE(I2Cx) can be used to check whether or not
  *         SMBus feature is supported by the I2Cx Instance.
  * @param  I2Cx I2C Instance.
  * @retval None
  */
__STATIC_INLINE void DDL_I2C_EnableSMBusPEC(I2C_TypeDef *I2Cx)
{
  SET_BIT(I2Cx->CR1, I2C_CR1_PECEN);
}

/**
  * @brief  Disable SMBus Packet Error Calculation (PEC).
  * @note   Macro @ref IS_SMBUS_ALL_INSTANCE(I2Cx) can be used to check whether or not
  *         SMBus feature is supported by the I2Cx Instance.
  * @param  I2Cx I2C Instance.
  * @retval None
  */
__STATIC_INLINE void DDL_I2C_DisableSMBusPEC(I2C_TypeDef *I2Cx)
{
  CLEAR_BIT(I2Cx->CR1, I2C_CR1_PECEN);
}

/**
  * @brief  Check if SMBus Packet Error Calculation (PEC) is enabled or disabled.
  * @note   Macro @ref IS_SMBUS_ALL_INSTANCE(I2Cx) can be used to check whether or not
  *         SMBus feature is supported by the I2Cx Instance.
  * @param  I2Cx I2C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_I2C_IsEnabledSMBusPEC(I2C_TypeDef *I2Cx)
{
  return (READ_BIT(I2Cx->CR1, I2C_CR1_PECEN) == (I2C_CR1_PECEN));
}

/**
  * @brief  Enable General Call.
  * @note   When enabled the Address 0x00 is ACKed.
  * @param  I2Cx I2C Instance.
  * @retval None
  */
__STATIC_INLINE void DDL_I2C_EnableGeneralCall(I2C_TypeDef *I2Cx)
{
  SET_BIT(I2Cx->CR1, I2C_CR1_SRBEN);
}

/**
  * @brief  Disable General Call.
  * @note   When disabled the Address 0x00 is NACKed.
  * @param  I2Cx I2C Instance.
  * @retval None
  */
__STATIC_INLINE void DDL_I2C_DisableGeneralCall(I2C_TypeDef *I2Cx)
{
  CLEAR_BIT(I2Cx->CR1, I2C_CR1_SRBEN);
}

/**
  * @brief  Check if General Call is enabled or disabled.
  * @param  I2Cx I2C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_I2C_IsEnabledGeneralCall(I2C_TypeDef *I2Cx)
{
  return (READ_BIT(I2Cx->CR1, I2C_CR1_SRBEN) == (I2C_CR1_SRBEN));
}

/**
  * @brief  Slave Mode Clock Stretching Enable
  * @param  I2Cx i2c instance
  * @retval None
  */
__STATIC_INLINE void DDL_I2C_EnableClockStretch(I2C_TypeDef *I2Cx)
{
  CLEAR_BIT(I2Cx->CR1, I2C_CR1_CLKSTRETCHD);
}

/**
  * @brief  Slave Mode Clock Stretching Disable
  * @param  I2Cx i2c instance
  * @retval None
  */
__STATIC_INLINE void DDL_I2C_DisableClockStretch(I2C_TypeDef *I2Cx)
{
  SET_BIT(I2Cx->CR1, I2C_CR1_CLKSTRETCHD);
}

/**
  * @brief  Get Slave Mode Clock Stretching Disable status
  * @param  I2Cx i2c instance
  * @retval Status:
  *         @arg @ref 1: Disable
  *         @arg @ref 0: Enable
  */
__STATIC_INLINE uint32_t DDL_I2C_IsEnabledClockStretch(I2C_TypeDef *I2Cx)
{
  return (uint32_t)(READ_BIT(I2Cx->CR1, I2C_CR1_CLKSTRETCHD) == 0U);
}

/**
  * @brief  Generate a START or RESTART condition
  * @note   The START bit can be set even if bus is BUSY or I2C is in slave mode.
  *         This action has no effect when RELOAD is set.
  * @param  I2Cx I2C Instance.
  * @retval None
  */
__STATIC_INLINE void DDL_I2C_GenerateStartCondition(I2C_TypeDef *I2Cx)
{
  SET_BIT(I2Cx->CR1, I2C_CR1_START);
}

/**
  * @brief  Generate a STOP condition after the current byte transfer (master mode).
  * @param  I2Cx I2C Instance.
  * @retval None
  */
__STATIC_INLINE void DDL_I2C_GenerateStopCondition(I2C_TypeDef *I2Cx)
{
  SET_BIT(I2Cx->CR1, I2C_CR1_STOP);
}

/**
  * @brief  Prepare the generation of a ACKnowledge or Non ACKnowledge condition after the address receive match code or next received byte.
  * @note   Usage in Slave or Master mode.
  * @param  I2Cx I2C Instance.
  * @param  TypeAcknowledge This parameter can be one of the following values:
  *         @arg @ref DDL_I2C_ACK
  *         @arg @ref DDL_I2C_NACK
  * @retval None
  */
__STATIC_INLINE void DDL_I2C_AcknowledgeNextData(I2C_TypeDef *I2Cx, uint32_t TypeAcknowledge)
{
  MODIFY_REG(I2Cx->CR1, I2C_CR1_ACKEN, TypeAcknowledge);
}

/**
  * @brief  Enable bit POS (master/host mode).
  * @note   In that case, the ACK bit controls the (N)ACK of the next byte received or the PEC bit indicates that the next byte in shift register is a PEC.
  * @param  I2Cx I2C Instance.
  * @retval None
  */
__STATIC_INLINE void DDL_I2C_EnableBitPOS(I2C_TypeDef *I2Cx)
{
  SET_BIT(I2Cx->CR1, I2C_CR1_ACKPOS);
}

/**
  * @brief  Disable bit POS (master/host mode).
  * @note   In that case, the ACK bit controls the (N)ACK of the current byte received or the PEC bit indicates that the current byte in shift register is a PEC.
  * @param  I2Cx I2C Instance.
  * @retval None
  */
__STATIC_INLINE void DDL_I2C_DisableBitPOS(I2C_TypeDef *I2Cx)
{
  CLEAR_BIT(I2Cx->CR1, I2C_CR1_ACKPOS);
}

/**
  * @brief  Check if bit POS  is enabled or disabled.
  * @param  I2Cx I2C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_I2C_IsEnabledBitPOS(I2C_TypeDef *I2Cx)
{
  return (READ_BIT(I2Cx->CR1, I2C_CR1_ACKPOS) == (I2C_CR1_ACKPOS));
}

/**
  * @brief  Enable transfer or internal comparison of the SMBus Packet Error byte (transmission or reception mode).
  * @note   Macro @ref IS_SMBUS_ALL_INSTANCE(I2Cx) can be used to check whether or not
  *         SMBus feature is supported by the I2Cx Instance.
  * @note   This feature is cleared by hardware when the PEC byte is transferred or compared,
  *         or by a START or STOP condition, it is also cleared by software.
  * @param  I2Cx I2C Instance.
  * @retval None
  */
__STATIC_INLINE void DDL_I2C_EnableSMBusPECCompare(I2C_TypeDef *I2Cx)
{
  SET_BIT(I2Cx->CR1, I2C_CR1_PEC);
}

/**
  * @brief  Disable transfer or internal comparison of the SMBus Packet Error byte (transmission or reception mode).
  * @note   Macro @ref IS_SMBUS_ALL_INSTANCE(I2Cx) can be used to check whether or not
  *         SMBus feature is supported by the I2Cx Instance.
  * @param  I2Cx I2C Instance.
  * @retval None
  */
__STATIC_INLINE void DDL_I2C_DisableSMBusPECCompare(I2C_TypeDef *I2Cx)
{
  CLEAR_BIT(I2Cx->CR1, I2C_CR1_PEC);
}

/**
  * @brief  Check if the SMBus Packet Error byte transfer or internal comparison is requested or not.
  * @note   Macro @ref IS_SMBUS_ALL_INSTANCE(I2Cx) can be used to check whether or not
  *         SMBus feature is supported by the I2Cx Instance.
  * @param  I2Cx I2C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_I2C_IsEnabledSMBusPECCompare(I2C_TypeDef *I2Cx)
{
  return (READ_BIT(I2Cx->CR1, I2C_CR1_PEC) == (I2C_CR1_PEC));
}

/**
  * @brief  Enable SMBus alert (Host or Device mode)
  * @note   Macro @ref IS_SMBUS_ALL_INSTANCE(I2Cx) can be used to check whether or not
  *         SMBus feature is supported by the I2Cx Instance.
  * @note   SMBus Device mode:
  *         - SMBus Alert pin is drived low and
  *           Alert Response Address Header acknowledge is enabled.
  *         SMBus Host mode:
  *         - SMBus Alert pin management is supported.
  * @param  I2Cx I2C Instance.
  * @retval None
  */
__STATIC_INLINE void DDL_I2C_EnableSMBusAlert(I2C_TypeDef *I2Cx)
{
  SET_BIT(I2Cx->CR1, I2C_CR1_ALERTEN);
}

/**
  * @brief  Disable SMBus alert (Host or Device mode)
  * @note   Macro @ref IS_SMBUS_ALL_INSTANCE(I2Cx) can be used to check whether or not
  *         SMBus feature is supported by the I2Cx Instance.
  * @note   SMBus Device mode:
  *         - SMBus Alert pin is not drived (can be used as a standard GPIO) and
  *           Alert Response Address Header acknowledge is disabled.
  *         SMBus Host mode:
  *         - SMBus Alert pin management is not supported.
  * @param  I2Cx I2C Instance.
  * @retval None
  */
__STATIC_INLINE void DDL_I2C_DisableSMBusAlert(I2C_TypeDef *I2Cx)
{
  CLEAR_BIT(I2Cx->CR1, I2C_CR1_ALERTEN);
}

/**
  * @brief  Check if SMBus alert (Host or Device mode) is enabled or disabled.
  * @note   Macro @ref IS_SMBUS_ALL_INSTANCE(I2Cx) can be used to check whether or not
  *         SMBus feature is supported by the I2Cx Instance.
  * @param  I2Cx I2C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_I2C_IsEnabledSMBusAlert(I2C_TypeDef *I2Cx)
{
  return (READ_BIT(I2Cx->CR1, I2C_CR1_ALERTEN) == (I2C_CR1_ALERTEN));
}

/**
  * @brief  Enable Reset of I2C peripheral.
  * @param  I2Cx I2C Instance.
  * @retval None
  */
__STATIC_INLINE void DDL_I2C_EnableReset(I2C_TypeDef *I2Cx)
{
  SET_BIT(I2Cx->CR1, I2C_CR1_SWRSTS);
}

/**
  * @brief  Disable Reset of I2C peripheral.
  * @param  I2Cx I2C Instance.
  * @retval None
  */
__STATIC_INLINE void DDL_I2C_DisableReset(I2C_TypeDef *I2Cx)
{
  CLEAR_BIT(I2Cx->CR1, I2C_CR1_SWRSTS);
}

/**
  * @brief  Check if the I2C peripheral is under reset state or not.
  * @param  I2Cx I2C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_I2C_IsResetEnabled(I2C_TypeDef *I2Cx)
{
  return (READ_BIT(I2Cx->CR1, I2C_CR1_SWRSTS) == (I2C_CR1_SWRSTS));
}

/**
  * @brief  Configure the Peripheral clock frequency.
  * @param  I2Cx I2C Instance.
  * @param  PeriphClock Peripheral Clock (in Hz)
  * @retval None
  */
__STATIC_INLINE void DDL_I2C_SetPeriphClock(I2C_TypeDef *I2Cx, uint32_t PeriphClock)
{
  MODIFY_REG(I2Cx->CR2, I2C_CR2_CLKFCFG, __DDL_I2C_FREQ_HZ_TO_MHZ(PeriphClock));
}

/**
  * @brief  Get the Peripheral clock frequency.
  * @param  I2Cx I2C Instance.
  * @retval Value of Peripheral Clock (in Hz)
  */
__STATIC_INLINE uint32_t DDL_I2C_GetPeriphClock(I2C_TypeDef *I2Cx)
{
  return (uint32_t)(__DDL_I2C_FREQ_MHZ_TO_HZ(READ_BIT(I2Cx->CR2, I2C_CR2_CLKFCFG)));
}

/**
  * @brief  Enable TXE interrupt.
  * @param  I2Cx I2C Instance.
  * @retval None
  */
__STATIC_INLINE void DDL_I2C_EnableIT_TX(I2C_TypeDef *I2Cx)
{
  SET_BIT(I2Cx->CR2, I2C_CR2_EVIEN | I2C_CR2_BUFIEN);
}

/**
  * @brief  Disable TXE interrupt.
  * @param  I2Cx I2C Instance.
  * @retval None
  */
__STATIC_INLINE void DDL_I2C_DisableIT_TX(I2C_TypeDef *I2Cx)
{
  CLEAR_BIT(I2Cx->CR2, I2C_CR2_EVIEN | I2C_CR2_BUFIEN);
}

/**
  * @brief  Check if the TXE Interrupt is enabled or disabled.
  * @param  I2Cx I2C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_I2C_IsEnabledIT_TX(I2C_TypeDef *I2Cx)
{
  return (READ_BIT(I2Cx->CR2, I2C_CR2_EVIEN | I2C_CR2_BUFIEN) == (I2C_CR2_EVIEN | I2C_CR2_BUFIEN));
}

/**
  * @brief  Enable RXNE interrupt.
  * @param  I2Cx I2C Instance.
  * @retval None
  */
__STATIC_INLINE void DDL_I2C_EnableIT_RX(I2C_TypeDef *I2Cx)
{
  SET_BIT(I2Cx->CR2, I2C_CR2_EVIEN | I2C_CR2_BUFIEN);
}

/**
  * @brief  Disable RXNE interrupt.
  * @param  I2Cx I2C Instance.
  * @retval None
  */
__STATIC_INLINE void DDL_I2C_DisableIT_RX(I2C_TypeDef *I2Cx)
{
  CLEAR_BIT(I2Cx->CR2, I2C_CR2_EVIEN | I2C_CR2_BUFIEN);
}

/**
  * @brief  Check if the RXNE Interrupt is enabled or disabled.
  * @param  I2Cx I2C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_I2C_IsEnabledIT_RX(I2C_TypeDef *I2Cx)
{
  return (READ_BIT(I2Cx->CR2, I2C_CR2_EVIEN | I2C_CR2_BUFIEN) == (I2C_CR2_EVIEN | I2C_CR2_BUFIEN));
}

/**
  * @brief  Enable Events interrupts.
  * @note   Any of these events will generate interrupt :
  *         Start Bit (SB)
  *         Address sent, Address matched (ADDR)
  *         10-bit header sent (ADD10)
  *         Stop detection  (STOPF)
  *         Byte transfer finished (BTF)
  *
  * @note   Any of these events will generate interrupt if Buffer interrupts are enabled too(using unitary function @ref DDL_I2C_EnableIT_BUF()) :
  *         Receive buffer not empty (RXNE)
  *         Transmit buffer empty (TXE)
  * @param  I2Cx I2C Instance.
  * @retval None
  */
__STATIC_INLINE void DDL_I2C_EnableIT_EVT(I2C_TypeDef *I2Cx)
{
  SET_BIT(I2Cx->CR2, I2C_CR2_EVIEN);
}

/**
  * @brief  Disable Events interrupts.
  * @note   Any of these events will generate interrupt :
  *         Start Bit (SB)
  *         Address sent, Address matched (ADDR)
  *         10-bit header sent (ADD10)
  *         Stop detection  (STOPF)
  *         Byte transfer finished (BTF)
  *         Receive buffer not empty (RXNE)
  *         Transmit buffer empty (TXE)
  * @param  I2Cx I2C Instance.
  * @retval None
  */
__STATIC_INLINE void DDL_I2C_DisableIT_EVT(I2C_TypeDef *I2Cx)
{
  CLEAR_BIT(I2Cx->CR2, I2C_CR2_EVIEN);
}

/**
  * @brief  Check if Events interrupts are enabled or disabled.
  * @param  I2Cx I2C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_I2C_IsEnabledIT_EVT(I2C_TypeDef *I2Cx)
{
  return (READ_BIT(I2Cx->CR2, I2C_CR2_EVIEN) == (I2C_CR2_EVIEN));
}

/**
  * @brief  Enable Buffer interrupts.
  * @note   Any of these Buffer events will generate interrupt if Events interrupts are enabled too(using unitary function @ref DDL_I2C_EnableIT_EVT()) :
  *         Receive buffer not empty (RXNE)
  *         Transmit buffer empty (TXE)
  * @param  I2Cx I2C Instance.
  * @retval None
  */
__STATIC_INLINE void DDL_I2C_EnableIT_BUF(I2C_TypeDef *I2Cx)
{
  SET_BIT(I2Cx->CR2, I2C_CR2_BUFIEN);
}

/**
  * @brief  Disable Buffer interrupts.
  * @note   Any of these Buffer events will generate interrupt :
  *         Receive buffer not empty (RXNE)
  *         Transmit buffer empty (TXE)
  * @param  I2Cx I2C Instance.
  * @retval None
  */
__STATIC_INLINE void DDL_I2C_DisableIT_BUF(I2C_TypeDef *I2Cx)
{
  CLEAR_BIT(I2Cx->CR2, I2C_CR2_BUFIEN);
}

/**
  * @brief  Check if Buffer interrupts are enabled or disabled.
  * @param  I2Cx I2C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_I2C_IsEnabledIT_BUF(I2C_TypeDef *I2Cx)
{
  return (READ_BIT(I2Cx->CR2, I2C_CR2_BUFIEN) == (I2C_CR2_BUFIEN));
}

/**
  * @brief  Enable Error interrupts.
  * @note   Macro @ref IS_SMBUS_ALL_INSTANCE(I2Cx) can be used to check whether or not
  *         SMBus feature is supported by the I2Cx Instance.
  * @note   Any of these errors will generate interrupt :
  *         Bus Error detection (BERR)
  *         Arbitration Loss (ARLO)
  *         Acknowledge Failure(AF)
  *         Overrun/Underrun (OVR)
  *         SMBus Timeout detection (TIMEOUT)
  *         SMBus PEC error detection (PECERR)
  *         SMBus Alert pin event detection (SMBALERT)
  * @param  I2Cx I2C Instance.
  * @retval None
  */
__STATIC_INLINE void DDL_I2C_EnableIT_ERR(I2C_TypeDef *I2Cx)
{
  SET_BIT(I2Cx->CR2, I2C_CR2_ERRIEN);
}

/**
  * @brief  Disable Error interrupts.
  * @note   Macro @ref IS_SMBUS_ALL_INSTANCE(I2Cx) can be used to check whether or not
  *         SMBus feature is supported by the I2Cx Instance.
  * @note   Any of these errors will generate interrupt :
  *         Bus Error detection (BERR)
  *         Arbitration Loss (ARLO)
  *         Acknowledge Failure(AF)
  *         Overrun/Underrun (OVR)
  *         SMBus Timeout detection (TIMEOUT)
  *         SMBus PEC error detection (PECERR)
  *         SMBus Alert pin event detection (SMBALERT)
  * @param  I2Cx I2C Instance.
  * @retval None
  */
__STATIC_INLINE void DDL_I2C_DisableIT_ERR(I2C_TypeDef *I2Cx)
{
  CLEAR_BIT(I2Cx->CR2, I2C_CR2_ERRIEN);
}

/**
  * @brief  Check if Error interrupts are enabled or disabled.
  * @param  I2Cx I2C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_I2C_IsEnabledIT_ERR(I2C_TypeDef *I2Cx)
{
  return (READ_BIT(I2Cx->CR2, I2C_CR2_ERRIEN) == (I2C_CR2_ERRIEN));
}

/**
  * @brief  Enable DMA transmission requests.
  * @param  I2Cx I2C Instance.
  * @retval None
  */
__STATIC_INLINE void DDL_I2C_EnableDMAReq_TX(I2C_TypeDef *I2Cx)
{
  SET_BIT(I2Cx->CR2, I2C_CR2_DMAEN);
}

/**
  * @brief  Disable DMA transmission requests.
  * @param  I2Cx I2C Instance.
  * @retval None
  */
__STATIC_INLINE void DDL_I2C_DisableDMAReq_TX(I2C_TypeDef *I2Cx)
{
  CLEAR_BIT(I2Cx->CR2, I2C_CR2_DMAEN);
}

/**
  * @brief  Check if DMA transmission requests are enabled or disabled.
  * @param  I2Cx I2C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_I2C_IsEnabledDMAReq_TX(I2C_TypeDef *I2Cx)
{
  return (READ_BIT(I2Cx->CR2, I2C_CR2_DMAEN) == (I2C_CR2_DMAEN));
}

/**
  * @brief  Enable DMA reception requests.
  * @param  I2Cx I2C Instance.
  * @retval None
  */
__STATIC_INLINE void DDL_I2C_EnableDMAReq_RX(I2C_TypeDef *I2Cx)
{
  SET_BIT(I2Cx->CR2, I2C_CR2_DMAEN);
}

/**
  * @brief  Disable DMA reception requests.
  * @param  I2Cx I2C Instance.
  * @retval None
  */
__STATIC_INLINE void DDL_I2C_DisableDMAReq_RX(I2C_TypeDef *I2Cx)
{
  CLEAR_BIT(I2Cx->CR2, I2C_CR2_DMAEN);
}

/**
  * @brief  Check if DMA reception requests are enabled or disabled.
  * @param  I2Cx I2C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_I2C_IsEnabledDMAReq_RX(I2C_TypeDef *I2Cx)
{
  return (READ_BIT(I2Cx->CR2, I2C_CR2_DMAEN) == (I2C_CR2_DMAEN));
}

/**
  * @brief  Get the data register address used for DMA transfer.
  * @param  I2Cx I2C Instance.
  * @retval Address of data register
  */
__STATIC_INLINE uint32_t DDL_I2C_DMA_GetRegAddr(I2C_TypeDef *I2Cx)
{
  return (uint32_t) & (I2Cx->DR);
}

/**
  * @brief  Enable DMA last transfer.
  * @note   This action mean that next DMA EOT is the last transfer.
  * @param  I2Cx I2C Instance.
  * @retval None
  */
__STATIC_INLINE void DDL_I2C_EnableLastDMA(I2C_TypeDef *I2Cx)
{
  SET_BIT(I2Cx->CR2, I2C_CR2_LTCFG);
}

/**
  * @brief  Disable DMA last transfer.
  * @note   This action mean that next DMA EOT is not the last transfer.
  * @param  I2Cx I2C Instance.
  * @retval None
  */
__STATIC_INLINE void DDL_I2C_DisableLastDMA(I2C_TypeDef *I2Cx)
{
  CLEAR_BIT(I2Cx->CR2, I2C_CR2_LTCFG);
}

/**
  * @brief  Check if DMA last transfer is enabled or disabled.
  * @param  I2Cx I2C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_I2C_IsEnabledLastDMA(I2C_TypeDef *I2Cx)
{
  return (READ_BIT(I2Cx->CR2, I2C_CR2_LTCFG) == (I2C_CR2_LTCFG));
}

/**
  * @brief  Set the Own Address1.
  * @param  I2Cx I2C Instance.
  * @param  OwnAddress1 This parameter must be a value between Min_Data=0 and Max_Data=0x3FF.
  * @param  OwnAddrSize This parameter can be one of the following values:
  *         @arg @ref DDL_I2C_OWNADDRESS1_7BIT
  *         @arg @ref DDL_I2C_OWNADDRESS1_10BIT
  * @retval None
  */
__STATIC_INLINE void DDL_I2C_SetOwnAddress1(I2C_TypeDef *I2Cx, uint32_t OwnAddress1, uint32_t OwnAddrSize)
{
  MODIFY_REG(I2Cx->SADDR1, I2C_SADDR1_ADDR | I2C_SADDR1_ADDRLEN, OwnAddress1 | OwnAddrSize);
}

/**
  * @brief  Enable acknowledge on Own Address2 match address.
  * @param  I2Cx I2C Instance.
  * @retval None
  */
__STATIC_INLINE void DDL_I2C_EnableOwnAddress2(I2C_TypeDef *I2Cx)
{
  SET_BIT(I2Cx->SADDR2, I2C_SADDR2_ADDRNUM);
}

/**
  * @brief  Disable  acknowledge on Own Address2 match address.
  * @param  I2Cx I2C Instance.
  * @retval None
  */
__STATIC_INLINE void DDL_I2C_DisableOwnAddress2(I2C_TypeDef *I2Cx)
{
  CLEAR_BIT(I2Cx->SADDR2, I2C_SADDR2_ADDRNUM);
}

/**
  * @brief  Check if Own Address1 acknowledge is enabled or disabled.
  * @param  I2Cx I2C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_I2C_IsEnabledOwnAddress2(I2C_TypeDef *I2Cx)
{
  return (READ_BIT(I2Cx->SADDR2, I2C_SADDR2_ADDRNUM) == (I2C_SADDR2_ADDRNUM));
}

/**
  * @brief  Set the 7bits Own Address2.
  * @note   This action has no effect if own address2 is enabled.
  * @param  I2Cx I2C Instance.
  * @param  OwnAddress2 This parameter must be a value between Min_Data=0 and Max_Data=0x7F.
  * @retval None
  */
__STATIC_INLINE void DDL_I2C_SetOwnAddress2(I2C_TypeDef *I2Cx, uint32_t OwnAddress2)
{
  MODIFY_REG(I2Cx->SADDR2, I2C_SADDR2_ADDR2, OwnAddress2);
}

/**
  * @brief  Read Receive Data register.
  * @param  I2Cx I2C Instance.
  * @retval Value between Min_Data=0x0 and Max_Data=0xFF
  */
__STATIC_INLINE uint8_t DDL_I2C_ReceiveData8(I2C_TypeDef *I2Cx)
{
  return (uint8_t)(READ_BIT(I2Cx->DR, I2C_DR_DATA));
}

/**
  * @brief  Write in Transmit Data Register .
  * @param  I2Cx I2C Instance.
  * @param  Data Value between Min_Data=0x0 and Max_Data=0xFF
  * @retval None
  */
__STATIC_INLINE void DDL_I2C_TransmitData8(I2C_TypeDef *I2Cx, uint8_t Data)
{
  MODIFY_REG(I2Cx->DR, I2C_DR_DATA, Data);
}

/**
  * @brief  Indicate the status of Start Bit (master mode).
  * @note   RESET: When No Start condition.
  *         SET: When Start condition is generated.
  * @param  I2Cx I2C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_I2C_IsActiveFlag_Start(I2C_TypeDef *I2Cx)
{
  return (READ_BIT(I2Cx->SR1, I2C_SR1_STARTFLG) == (I2C_SR1_STARTFLG));
}

/**
  * @brief  Indicate the status of Address sent (master mode) or Address matched flag (slave mode).
  * @note   RESET: Clear default value.
  *         SET: When the address is fully sent (master mode) or when the received slave address matched with one of the enabled slave address (slave mode).
  * @param  I2Cx I2C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_I2C_IsActiveFlag_ADDR(I2C_TypeDef *I2Cx)
{
  return (READ_BIT(I2Cx->SR1, I2C_SR1_ADDRFLG) == (I2C_SR1_ADDRFLG));
}

/**
  * @brief  Clear Address Matched flag.
  * @note   Clearing this flag is done by a read access to the I2Cx_SR1
  *         register followed by a read access to the I2Cx_SR2 register.
  * @param  I2Cx I2C Instance.
  * @retval None
  */
__STATIC_INLINE void DDL_I2C_ClearFlag_ADDR(I2C_TypeDef *I2Cx)
{
  __IO uint32_t tmpreg;
  tmpreg = I2Cx->SR1;
  (void) tmpreg;
  tmpreg = I2Cx->SR2;
  (void) tmpreg;
}

/**
  * @brief  Indicate the status of Byte Transfer Finished flag.
  *         RESET: When Data byte transfer not done.
  *         SET: When Data byte transfer succeeded.
  * @param  I2Cx I2C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_I2C_IsActiveFlag_BTF(I2C_TypeDef *I2Cx)
{
  return (READ_BIT(I2Cx->SR1, I2C_SR1_BTCFLG) == (I2C_SR1_BTCFLG));
}

/**
  * @brief  Indicate the status of 10-bit header sent (master mode).
  * @note   RESET: When no ADD10 event occurred.
  *         SET: When the master has sent the first address byte (header).
  * @param  I2Cx I2C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_I2C_IsActiveFlag_ADD10(I2C_TypeDef *I2Cx)
{
  return (READ_BIT(I2Cx->SR1, I2C_SR1_ADDR10FLG) == (I2C_SR1_ADDR10FLG));
}

/**
  * @brief  Indicate the status of Stop detection flag (slave mode).
  * @note   RESET: Clear default value.
  *         SET: When a Stop condition is detected.
  * @param  I2Cx I2C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_I2C_IsActiveFlag_STOP(I2C_TypeDef *I2Cx)
{
  return (READ_BIT(I2Cx->SR1, I2C_SR1_STOPFLG) == (I2C_SR1_STOPFLG));
}

/**
  * @brief  Clear Stop detection flag.
  * @note   Clearing this flag is done by a read access to the I2Cx_SR1
  *         register followed by a write access to I2Cx_CTRL1 register.
  * @param  I2Cx I2C Instance.
  * @retval None
  */
__STATIC_INLINE void DDL_I2C_ClearFlag_STOP(I2C_TypeDef *I2Cx)
{
  __IO uint32_t tmpreg;
  tmpreg = I2Cx->SR1;
  (void) tmpreg;
  SET_BIT(I2Cx->CR1, I2C_CR1_I2CEN);
}

/**
  * @brief  Indicate the status of Receive data register not empty flag.
  * @note   RESET: When Receive data register is read.
  *         SET: When the received data is copied in Receive data register.
  * @param  I2Cx I2C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_I2C_IsActiveFlag_RXNE(I2C_TypeDef *I2Cx)
{
  return (READ_BIT(I2Cx->SR1, I2C_SR1_RXBNEFLG) == (I2C_SR1_RXBNEFLG));
}

/**
  * @brief  Indicate the status of Transmit data register empty flag.
  * @note   RESET: When next data is written in Transmit data register.
  *         SET: When Transmit data register is empty.
  * @param  I2Cx I2C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_I2C_IsActiveFlag_TXE(I2C_TypeDef *I2Cx)
{
  return (READ_BIT(I2Cx->SR1, I2C_SR1_TXBEFLG) == (I2C_SR1_TXBEFLG));
}

/**
  * @brief  Indicate the status of Bus error flag.
  * @note   RESET: Clear default value.
  *         SET: When a misplaced Start or Stop condition is detected.
  * @param  I2Cx I2C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_I2C_IsActiveFlag_BERR(I2C_TypeDef *I2Cx)
{
  return (READ_BIT(I2Cx->SR1, I2C_SR1_BERRFLG) == (I2C_SR1_BERRFLG));
}

/**
  * @brief  Clear Bus error flag.
  * @param  I2Cx I2C Instance.
  * @retval None
  */
__STATIC_INLINE void DDL_I2C_ClearFlag_BERR(I2C_TypeDef *I2Cx)
{
  CLEAR_BIT(I2Cx->SR1, I2C_SR1_BERRFLG);
}

/**
  * @brief  Indicate the status of Arbitration lost flag.
  * @note   RESET: Clear default value.
  *         SET: When arbitration lost.
  * @param  I2Cx I2C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_I2C_IsActiveFlag_ARLO(I2C_TypeDef *I2Cx)
{
  return (READ_BIT(I2Cx->SR1, I2C_SR1_ALFLG) == (I2C_SR1_ALFLG));
}

/**
  * @brief  Clear Arbitration lost flag.
  * @param  I2Cx I2C Instance.
  * @retval None
  */
__STATIC_INLINE void DDL_I2C_ClearFlag_ARLO(I2C_TypeDef *I2Cx)
{
  CLEAR_BIT(I2Cx->SR1, I2C_SR1_ALFLG);
}

/**
  * @brief  Indicate the status of Acknowledge failure flag.
  * @note   RESET: No acknowledge failure.
  *         SET: When an acknowledge failure is received after a byte transmission.
  * @param  I2Cx I2C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_I2C_IsActiveFlag_AF(I2C_TypeDef *I2Cx)
{
  return (READ_BIT(I2Cx->SR1, I2C_SR1_AEFLG) == (I2C_SR1_AEFLG));
}

/**
  * @brief  Clear Acknowledge failure flag.
  * @param  I2Cx I2C Instance.
  * @retval None
  */
__STATIC_INLINE void DDL_I2C_ClearFlag_AF(I2C_TypeDef *I2Cx)
{
  CLEAR_BIT(I2Cx->SR1, I2C_SR1_AEFLG);
}

/**
  * @brief  Indicate the status of Overrun/Underrun flag.
  * @note   RESET: Clear default value.
  *         SET: When an overrun/underrun error occurs (Clock Stretching Disabled).
  * @param  I2Cx I2C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_I2C_IsActiveFlag_OVR(I2C_TypeDef *I2Cx)
{
  return (READ_BIT(I2Cx->SR1, I2C_SR1_OVRURFLG) == (I2C_SR1_OVRURFLG));
}

/**
  * @brief  Clear Overrun/Underrun flag.
  * @param  I2Cx I2C Instance.
  * @retval None
  */
__STATIC_INLINE void DDL_I2C_ClearFlag_OVR(I2C_TypeDef *I2Cx)
{
  CLEAR_BIT(I2Cx->SR1, I2C_SR1_OVRURFLG);
}

/**
  * @brief  Indicate the status of SMBus PEC error flag in reception.
  * @note   Macro @ref IS_SMBUS_ALL_INSTANCE(I2Cx) can be used to check whether or not
  *         SMBus feature is supported by the I2Cx Instance.
  * @param  I2Cx I2C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_I2C_IsActiveSMBusFlag_PECERR(I2C_TypeDef *I2Cx)
{
  return (READ_BIT(I2Cx->SR1, I2C_SR1_PECEFLG) == (I2C_SR1_PECEFLG));
}

/**
  * @brief  Clear SMBus PEC error flag.
  * @param  I2Cx I2C Instance.
  * @retval None
  */
__STATIC_INLINE void DDL_I2C_ClearSMBusFlag_PECERR(I2C_TypeDef *I2Cx)
{
  CLEAR_BIT(I2Cx->SR1, I2C_SR1_PECEFLG);
}

/**
  * @brief  Indicate the status of SMBus Timeout detection flag.
  * @note   Macro @ref IS_SMBUS_ALL_INSTANCE(I2Cx) can be used to check whether or not
  *         SMBus feature is supported by the I2Cx Instance.
  * @param  I2Cx I2C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_I2C_IsActiveSMBusFlag_TIMEOUT(I2C_TypeDef *I2Cx)
{
  return (READ_BIT(I2Cx->SR1, I2C_SR1_TTEFLG) == (I2C_SR1_TTEFLG));
}

/**
  * @brief  Clear SMBus Timeout detection flag.
  * @note   Macro @ref IS_SMBUS_ALL_INSTANCE(I2Cx) can be used to check whether or not
  *         SMBus feature is supported by the I2Cx Instance.
  * @param  I2Cx I2C Instance.
  * @retval None
  */
__STATIC_INLINE void DDL_I2C_ClearSMBusFlag_TIMEOUT(I2C_TypeDef *I2Cx)
{
  CLEAR_BIT(I2Cx->SR1, I2C_SR1_TTEFLG);
}

/**
  * @brief  Indicate the status of SMBus alert flag.
  * @note   Macro @ref IS_SMBUS_ALL_INSTANCE(I2Cx) can be used to check whether or not
  *         SMBus feature is supported by the I2Cx Instance.
  * @param  I2Cx I2C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_I2C_IsActiveSMBusFlag_ALERT(I2C_TypeDef *I2Cx)
{
  return (READ_BIT(I2Cx->SR1, I2C_SR1_SMBALTFLG) == (I2C_SR1_SMBALTFLG));
}

/**
  * @brief  Clear SMBus Alert flag.
  * @note   Macro @ref IS_SMBUS_ALL_INSTANCE(I2Cx) can be used to check whether or not
  *         SMBus feature is supported by the I2Cx Instance.
  * @param  I2Cx I2C Instance.
  * @retval None
  */
__STATIC_INLINE void DDL_I2C_ClearSMBusFlag_ALERT(I2C_TypeDef *I2Cx)
{
  CLEAR_BIT(I2Cx->SR1, I2C_SR1_SMBALTFLG);
}

/**
  * @brief  Indicate the status of Master/Slave flag.
  * @note   RESET: Slave Mode.
  *         SET: Master Mode.
  * @param  I2Cx I2C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_I2C_IsActiveFlag_MSL(I2C_TypeDef *I2Cx)
{
  return (READ_BIT(I2Cx->SR2, I2C_SR2_MSFLG) == (I2C_SR2_MSFLG));
}

/**
  * @brief  Indicate the status of Bus Busy flag.
  * @note   RESET: Clear default value.
  *         SET: When a Start condition is detected.
  * @param  I2Cx I2C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_I2C_IsActiveFlag_BUSY(I2C_TypeDef *I2Cx)
{
  return (READ_BIT(I2Cx->SR2, I2C_SR2_BUSBSYFLG) == (I2C_SR2_BUSBSYFLG));
}

/**
  * @brief  Indicate the value of transfer direction.
  * @note   RESET: Bus is in read transfer (peripheral point of view).
  *         SET: Bus is in write transfer (peripheral point of view).
  * @param  I2Cx I2C Instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_I2C_DIRECTION_WRITE
  *         @arg @ref DDL_I2C_DIRECTION_READ
  */
__STATIC_INLINE uint32_t DDL_I2C_GetTransferDirection(I2C_TypeDef *I2Cx)
{
  return (uint32_t)(READ_BIT(I2Cx->SR2, I2C_SR2_TRFLG));
}

/**
  * @brief  Indicate the status of General call address reception (Slave mode).
  * @note   RESET: No General call address
  *         SET: General call address received.
  * @note   This status is cleared by hardware after a STOP condition or repeated START condition.
  * @param  I2Cx I2C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_I2C_IsActiveFlag_GENCALL(I2C_TypeDef *I2Cx)
{
  return (READ_BIT(I2Cx->SR2, I2C_SR2_GENCALLFLG) == (I2C_SR2_GENCALLFLG));
}

/**
  * @brief  Indicate the status of SMBus Device default address reception (Slave mode).
  * @note   Macro @ref IS_SMBUS_ALL_INSTANCE(I2Cx) can be used to check whether or not
  *         SMBus feature is supported by the I2Cx Instance.
  * @note   RESET: No SMBus Device default address
  *         SET: SMBus Device default address received.
  * @note   This status is cleared by hardware after a STOP condition or repeated START condition.
  * @param  I2Cx I2C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_I2C_IsActiveSMBusFlag_SMBDEFAULT(I2C_TypeDef *I2Cx)
{
  return (READ_BIT(I2Cx->SR2, I2C_SR2_SMBDADDRFLG) == (I2C_SR2_SMBDADDRFLG));
}

/**
  * @brief  Indicate the status of SMBus Host address reception (Slave mode).
  * @note   Macro @ref IS_SMBUS_ALL_INSTANCE(I2Cx) can be used to check whether or not
  *         SMBus feature is supported by the I2Cx Instance.
  * @note   RESET: No SMBus Host address
  *         SET: SMBus Host address received.
  * @note   This status is cleared by hardware after a STOP condition or repeated START condition.
  * @param  I2Cx I2C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_I2C_IsActiveSMBusFlag_SMBHOST(I2C_TypeDef *I2Cx)
{
  return (READ_BIT(I2Cx->SR2, I2C_SR2_SMMHADDRFLG) == (I2C_SR2_SMMHADDRFLG));
}

/**
  * @brief  Indicate the status of Dual flag.
  * @note   RESET: Received address matched with OAR1.
  *         SET: Received address matched with OAR2.
  * @param  I2Cx I2C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_I2C_IsActiveFlag_DUAL(I2C_TypeDef *I2Cx)
{
  return (READ_BIT(I2Cx->SR2, I2C_SR2_DUALADDRFLG) == (I2C_SR2_DUALADDRFLG));
}

/**
  * @brief  Get the SMBus Packet Error byte calculated.
  * @note   Macro @ref IS_SMBUS_ALL_INSTANCE(I2Cx) can be used to check whether or not
  *         SMBus feature is supported by the I2Cx Instance.
  * @param  I2Cx I2C Instance.
  * @retval Value between Min_Data=0x00 and Max_Data=0xFF
  */
__STATIC_INLINE uint32_t DDL_I2C_GetSMBusPEC(I2C_TypeDef *I2Cx)
{
  return (uint32_t)(READ_BIT(I2Cx->SR2, I2C_SR2_PECVAL) >> I2C_SR2_PECVAL_Pos);
}

/**
  * @brief  Configure the SCL high and low period.
  * @note   This bit can only be programmed when the I2C is disabled (PE = 0).
  * @param  I2Cx I2C Instance.
  * @param  ClockPeriod This parameter must be a value between Min_Data=0x004 and Max_Data=0xFFF, except in FAST DUTY mode where Min_Data=0x001.
  * @retval None
  */
__STATIC_INLINE void DDL_I2C_SetClockPeriod(I2C_TypeDef *I2Cx, uint32_t ClockPeriod)
{
  MODIFY_REG(I2Cx->CLKCR, I2C_CLKCR_CLKCFG, ClockPeriod);
}

/**
  * @brief  Get the SCL high and low period.
  * @param  I2Cx I2C Instance.
  * @retval Value between Min_Data=0x004 and Max_Data=0xFFF, except in FAST DUTY mode where Min_Data=0x001.
  */
__STATIC_INLINE uint32_t DDL_I2C_GetClockPeriod(I2C_TypeDef *I2Cx)
{
  return (uint32_t)(READ_BIT(I2Cx->CLKCR, I2C_CLKCR_CLKCFG));
}

/**
  * @brief  Configure the SCL speed.
  * @note   This bit can only be programmed when the I2C is disabled (PE = 0).
  * @param  I2Cx I2C Instance.
  * @param  PeriphClock Peripheral Clock (in Hz)
  * @param  ClockSpeed This parameter must be a value lower than 400kHz (in Hz).
  * @param  DutyCycle This parameter can be one of the following values:
  *         @arg @ref DDL_I2C_DUTYCYCLE_2
  *         @arg @ref DDL_I2C_DUTYCYCLE_16_9
  * @retval None
  */
__STATIC_INLINE void DDL_I2C_ConfigSpeed(I2C_TypeDef *I2Cx, uint32_t PeriphClock, uint32_t ClockSpeed,
                                        uint32_t DutyCycle)
{
  uint32_t freqrange = 0x0U;
  uint32_t clockconfig = 0x0U;

  /* Compute frequency range */
  freqrange = __DDL_I2C_FREQ_HZ_TO_MHZ(PeriphClock);

  /* Configure I2Cx: Frequency range register */
  MODIFY_REG(I2Cx->CR2, I2C_CR2_CLKFCFG, freqrange);

  /* Configure I2Cx: Rise Time register */
  MODIFY_REG(I2Cx->RISETMAX, I2C_RISETMAX_RISETMAX, __DDL_I2C_RISE_TIME(freqrange, ClockSpeed));

  /* Configure Speed mode, Duty Cycle and Clock control register value */
  if (ClockSpeed > DDL_I2C_MAX_SPEED_STANDARD)
  {
      /* Set Speed mode at fast and duty cycle for Clock Speed request in fast clock range */
      clockconfig = DDL_I2C_CLOCK_SPEED_FAST_MODE                                     | \
                    __DDL_I2C_SPEED_FAST_TO_CCR(PeriphClock, ClockSpeed, DutyCycle)   | \
                    DutyCycle;
  }
  else
  {
      /* Set Speed mode at standard for Clock Speed request in standard clock range */
      clockconfig = DDL_I2C_CLOCK_SPEED_STANDARD_MODE                   | \
                    __DDL_I2C_SPEED_STANDARD_TO_CCR(PeriphClock, ClockSpeed);
  }

  /* Configure I2Cx: Clock control register */
  MODIFY_REG(I2Cx->CLKCR, (I2C_CLKCR_SPEEDCFG | I2C_CLKCR_FDUTYCFG | I2C_CLKCR_CLKCFG), clockconfig);
}

/**
  * @brief  Configure peripheral mode.
  * @note   Macro @ref IS_SMBUS_ALL_INSTANCE(I2Cx) can be used to check whether or not
  *         SMBus feature is supported by the I2Cx Instance.
  * @param  I2Cx I2C Instance.
  * @param  PeripheralMode This parameter can be one of the following values:
  *         @arg @ref DDL_I2C_MODE_I2C
  *         @arg @ref DDL_I2C_MODE_SMBUS_HOST
  *         @arg @ref DDL_I2C_MODE_SMBUS_DEVICE
  *         @arg @ref DDL_I2C_MODE_SMBUS_DEVICE_ARP
  * @retval None
  */
__STATIC_INLINE void DDL_I2C_SetMode(I2C_TypeDef *I2Cx, uint32_t PeripheralMode)
{
  MODIFY_REG(I2Cx->CR1, I2C_CR1_SMBEN | I2C_CR1_SMBTCFG | I2C_CR1_ARPEN, PeripheralMode);
}

/**
  * @brief  Get peripheral mode.
  * @note   Macro @ref IS_SMBUS_ALL_INSTANCE(I2Cx) can be used to check whether or not
  *         SMBus feature is supported by the I2Cx Instance.
  * @param  I2Cx I2C Instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_I2C_MODE_I2C
  *         @arg @ref DDL_I2C_MODE_SMBUS_HOST
  *         @arg @ref DDL_I2C_MODE_SMBUS_DEVICE
  *         @arg @ref DDL_I2C_MODE_SMBUS_DEVICE_ARP
  */
__STATIC_INLINE uint32_t DDL_I2C_GetMode(I2C_TypeDef *I2Cx)
{
  return (uint32_t)(READ_BIT(I2Cx->CR1, I2C_CR1_SMBEN | I2C_CR1_SMBTCFG | I2C_CR1_ARPEN));
}

/**
  * @brief  Configure Noise Filters (Analog and Digital).
  * @note   If the analog filter is also enabled, the digital filter is added to analog filter.
  *         The filters can only be programmed when the I2C is disabled (PE = 0).
  * @param  I2Cx I2C Instance.
  * @param  AnalogFilter This parameter can be one of the following values:
  *         @arg @ref DDL_I2C_ANALOGFILTER_ENABLE
  *         @arg @ref DDL_I2C_ANALOGFILTER_DISABLE
  * @param  DigitalFilter This parameter must be a value between Min_Data=0x00 (Digital filter disabled) and Max_Data=0x0F (Digital filter enabled and filtering capability up to 15*TPCLK1)
  *               This parameter is used to configure the digital noise filter on SDA and SCL input. The digital filter will suppress the spikes with a length of up to DNF[3:0]*TPCLK1.
  * @retval None
  */
__STATIC_INLINE void DDL_I2C_ConfigFilters(I2C_TypeDef *I2Cx, uint32_t AnalogFilter, uint32_t DigitalFilter)
{
  MODIFY_REG(I2Cx->FILTER, I2C_FILTER_ANFDISEN | I2C_FILTER_DNFCFG, AnalogFilter | DigitalFilter);
}

/**
  * @brief  Configure Digital Noise Filter.
  * @note   If the analog filter is also enabled, the digital filter is added to analog filter.
  *         This filter can only be programmed when the I2C is disabled (PE = 0).
  * @param  I2Cx I2C Instance.
  * @param  DigitalFilter This parameter must be a value between Min_Data=0x00 (Digital filter disabled) and Max_Data=0x0F (Digital filter enabled and filtering capability up to 15*TPCLK1)
  *               This parameter is used to configure the digital noise filter on SDA and SCL input. The digital filter will suppress the spikes with a length of up to DNF[3:0]*TPCLK1.
  * @retval None
  */
__STATIC_INLINE void DDL_I2C_SetDigitalFilter(I2C_TypeDef *I2Cx, uint32_t DigitalFilter)
{
  MODIFY_REG(I2Cx->FILTER, I2C_FILTER_DNFCFG, DigitalFilter);
}

/**
  * @brief  Get the current Digital Noise Filter configuration.
  * @param  I2Cx I2C Instance.
  * @retval Value between Min_Data=0x0 and Max_Data=0xF
  */
__STATIC_INLINE uint32_t DDL_I2C_GetDigitalFilter(I2C_TypeDef *I2Cx)
{
  return (uint32_t)(READ_BIT(I2Cx->FILTER, I2C_FILTER_DNFCFG));
}

/**
  * @brief  Enable Analog Noise Filter.
  * @note   This filter can only be programmed when the I2C is disabled (PE = 0).
  * @param  I2Cx I2C Instance.
  * @retval None
  */
__STATIC_INLINE void DDL_I2C_EnableAnalogFilter(I2C_TypeDef *I2Cx)
{
  CLEAR_BIT(I2Cx->FILTER, I2C_FILTER_ANFDISEN);
}

/**
  * @brief  Disable Analog Noise Filter.
  * @note   This filter can only be programmed when the I2C is disabled (PE = 0).
  * @param  I2Cx I2C Instance.
  * @retval None
  */
__STATIC_INLINE void DDL_I2C_DisableAnalogFilter(I2C_TypeDef *I2Cx)
{
  SET_BIT(I2Cx->FILTER, I2C_FILTER_ANFDISEN);
}

/**
  * @brief  Check if Analog Noise Filter is enabled or disabled.
  * @param  I2Cx I2C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_I2C_IsEnabledAnalogFilter(I2C_TypeDef *I2Cx)
{
  return (READ_BIT(I2Cx->FILTER, I2C_FILTER_ANFDISEN) == (I2C_FILTER_ANFDISEN));
}

/**
  * @}
  */

#if defined(USE_FULL_DDL_DRIVER)
/** @defgroup I2C_DDL_EF_Init Initialization and de-initialization functions
  * @{
  */

uint32_t DDL_I2C_Init(I2C_TypeDef *I2Cx, DDL_I2C_InitTypeDef *I2C_InitStruct);
uint32_t DDL_I2C_DeInit(I2C_TypeDef *I2Cx);
void DDL_I2C_StructInit(DDL_I2C_InitTypeDef *I2C_InitStruct);

/**
  * @}
  */
#endif /* USE_FULL_DDL_DRIVER */

/**
  * @}
  */

/**
  * @}
  */

#endif /* defined(I2C) */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* G32F031_DDL_I2C_H */
