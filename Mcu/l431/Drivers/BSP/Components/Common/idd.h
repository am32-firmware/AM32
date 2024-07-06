/**
  ******************************************************************************
  * @file    idd.h
  * @author  MCD Application Team
  * @brief   This file contains all the functions prototypes for the IDD driver.
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
#ifndef __IDD_H
#define __IDD_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/** @addtogroup BSP
  * @{
  */

/** @addtogroup Components
  * @{
  */
    
/** @addtogroup IDD
  * @{
  */

/** @defgroup IDD_Exported_Types IDD Exported Types
  * @{
  */

/** @defgroup IDD_Config_structure  IDD Configuration structure
  * @{
  */
typedef struct
{
  uint16_t AmpliGain;       /*!< Specifies ampli gain value
                                 */
  uint16_t VddMin;          /*!< Specifies minimum MCU VDD can reach to protect MCU from reset
                                  */
  uint16_t Shunt0Value;     /*!< Specifies value of Shunt 0 if existing
                                 */
  uint16_t Shunt1Value;     /*!< Specifies value of Shunt 1 if existing
                                 */
  uint16_t Shunt2Value;     /*!< Specifies value of Shunt 2 if existing
                                 */
  uint16_t Shunt3Value;     /*!< Specifies value of Shunt 3 if existing
                                 */
  uint16_t Shunt4Value;     /*!< Specifies value of Shunt 4 if existing
                                  */
  uint16_t Shunt0StabDelay; /*!< Specifies delay of Shunt 0 stabilization if existing
                                  */
  uint16_t Shunt1StabDelay; /*!< Specifies delay of Shunt 1 stabilization if existing
                                  */
  uint16_t Shunt2StabDelay; /*!< Specifies delay of Shunt 2 stabilization if existing
                                  */
  uint16_t Shunt3StabDelay; /*!< Specifies delay of Shunt 3 stabilization if existing
                                  */
  uint16_t Shunt4StabDelay; /*!< Specifies delay of Shunt 4 stabilization if existing
                                  */
  uint8_t ShuntNbOnBoard;   /*!< Specifies number of shunts that are present on board
                                 This parameter can be a value of @ref IDD_shunt_number */
  uint8_t ShuntNbUsed;      /*!< Specifies number of shunts used for measurement
                                 This parameter can be a value of @ref IDD_shunt_number */
  uint8_t VrefMeasurement;  /*!< Specifies if Vref is automatically measured before each Idd measurement
                                 This parameter can be a value of @ref IDD_Vref_Measurement */
  uint8_t Calibration;      /*!< Specifies if calibration is done before each Idd measurement
                                  */
  uint8_t PreDelayUnit;     /*!< Specifies Pre delay unit 
                                 This parameter can be a value of @ref IDD_PreDelay */
  uint8_t PreDelayValue;    /*!< Specifies Pre delay value in selected unit
                                  */
  uint8_t MeasureNb;        /*!< Specifies number of Measure to be performed 
                                 This parameter can be a value between 1 and 256 */
  uint8_t DeltaDelayUnit;   /*!< Specifies Delta delay unit
                                  This parameter can be a value of @ref IDD_DeltaDelay */
  uint8_t DeltaDelayValue;  /*!< Specifies Delta delay between 2 measures
                                  value can be between 1 and 128 */
}IDD_ConfigTypeDef;
/**
  * @}
  */

/** @defgroup IDD_Driver_structure  IDD Driver structure
  * @{
  */
typedef struct
{
  void       (*Init)(uint16_t);
  void       (*DeInit)(uint16_t);
  uint16_t   (*ReadID)(uint16_t);
  void       (*Reset)(uint16_t);
  void       (*LowPower)(uint16_t);
  void       (*WakeUp)(uint16_t);
  void       (*Start)(uint16_t);
  void       (*Config)(uint16_t,IDD_ConfigTypeDef);
  void       (*GetValue)(uint16_t, uint32_t *);
  void       (*EnableIT)(uint16_t);
  void       (*ClearIT)(uint16_t);
  uint8_t    (*GetITStatus)(uint16_t);
  void       (*DisableIT)(uint16_t);
  void       (*ErrorEnableIT)(uint16_t);
  void       (*ErrorClearIT)(uint16_t);
  uint8_t    (*ErrorGetITStatus)(uint16_t);
  void       (*ErrorDisableIT)(uint16_t);
  uint8_t    (*ErrorGetSrc)(uint16_t);
  uint8_t    (*ErrorGetCode)(uint16_t);
}IDD_DrvTypeDef;
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

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __IDD_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
