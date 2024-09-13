/**
  ******************************************************************************
  * @file    stm32l4r9i_discovery_audio.h
  * @author  MCD Application Team
  * @brief   This file contains the common defines and functions prototypes for
  *          the stm32l4r9i_discovery_audio.c driver.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32L4R9I_DISCOVERY_AUDIO_H
#define __STM32L4R9I_DISCOVERY_AUDIO_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#if defined(BSP_AUDIO_USE_RTOS)
#include "k_mem.h"
#else
#include <stdlib.h>
#endif
/* Include audio component Driver */
#include "../Components/cs42l51/cs42l51.h"
#include "stm32l4r9i_discovery.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L4R9I_DISCOVERY
  * @{
  */

/** @addtogroup STM32L4R9I_DISCOVERY_AUDIO
  * @{
  */

/** @defgroup STM32L4R9I_DISCOVERY_AUDIO_Exported_Types Exported Types
  * @{
  */
typedef void (*Audio_CallbackTypeDef)(void);

/**
  * @}
  */

/** @defgroup STM32L4R9I_DISCOVERY_AUDIO_Exported_Constants Exported Constants
  * @{
  */
/** @defgroup BSP_Audio_Out_Option BSP Audio Out Option
  * @{
  */
#define BSP_AUDIO_OUT_CIRCULARMODE      ((uint32_t)0x00000001) /* BUFFER CIRCULAR MODE */
#define BSP_AUDIO_OUT_NORMALMODE        ((uint32_t)0x00000002) /* BUFFER NORMAL MODE   */
#define BSP_AUDIO_OUT_STEREOMODE        ((uint32_t)0x00000004) /* STEREO MODE          */
#define BSP_AUDIO_OUT_MONOMODE          ((uint32_t)0x00000008) /* MONO MODE            */
/**
  * @}
  */

/** @defgroup BSP_Audio_Sample_Rate BSP Audio Sample Rate
  * @{
  */
#define BSP_AUDIO_FREQUENCY_96K         SAI_AUDIO_FREQUENCY_96K
#define BSP_AUDIO_FREQUENCY_48K         SAI_AUDIO_FREQUENCY_48K
#define BSP_AUDIO_FREQUENCY_44K         SAI_AUDIO_FREQUENCY_44K
#define BSP_AUDIO_FREQUENCY_32K         SAI_AUDIO_FREQUENCY_32K
#define BSP_AUDIO_FREQUENCY_22K         SAI_AUDIO_FREQUENCY_22K
#define BSP_AUDIO_FREQUENCY_16K         SAI_AUDIO_FREQUENCY_16K
#define BSP_AUDIO_FREQUENCY_11K         SAI_AUDIO_FREQUENCY_11K
#define BSP_AUDIO_FREQUENCY_8K          SAI_AUDIO_FREQUENCY_8K
/**
  * @}
  */
/*------------------------------------------------------------------------------
                          USER SAI defines parameters
 -----------------------------------------------------------------------------*/
/* SAI DMA Channel definitions */
#define DMA_MAX_SZE                          (uint32_t)0xFFFF

/* Disable SAIx PLL */
#define AUDIO_SAIx_PLL_DISABLE()             HAL_RCCEx_DisablePLLSAI1()

/*------------------------------------------------------------------------------
             CONFIGURATION: Audio Driver Configuration parameters
------------------------------------------------------------------------------*/

#define AUDIODATA_SIZE                      2   /* 16-bits audio data size */

/* Audio status definition */
#define AUDIO_OK                            0
#define AUDIO_ERROR                         1
#define AUDIO_TIMEOUT                       2

/* Audio out parameters */
#define DEFAULT_AUDIO_OUT_FREQ              BSP_AUDIO_FREQUENCY_48K
#define DEFAULT_AUDIO_OUT_BIT_RESOLUTION    ((uint8_t)16)
#define DEFAULT_AUDIO_OUT_CHANNEL_NBR       ((uint8_t)2) /* Mono = 1, Stereo = 2 */
#define DEFAULT_AUDIO_OUT_VOLUME            ((uint16_t)80)

/* AudioFreq * DataSize (2 bytes) * NumChannels (Stereo: 2) */
#define DEFAULT_AUDIO_IN_FREQ               BSP_AUDIO_FREQUENCY_16K
#define DEFAULT_AUDIO_IN_BIT_RESOLUTION     16
#define DEFAULT_AUDIO_IN_CHANNEL_NBR        1 /* Mono = 1, Stereo = 2 */
#define DEFAULT_AUDIO_IN_VOLUME             80

/*------------------------------------------------------------------------------
                    OPTIONAL Configuration defines parameters
------------------------------------------------------------------------------*/

/* Delay for the Codec to be correctly reset */
#define CODEC_RESET_DELAY           5

/*------------------------------------------------------------------------------
                           INPUT DEVICES definition
------------------------------------------------------------------------------*/
/* MP34DT01TR digital microphone on PCB top side */
#define INPUT_DEVICE_DIGITAL_MIC1      0x20    /* Left microphone  */
#define INPUT_DEVICE_DIGITAL_MIC2      0x40    /* Right microphone */
#define INPUT_DEVICE_DIGITAL_MIC       (INPUT_DEVICE_DIGITAL_MIC1 | INPUT_DEVICE_DIGITAL_MIC2)
/* Analog microphone input from 3.5 audio jack connector */
#define INPUT_DEVICE_ANALOG_MIC        INPUT_DEVICE_MIC1

/**
  * @}
  */

/** @defgroup STM32L4R9I_DISCOVERY_AUDIO_Exported_Variables Exported Variables
  * @{
  */
extern SAI_HandleTypeDef          BSP_AUDIO_hSai_Tx;
extern SAI_HandleTypeDef          BSP_AUDIO_hSai_Rx;
extern DFSDM_Filter_HandleTypeDef BSP_AUDIO_hDfsdmLeftFilter;
extern DFSDM_Filter_HandleTypeDef BSP_AUDIO_hDfsdmRightFilter;

 /**
  * @}
  */

/** @defgroup STM32L4R9I_DISCOVERY_AUDIO_Exported_Macros Exported Macros
  * @{
  */
#define DMA_MAX(_X_)                (((_X_) <= DMA_MAX_SZE)? (_X_):DMA_MAX_SZE)

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup STM32L4R9I_DISCOVERY_AUDIO_Exported_Functions Exported Functions
  * @{
  */
uint8_t BSP_AUDIO_OUT_Init(uint16_t OutputDevice, uint8_t Volume, uint32_t AudioFreq);
uint8_t BSP_AUDIO_OUT_DeInit(void);
uint8_t BSP_AUDIO_OUT_Play(uint16_t* pData, uint32_t Size);
uint8_t BSP_AUDIO_OUT_ChangeBuffer(uint16_t *pData, uint16_t Size);
uint8_t BSP_AUDIO_OUT_Pause(void);
uint8_t BSP_AUDIO_OUT_Resume(void);
uint8_t BSP_AUDIO_OUT_Stop(uint32_t Option);
uint8_t BSP_AUDIO_OUT_SetVolume(uint8_t Volume);
uint8_t BSP_AUDIO_OUT_SetFrequency(uint32_t AudioFreq);
void    BSP_AUDIO_OUT_ChangeAudioConfig(uint32_t AudioOutOption);
uint8_t BSP_AUDIO_OUT_SetMute(uint32_t Cmd);
uint8_t BSP_AUDIO_OUT_SetOutputMode(uint8_t Output);
void    BSP_AUDIO_OUT_RegisterCallbacks(Audio_CallbackTypeDef ErrorCallback,
                                        Audio_CallbackTypeDef HalfTransferCallback,
                                        Audio_CallbackTypeDef TransferCompleteCallback);

uint8_t BSP_AUDIO_IN_Init(uint32_t AudioFreq, uint32_t BitRes, uint32_t ChnlNbr);
uint8_t BSP_AUDIO_IN_InitEx(uint16_t InputDevice, uint32_t AudioFreq, uint32_t BitRes, uint32_t ChnlNbr);
uint8_t BSP_AUDIO_IN_DeInit(void);
uint8_t BSP_AUDIO_IN_Record(uint16_t *pData, uint32_t Size);
uint8_t BSP_AUDIO_IN_SetFrequency(uint32_t AudioFreq);
uint8_t BSP_AUDIO_IN_Stop(void);
uint8_t BSP_AUDIO_IN_Pause(void);
uint8_t BSP_AUDIO_IN_Resume(void);
void    BSP_AUDIO_IN_RegisterCallbacks(Audio_CallbackTypeDef ErrorCallback,
                                       Audio_CallbackTypeDef HalfTransferCallback,
                                       Audio_CallbackTypeDef TransferCompleteCallback);
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

#endif /* __STM32L4R9I_DISCOVERY_AUDIO_H */


