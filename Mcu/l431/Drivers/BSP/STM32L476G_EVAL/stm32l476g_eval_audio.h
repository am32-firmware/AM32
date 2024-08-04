/**
  ******************************************************************************
  * @file    stm32l476g_eval_audio.h
  * @author  MCD Application Team
  * @brief   This file contains the common defines and functions prototypes for
  *          the stm32l476g_eval_audio.c driver.
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
#ifndef __STM32L476G_EVAL_AUDIO_H
#define __STM32L476G_EVAL_AUDIO_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Include audio component Driver */
#include "../Components/wm8994/wm8994.h"
#include "stm32l476g_eval.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L476G_EVAL
  * @{
  */

/** @addtogroup STM32L476G_EVAL_AUDIO
  * @{
  */

/** @defgroup STM32L476G_EVAL_AUDIO_Exported_Types Exported Types
  * @{
  */

/**
  * @}
  */ 

/** @defgroup STM32L476G_EVAL_AUDIO_Exported_Constants Exported Constants
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
/** @defgroup CODEC_AudioFrame_SLOT_TDMMode Codex Audio frame slot in TDM mode
  * @brief In W8994 codec the Audio frame contains 4 slots : TDM Mode
  * TDM format :
  * +------------------|------------------|--------------------|-------------------+ 
  * | CODEC_SLOT0 Left | CODEC_SLOT1 Left | CODEC_SLOT0 Right  | CODEC_SLOT1 Right |
  * +------------------------------------------------------------------------------+
  * @{
  */
/* To have 2 separate audio stream in Both headphone and speaker the 4 slot must be activated */
#define CODEC_AUDIOFRAME_SLOT_0123                   SAI_SLOTACTIVE_0 | SAI_SLOTACTIVE_1 | SAI_SLOTACTIVE_2 | SAI_SLOTACTIVE_3
/* To have an audio stream in headphone only SAI Slot 0 and Slot 2 must be activated */ 
#define CODEC_AUDIOFRAME_SLOT_02                     SAI_SLOTACTIVE_0 | SAI_SLOTACTIVE_2 
/* To have an audio stream in speaker only SAI Slot 1 and Slot 3 must be activated */ 
#define CODEC_AUDIOFRAME_SLOT_13                     SAI_SLOTACTIVE_1 | SAI_SLOTACTIVE_3
/**
  * @}
  */
  
/* SAI peripheral configuration defines */
#define AUDIO_SAIx                            SAI1_Block_B
#define AUDIO_SAIx_CLK_ENABLE()               __HAL_RCC_SAI1_CLK_ENABLE()
#define AUDIO_SAIx_MCKB_SCKB_SDB_FSB_AF       GPIO_AF13_SAI1

#define AUDIO_SAIx_MCKB_SCKB_SDB_FSB_ENABLE()   __HAL_RCC_GPIOF_CLK_ENABLE()
#define AUDIO_SAIx_FSB_PIN                      GPIO_PIN_9
#define AUDIO_SAIx_SCKB_PIN                     GPIO_PIN_8
#define AUDIO_SAIx_SDB_PIN                      GPIO_PIN_6
#define AUDIO_SAIx_MCKB_PIN                     GPIO_PIN_7
#define AUDIO_SAIx_MCKB_SCKB_SDB_FSB_GPIO_PORT  GPIOF

#define AUDIO_SAIx_SDA_ENABLE()               __HAL_RCC_GPIOD_CLK_ENABLE()
#define AUDIO_SAIx_SDA_PIN                    GPIO_PIN_6
#define AUDIO_SAIx_SDA_GPIO_PORT              GPIOD

/* SAI DMA Channel definitions */
#define AUDIO_SAIx_DMAx_CLK_ENABLE()         __HAL_RCC_DMA2_CLK_ENABLE()
#define AUDIO_SAIx_DMAx_CHANNEL              DMA2_Channel2
#define AUDIO_SAIx_DMAx_IRQ                  DMA2_Channel2_IRQn
#define AUDIO_SAIx_DMAx_PERIPH_DATA_SIZE     DMA_PDATAALIGN_HALFWORD
#define AUDIO_SAIx_DMAx_MEM_DATA_SIZE        DMA_MDATAALIGN_HALFWORD
#define DMA_MAX_SZE                          (uint32_t)0xFFFF
   
#define AUDIO_SAIx_DMAx_IRQHandler           DMA2_Channel2_IRQHandler

/* Select the interrupt preemption priority for the DMA interrupt */
#define AUDIO_OUT_IRQ_PREPRIO           5   /* Select the preemption priority level(0 is the highest) */   

/*------------------------------------------------------------------------------
                        AUDIO IN CONFIGURATION
------------------------------------------------------------------------------*/
/* DFSDM Configuration defines */
#define AUDIO_DFSDMx_LEFT_CHANNEL                       DFSDM1_Channel4
#define AUDIO_DFSDMx_RIGHT_CHANNEL                      DFSDM1_Channel3
#define AUDIO_DFSDMx_LEFT_FILTER                        DFSDM1_Filter0
#define AUDIO_DFSDMx_RIGHT_FILTER                       DFSDM1_Filter1
#define AUDIO_DFSDMx_CLK_ENABLE()                       __HAL_RCC_DFSDM1_CLK_ENABLE()
#define AUDIO_DFSDMx_CKOUT_PIN                          GPIO_PIN_2
#define AUDIO_DFSDMx_DMIC_DATIN_PIN                     GPIO_PIN_0
#define AUDIO_DFSDMx_CKOUT_DMIC_DATIN_GPIO_PORT         GPIOC
#define AUDIO_DFSDMx_CKOUT_DMIC_DATIN_GPIO_CLK_ENABLE() __HAL_RCC_GPIOC_CLK_ENABLE()
#define AUDIO_DFSDMx_CKOUT_DMIC_DATIN_AF                GPIO_AF6_DFSDM1

/* DFSDM DMA Right and Left channels definitions */
#define AUDIO_DFSDMx_DMAx_CLK_ENABLE()                  __HAL_RCC_DMA1_CLK_ENABLE()
#define AUDIO_DFSDMx_DMAx_LEFT_CHANNEL                  DMA1_Channel4
#define AUDIO_DFSDMx_DMAx_RIGHT_CHANNEL                 DMA1_Channel5
#define AUDIO_DFSDMx_DMAx_LEFT_IRQ                      DMA1_Channel4_IRQn
#define AUDIO_DFSDMx_DMAx_RIGHT_IRQ                     DMA1_Channel5_IRQn
#define AUDIO_DFSDMx_DMAx_PERIPH_DATA_SIZE              DMA_PDATAALIGN_WORD
#define AUDIO_DFSDMx_DMAx_MEM_DATA_SIZE                 DMA_MDATAALIGN_WORD
   
#define AUDIO_DFSDM_DMAx_LEFT_IRQHandler                DMA1_Channel4_IRQHandler
#define AUDIO_DFSDM_DMAx_RIGHT_IRQHandler               DMA1_Channel5_IRQHandler
  
/* Select the interrupt preemption priority and subpriority for the IT/DMA interrupt */
#define AUDIO_IN_IRQ_PREPRIO                6   /* Select the preemption priority level(0 is the highest) */

/*------------------------------------------------------------------------------
             CONFIGURATION: Audio Driver Configuration parameters
------------------------------------------------------------------------------*/

#define AUDIODATA_SIZE                      2   /* 16-bits audio data size */

/* Audio status definition */     
#define AUDIO_OK                            0
#define AUDIO_ERROR                         1
#define AUDIO_TIMEOUT                       2

/* AudioFreq * DataSize (2 bytes) * NumChannels (Stereo: 2) */
#define DEFAULT_AUDIO_IN_FREQ               BSP_AUDIO_FREQUENCY_16K
#define DEFAULT_AUDIO_IN_BIT_RESOLUTION     16
#define DEFAULT_AUDIO_IN_CHANNEL_NBR        2 /* Mono = 1, Stereo = 2 */
#define DEFAULT_AUDIO_IN_VOLUME             64

/* PCM buffer input size in word */
#define INTERNAL_BUFF_SIZE                  256
   
/*------------------------------------------------------------------------------
                    OPTIONAL Configuration defines parameters
------------------------------------------------------------------------------*/

/* Delay for the Codec to be correctly reset */
#define CODEC_RESET_DELAY           5

/**
  * @}
  */
 
/** @defgroup STM32L476G_EVAL_AUDIO_Exported_Variables Exported Variables
  * @{
  */
extern __IO uint16_t AudioInVolume;
 /**
  * @}
  */
   
/** @defgroup STM32L476G_EVAL_AUDIO_Exported_Macros Exported Macros
  * @{
  */
#define DMA_MAX(_X_)                (((_X_) <= DMA_MAX_SZE)? (_X_):DMA_MAX_SZE)

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup STM32L476G_EVAL_AUDIO_Exported_Functions Exported Functions
  * @{
  */
uint8_t BSP_AUDIO_OUT_Init(uint16_t OutputDevice, uint8_t Volume, uint32_t AudioFreq);
uint8_t BSP_AUDIO_OUT_Play(uint16_t* pBuffer, uint32_t Size);
uint8_t BSP_AUDIO_OUT_ChangeBuffer(uint16_t *pData, uint16_t Size);
uint8_t BSP_AUDIO_OUT_Pause(void);
uint8_t BSP_AUDIO_OUT_Resume(void);
uint8_t BSP_AUDIO_OUT_Stop(uint32_t Option);
uint8_t BSP_AUDIO_OUT_SetVolume(uint8_t Volume);
void    BSP_AUDIO_OUT_SetFrequency(uint32_t AudioFreq);
void    BSP_AUDIO_OUT_SetAudioFrameSlot(uint32_t AudioFrameSlot);
void    BSP_AUDIO_OUT_ChangeAudioConfig(uint32_t AudioOutOption);
uint8_t BSP_AUDIO_OUT_SetMute(uint32_t Cmd);
uint8_t BSP_AUDIO_OUT_SetOutputMode(uint8_t Output);

/* User Callbacks: user has to implement these functions in his code if they are needed. */
/* This function is called when the requested data has been completely transferred.*/
void    BSP_AUDIO_OUT_TransferComplete_CallBack(void);

/* This function is called when half of the requested buffer has been transferred. */
void    BSP_AUDIO_OUT_HalfTransfer_CallBack(void);

/* This function is called when an Interrupt due to transfer error on or peripheral
   error occurs. */
void    BSP_AUDIO_OUT_Error_CallBack(void);

uint8_t BSP_AUDIO_IN_Init(uint32_t AudioFreq, uint32_t BitRes, uint32_t ChnlNbr);
uint8_t BSP_AUDIO_IN_Record(uint16_t *pData, uint32_t Size);
uint8_t BSP_AUDIO_IN_SetFrequency(uint32_t AudioFreq);
uint8_t BSP_AUDIO_IN_Stop(void);
uint8_t BSP_AUDIO_IN_Pause(void);
uint8_t BSP_AUDIO_IN_Resume(void);
uint8_t BSP_AUDIO_IN_SetVolume(uint8_t Volume);
void    BSP_AUDIO_IN_DeInit(void);
/* User Callbacks: user has to implement these functions in his code if they are needed. */
/* This function should be implemented by the user application.
   It is called into this driver when the current buffer is filled to prepare the next
   buffer pointer and its size. */
void    BSP_AUDIO_IN_TransferComplete_CallBack(void);
void    BSP_AUDIO_IN_HalfTransfer_CallBack(void);

/* This function is called when an Interrupt due to transfer error on or peripheral
   error occurs. */
void    BSP_AUDIO_IN_Error_Callback(void);

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

#endif /* __STM32L476G_EVAL_AUDIO_H */


