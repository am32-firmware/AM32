/**
  ******************************************************************************
  * @file    stm32l476g_eval_audio.c
  * @author  MCD Application Team
  * @brief   This file provides the Audio driver for the STM32L476G-EVAL evaluation board.
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

/*==============================================================================
                                 User NOTES
                                 
1. How To use this driver:
--------------------------
   + This driver supports STM32L4xx devices on STM32L476G-EVAL (MB1144) Evaluation boards.
        a) to play an audio file (all functions names start by BSP_AUDIO_OUT_xxx)
        b) to record an audio file through MP34DT01TR, ST MEMS (all functions names start by BSP_AUDIO_IN_xxx)

a) PLAY A FILE:
==============
   + Call the function BSP_AUDIO_OUT_Init(
                                    OutputDevice: physical output mode (OUTPUT_DEVICE_SPEAKER, 
                                                  OUTPUT_DEVICE_HEADPHONE or OUTPUT_DEVICE_BOTH)
                                    Volume      : Initial volume to be set (0 is min (mute), 100 is max (100%)
                                    AudioFreq   : Audio frequency in Hz (8000, 16000, 22500, 32000...)
                                                  this parameter is relative to the audio file/stream type.
                                   )
      This function configures all the hardware required for the audio application (codec, I2C, SAI, 
      GPIOs, DMA and interrupt if needed). This function returns AUDIO_OK if configuration is OK.
      If the returned value is different from AUDIO_OK or the function is stuck then the communication with
      the codec or the IOExpander has failed (try to un-plug the power or reset device in this case).
      - OUTPUT_DEVICE_SPEAKER  : only speaker will be set as output for the audio stream.
      - OUTPUT_DEVICE_HEADPHONE: only headphones will be set as output for the audio stream.
      - OUTPUT_DEVICE_BOTH     : both Speaker and Headphone are used as outputs for the audio stream
                                 at the same time.
   + Call the function BSP_AUDIO_OUT_Play(
                                  pBuffer: pointer to the audio data file address
                                  Size   : size of the buffer to be sent in Bytes
                                 )
      to start playing (for the first time) from the audio file/stream.
   + Call the function BSP_AUDIO_OUT_Pause() to pause playing   
   + Call the function BSP_AUDIO_OUT_Resume() to resume playing.
       Note. After calling BSP_AUDIO_OUT_Pause() function for pause, only BSP_AUDIO_OUT_Resume() should be called
          for resume (it is not allowed to call BSP_AUDIO_OUT_Play() in this case).
       Note. This function should be called only when the audio file is played or paused (not stopped).
   + For each mode, you may need to implement the relative callback functions into your code.
      The Callback functions are named BSP_AUDIO_OUT_XXXCallBack() and only their prototypes are declared in 
      the stm32l476g_eval_audio.h file. (refer to the example for more details on the callbacks implementations)
   + To Stop playing, to modify the volume level, the frequency, the audio frame slot, 
      the device output mode, the mute, the audio configuratio or the stop, use the functions: BSP_AUDIO_OUT_SetVolume(), 
      AUDIO_OUT_SetFrequency(), BSP_AUDIO_OUT_SetAudioFrameSlot(), BSP_AUDIO_OUT_SetOutputMode(),
      BSP_AUDIO_OUT_SetMute(), BSP_AUDIO_OUT_ChangeAudioConfig() and BSP_AUDIO_OUT_Stop().
   + The driver API and the callback functions are at the end of the stm32l476g_eval_audio.h file.
 
Driver architecture:
--------------------
   + This driver provide the High Audio Layer: consists of the function API exported in the stm32l476g_eval_audio.h file
     (BSP_AUDIO_OUT_Init(), BSP_AUDIO_OUT_Play() ...)
   + This driver provide also the Media Access Layer (MAL): which consists of functions allowing to access the media containing/
     providing the audio file/stream. These functions are also included as local functions into
     the stm32l476g_eval_audio_codec.c file (SAIx_MspInit() and SAIx_Init())   

Known Limitations:
------------------
   1- When using the Speaker, if the audio file quality is not high enough, the speaker output
      may produce high and uncomfortable noise level. To avoid this issue, to use speaker
      output properly, try to increase audio file sampling rate (typically higher than 48KHz).
      This operation will lead to larger file size.
   2- Communication with the audio codec (through I2C) may be corrupted if it is interrupted by some
      user interrupt routines (in this case, interrupts could be disabled just before the start of 
      communication then re-enabled when it is over). Note that this communication is only done at
      the configuration phase (BSP_AUDIO_OUT_Init() or BSP_AUDIO_OUT_Stop()) and when Volume control modification is 
      performed (BSP_AUDIO_OUT_SetVolume() or BSP_AUDIO_OUT_SetMute()or BSP_AUDIO_OUT_SetOutputMode()). 
      When the audio data is played, no communication is required with the audio codec.
   3- Parsing of audio file is not implemented (in order to determine audio file properties: Mono/Stereo, Data size, 
      File size, Audio Frequency, Audio Data header size ...). The configuration is fixed for the given audio file.
   4- If the TDM Format used to play in parallel 2 audio Stream (the first Stream is configured in codec SLOT0 and second 
      Stream in SLOT1) the Pause/Resume, volume and mute feature will control the both streams.
   5- Supports only Stereo audio streaming.
   6- Supports only 16-bits audio data size.

b) RECORD A FILE:
================
   + Call the function BSP_AUDIO_IN_Init(
                                    AudioFreq: Audio frequency in Hz (8000, 16000, 22500, 32000 ...)
                                    )
      This function configures all the hardware required for the audio application (SAI, 
      GPIOs, DMA and interrupt if needed). This function returns 0 if configuration is OK.

   + Call the function BSP_AUDIO_IN_Record(
                            pbuf Main buffer pointer for the recorded data storing  
                            size Current size of the recorded buffer
                            )
      to start recording from the microphones.

   + User needs to implement user callbacks to follow the process of data recorded in the record buffer
      (BSP_AUDIO_IN_HalfTransfer_CallBack()/BSP_AUDIO_IN_TransferComplete_CallBack()

   + Call the function AUDIO_IN_STOP() to stop recording 
==============================================================================*/

/* Includes ------------------------------------------------------------------*/
#include "stm32l476g_eval_audio.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L476G_EVAL
  * @{
  */

/** @defgroup STM32L476G_EVAL_AUDIO STM32L476G_EVAL AUDIO
  * @brief This file includes the low layer driver for wm8994 Audio Codec
  *        available on STM32L476G-EVAL evaluation board(MB1144).
  * @{
  */ 

/** @defgroup STM32L476G_EVAL_AUDIO_Private_Types Private Types
  * @{
  */
typedef struct
{
  uint32_t      Frequency;      /* Record Frequency */
  uint32_t      BitResolution;  /* Record bit resolution */
  uint32_t      ChannelNbr;     /* Record Channel Number */
  uint16_t      *pRecBuf;       /* Pointer to record user buffer */
  uint32_t      RecSize;        /* Size to record in mono, double size to record in stereo */
}AUDIOIN_TypeDef;
/**
  * @}
  */ 
  
/* Private defines ------------------------------------------------------------*/
/** @defgroup STM32L476G_EVAL_AUDIO_Private_Constants Private Constants
  * @{
  */
/**
  * @}
  */ 

/* Private macros ------------------------------------------------------------*/
/** @defgroup STM32L476G_EVAL_AUDIO_Private_Macros Private Macros
  * @{
  */
/*### PLAY ###*/
/* SCK(kHz) = SAI_CK_x/(SAIClockDivider*2*256) */
#define SAIClockDivider(__FREQUENCY__) \
        (__FREQUENCY__ == AUDIO_FREQUENCY_8K)  ? 12 \
      : (__FREQUENCY__ == AUDIO_FREQUENCY_11K) ? 2 \
      : (__FREQUENCY__ == AUDIO_FREQUENCY_16K) ? 6 \
      : (__FREQUENCY__ == AUDIO_FREQUENCY_22K) ? 1 \
      : (__FREQUENCY__ == AUDIO_FREQUENCY_32K) ? 3 \
      : (__FREQUENCY__ == AUDIO_FREQUENCY_44K) ? 0 \
      : (__FREQUENCY__ == AUDIO_FREQUENCY_48K) ? 2 : 1  \

/*### RECORD ###*/
#define DFSDMOverSampling(__FREQUENCY__) \
        (__FREQUENCY__ == AUDIO_FREQUENCY_8K)  ? 256 \
      : (__FREQUENCY__ == AUDIO_FREQUENCY_11K) ? 256 \
      : (__FREQUENCY__ == AUDIO_FREQUENCY_16K) ? 128 \
      : (__FREQUENCY__ == AUDIO_FREQUENCY_22K) ? 128 \
      : (__FREQUENCY__ == AUDIO_FREQUENCY_32K) ? 64 \
      : (__FREQUENCY__ == AUDIO_FREQUENCY_44K) ? 64  \
      : (__FREQUENCY__ == AUDIO_FREQUENCY_48K) ? 32 : 16  \

#define DFSDMClockDivider(__FREQUENCY__) \
        (__FREQUENCY__ == AUDIO_FREQUENCY_8K)  ? 24 \
      : (__FREQUENCY__ == AUDIO_FREQUENCY_11K) ? 4 \
      : (__FREQUENCY__ == AUDIO_FREQUENCY_16K) ? 24 \
      : (__FREQUENCY__ == AUDIO_FREQUENCY_22K) ? 4 \
      : (__FREQUENCY__ == AUDIO_FREQUENCY_32K) ? 24 \
      : (__FREQUENCY__ == AUDIO_FREQUENCY_44K) ? 4  \
      : (__FREQUENCY__ == AUDIO_FREQUENCY_48K) ? 32 : 32  \

#define DFSDMFilterOrder(__FREQUENCY__) \
        (__FREQUENCY__ == AUDIO_FREQUENCY_8K)  ? DFSDM_FILTER_SINC3_ORDER \
      : (__FREQUENCY__ == AUDIO_FREQUENCY_11K) ? DFSDM_FILTER_SINC3_ORDER \
      : (__FREQUENCY__ == AUDIO_FREQUENCY_16K) ? DFSDM_FILTER_SINC3_ORDER \
      : (__FREQUENCY__ == AUDIO_FREQUENCY_22K) ? DFSDM_FILTER_SINC3_ORDER \
      : (__FREQUENCY__ == AUDIO_FREQUENCY_32K) ? DFSDM_FILTER_SINC4_ORDER \
      : (__FREQUENCY__ == AUDIO_FREQUENCY_44K) ? DFSDM_FILTER_SINC4_ORDER  \
      : (__FREQUENCY__ == AUDIO_FREQUENCY_48K) ? DFSDM_FILTER_SINC4_ORDER : DFSDM_FILTER_SINC5_ORDER  \

#define DFSDMRightBitShift(__FREQUENCY__) \
        (__FREQUENCY__ == AUDIO_FREQUENCY_8K)  ? 3 \
      : (__FREQUENCY__ == AUDIO_FREQUENCY_11K) ? 3 \
      : (__FREQUENCY__ == AUDIO_FREQUENCY_16K) ? 0 \
      : (__FREQUENCY__ == AUDIO_FREQUENCY_22K) ? 0 \
      : (__FREQUENCY__ == AUDIO_FREQUENCY_32K) ? 3 \
      : (__FREQUENCY__ == AUDIO_FREQUENCY_44K) ? 3  \
      : (__FREQUENCY__ == AUDIO_FREQUENCY_48K) ? 0 : 0  \

/* Saturate the record PCM sample */
#define SaturaLH(N, L, H) (((N)<(L))?(L):(((N)>(H))?(H):(N)))

/**
  * @}
  */ 
  
/* Private variables ---------------------------------------------------------*/
/** @defgroup STM32L476G_EVAL_AUDIO_Private_Variables Private Variables
  * @{
  */
/*### PLAY ###*/
AUDIO_DrvTypeDef                *audio_drv;
SAI_HandleTypeDef               haudio_out_sai;

/*### RECORD ###*/
AUDIOIN_TypeDef                 audio_in_context;

DFSDM_Channel_HandleTypeDef     haudio_in_dfsdm_leftchannel;
DFSDM_Channel_HandleTypeDef     haudio_in_dfsdm_rightchannel;
DFSDM_Filter_HandleTypeDef      haudio_in_dfsdm_leftfilter;
DFSDM_Filter_HandleTypeDef      haudio_in_dfsdm_rightfilter;
DMA_HandleTypeDef               hdma_dfsdm_left;
DMA_HandleTypeDef               hdma_dfsdm_right;

/* Buffers for right and left samples */
int32_t                         LeftRecBuff[INTERNAL_BUFF_SIZE];
int32_t                         RightRecBuff[INTERNAL_BUFF_SIZE];

/* Buffers status flags */
uint32_t                        DmaLeftRecHalfBuffCplt  = 0;
uint32_t                        DmaLeftRecBuffCplt      = 0;
uint32_t                        DmaRightRecHalfBuffCplt = 0;
uint32_t                        DmaRightRecBuffCplt     = 0;

/* Application Buffer Trigger */
__IO uint32_t                   AppBuffTrigger          = 0;
uint32_t __IO                   AppBuffHalf             = 0;

/* Default record volume */
uint16_t __IO AudioInVolume = DEFAULT_AUDIO_IN_VOLUME;
    
/**
  * @}
  */ 

/* Private function prototypes -----------------------------------------------*/
/** @defgroup STM32L476G_EVAL_AUDIO_Private_Functions Private Functions
  * @{
  */
static void    CODEC_Reset(void);
static void    SAIx_MspInit(void);
static void    SAIx_Init(uint32_t AudioFreq);
static void    DFSDMx_ChannelMspInit(DFSDM_Channel_HandleTypeDef *hdfsdm_channel);
static void    DFSDMx_FilterMspInit(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
static void    DFSDMx_ChannelMspDeInit(DFSDM_Channel_HandleTypeDef *hdfsdm_channel);
static void    DFSDMx_FilterMspDeInit(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
static uint8_t DFSDMx_Init(uint32_t AudioFreq);
static uint8_t DFSDMx_DeInit(void);
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup STM32L476G_EVAL_AUDIO_Exported_Functions
  * @{
  */ 

/**
  * @brief  Configures the audio peripherals.
  * @param  OutputDevice: OUTPUT_DEVICE_SPEAKER, OUTPUT_DEVICE_HEADPHONE,
  *                       or OUTPUT_DEVICE_BOTH.
  * @param  Volume: Initial volume level (from 0 (Mute) to 100 (Max))
  * @param  AudioFreq: Audio frequency used to play the audio stream.
  * @note   The SAI PLL input clock must be done in the user application.  
  * @retval AUDIO_OK if correct communication, else wrong communication
  */
uint8_t BSP_AUDIO_OUT_Init(uint16_t OutputDevice, uint8_t Volume, uint32_t AudioFreq)
{ 
  uint8_t ret = AUDIO_ERROR;
  uint32_t deviceid = 0x00;
  RCC_PeriphCLKInitTypeDef RCC_ExCLKInitStruct;
  
  HAL_RCCEx_GetPeriphCLKConfig(&RCC_ExCLKInitStruct);
  if((AudioFreq == AUDIO_FREQUENCY_11K) || (AudioFreq == AUDIO_FREQUENCY_22K) || (AudioFreq == AUDIO_FREQUENCY_44K))
  {
    /* Configure PLLSAI prescalers */
    /* SAI clock config 
    PLLSAI1_VCO= 8 Mhz * PLLSAI1N = 8 * 24 = VCO_192M 
    SAI_CK_x = PLLSAI1_VCO/PLLSAI1P = 192/17 = 11.294 Mhz */  
    RCC_ExCLKInitStruct.PeriphClockSelection    = RCC_PERIPHCLK_SAI1;
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1N        = 24; 
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1P        = 17; 
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK;
    RCC_ExCLKInitStruct.Sai1ClockSelection      = RCC_SAI1CLKSOURCE_PLLSAI1;
    HAL_RCCEx_PeriphCLKConfig(&RCC_ExCLKInitStruct);
  }
  else /* AUDIO_FREQUENCY_8K, AUDIO_FREQUENCY_16K, AUDIO_FREQUENCY_48K, AUDIO_FREQUENCY_96K */
  {
    /* SAI clock config 
    PLLSAI1_VCO= 8 Mhz * PLLSAI1N = 8 * 43 = VCO_344M 
    SAI_CK_x = PLLSAI1_VCO/PLLSAI1P = 344/7 = 49.142 Mhz */  
    RCC_ExCLKInitStruct.PeriphClockSelection    = RCC_PERIPHCLK_SAI1;
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1N        = 43;
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1P        = 7;
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK;
    RCC_ExCLKInitStruct.Sai1ClockSelection      = RCC_SAI1CLKSOURCE_PLLSAI1;
    HAL_RCCEx_PeriphCLKConfig(&RCC_ExCLKInitStruct);    
  }
  
  /* SAI data transfer preparation:
  Prepare the Media to be used for the audio transfer from memory to SAI peripheral */
  SAIx_Init(AudioFreq);

  /* wm8994 codec initialization */
  deviceid = wm8994_drv.ReadID(AUDIO_I2C_ADDRESS);
  
  /* Reset the Codec Registers */
  CODEC_Reset();
  
  if((deviceid) == WM8994_ID)
  {  
    /* Initialize the audio driver structure */
    audio_drv = &wm8994_drv; 
    ret = AUDIO_OK;
  }
  else
  {
    ret = AUDIO_ERROR;
  }

  if(ret == AUDIO_OK)
  {
    /* Initialize the codec internal registers */
    audio_drv->Init(AUDIO_I2C_ADDRESS, OutputDevice, Volume, AudioFreq);
  }
  
  /* Set the request volume */
  BSP_AUDIO_OUT_SetVolume(Volume);
  
  return ret;
}

/**
  * @brief  Starts playing audio stream from a data buffer for a determined size. 
  * @param  pBuffer: Pointer to the buffer 
  * @param  Size: Number of audio data BYTES.
  * @retval AUDIO_OK if correct communication, else wrong communication
  */
uint8_t BSP_AUDIO_OUT_Play(uint16_t* pBuffer, uint32_t Size)
{
  /* Call the audio Codec Play function */
  if(audio_drv->Play(AUDIO_I2C_ADDRESS, pBuffer, Size) != 0)
  {  
    return AUDIO_ERROR;
  }
  else
  {
    /* Update the Media layer and enable it for play */  
    HAL_SAI_Transmit_DMA(&haudio_out_sai, (uint8_t *)pBuffer, DMA_MAX(Size));
        
    return AUDIO_OK;
  }
}

/**
  * @brief  Sends n-Bytes on the SAI interface.
  * @param  pData: pointer on data address 
  * @param  Size: number of data to be written
  * @retval AUDIO_OK if correct communication, else wrong communication
  */
uint8_t BSP_AUDIO_OUT_ChangeBuffer(uint16_t *pData, uint16_t Size)
{
   return (HAL_SAI_Transmit_DMA(&haudio_out_sai, (uint8_t *)pData, Size));
}

/**
  * @brief  This function Pauses the audio file stream. In case
  *         of using DMA, the DMA Pause feature is used.
  * @note When calling BSP_AUDIO_OUT_Pause() function for pause, only
  *       BSP_AUDIO_OUT_Resume() function should be called for resume
  *       (use of BSP_AUDIO_OUT_Play() function for resume could lead 
  *       to unexpected behavior).
  * @retval AUDIO_OK if correct communication, else wrong communication
  */
uint8_t BSP_AUDIO_OUT_Pause(void)
{
  /* Call the Audio Codec Pause function */
  if(audio_drv->Pause(AUDIO_I2C_ADDRESS) != 0)
  {
    return AUDIO_ERROR;
  }
  
  /* Pause DMA transfer of PCM samples towards the serial audio interface */  
  if (HAL_SAI_DMAPause(&haudio_out_sai)!= HAL_OK)
  {
    return AUDIO_ERROR;
  }
    
  return AUDIO_OK;
}

/**
  * @brief  This function  Resumes the audio file stream.  
  * @note When calling BSP_AUDIO_OUT_Pause() function for pause, only
  *       BSP_AUDIO_OUT_Resume() function should be called for resume
  *       (use of BSP_AUDIO_OUT_Play() function for resume could lead to 
  *       unexpected behavior).
  * @retval AUDIO_OK if correct communication, else wrong communication
  */
uint8_t BSP_AUDIO_OUT_Resume(void)
{    
  /* Call the Audio Codec Resume function */
  if(audio_drv->Resume(AUDIO_I2C_ADDRESS) != 0)
  {
    return AUDIO_ERROR;
  }

  /* Resume DMA transfer of PCM samples towards the serial audio interface */  
  if (HAL_SAI_DMAResume(&haudio_out_sai)!= HAL_OK)
  {
    return AUDIO_ERROR;
  }
  
  return AUDIO_OK;
}

/**
  * @brief  Stops audio playing and Power down the Audio Codec. 
  * @param  Option: could be one of the following parameters 
  *           - CODEC_PDWN_SW: for software power off (by writing registers). 
  *                            Then no need to reconfigure the Codec after power on.
  *           - CODEC_PDWN_HW: completely shut down the codec (physically). 
  *                            Then need to reconfigure the Codec after power on.  
  * @retval AUDIO_OK if correct communication, else wrong communication
  */
uint8_t BSP_AUDIO_OUT_Stop(uint32_t Option)
{
  /* Stop DMA transfer of PCM samples towards the serial audio interface */  
  if (HAL_SAI_DMAStop(&haudio_out_sai)!= HAL_OK)
  {
    return AUDIO_ERROR;
  }
  
  /* Call Audio Codec Stop function */
  if(audio_drv->Stop(AUDIO_I2C_ADDRESS, Option) != 0)
  {
    return AUDIO_ERROR;
  }
  
  if(Option == CODEC_PDWN_HW)
  { 
    /* Wait at least 100us */
    HAL_Delay(1);
  }
  
  return AUDIO_OK;
}

/**
  * @brief  Controls the current audio volume level. 
  * @param  Volume: Volume level to be set in percentage from 0% to 100% (0 for 
  *         Mute and 100 for Max volume level).
  * @retval AUDIO_OK if correct communication, else wrong communication
  */
uint8_t BSP_AUDIO_OUT_SetVolume(uint8_t Volume)
{
  /* Call the codec volume control function with converted volume value */
  if(audio_drv->SetVolume(AUDIO_I2C_ADDRESS, Volume) != 0)
  {
    return AUDIO_ERROR;
  }
  
  return AUDIO_OK;
}

/**
  * @brief  Enables or disables the MUTE mode by software 
  * @param  Cmd: Could be AUDIO_MUTE_ON to mute sound or AUDIO_MUTE_OFF to 
  *         unmute the codec and restore previous volume level.
  * @retval AUDIO_OK if correct communication, else wrong communication
  */
uint8_t BSP_AUDIO_OUT_SetMute(uint32_t Cmd)
{ 
  /* Call the Codec Mute function */
  if(audio_drv->SetMute(AUDIO_I2C_ADDRESS, Cmd) != 0)
  {
    return AUDIO_ERROR;
  }
  
  return AUDIO_OK;
}

/**
  * @brief  Switch dynamically (while audio file is played) the output target 
  *         (speaker or headphone).
  * @param  Output: The audio output target: OUTPUT_DEVICE_SPEAKER,
  *         OUTPUT_DEVICE_HEADPHONE or OUTPUT_DEVICE_BOTH
  * @retval AUDIO_OK if correct communication, else wrong communication
  */
uint8_t BSP_AUDIO_OUT_SetOutputMode(uint8_t Output)
{
  /* Call the Codec output device function */
  if(audio_drv->SetOutputMode(AUDIO_I2C_ADDRESS, Output) != 0)
  {
    return AUDIO_ERROR;
  }
  
  return AUDIO_OK;
}

/**
  * @brief  Update the audio frequency.
  * @param  AudioFreq: Audio frequency used to play the audio stream.
  * @note   This API should be called after the BSP_AUDIO_OUT_Init() to adjust the
  *         audio frequency.
  * @note   The SAI PLL configuration done within this function assumes that
  *         the SAI PLL input clock runs at 8 MHz.
  * @retval None
  */
void BSP_AUDIO_OUT_SetFrequency(uint32_t AudioFreq)
{ 
  RCC_PeriphCLKInitTypeDef RCC_ExCLKInitStruct;
  
  /* Retrieve actual RCC configuration */
  HAL_RCCEx_GetPeriphCLKConfig(&RCC_ExCLKInitStruct);
  
  /* Update the PLL configuration according to the new frequency */
  if((AudioFreq == AUDIO_FREQUENCY_11K) || (AudioFreq == AUDIO_FREQUENCY_22K) || (AudioFreq == AUDIO_FREQUENCY_44K))
  {
    /* Configure PLLSAI prescalers */
    /* SAI clock config 
    PLLSAI1_VCO= 8 Mhz * PLLSAI1N = 8 * 24 = VCO_192M 
    SAI_CK_x = PLLSAI1_VCO/PLLSAI1P = 192/17 = 11.294 Mhz */  
    RCC_ExCLKInitStruct.PeriphClockSelection    = RCC_PERIPHCLK_SAI1;
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1N        = 24; 
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1P        = 17; 
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK;
    RCC_ExCLKInitStruct.Sai1ClockSelection      = RCC_SAI1CLKSOURCE_PLLSAI1;
    HAL_RCCEx_PeriphCLKConfig(&RCC_ExCLKInitStruct);
  }
  else /* AUDIO_FREQUENCY_8K, AUDIO_FREQUENCY_16K, AUDIO_FREQUENCY_48K, AUDIO_FREQUENCY_96K */
  {
    /* SAI clock config 
    PLLSAI1_VCO= 8 Mhz * PLLSAI1N = 8 * 43 = VCO_344M 
    SAI_CK_x = PLLSAI1_VCO/PLLSAI1P = 344/7 = 49.142 Mhz */  
    RCC_ExCLKInitStruct.PeriphClockSelection    = RCC_PERIPHCLK_SAI1;
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1N        = 43;
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1P        = 7;
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK;
    RCC_ExCLKInitStruct.Sai1ClockSelection      = RCC_SAI1CLKSOURCE_PLLSAI1;
    HAL_RCCEx_PeriphCLKConfig(&RCC_ExCLKInitStruct);    
  }
  /* Disable SAI peripheral to allow access to SAI internal registers */
  __HAL_SAI_DISABLE(&haudio_out_sai);
  
  /* Update the SAI audio frequency configuration */
  haudio_out_sai.Init.Mckdiv = SAIClockDivider(AudioFreq);
  HAL_SAI_Init(&haudio_out_sai);
  
  /* Enable SAI peripheral to generate MCLK */
  __HAL_SAI_ENABLE(&haudio_out_sai);
}

/**
  * @brief  Updates the Audio frame slot configuration.
  * @param  AudioFrameSlot: specifies the audio Frame slot
  *         This parameter can be any value of @ref CODEC_AudioFrame_SLOT_TDMMode
  * @note   This API should be called after the BSP_AUDIO_OUT_Init() to adjust the
  *         audio frame slot.
  * @retval None
  */
void BSP_AUDIO_OUT_SetAudioFrameSlot(uint32_t AudioFrameSlot)
{ 
  /* Disable SAI peripheral to allow access to SAI internal registers */
  __HAL_SAI_DISABLE(&haudio_out_sai);
  
  /* Update the SAI audio frame slot configuration */
  haudio_out_sai.SlotInit.SlotActive = AudioFrameSlot;
  HAL_SAI_Init(&haudio_out_sai);
  
  /* Enable SAI peripheral to generate MCLK */
  __HAL_SAI_ENABLE(&haudio_out_sai);
}

/**
  * @brief  Changes the Audio Out Configuration.
  * @param  AudioOutOption: specifies the audio out new configuration
  *         This parameter can be any value of @ref BSP_Audio_Out_Option
  * @note   This API should be called after the BSP_AUDIO_OUT_Init() to adjust the
  *         audio out configuration.
  * @retval None
  */
void BSP_AUDIO_OUT_ChangeAudioConfig(uint32_t AudioOutOption)
{ 
  /********** Playback Buffer circular/normal mode **********/
  if(AudioOutOption & BSP_AUDIO_OUT_CIRCULARMODE)
  {
    /* Deinitialize the Stream to update DMA mode */
    HAL_DMA_DeInit(haudio_out_sai.hdmatx);
    
    /* Update the SAI audio Transfer DMA mode */
    haudio_out_sai.hdmatx->Init.Mode = DMA_CIRCULAR;
    
    /* Configure the DMA Stream with new Transfer DMA mode */
    HAL_DMA_Init(haudio_out_sai.hdmatx);      
  }
  else /* BSP_AUDIO_OUT_NORMALMODE */
  {
    /* Deinitialize the Stream to update DMA mode */
    HAL_DMA_DeInit(haudio_out_sai.hdmatx);
    
    /* Update the SAI audio Transfer DMA mode */
    haudio_out_sai.hdmatx->Init.Mode = DMA_NORMAL;
    
    /* Configure the DMA Stream with new Transfer DMA mode */
    HAL_DMA_Init(haudio_out_sai.hdmatx);      
  }
  
  /********** Playback Buffer stereo/mono mode **********/
  if(AudioOutOption & BSP_AUDIO_OUT_STEREOMODE)
  {
    /* Disable SAI peripheral to allow access to SAI internal registers */
    __HAL_SAI_DISABLE(&haudio_out_sai);
    
    /* Update the SAI audio frame slot configuration */
    haudio_out_sai.Init.MonoStereoMode = SAI_STEREOMODE;
    HAL_SAI_Init(&haudio_out_sai);
    
    /* Enable SAI peripheral to generate MCLK */
    __HAL_SAI_ENABLE(&haudio_out_sai);
  }
  else /* BSP_AUDIO_OUT_MONOMODE */
  {
    /* Disable SAI peripheral to allow access to SAI internal registers */
    __HAL_SAI_DISABLE(&haudio_out_sai);
    
    /* Update the SAI audio frame slot configuration */
    haudio_out_sai.Init.MonoStereoMode = SAI_MONOMODE;
    HAL_SAI_Init(&haudio_out_sai);
    
    /* Enable SAI peripheral to generate MCLK */
    __HAL_SAI_ENABLE(&haudio_out_sai);
  }
}

/**
  * @brief  Tx Transfer completed callbacks.
  * @param  hsai: SAI handle
  * @retval None
  */
void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai)
{
  /* Manage the remaining file size and new address offset: This function 
     should be coded by user (its prototype is already declared in stm32l476g_eval_audio.h) */
  BSP_AUDIO_OUT_TransferComplete_CallBack();
}

/**
  * @brief  Tx Half Transfer completed callbacks.
  * @param  hsai: SAI handle
  * @retval None
  */
void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
  /* Manage the remaining file size and new address offset: This function 
     should be coded by user (its prototype is already declared in stm32l476g_eval_audio.h) */
  BSP_AUDIO_OUT_HalfTransfer_CallBack();
}

/**
  * @brief  SAI error callbacks.
  * @param  hsai: SAI handle
  * @retval None
  */
void HAL_SAI_ErrorCallback(SAI_HandleTypeDef *hsai)
{
  BSP_AUDIO_OUT_Error_CallBack();
}

/**
  * @brief  Manages the DMA full Transfer complete event.
  * @retval None
  */
__weak void BSP_AUDIO_OUT_TransferComplete_CallBack(void)
{
}

/**
  * @brief  Manages the DMA Half Transfer complete event.
  * @retval None
  */
__weak void BSP_AUDIO_OUT_HalfTransfer_CallBack(void)
{
}

/**
  * @brief  Manages the DMA FIFO error event.
  * @retval None
  */
__weak void BSP_AUDIO_OUT_Error_CallBack(void)
{
}

/**
  * @}
  */

/** @addtogroup STM32L476G_EVAL_AUDIO_Private_Functions
  * @{
  */

/*******************************************************************************
                            Static Functions
*******************************************************************************/
/**
  * @brief  Initializes SAI MSP.
  * @retval None
  */
static void SAIx_MspInit(void)
{ 
  static DMA_HandleTypeDef hdma_saiTx;
  GPIO_InitTypeDef  GPIO_InitStruct;  
  SAI_HandleTypeDef *hsai = &haudio_out_sai;

  /* Enable SAI clock */
  AUDIO_SAIx_CLK_ENABLE();
  
  /* Enable GPIO clock */
  AUDIO_SAIx_MCKB_SCKB_SDB_FSB_ENABLE();
  
  /* CODEC_SAI pins configuration: FS, SCK, MCK and SD pins ------------------*/
  GPIO_InitStruct.Pin = AUDIO_SAIx_FSB_PIN | AUDIO_SAIx_SCKB_PIN | AUDIO_SAIx_SDB_PIN | AUDIO_SAIx_MCKB_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = AUDIO_SAIx_MCKB_SCKB_SDB_FSB_AF;
  HAL_GPIO_Init(AUDIO_SAIx_MCKB_SCKB_SDB_FSB_GPIO_PORT, &GPIO_InitStruct);

  /* Enable the DMA clock */
  AUDIO_SAIx_DMAx_CLK_ENABLE();
    
  if(hsai->Instance == AUDIO_SAIx)
  {
    /* Configure the hdma_saiTx handle parameters */   
    hdma_saiTx.Init.Request             = DMA_REQUEST_1;
    hdma_saiTx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_saiTx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_saiTx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_saiTx.Init.PeriphDataAlignment = AUDIO_SAIx_DMAx_PERIPH_DATA_SIZE;
    hdma_saiTx.Init.MemDataAlignment    = AUDIO_SAIx_DMAx_MEM_DATA_SIZE;
    hdma_saiTx.Init.Mode                = DMA_NORMAL;
    hdma_saiTx.Init.Priority            = DMA_PRIORITY_HIGH;
    
    hdma_saiTx.Instance = AUDIO_SAIx_DMAx_CHANNEL;
    
    /* Associate the DMA handle */
    __HAL_LINKDMA(hsai, hdmatx, hdma_saiTx);
    
    /* Deinitialize the Stream for new transfer */
    HAL_DMA_DeInit(&hdma_saiTx);
    
    /* Configure the DMA Stream */
    HAL_DMA_Init(&hdma_saiTx);      
  }
  
  /* SAI DMA IRQ Channel configuration */
  HAL_NVIC_SetPriority(AUDIO_SAIx_DMAx_IRQ, AUDIO_OUT_IRQ_PREPRIO, 0);
  HAL_NVIC_EnableIRQ(AUDIO_SAIx_DMAx_IRQ); 
}

/**
  * @brief  Initializes the Audio Codec audio interface (SAI).
  * @param  AudioFreq: Audio frequency to be configured for the SAI peripheral.
  * @note   The default SlotActive configuration is set to CODEC_AUDIOFRAME_SLOT_0123 
  *         and user can update this configuration using 
  * @retval None
  */
static void SAIx_Init(uint32_t AudioFreq)
{
  /* Initialize the haudio_out_sai Instance parameter */
  haudio_out_sai.Instance = AUDIO_SAIx;
  
  /* Disable SAI peripheral to allow access to SAI internal registers */
  __HAL_SAI_DISABLE(&haudio_out_sai);
  
  /* Configure SAI_Block_x 
  LSBFirst: Disabled 
  DataSize: 16 */
  haudio_out_sai.Init.MonoStereoMode = SAI_STEREOMODE;
  haudio_out_sai.Init.Mckdiv = SAIClockDivider(AudioFreq);
  haudio_out_sai.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_MCKDIV;
  haudio_out_sai.Init.AudioMode = SAI_MODEMASTER_TX;
  haudio_out_sai.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  haudio_out_sai.Init.Protocol = SAI_FREE_PROTOCOL;
  haudio_out_sai.Init.DataSize = SAI_DATASIZE_16;
  haudio_out_sai.Init.FirstBit = SAI_FIRSTBIT_MSB;
  haudio_out_sai.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
  haudio_out_sai.Init.Synchro = SAI_ASYNCHRONOUS;
  haudio_out_sai.Init.OutputDrive = SAI_OUTPUTDRIVE_ENABLE;
  haudio_out_sai.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_1QF;
  
  /* Configure SAI_Block_x Frame 
  Frame Length: 64
  Frame active Length: 32
  FS Definition: Start frame + Channel Side identification
  FS Polarity: FS active Low
  FS Offset: FS asserted one bit before the first bit of slot 0 */ 
  haudio_out_sai.FrameInit.FrameLength = 64; 
  haudio_out_sai.FrameInit.ActiveFrameLength = 32;
  haudio_out_sai.FrameInit.FSDefinition = SAI_FS_CHANNEL_IDENTIFICATION;
  haudio_out_sai.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  haudio_out_sai.FrameInit.FSOffset = SAI_FS_BEFOREFIRSTBIT;
  
  /* Configure SAI Block_x Slot 
  Slot First Bit Offset: 0
  Slot Size  : 16
  Slot Number: 4
  Slot Active: All slot actives */
  haudio_out_sai.SlotInit.FirstBitOffset = 0;
  haudio_out_sai.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
  haudio_out_sai.SlotInit.SlotNumber = 4; 
  haudio_out_sai.SlotInit.SlotActive = CODEC_AUDIOFRAME_SLOT_0123;

  /* Init the SAI */
  SAIx_MspInit();

  HAL_SAI_Init(&haudio_out_sai);
  
  /* Enable SAI peripheral to generate MCLK */
  __HAL_SAI_ENABLE(&haudio_out_sai);
}

/**
  * @brief  Resets the audio codec. It restores the default configuration of the 
  *         codec (this function shall be called before initializing the codec).
  * @retval None
  */
static void CODEC_Reset(void)
{
  /* Initialize the audio driver structure */
  audio_drv = &wm8994_drv; 
  
  audio_drv->Reset(AUDIO_I2C_ADDRESS);
}

/**
  * @}
  */

/** @addtogroup STM32L476G_EVAL_AUDIO_Exported_Functions
  * @{
  */
  
/**
  * @brief  Initializes wave recording.
  * @note   This function assumes that the SAI input clock (through PLL_M)
  *         is already configured and ready to be used.  
  * @param  AudioFreq: Audio frequency to be configured for the SAI peripheral. 
  * @param  BitRes: Audio frequency to be configured for the SAI peripheral.
  * @param  ChnlNbr: Audio frequency to be configured for the SAI peripheral.
  * @retval AUDIO_OK if correct communication, else wrong communication
  */
uint8_t BSP_AUDIO_IN_Init(uint32_t AudioFreq, uint32_t BitRes, uint32_t ChnlNbr)
{
  RCC_PeriphCLKInitTypeDef RCC_ExCLKInitStruct;
  uint32_t ret = AUDIO_ERROR;

  /* Store the audio record context */
  audio_in_context.Frequency     = AudioFreq;
  audio_in_context.BitResolution = BitRes;
  audio_in_context.ChannelNbr    = ChnlNbr;
  
  /* Update the PLL configuration according to the new frequency */
  if((AudioFreq == AUDIO_FREQUENCY_11K) || (AudioFreq == AUDIO_FREQUENCY_22K) || (AudioFreq == AUDIO_FREQUENCY_44K))
  {
    /* Configure PLLSAI prescalers */
    /* SAI clock config 
    PLLSAI1_VCO= 8 Mhz * PLLSAI1N = 8 * 24 = VCO_192M 
    SAI_CK_x = PLLSAI1_VCO/PLLSAI1P = 192/17 = 11.294 Mhz */  
    RCC_ExCLKInitStruct.PeriphClockSelection    = RCC_PERIPHCLK_SAI1;
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1N        = 24; 
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1P        = 17; 
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK;
    RCC_ExCLKInitStruct.Sai1ClockSelection      = RCC_SAI1CLKSOURCE_PLLSAI1;
    HAL_RCCEx_PeriphCLKConfig(&RCC_ExCLKInitStruct);
  }
  else /* AUDIO_FREQUENCY_8K, AUDIO_FREQUENCY_16K, AUDIO_FREQUENCY_48K, AUDIO_FREQUENCY_96K */
  {
    /* SAI clock config 
    PLLSAI1_VCO= 8 Mhz * PLLSAI1N = 8 * 43 = VCO_344M 
    SAI_CK_x = PLLSAI1_VCO/PLLSAI1P = 344/7 = 49.142 Mhz */  
    RCC_ExCLKInitStruct.PeriphClockSelection    = RCC_PERIPHCLK_SAI1;
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1N        = 43;
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1P        = 7;
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK;
    RCC_ExCLKInitStruct.Sai1ClockSelection      = RCC_SAI1CLKSOURCE_PLLSAI1;
    HAL_RCCEx_PeriphCLKConfig(&RCC_ExCLKInitStruct);    
  }

  /* SAI data transfer preparation:
  Prepare the Media to be used for the audio transfer from memory to SAI peripheral */
  ret = DFSDMx_Init(AudioFreq);

  /* Return AUDIO_OK when all operations are correctly done */
  return ret;
}

/**
  * @brief  DeInitializes the audio peripheral.
  * @retval None
  */
void BSP_AUDIO_IN_DeInit(void)
{
  DFSDMx_DeInit();
}

/**
  * @brief  Starts audio recording.
  * @param  pbuf: Main buffer pointer for the recorded data storing  
  * @param  size: Current size of the recorded buffer
  * @note   The Right channel is start at first with synchro on start of Left channel
  * @retval AUDIO_OK if correct communication, else wrong communication
  */
uint8_t BSP_AUDIO_IN_Record(uint16_t* pbuf, uint32_t size)
{
  uint8_t ret = AUDIO_ERROR;
  
  audio_in_context.pRecBuf = pbuf;
  audio_in_context.RecSize = size;
  /* Reset Application Buffer Trigger */
  AppBuffTrigger = 0;
  AppBuffHalf = 0;
    
  if(audio_in_context.ChannelNbr == 2)
  {
    /* Call the Media layer start function for right channel */
    if(HAL_OK != HAL_DFSDM_FilterRegularStart_DMA(&haudio_in_dfsdm_rightfilter, (int32_t*)RightRecBuff, INTERNAL_BUFF_SIZE))
    {
      return ret;
    }
  }
  
  /* Call the Media layer start function for left channel */
  if(HAL_OK != HAL_DFSDM_FilterRegularStart_DMA(&haudio_in_dfsdm_leftfilter, (int32_t*)LeftRecBuff, INTERNAL_BUFF_SIZE))
  {
    return ret;
  }

  /* Return AUDIO_OK when all operations are correctly done */
  ret = AUDIO_OK;
  
  return ret;
}

/**
  * @brief  Updates the audio frequency.
  * @param  AudioFreq: Audio frequency used to record the audio stream.
  * @note   This API should be called after the BSP_AUDIO_IN_Init() to adjust the
  *         audio frequency.
  * @retval AUDIO_OK if correct communication, else wrong communication
  */
uint8_t BSP_AUDIO_IN_SetFrequency(uint32_t AudioFreq)
{ 
  RCC_PeriphCLKInitTypeDef RCC_ExCLKInitStruct;
  
  HAL_RCCEx_GetPeriphCLKConfig(&RCC_ExCLKInitStruct);
  
  /* Update the PLL configuration according to the new frequency */
  if((AudioFreq == AUDIO_FREQUENCY_11K) || (AudioFreq == AUDIO_FREQUENCY_22K) || (AudioFreq == AUDIO_FREQUENCY_44K))
  {
    /* Configure PLLSAI prescalers */
    /* SAI clock config 
    PLLSAI1_VCO= 8 Mhz * PLLSAI1N = 8 * 24 = VCO_192M 
    SAI_CK_x = PLLSAI1_VCO/PLLSAI1P = 192/17 = 11.294 Mhz */  
    RCC_ExCLKInitStruct.PeriphClockSelection    = RCC_PERIPHCLK_SAI1;
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1N        = 24; 
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1P        = 17; 
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK;
    RCC_ExCLKInitStruct.Sai1ClockSelection      = RCC_SAI1CLKSOURCE_PLLSAI1;
    HAL_RCCEx_PeriphCLKConfig(&RCC_ExCLKInitStruct);
  }
  else /* AUDIO_FREQUENCY_8K, AUDIO_FREQUENCY_16K, AUDIO_FREQUENCY_48K, AUDIO_FREQUENCY_96K */
  {
    /* SAI clock config 
    PLLSAI1_VCO= 8 Mhz * PLLSAI1N = 8 * 43 = VCO_344M 
    SAI_CK_x = PLLSAI1_VCO/PLLSAI1P = 344/7 = 49.142 Mhz */  
    RCC_ExCLKInitStruct.PeriphClockSelection    = RCC_PERIPHCLK_SAI1;
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1N        = 43;
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1P        = 7;
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK;
    RCC_ExCLKInitStruct.Sai1ClockSelection      = RCC_SAI1CLKSOURCE_PLLSAI1;
    HAL_RCCEx_PeriphCLKConfig(&RCC_ExCLKInitStruct);    
  }

  /* De-initializes the Digital Filter for Sigma-Delta Modulators interface */
  if(AUDIO_OK != DFSDMx_DeInit())
  {
    return AUDIO_ERROR;
  }
  
  /* Initializes the Digital Filter for Sigma-Delta Modulators interface */
  if(AUDIO_OK != DFSDMx_Init(AudioFreq))
  {
    return AUDIO_ERROR;
  }
  
  return AUDIO_OK;
}

/**
  * @brief  Regular conversion complete callback. 
  * @note   In interrupt mode, user has to read conversion value in this function
            using HAL_DFSDM_FilterGetRegularValue.
  * @param  hdfsdm_filter : DFSDM filter handle.
  * @retval None
  */
void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
  uint32_t index;

  /* Stereo Channels treatment */
  if(audio_in_context.ChannelNbr == 2)
  {
    if(hdfsdm_filter == &haudio_in_dfsdm_leftfilter)
    {
      DmaLeftRecBuffCplt = 1;
    }
    else
    {
      DmaRightRecBuffCplt = 1;
    }

    if((DmaRightRecBuffCplt == 1) && (DmaLeftRecBuffCplt == 1))
    {
      if(AppBuffTrigger >= audio_in_context.RecSize)
        AppBuffTrigger = 0;

      for(index = (INTERNAL_BUFF_SIZE/2) ; index < INTERNAL_BUFF_SIZE; index++)
      {
        audio_in_context.pRecBuf[AppBuffTrigger]     = (uint16_t)(SaturaLH((LeftRecBuff[index] >> 8), -32760, 32760));
        audio_in_context.pRecBuf[AppBuffTrigger + 1] = (uint16_t)(SaturaLH((RightRecBuff[index] >> 8), -32760, 32760));
        AppBuffTrigger += 2;
      }
      DmaRightRecBuffCplt = DmaLeftRecBuffCplt = 0;
    }
  }
  else /* Mono Channel treatment */
  {
    for(index = (INTERNAL_BUFF_SIZE/2); index < INTERNAL_BUFF_SIZE; index++)
    {
      audio_in_context.pRecBuf[AppBuffTrigger]     = (uint16_t)(SaturaLH((LeftRecBuff[index] >> 8), -32760, 32760));
      AppBuffTrigger++;
    }
  }

  /* Update Trigger with Remaining Byte before callback if necessary */
  if(AppBuffTrigger >= audio_in_context.RecSize)
  {
    /* Reset Application Buffer Trigger */
    AppBuffTrigger = 0;
    AppBuffHalf = 0;
    
    /* Call the record update function to get the next buffer to fill and its size (size is ignored) */
    BSP_AUDIO_IN_TransferComplete_CallBack();
  }
  else if((AppBuffTrigger >= audio_in_context.RecSize/2))
  {
     if(AppBuffHalf == 0)
     {
        AppBuffHalf = 1;
        /* Manage the remaining file size and new address offset: This function 
           should be coded by user (its prototype is already declared in stm32l476g_eval_audio.h) */
        BSP_AUDIO_IN_HalfTransfer_CallBack();
     }
  }
}

/**
  * @brief  Half regular conversion complete callback. 
  * @param  hdfsdm_filter : DFSDM filter handle.
  * @retval None
  */
void HAL_DFSDM_FilterRegConvHalfCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
  uint32_t index;

  /* Stereo Channels treatment */
  if(audio_in_context.ChannelNbr == 2)
  {
    if(hdfsdm_filter == &haudio_in_dfsdm_leftfilter)
    {
      DmaLeftRecHalfBuffCplt = 1;
    }
    else
    {
      DmaRightRecHalfBuffCplt = 1;
    }

    if((DmaRightRecHalfBuffCplt == 1) && (DmaLeftRecHalfBuffCplt == 1))
    {
      if(AppBuffTrigger >= audio_in_context.RecSize)
        AppBuffTrigger = 0;

      for(index = 0; index < INTERNAL_BUFF_SIZE/2; index++)
      {
        audio_in_context.pRecBuf[AppBuffTrigger]     = (int16_t)(SaturaLH((LeftRecBuff[index] >> 8), -32760, 32760));
        audio_in_context.pRecBuf[AppBuffTrigger + 1] = (int16_t)(SaturaLH((RightRecBuff[index] >> 8), -32760, 32760));
        AppBuffTrigger += 2;
      }
      DmaRightRecHalfBuffCplt = DmaLeftRecHalfBuffCplt = 0;
    }
  }
  else /* Mono Channel treatment */
  {
    for(index = 0; index < (INTERNAL_BUFF_SIZE/2); index++)
    {
      audio_in_context.pRecBuf[AppBuffTrigger]     = (uint16_t)(SaturaLH((LeftRecBuff[index] >> 8), -32760, 32760));
      AppBuffTrigger++;
    }
  }

  /* Update Trigger with Remaining Byte before callback if necessary */
  if(AppBuffTrigger >= audio_in_context.RecSize)
  {
    /* Reset Application Buffer Trigger */
    AppBuffTrigger = 0;
    AppBuffHalf = 0;
    
    /* Call the record update function to get the next buffer to fill and its size (size is ignored) */
    BSP_AUDIO_IN_TransferComplete_CallBack();
  }
  else if((AppBuffTrigger >= audio_in_context.RecSize/2))
  {
     if(AppBuffHalf == 0)
     {
        AppBuffHalf = 1;
        /* Manage the remaining file size and new address offset: This function 
        should be coded by user (its prototype is already declared in stm32l476g_eval_audio.h) */
        BSP_AUDIO_IN_HalfTransfer_CallBack();
     }
  }
}

/**
  * @brief  Stops audio recording.
  * @retval AUDIO_OK if correct communication, else wrong communication
  */
uint8_t BSP_AUDIO_IN_Stop(void)
{
  uint32_t ret = AUDIO_ERROR;
  
  AppBuffTrigger = 0;
  AppBuffHalf    = 0;
  
  if(audio_in_context.ChannelNbr == 2)
  {
    /* Call the Media layer stop function for right channel */
    if(HAL_OK != HAL_DFSDM_FilterRegularStop_DMA(&haudio_in_dfsdm_rightfilter))
  {
    return ret;
  }
  }
  
  /* Call the Media layer stop function for left channel */
  if(HAL_OK != HAL_DFSDM_FilterRegularStop_DMA(&haudio_in_dfsdm_leftfilter))
  {
    return ret;
  }

  /* Return AUDIO_OK when all operations are correctly done */
  ret = AUDIO_OK;
  
  return ret;
}

/**
  * @brief  Pauses the audio file stream.
  * @retval AUDIO_OK if correct communication, else wrong communication
  */
uint8_t BSP_AUDIO_IN_Pause(void)
{    
  if(audio_in_context.ChannelNbr == 2)
  {
  /* Call the Media layer stop function */
    if(HAL_OK != HAL_DFSDM_FilterRegularStop_DMA(&haudio_in_dfsdm_rightfilter))
  {
    return AUDIO_ERROR;
  }
  }
  
  /* Call the Media layer stop function */
  if(HAL_OK != HAL_DFSDM_FilterRegularStop_DMA(&haudio_in_dfsdm_leftfilter))
  {
    return AUDIO_ERROR;
  }
  
  /* Return AUDIO_OK when all operations are correctly done */
  return AUDIO_OK;
}

/**
  * @brief  Resumes the audio file stream.
  * @note   The Right channel is start a first with synchro on start Left channel
  * @retval AUDIO_OK if correct communication, else wrong communication
  */
uint8_t BSP_AUDIO_IN_Resume(void)
{    
  uint8_t ret = AUDIO_ERROR;
  
  if(audio_in_context.ChannelNbr == 2)
  {
    /* Call the Media layer start function for right channel */
    if(HAL_OK != HAL_DFSDM_FilterRegularStart_DMA(&haudio_in_dfsdm_rightfilter, (int32_t*)RightRecBuff, (audio_in_context.RecSize/DEFAULT_AUDIO_IN_CHANNEL_NBR)))
    {
      return ret;
    }
  }
  
  /* Call the Media layer start function for left channel */
  if(HAL_OK != HAL_DFSDM_FilterRegularStart_DMA(&haudio_in_dfsdm_leftfilter, (int32_t*)LeftRecBuff, (audio_in_context.RecSize/DEFAULT_AUDIO_IN_CHANNEL_NBR)))
  {
    return ret;
  }
  
  /* Return AUDIO_OK when all operations are correctly done */
  return AUDIO_OK;
}

/**
  * @brief  Controls the audio in volume level. 
  * @param  Volume: Volume level to be set in percentage from 0% to 100% (0 for 
  *         Mute and 100 for Max volume level).
  * @retval AUDIO_OK if correct communication, else wrong communication
  */
uint8_t BSP_AUDIO_IN_SetVolume(uint8_t Volume)
{
  /* Set the Global variable AudioInVolume  */
  AudioInVolume = Volume; 
  
  /* Return AUDIO_OK when all operations are correctly done */
  return AUDIO_OK;
}

/**
  * @brief  User callback when record buffer is filled.
  * @retval None
  */
__weak void BSP_AUDIO_IN_TransferComplete_CallBack(void)
{
  /* This function should be implemented by the user application.
     It is called into this driver when the current buffer is filled
     to prepare the next buffer pointer and its size. */
}

/**
  * @brief  Manages the DMA Half Transfer complete event.
  * @retval None
  */
__weak void BSP_AUDIO_IN_HalfTransfer_CallBack(void)
{ 
  /* This function should be implemented by the user application.
     It is called into this driver when the current buffer is filled
     to prepare the next buffer pointer and its size. */
}

/**
  * @brief  Audio IN Error callback function.
  * @retval None
  */
__weak void BSP_AUDIO_IN_Error_Callback(void)
{   
  /* This function is called when an Interrupt due to transfer error on or peripheral
     error occurs. */
}

/**
  * @}
  */

/** @addtogroup STM32L476G_EVAL_AUDIO_Private_Functions
  * @{
  */

/*******************************************************************************
                            Static Functions
*******************************************************************************/
/**
  * @brief  Initializes the Digital Filter for Sigma-Delta Modulators interface (DFSDM).
  * @param  AudioFreq: Audio frequency to be used to set correctly the DFSDM peripheral.
  * @retval AUDIO_OK if correct communication, else wrong communication
  */
static uint8_t DFSDMx_Init(uint32_t AudioFreq)
{
  /*####CHANNEL 4####*/
  haudio_in_dfsdm_leftchannel.Init.OutputClock.Activation   = ENABLE;
  haudio_in_dfsdm_leftchannel.Init.OutputClock.Selection    = DFSDM_CHANNEL_OUTPUT_CLOCK_AUDIO;
  /* Set the DFSDM clock OUT audio frequency configuration */
  haudio_in_dfsdm_leftchannel.Init.OutputClock.Divider      = DFSDMClockDivider(AudioFreq);
  haudio_in_dfsdm_leftchannel.Init.Input.Multiplexer        = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  haudio_in_dfsdm_leftchannel.Init.Input.DataPacking        = DFSDM_CHANNEL_STANDARD_MODE;
  haudio_in_dfsdm_leftchannel.Init.Input.Pins               = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  /* Request to sample stable data for LEFT micro on Rising edge */
  haudio_in_dfsdm_leftchannel.Init.SerialInterface.Type     = DFSDM_CHANNEL_SPI_RISING;
  haudio_in_dfsdm_leftchannel.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  haudio_in_dfsdm_leftchannel.Init.Awd.FilterOrder          = DFSDM_CHANNEL_SINC1_ORDER;
  haudio_in_dfsdm_leftchannel.Init.Awd.Oversampling         = 10;
  haudio_in_dfsdm_leftchannel.Init.Offset                   = 0;
  haudio_in_dfsdm_leftchannel.Init.RightBitShift            = DFSDMRightBitShift(AudioFreq);

  haudio_in_dfsdm_leftchannel.Instance                      = DFSDM1_Channel4;
  if(HAL_DFSDM_ChannelGetState(&haudio_in_dfsdm_leftchannel) == HAL_DFSDM_CHANNEL_STATE_RESET)
  {
    /* Init the DFSDM Channel */
    DFSDMx_ChannelMspInit(&haudio_in_dfsdm_leftchannel);
  }

  if(HAL_OK != HAL_DFSDM_ChannelInit(&haudio_in_dfsdm_leftchannel))
  {
    return AUDIO_ERROR;
  }

  /*####CHANNEL 3####*/
  haudio_in_dfsdm_rightchannel.Init.OutputClock.Activation   = ENABLE;
  haudio_in_dfsdm_rightchannel.Init.OutputClock.Selection    = DFSDM_CHANNEL_OUTPUT_CLOCK_AUDIO;
  /* Set the DFSDM clock OUT audio frequency configuration */
  haudio_in_dfsdm_rightchannel.Init.OutputClock.Divider      = DFSDMClockDivider(AudioFreq);
  haudio_in_dfsdm_rightchannel.Init.Input.Multiplexer        = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  haudio_in_dfsdm_rightchannel.Init.Input.DataPacking        = DFSDM_CHANNEL_STANDARD_MODE;
  haudio_in_dfsdm_rightchannel.Init.Input.Pins               = DFSDM_CHANNEL_FOLLOWING_CHANNEL_PINS;
  /* Request to sample stable data for RIGHT micro on Falling edge */
  haudio_in_dfsdm_rightchannel.Init.SerialInterface.Type     = DFSDM_CHANNEL_SPI_FALLING;
  haudio_in_dfsdm_rightchannel.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  haudio_in_dfsdm_rightchannel.Init.Awd.FilterOrder          = DFSDM_CHANNEL_SINC1_ORDER;
  haudio_in_dfsdm_rightchannel.Init.Awd.Oversampling         = 10;
  haudio_in_dfsdm_rightchannel.Init.Offset                   = 0;
  haudio_in_dfsdm_rightchannel.Init.RightBitShift            = DFSDMRightBitShift(AudioFreq);

  haudio_in_dfsdm_rightchannel.Instance                      = DFSDM1_Channel3;

  if(HAL_DFSDM_ChannelGetState(&haudio_in_dfsdm_rightchannel) == HAL_DFSDM_CHANNEL_STATE_RESET)
  {
    /* Init the DFSDM Channel */
    DFSDMx_ChannelMspInit(&haudio_in_dfsdm_rightchannel);
  }

  if(HAL_OK != HAL_DFSDM_ChannelInit(&haudio_in_dfsdm_rightchannel))
  {
    return AUDIO_ERROR;
  }
  
  /*####FILTER 0####*/
  haudio_in_dfsdm_leftfilter.Init.RegularParam.Trigger         = DFSDM_FILTER_SW_TRIGGER;
  haudio_in_dfsdm_leftfilter.Init.RegularParam.FastMode        = ENABLE;
  haudio_in_dfsdm_leftfilter.Init.RegularParam.DmaMode         = ENABLE;
  haudio_in_dfsdm_leftfilter.Init.InjectedParam.Trigger        = DFSDM_FILTER_SW_TRIGGER;
  haudio_in_dfsdm_leftfilter.Init.InjectedParam.ScanMode       = DISABLE;
  haudio_in_dfsdm_leftfilter.Init.InjectedParam.DmaMode        = DISABLE;
  haudio_in_dfsdm_leftfilter.Init.InjectedParam.ExtTrigger     = DFSDM_FILTER_EXT_TRIG_TIM8_TRGO;
  haudio_in_dfsdm_leftfilter.Init.InjectedParam.ExtTriggerEdge = DFSDM_FILTER_EXT_TRIG_BOTH_EDGES;
  haudio_in_dfsdm_leftfilter.Init.FilterParam.SincOrder        = DFSDMFilterOrder(AudioFreq);
  /* Set the DFSDM Filters Oversampling to have correct sample rate */
  haudio_in_dfsdm_leftfilter.Init.FilterParam.Oversampling     = DFSDMOverSampling(AudioFreq);
  haudio_in_dfsdm_leftfilter.Init.FilterParam.IntOversampling  = 1;
  
  haudio_in_dfsdm_leftfilter.Instance                          = AUDIO_DFSDMx_LEFT_FILTER;

  if(HAL_DFSDM_FilterGetState(&haudio_in_dfsdm_leftfilter) == HAL_DFSDM_FILTER_STATE_RESET)
  {
    /* Init the DFSDM Filter */
    DFSDMx_FilterMspInit(&haudio_in_dfsdm_leftfilter);
  }

  if(HAL_OK != HAL_DFSDM_FilterInit(&haudio_in_dfsdm_leftfilter))
  {
    return AUDIO_ERROR;
  }
  
  /* Configure injected channel */
  if(HAL_OK != HAL_DFSDM_FilterConfigRegChannel(&haudio_in_dfsdm_leftfilter, DFSDM_CHANNEL_4, DFSDM_CONTINUOUS_CONV_ON))
  {
    return AUDIO_ERROR;
  }

  /*####FILTER 1####*/
  haudio_in_dfsdm_rightfilter.Init.RegularParam.Trigger         = DFSDM_FILTER_SYNC_TRIGGER;
  haudio_in_dfsdm_rightfilter.Init.RegularParam.FastMode        = ENABLE;
  haudio_in_dfsdm_rightfilter.Init.RegularParam.DmaMode         = ENABLE;
  haudio_in_dfsdm_rightfilter.Init.InjectedParam.Trigger        = DFSDM_FILTER_SW_TRIGGER;
  haudio_in_dfsdm_rightfilter.Init.InjectedParam.ScanMode       = DISABLE;
  haudio_in_dfsdm_rightfilter.Init.InjectedParam.DmaMode        = DISABLE;
  haudio_in_dfsdm_rightfilter.Init.InjectedParam.ExtTrigger     = DFSDM_FILTER_EXT_TRIG_TIM8_TRGO;
  haudio_in_dfsdm_rightfilter.Init.InjectedParam.ExtTriggerEdge = DFSDM_FILTER_EXT_TRIG_BOTH_EDGES;
  haudio_in_dfsdm_rightfilter.Init.FilterParam.SincOrder        = DFSDMFilterOrder(AudioFreq);
  /* Set the DFSDM Filters Oversampling to have correct sample rate */
  haudio_in_dfsdm_rightfilter.Init.FilterParam.Oversampling     = DFSDMOverSampling(AudioFreq);
  haudio_in_dfsdm_rightfilter.Init.FilterParam.IntOversampling  = 1;
  
  haudio_in_dfsdm_rightfilter.Instance                          = AUDIO_DFSDMx_RIGHT_FILTER;

  if(HAL_DFSDM_FilterGetState(&haudio_in_dfsdm_rightfilter) == HAL_DFSDM_FILTER_STATE_RESET)
  {
    /* Init the DFSDM Filter */
    DFSDMx_FilterMspInit(&haudio_in_dfsdm_rightfilter);
  }

  if(HAL_OK != HAL_DFSDM_FilterInit(&haudio_in_dfsdm_rightfilter))
  {
    return AUDIO_ERROR;
  }
  
  /* Configure injected channel */
  if(HAL_OK != HAL_DFSDM_FilterConfigRegChannel(&haudio_in_dfsdm_rightfilter, DFSDM_CHANNEL_3, DFSDM_CONTINUOUS_CONV_ON))
  {
    return AUDIO_ERROR;
  }
  
  return AUDIO_OK;
}

/**
  * @brief  De-initializes the Digital Filter for Sigma-Delta Modulators interface (DFSDM).
  * @retval AUDIO_OK if correct communication, else wrong communication
  */
static uint8_t DFSDMx_DeInit(void)
{
  /* De-initializes the DFSDM filters to allow access to DFSDM internal registers */
  if(HAL_OK != HAL_DFSDM_FilterDeInit(&haudio_in_dfsdm_leftfilter))
  {
    return AUDIO_ERROR;
  }

  if(HAL_OK != HAL_DFSDM_FilterDeInit(&haudio_in_dfsdm_rightfilter))
  {
    return AUDIO_ERROR;
  }

  DFSDMx_FilterMspDeInit(&haudio_in_dfsdm_leftfilter);
  DFSDMx_FilterMspDeInit(&haudio_in_dfsdm_rightfilter);

  /* De-initializes the DFSDM channels to allow access to DFSDM internal registers */
  if(HAL_OK != HAL_DFSDM_ChannelDeInit(&haudio_in_dfsdm_leftchannel))
  {
    return AUDIO_ERROR;
  }

  if(HAL_OK != HAL_DFSDM_ChannelDeInit(&haudio_in_dfsdm_rightchannel))
  {
    return AUDIO_ERROR;
  }

  DFSDMx_ChannelMspDeInit(&haudio_in_dfsdm_leftchannel);
  DFSDMx_ChannelMspDeInit(&haudio_in_dfsdm_rightchannel);

  return AUDIO_OK;
}

/**
  * @brief  Initializes the DFSDM channel MSP.
  * @param  hdfsdm_channel : DFSDM channel handle.
  * @retval None
  */
static void DFSDMx_ChannelMspInit(DFSDM_Channel_HandleTypeDef *hdfsdm_channel)
{
  GPIO_InitTypeDef  GPIO_InitStruct;  

  /* Enable DFSDM clock */
  AUDIO_DFSDMx_CLK_ENABLE();
  
  /* Enable GPIO clock */
  AUDIO_DFSDMx_CKOUT_DMIC_DATIN_GPIO_CLK_ENABLE();
  
  /* DFSDM pins configuration: DFSDM_CKOUT, DMIC_DATIN pins ------------------*/
  GPIO_InitStruct.Pin = AUDIO_DFSDMx_CKOUT_PIN | AUDIO_DFSDMx_DMIC_DATIN_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = AUDIO_DFSDMx_CKOUT_DMIC_DATIN_AF;
  HAL_GPIO_Init(AUDIO_DFSDMx_CKOUT_DMIC_DATIN_GPIO_PORT, &GPIO_InitStruct);
}

/**
  * @brief  DeInitializes the DFSDM channel MSP.
  * @param  hdfsdm_channel : DFSDM channel handle.
  * @retval None
  */
static void DFSDMx_ChannelMspDeInit(DFSDM_Channel_HandleTypeDef *hdfsdm_channel)
{
  GPIO_InitTypeDef  GPIO_InitStruct;  

  /* DFSDM pins configuration: DFSDM_CKOUT, DMIC_DATIN pins ------------------*/
  GPIO_InitStruct.Pin = AUDIO_DFSDMx_CKOUT_PIN | AUDIO_DFSDMx_DMIC_DATIN_PIN;
  HAL_GPIO_DeInit(AUDIO_DFSDMx_CKOUT_DMIC_DATIN_GPIO_PORT, GPIO_InitStruct.Pin);
}

/**
  * @brief  Initializes the DFSDM filter MSP.
  * @param  hdfsdm_filter : DFSDM filter handle.
  * @retval None
  */
static void DFSDMx_FilterMspInit(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
  DMA_HandleTypeDef *hdma_dfsdmReg;
  
  /* Enable DFSDM clock */
  AUDIO_DFSDMx_CLK_ENABLE();
  
  /* Enable the DMA clock */
  AUDIO_DFSDMx_DMAx_CLK_ENABLE();
    
  if(hdfsdm_filter->Instance == AUDIO_DFSDMx_LEFT_FILTER)
  {
    hdma_dfsdmReg = &hdma_dfsdm_left;
  }
  else /* AUDIO_DFSDMx_RIGHT_FILTER */
  {
    hdma_dfsdmReg = &hdma_dfsdm_right;
  }

  /* Configure the hdma_dfsdmReg handle parameters */   
  hdma_dfsdmReg->Init.Request             = DMA_REQUEST_0;
  hdma_dfsdmReg->Init.Direction           = DMA_PERIPH_TO_MEMORY;
  hdma_dfsdmReg->Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_dfsdmReg->Init.MemInc              = DMA_MINC_ENABLE;
  hdma_dfsdmReg->Init.PeriphDataAlignment = AUDIO_DFSDMx_DMAx_PERIPH_DATA_SIZE;
  hdma_dfsdmReg->Init.MemDataAlignment    = AUDIO_DFSDMx_DMAx_MEM_DATA_SIZE;
  hdma_dfsdmReg->Init.Mode                = DMA_CIRCULAR;
  hdma_dfsdmReg->Init.Priority            = DMA_PRIORITY_HIGH;
  
  if(hdfsdm_filter->Instance == AUDIO_DFSDMx_LEFT_FILTER)
  {
    hdma_dfsdmReg->Instance               = AUDIO_DFSDMx_DMAx_LEFT_CHANNEL;
  }
  else /* AUDIO_DFSDMx_RIGHT_FILTER */
  {
    hdma_dfsdmReg->Instance               = AUDIO_DFSDMx_DMAx_RIGHT_CHANNEL;
  }
  
  /* Associate the DMA handle */
  __HAL_LINKDMA(hdfsdm_filter, hdmaReg, *hdma_dfsdmReg);
  
  /* Reset DMA handle state */
  __HAL_DMA_RESET_HANDLE_STATE(hdma_dfsdmReg);

  /* Configure the DMA Channel */
  HAL_DMA_Init(hdma_dfsdmReg);      

  /* DMA IRQ Channel configuration */
  if(hdfsdm_filter->Instance == AUDIO_DFSDMx_LEFT_FILTER)
  {
    HAL_NVIC_SetPriority(AUDIO_DFSDMx_DMAx_LEFT_IRQ, AUDIO_OUT_IRQ_PREPRIO, 0);
    HAL_NVIC_EnableIRQ(AUDIO_DFSDMx_DMAx_LEFT_IRQ);
  }
  else /* AUDIO_DFSDMx_RIGHT_FILTER */
  {
    HAL_NVIC_SetPriority(AUDIO_DFSDMx_DMAx_RIGHT_IRQ, AUDIO_OUT_IRQ_PREPRIO, 0);
    HAL_NVIC_EnableIRQ(AUDIO_DFSDMx_DMAx_RIGHT_IRQ);
  }
}

/**
  * @brief  DeInitializes the DFSDM filter MSP.
  * @param  hdfsdm_filter : DFSDM filter handle.
  * @retval None
  */
static void DFSDMx_FilterMspDeInit(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
  DMA_HandleTypeDef *hdma_dfsdmReg;
  
  if(hdfsdm_filter->Instance == AUDIO_DFSDMx_LEFT_FILTER)
  {
    hdma_dfsdmReg = &hdma_dfsdm_left;
  }
  else /* AUDIO_DFSDMx_RIGHT_FILTER */
  {
    hdma_dfsdmReg = &hdma_dfsdm_right;
  }

  /* Configure the DMA Channel */
  HAL_DMA_DeInit(hdma_dfsdmReg);
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

/**
  * @}
  */


