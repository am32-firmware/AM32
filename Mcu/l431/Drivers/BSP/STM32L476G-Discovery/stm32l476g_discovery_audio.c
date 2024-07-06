/**
  ******************************************************************************
  * @file    stm32l476g_discovery_audio.c
  * @author  MCD Application Team
  * @brief   This file provides a set of functions needed to manage the
  *          Audio driver for the STM32L476G-Discovery board.
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
   + This driver supports STM32L4xx devices on STM32L476G-Discovery (MB1184) Discovery boards.
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
      the audio codec has failed.
      - OUTPUT_DEVICE_SPEAKER  : only speaker will be set as output for the audio stream.
      - OUTPUT_DEVICE_HEADPHONE: only headphones will be set as output for the audio stream.
      - OUTPUT_DEVICE_BOTH     : both Speaker and Headphone are used as outputs for the audio stream
                                 at the same time.

   + Call the function BSP_AUDIO_OUT_RegisterCallbacks to register user callbacks
     required to manage audio data streaming towards the audio codec (ErrorCallback(),
     HalfTransfer_CallBack() and TransferComplete_CallBack()).

   + Call the function BSP_AUDIO_OUT_Play() to start audio playback (for the first time).
   + Call the function BSP_AUDIO_OUT_Pause() to pause audio playabck
   + Call the function BSP_AUDIO_OUT_Resume() to resume audio playback.
       Note. After calling BSP_AUDIO_OUT_Pause() function for pause, only BSP_AUDIO_OUT_Resume() should be called
          for resume (it is not allowed to call BSP_AUDIO_OUT_Play() in this case).
       Note. This function should be called only when the audio file is played or paused (not stopped).
   + Call the function BSP_AUDIO_OUT_Stop() to stop audio playback.
   + To modify the volume level, the sampling frequency, the device output mode,
      the mute status or the audio configuration or the stop, use the functions: BSP_AUDIO_OUT_SetVolume(),
      AUDIO_OUT_SetFrequency(), BSP_AUDIO_OUT_SetOutputMode(), BSP_AUDIO_OUT_SetMute()and
      BSP_AUDIO_OUT_ChangeAudioConfig().

Driver architecture:
--------------------
   + This driver provides the audio layer high level API: it consists in functions
     exported in the stm32l476g_discovery_audio.h file (e.g. BSP_AUDIO_OUT_Init(),
     BSP_AUDIO_OUT_Play(), ...).
   + This driver also includes the Media Access Layer (MAL): it consists in
     functions allowing to access setup the audio devices. These functions
     are  included as local functions into the stm32l476g_discovery_audio.c file
     (e.g. AUDIO_SAIx_Init()).

Known Limitations:
------------------
   1- Communication with the audio codec (through I2C) may be corrupted if it is interrupted by some
      user interrupt routines (in this case, interrupts could be disabled just before the start of
      communication then re-enabled when it is over). Note that this communication is only done at
      the configuration phase (BSP_AUDIO_OUT_Init() or BSP_AUDIO_OUT_Stop()) and when Volume control modification is
      performed (BSP_AUDIO_OUT_SetVolume() or BSP_AUDIO_OUT_SetMute()or BSP_AUDIO_OUT_SetOutputMode()).
      When the audio data is played, no communication is required with the audio codec.
   2- Parsing of audio file is not implemented (in order to determine audio file properties: Mono/Stereo, Data size,
      File size, Audio Frequency, Audio Data header size ...). The configuration is fixed for the given audio file.
   3- Supports only 16-bits audio data size.

b) RECORD A FILE:
================
   + Call the function BSP_AUDIO_IN_Init(
                                    AudioFreq: Audio frequency in Hz (8000, 16000, 22500, 32000 ...)
                                    )
      This function configures all the hardware required for the audio application (DFSDM,
      GPIOs, DMA and interrupt if needed). This function returns AUDIO_OK if the
      configuration completes successfully.

   + Call the function BSP_AUDIO_IN_RegisterCallbacks to register user callbacks
     used to stream audio data toward the record buffer (ErrorCallback(),
     HalfTransfer_CallBack() and TransferComplete_CallBack()).

   + Call the function BSP_AUDIO_IN_Record(
                            pbuf Main buffer pointer for the recorded data storing
                            size Current size of the recorded buffer
                            )
      to start recording from the microphone.

   + Call the function AUDIO_IN_STOP() to stop recording
==============================================================================*/

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "stm32l476g_discovery_audio.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L476G_DISCOVERY
  * @{
  */

/** @defgroup STM32L476G_DISCOVERY_AUDIO STM32L476G-DISCOVERY AUDIO
  * @brief This file includes the low layer driver for cs43l22 Audio Codec
  *        available on STM32L476G-Discovery board(MB1184).
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/** @defgroup STM32L476G_DISCOVERY_AUDIO_Private_Types Private Types
  * @{
  */
typedef struct
{
  AUDIO_DrvTypeDef     *AudioDrv;            /* Audio codec driver */
  Audio_CallbackTypeDef CbError;            /* pointer to the callback function invoked when ... */
  Audio_CallbackTypeDef CbHalfTransfer;     /* pointer to the callback function invoked when ... */
  Audio_CallbackTypeDef CbTransferComplete; /* pointer to the callback function invoked when ... */
} AUDIO_OUT_TypeDef;

typedef struct
{
  DFSDM_Channel_HandleTypeDef hDfsdmLeftChannel;  /* DFSDM channel handle used for left channel */
  DMA_HandleTypeDef           hDmaDfsdmLeft;      /* DMA handle used for DFSDM regular conversions on left channel */
  int32_t                    *LeftRecBuff;        /* Buffers for left samples */
  uint32_t                Frequency;        /* Record Frequency */
  uint32_t                BitResolution;    /* Record bit resolution */
  uint32_t                ChannelNbr;       /* Record Channel Number */
  uint16_t               *pRecBuf;          /* Pointer to record user buffer */
  uint32_t                RecSize;          /* Size to record in mono, double size to record in stereo */
  Audio_CallbackTypeDef       CbError;            /* pointer to the callback function invoked when a DMA transfer fails */
  Audio_CallbackTypeDef       CbHalfTransfer;     /* pointer to the callback function invoked when half of the DMA transfer is completed */
  Audio_CallbackTypeDef       CbTransferComplete; /* pointer to the callback function invoked when the DMA transfer is completed */
} AUDIO_IN_TypeDef;

/**
  * @}
  */

/* Private defines ------------------------------------------------------------*/
/** @defgroup STM32L476G_DISCOVERY_AUDIO_Private_Constants Private Constants
  * @{
  */
/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/** @defgroup STM32L476G_DISCOVERY_AUDIO_Private_Macros Private Macros
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
      : (__FREQUENCY__ == AUDIO_FREQUENCY_44K) ? DFSDM_FILTER_SINC3_ORDER  \
      : (__FREQUENCY__ == AUDIO_FREQUENCY_48K) ? DFSDM_FILTER_SINC4_ORDER : DFSDM_FILTER_SINC5_ORDER  \
 
#define DFSDMRightBitShift(__FREQUENCY__) \
        (__FREQUENCY__ == AUDIO_FREQUENCY_8K)  ? 6 \
      : (__FREQUENCY__ == AUDIO_FREQUENCY_11K) ? 6 \
      : (__FREQUENCY__ == AUDIO_FREQUENCY_16K) ? 3 \
      : (__FREQUENCY__ == AUDIO_FREQUENCY_22K) ? 3 \
      : (__FREQUENCY__ == AUDIO_FREQUENCY_32K) ? 6 \
      : (__FREQUENCY__ == AUDIO_FREQUENCY_44K) ? 0 \
      : (__FREQUENCY__ == AUDIO_FREQUENCY_48K) ? 2 : 0  \
 
/* Saturate the record PCM sample */
#define SaturaLH(N, L, H) (((N)<(L))?(L):(((N)>(H))?(H):(N)))

/**
  * @}
  */

/* Private variables ---------------------------------------------------------*/
/** @defgroup STM32L476G_DISCOVERY_AUDIO_Private_Variables Private Variables
  * @{
  */
/* Audio output context information */
static AUDIO_OUT_TypeDef hAudioOut;

/* Audio input context information */
static AUDIO_IN_TypeDef hAudioIn;

/* SAI DMA handle */
static DMA_HandleTypeDef hDmaSai;
/**
  * @}
  */

/* Exported variables ---------------------------------------------------------*/
/** @defgroup STM32L476G_DISCOVERY_AUDIO_Exported_Variables Exported Variables
  * @{
  */
/* SAIx handle */
SAI_HandleTypeDef               BSP_AUDIO_hSai;

/* DFSDM filter handle */
DFSDM_Filter_HandleTypeDef      BSP_AUDIO_hDfsdmLeftFilter;
/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/
/** @defgroup STM32L476G_DISCOVERY_AUDIO_Private_Functions Private Functions
  * @{
  */
static void    AUDIO_CODEC_Reset(void);
static uint8_t AUDIO_SAIx_Init(uint32_t AudioFreq);
static uint8_t AUDIO_SAIx_DeInit(void);
static uint8_t AUDIO_DFSDMx_Init(uint32_t AudioFreq);
static uint8_t AUDIO_DFSDMx_DeInit(void);
static uint8_t AUDIO_SAIPLLConfig(uint32_t AudioFreq);
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup STM32L476G_DISCOVERY_AUDIO_Exported_Functions
  * @{
  */

/**
  * @brief  Configures the audio codec related peripherals.
  * @param  OutputDevice: OUTPUT_DEVICE_SPEAKER, OUTPUT_DEVICE_HEADPHONE,
  *                       or OUTPUT_DEVICE_BOTH.
  * @param  Volume: Initial volume level (from 0 (Mute) to 100 (Max))
  * @param  AudioFreq: Audio frequency used to play the audio stream.ion.
  * @retval BSP AUDIO status
  * @note   The SAI PLL input clock must be configure in the user application.
  *         The SAI PLL configuration done within this function assumes that
  *         the SAI PLL input clock runs at 8 MHz.
  */
uint8_t BSP_AUDIO_OUT_Init(uint16_t OutputDevice,
                           uint8_t Volume,
                           uint32_t AudioFreq)
{
  /* Initialize the audio output context */
  hAudioOut.AudioDrv           = &cs43l22_drv;
  hAudioOut.CbError            = (Audio_CallbackTypeDef)NULL;
  hAudioOut.CbHalfTransfer     = (Audio_CallbackTypeDef)NULL;
  hAudioOut.CbTransferComplete = (Audio_CallbackTypeDef)NULL;

  /* Configure the SAI PLL according to the requested audio frequency */
  if (AUDIO_SAIPLLConfig(AudioFreq) != AUDIO_OK)
  {
    return AUDIO_ERROR;
  }

  /* SAI data transfer preparation: prepare the Media to be used for the audio
     transfer from memory to SAI peripheral. */
  if (AUDIO_SAIx_Init(AudioFreq) != AUDIO_OK)
  {
    return AUDIO_ERROR;
  }

  /* Retrieve audio codec identifier */
  if (cs43l22_drv.ReadID(AUDIO_I2C_ADDRESS) != CS43L22_ID)
  {
    return AUDIO_ERROR;
  }

  /* Reset the audio codec Registers */
  AUDIO_CODEC_Reset();

  /* Initialize the audio codec internal registers */
  if (hAudioOut.AudioDrv->Init(AUDIO_I2C_ADDRESS,
                               OutputDevice,
                               Volume,
                               AudioFreq) != 0)
  {
    return AUDIO_ERROR;
  }

  /* Set the requested volume */
  BSP_AUDIO_OUT_SetVolume(Volume);

  return AUDIO_OK;
}

/**
  * @brief  De-Initializes audio codec related peripherals
  * @retval BSP AUDIO status

  */
uint8_t BSP_AUDIO_OUT_DeInit(void)
{
  /* De-initializes the Audio Codec audio interface */
  if (AUDIO_SAIx_DeInit() != AUDIO_OK)
  {
    return AUDIO_ERROR;
  }

  /* DeInit Audio component interface */
  hAudioOut.AudioDrv->DeInit();

  return AUDIO_OK;
}

/**
  * @brief  Starts playing audio stream from a data buffer for a determined size.
  * @param  pData: pointer on PCM samples buffer
  * @param  Size: Number of audio data BYTES.
  * @retval BSP AUDIO status
  */
uint8_t BSP_AUDIO_OUT_Play(uint16_t *pData, uint32_t Size)
{
  /* Initiate a DMA transfer of PCM samples towards the serial audio interface */
  if (HAL_SAI_Transmit_DMA(&BSP_AUDIO_hSai, (uint8_t *)pData, DMA_MAX(Size)) != HAL_OK)
  {
    return AUDIO_ERROR;
  }

  /* Call the audio Codec Play function */
  if (hAudioOut.AudioDrv->Play(AUDIO_I2C_ADDRESS, pData, Size) != 0)
  {
    return AUDIO_ERROR;
  }

  return AUDIO_OK;
}

/**
  * @brief  Sends n-Bytes on the SAI interface.
  * @param  pData: pointer on PCM samples buffer
  * @param  Size: number of data to be written
  * @retval BSP AUDIO status
  */
uint8_t BSP_AUDIO_OUT_ChangeBuffer(uint16_t *pData, uint16_t Size)
{
  /* Initiate a DMA transfer of PCM samples towards the serial audio interface */
  if (HAL_SAI_Transmit_DMA(&BSP_AUDIO_hSai, (uint8_t *)pData, Size) != HAL_OK)
  {
    return AUDIO_ERROR;
  }

  return AUDIO_OK;
}

/**
  * @brief  This function Pauses the audio file stream. In case
  *         of using DMA, the DMA Pause feature is used.
  * @note When calling BSP_AUDIO_OUT_Pause() function for pause, only
  *       BSP_AUDIO_OUT_Resume() function should be called for resume
  *       (use of BSP_AUDIO_OUT_Play() function for resume could lead
  *       to unexpected behavior).
  * @retval BSP AUDIO status
  */
uint8_t BSP_AUDIO_OUT_Pause(void)
{
  /* Call the Audio Codec Pause function */
  if (hAudioOut.AudioDrv->Pause(AUDIO_I2C_ADDRESS) != 0)
  {
    return AUDIO_ERROR;
  }

  /* Pause DMA transfer of PCM samples towards the serial audio interface */
  if (HAL_SAI_DMAPause(&BSP_AUDIO_hSai) != HAL_OK)
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
  * @retval BSP AUDIO status
  */
uint8_t BSP_AUDIO_OUT_Resume(void)
{
  /* Call the Audio Codec Resume function */
  if (hAudioOut.AudioDrv->Resume(AUDIO_I2C_ADDRESS) != 0)
  {
    return AUDIO_ERROR;
  }

  /* Resume DMA transfer of PCM samples towards the serial audio interface */
  if (HAL_SAI_DMAResume(&BSP_AUDIO_hSai) != HAL_OK)
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
  * @retval BSP AUDIO status
  */
uint8_t BSP_AUDIO_OUT_Stop(uint32_t Option)
{
  /* Call Audio Codec Stop function */
  if (hAudioOut.AudioDrv->Stop(AUDIO_I2C_ADDRESS, Option) != 0)
  {
    return AUDIO_ERROR;
  }

  if (Option == CODEC_PDWN_HW)
  {
    /* Wait at least 100us */
    HAL_Delay(1);
  }

  /* Stop DMA transfer of PCM samples towards the serial audio interface */
  if (HAL_SAI_DMAStop(&BSP_AUDIO_hSai) != HAL_OK)
  {
    return AUDIO_ERROR;
  }

  return AUDIO_OK;
}

/**
  * @brief  Controls the current audio volume level.
  * @param  Volume: Volume level to be set in percentage from 0% to 100% (0 for
  *         Mute and 100 for Max volume level).
  * @retval BSP AUDIO status
  */
uint8_t BSP_AUDIO_OUT_SetVolume(uint8_t Volume)
{
  /* Call the codec volume control function with converted volume value */
  if (hAudioOut.AudioDrv->SetVolume(AUDIO_I2C_ADDRESS, Volume) != 0)
  {
    return AUDIO_ERROR;
  }

  return AUDIO_OK;
}

/**
  * @brief  Enables or disables the MUTE mode by software
  * @param  Cmd: Could be AUDIO_MUTE_ON to mute sound or AUDIO_MUTE_OFF to
  *         unmute the codec and restore previous volume level.
  * @retval BSP AUDIO status
  */
uint8_t BSP_AUDIO_OUT_SetMute(uint32_t Cmd)
{
  /* Call the Codec Mute function */
  if (hAudioOut.AudioDrv->SetMute(AUDIO_I2C_ADDRESS, Cmd) != 0)
  {
    return AUDIO_ERROR;
  }

  return AUDIO_OK;
}

/**
  * @brief  Switch dynamically (while audio file is being played) the output
  *          target (speaker or headphone).
  * @param  Output: The audio output target: OUTPUT_DEVICE_SPEAKER,
  *         OUTPUT_DEVICE_HEADPHONE or OUTPUT_DEVICE_BOTH
  * @retval BSP AUDIO status
  */
uint8_t BSP_AUDIO_OUT_SetOutputMode(uint8_t Output)
{
  /* Call the Codec output device function */
  if (hAudioOut.AudioDrv->SetOutputMode(AUDIO_I2C_ADDRESS, Output) != 0)
  {
    return AUDIO_ERROR;
  }

  return AUDIO_OK;
}

/**
  * @brief  Updates the audio frequency.
  * @param  AudioFreq: Audio frequency used to play the audio stream.
  * @note   The SAI PLL input clock must be configure in the user application.
  *         The SAI PLL configuration done within this function assumes that
  *         the SAI PLL input clock runs at 8 MHz.
  * @retval BSP AUDIO status
  */
uint8_t BSP_AUDIO_OUT_SetFrequency(uint32_t AudioFreq)
{
  /* Configure the SAI PLL according to the requested audio frequency */
  if (AUDIO_SAIPLLConfig(AudioFreq) != AUDIO_OK)
  {
    return AUDIO_ERROR;
  }

  /* Disable SAI peripheral to allow access to SAI internal registers */
  __HAL_SAI_DISABLE(&BSP_AUDIO_hSai);

  /* Update the SAI audio frequency configuration */
  BSP_AUDIO_hSai.Init.Mckdiv = SAIClockDivider(AudioFreq);
  HAL_SAI_Init(&BSP_AUDIO_hSai);

  /* Enable SAI peripheral to generate MCLK */
  __HAL_SAI_ENABLE(&BSP_AUDIO_hSai);

  return AUDIO_OK;
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
  if (AudioOutOption & BSP_AUDIO_OUT_CIRCULARMODE)
  {
    /* Deinitialize the Stream to update DMA mode */
    HAL_DMA_DeInit(BSP_AUDIO_hSai.hdmatx);

    /* Update the SAI audio Transfer DMA mode */
    BSP_AUDIO_hSai.hdmatx->Init.Mode = DMA_CIRCULAR;

    /* Configure the DMA Stream with new Transfer DMA mode */
    HAL_DMA_Init(BSP_AUDIO_hSai.hdmatx);
  }
  else /* BSP_AUDIO_OUT_NORMALMODE */
  {
    /* Deinitialize the Stream to update DMA mode */
    HAL_DMA_DeInit(BSP_AUDIO_hSai.hdmatx);

    /* Update the SAI audio Transfer DMA mode */
    BSP_AUDIO_hSai.hdmatx->Init.Mode = DMA_NORMAL;

    /* Configure the DMA Stream with new Transfer DMA mode */
    HAL_DMA_Init(BSP_AUDIO_hSai.hdmatx);
  }

  /********** Playback Buffer stereo/mono mode **********/
  if (AudioOutOption & BSP_AUDIO_OUT_STEREOMODE)
  {
    /* Disable SAI peripheral to allow access to SAI internal registers */
    __HAL_SAI_DISABLE(&BSP_AUDIO_hSai);

    /* Update the SAI audio frame slot configuration */
    BSP_AUDIO_hSai.Init.MonoStereoMode = SAI_STEREOMODE;
    HAL_SAI_Init(&BSP_AUDIO_hSai);

    /* Enable SAI peripheral to generate MCLK */
    __HAL_SAI_ENABLE(&BSP_AUDIO_hSai);
  }
  else /* BSP_AUDIO_OUT_MONOMODE */
  {
    /* Disable SAI peripheral to allow access to SAI internal registers */
    __HAL_SAI_DISABLE(&BSP_AUDIO_hSai);

    /* Update the SAI audio frame slot configuration */
    BSP_AUDIO_hSai.Init.MonoStereoMode = SAI_MONOMODE;
    HAL_SAI_Init(&BSP_AUDIO_hSai);

    /* Enable SAI peripheral to generate MCLK */
    __HAL_SAI_ENABLE(&BSP_AUDIO_hSai);
  }
}

/**
  * @brief  register user callback functions
  * @param  ErrorCallback: pointer to the error callback function
  * @param  HalfTransferCallback: pointer to the half transfer callback function
  * @param  TransferCompleteCallback: pointer to the transfer complete callback function
  * @retval None
  */
void BSP_AUDIO_OUT_RegisterCallbacks(Audio_CallbackTypeDef ErrorCallback,
                                     Audio_CallbackTypeDef HalfTransferCallback,
                                     Audio_CallbackTypeDef TransferCompleteCallback)
{
  hAudioOut.CbError            = ErrorCallback;
  hAudioOut.CbHalfTransfer     = HalfTransferCallback;
  hAudioOut.CbTransferComplete = TransferCompleteCallback;
}

/**
  * @brief  Tx Transfer completed callbacks.
  * @param  hsai: SAI handle
  * @retval None
  */
void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai)
{
  /* Invoke the registered 'TransferComplete' function (if any) */
  if (hAudioOut.CbTransferComplete != (Audio_CallbackTypeDef)NULL)
  {
    hAudioOut.CbTransferComplete();
  }
}

/**
  * @brief  Tx Half Transfer completed callbacks.
  * @param  hsai: SAI handle
  * @retval None
  */
void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
  /* Invoke the registered 'HalfTransfer' callback function (if any) */
  if (hAudioOut.CbHalfTransfer != (Audio_CallbackTypeDef)NULL)
  {
    hAudioOut.CbHalfTransfer();
  }
}

/**
  * @brief  SAI error callbacks.
  * @param  hsai: SAI handle
  * @retval None
  */
void HAL_SAI_ErrorCallback(SAI_HandleTypeDef *hsai)
{
  /* Invoke the registered 'ErrorCallback' callback function (if any) */
  if (hAudioOut.CbError != (Audio_CallbackTypeDef)NULL)
  {
    hAudioOut.CbError();
  }
}

/**
  * @}
  */

/** @addtogroup STM32L476G_EVAL_AUDIO_Exported_Functions
  * @{
  */

/**
  * @brief  Initializes microphone related peripherals.
  * @note   This function assumes that the SAI input clock (through PLL_M)
  *         is already configured and ready to be used.
  * @param  AudioFreq: Audio frequency to be configured for the SAI peripheral.
  * @param  BitRes: Audio frequency to be configured for the SAI peripheral.
  * @param  ChnlNbr: Audio frequency to be configured for the SAI peripheral.
  * @retval BSP AUDIO status
  */
uint8_t BSP_AUDIO_IN_Init(uint32_t AudioFreq, uint32_t BitRes, uint32_t ChnlNbr)
{
  /* Update the audio input context */
  hAudioIn.Frequency          = AudioFreq;
  hAudioIn.BitResolution      = BitRes;
  hAudioIn.ChannelNbr         = ChnlNbr;
  hAudioIn.CbError            = (Audio_CallbackTypeDef)NULL;
  hAudioIn.CbHalfTransfer     = (Audio_CallbackTypeDef)NULL;
  hAudioIn.CbTransferComplete = (Audio_CallbackTypeDef)NULL;

  /* Configure the SAI PLL according to the requested audio frequency */
  if (AUDIO_SAIPLLConfig(AudioFreq) != AUDIO_OK)
  {
    return AUDIO_ERROR;
  }

  /* Initializes the Digital Filter for Sigma-Delta Modulators interface */
  if (AUDIO_DFSDMx_Init(AudioFreq) != AUDIO_OK)
  {
    return AUDIO_ERROR;
  }

  return AUDIO_OK;
}

/**
  * @brief  De-Initializes microphone related peripherals.
  * @retval BSP AUDIO status

  */
uint8_t BSP_AUDIO_IN_DeInit(void)
{
  /* De-initializes the Digital Filter for Sigma-Delta Modulators interface */
  if (AUDIO_DFSDMx_DeInit() != AUDIO_OK)
  {
    return AUDIO_ERROR;
  }

  /* Reset the audio input context */
  memset(&hAudioIn, 0, sizeof(hAudioIn));

  return AUDIO_OK;
}

/**
  * @brief  Starts audio recording.
  * @param  pbuf: Main buffer pointer for the recorded data storing
  * @param  size: Current size of the recorded buffer
  * @note   The Right channel is start at first with synchro on start of Left channel
  * @retval BSP AUDIO status
  */
uint8_t BSP_AUDIO_IN_Record(uint16_t *pbuf, uint32_t size)
{
  hAudioIn.pRecBuf = pbuf;
  hAudioIn.RecSize = size;

  /* Allocate hAudioIn.LeftRecBuff buffer */
#if defined(BSP_AUDIO_USE_RTOS)
  hAudioIn.LeftRecBuff  = (int32_t *)k_malloc(size * sizeof(int32_t));
#else
  hAudioIn.LeftRecBuff  = (int32_t *)malloc(size * sizeof(int32_t));
#endif
  if (hAudioIn.LeftRecBuff == NULL)
  {
    return AUDIO_ERROR;
  }

  /* Call the Media layer start function for left channel */
  if (HAL_DFSDM_FilterRegularStart_DMA(&BSP_AUDIO_hDfsdmLeftFilter,
                                       (int32_t *)hAudioIn.LeftRecBuff,
                                       (hAudioIn.RecSize / DEFAULT_AUDIO_IN_CHANNEL_NBR)) != HAL_OK)
  {
    return AUDIO_ERROR;
  }

  return AUDIO_OK;
}

/**
  * @brief  Updates the audio frequency.
  * @param  AudioFreq: Audio frequency used to record the audio stream.
  * @note   This API should be called after the BSP_AUDIO_IN_Init() to adjust the
  *         audio frequency.
  * @retval BSP AUDIO status
  */
uint8_t BSP_AUDIO_IN_SetFrequency(uint32_t AudioFreq)
{
  /* Configure the SAI PLL according to the requested audio frequency */
  if (AUDIO_SAIPLLConfig(AudioFreq) != AUDIO_OK)
  {
    return AUDIO_ERROR;
  }

  /* De-initializes the Digital Filter for Sigma-Delta Modulators interface */
  if (AUDIO_DFSDMx_DeInit() != AUDIO_OK)
  {
    return AUDIO_ERROR;
  }

  /* Initializes the Digital Filter for Sigma-Delta Modulators interface */
  if (AUDIO_DFSDMx_Init(AudioFreq) != AUDIO_OK)
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
  uint32_t recbufsize = (hAudioIn.RecSize / DEFAULT_AUDIO_IN_CHANNEL_NBR);

  for (index = (recbufsize / 2); index < recbufsize; index++)
  {
    hAudioIn.pRecBuf[index] = (uint16_t)(SaturaLH((hAudioIn.LeftRecBuff[index] >> 8), -32768, 32767));
  }

  /* Invoke the registered 'TransferComplete' function (if any) */
  if (hAudioIn.CbTransferComplete != (Audio_CallbackTypeDef)NULL)
  {
    hAudioIn.CbTransferComplete();
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
  uint32_t recbufsize = (hAudioIn.RecSize / DEFAULT_AUDIO_IN_CHANNEL_NBR);


  for (index = 0; index < (recbufsize / 2); index++)
  {
    hAudioIn.pRecBuf[index] = (uint16_t)(SaturaLH((hAudioIn.LeftRecBuff[index] >> 8), -32768, 32767));
  }

  /* Invoke the registered 'HalfTransfer' callback function (if any) */
  if (hAudioIn.CbHalfTransfer != (Audio_CallbackTypeDef)NULL)
  {
    hAudioIn.CbHalfTransfer();
  }
}

/**
  * @brief  Error callback.
  * @param  hdfsdm_filter : DFSDM filter handle.
  * @retval None
  */
void HAL_DFSDM_FilterErrorCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
  /* Invoke the registered 'ErrorCallback' callback function (if any) */
  if (hAudioIn.CbError != (Audio_CallbackTypeDef)NULL)
  {
    hAudioIn.CbError();
  }
}

/**
  * @brief  Stops audio recording.
  * @retval BSP AUDIO status
  */
uint8_t BSP_AUDIO_IN_Stop(void)
{
  /* Call the Media layer stop function for left channel */
  if (HAL_DFSDM_FilterRegularStop_DMA(&BSP_AUDIO_hDfsdmLeftFilter) != HAL_OK)
  {
    return AUDIO_ERROR;
  }

  /* Free hAudioIn.LeftRecBuff buffer */
#if defined(BSP_AUDIO_USE_RTOS)
  k_free((void *)hAudioIn.LeftRecBuff);
#else
  free((void *)hAudioIn.LeftRecBuff);
#endif

  return AUDIO_OK;
}

/**
  * @brief  Pauses the audio file stream.
  * @retval BSP AUDIO status
  */
uint8_t BSP_AUDIO_IN_Pause(void)
{
  /* Call the Media layer stop function */
  if (HAL_DFSDM_FilterRegularStop_DMA(&BSP_AUDIO_hDfsdmLeftFilter) != HAL_OK)
  {
    return AUDIO_ERROR;
  }

  return AUDIO_OK;
}

/**
  * @brief  Resumes the audio file stream.
  * @retval BSP AUDIO status
  */
uint8_t BSP_AUDIO_IN_Resume(void)
{
  /* Call the Media layer start function for left channel */
  if (HAL_DFSDM_FilterRegularStart_DMA(&BSP_AUDIO_hDfsdmLeftFilter,
                                       (int32_t *)hAudioIn.LeftRecBuff,
                                       (hAudioIn.RecSize / DEFAULT_AUDIO_IN_CHANNEL_NBR)) != HAL_OK)
  {
    return AUDIO_ERROR;
  }

  return AUDIO_OK;
}

/**
  * @brief  register user callback functions
  * @param  ErrorCallback: pointer to the error callback function
  * @param  HalfTransferCallback: pointer to the half transfer callback function
  * @param  TransferCompleteCallback: pointer to the transfer complete callback function
  * @retval None
  */
void BSP_AUDIO_IN_RegisterCallbacks(Audio_CallbackTypeDef ErrorCallback,
                                    Audio_CallbackTypeDef HalfTransferCallback,
                                    Audio_CallbackTypeDef TransferCompleteCallback)
{
  hAudioIn.CbError            = ErrorCallback;
  hAudioIn.CbHalfTransfer     = HalfTransferCallback;
  hAudioIn.CbTransferComplete = TransferCompleteCallback;
}
/**
  * @}
  */

/* private functions --------------------------------------------------------*/
/** @addtogroup STM32L476G_DISCOVERY_AUDIO_Private_Functions
  * @{
  */
/**
  * @brief  Initializes the Audio Codec audio interface (SAI).
  * @param  AudioFreq: Audio frequency to be configured for the SAI peripheral.
  * @note   The default SlotActive configuration is set to CODEC_AUDIOFRAME_SLOT_0123
  *         and user can update this configuration using
  * @retval BSP AUDIO status
  */
static uint8_t AUDIO_SAIx_Init(uint32_t AudioFreq)
{
  /* Disable SAI peripheral to allow access to SAI internal registers */
  __HAL_SAI_DISABLE(&BSP_AUDIO_hSai);

  /* Initialize the BSP_AUDIO_hSai Instance parameter */
  BSP_AUDIO_hSai.Instance = AUDIO_SAIx;

  /* Configure SAI_Block_x
  LSBFirst: Disabled
  DataSize: 16 */
  BSP_AUDIO_hSai.Init.AudioMode      = SAI_MODEMASTER_TX;
  BSP_AUDIO_hSai.Init.Synchro        = SAI_ASYNCHRONOUS;
  BSP_AUDIO_hSai.Init.SynchroExt     = SAI_SYNCEXT_DISABLE;
  BSP_AUDIO_hSai.Init.OutputDrive    = SAI_OUTPUTDRIVE_ENABLE;
  BSP_AUDIO_hSai.Init.NoDivider      = SAI_MASTERDIVIDER_ENABLE;
  BSP_AUDIO_hSai.Init.FIFOThreshold  = SAI_FIFOTHRESHOLD_1QF;
  BSP_AUDIO_hSai.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_MCKDIV;
  BSP_AUDIO_hSai.Init.Mckdiv         = SAIClockDivider(AudioFreq);
  BSP_AUDIO_hSai.Init.MonoStereoMode = SAI_STEREOMODE;
  BSP_AUDIO_hSai.Init.CompandingMode = SAI_NOCOMPANDING;
  BSP_AUDIO_hSai.Init.TriState       = SAI_OUTPUT_NOTRELEASED;
  BSP_AUDIO_hSai.Init.Protocol       = SAI_FREE_PROTOCOL;
  BSP_AUDIO_hSai.Init.DataSize       = SAI_DATASIZE_16;
  BSP_AUDIO_hSai.Init.FirstBit       = SAI_FIRSTBIT_MSB;
  BSP_AUDIO_hSai.Init.ClockStrobing  = SAI_CLOCKSTROBING_FALLINGEDGE;

  /* Configure SAI_Block_x Frame
  Frame Length: 32
  Frame active Length: 16
  FS Definition: Start frame + Channel Side identification
  FS Polarity: FS active Low
  FS Offset: FS asserted one bit before the first bit of slot 0 */
  BSP_AUDIO_hSai.FrameInit.FrameLength = 32;
  BSP_AUDIO_hSai.FrameInit.ActiveFrameLength = 16;
  BSP_AUDIO_hSai.FrameInit.FSDefinition = SAI_FS_CHANNEL_IDENTIFICATION;
  BSP_AUDIO_hSai.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  BSP_AUDIO_hSai.FrameInit.FSOffset = SAI_FS_BEFOREFIRSTBIT;

  /* Configure SAI Block_x Slot
  Slot First Bit Offset: 0
  Slot Size  : 16
  Slot Number: 2
  Slot Active: Slots 0 and 1 actives */
  BSP_AUDIO_hSai.SlotInit.FirstBitOffset = 0;
  BSP_AUDIO_hSai.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
  BSP_AUDIO_hSai.SlotInit.SlotNumber = 2;
  BSP_AUDIO_hSai.SlotInit.SlotActive = SAI_SLOTACTIVE_0 | SAI_SLOTACTIVE_1;

  /* Initializes the SAI peripheral*/
  if (HAL_SAI_Init(&BSP_AUDIO_hSai) != HAL_OK)
  {
    return AUDIO_ERROR;
  }

  /* Enable SAI peripheral to generate MCLK */
  __HAL_SAI_ENABLE(&BSP_AUDIO_hSai);

  return AUDIO_OK;

}

/**
  * @brief  De-initializes the Audio Codec audio interface (SAI).
  * @retval BSP AUDIO status
  */
static uint8_t AUDIO_SAIx_DeInit(void)
{
  /* Disable the SAI audio block */
  __HAL_SAI_DISABLE(&BSP_AUDIO_hSai);

  /* De-initializes the SAI peripheral */
  if (HAL_SAI_DeInit(&BSP_AUDIO_hSai) != HAL_OK)
  {
    return AUDIO_ERROR;
  }

  /* Disable SAIx PLL */
  if (AUDIO_SAIx_PLL_DISABLE() != AUDIO_OK)
  {
    return AUDIO_ERROR;
  }

  return AUDIO_OK;
}

/**
  * @brief  SAI MSP Init
  * @param  hsai : pointer to a SAI_HandleTypeDef structure
  * @retval None
  */
void HAL_SAI_MspInit(SAI_HandleTypeDef *hsai)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  /* Enable SAI clock */
  AUDIO_SAIx_CLK_ENABLE();

  /* Enable GPIO clock */
  AUDIO_SAIx_MCK_SCK_SD_FS_ENABLE();

  /* CODEC_SAI pins configuration: FS, SCK, MCK and SD pins ------------------*/
  GPIO_InitStruct.Pin = AUDIO_SAIx_FS_PIN | AUDIO_SAIx_SCK_PIN | AUDIO_SAIx_SD_PIN | AUDIO_SAIx_MCK_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = AUDIO_SAIx_MCK_SCK_SD_FS_AF;
  HAL_GPIO_Init(AUDIO_SAIx_MCK_SCK_SD_FS_GPIO_PORT, &GPIO_InitStruct);

  /* Enable the DMA clock */
  AUDIO_SAIx_DMAx_CLK_ENABLE();

  if (hsai->Instance == AUDIO_SAIx)
  {
    /* Configure the hDmaSai handle parameters */
    hDmaSai.Init.Request             = DMA_REQUEST_1;
    hDmaSai.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hDmaSai.Init.PeriphInc           = DMA_PINC_DISABLE;
    hDmaSai.Init.MemInc              = DMA_MINC_ENABLE;
    hDmaSai.Init.PeriphDataAlignment = AUDIO_SAIx_DMAx_PERIPH_DATA_SIZE;
    hDmaSai.Init.MemDataAlignment    = AUDIO_SAIx_DMAx_MEM_DATA_SIZE;
    hDmaSai.Init.Mode                = DMA_NORMAL;
    hDmaSai.Init.Priority            = DMA_PRIORITY_HIGH;

    hDmaSai.Instance = AUDIO_SAIx_DMAx_CHANNEL;

    /* Associate the DMA handle */
    __HAL_LINKDMA(hsai, hdmatx, hDmaSai);

    /* Deinitialize the Stream for new transfer */
    HAL_DMA_DeInit(&hDmaSai);

    /* Configure the DMA Stream */
    HAL_DMA_Init(&hDmaSai);
  }

  /* SAI DMA IRQ Channel configuration */
  HAL_NVIC_SetPriority(AUDIO_SAIx_DMAx_IRQ, AUDIO_OUT_IRQ_PREPRIO, 0);
  HAL_NVIC_EnableIRQ(AUDIO_SAIx_DMAx_IRQ);
}

/**
  * @brief  SAI MSP De-init
  * @param  hsai : pointer to a SAI_HandleTypeDef structure
  * @retval None
  */
void HAL_SAI_MspDeInit(SAI_HandleTypeDef *hsai)
{
  /* Disable SAI DMA Channel IRQ  */
  HAL_NVIC_DisableIRQ(AUDIO_SAIx_DMAx_IRQ);

  /* Reset the DMA Stream configuration*/
  HAL_DMA_DeInit(&hDmaSai);

  /* Disable the DMA clock */
  AUDIO_SAIx_DMAx_CLK_DISABLE();

  /* De-initialize FS, SCK, MCK and SD pins*/
  HAL_GPIO_DeInit(AUDIO_SAIx_MCK_SCK_SD_FS_GPIO_PORT,
                  AUDIO_SAIx_FS_PIN | AUDIO_SAIx_SCK_PIN | AUDIO_SAIx_SD_PIN | AUDIO_SAIx_MCK_PIN);

  /* Disable GPIO clock */
  AUDIO_SAIx_MCK_SCK_SD_FS_DISABLE();

  /* Disable SAI clock */
  AUDIO_SAIx_CLK_DISABLE();
}

/**
  * @brief  Resets the audio codec. It restores the default configuration of the
  *         codec (this function shall be called before initializing the codec).
  * @retval None
  */
static void AUDIO_CODEC_Reset(void)
{
  /* Initialize the audio driver structure */
  hAudioOut.AudioDrv = &cs43l22_drv;

  hAudioOut.AudioDrv->Reset(AUDIO_I2C_ADDRESS);
}

/**
  * @}
  */

/** @addtogroup STM32L476G_DISCOVERY_AUDIO_Private_Functions
  * @{
  */

/**
  * @brief  Initializes the Digital Filter for Sigma-Delta Modulators interface (DFSDM).
  * @param  AudioFreq: Audio frequency to be used to set correctly the DFSDM peripheral.
  * @retval BSP AUDIO status
  */
static uint8_t AUDIO_DFSDMx_Init(uint32_t AudioFreq)
{
  /*####CHANNEL 2####*/
  hAudioIn.hDfsdmLeftChannel.Init.OutputClock.Activation   = ENABLE;
  hAudioIn.hDfsdmLeftChannel.Init.OutputClock.Selection    = DFSDM_CHANNEL_OUTPUT_CLOCK_AUDIO;
  /* Set the DFSDM clock OUT audio frequency configuration */
  hAudioIn.hDfsdmLeftChannel.Init.OutputClock.Divider      = DFSDMClockDivider(AudioFreq);
  hAudioIn.hDfsdmLeftChannel.Init.Input.Multiplexer        = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hAudioIn.hDfsdmLeftChannel.Init.Input.DataPacking        = DFSDM_CHANNEL_STANDARD_MODE;
  hAudioIn.hDfsdmLeftChannel.Init.Input.Pins               = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  /* Request to sample stable data for LEFT micro on Rising edge */
  hAudioIn.hDfsdmLeftChannel.Init.SerialInterface.Type     = DFSDM_CHANNEL_SPI_RISING;
  hAudioIn.hDfsdmLeftChannel.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hAudioIn.hDfsdmLeftChannel.Init.Awd.FilterOrder          = DFSDM_CHANNEL_SINC1_ORDER;
  hAudioIn.hDfsdmLeftChannel.Init.Awd.Oversampling         = 10;
  hAudioIn.hDfsdmLeftChannel.Init.Offset                   = 0;
  hAudioIn.hDfsdmLeftChannel.Init.RightBitShift            = DFSDMRightBitShift(AudioFreq);

  hAudioIn.hDfsdmLeftChannel.Instance                      = DFSDM1_Channel2;

  /* Init the DFSDM Channel */
  if (HAL_DFSDM_ChannelInit(&hAudioIn.hDfsdmLeftChannel) != HAL_OK)
  {
    return AUDIO_ERROR;
  }

  /*####FILTER 0####*/
  BSP_AUDIO_hDfsdmLeftFilter.Init.RegularParam.Trigger         = DFSDM_FILTER_SW_TRIGGER;
  BSP_AUDIO_hDfsdmLeftFilter.Init.RegularParam.FastMode        = ENABLE;
  BSP_AUDIO_hDfsdmLeftFilter.Init.RegularParam.DmaMode         = ENABLE;
  BSP_AUDIO_hDfsdmLeftFilter.Init.InjectedParam.Trigger        = DFSDM_FILTER_SW_TRIGGER;
  BSP_AUDIO_hDfsdmLeftFilter.Init.InjectedParam.ScanMode       = DISABLE;
  BSP_AUDIO_hDfsdmLeftFilter.Init.InjectedParam.DmaMode        = DISABLE;
  BSP_AUDIO_hDfsdmLeftFilter.Init.InjectedParam.ExtTrigger     = DFSDM_FILTER_EXT_TRIG_TIM8_TRGO;
  BSP_AUDIO_hDfsdmLeftFilter.Init.InjectedParam.ExtTriggerEdge = DFSDM_FILTER_EXT_TRIG_BOTH_EDGES;
  BSP_AUDIO_hDfsdmLeftFilter.Init.FilterParam.SincOrder        = DFSDMFilterOrder(AudioFreq);
  /* Set the DFSDM Filters Oversampling to have correct sample rate */
  BSP_AUDIO_hDfsdmLeftFilter.Init.FilterParam.Oversampling     = DFSDMOverSampling(AudioFreq);
  BSP_AUDIO_hDfsdmLeftFilter.Init.FilterParam.IntOversampling  = 1;

  BSP_AUDIO_hDfsdmLeftFilter.Instance                          = AUDIO_DFSDMx_LEFT_FILTER;

  /* Init the DFSDM Filter */
  if (HAL_DFSDM_FilterInit(&BSP_AUDIO_hDfsdmLeftFilter) != HAL_OK)
  {
    return AUDIO_ERROR;
  }

  /* Configure regular channel */
  if (HAL_DFSDM_FilterConfigRegChannel(&BSP_AUDIO_hDfsdmLeftFilter,
                                       DFSDM_CHANNEL_2,
                                       DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
  {
    return AUDIO_ERROR;
  }

  return AUDIO_OK;
}

/**
  * @brief  De-initializes the Digital Filter for Sigma-Delta Modulators interface (DFSDM).
  * @retval BSP AUDIO status
  */
static uint8_t AUDIO_DFSDMx_DeInit(void)
{
  /* De-initializes the DFSDM filters to allow access to DFSDM internal registers */
  if (HAL_DFSDM_FilterDeInit(&BSP_AUDIO_hDfsdmLeftFilter) != HAL_OK)
  {
    return AUDIO_ERROR;
  }

  /* De-initializes the DFSDM channels to allow access to DFSDM internal registers */
  if (HAL_DFSDM_ChannelDeInit(&hAudioIn.hDfsdmLeftChannel) != HAL_OK)
  {
    return AUDIO_ERROR;
  }

  /* Disable DFSDM clock */
  AUDIO_DFSDMx_CLK_DISABLE();

  /* Disable SAIx PLL */
  if (AUDIO_SAIx_PLL_DISABLE() != AUDIO_OK)
  {
    return AUDIO_ERROR;
  }

  /* DFSDM reset */
  __HAL_RCC_DFSDM1_FORCE_RESET();
  __HAL_RCC_DFSDM1_RELEASE_RESET();

  return AUDIO_OK;
}

/**
  * @brief  Initializes the DFSDM channel MSP.
  * @param  hdfsdm_channel : DFSDM channel handle.
  * @retval None
  */
void HAL_DFSDM_ChannelMspInit(DFSDM_Channel_HandleTypeDef *hdfsdm_channel)
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
  * @brief  De-initializes the DFSDM channel MSP.
  * @param  hdfsdm_channel : DFSDM channel handle.
  * @retval None
  */
void HAL_DFSDM_ChannelMspDeInit(DFSDM_Channel_HandleTypeDef *hdfsdm_channel)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  /* Enable GPIO clock */
  AUDIO_DFSDMx_CKOUT_DMIC_DATIN_GPIO_CLK_ENABLE();

  /* DFSDM pins configuration: DFSDM_CKOUT */
  GPIO_InitStruct.Pin = AUDIO_DFSDMx_CKOUT_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(AUDIO_DFSDMx_CKOUT_DMIC_DATIN_GPIO_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(AUDIO_DFSDMx_CKOUT_DMIC_DATIN_GPIO_PORT, AUDIO_DFSDMx_CKOUT_PIN, GPIO_PIN_RESET);


  /* De-initialize DMIC_DATIN pin */
  HAL_GPIO_DeInit(AUDIO_DFSDMx_CKOUT_DMIC_DATIN_GPIO_PORT, AUDIO_DFSDMx_DMIC_DATIN_PIN);
}

/**
  * @brief  Initializes the DFSDM filter MSP.
  * @param  hdfsdm_filter : DFSDM filter handle.
  * @retval None
  */
void HAL_DFSDM_FilterMspInit(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
  /* Enable DFSDM clock */
  AUDIO_DFSDMx_CLK_ENABLE();

  /* Enable the DMA clock */
  AUDIO_DFSDMx_DMAx_CLK_ENABLE();

  /* Configure the hAudioIn.hDmaDfsdmLeft handle parameters */
  hAudioIn.hDmaDfsdmLeft.Init.Request             = DMA_REQUEST_0;
  hAudioIn.hDmaDfsdmLeft.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  hAudioIn.hDmaDfsdmLeft.Init.PeriphInc           = DMA_PINC_DISABLE;
  hAudioIn.hDmaDfsdmLeft.Init.MemInc              = DMA_MINC_ENABLE;
  hAudioIn.hDmaDfsdmLeft.Init.PeriphDataAlignment = AUDIO_DFSDMx_DMAx_PERIPH_DATA_SIZE;
  hAudioIn.hDmaDfsdmLeft.Init.MemDataAlignment    = AUDIO_DFSDMx_DMAx_MEM_DATA_SIZE;
  hAudioIn.hDmaDfsdmLeft.Init.Mode                = DMA_CIRCULAR;
  hAudioIn.hDmaDfsdmLeft.Init.Priority            = DMA_PRIORITY_HIGH;

  hAudioIn.hDmaDfsdmLeft.Instance               = AUDIO_DFSDMx_DMAx_LEFT_CHANNEL;

  /* Associate the DMA handle */
  __HAL_LINKDMA(hdfsdm_filter, hdmaReg, hAudioIn.hDmaDfsdmLeft);

  /* Reset DMA handle state */
  __HAL_DMA_RESET_HANDLE_STATE(&hAudioIn.hDmaDfsdmLeft);

  /* Configure the DMA Channel */
  HAL_DMA_Init(&hAudioIn.hDmaDfsdmLeft);

  /* DMA IRQ Channel configuration */
  HAL_NVIC_SetPriority(AUDIO_DFSDMx_DMAx_LEFT_IRQ, AUDIO_OUT_IRQ_PREPRIO, 0);
  HAL_NVIC_EnableIRQ(AUDIO_DFSDMx_DMAx_LEFT_IRQ);
}

/**
 * @brief  De-initializes the DFSDM filter MSP.
 * @param  hdfsdm_filter : DFSDM filter handle.
 * @retval None
 */
void HAL_DFSDM_FilterMspDeInit(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
  /* Disable DMA  Channel IRQ */
  HAL_NVIC_DisableIRQ(AUDIO_DFSDMx_DMAx_LEFT_IRQ);

  /* De-initialize the DMA Channel */
  HAL_DMA_DeInit(&hAudioIn.hDmaDfsdmLeft);

  /* Disable the DMA clock */
  AUDIO_DFSDMx_DMAx_CLK_DISABLE();
}

/**
  * @brief  Configures the SAI PLL clock according to the required audio frequency.
  * @param  Frequency: Audio frequency.
  * @retval BSP AUDIO status
  * @note   The SAI PLL input clock must be configured in the user application.
  *         The SAI PLL configuration done within this function assumes that
  *         the SAI PLL input clock runs at 8 MHz.
  */
static uint8_t AUDIO_SAIPLLConfig(uint32_t Frequency)
{
  RCC_PeriphCLKInitTypeDef RCC_ExCLKInitStruct;

  /* Retrieve actual RCC configuration */
  HAL_RCCEx_GetPeriphCLKConfig(&RCC_ExCLKInitStruct);

  if ((Frequency == AUDIO_FREQUENCY_11K)
      || (Frequency == AUDIO_FREQUENCY_22K)
      || (Frequency == AUDIO_FREQUENCY_44K))
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
  }

  if (HAL_RCCEx_PeriphCLKConfig(&RCC_ExCLKInitStruct) != HAL_OK)
  {
    return AUDIO_ERROR;
  }

  return AUDIO_OK;
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


