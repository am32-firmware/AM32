/*
 * ADC.c
 *
 *  Created on: May 20, 2020
 *      Author: Alka
 */
 #include "ADC.h"
#include "ntc_tables.h"


 #ifdef USE_ADC_1_2
 uint16_t ADC1DataDMA[2];
 uint16_t ADC2DataDMA[2];
 #else

 #ifdef USE_ADC_INPUT
 uint16_t ADCDataDMA[4];
 #else
 uint16_t ADCDataDMA[3];
 #endif
 #endif
 
 extern uint16_t ADC_raw_temp;
 extern uint16_t ADC_raw_volts;
 extern uint16_t ADC_raw_current;
 extern uint16_t ADC_raw_input;
 extern uint16_t ADC_raw_ntc;

#define ADC_DELAY_CALIB_ENABLE_CPU_CYCLES \
    (LL_ADC_DELAY_CALIB_ENABLE_ADC_CYCLES * 64)

 void ADC_DMA_Callback()
{ // read dma buffer and set extern variables
 #ifdef USE_ADC_1_2
     ADC_raw_temp = ADC1DataDMA[0];
     ADC_raw_ntc = ADC1DataDMA[1];
     ADC_raw_volts = ADC2DataDMA[0];
     ADC_raw_current = ADC2DataDMA[1];
  
 #else

     ADC_raw_temp = ADCDataDMA[0];
     ADC_raw_volts = ADCDataDMA[1];
     ADC_raw_current = ADCDataDMA[2];
 #endif
 }

 void enableADC_DMA()
{
 #ifdef USE_ADC_1_2
//    NVIC_SetPriority(DMA1_Channel3_IRQn, 3);
//    NVIC_EnableIRQ(DMA1_Channel3_IRQn);
//  
//    NVIC_SetPriority(DMA1_Channel2_IRQn, 3);
//    NVIC_EnableIRQ(DMA1_Channel2_IRQn);

    LL_DMA_ConfigAddresses(
        DMA1, LL_DMA_CHANNEL_4,
        LL_ADC_DMA_GetRegAddr(ADC2, LL_ADC_DMA_REG_REGULAR_DATA),
        (uint32_t)&ADC2DataDMA, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  
      LL_DMA_ConfigAddresses(
        DMA1, LL_DMA_CHANNEL_2,
        LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA),
        (uint32_t)&ADC1DataDMA, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

    /* Set DMA transfer size */

    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_4, 2);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, 2);
 

    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_4);
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);
  
  #else
  // enables channel

    NVIC_SetPriority(DMA1_Channel2_IRQn, 3);
    NVIC_EnableIRQ(DMA1_Channel2_IRQn);

    LL_DMA_ConfigAddresses(
        DMA1, LL_DMA_CHANNEL_2,
        LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA),
        (uint32_t)&ADCDataDMA, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

    /* Set DMA transfer size */
 #ifdef USE_ADC_INPUT
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, 4);
 #else
  #ifdef USE_CURRENT_SENSE
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, 3);
  #else
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, 2);
  #endif
 #endif

    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);
    
    #endif
}

void activateADC(void)
{
#ifdef USE_ADC_1_2
  __IO uint32_t wait_loop_index = 0U;
  #if (USE_TIMEOUT == 1)
  uint32_t Timeout = 0U; /* Variable used for timeout management */
  #endif /* USE_TIMEOUT */
  if (LL_ADC_IsEnabled(ADC1) == 0)
  {
    LL_ADC_DisableDeepPowerDown(ADC1);
    LL_ADC_EnableInternalRegulator(ADC1);
    wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
    while(wait_loop_index != 0)
    {
      wait_loop_index--;
    }
    LL_ADC_StartCalibration(ADC1, LL_ADC_SINGLE_ENDED);
    #if (USE_TIMEOUT == 1)
    Timeout = ADC_CALIBRATION_TIMEOUT_MS;
    #endif /* USE_TIMEOUT */
        while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0)
    {
    }
    wait_loop_index = (ADC_DELAY_CALIB_ENABLE_CPU_CYCLES >> 1);
    while(wait_loop_index != 0)
    {
      wait_loop_index--;
    }
    LL_ADC_Enable(ADC1);
    
    while (LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0)
    {
    }
  } 
  
    if (LL_ADC_IsEnabled(ADC2) == 0)
  {
    LL_ADC_DisableDeepPowerDown(ADC2);
    LL_ADC_EnableInternalRegulator(ADC2);
    wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
    while(wait_loop_index != 0)
    {
      wait_loop_index--;
    }
    LL_ADC_StartCalibration(ADC2, LL_ADC_SINGLE_ENDED);
    #if (USE_TIMEOUT == 1)
    Timeout = ADC_CALIBRATION_TIMEOUT_MS;
    #endif /* USE_TIMEOUT */
        while (LL_ADC_IsCalibrationOnGoing(ADC2) != 0)
    {
    }
    wait_loop_index = (ADC_DELAY_CALIB_ENABLE_CPU_CYCLES >> 1);
    while(wait_loop_index != 0)
    {
      wait_loop_index--;
    }
    LL_ADC_Enable(ADC2);
    
    while (LL_ADC_IsActiveFlag_ADRDY(ADC2) == 0)
    {
    }
  } 
  
  
#else  
  __IO uint32_t wait_loop_index = 0U;
  #if (USE_TIMEOUT == 1)
  uint32_t Timeout = 0U; /* Variable used for timeout management */
  #endif /* USE_TIMEOUT */
  if (LL_ADC_IsEnabled(ADC1) == 0)
  {
    /* Disable ADC deep power down (enabled by default after reset state) */
    LL_ADC_DisableDeepPowerDown(ADC1);
    
    /* Enable ADC internal voltage regulator */
    LL_ADC_EnableInternalRegulator(ADC1);
    wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
    while(wait_loop_index != 0)
    {
      wait_loop_index--;
    }
    
    /* Run ADC self calibration */
    LL_ADC_StartCalibration(ADC1, LL_ADC_SINGLE_ENDED);
    
    /* Poll for ADC effectively calibrated */
    #if (USE_TIMEOUT == 1)
    Timeout = ADC_CALIBRATION_TIMEOUT_MS;
    #endif /* USE_TIMEOUT */
    
    while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0)
    {
    }
    wait_loop_index = (ADC_DELAY_CALIB_ENABLE_CPU_CYCLES >> 1);
    while(wait_loop_index != 0)
    {
      wait_loop_index--;
    }
    
    /* Enable ADC */
    LL_ADC_Enable(ADC1);
    
    while (LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0)
    {
    }
  }                                
#endif
}

#ifdef USE_ADC_1_2

void ADC_Init(void){
  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_RCC_SetADCClockSource(LL_RCC_ADC12_CLKSOURCE_SYSCLK);

  /* Peripheral clock enable */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC12);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
#ifdef EPROPELLED_6S_G431_4in1
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOF);
#endif
  /**ADC2 GPIO Configuration
  PA6   ------> ADC2_IN3
  PA7   ------> ADC2_IN4
  pb1  -------> adc1_in12
  */

  GPIO_InitStruct.Pin = NTC_ADC_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
#ifdef EPROPELLED_6S_G431_4in1
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
#endif
  
  GPIO_InitStruct.Pin = VOLTAGE_ADC_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = CURRENT_ADC_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
#ifdef EPROPELLED_6S_G431_4in1
  LL_GPIO_Init(GPIOF, &GPIO_InitStruct);
#endif


  /*BEGIN ADC1 SETUP */

  LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};

  LL_RCC_SetADCClockSource(LL_RCC_ADC12_CLKSOURCE_SYSCLK);

  /* Peripheral clock enable */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC12);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);


  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_2, LL_DMAMUX_REQ_ADC1);
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PRIORITY_LOW);
  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MODE_CIRCULAR);
  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PDATAALIGN_WORD);
  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MDATAALIGN_HALFWORD);


  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_ENABLE_2RANKS;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_LIMITED;
  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
  LL_ADC_SetGainCompensation(ADC1, 0);
  LL_ADC_SetOverSamplingScope(ADC1, LL_ADC_OVS_DISABLE);
  ADC_CommonInitStruct.CommonClock = LL_ADC_CLOCK_SYNC_PCLK_DIV4;
  ADC_CommonInitStruct.Multimode = LL_ADC_MULTI_INDEPENDENT;
  LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);

  LL_ADC_DisableDeepPowerDown(ADC1);
  LL_ADC_EnableInternalRegulator(ADC1);

  uint32_t wait_loop_index;
  wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
  while(wait_loop_index != 0)
  {
    wait_loop_index--;
  }

  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_TEMPSENSOR_ADC1);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_TEMPSENSOR_ADC1, LL_ADC_SAMPLINGTIME_47CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_TEMPSENSOR_ADC1, LL_ADC_SINGLE_ENDED);
  LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_PATH_INTERNAL_TEMPSENSOR);

  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_2, NTC_ADC_CHANNEL);
#if defined(EPROPELLED_6S_G431_SINGLE) || defined(EPROPELLED_6S_G431_4in1)
  LL_ADC_SetChannelSamplingTime(ADC1, NTC_ADC_CHANNEL, LL_ADC_SAMPLINGTIME_47CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC1, NTC_ADC_CHANNEL, LL_ADC_SINGLE_ENDED);
#else
  LL_ADC_SetChannelSamplingTime(ADC1, VOLTAGE_ADC_CHANNEL, LL_ADC_SAMPLINGTIME_47CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC1, VOLTAGE_ADC_CHANNEL, LL_ADC_SINGLE_ENDED);
#endif

 

  /*ADC 2 setup */
  
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_4, LL_DMAMUX_REQ_ADC2);
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_4, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_4, LL_DMA_PRIORITY_LOW);
  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_4, LL_DMA_MODE_CIRCULAR);
  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_4, LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_4, LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_4, LL_DMA_PDATAALIGN_WORD);
  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_4, LL_DMA_MDATAALIGN_HALFWORD);


  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
  LL_ADC_Init(ADC2, &ADC_InitStruct);
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_ENABLE_2RANKS;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_LIMITED;
  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;
  LL_ADC_REG_Init(ADC2, &ADC_REG_InitStruct);
  LL_ADC_SetGainCompensation(ADC2, 0);
  LL_ADC_SetOverSamplingScope(ADC2, LL_ADC_OVS_DISABLE);
  LL_ADC_DisableDeepPowerDown(ADC2);
  LL_ADC_EnableInternalRegulator(ADC2);

  wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
  while(wait_loop_index != 0)
  {
    wait_loop_index--;
  }

  #if defined(EPROPELLED_6S_G431_SINGLE) || defined(EPROPELLED_6S_G431_4in1)

  LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_1, VOLTAGE_ADC_CHANNEL);
  LL_ADC_SetChannelSamplingTime(ADC2, VOLTAGE_ADC_CHANNEL, LL_ADC_SAMPLINGTIME_2CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC2, VOLTAGE_ADC_CHANNEL, LL_ADC_SINGLE_ENDED);

  LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_2, CURRENT_ADC_CHANNEL);
  LL_ADC_SetChannelSamplingTime(ADC2, CURRENT_ADC_CHANNEL, LL_ADC_SAMPLINGTIME_47CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC2, CURRENT_ADC_CHANNEL, LL_ADC_SINGLE_ENDED);

  #else
  LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_3);
  LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_3, LL_ADC_SAMPLINGTIME_2CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC2, LL_ADC_CHANNEL_3, LL_ADC_SINGLE_ENDED);

  LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_4);
  LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_4, LL_ADC_SAMPLINGTIME_47CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC2, LL_ADC_CHANNEL_4, LL_ADC_SINGLE_ENDED);
  #endif

}
#else




void ADC_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
  LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_RCC_SetADCClockSource(LL_RCC_ADC12_CLKSOURCE_SYSCLK);

  /* Peripheral clock enable */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC12);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  /**ADC1 GPIO Configuration
  PA3   ------> ADC1_IN4
  */
  GPIO_InitStruct.Pin = VOLTAGE_ADC_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
 #ifdef USE_CURRENT_SENSE
  GPIO_InitStruct.Pin = CURRENT_ADC_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
#endif
  /* ADC1 DMA Init */

  /* ADC1 Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_2, LL_DMAMUX_REQ_ADC1);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MODE_CIRCULAR);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PDATAALIGN_WORD);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MDATAALIGN_HALFWORD);

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_ENABLE_2RANKS;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_LIMITED;
  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
  LL_ADC_SetGainCompensation(ADC1, 0);
  LL_ADC_SetOverSamplingScope(ADC1, LL_ADC_OVS_DISABLE);
  ADC_CommonInitStruct.CommonClock = LL_ADC_CLOCK_SYNC_PCLK_DIV4;
  ADC_CommonInitStruct.Multimode = LL_ADC_MULTI_INDEPENDENT;
  LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);

  /* Disable ADC deep power down (enabled by default after reset state) */
  LL_ADC_DisableDeepPowerDown(ADC1);
  /* Enable ADC internal voltage regulator */
  LL_ADC_EnableInternalRegulator(ADC1);
  /* Delay for ADC internal voltage regulator stabilization. */
  /* Compute number of CPU cycles to wait for, from delay in us. */
  /* Note: Variable divided by 2 to compensate partially */
  /* CPU processing cycles (depends on compilation optimization). */
  /* Note: If system core clock frequency is below 200kHz, wait time */
  /* is only a few CPU processing cycles. */
  uint32_t wait_loop_index;
  wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
  while(wait_loop_index != 0)
  {
    wait_loop_index--;
  }

  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_TEMPSENSOR_ADC1);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_TEMPSENSOR_ADC1, LL_ADC_SAMPLINGTIME_47CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_TEMPSENSOR_ADC1, LL_ADC_SINGLE_ENDED);
  LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_PATH_INTERNAL_TEMPSENSOR);

  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_2, VOLTAGE_ADC_CHANNEL);
  LL_ADC_SetChannelSamplingTime(ADC1, VOLTAGE_ADC_CHANNEL, LL_ADC_SAMPLINGTIME_47CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC1, VOLTAGE_ADC_CHANNEL, LL_ADC_SINGLE_ENDED);
#ifdef USE_CURRENT_SENSE
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_3, CURRENT_ADC_CHANNEL);
  LL_ADC_SetChannelSamplingTime(ADC1, CURRENT_ADC_CHANNEL, LL_ADC_SAMPLINGTIME_47CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC1, CURRENT_ADC_CHANNEL, LL_ADC_SINGLE_ENDED);
#endif  
   
}
#endif

#ifdef USE_LMT87
typedef struct {
    int16_t temp_c;
    uint16_t mv;
} lmt87_entry_t;

/*
 * LMT87 approximate table.
 * Generated from:
 * Vtemp = 2230.8 - 13.582 * (T - 30) - 0.00433 * (T - 30)^2
 *
 * Table is sorted from cold to hot.
 * Voltage decreases as temperature increases.
 */
static const lmt87_entry_t lmt87_table[] = {
    {  0, 3290 },//Original value is -50 but saturated to 0 due to uint8 in serial telemetry 
    {  0, 3160 },//Original value is -40
    {  0, 3030 },//Original value is -30
    {  0, 2899 },//Original value is -20
    {  0, 2767 },//Original value is -10
    {    0, 2634 },
    {   10, 2501 },
    {   20, 2366 },
    {   30, 2231 },
    {   40, 2095 },
    {   50, 1957 },
    {   60, 1819 },
    {   70, 1681 },
    {   80, 1541 },
    {   90, 1400 },
    {  100, 1259 },
    {  110, 1117 },
    {  120,  973 },
    {  130,  829 },
    {  140,  684 },
    {  150,  539 },
};
#define LMT87_TABLE_SIZE   (sizeof(lmt87_table) / sizeof(lmt87_table[0]))
#define ADC_VREF_MV        3300
#define ADC_MAX_COUNT      4095

int16_t getLMT87Degrees(uint16_t adc_raw)
{
    uint32_t vtemp_mv;
    uint32_t i;

    /*
     * Convert 12-bit ADC raw value to millivolts.
     * Example: 2614 -> about 2106 mV with 3.3V reference.
     */
    vtemp_mv = ((uint32_t)adc_raw * ADC_VREF_MV) / ADC_MAX_COUNT;

    /*
     * Clamp cold side.
     * If voltage is higher than table maximum, sensor is colder than table range.
     */
    if (vtemp_mv >= lmt87_table[0].mv) {
        return lmt87_table[0].temp_c;
    }

    /*
     * Clamp hot side.
     * If voltage is lower than table minimum, sensor is hotter than table range.
     */
    if (vtemp_mv <= lmt87_table[LMT87_TABLE_SIZE - 1].mv) {
        return lmt87_table[LMT87_TABLE_SIZE - 1].temp_c;
    }

    /*
     * Find the two table points around vtemp_mv.
     * Table voltage decreases as temperature increases.
     */
    for (i = 0; i < (LMT87_TABLE_SIZE - 1); i++) {
        uint16_t mv_high = lmt87_table[i].mv;
        uint16_t mv_low  = lmt87_table[i + 1].mv;

        if ((vtemp_mv <= mv_high) && (vtemp_mv >= mv_low)) {
            int16_t temp_low_c  = lmt87_table[i].temp_c;
            int16_t temp_high_c = lmt87_table[i + 1].temp_c;

            /*
             * Linear interpolation.
             *
             * Because voltage decreases with temperature:
             *
             * temp = temp_low_c +
             *        ((mv_high - vtemp_mv) * temperature_step) /
             *        (mv_high - mv_low)
             */
            return temp_low_c +
                   (int16_t)(((uint32_t)(mv_high - vtemp_mv) *
                              (uint32_t)(temp_high_c - temp_low_c)) /
                              (uint32_t)(mv_high - mv_low));
        }
    }

    /*
     * Should not reach here because of clamp checks.
     */
    return lmt87_table[LMT87_TABLE_SIZE - 1].temp_c;
}
#endif

#ifdef USE_NTC
int16_t getNTCDegrees(uint16_t ntcrawtemp){
  int p1,p2;
  p1 = NTC_table[ (ntcrawtemp >> 6)  ];
  p2 = NTC_table[ (ntcrawtemp >> 6)+1];
  return p1 - ( (p1-p2) * (ntcrawtemp & 0x003F) ) / 64;
}
#endif
