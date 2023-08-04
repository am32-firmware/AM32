/*
 * ADC.c
 *
 *  Created on: May 20, 2020
 *      Author: Alka
 */
#include "ADC.h"
#include "functions.h"

#ifdef USE_ADC_INPUT
uint16_t ADCDataDMA[4];
#else
uint16_t ADCDataDMA[3];
#endif


extern uint16_t ADC_raw_temp;
extern uint16_t ADC_raw_volts;
extern uint16_t ADC_raw_current;
extern uint16_t ADC_raw_input;

void ADC_DMA_Callback(){  // read dma buffer and set extern variables

#ifdef USE_ADC_INPUT
	ADC_raw_temp =    ADCDataDMA[3];
	ADC_raw_volts  = ADCDataDMA[1]/2;
	ADC_raw_current =ADCDataDMA[2];
	ADC_raw_input = ADCDataDMA[0];


#else
ADC_raw_temp =    ADCDataDMA[2];
#ifdef PA6_VOLTAGE
ADC_raw_volts  = ADCDataDMA[1];
ADC_raw_current =ADCDataDMA[0];
#else
ADC_raw_volts  = ADCDataDMA[0];
ADC_raw_current =ADCDataDMA[1];
#endif
#endif
}


void ADC_Init(void){
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_ADC);
    rcu_periph_clock_enable(RCU_DMA);
	  rcu_adc_clock_config(RCU_ADCCK_APB2_DIV6);
	
    /* config the GPIO as analog mode */
    gpio_mode_set(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO_PIN_3|GPIO_PIN_6);

	    dma_parameter_struct dma_data_parameter;
    
    /* ADC DMA_channel configuration */
    dma_deinit(DMA_CH0);

    /* initialize DMA single data mode */
    dma_data_parameter.periph_addr  = (uint32_t)(&ADC_RDATA);
    dma_data_parameter.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
    dma_data_parameter.memory_addr  = (uint32_t)(&ADCDataDMA);
    dma_data_parameter.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
    dma_data_parameter.periph_width = DMA_PERIPHERAL_WIDTH_16BIT;
    dma_data_parameter.memory_width = DMA_MEMORY_WIDTH_16BIT;  
    dma_data_parameter.direction    = DMA_PERIPHERAL_TO_MEMORY;
    dma_data_parameter.number       = 3U;
    dma_data_parameter.priority     = DMA_PRIORITY_LOW;
    dma_init(DMA_CH0, &dma_data_parameter);

    dma_circulation_enable(DMA_CH0);
  
    /* enable DMA channel */
    dma_channel_enable(DMA_CH0);
	
	  adc_special_function_config(ADC_CONTINUOUS_MODE, ENABLE);
    /* ADC scan function enable */
    adc_special_function_config(ADC_SCAN_MODE, ENABLE);
    /* ADC data alignment config */
    adc_data_alignment_config(ADC_DATAALIGN_RIGHT);
    /* ADC channel length config */
    adc_channel_length_config(ADC_REGULAR_CHANNEL, 3U);
 
    adc_tempsensor_vrefint_enable();         // enable the temp sensor
 
    /* ADC regular channel config */
    adc_regular_channel_config(0, ADC_CHANNEL_3, ADC_SAMPLETIME_55POINT5);
    adc_regular_channel_config(1, ADC_CHANNEL_6, ADC_SAMPLETIME_55POINT5);
    adc_regular_channel_config(2, ADC_CHANNEL_16, ADC_SAMPLETIME_55POINT5);
    

    /* ADC trigger config */
    adc_external_trigger_source_config(ADC_REGULAR_CHANNEL, ADC_EXTTRIG_REGULAR_NONE); 
    adc_external_trigger_config(ADC_REGULAR_CHANNEL, ENABLE);

    /* enable ADC interface */
    adc_enable();
    delayMicros(1000);
    /* ADC calibration and reset calibration */
    adc_calibration_enable();

    /* ADC DMA function enable */
    adc_dma_mode_enable();
    /* ADC software trigger enable */
    adc_software_trigger_enable(ADC_REGULAR_CHANNEL);
	
	
}


//void ADC_Init(void)
//{
//  LL_ADC_InitTypeDef ADC_InitStruct = {0};
//  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};

//  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

//  /* Peripheral clock enable */
//  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_ADC1);

//  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
//  /**ADC GPIO Configuration
//  PA3   ------> ADC_IN3
//  PA6   ------> ADC_IN6
//  */
//#ifdef USE_ADC_INPUT
//  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//#endif


//  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

//  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);


//  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

//  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_LOW);

//  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_CIRCULAR);

//  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);

//  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);

//  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_HALFWORD);

//  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_HALFWORD);


//#ifdef USE_ADC_INPUT
//  LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_2);
//#endif

//  LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_3);
//  /** Configure Regular Channel
//  */
//  LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_6);
//  /** Configure Regular Channel
//  */
//  LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_TEMPSENSOR);
//  /** Configure Internal Channel
//  */
//  LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_PATH_INTERNAL_TEMPSENSOR);
//  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
//  */
//  ADC_InitStruct.Clock = LL_ADC_CLOCK_SYNC_PCLK_DIV4;
//  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
//  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
//  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
//  LL_ADC_Init(ADC1, &ADC_InitStruct);
//  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_EXT_TIM1_CH4;
//  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
//  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
//  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_LIMITED;
//  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;
//  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
//  LL_ADC_REG_SetTriggerEdge(ADC1, LL_ADC_REG_TRIG_EXT_FALLING);
//  LL_ADC_REG_SetSequencerScanDirection(ADC1, LL_ADC_REG_SEQ_SCAN_DIR_FORWARD);
//  LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_71CYCLES_5);
//  LL_ADC_DisableIT_EOC(ADC1);
//  LL_ADC_DisableIT_EOS(ADC1);

//}
