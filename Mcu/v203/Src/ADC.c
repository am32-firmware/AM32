/*
 * ADC.c
 *
 *  Created on: May 20, 2020
 *      Author: Alka
 *      Modified by TempersLee June 21, 2024
 */
#include "ADC.h"

#include "functions.h"
#ifdef USE_ADC
#ifdef USE_ADC_INPUT
uint16_t ADCDataDMA[4];
#else
uint16_t ADCDataDMA[3];
#endif

extern uint16_t ADC_raw_temp;
extern uint16_t ADC_raw_volts;
extern uint16_t ADC_raw_current;
extern uint16_t ADC_raw_input;

void ADC_DMA_Callback()
{ // read dma buffer and set extern variables

#ifdef USE_ADC_INPUT
    ADC_raw_temp = ADCDataDMA[3];
    ADC_raw_volts = ADCDataDMA[1] / 2;
    ADC_raw_current = ADCDataDMA[2];
    ADC_raw_input = ADCDataDMA[0];

#else
    ADC_raw_temp = ADCDataDMA[2];
#ifdef PA6_VOLTAGE
    ADC_raw_volts = ADCDataDMA[1];
    ADC_raw_current = ADCDataDMA[0];
#else
    ADC_raw_volts = ADCDataDMA[0];
    ADC_raw_current = ADCDataDMA[1];
#endif
#endif
}

void ADCInit(void)
{
    //for DMA channel
    DMA_InitTypeDef DMA_InitStructure = {0};
    ADC_InitTypeDef ADC_InitStruct = {0};
    GPIO_InitTypeDef GPIO_InitStruct = {0};


    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&ADC1->RDATAR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADCDataDMA[0];
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
#ifdef USE_ADC_INPUT
    DMA_InitStructure.DMA_BufferSize = 4;
#else
    DMA_InitStructure.DMA_BufferSize = 3;
#endif
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);

    DMA_ITConfig(DMA1_Channel1,DMA_IT_TC | DMA_IT_TE, ENABLE);
    DMA_Cmd(DMA1_Channel1, ENABLE);

    NVIC_SetPriority(DMA1_Channel1_IRQn, 0xC0);
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);


    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div2);    //frequency is 48MHz

   /**ADC GPIO Configuration
   PA1   ------> ADC_IN1
   PA6   ------> ADC_IN6
   */
 #ifdef USE_ADC_INPUT
     GPIO_InitStruct.GPIO_Pin = INPUT_PIN;
     GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN;
     GPIO_Init(INPUT_PIN_PORT, &GPIO_InitStruct);
 #endif

   GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_6;
   GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN;
   GPIO_Init(GPIOA, &GPIO_InitStruct);

   ADC_DeInit(ADC1);
   ADC_InitStruct.ADC_Mode = ADC_Mode_Independent;
   ADC_InitStruct.ADC_ScanConvMode = ENABLE;
   ADC_InitStruct.ADC_ContinuousConvMode = DISABLE;
   ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
   ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
 #ifdef USE_ADC_INPUT
   ADC_InitStruct.ADC_NbrOfChannel = 4;
 #else
   ADC_InitStruct.ADC_NbrOfChannel = 3;
 #endif
   ADC_Init(ADC1, &ADC_InitStruct);


 #ifdef USE_ADC_INPUT
   ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_7Cycles5);
   ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_7Cycles5);
   ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 3, ADC_SampleTime_7Cycles5);
   ADC_RegularChannelConfig(ADC1, ADC_Channel_TempSensor, 4, ADC_SampleTime_7Cycles5);
 #else
   ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_7Cycles5);
   ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 2, ADC_SampleTime_7Cycles5);
   ADC_RegularChannelConfig(ADC1, ADC_Channel_TempSensor, 3, ADC_SampleTime_7Cycles5);
 #endif


   //activate ADC
   ADC_DMACmd(ADC1, ENABLE);
   ADC_Cmd(ADC1, ENABLE);

   ADC_BufferCmd(ADC1, DISABLE); //disable buffer

   ADC_ResetCalibration(ADC1);
   while(ADC_GetResetCalibrationStatus(ADC1));
   ADC_StartCalibration(ADC1);
   while(ADC_GetCalibrationStatus(ADC1));


   ADC_BufferCmd(ADC1, ENABLE);
   ADC_TempSensorVrefintCmd( ENABLE );
}

//trigger ADC
void startADCConversion()
{
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}
int16_t getConvertedDegrees(uint16_t adcrawtemp)
{
    return TempSensor_Volt_To_Temper(ADC_raw_temp * 3300 / 4096);  //maybe 4096 is better
}

#endif // USE_ADC
