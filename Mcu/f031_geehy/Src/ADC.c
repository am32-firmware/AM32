/*
 * peripherals.c
 *
 *  Created on: 4. 24, 2026
 *      Author: Nong Jun
 */
 
#include "ADC.h"

#ifdef USE_ADC_INPUT
uint16_t ADCDataDMA[4];
#else
//uint16_t ADCDataDMA[4];
#endif

extern uint16_t ADC_raw_temp;
extern uint16_t ADC_raw_volts;
extern uint16_t ADC_raw_current;
extern uint16_t ADC_raw_input;

void ADC_DMA_Callback()
{ // read dma buffer and set extern variables

#ifdef USE_ADC_INPUT
    ADC_raw_temp = ADC->DR2;
    ADC_raw_volts = ADC->DR0;
    ADC_raw_current = ADC->DR1;
    ADC_raw_input = ADC->DR3;

#else
    ADC_raw_temp = ADC->DR2;
    ADC_raw_volts = ADC->DR0;
    ADC_raw_current = ADC->DR1;

#endif
    DDL_ADC_StopConversion(ADC);
}

void ADC_Init(void)
{
    DDL_GPIO_InitTypeDef    GPIO_InitStruct;

    /* Peripheral clock enable */
    DDL_RCC_Unlock();
    DDL_AHB_GRP1_EnableClock(DDL_AHB_GRP1_PERIPH_GPIOA);
    DDL_AHB_GRP1_EnableClock(DDL_AHB_GRP1_PERIPH_GPIOB);
    DDL_APB_GRP1_EnableClock(DDL_APB_GRP1_PERIPH_ADC);
    DDL_RCC_Lock();
    
    /* ADC GPIO Configuration */
    GPIO_InitStruct.Pin  = CURRENT_SENSE_ADC_PIN;
    GPIO_InitStruct.Mode = DDL_GPIO_MODE_ANALOG;
    DDL_GPIO_Init(CURRENT_SENSE_ADC_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = VOLTAGE_SENSE_ADC_PIN;
    GPIO_InitStruct.Mode = DDL_GPIO_MODE_ANALOG;
    DDL_GPIO_Init(VOLTAGE_SENSE_ADC_PORT, &GPIO_InitStruct);

//    GPIO_InitStruct.Pin = INPUT_SENSE_ADC_PIN;
//    GPIO_InitStruct.Mode = DDL_GPIO_MODE_ANALOG;
//    DDL_GPIO_Init(INPUT_SENSE_ADC_PORT, &GPIO_InitStruct);

    DDL_RCC_Unlock();
    DDL_RCC_SetADCClkDiv(DDL_RCC_ADCCLK_DIVISION_2);
    DDL_RCC_Lock();

    /* adc Configuration */
    DDL_ADC_SetVREF(ADC, DDL_ADC_VREF_SEL_AVDD);
    DDL_ADC_REG_SetContinuousMode(ADC, DDL_ADC_REG_CONV_SINGLE);
    DDL_ADC_SetDataAlignment(ADC, DDL_ADC_ALIGNMENT_RIGHT);
    DDL_ADC_REG_SetDMATransfer(ADC, DDL_ADC_REG_DMA_TRANSFER_CIRCULAR_MODE);
    DDL_ADC_REG_SetTriggerSource(ADC, DDL_ADC_REG_TRIG_SOFTWARE); 
    DDL_ADC_SetChannelSamplingTime(ADC, DDL_ADC_CHANNEL_7, DDL_ADC_SAMPLINGTIME_8_SCYCLES);
    DDL_ADC_SetChannelSamplingTime(ADC, DDL_ADC_CHANNEL_12,DDL_ADC_SAMPLINGTIME_8_SCYCLES);
    DDL_ADC_SetChannelSamplingTime(ADC, DDL_ADC_CHANNEL_3, DDL_ADC_SAMPLINGTIME_8_SCYCLES);
    DDL_ADC_SetChannelSamplingTime(ADC, DDL_ADC_CHANNEL_8, DDL_ADC_SAMPLINGTIME_32_SCYCLES);
    DDL_ADC_SEQ_SetSectionNumber(ADC, DDL_ADC_SEQ_NUMBER_1);
    DDL_ADC_SEQ_SetSectionTimes(ADC, DDL_ADC_SEQ_SECTION_1, DDL_ADC_SEQ_TRS_TIME_3);
    DDL_ADC_REG_SetSequencerRanks(ADC, DDL_ADC_REG_RANK_1, VOLTAGE_ADC_CHANNEL);
    DDL_ADC_REG_SetSequencerRanks(ADC, DDL_ADC_REG_RANK_2, CURRENT_ADC_CHANNEL);
    DDL_ADC_REG_SetSequencerRanks(ADC, DDL_ADC_REG_RANK_3, DDL_ADC_CHANNEL_8);
    //DDL_ADC_REG_SetSequencerRanks(ADC, DDL_ADC_REG_RANK_4, INPUT_ADC_CHANNEL);
    DDL_ADC_SEQ_SetGapTime(ADC,DDL_ADC_GAPTIME_ADCCLK_0);
    DDL_OPA_Enable(OPA,DDL_OPA_CHANNEL_1);
    DDL_ADC_SEQ_Enable(ADC);
    DDL_ADC_ClearFlag_EOS(ADC);
    DDL_ADC_ClearFlag_EOSMP(ADC);
    DDL_ADC_EnableTS(ADC);
    DDL_ADC_StopConversion(ADC);
    DDL_ADC_Enable(ADC);
}

int16_t getConvertedDegrees(uint16_t adcrawtemp)
{
    return -((int32_t)adcrawtemp*8 - 13656)/39 + 25;
}

