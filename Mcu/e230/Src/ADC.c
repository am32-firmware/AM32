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

void ADC_Init(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_ADC);
    rcu_periph_clock_enable(RCU_DMA);
    rcu_adc_clock_config(RCU_ADCCK_APB2_DIV6);

    /* config the GPIO as analog mode */
    gpio_mode_set(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE,
        VOLTAGE_ADC_PIN | CURRENT_ADC_PIN);

    dma_parameter_struct dma_data_parameter;

    /* ADC DMA_channel configuration */
    dma_deinit(DMA_CH0);

    /* initialize DMA single data mode */
    dma_data_parameter.periph_addr = (uint32_t)(&ADC_RDATA);
    dma_data_parameter.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_data_parameter.memory_addr = (uint32_t)(&ADCDataDMA);
    dma_data_parameter.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_data_parameter.periph_width = DMA_PERIPHERAL_WIDTH_16BIT;
    dma_data_parameter.memory_width = DMA_MEMORY_WIDTH_16BIT;
    dma_data_parameter.direction = DMA_PERIPHERAL_TO_MEMORY;
    dma_data_parameter.number = 3U;
    dma_data_parameter.priority = DMA_PRIORITY_LOW;
    dma_init(DMA_CH0, &dma_data_parameter);

    dma_circulation_enable(DMA_CH0);

    /* enable DMA channel */
    dma_channel_enable(DMA_CH0);

    adc_special_function_config(ADC_CONTINUOUS_MODE, DISABLE);
    /* ADC scan function enable */
    adc_special_function_config(ADC_SCAN_MODE, ENABLE);
    /* ADC data alignment config */
    adc_data_alignment_config(ADC_DATAALIGN_RIGHT);
    /* ADC channel length config */
    adc_channel_length_config(ADC_REGULAR_CHANNEL, 3U);

    adc_tempsensor_vrefint_enable(); // enable the temp sensor

    /* ADC regular channel config */
    adc_regular_channel_config(0, VOLTAGE_ADC_CHANNEL, ADC_SAMPLETIME_55POINT5);
    adc_regular_channel_config(1, CURRENT_ADC_CHANNEL, ADC_SAMPLETIME_55POINT5);
    adc_regular_channel_config(2, ADC_CHANNEL_16, ADC_SAMPLETIME_55POINT5);

    /* ADC trigger config */
    adc_external_trigger_source_config(ADC_REGULAR_CHANNEL,
        ADC_EXTTRIG_REGULAR_NONE);
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
