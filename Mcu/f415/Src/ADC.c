/*
 * ADC.c
 *
 *  Created on: May 20, 2020
 *      Author: Alka
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
{
  // read dma buffer and set extern variables

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
  dma_init_type dma_init_struct;
  crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
  dma_flexible_config(DMA1,FLEX_CHANNEL1,DMA_FLEXIBLE_ADC1);
  nvic_irq_enable(DMA1_Channel1_IRQn, 2, 0);
  dma_reset(DMA1_CHANNEL1);
  dma_default_para_init(&dma_init_struct);
  dma_init_struct.buffer_size = 3;
  dma_init_struct.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
  dma_init_struct.memory_base_addr = (uint32_t)&ADCDataDMA;
  dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;
  dma_init_struct.memory_inc_enable = TRUE;
  dma_init_struct.peripheral_base_addr = (uint32_t) & (ADC1->odt);
  dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
  dma_init_struct.peripheral_inc_enable = FALSE;
  dma_init_struct.priority = DMA_PRIORITY_HIGH;
  dma_init_struct.loop_mode_enable = TRUE;
  dma_init(DMA1_CHANNEL1, &dma_init_struct);

  dma_interrupt_enable(DMA1_CHANNEL1, DMA_FDT_INT, TRUE);
  dma_channel_enable(DMA1_CHANNEL1, TRUE);

  adc_base_config_type adc_base_struct;
  crm_periph_clock_enable(CRM_ADC1_PERIPH_CLOCK, TRUE);
  crm_adc_clock_div_set(CRM_ADC_DIV_6);

  adc_base_default_para_init(&adc_base_struct);
  adc_base_struct.sequence_mode = TRUE;
  adc_base_struct.repeat_mode = TRUE;
  adc_base_struct.data_align = ADC_RIGHT_ALIGNMENT;
  adc_base_struct.ordinary_channel_length = 3;
  adc_base_config(ADC1, &adc_base_struct);

  adc_ordinary_channel_set(ADC1, ADC_CHANNEL_3, 1, ADC_SAMPLETIME_28_5);
  adc_ordinary_channel_set(ADC1, ADC_CHANNEL_6, 2, ADC_SAMPLETIME_28_5);
  adc_ordinary_channel_set(ADC1, ADC_CHANNEL_16, 3, ADC_SAMPLETIME_28_5);

  adc_tempersensor_vintrv_enable(TRUE);
  adc_ordinary_conversion_trigger_set(ADC1, ADC12_ORDINARY_TRIG_SOFTWARE, TRUE);

  adc_dma_mode_enable(ADC1, TRUE);

  adc_enable(ADC1, TRUE);
  adc_calibration_init(ADC1);
  while (adc_calibration_init_status_get(ADC1))
    ;
  adc_calibration_start(ADC1);
  while (adc_calibration_status_get(ADC1))
    ;
}

void startADCConversion()
{
  adc_ordinary_software_trigger_enable(ADC1, TRUE);
}
int16_t getConvertedDegrees(uint16_t adcrawtemp)
{
  return (12600 - (int32_t)adcrawtemp * 33000 / 4096) / -42 + 5;
}

#endif // USE_ADC