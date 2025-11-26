
#include "ADC.h"
#include "ntc_tables.h"
#include "functions.h"
#include "targets.h"

#ifdef USE_ADC

#ifdef PA6_NTC_ONLY
uint16_t ADCDataDMA[1];
#else
#if defined(USE_ADC_INPUT) || defined(USE_NTC)
uint16_t ADCDataDMA[5];
#else
uint16_t ADCDataDMA[4];
#endif
#endif

extern uint16_t ADC_raw_temp;
extern uint16_t ADC_raw_volts;
extern uint16_t ADC_raw_current;
extern uint16_t ADC_raw_input;
extern uint16_t ADC_raw_ntc;

void ADC_DMA_Callback()
{ // read dma buffer and set extern variables
#ifdef PA6_NTC_ONLY
    ADC_raw_temp = ADCDataDMA[0];
#else
#ifdef USE_ADC_INPUT
    ADC_raw_temp = ADCDataDMA[3];
    ADC_raw_volts = ADCDataDMA[1] / 2;
    ADC_raw_current = ADCDataDMA[2];
    ADC_raw_input = ADCDataDMA[0];
#else
  #ifdef USE_NTC
    ADC_raw_ntc = ADCDataDMA[4];
    ADC_raw_temp = ADCDataDMA[3];
    ADC_raw_volts = ADCDataDMA[0];
    ADC_raw_current = ADCDataDMA[1];
  #else
     ADC_raw_temp = ADCDataDMA[3];
    ADC_raw_volts = ADCDataDMA[0];
    ADC_raw_current = ADCDataDMA[1];
  #endif
#endif
#endif
}

void ADC_Init(void)
{
#ifdef PA2_VOLTAGE
    gpio_mode_QUICK(GPIOA, GPIO_MODE_ANALOG, GPIO_PULL_NONE, GPIO_PINS_2);
#else
    gpio_mode_QUICK(GPIOA, GPIO_MODE_ANALOG, GPIO_PULL_NONE, CURRENT_ADC_PIN);
#endif
    gpio_mode_QUICK(GPIOA, GPIO_MODE_ANALOG, GPIO_PULL_NONE, VOLTAGE_ADC_PIN);
#ifdef USE_NTC
    gpio_mode_QUICK(GPIOA, GPIO_MODE_ANALOG, GPIO_PULL_NONE, NTC_ADC_PIN);
#endif 


	dma_init_type dma_init_struct;
    crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
    // nvic_irq_enable(DMA1_Channel1_IRQn, 3, 0);
    dma_reset(DMA1_CHANNEL1);
    dma_default_para_init(&dma_init_struct);
#ifdef USE_NTC
    dma_init_struct.buffer_size = 5;
#else
    dma_init_struct.buffer_size = 4;
#endif
    dma_init_struct.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
    dma_init_struct.memory_base_addr = (uint32_t)&ADCDataDMA;
    dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;
    dma_init_struct.memory_inc_enable = TRUE;
    dma_init_struct.peripheral_base_addr = (uint32_t)&ADC1->odt;
    dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
    dma_init_struct.peripheral_inc_enable = FALSE;
    dma_init_struct.priority = DMA_PRIORITY_HIGH;
    dma_init_struct.loop_mode_enable = TRUE;
    dma_init(DMA1_CHANNEL1, &dma_init_struct);

    // dma_interrupt_enable(DMA1_CHANNEL1, DMA_FDT_INT, TRUE);
    dma_channel_enable(DMA1_CHANNEL1, TRUE);

    adc_base_config_type adc_base_struct;
    crm_periph_clock_enable(CRM_ADC1_PERIPH_CLOCK, TRUE);
    crm_adc_clock_div_set(CRM_ADC_DIV_16);

    adc_base_default_para_init(&adc_base_struct);
    adc_base_struct.sequence_mode = TRUE;
    adc_base_struct.repeat_mode = TRUE;
    adc_base_struct.data_align = ADC_RIGHT_ALIGNMENT;
#ifdef USE_NTC
    adc_base_struct.ordinary_channel_length = 5;
    adc_base_config(ADC1, &adc_base_struct);
    adc_ordinary_channel_set(ADC1, VOLTAGE_ADC_CHANNEL, 1, ADC_SAMPLETIME_28_5);
    adc_ordinary_channel_set(ADC1, CURRENT_ADC_CHANNEL, 2, ADC_SAMPLETIME_28_5);
    adc_ordinary_channel_set(ADC1, ADC_CHANNEL_17, 3, ADC_SAMPLETIME_28_5);
    adc_ordinary_channel_set(ADC1, TEMP_ADC_CHANNEL, 4, ADC_SAMPLETIME_239_5);
    adc_ordinary_channel_set(ADC1, NTC_ADC_CHANNEL, 5, ADC_SAMPLETIME_28_5);
#else
    adc_base_struct.ordinary_channel_length = 4;
    adc_base_config(ADC1, &adc_base_struct);
    adc_ordinary_channel_set(ADC1, VOLTAGE_ADC_CHANNEL, 1, ADC_SAMPLETIME_28_5);
    adc_ordinary_channel_set(ADC1, CURRENT_ADC_CHANNEL, 2, ADC_SAMPLETIME_28_5);
    adc_ordinary_channel_set(ADC1, ADC_CHANNEL_17, 3, ADC_SAMPLETIME_28_5);
    adc_ordinary_channel_set(ADC1, TEMP_ADC_CHANNEL, 4, ADC_SAMPLETIME_239_5);
#endif
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

#ifdef USE_NTC
int16_t getNTCDegrees(uint16_t ntcrawtemp){
  int p1,p2;
  p1 = NTC_table[ (ntcrawtemp >> 6)  ];
  p2 = NTC_table[ (ntcrawtemp >> 6)+1];
  return p1 - ( (p1-p2) * (ntcrawtemp & 0x003F) ) / 64;
}
#endif

int16_t getConvertedDegrees(uint16_t adcrawtemp)
{
    return (12800 - (int32_t)adcrawtemp * 33000 / 4096) / -42 + 25;
}

#endif // USE_ADC
