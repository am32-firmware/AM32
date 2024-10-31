#pragma once
#include "main.h"
#include "stm32h563xx.h"
#include "targets.h"
#include "stdbool.h"

void ADC_DMA_Callback();

void ADC_setup();
void adc_initialize(ADC_TypeDef* adc);
void adc_set_regular_sequence(ADC_TypeDef* adc, uint8_t* channels, uint8_t length);
void adc_enable(ADC_TypeDef* adc);
void adc_start(ADC_TypeDef* adc);
void adc_set_continuous_mode(ADC_TypeDef* adc, bool enabled);

typedef enum
{
    ADC_SAMPLE_TIME_2_5 = 0b000,
    ADC_SAMPLE_TIME_6_5 = 0b001,
    ADC_SAMPLE_TIME_12_5 = 0b010,
    ADC_SAMPLE_TIME_24_5 = 0b011,
    ADC_SAMPLE_TIME_47_5 = 0b100,
    ADC_SAMPLE_TIME_92_5 = 0b101,
    ADC_SAMPLE_TIME_247_5 = 0b110,
    ADC_SAMPLE_TIME_640_5 = 0b111,
} adcSampleTime_e;
void adc_set_sample_time(ADC_TypeDef* adc, uint8_t channel, adcSampleTime_e sample_time);
