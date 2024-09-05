/*
 * ADC.h
 *
 *  Created on: May 20, 2020
 *      Author: Alka
 */

#include "main.h"
#include "stm32h563xx.h"
#include "targets.h"
#include "stdbool.h"
#ifndef ADC_H_
#define ADC_H_

void ADC_DMA_Callback();
void enableADC_DMA();
void activateADC();
void ADC_Init(void);

void adc_initialize(ADC_TypeDef* adc);
void adc_set_regular_sequence(ADC_TypeDef* adc, uint8_t* channels, uint8_t length);
void adc_enable(ADC_TypeDef* adc);
void adc_start(ADC_TypeDef* adc);
void adc_set_continuous_mode(ADC_TypeDef* adc, bool enabled);
void adc_set_sample_time(ADC_TypeDef* adc, uint8_t channel, uint8_t sample_time);
#endif /* ADC_H_ */
