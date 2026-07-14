/*
 * ADC.h
 *
 *  Created on: May 20, 2020
 *      Author: Alka
 */

#include "main.h"
#include "targets.h"

#ifndef ADC_H_
#define ADC_H_

void ADC_DMA_Callback();
void enableADC_DMA();
void activateADC();
void ADC_Init(void);

void Configure_DMA();

void Configure_ADC();

void Activate_ADC();

#ifdef USE_LMT87
int16_t getLMT87Degrees(uint16_t adc_raw);
#endif
int16_t getNTCDegrees(uint16_t ntcrawtemp);
#ifndef USE_TIMEOUT
#define USE_TIMEOUT 0
#endif

#endif /* ADC_H_ */
