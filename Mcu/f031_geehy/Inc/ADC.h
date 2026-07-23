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

void ADC_DMA_Callback(void);
void enableADC_DMA(void);
void activateADC(void);
void ADC_Init(void);

void Configure_DMA(void);

void Configure_ADC(void);

void Activate_ADC(void);

int16_t getConvertedDegrees(uint16_t adcrawtemp);

#endif /* ADC_H_ */
