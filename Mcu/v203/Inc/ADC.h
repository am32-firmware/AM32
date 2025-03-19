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
void ADCInit(void);
void startADCConversion();
int16_t getConvertedDegrees(uint16_t adcrawtemp);

#endif /* ADC_H_ */
