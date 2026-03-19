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
void startADCConversion();
int16_t getConvertedDegrees(uint16_t adcrawtemp);
int16_t getNTCDegrees(uint16_t ntcrawtemp);
#endif /* ADC_H_ */
