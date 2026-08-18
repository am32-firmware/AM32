/*
 * ADC.h
 *
 *  Created on: Aug. 4, 2026
 *      Author: Nong jun
 */
 
#include "main.h"
#include "targets.h"

#ifndef ADC_H_
#define ADC_H_

void ADC_DMA_Callback(void);
void ADC_Init(void);
int16_t getConvertedDegrees(uint16_t adcrawtemp);

#endif /* ADC_H_ */
