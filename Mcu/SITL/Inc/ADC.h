/*
 * ADC.h - SITL
 */

#include "main.h"
#include "targets.h"

#ifndef ADC_H_
#define ADC_H_

void ADC_DMA_Callback(void);
void enableADC_DMA(void);
void activateADC(void);
void ADC_Init(void);

#endif /* ADC_H_ */
