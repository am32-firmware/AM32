/*
 * ADC.h
 *
 *  Created on: May 20, 2020
 *      Author: Alka
 */

#ifndef ADC_H_
#define ADC_H_

#include "main.h"
#include "targets.h"

void ADC_DMA_Callback();
void enableADC_DMA();
void activateADC();
#ifdef WCH
// the WCH SPL owns the name ADC_Init(ADC_TypeDef*, ADC_InitTypeDef*)
// (ch32v20x_adc.h), so the CH32 port keeps its ADCInit name
void ADCInit(void);
#else
void ADC_Init(void);
#endif
void startADCConversion(void);
int16_t getConvertedDegrees(uint16_t adcrawtemp);
int16_t getNTCDegrees(uint16_t ntcrawtemp);
int16_t computeTemperature(uint16_t raw_temp_val1, uint16_t raw_temp_val2);

#ifdef NXP
uint32_t LPADC_GetGainConvResult(float gainAdjustment);
void calibADC(void);
void enableADC(void);
void initADC(void);
#endif

#endif /* ADC_H_ */
