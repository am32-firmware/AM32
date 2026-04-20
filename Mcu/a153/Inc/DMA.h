/*
 * DMA.h
 *
 *  Created on: 14 Nov 2024
 *      Author: Youri
 */

#ifndef MCU_A153_INC_DMA_H_
#define MCU_A153_INC_DMA_H_

#include "main.h"

void initDMA_DshotPWM(void);
void initDMA_ADC(void);
void initDMA_UART(void);

void enableDMA_DshotPWM(void);
void enableDMA_ADC(void);
void enableDMA_UART(void);

void doDshotCorrection(void);

#endif /* MCU_A153_INC_DMA_H_ */
