/*
 * mcxa13x_it.h
 *
 *  Created on: 14 Nov 2024
 *      Author: Youri
 */

#ifndef MCU_A13X_INC_MCXA13X_IT_H_
#define MCU_A13X_INC_MCXA13X_IT_H_

#include "main.h"

void GPIO1_IRQHandler(void);
void GPIO2_IRQHandler(void);

void CMP0_IRQHandler(void);
void CMP1_IRQHandler(void);
void LPTMR0_IRQHandler(void);
void DMA_CH0_IRQHandler(void);
void DMA_CH1_IRQHandler(void);
void LPSPI0_IRQHandler(void);

#endif /* MCU_A13X_INC_MCXA13X_IT_H_ */
