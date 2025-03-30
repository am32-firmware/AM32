/*
 * serial_telemetry.h
 *
 *  Created on: May 13, 2020
 *      Author: Alka
 *      modified by TempersLee June 21, 2024
 */

#include "main.h"

#ifndef SERIAL_TELEMETRY_H_
#define SERIAL_TELEMETRY_H_

void telem_UART_Init(void);
void send_telem_DMA(uint8_t bytes);

//void telem_UART_Init_CH4(void);
//void send_telem_DMA_CH4();

#endif /* SERIAL_TELEMETRY_H_ */
