/*
 * serial_telemetry.h
 *
 *  Created on: May 13, 2020
 *      Author: Alka
 */

#include "main.h"

#ifndef SERIAL_TELEMETRY_H_
#define SERIAL_TELEMETRY_H_

void telem_UART_Init(void);
void send_telem_DMA(uint8_t bytes);

void telem_UART_Init_CH4(void);
void makeInfoPacket(void);

#endif /* SERIAL_TELEMETRY_H_ */
