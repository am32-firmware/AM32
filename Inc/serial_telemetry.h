/*
 * serial_telemetry.h
 *
 *  Created on: May 13, 2020
 *      Author: Alka
 */

#ifndef SERIAL_TELEMETRY_H_
#define SERIAL_TELEMETRY_H_

#include "main.h"

void telem_UART_Init(void);
void send_telem_DMA(uint8_t bytes);
void enable_telem_UART(void); // MCXA153 only

#endif /* SERIAL_TELEMETRY_H_ */
