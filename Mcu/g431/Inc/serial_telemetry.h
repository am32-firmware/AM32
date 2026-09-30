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

#include "ultra.h"
#ifdef ULTRA_DEDICATED
void setBaudRate(uint32_t baud);
#endif

#endif /* SERIAL_TELEMETRY_H_ */
