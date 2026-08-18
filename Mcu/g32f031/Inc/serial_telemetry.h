/*
 * serial_telemetry.h
 *
 *  Created on: Aug. 4, 2026
 *      Author: Nong jun
 */

#include "main.h"

#ifndef SERIAL_TELEMETRY_H_
#define SERIAL_TELEMETRY_H_

void telem_UART_Init(void);
void send_telem_DMA(uint8_t bytes);

#endif /* SERIAL_TELEMETRY_H_ */
