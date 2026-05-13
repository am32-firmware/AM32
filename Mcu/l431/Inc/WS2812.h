#ifndef INC_WS2812_H_
#define INC_WS2812_H_

#include "main.h"

/* This driver is non-blocking (SPI1 + DMA1 CH3): send_LED_RGB returns
 * immediately and motor IRQs are not disturbed during the transfer.
 * main.c gates the LED_PROFILE state machine on this flag so opting
 * into the profile on a bit-bang driver fails to compile. */
#define WS2812_NONBLOCKING 1

void WS2812_Init(void);
void send_LED_RGB(uint8_t red, uint8_t green, uint8_t blue);

#endif /* INC_WS2812_H_ */
