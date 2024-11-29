#ifndef INC_WS2812_H_
#define INC_WS2812_H_

#include "main.h"
#include <stdint.h>

void WS2812_Init(void);
void send_LED_RGB(uint8_t red, uint8_t green, uint8_t blue);

#endif /* INC_WS2812_H_ */