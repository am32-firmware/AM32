#pragma once

#include <inttypes.h>
#include "spi.h"

#define LED_RGB_R 0x000000ff
#define LED_RGB_B 0x0000ff00
#define LED_RGB_G 0x00ff0000

extern spi_t spi;

void led_initialize(void);
void led_on(void);
void led_off(void);
void led_toggle(void);
void led_write(uint32_t grb);
