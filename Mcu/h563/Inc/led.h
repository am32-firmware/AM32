#pragma once

#include "inttypes.h"

void led_initialize(void);
void led_on(void);
void led_off(void);
void led_toggle(void);
void led_write(uint32_t grb);
