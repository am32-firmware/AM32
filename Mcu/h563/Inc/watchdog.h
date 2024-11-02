#pragma once

#include <inttypes.h>

void watchdog_initialize();
void watchdog_enable(uint32_t period);
void watchdog_goodboy_pat();
