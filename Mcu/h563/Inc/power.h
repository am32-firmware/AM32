#pragma once

#include <inttypes.h>

#define POWER_VOSCR_3 (0b00) // default
#define POWER_VOSCR_2 (0b01)
#define POWER_VOSCR_1 (0b10)
#define POWER_VOSCR_0 (0b11)

void power_set_core_voltage(uint8_t vos);