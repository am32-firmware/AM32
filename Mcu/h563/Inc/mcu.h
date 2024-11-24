#pragma once

#include <inttypes.h>

void mcu_setup(uint16_t coreFrequencyMHz);
void mcu_enable_icache();
void mcu_setup_flash();
void mcu_setup_clocks(uint16_t coreFrequencyMHz);
void mcu_setup_core_voltage();
void mcu_setup_mpu();
