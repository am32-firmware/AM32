#pragma once

#include <stdbool.h>
#include <stdint.h>

void flash_lock();
void flash_unlock();
bool flash_busy();
bool flash_dbne();
void flash_erase_sector(uint8_t sector);
void flash_program_word(uint16_t word, uint32_t address);
bool flash_wbne();
void flash_enable_write();
void flash_disable_write();

void flash_enable_prefetch();
void flash_set_latency(uint8_t ws);
void flash_set_wrhfreq(uint8_t wrhf);


