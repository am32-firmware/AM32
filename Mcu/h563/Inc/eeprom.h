#pragma once

#include <stdbool.h>

#include "main.h"


// void save_to_flash(uint8_t *data);
// void read_flash(uint8_t* data, uint32_t address);
// void save_to_flash_bin(uint8_t *data, int length, uint32_t add);
void read_flash_bin(uint8_t* data, uint32_t add, int out_buff_len);
void save_flash_nolib(uint8_t* data, int length, uint32_t add);
void flash_lock();
void flash_unlock();
bool flash_busy();
bool flash_dbne();
void flash_erase_sector(uint8_t sector);
void flash_program_word(uint16_t word, uint32_t address);
bool flash_wbne();
void flash_enable_write();
void flash_disable_write();



