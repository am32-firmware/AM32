#pragma once

#include <stdint.h>

void read_flash_bin_hc(uint8_t* data, uint32_t add, int out_buff_len);
void save_flash_nolib_hc(uint8_t* data, int length, uint32_t add);
