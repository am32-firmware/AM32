#pragma once
#include <inttypes.h>

// uint32_t HCLK_FREQUENCY = 32000000;

void clock_hse_enable();
// 00: division by 1, hsi_ck, hsi_ker_ck = 64 MHz
// 01: division by 2, hsi_ck, hsi_ker_ck = 32 MHz (default after reset)
// 10: division by 4, hsi_ck, hsi_ker_ck = 16 MHz
// 11: division by 8, hsi_ck, hsi_ker_ck = 8 MHz
#define CLOCK_HSI_DIV1 (0b00)
#define CLOCK_HSI_DIV2 (0b01)
#define CLOCK_HSI_DIV4 (0b10)
#define CLOCK_HSI_DIV8 (0b11)


void clock_hsi_config_divider(uint8_t hsidiv);
#define CLOCK_PLL1_SRC_HSE (0b11)
void clock_pll1_set_source(uint8_t source);
