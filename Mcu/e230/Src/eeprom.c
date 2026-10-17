#include "eeprom.h"

#include <string.h>

#include "gd32e23x_fmc.h"

// #define APP_START (uint32_t)0x08001000
// #define FLASH_STORAGE 0x08005000  // at the 31kb mark
#define page_size 0x400 // 1 kb for f051
// uint32_t FLASH_FKEY1 =0x45670123;
// uint32_t FLASH_FKEY2 =0xCDEF89AB;

// wait for the flash controller, returns 1 when ready without errors (as fmc_ready_wait())
static uint8_t flashReady(void)
{
    uint32_t timeout = FMC_TIMEOUT_COUNT;
    while ((FMC_STAT & FMC_STAT_BUSY) && --timeout) {
    }
    return !(FMC_STAT & (FMC_STAT_BUSY | FMC_STAT_WPERR | FMC_STAT_PGERR));
}

void save_flash_nolib(uint8_t* data, int length, uint32_t add)
{ /// todo
    uint32_t data_to_FLASH[length / 4];
    memset(data_to_FLASH, 0, length / 4);
    for (int i = 0; i < length / 4; i++) {
        data_to_FLASH[i] = data[i * 4 + 3] << 24 | data[i * 4 + 2] << 16 | data[i * 4 + 1] << 8 | data[i * 4]; // make 16 bit
    }
    volatile uint32_t data_length = length / 4;

    // unlock flash

    if (FMC_CTL & FMC_CTL_LK) {
        FMC_KEY = UNLOCK_KEY0;
        FMC_KEY = UNLOCK_KEY1;
    }

    // erase page if address even divisable by 1024
    if ((add % 1024) == 0) {
        if (flashReady()) {
            FMC_CTL |= FMC_CTL_PER;
            FMC_ADDR = add;
            FMC_CTL |= FMC_CTL_START;
            flashReady();
            FMC_CTL &= ~FMC_CTL_PER;
        }
    }

    volatile uint32_t index = 0;
    while (index < data_length) {
        if (flashReady()) {
            FMC_CTL |= FMC_CTL_PG;
            REG32(add + (index * 4)) = data_to_FLASH[index];
            flashReady();
            FMC_CTL &= ~FMC_CTL_PG;
        }
        FMC_STAT = FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGERR;
        index++;
    }
    FMC_CTL |= FMC_CTL_LK;
}

void read_flash_bin(uint8_t* data, uint32_t add, int out_buff_len)
{
    // volatile uint32_t read_data;
    for (int i = 0; i < out_buff_len; i++) {
        data[i] = *(uint8_t*)(add + i);
    }
}
