/*
 *   modified by TempersLee June 21,2024 for CH32V203
 * */

#include "eeprom.h"
#include <string.h>
#include "debug.h"

#define page_size 0x100 // 256 byte pages for v203

void save_flash_nolib(uint8_t* data, int length, uint32_t add)
{
    if ((add & 0x03) != 0 || (length & 0x03) != 0 || (length+(add&0xFF)) > page_size) {
        return;
    }

    FLASH_Unlock_Fast();

    uint32_t flash_buffer[page_size/4];
    const uint8_t page_offset = add & 0xFFU;
    const uint32_t page_base = add & ~0xFFU;

    // get existing data
    memcpy(flash_buffer, (void*)page_base, page_size);

    // overwrite with new data
    memcpy(&flash_buffer[page_offset/4], data, length);

    // fast erase is 256 bytes at a time, normal Flash_Erasepage
    // is 4k at a time
    FLASH_ErasePage_Fast(page_base);

    FLASH_ProgramPage_Fast(page_base, flash_buffer);

    FLASH_Lock_Fast();
}

void read_flash_bin(uint8_t* data, uint32_t add, int out_buff_len)
{
    memcpy(data, (const uint8_t *)add, out_buff_len);
}
