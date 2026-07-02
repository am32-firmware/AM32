/*
 * eeprom_gd_at32.h
 *
 * Shared GD32/AT32 flash driver body (save_flash_nolib / read_flash_bin),
 * word programming through the vendor flash library, configured by macros
 * from the including Mcu/{e230,f421}/Src/eeprom.c. Not a standalone
 * translation unit.
 *
 * Required macros:
 *   EEPROM_FLASH_UNLOCK()          unlock the flash controller
 *   EEPROM_PAGE_ERASE(add)         erase the page/sector containing add
 *   EEPROM_WORD_PROGRAM(addr, val) program one 32-bit word
 *   EEPROM_CLEAR_FLAGS()           clear the controller status flags
 *   EEPROM_FLASH_LOCK()            lock the flash controller
 */

void save_flash_nolib(uint8_t* data, int length, uint32_t add)
{
    uint32_t data_to_FLASH[length / 4];
    memset(data_to_FLASH, 0, length / 4);
    for (int i = 0; i < length / 4; i++) {
        data_to_FLASH[i] = data[i * 4 + 3] << 24 | data[i * 4 + 2] << 16 | data[i * 4 + 1] << 8 | data[i * 4]; // make 16 bit
    }
    volatile uint32_t data_length = length / 4;

    // unlock flash

    EEPROM_FLASH_UNLOCK();

    // erase page if address even divisable by 1024
    if ((add % 1024) == 0) {
        EEPROM_PAGE_ERASE(add);
    }

    volatile uint32_t index = 0;
    while (index < data_length) {
        EEPROM_WORD_PROGRAM(add + (index * 4), data_to_FLASH[index]);
        EEPROM_CLEAR_FLAGS();
        index++;
    }
    EEPROM_FLASH_LOCK();
}

void read_flash_bin(uint8_t* data, uint32_t add, int out_buff_len)
{
    for (int i = 0; i < out_buff_len; i++) {
        data[i] = *(uint8_t*)(add + i);
    }
}
