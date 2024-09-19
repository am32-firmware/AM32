#include "stm32h563xx.h"
#include "targets.h"
#include "eeprom.h"

// first bank of high-cycle flash
// #define EEPROM_START_ADD 0x09000000;
int main()
{
    // read_flash_bin(data, EEPROM_START_ADD, 10);
    // for (uint8_t i = 0; i < 10; i++) {
    //     data[i]++;
    // }

    // flash_erase_sector(120);
    // flash_erase_sector(121);
    // flash_erase_sector(122);
    // flash_erase_sector(123);
    // flash_erase_sector(124);
    // flash_erase_sector(125);
    // flash_erase_sector(126);
    flash_erase_sector(127);
    // save_flash_nolib(data, 10, EEPROM_START_ADD);

    while(1) {
    }
}