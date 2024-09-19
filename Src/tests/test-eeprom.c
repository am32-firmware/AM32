#include "stm32h563xx.h"
#include "targets.h"
#include "eeprom.h"

uint8_t data[10];

// first bank of high-cycle flash
// #define EEPROM_START_ADD 0x09000000;
int main()
{
    read_flash_bin(data, EEPROM_START_ADD, 10);
    for (uint8_t i = 0; i < 10; i++) {
        data[i] += i;
    }

    save_flash_nolib(data, 10, EEPROM_START_ADD);

    while(1) {
    }
}