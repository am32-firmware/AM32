#include "stm32h563xx.h"
#include "targets.h"
#include "eeprom.h"

uint16_t data[10];

#define EEPROM_START_ADD
int main()
{
    read_flash_bin(data, EEPROM_START_ADD, 10);
    for (uint8_t i = 0; i < 10; i++) {
        data[i]++;
    }
    save_flash_nolib(data, 10, EEPROM_START_ADD);

    while(1) {
        // spi_write(&spi, data, 5);
    }
}