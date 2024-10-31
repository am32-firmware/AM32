// this test reads 1kB of flash from
// the dedicated eeprom area (EEPROM_START_ADD)

#include "targets.h"
#include "eeprom.h"

uint8_t data[1024];

int main()
{
    read_flash_bin(data, EEPROM_START_ADD, sizeof(data));

    while(1) {
    }
}