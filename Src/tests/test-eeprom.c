
// each position in the array adds it's index to the
// value on reset
// a debugger can be used to verify that the elements
// of data[] increase after each reset

#include "stm32h563xx.h"
#include "targets.h"
#include "eeprom.h"

// FLASH is only committed after an internal 128bit write buffer is
// filled (16 bytes)
#define DATA_SIZE 16
uint8_t data[DATA_SIZE];

int main()
{
    // read eeprom
    read_flash_bin(data, EEPROM_START_ADD, sizeof(data));

    // modify the data
    for (int i = 0; i < DATA_SIZE; i++) {
        data[i] += i;
    }

    // update eeprom with modified data
    save_flash_nolib(data, DATA_SIZE, EEPROM_START_ADD);

    while(1) {
    }
}