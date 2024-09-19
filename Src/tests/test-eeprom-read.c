// this test writes some bytes to eeprom
// use a memory inspection tool to verify the results

#include "stm32h563xx.h"
#include "targets.h"
#include "eeprom.h"

// uint8_t data[10] = {
//     0xff00,
//     0x00ff,
//     0x5555,
//     0xff55,
//     0x55ff,
//     0xabcd,
//     0x6789,
//     0x1234,
//     0xeaea,
//     0xabab,
// };

uint8_t data[10] = 
{
    0,0,0,0,0,0,0,0,0,0
};

// first bank of high-cycle flash
// #define EEPROM_START_ADD 0x09000000;
int main()
{
    read_flash_bin(data, EEPROM_START_ADD, 10);

    while(1) {
        // spi_write(&spi, data, 5);
    }
}