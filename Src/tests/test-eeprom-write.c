// this test writes some bytes to eeprom
// use a memory inspection tool to verify the results

// the flash sector is erased, then:
// then the elements of data are written to flash

// for h563 with option bytes configuration as
// EDATA1 = 1 (enabled)
// ESTART = 0x0 (1 page remapped)
// the last sector (127) (counting up) is mapped as the
// first page of high-cycle flash memory located at
// 0x0900a800 (counting down)
// if eight EDATA sectors are enabled (ESTART = 7)
// then EDATA[7] is located at 0x09000000

#include "stm32h563xx.h"
#include "targets.h"
#include "eeprom.h"

#define DATA_SIZE 0x100
uint8_t data[DATA_SIZE];

void fill_data()
{
    for (int i = 0 ; i < DATA_SIZE; i++)
    {
        data[i] = i;
    }
}

int main()
{
    fill_data();
    save_flash_nolib(data, sizeof(data), EEPROM_START_ADD);

    while(1) {
    }
}