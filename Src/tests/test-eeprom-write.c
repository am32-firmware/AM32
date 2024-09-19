// this test writes some bytes to eeprom
// use a memory inspection tool to verify the results

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

uint8_t data[10] = {
    0xff,
    0x00,
    0x55,
    0xae,
    0xab,
    0xcd,
    0xef,
    0x12,
    0x55,
    0x34,
};

int main()
{
    save_flash_nolib(data, 10, EEPROM_START_ADD);

    while(1) {
    }
}