// This program tests the functions
// in "led.h"
#include "led.h"
#include "mcu.h"
#include "targets.h"

void wait(uint32_t loops)
{
    for (uint32_t i = 0; i < loops; i++) {
        // no-operation cpu instruction
        asm("nop");
    }
}

uint8_t rotate_byte(uint8_t byte)
{
    uint8_t b = 0;
    for (uint8_t i = 0; i < 8; i++) {
        if (byte & (1 << i)) {
            b |= (1 << (7 - i));
        }
    }
    return b;
}

int main()
{
    mcu_setup(250);
    led_initialize();

    uint8_t byte = 0x1;
    byte = rotate_byte(byte);

    byte = 0x2;
    byte = rotate_byte(byte);

    while(1) {
        for (int i = 0; i < 0xff; i++)
        {
            led_write(0x1);
            wait(0xffffff);
            led_write(0x2);
            wait(0xffffff);
            led_write(0x100);
            wait(0xffffff);
            led_write(0x200);
            wait(0xffffff);
            led_write(0x10000);
            wait(0xffffff);
            led_write(0x20000);
            wait(0xffffff);
        }
    }
}