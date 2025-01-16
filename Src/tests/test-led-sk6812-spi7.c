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

int main()
{
    mcu_setup(250);
    led_initialize();

    while(1) {
        for (int i = 0; i < 0xff; i++)
        {
            led_write(0x2);
            wait(0xffffff);
            led_write(0x1);
            wait(0xffffff);
        }
    }
}