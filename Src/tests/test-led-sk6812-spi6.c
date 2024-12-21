// This program tests the functions
// in "led.h"
#include "debug.h"
#include "led.h"
#include "mcu.h"
#include "targets.h"

void wait(uint32_t loops)
{
    for (int i = 0; i < loops; i++)
    {
        asm("nop");
    }
}


int main()
{
    mcu_setup(250);
    led_initialize();
    debug_initialize();

    while(1) {
        for (int i = 0; i < 0xff; i++)
        {
            led_write(i);
            wait(0xffffff);
        }
    }
}