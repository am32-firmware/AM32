// This program tests the functions
// in "led.h"
#include "targets.h"
#include "mcu.h"
#include "led.h"

int main()
{
    mcu_setup(250);
    led_initialize();

    while(1) {
        led_write(0x00100000);
        led_write(0x00001000);
        led_write(0x00000010);
    }
}