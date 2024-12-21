// This program tests the functions
// in "led.h"
#include "targets.h"
#include "mcu.h"
#include "led.h"

void wait()
{
    for (int i = 0; i < 0xffffff; i++) {
        asm("nop");
    }
}
int main()
{
    mcu_setup(250);
    led_initialize();

    while(1) {
        led_write_rgb(128, 0, 0);
        wait();
        led_write_rgb(128, 128, 0);
        wait();
        led_write_rgb(0, 128, 0);
        wait();
        led_write_rgb(0, 128, 128);
        wait();
        led_write_rgb(0, 0, 128);
        wait();
        led_write_rgb(128, 0, 128);
        wait();
    }
}