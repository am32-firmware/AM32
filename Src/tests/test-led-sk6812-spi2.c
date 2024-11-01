// This program tests the functions
// in "led.h"
#include "targets.h"
#include "mcu.h"
#include "led.h"

int main()
{
    mcu_setup();
    led_initialize();

    while(1) {
        led_write(0x00100000);
        for (uint32_t i = 0; i < 0xffffff; i++) {
            asm("nop");
        }
        led_write(0x00001000);
        for (uint32_t i = 0; i < 0xffffff; i++) {
            asm("nop");
        }
        led_write(0x00000010);
        for (uint32_t i = 0; i < 0xffffff; i++) {
            asm("nop");
        }
    }
}