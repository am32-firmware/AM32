#include "led.h"
// this tests blinking the led using the default clock frequency
// h563 = 64MHz

int main()
{
    led_initialize();
    led_on();

    while(1) {
        for(int i = 0; i < 0x2ffff; i++) {
            asm("nop");
        }
        led_toggle();
    }
}