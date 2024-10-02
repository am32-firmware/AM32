#include "led.h"
#include "mcu.h"
// this tests blinking the led using the maximum clock frequency
// h563 = 250MHz

int main()
{
    mcu_setup();

    led_initialize();
    led_on();

    while(1) {
        for(int i = 0; i < 0x2ffff; i++) {
            asm("nop");
        }
        led_toggle();
    }
}
