#include "led.h"
#include "inttypes.h"
uint32_t value = 0;
int main()
{
    led_initialize();
    led_on();

    while(1) {
        for(int i = 0; i < 0x2ffffff; i++) {
            asm("nop");
        }
        
        led_write(value);
        value = value << 8;
    }
}
