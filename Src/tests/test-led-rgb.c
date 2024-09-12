#include "led.h"
#include "inttypes.h"
uint32_t index = 0;
uint32_t rgb[] = {
    LED_RGB_R,
    LED_RGB_G,
    LED_RGB_B
};

int main()
{
    led_initialize();
    led_on();

    while(1) {
        for(int i = 0; i < 0x2fffff; i++) {
            asm("nop");
        }
        
        led_write(rgb[index]);
        index = index + 1;
        index = index%3;
    }
}
