// this test drives a gpio low

#include "gpio.h"
#include "mcu.h"

gpio_t gpio = DEF_GPIO(GPIOD, 8, 0, GPIO_OUTPUT);

int main()
{
    mcu_setup(250);
    gpio_initialize(&gpio);
    gpio_reset(&gpio);

    // do nothing
    while (1) {
        for (int i = 0; i < 0xffffff; i++) {
            asm("nop");
        }
        gpio_toggle(&gpio);
    }
}