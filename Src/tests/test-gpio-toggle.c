// this test drives a gpio low

#include "gpio.h"
#include "mcu.h"
#include "targets.h"

gpio_t gpio = DEF_GPIO(LED_R_GPIO_PORT, LED_R_GPIO_PIN, 0, GPIO_OUTPUT);
gpio_t gpio2 = DEF_GPIO(BUTTON_GPIO_PORT, BUTTON_GPIO_PIN, 0, GPIO_OUTPUT);

int main()
{
    mcu_setup(250);
    gpio_initialize(&gpio);
    gpio_reset(&gpio);
    gpio_initialize(&gpio2);
    gpio_reset(&gpio2);

    // do nothing
    while (1) {
        for (int i = 0; i < 0xffffff; i++) {
            asm("nop");
        }
        gpio_toggle(&gpio);
    }
}