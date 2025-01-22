// this test drives a gpio high

#include "gpio.h"
#include "mcu.h"
#include "targets.h"

gpio_t gpio = DEF_GPIO(LED_R_GPIO_PORT, LED_R_GPIO_PIN, 0, GPIO_OUTPUT);
// gpio_t gpio2 = DEF_GPIO(BUTTON_GPIO_PORT, BUTTON_GPIO_PIN, 0, GPIO_OUTPUT);

int main()
{
    mcu_setup(250);
    gpio_initialize(&gpio);
    gpio_set(&gpio);
    // gpio_initialize(&gpio2);
    // gpio_set(&gpio2);
    // do nothing
    while (1);
}