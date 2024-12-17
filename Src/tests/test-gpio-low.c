// this test drives a gpio low

#include "gpio.h"

gpio_t gpio = DEF_GPIO(GPIOD, 12, 0, GPIO_OUTPUT);

int main()
{
    gpio_initialize(&gpio);
    gpio_reset(&gpio);

    // do nothing
    while (1);
}