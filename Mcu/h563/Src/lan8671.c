#include "lan8671.h"

#include "gpio.h"

void lan8671_shutdown()
{
    gpio_t gpioLANnRST = DEF_GPIO(GPIOG, 0, 0, GPIO_OUTPUT);
    gpio_initialize(&gpioLANnRST);
    gpio_reset(&gpioLANnRST);
}
