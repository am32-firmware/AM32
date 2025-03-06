#include "can.h"

#include "gpio.h"
#include "targets.h"

gpio_t gpioCANEnable = DEF_GPIO(CAN_STBY_GPIO_PORT, CAN_STBY_GPIO_PIN, 0, GPIO_OUTPUT); // blueESC

void can_disable()
{
    gpio_set(&gpioCANEnable);
}

void can_enable()
{
    gpio_reset(&gpioCANEnable);
}

void can_initialize()
{
    gpio_initialize(&gpioCANEnable);
    can_disable();
}
