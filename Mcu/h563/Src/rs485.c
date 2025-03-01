#include "rs485.h"

#include "gpio.h"
#include "targets.h"

gpio_t gpioRS485Enable = DEF_GPIO(RS485_USART_ENABLE_PORT, RS485_USART_ENABLE_PIN, 0, GPIO_OUTPUT); // blueESC

void rs485_disable()
{
    gpio_reset(&gpioRS485Enable);
}

void rs485_enable()
{
    gpio_set(&gpioRS485Enable);
}

void rs485_initialize()
{
    gpio_initialize(&gpioRS485Enable);
    rs485_disable();
}
