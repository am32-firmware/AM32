// Test the gpio pins used for auxilliary
// spi port
// (just toggle gpio pins, not the spi peripheral)
#include "functions.h"
#include "gpio.h"
#include "mcu.h"
#include "targets.h"
#include "utility-timer.h"

void testgpio(gpio_t* gpio)
{
    gpio_reset(gpio);
    delayMillis(2);
    gpio_set(gpio);
    delayMillis(2);
    gpio_reset(gpio);
}

int main()
{
    mcu_setup(250);

    utility_timer_initialize();
    utility_timer_enable();

    gpio_t gpioMainP2 = DEF_GPIO(
        MAIN_P2_PORT,
        MAIN_P2_PIN,
        0,
        GPIO_OUTPUT);
    gpio_t gpioMainP3 = DEF_GPIO(
        MAIN_P3_PORT,
        MAIN_P3_PIN,
        0,
        GPIO_OUTPUT);

    gpio_initialize(&gpioMainP2);
    gpio_initialize(&gpioMainP3);

    gpio_set_speed(&gpioMainP2, GPIO_SPEED_VERYFAST);
    gpio_set_speed(&gpioMainP3, GPIO_SPEED_VERYFAST);

    while(1) {
        testgpio(&gpioMainP2);
        testgpio(&gpioMainP3);
    }
}