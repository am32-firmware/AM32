// Test the gpio pins used for auxilliary
// spi port
// (just toggle gpio pins, not the spi peripheral)
#include "functions.h"
#include "gpio.h"
#include "mcu.h"
#include "rs485.h"
#include "targets.h"
#include "utility-timer.h"
#include "vreg.h"

void testgpio(gpio_t* gpio)
{
    gpio_reset(gpio);
    // delayMicros(1);
    gpio_set(gpio);
    // delayMicros(1);
    gpio_reset(gpio);
}

int main()
{
    mcu_setup(250);

    // rs485 initialization disables the transmitter
    rs485_initialize();

    vreg5V_initialize();
    vreg5V_enable();

    utility_timer_initialize();
    utility_timer_enable();

    gpio_t gpioCanTx = DEF_GPIO(
        MAIN_P3_PORT,
        MAIN_P3_PIN,
        0,
        GPIO_OUTPUT);

    gpio_initialize(&gpioCanTx);

    gpio_set_speed(&gpioCanTx, GPIO_SPEED_VERYFAST);

    while(1) {
        testgpio(&gpioCanTx);
    }
}