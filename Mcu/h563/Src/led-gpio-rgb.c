#include "led.h"
#include "gpio.h"
#include "targets.h"

static gpio_t gpioLedR = DEF_GPIO(LED_R_GPIO_PORT, LED_R_GPIO_PIN, 0, GPIO_OUTPUT);
static gpio_t gpioLedG = DEF_GPIO(LED_G_GPIO_PORT, LED_G_GPIO_PIN, 0, GPIO_OUTPUT);
static gpio_t gpioLedB = DEF_GPIO(LED_B_GPIO_PORT, LED_B_GPIO_PIN, 0, GPIO_OUTPUT);

void led_initialize(void)
{
    gpio_initialize(&gpioLedR);
    gpio_initialize(&gpioLedG);
    gpio_initialize(&gpioLedB);
    led_off();
}

void led_off(void)
{
    gpio_reset(&gpioLedR);
    gpio_reset(&gpioLedG);
    gpio_reset(&gpioLedB);
}

void led_on(void)
{
    gpio_set(&gpioLedR);
    gpio_set(&gpioLedG);
    gpio_set(&gpioLedB);
}

void led_toggle(void)
{
    gpio_toggle(&gpioLedR);
    gpio_toggle(&gpioLedG);
    gpio_toggle(&gpioLedB);
}

void led_write(uint32_t grb)
{
    if (grb & 0xff){
        gpio_set(&gpioLedB);
    } else {
        gpio_reset(&gpioLedB);
    }

    if (grb & (0xff << 4)){
        gpio_set(&gpioLedR);
    } else {
        gpio_reset(&gpioLedR);
    }

    if (grb & (0xff << 8)){
        gpio_set(&gpioLedB);
    } else {
        gpio_reset(&gpioLedB);
    }
}
