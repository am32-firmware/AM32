// this test is made to run on a nucleo-h563
// it enables pullup resistors on adjacent
// (broken out) pins to test the individual/
// parallel pullup resistance value

#include "gpio.h"

#define DELAY 8000000
// STM32 NUCLEO-H563
// CN9

// ZIO PIN 2
// STM32 PIN PD7
#define GPIO0_PORT GPIOD
#define GPIO0_PIN 7

// ZIO PIN 4
// STM32 PIN PD6
#define GPIO1_PORT GPIOD
#define GPIO1_PIN 6

// ZIO PIN 6
// STM32 PIN PD5
#define GPIO2_PORT GPIOD
#define GPIO2_PIN 5

// ZIO PIN 8
// STM32 PIN PD4
#define GPIO3_PORT GPIOD
#define GPIO3_PIN 4

// ZIO PIN 10
// STM32 PIN PD3
#define GPIO4_PORT GPIOD
#define GPIO4_PIN 3


gpio_t gpio0 = DEF_GPIO(GPIO0_PORT, GPIO0_PIN, 0, GPIO_INPUT);
gpio_t gpio1 = DEF_GPIO(GPIO1_PORT, GPIO1_PIN, 0, GPIO_INPUT);
gpio_t gpio2 = DEF_GPIO(GPIO2_PORT, GPIO2_PIN, 0, GPIO_INPUT);
gpio_t gpio3 = DEF_GPIO(GPIO3_PORT, GPIO3_PIN, 0, GPIO_INPUT);
gpio_t gpio4 = DEF_GPIO(GPIO4_PORT, GPIO4_PIN, 0, GPIO_INPUT);


void delay(uint32_t n)
{
    for (uint32_t i = 0; i < n; i++) {
        asm("nop");
    }
}

int main()
{

    gpio_initialize(&gpio0);
    gpio_initialize(&gpio1);
    gpio_initialize(&gpio2);
    gpio_initialize(&gpio3);
    gpio_initialize(&gpio4);

    delay(DELAY);
    gpio_configure_pupdr(&gpio0, GPIO_PULL_UP);
    delay(DELAY);
    gpio_configure_pupdr(&gpio1, GPIO_PULL_UP);
    delay(DELAY);
    gpio_configure_pupdr(&gpio2, GPIO_PULL_UP);
    delay(DELAY);
    gpio_configure_pupdr(&gpio3, GPIO_PULL_UP);
    delay(DELAY);
    gpio_configure_pupdr(&gpio4, GPIO_PULL_UP);

    // do nothing
    while (1);
}