// Test the gpio pins used for auxilliary
// spi port
// (just toggle gpio pins, not the spi peripheral)
#include "stm32h563xx.h"
#include "targets.h"
#include "gpio.h"
#include "dma.h"

void testgpio(gpio_t* gpio)
{
    gpio_reset(gpio);
    for (int i = 0; i < 0xfffff; i++) {
        asm("nop");
    }
    gpio_set(gpio);

    for (int i = 0; i < 0xfffff; i++) {
        asm("nop");
    }
    gpio_reset(gpio);
}
int main()
{
    gpio_t gpioVreg5VEnable = DEF_GPIO(
        VREG_5V_ENABLE_PORT,
        VREG_5V_ENABLE_PIN,
        0,
        GPIO_OUTPUT);
    gpio_t gpioSpiNSS = DEF_GPIO(
        AUX_SPI_NSS_PORT,
        AUX_SPI_NSS_PIN,
        0,
        GPIO_OUTPUT);
    gpio_t gpioSpiSCK = DEF_GPIO(
        AUX_SPI_SCK_PORT,
        AUX_SPI_SCK_PIN,
        0,
        GPIO_OUTPUT);
    gpio_t gpioSpiMISO = DEF_GPIO(
        AUX_SPI_MISO_PORT,
        AUX_SPI_MISO_PIN,
        0,
        GPIO_OUTPUT);
    gpio_t gpioSpiMOSI = DEF_GPIO(
        AUX_SPI_MOSI_PORT,
        AUX_SPI_MOSI_PIN,
        0,
        GPIO_OUTPUT);

    gpio_initialize(&gpioVreg5VEnable);
    gpio_initialize(&gpioSpiNSS);
    gpio_initialize(&gpioSpiSCK);
    gpio_initialize(&gpioSpiMISO);
    gpio_initialize(&gpioSpiMOSI);

    gpio_set_speed(&gpioVreg5VEnable, 0b11);
    gpio_set_speed(&gpioSpiNSS, 0b11);
    gpio_set_speed(&gpioSpiSCK, 0b11);
    gpio_set_speed(&gpioSpiMISO, 0b11);
    gpio_set_speed(&gpioSpiMOSI, 0b11);

    while(1) {
        testgpio(&gpioVreg5VEnable);
        testgpio(&gpioSpiNSS);
        testgpio(&gpioSpiSCK);
        testgpio(&gpioSpiMISO);
        testgpio(&gpioSpiMOSI);
    }
}