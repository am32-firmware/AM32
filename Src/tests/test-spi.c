#include "stm32h563xx.h"


#include "targets.h"
#include "spi.h"
#include "gpio.h"


int main()
{    
    gpio_t gpioDrv8323Enable = DEF_GPIO(
        GPIOE,
        2,
        0,
        GPIO_OUTPUT
    );
    gpio_initialize(&gpioDrv8323Enable);
    gpio_reset(&gpioDrv8323Enable);

    for (int i = 0; i < 0xfffff; i++) {
        asm("nop");
    }
    gpio_set(&gpioDrv8323Enable);

    for (int i = 0; i < 0xfffff; i++) {
        asm("nop");
    }

    spi_initialize(SPI5);
    gpio_t gpioSpiNSS = DEF_GPIO(
        GATE_DRIVER_SPI_NSS_PORT,
        GATE_DRIVER_SPI_NSS_PIN,
        GATE_DRIVER_SPI_NSS_AF,
        GPIO_AF);
    gpio_t gpioSpiSCK = DEF_GPIO(
        GATE_DRIVER_SPI_SCK_PORT,
        GATE_DRIVER_SPI_SCK_PIN,
        GATE_DRIVER_SPI_SCK_AF,
        GPIO_AF);
    gpio_t gpioSpiMISO = DEF_GPIO(
        GATE_DRIVER_SPI_MISO_PORT,
        GATE_DRIVER_SPI_MISO_PIN,
        GATE_DRIVER_SPI_MISO_AF,
        GPIO_AF);
    gpio_t gpioSpiMOSI = DEF_GPIO(
        GATE_DRIVER_SPI_MOSI_PORT,
        GATE_DRIVER_SPI_MOSI_PIN,
        GATE_DRIVER_SPI_MOSI_AF,
        GPIO_AF);

    

    gpio_initialize(&gpioSpiNSS);
    gpio_initialize(&gpioSpiSCK);
    gpio_initialize(&gpioSpiMISO);
    gpio_initialize(&gpioSpiMOSI);

    // SPI5->TXDR = 0;
    SPI5->CR1 |= SPI_CR1_CSTART;

    // while (SPI5->SR & SPI_SR_)
    // volatile uint16_t a = SPI5->RXDR;
    while(1) {

    }
}