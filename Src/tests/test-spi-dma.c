#include "stm32h563xx.h"
#include "targets.h"
#include "spi.h"
#include "gpio.h"
#include "dma.h"

static uint16_t spi_rx_buffer[256];
static uint16_t spi_tx_buffer[256];
static spi_t spi;

int main()
{
    // enable dma clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPDMA1EN;
    // enable spi clock
    RCC->APB3ENR |= RCC_APB3ENR_SPI5EN;


    gpio_t gpioDrv8323Enable = DEF_GPIO(
        GPIOF,
        0,
        0,
        GPIO_OUTPUT    );
    gpio_initialize(&gpioDrv8323Enable);
    gpio_set_speed(&gpioDrv8323Enable, 0b11);
    gpio_reset(&gpioDrv8323Enable);
    for (int i = 0; i < 0xfffff; i++) {
        asm("nop");
    }
    gpio_set(&gpioDrv8323Enable);

    for (int i = 0; i < 0xfffff; i++) {
        asm("nop");
    }

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
    

    spi.ref = SPI5;

    spi._rx_buffer = spi_rx_buffer;
    spi._tx_buffer = spi_tx_buffer;
    spi._rx_buffer_size = 256;
    spi._tx_buffer_size = 256;
    spi.rxDma = &dmaChannels[7];
    spi.txDma = &dmaChannels[0];

    spi_initialize(&spi);

    gpio_initialize(&gpioSpiNSS);
    gpio_initialize(&gpioSpiSCK);
    gpio_initialize(&gpioSpiMISO);
    gpio_initialize(&gpioSpiMOSI);

    // for (uint16_t i = 0; i < 200; i++) {
    //     spi_write(&spi, &i, 1);
    // }
    // uint16_t data = 0xf550;
    // spi_write(&spi, &data, 1);
    // spi_write(&spi, &data, 1);
    // spi_write(&spi, &data, 1);
    
    uint16_t data[] = {
        0xff00,
        0x5555,
        0x0550,
        0x5555,
        0x00ff,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
    };

    spi_write(&spi, data, 10);


    while(1) {
        spi_write(&spi, data, 5);

    }
}