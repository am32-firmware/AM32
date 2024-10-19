#include "stm32h563xx.h"
#include "targets.h"
#include "spi.h"
#include "gpio.h"
#include "dma.h"
#include "drv8323-spi.h"

uint16_t spi_rx_buffer[256];
uint16_t spi_tx_buffer[256];
spi_t spi;
drv8323_t drv;

void testgpio(gpio_t* gpio)
{
    gpio_reset(gpio);
    for (int i = 0; i < 0xff; i++) {
        asm("nop");
    }
    gpio_set(gpio);

    for (int i = 0; i < 0xff; i++) {
        asm("nop");
    }
    gpio_reset(gpio);
}
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
        GPIO_OUTPUT);
    gpio_initialize(&gpioDrv8323Enable);
    gpio_set_speed(&gpioDrv8323Enable, 0b11);
    testgpio(&gpioDrv8323Enable);


    gpio_t gpioSpiNSS = DEF_GPIO(
        GATE_DRIVER_SPI_NSS_PORT,
        GATE_DRIVER_SPI_NSS_PIN,
        0,
        GPIO_OUTPUT);
    gpio_t gpioSpiSCK = DEF_GPIO(
        GATE_DRIVER_SPI_SCK_PORT,
        GATE_DRIVER_SPI_SCK_PIN,
        0,
        GPIO_OUTPUT);
    gpio_t gpioSpiMISO = DEF_GPIO(
        GATE_DRIVER_SPI_MISO_PORT,
        GATE_DRIVER_SPI_MISO_PIN,
        0,
        GPIO_OUTPUT);
    gpio_t gpioSpiMOSI = DEF_GPIO(
        GATE_DRIVER_SPI_MOSI_PORT,
        GATE_DRIVER_SPI_MOSI_PIN,
        0,
        GPIO_OUTPUT);
    

    spi.ref = SPI5;

    spi._rx_buffer = spi_rx_buffer;
    spi._tx_buffer = spi_tx_buffer;
    spi._rx_buffer_size = 256;
    spi._tx_buffer_size = 256;
    spi.rxDma = &dmaChannels[7];
    spi.txDma = &dmaChannels[0];

    // spi_initialize(&spi);

    gpio_initialize(&gpioSpiNSS);
    gpio_initialize(&gpioSpiSCK);
    gpio_initialize(&gpioSpiMISO);
    gpio_initialize(&gpioSpiMOSI);

    gpio_set_speed(&gpioSpiNSS, 0b11);
    gpio_set_speed(&gpioSpiSCK, 0b11);
    gpio_set_speed(&gpioSpiMISO, 0b11);
    gpio_set_speed(&gpioSpiMOSI, 0b11);
    
    testgpio(&gpioSpiNSS);
    testgpio(&gpioSpiSCK);
    testgpio(&gpioSpiMISO);
    testgpio(&gpioSpiMOSI);
    // while (!drv8323_write_reg(&drv,
    //     DRV8323_REG_DRIVER_CONTROL |
    //     DRV8323_DRIVER_CONTROL_COAST
    // ));


    // for (uint16_t i = 0; i < 200; i++) {
    //     spi_write(&spi, &i, 1);
    // }
    // uint16_t data = 0xf550;
    // spi_write(&spi, &data, 1);
    // spi_write(&spi, &data, 1);
    // spi_write(&spi, &data, 1);
    
    // uint16_t data[] = {
    //     0xff00,
    //     0x5555,
    //     0x0550,
    //     0x5555,
    //     0x00ff,
    //     0x5555,
    //     0x5555,
    //     0x5555,
    //     0x5555,
    //     0x5555,
    // };

    // uint16_t data[] = {
    //     DRV8323_READ | DRV8323_REG_FAULT_STATUS_1,
    //     DRV8323_READ | DRV8323_REG_VGS_STATUS_2,
    //     DRV8323_READ | DRV8323_REG_DRIVER_CONTROL,
    //     DRV8323_READ | DRV8323_REG_GATE_DRIVE_HS,
    //     DRV8323_READ | DRV8323_REG_GATE_DRIVE_LS,
    //     DRV8323_READ | DRV8323_REG_OCP_CONTROL,
    //     DRV8323_READ | DRV8323_REG_CSA_CONTROL,
    // };

    // spi_write(&spi, data, 7);
    // uint16_t readData[10];
    // while(spi_rx_waiting(&spi) < 7);
    // spi_read(&spi, readData, 7);
    // uint16_t word = 0x5555;
    // spi_write_word(&spi, word);
    while(1) {
        // spi_write_word(&spi, word);

    testgpio(&gpioSpiNSS);
    testgpio(&gpioSpiSCK);
    testgpio(&gpioSpiMISO);
    testgpio(&gpioSpiMOSI);
        // spi_write(&spi, data, 5);
    }
}