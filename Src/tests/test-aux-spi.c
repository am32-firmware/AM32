// Test the gpio pins used for auxilliary
// spi port
// (just toggle gpio pins, not the spi peripheral)
#include "stm32h563xx.h"
#include "targets.h"
#include "spi.h"
#include "gpio.h"
#include "dma.h"
#include "mcu.h"
#include "vreg.h"

uint16_t spi_rx_buffer[256];
uint16_t spi_tx_buffer[256];
static spi_t* spi = &spis[AUX_AM32_SPI_PERIPH];

int main()
{
    mcu_setup(250);

    // enable spi clock
    AUX_SPI_ENABLE_CLOCK();

    // enable 5V regulator
    vreg5V_initialize();
    vreg5V_enable();

    gpio_t gpioSpiNSS = DEF_GPIO(
        AUX_SPI_NSS_PORT,
        AUX_SPI_NSS_PIN,
        AUX_SPI_NSS_AF,
        GPIO_AF);
    gpio_t gpioSpiSCK = DEF_GPIO(
        AUX_SPI_SCK_PORT,
        AUX_SPI_SCK_PIN,
        AUX_SPI_SCK_AF,
        GPIO_AF);
    gpio_t gpioSpiMISO = DEF_GPIO(
        AUX_SPI_MISO_PORT,
        AUX_SPI_MISO_PIN,
        AUX_SPI_MISO_AF,
        GPIO_AF);
    gpio_t gpioSpiMOSI = DEF_GPIO(
        AUX_SPI_MOSI_PORT,
        AUX_SPI_MOSI_PIN,
        AUX_SPI_MOSI_AF,
        GPIO_AF);

    spi->_rx_buffer = spi_rx_buffer;
    spi->_tx_buffer = spi_tx_buffer;
    spi->_rx_buffer_size = 256;
    spi->_tx_buffer_size = 256;
    spi->rxDma = 0;
    spi->txDma = &dmaChannels[AUX_SPI_TX_DMA_CHANNEL];
    spi->txDmaRequest = AUX_SPI_TX_DMA_REQ;
    spi->rxDmaRequest = AUX_SPI_RX_DMA_REQ;
    spi_initialize(spi);

    gpio_initialize(&gpioSpiNSS);
    gpio_initialize(&gpioSpiSCK);
    gpio_initialize(&gpioSpiMISO);
    gpio_initialize(&gpioSpiMOSI);

    gpio_set_speed(&gpioSpiNSS, GPIO_SPEED_VERYFAST);
    gpio_set_speed(&gpioSpiSCK, GPIO_SPEED_VERYFAST);
    gpio_set_speed(&gpioSpiMISO, GPIO_SPEED_VERYFAST);
    gpio_set_speed(&gpioSpiMOSI, GPIO_SPEED_VERYFAST);

    uint16_t word = 0x5555;
    while(1) {
        spi_write_word(spi, word);
        for (int i = 0; i < 0xfffff; i++) {
            asm("nop");
        }
        // spi_write(&spi, data, 5);
    }
}