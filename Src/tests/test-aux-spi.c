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

    // configure spi kernel clock as HSE via per_ck (25MHz)
    // spi_configure_rcc_clock_selection(spi, 0b100);

    spi->_rx_buffer = spi_rx_buffer;
    spi->_tx_buffer = spi_tx_buffer;
    spi->_rx_buffer_size = 256;
    spi->_tx_buffer_size = 256;
    // spi->rxDma = 0;
    spi->rxDma = &dmaChannels[AUX_SPI_RX_DMA_CHANNEL];
    spi->txDma = &dmaChannels[AUX_SPI_TX_DMA_CHANNEL];
    spi->rxDmaRequest = AUX_SPI_RX_DMA_REQ;
    spi->txDmaRequest = AUX_SPI_TX_DMA_REQ;

    // spi->CFG1_MBR = SPI_MBR_DIV_4; // kernel clock / 4
    // spi->CFG2 = ( SPI_CFG2_SSOE
    //             | SPI_CFG2_CPHA
    //             | SPI_CFG2_MASTER
    //             );

    // a 250MHz kernel clock / 32 gives an SPI clock of 7.8125MHz
    spi->CFG1_MBR = SPI_MBR_DIV_32; // prescaler = 32
    spi->CFG2 =
                ( SPI_CFG2_AFCNTR
                | SPI_CFG2_SSOE
                | SPI_CFG2_SSOM
                | SPI_CFG2_CPHA
                | SPI_CFG2_MASTER
                | (0b1111 << SPI_CFG2_MIDI_Pos)
                | (0b1111 << SPI_CFG2_MSSI_Pos)
                );
    spi_initialize(spi);

    gpio_initialize(&gpioSpiNSS);
    gpio_initialize(&gpioSpiSCK);
    gpio_initialize(&gpioSpiMISO);
    gpio_initialize(&gpioSpiMOSI);

    gpio_configure_pupdr(&gpioSpiMISO, GPIO_PULL_UP);
    // gpio_configure_pupdr(&gpioSpiMOSI, GPIO_PULL_DOWN);

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