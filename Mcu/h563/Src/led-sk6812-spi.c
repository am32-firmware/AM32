#include "led.h"
#include "targets.h"
#include "gpio.h"
#include "dma.h"
#include "spi.h"
#include "vreg.h"


// 2 bytes for leading 0 to prevent glitch on mosi line
// (MOSI line is pulled to the value of the first bit) for
// a moment before data transfer commences after spi
// is enabled
// 24 bytes for 24bit brg data
uint8_t data[26];

#define LED_T0 (0b11000000)
#define LED_T1 (0b11110000)

uint16_t led_spi_rx_buffer[256];
uint16_t led_spi_tx_buffer[256];
static spi_t* spi = &spis[LED_AM32_SPI_PERIPH];

void led_initialize()
{
    vreg5V_initialize();
    vreg5V_enable();

    // enable spi clock
    LED_SPI_ENABLE_CLOCK();

    gpio_t gpioSpiSCK = DEF_GPIO(
        LED_SPI_SCK_PORT,
        LED_SPI_SCK_PIN,
        LED_SPI_SCK_AF,
        GPIO_AF);

    gpio_t gpioSpiMOSI = DEF_GPIO(
        LED_SPI_MOSI_PORT,
        LED_SPI_MOSI_PIN,
        LED_SPI_MOSI_AF,
        GPIO_AF);

    // spi->ref = LED_SPI_PERIPH;

    // configure spi kernel clock as HSE via per_ck (25MHz)
    spi_configure_rcc_clock_selection(spi, 0b100);

    spi->_rx_buffer = led_spi_rx_buffer;
    spi->_tx_buffer = led_spi_tx_buffer;
    spi->_rx_buffer_size = 256;
    spi->_tx_buffer_size = 256;
    spi->rxDma = &dmaChannels[LED_RX_DMA_CHANNEL];
    spi->txDma = &dmaChannels[LED_TX_DMA_CHANNEL];

    spi->txDmaRequest = LL_GPDMA1_REQUEST_SPI2_TX;
    spi->rxDmaRequest = LL_GPDMA1_REQUEST_SPI2_RX;

    spi->CFG1_MBR = 0b001; // kernel clock / 2
    spi->CFG2 = ( SPI_CFG2_SSOE
                | SPI_CFG2_CPHA
                | SPI_CFG2_MASTER
    );
    spi_initialize(spi);

    gpio_initialize(&gpioSpiSCK);
    gpio_initialize(&gpioSpiMOSI);

    gpio_configure_pupdr(&gpioSpiMOSI, GPIO_PULL_DOWN);
    gpio_set_speed(&gpioSpiMOSI, 0b11);

    data[0] = 0;
    data[1] = 0;
}

void led_off(void)
{
    led_write(0);
}

void led_write(uint32_t brg)
{
    for (int i = 0; i < 24; i++) {
        if (brg & (1<<i)) {
            data[i+2] = LED_T1;
        } else {
            data[i+2] = LED_T0;
        }
    }
    spi_reset_buffers(spi);
    spi_write(spi, (uint16_t*)data, 13);
}
