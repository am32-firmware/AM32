// Test the gpio pins used for auxilliary
// spi port
// (just toggle gpio pins, not the spi peripheral)
#include "targets.h"
#include "spi.h"
#include "mcu.h"
#include "gpio.h"
#include "dma.h"
#include "vreg.h"


#define LED_T0 (0b11000000)
#define LED_T1 (0b11110000)

uint16_t spi_rx_buffer[256];
uint16_t spi_tx_buffer[256];
spi_t spi;

int main()
{
    mcu_setup(250);

    vreg5V_initialize();
    vreg5V_enable();

    // enable spi clock
    AUX_SPI_ENABLE_CLOCK();

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


    spi.ref = AUX_SPI_PERIPH;

    spi._rx_buffer = spi_rx_buffer;
    spi._tx_buffer = spi_tx_buffer;
    spi._rx_buffer_size = 256;
    spi._tx_buffer_size = 256;
    spi.rxDma = &dmaChannels[7];
    spi.txDma = &dmaChannels[0];

    spi.txDmaRequest = LL_GPDMA1_REQUEST_SPI4_TX;
    spi.rxDmaRequest = LL_GPDMA1_REQUEST_SPI4_RX;

    // spi.CFG1_MBR = 0b111;
    spi.CFG1_MBR = 0b001; // kernel clock / 4
    spi_initialize(&spi);
    // configure spi kernel clock as HSE (25MHz)
    spi_configure_rcc_clock_selection(&spi, 0b101);

    gpio_initialize(&gpioSpiNSS);
    gpio_initialize(&gpioSpiSCK);
    gpio_initialize(&gpioSpiMISO);
    gpio_initialize(&gpioSpiMOSI);

    gpio_configure_pupdr(&gpioSpiMOSI, GPIO_PULL_DOWN);
    gpio_set_speed(&gpioSpiNSS, GPIO_SPEED_VERYFAST);
    gpio_set_speed(&gpioSpiSCK, GPIO_SPEED_VERYFAST);
    gpio_set_speed(&gpioSpiMISO, GPIO_SPEED_VERYFAST);
    gpio_set_speed(&gpioSpiMOSI, GPIO_SPEED_VERYFAST);

    #define DL (50 + 20)
    uint16_t word0 = (LED_T0 << 8) | LED_T0;
    uint16_t word1 = (LED_T0 << 8) | LED_T1;
    uint16_t word2 = (LED_T1 << 8) | LED_T0;
    uint16_t word3 = (LED_T1 << 8) | LED_T1;
    uint16_t data[DL];
    for (int i = 0; i < 50; i++) {
        data[i] = 0;
    }
    for (int i = 50; i < 50 + 12; i++) {
        data[i] = word1;
    }

    spi_write(&spi, data, DL);
    // arbitrary delay


    while(1) {
        // spi_write_word(&spi, word);
        // for (int i = 0; i < 0xfffff; i++) {
        //     asm("nop");
        // }
        // spi_write(&spi, data, DL);
        // // arbitrary delay
        // for (int i = 0; i < 0xffff; i++) {
        //     asm("nop");
        // }
    }
}