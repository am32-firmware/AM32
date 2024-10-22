// Test the gpio pins used for auxilliary
// spi port
// (just toggle gpio pins, not the spi peripheral)
#include "targets.h"
#include "spi.h"
#include "gpio.h"
#include "dma.h"
#include "mcu.h"


#define LED_T0 (0b11000000)
#define LED_T1 (0b11110000)

uint16_t spi_rx_buffer[256];
uint16_t spi_tx_buffer[256];
spi_t spi;

void rgb_write(uint32_t rgb)
{
    uint8_t data[26];
    data[0] = 0;
    data[1] = 0;
    for (int i = 2; i < 26; i++) {
        if (rgb & (1<<i)) {
            data[i] = LED_T1;
        } else {
            data[i] = LED_T0;
        }
    }
    spi_reset_buffers(&spi);
    spi_write(&spi, (uint16_t*)data, 13);
}

int main()
{
    mcu_setup();
    // enable dma clocks
    dma_initialize();
    // enable spi clock
    AUX_SPI_ENABLE_CLOCK();

    // enable 5V regulator
    gpio_t gpioVreg5VEnable = DEF_GPIO(
        VREG_5V_ENABLE_PORT,
        VREG_5V_ENABLE_PIN,
        0,
        GPIO_OUTPUT);
    gpio_initialize(&gpioVreg5VEnable);
    gpio_set(&gpioVreg5VEnable);


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

    spi.CFG1_MBR = 0b001; // kernel clock / 4
    spi_initialize(&spi);
    // configure spi kernel clock as HSE (25MHz)
    spi_configure_rcc_clock_selection(&spi, 0b101);

    // gpio_initialize(&gpioSpiNSS);
    gpio_initialize(&gpioSpiSCK);
    // gpio_initialize(&gpioSpiMISO);
    gpio_initialize(&gpioSpiMOSI);

    gpio_configure_pupdr(&gpioSpiMOSI, GPIO_PULL_DOWN);
    gpio_set_speed(&gpioSpiNSS, 0b11);
    gpio_set_speed(&gpioSpiSCK, 0b11);
    gpio_set_speed(&gpioSpiMISO, 0b11);
    gpio_set_speed(&gpioSpiMOSI, 0b11);
    
    while(1) {
        rgb_write(0x00040000);
        for (uint32_t i = 0; i < 0x8fffff; i++) {
            asm("nop");
        }
        rgb_write(0x00000400);
        for (uint32_t i = 0; i < 0x8fffff; i++) {
            asm("nop");
        }
        rgb_write(0x00000004);
        for (uint32_t i = 0; i < 0x8fffff; i++) {
            asm("nop");
        }
    }
}