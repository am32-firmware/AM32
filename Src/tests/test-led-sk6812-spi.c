// Test the gpio pins used for auxilliary
// spi port
// (just toggle gpio pins, not the spi peripheral)
#include "stm32h563xx.h"
#include "targets.h"
#include "spi.h"
#include "gpio.h"
#include "dma.h"
#include "clock.h"
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
    spi_write(&spi, (uint16_t*)data, 13);
}

int main()
{
    // clock_hsi_config_divider(0b00);
    // clock_hse_enable();

    mcu_setup();
    // enable dma clocks
    dma_initialize();
    // enable spi clock
    LED_SPI_ENABLE_CLOCK();

    // enable 5V regulator
    gpio_t gpioVreg5VEnable = DEF_GPIO(
        VREG_5V_ENABLE_PORT,
        VREG_5V_ENABLE_PIN,
        0,
        GPIO_OUTPUT);
    gpio_initialize(&gpioVreg5VEnable);
    // gpio_set_speed(&gpioVreg5VEnable, 0b11);
    gpio_set(&gpioVreg5VEnable);

    gpio_t gpioSpiMOSI = DEF_GPIO(
        LED_SPI_MOSI_PORT,
        LED_SPI_MOSI_PIN,
        LED_SPI_MOSI_AF,
        GPIO_AF);

    spi.ref = LED_SPI_PERIPH;

    spi._rx_buffer = spi_rx_buffer;
    spi._tx_buffer = spi_tx_buffer;
    spi._rx_buffer_size = 256;
    spi._tx_buffer_size = 256;
    spi.rxDma = &dmaChannels[7];
    spi.txDma = &dmaChannels[0];

    spi.txDmaRequest = LL_GPDMA1_REQUEST_SPI2_TX;
    spi.rxDmaRequest = LL_GPDMA1_REQUEST_SPI2_RX;

    spi.CFG1_MBR = 0b001; // kernel clock / 2
    spi_initialize(&spi);
    // configure spi kernel clock as HSE via per_ck (25MHz)
    spi_configure_rcc_clock_selection(&spi, 0b100);

    gpio_initialize(&gpioSpiMOSI);

    gpio_configure_pupdr(&gpioSpiMOSI, GPIO_PULL_DOWN);
    gpio_set_speed(&gpioSpiMOSI, 0b11);
    
    spi_write_word(&spi, 0x5555);
    while(1);

    while(1) {
        rgb_write(0x00040000);
        for (uint32_t i = 0; i < 0xffffff; i++) {
            asm("nop");
        }
        rgb_write(0x00000400);
        for (uint32_t i = 0; i < 0xffffff; i++) {
            asm("nop");
        }
        rgb_write(0x00000004);
        for (uint32_t i = 0; i < 0xffffff; i++) {
            asm("nop");
        }
    }
}