// Test the gpio pins used for auxilliary
// spi port
// (just toggle gpio pins, not the spi peripheral)
#include "stm32h563xx.h"
#include "targets.h"
#include "spi.h"
#include "gpio.h"
#include "dma.h"
#include "clock.h"


#define LED_T0 (0b11000000)
#define LED_T1 (0b11110000)

uint16_t spi_rx_buffer[256];
uint16_t spi_tx_buffer[256];
spi_t spi;

int main()
{
    clock_hse_enable();
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
    // gpio_set_speed(&gpioVreg5VEnable, 0b11);
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
    gpio_set_speed(&gpioSpiNSS, 0b11);
    gpio_set_speed(&gpioSpiSCK, 0b11);
    gpio_set_speed(&gpioSpiMISO, 0b11);
    gpio_set_speed(&gpioSpiMOSI, 0b11);
    

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
    uint16_t word = (LED_T1 << 8) | LED_T0;
    uint16_t data[12];
    for (int i = 0; i < 12; i++) {
        data[i] = word;
    }
    while(1) {
        // spi_write_word(&spi, word);
        // for (int i = 0; i < 0xfffff; i++) {
        //     asm("nop");
        // }
        spi_write(&spi, data, 6);
    }
}