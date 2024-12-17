// This example reads the as5048

#include "stm32h563xx.h"
#include "targets.h"
#include "spi.h"
#include "gpio.h"
#include "dma.h"
#include "mcu.h"






// static uint16_t spi_rx_buffer[256];
// static uint16_t spi_tx_buffer[256];
// spi_t spi;
uint16_t readData[7];
const uint16_t defaultData[7] = {
    0,
    0,
    0,
    1023,
    2047,
    345,
    643
};

bool compare()
{
    for (int i = 0; i < 7; i++) {
        if (readData[i] != defaultData[i]) {
            for (;;); // spin forever
        }
    }
    return true;
}

uint16_t spi_rx_buffer[256];
uint16_t spi_tx_buffer[256];
spi_t* spi = &spis[AUX_AM32_SPI_PERIPH];

int main()
{
    mcu_setup(250);
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


    // 000: rcc_pclk3 selected as kernel clock (default after reset)
    // 001: pll2_q_ck selected as kernel clock
    // 010: pll3_q_ck selected as kernel clock
    // 011: hsi_ker_ck selected as kernel clock
    // 100: csi_ker_ck selected as kernel clock
    // 101: hse_ck selected as kernel clock
    // others: reserved, the kernel clock is disabled
    spi_configure_rcc_clock_selection(spi, 0b101);

    spi->ref = AUX_SPI_PERIPH;

    spi->_rx_buffer = spi_rx_buffer;
    spi->_tx_buffer = spi_tx_buffer;
    spi->_rx_buffer_size = 256;
    spi->_tx_buffer_size = 256;
    spi->rxDma = &dmaChannels[7];
    spi->txDma = &dmaChannels[0];

    spi->txDmaRequest = AUX_SPI_TX_DMA_REQ;
    spi->rxDmaRequest = AUX_SPI_RX_DMA_REQ;
    spi_initialize(spi);

    gpio_initialize(&gpioSpiNSS);
    gpio_initialize(&gpioSpiSCK);
    gpio_initialize(&gpioSpiMISO);
    gpio_initialize(&gpioSpiMOSI);

    gpio_set_speed(&gpioSpiNSS, 0b11);
    gpio_set_speed(&gpioSpiSCK, 0b11);
    gpio_set_speed(&gpioSpiMISO, 0b11);
    gpio_set_speed(&gpioSpiMOSI, 0b11);

    uint16_t data[] = {
        DRV8323_READ | DRV8323_REG_FAULT_STATUS_1,
        DRV8323_READ | DRV8323_REG_VGS_STATUS_2,
        DRV8323_READ | DRV8323_REG_DRIVER_CONTROL,
        DRV8323_READ | DRV8323_REG_GATE_DRIVE_HS,
        DRV8323_READ | DRV8323_REG_GATE_DRIVE_LS,
        DRV8323_READ | DRV8323_REG_OCP_CONTROL,
        DRV8323_READ | DRV8323_REG_CSA_CONTROL,
    };

    while(1) {
        spi_write(spi, data, 7);
        while(spi_rx_waiting(spi) < 7);
        spi_read(spi, readData, 7);
        compare();
    }
}