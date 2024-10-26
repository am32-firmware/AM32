// This example configures a couple
// of registers on the drv8323

#include "stm32h563xx.h"
#include "targets.h"
#include "spi.h"
#include "gpio.h"
#include "dma.h"
#include "drv8323-spi.h"
#include "mcu.h"
#include "utility-timer.h"

uint16_t spi_rx_buffer[256];
uint16_t spi_tx_buffer[256];
spi_t spi;
drv8323_t drv;

int main()
{
    mcu_setup();
    utility_timer_initialize();
    utility_timer_enable();
    // enable spi clock
    GATE_DRIVER_SPI_ENABLE_CLOCK();
    gpio_t gpioDrv8323Enable = DEF_GPIO(
        DRV_ENABLE_PORT,
        DRV_ENABLE_PIN,
        0,
        GPIO_OUTPUT);
    // gpio_t gpioDrv8323Enable = DEF_GPIO( // nucleo
    // GPIOF,
    // 0,
    // 0,
    // GPIO_OUTPUT);
    // gpio_initialize(&gpioDrv8323Enable);
    // gpio_set_speed(&gpioDrv8323Enable, 0b11);

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

    // 000: rcc_pclk3 selected as kernel clock (default after reset)
    // 001: pll2_q_ck selected as kernel clock
    // 010: pll3_q_ck selected as kernel clock
    // 011: hsi_ker_ck selected as kernel clock
    // 100: csi_ker_ck selected as kernel clock
    // 101: hse_ck selected as kernel clock
    // others: reserved, the kernel clock is disabled
    spi_configure_rcc_clock_selection(&spi, 0b101);

    spi._rx_buffer = spi_rx_buffer;
    spi._tx_buffer = spi_tx_buffer;
    spi._rx_buffer_size = 256;
    spi._tx_buffer_size = 256;
    spi.rxDma = &dmaChannels[7];
    spi.txDma = &dmaChannels[0];
    spi.txDmaRequest = LL_GPDMA1_REQUEST_SPI5_TX;
    spi.rxDmaRequest = LL_GPDMA1_REQUEST_SPI5_RX;
    // spi.CFG1_MBR = 0b011; // prescaler = 16 // this DOES NOT work on blueesc
    // spi.CFG1_MBR = 0b100; // prescaler = 32 // this works on blueesc
    spi.CFG1_MBR = 0b101; // prescaler = 64 // this works on blueesc
    // spi.CFG1_MBR = 0b100; // prescaler = 128 // this works on blueesc
    // spi.CFG1_MBR = 0b111; // prescaler = 256 // this works on blueesc

    drv.spi = &spi;

    gpio_t gpioDrv8323nFault = DEF_GPIO(
        DRV_FAULT_PORT,
        DRV_FAULT_PIN,
        0,
        GPIO_INPUT);
    // gpio_initialize(&gpioDrv8323nFault);
    // gpio_configure_pupdr(&gpioDrv8323nFault, GPIO_PULL_UP);


    gpio_t gpioDrv8323Cal = DEF_GPIO(
        DRV_CAL_PORT,
        DRV_CAL_PIN,
        0,
        GPIO_OUTPUT);
    // gpio_initialize(&gpioDrv8323Cal);
    // gpio_reset(&gpioDrv8323Cal);



    drv.gpioEnable = &gpioDrv8323Enable;
    drv.gpioNFault = &gpioDrv8323nFault;
    drv.gpioCal = &gpioDrv8323Cal;



    drv8323_initialize(&drv);


    gpio_initialize(&gpioSpiNSS);
    gpio_initialize(&gpioSpiSCK);
    gpio_initialize(&gpioSpiMISO);
    gpio_configure_pupdr(&gpioSpiMISO, GPIO_PULL_UP);
    gpio_initialize(&gpioSpiMOSI);
    gpio_set_speed(&gpioSpiMOSI, GPIO_SPEED_VERYFAST);

    while (!drv8323_write_reg(&drv,
        DRV8323_REG_CSA_CONTROL |
        DRV8323_CSA_VREF_DIV |
        DRV8323_CSA_CSA_GAIN_40VV |
        DRV8323_CSA_SEN_LVL_250mV
    ));

    while (!drv8323_write_reg(&drv,
        DRV8323_REG_OCP_CONTROL |
        DRV8323_OCP_DEADTIME_400ns |
        DRV8323_OCP_DEGLITCH_6us |
        DRV8323_OCP_VDSLVL_600mV
    ));

    while(1) {
    }
}