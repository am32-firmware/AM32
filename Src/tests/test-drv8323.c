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
    GADE_DRIVER_SPI_ENABLE_CLOCK();
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
    gpio_initialize(&gpioDrv8323Enable);
    gpio_set_speed(&gpioDrv8323Enable, 0b11);
    // gpio_reset(&gpioDrv8323Enable);
    // for (int i = 0; i < 0xfffff; i++) {
    //     asm("nop");
    // }
    // gpio_set(&gpioDrv8323Enable);

    // for (int i = 0; i < 0xfffff; i++) {
    //     asm("nop");
    // }

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
    // spi.CFG1_MBR = 0b101; // prescaler = 64 // this works on blueesc
    // spi.CFG1_MBR = 0b100; // prescaler = 128 // this works on blueesc
    spi.CFG1_MBR = 0b111; // prescaler = 256 // this works on blueesc
    // spi_initialize(&spi);

    drv.spi = &spi;
    drv.gpioEnable = &gpioDrv8323Enable;

    drv8323_initialize(&drv);
    gpio_initialize(&gpioSpiNSS);
    gpio_initialize(&gpioSpiSCK);
    gpio_initialize(&gpioSpiMISO);
    gpio_configure_pupdr(&gpioSpiMISO, GPIO_PULL_UP);
    gpio_initialize(&gpioSpiMOSI);
    gpio_set_speed(&gpioSpiMOSI, GPIO_SPEED_VERYFAST);

    // drv8323_read_reg(&drv, DRV8323_REG_CSA_CONTROL);
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
    DRV8323_OCP_VDSLVL_600mV));

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

    while(1) {
        // spi_write(&spi, data, 5);
    }
}