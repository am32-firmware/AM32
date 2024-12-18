// This example reads the as5048

#include "stm32h563xx.h"
#include "targets.h"
#include "spi.h"
#include "gpio.h"
#include "dma.h"
#include "mcu.h"
#include "vreg.h"


// NOP
// No operation dummy flag
#define AS5048_REG_NOP 0x0000
// Clear Error Flag
// Error register. All errors are
// cleared by access
#define AS5048_REG_CEF 0x0001
// Programming Control
// Programming control register.
// Programming must be enabled before burning
// the fuse(s). After programming is a verification
// mandatory. See programming procedure
#define AS5048_REG_PC 0x0003
// OTP Register Zero Position High
// Zero Position value high byte
#define AS5048_REG_ZPH 0x0016
// OTP Register Zero Position Low 6 LSBs
// Zero Position remaining 6 lower LSB's
#define AS5048_REG_ZPL 0x0017
// Diagnostics + Automatic Gain Control (AGC)
// Diagnostic Flags
// Automatic Gain Control value. 0 decimal
// represents high magnetic field, 255 decimal
// represents low magnetic field.
#define AS5048_REG_DAGC 0x3FFD
// Magnitude
// Magnitude output value of the CORDIC
#define AS5048_REG_MAGNITUDE 0x3FFE
// Angle
// Angle output value including zero position correction
#define AS5048_REG_ANGLE 0x3FFF




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


    // 000: rcc_pclk3 selected as kernel clock (default after reset)
    // 001: pll2_q_ck selected as kernel clock
    // 010: pll3_q_ck selected as kernel clock
    // 011: hsi_ker_ck selected as kernel clock
    // 100: csi_ker_ck selected as kernel clock
    // 101: hse_ck selected as kernel clock
    // others: reserved, the kernel clock is disabled
    // spi_configure_rcc_clock_selection(spi, 0b101);

    // spi->CFG1_MBR = 0b000; // prescaler = 2 // this DOES NOT work on blueesc
    // spi->CFG1_MBR = 0b001; // prescaler = 4 // this DOES NOT work on blueesc
    // spi->CFG1_MBR = 0b010; // prescaler = 8 // this DOES NOT work on blueesc
    // spi->CFG1_MBR = 0b011; // prescaler = 16 // this DOES NOT work on blueesc
    spi->CFG1_MBR = 0b100; // prescaler = 32 // this works on blueesc
    // spi->CFG1_MBR = 0b101; // prescaler = 64 // this works on blueesc
    // spi->CFG1_MBR = 0b100; // prescaler = 128 // this works on blueesc
    // spi->CFG1_MBR = 0b111; // prescaler = 256 // this works on blueesc

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

    gpio_configure_pupdr(&gpioSpiMISO, GPIO_PULL_UP);

    gpio_set_speed(&gpioSpiNSS, 0b11);
    gpio_set_speed(&gpioSpiSCK, 0b11);
    gpio_set_speed(&gpioSpiMISO, 0b11);
    gpio_set_speed(&gpioSpiMOSI, 0b11);

    uint16_t data[] = {
        0xFFFF,
        0xFFFF,
        0xFFFF,
        0xFFFF,
        0xFFFF,
        0xFFFF,
        0xFFFF,
        0xFFFF,
    };

    while(1) {
        spi_write(spi, data, 7);
        while(spi_rx_waiting(spi) < 7);
        spi_read(spi, readData, 7);
        for (int i = 0; i < 7; i ++) {
            readData[i] = readData[i] & 0x3FFF;
        }
        // compare();
    }
}