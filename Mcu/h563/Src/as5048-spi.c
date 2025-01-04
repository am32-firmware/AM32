#include "as5048-spi.h"
#include "gpio.h"
#include "targets.h"
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

// 15th bit (MSB) of spi package is parity bit (even parity)
#define AS5048_PKG_PARITY_Pos 15

// 14th bit of spi package is a flag
// read (1) / write (0) for command packages
// error flag for response packages
#define AS5048_PKG_FLAG_Pos 14

#define AS5048_PKG_DATA_MASK 0x3FFF

// 15th bit (MSB) of command is parity bit (even parity)
#define AS5048_CMD_PARITY_Pos AS5048_PKG_PARITY_Pos
// there is an odd number of '1' bits in the command
#define AS5048_CMD_PARITY_ODD (1 << AS5048_CMD_PARITY_Pos)
// there is an even number of '1' bits in the command
#define AS5048_CMD_PARITY_EVEN (0 << AS5048_CMD_PARITY_Pos)

#define AS5048_CMD_PARITY AS5048_CMD_PARITY_ODD

// 14th bit of command is read (1) / write (0) flag
#define AS5048_CMD_RW_Pos AS5048_PKG_FLAG_Pos
#define AS5048_CMD_READ (1 << AS5048_CMD_RW_Pos)
#define AS5048_CMD_WRITE (0 << AS5048_CMD_RW_Pos)

#define AS5048_CMD_RW_FLAG AS5048_CMD_READ

#define AS5048_CMD_READ_NOP (AS5048_CMD_PARITY_ODD | AS5048_CMD_READ | AS5048_REG_NOP)
#define AS5048_CMD_READ_CEF (AS5048_CMD_PARITY_EVEN | AS5048_CMD_READ | AS5048_REG_CEF)
#define AS5048_CMD_READ_ZPH (AS5048_CMD_PARITY_EVEN | AS5048_CMD_READ | AS5048_REG_ZPH)
#define AS5048_CMD_READ_ZPL (AS5048_CMD_PARITY_ODD | AS5048_CMD_READ | AS5048_REG_ZPL)
#define AS5048_CMD_READ_DAGC (AS5048_CMD_PARITY_EVEN | AS5048_CMD_READ | AS5048_REG_DAGC)
#define AS5048_CMD_READ_MAGNITUDE (AS5048_CMD_PARITY_EVEN | AS5048_CMD_READ | AS5048_REG_MAGNITUDE)
#define AS5048_CMD_READ_ANGLE (AS5048_CMD_PARITY_ODD | AS5048_CMD_READ | AS5048_REG_ANGLE)

uint16_t as5048SpiRxBuffer[256];
uint16_t as5048SpiTxBuffer[256];

void as5048_initialize(as5048_t* as5048)
{
    vreg5V_initialize();
    vreg5V_enable();

    as5048_initialize_spi(as5048);

    as5048_initialize_gpio(as5048);

    // as5048_read_all(as5048);
}

void as5048_initialize_gpio(as5048_t* as5048)
{
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

    gpio_initialize(&gpioSpiNSS);
    gpio_initialize(&gpioSpiSCK);
    gpio_initialize(&gpioSpiMISO);
    gpio_initialize(&gpioSpiMOSI);

    gpio_configure_pupdr(&gpioSpiMISO, GPIO_PULL_UP);

    gpio_set_speed(&gpioSpiNSS, GPIO_SPEED_VERYFAST);
    gpio_set_speed(&gpioSpiSCK, GPIO_SPEED_VERYFAST);
    gpio_set_speed(&gpioSpiMISO, GPIO_SPEED_VERYFAST);
    gpio_set_speed(&gpioSpiMOSI, GPIO_SPEED_VERYFAST);
}

void as5048_initialize_spi(as5048_t* as5048)
{
    // enable spi clock
    AUX_SPI_ENABLE_CLOCK();

    as5048->spi = &spis[AUX_AM32_SPI_PERIPH];

    // should not be necessary
    as5048->spi->ref = AUX_SPI_PERIPH;

    as5048->spi->_rx_buffer = as5048SpiRxBuffer;
    as5048->spi->_tx_buffer = as5048SpiTxBuffer;
    as5048->spi->_rx_buffer_size = 256;
    as5048->spi->_tx_buffer_size = 256;
    as5048->spi->rxDma = &dmaChannels[AUX_SPI_RX_DMA_CHANNEL];
    as5048->spi->txDma = &dmaChannels[AUX_SPI_TX_DMA_CHANNEL];
    as5048->spi->txDmaRequest = AUX_SPI_TX_DMA_REQ;
    as5048->spi->rxDmaRequest = AUX_SPI_RX_DMA_REQ;

    // a 250MHz kernel clock / 32 gives an SPI clock of 7.8125MHz
    as5048->spi->CFG1_MBR = SPI_MBR_DIV_32; // prescaler = 32
    // // a 250MHz kernel clock / 64 gives an SPI clock of ~3.91MHz
    // as5048->spi->CFG1_MBR = SPI_MBR_DIV_64; // prescaler = 64
    // // a 250MHz kernel clock / 128 gives an SPI clock of ~1.95MHz
    // as5048->spi->CFG1_MBR = SPI_MBR_DIV_128; // prescaler = 128
    // // a 250MHz kernel clock / 256 gives an SPI clock of ~0.98MHz
    // as5048->spi->CFG1_MBR = SPI_MBR_DIV_256; // prescaler = 256

    as5048->spi->CFG2 =
                ( SPI_CFG2_AFCNTR
                | SPI_CFG2_SSOE
                | SPI_CFG2_SSOM
                | SPI_CFG2_CPHA
                | SPI_CFG2_MASTER
                | (0b1111 << SPI_CFG2_MIDI_Pos)
                | (0b1111 << SPI_CFG2_MSSI_Pos)
                );
    spi_initialize(as5048->spi);
}

uint16_t as5048_prepare_command(uint16_t word, bool read_flag)
{
    uint16_t command = word;
    if (command & AS5048_CMD_READ) {
        // read flag is set, user error
        while (1);
    }
    command &= ~AS5048_CMD_RW_FLAG;
    command |= read_flag;

    command &= ~AS5048_CMD_PARITY;
    command |= as5048_parity(command) << AS5048_CMD_PARITY_Pos;

    return command;
}

bool as5048_write_reg(as5048_t* as5048, uint16_t reg, uint16_t data)
{
    uint16_t package = as5048_prepare_command(reg, false);
    as5048_spi_write_word(as5048, package);

    package = as5048_prepare_command(data, false);
    as5048_spi_write_word(as5048, package);

    uint16_t response =  as5048_read_reg(as5048, AS5048_REG_NOP);

    return response == data;
}

as5048_parity_e as5048_parity(uint16_t word)
{
    uint8_t parity = 0;
    for (int i = 0; i < AS5048_CMD_PARITY_Pos; i++) {
        parity = parity ^ ((word >> i) & 1);
    }

    as5048_parity_e result;
    if (parity) {
        result = AS5048_PARITY_EVEN;
    } else {
        result = AS5048_PARITY_ODD;
    }
    return result;
}


uint16_t as5048_read_reg(as5048_t* as5048, uint16_t word)
{
    word |= AS5048_CMD_READ;
    word |= as5048_parity(word) << AS5048_CMD_PARITY_Pos;
    as5048_spi_write_word(as5048, word);

    uint16_t response =  as5048_spi_write_word(as5048, AS5048_CMD_READ_NOP);
    if (((response >> AS5048_CMD_PARITY_Pos) & 1) ^ as5048_parity(response)) {
        // parity error
        while (1);
    }
    if (response & (1 << AS5048_PKG_FLAG_Pos)) {
        // error flag, read diagnostic register to determine the error cause
        uint16_t dagc = as5048_spi_write_word(as5048, AS5048_CMD_READ_DAGC);
        while (1);
    }

    return response & AS5048_PKG_DATA_MASK;
}

uint16_t as5048_spi_write_word(as5048_t* as5048, uint16_t word)
{
    return spi_write_word(as5048->spi, word);
}

void as5048_read_all(as5048_t* as5048)
{
    volatile uint32_t reg = 0;
    reg = as5048_read_reg(as5048, AS5048_REG_NOP);
    reg = as5048_read_reg(as5048, AS5048_REG_CEF);
    reg = as5048_read_reg(as5048, AS5048_REG_PC);
    reg = as5048_read_reg(as5048, AS5048_REG_ZPH);
    reg = as5048_read_reg(as5048, AS5048_REG_ZPL);
    reg = as5048_read_reg(as5048, AS5048_REG_DAGC);
    reg = as5048_read_reg(as5048, AS5048_REG_MAGNITUDE);
    reg = as5048_read_reg(as5048, AS5048_REG_ANGLE);
}

// get the angle in degrees * 100 (centidegrees)
uint16_t as5048_get_angle_degrees(as5048_t* as5048)
{
    uint16_t angle = as5048_read_angle(as5048);
    // 16 bits is the maximum shift without overflow
    uint16_t angle_degrees = (100 * angle * (360 << 16) / (1 << 14)) >> 16;
    return angle_degrees;
}

// read the 14-bit angle value from the device
uint16_t as5048_read_angle(as5048_t* as5048)
{
    return as5048_read_reg(as5048, AS5048_REG_ANGLE);
}

// read the zero position setting
uint16_t as5048_read_zero_position(as5048_t* as5048)
{

    uint16_t zph = as5048_read_reg(as5048, AS5048_REG_ZPH);
    uint16_t zpl = as5048_read_reg(as5048, AS5048_REG_ZPL);

    uint16_t zero_position = (zph << 6) & zpl;
    return zero_position;
}

bool as5048_set_zero_position(as5048_t* as5048)
{
    uint16_t current_angle = as5048_read_angle(as5048);
    return as5048_write_zero_position(as5048, current_angle);
}

bool as5048_write_zero_position(as5048_t* as5048, uint16_t zero_position)
{
    // lower 6 bits for zero position low (ZPL) register
    uint16_t zpl = zero_position & 0b111111;

    // upper byte (8 bits) for zero position high (ZPH) register
    uint16_t zph = zero_position >> 6;

    as5048_write_reg(as5048, AS5048_REG_ZPL, zpl);
    as5048_write_reg(as5048, AS5048_REG_ZPH, zph);
    return true;
}
