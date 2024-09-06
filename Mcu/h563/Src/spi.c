#include "spi.h"
#include "stm32h563xx.h"

/*
8.5.1.1 SPI
On DRV832x SPI devices, an SPI bus is used to set device configurations, operating parameters, and read out
diagnostic information. The SPI operates in slave mode and connects to a master controller. The SPI input dat a
(SDI) word consists of a 16-bit word, with a 5-bit command and 11 bits of data. The SPI output data (SDO) word
consists of 11-bit register data. The first 5 bits are don’t care bits.
A valid frame must meet the following conditions:
• The SCLK pin should be low when the nSCS pin transitions from high to low and from low to high.
• The nSCS pin should be pulled high for at least 400 ns between words.
• When the nSCS pin is pulled high, any signals at the SCLK and SDI pins are ignored and the SDO pin is
placed in the Hi-Z state.
• Data is captured on the falling edge of the SCLK pin and data is propagated on the rising edge of the SCLK
pin.
• The most significant bit (MSB) is shifted in and out first.
• A full 16 SCLK cycles must occur for transaction to be valid.
• If the data word sent to the SDI pin is less than or more than 16 bits, a frame error occurs and the data word
is ignored.
• For a write command, the existing data in the register being written to is shifted out on the SDO pin following
the 5-bit command data.
*/
void spi_initialize(SPI_TypeDef* spi)
{

    RCC->APB3ENR |= RCC_APB3ENR_SPI5EN;
    // set TSIZE
    SPI5->CR2 = 2;

    // master baud rate prescaler = 32
    SPI5->CFG1 |= 0b100 << SPI_CFG1_MBR_Pos;
    // enable SS output
    SPI5->CFG2 |= SPI_CFG2_SSOE;
    // SSOM = 1, SP = 000, MIDI > 1
    
    // SS is pulsed inactive between data frames
    SPI5->CFG2 |= SPI_CFG2_SSOM;

    // data is captured on the falling edge of SCK
    SPI5->CFG2 |= SPI_CFG2_CPHA;

    // spi master mode
    SPI5->CFG2 |= SPI_CFG2_MASTER;

    // 15 clock cycle periods delay inserted between two consecutive data frames
    SPI5->CFG2 |= 0b1111 << SPI_CFG2_MIDI_Pos;

    // set MSSI to 15
    // insert 15 clock cycle periods delay between SS opening
    // a session and the beginning of the first data frame
    SPI5->CFG2 |= 0b1111;



    // SPI5->CFG1 |= SPI_CFG1_TXDMAEN;
    // SPI5->CFG1 |= SPI_CFG1_RXDMAEN;

    // set DSIZE (frame width) to 16 bits
    SPI5->CFG1 |= 0b01111;

    SPI5->CR1 |= SPI_CR1_SPE;

    SPI5->TXDR = DRV8323_WRITE | DRV8323_REG_CSA_CONTROL | DRV8323_REG_CSA_CONTROL_VALUE;
    SPI5->TXDR = DRV8323_WRITE | DRV8323_REG_CSA_CONTROL | DRV8323_REG_CSA_CONTROL_VALUE;

}

