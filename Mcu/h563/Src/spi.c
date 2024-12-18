#include "spi.h"
#include "stm32h563xx.h"
#include "stm32h5xx_ll_dma.h"

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

spi_t spis[] = {
    {.ref = SPI1},
    {.ref = SPI2},
    {.ref = SPI3},
    {.ref = SPI4},
    {.ref = SPI5},
    {.ref = SPI6},
};

void spi_dma_cb(dmaChannel_t* dma)
{
    // dma->ref->CFCR |= DMA_IT_TCIF << dma->flagsShift;
    dma->ref->CFCR |= DMA_CFCR_TCF;
    spi_dma_transfer_complete_isr((spi_t*)dma->userParam);
}

void spi_txc_cb(spi_t* spi)
{

}

void spi_reset_buffers(spi_t* spi)
{
    while (spi->txDma->ref->CCR & DMA_CCR_EN)
    {}; // wait for current transfer to complete
    spi->_tx_head = 0;
    spi->_tx_tail = 0;
    spi->_rx_head = 0;
}

void spi_configure_rcc_clock_selection(spi_t* spi, uint8_t selection)
{
    switch ((uint32_t)spi->ref)
    {
        case SPI2_BASE:
            RCC->CCIPR3 |= selection << RCC_CCIPR3_SPI2SEL_Pos;
            break;
        case SPI4_BASE:
            RCC->CCIPR3 |= selection << RCC_CCIPR3_SPI4SEL_Pos;
            break;
        case SPI5_BASE:
            RCC->CCIPR3 |= selection << RCC_CCIPR3_SPI5SEL_Pos;
            break;
        default:
            break;
    }
}
void spi_initialize(spi_t* spi)
{
    // set the channel destination address
    spi->txDma->ref->CDAR = (uint32_t)&spi->ref->TXDR;
    // set the channel source address
    spi->txDma->ref->CSAR = (uint32_t)spi->_tx_buffer;

    // set source incrementing burst
    spi->txDma->ref->CTR1 |= DMA_CTR1_SINC;
    // set the peripheral hardware request selection
    spi->txDma->ref->CTR2 |= spi->txDmaRequest;
    // enable transfer complete interrupt
    spi->txDma->ref->CCR |= DMA_CCR_TCIE;
    spi->txDma->callback = spi_dma_cb;
    spi->txDma->userParam = (uint32_t)spi;

    // set source data width to half word (16 bit)
    spi->txDma->ref->CTR1 |= 0b01 << DMA_CTR1_SDW_LOG2_Pos;
    // spi->txDma->ref->CTR1 |= 0b00 << DMA_CTR1_SDW_LOG2_Pos;
    // set destination data width to half word (16 bit)
    spi->txDma->ref->CTR1 |= 0b01 << DMA_CTR1_DDW_LOG2_Pos;
    // spi->txDma->ref->CTR1 |= 0b00 << DMA_CTR1_DDW_LOG2_Pos;

    NVIC_SetPriority(spi->txDma->irqn, 0);
    NVIC_EnableIRQ(spi->txDma->irqn);

    // set the channel source address
    spi->rxDma->ref->CSAR = (uint32_t)&spi->ref->RXDR;

    // set the channel destination address
    spi->rxDma->ref->CDAR = (uint32_t)spi->_rx_buffer;

    // set destination incrementing burst
    spi->rxDma->ref->CTR1 |= DMA_CTR1_DINC;
    // set the peripheral hardware request selection
    spi->rxDma->ref->CTR2 |= spi->rxDmaRequest;

    // set the transfer length in bytes (not words!)
    spi->rxDma->ref->CBR1 = 2*256;
    // set the block repeated destination address offset
    spi->rxDma->ref->CBR2 |= 2*256 << DMA_CBR2_BRDAO_Pos;
    spi->rxDma->ref->CBR1 |= DMA_CBR1_BRDDEC;
    // configure single LLI to run repeatedly
    spi->rxDma->ref->CLLR = 0x08000004 & DMA_CLLR_LA_Msk;

    // set source data width to half word (16 bit)
    spi->rxDma->ref->CTR1 |= 0b01 << DMA_CTR1_SDW_LOG2_Pos;
    // spi->txDma->ref->CTR1 |= 0b00 << DMA_CTR1_SDW_LOG2_Pos;
    // set destination data width to half word (16 bit)
    spi->rxDma->ref->CTR1 |= 0b01 << DMA_CTR1_DDW_LOG2_Pos;
    // spi->rxDma->ref->CTR1 |= 0b00 << DMA_CTR1_DDW_LOG2_Pos;
    // spi->txDma->ref->CTR1 |= 0b00 << DMA_CTR1_DDW_LOG2_Pos;


    // lastly enable the dma channel
    spi->rxDma->ref->CCR |= DMA_CCR_EN;



    // // set TSIZE - transfer length in words
    // spi->ref->CR2 = 1;

    // master baud rate prescaler = 32
    spi->ref->CFG1 |= spi->CFG1_MBR << SPI_CFG1_MBR_Pos;

    // // master baud rate prescaler = 256
    // spi->ref->CFG1 |= 0b111 << SPI_CFG1_MBR_Pos;

    // spi always controls the state of the gpios,
    // even when disabled (SPE = 0)
    // spi->ref->CFG2 |= SPI_CFG2_AFCNTR;

    // enable hardware SS output
    spi->ref->CFG2 |= SPI_CFG2_SSOE;

    // // SSOM = 1, SP = 000, MIDI > 1
    // // SS is pulsed inactive between data frames
    // spi->ref->CFG2 |= SPI_CFG2_SSOM;

    // configure software management of SS signal input
    // 0: SS input value is determined by the SS PAD
    // 1: SS input value is determined by the SSI bit
    // When master uses hardware SS output (SSM = 0 and SSOE = 1) the SS signal input is
    // forced to not active state internally to prevent master mode fault error.
    // spi->ref->CFG2 |= SPI_CFG2_SSM;

    // spi->ref->CR1 |= SPI_CR1_SSI; // software set NSS signal

    // set clock polarity
    // spi->ref->CFG2 |= SPI_CFG2_CPOL;

    // set clock phase
    // data is captured on the falling edge of SCK
    spi->ref->CFG2 |= SPI_CFG2_CPHA;

    // spi master mode
    spi->ref->CFG2 |= SPI_CFG2_MASTER;

    // // 15 clock cycle periods delay inserted between two consecutive data frames
    // spi->ref->CFG2 |= 0b1111 << SPI_CFG2_MIDI_Pos;

    // // set MSSI to 15
    // // insert 15 clock cycle periods delay between SS opening
    // // a session and the beginning of the first data frame
    // spi->ref->CFG2 |= 0b1111 << SPI_CFG2_MSSI_Pos;

    // enable DMA requests on transmission
    spi->ref->CFG1 |= SPI_CFG1_TXDMAEN;

    // enable TXC TxFIFO transmission complete interrupt
    spi->ref->IER |= SPI_IER_EOTIE;
    switch((uint32_t)spi->ref) {
        case SPI2_BASE:
            NVIC_EnableIRQ(SPI2_IRQn);
            break;
        case SPI4_BASE:
            NVIC_EnableIRQ(SPI4_IRQn);
            break;
        case SPI5_BASE:
            NVIC_EnableIRQ(SPI5_IRQn);
            break;
    }
    // enable rx dma requests
    spi->ref->CFG1 |= SPI_CFG1_RXDMAEN;

    // set DSIZE (frame width) to 16 bits
    spi->ref->CFG1 |= 0b01111 << SPI_CFG1_DSIZE_Pos;
}

// NOTE overruns will result in lost data + corruption
// conditions will not be corrected
// data must be read out before an overrun occurs
uint8_t spi_read(spi_t* spi, uint16_t* word, uint8_t length) {
    uint16_t i = 0;

    uint16_t read_words = spi_rx_waiting(spi);

    if (length < read_words) {
        read_words = length;
    }

    while(i++ < read_words) {
      *word++ = spi->_rx_buffer[spi->_rx_head++];
    }

    return read_words;
}

// rxhead = position of first available byte
// 256 - CNDTR = position of last received byte
// rxhead = 0 CNDTR=256 = 0 data waiting
// rxhead = 0 CNDTR=255 = 1 data waiting
// rxhead = CNDTR-256 = rx empty
uint8_t spi_rx_waiting(spi_t* spi) {
    return 256 - (spi->rxDma->ref->CBR1 & DMA_CBR1_BNDT_Msk)/2 - spi->_rx_head;
}

// how many bytes are waiting to be shifted out
uint8_t spi_tx_waiting(spi_t* spi) {
    return spi->_tx_head - spi->_tx_tail;
}

// number of bytes waiting or number of bytes till the end of the buffer
// whichever is smallest
uint8_t spi_tx_dma_waiting(spi_t* spi) {
    if (spi->_tx_head >= spi->_tx_tail) {
        return spi->_tx_head - spi->_tx_tail;
    } else {
        return spi->_tx_buffer_size - spi->_tx_tail;
    }
}

// how many bytes available to write to buffer
// 255 byte maximum (not 256), this is to prevent overruns
uint8_t spi_tx_available(spi_t* spi)
{
    // must subtract one for tracking empty/patrial[255]/full states
    return 255 - spi_tx_waiting(spi);
}

void spi_start_tx_dma_transfer(spi_t* spi)
{
    // if (spi->txDma->ref->CCR & DMA_CCR_EN) {
    //     // dma busy doing transfer
    //     return;
    // }
    if (spi->ref->CR1 & SPI_CR1_SPE) {
        // spi busy doing transfer
        return;
    }


    spi->_dma_transfer_count = spi_tx_dma_waiting(spi);


    // spi_disable(spi);
    if (spi->_dma_transfer_count) {

        // // disable the spi
        // spi_disable(spi);

        // spi->ref->IFCR = 0xffffffff;
        spi->ref->IFCR |= SPI_IFCR_TXTFC;
        // set TSIZE - transfer length in words
        // spi must be disabled to set TSIZE
        spi->ref->CR2 = spi->_dma_transfer_count;
        // spi->ref->CR2 = 1;


        spi->txDma->ref->CBR1 = spi->_dma_transfer_count*2;
        // spi->txDma->ref->CBR1 = spi->_dma_transfer_count*2;
        spi->txDma->ref->CSAR = (uint32_t)(spi->_tx_buffer + spi->_tx_tail);
        //spi->ref->ICR |= spi_ICR_TCCF; // maybe not necessary
        spi->txDma->ref->CCR |= DMA_CCR_EN;

        // enable the spi
        spi_enable(spi);
        spi_start_transfer(spi);
        // spi->ref->CR1 |= SPI_CR1_CSTART; // spi must be enabled
    }

}

void spi_dma_transfer_complete_isr(spi_t* spi)
{
    // disable dma, so that we can reinitialize
    // memory pointer (CMAR) and number of data to transfer (CNDTR)
    spi->txDma->ref->CCR &= ~DMA_CCR_EN;

    // free available space, move head forward
    spi->_tx_tail += spi->_dma_transfer_count;

    // start transferring fresh data
    // spi_start_tx_dma_transfer(spi);
}

// if there is not space in the buffer, this function will return
// without sending any bytes
void spi_write(spi_t* spi, const uint16_t* data, uint8_t length)
{
    // no blocking
    if (spi_tx_available(spi) < length) {
        return;
    }

    // copy data to tx buffer
    for (uint8_t i = 0; i < length; i++) {
        spi->_tx_buffer[spi->_tx_head++] = data[i];
    }

    // start transferring the data
    spi_start_tx_dma_transfer(spi);
}

uint16_t spi_write_word(spi_t* spi, uint16_t word)
{
    // while (spi->ref->CR1 & SPI_CR1_SPE) {
    //     // spi busy doing transfer
    // }
    spi_disable(spi);
    // for some reason these flags must be cleared here
    // even though they are (at least externally) cleared
    // automatically when disable and enable are called
    spi->ref->IFCR |= SPI_IFCR_TXTFC | SPI_IFCR_EOTC;
    spi->ref->CR2 = 1;
    spi_enable(spi);
    spi->ref->TXDR = word;
    while (!(spi->ref->SR & SPI_SR_TXTF));
    spi_start_transfer(spi);
    while (!(spi->ref->SR & SPI_SR_EOT));
    // while (!(spi->ref->SR & SPI_SR_TXC));
    while (!(spi->ref->SR & SPI_SR_RXP));
    // asm("nop");
    return (uint16_t)spi->ref->RXDR;
}

void spi_enable(spi_t* spi)
{
    spi->ref->CR1 |= SPI_CR1_SPE;
}

void spi_disable(spi_t* spi)
{
    spi->ref->CR1 &= ~(SPI_CR1_SPE);
}

void spi_start_transfer(spi_t* spi)
{
    spi->ref->CR1 |= SPI_CR1_CSTART; // spi must be enabled
}

void SPI_IRQHandler(spi_t* spi)
{
    spi->ref->IFCR |= SPI_IFCR_EOTC;
    spi_disable(spi);
    spi_start_tx_dma_transfer(spi);
    // spi_dma_transfer_complete_isr(&spi);
}

void SPI1_IRQHandler(void)
{
    SPI_IRQHandler(&spis[SPI_1]);
}

void SPI2_IRQHandler(void)
{
    SPI_IRQHandler(&spis[SPI_2]);
}

void SPI3_IRQHandler(void)
{
    SPI_IRQHandler(&spis[SPI_3]);
}

void SPI4_IRQHandler(void)
{
    SPI_IRQHandler(&spis[SPI_4]);
}

void SPI5_IRQHandler(void)
{
    SPI_IRQHandler(&spis[SPI_5]);
}

void SPI6_IRQHandler(void)
{
    SPI_IRQHandler(&spis[SPI_6]);
}
