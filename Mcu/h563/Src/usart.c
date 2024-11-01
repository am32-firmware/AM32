#include "usart.h"
#include "stm32h563xx.h"
// #include "vector.h"
#include <string.h>
#include "clock.h"

extern uint32_t HCLK_FREQUENCY;

void usart_dma_cb(dmaChannel_t* dma)
{
    // dma->ref->CFCR |= DMA_IT_TCIF << dma->flagsShift;
    dma->ref->CFCR |= DMA_CFCR_TCF;
    usart_dma_transfer_complete_isr((usart_t*)dma->userParam);
}

void usart_initialize(usart_t* usart)
{
    // if (!usart) {
    //     return;
    // }

    // set the channel destination address
    usart->txDma->ref->CDAR = (uint32_t)&usart->ref->TDR;
    // set the channel source address
    usart->txDma->ref->CSAR = (uint32_t)usart->_tx_buffer;
    // // set the transfer length
    // usart->txDma->ref->CBR1 = 256;
    // set source incrementing burst
    usart->txDma->ref->CTR1 |= DMA_CTR1_SINC;
    // set the peripheral hardware request selection
    usart->txDma->ref->CTR2 |= usart->txDmaRequest;
    // enable transfer complete interrupt
    usart->txDma->ref->CCR |= DMA_CCR_TCIE;
    usart->txDma->callback = usart_dma_cb;
    usart->txDma->userParam = (uint32_t)usart;

    NVIC_SetPriority(usart->txDma->irqn, 0);
    NVIC_EnableIRQ(usart->txDma->irqn);

    // set the source address
    usart->rxDma->ref->CSAR = (uint32_t)&usart->ref->RDR;
    // set the destination address
    usart->rxDma->ref->CDAR = (uint32_t)usart->_rx_buffer;
    // set destination incrementing burst
    usart->rxDma->ref->CTR1 |= DMA_CTR1_DINC;

    // set the transfer length
    usart->rxDma->ref->CBR1 = 256;
    // set the block repeated destination address offset
    usart->rxDma->ref->CBR2 |= 256 << DMA_CBR2_BRDAO_Pos;
    usart->rxDma->ref->CBR1 |= DMA_CBR1_BRDDEC;

    // configure single LLI to run repeatedly
    usart->rxDma->ref->CLLR = 0x08000004 & DMA_CLLR_LA_Msk;


    // set the hardware request selections
    usart->rxDma->ref->CTR2 |= 25;


    // lastly enable the dma channel
    usart->rxDma->ref->CCR |= DMA_CCR_EN;

    // enable transmitter, receiver, and oversampling by 8
    // NOTE: oversampling by 8 must be used for max baud rate
    usart->ref->CR1 |= USART_CR1_TE | USART_CR1_RE /*| USART_CR1_OVER8*/;
    // usart->ref->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_OVER8;

    // set baudrate
    // for oversampling by 16
    usart->ref->BRR = HCLK_FREQUENCY/usart->_baudrate;
    // for oversampling by 8
    // usart->ref->BRR = 2*64000000/usart->_baudrate;

    // uint32_t usartdiv = 2*170000000 / baudrate;
    // uint16_t brr = (usartdiv & 0xfffffff0) | ((usartdiv & 0x0000000f) >> 1);
    // usart->ref->BRR = brr;

    // enable usart dma requests for receive and transmit
    usart->ref->CR3 |= USART_CR3_DMAR | USART_CR3_DMAT;
    //usart->ref->PRESC |= 0b0101;

    if (usart->swap) {
        // swap the physical RX/TX mapping
        usart->ref->CR2 |= USART_CR2_SWAP;
    }
    // enable usart
    usart->ref->CR1 |= USART_CR1_UE;
}

// NOTE overruns will result in lost data + corruption
// conditions will not be corrected
// data must be read out before an overrun occurs
uint8_t usart_read(usart_t* usart, uint8_t* byte, uint8_t length) {
    uint16_t i = 0;

    uint16_t read_bytes = usart_rx_waiting(usart);

    if (length < read_bytes) {
        read_bytes = length;
    }

    while(i++ < read_bytes) {
      *byte++ = usart->_rx_buffer[usart->_rx_head++];
    }

    return read_bytes;
}

// rxhead = position of first available byte
// 256 - CNDTR = position of last received byte
// rxhead = 0 CNDTR=256 = 0 data waiting
// rxhead = 0 CNDTR=255 = 1 data waiting
// rxhead = CNDTR-256 = rx empty
uint8_t usart_rx_waiting(usart_t* usart) {
    return 256 - (usart->rxDma->ref->CBR1 & DMA_CBR1_BNDT_Msk) - usart->_rx_head;
}

// how many bytes are waiting to be shifted out
uint8_t usart_tx_waiting(usart_t* usart) {
    return usart->_tx_head - usart->_tx_tail;
}

// number of bytes waiting or number of bytes till the end of the buffer
// whichever is smallest
uint8_t usart_tx_dma_waiting(usart_t* usart) {
    if (usart->_tx_head >= usart->_tx_tail) {
        return usart->_tx_head - usart->_tx_tail;
    } else {
        return usart->_tx_buffer_size - usart->_tx_tail;
    }
}

// how many bytes available to write to buffer
// 255 byte maximum (not 256), this is to prevent overruns
uint8_t usart_tx_available(usart_t* usart)
{
    // must subtract one for tracking empty/patrial[255]/full states
    return 255 - usart_tx_waiting(usart);
}

void usart_start_tx_dma_transfer(usart_t* usart)
{
    if (usart->txDma->ref->CCR & DMA_CCR_EN) {
        // dma busy doing transfer
        return;
    }

    usart->_dma_transfer_count = usart_tx_dma_waiting(usart);
    if (usart->_dma_transfer_count) {
        usart->txDma->ref->CBR1 = usart->_dma_transfer_count;
        usart->txDma->ref->CSAR = (uint32_t)(usart->_tx_buffer + usart->_tx_tail);
        //usart->ref->ICR |= USART_ICR_TCCF; // maybe not necessary
        usart->txDma->ref->CCR |= DMA_CCR_EN;
    }
}

void usart_dma_transfer_complete_isr(usart_t* usart)
{
    // disable dma, so that we can reinitialize
    // memory pointer (CMAR) and number of data to transfer (CNDTR)
    usart->txDma->ref->CCR &= ~DMA_CCR_EN;

    // free available space, move head forward
    usart->_tx_tail += usart->_dma_transfer_count;

    // start transferring fresh data
    usart_start_tx_dma_transfer(usart);
}

// if there is not space in the buffer, this function will return
// without sending any bytes
void usart_write(usart_t* usart, const uint8_t* data, uint8_t length)
{
    // no blocking
    if (usart_tx_available(usart) < length) {
        return;
    }

    // copy data to tx buffer
    for (uint8_t i = 0; i < length; i++) {
        usart->_tx_buffer[usart->_tx_head++] = data[i];
    }

    // start transferring the data
    usart_start_tx_dma_transfer(usart);
}

void usart_write_string(usart_t* usart, const char* string)
{
    usart_write(usart, (const uint8_t*)string, strlen(string));
}

void usart_write_int(usart_t* usart, uint32_t i)
{
  // output character buffer
  char c[10];
  // maximum character length for uint32_t
  uint8_t len = 10;

  // first character
  uint8_t p = i % 10;
  c[--len] = p + '0';

  while (i > 9) {
    i /= 10;
    p = i % 10;
    c[--len] = p + '0';
  }

  usart_write(usart, &c[len], 10 - len);
}

void usart_write_large_data(usart_t* usart, uint8_t* data, uint16_t length)
{
    // wait for any on-going transfer to complete
    while (usart->txDma->ref->CCR & DMA_CCR_EN);
    // we must clear the flag first or we will hang sporadically
    usart->txDma->ref->CBR1 = length;
    usart->txDma->ref->CSAR = (uint32_t)(data);
    usart->txDma->ref->CCR |= DMA_CCR_EN;
}
