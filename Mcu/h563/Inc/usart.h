#pragma once

#include "stm32h563xx.h"
#include "dma.h"

typedef struct {
    USART_TypeDef* ref;

    // circular buffers
    // buffer size MUST be 256
    // implementation takes advantage of integer overflow
    uint8_t* _rx_buffer;
    uint8_t* _tx_buffer;
    uint16_t _rx_buffer_size;
    uint16_t _tx_buffer_size;
    uint8_t _rx_head;
    uint8_t _tx_head;
    uint8_t _tx_tail;
    uint8_t _dma_transfer_count;
    uint32_t _baudrate;

    dmaChannel_t* rxDma;
    dmaChannel_t* txDma;

    uint8_t txDmaRequest;

    uint8_t _irqn;

    // swap the rx/tx signals and pads
    uint8_t swap;
} usart_t;

extern usart_t usarts[];

void usart_dma_transfer_complete_isr(usart_t* usart);
void usart_initialize(usart_t* usart);
uint8_t usart_read(usart_t* usart, uint8_t* byte, uint8_t length);
uint8_t usart_rx_waiting(usart_t* usart);
uint8_t usart_tx_waiting(usart_t* usart);
uint8_t usart_tx_available(usart_t* usart);
void usart_write(usart_t* usart, const uint8_t* data, uint8_t length);
void usart_write_large_data(usart_t* usart, uint8_t* data, uint16_t length);
void usart_write_string(usart_t* usart, const char* string);
void usart_write_int(usart_t* usart, uint32_t i);
