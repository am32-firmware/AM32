//white wire
#pragma once

#include "stm32h563xx.h"
#include "dma.h"
#include "stdbool.h"

#define GATE_DRIVER_SPI_NSS_PORT GPIOF
#define GATE_DRIVER_SPI_NSS_PIN 6
#define GATE_DRIVER_SPI_NSS_AF 5

// purple wire
#define GATE_DRIVER_SPI_SCK_PORT GPIOF
#define GATE_DRIVER_SPI_SCK_PIN 7
#define GATE_DRIVER_SPI_SCK_AF 5
// grey wire
#define GATE_DRIVER_SPI_MISO_PORT GPIOF
#define GATE_DRIVER_SPI_MISO_PIN 8
#define GATE_DRIVER_SPI_MISO_AF 5
// blue wire
#define GATE_DRIVER_SPI_MOSI_PORT GPIOF
#define GATE_DRIVER_SPI_MOSI_PIN 9
#define GATE_DRIVER_SPI_MOSI_AF 5

#define GATE_DRIVER_SPI_PERIPH SPI5

#define DRV8323_REG_CSA_CONTROL (0x6 << 11)
#define DRV8323_REG_CSA_CONTROL_VALUE 0b01011000001

#define DRV8323_WRITE (0 << 15)
#define DRV8323_READ (1 << 15)

typedef struct {
    SPI_TypeDef* ref;

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

    dmaChannel_t* rxDma;
    dmaChannel_t* txDma;

    uint8_t _irqn;
} spi_t;

void spi_dma_transfer_complete_isr(spi_t* spi);
void spi_initialize(spi_t* spi);
uint8_t spi_read(spi_t* spi, uint16_t* word, uint8_t length);
uint8_t spi_rx_waiting(spi_t* spi);
uint8_t spi_tx_waiting(spi_t* spi);
uint8_t spi_tx_available(spi_t* spi);
void spi_write(spi_t* spi, const uint16_t* data, uint8_t length);
void spi_enable(spi_t* spi);
void spi_start_transfer(spi_t* spi);
// void spi_dma_transfer_complete_isr(spi_t* spi);
// void spi_start_transfer(spi_t* spi);
// void spi_queue_transfer(spi_t* spi, uint16_t word);
// void spi_enable(spi_t* spi);
// void spi_disable(spi_t* spi);


