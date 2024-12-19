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


#define DRV8323_REG_FAULT_STATUS_1 (0x0 << 11)
#define DRV8323_REG_VGS_STATUS_2 (0x1 << 11)
#define DRV8323_REG_DRIVER_CONTROL (0x2 << 11)
#define DRV8323_REG_GATE_DRIVE_HS (0x3 << 11)
#define DRV8323_REG_GATE_DRIVE_LS (0x4 << 11)
#define DRV8323_REG_OCP_CONTROL (0x5 << 11)
#define DRV8323_REG_CSA_CONTROL (0x6 << 11)
#define DRV8323_REG_CSA_CONTROL_VALUE 0b01011000001

#define DRV8323_WRITE (0 << 15)
#define DRV8323_READ (1 << 15)

#define SPI_MBR_DIV_2 0b000
#define SPI_MBR_DIV_4 0b001
#define SPI_MBR_DIV_8 0b010
#define SPI_MBR_DIV_16 0b011
#define SPI_MBR_DIV_32 0b100
#define SPI_MBR_DIV_64 0b101
#define SPI_MBR_DIV_128 0b110
#define SPI_MBR_DIV_256 0b111

typedef struct {
    SPI_TypeDef* ref;

    // circular buffers
    // buffer size MUST be 256
    // implementation takes advantage of integer overflow
    uint16_t* _rx_buffer;
    uint16_t* _tx_buffer;
    uint16_t _rx_buffer_size;
    uint16_t _tx_buffer_size;
    uint8_t _rx_head;
    uint8_t _tx_head;
    uint8_t _tx_tail;
    uint8_t _dma_transfer_count;

    dmaChannel_t* rxDma;
    dmaChannel_t* txDma;

    uint8_t txDmaRequest;
    uint8_t rxDmaRequest;
    uint8_t CFG1_MBR;
    uint32_t CFG2;
    uint8_t _irqn;
} spi_t;

typedef enum {
    SPI_1 = 0,
    SPI_2 = 1,
    SPI_3 = 2,
    SPI_4 = 3,
    SPI_5 = 4,
    SPI_6 = 5,
} spi_e;

extern spi_t spis[];

void spi_dma_transfer_complete_isr(spi_t* spi);
void spi_configure_rcc_clock_selection(spi_t* spi, uint8_t selection);
void spi_initialize(spi_t* spi);
uint8_t spi_read(spi_t* spi, uint16_t* word, uint8_t length);
uint8_t spi_rx_waiting(spi_t* spi);
uint8_t spi_tx_waiting(spi_t* spi);
uint8_t spi_tx_available(spi_t* spi);
void spi_write(spi_t* spi, const uint16_t* data, uint8_t length);
void spi_write_dma(spi_t* spi, const uint16_t* data, uint8_t length);
bool spi_busy(spi_t* spi);
uint16_t spi_write_word(spi_t* spi, uint16_t word);
void spi_enable(spi_t* spi);
void spi_disable(spi_t* spi);
void spi_start_transfer(spi_t* spi);
void spi_reset_buffers(spi_t* spi);
// void spi_dma_transfer_complete_isr(spi_t* spi);
// void spi_start_transfer(spi_t* spi);
// void spi_queue_transfer(spi_t* spi, uint16_t word);
// void spi_enable(spi_t* spi);
// void spi_disable(spi_t* spi);


