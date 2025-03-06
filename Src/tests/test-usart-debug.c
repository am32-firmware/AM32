// This example transmits some data ("hello world\n")
// on the debug usart interface
// (programming connector)
#include "stm32h563xx.h"

#define TEST_USART_RX_PORT DEBUG_USART_RX_PORT
#define TEST_USART_RX_PIN DEBUG_USART_RX_PIN
#define TEST_USART_RX_AF DEBUG_USART_RX_AF

#define TEST_USART_TX_PORT DEBUG_USART_TX_PORT
#define TEST_USART_TX_PIN DEBUG_USART_TX_PIN
#define TEST_USART_TX_AF DEBUG_USART_TX_AF

#define TEST_USART_REF DEBUG_USART_REF

#define TEST_USART_BAUDRATE DEBUG_USART_BAUDRATE

#define TEST_USART_DMA_REQ DEBUG_USART_TX_DMA_REQ
#define TEST_USART_SWAP_IO 0

#include "dma.h"
#include "gpio.h"
#include "mcu.h"
#include "usart.h"
#include "targets.h"

static uint8_t usart_rx_buffer[256];
static uint8_t usart_tx_buffer[256];
static usart_t usart;

int main()
{
    mcu_setup(250);
    DEBUG_USART_ENABLE_CLOCK();
    gpio_t gpioUsartRx = DEF_GPIO(TEST_USART_RX_PORT, TEST_USART_RX_PIN, TEST_USART_RX_AF, GPIO_AF);
    gpio_t gpioUsartTx = DEF_GPIO(TEST_USART_TX_PORT, TEST_USART_TX_PIN, TEST_USART_TX_AF, GPIO_AF);
    gpio_initialize(&gpioUsartRx);
    gpio_initialize(&gpioUsartTx);
    gpio_set_speed(&gpioUsartRx, GPIO_SPEED_VERYFAST);
    gpio_set_speed(&gpioUsartTx, GPIO_SPEED_VERYFAST);

    usart.ref = TEST_USART_REF;

    usart._rx_buffer = usart_rx_buffer;
    usart._tx_buffer = usart_tx_buffer;
    usart._rx_buffer_size = 256;
    usart._tx_buffer_size = 256;
    usart.rxDma = &dmaChannels[DEBUG_USART_RX_DMA_CHANNEL];
    usart.txDma = &dmaChannels[DEBUG_USART_TX_DMA_CHANNEL];
    usart.txDmaRequest = TEST_USART_DMA_REQ;

    // usart._baudrate = TEST_USART_BAUDRATE;
    usart._baudrate = 3000000;
    usart.swap = TEST_USART_SWAP_IO;
    usart_initialize(&usart);

    while(1) {
        usart_write_string(&usart, "hello world\n");
    }
}