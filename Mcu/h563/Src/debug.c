#include "debug.h"
#include "usart.h"
#include "gpio.h"
#include "dma.h"
#include "targets.h"

usart_t usartDebug;
uint8_t usartDebugTxBuffer[256];
uint8_t usartDebugRxBuffer[256];

void debug_initialize()
{


DEBUG_USART_ENABLE_CLOCK();
    gpio_t gpioUsartRx = DEF_GPIO(DEBUG_USART_RX_PORT, DEBUG_USART_RX_PIN, DEBUG_USART_RX_AF, GPIO_AF);
    gpio_t gpioUsartTx = DEF_GPIO(DEBUG_USART_TX_PORT, DEBUG_USART_TX_PIN, DEBUG_USART_TX_AF, GPIO_AF);
    gpio_initialize(&gpioUsartRx);
    gpio_initialize(&gpioUsartTx);

    usartDebug.ref = DEBUG_USART_REF;

    usartDebug._rx_buffer = usartDebugRxBuffer;
    usartDebug._tx_buffer = usartDebugTxBuffer;
    usartDebug._rx_buffer_size = 256;
    usartDebug._tx_buffer_size = 256;
    usartDebug.rxDma = &dmaChannels[DEBUG_USART_RX_DMA_CHANNEL];
    usartDebug.txDma = &dmaChannels[DEBUG_USART_TX_DMA_CHANNEL];
    usartDebug.txDmaRequest = DEBUG_USART_DMA_REQ;

    usartDebug._baudrate = 3000000;
    usartDebug.swap = DEBUG_USART_SWAP_IO;
    usart_initialize(&usartDebug);
}

void debug_write_string(const char* string)
{
  usart_write_string(&usartDebug, string);
}
