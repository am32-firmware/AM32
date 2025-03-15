// this example tests rs485 transmit on the blueesc
// the rs485 driver must be enabled

#include "stm32h563xx.h"

#include "dma.h"
#include "gpio.h"
#include "mcu.h"
#include "rs485.h"
#include "usart.h"
#include "targets.h"

uint8_t usart_rx_buffer[256];
uint8_t usart_tx_buffer[256];
usart_t usart;


int main()
{
    mcu_setup(250);
    RS485_USART_ENABLE_CLOCK();

    usart.ref = RS485_USART_REF;

    usart._rx_buffer = usart_rx_buffer;
    usart._tx_buffer = usart_tx_buffer;
    usart._rx_buffer_size = 256;
    usart._tx_buffer_size = 256;
    usart.rxDma = &dmaChannels[7];
    usart.txDma = &dmaChannels[0];
    usart.txDmaRequest = RS485_USART_TX_DMA_REQ;

    usart._baudrate = 3000000;
    usart.swap = 0;
    usart_initialize(&usart);

    gpio_t gpioUsartRx = DEF_GPIO(RS485_USART_RX_PORT, RS485_USART_RX_PIN, RS485_USART_RX_AF, GPIO_AF);
    gpio_t gpioUsartTx = DEF_GPIO(RS485_USART_TX_PORT, RS485_USART_TX_PIN, RS485_USART_TX_AF, GPIO_AF);
    // gpio_initialize(&gpioUsartRx);
    gpio_initialize(&gpioUsartTx);

    rs485_initialize();
    rs485_enable();

    while(1) {
        usart_write_string(&usart, "hello world\n");
    }
}
