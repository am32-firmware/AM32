// this example tests rs485 transmit on the blueesc
// the rs485 driver must be enabled

#include "stm32h563xx.h"

#include "dma.h"
#include "gpio.h"
#include "mcu.h"
#include "usart.h"
#include "targets.h"

uint8_t usart_rx_buffer[256];
uint8_t usart_tx_buffer[256];
usart_t usart;


int main()
{
    mcu_setup();
    MAIN_USART_ENABLE_CLOCK();



    usart.ref = MAIN_USART_REF;

    usart._rx_buffer = usart_rx_buffer;
    usart._tx_buffer = usart_tx_buffer;
    usart._rx_buffer_size = 256;
    usart._tx_buffer_size = 256;
    usart.rxDma = &dmaChannels[7];
    usart.txDma = &dmaChannels[0];
    usart.txDmaRequest = MAIN_USART_DMA_REQ;

    usart._baudrate = 115200;
    usart.swap = 0;
    usart_initialize(&usart);

    gpio_t gpioUsartRx = DEF_GPIO(MAIN_USART_RX_PORT, MAIN_USART_RX_PIN, MAIN_USART_RX_AF, GPIO_AF);
    gpio_t gpioUsartTx = DEF_GPIO(MAIN_USART_TX_PORT, MAIN_USART_TX_PIN, MAIN_USART_TX_AF, GPIO_AF);
    // gpio_initialize(&gpioUsartRx);
    gpio_initialize(&gpioUsartTx);

    gpio_t gpioRS485Enable = DEF_GPIO(RS485_ENABLE_PORT, RS485_ENABLE_PIN, 0, GPIO_OUTPUT);
    gpio_initialize(&gpioRS485Enable);
    gpio_set(&gpioRS485Enable);

    while(1) {
        usart_write_string(&usart, "hello world\n");
    }
}