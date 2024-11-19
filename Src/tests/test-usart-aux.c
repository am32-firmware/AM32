#include "stm32h563xx.h"

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
    mcu_setup();
    AUX_USART_ENABLE_CLOCK();
    gpio_t gpioUsartRx = DEF_GPIO(AUX_USART_RX_PORT, AUX_USART_RX_PIN, AUX_USART_RX_AF, GPIO_AF);
    gpio_t gpioUsartTx = DEF_GPIO(AUX_USART_TX_PORT, AUX_USART_TX_PIN, AUX_USART_TX_AF, GPIO_AF);
    gpio_initialize(&gpioUsartRx);
    gpio_initialize(&gpioUsartTx);
    gpio_set_speed(&gpioUsartRx, GPIO_SPEED_VERYFAST);
    gpio_set_speed(&gpioUsartTx, GPIO_SPEED_VERYFAST);

    usart.ref = AUX_USART_TX_PERIPH;

    usart._rx_buffer = usart_rx_buffer;
    usart._tx_buffer = usart_tx_buffer;
    usart._rx_buffer_size = 256;
    usart._tx_buffer_size = 256;
    usart.rxDma = &dmaChannels[7];
    usart.txDma = &dmaChannels[0];
    usart.txDmaRequest = AUX_USART_TX_DMA_REQ;
    usart.rxDmaRequest = AUX_USART_RX_DMA_REQ;

    usart._baudrate = 115200;
    usart.swap = 0;
    usart_initialize(&usart);

    while(1) {
        usart_write_string(&usart, "hello world\n");
    }
}