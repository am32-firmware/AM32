#include "stm32h563xx.h"

#include "dma.h"
#include "gpio.h"
#include "mcu.h"
#include "usart.h"
#include "targets.h"
#include "vreg.h"

static uint8_t usart_rx_buffer[256];
static uint8_t usart_tx_buffer[256];
static usart_t usart;

int main()
{
    mcu_setup(250);

    vreg5V_initialize();
    vreg5V_enable();

    gpio_t gpioRS485Enable = DEF_GPIO(
        RS485_ENABLE_PORT,
        RS485_ENABLE_PIN,
        0,
        GPIO_OUTPUT);
    gpio_initialize(&gpioRS485Enable);
    gpio_set_speed(&gpioRS485Enable, GPIO_SPEED_VERYFAST);
    gpio_reset(&gpioRS485Enable);

    MAIN_USART_ENABLE_CLOCK();
    gpio_t gpioUsartRx = DEF_GPIO(MAIN_USART_RX_PORT, MAIN_USART_RX_PIN, MAIN_USART_RX_AF, GPIO_AF);
    gpio_t gpioUsartTx = DEF_GPIO(MAIN_USART_TX_PORT, MAIN_USART_TX_PIN, MAIN_USART_TX_AF, GPIO_AF);
    gpio_initialize(&gpioUsartRx);
    gpio_initialize(&gpioUsartTx);
    gpio_set_speed(&gpioUsartRx, GPIO_SPEED_VERYFAST);
    gpio_set_speed(&gpioUsartTx, GPIO_SPEED_VERYFAST);

    // gpio_configure_pupdr(&gpioUsartRx, GPIO_PULL_UP);
    gpio_configure_pupdr(&gpioUsartTx, GPIO_PULL_UP);

    usart.ref = MAIN_USART_REF;

    usart._rx_buffer = usart_rx_buffer;
    usart._tx_buffer = usart_tx_buffer;
    usart._rx_buffer_size = 256;
    usart._tx_buffer_size = 256;
    usart.rxDma = &dmaChannels[7];
    usart.txDma = &dmaChannels[0];
    usart.rxDmaRequest = MAIN_USART_RX_DMA_REQ;
    usart.txDmaRequest = MAIN_USART_TX_DMA_REQ;

    usart._baudrate = 6000000;
    usart.swap = 0;
    usart_initialize(&usart);

    while(1) {
        usart_write_string(&usart, "hello world\n");
    }
}