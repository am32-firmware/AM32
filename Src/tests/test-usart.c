#include "stm32h563xx.h"
// nucleo
// #define USART_RX_PORT GPIOD
// #define USART_RX_PIN 9
// #define USART_RX_AF 7

// #define USART_TX_PORT GPIOD
// #define USART_TX_PIN 8
// #define USART_TX_AF 7

// AUX, MAIN, DEBUG
#define DEBUG_USART_RX_PORT GPIOC
#define DEBUG_USART_RX_PIN 11
#define DEBUG_USART_RX_AF 8

#define TEST_USART_RX_PORT DEBUG_USART_RX_PORT
#define TEST_USART_RX_PIN DEBUG_USART_RX_PIN
#define TEST_USART_RX_AF DEBUG_USART_RX_AF

#define DEBUG_USART_TX_PORT GPIOC
#define DEBUG_USART_TX_PIN 10
#define DEBUG_USART_TX_AF 8

#define TEST_USART_TX_PORT DEBUG_USART_TX_PORT
#define TEST_USART_TX_PIN DEBUG_USART_TX_PIN
#define TEST_USART_TX_AF DEBUG_USART_TX_AF

#define DEBUG_USART_REF UART4

#define TEST_USART_REF DEBUG_USART_REF

#define DEBUG_USART_BAUDRATE 1000000

#define TEST_USART_BAUDRATE DEBUG_USART_BAUDRATE

#define TEST_USART_SWAP_IO 1

#include "dma.h"
#include "gpio.h"
#include "mcu.h"
#include "targets.h"
#include "usart.h"

static uint8_t usart_rx_buffer[256];
static uint8_t usart_tx_buffer[256];
static usart_t usart;

int main()
{
    mcu_setup();
    // DEBUG_USART_ENABLE_CLOCK();
    RCC->AHB1ENR |= RCC_AHB1ENR_GPDMA1EN;
    gpio_t gpioUsartRx = DEF_GPIO(TEST_USART_RX_PORT, TEST_USART_RX_PIN, TEST_USART_RX_AF, GPIO_AF);
    gpio_t gpioUsartTx = DEF_GPIO(TEST_USART_TX_PORT, TEST_USART_TX_PIN, TEST_USART_TX_AF, GPIO_AF);
    gpio_initialize(&gpioUsartRx);
    gpio_initialize(&gpioUsartTx);

    usart.ref = TEST_USART_REF;

    usart._rx_buffer = usart_rx_buffer;
    usart._tx_buffer = usart_tx_buffer;
    usart._rx_buffer_size = 256;
    usart._tx_buffer_size = 256;
    usart.rxDma = &dmaChannels[7];
    usart.txDma = &dmaChannels[0];

    usart._baudrate = TEST_USART_BAUDRATE;

    usart.swap = TEST_USART_SWAP_IO;
    usart_initialize(&usart);

    while(1) {
        usart_write_string(&usart, "hello world\n");
    }
}