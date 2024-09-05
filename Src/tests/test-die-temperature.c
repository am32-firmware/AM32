#include "stm32h563xx.h"
#define USART_RX_PORT GPIOD
#define USART_RX_PIN 9
#define USART_RX_AF 7

#define USART_TX_PORT GPIOD
#define USART_TX_PIN 8
#define USART_TX_AF 7
#include "targets.h"
#include "ADC.h"
#include "usart.h"
#include "gpio.h"
#include "dma.h"

static uint8_t usart_rx_buffer[256];
static uint8_t usart_tx_buffer[256];
static usart_t usart;

int main()
{
    adc_initialize(ADC1);
    uint8_t channels[] = {16};
    adc_set_regular_sequence(ADC1, channels, 1);
    adc_enable(ADC1);
    adc_start(ADC1);
    adc_set_continuous_mode(ADC1, true);
    adc_set_sample_time(ADC1, 16, 0b111);


    RCC->AHB1ENR |= RCC_AHB1ENR_GPDMA1EN;
    gpio_t gpioUsartRx = DEF_GPIO(USART_RX_PORT, USART_RX_PIN, USART_RX_AF, GPIO_AF);
    gpio_t gpioUsartTx = DEF_GPIO(USART_TX_PORT, USART_TX_PIN, USART_TX_AF, GPIO_AF);
    gpio_initialize(&gpioUsartRx);
    gpio_initialize(&gpioUsartTx);

    usart.ref = USART3;

    usart._rx_buffer = usart_rx_buffer;
    usart._tx_buffer = usart_tx_buffer;
    usart._rx_buffer_size = 256;
    usart._tx_buffer_size = 256;
    usart.rxDma = &dmaChannels[7];
    usart.txDma = &dmaChannels[0];

    usart_initialize(&usart);

    while(1) {
        usart_write_int(&usart, ADC1->DR);
        usart_write_string(&usart, "\n\r");
    }
}