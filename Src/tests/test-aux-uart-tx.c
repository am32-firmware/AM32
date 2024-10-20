#include "stm32h563xx.h"

#include "targets.h"
#include "usart.h"
#include "gpio.h"
#include "dma.h"

#include "clock.h"
static uint8_t usart_rx_buffer[256];
static uint8_t usart_tx_buffer[256];
static usart_t usart;

int main()
{
    clock_hse_enable();

    clock_pll1_set_source(CLOCK_PLL1_SRC_HSE);

    // HSE frequency is 25MHz
    // set prescaler to 25 for 1MHz input clock
    clock_pll1_configure_prescaler(25);
    
    // clock_system_set_source(CLOCK_SYS_SRC_HSE);

    // set system clock frequency to 64MHz
    clock_hsi_config_divider(CLOCK_HSI_DIV1);

    clock_update_hclk_frequency();

    dma_initialize();
    AUX_UART_ENABLE_CLOCK();

    usart.ref = AUX_UART_PERIPH;

    usart._rx_buffer = usart_rx_buffer;
    usart._tx_buffer = usart_tx_buffer;
    usart._rx_buffer_size = 256;
    usart._tx_buffer_size = 256;
    usart.rxDma = &dmaChannels[7];
    usart.txDma = &dmaChannels[0];
    usart.txDmaRequest = LL_GPDMA1_REQUEST_UART8_TX;

    usart._baudrate = 1000000;
    usart_initialize(&usart);

    gpio_t gpioUsartTx = DEF_GPIO(
        AUX_UART_TX_PORT,
        AUX_UART_TX_PIN,
        AUX_UART_TX_AF,
        GPIO_AF);
    gpio_initialize(&gpioUsartTx);

    while(1) {
        usart_write(&usart, "U", 1);
        // usart_write_string(&usart, "hello world\n");
    }
}