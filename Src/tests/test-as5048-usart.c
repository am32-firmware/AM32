// This example sets the zero position
// of the as5048 sensor using the
// as5048-spi.h driver
// the zero position setting is olitile in this example
// the sensor supports a one time programming (OTP)
// of the zero position registers, and the
// value of these registers is then kept in a permanent
// manner
// this example does not permanently program the zero position
// add the global variable `angle` to live watch
// to see the sensor angle reading

#include "as5048-spi.h"
#include "gpio.h"
#include "led.h"
#include "mcu.h"
#include "targets.h"
#include "usart.h"

as5048_t as5048;

uint32_t current_angle;

static uint8_t map(uint32_t value, uint32_t in_low, uint32_t in_high, uint32_t out_low, uint32_t out_high)
{
    if (value < in_low) {
        return out_low;
    } else if (value > in_high) {
        return out_high;
    }

    uint32_t in_range = in_high - in_low;

    uint32_t difference = value - in_low;

    uint32_t out_range = out_high - out_low;

    uint32_t output = out_range * difference / in_range;
    return output;

}
uint32_t angle_to_color(uint16_t angle)
{
    uint32_t tmp = angle * 1000;
    uint32_t threshold = ((1<<14) * 1000) / 3;

    uint32_t r = 0;
    uint32_t g = 0;
    uint32_t b = 0;
    if (tmp < threshold) {
        r = map(tmp, 0, threshold, 0, 0xff);
        g = 0xff - r;
    // } else if (tmp < 2*threshold) {
    //     g = map(tmp - threshold, 0, threshold, 0, 0xff);
    //     b = 0xff - g;
    // } else {
    //     b = map(tmp - 2*threshold, 0, threshold, 0, 0xff);
    //     r = 0xff - b;
    }
    return (b << 16) | (r << 8) | g;
}

uint8_t usart_rx_buffer[256];
uint8_t usart_tx_buffer[256];
usart_t usart;


void usart_init()
{
    DEBUG_USART_ENABLE_CLOCK();
    gpio_t gpioUsartRx = DEF_GPIO(DEBUG_USART_RX_PORT, DEBUG_USART_RX_PIN, DEBUG_USART_RX_AF, GPIO_AF);
    gpio_t gpioUsartTx = DEF_GPIO(DEBUG_USART_TX_PORT, DEBUG_USART_TX_PIN, DEBUG_USART_TX_AF, GPIO_AF);
    gpio_initialize(&gpioUsartRx);
    gpio_initialize(&gpioUsartTx);
    gpio_set_speed(&gpioUsartRx, GPIO_SPEED_VERYFAST);
    gpio_set_speed(&gpioUsartTx, GPIO_SPEED_VERYFAST);

    usart.ref = DEBUG_USART_REF;

    usart._rx_buffer = usart_rx_buffer;
    usart._tx_buffer = usart_tx_buffer;
    usart._rx_buffer_size = 256;
    usart._tx_buffer_size = 256;
    usart.rxDma = &dmaChannels[DEBUG_USART_RX_DMA_CHANNEL];
    usart.txDma = &dmaChannels[DEBUG_USART_TX_DMA_CHANNEL];
    usart.txDmaRequest = DEBUG_USART_TX_DMA_REQ;
    usart.rxDmaRequest = DEBUG_USART_RX_DMA_REQ;

    usart._baudrate = DEBUG_USART_BAUDRATE;
    usart.swap = DEBUG_USART_SWAP_IO;
    usart_initialize(&usart);
}

int main()
{
    mcu_setup(250);
    as5048_initialize(&as5048);
    led_initialize();
    usart_init();

    as5048_set_zero_position(&as5048);

    while(1) {
        current_angle = as5048_get_angle_degrees(&as5048);
        usart_write_int(&usart, current_angle);
    }
}
