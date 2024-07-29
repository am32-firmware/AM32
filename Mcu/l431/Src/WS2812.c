#include "WS2812.h"

#include "functions.h"
#include "targets.h"

#ifdef USE_LED_STRIP

void waitClockCycles(uint16_t cycles)
{
    // UTILITY_TIMER->CNT = 0;
    // while (UTILITY_TIMER->CNT < cycles) {
    // }
    DWT->CYCCNT = 0; // Reset the cycle counter
    while (DWT->CYCCNT < cycles) {
        // Wait until the specified number of cycles has passed
    }
}

void sendBit(uint8_t inbit)
{
    WS2812_PORT->BSRR = WS2812_PIN;
    waitClockCycles(CPU_FREQUENCY_MHZ >> (2 - inbit));
    WS2812_PORT->BRR = WS2812_PIN;
    waitClockCycles(CPU_FREQUENCY_MHZ >> (1 + inbit));
}

void send_LED_RGB(uint8_t red, uint8_t green, uint8_t blue)
{
    __disable_irq();
    uint32_t twenty_four_bit_color_number = green << 16 | red << 8 | blue;
    for (int i = 0; i < 24; i++) {
        sendBit((twenty_four_bit_color_number >> (23 - i)) & 1);
    }
    WS2812_PORT->BSRR = (1 << (WS2812_PIN + 16)); // Set pin low by writing to BSRR high half
    __enable_irq();
}

void WS2812_Init(void)
{
    LL_GPIO_SetPinMode(WS2812_PORT, WS2812_PIN, LL_GPIO_MODE_OUTPUT);
    // Ensure the DWT unit is enabled
    if (!(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk)) {
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; // Enable the cycle counter
    }
}

#endif