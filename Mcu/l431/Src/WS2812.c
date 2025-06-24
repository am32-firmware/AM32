#include "WS2812.h"

#include "targets.h"

#ifdef USE_LED_STRIP

#ifndef WS2812_PORT
    #error "WS2812_PORT is not defined"
#endif

#ifndef WS2812_PIN
    #error "WS2812_PIN is not defined"
#endif

static void enableDWT_CycleCounter(void)
{
    // Enable TRCENA in the DEMCR (Debug Exception and Monitor Control Register)
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    // Enable the cycle counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    // Reset the cycle counter
    DWT->CYCCNT = 0;
}

static void disableDWT_CycleCounter(void)
{
    // Disable the cycle counter
    DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;
    // Disable TRCENA in the DEMCR
    CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk;
}

static void waitClockCycles(uint16_t cycles)
{
    DWT->CYCCNT = 0; // Reset the cycle counter
    while (DWT->CYCCNT < cycles) {
        // Wait until the specified number of cycles has passed
    }
}

static void sendBit(uint8_t inbit)
{
    WS2812_PORT->BSRR = WS2812_PIN;
    waitClockCycles(CPU_FREQUENCY_MHZ >> (2 - inbit));
    WS2812_PORT->BRR = WS2812_PIN;
    waitClockCycles(CPU_FREQUENCY_MHZ >> (1 + inbit));
}

void send_LED_RGB(uint8_t red, uint8_t green, uint8_t blue)
{
    __disable_irq();
    enableDWT_CycleCounter();
    uint32_t twenty_four_bit_color_number = green << 16 | red << 8 | blue;
    for (int i = 0; i < 24; i++) {
        sendBit((twenty_four_bit_color_number >> (23 - i)) & 1);
    }
    WS2812_PORT->BSRR = (1 << (WS2812_PIN + 16)); // Set pin low by writing to BSRR high half
	disableDWT_CycleCounter();
    __enable_irq();
}

void WS2812_Init(void)
{
    LL_GPIO_SetPinMode(WS2812_PORT, WS2812_PIN, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinSpeed(WS2812_PORT, WS2812_PIN, LL_GPIO_SPEED_FREQ_HIGH);
    LL_GPIO_SetPinOutputType(WS2812_PORT, WS2812_PIN, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinPull(WS2812_PORT, WS2812_PIN, LL_GPIO_PULL_NO);
}

#endif