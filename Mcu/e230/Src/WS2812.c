/*
 * WS2812.c
 *
 *  Created on: Sep 9, 2020
 *      Author: Alka
 *
 *  Modified for GD32E230 on: Oct 23, 2025
 *       Author: Scarittagle
 */

#include "WS2812.h"

#include "targets.h"

char dma_busy;

uint16_t led_Buffer[RGB_BUFFER_SIZE] = {
    0,
    0,
    20,
    20,
    20,
    20,
    20,
    20,
    20,
    20,
    60,
    60,
    60,
    60,
    60,
    60,
    60,
    60,
    20,
    20,
    20,
    20,
    20,
    20,
    20,
    20,
    0,
    0,
};

void send_LED_DMA()
{
    dma_busy = 1;
    DMA_CHCTL(LED_DMA_CHANNEL) &= ~DMA_CHXCTL_CHEN;
    DMA_CHMADDR(LED_DMA_CHANNEL) = (uint32_t)led_Buffer;
    DMA_CHCNT(LED_DMA_CHANNEL) = RGB_BUFFER_SIZE & DMA_CHANNEL_CNT_MASK;
    DMA_CHCTL(LED_DMA_CHANNEL) |= DMA_CHXCTL_CHEN;
}

void send_LED_RGB(uint8_t red, uint8_t green, uint8_t blue)
{
    if (!dma_busy) {
        uint32_t twenty_four_bit_color_number = green << 16 | red << 8 | blue;

        for (int i = 0; i < 24; i++) {
            led_Buffer[i + 2] = (((twenty_four_bit_color_number >> (23 - i)) & 1) * 31) + 28;
            //                   Bit 0: 0*31+28 = 28 (~311ns HIGH, ~939ns LOW) - blog CODE_0
            //                   Bit 1: 1*31+28 = 59 (~656ns HIGH, ~594ns LOW) - blog CODE_1
        }
        send_LED_DMA();
    }
}

void WS2812_Init(void)
{
    // NVIC Settings
    NVIC_SetPriority(DMA_Channel3_4_IRQn, 1);
    NVIC_EnableIRQ(DMA_Channel3_4_IRQn);

    // Periph Clocks settings
    RCU_CLOCK_ENABLE(RCU_DMA);

    #ifdef LED_USES_PB4
    RCU_CLOCK_ENABLE(RCU_TIMER2);
    RCU_CLOCK_ENABLE(RCU_GPIOB);
    #endif
    #ifdef LED_USES_PA2
    RCU_CLOCK_ENABLE(RCU_TIMER14);
    RCU_CLOCK_ENABLE(RCU_GPIOA);
    #endif

    // Configure GPIO
    //PA2 --> TIM14
    //PB4 --> TIM2
    #ifdef LED_USES_PB4// that means the esc is using PA2 as signal pin, so we configure PB4 as ws2812 data pin
    gpioSetMode(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_4);
    gpioSetOutputOptions(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_4);
    gpioSetAf(GPIOB, GPIO_AF_1, GPIO_PIN_4);
    #endif
    #ifdef LED_USES_PA2 // same as above
    gpioSetMode(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_2);
    gpioSetOutputOptions(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_2);
    gpioSetAf(GPIOA, GPIO_AF_0, GPIO_PIN_2);
    #endif

    // Configure Timer: reset, then edge aligned up counting at 72mhz / 90 = 800khz
    if (LED_TIMER_REGISTER == TIMER2) {
        RCU_RESET_PULSE(RCU_TIMER2RST);
    } else {
        RCU_RESET_PULSE(RCU_TIMER14RST);
    }
    TIMER_PSC(LED_TIMER_REGISTER) = 0;
    TIMER_CAR(LED_TIMER_REGISTER) = 89; // 800KHz
    if (LED_TIMER_REGISTER == TIMER14) {
        TIMER_CREP(LED_TIMER_REGISTER) = 0;
    }
    TIMER_SWEVG(LED_TIMER_REGISTER) |= (uint32_t)TIMER_SWEVG_UPG;

    // CH0 pwm0 output enabled, high polarity, no compare shadow, idle high (TIMER14 only has idle state)
    TIMER_CH0CV(LED_TIMER_REGISTER) = 0;
    TIMER_CHCTL0(LED_TIMER_REGISTER) = (TIMER_CHCTL0(LED_TIMER_REGISTER)
        & ~(uint32_t)(TIMER_CHCTL0_CH0MS | TIMER_CHCTL0_CH0COMCTL | TIMER_CHCTL0_CH0COMSEN)) | TIMER_OC_MODE_PWM0;
    TIMER_CHCTL2(LED_TIMER_REGISTER) = (TIMER_CHCTL2(LED_TIMER_REGISTER)
        & ~(uint32_t)(TIMER_CHCTL2_CH0P | TIMER_CHCTL2_CH0NEN | TIMER_CHCTL2_CH0NP)) | TIMER_CHCTL2_CH0EN;
    if (LED_TIMER_REGISTER == TIMER14) {
        TIMER_CTL1(LED_TIMER_REGISTER) = (TIMER_CTL1(LED_TIMER_REGISTER)
            & ~(uint32_t)(TIMER_CTL1_ISO0 | TIMER_CTL1_ISO0N)) | TIMER_OC_IDLE_STATE_HIGH;
    }
    TIMER_CCHP(LED_TIMER_REGISTER) |= (uint32_t)TIMER_CCHP_POEN;
    // one DMA transfer to CH0CV on each update
    TIMER_DMACFG(LED_TIMER_REGISTER) = (TIMER_DMACFG(LED_TIMER_REGISTER) & ~(uint32_t)(TIMER_DMACFG_DMATA | TIMER_DMACFG_DMATC))
        | TIMER_DMACFG_DMATA_CH0CV | TIMER_DMACFG_DMATC_1TRANSFER;
    TIMER_DMAINTEN(LED_TIMER_REGISTER) |= (uint32_t)TIMER_DMA_UPD;
    TIMER_CTL0(LED_TIMER_REGISTER) |= (uint32_t)(TIMER_CTL0_ARSE | TIMER_CTL0_CEN);

    // DMA: led_Buffer half words to CH0CV, memory increment, high priority, not circular,
    // transfer complete and error interrupts
    dmaDeinit(LED_DMA_CHANNEL);
    DMA_CHPADDR(LED_DMA_CHANNEL) = (uint32_t)&TIMER_CH0CV(LED_TIMER_REGISTER);
    DMA_CHMADDR(LED_DMA_CHANNEL) = (uint32_t)&led_Buffer;
    DMA_CHCNT(LED_DMA_CHANNEL) = RGB_BUFFER_SIZE & DMA_CHANNEL_CNT_MASK;
    DMA_CHCTL(LED_DMA_CHANNEL) = DMA_PERIPHERAL_WIDTH_16BIT | DMA_MEMORY_WIDTH_16BIT | DMA_PRIORITY_HIGH
        | DMA_CHXCTL_MNAGA | DMA_CHXCTL_DIR | DMA_INT_FTF | DMA_INT_ERR;
    DMA_CHCTL(LED_DMA_CHANNEL) |= DMA_CHXCTL_CHEN;

}

