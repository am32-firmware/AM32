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
    dma_channel_disable(LED_DMA_CHANNEL); 
    dma_memory_address_config(LED_DMA_CHANNEL, (uint32_t)led_Buffer);
    dma_transfer_number_config(LED_DMA_CHANNEL, RGB_BUFFER_SIZE);
    dma_channel_enable(LED_DMA_CHANNEL);
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
    // Init Timer&DMA config structures
    timer_oc_parameter_struct timer_ocinitpara;
    timer_parameter_struct timer_initpara;
    dma_parameter_struct dma_initpara;

    // NVIC Settings
    NVIC_SetPriority(DMA_Channel3_4_IRQn, 1);
    NVIC_EnableIRQ(DMA_Channel3_4_IRQn);

    // Periph Clocks settings
    rcu_periph_clock_enable(RCU_DMA);

    #ifdef LED_USES_PB4
    rcu_periph_clock_enable(RCU_TIMER2);
    rcu_periph_clock_enable(RCU_GPIOB);
    #endif
    #ifdef LED_USES_PA2
    rcu_periph_clock_enable(RCU_TIMER14);
    rcu_periph_clock_enable(RCU_GPIOA);
    #endif

    // Configure GPIO
    //PA2 --> TIM14
    //PB4 --> TIM2
    #ifdef LED_USES_PB4// that means the esc is using PA2 as signal pin, so we configure PB4 as ws2812 data pin
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_4);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_4);
    gpio_af_set(GPIOB, GPIO_AF_1, GPIO_PIN_4);
    #endif
    #ifdef LED_USES_PA2 // same as above
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_2);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_2);
    gpio_af_set(GPIOA, GPIO_AF_0, GPIO_PIN_2);
    #endif

    // Configure Timer
    timer_deinit(LED_TIMER_REGISTER);
    timer_initpara.prescaler = 0;
    timer_initpara.alignedmode = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection = TIMER_COUNTER_UP;  
    timer_initpara.period = 89; // 800KHz
    timer_initpara.clockdivision = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(LED_TIMER_REGISTER, &timer_initpara);

    // Configure Timer Channel
    timer_ocinitpara.outputstate = TIMER_CCX_ENABLE;  
    timer_ocinitpara.outputnstate = TIMER_CCXN_DISABLE;
    timer_ocinitpara.ocpolarity = TIMER_OC_POLARITY_HIGH;
    timer_ocinitpara.ocnpolarity = TIMER_OCN_POLARITY_HIGH;
    timer_ocinitpara.ocidlestate = TIMER_OC_IDLE_STATE_HIGH;  
    timer_ocinitpara.ocnidlestate = TIMER_OC_IDLE_STATE_LOW;

    timer_channel_output_config(LED_TIMER_REGISTER, TIMER_CH_0, &timer_ocinitpara);
    timer_channel_output_pulse_value_config(LED_TIMER_REGISTER, TIMER_CH_0, 0);
    timer_channel_output_mode_config(LED_TIMER_REGISTER, TIMER_CH_0, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(LED_TIMER_REGISTER, TIMER_CH_0, TIMER_OC_SHADOW_DISABLE);
    timer_primary_output_config(LED_TIMER_REGISTER, ENABLE);
    timer_dma_transfer_config(LED_TIMER_REGISTER, TIMER_DMACFG_DMATA_CH0CV, TIMER_DMACFG_DMATC_1TRANSFER);
    timer_dma_enable(LED_TIMER_REGISTER, TIMER_DMA_UPD);
    timer_auto_reload_shadow_enable(LED_TIMER_REGISTER);
    timer_enable(LED_TIMER_REGISTER);

    dma_deinit(LED_DMA_CHANNEL);
    dma_initpara.periph_addr = (uint32_t)&TIMER_CH0CV(LED_TIMER_REGISTER);
    dma_initpara.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_initpara.memory_addr = (uint32_t)&led_Buffer;
    dma_initpara.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_initpara.periph_width = DMA_PERIPHERAL_WIDTH_16BIT;
    dma_initpara.memory_width = DMA_MEMORY_WIDTH_16BIT;
    dma_initpara.direction = DMA_MEMORY_TO_PERIPHERAL;
    dma_initpara.number = RGB_BUFFER_SIZE;
    dma_initpara.priority = DMA_PRIORITY_HIGH;
    dma_init(LED_DMA_CHANNEL, &dma_initpara);
    dma_circulation_disable(LED_DMA_CHANNEL);  // Normal mode, not circular
    dma_memory_to_memory_disable(LED_DMA_CHANNEL);
    dma_interrupt_enable(LED_DMA_CHANNEL, DMA_INT_FTF);  // Enable interrupts in init
    dma_interrupt_enable(LED_DMA_CHANNEL, DMA_INT_ERR);
    dma_channel_enable(LED_DMA_CHANNEL);

}

