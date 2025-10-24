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

#define RGB_BUFFER_SIZE 28

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

    #ifdef USE_TIMER_14_CHANNEL_0
    timer_counter_value_config(TIMER2, 0);  // Reset timer count to 0
    dma_interrupt_enable(DMA_CH2, DMA_INT_FTF);
    dma_interrupt_enable(DMA_CH2, DMA_INT_ERR);
    dma_channel_enable(DMA_CH2);
    timer_dma_enable(TIMER2, TIMER_DMA_CH0D);
    timer_channel_output_enable(TIMER2, TIMER_CH_0);
    timer_primary_output_config(TIMER2, ENABLE);
    timer_enable(TIMER2);
    #endif
    #ifdef USE_TIMER_2_CHANNEL_0
    timer_counter_value_config(TIMER14, 0);  // Reset timer count to 0
    dma_interrupt_enable(DMA_CH2, DMA_INT_FTF);
    dma_interrupt_enable(DMA_CH2, DMA_INT_ERR);
    dma_channel_enable(DMA_CH2);
    timer_dma_enable(TIMER14, TIMER_DMA_CH0D);
    timer_channel_output_enable(TIMER14, TIMER_CH_0);
    timer_primary_output_config(TIMER14, ENABLE);
    timer_enable(TIMER14);
    #endif
}

void send_LED_RGB(uint8_t red, uint8_t green, uint8_t blue)
{
    if (!dma_busy) {
        uint32_t twenty_four_bit_color_number = green << 16 | red << 8 | blue;

        for (int i = 0; i < 24; i++) {
            led_Buffer[i + 2] = (((twenty_four_bit_color_number >> (23 - i)) & 1) * 25) + 20;
            //                   Bit 0: 0*25+20 = 20 (277.8ns HIGH, 972.2ns LOW)
            //                   Bit 1: 1*25+20 = 45 (625ns HIGH, 625ns LOW)
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
    NVIC_SetPriority(DMA_Channel1_2_IRQn, 3);
    NVIC_EnableIRQ(DMA_Channel1_2_IRQn);

    // Periph Clocks settings
    rcu_periph_clock_enable(RCU_DMA);

    #ifdef USE_TIMER_14_CHANNEL_0
    rcu_periph_clock_enable(RCU_TIMER2);
    rcu_periph_clock_enable(RCU_GPIOB);
    #endif
    #ifdef USE_TIMER_2_CHANNEL_0
    rcu_periph_clock_enable(RCU_TIMER14);
    rcu_periph_clock_enable(RCU_GPIOA);
    #endif

    // Configure GPIO
    //PA2 --> TIM14
    //PB4 --> TIM2
    #ifdef USE_TIMER_14_CHANNEL_0 // that means the esc is using PA2 as signal pin, so we configure PB4 as ws2812 data pin
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_4);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_4);
    gpio_af_set(GPIOB, GPIO_AF_1, GPIO_PIN_4);
    #endif
    #ifdef USE_TIMER_2_CHANNEL_0 // same as above
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_2);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_2);
    gpio_af_set(GPIOA, GPIO_AF_0, GPIO_PIN_2);
    #endif

    // Configure Timer
    #ifdef USE_TIMER_14_CHANNEL_0
    timer_deinit(TIMER2);
    timer_initpara.prescaler = 0;
    timer_initpara.alignedmode = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection = TIMER_COUNTER_DOWN;
    timer_initpara.period = 89; // 800KHz
    timer_initpara.clockdivision = TIMER_CLOCKDIVISION_DIV1;
    timer_initpara.repetitioncounter = 0;

    // Configure Timer Channel
    timer_ocinitpara.outputstate = TIMER_CCX_DISABLE;
    timer_ocinitpara.outputnsate = TIMER_CCXN_DISABLE;
    timer_ocinitpara.ocpolarity = TIMER_OCPOLARITY_HIGH;
    timer_ocinitpara.ocnpolarity = TIMER_OCNPOLARITY_HIGH;
    timer_ocinitpara.ocidle = TIMER_OCIDLESTATE_LOW;
    timer_ocinitpara.ocnidle = TIMER_OCNIDLESTATE_LOW;

    timer_init(TIMER2, &timer_initpara);
    timer_auto_reload_shadow_enable(TIMER2);
    timer_channel_output_shadow_config(TIMER2, TIMER_CH_0, TIMER_OC_SHADOW_ENABLE);
    timer_channel_output_config(TIMER2, TIMER_CH_0, &timer_ocinitpara);
    timer_channel_output_pulse_value_config(TIMER2, TIMER_CH_0, 0);
    timer_channel_output_mode_config(TIMER2, TIMER_CH_0, TIMER_OC_MODE_PWM1);
    timer_channel_output_fast_config(TIMER2, TIMER_CH_0, TIMER_OC_FAST_DISABLE);


    // Configure DMA
    dma_deinit(DMA_CH2);
    dma_initpara.periph_addr = (uint32_t)(&((TIMER_TypeDef *)TIMER2)->CH0CV);
    dma_initpara.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_initpara.memory_addr = (uint32_t)&led_Buffer;
    dma_initpara.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_initpara.periph_width = DMA_PERIPHERAL_WIDTH_16BIT;
    dma_initpara.memory_width = DMA_MEMORY_WIDTH_16BIT;
    dma_initpara.direction = DMA_MEMORY_TO_PERIPHERAL;
    dma_initpara.number = RGB_BUFFER_SIZE;
    dma_initpara.priority = DMA_PRIORITY_HIGH;
    dma_init(DMA_CH2, &dma_initpara);
    #endif
    #ifdef USE_TIMER_2_CHANNEL_0
    timer_deinit(TIMER14);
    timer_initpara.prescaler = 0;
    timer_initpara.alignedmode = TIMER_COUNTER_DOWN;
    timer_initpara.counterdirection = TIMER_COUNTER_DOWN;
    timer_initpara.period = 89; // 800KHz
    timer_initpara.clockdivision = TIMER_CLOCKDIVISION_DIV1;
    timer_initpara.repetitioncounter = 0;

    // Configure Timer Channel
    timer_ocinitpara.outputstate = TIMER_CCX_DISABLE;
    timer_ocinitpara.outputnsate = TIMER_CCXN_DISABLE;
    timer_ocinitpara.ocpolarity = TIMER_OCPOLARITY_HIGH;
    timer_ocinitpara.ocnpolarity = TIMER_OCNPOLARITY_HIGH;
    timer_ocinitpara.ocidle = TIMER_OCIDLESTATE_LOW;
    timer_ocinitpara.ocnidle = TIMER_OCNIDLESTATE_LOW;

    timer_init(TIMER14, &timer_initpara);
    timer_auto_reload_shadow_enable(TIMER14);
    timer_channel_output_shadow_config(TIMER14, TIMER_CH_0, TIMER_OC_SHADOW_ENABLE);
    timer_channel_output_config(TIMER14, TIMER_CH_0, &timer_ocinitpara);
    timer_channel_output_pulse_value_config(TIMER14, TIMER_CH_0, 0);
    timer_channel_output_mode_config(TIMER14, TIMER_CH_0, TIMER_OC_MODE_PWM1);
    timer_channel_output_fast_config(TIMER14, TIMER_CH_0, TIMER_OC_FAST_DISABLE);


    // Configure DMA
    dma_deinit(DMA_CH2);
    dma_initpara.periph_addr = (uint32_t)(&((TIMER_TypeDef *)TIMER14)->CH0CV);
    dma_initpara.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_initpara.memory_addr = (uint32_t)&led_Buffer;
    dma_initpara.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_initpara.periph_width = DMA_PERIPHERAL_WIDTH_16BIT;
    dma_initpara.memory_width = DMA_MEMORY_WIDTH_16BIT;
    dma_initpara.direction = DMA_MEMORY_TO_PERIPHERAL;
    dma_initpara.number = RGB_BUFFER_SIZE;
    dma_initpara.priority = DMA_PRIORITY_HIGH;
    dma_init(DMA_CH2, &dma_initpara);
    #endif

}

