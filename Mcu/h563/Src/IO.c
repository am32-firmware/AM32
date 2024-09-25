/*
 * IO.c
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 */

#include "IO.h"

#include "dma.h"
#include "common.h"
#include "dshot.h"
#include "functions.h"
#include "serial_telemetry.h"
#include "stm32h563xx.h"
#include "targets.h"

char ic_timer_prescaler = (CPU_FREQUENCY_MHZ / 6);
uint32_t dma_buffer[64] = { 0 };
char out_put = 0;
uint8_t buffer_padding = 0;

void receiveDshotDma()
{
    out_put = 0;
    INPUT_TIMER_RESET();
    
    INPUT_TIMER->CCMR1 = 0x01;
    INPUT_TIMER->CCER = 0xa;
    INPUT_TIMER->PSC = ic_timer_prescaler;
    INPUT_TIMER->ARR = 0xFFFF;
    INPUT_TIMER->EGR |= TIM_EGR_UG;

    INPUT_TIMER->CNT = 0;

    dmaChannel_t* dmaCh = &dmaChannels[INPUT_TIMER_DMA_CHANNEL];
    dmaCh->ref->CDAR = (uint32_t)&dma_buffer;
    dmaCh->ref->CSAR = (uint32_t)&INPUT_TIMER->CCR1;
    dmaCh->ref->CBR1 = buffersize;
    dmaCh->ref->CTR2 = LL_GPDMA1_REQUEST_TIM1_CH1;
    dmaCh->ref->CTR1 |=
        DMA_CTR1_DINC | // destination incrementing burst
        (0b10 << DMA_CTR1_DDW_LOG2_Pos) | // 32 bit destination
        (0b01 << DMA_CTR1_SDW_LOG2_Pos); // 16 bit source
    dmaCh->ref->CCR |=
        DMA_CCR_DTEIE | // data transfer error interrupt enable
        DMA_CCR_TCIE | // transfer complete interrupt enable
        DMA_CCR_EN; // enable
    // dmaCh->ref->CCR = 0x98b;


    // msize 0b10
    // psize 0b01
    // MINC
    // teie
    // tcie
    // enable
    INPUT_TIMER->DIER |= TIM_DIER_CC1DE;
    INPUT_TIMER->CCER |= IC_TIMER_CHANNEL;
    INPUT_TIMER->CR1 |= TIM_CR1_CEN;
}

// void sendDshotDma()
// {
//     out_put = 1;
// #ifdef USE_TIMER_3_CHANNEL_1
//     //          // de-init timer 2
//     RCC->APB1RSTR |= LL_APB1_GRP1_PERIPH_TIM3;
//     RCC->APB1RSTR &= ~LL_APB1_GRP1_PERIPH_TIM3;
// #endif
// #ifdef USE_TIMER_15_CHANNEL_1
//     RCC->APB2RSTR |= LL_APB1_GRP2_PERIPH_TIM15;
//     RCC->APB2RSTR &= ~LL_APB1_GRP2_PERIPH_TIM15;
// #endif
//     INPUT_TIMER->CCMR1 = 0x60;
//     INPUT_TIMER->CCER = 0x3;
//     INPUT_TIMER->PSC = output_timer_prescaler;
//     INPUT_TIMER->ARR = 61;

//     INPUT_TIMER->EGR |= TIM_EGR_UG;
// #ifdef USE_TIMER_3_CHANNEL_1
//     dma_channels[INPUT_TIMER_DMA_CHANNEL]->CMAR = (uint32_t)&gcr;
//     dma_channels[INPUT_TIMER_DMA_CHANNEL]->CPAR = (uint32_t)&INPUT_TIMER->CCR1;
//     dma_channels[INPUT_TIMER_DMA_CHANNEL]->CNDTR = 23 + buffer_padding;
//     dma_channels[INPUT_TIMER_DMA_CHANNEL]->CCR = 0x99b;
// #endif
// #ifdef USE_TIMER_15_CHANNEL_1
//     //		  LL_DMA_ConfigAddresses(DMA1, INPUT_DMA_CHANNEL,
//     //(uint32_t)&gcr, (uint32_t)&INPUT_TIMER->CCR1,
//     // LL_DMA_GetDataTransferDirection(DMA1,
//     // INPUT_DMA_CHANNEL));
//     DMA1_Channel5->CMAR = (uint32_t)&gcr;
//     DMA1_Channel5->CPAR = (uint32_t)&INPUT_TIMER->CCR1;
//     DMA1_Channel5->CNDTR = 23 + buffer_padding;
//     DMA1_Channel5->CCR = 0x99b;
// #endif
//     INPUT_TIMER->DIER |= TIM_DIER_CC1DE;
//     INPUT_TIMER->CCER |= IC_TIMER_CHANNEL;
//     INPUT_TIMER->BDTR |= TIM_BDTR_MOE;
//     INPUT_TIMER->CR1 |= TIM_CR1_CEN;
// }

uint8_t getInputPinState() { return (INPUT_SIGNAL_PORT->IDR & INPUT_SIGNAL_LL_PIN); }

void setInputPolarityRising()
{
    LL_TIM_IC_SetPolarity(INPUT_TIMER, IC_TIMER_CHANNEL,
        LL_TIM_IC_POLARITY_RISING);
}

void setInputPullDown()
{
    LL_GPIO_SetPinPull(INPUT_SIGNAL_PORT, INPUT_SIGNAL_LL_PIN, LL_GPIO_PULL_DOWN);
}

void setInputPullUp()
{
    LL_GPIO_SetPinPull(INPUT_SIGNAL_PORT, INPUT_SIGNAL_LL_PIN, LL_GPIO_PULL_UP);
}

void setInputPullNone()
{
    LL_GPIO_SetPinPull(INPUT_SIGNAL_PORT, INPUT_SIGNAL_LL_PIN, LL_GPIO_PULL_NO);
}
