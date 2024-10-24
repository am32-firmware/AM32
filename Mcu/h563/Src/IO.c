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

// #include "signal.h"
char ic_timer_prescaler = (CPU_FREQUENCY_MHZ / 6);
uint32_t dma_buffer[64] = { 0 };
char out_put = 0;
uint8_t buffer_padding = 0;

extern void transfercomplete(); // defined in signal.c

void io_dma_cb(dmaChannel_t* dma)
{
    dma->ref->CFCR |= DMA_CFCR_TCF;
    transfercomplete();
    EXTI->SWIER1 |= EXTI_SWIER1_SWI15;
}

void sendDshotDma()
{}

void receiveDshotDma()
{
    out_put = 0;
    INPUT_TIMER_RESET();
    
    // configure channel as input mode
    // can only be done while channel is disabled
    // INPUT_TIMER->CCMR1 = 0x01;
    INPUT_TIMER_CCMR_CONFIG();

    // enable channel
    // INPUT_TIMER->CCER = 0xa;
    INPUT_TIMER_CCER_CONFIG();
    INPUT_TIMER->PSC = ic_timer_prescaler;
    INPUT_TIMER->ARR = 0xFFFF;
    INPUT_TIMER->EGR |= TIM_EGR_UG;

    INPUT_TIMER->CNT = 0;

    dmaChannel_t* dmaCh = &dmaChannels[INPUT_TIMER_DMA_CHANNEL];
    dmaCh->callback = io_dma_cb;
    dmaCh->ref->CDAR = (uint32_t)&dma_buffer;
    dmaCh->ref->CSAR = INPUT_TIMER_CCR;
    // for H5 BNDT = BYTEs to be transferred
    // for other stm32 BNDT = number of transfers
    dmaCh->ref->CBR1 = buffersize*2;
    // dmaCh->ref->CTR2 = DMA_REQ_INPUT_TIMER;
    dmaCh->ref->CTR2 = INPUT_TIMER_DMA_REQ;
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
    INPUT_TIMER->DIER |= INPUT_TIMER_DIER_CCDE;
    INPUT_TIMER->CCER |= IC_TIMER_CHANNEL;
    INPUT_TIMER->CR1 |= TIM_CR1_CEN;
}

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
