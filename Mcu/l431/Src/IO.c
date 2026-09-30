/*
 * IO.c
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 */

#include "IO.h"

#include "common.h"
#include "dshot.h"
#include "functions.h"
#include "serial_telemetry.h"
#include "signal.h"
#include "targets.h"
#include "ultra.h"

char ic_timer_prescaler = (CPU_FREQUENCY_MHZ / 4);
uint32_t dma_buffer[64] = { 0 };
volatile char out_put = 0;
uint8_t buffer_padding = 0;

#ifdef ULTRA_DEDICATED
extern void processDshot(void);

#ifdef USE_TIMER_3_CHANNEL_1
#define ULTRA_INPUT_DMA DMA1_Channel4
#endif
#ifdef USE_TIMER_15_CHANNEL_1
#define ULTRA_INPUT_DMA DMA1_Channel5
#endif

// ultra mode: the input capture DMA runs without a transfer complete
// interrupt and is drained here from the 20kHz loop; a packet is complete
// once the line has been idle for more than 2x the expected bit time.
// DMA_start_bit stays valid until the next accepted packet: the software
// EXTI that decodes it only preempts this routine on some MCUs
void runDshotCheck()
{
    if (ULTRA_INPUT_DMA->CNDTR < 63) {
        if (armed) {
            if ((IC_TIMER_REGISTER->CNT - dma_buffer[63 - ULTRA_INPUT_DMA->CNDTR]) > (uint32_t)(valid_packet_high << 1)) {
                if (ultraPacketStart(dma_buffer, ULTRA_INPUT_DMA->CNDTR, valid_packet_high, &DMA_start_bit)) {
                    transfercomplete();
                    EXTI->SWIER1 |= LL_EXTI_LINE_15;
                } else {
                    packet_length_badcounts++;
                }
                LL_DMA_DisableChannel(DMA1, INPUT_DMA_CHANNEL);
                ULTRA_INPUT_DMA->CNDTR = 64;
                LL_DMA_EnableChannel(DMA1, INPUT_DMA_CHANNEL);
                IC_TIMER_REGISTER->CNT = 0;
            }
        } else {
            if (ULTRA_INPUT_DMA->CNDTR <= 32) {
                DMA_start_bit = 0;
                transfercomplete();
                processDshot();
                LL_DMA_DisableChannel(DMA1, INPUT_DMA_CHANNEL);
                ULTRA_INPUT_DMA->CNDTR = 64;
                LL_DMA_EnableChannel(DMA1, INPUT_DMA_CHANNEL);
                IC_TIMER_REGISTER->CNT = 0;
            }
        }
    }
}
#endif // ULTRA_DEDICATED

void receiveDshotDma()
{
    out_put = 0;
#ifdef USE_TIMER_3_CHANNEL_1
    RCC->APB1RSTR |= LL_APB1_GRP1_PERIPH_TIM3;
    RCC->APB1RSTR &= ~LL_APB1_GRP1_PERIPH_TIM3;
#endif
#ifdef USE_TIMER_15_CHANNEL_1
    RCC->APB2RSTR |= LL_APB2_GRP1_PERIPH_TIM15;
    RCC->APB2RSTR &= ~LL_APB2_GRP1_PERIPH_TIM15;
#endif
#ifdef ULTRA_DEDICATED
    // lighter input filter, noise is handled in runDshotCheck()
    IC_TIMER_REGISTER->CCMR1 = 0x21;
#else
    IC_TIMER_REGISTER->CCMR1 = 0x41;
#endif
    IC_TIMER_REGISTER->CCER = 0xa;
    IC_TIMER_REGISTER->PSC = ic_timer_prescaler;
    IC_TIMER_REGISTER->ARR = 0xFFFF;
    IC_TIMER_REGISTER->EGR |= TIM_EGR_UG;

    IC_TIMER_REGISTER->CNT = 0;
#ifdef USE_TIMER_3_CHANNEL_1
    DMA1_Channel4->CMAR = (uint32_t)&dma_buffer;
    DMA1_Channel4->CPAR = (uint32_t)&IC_TIMER_REGISTER->CCR1;
#endif
#ifdef USE_TIMER_15_CHANNEL_1
    DMA1_Channel5->CMAR = (uint32_t)&dma_buffer;
    DMA1_Channel5->CPAR = (uint32_t)&IC_TIMER_REGISTER->CCR1;
#endif
#ifdef ULTRA_DEDICATED
    // polled mode: 64 deep capture, no transfer complete interrupt
    ULTRA_INPUT_DMA->CNDTR = 64;
    ULTRA_INPUT_DMA->CCR = 0x989;
#else
#ifdef USE_TIMER_3_CHANNEL_1
    DMA1_Channel4->CNDTR = buffersize;
    DMA1_Channel4->CCR = 0x98b;
#endif
#ifdef USE_TIMER_15_CHANNEL_1
    DMA1_Channel5->CNDTR = buffersize;
    DMA1_Channel5->CCR = 0x98b;
#endif
#endif
    IC_TIMER_REGISTER->DIER |= TIM_DIER_CC1DE;
    IC_TIMER_REGISTER->CCER |= IC_TIMER_CHANNEL;
    IC_TIMER_REGISTER->CR1 |= TIM_CR1_CEN;
}

void sendDshotDma()
{
    out_put = 1;
#ifdef USE_TIMER_3_CHANNEL_1
    //          // de-init timer 2
    RCC->APB1RSTR |= LL_APB1_GRP1_PERIPH_TIM3;
    RCC->APB1RSTR &= ~LL_APB1_GRP1_PERIPH_TIM3;
#endif
#ifdef USE_TIMER_15_CHANNEL_1
    RCC->APB2RSTR |= LL_APB2_GRP1_PERIPH_TIM15;
    RCC->APB2RSTR &= ~LL_APB2_GRP1_PERIPH_TIM15;
#endif
    IC_TIMER_REGISTER->CCMR1 = 0x60;
    IC_TIMER_REGISTER->CCER = 0x3;
    IC_TIMER_REGISTER->PSC = output_timer_prescaler;
    IC_TIMER_REGISTER->ARR = 110;

    IC_TIMER_REGISTER->EGR |= TIM_EGR_UG;
#ifdef USE_TIMER_3_CHANNEL_1
    DMA1_Channel4->CMAR = (uint32_t)&gcr;
    DMA1_Channel4->CPAR = (uint32_t)&IC_TIMER_REGISTER->CCR1;
    DMA1_Channel4->CNDTR = 23 + buffer_padding;
    DMA1_Channel4->CCR = 0x99b;
#endif
#ifdef USE_TIMER_15_CHANNEL_1
    //		  LL_DMA_ConfigAddresses(DMA1, INPUT_DMA_CHANNEL,
    //(uint32_t)&gcr, (uint32_t)&IC_TIMER_REGISTER->CCR1,
    // LL_DMA_GetDataTransferDirection(DMA1,
    // INPUT_DMA_CHANNEL));
    DMA1_Channel5->CMAR = (uint32_t)&gcr;
    DMA1_Channel5->CPAR = (uint32_t)&IC_TIMER_REGISTER->CCR1;
    DMA1_Channel5->CNDTR = 23 + buffer_padding;
    DMA1_Channel5->CCR = 0x99b;
#endif
    IC_TIMER_REGISTER->DIER |= TIM_DIER_CC1DE;
    IC_TIMER_REGISTER->CCER |= IC_TIMER_CHANNEL;
    IC_TIMER_REGISTER->BDTR |= TIM_BDTR_MOE;
    IC_TIMER_REGISTER->CR1 |= TIM_CR1_CEN;
}

uint8_t getInputPinState() { return (INPUT_PIN_PORT->IDR & INPUT_PIN); }

void setInputPolarityRising()
{
    LL_TIM_IC_SetPolarity(IC_TIMER_REGISTER, IC_TIMER_CHANNEL,
        LL_TIM_IC_POLARITY_RISING);
}

void setInputPullDown()
{
    LL_GPIO_SetPinPull(INPUT_PIN_PORT, INPUT_PIN, LL_GPIO_PULL_DOWN);
}

void setInputPullUp()
{
    LL_GPIO_SetPinPull(INPUT_PIN_PORT, INPUT_PIN, LL_GPIO_PULL_UP);
}

void enableHalfTransferInt() { LL_DMA_EnableIT_HT(DMA1, INPUT_DMA_CHANNEL); }
void setInputPullNone()
{
    LL_GPIO_SetPinPull(INPUT_PIN_PORT, INPUT_PIN, LL_GPIO_PULL_NO);
}
