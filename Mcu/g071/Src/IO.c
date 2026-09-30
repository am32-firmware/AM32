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

uint8_t buffer_padding = 7;
char ic_timer_prescaler = CPU_FREQUENCY_MHZ / 6;
uint32_t dma_buffer[64] = { 0 };
volatile char out_put = 0;

#ifdef ULTRA_DEDICATED
extern void processDshot(void);

// ultra mode: the input capture DMA runs without a transfer complete
// interrupt and is drained here from the 20kHz loop; a packet is complete
// once the line has been idle for more than 2x the expected bit time.
// DMA_start_bit stays valid until the next accepted packet: the software
// EXTI that decodes it only preempts this routine on some MCUs
void runDshotCheck()
{
    if (DMA1_Channel1->CNDTR < 63) {
        if (armed) {
            if ((IC_TIMER_REGISTER->CNT - dma_buffer[63 - DMA1_Channel1->CNDTR]) > (uint32_t)(valid_packet_high << 1)) {
                if (ultraPacketStart(dma_buffer, DMA1_Channel1->CNDTR, valid_packet_high, &DMA_start_bit)) {
                    transfercomplete();
                    EXTI->SWIER1 |= LL_EXTI_LINE_15;
                } else {
                    packet_length_badcounts++;
                }
                LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
                DMA1_Channel1->CNDTR = 64;
                LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
                IC_TIMER_REGISTER->CNT = 0;
            }
        } else {
            if (DMA1_Channel1->CNDTR <= 32) {
                DMA_start_bit = 0;
                transfercomplete();
                processDshot();
                LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
                DMA1_Channel1->CNDTR = 64;
                LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
                IC_TIMER_REGISTER->CNT = 0;
            }
        }
    }
}
#endif // ULTRA_DEDICATED

void receiveDshotDma()
{
#ifdef USE_TIMER_3_CHANNEL_1
    RCC->APBRSTR1 |= LL_APB1_GRP1_PERIPH_TIM3;
    RCC->APBRSTR1 &= ~LL_APB1_GRP1_PERIPH_TIM3;
    IC_TIMER_REGISTER->CCMR1 = 0x41;
    IC_TIMER_REGISTER->CCER = 0xa;
#endif
#ifdef USE_TIMER_16_CHANNEL_1
    LL_APB2_GRP1_ForceReset(LL_APB2_GRP1_PERIPH_TIM16); // de-init timer 2
    LL_APB2_GRP1_ReleaseReset(LL_APB2_GRP1_PERIPH_TIM16);
    IC_TIMER_REGISTER->CCMR1 = 0x41;
    IC_TIMER_REGISTER->CCER = 0xa;
#endif

    IC_TIMER_REGISTER->PSC = ic_timer_prescaler;
    IC_TIMER_REGISTER->ARR = 0xFFFF;
    IC_TIMER_REGISTER->EGR |= TIM_EGR_UG;
    out_put = 0;
    IC_TIMER_REGISTER->CNT = 0;
    DMA1_Channel1->CMAR = (uint32_t)&dma_buffer;
    DMA1_Channel1->CPAR = (uint32_t)&IC_TIMER_REGISTER->CCR1;
#ifdef ULTRA_DEDICATED
    // polled mode: 64 deep capture, no transfer complete interrupt,
    // lighter input filter - noise is handled in runDshotCheck()
    IC_TIMER_REGISTER->CCMR1 = 0x21;
    DMA1_Channel1->CNDTR = 64;
    DMA1_Channel1->CCR = 0x989;
#else
    DMA1_Channel1->CNDTR = buffersize;
    DMA1_Channel1->CCR = 0x98b;
#endif
    IC_TIMER_REGISTER->DIER |= TIM_DIER_CC1DE;
    IC_TIMER_REGISTER->CCER |= IC_TIMER_CHANNEL;
    IC_TIMER_REGISTER->CR1 |= TIM_CR1_CEN;
}

void sendDshotDma()
{
#ifdef USE_TIMER_3_CHANNEL_1
    RCC->APBRSTR1 |= LL_APB1_GRP1_PERIPH_TIM3;
    RCC->APBRSTR1 &= ~LL_APB1_GRP1_PERIPH_TIM3;
    IC_TIMER_REGISTER->CCMR1 = 0x60;
    IC_TIMER_REGISTER->CCER = 0x3;
#endif
#ifdef USE_TIMER_16_CHANNEL_1
    LL_APB2_GRP1_ForceReset(LL_APB2_GRP1_PERIPH_TIM16); // de-init timer 2
    LL_APB2_GRP1_ReleaseReset(LL_APB2_GRP1_PERIPH_TIM16);
    IC_TIMER_REGISTER->CCMR1 = 0x60;
    IC_TIMER_REGISTER->CCER = 0x3;
#endif
    IC_TIMER_REGISTER->PSC = output_timer_prescaler;
    IC_TIMER_REGISTER->ARR = 92;
    out_put = 1;
    LL_TIM_GenerateEvent_UPDATE(IC_TIMER_REGISTER);

    DMA1_Channel1->CMAR = (uint32_t)&gcr;
    DMA1_Channel1->CPAR = (uint32_t)&IC_TIMER_REGISTER->CCR1;
    DMA1_Channel1->CNDTR = 23 + buffer_padding;
    DMA1_Channel1->CCR = 0x99b;
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
