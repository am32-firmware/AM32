#include "targets.h"
#include "IO.h"
#include "dshot.h"
#include "serial_telemetry.h"
#include "functions.h"
#include "common.h"

uint8_t buffer_padding = 7;
char ic_timer_prescaler = CPU_FREQUENCY_MHZ / 6;
uint32_t dma_buffer[64] = { 0 };
char out_put = 0;

void receiveDshotDma(){
    LL_APB2_GRP1_ForceReset(LL_APB2_GRP1_PERIPH_TIM15); // de-init timer 2
    LL_APB2_GRP1_ReleaseReset(LL_APB2_GRP1_PERIPH_TIM15);
    IC_TIMER_REGISTER->CCMR1 = 0x1;
    IC_TIMER_REGISTER->CCER = 0xa;

    IC_TIMER_REGISTER->PSC = ic_timer_prescaler;
    IC_TIMER_REGISTER->ARR = 0xFFFF;
    IC_TIMER_REGISTER->EGR |= TIM_EGR_UG;
    out_put = 0;
    IC_TIMER_REGISTER->CNT = 0;
    DMA1_Channel5->CMAR = (uint32_t)&dma_buffer;
    DMA1_Channel5->CPAR = (uint32_t)&IC_TIMER_REGISTER->CCR1;
    DMA1_Channel5->CNDTR = buffersize;
    DMA1_Channel5->CCR = 0x98b;
    IC_TIMER_REGISTER->DIER |= TIM_DIER_CC1DE;
    IC_TIMER_REGISTER->CCER |= IC_TIMER_CHANNEL;
    IC_TIMER_REGISTER->CR1 |= TIM_CR1_CEN;
}

void sendDshotDma(){
    LL_APB2_GRP1_ForceReset(LL_APB2_GRP1_PERIPH_TIM15); // de-init timer 2
    LL_APB2_GRP1_ReleaseReset(LL_APB2_GRP1_PERIPH_TIM15);
    IC_TIMER_REGISTER->CCMR1 = 0x60;
    IC_TIMER_REGISTER->CCER = 0x3;

    IC_TIMER_REGISTER->PSC = output_timer_prescaler;
    IC_TIMER_REGISTER->ARR = 115;
    out_put = 1;
    LL_TIM_GenerateEvent_UPDATE(IC_TIMER_REGISTER);

    DMA1_Channel5->CMAR = (uint32_t)&gcr;
    DMA1_Channel5->CPAR = (uint32_t)&IC_TIMER_REGISTER->CCR1;
    DMA1_Channel5->CNDTR = 23 + buffer_padding;
    DMA1_Channel5->CCR = 0x99b;
    IC_TIMER_REGISTER->DIER |= TIM_DIER_CC1DE;
    IC_TIMER_REGISTER->CCER |= IC_TIMER_CHANNEL;
    IC_TIMER_REGISTER->BDTR |= TIM_BDTR_MOE;
    IC_TIMER_REGISTER->CR1 |= TIM_CR1_CEN;
}

uint8_t getInputPinState() { return (INPUT_PIN_PORT->IDR & INPUT_PIN); }

void setInputPolarityRising()
{
    LL_TIM_IC_SetPolarity(IC_TIMER_REGISTER, IC_TIMER_CHANNEL, LL_TIM_IC_POLARITY_RISING);
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