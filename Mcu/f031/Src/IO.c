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
#include "targets.h"

char ic_timer_prescaler = (CPU_FREQUENCY_MHZ / 5);
// char output_timer_prescaler;
// int buffersize = 32;
// int smallestnumber = 20000;
uint32_t dma_buffer[64];
char out_put = 0;
// char buffer_divider = 44;
// int dshot_runout_timer = 62500;
// uint32_t average_signal_pulse;
uint8_t buffer_padding = 0;

void changeToOutput()
{
    LL_DMA_SetDataTransferDirection(DMA1, INPUT_DMA_CHANNEL,
        LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    //	LL_TIM_DeInit(IC_TIMER_REGISTER);
    // MX_TIM2_Init(

#ifdef USE_TIMER_2_CHANNEL_3

    LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_TIM2); // de-init timer 2
    LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_TIM2);
    IC_TIMER_REGISTER->CCMR2 = 0x60;
    IC_TIMER_REGISTER->CCER = 0x200;
#endif

#ifdef USE_TIMER_2_CHANNEL_4
    LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_TIM2); // de-init timer 2
    LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_TIM2);
    IC_TIMER_REGISTER->CCMR2 = 0x6000;
    IC_TIMER_REGISTER->CCER = 0x2000;
#endif
#ifdef USE_TIMER_16
    LL_APB1_GRP2_ForceReset(LL_APB1_GRP2_PERIPH_TIM16);
    LL_APB1_GRP2_ReleaseReset(LL_APB1_GRP2_PERIPH_TIM16);
    IC_TIMER_REGISTER->CCMR1 = 0x60;
    IC_TIMER_REGISTER->CCER = 0x3;
#endif
#ifdef USE_TIMER_2_CHANNEL_1

    LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_TIM2); // de-init timer 2
    LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_TIM2);
    IC_TIMER_REGISTER->CCMR1 = 0x60;
    IC_TIMER_REGISTER->CCER = 0x3;
#endif
    IC_TIMER_REGISTER->PSC = output_timer_prescaler;
    IC_TIMER_REGISTER->ARR = 61;
    out_put = 1;
    LL_TIM_GenerateEvent_UPDATE(IC_TIMER_REGISTER);
}

void changeToInput()
{
    LL_DMA_SetDataTransferDirection(DMA1, INPUT_DMA_CHANNEL,
        LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

#ifdef USE_TIMER_2_CHANNEL_3
    LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_TIM2); // de-init timer 2
    LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_TIM2);
    IC_TIMER_REGISTER->CCMR2 = 0x71;
    IC_TIMER_REGISTER->CCER = 0xa00;
#endif

#ifdef USE_TIMER_2_CHANNEL_4
    LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_TIM2); // de-init timer 2
    LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_TIM2);
    IC_TIMER_REGISTER->CCMR2 = 0x100;
    IC_TIMER_REGISTER->CCER = 0xa000;
#endif
#ifdef USE_TIMER_16
    LL_APB1_GRP2_ForceReset(LL_APB1_GRP2_PERIPH_TIM16);
    LL_APB1_GRP2_ReleaseReset(LL_APB1_GRP2_PERIPH_TIM16);
    IC_TIMER_REGISTER->CCMR1 = 0x71;
    IC_TIMER_REGISTER->CCER = 0xa;
#endif
#ifdef USE_TIMER_2_CHANNEL_1
    LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_TIM2); // de-init timer 2
    LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_TIM2);
    IC_TIMER_REGISTER->CCMR1 = 0x71;
    IC_TIMER_REGISTER->CCER = 0xa;
#endif
    IC_TIMER_REGISTER->PSC = ic_timer_prescaler;
    IC_TIMER_REGISTER->ARR = 0xFFFF;
    LL_TIM_GenerateEvent_UPDATE(IC_TIMER_REGISTER);
    out_put = 0;
    //  TIM2->CCER = 0xa;
}

void sendDshotDma()
{
    changeToOutput();
#ifdef USE_TIMER_2_CHANNEL_1
    LL_DMA_ConfigAddresses(
        DMA1, INPUT_DMA_CHANNEL, (uint32_t)&gcr,
        (uint32_t)&IC_TIMER_REGISTER->CCR1,
        LL_DMA_GetDataTransferDirection(DMA1, INPUT_DMA_CHANNEL));
#endif

#ifdef USE_TIMER_2_CHANNEL_4
    LL_DMA_ConfigAddresses(
        DMA1, INPUT_DMA_CHANNEL, (uint32_t)&gcr,
        (uint32_t)&IC_TIMER_REGISTER->CCR4,
        LL_DMA_GetDataTransferDirection(DMA1, INPUT_DMA_CHANNEL));

#endif
#ifdef USE_TIMER_2_CHANNEL_3
    LL_DMA_ConfigAddresses(
        DMA1, INPUT_DMA_CHANNEL, (uint32_t)&gcr,
        (uint32_t)&IC_TIMER_REGISTER->CCR3,
        LL_DMA_GetDataTransferDirection(DMA1, INPUT_DMA_CHANNEL));

#endif
#ifdef USE_TIMER_16
    LL_DMA_ConfigAddresses(
        DMA1, INPUT_DMA_CHANNEL, (uint32_t)&gcr,
        (uint32_t)&IC_TIMER_REGISTER->CCR1,
        LL_DMA_GetDataTransferDirection(DMA1, INPUT_DMA_CHANNEL));
#endif

    LL_DMA_SetDataLength(DMA1, INPUT_DMA_CHANNEL, 23 + buffer_padding);
    LL_DMA_EnableIT_TC(DMA1, INPUT_DMA_CHANNEL);
    LL_DMA_EnableIT_TE(DMA1, INPUT_DMA_CHANNEL);
    LL_DMA_EnableChannel(DMA1, INPUT_DMA_CHANNEL);

#ifdef USE_TIMER_2_CHANNEL_4
    LL_TIM_EnableDMAReq_CC4(IC_TIMER_REGISTER);
#endif
#ifdef USE_TIMER_2_CHANNEL_3
    LL_TIM_EnableDMAReq_CC3(IC_TIMER_REGISTER);
#endif
#ifdef USE_TIMER_16
    LL_TIM_EnableDMAReq_CC1(IC_TIMER_REGISTER);
#endif
#ifdef USE_TIMER_2_CHANNEL_1
    LL_TIM_EnableDMAReq_CC1(IC_TIMER_REGISTER);
#endif
    LL_TIM_CC_EnableChannel(IC_TIMER_REGISTER, IC_TIMER_CHANNEL);
    LL_TIM_EnableAllOutputs(IC_TIMER_REGISTER);
    LL_TIM_EnableCounter(IC_TIMER_REGISTER);
}

void receiveDshotDma()
{
    changeToInput();
    IC_TIMER_REGISTER->CNT = 0;
#ifdef USE_TIMER_2_CHANNEL_4
    LL_DMA_ConfigAddresses(
        DMA1, INPUT_DMA_CHANNEL, (uint32_t)&IC_TIMER_REGISTER->CCR4,
        (uint32_t)&dma_buffer,
        LL_DMA_GetDataTransferDirection(DMA1, INPUT_DMA_CHANNEL));
#endif
#ifdef USE_TIMER_2_CHANNEL_3
    LL_DMA_ConfigAddresses(
        DMA1, INPUT_DMA_CHANNEL, (uint32_t)&IC_TIMER_REGISTER->CCR3,
        (uint32_t)&dma_buffer,
        LL_DMA_GetDataTransferDirection(DMA1, INPUT_DMA_CHANNEL));
#endif
#ifdef USE_TIMER_16
    LL_DMA_ConfigAddresses(
        DMA1, INPUT_DMA_CHANNEL, (uint32_t)&IC_TIMER_REGISTER->CCR1,
        (uint32_t)&dma_buffer,
        LL_DMA_GetDataTransferDirection(DMA1, INPUT_DMA_CHANNEL));
#endif
#ifdef USE_TIMER_2_CHANNEL_1
    LL_DMA_ConfigAddresses(
        DMA1, INPUT_DMA_CHANNEL, (uint32_t)&IC_TIMER_REGISTER->CCR1,
        (uint32_t)&dma_buffer,
        LL_DMA_GetDataTransferDirection(DMA1, INPUT_DMA_CHANNEL));
#endif
    LL_DMA_SetDataLength(DMA1, INPUT_DMA_CHANNEL, buffersize);
    LL_DMA_EnableIT_TC(DMA1, INPUT_DMA_CHANNEL);
    LL_DMA_EnableIT_TE(DMA1, INPUT_DMA_CHANNEL);
    LL_DMA_EnableChannel(DMA1, INPUT_DMA_CHANNEL);
#ifdef USE_TIMER_2_CHANNEL_4
    LL_TIM_EnableDMAReq_CC4(IC_TIMER_REGISTER);
    LL_TIM_EnableDMAReq_CC4(IC_TIMER_REGISTER);

#endif
#ifdef USE_TIMER_2_CHANNEL_3
    LL_TIM_EnableDMAReq_CC3(IC_TIMER_REGISTER);

#endif
#ifdef USE_TIMER_16
    LL_TIM_EnableDMAReq_CC1(IC_TIMER_REGISTER);

#endif
#ifdef USE_TIMER_2_CHANNEL_1
    LL_TIM_EnableDMAReq_CC1(IC_TIMER_REGISTER);

#endif
    LL_TIM_CC_EnableChannel(IC_TIMER_REGISTER, IC_TIMER_CHANNEL);
    LL_TIM_EnableCounter(IC_TIMER_REGISTER);
    //   TIM16->PSC = 1;
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
