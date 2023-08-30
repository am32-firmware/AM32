/*
 * IO.c
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 */

#include "targets.h"
#include "IO.h"
#include "dshot.h"
#include "serial_telemetry.h"
#include "functions.h"
#include "common.h"

char ic_timer_prescaler = CPU_FREQUENCY_MHZ / 5 - 2;
//char output_timer_prescaler;
//int buffersize = 32;
//int smallestnumber = 0;
uint32_t dma_buffer[64] = {0};
char out_put = 0;
uint8_t  buffer_padding = 0;
//int dshot_runout_timer = 62500;
//uint32_t average_signal_pulse;

void changeToOutput()
{
    //  LL_DMA_SetDataTransferDirection(DMA1, INPUT_DMA_CHANNEL, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    dma_transfer_direction_config(INPUT_DMA_CHANNEL, DMA_MEMORY_TO_PERIPHERAL);

    //     LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_TIM3);           // de-init timer 2
    //  LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_TIM3);

    timer_deinit(IC_TIMER_REGISTER);

    // IC_TIMER_REGISTER->CCMR1 = 0x60;
    TIMER_CHCTL0(IC_TIMER_REGISTER) = 0x60;
    // IC_TIMER_REGISTER->CCER = 0x3;
    TIMER_CHCTL2(IC_TIMER_REGISTER) = 0x3;

    //  IC_TIMER_REGISTER->PSC = output_timer_prescaler;
    //  IC_TIMER_REGISTER->ARR = 61;
    TIMER_PSC(IC_TIMER_REGISTER) = output_timer_prescaler;
    TIMER_CAR(IC_TIMER_REGISTER) = 100;

    out_put = 1;
    //  LL_TIM_GenerateEvent_UPDATE(IC_TIMER_REGISTER);
    timer_event_software_generate(IC_TIMER_REGISTER, TIMER_EVENT_SRC_UPG);
}

void changeToInput()
{
    //  gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_4);
    //  LL_DMA_SetDataTransferDirection(DMA1, INPUT_DMA_CHANNEL, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    dma_transfer_direction_config(INPUT_DMA_CHANNEL, DMA_PERIPHERAL_TO_MEMORY);

    //  LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_TIM3);           // de-init timer 2
    //    LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_TIM3);
    timer_deinit(IC_TIMER_REGISTER);

    // IC_TIMER_REGISTER->CCMR1 = 0x1;
    TIMER_CHCTL0(IC_TIMER_REGISTER) = 0x7001;
    //    IC_TIMER_REGISTER->CCER = 0xa;
    TIMER_CHCTL2(IC_TIMER_REGISTER) = 0xa;

    // IC_TIMER_REGISTER->PSC = ic_timer_prescaler;
    //    IC_TIMER_REGISTER->ARR = 0xFFFF;
    TIMER_PSC(IC_TIMER_REGISTER) = ic_timer_prescaler;
    TIMER_CAR(IC_TIMER_REGISTER) = 0xFFFF;

    timer_event_software_generate(IC_TIMER_REGISTER, TIMER_EVENT_SRC_UPG);
    out_put = 0;
}
void receiveDshotDma()
{
    changeToInput();
    TIMER_CNT(IC_TIMER_REGISTER) = 0;

    // dma_periph_address_config(INPUT_DMA_CHANNEL, (uint32_t)&TIMER_CH0CV(IC_TIMER_REGISTER));
    // dma_memory_address_config(INPUT_DMA_CHANNEL, (uint32_t)&dma_buffer);
    // DMA_CHPADDR(INPUT_DMA_CHANNEL) = (uint32_t)&TIMER_CH0CV(IC_TIMER_REGISTER);
    DMA_CHMADDR(INPUT_DMA_CHANNEL) = (uint32_t)&dma_buffer;

    dma_transfer_number_config(INPUT_DMA_CHANNEL, buffersize);
    timer_dma_enable(IC_TIMER_REGISTER, TIMER_DMA_CH0D) ;
    TIMER_CHCTL2(IC_TIMER_REGISTER) |= (uint32_t)TIMER_CCX_ENABLE;
    TIMER_CTL0(IC_TIMER_REGISTER) |= (uint32_t)TIMER_CTL0_CEN;
    DMA_CHCTL(INPUT_DMA_CHANNEL) = 0x0000098b;   // just set the whole reg in one go to enable
}

void sendDshotDma()
{
    changeToOutput();
    //  dma_periph_address_config(INPUT_DMA_CHANNEL, (uint32_t)&TIMER_CH0CV(IC_TIMER_REGISTER));
    //  dma_memory_address_config(INPUT_DMA_CHANNEL, (uint32_t)&gcr);
    //  DMA_CHPADDR(INPUT_DMA_CHANNEL) = (uint32_t)&TIMER_CH0CV(IC_TIMER_REGISTER);
    DMA_CHMADDR(INPUT_DMA_CHANNEL) = (uint32_t)&gcr;
    //        LL_DMA_SetDataLength(DMA1, INPUT_DMA_CHANNEL, 30);
    dma_transfer_number_config(INPUT_DMA_CHANNEL, 23 + buffer_padding);

    //LL_DMA_EnableIT_TC(DMA1, INPUT_DMA_CHANNEL);
    dma_interrupt_enable(INPUT_DMA_CHANNEL, DMA_INT_FTF);
    //    LL_DMA_EnableIT_TE(DMA1, INPUT_DMA_CHANNEL);
    dma_interrupt_enable(INPUT_DMA_CHANNEL, DMA_INT_ERR);

    //  LL_DMA_EnableChannel(DMA1, INPUT_DMA_CHANNEL);
    dma_channel_enable(INPUT_DMA_CHANNEL);

    //    LL_TIM_EnableDMAReq_CC1(IC_TIMER_REGISTER);
    timer_dma_enable(IC_TIMER_REGISTER, TIMER_DMA_CH0D) ;

    //        LL_TIM_CC_EnableChannel(IC_TIMER_REGISTER, IC_TIMER_CHANNEL);
    TIMER_CHCTL2(IC_TIMER_REGISTER) |= (uint32_t)TIMER_CCX_ENABLE;

    //  LL_TIM_EnableAllOutputs(IC_TIMER_REGISTER);
    timer_primary_output_config(IC_TIMER_REGISTER, ENABLE);
    TIMER_CTL0(IC_TIMER_REGISTER) |= (uint32_t)TIMER_CTL0_CEN;
}

#if 0
void detectInput()
{
    smallestnumber = 20000;
    average_signal_pulse = 0;
    dshot = 0;
    servoPwm = 0;
    int lastnumber = dma_buffer[0];

    for (int j = 1 ; j < 31; j++) {
        if (dma_buffer[j] - lastnumber > 0) {
            if ((dma_buffer[j] - lastnumber) < smallestnumber) {
                smallestnumber = dma_buffer[j] - lastnumber;

            }

            average_signal_pulse += (dma_buffer[j] - lastnumber);
        }
        lastnumber = dma_buffer[j];
    }
    average_signal_pulse = average_signal_pulse / 32 ;

    if ((smallestnumber > 1) && (smallestnumber <= 4) && (average_signal_pulse < 60)) {
        ic_timer_prescaler = 0;
        output_timer_prescaler = 0;
        dshot = 1;
        buffer_padding = 12;
        dshot_runout_timer = 65000;
        armed_count_threshold = 10000;
        buffersize = 32;
    }
    if ((smallestnumber > 4) && (smallestnumber <= 8) && (average_signal_pulse < 100)) {
        dshot = 1;
        ic_timer_prescaler = 1;
        output_timer_prescaler = 1;
        //       IC_TIMER_REGISTER->CNT = 0xffff;
        TIMER_CNT(IC_TIMER_REGISTER) = 0xffff;
        buffer_padding = 7;
        dshot_runout_timer = 65000;
        armed_count_threshold = 10000;
        buffersize = 32;
    }
    //   if ((smallestnumber > 100 )&&(smallestnumber < 400)){
    //       multishot = 1;
    //       armed_count_threshold = 1000;
    //       buffersize = 4;
    //   }
    //   if ((smallestnumber > 2000 )&&(smallestnumber < 3000)){
    //       oneshot42 = 1;
    //   }
    if (smallestnumber > 30 && smallestnumber < 20000) {
        servoPwm = 1;
        ic_timer_prescaler = 71;
        armed_count_threshold = 35;
        buffersize = 2;
    }

    if (smallestnumber == 0 || smallestnumber == 20000) {
        inputSet = 0;
    }
    else {
        inputSet = 1;
    }
}
#endif

uint8_t getInputPinState()
{
    return GPIO_ISTAT(INPUT_PIN_PORT) & (INPUT_PIN);
}

void setInputPolarityRising()
{
    TIMER_CHCTL2(IC_TIMER_REGISTER) |= (uint32_t)(TIMER_IC_POLARITY_RISING);
}

void setInputPullDown()
{
    gpio_mode_set(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, INPUT_PIN);
}

void setInputPullUp()
{
    gpio_mode_set(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, INPUT_PIN);
}

void enableHalfTransferInt()
{
    DMA_CHCTL(INPUT_DMA_CHANNEL) |= DMA_INT_HTF;
}
void setInputPullNone()
{
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, INPUT_PIN);
}
