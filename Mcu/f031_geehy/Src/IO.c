/*
 * peripherals.c
 *
 *  Created on: 4. 24, 2026
 *      Author: Nong Jun
 */
#include "IO.h"

#include "common.h"
#include "dshot.h"
#include "functions.h"
#include "targets.h"

char ic_timer_prescaler = 8;
uint32_t dma_buffer[32] = { 0 };
uint32_t last_dma_buffer[32] = { 0 };
volatile char out_put = 0;
uint8_t buffer_padding = 0;
void sendDshotDma()
{

    DDL_DMA_DisableChannel(DMA, DDL_DMA_CHANNEL_1);
    DDL_GTMR_DisableCounter(GTMR);

    GTMR->CCEN = 0X0;
    GTMR->SMCR = 0;
    GTMR->SMCR = 0;
    GTMR->PSC = output_timer_prescaler;
    GTMR->AUTORLD = 82;

    DDL_DMA_SetDataTransferDirection(DMA, DDL_DMA_CHANNEL_1, DDL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    DMA_Channel1->PADDR = (uint32_t)&GTMR->CC0;
    DMA_Channel1->M0ADDR = (uint32_t)&gcr;
    DMA_Channel1->NDATA = 23 + buffer_padding;;
    DMA_Channel1->SCFG = 0X08015551;

    GTMR->CCM1 = 0x60;
    DDL_GTMR_GenerateEvent_UPDATE(GTMR);
    GTMR->CEG =1;
    GTMR->SR = 0;
    GTMR->CC0 = 0;
    GTMR->CCEN |= 0x03;
    DDL_GTMR_EnableCounter(GTMR);
    DDL_DMA_EnableChannel(DMA, DDL_DMA_CHANNEL_1);
    GTMR->DIER |= GTMR_DIER_CC0DEN;
    out_put = 1;
}

void receiveDshotDma()
{

    DDL_DMA_DisableChannel(DMA, DDL_DMA_CHANNEL_1);
    DDL_GTMR_DisableCounter(GTMR);
    DDL_GTMR_SetTriggerInput(GTMR, DDL_GTMR_TS_TI1F_ED);
    GTMR->CCEN = 0X00;
    GTMR->CC0 = 0;
    GTMR->CNT = 0;

    GTMR->PSC = ic_timer_prescaler;
    GTMR->AUTORLD = 0xFFFFFFFF;
    GTMR->SMCR = 0XC4;

    DDL_DMA_SetDataTransferDirection(DMA, DDL_DMA_CHANNEL_1, DDL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    DMA_Channel1->PADDR = (uint32_t)&GTMR->CC0;
    DMA_Channel1->M0ADDR = (uint32_t)&last_dma_buffer;
    DMA_Channel1->NDATA = buffersize;
    DMA_Channel1->SCFG = 0X08015511;

    GTMR->CCM1 = 0X03;
    GTMR->CCEN =0x01;
    DDL_GTMR_GenerateEvent_UPDATE(GTMR);
    DDL_GTMR_EnableCounter(GTMR);
    DDL_DMA_EnableChannel(DMA, DDL_DMA_CHANNEL_1);
    GTMR->DIER |= GTMR_DIER_CC0DEN;
    out_put = 0;
}

void fill_dest_array()
{
    uint32_t sum = 0;
    for(int i=0; i<buffersize; i++){
        sum += last_dma_buffer[i];
        dma_buffer[i] = sum;
    }
}
uint8_t getInputPinState()
{
  return (INPUT_PIN_PORT->INDR & INPUT_PIN);
}

void setInputPolarityRising()
{

}

void setInputPullDown()
{
}

void setInputPullUp()
{
    DDL_GPIO_LockKey(GPIOB, DDL_GPIO_LOCK_DISABLE);
    DDL_GPIO_SetPinPull(INPUT_PIN_PORT, INPUT_PIN, DDL_GPIO_PULL_UP);
    DDL_GPIO_LockKey(GPIOB, DDL_GPIO_LOCK_ENABLE);
}

void enableHalfTransferInt()
{


}
void setInputPullNone()
{

}
