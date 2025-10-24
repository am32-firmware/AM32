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
#include "targets.h"

char ic_timer_prescaler = CPU_FREQUENCY_MHZ / 5 - 2;
uint32_t dma_buffer[64] = { 0 };
char out_put = 0;
uint8_t buffer_padding = 0;
uint8_t buffer_size = 32;
uint16_t change_time = 0;

void receiveDshotDma()
{
    #ifdef USE_TIMER_2_CHANNEL_0
    RCU_REG_VAL(RCU_TIMER2RST) |= BIT(RCU_BIT_POS(RCU_TIMER2RST));
    RCU_REG_VAL(RCU_TIMER2RST) &= ~BIT(RCU_BIT_POS(RCU_TIMER2RST));
    #endif
    #ifdef USE_TIMER_14_CHANNEL_0
    RCU_REG_VAL(RCU_TIMER14RST) |= BIT(RCU_BIT_POS(RCU_TIMER14RST));
    RCU_REG_VAL(RCU_TIMER14RST) &= ~BIT(RCU_BIT_POS(RCU_TIMER14RST));
    #endif

    TIMER_CHCTL0(IC_TIMER_REGISTER) = 0x41;
    TIMER_CHCTL2(IC_TIMER_REGISTER) = 0xa;
    TIMER_PSC(IC_TIMER_REGISTER) = ic_timer_prescaler;
    TIMER_CAR(IC_TIMER_REGISTER) = 0xFFFF;
    TIMER_SWEVG(IC_TIMER_REGISTER) |= (uint32_t)TIMER_EVENT_SRC_UPG;
    out_put = 0;
    TIMER_CNT(IC_TIMER_REGISTER) = 0;
    DMA_CHMADDR(INPUT_DMA_CHANNEL) = (uint32_t)&dma_buffer;
    DMA_CHCNT(INPUT_DMA_CHANNEL) = (buffersize & DMA_CHANNEL_CNT_MASK);
    TIMER_DMAINTEN(IC_TIMER_REGISTER) |= (uint32_t)TIMER_DMA_CH0D;
    TIMER_CHCTL2(IC_TIMER_REGISTER) |= (uint32_t)TIMER_CCX_ENABLE;
    TIMER_CTL0(IC_TIMER_REGISTER) |= (uint32_t)TIMER_CTL0_CEN;
    DMA_CHCTL(INPUT_DMA_CHANNEL) = 0x0000098b; // just set the whole reg in one go to enable
}

void sendDshotDma()
{
    #ifdef USE_TIMER_2_CHANNEL_0
    RCU_REG_VAL(RCU_TIMER2RST) |= BIT(RCU_BIT_POS(RCU_TIMER2RST));
    RCU_REG_VAL(RCU_TIMER2RST) &= ~BIT(RCU_BIT_POS(RCU_TIMER2RST));
    #endif
    #ifdef USE_TIMER_14_CHANNEL_0
    RCU_REG_VAL(RCU_TIMER14RST) |= BIT(RCU_BIT_POS(RCU_TIMER14RST));
    RCU_REG_VAL(RCU_TIMER14RST) &= ~BIT(RCU_BIT_POS(RCU_TIMER14RST));
    #endif
    
    TIMER_CHCTL0(IC_TIMER_REGISTER) = 0x60;
    TIMER_CHCTL2(IC_TIMER_REGISTER) = 0x3;
    TIMER_PSC(IC_TIMER_REGISTER) = output_timer_prescaler;
    TIMER_CAR(IC_TIMER_REGISTER) = 100;
    out_put = 1;
    TIMER_SWEVG(IC_TIMER_REGISTER) |= (uint32_t)TIMER_EVENT_SRC_UPG;
    DMA_CHMADDR(INPUT_DMA_CHANNEL) = (uint32_t)&gcr;
    DMA_CHCNT(INPUT_DMA_CHANNEL) = ((23 + buffer_padding) & DMA_CHANNEL_CNT_MASK);
    DMA_CHCTL(INPUT_DMA_CHANNEL) = 0x0000099b;
    TIMER_DMAINTEN(IC_TIMER_REGISTER) |= (uint32_t)TIMER_DMA_CH0D;
    TIMER_CHCTL2(IC_TIMER_REGISTER) |= (uint32_t)TIMER_CCX_ENABLE;
    TIMER_CCHP(IC_TIMER_REGISTER) |= (uint32_t)TIMER_CCHP_POEN;
    TIMER_CTL0(IC_TIMER_REGISTER) |= (uint32_t)TIMER_CTL0_CEN;
}

uint8_t getInputPinState() { return GPIO_ISTAT(INPUT_PIN_PORT) & (INPUT_PIN); }

void setInputPolarityRising()
{
    TIMER_CHCTL2(IC_TIMER_REGISTER) |= (uint32_t)(TIMER_IC_POLARITY_RISING);
}

void setInputPullDown()
{
    gpio_mode_set(INPUT_PIN_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLDOWN, INPUT_PIN);
}

void setInputPullUp()
{
    gpio_mode_set(INPUT_PIN_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, INPUT_PIN);
}

void enableHalfTransferInt() { DMA_CHCTL(INPUT_DMA_CHANNEL) |= DMA_INT_HTF; }
void setInputPullNone()
{
    gpio_mode_set(INPUT_PIN_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, INPUT_PIN);
}
