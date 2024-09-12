#pragma once
#include "stm32h563xx.h"

#define DEFINE_EXTI_IRQ(i, j) { \
    .dma = GPDMA ## i, \
    .ref = GPDMA ## i ## _ ## Channel ## j, \
    /* .muxChannel = DMAMUX1_Channel ## k, */\
    .flagsShift = 4*(j-1), \
    .irqn = GPDMA ## i ## _Channel ## j ## _IRQn \
}

#define EXTI_IRQ_ENABLE(c) c->channel->CCR |= DMA_CCR_EN
// #define DMA_CHANNEL_SET_CMAR(c, address) c->channel

// #define EXTI_CLEAR_FLAG(d, flag) d->dma->IFCR = (flag << d->flagsShift)
// #define EXTI_GET_FLAG_STATUS(d, flag) (d->dma->ISR & (flag << d->flagsShift))

// #define DMA_IT_GIF 0b0001
// #define DMA_IT_TCIF 0b0010
// #define DMA_IT_HTIF 0b0100
// #define DMA_IT_TEIF 0b1000

struct extiIRQ_s;
typedef void (*extiCallback_p)(struct extiIRQ_s* channel);

typedef struct extiIRQ_s
{
    GPIO_TypeDef* gpio;
    uint16_t* pin;
    uint8_t flagsShift;
    uint32_t userParam;
    uint32_t irqn;
    extiCallback_p callback;
} extiIRQ_t;

extern extiIRQ_t extiIRQs[];

void exti_initialize(EXTI_TypeDef exti);
