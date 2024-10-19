#pragma once
#include "stm32h563xx.h"
#include "stm32h5xx_ll_dma.h"

#define DEFINE_DMA_CHANNEL(i, j) { \
    .dma = GPDMA ## i, \
    .ref = GPDMA ## i ## _ ## Channel ## j, \
    /* .muxChannel = DMAMUX1_Channel ## k, */\
    .flagsShift = 4*(j-1), \
    .irqn = GPDMA ## i ## _Channel ## j ## _IRQn \
}

#define DMA_CHANNEL_ENABLE(c) c->channel->CCR |= DMA_CCR_EN
#define DMA_CHANNEL_SET_CMAR(c, address) c->channel

#define DMA_CLEAR_FLAG(d, flag) d->dma->IFCR = (flag << d->flagsShift)
#define DMA_GET_FLAG_STATUS(d, flag) (d->dma->ISR & (flag << d->flagsShift))

#define DMA_IT_GIF 0b0001
#define DMA_IT_TCIF 0b0010
#define DMA_IT_HTIF 0b0100
#define DMA_IT_TEIF 0b1000

struct dmaChannel_s;
typedef void (*dmaCallback_p)(struct dmaChannel_s* channel);

typedef struct dmaChannel_s
{
    DMA_TypeDef* dma;
    DMA_Channel_TypeDef* ref;
#if defined(STM32G4)
    DMAMUX_Channel_TypeDef* muxChannel;
    uint8_t muxRequestId;
#endif
    uint8_t flagsShift;
    uint32_t userParam;
    uint32_t irqn;
    dmaCallback_p callback;
} dmaChannel_t;

extern dmaChannel_t dmaChannels[];

void dma_initialize(void);
