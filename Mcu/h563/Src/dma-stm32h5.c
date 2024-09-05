#include "dma.h"
#include "stm32h563xx.h"

dmaChannel_t dmaChannels[] = {
    DEFINE_DMA_CHANNEL(1, 0),
    DEFINE_DMA_CHANNEL(1, 1),
    DEFINE_DMA_CHANNEL(1, 2),
    DEFINE_DMA_CHANNEL(1, 3),
    DEFINE_DMA_CHANNEL(1, 4),
    DEFINE_DMA_CHANNEL(1, 5),
    DEFINE_DMA_CHANNEL(1, 6),
    DEFINE_DMA_CHANNEL(1, 7),
};

void GPDMA1_Channel0_IRQHandler(void)
{
    dmaCallback_p cb = dmaChannels[0].callback;
    if (cb) {
        cb(&dmaChannels[0]);
    }
}
void DMA1_Channel2_IRQHandler(void)
{
    dmaCallback_p cb = dmaChannels[1].callback;
    if (cb) {
        cb(&dmaChannels[1]);
    }
}
void DMA1_Channel3_IRQHandler(void)
{
    dmaCallback_p cb = dmaChannels[2].callback;
    if (cb) {
        cb(&dmaChannels[2]);
    }
}
void DMA1_Channel4_IRQHandler(void)
{
    dmaCallback_p cb = dmaChannels[3].callback;
    if (cb) {
        cb(&dmaChannels[3]);
    }
}
void DMA1_Channel5_IRQHandler(void)
{
    dmaCallback_p cb = dmaChannels[4].callback;
    if (cb) {
        cb(&dmaChannels[4]);
    }
}

void dma_initialize(DMA_TypeDef dma)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPDMA1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPDMA2EN;
}
