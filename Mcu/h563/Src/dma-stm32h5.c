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
    DEFINE_DMA_CHANNEL(2, 0),
    DEFINE_DMA_CHANNEL(2, 1),
    DEFINE_DMA_CHANNEL(2, 2),
    DEFINE_DMA_CHANNEL(2, 3),
    DEFINE_DMA_CHANNEL(2, 4),
    DEFINE_DMA_CHANNEL(2, 5),
    DEFINE_DMA_CHANNEL(2, 6),
    DEFINE_DMA_CHANNEL(2, 7),
};

void GPDMA1_Channel0_IRQHandler(void)
{
    dmaCallback_p cb = dmaChannels[0].callback;
    if (cb) {
        cb(&dmaChannels[0]);
    }
}
void GPDMA1_Channel1_IRQHandler(void)
{
    dmaCallback_p cb = dmaChannels[1].callback;
    if (cb) {
        cb(&dmaChannels[1]);
    }
}
void GPDMA1_Channel2_IRQHandler(void)
{
    dmaCallback_p cb = dmaChannels[2].callback;
    if (cb) {
        cb(&dmaChannels[2]);
    }
}
void GPDMA1_Channel3_IRQHandler(void)
{
    dmaCallback_p cb = dmaChannels[3].callback;
    if (cb) {
        cb(&dmaChannels[3]);
    }
}
void GPDMA1_Channel4_IRQHandler(void)
{
    dmaCallback_p cb = dmaChannels[4].callback;
    if (cb) {
        cb(&dmaChannels[4]);
    }
}
void GPDMA1_Channel5_IRQHandler(void)
{
    dmaCallback_p cb = dmaChannels[5].callback;
    if (cb) {
        cb(&dmaChannels[5]);
    }
}
void GPDMA1_Channel6_IRQHandler(void)
{
    dmaCallback_p cb = dmaChannels[6].callback;
    if (cb) {
        cb(&dmaChannels[6]);
    }
}
void GPDMA1_Channel7_IRQHandler(void)
{
    dmaCallback_p cb = dmaChannels[7].callback;
    if (cb) {
        cb(&dmaChannels[7]);
    }
}
void GPDMA2_Channel7_IRQHandler(void)
{
    dmaCallback_p cb = dmaChannels[15].callback;
    if (cb) {
        cb(&dmaChannels[15]);
    }
}
void dma_initialize(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPDMA1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPDMA2EN;
}
