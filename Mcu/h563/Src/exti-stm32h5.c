#include "exti.h"
// #include "stm32f0xx_it.h"
#include "stm32h563xx.h"

#define EXTI_EXTICR1 (uint32_t*)(EXTI_BASE + 0x060)
#define EXTI_EXTICR2 (uint32_t*)(EXTI_BASE + 0x064)
#define EXTI_EXTICR3 (uint32_t*)(EXTI_BASE + 0x068)
#define EXTI_EXTICR4 (uint32_t*)(EXTI_BASE + 0x06c)

extiChannel_t extiChannels[] = {
    DEFINE_EXTI_CHANNEL(0,   EXTI_EXTICR1),
    DEFINE_EXTI_CHANNEL(1,   EXTI_EXTICR1),
    DEFINE_EXTI_CHANNEL(2,   EXTI_EXTICR1),
    DEFINE_EXTI_CHANNEL(3,   EXTI_EXTICR1),
    DEFINE_EXTI_CHANNEL(4,   EXTI_EXTICR2),
    DEFINE_EXTI_CHANNEL(5,   EXTI_EXTICR2),
    DEFINE_EXTI_CHANNEL(6,   EXTI_EXTICR2),
    DEFINE_EXTI_CHANNEL(7,   EXTI_EXTICR2),
    DEFINE_EXTI_CHANNEL(8,   EXTI_EXTICR3),
    DEFINE_EXTI_CHANNEL(9,   EXTI_EXTICR3),
    DEFINE_EXTI_CHANNEL(10, EXTI_EXTICR3),
    DEFINE_EXTI_CHANNEL(11, EXTI_EXTICR3),
    DEFINE_EXTI_CHANNEL(12, EXTI_EXTICR4),
    DEFINE_EXTI_CHANNEL(13, EXTI_EXTICR4),
    DEFINE_EXTI_CHANNEL(14, EXTI_EXTICR4),
    DEFINE_EXTI_CHANNEL(15, EXTI_EXTICR4),

};


// DEFINE_EXTI_IRQ_HANDLER(0);
void EXTI0_IRQHandler(void)
{
    extiCallback_p cb = extiChannels[0].callback;
    if (cb) {
        cb(&extiChannels[0]);
    }
}
void EXTI1_IRQHandler(void)
{
    extiCallback_p cb = extiChannels[1].callback;
    if (cb) {
        cb(&extiChannels[1]);
    }
}
void EXTI2_IRQHandler(void)
{
    extiCallback_p cb = extiChannels[2].callback;
    if (cb) {
        cb(&extiChannels[2]);
    }
}
void EXTI3_IRQHandler(void)
{
    extiCallback_p cb = extiChannels[3].callback;
    if (cb) {
        cb(&extiChannels[3]);
    }
}
void EXTI4_IRQHandler(void)
{
    extiCallback_p cb = extiChannels[4].callback;
    if (cb) {
        cb(&extiChannels[4]);
    }
}
void EXTI5_IRQHandler(void)
{
    extiCallback_p cb = extiChannels[5].callback;
    if (cb) {
        cb(&extiChannels[5]);
    }
}
void EXTI6_IRQHandler(void)
{
    extiCallback_p cb = extiChannels[6].callback;
    if (cb) {
        cb(&extiChannels[6]);
    }
}
void EXTI7_IRQHandler(void)
{
    extiCallback_p cb = extiChannels[7].callback;
    if (cb) {
        cb(&extiChannels[7]);
    }
}
void EXTI8_IRQHandler(void)
{
    extiCallback_p cb = extiChannels[8].callback;
    if (cb) {
        cb(&extiChannels[8]);
    }
}
void EXTI9_IRQHandler(void)
{
    extiCallback_p cb = extiChannels[9].callback;
    if (cb) {
        cb(&extiChannels[9]);
    }
}
void EXTI10_IRQHandler(void)
{
    extiCallback_p cb = extiChannels[10].callback;
    if (cb) {
        cb(&extiChannels[10]);
    }
}
void EXTI11_IRQHandler(void)
{
    extiCallback_p cb = extiChannels[11].callback;
    if (cb) {
        cb(&extiChannels[11]);
    }
}
void EXTI12_IRQHandler(void)
{
    extiCallback_p cb = extiChannels[12].callback;
    if (cb) {
        cb(&extiChannels[12]);
    }
}
void EXTI13_IRQHandler(void)
{
    extiCallback_p cb = extiChannels[13].callback;
    if (cb) {
        cb(&extiChannels[13]);
    }
}
void EXTI14_IRQHandler(void)
{
    extiCallback_p cb = extiChannels[14].callback;
    if (cb) {
        cb(&extiChannels[14]);
    }
}
void EXTI15_IRQHandler(void)
{
    extiCallback_p cb = extiChannels[15].callback;
    if (cb) {
        cb(&extiChannels[15]);
    }
}



// void EXTI1_IRQHandler(void)
// {
//     extiCallback_p cb = extiChannels[1].callback;
//     if (cb) {
//         cb(&extiChannels[1]);
//     }
// }
// void EXTI13_IRQHandler(void)
// {
//     extiCallback_p cb = extiChannels[13].callback;
//     if (cb) {
//         cb(&extiChannels[13]);
//     }
// }
void exti_configure_port(extiChannel_t* exti, exticr_e port)
{
    // switch ((uint32_t)port) {
    //     case GPIOA_BASE:
    //         cr_value = 0x00;
    //         break;
    //     case GPIOB_BASE:
    //         cr_value = 0x01;
    //         break;
    //     case GPIOC_BASE:
    //         cr_value = 0x02;
    //         break;
    //     case GPIOD_BASE:
    //         cr_value = 0x03;
    //         break;
    //     case GPIOE_BASE:
    //         cr_value = 0x04;
    //         break;
    //     case GPIOF_BASE:
    //         cr_value = 0x05;
    //         break;
    //     case GPIOG_BASE:
    //         cr_value = 0x06;
    //         break;
    //     case GPIOH_BASE:
    //         cr_value = 0x07;
    //         break;
    //     case GPIOI_BASE:
    //         EXTI_EXTICR1_EXTI2_2
    //         cr_value = 0x08;
    //         break;
    //     default:
    //         // others reserved
    //         break;
    // }

    *exti->cr |= port << exti->flagShift;
}

void exti_configure_trigger(extiChannel_t* exti, extiTrigger_e trigger)
{
    switch(trigger)
    {
        case EXTI_TRIGGER_NONE:
            EXTI->RTSR1 &= ~(1 << exti->channel);
            EXTI->FTSR1 &= ~(1 << exti->channel);
        case EXTI_TRIGGER_RISING:
            EXTI->FTSR1 &= ~(1 << exti->channel);
            EXTI->RTSR1 |= 1 << exti->channel;
            break;
        case EXTI_TRIGGER_FALLING:
            EXTI->RTSR1 |= 1 << exti->channel;
            EXTI->FTSR1 &= ~(1 << exti->channel);
        case EXTI_TRIGGER_RISING_FALLING:
            EXTI->RTSR1 |= 1 << exti->channel;
            EXTI->FTSR1 |= 1 << exti->channel;
            break;
    }
}

void exti_configure_cb(extiChannel_t* exti, extiCallback_p cb)
{
    exti->callback = cb;
}

// void exti_initialize(DMA_TypeDef dma)
// {
//     RCC->AHB1ENR |= RCC_AHB1ENR_GPDMA1EN;
//     RCC->AHB1ENR |= RCC_AHB1ENR_GPDMA2EN;
// }
