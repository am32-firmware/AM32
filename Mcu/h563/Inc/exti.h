#pragma once
#include <stdbool.h>

#include "stm32h563xx.h"
#include "stm32h5xx_ll_exti.h"
    // .flagShift = 8*(i%4), \

#define DEFINE_EXTI_CHANNEL(i, j) { \
    .channel = i, \
    .cr = j, \
    .flagShift = 8*(i%4), \
    .irqn = EXTI ## i ## _IRQn \
}

#define DEFINE_EXTI_IRQ_HANDLER(i) { \
void EXTI ## i ## _IRQHandler(void) \
{ \
    extiCallback_p = extiChannels[i].callback; \
    if (cb) { \
        cb(&extiChannels[i]); \
    } \
} \
}

// #define EXTI_IRQ_ENABLE(c) c->channel->CCR |= DMA_CCR_EN
// #define DMA_CHANNEL_SET_CMAR(c, address) c->channel

// #define EXTI_CLEAR_FLAG(d, flag) d->dma->IFCR = (flag << d->flagsShift)
// #define EXTI_GET_FLAG_STATUS(d, flag) (d->dma->ISR & (flag << d->flagsShift))

// #define DMA_IT_GIF 0b0001
// #define DMA_IT_TCIF 0b0010
// #define DMA_IT_HTIF 0b0100
// #define DMA_IT_TEIF 0b1000


#define EXTI_CHANNEL_FROM_PORT(port) \
    (((uint32_t)port - GPIOA_BASE) / 0x0400)
typedef enum
{
    EXTICR_GPIOA = 0x00,
    EXTICR_GPIOB = 0x01,
    EXTICR_GPIOC = 0x02,
    EXTICR_GPIOD = 0x03,
    EXTICR_GPIOE = 0x04,
    EXTICR_GPIOF = 0x05,
    EXTICR_GPIOG = 0x06,
    EXTICR_GPIOH = 0x07,
    EXTICR_GPIOI = 0x08,
} exticr_e;

typedef enum
{
    EXTI_TRIGGER_NONE = LL_EXTI_TRIGGER_NONE,
    EXTI_TRIGGER_RISING = LL_EXTI_TRIGGER_RISING,
    EXTI_TRIGGER_FALLING = LL_EXTI_TRIGGER_FALLING,
    EXTI_TRIGGER_RISING_FALLING = LL_EXTI_TRIGGER_RISING_FALLING,
} extiTrigger_e;

struct extiChannel_s;
typedef void (*extiCallback_p)(struct extiChannel_s* channel);

typedef struct extiChannel_s
{
    // GPIO_TypeDef* gpio;
    uint8_t channel;
    __IO uint32_t* cr;
    uint8_t flagShift;

    uint32_t irqn;
    uint32_t userParam;

    extiTrigger_e trigger;
    extiCallback_p callback;
    uint8_t portFlag;
    uint8_t pin;

} extiChannel_t;

#define EXTI_INTERRUPT_ENABLE_MASK(mask) { \
    EXTI->IMR1 |= (mask); \
}
#define EXTI_INTERRUPT_DISABLE_MASK(mask) { \
    EXTI->IMR1 &= ~mask; \
}
#define EXTI_NVIC_ENABLE(channel) { \
    NVIC_EnableIRQ(EXTI0_IRQn + channel); \
}
extern extiChannel_t extiChannels[];

void exti_configure_port(extiChannel_t* exti, exticr_e port);
void exti_configure_trigger(extiChannel_t* exti, extiTrigger_e trigger);
void exti_configure_nvic(extiChannel_t* exti, bool enable);
void exti_configure_cb(extiChannel_t* exti, extiCallback_p cb);
void exti_initialize(EXTI_TypeDef exti);
// void exti_