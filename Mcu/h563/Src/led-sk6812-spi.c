#include "led.h"
#include "target.h"
#include "gpio.h"
#include "dma.h"

#define LED_AF 2
#define LED_SK6812
#define OCM_PWM1 0b110

// nanoseconds
#if defined(LED_WS2812)
#define BIT0_TH_NS 400
#define BIT1_TH_NS 800
#define BIT_PERIOD_NS 1250 // symbol duration
#elif defined(LED_SK6812)
#define BIT0_TH_NS 300
#define BIT1_TH_NS 600
#define BIT_PERIOD_NS 1200 // symbol duration
#endif

// 24 for 8-bit rgb and 1 byte for a trailing 0 (to pull the wire low between frames)
uint16_t pulses[25];

#define LED_TIMER_NS_PER_TICK (1000000000 / HCLK_FREQUENCY)
#define BIT0_TICKS (BIT0_TH_NS / LED_TIMER_NS_PER_TICK)
#define BIT1_TICKS (BIT1_TH_NS / LED_TIMER_NS_PER_TICK)
#define BIT_PERIOD_TICKS (BIT_PERIOD_NS / LED_TIMER_NS_PER_TICK)
static dmaChannel_t *ledDma;
void led_initialize(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_TIM16_CLK_ENABLE();

    LED_TIMER->ARR = BIT_PERIOD_TICKS;
    LED_TIMER->CCMR1 |= (OCM_PWM1 << 4) | TIM_CCMR1_OC1PE;
    LED_TIMER->CCER |= TIM_CCER_CC1E;
    LED_TIMER->BDTR |= TIM_BDTR_MOE;
    LED_TIMER->CR1 |= TIM_CR1_CEN;

    LED_TIMER->DIER |= TIM_DIER_UDE;

    ledDma = &dmaChannels[3 - 1];
    ledDma->ref->CPAR = (uint32_t)&LED_TIMER->CCR1;
    ledDma->ref->CMAR = (uint32_t)&pulses[0];
    // 16 bit data size for memory
    ledDma->ref->CCR |= 0b01 << DMA_CCR_MSIZE_Pos;
    // 16 bit data size for peripheral
    ledDma->ref->CCR |= 0b01 << DMA_CCR_PSIZE_Pos;
    // read from memory
    ledDma->ref->CCR |= DMA_CCR_DIR;
    // memory increment
    ledDma->ref->CCR |= DMA_CCR_MINC;

    gpio_t gpioLed = DEF_GPIO(LED_PORT, LED_PIN, LED_AF, GPIO_AF);
    gpio_initialize(&gpioLed);

    led_write(0);
}

void led_write(uint32_t brg)
{
    for (int i = 0; i < 24; i++)
    {
        pulses[i] = brg & (1 << i) ? BIT1_TICKS : BIT0_TICKS;
    }
    // disable channel
    ledDma->ref->CCR &= ~DMA_CCR_EN;
    // setup number of data to transfer
    ledDma->ref->CNDTR = sizeof(pulses) / 2;
    // enable channel
    ledDma->ref->CCR |= DMA_CCR_EN;
    LED_TIMER->CNT = 0;
}
