#include "input-timer.h"

#include "stm32h5xx_ll_tim.h"

#include "clock.h"
#include "dma.h"
#include "gpio.h"
#include "targets.h"

void input_timer_initialize(void)
{
    // enable the timer clock
    INPUT_TIMER_ENABLE_CLOCK();

    // enable the timer interrupt in NVIC
    NVIC_SetPriority(dmaChannels[INPUT_TIMER_DMA_CHANNEL].irqn, 1);
    NVIC_EnableIRQ(dmaChannels[INPUT_TIMER_DMA_CHANNEL].irqn);
    // NVIC_SetPriority(IC_DMA_IRQ_NAME, 1);
    // NVIC_EnableIRQ(IC_DMA_IRQ_NAME);
    // INPUT_TIMER->TISEL = TIM_TISEL_TI1SEL_1;

    // configure timer
    clock_update_hclk_frequency();
    // 1MHz clock frequency
    INPUT_TIMER->PSC = (HCLK_FREQUENCY/1000000) - 1;
    // maximum period
    INPUT_TIMER->ARR = 0xffff;
    // INPUT_TIMER->ARR = 10000;
    input_timer_gpio_initialize();
}

void input_timer_gpio_initialize(void)
{
    gpio_t gpioInput = DEF_GPIO(
        INPUT_SIGNAL_PORT,
        INPUT_SIGNAL_PIN,
        INPUT_SIGNAL_AF,
        GPIO_AF);
    gpio_initialize(&gpioInput);
    // maximize drive strength
    // this only matters for output (bidirectional dshot)
    gpio_set_speed(&gpioInput, GPIO_SPEED_VERYFAST);
}

void input_timer_enable(void)
{
    LL_TIM_CC_EnableChannel(INPUT_TIMER,
        IC_TIMER_CHANNEL); // input capture and output compare
    LL_TIM_EnableCounter(INPUT_TIMER);
}

void inline resetInputCaptureTimer()
{
    INPUT_TIMER->PSC = 0;
    INPUT_TIMER->CNT = 0;
}
