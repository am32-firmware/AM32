#include "bridge.h"
#include "stm32h563xx.h"
#include "targets.h"
#include "gpio.h"
#include "clock.h"

commutationStep_e bridgeComStep = 0;

// phase 3 rises during step 6 and falls during step 2
timComStepChannelConfig comSteps[] =
{
    // 1 pwm, 2 low, 3 float -fall
    {
        .ccmr1 = (0b110<<4) | (0b100<<12),
        .ccmr2 = (0b100<<4) | (0b110<<12),
        .ccer = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC3E
    },
    // 1 pwm, 2 float -rise, 3 low
    {
        .ccmr1 = (0b110<<4) | (0b100<<12),
        .ccmr2 = (0b100<<4) | (0b110<<12),
        .ccer = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC3E | TIM_CCER_CC3NE | TIM_CCER_CC2E
    },
    // 1 float - fall, 2pwm, 3 low
    {
        .ccmr1 = (0b100<<4) | (0b110<<12),
        .ccmr2 = (0b100<<4) | (0b110<<12),
        .ccer = TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE | TIM_CCER_CC1E
    },
    // 1 low, 2pwm, 3 float-rise
    {
        .ccmr1 = (0b100<<4) | (0b110<<12),
        .ccmr2 = (0b100<<4) | (0b110<<12),
        .ccer = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC3E
    },
    // 1 low, 2 float -fall, 3 pwm
    {
        .ccmr1 = (0b100<<4) | (0b100<<12),
        .ccmr2 = (0b110<<4) | (0b110<<12),
        .ccer = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC3E | TIM_CCER_CC3NE | TIM_CCER_CC2E
    },
    // 1 float-rise, 2 low, 3 pwm
    {
        .ccmr1 = (0b100<<4) | (0b100<<12),
        .ccmr2 = (0b110<<4) | (0b110<<12),
        .ccer = TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE | TIM_CCER_CC1E
    },
};

void bridge_initialize()
{
    // __HAL_RCC_TIM1_CLK_ENABLE();
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    bridge_set_run_frequency(24000);
    //TIM1->BDTR |= 0x40;
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;
    TIM1->CCR4 = 1600;

    // 2048 counts period, 2048 steps of throttle resolution
    TIM1->ARR = 2047;

    TIM1->CCMR1 = comSteps[bridgeComStep].ccmr1; // set channel 1 pwm mode 1
    TIM1->CCMR2 = comSteps[bridgeComStep].ccmr2; // channel3
    // enable channel + channeln
    TIM1->CCER = comSteps[bridgeComStep].ccer;

    // enable TRGO2 on rising edge of channel 4 for adc trigger
    // TIM1->CR2 |= 0b0111 << TIM_CR2_MMS2_Pos;
    // enable TRGO on OC4REF for adc trigger
    TIM1->CR2 |= 0b111 << TIM_CR2_MMS_Pos;

    TIM1->BDTR |= TIM_BDTR_OSSR | TIM_BDTR_OSSI;
    bridge_set_deadtime_ns(2000);
    TIM1->CR2 |= TIM_CR2_CCPC;
    TIM1->CR1 |= TIM_CR1_CEN;

    // get preload aligned with bridgeComStep
    bridge_commutate();

    //TIM1->CR1 |= 0b11<<5;

    bridge_gpio_initialize();

}
void bridge_gpio_initialize()
{

    gpio_t gpioUh = DEF_GPIO(BRIDGE_UH_PORT, BRIDGE_UH_PIN, BRIDGE_UH_AF, GPIO_AF); 
    gpio_t gpioUl = DEF_GPIO(BRIDGE_UL_PORT, BRIDGE_UL_PIN, BRIDGE_UL_AF, GPIO_AF);
    gpio_t gpioVh = DEF_GPIO(BRIDGE_VH_PORT, BRIDGE_VH_PIN, BRIDGE_VH_AF, GPIO_AF);
    gpio_t gpioVl = DEF_GPIO(BRIDGE_VL_PORT, BRIDGE_VL_PIN, BRIDGE_VL_AF, GPIO_AF);
    gpio_t gpioWh = DEF_GPIO(BRIDGE_WH_PORT, BRIDGE_WH_PIN, BRIDGE_WH_AF, GPIO_AF);
    gpio_t gpioWl = DEF_GPIO(BRIDGE_WL_PORT, BRIDGE_WL_PIN, BRIDGE_WL_AF, GPIO_AF);
    
    gpio_initialize(&gpioUh);
    gpio_initialize(&gpioUl);
    gpio_initialize(&gpioVh);
    gpio_initialize(&gpioVl);
    gpio_initialize(&gpioWh);
    gpio_initialize(&gpioWl);
}

void bridge_set_mode_audio(void)
{
    bridge_set_audio_duty(0);
    TIM1->ARR = 0xff;
    TIM1->CCR4 = 0x80;
    bridge_set_audio_frequency(2000);
}

void bridge_set_mode_run(void)
{
    bridge_set_run_duty(0);
    TIM1->ARR = 2047;
    TIM1->CCR4 = 1600;
    bridge_set_run_frequency(16000);
}

void bridge_set_audio_frequency(uint16_t frequency)
{
    uint16_t psc = HCLK_FREQUENCY / TIM1->ARR / frequency;
    TIM1->PSC = psc;
}

void bridge_set_run_frequency(uint32_t f)
{
    // f = HCLK/(PSC * 2048)
    // PSC = HCLK/(F*2048)
    TIM1->PSC = HCLK_FREQUENCY/(f*2048) - 1;
    // TODO update event to load the prescaler into the shadow register
}

// for use in audio mode - duty is always 4 bit (0-16)
// duty may never exceed 12.5%
void bridge_set_audio_duty(uint8_t duty)
{
    // duty = duty & 0xf;
    // if (duty > 0xf) {
    //     duty = 0xf;
    // }
    TIM1->CCR1 = duty;
    TIM1->CCR2 = duty;
    TIM1->CCR3 = duty;
}

void bridge_set_run_duty(uint16_t duty)
{
    if (duty > 800) {
        duty = 800;
    }
    TIM1->CCR1 = duty;
    TIM1->CCR2 = duty;
    TIM1->CCR3 = duty;
    TIM1->CCR4 = duty + 1000;
}

void bridge_set_com_step(uint8_t step)
{
    bridgeComStep = step%6;
    TIM1->CCMR1 = comSteps[bridgeComStep].ccmr1;
    TIM1->CCMR2 = comSteps[bridgeComStep].ccmr2;
    TIM1->CCER = comSteps[bridgeComStep].ccer;
    TIM1->EGR |= TIM_EGR_COMG;
}

void bridge_commutate(void)
{
    TIM1->EGR |= TIM_EGR_COMG;
    bridgeComStep++;
    bridgeComStep = bridgeComStep % 6;
    TIM1->CCMR1 = comSteps[bridgeComStep].ccmr1;
    TIM1->CCMR2 = comSteps[bridgeComStep].ccmr2;
    TIM1->CCER = comSteps[bridgeComStep].ccer;
}

void bridge_enable(void)
{
    // enable main output
    TIM1->BDTR |= TIM_BDTR_MOE;
}

void bridge_disable(void)
{
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    bridge_set_run_duty(0);
    bridge_set_audio_duty(0);
}

void bridge_set_deadtime_ns(uint32_t deadtime)
{
    float ns_per_tick = 1000000000.0f/HCLK_FREQUENCY;
    uint32_t deadtime_ticks = (deadtime / ns_per_tick) + 1; // round up
    uint8_t dtg = 0xff;
    if (deadtime_ticks < 127) {
        dtg = deadtime_ticks;
    } else if (deadtime_ticks < 2*(64+63)) {
        uint8_t remainder = deadtime_ticks%2;
        deadtime_ticks /= 2;
        deadtime_ticks += remainder ? 1 : 0;
        deadtime_ticks -= 64;
        dtg = 0x80 | deadtime_ticks;
    } else if (deadtime_ticks < 8*(32+31)) {
        uint8_t remainder = deadtime_ticks%8;
        deadtime_ticks /= 8;
        deadtime_ticks += remainder ? 1 : 0;
        deadtime_ticks -= 32;
        dtg = 0xC0 | deadtime_ticks;
    } else if (deadtime_ticks < 16*(32+31)) {
        uint8_t remainder = deadtime_ticks%16;
        deadtime_ticks /= 16;
        deadtime_ticks += remainder ? 1 : 0;
        deadtime_ticks -= 32;
        dtg = 0xe0 | deadtime_ticks;
    }
    
    TIM1->BDTR = (TIM1->BDTR & ~0xf) | dtg;
    // TIM1->BDTR = (TIM1->BDTR & ~0xf);
}
