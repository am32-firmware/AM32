#include "bridge.h"
#include "debug.h"
#include "stm32h563xx.h"
#include "targets.h"
#include "gpio.h"
#include "clock.h"

commutationStep_e bridgeComStep = 0;

// phase 3 rises during step 6 and falls during step 2
// timComStepComplementaryChannelConfig comSteps[] =
timComStepChannelConfig comSteps[] =
{
    // 1 pwm, 2 low, 3 float -fall
    {
        .ccmr1 = (OCM_PWM1 << TIM_CCMR1_OC1M_Pos) | (OCM_INACTIVE << TIM_CCMR1_OC2M_Pos),
        .ccmr2 = (OCM_INACTIVE << TIM_CCMR2_OC3M_Pos) | (OCM_PWM1 << TIM_CCMR2_OC4M_Pos),
        .ccer = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC3E
    },
    // 1 pwm, 2 float -rise, 3 low
    {
        .ccmr1 = (OCM_PWM1 << TIM_CCMR1_OC1M_Pos) | (OCM_INACTIVE << TIM_CCMR1_OC2M_Pos),
        .ccmr2 = (OCM_INACTIVE << TIM_CCMR2_OC3M_Pos) | (OCM_PWM1 << TIM_CCMR2_OC4M_Pos),
        .ccer = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC3E | TIM_CCER_CC3NE | TIM_CCER_CC2E
    },
    // 1 float - fall, 2pwm, 3 low
    {
        .ccmr1 = (OCM_INACTIVE << TIM_CCMR1_OC1M_Pos) | (OCM_PWM1 << TIM_CCMR1_OC2M_Pos),
        .ccmr2 = (OCM_INACTIVE << TIM_CCMR2_OC3M_Pos) | (OCM_PWM1 << TIM_CCMR2_OC4M_Pos),
        .ccer = TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE | TIM_CCER_CC1E
    },
    // 1 low, 2pwm, 3 float-rise
    {
        .ccmr1 = (OCM_INACTIVE << TIM_CCMR1_OC1M_Pos) | (OCM_PWM1 << TIM_CCMR1_OC2M_Pos),
        .ccmr2 = (OCM_INACTIVE << TIM_CCMR2_OC3M_Pos) | (OCM_PWM1 << TIM_CCMR2_OC4M_Pos),
        .ccer = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC3E
    },
    // 1 low, 2 float -fall, 3 pwm
    {
        .ccmr1 = (OCM_INACTIVE << TIM_CCMR1_OC1M_Pos) | (OCM_INACTIVE << TIM_CCMR1_OC2M_Pos),
        .ccmr2 = (OCM_PWM1 << TIM_CCMR2_OC3M_Pos) | (OCM_PWM1 << TIM_CCMR2_OC4M_Pos),
        .ccer = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC3E | TIM_CCER_CC3NE | TIM_CCER_CC2E
    },
    // 1 float-rise, 2 low, 3 pwm
    {
        .ccmr1 = (OCM_INACTIVE << TIM_CCMR1_OC1M_Pos) | (OCM_INACTIVE << TIM_CCMR1_OC2M_Pos),
        .ccmr2 = (OCM_PWM1 << TIM_CCMR2_OC3M_Pos) | (OCM_PWM1 << TIM_CCMR2_OC4M_Pos),
        .ccer = TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE | TIM_CCER_CC1E
    },
};


// timComStepChannelConfig comSteps[] =
// {
//     // 1 pwm, 2 low, 3 float -fall
//     {
//         .ccmr1 = (OCM_PWM1 << TIM_CCMR1_OC1M_Pos) | (OCM_INACTIVE << TIM_CCMR1_OC2M_Pos),
//         .ccmr2 = (OCM_INACTIVE << TIM_CCMR2_OC3M_Pos) | (OCM_PWM1 << TIM_CCMR2_OC4M_Pos),
//         .ccer = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC3E
//     },
//     // 1 pwm, 2 float -rise, 3 low
//     {
//         .ccmr1 = (OCM_PWM1 << TIM_CCMR1_OC1M_Pos) | (OCM_INACTIVE << TIM_CCMR1_OC2M_Pos),
//         .ccmr2 = (OCM_INACTIVE << TIM_CCMR2_OC3M_Pos) | (OCM_PWM1 << TIM_CCMR2_OC4M_Pos),
//         .ccer = TIM_CCER_CC1E | TIM_CCER_CC3E | TIM_CCER_CC3NE | TIM_CCER_CC2E
//     },
//     // 1 float - fall, 2pwm, 3 low
//     {
//         .ccmr1 = (OCM_INACTIVE << TIM_CCMR1_OC1M_Pos) | (OCM_PWM1 << TIM_CCMR1_OC2M_Pos),
//         .ccmr2 = (OCM_INACTIVE << TIM_CCMR2_OC3M_Pos) | (OCM_PWM1 << TIM_CCMR2_OC4M_Pos),
//         .ccer = TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC3NE | TIM_CCER_CC1E
//     },
//     // 1 low, 2pwm, 3 float-rise
//     {
//         .ccmr1 = (OCM_INACTIVE << TIM_CCMR1_OC1M_Pos) | (OCM_PWM1 << TIM_CCMR1_OC2M_Pos),
//         .ccmr2 = (OCM_INACTIVE << TIM_CCMR2_OC3M_Pos) | (OCM_PWM1 << TIM_CCMR2_OC4M_Pos),
//         .ccer = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC3E
//     },
//     // 1 low, 2 float -fall, 3 pwm
//     {
//         .ccmr1 = (OCM_INACTIVE << TIM_CCMR1_OC1M_Pos) | (OCM_INACTIVE << TIM_CCMR1_OC2M_Pos),
//         .ccmr2 = (OCM_PWM1 << TIM_CCMR2_OC3M_Pos) | (OCM_PWM1 << TIM_CCMR2_OC4M_Pos),
//         .ccer = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC3E | TIM_CCER_CC2E
//     },
//     // 1 float-rise, 2 low, 3 pwm
//     {
//         .ccmr1 = (OCM_INACTIVE << TIM_CCMR1_OC1M_Pos) | (OCM_INACTIVE << TIM_CCMR1_OC2M_Pos),
//         .ccmr2 = (OCM_PWM1 << TIM_CCMR2_OC3M_Pos) | (OCM_PWM1 << TIM_CCMR2_OC4M_Pos),
//         .ccer = TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC1E
//     },
// };

void bridge_initialize()
{
    // __HAL_RCC_BRIDGE_TIMER_CLK_ENABLE();
    // RCC->APB2ENR |= RCC_APB2ENR_BRIDGE_TIMEREN;

    BRIDGE_TIMER_ENABLE_CLOCK();

    bridge_set_run_frequency(24000);
    bridge_set_deadtime_ns(200); // configure dead-time
    BRIDGE_TIMER->CCR1 = 0;
    BRIDGE_TIMER->CCR2 = 0;
    BRIDGE_TIMER->CCR3 = 0;
    BRIDGE_TIMER->CCR4 = 1600;

    // 2048 counts period, 2048 steps of throttle resolution
    BRIDGE_TIMER->ARR = 2047;

    BRIDGE_TIMER->CCMR1 = comSteps[bridgeComStep].ccmr1; // set channel 1 pwm mode 1
    BRIDGE_TIMER->CCMR2 = comSteps[bridgeComStep].ccmr2; // channel3
    // enable channel + channeln
    BRIDGE_TIMER->CCER = comSteps[bridgeComStep].ccer;

    // enable TRGO2 on rising edge of channel 4 for adc trigger
    // BRIDGE_TIMER->CR2 |= 0b0111 << TIM_CR2_MMS2_Pos;
    // enable TRGO on OC4REF for adc trigger
    BRIDGE_TIMER->CR2 |= 0b111 << TIM_CR2_MMS_Pos;

    BRIDGE_TIMER->BDTR |= TIM_BDTR_OSSR | TIM_BDTR_OSSI;
    // bridge_set_deadtime_ns(2000);
    // BRIDGE_TIMER->BDTR = (BRIDGE_TIMER->BDTR & ~0xf) | 127;
    BRIDGE_TIMER->CR2 |= TIM_CR2_CCPC;
    BRIDGE_TIMER->CR1 |= TIM_CR1_CEN;

    // get preload aligned with bridgeComStep
    bridge_commutate();

    //BRIDGE_TIMER->CR1 |= 0b11<<5;

    bridge_gpio_initialize();

    NVIC_SetPriority(BRIDGE_TIMER_CC4_IRQn, 3);
    NVIC_EnableIRQ(BRIDGE_TIMER_CC4_IRQn);

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

    gpio_set_speed(&gpioUh, GPIO_SPEED_VERYFAST);
    gpio_set_speed(&gpioUl, GPIO_SPEED_VERYFAST);
    gpio_set_speed(&gpioVh, GPIO_SPEED_VERYFAST);
    gpio_set_speed(&gpioVl, GPIO_SPEED_VERYFAST);
    gpio_set_speed(&gpioWh, GPIO_SPEED_VERYFAST);
    gpio_set_speed(&gpioWl, GPIO_SPEED_VERYFAST);
}

void bridge_set_mode_audio(void)
{
    bridge_set_audio_duty(0);
    BRIDGE_TIMER->ARR = 0xff;
    BRIDGE_TIMER->CCR4 = 0x80;
    bridge_set_audio_frequency(2000);
}

void bridge_set_mode_run(void)
{
    bridge_set_run_duty(0);
    BRIDGE_TIMER->ARR = 2047;
    BRIDGE_TIMER->CCR4 = 1600;
    bridge_set_run_frequency(16000);
}

void bridge_set_audio_frequency(uint16_t frequency)
{
    uint16_t psc = HCLK_FREQUENCY / BRIDGE_TIMER->ARR / frequency;
    BRIDGE_TIMER->PSC = psc;
}

void bridge_set_run_frequency(uint32_t f)
{
    // f = HCLK/(PSC * 2048)
    // PSC = HCLK/(F*2048)
    BRIDGE_TIMER->PSC = HCLK_FREQUENCY/(f*2048) - 1;
    // TODO update event to load the prescaler into the shadow register
}

// for use in audio mode - duty is always 4 bit (0-16)
// duty may never exceed 12.5%
void bridge_set_audio_duty(uint8_t duty)
{
    // limit duty to 0xff
    duty = duty & 0xff;

    BRIDGE_TIMER->CCR1 = duty;
    BRIDGE_TIMER->CCR2 = duty;
    BRIDGE_TIMER->CCR3 = duty;
}

void bridge_set_run_duty(uint16_t duty)
{
    if (duty > 0x1000) {
        duty = 0x1000;
    }
    BRIDGE_TIMER->CCR1 = duty;
    BRIDGE_TIMER->CCR2 = duty;
    BRIDGE_TIMER->CCR3 = duty;
    if (duty > 0x0400) { // sample during middle of on time
        // uint16_t sample_point = 0x0800 + ((duty - 0x0800)/2);
        // uint16_t sample_point = duty/2;
        uint16_t sample_point = duty - 50;
        BRIDGE_TIMER->CCR4 = sample_point;
    } else { // sample during middle of off time
        // uint16_t sample_point = duty/2;
        // uint16_t sample_point = duty + ((0x0800 - duty) / 2);
        uint16_t sample_point = 0x800 - 50;
        BRIDGE_TIMER->CCR4 = sample_point;
    }
}

void bridge_set_com_step(uint8_t step)
{
    bridgeComStep = step%6;
    BRIDGE_TIMER->CCMR1 = comSteps[bridgeComStep].ccmr1;
    BRIDGE_TIMER->CCMR2 = comSteps[bridgeComStep].ccmr2;
    BRIDGE_TIMER->CCER = comSteps[bridgeComStep].ccer;
    BRIDGE_TIMER->EGR |= TIM_EGR_COMG;
}

void bridge_commutate(void)
{
    BRIDGE_TIMER->EGR |= TIM_EGR_COMG;
    bridgeComStep++;
    bridgeComStep = bridgeComStep % 6;
    BRIDGE_TIMER->CCMR1 = comSteps[bridgeComStep].ccmr1;
    BRIDGE_TIMER->CCMR2 = comSteps[bridgeComStep].ccmr2;
    BRIDGE_TIMER->CCER = comSteps[bridgeComStep].ccer;
}

void bridge_enable(void)
{
    BRIDGE_TIMER->EGR |= TIM_EGR_UG;
    // enable main output
    BRIDGE_TIMER->BDTR |= TIM_BDTR_MOE;
}

void bridge_disable(void)
{
    BRIDGE_TIMER->BDTR &= ~TIM_BDTR_MOE;
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

    BRIDGE_TIMER->BDTR = (BRIDGE_TIMER->BDTR & ~TIM_BDTR_DTG_Msk) | dtg;
    // BRIDGE_TIMER->BDTR = (BRIDGE_TIMER->BDTR & ~0xf);
}

void bridge_sample_interrupt_enable()
{
    BRIDGE_TIMER->DIER |= TIM_DIER_CC4IE; // enable interrupt
}

void bridge_sample_interrupt_disable()
{
    BRIDGE_TIMER->DIER &= ~TIM_DIER_CC4IE; // disable interrupt
}

__WEAK void bridge_timer_irq_handler()
{
    if (BRIDGE_TIMER->SR & TIM_SR_CC4IF) {
        BRIDGE_TIMER->SR &= ~TIM_SR_CC4IF;
        debug_toggle_2();
    }
    // while (1);
}
