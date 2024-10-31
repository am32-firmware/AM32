// PERIPHERAL SETUP

#include "peripherals.h"

#include "mcu.h"
#include "comparator.h"
#include "gpio.h"
#include "ADC.h"
#include "serial_telemetry.h"
#include "stm32h563xx.h"
#include "stm32h5xx_ll_bus.h"
#include "targets.h"
#include "dma.h"
#include "comparator.h"
#include "bridge.h"
#include "drv8323-spi.h"
#include "ten-khz-timer.h"
#include "utility-timer.h"
#include "clock.h"

extern void interruptRoutine();
extern void processDshot();

void exti15cb(extiChannel_t* exti)
{
    uint32_t mask = 1 << exti->channel;
    // if (EXTI->RPR1 & mask) {
        EXTI->RPR1 |= mask;
    // } 
    // if (EXTI->FPR1 & mask) {
        EXTI->FPR1 |= mask;
    // }
    processDshot();
}

void phaseA_cb(extiChannel_t* exti)
{
    uint32_t mask = 1 << exti->channel;
    // if (EXTI->RPR1 & mask) {
        EXTI->RPR1 |= mask;
    // }
    // if (EXTI->FPR1 & mask) {
        EXTI->FPR1 |= mask;
    // }
    // if(gpio_read(&gpioCompPhaseA)) {
    //     gpio_reset(&gpioPhaseALed);
    // } else {
    //     gpio_set(&gpioPhaseALed);
    // }
    interruptRoutine();
}

void phaseB_cb(extiChannel_t* exti)
{
    uint32_t mask = 1 << exti->channel;
    // if (EXTI->RPR1 & mask) {
        EXTI->RPR1 |= mask;
    // }
    // if (EXTI->FPR1 & mask) {
        EXTI->FPR1 |= mask;
    // }
    // if(gpio_read(&gpioCompPhaseB)) {
    //     gpio_reset(&gpioPhaseBLed);
    // } else {
    //     gpio_set(&gpioPhaseBLed);
    // }
    interruptRoutine();
}

void phaseC_cb(extiChannel_t* exti)
{
    uint32_t mask = 1 << exti->channel;
    // if (EXTI->RPR1 & mask) {
        EXTI->RPR1 |= mask;
    // }
    // if (EXTI->FPR1 & mask) {
        EXTI->FPR1 |= mask;
    // }
    // if(gpio_read(&gpioCompPhaseC)) {
    //     gpio_reset(&gpioPhaseCLed);
    // } else {
    //     gpio_set(&gpioPhaseCLed);
    // }
    interruptRoutine();
}

void MX_TIM1_Init(void)
{
    LL_TIM_InitTypeDef TIM_InitStruct = { 0 };
    LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = { 0 };
    LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct = { 0 };
    LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    BRIDGE_TIMER_ENABLE_CLOCK();
    TIM_InitStruct.Prescaler = 0;
    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload = 1999;
    TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    TIM_InitStruct.RepetitionCounter = 0;
    LL_TIM_Init(BRIDGE_TIMER, &TIM_InitStruct);
    LL_TIM_EnableARRPreload(BRIDGE_TIMER);
    LL_TIM_SetClockSource(BRIDGE_TIMER, LL_TIM_CLOCKSOURCE_INTERNAL);
    LL_TIM_OC_EnablePreload(BRIDGE_TIMER, LL_TIM_CHANNEL_CH1);
#ifdef USE_SWAPPED_OUPUT
    TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM2;
#else
    TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
#endif
    TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
    TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
    TIM_OC_InitStruct.CompareValue = 0;
#ifdef USE_INVERTED_HIGH
    TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_LOW;
    TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_HIGH;
#else
    TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
    TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
#endif
#ifdef USE_INVERTED_LOW
    TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_LOW;
    TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_HIGH;
#else
    TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
    TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
#endif
    LL_TIM_OC_Init(BRIDGE_TIMER, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
    LL_TIM_OC_DisableFast(BRIDGE_TIMER, LL_TIM_CHANNEL_CH1);
    LL_TIM_OC_EnablePreload(BRIDGE_TIMER, LL_TIM_CHANNEL_CH2);
    TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
    TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
    LL_TIM_OC_Init(BRIDGE_TIMER, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
    LL_TIM_OC_DisableFast(BRIDGE_TIMER, LL_TIM_CHANNEL_CH2);
    LL_TIM_OC_EnablePreload(BRIDGE_TIMER, LL_TIM_CHANNEL_CH3);
    TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
    TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
    LL_TIM_OC_Init(BRIDGE_TIMER, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
    LL_TIM_OC_DisableFast(BRIDGE_TIMER, LL_TIM_CHANNEL_CH3);
    LL_TIM_OC_EnablePreload(BRIDGE_TIMER, LL_TIM_CHANNEL_CH4);
    TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
    TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
    LL_TIM_OC_Init(BRIDGE_TIMER, LL_TIM_CHANNEL_CH4, &TIM_OC_InitStruct);
    LL_TIM_OC_DisableFast(BRIDGE_TIMER, LL_TIM_CHANNEL_CH4);
    LL_TIM_SetTriggerOutput(BRIDGE_TIMER, LL_TIM_TRGO_RESET);
    LL_TIM_DisableMasterSlaveMode(BRIDGE_TIMER);
    TIM_BDTRInitStruct.OSSRState = LL_TIM_OSSR_DISABLE;
    TIM_BDTRInitStruct.OSSIState = LL_TIM_OSSI_DISABLE;
    TIM_BDTRInitStruct.LockLevel = LL_TIM_LOCKLEVEL_OFF;
    TIM_BDTRInitStruct.DeadTime = DEAD_TIME;
    TIM_BDTRInitStruct.BreakState = LL_TIM_BREAK_DISABLE;
    TIM_BDTRInitStruct.BreakPolarity = LL_TIM_BREAK_POLARITY_HIGH;
    TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_DISABLE;
    LL_TIM_BDTR_Init(BRIDGE_TIMER, &TIM_BDTRInitStruct);

    // LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    // LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

#ifdef USE_OPEN_DRAIN_LOW
#pragma message("using open drain low side")
#define LOW_OUTPUT_TYPE LL_GPIO_OUTPUT_OPENDRAIN
#else
#define LOW_OUTPUT_TYPE LL_GPIO_OUTPUT_PUSHPULL
#endif
#ifdef USE_OPEN_DRAIN_HIGH
#pragma message("using open drain high side")
#define HIGH_OUTPUT_TYPE LL_GPIO_OUTPUT_OPENDRAIN
#else
#define HIGH_OUTPUT_TYPE LL_GPIO_OUTPUT_PUSHPULL
#endif
    bridge_gpio_initialize();
    //  NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn, 2);
    //  NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
}
void initCorePeripherals(void)
{
    dma_initialize();
    MX_TIM1_Init();

    // waiting to make sure of parity with am32 implementation
    // bridge_initialize();
    interval_timer_initialize();

    COMPARATOR.phaseAcb = phaseA_cb;
    COMPARATOR.phaseBcb = phaseB_cb;
    COMPARATOR.phaseCcb = phaseC_cb;
    
    comparator_initialize(&COMPARATOR);
    com_timer_initialize();
    ten_khz_timer_initialize();
    utility_timer_initialize();
    input_timer_initialize();
#ifdef USE_SERIAL_TELEMETRY
    telem_UART_Init();
#endif
}

void initAfterJump(void)
{
    __enable_irq();
    mcu_setup();
}

void MX_IWDG_Init(void)
{
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_WWDG);
    IWDG->KR = 0x0000CCCCU;
    IWDG->KR = 0x00005555U;
    IWDG->PR = LL_IWDG_PRESCALER_16;
    IWDG->RLR = 4000;
    // while (IWDG->SR); // wait for the registers to be updated
    IWDG->KR = 0x0000AAAA;
}

void reloadWatchDogCounter()
{
    LL_IWDG_ReloadCounter(IWDG);
}

// interval timer
void interval_timer_initialize(void)
{
    INTERVAL_TIMER_ENABLE_CLOCK();
    // 2MHz on f051
    clock_update_hclk_frequency();
    TIM2->PSC = ((HCLK_FREQUENCY/1000000)/2) - 1;
    TIM2->ARR = 0xFFFF;
}

void interval_timer_enable(void)
{
    LL_TIM_EnableCounter(INTERVAL_TIMER);
    LL_TIM_GenerateEvent_UPDATE(INTERVAL_TIMER);
}

void com_timer_initialize(void)
{
    COM_TIMER_ENABLE_CLOCK();

    // 2MHz for f0
    COM_TIMER->PSC = CPU_FREQUENCY_MHZ/2 - 1;
    COM_TIMER->ARR = 4000;
    NVIC_SetPriority(COM_TIMER_IRQ, 0);
    NVIC_EnableIRQ(COM_TIMER_IRQ);
    LL_TIM_EnableARRPreload(COM_TIMER);
}

void input_timer_initialize(void)
{
    INPUT_TIMER_ENABLE_CLOCK();

    
    NVIC_SetPriority(dmaChannels[INPUT_TIMER_DMA_CHANNEL].irqn, 1);
    NVIC_EnableIRQ(dmaChannels[INPUT_TIMER_DMA_CHANNEL].irqn);
    // NVIC_SetPriority(IC_DMA_IRQ_NAME, 1);
    // NVIC_EnableIRQ(IC_DMA_IRQ_NAME);
    // INPUT_TIMER->TISEL = TIM_TISEL_TI1SEL_1;

    clock_update_hclk_frequency();
    // 1MHz clock frequency
    INPUT_TIMER->PSC = (HCLK_FREQUENCY/1000000) - 1;
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
    gpio_set_speed(&gpioInput, GPIO_SPEED_VERYFAST);
}

void input_timer_enable(void)
{
    LL_TIM_CC_EnableChannel(INPUT_TIMER,
        IC_TIMER_CHANNEL); // input capture and output compare
    LL_TIM_EnableCounter(INPUT_TIMER);
}

void disableComTimerInt() { COM_TIMER->DIER &= ~((0x1UL << (0U))); }

void enableComTimerInt() { COM_TIMER->DIER |= (0x1UL << (0U)); }

void setAndEnableComInt(uint16_t time)
{
    COM_TIMER->CNT = 0;
    COM_TIMER->ARR = time;
    COM_TIMER->SR = 0x00;
    COM_TIMER->DIER |= (0x1UL << (0U));
}

uint16_t getintervaTimerCount() { return INTERVAL_TIMER->CNT; }

void setintervaTimerCount(uint16_t intertime) { INTERVAL_TIMER->CNT = 0; }

void setPrescalerPWM(uint16_t presc) { BRIDGE_TIMER->PSC = presc; }

void setAutoReloadPWM(uint16_t relval) { BRIDGE_TIMER->ARR = relval; }

void inline setPWMCompare1(uint16_t compareone) { BRIDGE_TIMER->CCR1 = compareone; }
void inline setPWMCompare2(uint16_t comparetwo) { BRIDGE_TIMER->CCR2 = comparetwo; }
void inline setPWMCompare3(uint16_t comparethree) { BRIDGE_TIMER->CCR3 = comparethree; }

void inline generatePwmTimerEvent() { LL_TIM_GenerateEvent_UPDATE(BRIDGE_TIMER); }

void inline resetInputCaptureTimer()
{
    INPUT_TIMER->PSC = 0;
    INPUT_TIMER->CNT = 0;
}

void enableCorePeripherals()
{
    LL_TIM_CC_EnableChannel(BRIDGE_TIMER, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_EnableChannel(BRIDGE_TIMER, LL_TIM_CHANNEL_CH2);
    LL_TIM_CC_EnableChannel(BRIDGE_TIMER, LL_TIM_CHANNEL_CH3);
    LL_TIM_CC_EnableChannel(BRIDGE_TIMER, LL_TIM_CHANNEL_CH1N);
    LL_TIM_CC_EnableChannel(BRIDGE_TIMER, LL_TIM_CHANNEL_CH2N);
    LL_TIM_CC_EnableChannel(BRIDGE_TIMER, LL_TIM_CHANNEL_CH3N);

    LL_TIM_CC_EnableChannel(BRIDGE_TIMER,
        LL_TIM_CHANNEL_CH4); // timer used for timing adc read
    BRIDGE_TIMER->CCR4 = 100;

    /* Enable counter */
    LL_TIM_EnableCounter(BRIDGE_TIMER);
    LL_TIM_EnableAllOutputs(BRIDGE_TIMER);
    /* Force update generation */
    LL_TIM_GenerateEvent_UPDATE(BRIDGE_TIMER);

    input_timer_enable();

#ifndef BRUSHED_MODE
    LL_TIM_EnableCounter(COM_TIMER); // commutation_timer priority 0
    LL_TIM_GenerateEvent_UPDATE(COM_TIMER);
    // these two lines negate each other, the result is to trigger an
    // update event immediately
    LL_TIM_EnableIT_UPDATE(COM_TIMER);
    COM_TIMER->DIER &= ~((0x1UL << (0U))); // disable for now.
#endif

    utility_timer_enable();

    // depends on delay, so must come after utility timer counter enable
    drv8323_initialize(&DRV8323);

    interval_timer_enable();

    ten_khz_timer_enable();
    ten_khz_timer_interrupt_enable();

    // RCC->APB2ENR  &= ~(1 << 22);  // turn debug off
#ifdef USE_ADC
    ADC_setup();
#endif

    exti_configure_cb(&extiChannels[15], exti15cb);
    // interrupt for processDshot on exti line 15
    NVIC_SetPriority(EXTI15_IRQn, 2);
    NVIC_EnableIRQ(EXTI15_IRQn);
    EXTI->IMR1 |= (1 << 15);
}
