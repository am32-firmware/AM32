/*
 * peripherals.c
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 */

// PERIPHERAL SETUP

#include "peripherals.h"

#include "ADC.h"
#include "serial_telemetry.h"
#include "targets.h"

void initCorePeripherals(void)
{
    SystemClock_Config();
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
    FLASH->ACR |= FLASH_ACR_PRFTBE; //// prefetch buffer enable
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_COMP1_Init();
    MX_TIM14_Init();
    MX_TIM6_Init();
    MX_TIM17_Init();
    UN_TIM_Init();
#ifdef USE_SERIAL_TELEMETRY
    telem_UART_Init();
#endif
}

void initAfterJump(void)
{
    volatile uint32_t* VectorTable = (volatile uint32_t*)0x20000000;
    uint32_t vector_index = 0;
    for (vector_index = 0; vector_index < 48; vector_index++) {
        VectorTable[vector_index] = *(__IO uint32_t*)(APPLICATION_ADDRESS + (vector_index << 2)); // no VTOR on cortex-MO so
                                                                                                  // need to copy vector table
    }
    /* Enable the SYSCFG peripheral clock*/
    do {
        volatile uint32_t tmpreg;
        ((((RCC_TypeDef*)((((uint32_t)0x40000000U) + 0x00020000) + 0x00001000))
                 ->APB2ENR)
            |= ((0x1U << (0U))));
        /* Delay after an RCC peripheral clock enabling */
        tmpreg = ((((RCC_TypeDef*)((((uint32_t)0x40000000U) + 0x00020000) + 0x00001000))
                          ->APB2ENR)
            & ((0x1U << (0U))));
        ((void)(tmpreg));
    } while (0U);
    //	  /* Remap SRAM at 0x00000000 */
    do {
        ((SYSCFG_TypeDef*)(((uint32_t)0x40000000U) + 0x00010000))->CFGR1 &= ~((0x3U << (0U)));
        ((SYSCFG_TypeDef*)(((uint32_t)0x40000000U) + 0x00010000))->CFGR1 |= ((0x1U << (0U)) | (0x2U << (0U)));
    } while (0);

    if (SysTick_Config(SystemCoreClock / 1000)) {
        /* Capture error */
        while (1) {
        }
    }
    __enable_irq();
}

void SystemClock_Config(void)
{
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);

    if (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_1) {
        // Error_Handler();
    }
    LL_RCC_HSI_Enable();

    /* Wait till HSI is ready */
    while (LL_RCC_HSI_IsReady() != 1) {
    }
    LL_RCC_HSI_SetCalibTrimming(16);
    LL_RCC_HSI14_Enable();

    /* Wait till HSI14 is ready */
    while (LL_RCC_HSI14_IsReady() != 1) {
    }
    LL_RCC_HSI14_SetCalibTrimming(16);
    LL_RCC_LSI_Enable();

    /* Wait till LSI is ready */
    while (LL_RCC_LSI_IsReady() != 1) {
    }
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_12);
    LL_RCC_PLL_Enable();

    /* Wait till PLL is ready */
    while (LL_RCC_PLL_IsReady() != 1) {
    }
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

    /* Wait till System clock is ready */
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {
    }
    LL_Init1msTick(48000000);
    LL_SetSystemCoreClock(48000000);
    LL_RCC_HSI14_EnableADCControl();
    LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_PCLK1);
}

void MX_COMP1_Init(void)
{
    LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    /**COMP1 GPIO Configuration
    PA1   ------> COMP1_INP
    PA5   ------> COMP1_INM
    */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    NVIC_SetPriority(ADC1_COMP_IRQn, 0);
    NVIC_EnableIRQ(ADC1_COMP_IRQn);
}

void MX_IWDG_Init(void)
{
    IWDG->KR = 0x0000CCCCU;
    IWDG->KR = 0x00005555U;
    IWDG->PR = LL_IWDG_PRESCALER_16;
    IWDG->RLR = 4000;
    LL_IWDG_ReloadCounter(IWDG);
}

void MX_TIM1_Init(void)
{
    LL_TIM_InitTypeDef TIM_InitStruct = { 0 };
    LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = { 0 };
    LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct = { 0 };
    LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM1);

    TIM_InitStruct.Prescaler = 0;
    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload = 1999;
    TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    TIM_InitStruct.RepetitionCounter = 0;
    LL_TIM_Init(TIM1, &TIM_InitStruct);
    LL_TIM_EnableARRPreload(TIM1);
    LL_TIM_SetClockSource(TIM1, LL_TIM_CLOCKSOURCE_INTERNAL);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);
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
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
    LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH1);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH2);
    TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
    TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
    LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH2);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH3);
    TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
    TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
    LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH3);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH4);
    TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
    TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH4, &TIM_OC_InitStruct);
    LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH4);
    LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);
    LL_TIM_DisableMasterSlaveMode(TIM1);
    TIM_BDTRInitStruct.OSSRState = LL_TIM_OSSR_DISABLE;
    TIM_BDTRInitStruct.OSSIState = LL_TIM_OSSI_DISABLE;
    TIM_BDTRInitStruct.LockLevel = LL_TIM_LOCKLEVEL_OFF;
    TIM_BDTRInitStruct.DeadTime = DEAD_TIME;
    TIM_BDTRInitStruct.BreakState = LL_TIM_BREAK_DISABLE;
    TIM_BDTRInitStruct.BreakPolarity = LL_TIM_BREAK_POLARITY_HIGH;
    TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_DISABLE;
    LL_TIM_BDTR_Init(TIM1, &TIM_BDTRInitStruct);

    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
/**TIM1 GPIO Configuration
PA7   ------> TIM1_CH1N
PB0   ------> TIM1_CH2N
PB1   ------> TIM1_CH3N
PA8   ------> TIM1_CH1
PA9   ------> TIM1_CH2
PA10   ------> TIM1_CH3
*/
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
    GPIO_InitStruct.Pin = PHASE_A_GPIO_LOW;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LOW_OUTPUT_TYPE;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
    LL_GPIO_Init(PHASE_A_GPIO_PORT_LOW, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = PHASE_B_GPIO_LOW;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LOW_OUTPUT_TYPE;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
    LL_GPIO_Init(PHASE_B_GPIO_PORT_LOW, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = PHASE_C_GPIO_LOW;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LOW_OUTPUT_TYPE;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
    LL_GPIO_Init(PHASE_C_GPIO_PORT_LOW, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = PHASE_A_GPIO_HIGH;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = HIGH_OUTPUT_TYPE;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
    LL_GPIO_Init(PHASE_A_GPIO_PORT_HIGH, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = PHASE_B_GPIO_HIGH;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = HIGH_OUTPUT_TYPE;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
    LL_GPIO_Init(PHASE_B_GPIO_PORT_HIGH, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = PHASE_C_GPIO_HIGH;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = HIGH_OUTPUT_TYPE;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
    LL_GPIO_Init(PHASE_C_GPIO_PORT_HIGH, &GPIO_InitStruct);

    //  NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn, 2);
    //  NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
}

void MX_TIM2_Init(void)
{
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
    TIM2->PSC = 23;
    TIM2->ARR = 0xFFFF;
}

void MX_TIM6_Init(void)
{
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM6);

    NVIC_SetPriority(TIM6_DAC_IRQn, 3);
    NVIC_EnableIRQ(TIM6_DAC_IRQn);
    TIM6->PSC = 47;
    TIM6->ARR = 1000000 / LOOP_FREQUENCY_HZ;
}

void MX_TIM14_Init(void)
{
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM14);
    TIM14->PSC = 23;
    TIM14->ARR = 4000;
    NVIC_SetPriority(TIM14_IRQn, 0);
    NVIC_EnableIRQ(TIM14_IRQn);
    LL_TIM_EnableARRPreload(TIM14);
}

void MX_TIM16_Init(void)
{
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM16);
    //  NVIC_SetPriority(TIM16_IRQn, 2);
    //  NVIC_EnableIRQ(TIM16_IRQn);
    TIM16->PSC = 0;
    TIM16->ARR = 9000;
    LL_TIM_DisableARRPreload(TIM16);
}

void MX_TIM17_Init(void)
{
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM17);
    TIM17->PSC = 47;
    TIM17->ARR = 0XFFFF;
    LL_TIM_DisableARRPreload(TIM17);
}

void MX_DMA_Init(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
    NVIC_SetPriority(DMA1_Channel2_3_IRQn, 1);
    NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
    NVIC_SetPriority(DMA1_Channel4_5_IRQn, 1);
    NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);
}

void MX_GPIO_Init(void)
{
    /* GPIO Ports Clock Enable */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

    /**/
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_15);

/**/
#ifdef USE_ALKAS_DEBUG_LED
    LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
#endif
}

void UN_TIM_Init(void)
{
    // LL_TIM_InitTypeDef TIM_InitStruct = {0};

    LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    /* Peripheral clock enable */
#ifdef USE_TIMER_15_CHANNEL_1
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM15);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    /**TIM16 GPIO Configuration
    PA6   ------> TIM16_CH1
    */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
#endif

#ifdef USE_TIMER_3_CHANNEL_1

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
    /**TIM16 GPIO Configuration
    PA6   ------> TIM16_CH1
    */
    GPIO_InitStruct.Pin = INPUT_PIN;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
#endif

    /* TIM16 DMA Init */

    /* TIM16_CH1_UP Init */
    // LL_DMA_SetDataTransferDirection(DMA1, INPUT_DMA_CHANNEL,
    // LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

    // LL_DMA_SetChannelPriorityLevel(DMA1, INPUT_DMA_CHANNEL,
    // LL_DMA_PRIORITY_LOW);

    // LL_DMA_SetMode(DMA1, INPUT_DMA_CHANNEL, LL_DMA_MODE_NORMAL);

    // LL_DMA_SetPeriphIncMode(DMA1, INPUT_DMA_CHANNEL,
    // LL_DMA_PERIPH_NOINCREMENT);

    // LL_DMA_SetMemoryIncMode(DMA1, INPUT_DMA_CHANNEL, LL_DMA_MEMORY_INCREMENT);

    // LL_DMA_SetPeriphSize(DMA1, INPUT_DMA_CHANNEL, LL_DMA_PDATAALIGN_HALFWORD);

    // LL_DMA_SetMemorySize(DMA1, INPUT_DMA_CHANNEL, LL_DMA_MDATAALIGN_WORD);

    /* TIM16 interrupt Init */

#ifdef USE_TIMER_15_CHANNEL_1
    NVIC_SetPriority(IC_DMA_IRQ_NAME, 1);
    NVIC_EnableIRQ(IC_DMA_IRQ_NAME);
#endif
#ifdef USE_TIMER_3_CHANNEL_1
    NVIC_SetPriority(IC_DMA_IRQ_NAME, 1);
    NVIC_EnableIRQ(IC_DMA_IRQ_NAME);
#endif

    // TIM_InitStruct.Prescaler = 0;
    // TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
    // TIM_InitStruct.Autoreload = 63;
    // TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    // TIM_InitStruct.RepetitionCounter = 0;
    //  LL_TIM_Init(IC_TIMER_REGISTER, &TIM_InitStruct);
    IC_TIMER_REGISTER->PSC = 0;
    IC_TIMER_REGISTER->ARR = 63;

    // LL_TIM_DisableARRPreload(IC_TIMER_REGISTER);
    // LL_TIM_IC_SetActiveInput(IC_TIMER_REGISTER, IC_TIMER_CHANNEL,
    // LL_TIM_ACTIVEINPUT_DIRECTTI); LL_TIM_IC_SetPrescaler(IC_TIMER_REGISTER,
    // IC_TIMER_CHANNEL, LL_TIM_ICPSC_DIV1);
    // LL_TIM_IC_SetFilter(IC_TIMER_REGISTER, IC_TIMER_CHANNEL,
    // LL_TIM_IC_FILTER_FDIV1); LL_TIM_IC_SetPolarity(IC_TIMER_REGISTER,
    // IC_TIMER_CHANNEL, LL_TIM_IC_POLARITY_BOTHEDGE);
}

#ifdef USE_RGB_LED // has 3 color led
void LED_GPIO_init()
{
    LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    /* GPIO Ports Clock Enable */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

    LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_8);
    LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_5);
    LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_3);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

#endif

#ifdef USE_CUSTOM_LED
void initLed()
{
    LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
#endif

void reloadWatchDogCounter()
{
    LL_IWDG_ReloadCounter(IWDG);
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

void setPrescalerPWM(uint16_t presc) { TIM1->PSC = presc; }

void setAutoReloadPWM(uint16_t relval) { TIM1->ARR = relval; }

void setDutyCycleAll(uint16_t newdc)
{
    TIM1->CCR1 = newdc;
    TIM1->CCR2 = newdc;
    TIM1->CCR3 = newdc;
}

inline void setPWMCompare1(uint16_t compareone) { TIM1->CCR1 = compareone; }
inline void setPWMCompare2(uint16_t comparetwo) { TIM1->CCR2 = comparetwo; }
inline void setPWMCompare3(uint16_t comparethree) { TIM1->CCR3 = comparethree; }

inline void generatePwmTimerEvent() { LL_TIM_GenerateEvent_UPDATE(TIM1); }

inline void resetInputCaptureTimer()
{
    IC_TIMER_REGISTER->PSC = 0;
    IC_TIMER_REGISTER->CNT = 0;
}

void enableCorePeripherals()
{
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1N);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2N);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3N);

#ifdef MCU_G071
    LL_TIM_CC_EnableChannel(
        TIM1, LL_TIM_CHANNEL_CH5); // timer used for comparator blanking
#endif
    LL_TIM_CC_EnableChannel(TIM1,
        LL_TIM_CHANNEL_CH4); // timer used for timing adc read
    TIM1->CCR4 = 100; // set in 10khz loop to match pwm cycle timed to end of pwm on

    /* Enable counter */
    LL_TIM_EnableCounter(TIM1);
    LL_TIM_EnableAllOutputs(TIM1);
    /* Force update generation */
    LL_TIM_GenerateEvent_UPDATE(TIM1);

#ifdef USE_ADC_INPUT

#else
    LL_TIM_CC_EnableChannel(IC_TIMER_REGISTER,
        IC_TIMER_CHANNEL); // input capture and output compare
    LL_TIM_EnableCounter(IC_TIMER_REGISTER);
#endif

#ifdef USE_LED_STRIP
    send_LED_RGB(255, 0, 0);
#endif

#ifdef USE_RGB_LED
    LED_GPIO_init();
    GPIOB->BRR = LL_GPIO_PIN_8; // turn on red
    GPIOB->BSRR = LL_GPIO_PIN_5;
    GPIOB->BSRR = LL_GPIO_PIN_3; //
#endif

#ifdef USE_CUSTOM_LED
    initLed();
#endif

#ifndef BRUSHED_MODE
    LL_TIM_EnableCounter(COM_TIMER); // commutation_timer priority 0
    LL_TIM_GenerateEvent_UPDATE(COM_TIMER);
    LL_TIM_EnableIT_UPDATE(COM_TIMER);
    COM_TIMER->DIER &= ~((0x1UL << (0U))); // disable for now.
#endif
    LL_TIM_EnableCounter(UTILITY_TIMER);
    LL_TIM_GenerateEvent_UPDATE(UTILITY_TIMER);
    //
    LL_TIM_EnableCounter(INTERVAL_TIMER);
    LL_TIM_GenerateEvent_UPDATE(INTERVAL_TIMER);

    LL_TIM_EnableCounter(TEN_KHZ_TIMER); // 10khz timer
    LL_TIM_GenerateEvent_UPDATE(TEN_KHZ_TIMER);
    TEN_KHZ_TIMER->DIER |= (0x1UL << (0U)); // enable interrupt
    // RCC->APB2ENR  &= ~(1 << 22);  // turn debug off
#ifdef USE_ADC
    ADC_Init();
    enableADC_DMA();
    activateADC();
#endif

#ifndef MCU_F031
    __IO uint32_t wait_loop_index = 0;
    /* Enable comparator */
    LL_COMP_Enable(MAIN_COMP);
#ifdef N_VARIANT // needs comp 1 and 2
    LL_COMP_Enable(COMP1);
#endif
    wait_loop_index = ((LL_COMP_DELAY_STARTUP_US * (SystemCoreClock / (100000 * 2))) / 10);
    while (wait_loop_index != 0) {
        wait_loop_index--;
    }
#endif
    NVIC_SetPriority(EXTI4_15_IRQn, 2);
    NVIC_EnableIRQ(EXTI4_15_IRQn);
    EXTI->IMR |= (1 << 15);
}
