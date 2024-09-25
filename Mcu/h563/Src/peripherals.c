/*
 * peripherals.c
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 */

// PERIPHERAL SETUP

#include "peripherals.h"

#include "gpio.h"
#include "ADC.h"
#include "serial_telemetry.h"
#include "targets.h"

void msp_system_initialize()
{

  // Set core voltage regulator output scaling for maximum performance
  PWR->VOSCR |= 0b11 << PWR_VOSCR_VOS_Pos;
  while (!(PWR->VOSSR & PWR_VOSSR_VOSRDY));
  while (!(PWR->VOSSR & PWR_VOSSR_ACTVOSRDY));

  // set flash latency to 5 wait states for 250MHz SYSCLK
  FLASH->ACR |= 5 << FLASH_ACR_LATENCY_Pos;

  // enable prefetch buffer
  FLASH->ACR |= FLASH_ACR_PRFTEN;

  // ~~~~~~~~ USE HSE ~~~~~~~~~~~~~~
  // turn the HSE on
  RCC->CR |= RCC_CR_HSEON;

  // wait for high speed external oscillator (HSE) to be ready
  while (!(RCC->CR & RCC_CR_HSERDY));

  // // set pll clock source to HSI
  // RCC->PLL1CFGR |= 0b01 << RCC_PLL1CFGR_PLL1SRC_Pos;

  // // set pll clock source to HSE
  RCC->PLL1CFGR |= 0b11 << RCC_PLL1CFGR_PLL1SRC_Pos;

  // // // ~~~~~~~~ \USE HSE ~~~~~~~~~~~~~

  // The frequency of the reference clock provided to the PLLs (refx_ck) must range from 1 to
  // 16 MHz. The DIVMx dividers of the RCC PLL clock source selection register
  // (RCC_PLL1CFGR) must be properly programmed in order to match this condition.
  // divide by 12, 2MHz for a 24MHz HSE
  uint32_t pll1cfgr = RCC->PLL1CFGR;
  pll1cfgr &= ~RCC_PLL1CFGR_PLL1M_Msk;
  RCC->PLL1CFGR |= pll1cfgr | (12 << RCC_PLL1CFGR_PLL1M_Pos);

  // enable PLL1 p_clk output for use as SYSCLK
  RCC->PLL1CFGR |= 1 << RCC_PLL1CFGR_PLL1PEN_Pos;

  // set pll multiplier
  uint32_t pll1divr = RCC->PLL1DIVR;
  // pll1divr &= ~(RCC_PLL1DIVR_PLL1N_Msk);
  pll1divr &= ~(RCC_PLL1DIVR_PLL1N_Msk);
    // RCC->PLL1DIVR = pll1divr | ((250-1) << RCC_PLL1DIVR_PLL1N_Pos);
  // pll1divr |= ((SYSCLK_FREQUENCY/1000000 - 1) << RCC_PLL1DIVR_PLL1N_Pos);
  // RCC->PLL1DIVR = pll1divr | (249 << RCC_PLL1DIVR_PLL1N_Pos);
  RCC->PLL1DIVR = pll1divr | (249 << RCC_PLL1DIVR_PLL1N_Pos);

  // turn the pll on
  RCC->CR |= RCC_CR_PLL1ON;

  // wait for pll to be ready
  while (!(RCC->CR & RCC_CR_PLL1RDY));

  // switch system clock to pll1 p_clk
  RCC->CFGR1 |= 0b11 << RCC_CFGR1_SW_Pos;

  // wait for any ongoing cache invalidation
  while (ICACHE->CR & ICACHE_SR_BUSYF);
  // enable icache miss monitor, hit monitor, and icache itself
  ICACHE->CR |= ICACHE_CR_MISSMEN | ICACHE_CR_HITMEN | ICACHE_CR_EN;
}

void initCorePeripherals(void)
{
    msp_system_initialize();
    // LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);
    // LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
    MX_GPIO_Init();
    MX_DMA_Init();
    // MX_TIM1_Init();
    interval_timer_initialize();
    MX_COMP1_Init();
    com_timer_initialize();
    ten_khz_timer_initialize();
    // MX_TIM17_Init();
    utility_timer_initialize();
    UN_TIM_Init();
#ifdef USE_SERIAL_TELEMETRY
    telem_UART_Init();
#endif
}

void initAfterJump(void)
{
    __enable_irq();
}

void MX_IWDG_Init(void)
{
    IWDG->KR = 0x0000CCCCU;
    IWDG->KR = 0x00005555U;
    IWDG->PR = LL_IWDG_PRESCALER_16;
    IWDG->RLR = 4000;
    LL_IWDG_ReloadCounter(IWDG);
}

// void MX_TIM1_Init(void)
// {
//     LL_TIM_InitTypeDef TIM_InitStruct = { 0 };
//     LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = { 0 };
//     LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct = { 0 };
//     LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };
//     LL_APB1_GRP2_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

//     TIM_InitStruct.Prescaler = 0;
//     TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
//     TIM_InitStruct.Autoreload = 1999;
//     TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
//     TIM_InitStruct.RepetitionCounter = 0;
//     LL_TIM_Init(TIM1, &TIM_InitStruct);
//     LL_TIM_EnableARRPreload(TIM1);
//     LL_TIM_SetClockSource(TIM1, LL_TIM_CLOCKSOURCE_INTERNAL);
//     LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);
// #ifdef USE_SWAPPED_OUPUT
//     TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM2;
// #else
//     TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
// #endif
//     TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
//     TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
//     TIM_OC_InitStruct.CompareValue = 0;
// #ifdef USE_INVERTED_HIGH
//     TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_LOW;
//     TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_HIGH;
// #else
//     TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
//     TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
// #endif
// #ifdef USE_INVERTED_LOW
//     TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_LOW;
//     TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_HIGH;
// #else
//     TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
//     TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
// #endif
//     LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
//     LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH1);
//     LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH2);
//     TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
//     TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
//     LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
//     LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH2);
//     LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH3);
//     TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
//     TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
//     LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
//     LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH3);
//     LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH4);
//     TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
//     TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
//     LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH4, &TIM_OC_InitStruct);
//     LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH4);
//     LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);
//     LL_TIM_DisableMasterSlaveMode(TIM1);
//     TIM_BDTRInitStruct.OSSRState = LL_TIM_OSSR_DISABLE;
//     TIM_BDTRInitStruct.OSSIState = LL_TIM_OSSI_DISABLE;
//     TIM_BDTRInitStruct.LockLevel = LL_TIM_LOCKLEVEL_OFF;
//     TIM_BDTRInitStruct.DeadTime = DEAD_TIME;
//     TIM_BDTRInitStruct.BreakState = LL_TIM_BREAK_DISABLE;
//     TIM_BDTRInitStruct.BreakPolarity = LL_TIM_BREAK_POLARITY_HIGH;
//     TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_DISABLE;
//     LL_TIM_BDTR_Init(TIM1, &TIM_BDTRInitStruct);

//     LL_AHB1_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
//     LL_AHB1_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
// /**TIM1 GPIO Configuration
// PA7   ------> TIM1_CH1N
// PB0   ------> TIM1_CH2N
// PB1   ------> TIM1_CH3N
// PA8   ------> TIM1_CH1
// PA9   ------> TIM1_CH2
// PA10   ------> TIM1_CH3
// */
// #ifdef USE_OPEN_DRAIN_LOW
// #pragma message("using open drain low side")
// #define LOW_OUTPUT_TYPE LL_GPIO_OUTPUT_OPENDRAIN
// #else
// #define LOW_OUTPUT_TYPE LL_GPIO_OUTPUT_PUSHPULL
// #endif
// #ifdef USE_OPEN_DRAIN_HIGH
// #pragma message("using open drain high side")
// #define HIGH_OUTPUT_TYPE LL_GPIO_OUTPUT_OPENDRAIN
// #else
// #define HIGH_OUTPUT_TYPE LL_GPIO_OUTPUT_PUSHPULL
// #endif
//     GPIO_InitStruct.Pin = PHASE_A_GPIO_LOW;
//     GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
//     GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
//     GPIO_InitStruct.OutputType = LOW_OUTPUT_TYPE;
//     GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//     GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
//     LL_GPIO_Init(PHASE_A_GPIO_PORT_LOW, &GPIO_InitStruct);

//     GPIO_InitStruct.Pin = PHASE_B_GPIO_LOW;
//     GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
//     GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
//     GPIO_InitStruct.OutputType = LOW_OUTPUT_TYPE;
//     GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//     GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
//     LL_GPIO_Init(PHASE_B_GPIO_PORT_LOW, &GPIO_InitStruct);

//     GPIO_InitStruct.Pin = PHASE_C_GPIO_LOW;
//     GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
//     GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
//     GPIO_InitStruct.OutputType = LOW_OUTPUT_TYPE;
//     GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//     GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
//     LL_GPIO_Init(PHASE_C_GPIO_PORT_LOW, &GPIO_InitStruct);

//     GPIO_InitStruct.Pin = PHASE_A_GPIO_HIGH;
//     GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
//     GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
//     GPIO_InitStruct.OutputType = HIGH_OUTPUT_TYPE;
//     GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//     GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
//     LL_GPIO_Init(PHASE_A_GPIO_PORT_HIGH, &GPIO_InitStruct);

//     GPIO_InitStruct.Pin = PHASE_B_GPIO_HIGH;
//     GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
//     GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
//     GPIO_InitStruct.OutputType = HIGH_OUTPUT_TYPE;
//     GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//     GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
//     LL_GPIO_Init(PHASE_B_GPIO_PORT_HIGH, &GPIO_InitStruct);

//     GPIO_InitStruct.Pin = PHASE_C_GPIO_HIGH;
//     GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
//     GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
//     GPIO_InitStruct.OutputType = HIGH_OUTPUT_TYPE;
//     GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//     GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
//     LL_GPIO_Init(PHASE_C_GPIO_PORT_HIGH, &GPIO_InitStruct);

//     //  NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn, 2);
//     //  NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
// }

// interval timer
void interval_timer_initialize(void)
{
    INTERVAL_TIMER_ENABLE_CLOCK();
    TIM2->PSC = 23;
    TIM2->ARR = 0xFFFF;
}

void ten_khz_timer_initialize(void)
{
    TEN_KHZ_TIMER_ENABLE_CLOCK();
    NVIC_SetPriority(TEN_KHZ_TIMER_IRQn, 3);
    NVIC_EnableIRQ(TEN_KHZ_TIMER_IRQn);
    TEN_KHZ_TIMER->PSC = 47;
    TEN_KHZ_TIMER->ARR = 1000000 / LOOP_FREQUENCY_HZ;
}

void com_timer_initialize(void)
{
    COM_TIMER_ENABLE_CLOCK();
    COM_TIMER->PSC = 23;
    COM_TIMER->ARR = 4000;
    NVIC_SetPriority(COM_TIMER_IRQ, 0);
    NVIC_EnableIRQ(COM_TIMER_IRQ);
    LL_TIM_EnableARRPreload(COM_TIMER);
}

void MX_TIM16_Init(void)
{
    LL_APB1_GRP2_EnableClock(LL_APB2_GRP1_PERIPH_TIM16);
    //  NVIC_SetPriority(TIM16_IRQn, 2);
    //  NVIC_EnableIRQ(TIM16_IRQn);
    TIM16->PSC = 0;
    TIM16->ARR = 9000;
    LL_TIM_DisableARRPreload(TIM16);
}

void utility_timer_initialize(void)
{
    UTILITY_TIMER_ENABLE_CLOCK();
    UTILITY_TIMER->PSC = UTILITY_TIMER_PSC;
    UTILITY_TIMER->ARR = 0XFFFF;
    LL_TIM_DisableARRPreload(UTILITY_TIMER);
}

void utility_timer_enable(void)
{
    LL_TIM_EnableCounter(UTILITY_TIMER);
    LL_TIM_GenerateEvent_UPDATE(UTILITY_TIMER);
}

void MX_DMA_Init(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPDMA1);
    // NVIC_SetPriority(DMA1_Channel2_3_IRQn, 1);
    // NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
    // NVIC_SetPriority(DMA1_Channel4_5_IRQn, 1);
    // NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);
}

void MX_GPIO_Init(void)
{
    /* GPIO Ports Clock Enable */
    LL_AHB1_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    LL_AHB1_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);

    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_15);
}

void input_timer_initialize(void)
{
    INPUT_TIMER_ENABLE_CLOCK();

    NVIC_SetPriority(IC_DMA_IRQ_NAME, 1);
    NVIC_EnableIRQ(IC_DMA_IRQ_NAME);

    input_timer_gpio_initialize();
    INPUT_TIMER->PSC = 0;
    INPUT_TIMER->ARR = 63;

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
    //  LL_TIM_Init(INPUT_TIMER, &TIM_InitStruct);
    INPUT_TIMER->PSC = 0;
    INPUT_TIMER->ARR = 63;

    // LL_TIM_DisableARRPreload(INPUT_TIMER);
    // LL_TIM_IC_SetActiveInput(INPUT_TIMER, IC_TIMER_CHANNEL,
    // LL_TIM_ACTIVEINPUT_DIRECTTI); LL_TIM_IC_SetPrescaler(INPUT_TIMER,
    // IC_TIMER_CHANNEL, LL_TIM_ICPSC_DIV1);
    // LL_TIM_IC_SetFilter(INPUT_TIMER, IC_TIMER_CHANNEL,
    // LL_TIM_IC_FILTER_FDIV1); LL_TIM_IC_SetPolarity(INPUT_TIMER,
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

void setPrescalerPWM(uint16_t presc) { COM_TIMER->PSC = presc; }

void setAutoReloadPWM(uint16_t relval) { COM_TIMER->ARR = relval; }

void setDutyCycleAll(uint16_t newdc)
{
    TIM1->CCR1 = newdc;
    TIM1->CCR2 = newdc;
    TIM1->CCR3 = newdc;
}

void inline setPWMCompare1(uint16_t compareone) { COM_TIMER->CCR1 = compareone; }
void inline setPWMCompare2(uint16_t comparetwo) { COM_TIMER->CCR2 = comparetwo; }
void inline setPWMCompare3(uint16_t comparethree) { COM_TIMER->CCR3 = comparethree; }

void inline generatePwmTimerEvent() { LL_TIM_GenerateEvent_UPDATE(TIM1); }

void inline resetInputCaptureTimer()
{
    INPUT_TIMER->PSC = 0;
    INPUT_TIMER->CNT = 0;
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
    LL_TIM_CC_EnableChannel(INPUT_TIMER,
        IC_TIMER_CHANNEL); // input capture and output compare
    LL_TIM_EnableCounter(INPUT_TIMER);
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
    utility_timer_enable();
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

// #ifndef MCU_F031
//     __IO uint32_t wait_loop_index = 0;
//     /* Enable comparator */
//     LL_COMP_Enable(MAIN_COMP);
// #ifdef N_VARIANT // needs comp 1 and 2
//     LL_COMP_Enable(COMP1);
// #endif
//     wait_loop_index = ((LL_COMP_DELAY_STARTUP_US * (SystemCoreClock / (100000 * 2))) / 10);
//     while (wait_loop_index != 0) {
//         wait_loop_index--;
//     }
// #endif
    NVIC_SetPriority(EXTI15_IRQn, 2);
    NVIC_EnableIRQ(EXTI15_IRQn);
    EXTI->IMR1 |= (1 << 15);
}
