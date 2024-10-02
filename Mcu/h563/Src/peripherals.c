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
#include "stm32h5xx_ll_bus.h"
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
    input_timer_initialize();
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
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_WWDG);
    IWDG->KR = 0x0000CCCCU;
    IWDG->KR = 0x00005555U;
    IWDG->PR = LL_IWDG_PRESCALER_16;
    IWDG->RLR = 4000;
    // while (IWDG->SR); // wait for the registers to be updated
    IWDG->KR = 0x0000AAAA;
}

// interval timer
void interval_timer_initialize(void)
{
    INTERVAL_TIMER_ENABLE_CLOCK();
    TIM2->PSC = 23;
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

    INPUT_TIMER->PSC = 0;
    INPUT_TIMER->ARR = 63;
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

    LL_TIM_CC_EnableChannel(TIM1,
        LL_TIM_CHANNEL_CH4); // timer used for timing adc read
    TIM1->CCR4 = 100;    // max value for micros is 0xffff
    // max value for millis is 0xffff/10 = 6553; // set in 10khz loop to match pwm cycle timed to end of pwm on

    /* Enable counter */
    LL_TIM_EnableCounter(TIM1);
    LL_TIM_EnableAllOutputs(TIM1);
    /* Force update generation */
    LL_TIM_GenerateEvent_UPDATE(TIM1);

    input_timer_enable();

#ifndef BRUSHED_MODE
    LL_TIM_EnableCounter(COM_TIMER); // commutation_timer priority 0
    LL_TIM_GenerateEvent_UPDATE(COM_TIMER);
    LL_TIM_EnableIT_UPDATE(COM_TIMER);
    COM_TIMER->DIER &= ~((0x1UL << (0U))); // disable for now.
#endif

    utility_timer_enable();

    interval_timer_enable();

    ten_khz_timer_enable();
    ten_khz_timer_interrupt_enable();

    // RCC->APB2ENR  &= ~(1 << 22);  // turn debug off
#ifdef USE_ADC
    ADC_Init();
    enableADC_DMA();
    activateADC();
#endif

    // comparator interrupt?
    // NVIC_SetPriority(EXTI15_IRQn, 2);
    // NVIC_EnableIRQ(EXTI15_IRQn);
    // EXTI->IMR1 |= (1 << 15);
}
