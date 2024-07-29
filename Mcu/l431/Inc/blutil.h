/*
  MCU specific utility functions for the bootloader
 */
#pragma once

#define GPIO_PIN(n) (1U<<(n))

#define GPIO_PULL_NONE LL_GPIO_PULL_NO
#define GPIO_PULL_UP   LL_GPIO_PULL_UP
#define GPIO_PULL_DOWN LL_GPIO_PULL_DOWN

#define GPIO_OUTPUT_PUSH_PULL LL_GPIO_OUTPUT_PUSHPULL

static inline void gpio_mode_set_input(uint32_t pin, uint32_t pull_up_down)
{
    LL_GPIO_SetPinMode(input_port, pin, LL_GPIO_MODE_INPUT);
    LL_GPIO_SetPinPull(input_port, pin, pull_up_down);
}

static inline void gpio_mode_set_output(uint32_t pin, uint32_t output_mode)
{
    LL_GPIO_SetPinMode(input_port, pin, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(input_port, pin, output_mode);
}

static inline void gpio_set(uint32_t pin)
{
    LL_GPIO_SetOutputPin(input_port, pin);
}

static inline void gpio_clear(uint32_t pin)
{
    LL_GPIO_ResetOutputPin(input_port, pin);
}

static inline bool gpio_read(uint32_t pin)
{
    return LL_GPIO_IsInputPinSet(input_port, pin);
}

#define BL_TIMER TIM2

/*
  initialise timer for 1us per tick
 */
static inline void bl_timer_init(void)
{
    LL_TIM_InitTypeDef TIM_InitStruct = {0};

    /* Peripheral clock enable */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

    TIM_InitStruct.Prescaler = 79;
    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload = 0xFFFFFFFF;
    TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    LL_TIM_Init(BL_TIMER, &TIM_InitStruct);
    LL_TIM_DisableARRPreload(BL_TIMER);
    LL_TIM_SetClockSource(BL_TIMER, LL_TIM_CLOCKSOURCE_INTERNAL);
    LL_TIM_SetTriggerOutput(BL_TIMER, LL_TIM_TRGO_RESET);
    LL_TIM_DisableMasterSlaveMode(BL_TIMER);

    LL_TIM_SetCounterMode(BL_TIMER, LL_TIM_COUNTERMODE_UP);
    LL_TIM_EnableCounter(BL_TIMER);
}

/*
  disable timer ready for app start
 */
static inline void bl_timer_disable(void)
{
    LL_TIM_DeInit(BL_TIMER);
}

static inline uint32_t bl_timer_us(void)
{
    return LL_TIM_GetCounter(BL_TIMER);
}

static inline void bl_timer_reset(void)
{
    LL_TIM_SetCounter(BL_TIMER, 0);
}

/*
  initialise clocks
 */
static inline void bl_clock_config(void)
{
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
    while (LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_4) ;
    LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
    while (LL_PWR_IsActiveFlag_VOS() != 0) ;
    LL_RCC_MSI_Enable();

    /* Wait till MSI is ready */
    while (LL_RCC_MSI_IsReady() != 1) ;

    LL_RCC_MSI_EnableRangeSelection();
    LL_RCC_MSI_SetRange(LL_RCC_MSIRANGE_6);
    LL_RCC_MSI_SetCalibTrimming(0);
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_MSI, LL_RCC_PLLM_DIV_1, 40, LL_RCC_PLLR_DIV_2);
    LL_RCC_PLL_EnableDomain_SYS();
    LL_RCC_PLL_Enable();

    /* Wait till PLL is ready */
    while (LL_RCC_PLL_IsReady() != 1) ;
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

    /* Wait till System clock is ready */
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) ;

    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
}

static inline void bl_gpio_init(void)
{
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    LL_GPIO_ResetOutputPin(input_port, input_pin);
}

/*
  return true if the MCU booted under a software reset
 */
static inline bool bl_was_software_reset(void)
{
    return (RCC->CSR & RCC_CSR_SFTRSTF) != 0;
}

void Error_Handler()
{
    while (1) {}
}
