/*
  MCU specific utility functions for the bootloader
 */
/*
  based on https://github.com/AlkaMotors/g071Bootloader
 */
#pragma once

#define GPIO_PINS_2 (1U<<2)
#define GPIO_PINS_4 (1U<<4)

#define GPIO_PULL_NONE LL_GPIO_PULL_NO
#define GPIO_PULL_UP   LL_GPIO_PULL_UP
#define GPIO_PULL_DOWN LL_GPIO_PULL_DOWN

#define GPIO_OUTPUT_PUSH_PULL LL_GPIO_OUTPUT_PUSHPULL

// assume 8MHz crystal
uint32_t SystemCoreClock = 8000000U;

static inline void gpio_mode_set_input(uint32_t pin, uint32_t pull_up_down)
{
    LL_GPIO_SetPinPull(input_port, pin, pull_up_down);
}

static inline void gpio_mode_set_output(uint32_t pin, uint32_t output_mode)
{
    LL_GPIO_SetPinMode(input_port, pin, LL_GPIO_MODE_OUTPUT);
}

static inline void gpio_set(uint32_t pin)
{
    input_port->BSRR = pin;
}

static inline void gpio_clear(uint32_t pin)
{
    input_port->BRR = pin;
}

static inline bool gpio_read(uint32_t pin)
{
    return (input_port->IDR & pin) != 0;
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

    TIM_InitStruct.Prescaler = 47;
    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload = 0xFFFFFFFF;
    TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    LL_TIM_Init(BL_TIMER, &TIM_InitStruct);
    LL_TIM_DisableARRPreload(BL_TIMER);
    LL_TIM_SetClockSource(BL_TIMER, LL_TIM_CLOCKSOURCE_INTERNAL);
    LL_TIM_SetTriggerOutput(BL_TIMER, LL_TIM_TRGO_RESET);
    LL_TIM_DisableMasterSlaveMode(BL_TIMER);
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
    return BL_TIMER->CNT;
}

static inline void bl_timer_reset(void)
{
    BL_TIMER->CNT = 0;
}

/*
  initialise clocks
 */
static inline void bl_clock_config(void)
{
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);

    /* HSI configuration and activation */
    LL_RCC_HSI_Enable();
    while(LL_RCC_HSI_IsReady() != 1)
    {
    }

    /* Main PLL configuration and activation */
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_1, 8, LL_RCC_PLLR_DIV_2);
    LL_RCC_PLL_Enable();
    LL_RCC_PLL_EnableDomain_SYS();
    while(LL_RCC_PLL_IsReady() != 1)
    {
    }

    /* Set AHB prescaler*/
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

    /* Sysclk activation on the main PLL */
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
    while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
    {
    }

    /* Set APB1 prescaler*/
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
    LL_Init1msTick(64000000);
    LL_SetSystemCoreClock(64000000);
    /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
    LL_SetSystemCoreClock(64000000);
}

static inline void bl_gpio_init(void)
{
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);

    /**/
    GPIO_InitStruct.Pin = input_pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(input_port, &GPIO_InitStruct);
}

/*
  return true if the MCU booted under a software reset
 */
static inline bool bl_was_software_reset(void)
{
    return (RCC->CSR & RCC_CSR_SFTRSTF) != 0;
}

/*
  no need for any action in SystemInit
 */
void SystemInit()
{
}
