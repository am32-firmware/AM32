/*
  MCU specific utility functions for the bootloader
 */
#pragma once

#define GPIO_PIN(n) (1U<<(n))

/*
  16k ram
 */
#define RAM_BASE 0x20000000
#define RAM_SIZE 16*1024
#define STACK_TOP RAM_BASE+RAM_SIZE

/*
  32k flash
 */
#define BOARD_FLASH_SIZE 32

static inline void gpio_mode_set_input(uint32_t pin, uint32_t pull_up_down)
{
    const uint32_t pinsq = (pin*pin);
    input_port->cfgr = (input_port->cfgr & ~(pinsq * 0x3U)) | (pinsq * GPIO_MODE_INPUT);
    input_port->pull = (input_port->pull & ~(pinsq * 0x3U)) | (pinsq * pull_up_down);
}

static inline void gpio_mode_set_output(uint32_t pin, uint32_t output_mode)
{
    const uint32_t pinsq = (pin*pin);
    input_port->cfgr = (input_port->cfgr & ~(pinsq * 0x3U)) | (pinsq * GPIO_MODE_OUTPUT);
    input_port->omode = (input_port->omode & ~pin) | (output_mode << pin);
}

static inline void gpio_set(uint32_t pin)
{
    input_port->scr = pin;
}

static inline void gpio_clear(uint32_t pin)
{
    input_port->clr = pin;
}

static inline bool gpio_read(uint32_t pin)
{
    return (input_port->idt & pin) != 0;
}

#define BL_TIMER TMR3

/*
  initialise timer for 1us per tick
 */
static inline void bl_timer_init(void)
{
    crm_periph_clock_enable(CRM_TMR3_PERIPH_CLOCK, TRUE);
	
    BL_TIMER->div = 119;
    BL_TIMER->pr = 0xFFFF;
    BL_TIMER->ctrl1_bit.tmren = TRUE;
}

/*
  disable timer ready for app start
 */
static inline void bl_timer_disable(void)
{
    BL_TIMER->ctrl1_bit.tmren = FALSE;
}

static inline uint32_t bl_timer_us(void)
{
    return BL_TIMER->cval;
}

static inline void bl_timer_reset(void)
{
    BL_TIMER->cval = 0;
}

/*
  initialise clocks
 */
static inline void bl_clock_config(void)
{
    flash_psr_set(FLASH_WAIT_CYCLE_3);
    crm_reset();
    crm_clock_source_enable(CRM_CLOCK_SOURCE_HICK, TRUE);
    while(crm_flag_get(CRM_HICK_STABLE_FLAG) != SET)
    {
    }
    crm_pll_config(CRM_PLL_SOURCE_HICK, CRM_PLL_MULT_30);
    crm_clock_source_enable(CRM_CLOCK_SOURCE_PLL, TRUE);
    while(crm_flag_get(CRM_PLL_STABLE_FLAG) != SET)
    {
    }
    crm_ahb_div_set(CRM_AHB_DIV_1);
    crm_apb2_div_set(CRM_APB2_DIV_1);
    crm_apb1_div_set(CRM_APB1_DIV_1);
    crm_auto_step_mode_enable(TRUE);
    crm_sysclk_switch(CRM_SCLK_PLL);
    while(crm_sysclk_switch_status_get() != CRM_SCLK_PLL)
    {
    }
    crm_auto_step_mode_enable(FALSE);
}

static inline void bl_gpio_init(void)
{
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
	
    gpio_init_type gpio_init_struct;
    gpio_default_para_init(&gpio_init_struct);
	
    gpio_init_struct.gpio_pins  = input_pin;
    gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;

    gpio_init(input_port, &gpio_init_struct);
}

/*
  return true if the MCU booted under a software reset
 */
static inline bool bl_was_software_reset(void)
{
    return crm_flag_get(CRM_SW_RESET_FLAG) == SET;
}

/*
  no need for any action in SystemInit
 */
void SystemInit()
{
}

/*
  jump from the bootloader to the application code
 */
static inline void jump_to_application(void)
{
    __disable_irq();
    bl_timer_disable();
    const uint32_t app_address = STM32_FLASH_START + FIRMWARE_RELATIVE_START;
    const uint32_t *app_data = (const uint32_t *)app_address;
    const uint32_t stack_top = app_data[0];
    const uint32_t JumpAddress = app_data[1];

    // setup sp, msp and jump
    asm volatile(
        "mov sp, %0	\n"
        "msr msp, %0	\n"
        "bx	%1	\n"
	: : "r"(stack_top), "r"(JumpAddress) :);
}
