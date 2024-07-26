/*
  MCU specific utility functions for the bootloader
 */
#pragma once

static inline void gpio_mode_set_input(uint32_t pin, uint32_t pull_up_down)
{
    if (pin < 8) {
	const uint32_t pin4 = (pin*pin*pin*pin);
	input_port->cfglr = (input_port->cfglr & ~(pin4*0xfU)) | (pin4*(GPIO_MODE_INPUT | pull_up_down));
    } else {
	pin = (pin-8);
	const uint32_t pin4 = (pin*pin*pin*pin);
	input_port->cfghr = (input_port->cfglr & ~(pin4*0xfU)) | (pin4*(GPIO_MODE_INPUT | pull_up_down));
    }
}

static inline void gpio_mode_set_output(uint32_t pin, uint32_t output_mode)
{
    if (pin < 8) {
	const uint32_t pin4 = (pin*pin*pin*pin);
	input_port->cfglr = (input_port->cfglr & ~(pin4*0xfU)) | (pin4*(GPIO_MODE_OUTPUT | output_mode));
    } else {
	pin = (pin-8);
	const uint32_t pin4 = (pin*pin*pin*pin);
	input_port->cfghr = (input_port->cfghr & ~(pin4*0xfU)) | (pin4*(GPIO_MODE_OUTPUT | output_mode));
    }
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

    BL_TIMER->div = 143;
    BL_TIMER->pr = 0xFFFF;
    BL_TIMER->swevt_bit.ovfswtr = TRUE;
	
    BL_TIMER->ctrl1_bit.tmren = TRUE;
}

unsigned int system_core_clock           = HICK_VALUE; /*!< system clock frequency (core clock) */

/*
  cut down version of system_core_clock_update for HICK source only
 */
void system_core_clock_update(void)
{
    uint32_t temp = 0, div_value = 0;

    static const uint8_t sys_ahb_div_table[16] = { 0, 0, 0, 0, 0, 0, 0, 0,
        1, 2, 3, 4, 6, 7, 8, 9 };

    if (((CRM->misc2_bit.hick_to_sclk) != RESET) && ((CRM->misc1_bit.hickdiv) != RESET))
	system_core_clock = HICK_VALUE * 6;
    else
	system_core_clock = HICK_VALUE;

    /* compute sclk, ahbclk frequency */
    /* get ahb division */
    temp = CRM->cfg_bit.ahbdiv;
    div_value = sys_ahb_div_table[temp];
    /* ahbclk frequency */
    system_core_clock = system_core_clock >> div_value;
}

/*
  initialise clocks
 */
static inline void bl_clock_config(void)
{
    /* config flash psr register */
    flash_psr_set(FLASH_WAIT_CYCLE_4);

    /* reset crm */
    crm_reset();

    crm_clock_source_enable(CRM_CLOCK_SOURCE_HICK, TRUE);

    /* wait till hick is ready */
    while(crm_flag_get(CRM_HICK_STABLE_FLAG) != SET)
    {
    }

    /* config pll clock resource */
    crm_pll_config(CRM_PLL_SOURCE_HICK, CRM_PLL_MULT_36);

    /* enable pll */
    crm_clock_source_enable(CRM_CLOCK_SOURCE_PLL, TRUE);

    /* wait till pll is ready */
    while(crm_flag_get(CRM_PLL_STABLE_FLAG) != SET)
    {
    }

    /* config ahbclk */
    crm_ahb_div_set(CRM_AHB_DIV_1);

    /* config apb2clk */
    crm_apb2_div_set(CRM_APB2_DIV_2);

    /* config apb1clk */
    crm_apb1_div_set(CRM_APB1_DIV_2);

    /* enable auto step mode */
    crm_auto_step_mode_enable(TRUE);

    /* select pll as system clock source */
    crm_sysclk_switch(CRM_SCLK_PLL);

    /* wait till pll is used as system clock source */
    while(crm_sysclk_switch_status_get() != CRM_SCLK_PLL)
    {
    }

    /* disable auto step mode */
    crm_auto_step_mode_enable(FALSE);

    /* update system_core_clock global variable */
    system_core_clock_update();
}

static inline void bl_gpio_init(void)
{
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK , TRUE);
    gpio_pin_remap_config(SWJTAG_GMUX_010, TRUE);	//   pb4 GPIO
	
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
