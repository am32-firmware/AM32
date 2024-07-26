/*
  MCU specific utility functions for the bootloader
 */
#pragma once

#define GPIO_PINS_2 GPIO_PIN_2
#define GPIO_PINS_4 GPIO_PIN_4

#define GPIO_PULL_NONE GPIO_PUPD_NONE
#define GPIO_PULL_UP GPIO_PUPD_PULLUP
#define GPIO_PULL_DOWN GPIO_PUPD_PULLDOWN

#define GPIO_OUTPUT_PUSH_PULL GPIO_OTYPE_PP

static inline void gpio_mode_set_input(uint32_t pin, uint32_t pull_up_down)
{
    gpio_mode_set(input_port, GPIO_MODE_INPUT, pull_up_down, pin);
}

static inline void gpio_mode_set_output(uint32_t pin, uint32_t output_mode)
{
    gpio_mode_set(input_port, GPIO_MODE_OUTPUT, output_mode, pin);
}

static inline void gpio_set(uint32_t pin)
{
    gpio_bit_set(input_port, pin);
}

static inline void gpio_clear(uint32_t pin)
{
    gpio_bit_reset(input_port, pin);
}

static inline bool gpio_read(uint32_t pin)
{
    return (gpio_input_port_get(input_port) & pin) != 0;
}

#define BL_TIMER TIMER16

/*
  initialise timer for 1us per tick
 */
static inline void bl_timer_init(void)
{
    rcu_periph_clock_enable(RCU_TIMER16);
    TIMER_CAR(BL_TIMER) = 0xFFFF;
    TIMER_PSC(BL_TIMER) = 35;
    timer_auto_reload_shadow_enable(BL_TIMER);
    timer_enable(BL_TIMER);
}

/*
  disable timer ready for app start
 */
static inline void bl_timer_disable(void)
{
    timer_disable(BL_TIMER);
}

static inline uint32_t bl_timer_us(void)
{
    return timer_counter_read(BL_TIMER);
}

static inline void bl_timer_reset(void)
{
    timer_counter_value_config(BL_TIMER, 0);
}

/*
  initialise clocks
 */
static inline void bl_clock_config(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
}

static inline void bl_gpio_init(void)
{
}

/*
  return true if the MCU booted under a software reset
 */
static inline bool bl_was_software_reset(void)
{
    return (RCU_RSTSCK & RCU_RSTSCK_SWRSTF) != 0;
}
