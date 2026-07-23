/*
  main.h for the SITL "MCU". Provides just enough of the STM32 LL/CMSIS
  surface for the shared firmware sources to build natively, backed by the
  emulation in Mcu/SITL/Src
 */

#ifndef __MAIN_H
#define __MAIN_H

#include <stdint.h>
#include <stdbool.h>

#include "sitl.h"
#include "sitl_nvic.h"

// emulated timers. Dereferencing syncs CNT from simulated time
#define TIM1 sitl_tim_deref(SITL_TIM1_IDX)
#define TIM2 sitl_tim_deref(SITL_TIM2_IDX)
#define TIM6 sitl_tim_deref(SITL_TIM6_IDX)
#define TIM16 sitl_tim_deref(SITL_TIM16_IDX)
#define TIM17 sitl_tim_deref(SITL_TIM17_IDX)

// minimal register structs so common.h externs compile
typedef struct {
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BSRR;
    volatile uint32_t BRR;
} GPIO_TypeDef;

typedef struct {
    volatile uint32_t CSR;
} COMP_TypeDef;

extern GPIO_TypeDef sitl_gpio_dummy;
extern COMP_TypeDef sitl_comp_dummy[2];
#define GPIOA (&sitl_gpio_dummy)
#define GPIOB (&sitl_gpio_dummy)
#define COMP1 (&sitl_comp_dummy[0])
#define COMP2 (&sitl_comp_dummy[1])

// ADC: the simulation provides the raw values directly, conversion start
// is a no-op and the "temperature calculation" is a pass through (the
// simulation stores degrees C in ADC_raw_temp)
#define ADC1 0
#define LL_ADC_REG_StartConversion(adc) do { (void)(adc); } while (0)
#define LL_ADC_RESOLUTION_12B 0
#define __LL_ADC_CALC_TEMPERATURE(vref, raw, res) ((int32_t)(raw))

// watchdog
#define IWDG 0
#define LL_IWDG_ReloadCounter(wdg) sitl_watchdog_reload()

#ifndef RESET
#define RESET 0
#endif

void Error_Handler(void);

#endif /* __MAIN_H */
