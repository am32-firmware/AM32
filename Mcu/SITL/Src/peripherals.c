/*
  peripherals.c - SITL clock/peripheral bring-up, mirroring the g431
  version with the hardware replaced by the emulation
 */

#include "peripherals.h"

#include "ADC.h"
#include "comparator.h"
#include "sitl.h"
#include "targets.h"

#include <stdio.h>
#include <stdlib.h>

GPIO_TypeDef sitl_gpio_dummy;
COMP_TypeDef sitl_comp_dummy[2];

void initAfterJump(void) { }

void SystemClock_Config(void) { }
void MX_GPIO_Init(void) { }
void MX_DMA_Init(void) { }
void MX_COMP1_Init(void) { }
void MX_COMP2_Init(void) { }
void MX_TIM2_Init(void) { }
void MX_TIM3_Init(void) { }
void MX_TIM15_Init(void) { }
void MX_TIM17_Init(void) { }

void MX_TIM1_Init(void)
{
    sitl_tim1_set_psc(0);
    sitl_tim1_set_arr(TIM1_AUTORELOAD);
    // dead time as on the g431 target; loadEEpromSettings ORs
    // dead_time_override into BDTR on top of this
    sitl_tim_deref(SITL_TIM1_IDX)->BDTR = DEAD_TIME;
    sitl_tim1_force_update();
}

void MX_TIM6_Init(void)
{
    sitl_tim_deref(SITL_TIM6_IDX)->ARR = 1000000 / LOOP_FREQUENCY_HZ;
}

void MX_TIM16_Init(void) { }

void MX_IWDG_Init(void)
{
    sitl_watchdog_enable();
}

void initCorePeripherals(void)
{
    sitl_timers_init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM6_Init();
    MX_TIM16_Init();
    MX_TIM17_Init();
    MX_COMP1_Init();
    MX_COMP2_Init();
}

void enableCorePeripherals(void)
{
    // default NVIC priorities matching the g431 target
    sitl_nvic_set_priority(SITL_IRQ_COMP, 0);
    sitl_nvic_set_priority(SITL_IRQ_COM, 0);
    sitl_nvic_set_priority(SITL_IRQ_TENKHZ, 2);
    sitl_nvic_set_priority(SITL_IRQ_DMA, 1);
    sitl_nvic_set_priority(SITL_IRQ_EXTI15, 2);
    sitl_nvic_set_priority(SITL_IRQ_CAN, 4);

    sitl_nvic_enable_irq(SITL_IRQ_COMP);
    sitl_nvic_enable_irq(SITL_IRQ_COM);
    sitl_nvic_enable_irq(SITL_IRQ_TENKHZ);
    sitl_nvic_enable_irq(SITL_IRQ_DMA);
    sitl_nvic_enable_irq(SITL_IRQ_EXTI15);
    sitl_nvic_enable_irq(SITL_IRQ_CAN);

    sitl_tenkhz_enable();
}

void setPWMCompare1(uint16_t compareone)
{
    sitl_tim1_set_duty(0, compareone);
}

void setPWMCompare2(uint16_t comparetwo)
{
    sitl_tim1_set_duty(1, comparetwo);
}

void setPWMCompare3(uint16_t comparethree)
{
    sitl_tim1_set_duty(2, comparethree);
}

void generatePwmTimerEvent(void)
{
    sitl_tim1_force_update();
}

void resetInputCaptureTimer(void)
{
    sitl_input_timer_reset();
}

void reloadWatchDogCounter(void)
{
    sitl_watchdog_reload();
}

void Error_Handler(void)
{
    fprintf(stderr, "SITL: Error_Handler called\n");
    exit(1);
}
