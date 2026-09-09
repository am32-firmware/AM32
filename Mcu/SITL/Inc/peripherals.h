/*
  peripherals.h for the SITL MCU. Same macro API as the g431 version with
  the register pokes routed into the emulation
 */

#ifndef PERIPHERALS_H_
#define PERIPHERALS_H_
#endif /* PERIPHERALS_H_ */

#include "ADC.h"
#include "main.h"

#define INTERVAL_TIMER_COUNT (sitl_interval_timer_count())
#define RELOAD_WATCHDOG_COUNTER() sitl_watchdog_reload()
#define DISABLE_COM_TIMER_INT() sitl_com_int_disable()
#define ENABLE_COM_TIMER_INT() sitl_com_int_enable()
#define SET_AND_ENABLE_COM_INT(time) sitl_com_int_arm(time)
#define SET_INTERVAL_TIMER_COUNT(intertime) sitl_interval_timer_set(intertime)
#define SET_PRESCALER_PWM(presc) sitl_tim1_set_psc(presc)
#define SET_AUTO_RELOAD_PWM(relval) sitl_tim1_set_arr(relval)
#define SET_DUTY_CYCLE_ALL(newdc) sitl_tim1_set_duty_all(newdc)

void initAfterJump(void);
void initCorePeripherals(void);
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void MX_ADC1_Init(void);
void MX_COMP2_Init(void);
void MX_COMP1_Init(void);
void MX_TIM1_Init(void);
void MX_TIM2_Init(void);
void MX_TIM3_Init(void);
void MX_TIM14_Init(void);
void MX_TIM17_Init(void);
void MX_TIM16_Init(void);
void MX_IWDG_Init(void);
void MX_TIM6_Init(void);
void MX_TIM15_Init(void);
void resetInputCaptureTimer(void);
void setPWMCompare1(uint16_t compareone);
void setPWMCompare2(uint16_t comparetwo);
void setPWMCompare3(uint16_t comparethree);
void enableCorePeripherals(void);
void reloadWatchDogCounter(void);
void generatePwmTimerEvent(void);
