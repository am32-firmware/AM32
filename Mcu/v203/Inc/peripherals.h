/*
 * peripherals.h
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 *      Modified by TempersLee June,21 2024
 */

#ifndef PERIPHERALS_H_
#define PERIPHERALS_H_

#endif /* PERIPHERALS_H_ */

#include "main.h"

#define INTERVAL_TIMER_COUNT (INTERVAL_TIMER->CNT)
#define RELOAD_WATCHDOG_COUNTER() ( IWDG->CTLR = 0xAAAA)
#define DISABLE_COM_TIMER_INT() (COM_TIMER->DMAINTENR &= ~TIM_IT_Update)
#define ENABLE_COM_TIMER_INT() (COM_TIMER->DMAINTENR |= TIM_IT_Update)
#define SET_AND_ENABLE_COM_INT(time)                                    \
    (COM_TIMER->CNT = 0, COM_TIMER->ATRLR = time, COM_TIMER->INTFR = 0x00, \
        COM_TIMER->DMAINTENR |= TIM_IT_Update)
#define SET_INTERVAL_TIMER_COUNT(intertime) (INTERVAL_TIMER->CNT = intertime)
#define SET_PRESCALER_PWM(presc) (TIM1->PSC = presc)
#define SET_AUTO_RELOAD_PWM(relval) (TIM1->ATRLR = relval)
#define SET_DUTY_CYCLE_ALL(newdc) \
    (TIM1->CH1CVR = newdc, TIM1->CH2CVR = newdc, TIM1->CH3CVR = newdc)

void initAfterJump(void);
void initCorePeripherals(void);
// void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_DMA_Init(void);
// static void MX_ADC_Init(void);
void AT_COMP_Init(void);
void TIM1_Init(void);
void Intervak_TIM4_Init(void);
void system_clock_config(void);
void MX_IWDG_Init(void);
void COM_TIM3_Init(void);
void TenkHz_SysTick_Init(void);
void TIM10_Init(void);
void disableComTimerInt(void);
void enableComTimerInt(void);
void setAndEnableComInt(uint16_t time);
uint16_t getintervaTimerCount();
void setintervaTimerCount(uint16_t intertime);
void setAutoReloadPWM(uint16_t relval);
void setDutyCycleAll(uint16_t newdc);
void resetInputCaptureTimer();
void setPWMCompare1(uint16_t compareone);
void setPWMCompare2(uint16_t comparetwo);
void setPWMCompare3(uint16_t comparethree);
void enableCorePeripherals(void);
void reloadWatchDogCounter(void);
void generatePwmTimerEvent(void);
void UN_TIM2_Init(void);
void setPrescalerPWM(uint16_t presc);
void LED_GPIO_init(void);
