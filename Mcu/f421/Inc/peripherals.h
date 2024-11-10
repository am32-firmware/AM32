/*
 * peripherals.h
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 */

#ifndef PERIPHERALS_H_
#define PERIPHERALS_H_

#endif /* PERIPHERALS_H_ */

#include "main.h"
#define INTERVAL_TIMER_COUNT (INTERVAL_TIMER->cval)
#define RELOAD_WATCHDOG_COUNTER() (WDT->cmd = WDT_CMD_RELOAD)
#define DISABLE_COM_TIMER_INT() (COM_TIMER->iden &= ~TMR_OVF_INT)
#define ENABLE_COM_TIMER_INT() (COM_TIMER->iden |= TMR_OVF_INT)
#define SET_AND_ENABLE_COM_INT(time)                                    \
    (COM_TIMER->cval = 0, COM_TIMER->pr = time, COM_TIMER->ists = 0x00, \
        COM_TIMER->iden |= TMR_OVF_INT)
#define SET_INTERVAL_TIMER_COUNT(intertime) (INTERVAL_TIMER->cval = intertime)
#define SET_PRESCALER_PWM(presc) (TMR1->div = presc)
#define SET_AUTO_RELOAD_PWM(relval) (TMR1->pr = relval)
#define SET_DUTY_CYCLE_ALL(newdc) \
    (TMR1->c1dt = newdc, TMR1->c2dt = newdc, TMR1->c3dt = newdc)

void initAfterJump(void);
void initCorePeripherals(void);
// void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_DMA_Init(void);
// static void MX_ADC_Init(void);
void AT_COMP_Init(void);
void TIM1_Init(void);
void TIM6_Init(void);
void system_clock_config(void);
void MX_IWDG_Init(void);
void TIM17_Init(void);
void TIM14_Init(void);
void TIM16_Init(void);
// static void MX_USART1_UART_Init(void);
void resetInputCaptureTimer();
void setPWMCompare1(uint16_t compareone);
void setPWMCompare2(uint16_t comparetwo);
void setPWMCompare3(uint16_t comparethree);
void enableCorePeripherals(void);
void reloadWatchDogCounter(void);
void generatePwmTimerEvent(void);
void UN_TIM_Init(void);
void LED_GPIO_init(void);
