/*
 * peripherals.h
 *
 *  Created on: Aug. 4, 2026
 *      Author: Nong jun
 */

#ifndef PERIPHERALS_H_
#define PERIPHERALS_H_

#include "ADC.h"
#include "main.h"

#define INTERVAL_TIMER                          UTILITY_TIMER
#define INTERVAL_TIMER_COUNT                    (INTERVAL_TIMER->CNT<<1)
#define SET_INTERVAL_TIMER_COUNT(time)          (INTERVAL_TIMER->CNT=time)

#define SET_PRESCALER_PWM(presc)                (ATMR->PSC = presc)
#define SET_AUTO_RELOAD_PWM(relval)             (ATMR->AUTORLD = relval)
#define SET_DUTY_CYCLE_ALL(newdc)               (ATMR->CC0 = newdc,ATMR->CC1 = newdc,ATMR->CC2 = newdc)
#define DISABLE_COM_TIMER_INT()                 (COM_TIMER->IER &= ~BTMR_IER_UIEN)
#define SET_AND_ENABLE_COM_INT(time)            (COM_TIMER->AUTORLD = (time)>>1,COM_TIMER->CEG |= BTMR_CEG_UEG,COM_TIMER->IER |= BTMR_IER_UIEN,COM_TIMER->SR = 0X00)

void MX_IWDG_Init(void);
void resetInputCaptureTimer();
void setPWMCompare1(uint16_t compareone);
void setPWMCompare2(uint16_t comparetwo);
void setPWMCompare3(uint16_t comparethree);
void pwm_compare_config(uint16_t u16_compare);
void initAfterJump(void);
void initCorePeripherals(void);
void enableCorePeripherals(void);
void reloadWatchDogCounter(void);
void generatePwmTimerEvent(void);

void gpio_test_set(void);
void gpio_test_reset(void);
void gpio_test_inv(void);
void RELOAD_WATCHDOG_COUNTER(void);

#endif /* PERIPHERALS_H_ */
