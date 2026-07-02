/*
 * peripherals_macros_ch32.h
 *
 * Timer/watchdog register macro sheet for the CH32V203 port.
 * Selected by Inc/peripherals.h via the WCH vendor macro.
 */

#ifndef PERIPHERALS_MACROS_CH32_H_
#define PERIPHERALS_MACROS_CH32_H_

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

#endif /* PERIPHERALS_MACROS_CH32_H_ */
