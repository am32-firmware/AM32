/*
 * peripherals_macros_stm32_ll.h
 *
 * Timer/watchdog register macro sheet shared by the six STM32 families
 * (f031, f051, g031, g071, g431, l431) — their sheets were byte-identical.
 * Included by Mcu/<mcu>/Inc/peripherals_mcu.h; the timer instances behind
 * COM_TIMER/INTERVAL_TIMER come from Inc/targets.h per target.
 */

#ifndef PERIPHERALS_MACROS_STM32_LL_H_
#define PERIPHERALS_MACROS_STM32_LL_H_

#define INTERVAL_TIMER_COUNT (INTERVAL_TIMER->CNT)
#define RELOAD_WATCHDOG_COUNTER() (LL_IWDG_ReloadCounter(IWDG))
#define DISABLE_COM_TIMER_INT() (COM_TIMER->DIER &= ~((0x1UL << (0U))))
#define ENABLE_COM_TIMER_INT() (COM_TIMER->DIER |= (0x1UL << (0U)))
#define SET_AND_ENABLE_COM_INT(time)                                  \
    (COM_TIMER->CNT = 0, COM_TIMER->ARR = time, COM_TIMER->SR = 0x00, \
        COM_TIMER->DIER |= (0x1UL << (0U)))
#define SET_INTERVAL_TIMER_COUNT(intertime) (INTERVAL_TIMER->CNT = intertime)
#define SET_PRESCALER_PWM(presc) (TIM1->PSC = presc)
#define SET_AUTO_RELOAD_PWM(relval) (TIM1->ARR = relval)
#define SET_DUTY_CYCLE_ALL(newdc) \
    (TIM1->CCR1 = newdc, TIM1->CCR2 = newdc, TIM1->CCR3 = newdc)

#endif /* PERIPHERALS_MACROS_STM32_LL_H_ */
