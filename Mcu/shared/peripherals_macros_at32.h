/*
 * peripherals_macros_at32.h
 *
 * Timer/watchdog register macro sheet shared by the AT32 families
 * (f415, f421) — their sheets were byte-identical.
 * Included by Mcu/<mcu>/Inc/peripherals_mcu.h.
 */

#ifndef PERIPHERALS_MACROS_AT32_H_
#define PERIPHERALS_MACROS_AT32_H_

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

#endif /* PERIPHERALS_MACROS_AT32_H_ */
