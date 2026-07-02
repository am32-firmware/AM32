/*
 * peripherals_mcu.h — AT32F415 register macro sheet + private declarations.
 */

#ifndef PERIPHERALS_MCU_H_
#define PERIPHERALS_MCU_H_

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

void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void AT_COMP_Init(void);
void TIM1_Init(void);
void TIM4_Init(void);
void system_clock_config(void);
void TIM11_Init(void);
void TIM9_Init(void);
void TIM10_Init(void);
void reloadWatchDogCounter(void);
void UN_TIM_Init(void);
void LED_GPIO_init(void);

#endif /* PERIPHERALS_MCU_H_ */
