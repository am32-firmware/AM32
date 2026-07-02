/*
 * peripherals_mcu.h — STM32F031 register macro sheet + private declarations.
 */

#ifndef PERIPHERALS_MCU_H_
#define PERIPHERALS_MCU_H_

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

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void MX_TIM14_Init(void);
void MX_TIM16_Init(void);
void MX_TIM1_Init(void);
void MX_TIM3_Init(void);
void MX_TIM17_Init(void);
void MX_TIM2_Init(void);
void TEN_KHZ_Timer_Init(void);
void reloadWatchDogCounter(void);
void UN_TIM_Init(void);
void UN_GPIO_Init(void);
void LED_GPIO_init(void);

#endif /* PERIPHERALS_MCU_H_ */
