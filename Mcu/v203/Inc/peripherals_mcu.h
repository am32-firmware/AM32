/*
 * peripherals_mcu.h — CH32V203 register macro sheet + private declarations.
 */

#ifndef PERIPHERALS_MCU_H_
#define PERIPHERALS_MCU_H_

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

void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void AT_COMP_Init(void);
void TIM1_Init(void);
void Intervak_TIM4_Init(void);
void system_clock_config(void);
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
void reloadWatchDogCounter(void);
void UN_TIM2_Init(void);
void setPrescalerPWM(uint16_t presc);
void LED_GPIO_init(void);

#endif /* PERIPHERALS_MCU_H_ */
