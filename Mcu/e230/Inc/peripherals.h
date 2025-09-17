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

#define INTERVAL_TIMER_COUNT (TIMER_CNT(INTERVAL_TIMER))
#define RELOAD_WATCHDOG_COUNTER() (fwdgt_counter_reload())
#define DISABLE_COM_TIMER_INT() \
    (TIMER_DMAINTEN(COM_TIMER) &= (~(uint32_t)TIMER_INT_UP))
#define ENABLE_COM_TIMER_INT() \
    (TIMER_DMAINTEN(COM_TIMER) |= (uint32_t)TIMER_INT_UP)
#define SET_AND_ENABLE_COM_INT(time)                        \
    (TIMER_CNT(COM_TIMER) = 0, TIMER_CAR(COM_TIMER) = time, \
        TIMER_INTF(COM_TIMER) = 0x00,                       \
        TIMER_DMAINTEN(COM_TIMER) |= (uint32_t)TIMER_INT_UP)
#define SET_INTERVAL_TIMER_COUNT(intertime) \
    (TIMER_CNT(INTERVAL_TIMER) = intertime)
#define SET_PRESCALER_PWM(presc) (TIMER_PSC(TIMER0) = presc)
#define SET_AUTO_RELOAD_PWM(relval) (TIMER_CAR(TIMER0) = relval)
#define SET_DUTY_CYCLE_ALL(newdc)                              \
    (TIMER_CH0CV(TIMER0) = newdc, TIMER_CH1CV(TIMER0) = newdc, \
        TIMER_CH2CV(TIMER0) = newdc)

void initAfterJump(void);
void initCorePeripherals(void);
// void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_DMA_Init(void);
// static void MX_ADC_Init(void);
void COMP_Init(void);
void TIM0_Init(void);
void TIMER2_Init(void);
void TIMER5_Init(void);
void MX_IWDG_Init(void);
void TIMER15_Init(void);
void TIMER16_Init(void);
void TIMER13_Init(void);
// static void MX_USART1_UART_Init(void);
void setIndividualRGBLed(uint8_t red, uint8_t green, uint8_t blue);
void resetInputCaptureTimer();
void setPWMCompare1(uint16_t compareone);
void setPWMCompare2(uint16_t comparetwo);
void setPWMCompare3(uint16_t comparethree);
void enableCorePeripherals(void);
void reloadWatchDogCounter(void);
void generatePwmTimerEvent(void);
void UN_TIM_Init(void);
void LED_GPIO_init(void);
