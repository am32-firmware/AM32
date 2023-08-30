/*
 * peripherals.h
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 */

#ifndef PERIPHERALS_H_
#define PERIPHERALS_H_

#include "main.h"

void initAfterJump(void);
void initCorePeripherals(void);
//void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_DMA_Init(void);
//static void MX_ADC_Init(void);
void AT_COMP_Init(void);
void TIM1_Init(void);
void TIM4_Init(void);
void system_clock_config(void);
void MX_IWDG_Init(void);
void TIM11_Init(void);
void TIM9_Init(void);
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
void UN_TIM_Init(void);
void setPrescalerPWM(uint16_t presc);
void LED_GPIO_init(void);

#endif /* PERIPHERALS_H_ */
