#pragma once

#include "main.h"
#include "commutation-timer.h"
// #include "input-timer.h"

#define INTERVAL_TIMER_COUNT (INTERVAL_TIMER->CNT)
#define RELOAD_WATCHDOG_COUNTER() (LL_IWDG_ReloadCounter(IWDG))

#define SET_INTERVAL_TIMER_COUNT(intertime) (INTERVAL_TIMER->CNT = intertime)
#define SET_PRESCALER_PWM(presc) (BRIDGE_TIMER->PSC = presc)
#define SET_AUTO_RELOAD_PWM(relval) (BRIDGE_TIMER->ARR = relval)
#define SET_DUTY_CYCLE_ALL(newdc) \
    (BRIDGE_TIMER->CCR1 = newdc, BRIDGE_TIMER->CCR2 = newdc, BRIDGE_TIMER->CCR3 = newdc)

void initAfterJump(void);
void initCorePeripherals(void);
void SystemClock_Config(void);
// static void MX_ADC_Init(void);
void interval_timer_initialize(void);
void interval_timer_enable(void);

// called from main (can't be renamed)
void MX_IWDG_Init(void);
void reloadWatchDogCounter(void);

void setAutoReloadPWM(uint16_t relval);
void setDutyCycleAll(uint16_t newdc);
void setPWMCompare1(uint16_t compareone);
void setPWMCompare2(uint16_t comparetwo);
void setPWMCompare3(uint16_t comparethree);
void enableCorePeripherals(void);
void generatePwmTimerEvent(void);
void setPrescalerPWM(uint16_t presc);
