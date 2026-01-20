/*
 * timers.h
 *
 *  Created on: 14 Nov 2024
 *      Author: Youri
 */

#ifndef MCU_A153_INC_TIMERS_H_
#define MCU_A153_INC_TIMERS_H_

#include "main.h"

/*
 * @brief 	ternary operator result if interrupts are enabled
 */
#define RELOAD_WATCHDOG_COUNTER_ZERO { \
	__disable_irq(); \
	WWDT0->FEED = WWDT_FEED_FEED(0xaa); \
	WWDT0->FEED = WWDT_FEED_FEED(0x55); \
	__enable_irq();	\
}

/*
 * @brief 	ternary operator result if interrupts are disabled
 */
#define RELOAD_WATCHDOG_COUNTER_NON_ZERO { \
	WWDT0->FEED = WWDT_FEED_FEED(0xaa); \
	WWDT0->FEED = WWDT_FEED_FEED(0x55); \
}

/*
 * @brief 	Reloads the watchdog timer counter
 * 			__get_PRIMASK returns 0 if interrupts are enabled
 */
#define RELOAD_WATCHDOG_COUNTER() { \
	(__get_PRIMASK()) ? (RELOAD_WATCHDOG_COUNTER_NON_ZERO) : (RELOAD_WATCHDOG_COUNTER_ZERO); \
}

/*
 * @brief 	Resets timer counter, sets reload value, clears interrupt and enables the interrupt request
 * TODO I changed MSR to MR
 */
#define SET_AND_ENABLE_COM_INT(time) { \
	CTIMER1->MR[0] = time; \
	CTIMER1->TC = 0; \
	CTIMER1->IR = CTIMER_IR_MR0INT(1); \
	modifyReg32(&CTIMER1->MCR, CTIMER_MCR_MR0I_MASK, CTIMER_MCR_MR0I(1)); \
}

/*
 * @brief 	Enables COM_TIMER interrupt
 */
#define ENABLE_COM_TIMER_INT() { \
	modifyReg32(&CTIMER1->MCR, CTIMER_MCR_MR0I_MASK, CTIMER_MCR_MR0I(1)); \
}

/*
 * @brief 	Disables COM_TIMER interrupt
 */
#define DISABLE_COM_TIMER_INT() { \
	modifyReg32(&CTIMER1->MCR, CTIMER_MCR_MR0I_MASK, 0); \
}

/*
 * @brief 	Sets the interval timer count
 */
#define SET_INTERVAL_TIMER_COUNT(intertime) { \
	CTIMER2->TC = intertime; \
}

#define INTERVAL_TIMER_COUNT (CTIMER2->TC)

void MX_IWDG_Init(void);
void initDshotPWMTimer(void);
void initComTimer(void);
void initIntervalTimer(void);
void initSystickTimer(void);
void initDelayTimer(void);
void initTenKHzTimer(void);

void enableDshotPWMTimer(void);
void enableComTimer(void);
void enableIntervalTimer(void);
void enableSystickTimer(void);
void enableTenKHzTimer(void);

void resetInputCaptureTimer(void);

#endif /* MCU_A153_INC_TIMERS_H_ */
