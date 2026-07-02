/*
 * peripherals.h
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 */

#ifndef PERIPHERALS_H_
#define PERIPHERALS_H_

#include "main.h"
#include "targets.h"

// timer/watchdog register macro sheet, selected by the vendor macro the
// active target defines in targets.h; the NXP port (a153) keeps its
// equivalents in timers.h/flexpwm.h, reached via its main.h
#if defined(STMICRO)
#include "../Mcu/shared/peripherals_macros_stm32_ll.h"
#elif defined(ARTERY)
#include "../Mcu/shared/peripherals_macros_at32.h"
#elif defined(GIGADEVICES)
#include "../Mcu/shared/peripherals_macros_gd32.h"
#elif defined(WCH)
#include "../Mcu/shared/peripherals_macros_ch32.h"
#endif

void initAfterJump(void);
void initCorePeripherals(void);
void enableCorePeripherals(void);
void MX_IWDG_Init(void);
void setPWMCompare1(uint16_t compareone);
void setPWMCompare2(uint16_t comparetwo);
void setPWMCompare3(uint16_t comparethree);
void resetInputCaptureTimer();
void generatePwmTimerEvent(void);
void setIndividualRGBLed(uint8_t red, uint8_t green, uint8_t blue);

#endif /* PERIPHERALS_H_ */
