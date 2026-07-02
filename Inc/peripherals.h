/*
 * peripherals.h
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 */

#ifndef PERIPHERALS_H_
#define PERIPHERALS_H_

#include "main.h"
// per-MCU register macro sheet + family-private init declarations,
// resolved from Mcu/<mcu>/Inc (per-MCU include path precedes root Inc)
#include "peripherals_mcu.h"

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
