/*
 * peripherals.h
 *
 *  Created on: 14 Nov 2024
 *      Author: Youri
 */

#ifndef PERIPHERALS_H_
#define PERIPHERALS_H_

#include "main.h"

void initAfterJump(void);
void initCorePeripherals(void);
void SystemInit(void);
void SystemClock_Config(void);
void initGPIO(void);
void initSPI(void);
void enableCorePeripherals(void);

#endif /* PERIPHERALS_H_ */
