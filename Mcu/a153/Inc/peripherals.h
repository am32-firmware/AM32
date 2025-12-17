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
void SystemInit(void);
void SystemClock_Config(void);
void initGPIO(void);
void initSPI(void);
void enableCorePeripherals(void);

#endif /* PERIPHERALS_H_ */
