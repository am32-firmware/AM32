/*
 * g32f031_int.c
 *
 *  Created on: Aug. 4, 2026
 *      Author: Nong jun
 */

#ifndef G32F031_INT_H
#define G32F031_INT_H

#include "main.h"

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);

#endif /* G32F031_INT_H */

