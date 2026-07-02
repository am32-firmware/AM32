/*
 * comparator.h
 *
 *  Created on: May 9, 2020
 *      Author: Alka
 */

#ifndef COMPARATOR_H_
#define COMPARATOR_H_

#include "main.h"

void maskPhaseInterrupts();
void changeCompInput();
void enableCompInterrupts();
uint8_t getCompOutputLevel();

#ifdef MCU_A153
void initComp0(void);
void initComp1(void);
void enableComparator(void);
void disableComparators(void);
void changeMainComp(LPCMP_Type* CMPx);
#endif

extern volatile char rising;
extern char step;

#endif /* COMPARATOR_H_ */
