/*
 * comparator.h
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 */

#ifndef COMPARATOR_H_
#define COMPARATOR_H_
#endif /* COMPARATOR_H_ */

#define COMP_PA0 0b1100101
#define COMP_PA4 0b1000101
#define COMP_PA5 0b1010101

#include "main.h"

void maskPhaseInterrupts();
void changeCompInput();
void enableCompInterrupts();
uint8_t getCompOutputLevel();

extern char rising;
extern char step;
