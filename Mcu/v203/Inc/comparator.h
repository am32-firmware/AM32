/*
 * comparator.h
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 *      Modified by TempersLee June 21, 2024
 */

#ifndef COMPARATOR_H_
#define COMPARATOR_H_
#endif /* COMPARATOR_H_ */

#include "main.h"

void maskPhaseInterrupts();
void changeCompInput();
void enableCompInterrupts();
uint8_t getCompOutputLevel();

extern char rising;
extern char step;
