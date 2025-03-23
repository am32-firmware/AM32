/*
 * comparator.h
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 */

#ifndef COMPARATOR_H_
#define COMPARATOR_H_
#endif /* COMPARATOR_H_ */

#include "main.h"

//#define COMP_PA0 0b1100001
//#define COMP_PA4 0b1000001
//#define COMP_PA5 0b1010001
#define COMP_PB7 0b1100001
#define COMP_PB1 0b1000001
#define COMP_PC14 0b1010001

void maskPhaseInterrupts();
void changeCompInput();
void enableCompInterrupts();
//uint8_t getCompOutputLevel();
extern char rising;
extern char step;
