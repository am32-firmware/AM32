/*
 * comparator.h
 *
 *  Created on: Aug. 4, 2026
 *      Author: Nong jun
 */
 
#ifndef COMPARATOR_H_
#define COMPARATOR_H_

#include "main.h"

void changeCompInput(void);
void maskPhaseInterrupts();
void enableCompInterrupts(void);
uint8_t getCompOutputLevel(void);
extern volatile char rising;
extern char step;
extern COMP1_TypeDef* currentComp;

#endif /* COMPARATOR_H_ */
