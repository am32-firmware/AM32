/*
 * comparator.h
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 */

#ifndef COMPARATOR_H_
#define COMPARATOR_H_
#endif /* COMPARATOR_H_ */

// Medium speed comparator
//#define COMP_PA0 0b1100101
//#define COMP_PA4 0b1000101
//#define COMP_PA5 0b1010101

// High speed comparator
// #define COMP_PA0 0b1100001
// #define COMP_PA4 0b1000001
// #define COMP_PA5 0b1010001

#include "main.h"

void maskPhaseInterrupts();
void changeCompInput();
void enableCompInterrupts();

extern COMP_TypeDef* active_COMP;

// read the comparator output level inline, this is called up to
// filter_level times per zero cross so call overhead matters
static inline uint8_t getCompOutputLevel(void)
{
    return LL_COMP_ReadOutputLevel(active_COMP);
}

extern volatile char rising;
extern char step;
