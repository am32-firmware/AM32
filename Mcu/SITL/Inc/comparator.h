/*
 * comparator.h - SITL
 */

#ifndef COMPARATOR_H_
#define COMPARATOR_H_

#include "main.h"

uint8_t getCompOutputLevel(void);
void maskPhaseInterrupts(void);
void enableCompInterrupts(void);
void changeCompInput(void);

extern volatile char rising;
extern char step;

#endif /* COMPARATOR_H_ */
