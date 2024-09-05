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
void comparator_gpio_initialize();
void comparator_exti_initialize();

