#pragma once

#include "stm32h563xx.h"
#include "gpio.h"
#include "exti.h"

typedef struct {
    gpio_t* phaseA;
    gpio_t* phaseB;
    gpio_t* phaseC;
    extiCallback_p phaseAcb;
    extiCallback_p phaseBcb;
    extiCallback_p phaseCcb;
} comparator_t;

extern comparator_t COMPARATOR;
extern comparator_t comparatorChannels[];

void comparator_initialize(comparator_t* comp);
void comparator_initialize_gpio(gpio_t* gpio);
void comparator_initialize_exti(gpio_t* gpio);

void comparator_initialize_gpio_exti(gpio_t* gpio);
void comparator_gpio_exti_nvic_enable(gpio_t* gpio);

void comparator_disable_interrupts(comparator_t* comp);
void comparator_enable_interrupts(comparator_t* comp);

void maskPhaseInterrupts();
void changeCompInput();
void enableCompInterrupts();
uint8_t getCompOutputLevel();

extern char rising;
extern char step;
void comparator_gpio_initialize();
void comparator_exti_initialize();

