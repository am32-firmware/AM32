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

void comparator_initialize(comparator_t* comp);
void comparator_initialize_gpio(gpio_t* gpio);
void comparator_initialize_exti(gpio_t* gpio);

void comparator_initialize_gpio_exti(gpio_t* gpio);
void comparator_gpio_exti_nvic_enable(gpio_t* gpio);
void comparator_gpio_exti_nvic_disable(gpio_t* gpio);
void comparator_nvic_set_priority(comparator_t* comp, uint32_t priority);

void comparator_configure_callbacks(comparator_t* comp);
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

