#pragma once

#include "stm32h563xx.h"
#include "gpio.h"

typedef struct {
    // circular buffers
    // buffer size MUST be 256
    // implementation takes advantage of integer overflow
    // uint8_t* _rx_buffer;
    // uint8_t* _tx_buffer;
    // uint16_t _rx_buffer_size;
    // uint16_t _tx_buffer_size;
    // uint8_t _rx_head;
    // uint8_t _tx_head;
    // uint8_t _tx_tail;
    // uint8_t _dma_transfer_count;

    // dmaChannel_t* rxDma;
    // dmaChannel_t* txDma;
    gpio_t* phaseA;
    gpio_t* phaseB;
    gpio_t* phaseC;

    uint8_t _irqn;
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

