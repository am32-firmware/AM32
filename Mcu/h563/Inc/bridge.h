#pragma once

#include "gpio.h"
#include "commutation.h"

#define OCM_PWM1 0b0110
#define OCM_INACTIVE 0b0100
#define OCM_ACTIVE 0b0101

void bridge_initialize();
void bridge_gpio_initialize();

typedef struct
{
    uint32_t ccmr1;
    uint32_t ccmr2;
    uint32_t ccer;
} timComStepChannelConfig;

extern commutationStep_e bridgeComStep;
extern timComStepChannelConfig comSteps[];

void bridge_set_deadtime_ns(uint32_t deadtime);
void bridge_commutate(void);
void bridge_set_com_step(uint8_t step);
void bridge_set_audio_duty(uint8_t duty);
void bridge_set_run_duty(uint16_t duty);
void bridge_set_mode_audio();
void bridge_set_mode_run();
void bridge_set_commutation_step(commutationStep_e step);
void bridge_set_audio_frequency(uint16_t f);
void bridge_set_run_frequency(uint32_t f);
void bridge_enable();
void bridge_disable();
void bridge_play_note(uint32_t f, uint8_t v);
