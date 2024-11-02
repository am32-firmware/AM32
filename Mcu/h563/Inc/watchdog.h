// independent watchdog (IWDG) driver

#pragma once

#include <inttypes.h>

typedef enum
{
    IWDG_PRESCALER_4,
    IWDG_PRESCALER_8,
    IWDG_PRESCALER_16,
    IWDG_PRESCALER_32,
    IWDG_PRESCALER_64,
    IWDG_PRESCALER_128,
    IWDG_PRESCALER_256,
    IWDG_PRESCALER_512,
    IWDG_PRESCALER_1024,
} iwdgPrescaler_e;

void watchdog_initialize(
    iwdgPrescaler_e prescaler,
    uint16_t reload);

void watchdog_initialize_period(uint32_t period_us);
void watchdog_enable();
void watchdog_goodboy_pat();
void watchdog_set_prescaler(iwdgPrescaler_e prescaler);
void watchdog_set_reload(uint32_t reload);
