/*
  sitl_nvic.h - CMSIS style interrupt controls mapped onto the SITL runtime
 */

#pragma once

#include "sitl.h"

typedef int IRQn_Type;

static inline void NVIC_SetPriority(IRQn_Type irq, uint32_t prio)
{
    sitl_nvic_set_priority(irq, prio);
}

static inline void NVIC_EnableIRQ(IRQn_Type irq)
{
    sitl_nvic_enable_irq(irq);
}

static inline void NVIC_DisableIRQ(IRQn_Type irq)
{
    sitl_nvic_disable_irq(irq);
}

#define NVIC_SystemReset() sitl_system_reset()

static inline void __disable_irq(void)
{
    sitl_primask_set();
}

static inline void __enable_irq(void)
{
    sitl_primask_clear();
}

static inline uint32_t __get_PRIMASK(void)
{
    return sitl_primask_get();
}
