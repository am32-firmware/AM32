#pragma once
#include <stdbool.h>
#include <stdint.h>
extern uint32_t timer_count, timer_delay, irq_mask;
extern unsigned timer_reads, bridge_off, comparator_masks, edge_on_read;
extern bool timer_enabled, edge_pending, comparator_pre;
extern uint16_t TIMER1_MAX_ARR;
static inline uint32_t test_count(void)
{
    if (++timer_reads == edge_on_read) {
        edge_pending = true;
    }
    return timer_count;
}
static inline uint32_t __get_PRIMASK(void) { return irq_mask; }
static inline void __disable_irq(void) { irq_mask = 1; }
static inline void __enable_irq(void) { irq_mask = 0; }
