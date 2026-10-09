/*
  sitl_it.c - the SITL "vector table", dispatching emulated interrupts to
  the firmware handlers. Mirrors Mcu/g431/Src/stm32g4xx_it.c
 */

#include "sitl.h"
#include "targets.h"
#include "../sim/sitl_config.h"

extern void PeriodElapsedCallback(void);
extern void interruptRoutine(void);
extern void tenKhzRoutine(void);
extern void processDshot(void);

// provided by sys_can_SITL.c when DroneCAN is compiled in
void sitl_can_irq(void) __attribute__((weak));
void sitl_can_irq(void) { }

// CAN statistics for --verbose, overridden by sys_can_SITL.c
void sitl_can_stats(uint32_t stats[4]) __attribute__((weak));
void sitl_can_stats(uint32_t stats[4])
{
    stats[0] = stats[1] = stats[2] = stats[3] = 0;
}

extern volatile uint32_t commutation_interval;
extern volatile uint32_t average_interval;
extern volatile char rising;
extern uint8_t getCompOutputLevel(void);
extern uint32_t current_EXTI_LINE;

/*
  comparator EXTI interrupt with the same blanking gate as the g431
  handler: ignore zero crossings in the first half of the expected
  commutation interval
 */
extern void motor_log_event(int kind, uint32_t a, uint32_t b, uint32_t c);

static void comp_irq(void)
{
    const uint32_t lines = SITL_EXTI_LINE_21 | SITL_EXTI_LINE_22;
    const uint32_t line = current_EXTI_LINE;
    sitl_exti.PR &= ~(lines & ~line);
    if (!(sitl_exti.IMR & line)) {
        // a flag latched while the line was masked, serviced by a pend that
        // was already outstanding: not an observation (as the ports do)
        sitl_exti.PR &= ~lines;
        return;
    }
    const uint32_t cnt = sitl_interval_timer_count();
    // G431 gates on commutation_interval, the F051/G071 handlers on average_interval
    const uint32_t gate = sitl_cfg.sim.comparator_hold_pending ? (average_interval >> 1) : (commutation_interval >> 1);
    if (cnt > gate) {
        if (sitl_exti.PR & line) {
            sitl_exti.PR &= ~lines;
            motor_log_event(3 /*MEV_COMP_RUN*/, cnt, 0, 0);
            interruptRoutine();
        } else {
            // pend delivered with no pending line: the edge was consumed
            // by an earlier handler
            motor_log_event(3 /*MEV_COMP_RUN*/, cnt, 1, 0);
        }
    } else {
        motor_log_event(2 /*MEV_COMP_BLANKED*/, cnt, gate, 0);
        if (sitl_exti.PR & line) {
            if (sitl_cfg.sim.comparator_hold_pending && getCompOutputLevel() != rising) {
                // F051/G071 style: a pending edge with the post-crossing level
                // stays pending and the handler re-enters until the gate opens;
                // the re-entry is immediate, not a new entry-latency period
                sitl_irq_repend(SITL_IRQ_COMP);
            } else {
                sitl_exti.PR &= ~lines;
            }
        } else {
            sitl_exti.PR &= ~lines;
        }
    }
}

void sitl_irq_handler(int irq)
{
    switch (irq) {
    case SITL_IRQ_COMP:
        comp_irq();
        break;
    case SITL_IRQ_COM:
        PeriodElapsedCallback();
        break;
    case SITL_IRQ_TENKHZ:
        tenKhzRoutine();
        break;
    case SITL_IRQ_DMA:
        sitl_input_dma_irq();
        break;
    case SITL_IRQ_EXTI15:
        processDshot();
        break;
    case SITL_IRQ_CAN:
        sitl_can_irq();
        break;
    default:
        break;
    }
}
