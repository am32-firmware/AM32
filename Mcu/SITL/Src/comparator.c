/*
  comparator.c - SITL BEMF comparator emulation. Mirrors the g431 logic:
  the floating phase is compared against the virtual neutral point, EXTI
  edge selection follows `rising` (rising BEMF arms a falling edge on the
  comparator output)
 */

#include "comparator.h"

#include "common.h"
#include "sitl.h"
#include "targets.h"

sitl_exti_t sitl_exti;

volatile uint8_t sitl_comp_phase = 2;
volatile uint8_t sitl_comp_out;

COMP_TypeDef* active_COMP = &sitl_comp_dummy[1];
uint32_t current_EXTI_LINE = SITL_EXTI_LINE_22;

uint8_t getCompOutputLevel(void)
{
    if (sitl_in_sim_thread()) {
        // called from interrupt context (filter loop in interruptRoutine):
        // account for the read time so consecutive reads see fresh samples
        sitl_isr_read_tick();
    } else {
        // mainline polling (old_routine commutation) is progress
        sitl_fw_progress();
    }
    return sitl_comp_out;
}

void maskPhaseInterrupts(void)
{
    sitl_exti.IMR &= ~(SITL_EXTI_LINE_21 | SITL_EXTI_LINE_22);
    sitl_exti.PR &= ~(SITL_EXTI_LINE_21 | SITL_EXTI_LINE_22);
}

void enableCompInterrupts(void)
{
    sitl_exti.IMR |= current_EXTI_LINE;
    // as on the STM32: an edge that latched PR while masked fires as
    // soon as the interrupt is unmasked
    if (sitl_exti.PR & current_EXTI_LINE) {
        sitl_irq_pend(SITL_IRQ_COMP);
    }
}

void changeCompInput(void)
{
    if (step == 1 || step == 4) { // c floating
        sitl_comp_phase = 2;
        current_EXTI_LINE = SITL_EXTI_LINE_22;
        active_COMP = &sitl_comp_dummy[1];
    }
    if (step == 2 || step == 5) { // a floating
        sitl_comp_phase = 0;
        current_EXTI_LINE = SITL_EXTI_LINE_21;
        active_COMP = &sitl_comp_dummy[0];
    }
    if (step == 3 || step == 6) { // b floating
        sitl_comp_phase = 1;
        current_EXTI_LINE = SITL_EXTI_LINE_22;
        active_COMP = &sitl_comp_dummy[1];
    }
    if (rising) {
        sitl_exti.RTSR &= ~(SITL_EXTI_LINE_21 | SITL_EXTI_LINE_22);
        sitl_exti.FTSR |= current_EXTI_LINE;
    } else { // falling bemf
        sitl_exti.RTSR |= current_EXTI_LINE;
        sitl_exti.FTSR &= ~(SITL_EXTI_LINE_21 | SITL_EXTI_LINE_22);
    }
}
