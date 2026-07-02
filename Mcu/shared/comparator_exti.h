/*
 * comparator_exti.h
 *
 * Shared comparator/EXTI implementation for the GD32E230 and AT32F415
 * ports, which are structurally identical and differ only in register
 * spelling. #included at the end of Mcu/{e230,f415}/Src/comparator.c
 * after the family defines its register macros. Not a standalone
 * translation unit.
 *
 * Required lvalue/expression macros:
 *   COMPARATOR_READ()    comparator output level (0/1)
 *   COMP_EXTI_INTEN      EXTI interrupt-enable register
 *   COMP_EXTI_PENDING    EXTI pending register
 *   COMP_MUX_SELECT      comparator input-select register
 *   COMP_EXTI_RISING     EXTI rising-edge-enable register
 *   COMP_EXTI_FALLING    EXTI falling-edge-enable register
 */

uint8_t getCompOutputLevel() { return COMPARATOR_READ(); }

void maskPhaseInterrupts()
{
    COMP_EXTI_INTEN &= ~(uint32_t)EXTI_LINE;
    COMP_EXTI_PENDING = (uint32_t)EXTI_LINE;
}

void enableCompInterrupts()
{
    COMP_EXTI_INTEN |= (uint32_t)EXTI_LINE;
}

void changeCompInput()
{
    if (step == 1 || step == 4) { // c floating
        COMP_MUX_SELECT = PHASE_C_COMP;
    }
    if (step == 2 || step == 5) { // a floating
        COMP_MUX_SELECT = PHASE_A_COMP;
    }
    if (step == 3 || step == 6) { // b floating
        COMP_MUX_SELECT = PHASE_B_COMP;
    }
    if (rising) {
        COMP_EXTI_RISING &= ~(uint32_t)EXTI_LINE;
        COMP_EXTI_FALLING |= (uint32_t)EXTI_LINE;
    } else {
        // falling bemf
        COMP_EXTI_RISING |= (uint32_t)EXTI_LINE;
        COMP_EXTI_FALLING &= ~(uint32_t)EXTI_LINE;
    }
}
