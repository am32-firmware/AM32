#include "stm32h563xx.h"
#include "targets.h"
#include "comparator.h"

int main()
{
    comparator_gpio_initialize();
    comparator_exti_initialize();
    enableCompInterrupts();
    while(1) {

    }
}

void EXTI13_IRQHandler() {
    EXTI->FPR1 |= EXTI_FPR1_FPIF13;
    EXTI->RPR1 |= EXTI_RPR1_RPIF13;
}