// #include "stm32h563xx.h"

#include "targets.h"
#include "comparator.h"
// #include "led.h"


gpio_t gpioCompPhaseA = DEF_GPIO(COMPA_GPIO_PORT, COMPA_GPIO_PIN, 0, GPIO_INPUT);
gpio_t gpioCompPhaseB = DEF_GPIO(COMPB_GPIO_PORT, COMPB_GPIO_PIN, 0, GPIO_INPUT);
gpio_t gpioCompPhaseC = DEF_GPIO(COMPC_GPIO_PORT, COMPC_GPIO_PIN, 0, GPIO_INPUT);

static comparator_t comp = {
    .phaseA = &gpioCompPhaseA
    .phaseB = &gpioCompPhaseB
    .phaseC = &gpioCompPhaseC
};


int main()
{
    comparator_initialize(&comp);

    // comparator_gpio_initialize();
    // comparator_exti_initialize();

    // enableCompInterrupts();

    while(1) {

    }
}

// void compAcb() {

// }


// void EXTI13_IRQHandler() {
//     // EXTI->
//     // FPR1 |= EXTI_FPR1_FPIF13;
//     // EXTI->RPR1 |= EXTI_RPR1_RPIF13;
// }