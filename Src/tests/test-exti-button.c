#include "stm32h563xx.h"
#include "targets.h"
#include "gpio.h"
#include "exti.h"

gpio_t gpioButton = DEF_GPIO(GPIOC, 13, 0, GPIO_INPUT);

int main()
{
    gpio_initialize(&gpioButton);

    // comparator_gpio_initialize();
    // comparator_exti_initialize();
    // enableCompInterrupts();

    exti_configure_port(&extiChannels[gpioButton.pin], EXTI_CHANNEL_FROM_PORT(gpioButton.port));
    while(1) {

    }
}

void EXTI13_IRQHandler() {
    EXTI->FPR1 |= EXTI_FPR1_FPIF13;
    EXTI->RPR1 |= EXTI_RPR1_RPIF13;
}