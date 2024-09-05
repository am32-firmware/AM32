#include "stm32h563xx.h"
#define COMPA_GPIO_PORT GPIOF
#define COMPA_GPIO_PIN 4

#define COMPB_GPIO_PORT GPIOC
#define COMPB_GPIO_PIN 15

#define COMPC_GPIO_PORT GPIOC
#define COMPC_GPIO_PIN 14

#include "comparator.h"
#include "stm32h5xx_ll_gpio.h"
#include "targets.h"
#include "gpio.h"


void comparator_gpio_initialize()
{
    gpio_t gpioCompPhaseA = DEF_GPIO(COMPA_GPIO_PORT, COMPA_GPIO_PIN, 0, GPIO_INPUT);
    gpio_t gpioCompPhaseB = DEF_GPIO(COMPB_GPIO_PORT, COMPB_GPIO_PIN, 0, GPIO_INPUT);
    gpio_t gpioCompPhaseC = DEF_GPIO(COMPC_GPIO_PORT, COMPC_GPIO_PIN, 0, GPIO_INPUT);
    gpio_initialize(&gpioCompPhaseA);
    gpio_initialize(&gpioCompPhaseB);
    gpio_initialize(&gpioCompPhaseC);

    // gpio_t gpioButton = DEF_GPIO(GPIOC, 13, 0, GPIO_INPUT);
    // gpio_initialize(&gpioButton);
}

void comparator_exti_initialize()
{
    // EXTI4 is PF4
    EXTI->EXTICR[1] |= 0x05 << EXTI_EXTICR2_EXTI4_Pos;
    // EXTI15 is PC15
    EXTI->EXTICR[3] |= 0x02 << EXTI_EXTICR4_EXTI15_Pos;
    // EXTI14 is PC14
    EXTI->EXTICR[3] |= 0x02 << EXTI_EXTICR4_EXTI14_Pos;

    EXTI->EXTICR[3] |= 0x02 << EXTI_EXTICR4_EXTI13_Pos;

    EXTI->RTSR1 |= EXTI_RTSR1_RT13;
    EXTI->FTSR1 |= EXTI_FTSR1_FT13;
    NVIC_EnableIRQ(EXTI13_IRQn);
}

uint8_t getCompOutputLevel()
{
    if (step == 1 || step == 4) { // c floating
        return GPIOC->IDR & GPIO_IDR_ID14;
    } else if (step == 2 || step == 5) { // a floating
        return GPIOF->IDR & GPIO_IDR_ID4;
    } else /*if (step == 3 || step == 6)*/ { // b floating
        return GPIOC->IDR & GPIO_IDR_ID15;
    }
}

void maskPhaseInterrupts()
{
    EXTI->IMR1 &= ~EXTI_IMR1_IM13;
}

void enableCompInterrupts()
{
    EXTI->IMR1 |= EXTI_IMR1_IM13;
}
// reset value
#define EXTI_IMR1_CLEAR_MASK 0xfffe0000
#define EXTI_RTSR1_BITS (EXTI_RTSR1_RT4 | EXTI_RTSR1_RT14 | EXTI_RTSR1_RT15)
#define EXTI_FTSR1_BITS (EXTI_FTSR1_FT4 | EXTI_FTSR1_FT14 | EXTI_FTSR1_FT15)
void changeCompInput()
{
    EXTI->IMR1 &= EXTI_IMR1_CLEAR_MASK;
    if (step == 1 || step == 4) { // c floating
        EXTI->IMR1 |= EXTI_IMR1_IM14;
    } else if (step == 2 || step == 5) { // a floating
        EXTI->IMR1 |= EXTI_IMR1_IM4;
    } else /*if (step == 3 || step == 6)*/ { // b floating
        EXTI->IMR1 |= EXTI_IMR1_IM15;
    }
    if (rising) {
        EXTI->FTSR1 = 0;
        EXTI->RTSR1 = EXTI_RTSR1_BITS;
    } else { // falling bemf
        EXTI->RTSR1 = 0;
        EXTI->FTSR1 = EXTI_FTSR1_BITS;
    }
}
