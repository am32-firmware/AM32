#include "stm32h563xx.h"
// #define COMPA_GPIO_PORT GPIOF
// #define COMPA_GPIO_PIN 4

// #define COMPB_GPIO_PORT GPIOC
// #define COMPB_GPIO_PIN 15

// #define COMPC_GPIO_PORT GPIOC
// #define COMPC_GPIO_PIN 14

#include "comparator.h"
#include "stm32h5xx_ll_gpio.h"
#include "targets.h"
#include "gpio.h"
#include "exti.h"

gpio_t gpioCompPhaseA = DEF_GPIO(COMPA_GPIO_PORT, COMPA_GPIO_PIN, 0, GPIO_INPUT);
gpio_t gpioCompPhaseB = DEF_GPIO(COMPB_GPIO_PORT, COMPB_GPIO_PIN, 0, GPIO_INPUT);
gpio_t gpioCompPhaseC = DEF_GPIO(COMPC_GPIO_PORT, COMPC_GPIO_PIN, 0, GPIO_INPUT);

comparator_t COMPARATOR = {
    .phaseA = &gpioCompPhaseA,
    .phaseB = &gpioCompPhaseB,
    .phaseC = &gpioCompPhaseC,

    // .phaseA = &gpioCompPhaseC,
    // .phaseB = &gpioCompPhaseA,
    // .phaseC = &gpioCompPhaseB,

    // .phaseA = &gpioCompPhaseB,
    // .phaseB = &gpioCompPhaseC,
    // .phaseC = &gpioCompPhaseA,

    // this spins, but doesn't close the loop
    // .phaseA = &gpioCompPhaseC,
    // .phaseB = &gpioCompPhaseB,
    // .phaseC = &gpioCompPhaseA,

    // .phaseA = &gpioCompPhaseA,
    // .phaseB = &gpioCompPhaseC,
    // .phaseC = &gpioCompPhaseB,

    // .phaseA = &gpioCompPhaseB,
    // .phaseB = &gpioCompPhaseA,
    // .phaseC = &gpioCompPhaseC,

};
void comparator_initialize(comparator_t* comp)
{
    comparator_initialize_gpio_exti(comp->phaseA);
    if (comp->phaseAcb) {
        exti_configure_cb(&extiChannels[comp->phaseA->pin], comp->phaseAcb);
        comparator_gpio_exti_nvic_enable(comp->phaseA);
    }

    comparator_initialize_gpio_exti(comp->phaseB);
    if (comp->phaseBcb) {
        exti_configure_cb(&extiChannels[comp->phaseB->pin], comp->phaseBcb);
        comparator_gpio_exti_nvic_enable(comp->phaseB);
    }

    comparator_initialize_gpio_exti(comp->phaseC);
    if (comp->phaseCcb) {
        exti_configure_cb(&extiChannels[comp->phaseC->pin], comp->phaseCcb);
        comparator_gpio_exti_nvic_enable(comp->phaseC);
    }

    comparator_initialize_gpio(comp->phaseA);
    comparator_initialize_gpio(comp->phaseB);
    comparator_initialize_gpio(comp->phaseC);
}

void comparator_initialize_gpio(gpio_t* gpio)
{
    gpio_initialize(gpio);
}

void comparator_gpio_exti_nvic_enable(gpio_t* gpio)
{
    exti_configure_trigger(&extiChannels[gpio->pin], EXTI_TRIGGER_RISING_FALLING);
    EXTI_NVIC_ENABLE(gpio->pin);
}

void comparator_nvic_set_priority(comparator_t* comp, uint32_t priority)
{
    EXTI_NVIC_SET_PRIORITY(comp->phaseA->pin, priority);
    EXTI_NVIC_SET_PRIORITY(comp->phaseB->pin, priority);
    EXTI_NVIC_SET_PRIORITY(comp->phaseC->pin, priority);
}

void comparator_initialize_gpio_exti(gpio_t* gpio)
{
    uint32_t cr_value;
    switch ((uint32_t)gpio->port) {
        case GPIOA_BASE:
            cr_value = EXTICR_GPIOA;
            break;
        case GPIOB_BASE:
            cr_value = EXTICR_GPIOB;
            break;
        case GPIOC_BASE:
            cr_value = EXTICR_GPIOC;
            break;
        case GPIOD_BASE:
            cr_value = EXTICR_GPIOD;
            break;
        case GPIOE_BASE:
            cr_value = EXTICR_GPIOE;
            break;
        case GPIOF_BASE:
            cr_value = EXTICR_GPIOF;
            break;
        case GPIOG_BASE:
            cr_value = EXTICR_GPIOG;
            break;
        case GPIOH_BASE:
            cr_value = EXTICR_GPIOH;
            break;
        case GPIOI_BASE:
            cr_value = EXTICR_GPIOI;
            break;
        default:
            // others reserved
            break;
    }

    exti_configure_port(&extiChannels[gpio->pin], cr_value);
}

uint8_t getCompOutputLevel()
{
    if (step == 1 || step == 4) { // c floating
        return gpio_read(COMPARATOR.phaseC);
    } else if (step == 2 || step == 5) { // a floating
        return gpio_read(COMPARATOR.phaseA);
    } else /*if (step == 3 || step == 6)*/ { // b floating
        return gpio_read(COMPARATOR.phaseB);
    }
}

void maskPhaseInterrupts()
{
    comparator_disable_interrupts(&COMPARATOR);
}

void comparator_disable_interrupts(comparator_t* comp)
{
    EXTI_INTERRUPT_DISABLE_MASK(
        (1 << comp->phaseA->pin) |
        (1 << comp->phaseB->pin) |
        (1 << comp->phaseC->pin)
    )
}

void enableCompInterrupts()
{
    comparator_enable_interrupts(&COMPARATOR);
}
void comparator_enable_interrupts(comparator_t* comp)
{
    // EXTI_INTERRUPT_ENABLE_MASK(
    //     (1 << comp->phaseA->pin) |
    //     (1 << comp->phaseB->pin) |
    //     (1 << comp->phaseC->pin)
    // )

    if (step == 1 || step == 4) { // c floating
        EXTI_INTERRUPT_ENABLE_MASK(1 << COMPARATOR.phaseC->pin);
    } else if (step == 2 || step == 5) { // a floating
        EXTI_INTERRUPT_ENABLE_MASK(1 << COMPARATOR.phaseA->pin);
    } else /*if (step == 3 || step == 6)*/ { // b floating
        EXTI_INTERRUPT_ENABLE_MASK(1 << COMPARATOR.phaseB->pin);
    }
}
// reset value | exti 15 used for setInput
#define EXTI_IMR1_CLEAR_MASK (0xfffe0000 | 1<<15)
#define EXTI_RTSR1_BITS (EXTI_RTSR1_RT4 | EXTI_RTSR1_RT14 | EXTI_RTSR1_RT15)
#define EXTI_FTSR1_BITS (EXTI_FTSR1_FT4 | EXTI_FTSR1_FT14 | EXTI_FTSR1_FT15)
void changeCompInput()
{
    EXTI->IMR1 &= EXTI_IMR1_CLEAR_MASK;
    // comparator_disable_interrupts(&COMPARATOR);
    if (step == 1 || step == 4) { // c floating
        EXTI_INTERRUPT_ENABLE_MASK(1 << COMPARATOR.phaseC->pin);
    } else if (step == 2 || step == 5) { // a floating
        EXTI_INTERRUPT_ENABLE_MASK(1 << COMPARATOR.phaseA->pin);
    } else /*if (step == 3 || step == 6)*/ { // b floating
        EXTI_INTERRUPT_ENABLE_MASK(1 << COMPARATOR.phaseB->pin);
    }
    if (rising) {
        EXTI->FTSR1 = 0;
        EXTI->FPR1  = 0;
        EXTI->RTSR1 = 
            (1 << COMPARATOR.phaseA->pin) |
            (1 << COMPARATOR.phaseB->pin) |
            (1 << COMPARATOR.phaseC->pin);
    } else { // falling bemf
        EXTI->RTSR1 = 0;
        EXTI->RPR1 = 0;
        // EXTI->RTSR1 = EXTI_RTSR1_BITS;
        EXTI->FTSR1 = 
            (1 << COMPARATOR.phaseA->pin) |
            (1 << COMPARATOR.phaseB->pin) |
            (1 << COMPARATOR.phaseC->pin);
    }
}