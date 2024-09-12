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

void comparator_initialize(comparator_t* comp)
{
    // uint8_t i = 0;
    // while (i < 3) {
        
    // }
    comparator_initialize_gpio_exti(comp->ref);
    comparator_gpio_exti_nvic_enable(comp->ref);
    comparator_initialize_gpio(comp->ref);
}

void comparator_initialize_gpio(gpio_t* gpio)
{
    gpio_initialize(gpio);
}

// void comparator_gpio_initialize()
// {
//     gpio_initialize(&gpioCompPhaseA);
//     gpio_initialize(&gpioCompPhaseB);
//     gpio_initialize(&gpioCompPhaseC);

//     // gpio_t gpioButton = DEF_GPIO(GPIOC, 13, 0, GPIO_INPUT);
//     // gpio_initialize(&gpioButton);
// }

void comparator_gpio_exti_nvic_enable(gpio_t* gpio)
{

    EXTI->RTSR1 |= 1 << gpio->pin;
    EXTI->FTSR1 |= 1 << gpio->pin;

    NVIC_EnableIRQ(EXTI0_IRQn + gpio->pin);

}

void comparator_initialize_gpio_exti(gpio_t* gpio)
{
    // control register
    uint32_t cr;
    // find the control register (0-3) (CR1-CR4)
    switch (gpio->pin) 
    {
        case 0:
        case 1:
        case 2:
        case 3:
        {
            cr = EXTI->EXTICR[0];
            break;
        }
        case 4:
        case 5:
        case 6:
        case 7:
        {
            cr = EXTI->EXTICR[1];
            break;
        }
        case 8:
        case 9:
        case 10:
        case 11:
        {
            cr = EXTI->EXTICR[2];
            break;
        }
        case 12:
        case 13:
        case 14:
        case 15:
        {
            cr = EXTI->EXTICR[3];
            break;
        }
    }

    uint32_t cr_value;
    switch ((uint32_t)gpio->port) {
        case GPIOA_BASE:
            cr_value = 0x00;
            break;
        case GPIOB_BASE:
            cr_value = 0x01;
            break;
        case GPIOC_BASE:
            cr_value = 0x02;
            break;
        case GPIOD_BASE:
            cr_value = 0x03;
            break;
        case GPIOE_BASE:
            cr_value = 0x04;
            break;
        case GPIOF_BASE:
            cr_value = 0x05;
            break;
        case GPIOG_BASE:
            cr_value = 0x06;
            break;
        case GPIOH_BASE:
            cr_value = 0x07;
            break;
        case GPIOI_BASE:
            cr_value = 0x08;
            break;
        default:
            // others reserved
            break;
    }

    uint32_t cr_shift = gpio->pin % 4;

    // modify the register
    *(uint32_t*)cr |= cr_value << cr_shift;
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
