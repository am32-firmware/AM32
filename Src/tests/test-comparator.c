// this is a test of the comparator.h interface
// it will have the 3 leds reflect the state of the 3 phases

#include "targets.h"
#include "comparator.h"
#include "exti.h"
#include "led.h"
#include "mcu.h"

// IMR1 reset value is 0xfffe0000
gpio_t gpioCompPhaseATest = DEF_GPIO(COMPA_GPIO_PORT, COMPA_GPIO_PIN, 0, GPIO_INPUT);
gpio_t gpioCompPhaseBTest = DEF_GPIO(COMPB_GPIO_PORT, COMPB_GPIO_PIN, 0, GPIO_INPUT);
gpio_t gpioCompPhaseCTest = DEF_GPIO(COMPC_GPIO_PORT, COMPC_GPIO_PIN, 0, GPIO_INPUT);

uint32_t brg;
#define BLUE_VALUE (0x00100000)
#define GREEN_VALUE (0x00001000)
#define RED_VALUE (0x00000010)

void phaseATestcb(extiChannel_t* exti)
{
    uint32_t mask = 1 << exti->channel;
    if (EXTI->RPR1 & mask) {
        EXTI->RPR1 |= mask;
    }
    if (EXTI->FPR1 & mask) {
        EXTI->FPR1 |= mask;
    }
    if(gpio_read(&gpioCompPhaseATest)) {
        brg |= BLUE_VALUE;
    } else {
        brg &= ~BLUE_VALUE;
    }
    led_write(brg);
}

void phaseBTestcb(extiChannel_t* exti)
{
    uint32_t mask = 1 << exti->channel;
    if (EXTI->RPR1 & mask) {
        EXTI->RPR1 |= mask;
    }
    if (EXTI->FPR1 & mask) {
        EXTI->FPR1 |= mask;
    }
    if(gpio_read(&gpioCompPhaseBTest)) {
        brg |= RED_VALUE;
    } else {
        brg &= ~RED_VALUE;
    }
    led_write(brg);
}

void phaseCTestcb(extiChannel_t* exti)
{
    uint32_t mask = 1 << exti->channel;
    if (EXTI->RPR1 & mask) {
        EXTI->RPR1 |= mask;
    }
    if (EXTI->FPR1 & mask) {
        EXTI->FPR1 |= mask;
    }
    if(gpio_read(&gpioCompPhaseCTest)) {
        brg |= GREEN_VALUE;
    } else {
        brg &= ~GREEN_VALUE;
    }
    led_write(brg);
}

comparator_t comp = {
    .phaseA = &gpioCompPhaseATest,
    .phaseB = &gpioCompPhaseBTest,
    .phaseC = &gpioCompPhaseCTest,
    .phaseAcb = phaseATestcb,
    .phaseBcb = phaseBTestcb,
    .phaseCcb = phaseCTestcb
};

int main()
{
    mcu_setup(250);
    led_initialize();

    comparator_initialize(&comp);

    comparator_enable_interrupts(&comp);

    while(1) {
    }
}
