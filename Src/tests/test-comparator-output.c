// this is a test of the comparator.h interface
// it will have the 3 leds reflect the state of the 3 phases
// it will also output the comparator state to the GPIO on
// the aux port connector

#include "comparator.h"
#include "debug.h"
#include "exti.h"
#include "functions.h"
#include "led.h"
#include "mcu.h"
#include "targets.h"
#include "utility-timer.h"

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
        debug_set_3();
        brg |= BLUE_VALUE;
    } else {
        debug_reset_3();
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
        debug_set_2();
        brg |= RED_VALUE;
    } else {
        debug_reset_2();
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
        debug_set_1();
        brg |= GREEN_VALUE;
    } else {
        debug_reset_1();
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
    utility_timer_initialize();
    utility_timer_enable();

    debug_initialize();

    led_initialize();

    delayMillis(65);
    delayMillis(65);
    delayMillis(65);
    delayMillis(65);
    led_write(0x00000010);
    delayMillis(65);
    delayMillis(65);
    delayMillis(65);
    delayMillis(65);
    led_write(0x00001000);
    delayMillis(65);
    delayMillis(65);
    delayMillis(65);
    delayMillis(65);
    led_write(0x00100000);
    delayMillis(65);
    delayMillis(65);
    delayMillis(65);
    delayMillis(65);

    comparator_initialize(&comp);

    // set a low priority on comparator interrupt
    // this is necessary for this example
    // to use the sk6812 led spi interrupt
    comparator_nvic_set_priority(&comp, 4);

    comparator_enable_interrupts(&comp);

    while(1) {
    }
}
